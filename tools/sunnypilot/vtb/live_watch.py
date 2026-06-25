#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VTB cereal watcher for in-car MADS / inertia-comp testing -- compact, line-buffered, parseable.

Two analysis monitors:
  --mads     MADS finger-count desync monitor. Mirrors storm_state_dump.py columns + a grant-latency
             timer (an lkas buttonEvent / carControl.latActive rising -> panda controlsAllowedLateral;
             healthy < ~0.1 s) and a LOUD controlsMismatchLateral ("storm") flag.
  --inertia  VTB inertia-comp fit-readiness monitor. Mirrors the on-device DevUI FIT meter (qualifying
             hands-off high-alpha samples toward the 200-sample offline-fit gate) + live |alphaFilt| peak.
  --both     both monitors; lines tagged MADS / INRT.

Two run modes:
  LIVE (default, on the comma): subscribes to the running msgq.
       *** REFUSES to run while the car is moving (IsOnroad) *** -- a 100 Hz polling subscriber steals
       CPU from the control stack and can make openpilot disengage. Use --replay for analysis; --force
       only for an offroad bench. Runs at low priority (os.nice) as defense in depth.
         cd /data/openpilot && PYTHONPATH=/data:/data/openpilot \
           /usr/local/venv/bin/python tools/sunnypilot/vtb/live_watch.py --both
  --replay ROUTE|PATH (offline, on a laptop): replays a saved route's rlog through the SAME monitors
       (zero device load) and prints a summary. ROUTE is a route name resolved under
       ~/.comma/media/0/realdata or ~/vtb-routes, or an explicit rlog.zst / segment-dir path.
         .venv/bin/python tools/sunnypilot/vtb/live_watch.py --both --replay ROUTE_ID
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import time

# --- bootstrap: make 'openpilot' resolve to this repo root regardless of dir name ---
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
try:
  import openpilot  # noqa: F401
except ModuleNotFoundError:
  import types
  _pkg = types.ModuleType("openpilot")
  _pkg.__path__ = [_REPO]
  sys.modules["openpilot"] = _pkg

# cereal.messaging is imported lazily inside run_live() so --replay works without a live msgq env.

# --- VTB fit thresholds: KEEP IN SYNC with selfdrive/ui/sunnypilot/onroad/developer_ui/vtb_fit.py ---
# Replicated (not imported) on purpose: importing that module pulls in pyray via the developer_ui
# package __init__.py, which needs a GL/display context this headless process does not have.
# If you retune vtb_fit.py's thresholds or the qualify/status logic, MIRROR THE CHANGE HERE.
VTB_ALPHA_FLOOR = 5.0    # rad/s^2 - min |alpha| excitation for a usable ID sample
VTB_DEADZONE_NM = 0.5    # Nm - hands-off torque gate
VTB_FIT_SAMPLES = 200    # samples - fit's "n < 200 -> INSUFFICIENT" gate (= 2.0 s at 100 Hz)
VTB_FF_LIMIT = 2.5       # Nm - inertia FF clamp

GRANT_OK_S = 0.1         # grant latency below this is healthy
GRANT_TIMEOUT_S = 0.5    # no controlsAllowedLateral this long after a request -> warn (storm precursor)
DT_MAX_S = 0.05          # clamp the per-tick fit integration so a stall can't inflate the count
INERTIA_HEARTBEAT_S = 2.0  # live: refresh the FIT line at least this often so alphaPk stays visible

IGNORED_SAFETY_MODELS = ("silent", "noOutput")
LOCAL_LOG_ROOTS = ("~/.comma/media/0/realdata", "~/vtb-routes")


def vtb_sample_qualifies(coop, car_state) -> bool:
  """Mirror vtb_fit.vtb_sample_qualifies / fit_steer_inertia.id_mask: hands-OFF, openpilot-steered,
  high-alpha -- the only moments where J is identifiable from tau = J*alpha."""
  return bool(coop.coopActive
              and not car_state.steeringPressed
              and abs(car_state.steeringTorque) < VTB_DEADZONE_NM
              and abs(coop.alphaFilt) > VTB_ALPHA_FLOOR)


def vtb_fit_status(coop, n: int) -> str:
  """Mirror vtb_fit.vtb_fit_status: sign_error / saturated / ready / gathering, in priority order."""
  ti, af = coop.tauInertia, coop.alphaFilt
  if abs(ti) > 0.01 and abs(af) > 0.01 and ti * af < 0:
    return "sign_error"
  if abs(ti) >= 0.95 * VTB_FF_LIMIT:
    return "saturated"
  if n >= VTB_FIT_SAMPLES:
    return "ready"
  return "gathering"


def emit(t: float, tag: str, body: str) -> None:
  """One flushed line: seconds-since-start timestamp, optional demux tag, body."""
  print((f"{t:8.2f} {tag} " if tag else f"{t:8.2f} ") + body, flush=True)


def _fv(v) -> str:
  return f"{v:4.1f}" if v is not None else " n/a"


class Monitor:
  SERVICES: list[str] = []

  def __init__(self, tag: str = ""):
    self.tag = tag
    self.verbose = True       # live streams every state line; replay sets False (notable events + summary)
    self._last_wait = -1e9

  def wait(self, sm, t: float) -> None:
    """Rate-limited offroad/services-down banner while this monitor's services aren't alive (live only)."""
    if t - self._last_wait > 1.0:
      dead = [s for s in self.SERVICES if not sm.alive[s]]
      emit(t, self.tag, f"waiting for: {', '.join(dead)}")
      self._last_wait = t

  def update(self, sm, t: float) -> None:
    raise NotImplementedError

  def summary(self) -> str:
    return ""


class MadsMonitor(Monitor):
  SERVICES = ['pandaStates', 'carState', 'carControl', 'selfdriveState', 'onroadEventsSP']

  def __init__(self, tag: str = ""):
    super().__init__(tag)
    # cached state (mirror storm_state_dump.py)
    self.cal = self.pal = None              # controlsAllowed, controlsAllowedLateral (chosen panda)
    self.lat_active = self.enabled_cc = None
    self.ss_en = self.ss_act = None
    self.sft = self.sfp = self.veg = self.spress = None
    self.last_key = None
    # grant-latency tracking
    self.req_t = None                       # time of the engage request edge being timed
    self.req_kind = ""                      # "lkas" or "latActive"
    self.prev_lat_active = False
    self.prev_pal = None
    self._storm_prev = False
    # summary counters
    self.n_lkas = self.n_grant_ok = self.n_grant_slow = self.n_timeout = self.n_storm = 0
    self.max_lat = 0.0

  def _arm(self, t: float, kind: str) -> None:
    # first request edge of an engage cycle wins; ignore once already waiting or already granted
    if self.req_t is None and not self.pal:
      self.req_t = t
      self.req_kind = kind

  def update(self, sm, t: float) -> None:
    if sm.updated['pandaStates']:
      for ps in sm['pandaStates']:
        if str(ps.safetyModel) not in IGNORED_SAFETY_MODELS:
          self.cal, self.pal = ps.controlsAllowed, ps.controlsAllowedLateral
          break
      if self.pal and not self.prev_pal and self.req_t is not None:   # grant resolved
        lat = t - self.req_t
        ok = lat < GRANT_OK_S
        self.n_grant_ok += int(ok)
        self.n_grant_slow += int(not ok)
        self.max_lat = max(self.max_lat, lat)
        emit(t, self.tag, f"GRANT  lat={lat:.3f}s {'OK' if ok else 'SLOW'}  ({self.req_kind}->pLat)")
        self.req_t = None
      if self.prev_pal and not self.pal:                              # disengage/revoke ends the cycle
        self.req_t = None
      self.prev_pal = self.pal

    if sm.updated['carControl']:
      self.lat_active, self.enabled_cc = sm['carControl'].latActive, sm['carControl'].enabled
      if self.lat_active and not self.prev_lat_active:
        self._arm(t, "latActive")
      self.prev_lat_active = self.lat_active

    if sm.updated['selfdriveState']:
      self.ss_en, self.ss_act = sm['selfdriveState'].enabled, sm['selfdriveState'].active

    btxt = ""
    if sm.updated['carState']:
      cs = sm['carState']
      self.sft, self.sfp = cs.steerFaultTemporary, cs.steerFaultPermanent
      self.veg, self.spress = cs.vEgo, cs.steeringPressed
      btxt = "".join(f" BTN:{b.type}:{int(b.pressed)}" for b in cs.buttonEvents)
      for b in cs.buttonEvents:
        if str(b.type) == "lkas" and b.pressed:
          self.n_lkas += 1
          self._arm(t, "lkas")

    if sm.updated['onroadEventsSP']:
      evs = sm['onroadEventsSP'].events
      has_storm = any(str(e.name) == "controlsMismatchLateral" for e in evs)
      if has_storm and not self._storm_prev:                          # count distinct storm onsets
        self.n_storm += 1
        ev = next(e for e in evs if str(e.name) == "controlsMismatchLateral")
        emit(t, self.tag, f"*** controlsMismatchLateral *** immediateDisable={ev.immediateDisable} vEgo={_fv(self.veg)}  <<<< STORM")
      self._storm_prev = has_storm

    # grant timeout: abandon the cycle (clear req_t) so a later grant can't report a stale latency
    # and the next request edge re-arms cleanly.
    if self.req_t is not None and not self.pal and (t - self.req_t) > GRANT_TIMEOUT_S:
      self.n_timeout += 1
      emit(t, self.tag, f"GRANT-TIMEOUT  no pLat {t - self.req_t:.2f}s after {self.req_kind} request  <-- WATCH")
      self.req_t = None

    key = (self.cal, self.pal, self.lat_active, self.enabled_cc, self.ss_en, self.ss_act, self.sft, self.sfp, self.spress)
    if self.verbose and (key != self.last_key or btxt):
      self.last_key = key
      emit(t, self.tag, f"pAllow={self.cal} pLat={self.pal} | ccLatAct={self.lat_active} ccEn={self.enabled_cc} | " +
                        f"ssEn={self.ss_en} ssAct={self.ss_act} | sFaultT={self.sft} sFaultP={self.sfp} " +
                        f"vEgo={_fv(self.veg)} sPress={self.spress}{btxt}")

  def summary(self) -> str:
    verdict = "CLEAN" if (self.n_storm == 0 and self.n_timeout == 0) else "CHECK FAILURES"
    return (f"MADS lkas_taps={self.n_lkas} grants_ok={self.n_grant_ok} grants_slow={self.n_grant_slow} " +
            f"timeouts={self.n_timeout} storms={self.n_storm} max_lat={self.max_lat:.3f}s -> {verdict}")


class InertiaMonitor(Monitor):
  SERVICES = ['carStateSP', 'carState']

  def __init__(self, tag: str = ""):
    super().__init__(tag)
    self._seconds = 0.0          # cumulative qualifying time (mirrors VtbFitCoverageElement._seconds)
    self._last_t = None
    self._alpha_peak = 0.0       # |alphaFilt| peak since the last emitted line
    self._prev_status = None
    self._last_emit_n = -1
    self._last_emit_t = -1e9
    # summary state
    self.max_n = 0
    self.alpha_peak_all = 0.0
    self.saw_saturated = False
    self.saw_sign_error = False

  def update(self, sm, t: float) -> None:
    if not sm.updated['carStateSP']:       # carStateSP is the 100 Hz driver carrying coopSteering
      return
    coop = sm['carStateSP'].coopSteering
    cs = sm['carState']

    # Integrate qualifying *time* (x100 -> equivalent 100 Hz samples), matching the DevUI's frame-time
    # accumulation rather than a message count, so the live meter tracks the on-screen FIT meter.
    if self._last_t is not None and vtb_sample_qualifies(coop, cs):
      self._seconds += min(t - self._last_t, DT_MAX_S)
    self._last_t = t
    n = min(int(self._seconds * 100.0), VTB_FIT_SAMPLES)
    status = vtb_fit_status(coop, n)
    self._alpha_peak = max(self._alpha_peak, abs(coop.alphaFilt))
    self.max_n = max(self.max_n, n)
    self.alpha_peak_all = max(self.alpha_peak_all, abs(coop.alphaFilt))
    self.saw_saturated = self.saw_saturated or status == "saturated"
    self.saw_sign_error = self.saw_sign_error or status == "sign_error"

    trigger = status != self._prev_status or (n // 10) != (self._last_emit_n // 10)
    if self.verbose:
      trigger = trigger or (t - self._last_emit_t) > INERTIA_HEARTBEAT_S
    if trigger:
      body = (f"FIT n={n:3d}/{VTB_FIT_SAMPLES} {status:9s} alphaPk={self._alpha_peak:5.1f} " +
              f"|tau|={abs(cs.steeringTorque):4.2f} vEgo={cs.vEgo:4.1f} " +
              f"coop={int(coop.coopActive)} comp={int(coop.inertiaCompActive)} shadow={int(coop.shadowActive)} J={coop.inertiaJUsed:.3f}")
      if status == "saturated":
        body += f"  tauI={coop.tauInertia:+.2f} af={coop.alphaFilt:+.1f}  <-- FF clamped"
      elif status == "sign_error":
        body += f"  tauI={coop.tauInertia:+.2f} af={coop.alphaFilt:+.1f}  <-- POLARITY BUG"
      emit(t, self.tag, body)
      self._prev_status, self._last_emit_n, self._last_emit_t = status, n, t
      self._alpha_peak = 0.0

  def summary(self) -> str:
    meter = int(self._seconds * 100.0)
    ready = self.max_n >= VTB_FIT_SAMPLES
    flags = [f for f, seen in (("saturated", self.saw_saturated), ("sign_error", self.saw_sign_error)) if seen]
    # This mirrors the DevUI meter (logged alphaFilt). The offline fit recomputes alpha and is stricter,
    # so the meter is OPTIMISTIC -- /vtb-tune gives the authoritative fit coverage (often much lower).
    return (f"INERTIA meter_samples={meter} (DevUI {self.max_n}/{VTB_FIT_SAMPLES}{', READY' if ready else ''}) " +
            f"alpha_peak={self.alpha_peak_all:.1f}rad/s^2" + (f" flags={','.join(flags)}" if flags else "") +
            "  NOTE: DevUI meter is optimistic -- run /vtb-tune for authoritative fit coverage")


class ReplaySubMaster:
  """Minimal SubMaster-like shim over LogReader for offline --replay: exposes sm[svc], sm.updated[svc],
  sm.alive[svc] and sm.update() so the live monitors run UNCHANGED. One message per update(); time comes
  from logMonoTime. Retains the latest Event per service (_keepalive) so cached capnp readers stay valid
  across segment boundaries (LogReader frees a segment's events once it advances)."""

  def __init__(self, services: list[str], paths: list[str]):
    from openpilot.tools.lib.logreader import LogReader
    self.services = list(services)
    self._data: dict = dict.fromkeys(self.services)
    self._keepalive: dict = {}
    self.updated = dict.fromkeys(self.services, False)
    self.alive = dict.fromkeys(self.services, False)
    self._iter = iter(LogReader(paths, sort_by_time=True))
    self._t0 = None
    self.t = 0.0
    self.done = False

  def __getitem__(self, s: str):
    return self._data[s]

  def update(self, timeout: int = 0) -> None:
    for s in self.services:
      self.updated[s] = False
    while True:
      try:
        msg = next(self._iter)
      except StopIteration:
        self.done = True
        return
      if self._t0 is None:
        self._t0 = msg.logMonoTime
      self.t = (msg.logMonoTime - self._t0) / 1e9
      w = msg.which()
      if w in self._data:
        self._keepalive[w] = msg              # keep the Event alive so _data[w]'s buffer survives
        self._data[w] = getattr(msg, w)
        self.updated[w] = True
        self.alive[w] = True
        return


def resolve_replay(arg: str) -> list[str]:
  """Resolve a --replay argument to a sorted list of rlog.zst paths: an explicit file, a directory
  (recursive), or a route name searched under the known local log roots."""
  p = os.path.expanduser(arg)
  if os.path.isfile(p):
    return [p]
  if os.path.isdir(p):
    segs = sorted(glob.glob(os.path.join(p, "**", "rlog.zst"), recursive=True))
    if segs:
      return segs
    raise SystemExit(f"live_watch: no rlog.zst found under {p}")
  for root in LOCAL_LOG_ROOTS:
    segs = glob.glob(os.path.join(os.path.expanduser(root), f"{arg}--*", "rlog.zst"))
    if segs:
      return sorted(segs, key=lambda q: int(os.path.basename(os.path.dirname(q)).rsplit("--", 1)[1]))
  raise SystemExit(f"live_watch: route '{arg}' not found locally under {', '.join(LOCAL_LOG_ROOTS)}")


def build_monitors(args) -> list[Monitor]:
  mons: list[Monitor] = []
  if args.mads or args.both:
    mons.append(MadsMonitor("MADS" if args.both else ""))
  if args.inertia or args.both:
    mons.append(InertiaMonitor("INRT" if args.both else ""))
  return mons


def run_replay(args, monitors: list[Monitor], services: list[str]) -> int:
  paths = resolve_replay(args.replay)
  for m in monitors:
    m.verbose = False                          # notable events + summary only (full routes are long)
  emit(0.0, "", f"replay {args.replay}: {len(paths)} seg(s), services={services}")
  sm = ReplaySubMaster(services, paths)
  while True:
    sm.update()
    if sm.done:
      break
    for m in monitors:
      if all(sm.alive[s] for s in m.SERVICES):
        m.update(sm, sm.t)
  emit(sm.t, "", "=== replay summary ===")
  for m in monitors:
    line = m.summary()
    if line:
      emit(sm.t, m.tag, line)
  return 0


def run_live(args, monitors: list[Monitor], services: list[str]) -> int:
  # SAFETY: a 100 Hz polling subscriber steals CPU from the control stack -> openpilot can disengage.
  try:
    from openpilot.common.params import Params
    onroad = Params().get_bool("IsOnroad")
  except Exception:
    onroad = False
  if onroad and not args.force:
    raise SystemExit("live_watch: refusing to run on a MOVING comma (IsOnroad). " +
                     "Use --replay <route> offline, or --force for an offroad bench.")
  try:
    os.nice(10)                                # defense in depth: lowest priority, never preempt control
  except OSError:
    pass

  from cereal import messaging
  sm = messaging.SubMaster(services)
  emit(0.0, "", f"live_watch up: services={services}")
  t0 = time.monotonic()
  try:
    while True:
      sm.update(args.timeout)
      t = time.monotonic() - t0
      for m in monitors:
        if all(sm.alive[s] for s in m.SERVICES):
          m.update(sm, t)
        else:
          m.wait(sm, t)
  except KeyboardInterrupt:
    emit(time.monotonic() - t0, "", "live_watch stopped")
    return 0


def main() -> int:
  ap = argparse.ArgumentParser(description="VTB cereal watcher for MADS / inertia-comp testing.")
  g = ap.add_mutually_exclusive_group(required=True)
  g.add_argument("--mads", action="store_true", help="MADS finger-count desync monitor")
  g.add_argument("--inertia", action="store_true", help="VTB inertia-comp fit-readiness monitor")
  g.add_argument("--both", action="store_true", help="both monitors (lines tagged MADS/INRT)")
  ap.add_argument("--replay", metavar="ROUTE|PATH", help="offline: replay a saved route's rlog instead of live msgq")
  ap.add_argument("--timeout", type=int, default=100, help="live SubMaster.update timeout (ms)")
  ap.add_argument("--force", action="store_true", help="override the onroad safety guard (live, offroad bench only)")
  args = ap.parse_args()

  monitors = build_monitors(args)
  services = sorted({s for m in monitors for s in m.SERVICES})
  if args.replay:
    return run_replay(args, monitors, services)
  return run_live(args, monitors, services)


if __name__ == "__main__":
  raise SystemExit(main())
