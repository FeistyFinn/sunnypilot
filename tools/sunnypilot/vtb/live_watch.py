#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VTB live cereal watcher -- on-device, headless, SSH-friendly real-time monitor for in-car
test sessions. The live equivalent of the offline storm_*.py / analyze_shadow.py LogReader tools:
it subscribes to the running msgq (so it must run ON the comma device, where msgq is local
shared memory) and streams compact, line-buffered, parseable status to stdout.

Two phases:
  --mads     MADS finger-count desync monitor. Mirrors storm_state_dump.py's columns and adds a
             grant-latency timer: from a MADS engage request (an lkas buttonEvent press OR
             carControl.latActive rising) to the panda granting controlsAllowedLateral. Healthy is
             < ~0.1 s; a finger-count desync leaves the grant pending and fires the LOUD
             controlsMismatchLateral ~2.0 s later.
  --inertia  VTB inertia-comp fit-readiness monitor. Mirrors the on-device DevUI FIT meter exactly:
             accumulates qualifying hands-off high-alpha samples toward the offline fit's 200-sample
             gate, and surfaces the live |alphaFilt| peak so the driver can tell if the excitation
             is aggressive enough.
  --both     both monitors on one SubMaster; lines tagged MADS / INRT for demux.

Run on the device (cwd /data/openpilot, venv python -- system python3 lacks zmq):
  cd /data/openpilot && PYTHONPATH=/data:/data/openpilot \
    /usr/local/venv/bin/python tools/sunnypilot/vtb/live_watch.py --both
"""
from __future__ import annotations

import argparse
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

from cereal import messaging

# --- VTB fit thresholds: KEEP IN SYNC with selfdrive/ui/sunnypilot/onroad/developer_ui/vtb_fit.py ---
# Replicated (not imported) on purpose: importing that module pulls in pyray via the developer_ui
# package __init__.py, which needs a GL/display context this headless on-device process does not have.
# If you retune vtb_fit.py's thresholds or the qualify/status logic, MIRROR THE CHANGE HERE.
VTB_ALPHA_FLOOR = 5.0    # rad/s^2 - min |alpha| excitation for a usable ID sample
VTB_DEADZONE_NM = 0.5    # Nm - hands-off torque gate
VTB_FIT_SAMPLES = 200    # samples - fit's "n < 200 -> INSUFFICIENT" gate (= 2.0 s at 100 Hz)
VTB_FF_LIMIT = 2.5       # Nm - inertia FF clamp

GRANT_OK_S = 0.1         # grant latency below this is healthy
GRANT_TIMEOUT_S = 0.5    # no controlsAllowedLateral this long after a request -> warn (storm precursor)
DT_MAX_S = 0.05          # clamp the per-tick fit integration so a stall can't inflate the count
INERTIA_HEARTBEAT_S = 2.0  # refresh the FIT line at least this often so alphaPk stays visible

IGNORED_SAFETY_MODELS = ("silent", "noOutput")


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
    self._last_wait = -1e9

  def wait(self, sm, t: float) -> None:
    """Rate-limited offroad/services-down banner while this monitor's services aren't alive."""
    if t - self._last_wait > 1.0:
      dead = [s for s in self.SERVICES if not sm.alive[s]]
      emit(t, self.tag, f"waiting for: {', '.join(dead)}")
      self._last_wait = t

  def update(self, sm, t: float) -> None:
    raise NotImplementedError


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
    self.timeout_warned = False

  def _arm(self, t: float, kind: str) -> None:
    # first request edge of an engage cycle wins; ignore once already waiting or already granted
    if self.req_t is None and not self.pal:
      self.req_t = t
      self.req_kind = kind
      self.timeout_warned = False

  def update(self, sm, t: float) -> None:
    if sm.updated['pandaStates']:
      for ps in sm['pandaStates']:
        if str(ps.safetyModel) not in IGNORED_SAFETY_MODELS:
          self.cal, self.pal = ps.controlsAllowed, ps.controlsAllowedLateral
          break
      if self.pal and not self.prev_pal and self.req_t is not None:   # grant resolved
        lat = t - self.req_t
        emit(t, self.tag, f"GRANT  lat={lat:.3f}s {'OK' if lat < GRANT_OK_S else 'SLOW'}  ({self.req_kind}->pLat)")
        self.req_t = None
        self.timeout_warned = False
      if self.prev_pal and not self.pal:                              # disengage/revoke ends the cycle
        self.req_t = None
        self.timeout_warned = False
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
          self._arm(t, "lkas")

    if sm.updated['onroadEventsSP']:
      for e in sm['onroadEventsSP'].events:
        if str(e.name) == "controlsMismatchLateral":
          emit(t, self.tag, f"*** controlsMismatchLateral *** enable={e.enable} immediateDisable={e.immediateDisable} " +
                            f"vEgo={_fv(self.veg)}  <<<< STORM")

    if self.req_t is not None and not self.timeout_warned and not self.pal and (t - self.req_t) > GRANT_TIMEOUT_S:
      emit(t, self.tag, f"GRANT-TIMEOUT  no pLat {t - self.req_t:.2f}s after {self.req_kind} request  <-- WATCH")
      self.timeout_warned = True

    key = (self.cal, self.pal, self.lat_active, self.enabled_cc, self.ss_en, self.ss_act, self.sft, self.sfp, self.spress)
    if key != self.last_key or btxt:
      self.last_key = key
      emit(t, self.tag, f"pAllow={self.cal} pLat={self.pal} | ccLatAct={self.lat_active} ccEn={self.enabled_cc} | " +
                        f"ssEn={self.ss_en} ssAct={self.ss_act} | sFaultT={self.sft} sFaultP={self.sfp} " +
                        f"vEgo={_fv(self.veg)} sPress={self.spress}{btxt}")


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

    if status != self._prev_status or (n // 10) != (self._last_emit_n // 10) or (t - self._last_emit_t) > INERTIA_HEARTBEAT_S:
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


def build_monitors(args) -> list[Monitor]:
  mons: list[Monitor] = []
  if args.mads or args.both:
    mons.append(MadsMonitor("MADS" if args.both else ""))
  if args.inertia or args.both:
    mons.append(InertiaMonitor("INRT" if args.both else ""))
  return mons


def main() -> int:
  ap = argparse.ArgumentParser(description="VTB live cereal watcher (run on the comma device).")
  g = ap.add_mutually_exclusive_group(required=True)
  g.add_argument("--mads", action="store_true", help="MADS finger-count desync monitor")
  g.add_argument("--inertia", action="store_true", help="VTB inertia-comp fit-readiness monitor")
  g.add_argument("--both", action="store_true", help="both monitors (lines tagged MADS/INRT)")
  ap.add_argument("--timeout", type=int, default=100, help="SubMaster.update timeout (ms)")
  args = ap.parse_args()

  monitors = build_monitors(args)
  services = sorted({s for m in monitors for s in m.SERVICES})
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


if __name__ == "__main__":
  raise SystemExit(main())
