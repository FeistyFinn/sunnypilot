#!/usr/bin/env python3
"""Per-drive evaluator for the VTB *integrated changes* that the other offline tools don't cover.

`analyze_shadow.py` / `fit_steer_inertia.py` / `transcribe_events.py` read only the coop-FF /
MADS-finger signal set (`logio.VTB_SIGNAL_SPEC` + the MADS services). Five recently-integrated
changes fall outside that set, so this tool does a direct `LogReader` pass and scores each:

  1. SCC-V anticipatory curve braking   -> longitudinalPlanSP.smartCruiseControl.vision
  2. Coop opposing-direction + cap       -> carStateSP.coopSteering.{angleOverride,blendedAngleDeg}
  3. ACC-cancel buttonEvents append fix  -> carState.buttonEvents(cancel) + onroadEventsSP(manualLongitudinalRequired)
  4. Scroll-wheel gap-adjust gesture      -> carState.buttonEvents(gapAdjustCruise) + carState.genericToggle
  5. Panda #6 heartbeat engaged-gating   -> pandaStates.{safetyModel,heartbeatLost,faultStatus,controlsAllowed}
                                            + onroadEvents.{commIssue,controlsMismatch,relayMalfunction}

Each change gets a one-line verdict. "not exercised" (a feature that never had the conditions to
fire on this drive) is reported distinctly from "misbehaved" so absence isn't read as breakage.

Offline only (reads pulled rlogs on the Mac). Reuses logio.resolve_segments + LogReader, matching
the conventions of the sibling analyzers (bootstrap block, --routes CLI, per-route text + verdict).

  .venv/bin/python tools/sunnypilot/vtb/eval_integrated.py --routes <route>     # one or more routes
  .venv/bin/python tools/sunnypilot/vtb/eval_integrated.py                      # all local routes
"""
import argparse
import os
import sys
from collections import Counter

# --- bootstrap: make 'openpilot' resolve to this repo root regardless of dir name ---
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
try:
  import openpilot  # noqa: F401
except ModuleNotFoundError:
  import types
  _pkg = types.ModuleType("openpilot")
  _pkg.__path__ = [_REPO]
  sys.modules["openpilot"] = _pkg

import numpy as np

from openpilot.tools.sunnypilot.vtb import logio

# --- constants (Tesla coop; derivations from opendbc/opendbc/car/tesla/values.py + coop_steering.py) ---
DT_LAT_CTRL = 0.02                     # DT_CTRL(0.01) * Tesla STEER_STEP(2)
MAX_ANGLE_RATE = 5.0                   # deg/frame, CoopSteeringCarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE
ANGLE_RATE_CEIL_DPS = MAX_ANGLE_RATE / DT_LAT_CTRL   # 250 deg/s hard cap on the delivered coop angle
STEER_ANGLE_MAX = 360.0                # deg, Tesla STEER_ANGLE_MAX (override angle is clamped to this)
EPS_FAULT_DPS = 12.0 / DT_LAT_CTRL     # 600 deg/s — the Tesla EPS hard-fault rate (values.py: faults at 12 deg/frame)
OVR_RUNAWAY_DEG = 180.0                # |angleOverride| this large = the opposing-consume ran away toward the clamp
# SCC-V TURNING table (vision_controller.py): a_target = interp(desired_lat_acc, BP, V)
SCCV_BP = [0.0, 1.5, 2.0, 2.8, 3.0, 3.6]           # 3.0 == MAX_LATERAL_ACCEL_NO_ROLL (the new breakpoint)
SCCV_V = [2.0, 0.5, 0.0, -0.4, -1.5, -3.5]         # ACCEL_MAX … the hub-corrected -1.5 … ACCEL_MIN
SILENT_MODELS = ("silent", "noOutput")
HEARTBEAT_EVENTS = ("commIssue", "commIssueAvgFreq", "controlsMismatch", "relayMalfunction",
                    "controlsMismatchLateral")


def _pct(a, p):
  return float(np.percentile(a, p)) if len(a) else 0.0


class RouteEval:
  def __init__(self):
    # SCC-V
    self.sccv_state = Counter()
    self.sccv_turn_latacc = []          # currentLateralAccel while turning
    self.sccv_turn_atarget = []         # aTarget while turning (paired with latacc)
    self.sccv_anomaly = 0               # turning frames with aTarget>0 while lat_acc>2.8 (the +1.5-bug shape)
    self.sccv_seen = False
    # coop
    self.coop_frames = 0
    self.coop_active = 0
    self.coop_latactive = 0
    self.ovr_abs = []                   # |angleOverride| while coop+latActive
    self._blend_prev = None             # (t, blendedAngleDeg) for rate
    self.blend_rate_dps = []            # |d blendedAngleDeg/dt| while coop+latActive
    self.cap_breaches = 0               # frames where delivered rate > ANGLE_RATE_CEIL_DPS
    # buttons
    self.btn = Counter()                # pressed-edge counts per ButtonType
    self.generic_toggle_frames = 0
    self.cs_frames = 0
    # engagement
    self.engages = 0
    self.disengages = 0
    self._sds_active_prev = None
    self.engaged_frames = 0
    # panda / heartbeat
    self.panda_frames = 0
    self.hb_lost = 0
    self.unexpected_silent = 0          # tesla-capable panda in silent/noOutput while onroad+engaged
    self.fault_status = Counter()
    self.fault_while_engaged = Counter()
    self.control_idx = set()            # pandaStates indices that have shown a non-silent (control) model
    self._ca_prev = {}                  # per-panda-index controlsAllowed, for drop-edge detection
    self.ca_drops_engaged = 0           # controlsAllowed True->False while engaged (surprise disengage)
    # events
    self.onroad_events = Counter()
    self.onroad_events_sp = Counter()
    self.dur_s = 0.0
    self._t0 = None
    self._t1 = None

  def feed(self, m):
    w = m.which()
    mono = m.logMonoTime * 1e-9
    if self._t0 is None:
      self._t0 = mono
    self._t1 = mono

    if w == "longitudinalPlanSP":
      self.sccv_seen = True
      v = m.longitudinalPlanSP.smartCruiseControl.vision
      st = str(v.state)
      self.sccv_state[st] += 1
      if st == "turning":
        la, at = float(v.currentLateralAccel), float(v.aTarget)
        self.sccv_turn_latacc.append(la)
        self.sccv_turn_atarget.append(at)
        if la > 2.8 and at > 0.0:
          self.sccv_anomaly += 1

    elif w == "carStateSP":
      # coop cap/override metrics come from _coop_cap_pass (cached scalar join); here just tally coverage
      self.coop_frames += 1
      if m.carStateSP.coopSteering.coopActive:
        self.coop_active += 1

    elif w == "carState":
      cs = m.carState
      self.cs_frames += 1
      if cs.genericToggle:
        self.generic_toggle_frames += 1
      for be in cs.buttonEvents:
        if be.pressed:
          self.btn[str(be.type)] += 1

    elif w == "selfdriveState":
      ss = m.selfdriveState
      if ss.active:
        self.engaged_frames += 1
      if self._sds_active_prev is not None and ss.active != self._sds_active_prev:
        if ss.active:
          self.engages += 1
        else:
          self.disengages += 1
      self._sds_active_prev = ss.active

    elif w == "pandaStates":
      for i, p in enumerate(m.pandaStates):
        sm = str(p.safetyModel)
        is_control = sm not in SILENT_MODELS
        if is_control:
          self.control_idx.add(i)
          # count only the "real" control panda's health (skip an always-silent secondary/harness panda)
          self.panda_frames += 1
          fs = str(p.faultStatus)
          self.fault_status[fs] += 1
          if self._sds_active_prev and fs != "faultNone":
            self.fault_while_engaged[fs] += 1
          if p.heartbeatLost:
            self.hb_lost += 1
          ca = bool(p.controlsAllowed)
          if self._ca_prev.get(i) is True and ca is False and self._sds_active_prev:
            self.ca_drops_engaged += 1
          self._ca_prev[i] = ca
        elif i in self.control_idx and self._sds_active_prev:
          # a panda that WAS a control panda dropped to SILENT while engaged -> heartbeat/safety regression
          self.unexpected_silent += 1

    elif w == "onroadEvents":
      for e in m.onroadEvents:
        nm = str(e.name)
        if nm in HEARTBEAT_EVENTS:
          self.onroad_events[nm] += 1

    elif w == "onroadEventsSP":
      for e in m.onroadEventsSP.events:   # SP variant wraps the list in .events (base onroadEvents is a direct list)
        self.onroad_events_sp[str(e.name)] += 1

  def _coop_cap_pass(self, segs):
    """Second, lightweight join: blendedAngleDeg rate + angleOverride excursion during coop+latActive.
    Uses read_signals (cached scalar spec) for carStateSP.coop + carControl.latActive, ZOH-aligned."""
    spec = {
      "carStateSP": {"coopSteering.coopActive": np.bool_, "coopSteering.angleOverride": np.float64,
                     "coopSteering.blendedAngleDeg": np.float64},
      "carControl": {"latActive": np.bool_},
    }
    sig = logio.read_signals(segs, spec=spec, use_cache=True)
    t = sig["carStateSP"]["t"]
    if len(t) < 8:
      return
    coop = sig["carStateSP"]["coopSteering.coopActive"]
    ovr = sig["carStateSP"]["coopSteering.angleOverride"]
    blend = sig["carStateSP"]["coopSteering.blendedAngleDeg"]
    lat = logio.zoh_align(sig["carControl"]["t"], sig["carControl"]["latActive"], t, fill=False)
    active = coop & lat
    self.coop_latactive = int(active.sum())
    self.ovr_abs = list(np.abs(ovr[active]))
    # Delivered-angle rate: blendedAngleDeg is a staircase (steps <=MAX_ANGLE_RATE deg per 20 ms control
    # frame) sampled ~100 Hz, so a single-sample diff spikes at each step and at coop on/off resets
    # (blendedAngleDeg snaps to the planner angle). Smooth over ~50 ms so only a SUSTAINED rate above the
    # cap counts as a real saturation failure; require both endpoints coop+latActive and a sane dt.
    k = 5
    blend_s = np.convolve(blend, np.ones(k) / k, mode="same")
    dt = np.diff(t)
    rate = np.abs(np.diff(blend_s)) / dt
    win = active[:-1] & active[1:] & (dt > 0.005) & (dt < 0.05)
    self.blend_rate_dps = list(rate[win])
    self.cap_breaches = int((rate[win] > ANGLE_RATE_CEIL_DPS).sum())

  def report(self, route):
    self.dur_s = (self._t1 - self._t0) if (self._t0 is not None) else 0.0
    L = []
    L.append(f"### {route}  ({self.dur_s/60:.1f} min, {self.cs_frames} carState frames, "
             + f"engages={self.engages} disengages={self.disengages})")

    # 1. SCC-V
    if not self.sccv_seen:
      L.append("  [SCC-V]        no longitudinalPlanSP -> NOT EXERCISED (SCC-V off or no plan)")
      v_sccv = "not exercised"
    else:
      turn = len(self.sccv_turn_latacc)
      maxla = max(self.sccv_turn_latacc) if turn else 0.0
      # did the new 3.0-breakpoint braking region fire? frames with lat_acc >= 2.8 and aTarget <= -0.4
      la = np.array(self.sccv_turn_latacc)
      at = np.array(self.sccv_turn_atarget)
      near30 = (la >= 2.8)
      braked30 = int((near30 & (at <= -0.4)).sum())
      active_states = sum(v for k, v in self.sccv_state.items() if k != "disabled")
      states = " ".join(f"{k}={v}" for k, v in self.sccv_state.most_common())
      L.append(f"  [SCC-V]        states: {states}")
      L.append(f"                 turning={turn} frames, max desired_lat_acc={maxla:.2f} m/s^2; "
               + f"frames past 2.8 breakpoint braking(<=-0.4)={braked30}; mid-corner accel anomalies(aTarget>0 & lat>2.8)={self.sccv_anomaly}")
      if self.sccv_anomaly:
        v_sccv = "ANOMALY (positive accel mid-corner — the +1.5 bug shape)"
      elif active_states == 0:
        v_sccv = "not exercised — vision stayed 'disabled' all drive (SmartCruiseControl toggle off / never engaged)"
      elif maxla < 2.8:
        v_sccv = f"active but no curve reached the new 2.8-3.0 breakpoint (max lat_acc {maxla:.2f})"
      elif braked30:
        v_sccv = f"OK — braked on curves into the 2.8-3.6 region ({braked30} frames), no positive-accel anomaly"
      else:
        v_sccv = "turning fired but no deep-curve braking sampled"

    # 2. coop opposing-direction + cap
    if self.coop_active == 0:
      L.append("  [coop]         coopActive never true -> NOT EXERCISED (TeslaCoopSteering off?)")
      v_coop = "not exercised"
    else:
      ovr_p95, ovr_max = _pct(self.ovr_abs, 95), (max(self.ovr_abs) if self.ovr_abs else 0.0)
      rate_p95 = _pct(self.blend_rate_dps, 95)
      rate_max = max(self.blend_rate_dps) if self.blend_rate_dps else 0.0
      L.append(f"  [coop]         coopActive={self.coop_active}/{self.coop_frames} frames, "
               + f"coop+latActive={self.coop_latactive} frames")
      L.append(f"                 |angleOverride| p95={ovr_p95:.2f} max={ovr_max:.2f} deg (clamp {STEER_ANGLE_MAX:.0f}); "
               + f"delivered-angle rate p95={rate_p95:.0f} max={rate_max:.0f} deg/s (cap {ANGLE_RATE_CEIL_DPS:.0f}); "
               + f"cap breaches={self.cap_breaches}")
      breach_frac = self.cap_breaches / max(1, len(self.blend_rate_dps))
      if ovr_max > OVR_RUNAWAY_DEG:
        # the code-review concern: opposing-consume grows angleOverride toward the clamp
        v_coop = f"OVERRIDE RUNAWAY (|angleOverride| max {ovr_max:.1f} deg approaching {STEER_ANGLE_MAX:.0f} clamp)"
      elif rate_max > EPS_FAULT_DPS or breach_frac > 1e-3:
        # a would-EPS-fault rate, or the soft cap sustained (>0.1% of frames) = real saturation failure
        v_coop = (f"CAP CONCERN ({self.cap_breaches} frames >{ANGLE_RATE_CEIL_DPS:.0f} deg/s, max {rate_max:.0f}, "
                  + f"{breach_frac*100:.3f}% — review saturation)")
      elif self.coop_latactive == 0:
        v_coop = "coop on but never steered (latActive never coincided) — override path not exercised"
      else:
        note = (f"; {self.cap_breaches} isolated frames grazed the {ANGLE_RATE_CEIL_DPS:.0f} deg/s soft cap "
                + f"(all <EPS-fault {EPS_FAULT_DPS:.0f}) — staircase-sampling noise") if self.cap_breaches else ""
        v_coop = (f"OK — override bounded (max {ovr_max:.1f} deg), delivered rate p95={rate_p95:.0f} deg/s "
                  + f"within limits{note}")

    # 3. ACC-cancel append fix
    cancels = self.btn.get("cancel", 0)
    mlr = self.onroad_events_sp.get("manualLongitudinalRequired", 0)
    L.append(f"  [ACC-cancel]   cancel buttonEvents={cancels}; manualLongitudinalRequired alerts={mlr}")
    if cancels == 0:
      v_cancel = "not exercised (no stalk ACC-cancel this drive)"
    else:
      v_cancel = f"OK — {cancels} cancel event(s) survived the append (would be 0 if clobbered)" + \
                 (f", {mlr} manualLongitudinalRequired follow-ups" if mlr else "")

    # 4. gap-adjust gesture
    gap = self.btn.get("gapAdjustCruise", 0)
    L.append(f"  [gap-adjust]   gapAdjustCruise buttonEvents={gap}; genericToggle(scroll) frames={self.generic_toggle_frames}")
    if gap == 0 and self.generic_toggle_frames == 0:
      v_gap = "not exercised (no scroll-wheel gap-adjust combo used)"
    elif gap:
      v_gap = f"OK — {gap} gap-adjust gesture(s) fired"
    else:
      v_gap = f"scroll used ({self.generic_toggle_frames} frames) but no gas+scroll gap-adjust combo"

    # 5. panda heartbeat gating
    fs = " ".join(f"{k}={v}" for k, v in self.fault_status.most_common())
    hb_evts = " ".join(f"{k}={v}" for k, v in self.onroad_events.most_common()) or "none"
    L.append(f"  [panda-hb]     heartbeatLost frames={self.hb_lost}; unexpected SILENT frames={self.unexpected_silent}; "
             + f"ctrlAllowed-drops-while-engaged={self.ca_drops_engaged}")
    L.append(f"                 faultStatus (context, hardware flag — NOT a #6 heartbeat signal): {fs}; "
             + f"heartbeat/comm events: {hb_evts}")
    # The #6 change gates the SILENT-mode timeout on heartbeat_engaged. Its regression signals are a
    # heartbeat loss, an UNEXPECTED SILENT drop, or the resulting commIssue/controlsMismatch/relay events.
    # faultStatus=faultTemp is a persistent pre-existing panda flag (present on pre-#6 baseline drives too),
    # so it is reported as context but does NOT count as a #6 regression; only faultPerm would.
    regression = (self.hb_lost or self.unexpected_silent
                  or self.onroad_events.get("commIssue", 0) or self.onroad_events.get("controlsMismatch", 0)
                  or self.onroad_events.get("relayMalfunction", 0) or self.fault_while_engaged.get("faultPerm", 0))
    if regression:
      v_hb = ("REGRESSION SIGNS — "
              + ", ".join(filter(None, [
                  f"heartbeatLost×{self.hb_lost}" if self.hb_lost else "",
                  f"unexpectedSILENT×{self.unexpected_silent}" if self.unexpected_silent else "",
                  f"commIssue×{self.onroad_events.get('commIssue',0)}" if self.onroad_events.get('commIssue') else "",
                  f"controlsMismatch×{self.onroad_events.get('controlsMismatch',0)}" if self.onroad_events.get('controlsMismatch') else "",
                  f"relayMalfunction×{self.onroad_events.get('relayMalfunction',0)}" if self.onroad_events.get('relayMalfunction') else "",
                  f"faultPerm×{self.fault_while_engaged.get('faultPerm',0)}" if self.fault_while_engaged.get('faultPerm') else "",
              ])))
    else:
      v_hb = "OK — no heartbeatLost / unexpected SILENT / commIssue / controlsMismatch (faultTemp is pre-existing, not #6)"

    L.append("  VERDICTS:")
    L.append(f"    SCC-V curve braking : {v_sccv}")
    L.append(f"    coop opposing/cap   : {v_coop}")
    L.append(f"    ACC-cancel fix      : {v_cancel}")
    L.append(f"    gap-adjust gesture  : {v_gap}")
    L.append(f"    panda #6 heartbeat  : {v_hb}")
    return "\n".join(L)


def eval_route(route):
  from openpilot.tools.lib.logreader import LogReader
  segs = logio.resolve_segments(route, missing_ok=True)
  if not segs:
    return f"### {route}\n  no rlog.zst local -> run /vtb-pull first"
  ev = RouteEval()
  for seg in segs:
    for m in LogReader([seg], sort_by_time=True):
      ev.feed(m)
  try:
    ev._coop_cap_pass(segs)   # cached scalar join for the coop cap metrics
  except Exception as e:      # cap metrics are best-effort; never fail the whole eval
    print(f"  (coop cap pass skipped: {type(e).__name__}: {e})", file=sys.stderr)
  return ev.report(route)


def main():
  ap = argparse.ArgumentParser(description="Evaluate the VTB integrated changes on pulled drive(s).")
  ap.add_argument("--routes", nargs="*", help="route names / seg dirs / rlog paths (default: all local)")
  args = ap.parse_args()
  routes = args.routes or logio.list_local_routes()
  if not routes:
    print("no local routes found under ~/.comma/media/0/realdata")
    return 1
  print(f"VTB integrated-changes evaluation — {len(routes)} route(s)\n")
  for r in routes:
    print(eval_route(r))
    print()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
