#!/usr/bin/env python3
"""
Bench proof for the Tesla MADS N-finger gesture fix.

The `controlsMismatchLateral` "TAKE CONTROL IMMEDIATELY" storm is a *stateful trajectory* bug:
the MADS mismatch counter (mads.py data_sample) only accumulates while lateral is `active` and
selfdrived is `not enabled` and the panda has revoked lateral -- a state reached when the buggy
N-finger code emits a *thrash* of lkas toggles. Replaying selfdrived in isolation can't reproduce
it (its replay init force-sets State.enabled and starts mid-history), so we prove the fix at the
exact level it changes: button generation.

This steps the recorded storm segments in time order, tracks the live infotainment touch count
from the vehicle-bus CAN exactly as carstate_ext does (cp_adas.vl["UI_status2"]["UI_activeTouchPoints"]),
and at each carState frame compares:
  - CONTROL: the buttonEvents the *buggy build actually logged* (carState.buttonEvents)
  - TEST:    what the deployed CarStateExt.mads_gesture_button_events() produces from the same touch count

A clean fix => CONTROL shows a thrash of lkas toggles (and/or unknown events); TEST shows exactly
one clean toggle per real gesture and never the thrash.

Run on the device (build + deps live there):
  PYTHONPATH=/data/openpilot /usr/local/venv/bin/python /data/openpilot/tools/sunnypilot/vtb/storm_button_replay.py \
      /data/media/0/realdata/ROUTE_ID--0 \
      /data/media/0/realdata/ROUTE_ID--1 \
      /data/media/0/realdata/ROUTE_ID--2
"""
import sys
import os

from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.tesla.values import DBC, CANBUS
from opendbc.sunnypilot.car.tesla.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

from openpilot.tools.lib.logreader import LogReader

ButtonType = structs.CarState.ButtonEvent.Type


def new_ext(fingerprint: str, fingers: int = 5) -> CarStateExt:
  cp_sp = structs.CarParamsSP()
  flags = TeslaFlagsSP.HAS_VEHICLE_BUS.value
  flags |= {5: TeslaFlagsSP.MADS_TOGGLE_FINGERS_5.value,
            4: TeslaFlagsSP.MADS_TOGGLE_FINGERS_4.value}.get(fingers, 0)
  cp_sp.flags = flags
  cp = structs.CarParams()
  cp.carFingerprint = fingerprint
  return CarStateExt(cp, cp_sp)


def summarize_events(evs):
  lkas_press = sum(1 for e in evs if e.pressed and e.type == ButtonType.lkas)
  unknown = sum(1 for e in evs if e.type == ButtonType.unknown)
  return lkas_press, unknown


def main(seg_paths):
  # carFingerprint from the first carParams we see (needed to pick the DBC)
  fp = None
  for p in seg_paths:
    for msg in LogReader(os.path.join(p, "rlog.zst")):
      if msg.which() == "carParams":
        fp = msg.carParams.carFingerprint
        break
    if fp:
      break
  if not fp:
    print("FATAL: no carParams in segments")
    return 1
  print(f"carFingerprint = {fp}")
  print(f"adas DBC       = {DBC[fp][Bus.adas]}   (bus={CANBUS.vehicle})")

  cp_adas = CANParser(DBC[fp][Bus.adas], [], CANBUS.vehicle)
  ext = new_ext(fp, fingers=5)

  latest_touch = 0
  touch_hist = {}
  control_evs = []        # buttonEvents the buggy build logged
  test_evs = []           # buttonEvents the deployed fix produces from the same touch stream
  control_lkas_times = []
  test_lkas_times = []
  n_carstate = 0

  for p in seg_paths:
    for msg in LogReader(os.path.join(p, "rlog.zst")):
      w = msg.which()
      if w == "can":
        frames = [(c.address, c.dat, c.src) for c in msg.can]
        cp_adas.update([msg.logMonoTime, frames])
        try:
          latest_touch = int(cp_adas.vl["UI_status2"]["UI_activeTouchPoints"])
        except Exception:
          pass
      elif w == "carState":
        n_carstate += 1
        touch_hist[latest_touch] = touch_hist.get(latest_touch, 0) + 1
        # CONTROL: what the buggy build actually emitted
        old = list(msg.carState.buttonEvents)
        control_evs.extend(old)
        for e in old:
          if e.pressed and e.type == ButtonType.lkas:
            control_lkas_times.append(msg.logMonoTime / 1e9)
        # TEST: deployed fix, same touch count
        new = ext.mads_gesture_button_events(latest_touch)
        test_evs.extend(new)
        for e in new:
          if e.pressed and e.type == ButtonType.lkas:
            test_lkas_times.append(msg.logMonoTime / 1e9)

  c_lkas, c_unk = summarize_events(control_evs)
  t_lkas, t_unk = summarize_events(test_evs)

  def min_gap(ts):
    return min((b - a for a, b in zip(ts, ts[1:], strict=False)), default=float("inf"))

  print(f"\ncarState frames processed : {n_carstate}")
  print(f"touch-count histogram     : {dict(sorted(touch_hist.items()))}")
  print("\n--- CONTROL (buggy build's logged buttonEvents) ---")
  print(f"  lkas press events : {c_lkas}")
  print(f"  unknown events    : {c_unk}")
  print(f"  min gap between lkas presses (s): {min_gap(control_lkas_times):.3f}")
  print(f"  lkas press times (s, first 30): {[round(t,2) for t in control_lkas_times[:30]]}")
  print("\n--- TEST (deployed mads_gesture_button_events, same touch stream) ---")
  print(f"  lkas press events : {t_lkas}")
  print(f"  unknown events    : {t_unk}")
  print(f"  min gap between lkas presses (s): {min_gap(test_lkas_times):.3f}")
  print(f"  lkas press times (s): {[round(t,2) for t in test_lkas_times]}")

  print("\n=== VERDICT ===")
  thrash = c_lkas > t_lkas and min_gap(control_lkas_times) < 0.5
  if c_lkas == 0:
    print("  INCONCLUSIVE: control logged 0 lkas presses -- storm not in these segments / not the buggy build.")
  elif thrash:
    print(f"  FIX CONFIRMED: control thrashes ({c_lkas} lkas presses, {c_unk} unknown, " +
          f"min gap {min_gap(control_lkas_times):.3f}s) -> deployed code collapses the SAME touch stream " +
          f"to {t_lkas} clean toggle(s), {t_unk} unknown. The button thrash that is the necessary " +
          "precondition for controlsMismatchLateral is eliminated.")
  else:
    print(f"  REVIEW: control={c_lkas} lkas / test={t_lkas} lkas -- not the expected thrash pattern; inspect above.")
  return 0


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:] or [
    "/data/media/0/realdata/ROUTE_ID--0",
    "/data/media/0/realdata/ROUTE_ID--1",
    "/data/media/0/realdata/ROUTE_ID--2",
  ]))
