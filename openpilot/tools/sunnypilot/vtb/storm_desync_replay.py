#!/usr/bin/env python3
"""
Cross-layer regression proof for the Tesla MADS N-finger fix.

`storm_button_replay.py` proves the *openpilot* side (the gesture no longer thrashes). This script
proves the thing that actually caused the second storm: the **openpilot <-> panda desync**. The
`controlsMismatchLateral` "TAKE CONTROL IMMEDIATELY" alert fires when openpilot goes MADS-active
(lateral) but the panda has NOT granted `controls_allowed_lateral` for ~2.0 s. The original panda
matched the infotainment touch count with an exact `== 3`, while openpilot fires on `>= N`; the touch
signal is noisy and skips integer values (a "3-finger" tap can register as 4, or jump straight to 5),
so the two layers disagreed and stormed -- *even with the menu set to 3*.

This steps a recorded route's live touch count (cp_adas.vl["UI_status2"]["UI_activeTouchPoints"],
exactly as carstate_ext reads it) through BOTH layers, frame-aligned, for a given finger count N:
  - openpilot: opendbc CarStateExt.mads_gesture_button_events(touch) -> lkas toggle -> MADS active
  - panda:     the real libsafety Tesla model with current_safety_param_sp carrying N, fed the same
               (repacked) UI_status2 frames -> controls_allowed_lateral

and reports every frame where openpilot wants lateral but the panda has NOT granted it -- the exact
precondition for the storm. With the fix (panda uses `>= N` with the threaded N) the two layers agree
and there is no sustained desync window. Run it for N in {3,4,5} and on today's route once pulled.

Build/deps: the safety lib compiles on macOS via the opendbc venv recipe (see the project docs):
  cd opendbc_repo && uv venv && uv pip install -e . numpy cffi pytest pytest-xdist
  PYTHONPATH=$(pwd) .venv/bin/python ../tools/sunnypilot/vtb/storm_desync_replay.py <seg> [<seg> ...] [--fingers 3]
On the device:
  PYTHONPATH=/data/openpilot /usr/local/venv/bin/python \
    /data/openpilot/tools/sunnypilot/vtb/storm_desync_replay.py /data/media/0/realdata/<route>--0 [...]
"""
import os
import sys

from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.tesla.values import DBC, CANBUS
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.tesla.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP, TeslaSafetyFlagsSP
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety

from openpilot.tools.lib.logreader import LogReader

ButtonType = structs.CarState.ButtonEvent.Type

# A sustained openpilot-active-but-panda-denied window this long would let the 200-frame
# (~2.0 s @ 100 Hz) controlsMismatchLateral counter fire. Well under that is the pass bar.
DESYNC_FAIL_FRAMES = 50


def op_flags(fingers: int) -> int:
  return TeslaFlagsSP.HAS_VEHICLE_BUS.value | {4: TeslaFlagsSP.MADS_TOGGLE_FINGERS_4.value,
                                               5: TeslaFlagsSP.MADS_TOGGLE_FINGERS_5.value}.get(fingers, 0)


def sp_safety_param(fingers: int) -> int:
  return TeslaSafetyFlagsSP.HAS_VEHICLE_BUS | {4: TeslaSafetyFlagsSP.MADS_TOGGLE_FINGERS_4,
                                               5: TeslaSafetyFlagsSP.MADS_TOGGLE_FINGERS_5}.get(fingers, 0)


def new_ext(fingerprint: str, fingers: int) -> CarStateExt:
  cp_sp = structs.CarParamsSP()
  cp_sp.flags = op_flags(fingers)
  cp = structs.CarParams()
  cp.carFingerprint = fingerprint
  return CarStateExt(cp, cp_sp)


def setup_panda(fingers: int):
  safety = libsafety_py.libsafety
  safety.set_current_safety_param_sp(sp_safety_param(fingers))
  safety.set_safety_hooks(CarParams.SafetyModel.tesla, 0)
  safety.init_tests()
  safety.set_mads_params(True, False, False)  # MADS enabled, as it is on the road
  return safety


def replay(seg_paths, fingers: int) -> int:
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

  print(f"carFingerprint = {fp}    fingers (N) = {fingers}")
  cp_adas = CANParser(DBC[fp][Bus.adas], [], CANBUS.vehicle)
  ext = new_ext(fp, fingers)
  safety = setup_panda(fingers)
  packer = CANPackerSafety(DBC[fp][Bus.adas])

  latest_touch = 0
  op_active = False           # openpilot MADS lateral intent (lkas toggles it)
  desync_run = 0              # consecutive frames of op-active-but-panda-denied
  desync_windows = []         # (start_t, frames, touch_at_engage)
  n_frames = 0
  engage_at_touch = 0

  for p in seg_paths:
    for msg in LogReader(os.path.join(p, "rlog.zst")):
      w = msg.which()
      if w == "can":
        cp_adas.update([msg.logMonoTime, [(c.address, c.dat, c.src) for c in msg.can]])
        try:
          latest_touch = int(cp_adas.vl["UI_status2"]["UI_activeTouchPoints"])
        except Exception:
          pass
      elif w == "carState":
        n_frames += 1
        t = msg.logMonoTime / 1e9

        # openpilot: same gesture logic the device runs; an lkas press toggles MADS lateral
        for e in ext.mads_gesture_button_events(latest_touch):
          if e.pressed and e.type == ButtonType.lkas:
            op_active = not op_active
            engage_at_touch = latest_touch

        # panda: feed the SAME touch count (repacked) to the real safety model, read the grant
        safety.safety_rx_hook(packer.make_can_msg_safety("UI_status2", CANBUS.vehicle,
                                                         {"UI_activeTouchPoints": latest_touch}))
        panda_grant = safety.get_controls_allowed_lateral()

        if op_active and not panda_grant:
          if desync_run == 0:
            desync_windows.append([round(t, 2), 0, engage_at_touch])
          desync_run += 1
          desync_windows[-1][1] = desync_run
        else:
          desync_run = 0

  worst = max((w[1] for w in desync_windows), default=0)
  print(f"carState frames           : {n_frames}")
  print(f"desync windows (op-active & panda-denied): {len(desync_windows)}")
  for start_t, frames, touch in desync_windows[:20]:
    print(f"    @ {start_t:8.2f}s  {frames:4d} frames  (engaged at touch={touch})")
  print(f"worst sustained desync    : {worst} frames  (storm fires at 200; fail bar {DESYNC_FAIL_FRAMES})")

  print("\n=== VERDICT ===")
  if worst == 0:
    print(f"  CLEAN: openpilot and panda agree on every frame at N={fingers} -- no storm precondition.")
    return 0
  if worst < DESYNC_FAIL_FRAMES:
    print(f"  OK-ish: brief {worst}-frame desync (socket-skew tier), below the {DESYNC_FAIL_FRAMES}-frame bar.")
    return 0
  print(f"  DESYNC: sustained {worst}-frame window -> would storm. Panda is not honoring N={fingers}.")
  return 2


def main(argv) -> int:
  fingers = 3
  segs = []
  i = 0
  while i < len(argv):
    if argv[i] == "--fingers":
      fingers = int(argv[i + 1])
      i += 2
    else:
      segs.append(argv[i])
      i += 1
  if not segs:
    segs = [f"/data/media/0/realdata/ROUTE_ID--{s}" for s in (0, 1, 2)]
  return replay(segs, fingers)


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:]))
