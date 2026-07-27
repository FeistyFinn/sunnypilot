#!/usr/bin/env python3
"""Decode the EPAS frame (0x370, bus 0) during the storms to pin which panda steering_disengage
trigger fired: hands_on_level>=3 vs |torsion_bar_torque|>500cNm (tesla.h:147-149). Correlate with
controls_allowed_lateral (pLat) and steerFaultTemporary."""
import sys
import os

from openpilot.tools.lib.logreader import LogReader

WINDOWS = [(16.5, 20.0), (124.5, 127.5)]
DISENGAGE_TORQUE = 500  # cNm (tesla.h TESLA_STEERING_DISENGAGE_TORQUE)


def in_win(rt):
  return any(a <= rt <= b for a, b in WINDOWS)


def main(seg_paths):
  t0 = None
  hol = tbt = None      # hands_on_level, torsion_bar_torque (cNm)
  pal = None            # controls_allowed_lateral
  sft = None            # steerFaultTemporary
  last_key = None

  for p in seg_paths:
    for msg in LogReader(os.path.join(p, "rlog.zst")):
      t = msg.logMonoTime / 1e9
      if t0 is None:
        t0 = t
      rt = t - t0
      w = msg.which()
      if w == "can":
        for c in msg.can:
          if c.src == 0 and c.address == 0x370:
            d = c.dat
            hol = d[4] >> 6
            tbt = (((d[2] & 0x0F) << 8) | d[3]) - 2050
      elif w == "pandaStates":
        for ps in msg.pandaStates:
          if str(ps.safetyModel) not in ("silent", "noOutput"):
            pal = ps.controlsAllowedLateral
            break
      elif w == "carState":
        sft = msg.carState.steerFaultTemporary
        # hol and tbt are only ever assigned together (same 0x370 frame), so the tbt check is
        # redundant at runtime -- it is here so the |tbt| arithmetic below is provably not-None.
        if in_win(rt) and hol is not None and tbt is not None:
          diseng = (hol >= 3) or (abs(tbt) > DISENGAGE_TORQUE)
          trig = []
          if hol >= 3:
            trig.append("HANDS>=3")
          if abs(tbt) > DISENGAGE_TORQUE:
            trig.append(">5Nm")
          key = (hol, abs(tbt) > DISENGAGE_TORQUE, pal, sft, diseng)
          if key != last_key:
            last_key = key
            print(f"{rt:7.2f}  hands_on_level={hol}  torsionBar={tbt:5d}cNm ({tbt/100:+.2f}Nm)  " +
                  f"-> steering_disengage={diseng} [{','.join(trig) or '-'}]  | pLat={pal} steerFaultTemp={sft}")
  return 0


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:] or [
    "/data/media/0/realdata/ROUTE_ID--0",
    "/data/media/0/realdata/ROUTE_ID--1",
    "/data/media/0/realdata/ROUTE_ID--2",
  ]))
