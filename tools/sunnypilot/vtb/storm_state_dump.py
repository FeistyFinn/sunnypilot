#!/usr/bin/env python3
"""Per-cycle state around each controlsMismatchLateral storm, to find WHY panda revoked lateral
and whether the spurious `unknown` button burst is causally involved (vs. a plain lkas-engage +
steerTempUnavailable coincidence that the fix would NOT address)."""
import sys
import os

from opendbc.car import structs
from openpilot.tools.lib.logreader import LogReader

ButtonType = structs.CarState.ButtonEvent.Type
WINDOWS = [(108.0, 120.0)]


def in_win(rt):
  return any(a <= rt <= b for a, b in WINDOWS)


def main(seg_paths):
  t0 = None
  # latest cached state
  cal = pal = cont = None       # controlsAllowed, controlsAllowedLateral (chosen panda), safetyModel
  lat_active = enabled_cc = None
  ss_en = ss_act = None
  sft = sfp = veg = spress = None
  last_key = None

  for p in seg_paths:
    for msg in LogReader(os.path.join(p, "rlog.zst")):
      t = msg.logMonoTime / 1e9
      if t0 is None:
        t0 = t
      rt = t - t0
      w = msg.which()
      if w == "pandaStates":
        for ps in msg.pandaStates:
          if str(ps.safetyModel) not in ("silent", "noOutput"):
            cal, pal, cont = ps.controlsAllowed, ps.controlsAllowedLateral, str(ps.safetyModel)
            break
      elif w == "carControl":
        lat_active = msg.carControl.latActive
        enabled_cc = msg.carControl.enabled
      elif w == "selfdriveState":
        ss_en = msg.selfdriveState.enabled
        ss_act = msg.selfdriveState.active
      elif w == "carState":
        sft = msg.carState.steerFaultTemporary
        sfp = msg.carState.steerFaultPermanent
        veg = msg.carState.vEgo
        spress = msg.carState.steeringPressed
        btxt = "".join(f" BTN:{b.type}:{int(b.pressed)}" for b in msg.carState.buttonEvents)
        if in_win(rt):
          # print on any state change OR whenever there is a button event
          key = (cal, pal, lat_active, enabled_cc, ss_en, ss_act, sft, sfp, spress)
          if key != last_key or btxt:
            last_key = key
            print(f"{rt:7.2f} pAllow={cal} pLat={pal} | ccLatAct={lat_active} ccEn={enabled_cc} "
                  f"| ssEn={ss_en} ssAct={ss_act} | sFaultT={sft} sFaultP={sfp} vEgo={veg:4.1f} "
                  f"sPress={spress}{btxt}")
  return 0


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:] or [
    "/data/media/0/realdata/ROUTE_ID--0",
    "/data/media/0/realdata/ROUTE_ID--1",
    "/data/media/0/realdata/ROUTE_ID--2",
  ]))
