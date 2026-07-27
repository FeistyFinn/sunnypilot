#!/usr/bin/env python3
"""Dump the actual onroad events + on-screen alerts in the storm segments, with timing, and
cross-reference against the OLD buggy buttonEvents (lkas + unknown). Establishes the GROUND TRUTH
for what the 'severe error' actually was, instead of assuming controlsMismatchLateral."""
import sys
import os
from collections import defaultdict
from typing import Any

from opendbc.car import structs
from openpilot.tools.lib.logreader import LogReader

ButtonType = structs.CarState.ButtonEvent.Type


def main(seg_paths):
  t0 = None
  # list[Any]: the record is heterogeneous -- [int count, float|None first_t, float|None last_t]
  ev: defaultdict[str, list[Any]] = defaultdict(lambda: [0, None, None])      # onroadEvents name -> record
  ev_sp: defaultdict[str, list[Any]] = defaultdict(lambda: [0, None, None])   # onroadEventsSP name -> record
  alerts = []                                    # (t, alertText1, alertText2) on change
  last_alert = None
  btn_times = []                                 # (t, pressed, type) for non-empty carState.buttonEvents

  for p in seg_paths:
    for msg in LogReader(os.path.join(p, "rlog.zst")):
      t = msg.logMonoTime / 1e9
      if t0 is None:
        t0 = t
      rt = t - t0
      w = msg.which()
      if w == "onroadEvents":
        for e in msg.onroadEvents:
          rec = ev[str(e.name)]
          rec[0] += 1
          if rec[1] is None:
            rec[1] = rt
          rec[2] = rt
      elif w == "onroadEventsSP":
        for e in msg.onroadEventsSP.events:
          rec = ev_sp[str(e.name)]
          rec[0] += 1
          if rec[1] is None:
            rec[1] = rt
          rec[2] = rt
      elif w == "selfdriveState":
        a = (msg.selfdriveState.alertText1, msg.selfdriveState.alertText2)
        if a != last_alert and (a[0] or a[1]):
          alerts.append((rt, a[0], a[1]))
          last_alert = a
      elif w == "carState":
        for b in msg.carState.buttonEvents:
          btn_times.append((rt, b.pressed, str(b.type)))

  print("=== onroadEvents (upstream) seen ===")
  for name, (c, f, l) in sorted(ev.items(), key=lambda kv: kv[1][1]):
    print(f"  {name:32s} count={c:6d}  first={f:7.2f}s last={l:7.2f}s")
  print("\n=== onroadEventsSP (sunnypilot) seen ===")
  for name, (c, f, l) in sorted(ev_sp.items(), key=lambda kv: kv[1][1]):
    print(f"  {name:32s} count={c:6d}  first={f:7.2f}s last={l:7.2f}s")

  print("\n=== on-screen alert transitions (selfdriveState) ===")
  for rt, a1, a2 in alerts:
    print(f"  {rt:7.2f}s  | {a1!r} | {a2!r}")

  print("\n=== buttonEvents timeline (lkas + unknown) ===")
  for rt, pressed, typ in btn_times:
    if typ in ("lkas", "unknown"):
      print(f"  {rt:7.2f}s  {typ:8s} pressed={pressed}")
  return 0


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:] or [
    "/data/media/0/realdata/ROUTE_ID--0",
    "/data/media/0/realdata/ROUTE_ID--1",
    "/data/media/0/realdata/ROUTE_ID--2",
  ]))
