#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VTB per-segment event transcription -- decode each pulled rlog ONCE (offline) and persist a compact,
grep-able timeline of the notable VTB events to `events.jsonl` inside each segment dir, plus an
`events.meta.json` header for idempotency. The point: a parallel session / agent can read what
happened on a drive without re-running a replay or owning a cereal decoder.

It REUSES the existing event detectors rather than reinventing rlog decoding:
  - MADS grant / grant-timeout / storm  <- mads_events.MadsEventDetector (the SAME state machine
    live_watch.py drives, fed in the SAME order, so the storm/grant COUNTS match live_watch exactly;
    the detector persists across a route's segments, so a storm spanning a segment boundary is one
    onset, not two).
  - inertia-comp FF / coop / deadzone-guard  <- carStateSP.coopSteering, with the deadzone-guard
    condition and DEADZONE_NM taken from analyze_shadow.py, and the fit-qualify gate from
    live_watch.vtb_sample_qualifies -- so the per-segment summary is a superset of what
    analyze_shadow prints, computed in the same single pass.

Events (one JSON object per line, in timeline order):
  mads_grant, mads_grant_timeout, mads_storm   -- finger-count health
  engage, disengage                            -- selfdriveState.active edges
  coop_on, coop_off                            -- cooperative-steering session edges
  ff_engaged, ff_idle                          -- inertia FF actually doing something (Schmitt-gated)
  deadzone_guard_violation                     -- the FF-must-be-0-in-deadzone canary (should never fire)
  segment_summary                              -- per-segment rollup (last line of each file)
Each carries `t` (seconds from the route's first logged message, matching live_watch's clock) and
`mono` (raw logMonoTime ns, for exact ordering / cross-segment joins).

Usage (OFFLINE, on the Mac -- never on a moving comma):
  .venv/bin/python tools/sunnypilot/vtb/transcribe_events.py --routes ROUTE_ID
  .venv/bin/python tools/sunnypilot/vtb/transcribe_events.py --all          # backfill every local drive
  .venv/bin/python tools/sunnypilot/vtb/transcribe_events.py --routes <r> --force   # re-transcribe
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from types import SimpleNamespace

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

from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.sunnypilot.vtb.mads_events import MadsEventDetector, MADS_SERVICES
from openpilot.tools.sunnypilot.vtb.live_watch import vtb_sample_qualifies, resolve_replay, LOCAL_LOG_ROOTS
from openpilot.tools.sunnypilot.vtb.analyze_shadow import DEADZONE_NM

SCHEMA_VERSION = 1
FF_ON_NM = 0.05      # Schmitt: |tauInertia| rising above this (while coop) -> ff_engaged
FF_OFF_NM = 0.01     # |tauInertia| falling below this -> ff_idle (= vtb_fit_status's "FF active" floor)
GUARD_EPS = 1e-6     # analyze_shadow's deadzone-guard test: |tauInertia| > this inside the deadzone
_MADS = set(MADS_SERVICES)


def _seg_dir(rlog: str) -> str:
  return os.path.dirname(rlog)


def _events_path(seg_dir: str) -> str:
  return os.path.join(seg_dir, "events.jsonl")


def _meta_path(seg_dir: str) -> str:
  return os.path.join(seg_dir, "events.meta.json")


def _seg_current(rlog: str) -> bool:
  """True if this segment already has an up-to-date sidecar (same schema, same rlog size+mtime)."""
  sd = _seg_dir(rlog)
  if not (os.path.exists(_events_path(sd)) and os.path.exists(_meta_path(sd))):
    return False
  try:
    with open(_meta_path(sd)) as f:
      meta = json.load(f)
    st = os.stat(rlog)
  except (OSError, ValueError):
    return False
  return (meta.get("schema") == SCHEMA_VERSION and meta.get("rlog_size") == st.st_size
          and abs(meta.get("rlog_mtime", -1.0) - st.st_mtime) < 1e-6)


def _pcts(vals: list[float]) -> dict:
  if not vals:
    return {"p50": None, "p95": None, "max": None}
  a = np.abs(np.asarray(vals, dtype=float))
  return {"p50": round(float(np.percentile(a, 50)), 4),
          "p95": round(float(np.percentile(a, 95)), 4),
          "max": round(float(a.max()), 4)}


def _write_sidecar(seg_dir: str, rlog: str, events: list[dict], extra: dict) -> None:
  """Atomically write events.jsonl + events.meta.json (write-tmp then os.replace, same-dir rename)."""
  ev_tmp = _events_path(seg_dir) + ".tmp"
  with open(ev_tmp, "w") as f:
    for e in events:
      f.write(json.dumps(e, separators=(",", ":")) + "\n")
  os.replace(ev_tmp, _events_path(seg_dir))
  st = os.stat(rlog)
  meta = {"schema": SCHEMA_VERSION, "rlog_size": st.st_size, "rlog_mtime": st.st_mtime,
          "n_events": len(events), **extra}
  mp_tmp = _meta_path(seg_dir) + ".tmp"
  with open(mp_tmp, "w") as f:
    json.dump(meta, f)
  os.replace(mp_tmp, _meta_path(seg_dir))


def transcribe_route(route: str, force: bool = False) -> dict:
  """Transcribe every segment of one route in a single continuous pass (detector + edge state persist
  across segments, so boundary-spanning storms/sessions aren't double-counted), writing one
  events.jsonl per segment. Idempotent at the route level: skips iff every segment is already current."""
  paths = resolve_replay(route)          # sorted rlog.zst paths (route name, seg dir, or explicit file)
  if not paths:
    raise SystemExit(f"no rlog.zst found for {route}")
  if not force and all(_seg_current(p) for p in paths):
    return {"route": route, "skipped": True, "n_segs": len(paths)}

  det = MadsEventDetector()              # persists across the route's segments
  seen: set[str] = set()                 # MADS all-alive gate (mirrors run_replay's all(alive) guard)
  prev_active = prev_coop = None         # selfdriveState.active / coopActive edge state (persist)
  ff_on = False
  prev_guard = False
  last_vego = 0.0
  last_spress = False
  last_storque = 0.0
  route_t0 = None

  tot_events = tot_storms = tot_guard = 0
  for seg_idx, rlog in enumerate(paths):
    sd = _seg_dir(rlog)
    events: list[dict] = []
    seg_mono0 = None
    n = ncoop = nshadow = ncomp = nnudge = nqual = 0
    g_viol = g_checked = seg_lkas = 0
    tauI_nudge: list[float] = []
    alpha_nudge: list[float] = []
    j_used: set[float] = set()
    seg_mads = {"grants_ok": 0, "grants_slow": 0, "timeouts": 0, "storms": 0}

    for msg in LogReader([rlog], sort_by_time=True):
      mono = msg.logMonoTime
      if route_t0 is None:
        route_t0 = mono
      if seg_mono0 is None:
        seg_mono0 = mono
      t = (mono - route_t0) / 1e9
      w = msg.which()

      # latest carState scalars (needed by the carStateSP block + storm vEgo); cached unconditionally
      if w == "carState":
        cs = msg.carState
        last_vego = float(cs.vEgo)
        last_spress = bool(cs.steeringPressed)
        last_storque = float(cs.steeringTorque)

      # --- MADS state machine (gated on all 5 MADS services alive, exactly like live_watch) ---
      if w in _MADS:
        seen.add(w)
        if _MADS <= seen:
          evs: list[dict] = []
          if w == "pandaStates":
            ps = [(str(p.safetyModel), p.controlsAllowed, p.controlsAllowedLateral) for p in msg.pandaStates]
            evs += det.on_panda(t, ps)
          elif w == "carControl":
            det.on_lat_active(t, bool(msg.carControl.latActive))
          elif w == "carState":
            np_lkas = sum(1 for b in msg.carState.buttonEvents if str(b.type) == "lkas" and b.pressed)
            det.on_lkas(t, np_lkas)
            seg_lkas += np_lkas
          elif w == "onroadEventsSP":
            storm = next((e for e in msg.onroadEventsSP.events if str(e.name) == "controlsMismatchLateral"), None)
            evs += det.on_storm(t, storm is not None, storm.immediateDisable if storm else False, last_vego)
          evs += det.tick(t)
          for ev in evs:
            ev["t"] = round(t, 3)
            ev["mono"] = mono
            events.append(ev)
            if ev["event"] == "mads_grant":
              seg_mads["grants_ok" if ev["ok"] else "grants_slow"] += 1
            elif ev["event"] == "mads_grant_timeout":
              seg_mads["timeouts"] += 1
            elif ev["event"] == "mads_storm":
              seg_mads["storms"] += 1

      # --- engage / disengage (selfdriveState.active edges) ---
      if w == "selfdriveState":
        act = bool(msg.selfdriveState.active)
        if prev_active is not None and act != prev_active:
          events.append({"event": "engage" if act else "disengage", "t": round(t, 3), "mono": mono,
                         "enabled": bool(msg.selfdriveState.enabled)})
        prev_active = act

      # --- cooperative-steering FF telemetry (carStateSP.coopSteering @ 100 Hz) ---
      if w == "carStateSP":
        c = msg.carStateSP.coopSteering
        coop = bool(c.coopActive)
        ti = float(c.tauInertia)
        al = float(c.alphaFilt)
        ju = round(float(c.inertiaJUsed), 4)
        tau_raw = float(c.tauIntent) + ti          # carstate logs tauIntent = tau_raw - tauInertia
        n += 1

        if prev_coop is not None and coop != prev_coop:
          ev = {"event": "coop_on" if coop else "coop_off", "t": round(t, 3), "mono": mono}
          if coop:
            ev["vego"] = round(last_vego, 2)
          events.append(ev)
        prev_coop = coop

        if coop:
          ncoop += 1
          nshadow += int(bool(c.shadowActive))
          ncomp += int(bool(c.inertiaCompActive))
          j_used.add(ju)
          if vtb_sample_qualifies(c, SimpleNamespace(steeringPressed=last_spress, steeringTorque=last_storque)):
            nqual += 1
          if abs(tau_raw) > DEADZONE_NM:             # driver nudge: FF is allowed to act
            nnudge += 1
            tauI_nudge.append(abs(ti))
            alpha_nudge.append(abs(al))
            prev_guard = False                       # leaving the deadzone resets the guard-violation edge
          else:                                      # inside deadzone: FF MUST be ~0 (guard)
            g_checked += 1
            violation = abs(ti) > GUARD_EPS
            if violation:
              g_viol += 1
              if not prev_guard:
                events.append({"event": "deadzone_guard_violation", "t": round(t, 3), "mono": mono,
                               "tau_inertia": round(ti, 4), "tau_raw": round(tau_raw, 4)})
            prev_guard = violation
          # FF engaged/idle, Schmitt-triggered to avoid chatter around the 0.01 Nm floor
          if not ff_on and abs(ti) > FF_ON_NM:
            ff_on = True
            events.append({"event": "ff_engaged", "t": round(t, 3), "mono": mono,
                           "tau_inertia": round(ti, 4), "alpha": round(al, 3), "j_used": ju})
          elif ff_on and abs(ti) < FF_OFF_NM:
            ff_on = False
            events.append({"event": "ff_idle", "t": round(t, 3), "mono": mono})
        else:
          prev_guard = False
          if ff_on:
            ff_on = False
            events.append({"event": "ff_idle", "t": round(t, 3), "mono": mono})

    # --- per-segment summary (last line of the file) ---
    # seg_mono0 / route_t0 are None only for a segment with zero decodable messages (the first such
    # segment of a route leaves route_t0 None too) -> a None-safe t avoids a TypeError on a corrupt rlog.
    seg_t = round((seg_mono0 - route_t0) / 1e9, 3) if (seg_mono0 is not None and route_t0 is not None) else 0.0
    summary = {"event": "segment_summary", "seg": os.path.basename(sd), "seg_idx": seg_idx,
               "t": seg_t, "mono": seg_mono0,
               "n_carstatesp": n, "dur_s": round(n / 100.0, 1),
               "n_coop": ncoop, "n_shadow": nshadow, "n_comp": ncomp, "n_nudge": nnudge,
               "qualifying": nqual,
               "tauI_nudge_nm": _pcts(tauI_nudge), "alpha_nudge": _pcts(alpha_nudge),
               "guard_violations": g_viol, "guard_checked": g_checked,
               "j_used": sorted(j_used),
               "mads": {**seg_mads, "lkas_taps": seg_lkas}}
    events.append(summary)
    _write_sidecar(sd, rlog, events, {"route": route, "seg": os.path.basename(sd)})

    tot_events += len(events)
    tot_storms += seg_mads["storms"]
    tot_guard += g_viol

  return {"route": route, "skipped": False, "n_segs": len(paths),
          "events": tot_events, "storms": tot_storms, "guard": tot_guard}


def local_routes() -> list[str]:
  names: set[str] = set()
  for root in LOCAL_LOG_ROOTS:
    for d in glob.glob(os.path.join(os.path.expanduser(root), "*--*--*")):
      if os.path.exists(os.path.join(d, "rlog.zst")):   # skip partial pulls (qcamera/qlog but no rlog)
        names.add(os.path.basename(d).rsplit("--", 1)[0])
  return sorted(names)


def main() -> int:
  ap = argparse.ArgumentParser(description="Transcribe per-segment VTB events to events.jsonl sidecars (offline).")
  ap.add_argument("--routes", nargs="*", default=None, help="route names / seg dirs / rlog paths to transcribe")
  ap.add_argument("--all", action="store_true", help="transcribe every local drive (backfill)")
  ap.add_argument("--force", action="store_true", help="re-transcribe even if the sidecar is up-to-date")
  args = ap.parse_args()

  # SAFETY: this decodes every segment (CPU-heavy). Never let it run on a MOVING comma -> commIssue.
  try:
    from openpilot.common.params import Params
    onroad = Params().get_bool("IsOnroad")
  except Exception:
    onroad = False
  if onroad:
    raise SystemExit("transcribe_events: refusing to run on a MOVING comma (IsOnroad) -- run offline on the Mac.")
  try:
    os.nice(10)                                    # defense in depth: never preempt anything important
  except OSError:
    pass

  if args.all:
    routes = local_routes()
  elif args.routes:
    routes = args.routes
  else:
    raise SystemExit("transcribe_events: pass --routes <route...> or --all")
  if not routes:
    raise SystemExit("transcribe_events: no routes found")

  n_done = n_skip = n_err = 0
  for route in routes:
    try:
      r = transcribe_route(route, force=args.force)
    except (SystemExit, Exception) as e:           # one bad/corrupt route must not abort an --all run
      print(f"{route}: SKIP -- {e}", flush=True)
      n_err += 1
      continue
    if r.get("skipped"):
      print(f"{route}: up-to-date ({r['n_segs']} segs) -- skipped (use --force to redo)", flush=True)
      n_skip += 1
    else:
      print(f"{route}: {r['n_segs']} seg(s), {r['events']} events, {r['storms']} storms, " +
            f"{r['guard']} guard-violations", flush=True)
      n_done += 1
  print(f"\ndone: {n_done} transcribed, {n_skip} up-to-date, {n_err} skipped", flush=True)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
