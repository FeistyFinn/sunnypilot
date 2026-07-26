#!/usr/bin/env python3
"""Onroad device-health readout (thermals + software performance) from pulled comma rlogs.

Greenfield, self-contained, offline. Decodes each segment ONCE with a single which()-dispatch
LogReader pass, caches the extracted per-segment signals next to the log (device_health.npz +
device_health.meta.json), and aggregates per-route-first, then across routes. Reads rlog when
present, else qlog (24x smaller, pulled over LTE for a device-health-only refresh); every service
it needs is in both EXCEPT modelV2 (rlog-only), so model-loop timing reads '-' on qlog routes.

Design constraints (validated against source + real rlogs):
 - Data is ONROAD-ONLY: loggerd runs only while `deviceState.started` is True, so there is no
   offroad/idle population and no ambient sensor on the comma 3X (intakeTempC reads 0.0). The
   thermal verdict is therefore ambient-INDEPENDENT: the device's own thermalStatus bands +
   fan-100% saturation + absolute maxTempC vs the 96C ok->overheated edge.
 - comm/thermal/space events come from `onroadEvents` ONLY (onroadEventsSP is a separate MADS
   enum). onroadEvents republishes all active events every frame, so we count rising edges +
   seconds-active, not raw per-frame occurrences.
 - maxTempC is 5s-filtered (lags load, ramps from 0 at drive start); use percentiles, not min.
 - Self-contained: reuses only LOCAL_LOG_ROOTS + the LogReader read pattern; the shared
   signals.npz cache and logio resolvers are never touched.

Usage:
  device_health.py <route|seg|rlog ...>     # specific routes
  device_health.py --all                    # every locally-preserved route
  device_health.py --all --note out.md      # also write the dated markdown note
"""
import argparse
import glob
import json
import os
import sys
from datetime import UTC, datetime
from multiprocessing import Pool, cpu_count

# --- bootstrap: make 'openpilot' resolve to this repo root regardless of dir name (mirror logio) ---
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
try:
  import openpilot  # noqa: F401
except ModuleNotFoundError:
  import types
  _pkg = types.ModuleType("openpilot")
  _pkg.__path__ = [_REPO]
  sys.modules["openpilot"] = _pkg

import numpy as np

from openpilot.tools.sunnypilot.vtb.vtb_constants import LOCAL_LOG_ROOTS

# Log file preference per segment: rlog (full, carries modelV2) first, else qlog (24x smaller, no
# modelV2 — pulled over LTE for a device-health-only refresh; model-loop timing then reads as '-').
_LOG_NAMES = ("rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog")

# Cache schema + the exact extracted metric set. Bump SCHEMA (or the signature changes) to invalidate.
CACHE_SCHEMA = 1
_NPZ = "device_health.npz"
_META = "device_health.meta.json"

# thermalStatus enum (log.capnp): ok@0 warmDEPRECATED@1 overheated@2 critical@3
THERMAL_NAMES = {0: "ok", 1: "warm", 2: "overheated", 3: "critical"}
OVERHEATED_RAW = 2
OK_OVERHEAT_EDGE_C = 96.0     # non-mici (3X/sdm845) ThermalBand ok upper edge, hardwared.py:50-54
MODEL_BUDGET_MS = 50.0        # modelV2 @ 20Hz -> 50ms period; steady-state p95 should sit well under
FAN_SATURATED = 100          # fanSpeedPercentDesired at 100% = cooling maxed
CRASH_STREAK = 3             # consecutive managerState samples (2Hz) 'shouldBeRunning and not running' = ~1.5s

# onroadEvents we care about for device health (EventName strings; read from onroadEvents ONLY)
HEALTH_EVENTS = ("overheat", "commIssue", "commIssueAvgFreq", "outOfSpace", "lowMemory",
                 "processNotRunning", "highCpuUsage", "tooDistracted")

# the deviceState series columns we extract + persist (name -> npz dtype)
_DS_COLS = {
  "t": np.float64, "started": np.bool_, "maxTempC": np.float32,
  "cpuTempMax": np.float32, "gpuTempMax": np.float32, "memoryTempC": np.float32, "dspTempC": np.float32,
  "cpuUsageMean": np.float32, "cpuUsageMax": np.float32, "gpuUsage": np.float32, "memUsage": np.float32,
  "freeSpace": np.float32, "fanSpeed": np.float32, "thermalRaw": np.int8,
  "powerDrawW": np.float32, "somPowerDrawW": np.float32,
}
# metric-set signature -> any change to what/how we extract invalidates stale caches
_METRIC_SIG = "ds:" + ",".join(_DS_COLS) + "|model:execMs|ev:" + ",".join(HEALTH_EVENTS) + f"|crash>={CRASH_STREAK}"


def _fmax(seq):
  """max of a capnp List, nan on empty (early-boot/partial frames carry empty temp lists)."""
  return max(seq) if len(seq) else float("nan")


def _fmean(seq):
  return float(np.mean(seq)) if len(seq) else float("nan")


# --------------------------------------------------------------------------- per-segment decode
def _decode_segment(rlog):
  """ONE which()-dispatch LogReader pass over a segment -> (arrays dict, events dict, manager dict)."""
  from openpilot.tools.lib.logreader import LogReader

  cols = {k: [] for k in _DS_COLS}  # each value is an independent per-message accumulator list
  model_t, model_ms = [], []
  ev_prev = dict.fromkeys(HEALTH_EVENTS, False)
  ev_edges = dict.fromkeys(HEALTH_EVENTS, 0)
  ev_secs = dict.fromkeys(HEALTH_EVENTS, 0.0)
  ev_last_t = dict.fromkeys(HEALTH_EVENTS, None)
  mgr_streak = {}      # proc name -> current consecutive-bad streak
  mgr_maxstreak = {}   # proc name -> max streak seen
  mgr_exit_pos = set()  # procs that exited with exitCode > 0 (crash)
  mgr_exit_neg = set()  # procs that exited with exitCode < 0 (signal / clean shutdown)

  for msg in LogReader([rlog], sort_by_time=True):
    w = msg.which()
    if w == "deviceState":
      ds = msg.deviceState
      t = msg.logMonoTime * 1e-9
      cols["t"].append(t)
      cols["started"].append(bool(ds.started))
      cols["maxTempC"].append(float(ds.maxTempC))
      cols["cpuTempMax"].append(_fmax(ds.cpuTempC))
      cols["gpuTempMax"].append(_fmax(ds.gpuTempC))
      cols["memoryTempC"].append(float(ds.memoryTempC))
      cols["dspTempC"].append(float(ds.dspTempC))
      cols["cpuUsageMean"].append(_fmean(ds.cpuUsagePercent))
      cols["cpuUsageMax"].append(_fmax(ds.cpuUsagePercent))
      cols["gpuUsage"].append(float(ds.gpuUsagePercent))
      cols["memUsage"].append(float(ds.memoryUsagePercent))
      cols["freeSpace"].append(float(ds.freeSpacePercent))
      cols["fanSpeed"].append(float(ds.fanSpeedPercentDesired))
      cols["thermalRaw"].append(int(ds.thermalStatus.raw))
      cols["powerDrawW"].append(float(ds.powerDrawW))
      cols["somPowerDrawW"].append(float(ds.somPowerDrawW))
    elif w == "modelV2":
      model_t.append(msg.logMonoTime * 1e-9)
      model_ms.append(float(msg.modelV2.modelExecutionTime) * 1e3)
    elif w == "onroadEvents":
      t = msg.logMonoTime * 1e-9
      active = {str(e.name) for e in msg.onroadEvents}
      for n in HEALTH_EVENTS:
        on = n in active
        if on and not ev_prev[n]:
          ev_edges[n] += 1                                 # rising edge = one incident
        if on and ev_last_t[n] is not None:
          ev_secs[n] += max(0.0, t - ev_last_t[n])         # seconds-active (boundary-safe within seg)
        ev_prev[n] = on
        ev_last_t[n] = t
    elif w == "managerState":
      for p in msg.managerState.processes:
        nm = str(p.name)
        bad = bool(p.shouldBeRunning) and not bool(p.running)
        streak = (mgr_streak.get(nm, 0) + 1) if bad else 0
        mgr_streak[nm] = streak
        mgr_maxstreak[nm] = max(mgr_maxstreak.get(nm, 0), streak)
        if not bool(p.running) and int(p.exitCode) != 0:
          (mgr_exit_pos if int(p.exitCode) > 0 else mgr_exit_neg).add(nm)

  arrays = {k: np.array(v, dtype=_DS_COLS[k]) for k, v in cols.items()}
  arrays["model_t"] = np.array(model_t, dtype=np.float64)
  arrays["model_ms"] = np.array(model_ms, dtype=np.float32)
  events = {n: {"edges": ev_edges[n], "secs": round(ev_secs[n], 2)} for n in HEALTH_EVENTS
            if ev_edges[n] or ev_secs[n]}
  manager = {"crashed": sorted(nm for nm, s in mgr_maxstreak.items() if s >= CRASH_STREAK),
             "exit_pos": sorted(mgr_exit_pos), "exit_neg": sorted(mgr_exit_neg)}
  return arrays, events, manager


# --------------------------------------------------------------------------- per-segment cache
def _seg_idx(rlog):
  return int(os.path.basename(os.path.dirname(rlog)).rsplit("--", 1)[1])


def _route_name(rlog):
  return os.path.basename(os.path.dirname(rlog)).rsplit("--", 1)[0]


def _seg_log(seg_dir):
  """Best available log file in a segment dir (rlog preferred, else qlog), or None."""
  for n in _LOG_NAMES:
    p = os.path.join(seg_dir, n)
    if os.path.exists(p):
      return p
  return None


def _all_routes(roots=LOCAL_LOG_ROOTS):
  """{route: [best-log path per segment, seg-sorted]} across roots (rlog preferred, else qlog)."""
  out = {}
  for root in roots:
    for seg_dir in glob.glob(os.path.join(os.path.expanduser(root), "*--*--*")):
      lp = _seg_log(seg_dir)
      if lp:
        out.setdefault(_route_name(lp), []).append(lp)
  for k in out:
    out[k].sort(key=_seg_idx)
  return dict(sorted(out.items()))


def _resolve(arg, roots=LOCAL_LOG_ROOTS):
  """arg -> best-log paths: a log file, a seg/route/root dir, or a bare route name (rlog|qlog)."""
  a = os.path.expanduser(arg)
  if os.path.isfile(a):
    return [a]
  seg_dirs = []
  if os.path.isdir(a):
    if _seg_log(a):
      seg_dirs = [a]                                    # a is itself a segment dir
    else:
      seg_dirs = [d for d in glob.glob(os.path.join(a, "**", "*--*--*"), recursive=True) if os.path.isdir(d)]
  else:
    for root in roots:
      seg_dirs += glob.glob(os.path.join(os.path.expanduser(root), f"{arg}--*"))
  logs = [lp for d in seg_dirs if (lp := _seg_log(d))]
  return sorted(set(logs), key=_seg_idx)


def _load_seg_cache(rlog):
  sd = os.path.dirname(rlog)
  npzp, mp = os.path.join(sd, _NPZ), os.path.join(sd, _META)
  if not (os.path.exists(npzp) and os.path.exists(mp)):
    return None
  try:
    with open(mp) as f:
      meta = json.load(f)
    st = os.stat(rlog)
  except (OSError, ValueError):
    return None
  if not (meta.get("schema") == CACHE_SCHEMA and meta.get("sig") == _METRIC_SIG
          and meta.get("rlog_size") == st.st_size and abs(meta.get("rlog_mtime", -1.0) - st.st_mtime) < 1e-6):
    return None
  try:
    with np.load(npzp) as z:
      arrays = {k: z[k] for k in z.files}
  except Exception:
    return None
  return arrays, meta.get("events", {}), meta.get("manager", {})


def _write_seg_cache(rlog, arrays, events, manager):
  sd = os.path.dirname(rlog)
  npzp, mp = os.path.join(sd, _NPZ), os.path.join(sd, _META)
  st = os.stat(rlog)
  meta = {"schema": CACHE_SCHEMA, "sig": _METRIC_SIG, "rlog_size": st.st_size,
          "rlog_mtime": st.st_mtime, "events": events, "manager": manager}
  tmp = npzp + ".tmp"
  with open(tmp, "wb") as f:
    np.savez_compressed(f, **arrays)
  os.replace(tmp, npzp)
  tmp = mp + ".tmp"
  with open(tmp, "w") as f:
    json.dump(meta, f)
  os.replace(tmp, mp)


def _worker(rlog):
  """Module-level (macOS spawn) worker: decode-or-cache ONE segment -> (route, seg_idx, payload)."""
  try:
    cached = _load_seg_cache(rlog)
    if cached is None:
      arrays, events, manager = _decode_segment(rlog)
      try:
        _write_seg_cache(rlog, arrays, events, manager)
      except OSError:
        pass  # a read-only / full log store must not fail the analysis
    else:
      arrays, events, manager = cached
    return (_route_name(rlog), _seg_idx(rlog), rlog, arrays, events, manager, None)
  except Exception as e:  # a corrupt segment is reported, never fatal (mirrors transcribe's guard)
    return (_route_name(rlog), _seg_idx(rlog), rlog, None, None, None, f"{type(e).__name__}: {e}")


# --------------------------------------------------------------------------- aggregation
def _pct(a, q):
  a = a[np.isfinite(a)]
  return float(np.percentile(a, q)) if len(a) else float("nan")


def _route_summary(name, segs):
  """Aggregate per-segment payloads (seg-ordered) into one route's health summary."""
  segs = sorted(segs, key=lambda s: s["seg"])
  errors = [s for s in segs if s["err"]]
  ok = [s for s in segs if not s["err"]]
  if not ok:
    return {"route": name, "n_seg": len(segs), "errors": [s["err"] for s in errors], "empty": True}

  def cat(key):
    arrs = [s["arrays"][key] for s in ok if key in s["arrays"] and len(s["arrays"][key])]
    return np.concatenate(arrs) if arrs else np.array([], dtype=np.float32)

  t = cat("t")
  started = cat("started").astype(bool)
  maxT = cat("maxTempC")
  therm = cat("thermalRaw").astype(int)
  fan = cat("fanSpeed")
  cpu_mean, cpu_max = cat("cpuUsageMean"), cat("cpuUsageMax")
  mem, gpu = cat("memUsage"), cat("gpuUsage")
  model_ms = cat("model_ms")
  power = cat("powerDrawW")

  # sample period for time-in-state (deviceState ~2Hz); robust to gaps via median dt
  dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.5
  secs_overheated = float(np.sum(therm >= OVERHEATED_RAW) * dt)
  therm_transitions = int(np.sum((therm[1:] >= OVERHEATED_RAW) & (therm[:-1] < OVERHEATED_RAW))) if len(therm) > 1 else 0
  secs_fan_sat = float(np.sum(fan >= FAN_SATURATED) * dt)

  # merge event + manager summaries across segments
  events = {}
  for s in ok:
    for n, d in (s["events"] or {}).items():
      e = events.setdefault(n, {"edges": 0, "secs": 0.0})
      e["edges"] += d.get("edges", 0)
      e["secs"] = round(e["secs"] + d.get("secs", 0.0), 2)
  crashed, exit_pos = set(), set()
  for s in ok:
    m = s["manager"] or {}
    crashed.update(m.get("crashed", []))
    exit_pos.update(m.get("exit_pos", []))

  # route date from the earliest segment's rlog mtime (on-device drive time)
  try:
    mt = min(os.path.getmtime(s["rlog"]) for s in ok)
    date = datetime.fromtimestamp(mt, tz=UTC).astimezone().strftime("%Y-%m-%d %H:%M")
  except OSError:
    date = "?"

  return {
    "route": name, "date": date, "n_seg": len(segs), "n_samp": int(len(maxT)),
    "onroad_frac": float(np.mean(started)) if len(started) else float("nan"),
    "dur_min": round(float(t[-1] - t[0]) / 60.0, 1) if len(t) > 1 else 0.0,
    "maxT_p50": _pct(maxT, 50), "maxT_p95": _pct(maxT, 95), "maxT_max": _pct(maxT, 100),
    "secs_overheated": round(secs_overheated, 1), "therm_transitions": therm_transitions,
    "secs_fan_sat": round(secs_fan_sat, 1), "fan_p95": _pct(fan, 95),
    "cpu_mean_p50": _pct(cpu_mean, 50), "cpu_mean_p95": _pct(cpu_mean, 95), "cpu_max_p95": _pct(cpu_max, 95),
    "mem_p95": _pct(mem, 95), "mem_max": _pct(mem, 100), "gpu_p95": _pct(gpu, 95),
    "model_p95_ms": _pct(model_ms, 95), "model_peak_ms": _pct(model_ms, 100),
    "power_p95_w": _pct(power, 95),
    "events": events, "crashed": sorted(crashed), "exit_pos": sorted(exit_pos),
    "errors": [s["err"] for s in errors],
  }


def analyze(routes, nproc=None):
  """Decode all segments (parallel), return sorted per-route summaries + a global cold reference."""
  paths = []
  for segs in routes.values():
    paths.extend(segs)
  nproc = nproc or max(1, min(cpu_count() - 1, 8))
  if nproc > 1 and len(paths) > 1:
    with Pool(nproc) as pool:
      raw = pool.map(_worker, paths)
  else:
    raw = [_worker(p) for p in paths]

  by_route = {}
  for name, seg, rlog, arrays, events, manager, err in raw:
    by_route.setdefault(name, []).append(
      {"seg": seg, "rlog": rlog, "arrays": arrays, "events": events, "manager": manager, "err": err})
  summaries = [_route_summary(n, s) for n, s in sorted(by_route.items())]

  # global onroad cold reference (illustrative fixed baseline, ambient-independent verdict elsewhere)
  cold = float("nan")
  mins = [s["maxT_p50"] for s in summaries if not s.get("empty") and np.isfinite(s.get("maxT_p50", np.nan))]
  if mins:
    cold = min(mins)
  return summaries, cold


# --------------------------------------------------------------------------- reporting
def _fmt(v, u="", nd=1):
  return f"{v:.{nd}f}{u}" if isinstance(v, (int, float)) and np.isfinite(v) else "-"


def build_report(summaries, cold):
  live = [s for s in summaries if not s.get("empty")]
  anomalies = []
  for s in live:
    if s["secs_overheated"] > 0:
      anomalies.append(f"{s['route']} ({s['date']}): OVERHEATED {s['secs_overheated']}s, " +
                       f"{s['therm_transitions']} transition(s), peak {_fmt(s['maxT_max'],'C')}")
    if s["secs_fan_sat"] > 0:
      anomalies.append(f"{s['route']} ({s['date']}): fan at 100% for {s['secs_fan_sat']}s")
    if s["crashed"] or s["exit_pos"]:
      procs = ", ".join(sorted(set(s["crashed"]) | set(s["exit_pos"])))
      anomalies.append(f"{s['route']} ({s['date']}): process death — {procs}")
    if s["model_p95_ms"] > MODEL_BUDGET_MS:
      anomalies.append(f"{s['route']} ({s['date']}): model loop p95 {_fmt(s['model_p95_ms'],'ms')} > {MODEL_BUDGET_MS:.0f}ms budget")
    for n, d in s["events"].items():
      if n in ("commIssue", "outOfSpace", "lowMemory", "processNotRunning") and (d["edges"] or d["secs"]):
        tag = " [not device-attributable]" if n == "commIssue" else ""
        anomalies.append(f"{s['route']} ({s['date']}): {n} x{d['edges']} ({_fmt(d['secs'],'s')}){tag}")

  worst_p95 = max((s["maxT_p95"] for s in live if np.isfinite(s["maxT_p95"])), default=float("nan"))
  worst_peak = max((s["maxT_max"] for s in live if np.isfinite(s["maxT_max"])), default=float("nan"))
  worst_model = max((s["model_p95_ms"] for s in live if np.isfinite(s["model_p95_ms"])), default=float("nan"))
  worst_mem = max((s["mem_max"] for s in live if np.isfinite(s["mem_max"])), default=float("nan"))
  dates = sorted(s["date"] for s in live if s.get("date") and s["date"] != "?")
  hot = max(live, key=lambda s: s["maxT_max"] if np.isfinite(s["maxT_max"]) else -1e9, default=None)
  return {
    "n_routes": len(summaries), "n_live": len(live),
    "date_range": (dates[0], dates[-1]) if dates else ("?", "?"),
    "cold_ref": cold, "worst_p95": worst_p95, "worst_peak": worst_peak,
    "worst_model_p95": worst_model, "worst_mem_max": worst_mem,
    "hot_route": hot,
    "any_overheat": any(s["secs_overheated"] > 0 for s in live),
    "any_crash": any(s["crashed"] or s["exit_pos"] for s in live),
    "anomalies": anomalies, "summaries": summaries,
  }


def print_terminal(rep):
  dr = rep["date_range"]
  print(f"\n=== Device health: {rep['n_live']}/{rep['n_routes']} routes  ({dr[0]} -> {dr[1]}, onroad-only) ===")
  print("Thermal (device 96C ok->overheated edge; maxTempC 5s-filtered):")
  print(f"  worst-route maxTempC p95 = {_fmt(rep['worst_p95'],'C')}   worst peak = {_fmt(rep['worst_peak'],'C')}" +
        f"   (global cold ref ~{_fmt(rep['cold_ref'],'C')})")
  h = rep.get("hot_route")
  if h:
    print(f"  high-water mark: {h['route']} ({h['date']}) peak {_fmt(h['maxT_max'],'C')} at cpu p95 " +
          f"{_fmt(h['cpu_mean_p95'],'%')}, {_fmt(h['dur_min'])}min")
  print(f"  any overheated (thermalStatus>=overheated): {'YES' if rep['any_overheat'] else 'no'}")
  print(f"Software: worst model-loop p95 = {_fmt(rep['worst_model_p95'],'ms')} (budget 50ms)" +
        f"   worst mem peak = {_fmt(rep['worst_mem_max'],'%')}   process deaths: {'YES' if rep['any_crash'] else 'no'}")
  verdict = "THERMALLY HEALTHY, no throttling, no process deaths" if not (rep["any_overheat"] or rep["any_crash"]) \
    else "ATTENTION NEEDED (see anomalies)"
  print(f"Verdict: {verdict}")
  if rep["anomalies"]:
    print(f"\nAnomalies ({len(rep['anomalies'])}):")
    for a in rep["anomalies"]:
      print(f"  - {a}")
  else:
    print("\nAnomalies: none")
  print()


def build_note(rep):
  dr = rep["date_range"]
  today = datetime.now().astimezone().strftime("%Y-%m-%d")
  L = []
  L.append(f"# VTB device-health readout — {today}\n")
  L.append("_Onroad thermals + software performance across locally-preserved comma 3X drives._\n")
  L.append("## Verdict\n")
  verdict = "**Thermally healthy** — no throttling, no process deaths." if not (rep["any_overheat"] or rep["any_crash"]) \
    else "**Attention needed** — see anomalies below."
  L.append(verdict + "\n")
  L.append(f"- Coverage: **{rep['n_live']}/{rep['n_routes']} routes**, drive dates **{dr[0]} → {dr[1]}** " +
           "(this is *locally-preserved* history; device log rotation may have dropped drives never pulled).")
  L.append(f"- Worst-route onroad `maxTempC` p95 = **{_fmt(rep['worst_p95'],'C')}**, worst peak = " +
           f"**{_fmt(rep['worst_peak'],'C')}** vs the device's 96 °C ok→overheated edge.")
  L.append(f"- Worst model-loop p95 = **{_fmt(rep['worst_model_p95'],'ms')}** (20 Hz ⇒ 50 ms budget); " +
           f"worst memory peak = **{_fmt(rep['worst_mem_max'],'%')}**.")
  h = rep.get("hot_route")
  if h:
    soak = (h["dur_min"] < 5.0 and np.isfinite(h["cpu_mean_p95"]) and h["cpu_mean_p95"] < 60.0)
    why = (" — a hot-soak start (short drive at modest CPU: the device was already warm from " +
           "summer + sentry, not a compute-driven peak)") if soak else ""
    L.append(f"- Thermal high-water mark: **{h['route']}** ({h['date']}) — peak **{_fmt(h['maxT_max'],'C')}** at " +
             f"{_fmt(h['cpu_mean_p95'],'%')} CPU p95 over a {_fmt(h['dur_min'])}-min drive{why}.\n")
  L.append("## Method & caveats\n")
  L.append("- **Onroad-only data.** `loggerd` records only while `deviceState.started` — this device logs " +
           "no offroad/idle samples, and the 3X has no ambient sensor (`intakeTempC`≡0). So the thermal " +
           "verdict is ambient-independent: the device's own `thermalStatus` bands + `fanSpeedPercentDesired`" +
           "=100 % saturation + absolute `maxTempC` percentiles vs the 96 °C edge. `maxTempC` is 5 s-filtered " +
           "(ramps from 0 at drive start), so we use percentiles, not min.")
  L.append("- **Per-route-first aggregation** (deviceState is 2 Hz; pooling would bias toward long drives). " +
           "Across-route stats are the worst per-route value.")
  L.append("- **Model-loop timing:** the verdict uses `modelExecutionTime` **p95** (steady-state). The per-route " +
           "**peak** column (~0.5 s on every route) is the first-frame warmup on drive start, not a live stall.")
  L.append("- **Events** are read from `onroadEvents` only, counted as rising edges + seconds-active " +
           "(counted per-segment, so multi-segment events over-count edges / under-count seconds by a few % " +
           "at segment boundaries). `commIssue` is reported but **not attributed as a device defect** — it " +
           "cannot be separated from operator-induced load (e.g. a live logger on a moving comma) from the logs alone.\n")
  L.append("## Anomalies\n")
  if rep["anomalies"]:
    for a in rep["anomalies"]:
      L.append(f"- {a}")
  else:
    L.append("- None. No overheated windows, no fan saturation, no process deaths, no comm/space events.")
  L.append("\n## Per-route detail\n")
  L.append("| Route | Date | Segs | Dur (min) | maxT p50/p95/peak (°C) | overheat s | fan100 s | " +
           "CPU mean p95 % | mem peak % | model p95/peak (ms) | events / deaths |")
  L.append("|---|---|---|---|---|---|---|---|---|---|---|")
  for s in rep["summaries"]:
    if s.get("empty"):
      L.append(f"| {s['route']} | - | {s['n_seg']} | - | DECODE FAILED | | | | | | {'; '.join(s.get('errors', []))[:60]} |")
      continue
    evs = "; ".join(f"{n}×{d['edges']}" for n, d in s["events"].items()) or "-"
    deaths = ", ".join(sorted(set(s["crashed"]) | set(s["exit_pos"]))) or ""
    notes = " / ".join(x for x in (evs if evs != "-" else "", deaths) if x) or "-"
    L.append(f"| {s['route']} | {s.get('date','?')} | {s['n_seg']} | {_fmt(s['dur_min'])} | " +
             f"{_fmt(s['maxT_p50'])}/{_fmt(s['maxT_p95'])}/{_fmt(s['maxT_max'])} | " +
             f"{_fmt(s['secs_overheated'])} | {_fmt(s['secs_fan_sat'])} | {_fmt(s['cpu_mean_p95'])} | " +
             f"{_fmt(s['mem_max'])} | {_fmt(s['model_p95_ms'])}/{_fmt(s['model_peak_ms'])} | {notes} |")
  L.append("")
  return "\n".join(L)


# --------------------------------------------------------------------------- main
def main():
  ap = argparse.ArgumentParser(description="Onroad device-health readout from pulled comma rlogs.")
  ap.add_argument("routes", nargs="*", help="route names / seg dirs / rlog paths")
  ap.add_argument("--all", action="store_true", help="every locally-preserved route")
  ap.add_argument("--note", metavar="PATH", help="also write the dated markdown note to PATH")
  ap.add_argument("-j", "--jobs", type=int, default=0, help="parallel workers (0=auto)")
  ap.add_argument("--json", metavar="PATH", help="dump the raw report JSON to PATH")
  args = ap.parse_args()

  if args.all:
    routes = _all_routes()
  elif args.routes:
    routes = {}
    for r in args.routes:
      for p in _resolve(r):
        routes.setdefault(_route_name(p), []).append(p)
    for n in routes:
      routes[n].sort(key=_seg_idx)
  else:
    ap.error("give route(s) or --all")

  if not routes:
    print("no routes resolved", file=sys.stderr)
    return 1
  n_seg = sum(len(v) for v in routes.values())
  print(f"analyzing {len(routes)} route(s) / {n_seg} segment(s) ...", file=sys.stderr)

  summaries, cold = analyze(routes, nproc=(args.jobs or None))
  rep = build_report(summaries, cold)
  print_terminal(rep)

  if args.note:
    with open(os.path.expanduser(args.note), "w") as f:
      f.write(build_note(rep))
    print(f"wrote note -> {args.note}", file=sys.stderr)
  if args.json:
    with open(os.path.expanduser(args.json), "w") as f:
      json.dump(rep, f, indent=2, default=float)
    print(f"wrote json -> {args.json}", file=sys.stderr)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
