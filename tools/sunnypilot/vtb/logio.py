"""Shared rlog parse + signal-cache layer for the VTB offline tools.

One route/segment resolver, one zero-order-hold aligner, and one multi-service signal
extractor with a per-segment parsed-signal cache (`signals.npz` + `signals.meta.json`
next to each `rlog.zst`, mirroring transcribe_events' `events.jsonl` sidecar). This lets
`fit_steer_inertia` / `analyze_shadow` become thin adapters so an rlog is decoded ONCE
and reused across tools and across runs (pulled rlogs are immutable, so the cache is
durable).

Import-light: only stdlib + numpy at module load. `LogReader` is imported lazily inside
the decode path so `import logio` stays cheap for headless callers (live_watch).

Behavior note: `read_signals` decodes each segment with a fresh `LogReader([seg])` and
concatenates across segments in `paths` order. `LogReader(sort_by_time=True)` only sorts
WITHIN a segment, never across segment boundaries, so this is bit-identical to the old
`LogReader(all_paths)` single-pass loops while bounding memory (no `__lrs` accumulation).
"""
import glob
import json
import os
import zipfile

import sys

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

from openpilot.tools.sunnypilot.vtb.vtb_constants import LOCAL_LOG_ROOTS

SIGNALS_SCHEMA = 1
_NPZ = "signals.npz"
_META = "signals.meta.json"

# The VTB signal set: {service: {field_path: numpy dtype}}. A field_path may be dotted
# for a nested capnp group (e.g. "coopSteering.alphaFilt"). Each tool requests a subset;
# one signals.npz superset serves them all (fit reads carState+carControl, analyze_shadow
# reads carStateSP+carState).
VTB_SIGNAL_SPEC: dict[str, dict[str, type]] = {
  "carState": {
    "steeringTorque": np.float64, "steeringRateDeg": np.float64, "steeringAngleDeg": np.float64,
    "steeringPressed": np.bool_, "vEgo": np.float64,
  },
  "carControl": {"latActive": np.bool_},
  "carStateSP": {
    "coopSteering.coopActive": np.bool_, "coopSteering.inertiaCompActive": np.bool_,
    "coopSteering.shadowActive": np.bool_, "coopSteering.alphaFilt": np.float64,
    "coopSteering.tauInertia": np.float64, "coopSteering.tauIntent": np.float64,
    "coopSteering.inertiaJUsed": np.float64, "coopSteering.angleOverride": np.float64,
  },
}


# --------------------------------------------------------------------------- resolvers
def _seg_idx(rlog_path: str) -> int:
  """Numeric segment index parsed from a `<route>--<idx>/rlog.zst` path."""
  return int(os.path.basename(os.path.dirname(rlog_path)).rsplit("--", 1)[1])


def resolve_segments(arg: str, roots=LOCAL_LOG_ROOTS, missing_ok: bool = False) -> list[str]:
  """Resolve `arg` to a seg-index-sorted list of rlog.zst paths.
   - an explicit rlog.zst file          -> [that file]
   - a segment / route / root directory -> its rlog.zst(s), recursive
   - a bare route name (000001xx--yyyy) -> <root>/<name>--*/rlog.zst across roots
  Raises SystemExit if nothing resolves, unless missing_ok. Generalizes the four
  hand-rolled globs (fit.rlog_paths, analyze.find_routes, live_watch.resolve_replay,
  transcribe.local_routes)."""
  arg_x = os.path.expanduser(arg)
  if os.path.isfile(arg_x):
    return [arg_x]
  if os.path.isdir(arg_x):
    segs = glob.glob(os.path.join(arg_x, "**", "rlog.zst"), recursive=True)
  else:
    segs = []
    for root in roots:
      segs += glob.glob(os.path.join(os.path.expanduser(root), f"{arg}--*", "rlog.zst"))
  segs = sorted(set(segs), key=_seg_idx)
  if not segs and not missing_ok:
    raise SystemExit(f"no rlog.zst found for {arg!r} (expected under {' or '.join(roots)})")
  return segs


def list_local_routes(roots=LOCAL_LOG_ROOTS, require_rlog: bool = True) -> list[str]:
  """Sorted unique route names under `roots` (basename minus the trailing --<seg>)."""
  names: set[str] = set()
  for root in roots:
    for d in glob.glob(os.path.join(os.path.expanduser(root), "*--*--*")):
      if require_rlog and not os.path.exists(os.path.join(d, "rlog.zst")):
        continue
      names.add(os.path.basename(d).rsplit("--", 1)[0])
  return sorted(names)


def group_routes(roots=LOCAL_LOG_ROOTS, routes_filter=None) -> dict[str, list[str]]:
  """{route: seg-sorted rlog paths} under `roots`, optionally filtered to `routes_filter`."""
  out: dict[str, list[str]] = {}
  for root in roots:
    for seg in glob.glob(os.path.join(os.path.expanduser(root), "*--*--*")):
      rlog = os.path.join(seg, "rlog.zst")
      if not os.path.exists(rlog):
        continue
      name = os.path.basename(seg).rsplit("--", 1)[0]
      if routes_filter and name not in routes_filter:
        continue
      out.setdefault(name, []).append(rlog)
  for name in out:
    out[name].sort(key=_seg_idx)
  return dict(sorted(out.items()))


# --------------------------------------------------------------------------- alignment
def zoh_align(src_t: np.ndarray, src_vals: np.ndarray, dst_t: np.ndarray, fill=None) -> np.ndarray:
  """Zero-order hold: for each `dst_t`, the most-recent `src` sample at/at-or-before it.
   - fill is None : dst samples before the first src time CLAMP to src_vals[0] (analyze vEgo)
   - fill given   : dst samples before the first src time take `fill`           (fit latActive)
  Empty src -> zeros(len(dst)) of src dtype (clamp) or full(len(dst), fill)."""
  src_t = np.asarray(src_t)
  src_vals = np.asarray(src_vals)
  dst_t = np.asarray(dst_t)
  if len(src_t) == 0:
    if fill is None:
      return np.zeros(len(dst_t), dtype=src_vals.dtype)
    return np.full(len(dst_t), fill)
  idx = np.searchsorted(src_t, dst_t, side="right") - 1
  held = src_vals[np.clip(idx, 0, len(src_vals) - 1)]
  if fill is None:
    return held
  return np.where(idx >= 0, held, fill)


# --------------------------------------------------------------------------- extraction
def _dotted_get(obj, path: str):
  for part in path.split("."):
    obj = getattr(obj, part)
  return obj


def _flat_keys(spec: dict) -> dict[str, type]:
  """{f'{service}~{field}': dtype} for every field + a float64 '~t' per service."""
  keys: dict[str, type] = {}
  for svc, fields in spec.items():
    keys[f"{svc}~t"] = np.float64
    for field, dt in fields.items():
      keys[f"{svc}~{field}"] = dt
  return keys


def _decode_segment(rlog: str, spec: dict) -> dict[str, np.ndarray]:
  """Decode ONE segment into a flat {f'{service}~{field}': ndarray} dict (incl. '~t' in seconds)."""
  from openpilot.tools.lib.logreader import LogReader
  cols: dict[str, dict[str, list]] = {svc: {"t": [], **{f: [] for f in fields}} for svc, fields in spec.items()}
  for msg in LogReader([rlog], sort_by_time=True):
    w = msg.which()
    if w not in spec:
      continue
    sub = getattr(msg, w)
    cols[w]["t"].append(msg.logMonoTime)
    for field in spec[w]:
      cols[w][field].append(_dotted_get(sub, field))
  flat: dict[str, np.ndarray] = {}
  for svc, fields in spec.items():
    flat[f"{svc}~t"] = np.array(cols[svc]["t"], dtype=np.float64) * 1e-9
    for field, dt in fields.items():
      flat[f"{svc}~{field}"] = np.array(cols[svc][field], dtype=dt)
  return flat


def _load_cache(rlog: str, spec: dict):
  """Return the requested flat arrays if a fresh, superset-satisfying cache exists, else None."""
  sd = os.path.dirname(rlog)
  mp, npzp = os.path.join(sd, _META), os.path.join(sd, _NPZ)
  if not (os.path.exists(mp) and os.path.exists(npzp)):
    return None
  try:
    with open(mp) as f:
      meta = json.load(f)
    st = os.stat(rlog)
  except (OSError, ValueError):
    return None
  if not (meta.get("schema") == SIGNALS_SCHEMA and meta.get("rlog_size") == st.st_size
          and abs(meta.get("rlog_mtime", -1.0) - st.st_mtime) < 1e-6):
    return None
  have = meta.get("dtypes", {})
  want = _flat_keys(spec)
  if any(have.get(k) != np.dtype(dt).name for k, dt in want.items()):
    return None  # a requested field is absent or dtype-mismatched -> miss (decode superset)
  try:
    with np.load(npzp) as z:
      return {k: z[k] for k in want}
  except (OSError, ValueError, zipfile.BadZipFile):
    return None


def _write_cache(rlog: str, flat: dict[str, np.ndarray]) -> None:
  """Atomically write signals.npz + signals.meta.json, merging (never shrinking) any existing cache."""
  sd = os.path.dirname(rlog)
  npzp, mp = os.path.join(sd, _NPZ), os.path.join(sd, _META)
  merged: dict[str, np.ndarray] = {}
  if os.path.exists(npzp):
    try:
      with np.load(npzp) as z:
        merged = {k: z[k] for k in z.files}
    except (OSError, ValueError, zipfile.BadZipFile):
      merged = {}
  merged.update(flat)  # same rlog -> same data; new decode wins and adds any missing fields
  st = os.stat(rlog)
  meta = {"schema": SIGNALS_SCHEMA, "rlog_size": st.st_size, "rlog_mtime": st.st_mtime,
          "keys": sorted(merged), "dtypes": {k: merged[k].dtype.name for k in merged}}
  npz_tmp = npzp + ".tmp"
  with open(npz_tmp, "wb") as f:            # file object -> np.savez won't append ".npz"
    np.savez(f, **merged)
  os.replace(npz_tmp, npzp)
  meta_tmp = mp + ".tmp"
  with open(meta_tmp, "w") as f:
    json.dump(meta, f)
  os.replace(meta_tmp, mp)


def read_signals(paths, spec: dict = VTB_SIGNAL_SPEC, use_cache: bool = True) -> dict[str, dict[str, np.ndarray]]:
  """Decode segment(s) once into {service: {"t": seconds, "<field>": ndarray}}, concatenated
  across `paths`. Reads/writes the per-segment signals.npz cache when use_cache."""
  per_seg: list[dict[str, np.ndarray]] = []
  for p in paths:
    flat = _load_cache(p, spec) if use_cache else None
    if flat is None:
      flat = _decode_segment(p, spec)
      if use_cache:
        _write_cache(p, flat)
    per_seg.append(flat)
  out: dict[str, dict[str, np.ndarray]] = {}
  for svc, fields in spec.items():
    out[svc] = {}
    for field, dt in [("t", np.float64), *fields.items()]:
      fk = f"{svc}~{field}"
      arrs = [s[fk] for s in per_seg if fk in s]
      out[svc][field] = np.concatenate(arrs) if arrs else np.array([], dtype=dt)
  return out


def main() -> int:
  """Pre-warm the signals.npz cache for route(s) so a subsequent parallel report run
  (fit_steer_inertia + analyze_shadow) hits the cache instead of each decoding the rlogs."""
  import argparse
  ap = argparse.ArgumentParser(description="Pre-warm the VTB signal cache (signals.npz).")
  ap.add_argument("routes", nargs="+", help="route names / seg dirs / rlog paths")
  args = ap.parse_args()
  segs = 0
  for r in args.routes:
    paths = resolve_segments(r, missing_ok=True)
    if not paths:
      print(f"warm: no rlogs for {r!r}")
      continue
    read_signals(paths)   # decode-if-needed + write signals.npz per segment
    segs += len(paths)
  print(f"warm: cached {segs} segment(s) across {len(args.routes)} route(s)")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
