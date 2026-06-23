#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VTB inertia-comp system-ID: derive STEER_INERTIA_J from baseline drive logs.

The cooperative-steering inertia feedforward subtracts J * alpha_wheel from the
measured driver torque before the deadzone + gain stage:

    tau_intent = tau_measured - J * alpha_wheel
      tau_measured = carState.steeringTorque                 (Nm)
      alpha_wheel  = LPF(d/dt steeringRateDeg, RC) in rad/s^2

During hands-off, openpilot-driven steering (latActive, not steeringPressed,
|tau| < deadzone) the driver applies ~no intent torque, so the torsion-bar
torque is the reaction that accelerates the free steering wheel's inertia plus
column damping / return effects:

    tau_measured ~= J*alpha + b*omega + k*theta + c

Regressing tau on [alpha, omega, theta, 1] over that set gives J as the
coefficient on alpha (b/k/c are nuisance terms, reported as sanity gauges).
This recomputes alpha with the EXACT discretisation the FF uses (DT_LAT_CTRL +
FirstOrderFilter) so the fitted J transfers 1:1 to coop_steering.py.

Usage:
  python tools/sunnypilot/vtb/fit_steer_inertia.py                 # both of today's drives
  python tools/sunnypilot/vtb/fit_steer_inertia.py --routes ROUTE_ID
  python tools/sunnypilot/vtb/fit_steer_inertia.py --rc 0.03 0.04 0.05   # sweep the LPF RC
"""
from __future__ import annotations

import argparse
import glob
import math
import os
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

from openpilot.tools.lib.logreader import LogReader
from openpilot.common.realtime import DT_CTRL
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.tesla.values import CarControllerParams

# Mirror coop_steering.py's discretisation + thresholds.
DT_LAT_CTRL = DT_CTRL * CarControllerParams.STEER_STEP
DEFAULT_RC = 0.04                  # STEER_ALPHA_FILTER_RC
DEADZONE_NM = 0.5                  # STEER_OVERRIDE_MIN_TORQUE
J_LIT_LO, J_LIT_HI = 0.05, 0.15    # literature plausibility band (kg*m^2)
GAP_S = 0.05                       # carState spacing above this starts a new contiguous run
ALPHA_FLOOR = 5.0                  # rad/s^2 — min |alpha| excitation for a usable ID sample


def default_routes() -> list[str]:
  root = os.path.expanduser("~/.comma/media/0/realdata")
  names = set()
  for d in glob.glob(os.path.join(root, "*--*--*")):
    base = os.path.basename(d)
    names.add(base.rsplit("--", 1)[0])
  return sorted(names)


def rlog_paths(route: str) -> list[str]:
  root = os.path.expanduser("~/.comma/media/0/realdata")
  segs = glob.glob(os.path.join(root, f"{route}--*", "rlog.zst"))
  # sort by numeric segment index
  return sorted(segs, key=lambda p: int(os.path.basename(os.path.dirname(p)).rsplit("--", 1)[1]))


def load_route(route: str) -> dict[str, np.ndarray]:
  """Extract time-aligned carState + latActive at carState's native 100 Hz."""
  paths = rlog_paths(route)
  if not paths:
    raise SystemExit(f"no rlog.zst found locally for route {route} (expected under ~/.comma/media/0/realdata)")
  lr = LogReader(paths, sort_by_time=True)

  cs_t, tau, rate, angle, pressed, vego = [], [], [], [], [], []
  cc_t, cc_lat = [], []
  for msg in lr:
    w = msg.which()
    if w == "carState":
      cs = msg.carState
      cs_t.append(msg.logMonoTime)
      tau.append(cs.steeringTorque)
      rate.append(cs.steeringRateDeg)
      angle.append(cs.steeringAngleDeg)
      pressed.append(cs.steeringPressed)
      vego.append(cs.vEgo)
    elif w == "carControl":
      cc_t.append(msg.logMonoTime)
      cc_lat.append(bool(msg.carControl.latActive))

  cs_t = np.array(cs_t, dtype=np.float64) * 1e-9
  out = dict(t=cs_t,
             tau=np.array(tau, dtype=np.float64),
             rate=np.array(rate, dtype=np.float64),
             angle=np.array(angle, dtype=np.float64),
             pressed=np.array(pressed, dtype=bool),
             vego=np.array(vego, dtype=np.float64))
  # align latActive (zero-order hold of most-recent carControl) onto carState timeline
  if cc_t:
    cc_t = np.array(cc_t, dtype=np.float64) * 1e-9
    cc_lat = np.array(cc_lat, dtype=bool)
    idx = np.searchsorted(cc_t, cs_t, side="right") - 1
    out["lat"] = np.where(idx >= 0, cc_lat[np.clip(idx, 0, len(cc_lat) - 1)], False)
  else:
    out["lat"] = np.zeros_like(cs_t, dtype=bool)
  out["route"] = route
  return out


def contiguous_runs(t: np.ndarray) -> list[tuple[int, int]]:
  """Index ranges where consecutive carState samples are < GAP_S apart."""
  if len(t) == 0:
    return []
  brk = np.where(np.diff(t) > GAP_S)[0]
  starts = np.concatenate(([0], brk + 1))
  ends = np.concatenate((brk + 1, [len(t)]))
  return [(s, e) for s, e in zip(starts, ends) if e - s >= 3]


def resample_and_alpha(d: dict, rc: float) -> dict[str, np.ndarray]:
  """Resample onto a uniform DT_LAT_CTRL grid per contiguous run and compute the FF's alpha."""
  cols = {k: [] for k in ("tau", "rate", "angle", "vego", "alpha", "pressed", "lat")}
  for s, e in contiguous_runs(d["t"]):
    t = d["t"][s:e]
    grid = np.arange(t[0], t[-1], DT_LAT_CTRL)
    if len(grid) < 4:
      continue
    g_tau = np.interp(grid, t, d["tau"][s:e])
    g_rate = np.interp(grid, t, d["rate"][s:e])
    g_angle = np.interp(grid, t, d["angle"][s:e])
    g_vego = np.interp(grid, t, d["vego"][s:e])
    # boolean signals: nearest-sample
    nn = np.clip(np.searchsorted(t, grid, side="right") - 1, 0, len(t) - 1)
    g_pressed = d["pressed"][s:e][nn]
    g_lat = d["lat"][s:e][nn]

    # alpha exactly as coop_steering.py: diff(rate_deg)/dt -> LPF -> radians
    filt = FirstOrderFilter(0.0, rc, DT_LAT_CTRL, initialized=False)
    alpha = np.zeros_like(g_rate)
    prev = g_rate[0]
    for i in range(len(g_rate)):
      raw = (g_rate[i] - prev) / DT_LAT_CTRL
      alpha[i] = math.radians(filt.update(raw))
      prev = g_rate[i]

    cols["tau"].append(g_tau); cols["rate"].append(g_rate); cols["angle"].append(g_angle)
    cols["vego"].append(g_vego); cols["alpha"].append(alpha)
    cols["pressed"].append(g_pressed); cols["lat"].append(g_lat)
  if not cols["tau"]:
    return {k: np.array([]) for k in cols}
  return {k: np.concatenate(v) for k, v in cols.items()}


def robust_fit(X: np.ndarray, y: np.ndarray, iters: int = 12) -> tuple[np.ndarray, np.ndarray]:
  """IRLS Huber regression. Returns (coef, residuals)."""
  w = np.ones(len(y))
  coef = np.zeros(X.shape[1])
  for _ in range(iters):
    sw = np.sqrt(w)
    coef, *_ = np.linalg.lstsq(X * sw[:, None], y * sw, rcond=None)
    r = y - X @ coef
    s = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-9
    a = np.abs(r) / s
    k = 1.345
    w = np.where(a <= k, 1.0, k / np.maximum(a, 1e-9))
  return coef, y - X @ coef


def design(seg: dict, mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  alpha = seg["alpha"][mask]
  omega = np.radians(seg["rate"][mask])
  theta = np.radians(seg["angle"][mask])
  X = np.column_stack([alpha, omega, theta, np.ones_like(alpha)])
  return X, seg["tau"][mask]


def id_mask(seg: dict) -> np.ndarray:
  return (seg["lat"] & ~seg["pressed"]
          & (np.abs(seg["tau"]) < DEADZONE_NM)
          & (np.abs(seg["alpha"]) > ALPHA_FLOOR))


def r2(y, resid):
  ss = np.sum((y - y.mean()) ** 2)
  return 1.0 - np.sum(resid ** 2) / ss if ss > 0 else float("nan")


def fit_report(seg: dict, rc: float, route_label: str) -> dict:
  mask = id_mask(seg)
  n = int(mask.sum())
  total = len(seg["tau"])
  cov = 100.0 * n / total if total else 0.0
  out = {"route": route_label, "rc": rc, "n_id": n, "n_total": total, "coverage_pct": cov}
  if n < 200:
    out["status"] = "INSUFFICIENT"
    return out

  X, y = design(seg, mask)
  coef, resid = robust_fit(X, y)
  J, b, k, c = coef
  out.update(J=J, damping=b, stiffness=k, offset=c, r2=r2(y, resid),
             resid_rms=float(np.sqrt(np.mean(resid ** 2))), status="OK")

  # bootstrap CI on J
  rng = np.random.default_rng(0)
  Js = []
  for _ in range(300):
    bi = rng.integers(0, n, n)
    cf, _ = robust_fit(X[bi], y[bi], iters=6)
    Js.append(cf[0])
  out["J_lo"], out["J_hi"] = np.percentile(Js, [2.5, 97.5])

  # speed-bin invariance (J should be ~speed-independent if it's really inertia)
  v = seg["vego"][mask]
  bins = [(0, 8), (8, 18), (18, 100)]
  out["J_by_speed"] = []
  for lo, hi in bins:
    bm = (v >= lo) & (v < hi)
    if bm.sum() >= 150:
      cf, _ = robust_fit(X[bm], y[bm], iters=8)
      out["J_by_speed"].append((lo, hi, int(bm.sum()), float(cf[0])))

  # forward-check: does tau - J*alpha shrink the hands-off torque RMS?
  tau_raw_rms = float(np.sqrt(np.mean(y ** 2)))
  tau_int = y - J * seg["alpha"][mask]
  out["tau_raw_rms"], out["tau_intent_rms"] = tau_raw_rms, float(np.sqrt(np.mean(tau_int ** 2)))
  return out


def banner(s):
  print("\n" + "=" * 78 + f"\n{s}\n" + "=" * 78)


def print_fit(r: dict):
  if r["status"] == "INSUFFICIENT":
    print(f"  [{r['route']}] rc={r['rc']:.3f}  coverage {r['coverage_pct']:.2f}% "
          f"({r['n_id']}/{r['n_total']})  -> INSUFFICIENT excitation for a fit")
    return
  print(f"  [{r['route']}] rc={r['rc']:.3f}  J = {r['J']:.4f} kg*m^2 "
        f"[{r['J_lo']:.4f}, {r['J_hi']:.4f}]  R^2={r['r2']:.3f}  "
        f"resid_rms={r['resid_rms']:.3f} Nm  n={r['n_id']} ({r['coverage_pct']:.1f}% cov)")
  print(f"        nuisance: damping={r['damping']:.4f}  stiffness={r['stiffness']:.4f}  offset={r['offset']:.3f}")
  if r.get("J_by_speed"):
    sb = "  ".join(f"{lo}-{hi}m/s:{j:.3f}(n={n})" for lo, hi, n, j in r["J_by_speed"])
    print(f"        J vs speed: {sb}")
  print(f"        forward-check: hands-off tau RMS {r['tau_raw_rms']:.3f} -> intent RMS "
        f"{r['tau_intent_rms']:.3f} Nm  ({100*(1-r['tau_intent_rms']/r['tau_raw_rms']):.0f}% reduction)")


def main():
  ap = argparse.ArgumentParser(description="Derive STEER_INERTIA_J from baseline VTB drive logs")
  ap.add_argument("--routes", nargs="*", default=None, help="route names (default: all local drives)")
  ap.add_argument("--rc", nargs="*", type=float, default=[DEFAULT_RC], help="alpha LPF RC value(s) to try")
  args = ap.parse_args()

  routes = args.routes or default_routes()
  banner(f"VTB inertia-J fit  |  DT_LAT_CTRL={DT_LAT_CTRL*1000:.1f}ms (STEER_STEP={CarControllerParams.STEER_STEP})  "
         f"deadzone={DEADZONE_NM}Nm  alpha_floor={ALPHA_FLOOR}rad/s^2")
  print(f"routes: {routes}")

  raw = {r: load_route(r) for r in routes}
  print("loaded:  " + "  ".join(f"{r}={len(raw[r]['t'])} carState" for r in routes))

  best = None
  for rc in args.rc:
    banner(f"RC = {rc:.3f} s")
    per_route = {}
    for r in routes:
      seg = resample_and_alpha(raw[r], rc)
      rep = fit_report(seg, rc, r)
      per_route[r] = (seg, rep)
      print_fit(rep)

    # pooled fit across all routes (split-half consistency too)
    segs = [s for s, _ in per_route.values()]
    pooled = {k: np.concatenate([s[k] for s in segs]) for k in segs[0]}
    prep = fit_report(pooled, rc, "POOLED")
    print_fit(prep)
    if prep["status"] == "OK":
      mask = id_mask(pooled)
      X, y = design(pooled, mask)
      half = len(y) // 2
      (cf1, _), (cf2, _) = robust_fit(X[:half], y[:half]), robust_fit(X[half:], y[half:])
      print(f"        split-half J: {cf1[0]:.4f} | {cf2[0]:.4f}  (consistency check)")
      if best is None:
        best = prep

  banner("RECOMMENDATION")
  if best is None or best["status"] != "OK":
    print("  Data lacks sufficient hands-off wheel-acceleration excitation for a confident J fit.")
    print("  -> Do a targeted excitation drive: brisk hand-off slaloms / lane wanders in a lot,")
    print("     or sweep the wheel through curves at varied speed, then re-run /vtb-tune.")
    sys.exit(2)
  J = best["J"]
  J_clamped = float(np.clip(J, J_LIT_LO, J_LIT_HI))
  in_band = J_LIT_LO <= J <= J_LIT_HI
  print(f"  Fitted J (pooled, robust) = {J:.4f} kg*m^2  CI[{best['J_lo']:.4f}, {best['J_hi']:.4f}]")
  print(f"  Literature band [{J_LIT_LO}, {J_LIT_HI}] -> {'IN BAND' if in_band else 'OUT OF BAND (clamped)'}")
  print(f"  Recommended STEER_INERTIA_J = {round(J_clamped, 3)}  (current placeholder 0.08)")
  if best["tau_intent_rms"] >= best["tau_raw_rms"]:
    print("  WARNING: forward-check did not reduce hands-off torque RMS — treat J as low-confidence.")


if __name__ == "__main__":
  main()
