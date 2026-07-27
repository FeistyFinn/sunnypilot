#!/usr/bin/env python3
"""Diagnose whether STEER_INERTIA_J is *identifiable* from the given VTB drives.

The inertia fit (fit_steer_inertia.py) needs samples that are simultaneously openpilot-steered
(latActive), hands-off (not steeringPressed), in the torque deadzone (|tau|<0.5Nm) AND excited
(|alpha|>5 rad/s^2), so the torsion-bar torque is the *pure inertial reaction* J*alpha. This tool
shows where that funnel collapses (almost always at the alpha floor) and renders a verdict on whether
J can be trusted from the data at all -- or whether to set J physically and tune by feel instead.

It is read-only and offline; it reuses fit_steer_inertia's loaders/masks verbatim so the qualifying
count here equals the fit's n_id exactly.

  python openpilot/tools/sunnypilot/vtb/diagnose_inertia_id.py                       # all local drives
  python openpilot/tools/sunnypilot/vtb/diagnose_inertia_id.py --routes ROUTE_ID ROUTE_ID
"""
from __future__ import annotations

import argparse
import os
import sys

# --- bootstrap: make 'openpilot' resolve to this repo root regardless of dir name (mirror fit_steer_inertia) ---
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
try:
  import openpilot  # noqa: F401
except ModuleNotFoundError:
  import types
  _pkg = types.ModuleType("openpilot")
  _pkg.__path__ = [_REPO]
  sys.modules["openpilot"] = _pkg

import numpy as np

from openpilot.tools.sunnypilot.vtb.fit_steer_inertia import (
  default_routes, load_route, resample_and_alpha, id_mask, fit_report, banner, pool_segs,
  DEADZONE_NM, ALPHA_FLOOR, DT_LAT_CTRL, DEFAULT_RC,
)

# A J is only "trustworthy" if ALL of these hold on the pooled qualifying set.
TARGET_N = 1000           # qualifying samples (~20 s of pure excitation at the 50 Hz control grid)
TARGET_ALPHA_P95 = 15.0   # rad/s^2 - dynamic range must clear the 5 floor by a wide margin
TARGET_R2 = 0.30          # the J*alpha model must explain a real fraction of variance
TARGET_CI_REL = 0.30      # bootstrap CI half-width < 30% of |J|

ALPHA_BINS = [0.0, 1.0, 2.0, 5.0, 10.0, 20.0, np.inf]
SPEED_BINS = [(0, 8), (8, 18), (18, 100)]   # m/s, matches fit_steer_inertia.fit_report

PHYSICS = f"""WHY OFFLINE J IS UNIDENTIFIABLE (a structural catch-22, not a data shortfall):
  The fit needs HANDS-OFF samples (driver intent ~ 0) so the torsion-bar torque is the pure
  inertial reaction J*alpha. But openpilot rate-limits the commanded steering angle
  (MAX_ANGLE_RATE = 5 deg/frame = {5.0 / DT_LAT_CTRL:.0f} deg/s) and jerk-limits curvature
  (MAX_LATERAL_JERK ~ 3.6 m/s^3) -- a jerk cap IS an angular-acceleration cap. So hands-off
  steering is smooth by design and |alpha| stays far below the 5 rad/s^2 floor.
  High |alpha| only occurs HANDS-ON (the driver moving the wheel), which the mask must exclude
  because then tau is driver intent, not J*alpha. The only on-device source above the floor is
  the engage resume-ramp (300 deg/s^2 ~ 5.2 rad/s^2) -- a one-shot at engagement."""

RECOMMENDATION = """RECOMMENDATION:
  Do NOT chase an offline J fit. Keep the physical default STEER_INERTIA_J = 0.08 kg*m^2
  (column + wheel inertia; literature band [0.05, 0.15]). J need not be precise: tau_inertia = J*alpha
  is hard-clamped to 2.5 Nm and deadzone-guarded, so J only affects FEEL during hands-on nudges.
  To tune feel, sweep TeslaCoopSteeringInertiaJ across 0.05 / 0.10 / 0.15 and pick by feel,
  validating each with analyze_shadow.py (hands-off torque-RMS reduction + 0 deadzone violations)."""


def masks(seg: dict) -> dict[str, np.ndarray]:
  """The ID-condition cascade, each stage a strict subset of the previous."""
  lat = seg["lat"]
  handsoff = lat & ~seg["pressed"]
  base = handsoff & (np.abs(seg["tau"]) < DEADZONE_NM)   # hands-off, openpilot-steering, no driver intent
  qual = id_mask(seg)                                    # base & (|alpha| > ALPHA_FLOOR) -- identical to the fit
  return {"lat": lat, "handsoff": handsoff, "base": base, "qual": qual}


def pct(x: np.ndarray, q: float) -> float:
  return float(np.percentile(x, q)) if len(x) else float("nan")


def print_funnel(seg: dict) -> None:
  m = masks(seg)
  total = len(seg["tau"])
  rows = [("total resampled", total, total),
          ("latActive", int(m["lat"].sum()), total),
          ("& hands-off", int(m["handsoff"].sum()), int(m["lat"].sum())),
          ("& in-deadzone (base pool)", int(m["base"].sum()), int(m["handsoff"].sum())),
          (f"& |alpha|>{ALPHA_FLOOR:g} (qualifying)", int(m["qual"].sum()), int(m["base"].sum()))]
  print(f"  {'stage':<32} {'count':>10} {'% of prev':>10} {'% of total':>11}")
  for label, n, prev in rows:
    pp = f"{100.0 * n / prev:.1f}%" if prev else "-"
    pt = f"{100.0 * n / total:.2f}%" if total else "-"
    print(f"  {label:<32} {n:>10} {pp:>10} {pt:>11}")


def print_alpha_dist(seg: dict) -> None:
  base = masks(seg)["base"]
  a = np.abs(seg["alpha"][base])
  if not len(a):
    print("  (no in-deadzone hands-off samples)")
    return
  print(f"  |alpha| over the base pool (n={len(a)}):  " +
        f"p50={pct(a, 50):.2f}  p90={pct(a, 90):.2f}  p95={pct(a, 95):.2f}  " +
        f"p99={pct(a, 99):.2f}  max={a.max():.2f}  rad/s^2")
  counts, _ = np.histogram(a, bins=ALPHA_BINS)
  labels = ["[0,1)", "[1,2)", "[2,5)", "[5,10)", "[10,20)", "[20,inf)"]
  print("  histogram:  " + "  ".join(f"{lab}:{c}({100.0 * c / len(a):.1f}%)" for lab, c in zip(labels, counts, strict=True)))
  frac_below = float((a <= ALPHA_FLOOR).mean())
  if frac_below > 0.99:
    print(f"  -> the alpha floor is the BINDING constraint: {100 * frac_below:.1f}% of base-pool mass sits " +
          f"below {ALPHA_FLOOR:g} rad/s^2 (need p95 >> {TARGET_ALPHA_P95:g}; have {pct(a, 95):.2f}).")


def print_speed_map(seg: dict) -> None:
  m = masks(seg)
  v = seg["vego"]
  print(f"  {'speed band':<14} {'base':>8} {'qualifying':>11} {'qual %':>8} {'|a| p95':>9}")
  for lo, hi in SPEED_BINS:
    band = (v >= lo) & (v < hi)
    base_n = int((m["base"] & band).sum())
    qual_n = int((m["qual"] & band).sum())
    ap95 = pct(np.abs(seg["alpha"][m["base"] & band]), 95)
    qpct = f"{100.0 * qual_n / base_n:.2f}%" if base_n else "-"
    print(f"  {f'{lo}-{hi} m/s':<14} {base_n:>8} {qual_n:>11} {qpct:>8} {ap95:>9.2f}")


def render_verdict(pooled: dict, rc: float) -> bool:
  """Returns True iff J is trustworthy from this data."""
  rep = fit_report(pooled, rc, "POOLED")
  qmask = id_mask(pooled)
  ap95 = pct(np.abs(pooled["alpha"][qmask]), 95)

  if rep["status"] != "OK":
    print(f"  pooled fit: INSUFFICIENT (n_id={rep['n_id']} < 200 gate, coverage {rep['coverage_pct']:.2f}%).")
    print("  Verdict: J is NOT identifiable from this data.\n")
    print(PHYSICS)
    print(RECOMMENDATION)
    return False

  J, r2 = rep["J"], rep["r2"]
  ci_rel = (rep["J_hi"] - rep["J_lo"]) / (2 * abs(J)) if J else float("inf")
  checks = {
    f"n_id >= {TARGET_N}": rep["n_id"] >= TARGET_N,
    f"alpha p95 >= {TARGET_ALPHA_P95:g}": ap95 >= TARGET_ALPHA_P95,
    f"R^2 >= {TARGET_R2:g}": r2 >= TARGET_R2,
    f"CI/|J| < {TARGET_CI_REL:g}": ci_rel < TARGET_CI_REL,
  }
  trustworthy = all(checks.values())
  print(f"  pooled fit: J={J:.4f} kg*m^2  CI[{rep['J_lo']:.4f},{rep['J_hi']:.4f}] (rel={ci_rel:.2f})  " +
        f"R^2={r2:.3f}  n_id={rep['n_id']} ({rep['coverage_pct']:.2f}% cov)  qual |a| p95={ap95:.2f}")
  print("  trustworthy-J gates:  " + "  ".join(f"{name}:{'PASS' if ok else 'FAIL'}" for name, ok in checks.items()))
  if trustworthy:
    print(f"\n  Verdict: J IS trustworthy -> STEER_INERTIA_J = {J:.3f} kg*m^2.")
    return True
  print("\n  Verdict: a fit cleared the 200-sample floor but FAILS the trust gates above " +
        "(J near-zero / weak R^2 / wide CI). J is NOT trustworthy.\n")
  print(PHYSICS)
  print(RECOMMENDATION)
  return False


def main() -> int:
  ap = argparse.ArgumentParser(description="Diagnose whether STEER_INERTIA_J is identifiable from VTB drive logs")
  ap.add_argument("--routes", nargs="*", default=None, help="route names (default: all local drives in both roots)")
  ap.add_argument("--rc", type=float, default=DEFAULT_RC, help="alpha LPF RC (default %(default)s)")
  args = ap.parse_args()

  routes = args.routes or default_routes()
  banner(f"VTB inertia-J identifiability check  |  deadzone={DEADZONE_NM}Nm  alpha_floor={ALPHA_FLOOR}rad/s^2  rc={args.rc}s")
  print(f"routes ({len(routes)}): {routes}")

  segs, used = [], []
  print("\nper-route ID-condition funnel:")
  print(f"  {'route':<26} {'total':>8} {'latAct':>8} {'handsOff':>9} {'inDz':>9} {'|a|>5':>7}")
  for r in routes:
    try:
      seg = resample_and_alpha(load_route(r), args.rc)
    except SystemExit as e:
      print(f"  {r:<26} SKIP ({e})")
      continue
    if not len(seg["tau"]):
      print(f"  {r:<26} (no usable resampled samples)")
      continue
    m = masks(seg)
    print(f"  {r:<26} {len(seg['tau']):>8} {int(m['lat'].sum()):>8} {int(m['handsoff'].sum()):>9} " +
          f"{int(m['base'].sum()):>9} {int(m['qual'].sum()):>7}")
    segs.append(seg)
    used.append(r)

  if not segs:
    print("\nNo routes with usable carState data.")
    return 1

  # Use fit_steer_inertia's pooler: it shifts run_bounds offsets and keeps run_bounds a list
  # (a naive np.concatenate over every key turns run_bounds into an array -> spectral fit crashes).
  pooled = pool_segs(segs)

  banner(f"POOLED  ({len(used)} routes, {len(pooled['tau'])} resampled samples)")
  print("ID-condition funnel:")
  print_funnel(pooled)
  print("\nexcitation (|alpha|) distribution:")
  print_alpha_dist(pooled)
  print("\nspeed x |alpha| map:")
  print_speed_map(pooled)
  print("\nNote: this uses the fit's recomputed alpha; analyze_shadow.py reports the on-device logged " +
        "alphaFilt (p95/max) as an independent cross-check -- it shows the same floor.\n")

  banner("VERDICT")
  trustworthy = render_verdict(pooled, args.rc)
  return 0 if trustworthy else 2


if __name__ == "__main__":
  sys.exit(main())
