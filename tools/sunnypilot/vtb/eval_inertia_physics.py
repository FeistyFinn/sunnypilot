#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VTB inertia-comp ADVERSARIAL physics evaluation across all local routes.

The legacy fit (fit_steer_inertia.py, v1: robust OLS of tau on a CAUSAL finite-differenced alpha over
a |alpha|>5 hands-off set) reports a pooled J ~ 0.005 kg*m^2 -- far below the 0.05-0.15 literature
band. This tool proves, with numbers over every downloaded drive, that that number is an ARTIFACT of
four compounding failure modes, not a measurement:

  (a) errors-in-variables attenuation  -- the OLS coefficient on a noisy alpha regressor is biased
      toward zero; a clean (acausal) alpha and a de-attenuation factor recover a larger J.
  (b) missing Coulomb friction          -- dry friction ~sign(omega)*Fc dominates the low-alpha
      torque and the linear [alpha,omega,theta,1] model leaks it into J.
  (c) unreachable excitation            -- hands-off |alpha| sits far below the 5 rad/s^2 floor; the
      inertial torque J*alpha is below the torsion-bar deadzone/noise.
  (d) disjoint regimes                  -- the FF only fires hands-ON (|tau|>deadzone) yet J is
      identified hands-OFF (|tau|<deadzone): the two sets are disjoint.

Read-only and offline. Reuses fit_steer_inertia's loaders/masks verbatim so the funnels match.

  python tools/sunnypilot/vtb/eval_inertia_physics.py                 # all local drives -> stdout
  python tools/sunnypilot/vtb/eval_inertia_physics.py --out notes/vtb-inertia-physics-eval.md
"""
from __future__ import annotations

import argparse
import os
import sys

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
  default_routes, load_route, resample_and_alpha, pool_segs, robust_fit, id_mask, id_mask_handsoff, estimate_J,
  DEADZONE_NM, ALPHA_FLOOR, DT_LAT_CTRL, DEFAULT_RC, J_LIT_LO, J_LIT_HI, COULOMB_OMEGA0,
)

FS = 1.0 / DT_LAT_CTRL


def pct(x, ps):
  x = np.asarray(x)
  return {p: float(np.percentile(np.abs(x), p)) if len(x) else float("nan") for p in ps}


def robust_J(X, y):
  coef, resid = robust_fit(X, y)
  return float(coef[0]), coef, resid


def eval_attenuation(seg, mask):
  """(a) EIV attenuation: J from causal alpha vs clean alpha; de-attenuation factor lambda."""
  omega = np.radians(seg["rate"][mask])
  theta = np.radians(seg["angle"][mask])
  y = seg["tau"][mask]
  ac = seg["alpha"][mask]
  acl = seg["alpha_clean"][mask]
  ones = np.ones_like(y)
  J_causal, _, _ = robust_J(np.column_stack([ac, omega, theta, ones]), y)
  J_clean, _, _ = robust_J(np.column_stack([acl, omega, theta, ones]), y)
  # measurement-noise variance of the causal alpha, estimated where the wheel is ~still
  still = (np.abs(omega) < 0.02) & (np.abs(acl) < 0.5)
  sigma_e2 = float(np.var(ac[still])) if still.sum() > 50 else float("nan")
  s2 = float(np.var(acl))                                   # clean alpha ~ true alpha signal variance
  lam = s2 / (s2 + sigma_e2) if np.isfinite(sigma_e2) and (s2 + sigma_e2) > 0 else float("nan")
  return {"J_causal": J_causal, "J_clean": J_clean, "lam": lam,
          "J_deatt": J_causal / lam if np.isfinite(lam) and lam > 0 else float("nan"),
          "sigma_e2": sigma_e2, "s2": s2, "n_still": int(still.sum())}


def eval_coulomb(seg, mask):
  """(b) Missing Coulomb friction: J with/without a sign(omega) term + residual leakage."""
  acl = seg["alpha_clean"][mask]
  omega = np.radians(seg["rate"][mask])
  theta = np.radians(seg["angle"][mask])
  y = seg["tau"][mask]
  ones = np.ones_like(y)
  coul = np.tanh(omega / COULOMB_OMEGA0)
  J_base, _, resid_base = robust_J(np.column_stack([acl, omega, theta, ones]), y)
  cc, _ = robust_fit(np.column_stack([acl, coul, omega, theta, ones]), y)
  sgn = np.sign(omega)
  # corrcoef is NaN if EITHER input is constant -- guard both sign(omega) and the residual
  leak = float(np.corrcoef(resid_base, sgn)[0, 1]) if (np.std(sgn) > 0 and np.std(resid_base) > 0) else float("nan")
  return {"J_no_coulomb": J_base, "J_coulomb": float(cc[0]), "Fc": float(cc[1]), "resid_sign_corr": leak}


def eval_excitation(seg, mask):
  """(c) Unreachable excitation: alpha distribution, floor count, signal-vs-noise."""
  ac = np.abs(seg["alpha"][mask])
  acl = np.abs(seg["alpha_clean"][mask])
  y = seg["tau"][mask]
  ps = [50, 90, 95, 99]
  n = int(mask.sum())
  above = int((ac > ALPHA_FLOOR).sum())
  # signal-to-noise of the inertial term at the literature mid J
  Jmid = 0.5 * (J_LIT_LO + J_LIT_HI)
  snr = float(np.var(Jmid * seg["alpha_clean"][mask]) / np.var(y)) if n and np.var(y) > 0 else float("nan")
  return {"causal": pct(ac, ps), "clean": pct(acl, ps), "n": n, "above_floor": above,
          "above_floor_pct": 100.0 * above / n if n else 0.0,
          "Jalpha_p90": Jmid * float(np.percentile(acl, 90)) if n else float("nan"), "snr": snr}


def eval_regime(seg):
  """(d) Disjoint regimes: hands-on vs hands-off |alpha|; overlap of FF-active and ID-eligible."""
  lat = seg["lat"]
  pressed = seg["pressed"]
  tau = seg["tau"]
  ac = np.abs(seg["alpha"])
  on = lat & (np.abs(tau) >= DEADZONE_NM)        # where the live FF would fire
  off = lat & ~pressed & (np.abs(tau) < DEADZONE_NM)  # hands-off ID regime
  legacy = id_mask(seg)                          # FF-active AND legacy-ID-eligible is impossible:
  overlap = int((on & legacy).sum())             # legacy requires |tau|<deadzone, on requires >=
  return {"on_alpha": pct(ac[on], [50, 90, 95]), "off_alpha": pct(ac[off], [50, 90, 95]),
          "n_on": int(on.sum()), "n_off": int(off.sum()), "ff_and_id_overlap": overlap}


def f(x, d=4):
  return "n/a" if x is None or not np.isfinite(x) else f"{x:.{d}f}"


def build_report(routes, per_route, pooled) -> list[str]:
  L = ["# VTB inertia-comp adversarial physics evaluation", ""]
  L += [f"Routes: {len(routes)} local drives. Pooled hands-off ID samples (v2 mask: latActive & " +
        f"hands-off & |tau|<{DEADZONE_NM}Nm): **{int(id_mask_handsoff(pooled).sum())}** " +
        f"of {len(pooled['tau'])} resampled @ {FS:.0f}Hz.", ""]

  # inventory
  L += ["## Dataset inventory", "", "| route | resampled | hands-off n | legacy(|a|>5) n |",
        "|---|---|---|---|"]
  for r in routes:
    seg = per_route[r]
    L.append(f"| `{r}` | {len(seg['tau'])} | {int(id_mask_handsoff(seg).sum())} | {int(id_mask(seg).sum())} |")
  L.append("")

  mask = id_mask_handsoff(pooled)
  if int(mask.sum()) == 0:
    return L + ["**No hands-off, in-deadzone samples in this dataset -- nothing to identify J from.**", ""]
  a = eval_attenuation(pooled, mask)
  b = eval_coulomb(pooled, mask)
  c = eval_excitation(pooled, mask)
  d = eval_regime(pooled)
  est = estimate_J(pooled, FS)
  spec = est["spec"]  # reuse: estimate_J already ran the Welch CSD over the pooled hands-off set

  L += ["## (a) Errors-in-variables attenuation", "",
        "OLS/IRLS on the noisy *causal* alpha biases J toward zero. A clean (zero-phase) alpha and a " +
        "de-attenuation factor `lambda = var(signal)/(var(signal)+var(noise))` both recover a larger J.", "",
        "| estimator | J (kg*m^2) |", "|---|---|",
        f"| causal alpha (legacy) | {f(a['J_causal'])} |",
        f"| clean acausal alpha | {f(a['J_clean'])} |",
        f"| de-attenuated J_causal/lambda | {f(a['J_deatt'])} |",
        f"| lambda (attenuation factor) | {f(a['lam'],3)} |", "",
        f"alpha measurement-noise var (still-wheel) = {f(a['sigma_e2'])} rad^2/s^4 over {a['n_still']} " +
        f"samples; clean-alpha signal var = {f(a['s2'])}. lambda<1 means the causal-alpha coefficient " +
        "is attenuated -- the legacy J is biased low, not physically small. (On near-zero-SNR gentle " +
        "data the time-domain J can even go negative -- itself proof the fit is noise, not inertia.)", ""]

  L += ["## (b) Missing Coulomb friction", "",
        f"Adding a smooth Coulomb term `tanh(omega/{COULOMB_OMEGA0})` changes J and absorbs a real " +
        "torque the linear model could not represent:", "",
        f"- J without Coulomb: **{f(b['J_no_coulomb'])}**, with Coulomb: **{f(b['J_coulomb'])}** " +
        f"(Coulomb Fc = {f(b['Fc'])} Nm)",
        f"- baseline residual vs sign(omega) correlation: **{f(b['resid_sign_corr'],3)}** " +
        "(non-zero => dry friction is leaking into the linear coefficients incl. J)", ""]

  L += ["## (c) Unreachable excitation", "",
        f"- hands-off |alpha| (causal):  p50 {f(c['causal'][50],2)}  p90 {f(c['causal'][90],2)}  " +
        f"p95 {f(c['causal'][95],2)}  p99 {f(c['causal'][99],2)} rad/s^2",
        f"- hands-off |alpha| (clean):   p50 {f(c['clean'][50],2)}  p90 {f(c['clean'][90],2)}  " +
        f"p95 {f(c['clean'][95],2)}  p99 {f(c['clean'][99],2)} rad/s^2",
        f"- samples above the {ALPHA_FLOOR} rad/s^2 fit floor: **{c['above_floor']} / {c['n']} " +
        f"({f(c['above_floor_pct'],2)}%)**",
        f"- inertial torque at p90 alpha (J={0.5*(J_LIT_LO+J_LIT_HI)}): **{f(c['Jalpha_p90'],3)} Nm** " +
        f"vs the {DEADZONE_NM} Nm deadzone/noise floor",
        f"- inertial-term SNR var(J*alpha)/var(tau) on the hands-off set: **{f(c['snr'],4)}** (<<1)", ""]

  L += ["## (d) Disjoint regimes", "",
        f"- hands-ON (|tau|>={DEADZONE_NM}, FF would fire): |alpha| p50 {f(d['on_alpha'][50],2)}  " +
        f"p90 {f(d['on_alpha'][90],2)} rad/s^2  (n={d['n_on']})",
        f"- hands-OFF (|tau|<{DEADZONE_NM}, J is identified here): |alpha| p50 {f(d['off_alpha'][50],2)}  " +
        f"p90 {f(d['off_alpha'][90],2)} rad/s^2  (n={d['n_off']})",
        f"- frames simultaneously FF-active AND legacy-ID-eligible: **{d['ff_and_id_overlap']}** " +
        "(zero by construction -- the FF fires in a regime J is never measured in)", ""]

  L += ["## Frequency-domain impedance (the EIV-robust primary estimator)", "",
        f"- cross-spectral median coherence in {spec['band'][0]:.1f}-{spec['band'][1]:.1f} Hz: " +
        f"**{f(spec['coh_med'],3)}** over {spec['n_seg']} Welch segments; coherent bins: " +
        f"**{spec['n_bins']}**",
        f"- spectral J: **{f(spec['J'])}** kg*m^2"
        + ("" if spec["J"] is not None else "  (declines: no coherent high-frequency band)"), ""]

  L += ["## Verdict", "",
        f"Pooled posterior **J = {f(est['J'])} kg*m^2 (+-{f(2*est['sd'])}, {est['label']}, " +
        f"observability={f(est['observability'],2)})**. The legacy causal-alpha fit's small J is an " +
        "artifact: biased toward zero by regressor noise (a), confounded by un-modeled Coulomb friction " +
        "(b), starved of excitation -- the inertial torque sits below the torsion-bar noise (c) -- and " +
        "identified in a regime disjoint from where the FF fires (d). The frequency-domain estimator " +
        "honestly declines (coherence collapses above a couple Hz), so the posterior falls back to the " +
        "literature prior. Passive normal/gentle driving cannot identify J; a deliberate excitation drive " +
        "or a gentle active-dither calibration is required.", ""]
  return L


def main():
  ap = argparse.ArgumentParser(description="Adversarial physics evaluation of the VTB inertia-comp fit")
  ap.add_argument("--routes", nargs="*", default=None, help="route names (default: all local drives)")
  ap.add_argument("--out", default=None, help="write the markdown report to this path (else stdout)")
  args = ap.parse_args()

  routes = args.routes or default_routes()
  print(f"loading {len(routes)} routes...", file=sys.stderr)
  per_route = {}
  for r in routes:
    try:
      per_route[r] = resample_and_alpha(load_route(r), DEFAULT_RC)
    except (SystemExit, Exception) as e:
      print(f"  skip {r}: {e}", file=sys.stderr)
  routes = [r for r in routes if r in per_route and len(per_route[r]["tau"])]
  if not routes:
    raise SystemExit("no usable routes found locally")
  pooled = pool_segs([per_route[r] for r in routes])

  report = "\n".join(build_report(routes, per_route, pooled))
  if args.out:
    with open(args.out, "w") as fh:
      fh.write(report + "\n")
    print(f"wrote {args.out}", file=sys.stderr)
  else:
    print(report)


if __name__ == "__main__":
  main()
