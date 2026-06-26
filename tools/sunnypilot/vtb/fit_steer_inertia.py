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
ALPHA_FLOOR = 5.0                  # rad/s^2 — min |alpha| excitation for a usable LEGACY ID sample
# Local rlog roots, mirroring live_watch.py: recent pulls live under realdata, legacy shadow
# drives under ~/vtb-routes. Search both so a pooled fit can span the whole campaign.
LOCAL_LOG_ROOTS = ("~/.comma/media/0/realdata", "~/vtb-routes")

# --- physics-honest estimator (v2) constants ---
# The legacy fit regresses tau on a CAUSAL, finite-differenced alpha over a tiny |alpha|>5 hands-off
# set. That is biased toward zero (errors-in-variables: alpha is the noisiest regressor) and starved
# of data (OP's rate limiter caps hands-off alpha well below 5). v2 fixes both: a zero-phase clean
# alpha (Savitzky-Golay), a Coulomb-friction term, a frequency-domain cross-spectral impedance
# (EIV-robust), and Bayesian shrinkage to a literature prior so every drive returns a defensible J.
SG_WINDOW = 11                     # Savitzky-Golay window (odd) for the clean acausal alpha
SG_POLY = 3                        # SG polynomial order
COULOMB_OMEGA0 = 0.02              # rad/s — tanh knee for the smooth Coulomb friction term
NPERSEG = 256                      # Welch segment length (~5.1 s at 50 Hz) for the impedance estimate
COH_MIN = 0.5                      # coherence gate for the spectral impedance band
SPEC_F_LO, SPEC_F_HI = 0.5, 10.0   # Hz band for the Re(Z) = k - J*w^2 impedance fit
J_PRIOR_MU, J_PRIOR_SD = 0.10, 0.025   # literature prior N(mu, sd^2): +-2 sd ~ [0.05, 0.15]


def default_routes() -> list[str]:
  names = set()
  for root in LOCAL_LOG_ROOTS:
    for d in glob.glob(os.path.join(os.path.expanduser(root), "*--*--*")):
      base = os.path.basename(d)
      names.add(base.rsplit("--", 1)[0])
  return sorted(names)


def rlog_paths(route: str) -> list[str]:
  segs: list[str] = []
  for root in LOCAL_LOG_ROOTS:
    segs += glob.glob(os.path.join(os.path.expanduser(root), f"{route}--*", "rlog.zst"))
  # sort by numeric segment index
  return sorted(segs, key=lambda p: int(os.path.basename(os.path.dirname(p)).rsplit("--", 1)[1]))


def load_route(route: str) -> dict[str, np.ndarray]:
  """Extract time-aligned carState + latActive at carState's native 100 Hz."""
  paths = rlog_paths(route)
  if not paths:
    raise SystemExit(f"no rlog.zst found locally for route {route} (expected under {' or '.join(LOCAL_LOG_ROOTS)})")
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
  return [(s, e) for s, e in zip(starts, ends, strict=True) if e - s >= 3]


def resample_and_alpha(d: dict, rc: float) -> dict[str, np.ndarray]:
  """Resample onto a uniform DT_LAT_CTRL grid per contiguous run. Computes BOTH:
   - `alpha`       : the FF's CAUSAL alpha (diff(rate)/dt -> LPF -> radians), bit-identical to
                     coop_steering.py so the legacy fit transfers 1:1 to the live FF; and
   - `alpha_clean` : a ZERO-PHASE alpha (Savitzky-Golay 2nd derivative of the angle), much less
                     noisy, used by the v2 estimator to dodge errors-in-variables attenuation.
  Also returns `run_bounds` (index ranges into the concatenated arrays for each contiguous time-run)
  so the spectral impedance estimator can window over genuinely continuous stretches."""
  cols = {k: [] for k in ("tau", "rate", "angle", "vego", "alpha", "alpha_clean", "pressed", "lat")}
  run_lens = []
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

    cols["tau"].append(g_tau)
    cols["rate"].append(g_rate)
    cols["angle"].append(g_angle)
    cols["vego"].append(g_vego)
    cols["alpha"].append(alpha)
    cols["alpha_clean"].append(clean_alpha(g_angle, DT_LAT_CTRL))
    cols["pressed"].append(g_pressed)
    cols["lat"].append(g_lat)
    run_lens.append(len(g_rate))
  if not cols["tau"]:
    # keep the boolean signals bool-typed: a float64 np.array([]) breaks `~pressed` in id_mask_handsoff
    # and, when concatenated with a valid route's bool arrays, silently coerces them to float too.
    out = {k: np.array([], dtype=bool if k in ("pressed", "lat") else float) for k in cols}
    out["run_bounds"] = []
    return out
  out = {k: np.concatenate(v) for k, v in cols.items()}
  bounds, off = [], 0
  for length in run_lens:
    bounds.append((off, off + length))
    off += length
  out["run_bounds"] = bounds
  return out


def pool_segs(segs: list[dict]) -> dict:
  """Concatenate per-route resampled segs into one pooled seg, shifting run_bounds offsets so the
  pooled run boundaries stay correct (run_bounds is a list, not an array, so it can't be concatenated
  blindly)."""
  arr_keys = [k for k in segs[0] if k != "run_bounds"]
  pooled = {k: np.concatenate([s[k] for s in segs]) for k in arr_keys}
  bounds, off = [], 0
  for s in segs:
    for a, b in s.get("run_bounds", []):
      bounds.append((a + off, b + off))
    off += len(s["tau"])
  pooled["run_bounds"] = bounds
  return pooled


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


# ------------------------- v2 physics-honest estimator -------------------------
# The legacy fit (design + robust_fit + id_mask above) regresses tau on a CAUSAL, finite-differenced
# alpha over a tiny |alpha|>5 hands-off set. It is biased toward zero (errors-in-variables: alpha is
# the noisiest regressor) and data-starved (OP's rate limiter keeps hands-off alpha far below 5). The
# v2 estimator below fixes both. The legacy functions are kept for the adversarial harness, which
# compares the two head to head.

def savgol_coeffs(window: int, poly: int, deriv: int, delta: float) -> np.ndarray:
  """Savitzky-Golay FIR coefficients for the `deriv`-th derivative at the window center. The
  deriv-th derivative of the local least-squares polynomial is deriv! * b[deriv] where
  b = (A^T A)^-1 A^T y, A[i,j] = z_i^j, z_i = i-center; scaled by 1/delta^deriv for physical units."""
  m = (window - 1) // 2
  z = np.arange(-m, m + 1, dtype=float)
  A = np.vander(z, poly + 1, increasing=True)
  pinv_row = (np.linalg.pinv(A.T @ A) @ A.T)[deriv]
  return math.factorial(deriv) * pinv_row / (delta ** deriv)


def clean_alpha(angle_deg: np.ndarray, dt: float) -> np.ndarray:
  """Zero-phase wheel angular acceleration (rad/s^2) via a Savitzky-Golay 2nd derivative of the
  steering angle. Far less noisy than the FF's causal diff->LPF, so its regression coefficient is
  not biased toward zero by regressor noise (the errors-in-variables fix)."""
  angle_deg = np.asarray(angle_deg, dtype=float)
  n = len(angle_deg)
  if n < SG_WINDOW:
    return np.zeros(n)
  c = savgol_coeffs(SG_WINDOW, SG_POLY, 2, dt)        # deg/s^2 per deg sample (symmetric for deriv=2)
  acc = np.convolve(angle_deg, c[::-1], mode="same")  # deg/s^2
  m = (SG_WINDOW - 1) // 2
  acc[:m] = acc[m]            # 'same' zero-pads the edges -> clamp the invalid ends to nearest interior
  acc[-m:] = acc[-m - 1]
  return np.radians(acc)


def id_mask_handsoff(seg: dict) -> np.ndarray:
  """v2 ID set: ALL hands-off, openpilot-steered, in-deadzone samples (no |alpha|>5 floor). Low
  excitation is handled honestly via coherence + prior shrinkage, not by discarding the data."""
  return seg["lat"] & ~seg["pressed"] & (np.abs(seg["tau"]) < DEADZONE_NM)


def handsoff_runs(seg: dict, mask: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
  """Maximal contiguous (in time -- within one run_bounds range) hands-off stretches, returned as
  (theta_rad, tau_Nm) pairs for the spectral estimator."""
  out = []
  bounds = seg.get("run_bounds") or [(0, len(seg["tau"]))]
  theta = np.radians(seg["angle"])
  tau = seg["tau"]
  for s, e in bounds:
    mrun = mask[s:e]
    if not mrun.any():
      continue
    idx = np.where(mrun)[0]
    for chunk in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
      if len(chunk) >= 16:
        out.append((theta[s + chunk], tau[s + chunk]))
  return out


def design_v2(seg: dict, mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """v2 time-domain design (no intercept column -- TLS centers): regressors are the clean acausal
  alpha, a smooth Coulomb-friction term tanh(omega/omega0), viscous damping omega, and stiffness
  theta. Coulomb is the dominant low-alpha torque the legacy [alpha,omega,theta,1] model could not
  represent, so without it dry friction leaks into J."""
  a = seg["alpha_clean"][mask]
  omega = np.radians(seg["rate"][mask])
  theta = np.radians(seg["angle"][mask])
  X = np.column_stack([a, np.tanh(omega / COULOMB_OMEGA0), omega, theta])
  return X, seg["tau"][mask]


def tls_fit(X: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float, float]:
  """Total Least Squares (errors-in-variables) with intercept, via SVD on the column-standardized,
  mean-centered [X|y]. TLS treats regressor noise symmetrically, so the leading (alpha) coefficient
  is not biased toward zero the way OLS would. Returns (coef, intercept, var_of_coef0), coef[0]=J."""
  n, p = X.shape
  xm = X.mean(axis=0)
  ym = float(y.mean())
  sx = X.std(axis=0)
  sx[sx == 0] = 1.0
  sy = float(y.std()) or 1.0
  Z = np.column_stack([(X - xm) / sx, (y - ym) / sy])
  try:
    _, _, Vt = np.linalg.svd(Z, full_matrices=False)
  except np.linalg.LinAlgError:
    return np.full(p, np.nan), float("nan"), float("nan")
  v = Vt[-1]
  if abs(v[-1]) < 1e-12:
    return np.full(p, np.nan), float("nan"), float("nan")
  coef = (-v[:-1] / v[-1]) * sy / sx
  intercept = ym - xm @ coef
  # serviceable variance proxy for coef[0]: OLS covariance on the (clean) design
  resid = y - (X @ coef + intercept)
  dof = max(n - p - 1, 1)
  sigma2 = float(resid @ resid) / dof
  try:
    Xi = np.column_stack([X, np.ones(n)])
    var0 = float((sigma2 * np.linalg.pinv(Xi.T @ Xi))[0, 0])
  except np.linalg.LinAlgError:
    var0 = float("nan")
  return coef, float(intercept), var0


def spectral_inertia(seg: dict, mask: np.ndarray, fs: float) -> dict:
  """Cross-spectral mechanical-impedance estimate of J (EIV-robust). With theta as input the
  angle-referenced impedance is Z(w)=tau/theta = (k - J*w^2) + j*b*w, so Re(Z)=k - J*w^2. Estimate
  Z from Welch cross-spectra (which average out *uncorrelated* theta/tau measurement noise) via the
  H1 estimator Z=S_xy/S_xx, then WLS-fit Re(Z) ~ k - J*w^2 over the high-coherence band, weighting
  by coherence*power. If hands-off high-frequency excitation is too weak (the gentle-drive case) the
  band collapses and J is returned None -- the estimator declines rather than fabricating a number."""
  runs = handsoff_runs(seg, mask)
  nperseg = NPERSEG
  Sxx = np.zeros(nperseg // 2 + 1)
  Syy = np.zeros_like(Sxx)
  Sxy = np.zeros_like(Sxx, dtype=complex)
  win = np.hanning(nperseg)
  nseg = 0
  for theta, tau in runs:
    if len(theta) < nperseg:
      continue
    for s in range(0, len(theta) - nperseg + 1, nperseg // 2):
      xs = (theta[s:s + nperseg] - theta[s:s + nperseg].mean()) * win
      ys = (tau[s:s + nperseg] - tau[s:s + nperseg].mean()) * win
      X = np.fft.rfft(xs)
      Y = np.fft.rfft(ys)
      Sxx += (X.conj() * X).real
      Syy += (Y.conj() * Y).real
      Sxy += X.conj() * Y
      nseg += 1
  out = {"n_seg": nseg, "n_bins": 0, "J": None, "var": float("nan"), "coh_med": float("nan"),
         "band": (SPEC_F_LO, SPEC_F_HI)}
  if nseg < 4:
    return out
  Sxx /= nseg
  Syy /= nseg
  Sxy /= nseg
  f = np.fft.rfftfreq(nperseg, 1.0 / fs)
  w = 2 * np.pi * f
  with np.errstate(divide="ignore", invalid="ignore"):
    coh = (np.abs(Sxy) ** 2) / (Sxx * Syy)
    Z = Sxy / Sxx
  inband = (f >= SPEC_F_LO) & (f <= SPEC_F_HI)
  out["coh_med"] = float(np.nanmedian(coh[inband])) if inband.any() else float("nan")
  band = inband & (coh > COH_MIN) & np.isfinite(Z.real)
  out["n_bins"] = int(band.sum())
  if out["n_bins"] < 3:
    return out
  A = np.column_stack([np.ones(out["n_bins"]), w[band] ** 2])
  yv = Z.real[band]
  wt = coh[band] * Sxx[band]
  if wt.sum() <= 0:
    return out
  try:
    ATA = A.T @ (wt[:, None] * A)
    b = np.linalg.solve(ATA, A.T @ (wt * yv))
    cov = np.linalg.inv(ATA)
  except np.linalg.LinAlgError:
    return out
  resid = yv - A @ b
  sigma2 = float((wt * resid ** 2).sum() / max(wt.sum(), 1e-12)) * out["n_bins"] / max(out["n_bins"] - 2, 1)
  out["J"] = float(-b[1])
  out["var"] = float(abs(cov[1, 1]) * sigma2)
  out["coef_k"] = float(b[0])
  return out


def bayesian_shrink(estimates: list[tuple[float, float]]) -> dict:
  """Gaussian-conjugate posterior for J: prior N(J_PRIOR_MU, J_PRIOR_SD^2) combined with the data
  estimates (J_i, var_i) by inverse-variance. Uninformative drives -> posterior ~ prior (defensible
  default + honest wide CI); real excitation -> posterior pulled to the data. observability =
  data precision / prior precision (>1 => the data outweighed the prior)."""
  prec0 = 1.0 / J_PRIOR_SD ** 2
  prec, num, data_prec = prec0, J_PRIOR_MU * prec0, 0.0
  for J, var in estimates:
    if J is None or not np.isfinite(J) or var is None or not np.isfinite(var) or var <= 0:
      continue
    prec += 1.0 / var
    num += J / var
    data_prec += 1.0 / var
  observability = data_prec / prec0
  return {"J": num / prec, "sd": 1.0 / math.sqrt(prec), "observability": observability,
          "label": "DATA-INFORMED" if observability > 1.0 else "PRIOR-DOMINATED"}


def estimate_J(seg: dict, fs: float) -> dict:
  """Layered v2 estimator. Primary: cross-spectral impedance (EIV-robust). Cross-check: TLS +
  Coulomb on the clean acausal alpha. Synthesis: Bayesian shrinkage to the literature prior."""
  mask = id_mask_handsoff(seg)
  n = int(mask.sum())
  res = {"n_id": n, "spec": spectral_inertia(seg, mask, fs), "tls": None}
  estimates = []
  # Gate the spectral J the same way as TLS: a spurious coherent band can slope the wrong way and
  # return a negative / out-of-band J with a small variance, which would otherwise dominate the
  # inverse-variance posterior and falsely read DATA-INFORMED.
  spec = res["spec"]
  spec["used"] = bool(spec["J"] is not None and np.isfinite(spec["J"]) and 0.0 < spec["J"] < 0.30
                      and np.isfinite(spec["var"]) and spec["var"] > 0)
  if spec["used"]:
    estimates.append((spec["J"], spec["var"]))
  if n >= 50:
    X, y = design_v2(seg, mask)
    coef, intercept, var0 = tls_fit(X, y)
    a = seg["alpha_clean"][mask]
    resid = y - (X @ coef + intercept)
    tls_r2 = r2(y, resid)
    raw_rms = float(np.sqrt(np.mean(y ** 2))) or 1.0
    fwd = 1.0 - float(np.sqrt(np.mean((y - coef[0] * a) ** 2))) / raw_rms  # forward-check reduction
    # The TLS time-domain fit is only TRUSTWORTHY when it (1) lands a physically-plausible J, (2)
    # actually explains hands-off torque variance (R^2), and (3) subtracting J*alpha shrinks the
    # hands-off torque RMS (forward-check). On near-zero-alpha-SNR data the fit chases noise: it can
    # still produce a small-CI number (large n), so range alone is not enough -- the R^2/forward-check
    # gates reject it. Without these gates the pooled noise fit falsely reads DATA-INFORMED.
    plausible = np.isfinite(coef[0]) and 0.0 < coef[0] < 0.30 and np.isfinite(var0) and var0 > 0
    trustworthy = bool(plausible and tls_r2 > 0.10 and fwd > 0.0)
    res["tls"] = {"J": float(coef[0]), "coulomb": float(coef[1]), "damping": float(coef[2]),
                  "stiffness": float(coef[3]), "offset": intercept, "var": var0,
                  "r2": float(tls_r2), "fwd": float(fwd), "used": trustworthy}
    if trustworthy:
      estimates.append((coef[0], var0))
  res.update(bayesian_shrink(estimates))
  return res


def fit_report(seg: dict, rc: float, route_label: str) -> dict:
  fs = 1.0 / DT_LAT_CTRL
  est = estimate_J(seg, fs)
  n = est["n_id"]
  total = len(seg["tau"])
  cov = 100.0 * n / total if total else 0.0
  J, sd = est["J"], est["sd"]
  out = {"route": route_label, "rc": rc, "n_id": n, "n_total": total, "coverage_pct": cov,
         "J": J, "sd": sd, "J_lo": J - 2 * sd, "J_hi": J + 2 * sd,
         "observability": est["observability"], "label": est["label"],
         "spec": est["spec"], "tls": est["tls"],
         # defaults so downstream key access never KeyErrors (overwritten below when the TLS fit runs)
         "r2": float("nan"), "resid_rms": float("nan"),
         "damping": float("nan"), "stiffness": float("nan"), "offset": float("nan"), "coulomb": float("nan"),
         "tau_raw_rms": float("nan"), "tau_intent_rms": float("nan"),
         # status OK only when the data actually moved the prior (DATA-INFORMED); else INSUFFICIENT
         # (still carries a defensible prior-shrunk J -- it just shouldn't be trusted as a measurement)
         "status": "OK" if est["label"] == "DATA-INFORMED" else "INSUFFICIENT"}
  # hands-off diagnostics from the TLS+Coulomb fit (nuisance terms, R^2, forward-check)
  mask = id_mask_handsoff(seg)
  if n >= 50 and est["tls"] is not None and np.isfinite(est["tls"]["J"]):
    X, y = design_v2(seg, mask)
    tls = est["tls"]
    pred = X @ np.array([tls["J"], tls["coulomb"], tls["damping"], tls["stiffness"]]) + tls["offset"]
    resid = y - pred
    a = seg["alpha_clean"][mask]
    out.update(damping=tls["damping"], stiffness=tls["stiffness"], offset=tls["offset"],
               coulomb=tls["coulomb"], r2=r2(y, resid), resid_rms=float(np.sqrt(np.mean(resid ** 2))),
               tau_raw_rms=float(np.sqrt(np.mean(y ** 2))),
               tau_intent_rms=float(np.sqrt(np.mean((y - J * a) ** 2))))
  return out


def banner(s):
  print("\n" + "=" * 78 + f"\n{s}\n" + "=" * 78)


def _print_v2_detail(r: dict):
  """Spectral + TLS component lines + nuisance/forward-check (shared by OK and INSUFFICIENT)."""
  sp = r.get("spec") or {}
  if sp.get("J") is not None:
    excl = "" if sp.get("used", True) else "  (implausible -> excluded from posterior)"
    print(f"        spectral J = {sp['J']:.4f} kg*m^2  (coherent bins={sp['n_bins']}, " +
          f"band {sp['band'][0]:.1f}-{sp['band'][1]:.1f}Hz, med coh={sp['coh_med']:.2f}, segs={sp['n_seg']}){excl}")
  else:
    cm = sp.get("coh_med", float("nan"))
    print(f"        spectral J = n/a  (no coherent high-freq band: bins={sp.get('n_bins', 0)}, " +
          f"med coh={cm:.2f}, segs={sp.get('n_seg', 0)} -> too little hands-off excitation)")
  tls = r.get("tls")
  if tls is not None and np.isfinite(tls["J"]):
    flag = "" if tls.get("used") else (f"  (R^2={tls.get('r2', float('nan')):.2f}, " +
                                       f"fwd={100*tls.get('fwd', float('nan')):.0f}% -> excluded from posterior)")
    print(f"        TLS+Coulomb J = {tls['J']:.4f} kg*m^2   Coulomb Fc={tls['coulomb']:.4f} Nm  " +
          f"damping={tls['damping']:.4f}  stiffness={tls['stiffness']:.4f}{flag}")
  if "tau_raw_rms" in r and r["tau_raw_rms"] > 0:
    red = 100 * (1 - r["tau_intent_rms"] / r["tau_raw_rms"])
    print(f"        forward-check: hands-off tau RMS {r['tau_raw_rms']:.3f} -> intent RMS " +
          f"{r['tau_intent_rms']:.3f} Nm  ({red:.0f}% reduction)")


def print_fit(r: dict):
  if r["status"] == "INSUFFICIENT":
    # legacy line parsed by the report workflows -- now means PRIOR-DOMINATED (data didn't move J)
    print(f"  [{r['route']}] rc={r['rc']:.3f}  coverage {r['coverage_pct']:.2f}% " +
          f"({r['n_id']}/{r['n_total']})  -> INSUFFICIENT excitation for a fit")
    print(f"        posterior J = {r['J']:.4f} +-{2*r['sd']:.4f} kg*m^2  " +
          f"({r['label']}, observability={r['observability']:.2f})")
    _print_v2_detail(r)
    return
  # legacy POOLED/per-route line (parsed by the report workflows); J/CI are the posterior
  print(f"  [{r['route']}] rc={r['rc']:.3f}  J = {r['J']:.4f} kg*m^2 " +
        f"[{r['J_lo']:.4f}, {r['J_hi']:.4f}]  R^2={r.get('r2', float('nan')):.3f}  " +
        f"resid_rms={r.get('resid_rms', float('nan')):.3f} Nm  n={r['n_id']} ({r['coverage_pct']:.1f}% cov)")
  print(f"        verdict: {r['label']}  observability={r['observability']:.2f}")
  if np.isfinite(r.get("damping", float("nan"))):
    print(f"        nuisance: damping={r['damping']:.4f}  stiffness={r['stiffness']:.4f}  offset={r['offset']:.3f}")
  _print_v2_detail(r)


def main():
  ap = argparse.ArgumentParser(description="Derive STEER_INERTIA_J from baseline VTB drive logs")
  ap.add_argument("--routes", nargs="*", default=None, help="route names (default: all local drives)")
  ap.add_argument("--rc", nargs="*", type=float, default=[DEFAULT_RC], help="alpha LPF RC value(s) to try")
  args = ap.parse_args()

  routes = args.routes or default_routes()
  if not routes:
    raise SystemExit(f"no local drives found under {' or '.join(LOCAL_LOG_ROOTS)} (pull some first, or pass --routes)")
  banner(f"VTB inertia-J fit  |  DT_LAT_CTRL={DT_LAT_CTRL*1000:.1f}ms (STEER_STEP={CarControllerParams.STEER_STEP})  " +
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
    pooled = pool_segs(segs)
    prep = fit_report(pooled, rc, "POOLED")
    print_fit(prep)
    mask = id_mask_handsoff(pooled)
    if int(mask.sum()) >= 100:
      X, y = design_v2(pooled, mask)
      half = len(y) // 2
      c1, _, _ = tls_fit(X[:half], y[:half])
      c2, _, _ = tls_fit(X[half:], y[half:])
      print(f"        split-half J (TLS): {c1[0]:.4f} | {c2[0]:.4f}  (consistency check)")
    if best is None:
      best = prep

  banner("RECOMMENDATION")
  J = best["J"]
  # The "Fitted J (pooled, robust) = …" and "Recommended STEER_INERTIA_J = …" lines are printed ONLY
  # when DATA-INFORMED -- the vtb-report workflows key status=OK off the presence of those exact lines.
  # A PRIOR-DOMINATED result still surfaces the defensible prior J, but under different wording + exit 2.
  if best["status"] == "OK":
    J_clamped = float(np.clip(J, J_LIT_LO, J_LIT_HI))
    in_band = J_LIT_LO <= J <= J_LIT_HI
    print(f"  Fitted J (pooled, robust) = {J:.4f} kg*m^2  CI[{best['J_lo']:.4f}, {best['J_hi']:.4f}]")
    print(f"  Estimator verdict: DATA-INFORMED  (observability={best['observability']:.2f})")
    print(f"  Literature band [{J_LIT_LO}, {J_LIT_HI}] -> {'IN BAND' if in_band else 'OUT OF BAND (clamped)'}")
    print(f"  Recommended STEER_INERTIA_J = {round(J_clamped, 3)}  (current placeholder 0.08)")
  else:
    print(f"  Estimator verdict: PRIOR-DOMINATED  (observability={best['observability']:.2f}; the data did " +
          "not move J off the prior).")
    print(f"  Prior-default J = {J:.4f} kg*m^2  [{best['J_lo']:.4f}, {best['J_hi']:.4f}] -- the DEFENSIBLE " +
          "DEFAULT, NOT a per-vehicle measurement.")
    print("  The drives lacked the hands-off wheel-acceleration excitation to identify J. For a data-driven")
    print("  J: a targeted excitation drive (brisk hands-off slaloms / lane wanders) or the gentle active-")
    print("  dither calibration, then re-run /vtb-tune.")
    sys.exit(2)


if __name__ == "__main__":
  main()
