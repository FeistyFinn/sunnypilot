"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Synthetic-signal regression tests for the v2 physics-honest inertia-J estimator. These prove the
estimator actually recovers a known J from data with realistic structure (clean acausal alpha, a
Coulomb-friction term, errors-in-variables via TLS, and the cross-spectral impedance), and that it
honestly declines (PRIOR-DOMINATED) when the excitation is too weak.
"""
import math
import numpy as np

from openpilot.tools.sunnypilot.vtb.fit_steer_inertia import (
  savgol_coeffs, clean_alpha, tls_fit, spectral_inertia, bayesian_shrink, estimate_J,
  id_mask_handsoff, COULOMB_OMEGA0, DT_LAT_CTRL, J_PRIOR_MU,
)

FS = 1.0 / DT_LAT_CTRL


def _multisine(t, comps):
  """Sum of sinusoids; returns (theta, omega, alpha) in consistent units (theta's unit, /s, /s^2)."""
  theta = np.zeros_like(t)
  omega = np.zeros_like(t)
  alpha = np.zeros_like(t)
  for a, f, ph in comps:
    w = 2 * math.pi * f
    theta += a * np.sin(w * t + ph)
    omega += a * w * np.cos(w * t + ph)
    alpha += -a * w * w * np.sin(w * t + ph)
  return theta, omega, alpha


def _make_seg(comps, J, b, k, n=4000, tau_noise=0.0, seed=0, target_tau=0.3):
  """Build a hands-off seg with tau = J*alpha + b*omega + k*theta, scaled so |tau| stays inside the
  deadzone (as real torsion-bar reaction torque does). alpha_clean is set to the analytic alpha to
  isolate the regression from the SG differentiator (which is tested separately)."""
  rng = np.random.default_rng(seed)
  t = np.arange(n) * DT_LAT_CTRL
  theta, omega, alpha = _multisine(t, comps)
  tau = J * alpha + b * omega + k * theta
  scale = target_tau / (np.max(np.abs(tau)) or 1.0)
  theta, omega, alpha, tau = theta * scale, omega * scale, alpha * scale, tau * scale
  tau = tau + tau_noise * rng.standard_normal(n)
  return {"tau": tau, "rate": np.degrees(omega), "angle": np.degrees(theta),
          "alpha": alpha, "alpha_clean": alpha, "vego": np.full(n, 15.0),
          "pressed": np.zeros(n, bool), "lat": np.ones(n, bool), "run_bounds": [(0, n)]}


def test_savgol_2nd_deriv_of_quadratic_is_exact():
  # 2nd derivative of 0.5*a*t^2 is the constant a; SG (poly>=2) must recover it in the interior.
  dt = DT_LAT_CTRL
  t = np.arange(200) * dt
  a = 3.0
  y = 0.5 * a * t ** 2
  c = savgol_coeffs(11, 3, 2, dt)
  acc = np.convolve(y, c[::-1], mode="same")
  assert np.allclose(acc[10:-10], a, rtol=1e-6, atol=1e-6)


def test_clean_alpha_matches_analytic_sine():
  # clean_alpha of a 1 Hz sine angle matches the analytic -A*w^2*sin in rad/s^2 (interior).
  dt = DT_LAT_CTRL
  t = np.arange(1000) * dt
  amp_rad, f = 0.05, 1.0
  w = 2 * math.pi * f
  angle_deg = np.degrees(amp_rad * np.sin(w * t))
  truth = -amp_rad * w * w * np.sin(w * t)        # rad/s^2
  est = clean_alpha(angle_deg, dt)
  m = 20
  assert np.allclose(est[m:-m], truth[m:-m], rtol=0.05, atol=0.05)


def test_tls_recovers_j_with_coulomb():
  # Build tau = J*alpha + Fc*tanh(omega/w0) + b*omega + k*theta + noise; TLS+Coulomb must recover J
  # (and Fc), where ordinary OLS would attenuate J toward zero from the noisy alpha regressor.
  rng = np.random.default_rng(1)
  n = 6000
  t = np.arange(n) * DT_LAT_CTRL
  theta, omega, alpha = _multisine(t, [(0.03, 0.7, 0.1), (0.02, 1.7, 1.0), (0.01, 3.3, 2.0)])
  J, Fc, b, k = 0.10, 0.08, 0.04, 0.3
  tau = J * alpha + Fc * np.tanh(omega / COULOMB_OMEGA0) + b * omega + k * theta
  # add noise to the alpha REGRESSOR (errors-in-variables) and a little to tau
  alpha_noisy = alpha + 0.4 * rng.standard_normal(n)
  tau = tau + 0.01 * rng.standard_normal(n)
  X = np.column_stack([alpha_noisy, np.tanh(omega / COULOMB_OMEGA0), omega, theta])
  coef, intercept, var0 = tls_fit(X, tau)
  assert abs(coef[0] - J) < 0.03            # J recovered despite regressor noise
  assert abs(coef[1] - Fc) < 0.03           # Coulomb term recovered
  assert np.isfinite(var0) and var0 > 0


def test_ols_attenuates_with_noisy_alpha_regressor():
  # Demonstrate the core bias the redesign targets: when the alpha regressor carries measurement
  # noise comparable to its own signal variance (the real low-excitation regime), the OLS coefficient
  # on alpha is biased toward zero (errors-in-variables attenuation). Low frequencies keep alpha's
  # variance modest so noise std ~ signal std and the attenuation is unmistakable. The matching
  # de-attenuated recovery is exercised cleanly by test_tls_recovers_j_with_coulomb.
  rng = np.random.default_rng(2)
  n = 12000
  t = np.arange(n) * DT_LAT_CTRL
  theta, omega, alpha = _multisine(t, [(0.05, 0.3, 0.1), (0.04, 0.6, 0.5), (0.03, 0.9, 1.5)])
  J, b, k = 0.10, 0.04, 0.3
  tau = J * alpha + b * omega + k * theta
  alpha_noisy = alpha + 0.6 * rng.standard_normal(n)   # regressor noise ~ alpha's own std
  X = np.column_stack([alpha_noisy, np.tanh(omega / COULOMB_OMEGA0), omega, theta])
  ols = np.linalg.lstsq(np.column_stack([X, np.ones(n)]), tau, rcond=None)[0]
  assert 0.0 < ols[0] < 0.8 * J    # OLS on the noisy alpha regressor is attenuated toward zero


def test_spectral_recovers_j():
  # Cross-spectral impedance recovers J from a broadband, low-noise hands-off seg.
  comps = [(0.03, 0.8, 0.0), (0.02, 1.5, 0.4), (0.015, 2.7, 1.1),
           (0.01, 4.5, 2.0), (0.008, 6.5, 0.7), (0.006, 8.5, 1.9)]
  seg = _make_seg(comps, J=0.10, b=0.04, k=0.4, n=8000, tau_noise=1e-4, seed=3)
  mask = id_mask_handsoff(seg)
  assert mask.all()                                  # tau stayed inside the deadzone
  spec = spectral_inertia(seg, mask, FS)
  assert spec["J"] is not None and spec["n_bins"] >= 3
  assert abs(spec["J"] - 0.10) < 0.03


def test_bayesian_shrink_prior_and_data():
  # No data -> posterior is the prior. One tight data estimate -> posterior pulled to the data.
  prior = bayesian_shrink([])
  assert abs(prior["J"] - J_PRIOR_MU) < 1e-9 and prior["label"] == "PRIOR-DOMINATED"
  informed = bayesian_shrink([(0.07, 1e-5)])
  assert informed["label"] == "DATA-INFORMED" and abs(informed["J"] - 0.07) < 0.01


def test_estimate_j_excited_is_data_informed():
  comps = [(0.03, 0.8, 0.0), (0.02, 1.6, 0.5), (0.015, 2.9, 1.2),
           (0.01, 4.7, 2.1), (0.008, 6.7, 0.6), (0.006, 8.7, 1.8)]
  seg = _make_seg(comps, J=0.10, b=0.04, k=0.4, n=9000, tau_noise=1e-4, seed=4)
  est = estimate_J(seg, FS)
  assert est["label"] == "DATA-INFORMED"
  assert abs(est["J"] - 0.10) < 0.03


def test_estimate_j_gentle_is_prior_dominated():
  # Gentle: only very-low-frequency, tiny motion + heavy tau noise -> no coherent high-freq band and
  # a noise-dominated TLS -> the estimator must fall back to the prior, not fabricate a J.
  seg = _make_seg([(0.02, 0.1, 0.0)], J=0.10, b=0.04, k=0.4, n=9000, tau_noise=0.15, seed=5)
  est = estimate_J(seg, FS)
  assert est["label"] == "PRIOR-DOMINATED"
  assert abs(est["J"] - J_PRIOR_MU) < 0.04      # stays near the defensible prior default
