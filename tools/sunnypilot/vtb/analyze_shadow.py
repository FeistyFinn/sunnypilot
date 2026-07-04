#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VTB inertia-comp SHADOW-MODE analysis.

Reads the on-device cooperative-steering FF telemetry logged to CarStateSP.coopSteering
(populated by opendbc carstate_ext.update_coop_steering_sp) and reports, from real drives,
what the inertia feedforward (FF) was doing while it ran in shadow (computed + logged but
NOT applied to steering):

  - coverage: how much of the drive had coop steering active / the FF in shadow
  - FF magnitude: |tau_inertia| = |J*alpha| and |alpha_filt| during driver nudges
  - guard sanity: tau_inertia must be ~0 whenever the driver is inside the deadzone
  - counterfactual: how much the applied steering offset (angleOverride) WOULD have changed
    if the FF were applied live -- the safety preview for going live

The telemetry is self-contained: the raw driver torque is reconstructed as
    tau_raw = tauIntent + tauInertia
(carstate logs tauIntent = tau_raw - tauInertia), so no cross-message alignment is needed
for the FF analysis. vEgo (for speed context) is pulled from carState by nearest-time.

Usage:
  python tools/sunnypilot/vtb/analyze_shadow.py                       # all routes under the dir
  python tools/sunnypilot/vtb/analyze_shadow.py --dir ~/.comma/media/0/realdata
  python tools/sunnypilot/vtb/analyze_shadow.py --routes ROUTE_ID
"""
from __future__ import annotations

import argparse
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

from openpilot.tools.sunnypilot.vtb import logio
from openpilot.tools.sunnypilot.vtb.vtb_constants import DEADZONE_NM, LOCAL_LOG_ROOTS

DEFAULT_DIR = LOCAL_LOG_ROOTS[0]   # single local root; DEADZONE_NM re-exported (transcribe imports it)


def deadzone(x: np.ndarray, dz: float = DEADZONE_NM) -> np.ndarray:
  """Continuous deadzone: 0 inside [-dz, dz], else x - sign(x)*dz (matches coop_steering.py)."""
  return x - np.clip(x, -dz, dz)


def find_routes(root: str, routes: list[str] | None) -> dict[str, list[str]]:
  return logio.group_routes(roots=(root,), routes_filter=routes)


# carStateSP.coopSteering columns in fixed order (the POOLED concat relies on this key set)
_COLS = ("t", "coop", "comp", "shadow", "alpha", "tau_inertia", "tau_intent", "j_used", "angle_override")


def load_route(paths: list[str]) -> dict[str, np.ndarray]:
  sig = logio.read_signals(paths)   # decoded once + cached (one signals.npz shared with fit_steer_inertia)
  c = sig["carStateSP"]
  d = {
    "t": c["t"],
    "coop": c["coopSteering.coopActive"],
    "comp": c["coopSteering.inertiaCompActive"],
    "shadow": c["coopSteering.shadowActive"],
    "alpha": c["coopSteering.alphaFilt"],
    "tau_inertia": c["coopSteering.tauInertia"],
    "tau_intent": c["coopSteering.tauIntent"],
    "j_used": c["coopSteering.inertiaJUsed"],
    "angle_override": c["coopSteering.angleOverride"],
  }
  d["tau_raw"] = d["tau_intent"] + d["tau_inertia"]   # reconstruct raw driver torque
  # align vEgo (nearest prior carState sample; clamp pre-start to the first sample — old behavior)
  d["vego"] = logio.zoh_align(sig["carState"]["t"], sig["carState"]["vEgo"], d["t"], fill=None)
  return d


def pct(x: np.ndarray, q: float) -> float:
  return float(np.percentile(x, q)) if len(x) else float("nan")


def analyze(d: dict, label: str) -> dict:
  n = len(d["t"])
  coop = d["coop"]
  nudge = coop & (np.abs(d["tau_raw"]) > DEADZONE_NM)        # driver actually engaging the wheel
  r: dict = {"label": label, "n": n,
             "n_coop": int(coop.sum()), "n_shadow": int((coop & d["shadow"]).sum()),
             "n_comp_live": int((coop & d["comp"]).sum()), "n_nudge": int(nudge.sum())}
  if n:
    r["dur_min"] = n / 6000.0  # carStateSP @ 100 Hz; robust to per-route logMonoTime resets when pooled

  # FF magnitude during nudges
  if nudge.sum():
    ti = np.abs(d["tau_inertia"][nudge])
    al = np.abs(d["alpha"][nudge])
    raw = np.abs(d["tau_raw"][nudge])
    frac = ti / np.maximum(raw, 1e-6)
    r["tauI_p50"], r["tauI_p95"], r["tauI_max"] = pct(ti, 50), pct(ti, 95), float(ti.max())
    r["alpha_p50"], r["alpha_p95"], r["alpha_max"] = pct(al, 50), pct(al, 95), float(al.max())
    r["frac_p50"], r["frac_p95"], r["frac_max"] = pct(frac, 50), pct(frac, 95), float(frac.max())

  # guard sanity: inside the deadzone the FF must be zero
  inside = coop & (np.abs(d["tau_raw"]) <= DEADZONE_NM)
  r["guard_violations"] = int((inside & (np.abs(d["tau_inertia"]) > 1e-6)).sum())
  r["guard_checked"] = int(inside.sum())

  # J actually used
  ju = d["j_used"][coop]
  r["j_used"] = sorted({round(float(v), 4) for v in ju}) if len(ju) else []

  # counterfactual: applied override is driven off tau_raw (shadow); live would drive off tau_intent.
  # so would-be live override ~= angleOverride * deadzone(tau_intent)/deadzone(tau_raw).
  dz_raw = deadzone(d["tau_raw"])
  dz_int = deadzone(d["tau_intent"])
  cf = coop & (np.abs(dz_raw) > 0.05) & (np.abs(d["angle_override"]) > 1e-3)
  if cf.sum():
    ratio = np.clip(dz_int[cf] / dz_raw[cf], -2.0, 2.0)
    wouldbe = d["angle_override"][cf] * ratio
    delta = np.abs(d["angle_override"][cf] - wouldbe)
    ovr = np.abs(d["angle_override"][cf])
    r["ovr_abs_p95"], r["ovr_abs_max"] = pct(ovr, 95), float(ovr.max())
    r["cf_delta_p50"], r["cf_delta_p95"], r["cf_delta_max"] = pct(delta, 50), pct(delta, 95), float(delta.max())
    r["cf_n"] = int(cf.sum())
  return r


def fmt(r: dict) -> str:
  head = f"\n### {r['label']}  (n={r['n']} carStateSP frames"
  head += f", {r.get('dur_min', 0):.1f} min)" if r["n"] else ")"
  L = [head]
  if not r["n"]:
    return L[0] + "\n  (no carStateSP telemetry found)"
  L.append(f"  coverage: coop active {r['n_coop']} ({100 * r['n_coop'] / r['n']:.0f}%), " +
           f"shadow {r['n_shadow']}, live-comp {r['n_comp_live']}, driver-nudge frames {r['n_nudge']}")
  if r["n_coop"] == 0:
    L.append("  -> coop steering never active on this route; no FF telemetry to analyze")
    return "\n".join(L)
  if "tauI_p50" in r:
    L.append(f"  FF during nudges: |tau_inertia| p50/p95/max = {r['tauI_p50']:.3f}/{r['tauI_p95']:.3f}/{r['tauI_max']:.3f} Nm")
    L.append(f"                    |alpha_filt|  p50/p95/max = {r['alpha_p50']:.2f}/{r['alpha_p95']:.2f}/{r['alpha_max']:.2f} rad/s^2")
    L.append(f"                    FF/|tau_raw|  p50/p95/max = {100 * r['frac_p50']:.0f}%/{100 * r['frac_p95']:.0f}%/{100 * r['frac_max']:.0f}%")
  guard_ok = "OK" if r["guard_violations"] == 0 else "CHECK"
  L.append(f"  deadzone guard: {r['guard_violations']} violations / {r['guard_checked']} in-deadzone frames ({guard_ok})")
  L.append(f"  J used (kg*m^2): {r['j_used']}")
  if "cf_delta_p95" in r:
    L.append(f"  applied override |angleOverride| p95/max = {r['ovr_abs_p95']:.2f}/{r['ovr_abs_max']:.2f} deg  (n={r['cf_n']})")
    L.append("  COUNTERFACTUAL if FF applied live: |delta angleOverride| p50/p95/max = " +
             f"{r['cf_delta_p50']:.2f}/{r['cf_delta_p95']:.2f}/{r['cf_delta_max']:.2f} deg")
  return "\n".join(L)


def main():
  ap = argparse.ArgumentParser(description="Analyze VTB inertia-comp shadow-mode telemetry from drive rlogs")
  ap.add_argument("--dir", default=DEFAULT_DIR, help=f"dir with <route>--<seg>/rlog.zst (default {DEFAULT_DIR})")
  ap.add_argument("--routes", nargs="*", default=None, help="route names to include (default: all)")
  args = ap.parse_args()

  routes = find_routes(args.dir, args.routes)
  if not routes:
    raise SystemExit(f"no rlog.zst found under {os.path.expanduser(args.dir)}")

  print("=" * 88)
  print(f"VTB shadow-mode analysis  |  dir={os.path.expanduser(args.dir)}  deadzone={DEADZONE_NM}Nm")
  print(f"routes: {list(routes)}")
  print("=" * 88)

  loaded = {name: load_route(paths) for name, paths in routes.items()}
  for name, d in loaded.items():
    print(fmt(analyze(d, name)))

  # pooled
  if len(loaded) > 1:
    keys = list(_COLS) + ["tau_raw", "vego"]
    pooled = {k: np.concatenate([loaded[n][k] for n in loaded]) for k in keys}
    print(fmt(analyze(pooled, "POOLED (all routes)")))

  print("\n" + "=" * 88)
  print("Notes: J is hard to identify from normal driving (hands-off wheel accel is small); the")
  print("counterfactual above uses the on-device J. For a confident J fit do a targeted excitation")
  print("drive and run /vtb-tune (tools/sunnypilot/vtb/fit_steer_inertia.py).")


if __name__ == "__main__":
  main()
