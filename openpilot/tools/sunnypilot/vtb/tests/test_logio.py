"""Tests for the shared VTB log-parse + signal-cache layer (openpilot/tools/sunnypilot/vtb/logio.py).

Unlike test_fit_steer_inertia.py (which mocks at the numpy-dict level), these exercise the real
rlog -> arrays path: a synthetic rlog is built with save_log()/new_message() and read back through
resolve_segments -> read_signals -> signals.npz cache, plus the fit/analyze_shadow load_route adapters.
"""
import os

import numpy as np
import pytest

from openpilot.cereal import messaging
from openpilot.tools.lib.logreader import save_log
from openpilot.tools.sunnypilot.vtb import logio
from openpilot.tools.sunnypilot.vtb import fit_steer_inertia as fit
from openpilot.tools.sunnypilot.vtb import analyze_shadow as shadow

# Synthetic carState frames at 100 Hz (t in seconds); tau/rate/angle/vego/pressed known per frame.
_CS_T = [0.00, 0.01, 0.02, 0.03]
_TAU = [1.0, 2.0, 3.0, 4.0]
_RATE = [10.0, 20.0, 30.0, 40.0]
_ANGLE = [5.0, 6.0, 7.0, 8.0]
_VEGO = [11.0, 12.0, 13.0, 14.0]
_PRESSED = [False, True, False, True]
# carControl frames (latActive) and carStateSP frames on their own clocks.
_CC_T = [0.005, 0.025]
_CC_LAT = [True, False]
_SP_T = [0.012, 0.022, 0.032]
_SP_COOP = [True, True, False]
_SP_ALPHA = [0.1, 0.2, 0.3]
_SP_TAUI = [0.4, 0.5, 0.6]
_SP_TAUINT = [0.7, 0.8, 0.9]


def _ns(sec: float) -> int:
  return int(round(sec * 1e9))


def _build_route(tmp_path, route="vtbtest01--0123456789", seg=0) -> str:
  """Write a synthetic <route>--<seg>/rlog.zst under tmp_path; return the segment dir."""
  msgs = []
  for t, tau, rate, angle, v, pressed in zip(_CS_T, _TAU, _RATE, _ANGLE, _VEGO, _PRESSED, strict=True):
    m = messaging.new_message("carState")
    m.logMonoTime = _ns(t)
    m.carState.steeringTorque = tau
    m.carState.steeringRateDeg = rate
    m.carState.steeringAngleDeg = angle
    m.carState.vEgo = v
    m.carState.steeringPressed = pressed
    msgs.append(m)
  for t, lat in zip(_CC_T, _CC_LAT, strict=True):
    m = messaging.new_message("carControl")
    m.logMonoTime = _ns(t)
    m.carControl.latActive = lat
    msgs.append(m)
  for t, coop, al, ti, tint in zip(_SP_T, _SP_COOP, _SP_ALPHA, _SP_TAUI, _SP_TAUINT, strict=True):
    m = messaging.new_message("carStateSP")
    m.logMonoTime = _ns(t)
    m.carStateSP.coopSteering.coopActive = coop
    m.carStateSP.coopSteering.shadowActive = coop
    m.carStateSP.coopSteering.alphaFilt = al
    m.carStateSP.coopSteering.tauInertia = ti
    m.carStateSP.coopSteering.tauIntent = tint
    msgs.append(m)
  # Emit deliberately OUT of time order to exercise LogReader(sort_by_time=True).
  msgs = msgs[::-1]
  seg_dir = os.path.join(str(tmp_path), f"{route}--{seg}")
  os.makedirs(seg_dir, exist_ok=True)
  save_log(os.path.join(seg_dir, "rlog.zst"), [m.as_reader() for m in msgs])
  return seg_dir


def test_resolvers(tmp_path):
  seg_dir = _build_route(tmp_path)
  rlog = os.path.join(seg_dir, "rlog.zst")
  # explicit file, seg dir, and route-name-under-root all resolve to the same rlog
  assert logio.resolve_segments(rlog) == [rlog]
  assert logio.resolve_segments(seg_dir) == [rlog]
  assert logio.resolve_segments("vtbtest01--0123456789", roots=(str(tmp_path),)) == [rlog]
  assert logio.list_local_routes(roots=(str(tmp_path),)) == ["vtbtest01--0123456789"]
  assert logio.group_routes(roots=(str(tmp_path),)) == {"vtbtest01--0123456789": [rlog]}
  with pytest.raises(SystemExit):
    logio.resolve_segments("nope--nope", roots=(str(tmp_path),))
  assert logio.resolve_segments("nope--nope", roots=(str(tmp_path),), missing_ok=True) == []


def test_read_signals_values_dtypes(tmp_path):
  paths = logio.resolve_segments(_build_route(tmp_path))
  sig = logio.read_signals(paths, use_cache=False)
  cs = sig["carState"]
  assert np.all(np.diff(cs["t"]) > 0)                                    # sorted despite reversed emit
  np.testing.assert_allclose(cs["t"], _CS_T)
  np.testing.assert_allclose(cs["steeringTorque"], _TAU)
  np.testing.assert_allclose(cs["vEgo"], _VEGO)
  assert cs["steeringPressed"].dtype == np.bool_
  np.testing.assert_array_equal(cs["steeringPressed"], _PRESSED)
  assert sig["carControl"]["latActive"].dtype == np.bool_
  # dotted nested extraction
  np.testing.assert_allclose(sig["carStateSP"]["coopSteering.alphaFilt"], _SP_ALPHA)
  assert sig["carStateSP"]["coopSteering.coopActive"].dtype == np.bool_
  np.testing.assert_array_equal(sig["carStateSP"]["coopSteering.coopActive"], _SP_COOP)


def test_cache_hit_miss_and_subset(tmp_path, monkeypatch):
  paths = logio.resolve_segments(_build_route(tmp_path))
  fresh = logio.read_signals(paths, use_cache=True)                      # miss -> decode + write cache
  sd = os.path.dirname(paths[0])
  assert os.path.exists(os.path.join(sd, "signals.npz"))
  assert os.path.exists(os.path.join(sd, "signals.meta.json"))

  calls = []
  orig = logio._decode_segment
  monkeypatch.setattr(logio, "_decode_segment", lambda *a, **k: (calls.append(1), orig(*a, **k))[1])
  cached = logio.read_signals(paths, use_cache=True)                     # hit -> NO decode
  assert calls == []
  for svc in fresh:
    for k in fresh[svc]:
      np.testing.assert_array_equal(fresh[svc][k], cached[svc][k])
      assert fresh[svc][k].dtype == cached[svc][k].dtype

  # a subset spec still hits the superset cache (no decode)
  sub = logio.read_signals(paths, spec={"carState": {"vEgo": np.float64}}, use_cache=True)
  assert calls == []
  np.testing.assert_allclose(sub["carState"]["vEgo"], _VEGO)

  # bumping the rlog mtime invalidates the cache -> decode runs again
  st = os.stat(paths[0])
  os.utime(paths[0], (st.st_atime, st.st_mtime + 5.0))
  logio.read_signals(paths, use_cache=True)
  assert calls == [1]


def test_zoh_align():
  src_t = np.array([1.0, 3.0])
  src_v = np.array([10.0, 30.0])
  dst = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
  # clamp mode (fill=None): pre-start takes src[0]
  np.testing.assert_array_equal(logio.zoh_align(src_t, src_v, dst, fill=None), [10, 10, 10, 30, 30])
  # fill mode: pre-start takes the fill value
  np.testing.assert_array_equal(logio.zoh_align(src_t, np.array([True, False]), dst, fill=False),
                                [False, True, True, False, False])
  # empty src
  assert logio.zoh_align(np.array([]), np.array([], dtype=float), dst, fill=None).tolist() == [0] * 5
  assert logio.zoh_align(np.array([]), np.array([], dtype=bool), dst, fill=False).tolist() == [False] * 5


def test_load_route_adapters(tmp_path):
  """fit.load_route + analyze_shadow.load_route through the synthetic rlog == hand-computed."""
  seg_dir = _build_route(tmp_path)
  rlog = os.path.join(seg_dir, "rlog.zst")

  d = fit.load_route(seg_dir)          # resolve_segments accepts a seg-dir path
  np.testing.assert_allclose(d["t"], _CS_T)
  np.testing.assert_allclose(d["tau"], _TAU)
  assert d["pressed"].dtype == bool
  # lat = ZOH of carControl onto carState clock, fill=False before the first carControl sample
  #   cc_t=[.005,.025] lat=[T,F] ; cs_t=[0,.01,.02,.03] -> [F, T, T, F]
  np.testing.assert_array_equal(d["lat"], [False, True, True, False])

  s = shadow.load_route([rlog])
  np.testing.assert_allclose(s["t"], _SP_T)
  np.testing.assert_array_equal(s["coop"], _SP_COOP)
  np.testing.assert_allclose(s["tau_raw"], np.array(_SP_TAUINT) + np.array(_SP_TAUI))
  # vego = ZOH of carState.vEgo onto carStateSP clock (clamp): sp_t=[.012,.022,.032] -> cs idx [1,2,3]
  np.testing.assert_allclose(s["vego"], [_VEGO[1], _VEGO[2], _VEGO[3]])
