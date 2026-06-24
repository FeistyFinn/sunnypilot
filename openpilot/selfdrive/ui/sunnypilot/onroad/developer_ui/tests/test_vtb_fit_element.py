"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Unit tests for the VTB inertia-comp fit-readiness logic. Tests the pure (GL-free) helpers in
developer_ui/vtb_fit.py against the offline fit's id_mask so the live meter stays faithful.
"""
from types import SimpleNamespace

from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui.vtb_fit import (
  VTB_ALPHA_FLOOR, VTB_DEADZONE_NM, VTB_FF_LIMIT, VTB_FIT_SAMPLES,
  vtb_sample_qualifies, vtb_fit_status,
)


def _coop(coopActive=True, alphaFilt=8.0, tauInertia=0.6, **kw):
  return SimpleNamespace(coopActive=coopActive, alphaFilt=alphaFilt, tauInertia=tauInertia,
                         inertiaCompActive=kw.get("inertiaCompActive", False),
                         shadowActive=kw.get("shadowActive", True),
                         inertiaJUsed=kw.get("inertiaJUsed", 0.08))


def _car(steeringPressed=False, steeringTorque=0.1):
  return SimpleNamespace(steeringPressed=steeringPressed, steeringTorque=steeringTorque)


class TestSampleQualifies:
  def test_hands_off_excited_qualifies(self):
    # The intended case: coop active, hands off, low torque, strong wheel acceleration.
    assert vtb_sample_qualifies(_coop(alphaFilt=8.0), _car(steeringPressed=False, steeringTorque=0.1))

  def test_coop_inactive_disqualifies(self):
    assert not vtb_sample_qualifies(_coop(coopActive=False), _car())

  def test_hands_on_disqualifies(self):
    # steeringPressed is the fit's ~pressed gate.
    assert not vtb_sample_qualifies(_coop(), _car(steeringPressed=True))

  def test_torque_above_deadzone_disqualifies(self):
    assert not vtb_sample_qualifies(_coop(), _car(steeringTorque=VTB_DEADZONE_NM + 0.01))

  def test_torque_at_deadzone_disqualifies(self):
    # Strict < deadzone, matching the fit (|tau| < DEADZONE_NM).
    assert not vtb_sample_qualifies(_coop(), _car(steeringTorque=VTB_DEADZONE_NM))

  def test_low_alpha_disqualifies(self):
    assert not vtb_sample_qualifies(_coop(alphaFilt=VTB_ALPHA_FLOOR), _car())

  def test_alpha_just_above_floor_qualifies(self):
    assert vtb_sample_qualifies(_coop(alphaFilt=VTB_ALPHA_FLOOR + 0.01), _car())

  def test_negative_alpha_qualifies_on_magnitude(self):
    assert vtb_sample_qualifies(_coop(alphaFilt=-(VTB_ALPHA_FLOOR + 0.01)), _car())


class TestFitStatus:
  def test_gathering_below_threshold(self):
    assert vtb_fit_status(_coop(tauInertia=0.6, alphaFilt=8.0), n=VTB_FIT_SAMPLES - 1) == "gathering"

  def test_ready_at_threshold(self):
    assert vtb_fit_status(_coop(tauInertia=0.6, alphaFilt=8.0), n=VTB_FIT_SAMPLES) == "ready"

  def test_saturated_same_sign(self):
    assert vtb_fit_status(_coop(tauInertia=0.96 * VTB_FF_LIMIT, alphaFilt=8.0), n=VTB_FIT_SAMPLES) == "saturated"

  def test_sign_error_takes_priority_over_saturation(self):
    # Opposite signs on a saturating tau -> still flagged as the (more serious) polarity bug.
    assert vtb_fit_status(_coop(tauInertia=-0.99 * VTB_FF_LIMIT, alphaFilt=8.0), n=10) == "sign_error"

  def test_sign_error_opposite_signs(self):
    assert vtb_fit_status(_coop(tauInertia=-0.6, alphaFilt=8.0), n=10) == "sign_error"

  def test_near_zero_not_sign_error(self):
    # Tiny noise around zero must not trip the sign-error guard.
    assert vtb_fit_status(_coop(tauInertia=0.005, alphaFilt=-0.005), n=10) == "gathering"
