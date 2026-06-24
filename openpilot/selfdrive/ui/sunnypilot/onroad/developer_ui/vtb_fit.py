"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Pure (GL-free) helpers for the VTB inertia-comp fit-readiness Developer UI readout.
Kept free of pyray / ui_state so the logic is unit-testable without a display context.

Thresholds mirror tools/sunnypilot/vtb/fit_steer_inertia.py so the live meter matches what the
offline system-ID fit will actually accept. Keep these in sync if the fit/algorithm retune.
"""

VTB_ALPHA_FLOOR = 5.0    # rad/s^2 - min |alpha| excitation for a usable ID sample (fit ALPHA_FLOOR)
VTB_DEADZONE_NM = 0.5    # Nm - hands-off torque gate (fit DEADZONE_NM == coop STEER_OVERRIDE_MIN_TORQUE)
VTB_FIT_SAMPLES = 200    # samples - fit's "n < 200 -> INSUFFICIENT" gate (= 2.0 s at 100 Hz)
VTB_FF_LIMIT = 2.5       # Nm - inertia FF clamp (coop_steering.py STEER_INERTIA_TORQUE_LIMIT)


def vtb_sample_qualifies(coop, car_state) -> bool:
  """Mirror fit_steer_inertia.id_mask: lat & ~pressed & |tau| < deadzone & |alpha| > floor.

  J is only identifiable from hands-off, openpilot-steered, high-acceleration moments (driver torque
  would otherwise contaminate the tau = J*alpha regression), so a "good" sample is hands-OFF, not -on.
  """
  return bool(coop.coopActive
              and not car_state.steeringPressed
              and abs(car_state.steeringTorque) < VTB_DEADZONE_NM
              and abs(coop.alphaFilt) > VTB_ALPHA_FLOOR)


def vtb_fit_status(coop, n: int) -> str:
  """FF-health verdict driving the FIT readout color, in priority order.

  Returns one of: "sign_error", "saturated", "ready", "gathering". The deadzone-guard case is
  intentionally absent: tauInertia is zeroed inside the deadzone before being published, so it can
  never fire on the cereal signal.
  """
  ti, af = coop.tauInertia, coop.alphaFilt
  # tauInertia = J * alphaFilt with J >= 0, so they must share sign; opposite => polarity bug.
  if abs(ti) > 0.01 and abs(af) > 0.01 and ti * af < 0:
    return "sign_error"
  # FF hard-clamped: J too high or alpha too large, override no longer tracks true intent.
  if abs(ti) >= 0.95 * VTB_FF_LIMIT:
    return "saturated"
  if n >= VTB_FIT_SAMPLES:
    return "ready"
  return "gathering"
