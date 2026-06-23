"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os
import pyray as rl
rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)
os.environ["OFFSCREEN"] = "1"  # run raylib without an FPS limit / visible window

import pytest

from openpilot.system.ui.lib.application import gui_app


@pytest.fixture(scope="module", autouse=True)
def _gui_app():
  # The Tesla settings page builds raylib widgets, which need the font context initialized.
  gui_app.init_window("tesla-settings-test")
  try:
    yield
  finally:
    gui_app.close()


def test_coop_steering_toggle_wired_to_param():
  """The Cooperative Steering toggle must write the exact param the opendbc Tesla interface
  reads (TeslaCoopSteering) to set the CP_SP COOP_STEERING flag. If this drifts, the on-device
  toggle silently stops enabling VTB."""
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.tesla import TeslaSettings

  settings = TeslaSettings()
  assert settings.items == [
    settings.coop_steering_toggle,
    settings.coop_steering_inertia_comp_toggle,
    settings.coop_steering_inertia_shadow_toggle,
    settings.mads_toggle_fingers_item,
  ]
  assert settings.coop_steering_toggle.action_item.toggle.param_key == "TeslaCoopSteering"


def test_inertia_comp_toggle_wired_to_param():
  """The inertia-comp sub-toggle must write TeslaCoopSteeringInertiaComp -- the param the
  opendbc Tesla interface reads to set the CP_SP COOP_STEERING_INERTIA_COMP flag. Drift here
  silently breaks the in-field debug fallback for the FF term."""
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.tesla import TeslaSettings

  settings = TeslaSettings()
  assert settings.coop_steering_inertia_comp_toggle.action_item.toggle.param_key == "TeslaCoopSteeringInertiaComp"


def test_inertia_shadow_toggle_wired_to_param():
  """The shadow-mode sub-toggle must write TeslaCoopSteeringInertiaShadow -- the param the opendbc
  Tesla interface reads to set the CP_SP COOP_STEERING_INERTIA_SHADOW flag (FF computed + logged but
  NOT applied). Drift here would mislead the driver about whether the FF is actually steering."""
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.tesla import TeslaSettings

  settings = TeslaSettings()
  assert settings.coop_steering_inertia_shadow_toggle.action_item.toggle.param_key == "TeslaCoopSteeringInertiaShadow"


def test_mads_toggle_fingers_wired_to_param():
  """The MADS-toggle finger count picker must write TeslaInfotainmentMadsToggleFingers -- the
  param the opendbc Tesla interface reads to set the CP_SP MADS_TOGGLE_FINGERS_* flag bits."""
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.tesla import TeslaSettings

  settings = TeslaSettings()
  option = settings.mads_toggle_fingers_item.action_item
  assert option.param_key == "TeslaInfotainmentMadsToggleFingers"
  assert option.min_value == 3
  assert option.max_value == 5


def test_update_settings_locks_toggle_onroad(monkeypatch):
  """The settings widgets are editable only offroad (matches the panda safety gate that blocks
  the params while the car is on). update_settings() must not crash and must reflect the
  offroad state on every widget."""
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.tesla import TeslaSettings
  from openpilot.selfdrive.ui.ui_state import ui_state

  settings = TeslaSettings()

  monkeypatch.setattr(ui_state, "is_offroad", lambda: True)
  settings.update_settings()
  assert settings.coop_steering_toggle.action_item.enabled
  assert settings.coop_steering_inertia_comp_toggle.action_item.enabled
  assert settings.coop_steering_inertia_shadow_toggle.action_item.enabled
  assert settings.mads_toggle_fingers_item.action_item.enabled

  monkeypatch.setattr(ui_state, "is_offroad", lambda: False)
  settings.update_settings()
  assert not settings.coop_steering_toggle.action_item.enabled
  assert not settings.coop_steering_inertia_comp_toggle.action_item.enabled
  assert not settings.coop_steering_inertia_shadow_toggle.action_item.enabled
  assert not settings.mads_toggle_fingers_item.action_item.enabled
