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


class _FakeCPSP:
  """Minimal stand-in for CarParamsSP -- update_settings() only reads .flags."""
  def __init__(self, flags: int):
    self.flags = flags


def test_coop_steering_toggle_wired_to_param():
  """The Cooperative Steering toggle must write the exact param the opendbc Tesla interface
  reads (TeslaCoopSteering) to set the CP_SP COOP_STEERING flag. If this drifts, the on-device
  toggle silently stops enabling VTB."""
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.tesla import TeslaSettings

  settings = TeslaSettings()
  assert settings.items == [
    settings.coop_steering_toggle,
    settings.mads_screen_button,
  ]
  assert settings.coop_steering_toggle.action_item.toggle.param_key == "TeslaCoopSteering"


def test_mads_screen_button_wired_to_param():
  """The picker must write TeslaMadsScreenButton -- the param the opendbc Tesla interface reads to
  set the CP_SP MADS_SCREEN_BUTTON_*_FINGER flag + safetyParam bits."""
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.tesla import TeslaSettings

  settings = TeslaSettings()
  action = settings.mads_screen_button.action_item
  assert action.param_key == "TeslaMadsScreenButton"
  assert len(action.buttons) == 4


def test_mads_screen_button_index_matches_enum_ordinal():
  """LOAD-BEARING: MultipleButtonActionSP writes the selected button INDEX straight to the param,
  and opendbc reads that int as a MadsScreenButtonType ordinal. So button order must equal enum
  order -- reordering the buttons would silently remap every user's setting."""
  from opendbc.sunnypilot.car.tesla.values import MadsScreenButtonType
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.tesla import TeslaSettings

  labels = [b() if callable(b) else b for b in TeslaSettings().mads_screen_button.action_item.buttons]
  assert labels[MadsScreenButtonType.OFF] == "Off"
  assert labels[MadsScreenButtonType.THREE_FINGER] == "3 Finger"
  assert labels[MadsScreenButtonType.FOUR_FINGER] == "4 Finger"
  assert labels[MadsScreenButtonType.FIVE_FINGER] == "5 Finger"


def test_mads_screen_button_hidden_without_vehicle_bus(monkeypatch):
  """The infotainment gesture only exists on cars wired with the deprecated Tesla harness, so the
  picker is hidden unless CP_SP reports HAS_VEHICLE_BUS. CP_SP is None until a car has been seen."""
  from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.tesla import TeslaSettings
  from openpilot.selfdrive.ui.ui_state import ui_state

  settings = TeslaSettings()
  monkeypatch.setattr(ui_state, "is_offroad", lambda: True)

  monkeypatch.setattr(ui_state, "CP_SP", None, raising=False)
  settings.update_settings()
  assert not settings.mads_screen_button.is_visible

  monkeypatch.setattr(ui_state, "CP_SP", _FakeCPSP(TeslaFlagsSP.COOP_STEERING), raising=False)
  settings.update_settings()
  assert not settings.mads_screen_button.is_visible

  monkeypatch.setattr(ui_state, "CP_SP", _FakeCPSP(TeslaFlagsSP.HAS_VEHICLE_BUS), raising=False)
  settings.update_settings()
  assert settings.mads_screen_button.is_visible


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
  assert settings.mads_screen_button.action_item.enabled

  monkeypatch.setattr(ui_state, "is_offroad", lambda: False)
  settings.update_settings()
  assert not settings.coop_steering_toggle.action_item.enabled
  assert not settings.mads_screen_button.action_item.enabled
