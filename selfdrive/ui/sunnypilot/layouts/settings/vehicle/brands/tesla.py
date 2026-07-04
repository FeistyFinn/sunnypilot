"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import option_item_sp, toggle_item_sp

COOP_STEERING_MIN_KMH = 23
OEM_STEERING_MIN_KMH = 48
KM_TO_MILE = 0.621371


class TeslaSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.coop_steering_toggle = toggle_item_sp(tr("Cooperative Steering (Beta)"), "", param="TeslaCoopSteering")
    self.mads_toggle_fingers_item = option_item_sp(
      title=tr("MADS Toggle Touch Points"),
      param="TeslaInfotainmentMadsToggleFingers",
      min_value=3, max_value=5, value_change_step=1,
      description="",
    )
    self.items = [self.coop_steering_toggle, self.mads_toggle_fingers_item]

  def update_settings(self):
    is_metric = ui_state.is_metric
    unit = "km/h" if is_metric else "mph"

    display_value_coop = COOP_STEERING_MIN_KMH if is_metric else round(COOP_STEERING_MIN_KMH * KM_TO_MILE)
    display_value_oem = OEM_STEERING_MIN_KMH if is_metric else round(OEM_STEERING_MIN_KMH * KM_TO_MILE)

    coop_steering_disabled_msg = tr("Enable \"Always Offroad\" in Device panel, or turn vehicle off to toggle.")
    coop_steering_warning = tr(f"Warning: May experience steering oscillations below {display_value_oem} {unit} during turns, " +
                               "recommend disabling this feature if you experience these.")
    coop_steering_desc = (
      f"<b>{coop_steering_warning}</b><br><br>" +
      f"{tr('Allows the driver to provide limited steering input while openpilot is engaged.')}<br>" +
      f"{tr(f'Only works above {display_value_coop} {unit}.')}"
    )
    mads_fingers_desc = tr(
      "Number of simultaneous fingers on the infotainment screen that toggle MADS. " +
      "Default 5 avoids accidental triggers from map zoom or climate gestures. " +
      "Deprecated Tesla harness only."
    )

    if not ui_state.is_offroad():
      coop_steering_desc = f"<b>{coop_steering_disabled_msg}</b><br><br>{coop_steering_desc}"
      mads_fingers_desc = f"<b>{coop_steering_disabled_msg}</b><br><br>{mads_fingers_desc}"

    self.coop_steering_toggle.set_description(coop_steering_desc)
    self.mads_toggle_fingers_item.set_description(mads_fingers_desc)

    offroad = ui_state.is_offroad()
    self.coop_steering_toggle.action_item.set_enabled(offroad)
    self.mads_toggle_fingers_item.action_item.set_enabled(offroad)
