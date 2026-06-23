"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import option_item_sp, toggle_item_sp

class TeslaSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.coop_steering_toggle = toggle_item_sp(tr("Cooperative Steering"), "", param="TeslaCoopSteering")
    self.coop_steering_inertia_comp_toggle = toggle_item_sp(
      title=tr("Cooperative Steering Inertia Compensation"),
      description="",
      param="TeslaCoopSteeringInertiaComp",
    )
    self.coop_steering_inertia_shadow_toggle = toggle_item_sp(
      title=tr("Inertia Compensation: Shadow Mode (log only — no steering)"),
      description="",
      param="TeslaCoopSteeringInertiaShadow",
    )
    self.mads_toggle_fingers_item = option_item_sp(
      title=tr("MADS Toggle Touch Points"),
      param="TeslaInfotainmentMadsToggleFingers",
      min_value=3, max_value=5, value_change_step=1,
      description="",
    )
    self.items = [self.coop_steering_toggle, self.coop_steering_inertia_comp_toggle,
                  self.coop_steering_inertia_shadow_toggle, self.mads_toggle_fingers_item]

  def update_settings(self):
    coop_steering_desc = (
      f"{tr('Converts light steering input into steering-wheel rotation.')}<br>" +
      f"{tr('The faster you go, the stiffer the steering gets.')}"
    )
    inertia_comp_desc = tr(
      "Subtracts the estimated wheel-inertia torque from driver torque before mapping to angle. " +
      "Disable to fall back to the un-compensated v1 algorithm if the compensation feels off. " +
      "While Shadow Mode (below) is on, the feedforward is only computed and logged — it does not steer."
    )
    inertia_shadow_desc = tr(
      "SHADOW MODE: the inertia feedforward is computed and logged for analysis but is NOT applied to " +
      "steering — baseline cooperative steering is unchanged. Turn this OFF to run the feedforward " +
      "live (it will actually adjust steering)."
    )
    mads_fingers_desc = tr(
      "Number of simultaneous fingers on the infotainment screen that toggle MADS. " +
      "Default 5 avoids accidental triggers from map zoom or climate gestures. " +
      "Deprecated Tesla harness only."
    )

    enable_offroad_msg = tr("Enable \"Always Offroad\" in Device panel, or turn vehicle off to toggle.")
    if not ui_state.is_offroad():
      coop_steering_desc = f"<b>{enable_offroad_msg}</b><br><br>{coop_steering_desc}"
      inertia_comp_desc = f"<b>{enable_offroad_msg}</b><br><br>{inertia_comp_desc}"
      inertia_shadow_desc = f"<b>{enable_offroad_msg}</b><br><br>{inertia_shadow_desc}"
      mads_fingers_desc = f"<b>{enable_offroad_msg}</b><br><br>{mads_fingers_desc}"

    self.coop_steering_toggle.set_description(coop_steering_desc)
    self.coop_steering_inertia_comp_toggle.set_description(inertia_comp_desc)
    self.coop_steering_inertia_shadow_toggle.set_description(inertia_shadow_desc)
    self.mads_toggle_fingers_item.set_description(mads_fingers_desc)

    offroad = ui_state.is_offroad()
    self.coop_steering_toggle.action_item.set_enabled(offroad)
    self.coop_steering_inertia_comp_toggle.action_item.set_enabled(offroad)
    self.coop_steering_inertia_shadow_toggle.action_item.set_enabled(offroad)
    self.mads_toggle_fingers_item.action_item.set_enabled(offroad)
