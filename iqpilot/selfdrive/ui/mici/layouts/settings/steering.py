"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""

from iqpilot.cereal import car
from iqpilot.common.params import Params
from iqpilot.selfdrive.controls.lib.helpers.lane_change import AutoLaneChangeMode
from iqpilot.selfdrive.ui.mici.widgets.stock_button import BigButton, BigParamControl
from iqpilot.selfdrive.ui.mici.layouts.settings.iq_widgets import MappedParamToggle
from iqpilot.selfdrive.ui.ui_state import ui_state
from iqpilot.system.ui.lib.application import gui_app
from iqpilot.system.ui.widgets.scroller import NavScroller
from iqpilot.system.ui.lib.multilang import tr


def _aol_modes() -> list[str]:
  return [tr("stay engaged"), tr("standby"), tr("disengage")]


class SabBrakeToggle(BigParamControl):
  """Driver-intervention toggle backed by AolSteeringMode == 2."""
  def __init__(self):
    super().__init__(tr("Driver Intervention Handling"), "AolEnabled")

  def refresh(self):
    self.set_checked(int(self.params.get("AolSteeringMode", return_default=True)) == 2)

  def _handle_mouse_release(self, mouse_pos):
    super(BigParamControl, self)._handle_mouse_release(mouse_pos)
    enabled = self._checked
    current_mode = int(self.params.get("AolSteeringMode", return_default=True))
    if enabled:
      self.params.put("AolSteeringMode", 2)
    elif current_mode == 2:
      self.params.put("AolSteeringMode", 1)


def _has_limited_sab_options() -> bool:
  brand = ""
  if ui_state.is_offroad():
    bundle = ui_state.params.get("CarPlatformBundle")
    if bundle:
      brand = bundle.get("brand", "")
  if not brand:
    brand = ui_state.CP.brand if ui_state.CP else ""
  return brand == "rivian"


class SabSettingsPanel(NavScroller):
  def __init__(self):
    super().__init__()
    self._main_cruise = BigParamControl(tr("Availability While Cruise Changes"), "AolMainCruiseAllowed")
    self._brake = SabBrakeToggle()
    self._mode = MappedParamToggle(tr("Brake Response Mode"), "AolSteeringMode",
                                   _aol_modes(), [0, 1, 2])
    self._steer_override = BigParamControl(tr("Pause While You Steer"), "AolPauseOnSteeringOverride")
    self._scroller.add_widgets([self._main_cruise, self._brake, self._mode, self._steer_override])

  def show_event(self):
    super().show_event()
    limited = _has_limited_sab_options()
    if limited:
      ui_state.params.remove("AolMainCruiseAllowed")
      ui_state.params.put_bool("AolUnifiedEngagementMode", True)
      ui_state.params.put("AolSteeringMode", 2)
    offroad = ui_state.is_offroad()
    for w in (self._main_cruise, self._brake, self._mode):
      w.refresh()
      w.set_enabled(offroad and not limited)
    self._steer_override.refresh()
    self._steer_override.set_enabled(offroad)


class LaneChangePanel(NavScroller):
  def __init__(self):
    super().__init__()
    self._timer = MappedParamToggle(tr("Auto Lane Change"), "IQLaneChangeTimer",
                                    [tr("off"), tr("nudge"), tr("no nudge"), "0.5 s", "1 s", "2 s", "3 s"],
                                    [-1, 0, 1, 2, 3, 4, 5])
    self._bsm_delay = BigParamControl(tr("Delay with Blind Spot"), "IQLaneChangeBsmDelay")
    self._edge_guard = BigParamControl(tr("Lane Edge Guard"), "IQEdgeGuard")
    self._edge_guard.set_value(tr("Blocks lane changes when a road edge is detected on the target side."))
    self._continuous = BigParamControl(tr("Continuous Changes"), "LaneChangeContinuous")
    self._scroller.add_widgets([self._timer, self._bsm_delay, self._edge_guard, self._continuous])

  def show_event(self):
    super().show_event()
    self._timer.refresh()
    enable_bsm = bool(ui_state.CP and ui_state.CP.enableBsm)
    if not enable_bsm and ui_state.params.get_bool("IQLaneChangeBsmDelay"):
      ui_state.params.remove("IQLaneChangeBsmDelay")
    self._bsm_delay.refresh()
    self._bsm_delay.set_enabled(
      enable_bsm and int(ui_state.params.get("IQLaneChangeTimer", return_default=True)) > AutoLaneChangeMode.NUDGE
    )
    self._edge_guard.refresh()
    self._continuous.refresh()


class SteeringLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._sab_panel = SabSettingsPanel()
    self._lc_panel = LaneChangePanel()

    self._aol = BigParamControl(tr("AOL"), "AolEnabled", toggle_callback=self._on_aol_toggled)
    self._sab_settings_button = BigButton(tr("steering assistance behavior"))
    self._sab_settings_button.set_click_callback(lambda: gui_app.push_widget(self._sab_panel))
    self._lane_change = BigButton(tr("lane change"))
    self._lane_change.set_click_callback(lambda: gui_app.push_widget(self._lc_panel))
    self._nnff = BigParamControl(tr("Neural Net FF"), "NeuralNetworkFeedForward", toggle_callback=self._on_nnff_toggled)

    self._scroller.add_widgets([
      self._aol, self._sab_settings_button, self._lane_change,
      self._nnff,
    ])

  def _aol_mode_str(self) -> str:
    try:
      return _aol_modes()[int(ui_state.params.get("AolSteeringMode", return_default=True))]
    except (TypeError, ValueError, IndexError):
      return _aol_modes()[0]

  def _on_aol_toggled(self, checked: bool):
    if checked:
      ui_state.params.put_bool("AolUnifiedEngagementMode", True)

  def _on_nnff_toggled(self, checked: bool):
    return None

  def _refresh(self):
    offroad = ui_state.is_offroad()
    self._aol.refresh()
    self._aol.set_value(self._aol_mode_str())
    self._nnff.refresh()

    steering_supported = (ui_state.CP is not None and
                          ui_state.CP.steerControlType != car.CarParams.SteerControlType.angle)
    if not steering_supported:
      ui_state.params.remove("NeuralNetworkFeedForward")
      self._nnff.refresh()

    self._aol.set_enabled(offroad)
    self._sab_settings_button.set_enabled(offroad and self._aol._checked)
    self._nnff.set_enabled(offroad and steering_supported)

  def _update_state(self):
    super()._update_state()
    self._refresh()

  def show_event(self):
    super().show_event()
    self._refresh()
