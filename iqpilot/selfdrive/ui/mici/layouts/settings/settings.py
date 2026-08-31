"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""

from iqpilot.common.params import Params
from iqpilot.system.ui.widgets.scroller import NavScroller
from iqpilot.selfdrive.ui.mici.widgets.stock_button import BigButton
from iqpilot.selfdrive.ui.mici.layouts.settings.toggles import TogglesLayoutMici
from iqpilot.selfdrive.ui.mici.layouts.settings.steering import SteeringLayoutMici
from iqpilot.selfdrive.ui.mici.layouts.settings.cruise import CruiseLayoutMici
from iqpilot.selfdrive.ui.mici.layouts.settings.visuals import VisualsLayoutMici
from iqpilot.selfdrive.ui.mici.layouts.settings.models import ModelsLayoutMici
from iqpilot.selfdrive.ui.mici.layouts.settings.display import DisplayLayoutMici
from iqpilot.selfdrive.ui.mici.layouts.settings.drive_history import TripsLayoutMici
from iqpilot.selfdrive.ui.mici.layouts.settings.vehicle import VehicleLayoutMici
from iqpilot.selfdrive.ui.mici.layouts.settings.dashcam import DashcamLayoutMici
from iqpilot.selfdrive.ui.mici.layouts.settings.network.network_layout import NetworkLayoutMici
from iqpilot.selfdrive.ui.mici.layouts.settings.device import DeviceLayoutMici, PairBigButton
from iqpilot.selfdrive.ui.mici.layouts.settings.developer import DeveloperLayoutMici
from iqpilot.selfdrive.ui.mici.layouts.settings.software import SoftwareLayoutMici
from iqpilot.system.ui.lib.application import gui_app, FontWeight
from iqpilot.system.ui.lib.multilang import tr


class SettingsBigButton(BigButton):
  def _get_label_font_size(self):
    return 64


class CruiseModeButton(SettingsBigButton):
  """Cruise menu button whose icon reflects the active longitudinal mode."""
  _ICON_SIZE = 60

  def __init__(self):
    super().__init__(tr("cruise"), "", gui_app.texture("icons_mici/speedometer.png", self._ICON_SIZE, self._ICON_SIZE))
    self._p = Params()
    self._icons = [
      gui_app.texture("icons_mici/speedometer.png", self._ICON_SIZE, self._ICON_SIZE),
      gui_app.texture("icons_mici/iqstandard_mode_mici.png", self._ICON_SIZE, self._ICON_SIZE),
      gui_app.texture("icons_mici/iqdynamic_mode_mici.png", self._ICON_SIZE, self._ICON_SIZE),
      gui_app.texture("icons_mici/experimental_mode_mici.png", self._ICON_SIZE, self._ICON_SIZE),
    ]

  def _mode_index(self) -> int:
    if not self._p.get_bool("AlphaLongitudinalEnabled"):
      return 0
    if not self._p.get_bool("ExperimentalMode"):
      return 1
    return 2 if self._p.get_bool("IQDynamicMode") else 3

  def _update_state(self):
    super()._update_state()
    self.set_icon(self._icons[self._mode_index()])


class SettingsLayout(NavScroller):
  def __init__(self):
    super().__init__()
    self._params = Params()

    toggles_panel = TogglesLayoutMici()
    toggles_btn = SettingsBigButton(tr("toggles"), "", gui_app.texture("icons_mici/settings.png", 64, 64))
    toggles_btn.set_click_callback(lambda: gui_app.push_widget(toggles_panel))

    steering_panel = SteeringLayoutMici()
    steering_btn = SettingsBigButton(tr("steering"), "", gui_app.texture("icons_mici/wheel.png", 64, 64))
    steering_btn.set_click_callback(lambda: gui_app.push_widget(steering_panel))

    cruise_panel = CruiseLayoutMici()
    cruise_btn = CruiseModeButton()
    cruise_btn.set_click_callback(lambda: gui_app.push_widget(cruise_panel))

    visuals_panel = VisualsLayoutMici()
    visuals_btn = SettingsBigButton(tr("visuals"), "", gui_app.texture("icons_mici/onroad/eye_fill.png", 64, 46))
    visuals_btn.set_click_callback(lambda: gui_app.push_widget(visuals_panel))

    models_panel = ModelsLayoutMici()
    models_btn = SettingsBigButton(tr("models"), "", gui_app.texture("icons_mici/models.png", 60, 60))
    models_btn.set_click_callback(lambda: gui_app.push_widget(models_panel))

    display_panel = DisplayLayoutMici()
    display_btn = SettingsBigButton(tr("display"), "", gui_app.texture("icons_mici/settings/brightness.png", 62, 62))
    display_btn.set_click_callback(lambda: gui_app.push_widget(display_panel))

    trips_panel = TripsLayoutMici()
    trips_btn = SettingsBigButton(tr("trips"), "", gui_app.texture("icons_mici/settings/trips.png", 62, 56))
    trips_btn.set_click_callback(lambda: gui_app.push_widget(trips_panel))

    vehicle_panel = VehicleLayoutMici()
    vehicle_btn = SettingsBigButton(tr("vehicle"), "", gui_app.texture("icons_mici/settings/vehicle.png", 70, 56))
    vehicle_btn.set_click_callback(lambda: gui_app.push_widget(vehicle_panel))

    dashcam_panel = DashcamLayoutMici()
    dashcam_btn = SettingsBigButton(tr("dashcam"), "", gui_app.texture("icons_mici/settings/camera.png", 64, 56))
    dashcam_btn.set_click_callback(lambda: gui_app.push_widget(dashcam_panel))

    network_panel = NetworkLayoutMici(back_callback=lambda: gui_app.pop_widget())
    network_btn = SettingsBigButton(tr("network"), "", gui_app.texture("icons_mici/settings/network/wifi_strength_full.png", 76, 56))
    network_btn.set_click_callback(lambda: gui_app.push_widget(network_panel))


    device_panel = DeviceLayoutMici()
    device_btn = SettingsBigButton(tr("device"), "", gui_app.texture("icons_mici/settings/device_icon.png", 72, 58))
    device_btn.set_click_callback(lambda: gui_app.push_widget(device_panel))

    software_panel = SoftwareLayoutMici()
    software_btn = SettingsBigButton(tr("software"), "", gui_app.texture("icons_mici/settings/sd_card.png", 60, 72))
    software_btn.set_click_callback(lambda: gui_app.push_widget(software_panel))

    developer_panel = DeveloperLayoutMici()
    developer_btn = SettingsBigButton(tr("developer"), "", gui_app.texture("icons_mici/settings/developer_icon.png", 64, 60))
    developer_btn.set_click_callback(lambda: gui_app.push_widget(developer_panel))

    self._scroller.add_widgets([
      device_btn,
      network_btn,
      PairBigButton(),
      models_btn,
      software_btn,
      steering_btn,
      cruise_btn,
      visuals_btn,
      display_btn,
      dashcam_btn,
      vehicle_btn,
      toggles_btn,
      trips_btn,
      developer_btn,
    ])

    self._font_medium = gui_app.font(FontWeight.MEDIUM)
