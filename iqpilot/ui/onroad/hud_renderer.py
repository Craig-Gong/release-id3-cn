"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
import pyray as rl

from iqpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from iqpilot.selfdrive.ui.ui_state import ui_state
from iqpilot.selfdrive.ui.onroad.hud_renderer import HudRenderer
from iqpilot.ui.onroad.hud_overlays import (
  IQDevMetricsOverlay,
  RoadNameRenderer,
  IQAccelBar,
  IQSpeedLimitOverlay,
  IQTurnSignalOverlay,
  IQSpeedOverlay,
)
from iqpilot.ui.onroad.nav_map_panel import NavMapPanel
from iqpilot.ui.onroad.soft_warning import SoftWarningRenderer
from iqpilot.ui.onroad.emac_status import EmacStatusRenderer

ENABLE_FLOATING_NAV_MAP_PANEL = False
ENABLE_SPLIT_NAV_MAP_PANEL = True


class IQHudRenderer(HudRenderer):
  def __init__(self):
    super().__init__()
    self.developer_ui = IQDevMetricsOverlay()
    self.emac_status = EmacStatusRenderer()
    self.nav_map_panel = NavMapPanel()
    self.road_name_renderer = RoadNameRenderer()
    self.rocket_fuel = IQAccelBar()
    self.speed_limit_renderer = IQSpeedLimitOverlay()
    self.turn_signal_controller = IQTurnSignalOverlay()
    self.speed_renderer = IQSpeedOverlay()
    self.soft_warning_renderer = SoftWarningRenderer()
    self._torque_bar = TorqueBar(scale=3.0, always=True)

  def _update_state(self) -> None:
    super()._update_state()
    if ENABLE_FLOATING_NAV_MAP_PANEL or ENABLE_SPLIT_NAV_MAP_PANEL:
      self.nav_map_panel.update()
    self.emac_status.update()
    self.road_name_renderer.update()
    self.speed_limit_renderer.update()
    has_limit = self.speed_limit_renderer.speed_limit_valid or self.speed_limit_renderer.speed_limit_last_valid
    self.limit_available = has_limit
    self.limit_speed_text = str(round(self.speed_limit_renderer.speed_limit_last)) if has_limit else "---"
    offset = round(self.speed_limit_renderer.speed_limit_offset)
    self.limit_offset_text = f"{offset:+d}" if has_limit and offset != 0 else ""
    self.turn_signal_controller.update()
    self.speed_renderer.update()
    self.soft_warning_renderer.update()

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    self.speed_renderer.render(rect)

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)

    torque_rect = rect
    if ui_state.developer_ui in (IQDevMetricsOverlay.DEV_UI_BOTTOM, IQDevMetricsOverlay.DEV_UI_BOTH):
      torque_rect = rl.Rectangle(rect.x, rect.y, rect.width, rect.height - IQDevMetricsOverlay.BOTTOM_BAR_HEIGHT)
    if ui_state.torque_bar:
      self._torque_bar.render(torque_rect)

    if not self.split_nav_enabled():
      self.developer_ui.render(rect)
    if ENABLE_FLOATING_NAV_MAP_PANEL:
      self.nav_map_panel.render(rect)
    self.emac_status.render(rect)
    self.road_name_renderer.render(torque_rect)
    self.turn_signal_controller.render(rect)
    self.soft_warning_renderer.render(rect)
    self.rocket_fuel.render(rect, ui_state.sm)

  def split_nav_enabled(self) -> bool:
    if not ENABLE_SPLIT_NAV_MAP_PANEL:
      return False
    if hasattr(self.nav_map_panel, "maps_enabled"):
      return bool(self.nav_map_panel.maps_enabled())
    return bool(getattr(self.nav_map_panel, "_maps_enabled", False))

  def render_split_nav(self, rect: rl.Rectangle) -> None:
    if self.split_nav_enabled():
      self.nav_map_panel.render_split(rect)

  def render_full_width_overlays(self, rect: rl.Rectangle) -> None:
    if self.split_nav_enabled():
      self.developer_ui.render(rect)
