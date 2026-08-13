"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.iqpilot.ui.onroad.hud_overlays import IQBlindSpotOverlay
from openpilot.iqpilot.ui.onroad.display_speed_limit import (
  min_display_speed_limit_mps,
  read_iqlink_road_speed_mps,
)

_LIMIT_FONT = 72
_LIMIT_RIGHT_PAD = 18
_LIMIT_TOP_PAD = 12
_LIMIT_COLOR = rl.Color(255, 255, 255, int(255 * 0.9))


class IQMiciHudRenderer(HudRenderer):
  """Stock Mici HUD extended with IQ.Pilot's own onroad overlays.

  Overlays live in a list so the renderer stays overlay-agnostic — each just needs
  update()/render(rect); blind-spot state is surfaced by any overlay that exposes it.
  """

  def __init__(self):
    super().__init__()
    self._overlays = [IQBlindSpotOverlay()]
    self._display_limit_text = "---"
    self._font_limit = gui_app.font(FontWeight.DISPLAY)

  def _update_state(self) -> None:
    super()._update_state()
    for overlay in self._overlays:
      overlay.update()
    self._refresh_display_limit()

  def _refresh_display_limit(self) -> None:
    sm = ui_state.sm
    acc = 0.0
    vision = 0.0
    if sm.recv_frame["carState"] >= ui_state.started_frame:
      acc = float(sm["carState"].cruiseState.speedLimit or 0.0)
    if sm.recv_frame.get("iqLiveData", 0) >= ui_state.started_frame:
      live = sm["iqLiveData"]
      if bool(getattr(live, "speedLimitValid", False)):
        vision = float(getattr(live, "speedLimit", 0.0) or 0.0)
    nav_road = read_iqlink_road_speed_mps()
    limit_mps = min_display_speed_limit_mps(acc, nav_road, vision)
    if limit_mps is None:
      self._display_limit_text = "---"
      return
    scale = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self._display_limit_text = str(max(0, round(limit_mps * scale)))

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self._draw_display_limit(rect)
    for overlay in self._overlays:
      overlay.render(rect)

  def _draw_display_limit(self, rect: rl.Rectangle) -> None:
    """Always-on road speed limit on the right edge (independent of SLC mode)."""
    text = self._display_limit_text
    size = measure_text_cached(self._font_limit, text, _LIMIT_FONT)
    x = rect.x + rect.width - _LIMIT_RIGHT_PAD - size.x
    y = rect.y + _LIMIT_TOP_PAD
    rl.draw_text_ex(self._font_limit, text, rl.Vector2(x, y), _LIMIT_FONT, 0, _LIMIT_COLOR)

  def _has_blind_spot_detected(self) -> bool:
    return any(getattr(overlay, "detected", False) for overlay in self._overlays)
