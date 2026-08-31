"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
import pyray as rl

from iqpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from iqpilot.ui.onroad.hud_overlays import IQBlindSpotOverlay
from iqpilot.ui.mici.onroad.emac_source import EmacSourceIndicator

class IQMiciHudRenderer(HudRenderer):
  def __init__(self):
    super().__init__()
    self._overlays = [IQBlindSpotOverlay(), EmacSourceIndicator()]

  def _update_state(self) -> None:
    super()._update_state()
    for overlay in self._overlays:
      overlay.update()

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    for overlay in self._overlays:
      overlay.render(rect)

  def _has_blind_spot_detected(self) -> bool:
    return any(getattr(overlay, "detected", False) for overlay in self._overlays)
