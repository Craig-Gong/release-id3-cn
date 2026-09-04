"""C3XL HUD geometry: junction bar aligns with MAX + speed-limit chip edges.

MAX:   x = rect.x + 60 + (172 - set_w)//2, y = rect.y + 45, w = set_w, h = 204
Limit: x = rect.x + 60 + set_w + 30 - 6,   y = rect.y + 45 - 6, w = set_w, h = 216
Bar sits below those chips; width is MAX.left → limit.right even if SLA is off.
"""
from __future__ import annotations

from dataclasses import dataclass

SET_SPEED_W_METRIC = 200
SET_SPEED_W_IMPERIAL = 172
SET_SPEED_H = 204
MAX_X0 = 60
MAX_Y0 = 45
LIMIT_GAP = 30
LIMIT_PAD = 6
BAR_GAP_BELOW = 12
# Taller band (same width) so headline + metric capsules read clearly on-road.
BAR_HEIGHT = 130
SIGNAL_W = 52
SIGNAL_PAD_X = 14
CONTENT_GAP = 16
CAPSULE_W = 108
CAPSULE_H = 44
CAPSULE_GAP = 8
CAPSULE_RIGHT_PAD = 16
# Lane-change / turn guide sits under the junction signal bar.
# Badge centers on the signal column; title x matches junction headline.
LANE_GUIDE_GAP = 10
LANE_GUIDE_HEIGHT = 72
LANE_BADGE_W = 48  # centered in SIGNAL_W column
LANE_TEXT_SIZE = 32


@dataclass(frozen=True)
class HudBand:
  x: float
  y: float
  w: float
  h: float

  @property
  def right(self) -> float:
    return self.x + self.w


def set_speed_width(metric: bool = True) -> int:
  return SET_SPEED_W_METRIC if metric else SET_SPEED_W_IMPERIAL


def max_chip_rect(hud_x: float, hud_y: float, *, metric: bool = True) -> HudBand:
  set_w = set_speed_width(metric)
  x = hud_x + MAX_X0 + (SET_SPEED_W_IMPERIAL - set_w) // 2
  return HudBand(x, hud_y + MAX_Y0, float(set_w), float(SET_SPEED_H))


def limit_chip_rect(hud_x: float, hud_y: float, *, metric: bool = True) -> HudBand:
  set_w = set_speed_width(metric)
  x = hud_x + MAX_X0 + set_w + LIMIT_GAP - LIMIT_PAD
  y = hud_y + MAX_Y0 - LIMIT_PAD
  return HudBand(x, y, float(set_w), float(SET_SPEED_H + LIMIT_PAD * 2))


def junction_bar_rect(hud_x: float, hud_y: float, *, metric: bool = True) -> HudBand:
  max_r = max_chip_rect(hud_x, hud_y, metric=metric)
  limit_r = limit_chip_rect(hud_x, hud_y, metric=metric)
  y = max(max_r.y + max_r.h, limit_r.y + limit_r.h) + BAR_GAP_BELOW
  return HudBand(max_r.x, y, limit_r.right - max_r.x, float(BAR_HEIGHT))


def lane_guide_rect(hud_x: float, hud_y: float, *, metric: bool = True) -> HudBand:
  """Same width as junction bar; sits directly underneath with a small gap."""
  bar = junction_bar_rect(hud_x, hud_y, metric=metric)
  return HudBand(bar.x, bar.y + bar.h + LANE_GUIDE_GAP, bar.w, float(LANE_GUIDE_HEIGHT))
