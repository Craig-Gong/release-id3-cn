"""MAX cluster geometry for stacking junction / lane-guide cards underneath.

Keep in sync with HudRenderer._draw_set_speed (UI_CONFIG). Official IQ draws
LIMIT inside the MAX box, so the cluster is that box only.
"""
from __future__ import annotations

SET_SPEED_W_METRIC = 186
SET_SPEED_W_IMPERIAL = 174
SET_SPEED_H = 228
CLUSTER_GAP = 16
CARD_GAP = 12
JUNC_CARD_H = 120
GUIDE_CARD_H = 64


def cluster_box(screen_x: float, screen_y: float, *, metric: bool) -> tuple[float, float, float, float]:
  """Return (x, y, w, h) of the MAX set-speed box."""
  set_w = SET_SPEED_W_METRIC if metric else SET_SPEED_W_IMPERIAL
  x = screen_x + 60 + (SET_SPEED_W_IMPERIAL - set_w) // 2
  y = screen_y + 45
  return x, y, set_w, SET_SPEED_H


def stack_layout(cluster_bottom: float, *, junction: bool, guide: bool) -> tuple[float | None, float | None]:
  """Place cards under the MAX box. Returns (junction_y, guide_y)."""
  y = cluster_bottom + CLUSTER_GAP
  junction_y = None
  guide_y = None
  if junction:
    junction_y = y
    y += JUNC_CARD_H + CARD_GAP
  if guide:
    guide_y = y
  return junction_y, guide_y
