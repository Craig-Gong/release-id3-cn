"""C3XL onroad traffic / lane chip aligned with MAX + speed-limit edges."""
from __future__ import annotations

import time

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.nav.hud_layout import LAMP_COL_W, junction_bar_rect
from openpilot.sunnypilot.nav.snapshot import read_snapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.junction_hud import (
  GreenFlashState,
  JunctionView,
  build_junction_view,
)
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

_BG = rl.Color(8, 10, 14, 188)
_BORDER = rl.Color(255, 255, 255, 42)
_HEAD = rl.Color(248, 250, 252, 245)
_DETAIL = rl.Color(186, 198, 214, 230)
_IDLE_HEAD = rl.Color(214, 222, 232, 220)
_STRIPE_IDLE = rl.Color(120, 132, 148, 200)

_LAMP_ON = {
  "red": rl.Color(232, 56, 62, 255),
  "yellow": rl.Color(236, 186, 28, 255),
  "green": rl.Color(56, 204, 118, 255),
}
_LAMP_OFF = {
  "red": rl.Color(92, 28, 32, 160),
  "yellow": rl.Color(92, 74, 22, 150),
  "green": rl.Color(22, 78, 48, 150),
}
_ACCENT = {
  "red": rl.Color(210, 48, 52, 230),
  "yellow": rl.Color(214, 168, 24, 230),
  "green": rl.Color(72, 196, 112, 230),
  "none": _STRIPE_IDLE,
}


class JunctionHudRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._view = JunctionView(False, "none", "", "", True, False)
    self._flash = GreenFlashState()
    try:
      self._font_head = gui_app.font(FontWeight.UNIFONT)
      self._font_detail = gui_app.font(FontWeight.UNIFONT)
    except Exception:
      self._font_head = gui_app.font(FontWeight.SEMI_BOLD)
      self._font_detail = gui_app.font(FontWeight.MEDIUM)

  def update(self) -> None:
    if ui_state.sm.recv_frame["carState"] < ui_state.started_frame:
      self._view = JunctionView(False, "none", "", "", True, False)
      return
    snap = read_snapshot()
    engaged = bool(ui_state.engaged)
    has_lead = False
    model_stop = False
    hold = False
    try:
      has_lead = bool(getattr(ui_state.sm["radarState"].leadOne, "status", False))
    except Exception:
      pass
    try:
      model_stop = bool(ui_state.sm["modelV2"].action.shouldStop)
    except Exception:
      pass
    try:
      hold = bool(ui_state.sm["carState"].standstill) and model_stop
    except Exception:
      pass
    stopping = bool((snap.stop_for_light or model_stop or hold) and not has_lead)
    flashing = self._flash.update(
      stopping=stopping, light=snap.light_token, engaged=engaged, now=time.monotonic(),
    )
    self._view = build_junction_view(
      engaged=engaged,
      has_lead=has_lead,
      model_stop=model_stop,
      standstill_hold=hold,
      snap=snap,
      green_flash=flashing,
    )

  def _render(self, rect: rl.Rectangle) -> None:
    view = self._view
    if not view.show:
      return
    metric = bool(ui_state.is_metric)
    band = junction_bar_rect(rect.x, rect.y, metric=metric)
    bar = rl.Rectangle(band.x, band.y, band.w, band.h)
    rl.draw_rectangle_rounded(bar, 0.22, 12, _BG)
    rl.draw_rectangle_rounded_lines_ex(bar, 0.22, 12, 2.0, _BORDER)

    accent = _ACCENT.get(view.light, _STRIPE_IDLE)
    stripe = rl.Rectangle(bar.x + 8, bar.y + 10, 7, bar.height - 20)
    rl.draw_rectangle_rounded(stripe, 0.8, 6, accent)

    self._draw_lamps(bar, view.light)
    text_x = bar.x + 8 + 7 + 10 + LAMP_COL_W
    text_right = bar.x + bar.width - 18
    inner_w = max(40.0, text_right - text_x)
    mid_y = bar.y + bar.height / 2

    headline = view.headline
    detail = view.detail
    head_color = _IDLE_HEAD if view.idle else _HEAD
    if detail:
      head_size = 40 if len(headline) <= 4 else 34
      det_size = 28
      head_sz = measure_text_cached(self._font_head, headline, head_size)
      det_sz = measure_text_cached(self._font_detail, detail, det_size)
      rl.draw_text_ex(
        self._font_head, headline,
        rl.Vector2(text_x, mid_y - head_sz.y / 2),
        head_size, 0, head_color,
      )
      rl.draw_text_ex(
        self._font_detail, detail,
        rl.Vector2(text_right - det_sz.x, mid_y - det_sz.y / 2),
        det_size, 0, _DETAIL,
      )
    else:
      head_size = 40
      head_sz = measure_text_cached(self._font_head, headline, head_size)
      rl.draw_text_ex(
        self._font_head, headline,
        rl.Vector2(text_x + (inner_w - head_sz.x) / 2, mid_y - head_sz.y / 2),
        head_size, 0, head_color,
      )

  def _draw_lamps(self, bar: rl.Rectangle, light: str) -> None:
    col_x = bar.x + 8 + 7 + 10
    cy = bar.y + bar.height / 2
    order = ("red", "yellow", "green")
    r = 9.0
    gap = 6.0
    total = 2 * r * 3 + gap * 2
    y0 = cy - total / 2 + r
    for i, name in enumerate(order):
      y = y0 + i * (2 * r + gap)
      on = light == name
      fill = _LAMP_ON[name] if on else _LAMP_OFF[name]
      center = rl.Vector2(col_x + LAMP_COL_W / 2 - 8, y)
      rl.draw_circle_v(center, r + 1.5, rl.Color(0, 0, 0, 90))
      rl.draw_circle_v(center, r, fill)
      if on:
        rl.draw_ring(center, r - 2.2, r + 0.6, 0, 360, 24, rl.Color(255, 255, 255, 70))
      else:
        rl.draw_ring(center, r - 1.4, r, 0, 360, 18, rl.Color(255, 255, 255, 28))
