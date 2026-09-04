"""C3XL onroad traffic + lane-guide chips aligned with MAX + speed-limit edges.

Junction signal bar on top; when navigation recommends a lane / turn, a
compact guide strip sits directly underneath (same width). eGPU status sits
under the light bar, or under the lane-guide strip when that is showing.
"""
from __future__ import annotations

import time

import pyray as rl

from openpilot.common.params import UnknownKeyName
from openpilot.sunnypilot.selfdrive.ui.egpu_hud import HudEgpuView, build_hud_egpu_view
from openpilot.selfdrive.ui.egpu_status import chestnut_usb_speed_mbps, resolve_egpu_connection
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.nav.hud_copy import METERS, SECONDS
from openpilot.sunnypilot.nav.hud_layout import (
  CAPSULE_GAP, CAPSULE_H, CAPSULE_RIGHT_PAD, CAPSULE_W,
  CONTENT_GAP, EGPU_DC_PILL_GAP, EGPU_DC_UNIT, EGPU_DC_VALUE, EGPU_DETAIL_SIZE,
  EGPU_HEAD_SIZE, EGPU_PAD, EGPU_RAIL_W, EGPU_RAIL_X, EGPU_TILE_UNIT,
  EGPU_TILE_VALUE, LANE_BADGE_W, LANE_TEXT_SIZE, SIGNAL_PAD_X, SIGNAL_W,
  dc_pill_width, egpu_status_rect, junction_bar_rect, lane_guide_rect,
)
from openpilot.sunnypilot.nav.snapshot import read_snapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.junction_hud import (
  GreenFlashState,
  JunctionView,
  LaneGuideView,
  build_junction_view,
  build_lane_guide_view,
)
from openpilot.sunnypilot.system.ecoflow.status import ecoflow_dc_label, read_status
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

_BG = rl.Color(10, 12, 16, 210)
_BG_IDLE = rl.Color(12, 14, 18, 188)
_BG_LANE = rl.Color(14, 18, 24, 176)
_BORDER = rl.Color(255, 255, 255, 38)
_BORDER_LANE = rl.Color(120, 190, 220, 55)
_HEAD = rl.Color(250, 252, 255, 250)
_DETAIL = rl.Color(168, 184, 204, 235)
_IDLE_HEAD = rl.Color(198, 208, 220, 230)
_MUTED = rl.Color(120, 132, 148, 200)
_SIGNAL_WELL = rl.Color(4, 6, 10, 230)
_SIGNAL_RIM = rl.Color(255, 255, 255, 28)
_CAPSULE_BG = rl.Color(255, 255, 255, 18)
_CAPSULE_EDGE = rl.Color(255, 255, 255, 36)
_CAPSULE_NUM = rl.Color(250, 252, 255, 250)
_CAPSULE_UNIT = rl.Color(148, 164, 184, 230)
_LANE_ACCENT = rl.Color(88, 198, 220, 255)
_LANE_BADGE_BG = rl.Color(18, 28, 36, 220)
_LANE_BADGE_RIM = rl.Color(88, 198, 220, 70)
_LANE_TEXT = rl.Color(230, 244, 250, 245)
_LANE_SUB = rl.Color(140, 178, 196, 210)
_BG_EGPU = rl.Color(10, 14, 18, 214)
_BORDER_EGPU = rl.Color(255, 255, 255, 32)
_EGPU_TEXT = rl.Color(250, 252, 255, 250)
_EGPU_DETAIL = rl.Color(168, 184, 196, 230)
_EGPU_TILE_BG = rl.Color(255, 255, 255, 16)
_EGPU_TILE_EDGE = rl.Color(255, 255, 255, 28)
_EGPU_TILE_VALUE = rl.Color(250, 252, 255, 250)
_EGPU_TILE_UNIT = rl.Color(148, 164, 184, 230)
_EGPU_ACCENT = {
  "good": rl.Color(56, 214, 126, 255),
  "warning": rl.Color(242, 196, 36, 255),
  "danger": rl.Color(236, 64, 70, 255),
  "muted": rl.Color(120, 132, 148, 200),
}
_DC_PILL = {
  "on": (rl.Color(28, 64, 46, 230), rl.Color(88, 224, 148, 255)),
  "off": (rl.Color(64, 24, 28, 230), rl.Color(236, 96, 100, 255)),
  "idle": (rl.Color(255, 255, 255, 16), rl.Color(160, 172, 184, 230)),
  "unknown": (rl.Color(255, 255, 255, 16), rl.Color(160, 172, 184, 230)),
}

_LAMP_ON = {
  "red": rl.Color(236, 64, 70, 255),
  "yellow": rl.Color(242, 196, 36, 255),
  "green": rl.Color(56, 214, 126, 255),
}
_LAMP_OFF = {
  "red": rl.Color(68, 22, 26, 175),
  "yellow": rl.Color(68, 54, 18, 165),
  "green": rl.Color(18, 58, 38, 165),
}
_ACCENT = {
  "red": rl.Color(224, 56, 62, 255),
  "yellow": rl.Color(228, 178, 28, 255),
  "green": rl.Color(64, 208, 120, 255),
  "none": _MUTED,
}

class JunctionHudRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._view = JunctionView(False, "none", "", "", True, False)
    self._lane = LaneGuideView(False, "", "none")
    self._egpu = HudEgpuView()
    self._flash = GreenFlashState()
    try:
      self._font_head = gui_app.font(FontWeight.UNIFONT)
      self._font_detail = gui_app.font(FontWeight.UNIFONT)
    except Exception:
      self._font_head = gui_app.font(FontWeight.SEMI_BOLD)
      self._font_detail = gui_app.font(FontWeight.MEDIUM)
    try:
      self._font_num = gui_app.font(FontWeight.SEMI_BOLD)
      self._font_unit = gui_app.font(FontWeight.MEDIUM)
    except Exception:
      self._font_num = self._font_head
      self._font_unit = self._font_detail

  def update(self) -> None:
    if ui_state.sm.recv_frame["carState"] < ui_state.started_frame:
      self._view = JunctionView(False, "none", "", "", True, False)
      self._lane = LaneGuideView(False, "", "none")
      self._egpu = HudEgpuView()
      return
    snap = read_snapshot()
    engaged = bool(ui_state.engaged)
    has_lead = False
    model_stop = False
    hold = False
    try:
      lead = ui_state.sm["radarState"].leadOne
      has_lead = bool(getattr(lead, "present", False) or getattr(lead, "status", False))
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
    self._lane = build_lane_guide_view(engaged=engaged, snap=snap)
    self._egpu = self._build_egpu_view(engaged)

  def _ecoflow_enabled(self) -> bool:
    try:
      return bool(ui_state.params.get_bool("EcoflowEnabled"))
    except UnknownKeyName:
      return False
    except Exception:
      return False

  def _build_egpu_view(self, engaged: bool) -> HudEgpuView:
    sm = ui_state.sm
    connected = False
    model_alive = False
    model_big = False
    telemetry_valid = False
    usb_speed = 0
    fps = power = temp = 0.0
    mem_used = mem_total = 0
    try:
      connected = resolve_egpu_connection(sm["deviceState"])
      usb_speed = chestnut_usb_speed_mbps(sm["deviceState"])
    except Exception:
      pass
    try:
      model_seen = sm.recv_frame["modelV2"] > ui_state.started_frame
      model_alive = bool(model_seen and sm.alive["modelV2"])
      model_big = bool(model_alive and sm["modelV2"].big)
    except Exception:
      pass
    try:
      telemetry_valid = bool(sm.alive["chestnutState"] and sm.valid["chestnutState"])
      tel = sm["chestnutState"]
      fps = float(tel.modelFps)
      power = float(tel.powerDrawW)
      temp = float(tel.tempC)
      mem_used = int(tel.memoryUsedMb)
      mem_total = int(tel.memoryTotalMb)
    except Exception:
      telemetry_valid = False
    dc_label = ecoflow_dc_label(enabled=self._ecoflow_enabled(), snap=read_status())
    return build_hud_egpu_view(
      engaged=engaged, connected=connected, compiled=ui_state.usbgpu_compiled,
      loading=ui_state.usbgpu_loading, active=ui_state.usbgpu_active,
      model_alive=model_alive, model_big=model_big, telemetry_valid=telemetry_valid,
      loading_progress=ui_state.usbgpu_loading_progress, usb_speed_mbps=usb_speed,
      model_fps=fps, power_w=power, temp_c=temp,
      memory_used_mb=mem_used, memory_total_mb=mem_total, dc_label=dc_label,
    )

  def _render(self, rect: rl.Rectangle) -> None:
    view = self._view
    if not view.show:
      return
    metric = bool(ui_state.is_metric)
    band = junction_bar_rect(rect.x, rect.y, metric=metric)
    bar = rl.Rectangle(band.x, band.y, band.w, band.h)
    bg = _BG_IDLE if view.idle else _BG
    rl.draw_rectangle_rounded(bar, 0.16, 14, bg)
    rl.draw_rectangle_rounded_lines_ex(bar, 0.16, 14, 1.8, _BORDER)

    accent = _ACCENT.get(view.light, _MUTED)
    rail = rl.Rectangle(bar.x + 6, bar.y + 16, 5, bar.height - 32)
    rl.draw_rectangle_rounded(rail, 0.9, 6, accent)

    signal_x = bar.x + SIGNAL_PAD_X
    signal_y = bar.y + 14
    signal_h = bar.height - 28
    self._draw_signal(signal_x, signal_y, SIGNAL_W, signal_h, view.light)

    text_x = signal_x + SIGNAL_W + CONTENT_GAP
    text_right = bar.x + bar.width - CAPSULE_RIGHT_PAD
    capsules = self._metric_capsules(view)
    if capsules:
      text_right = bar.x + bar.width - CAPSULE_RIGHT_PAD - CAPSULE_W - 12
      self._draw_capsules(bar, capsules, view.light)
    inner_w = max(48.0, text_right - text_x)
    self._draw_copy(text_x, bar.y, inner_w, bar.height, view)

    if self._lane.show:
      lane_band = lane_guide_rect(rect.x, rect.y, metric=metric)
      self._draw_lane_guide(rl.Rectangle(lane_band.x, lane_band.y, lane_band.w, lane_band.h), self._lane)

    if self._egpu.show:
      egpu_band = egpu_status_rect(
        rect.x, rect.y, metric=metric, lane_guide=self._lane.show, compact=not bool(self._egpu.metrics),
      )
      self._draw_egpu(rl.Rectangle(egpu_band.x, egpu_band.y, egpu_band.w, egpu_band.h), self._egpu)

  def _draw_lane_guide(self, bar: rl.Rectangle, lane: LaneGuideView) -> None:
    rl.draw_rectangle_rounded(bar, 0.28, 12, _BG_LANE)
    rl.draw_rectangle_rounded_lines_ex(bar, 0.28, 12, 1.5, _BORDER_LANE)

    # Same left rail as junction bar
    wash = rl.Rectangle(bar.x + 6, bar.y + 10, 3, bar.height - 20)
    rl.draw_rectangle_rounded(wash, 0.9, 4, _LANE_ACCENT)

    # Badge centered in the traffic-signal column (SIGNAL_PAD_X + SIGNAL_W)
    signal_x = bar.x + SIGNAL_PAD_X
    cx = signal_x + SIGNAL_W / 2
    cy = bar.y + bar.height / 2
    rl.draw_circle_v(rl.Vector2(cx, cy), LANE_BADGE_W / 2, _LANE_BADGE_BG)
    rl.draw_ring(rl.Vector2(cx, cy), LANE_BADGE_W / 2 - 1.2, LANE_BADGE_W / 2, 0, 360, 36, _LANE_BADGE_RIM)
    self._draw_lane_arrow(cx, cy, lane.kind)

    label = "车道引导" if lane.kind in ("left", "right") else "转向提示"
    # Title x = junction headline ("暂无信号"); right pad = CAPSULE_RIGHT_PAD
    size = LANE_TEXT_SIZE
    lab_sz = measure_text_cached(self._font_head, label, size)
    txt_sz = measure_text_cached(self._font_head, lane.text, size)
    text_left = signal_x + SIGNAL_W + CONTENT_GAP
    text_right = bar.x + bar.width - CAPSULE_RIGHT_PAD
    mid = bar.y + bar.height / 2
    rl.draw_text_ex(
      self._font_head, label,
      rl.Vector2(text_left, mid - lab_sz.y / 2),
      size, 0, _LANE_TEXT,
    )
    rl.draw_text_ex(
      self._font_head, lane.text,
      rl.Vector2(text_right - txt_sz.x, mid - txt_sz.y / 2),
      size, 0, _LANE_TEXT,
    )

  def _draw_lane_arrow(self, cx: float, cy: float, kind: str) -> None:
    """Thin stroke-style chevron / turn — keeps the badge light."""
    c = _LANE_ACCENT
    thick = 3.0
    scale = LANE_BADGE_W / 40.0
    if kind in ("left", "right"):
      s = 1.0 if kind == "right" else -1.0
      x0, x1 = cx - s * 9.5 * scale, cx + s * 6.0 * scale
      rl.draw_line_ex(rl.Vector2(x0, cy), rl.Vector2(x1, cy), thick, c)
      tip = rl.Vector2(cx + s * 11.0 * scale, cy)
      rl.draw_line_ex(tip, rl.Vector2(cx + s * 1.8 * scale, cy - 9.0 * scale), thick, c)
      rl.draw_line_ex(tip, rl.Vector2(cx + s * 1.8 * scale, cy + 9.0 * scale), thick, c)
      return
    # Turn: classic ↰ / ↱ — stem rises from bottom, tip high
    s = 1.0 if kind == "turn_right" else -1.0
    stem_x = cx - s * 4.5 * scale
    arm_y = cy - 9.5 * scale
    rl.draw_line_ex(rl.Vector2(stem_x, cy + 10.5 * scale), rl.Vector2(stem_x, arm_y), thick, c)
    rl.draw_line_ex(rl.Vector2(stem_x, arm_y), rl.Vector2(cx + s * 1.8 * scale, arm_y), thick, c)
    tip = rl.Vector2(cx + s * 11.5 * scale, arm_y)
    rl.draw_triangle(
      tip,
      rl.Vector2(cx + s * 3.5 * scale, arm_y - 6.5 * scale),
      rl.Vector2(cx + s * 3.5 * scale, arm_y + 6.5 * scale),
      c,
    )

  def _draw_egpu(self, bar: rl.Rectangle, view: HudEgpuView) -> None:
    accent = _EGPU_ACCENT.get(view.severity, _MUTED)
    rl.draw_rectangle_rounded(bar, 0.14, 14, _BG_EGPU)
    rl.draw_rectangle_rounded_lines_ex(bar, 0.14, 14, 1.6, _BORDER_EGPU)
    wash = rl.Rectangle(bar.x + EGPU_RAIL_X, bar.y + 18, EGPU_RAIL_W, bar.height - 36)
    rl.draw_rectangle_rounded(wash, 0.9, 4, accent)

    pad_x = bar.x + EGPU_RAIL_X + EGPU_RAIL_W + EGPU_PAD
    pad_right = bar.x + bar.width - EGPU_PAD
    inner_w = max(48.0, pad_right - pad_x)
    header_h = 56.0 if view.metrics else bar.height - 36.0
    header_y = bar.y + 14.0 if view.metrics else bar.y + (bar.height - header_h) / 2

    u_sz = measure_text_cached(self._font_unit, "12V", EGPU_DC_UNIT)
    v_sz = measure_text_cached(self._font_head, view.dc_text, EGPU_DC_VALUE)
    pill_w = dc_pill_width(u_sz.x, v_sz.x)
    pill_h = 44.0
    pill = rl.Rectangle(pad_right - pill_w, header_y + (header_h - pill_h) / 2, pill_w, pill_h)
    self._draw_dc_pill(pill, view.dc_text, view.dc_kind, u_sz, v_sz)

    title_right = pill.x - EGPU_PAD
    title_w = max(48.0, title_right - pad_x)
    head_size = EGPU_HEAD_SIZE
    det_size = EGPU_DETAIL_SIZE
    head_sz = measure_text_cached(self._font_head, view.headline, head_size)
    det_sz = measure_text_cached(self._font_detail, view.detail, det_size) if view.detail else rl.Vector2(0, 0)
    while head_sz.x > title_w and head_size > 22:
      head_size -= 2
      head_sz = measure_text_cached(self._font_head, view.headline, head_size)
    while view.detail and det_sz.x > title_w and det_size > 16:
      det_size -= 2
      det_sz = measure_text_cached(self._font_detail, view.detail, det_size)

    if view.detail:
      gap = 4.0
      block = head_sz.y + gap + det_sz.y
      ty = header_y + (header_h - block) / 2
      rl.draw_text_ex(self._font_head, view.headline, rl.Vector2(pad_x, ty), head_size, 0, _EGPU_TEXT)
      rl.draw_text_ex(
        self._font_detail, view.detail,
        rl.Vector2(pad_x, ty + head_sz.y + gap), det_size, 0, _EGPU_DETAIL,
      )
    else:
      rl.draw_text_ex(
        self._font_head, view.headline,
        rl.Vector2(pad_x, header_y + (header_h - head_sz.y) / 2),
        head_size, 0, _EGPU_TEXT,
      )

    if not view.metrics:
      return

    tile_top = header_y + header_h + 8.0
    tile_bottom = bar.y + bar.height - 14.0
    self._draw_egpu_tiles(pad_x, tile_top, inner_w, tile_bottom - tile_top, view.metrics)

  def _draw_dc_pill(self, pill: rl.Rectangle, text: str, kind: str, u_sz, v_sz) -> None:
    bg, fg = _DC_PILL.get(kind, _DC_PILL["unknown"])
    rl.draw_rectangle_rounded(pill, 0.42, 10, bg)
    rl.draw_rectangle_rounded_lines_ex(pill, 0.42, 10, 1.2, rl.Color(fg.r, fg.g, fg.b, 70))
    content = u_sz.x + EGPU_DC_PILL_GAP + v_sz.x
    tx = pill.x + (pill.width - content) / 2
    mid = pill.y + pill.height / 2
    rl.draw_text_ex(
      self._font_unit, "12V",
      rl.Vector2(tx, mid - u_sz.y / 2 + 1), EGPU_DC_UNIT, 0, rl.Color(fg.r, fg.g, fg.b, 190),
    )
    rl.draw_text_ex(
      self._font_head, text,
      rl.Vector2(tx + u_sz.x + EGPU_DC_PILL_GAP, mid - v_sz.y / 2), EGPU_DC_VALUE, 0, fg,
    )

  def _draw_text_centered(self, font, text: str, cx: float, y: float, size: float, color) -> None:
    sz = measure_text_cached(font, text, size)
    rl.draw_text_ex(font, text, rl.Vector2(round(cx - sz.x / 2), round(y)), size, 0, color)

  def _draw_egpu_tiles(self, x: float, y: float, width: float, height: float,
                       metrics) -> None:
    n = len(metrics)
    cols = 2 if n == 4 else 3
    gap = 8.0
    rows = (n + cols - 1) // cols
    tile_w = (width - gap * (cols - 1)) / cols
    tile_h = (height - gap * (rows - 1)) / rows if rows else height
    for i, metric in enumerate(metrics):
      r, c = i // cols, i % cols
      span = 2 if (i == n - 1 and c == 1 and n % cols) else 1
      box = rl.Rectangle(x + c * (tile_w + gap), y + r * (tile_h + gap),
                         tile_w * span + gap * (span - 1), tile_h)
      rl.draw_rectangle_rounded(box, 0.22, 10, _EGPU_TILE_BG)
      rl.draw_rectangle_rounded_lines_ex(box, 0.22, 10, 1.2, _EGPU_TILE_EDGE)
      val_size = EGPU_TILE_VALUE
      unit_size = EGPU_TILE_UNIT
      val_sz = measure_text_cached(self._font_num, metric.value, val_size)
      while val_sz.x > box.width - 16 and val_size > 18:
        val_size -= 2
        val_sz = measure_text_cached(self._font_num, metric.value, val_size)
      unit_sz = measure_text_cached(self._font_unit, metric.unit, unit_size)
      line_gap = 2.0
      stack = val_sz.y + line_gap + unit_sz.y
      cx = box.x + box.width * 0.5
      text_y = box.y + (box.height - stack) * 0.5
      self._draw_text_centered(self._font_num, metric.value, cx, text_y, val_size, _EGPU_TILE_VALUE)
      self._draw_text_centered(
        self._font_unit, metric.unit, cx, text_y + val_sz.y + line_gap, unit_size, _EGPU_TILE_UNIT,
      )

  def _metric_capsules(self, view: JunctionView) -> list[tuple[str, str]]:
    if view.idle or not view.has_metric_capsules:
      return []
    out: list[tuple[str, str]] = []
    if view.dist_m >= 1.0:
      out.append((str(int(round(view.dist_m))), METERS))
    if view.remain_s >= 1.0:
      out.append((str(int(view.remain_s)), SECONDS))
    return out

  def _draw_capsules(self, bar: rl.Rectangle, capsules: list[tuple[str, str]], light: str) -> None:
    n = len(capsules)
    total_h = n * CAPSULE_H + max(0, n - 1) * CAPSULE_GAP
    y0 = bar.y + (bar.height - total_h) / 2
    x = bar.x + bar.width - CAPSULE_RIGHT_PAD - CAPSULE_W
    accent = _ACCENT.get(light, _MUTED)
    for i, (num, unit) in enumerate(capsules):
      y = y0 + i * (CAPSULE_H + CAPSULE_GAP)
      box = rl.Rectangle(x, y, CAPSULE_W, CAPSULE_H)
      rl.draw_rectangle_rounded(box, 0.42, 10, _CAPSULE_BG)
      rl.draw_rectangle_rounded_lines_ex(box, 0.42, 10, 1.4, _CAPSULE_EDGE)
      pip = rl.Rectangle(x + 8, y + 10, 4, CAPSULE_H - 20)
      rl.draw_rectangle_rounded(pip, 0.9, 4, accent)

      num_size = 28
      unit_size = 20
      num_sz = measure_text_cached(self._font_head, num, num_size)
      unit_sz = measure_text_cached(self._font_detail, unit, unit_size)
      content_w = num_sz.x + 6 + unit_sz.x
      tx = x + 18 + (CAPSULE_W - 26 - content_w) / 2
      mid = y + CAPSULE_H / 2
      rl.draw_text_ex(self._font_head, num, rl.Vector2(tx, mid - num_sz.y / 2), num_size, 0, _CAPSULE_NUM)
      rl.draw_text_ex(
        self._font_detail, unit,
        rl.Vector2(tx + num_sz.x + 6, mid - unit_sz.y / 2 + 1),
        unit_size, 0, _CAPSULE_UNIT,
      )

  def _draw_copy(self, x: float, bar_y: float, inner_w: float, bar_h: float, view: JunctionView) -> None:
    headline = view.headline
    detail = view.detail
    head_color = _IDLE_HEAD if view.idle else _HEAD
    has_caps = bool(self._metric_capsules(view))
    text_right = x + inner_w

    if detail and not has_caps:
      head_size = 44 if len(headline) <= 4 else 38
      det_size = 34 if head_size >= 44 else 30
      head_sz = measure_text_cached(self._font_head, headline, head_size)
      det_sz = measure_text_cached(self._font_detail, detail, det_size)
      gap = 16.0
      while head_sz.x + det_sz.x + gap > inner_w and head_size > 30:
        head_size -= 2
        det_size = max(26, det_size - 2)
        head_sz = measure_text_cached(self._font_head, headline, head_size)
        det_sz = measure_text_cached(self._font_detail, detail, det_size)
      mid = bar_y + bar_h / 2
      rl.draw_text_ex(
        self._font_head, headline,
        rl.Vector2(x, mid - head_sz.y / 2),
        head_size, 0, head_color,
      )
      rl.draw_text_ex(
        self._font_detail, detail,
        rl.Vector2(text_right - det_sz.x, mid - det_sz.y / 2),
        det_size, 0, _DETAIL,
      )
    elif has_caps:
      head_size = 52 if len(headline) <= 4 else 44
      head_sz = measure_text_cached(self._font_head, headline, head_size)
      rl.draw_text_ex(
        self._font_head, headline,
        rl.Vector2(x, bar_y + (bar_h - head_sz.y) / 2),
        head_size, 0, head_color,
      )
    else:
      head_size = 50
      head_sz = measure_text_cached(self._font_head, headline, head_size)
      rl.draw_text_ex(
        self._font_head, headline,
        rl.Vector2(x, bar_y + (bar_h - head_sz.y) / 2),
        head_size, 0, head_color,
      )

  def _draw_signal(self, x: float, y: float, w: float, h: float, light: str) -> None:
    well = rl.Rectangle(x, y, w, h)
    rl.draw_rectangle_rounded(well, 0.30, 10, _SIGNAL_WELL)
    rl.draw_rectangle_rounded_lines_ex(well, 0.30, 10, 1.5, _SIGNAL_RIM)

    order = ("red", "yellow", "green")
    r = 12.0
    gap = 11.0
    total = 2 * r * 3 + gap * 2
    cx = x + w / 2
    y0 = y + (h - total) / 2 + r
    for i, name in enumerate(order):
      cy = y0 + i * (2 * r + gap)
      on = light == name
      fill = _LAMP_ON[name] if on else _LAMP_OFF[name]
      center = rl.Vector2(cx, cy)
      rl.draw_circle_v(center, r + 2.2, rl.Color(0, 0, 0, 110))
      rl.draw_circle_v(center, r, fill)
      if on:
        rl.draw_circle_v(center, r + 6.0, rl.Color(fill.r, fill.g, fill.b, 36))
        rl.draw_ring(center, r - 2.6, r + 1.0, 0, 360, 28, rl.Color(255, 255, 255, 90))
      else:
        rl.draw_ring(center, r - 1.6, r, 0, 360, 20, rl.Color(255, 255, 255, 22))
