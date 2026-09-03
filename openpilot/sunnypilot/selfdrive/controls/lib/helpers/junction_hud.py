"""Junction / traffic-stop HUD gate used by the C3XL overlay."""
from __future__ import annotations

from dataclasses import dataclass

from openpilot.sunnypilot.nav.protocol import lane_hint
from openpilot.sunnypilot.nav.snapshot import NavSnapshot

GREEN_FLASH_S = 1.5
_LIGHTS = ("red", "yellow", "green")


def light_token(raw: str | None) -> str:
  token = str(raw or "none").strip().lower()
  return token if token in _LIGHTS else "none"


def junction_stop_active(*, has_lead: bool, nav_red: bool, model_stop: bool,
                         standstill_hold: bool, light: str | None = None) -> bool:
  if has_lead:
    return False
  if light_token(light) == "green":
    return False
  return bool(nav_red or model_stop or standstill_hold)


@dataclass(frozen=True)
class JunctionView:
  show: bool
  light: str
  headline: str
  detail: str
  idle: bool
  following: bool


def _stop_headline(light: str) -> str:
  if light == "red":
    return "红灯"
  if light == "yellow":
    return "黄灯"
  if light == "green":
    return "绿灯"
  return "前方停车"


def _stop_detail(light: str, dist_m: float, remain_s: float) -> str:
  if light == "green":
    return "可通行"
  parts: list[str] = []
  if dist_m >= 1.0:
    parts.append(f"{int(round(dist_m))} 米")
  if remain_s >= 1.0:
    parts.append(f"{int(remain_s)} 秒")
  return " · ".join(parts)


def build_junction_view(*, engaged: bool, has_lead: bool, model_stop: bool,
                        standstill_hold: bool, snap: NavSnapshot,
                        green_flash: bool) -> JunctionView:
  if not engaged:
    return JunctionView(False, "none", "", "", False, False)

  light = snap.light_token if snap.iqlink_enabled else "none"
  nav_red = bool(snap.iqlink_enabled and snap.stop_for_light)
  stopping = junction_stop_active(
    has_lead=has_lead, nav_red=nav_red, model_stop=model_stop,
    standstill_hold=standstill_hold, light=light,
  )

  if has_lead and (nav_red or model_stop or standstill_hold):
    hint = lane_hint(snap)
    return JunctionView(True, "none", "跟前车", hint, True, True)

  if green_flash and not has_lead:
    return JunctionView(True, "green", "绿灯", "可通行", False, False)

  if stopping:
    headline = _stop_headline(light)
    detail = _stop_detail(light, snap.dist_m, snap.remain_s)
    if not detail and light == "none":
      detail = "注意前方"
    return JunctionView(True, light, headline, detail, False, False)

  hint = lane_hint(snap)
  if snap.iqlink_enabled and snap.link_ok:
    idle_h = "暂无信号"
    idle_d = hint or "等待识别"
  elif snap.iqlink_enabled:
    idle_h = "等待配对"
    idle_d = "IQ-link"
  else:
    idle_h = "暂无信号"
    idle_d = hint
  return JunctionView(True, "none", idle_h, idle_d, True, False)


@dataclass
class GreenFlashState:
  pending: bool = False
  green_until: float = 0.0
  last_light: str = "none"

  def update(self, *, stopping: bool, light: str, engaged: bool, now: float) -> bool:
    if stopping:
      self.pending = True
    token = light_token(light)
    if token == "green" and self.last_light != "green" and self.pending and engaged:
      self.green_until = now + GREEN_FLASH_S
    self.last_light = token
    flashing = bool(engaged and token == "green" and self.pending and now < self.green_until)
    if self.green_until > 0.0 and now >= self.green_until:
      self.pending = False
      self.green_until = 0.0
    return flashing
