"""Shared junction / traffic-light HUD state for camera overlay."""

from __future__ import annotations

from dataclasses import dataclass

GREEN_FLASH_S = 1.5
_LIGHTS = ("red", "yellow", "green")


@dataclass(frozen=True)
class JunctionHudSnapshot:
  active: bool
  light: str  # "red", "yellow", "green", "none", or "miss"
  dist_m: float
  remain_s: float

  @property
  def headline(self) -> str:
    if self.light == "red":
      return "红灯"
    if self.light == "yellow":
      return "黄灯"
    if self.light == "green":
      return "绿灯"
    if self.light == "miss":
      return "信号灯"
    return "前方停车"

  @property
  def detail(self) -> str:
    if self.caption:
      return self.caption
    return "  ".join(self.metrics)

  @property
  def caption(self) -> str:
    if self.light == "green":
      return "可通行"
    if self.light == "miss":
      return "未检测到"
    if self.light == "none":
      return "注意前方"
    return ""

  @property
  def metrics(self) -> tuple[str, ...]:
    if self.light in ("green", "miss", "none"):
      return ()
    parts: list[str] = []
    if self.dist_m >= 1.0:
      parts.append(f"{int(round(self.dist_m))}米")
    if self.remain_s >= 1.0:
      parts.append(f"{int(self.remain_s)}秒")
    return tuple(parts)


@dataclass
class GreenFlashState:
  pending: bool = False
  green_until: float = 0.0
  last_light: str = "none"


def light_token(raw: str | None) -> str:
  token = str(raw or "none").strip().lower()
  return token if token in _LIGHTS else "none"


def read_junction_snapshot(sm, *, engaged: bool) -> JunctionHudSnapshot:
  inactive = JunctionHudSnapshot(False, "none", 0.0, 0.0)
  if not engaged:
    return inactive
  try:
    if not bool(sm["iqPlan"].e2eAlerts.junctionStop):
      return inactive
  except Exception:
    return inactive

  light = "none"
  dist_m = 0.0
  remain_s = 0.0
  try:
    nav = sm["iqNavState"]
    token = light_token(getattr(nav, "trafficLight", "none"))
    if token in ("red", "yellow", "green"):
      light = token
    dist_m = float(getattr(nav, "trafficLightDistM", 0.0) or 0.0)
    remain_s = float(getattr(nav, "trafficLightRemainS", 0.0) or 0.0)
  except Exception:
    pass
  return JunctionHudSnapshot(True, light, dist_m, remain_s)


def merge_green_flash(
  base: JunctionHudSnapshot,
  *,
  engaged: bool,
  has_lead: bool,
  light: str,
  dist_m: float,
  remain_s: float,
  state: GreenFlashState,
  now: float,
) -> JunctionHudSnapshot:
  """Brief green bar after a junction stop cue; does not affect junctionStop."""
  if base.active:
    state.pending = True

  token = light_token(light)
  if token == "green" and state.last_light != "green" and state.pending and engaged:
    state.green_until = now + GREEN_FLASH_S
  state.last_light = token

  if (
    engaged
    and not has_lead
    and token == "green"
    and state.pending
    and now < state.green_until
  ):
    return JunctionHudSnapshot(True, "green", dist_m, remain_s)

  if state.green_until > 0.0 and now >= state.green_until:
    state.pending = False
    state.green_until = 0.0

  return base


def junction_accent_rgb(light: str) -> tuple[int, int, int]:
  if light == "red":
    return (210, 48, 52)
  if light == "yellow":
    return (214, 168, 24)
  if light == "green":
    return (72, 196, 112)
  return (176, 186, 198)
