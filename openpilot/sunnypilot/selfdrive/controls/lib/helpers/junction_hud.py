"""Junction / traffic-stop HUD gate used by the C3XL overlay."""
from __future__ import annotations

from dataclasses import dataclass

from openpilot.sunnypilot.nav.hud_copy import (
  FOLLOW_LEAD, GO_AHEAD, NAV_EMPTY, NO_SIGNAL, STOP_AHEAD, STOP_GREEN, STOP_RED,
  STOP_YELLOW, STRAIGHT_AHEAD, STRAIGHT_LANE, WAIT_DETECT, WAIT_PAIR, WATCH_AHEAD,
)
from openpilot.sunnypilot.nav.protocol import format_tbt_capsule, lane_hint
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
  dist_m: float = 0.0
  remain_s: float = 0.0

  @property
  def has_metric_capsules(self) -> bool:
    return self.dist_m >= 1.0 or self.remain_s >= 1.0


@dataclass(frozen=True)
class LaneGuideView:
  show: bool
  text: str
  kind: str  # left | right | turn_left | turn_right | straight | exit | arrive | none
  empty: bool = False
  enter: str = ""
  capsule: tuple[str, str] | None = None


def _stop_headline(light: str) -> str:
  if light == "red":
    return STOP_RED
  if light == "yellow":
    return STOP_YELLOW
  if light == "green":
    return STOP_GREEN
  return STOP_AHEAD


def _lane_kind(snap: NavSnapshot, text: str) -> str:
  rec = (snap.lane_recommend or "none").lower()
  if rec == "left":
    return "left"
  if rec == "right":
    return "right"
  if snap.send_turn and snap.maneuver_dir == "left":
    return "turn_left"
  if snap.send_turn and snap.maneuver_dir == "right":
    return "turn_right"
  maneuver = (snap.maneuver or "none").lower()
  if maneuver == "exit":
    return "exit"
  if maneuver == "arrive":
    return "arrive"
  if text in (STRAIGHT_AHEAD, STRAIGHT_LANE) or rec == "straight":
    return "straight"
  return "none"


def build_lane_guide_view(*, onroad: bool, snap: NavSnapshot) -> LaneGuideView:
  """Persistent nav card: always shown onroad (empty → 暂无导航推送)."""
  if not onroad:
    return LaneGuideView(False, "", "none", True)

  text = lane_hint(snap)
  kind = _lane_kind(snap, text)
  enter = (snap.enter_road or "").strip()
  # Avoid duplicating maneuver words as "进入 · 右转"
  if enter in ("左转", "右转", "调头", "直行", "测试右转", "前方红灯"):
    enter = ""

  capsule = None
  if float(snap.tbt_dist or 0.0) >= 1.0 and kind not in ("none", "left", "right"):
    capsule = format_tbt_capsule(snap.tbt_dist)

  empty = not text and not enter
  if empty:
    return LaneGuideView(True, NAV_EMPTY, "none", True)
  if not text:
    text = NAV_EMPTY
    kind = "none"
  return LaneGuideView(True, text, kind, False, enter, capsule)


def build_junction_view(*, onroad: bool, has_lead: bool, model_stop: bool,
                        standstill_hold: bool, snap: NavSnapshot,
                        green_flash: bool) -> JunctionView:
  if not onroad:
    return JunctionView(False, "none", "", "", False, False)

  light = snap.light_token if snap.iqlink_enabled else "none"
  nav_red = bool(snap.iqlink_enabled and snap.stop_for_light)
  stopping = junction_stop_active(
    has_lead=has_lead, nav_red=nav_red, model_stop=model_stop,
    standstill_hold=standstill_hold, light=light,
  )

  # Lane recommend lives in LaneGuideView — never as junction detail.
  if has_lead and (nav_red or model_stop or standstill_hold):
    return JunctionView(True, "none", FOLLOW_LEAD, "", True, True)

  if green_flash and not has_lead:
    return JunctionView(True, "green", STOP_GREEN, GO_AHEAD, False, False)

  if stopping:
    headline = _stop_headline(light)
    dist = float(snap.dist_m or 0.0)
    remain = float(snap.remain_s or 0.0)
    if light == "green":
      return JunctionView(True, light, headline, GO_AHEAD, False, False)
    detail = ""
    if dist < 1.0 and remain < 1.0 and light == "none":
      detail = WATCH_AHEAD
    return JunctionView(True, light, headline, detail, False, False, dist, remain)

  if snap.iqlink_enabled and snap.link_ok:
    return JunctionView(True, "none", NO_SIGNAL, WAIT_DETECT, True, False)
  if snap.iqlink_enabled:
    return JunctionView(True, "none", WAIT_PAIR, "IQ-link", True, False)
  return JunctionView(True, "none", NO_SIGNAL, "", True, False)


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
