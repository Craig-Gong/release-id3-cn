"""Carrot / amapauto flat JSON → NavSnapshot fields. Wire names from PROTOCOL.md."""
from __future__ import annotations

import math
from typing import Any

from openpilot.sunnypilot.nav.hud_copy import (
  ARRIVE_SOON, EXIT_AHEAD, KILOMETERS, LANE_LEFT, LANE_RIGHT, METERS,
  STRAIGHT_AHEAD, STRAIGHT_LANE, TURN_LEFT, TURN_RIGHT,
)
from openpilot.sunnypilot.nav.snapshot import NavSnapshot

_TURN_LEFT = {1, 12, 16, 17, 18}
_TURN_RIGHT = {2, 13, 19, 20, 21}
_LC_LEFT = {3, 7, 9, 22, 23}
_LC_RIGHT = {4, 8, 10, 24, 25}
_ROUNDABOUT = {5, 14, 15}
_EXIT = {6, 11}

_RED_LIGHT_ACCEL = -2.0
_RED_LIGHT_DECEL = 2.0
# Floor when TrafficStopOffset is 0 / unset. Live slider applies to IQ-link red
# (amap light distance ≠ stop line). Cap at 6 m so a vision-only 8–10 m CD210
# setting does not make head-car nav stops absurdly early.
NAV_STOP_MARGIN_M = 3.0
NAV_STOP_MARGIN_MAX_M = 6.0
_offset_cache = NAV_STOP_MARGIN_M
_offset_n = 0
_YELLOW_STOP_DIST_M = 30.0
LIGHT_TURN_WINDOW_M = 150.0
# Toast / send_turn / nav-led longitudinal prep.
TURN_DESIRE_WINDOW_M = 150.0
# Lateral desire + turn-in: start before the corner so lead/desire is on
# when "快到路口". Keep below toast 150 m so the first rising edge is not
# spent at 150 m; desire_helper re-pulses harder in the last ~50 m.
NAV_LATERAL_TURN_M = 120.0
NEAR_DEST_REMAIN_M = 150.0
_NAV_RED_HARD_A = -3.5
_NAV_RED_HOLD_A = -1.5
# Final meters before the intended stop: force shouldStop even if still rolling.
_NAV_RED_NEAR_M = 3.5


def _f(data: dict[str, Any], key: str, default: float = 0.0) -> float:
  try:
    v = data.get(key, default)
    if v is None:
      return default
    return float(v)
  except (TypeError, ValueError):
    return default


def _s(data: dict[str, Any], key: str, default: str = "") -> str:
  v = data.get(key, default)
  return "" if v is None else str(v)


def flatten_payload(payload: dict[str, Any]) -> dict[str, Any]:
  if not isinstance(payload, dict):
    return {}
  if isinstance(payload.get("rgdata"), dict):
    return dict(payload["rgdata"])
  return dict(payload)


def _turn_bucket(turn_type: int) -> str:
  if turn_type in _TURN_LEFT:
    return "turn_left"
  if turn_type in _TURN_RIGHT:
    return "turn_right"
  if turn_type in _LC_LEFT:
    return "lc_left"
  if turn_type in _LC_RIGHT:
    return "lc_right"
  if turn_type in _ROUNDABOUT:
    return "roundabout"
  if turn_type in _EXIT:
    return "exit"
  return "none"


def _dir_from_bucket(bucket: str) -> str:
  if "left" in bucket:
    return "left"
  if "right" in bucket:
    return "right"
  return "none"


def approach_speed_ms(dist_m: float, decel: float, cap_ms: float = 0.0) -> float:
  d = max(0.0, float(dist_m))
  a = max(0.01, float(decel))
  v = math.sqrt(2.0 * a * d)
  if cap_ms > 0.0:
    return min(v, cap_ms)
  return v


def nav_stop_margin_m(offset_m: float | None = None) -> float:
  """Meters short of amap trafficLightDistM for IQ-link red.

  Uses TrafficStopOffset when > 0, floored at 3 m and capped at 6 m.
  Offset 0 / unset keeps the 3 m default.
  """
  if offset_m is None:
    return NAV_STOP_MARGIN_M
  try:
    o = float(offset_m)
  except (TypeError, ValueError):
    return NAV_STOP_MARGIN_M
  if o <= 0.0:
    return NAV_STOP_MARGIN_M
  return min(max(o, NAV_STOP_MARGIN_M), NAV_STOP_MARGIN_MAX_M)


def nav_red_speed_ms(light_dist: float, road_ms: float, margin: float) -> float:
  """Target speed for a nav red. Already at/past the intended stop → 0, not a 0.5 m crawl."""
  d = float(light_dist or 0.0)
  m = float(margin)
  if d <= 0.0 or d <= m:
    return 0.0
  v = approach_speed_ms(d - m, _RED_LIGHT_DECEL, cap_ms=road_ms)
  return 0.0 if v <= 0.05 else v


def nav_red_accel_cap(v_ego: float, light_dist: float, margin: float,
                      accel_target: float = _RED_LIGHT_ACCEL) -> float:
  """Kinematic accel ceiling toward (light_dist - margin). Always on for nav red.

  Far + already slow → near 0 (no absurd early -2 from 100 m). On the 2 m/s²
  approach curve → about -2. Above the curve / inside the margin → harder.
  """
  remaining = float(light_dist or 0.0) - float(margin)
  v = max(0.0, float(v_ego))
  if remaining <= 0.0:
    return min(float(accel_target), _NAV_RED_HOLD_A)
  a_kin = -(v * v) / (2.0 * max(remaining, 0.3))
  a_kin = max(a_kin, _NAV_RED_HARD_A)
  if a_kin >= -0.05:
    return 0.0
  return a_kin


def nav_red_force_stop(v_ego: float, light_dist: float, margin: float) -> bool:
  """True when already at/past the intended stop or in the final hold zone."""
  d = float(light_dist or 0.0)
  remaining = d - float(margin)
  if d <= 0.0 or remaining <= 0.0:
    return True
  if remaining <= _NAV_RED_NEAR_M:
    return True
  return nav_red_speed_ms(d, 0.0, margin) <= 0.5 and float(v_ego) <= 1.5


def traffic_stop_margin_m() -> float:
  """Cached live TrafficStopOffset → nav margin. Safe for iqlinkd + planner."""
  global _offset_cache, _offset_n
  _offset_n += 1
  if _offset_n % 15 != 1:
    return _offset_cache
  try:
    from openpilot.common.params import Params
    from openpilot.sunnypilot.selfdrive.controls.lib.helpers.traffic_stop_offset import (
      DEFAULT_OFFSET_M, TRAFFIC_STOP_OFFSET_PARAM, _sanitize_offset_m,
    )
    raw = Params().get(TRAFFIC_STOP_OFFSET_PARAM, return_default=True)
    offset = _sanitize_offset_m(raw if raw is not None else DEFAULT_OFFSET_M)
    _offset_cache = nav_stop_margin_m(offset)
  except Exception:
    pass
  return _offset_cache


def parse_carrot(payload: dict[str, Any], *, now: float, link_ok: bool,
                 link_state: int, enabled: bool) -> NavSnapshot | None:
  data = flatten_payload(payload)
  if "nRoadLimitSpeed" not in data:
    return None

  road_kph = _f(data, "nRoadLimitSpeed")
  road_ms = max(road_kph, 0.0) / 3.6
  turn_type = int(_f(data, "nTBTTurnType"))
  turn_dist = _f(data, "nTBTDist")
  bucket = _turn_bucket(turn_type)
  direction = _dir_from_bucket(bucket)
  is_lc = bucket.startswith("lc")
  is_turn = bucket.startswith("turn")
  near_turn = 0.0 < turn_dist <= TURN_DESIRE_WINDOW_M
  lane_rec = _s(data, "laneRecommend", "none").strip().lower() or "none"

  send_turn = bool(is_turn and near_turn)
  # Urban: Gaode often labels intersections as lc_*. Promote toast / desire
  # when near; maneuver stays fork so HUD / soft-curve still know it is LC.
  if is_lc and near_turn and lane_rec != "straight":
    send_turn = True

  light = _s(data, "trafficLight", "none").strip().lower() or "none"
  light_dist = _f(data, "trafficLightDistM")
  remain_s = int(_f(data, "trafficLightRemainS"))
  remain_go = "trafficLightRemainS" in data and remain_s == 1
  right_turn_pending = bucket == "turn_right" and 0.0 < turn_dist <= LIGHT_TURN_WINDOW_M

  stop_for_light = False
  speed_target = road_ms
  accel_target = 0.0
  if not right_turn_pending:
    if light == "red":
      stop_for_light = not remain_go
    elif light == "yellow" and 0.0 < light_dist <= _YELLOW_STOP_DIST_M:
      stop_for_light = True
  if stop_for_light:
    margin = traffic_stop_margin_m()
    speed_target = nav_red_speed_ms(light_dist, road_ms, margin)
    accel_target = _RED_LIGHT_ACCEL

  maneuver = "none"
  if bucket.startswith("turn"):
    maneuver = "turn"
  elif bucket.startswith("lc"):
    maneuver = "fork"
  elif bucket == "exit":
    maneuver = "exit"
  elif bucket == "roundabout":
    maneuver = "roundabout"

  go_dist = _f(data, "nGoPosDist")
  go_time = _f(data, "nGoPosTime")
  if 0.0 < go_dist <= NEAR_DEST_REMAIN_M:
    send_turn = False
    maneuver = "arrive"

  # Partner maps NEXT_ROAD_NAME → szTBTMainText (enter); cur road → szPosRoadName.
  enter_road = (_s(data, "szTBTMainText") or _s(data, "szNearDirName")).strip()
  goal_name = _s(data, "szGoalName").strip()

  return NavSnapshot(
    ts=float(now),
    link_ok=bool(link_ok),
    link_state=int(link_state),
    iqlink_enabled=bool(enabled),
    traffic_light=light,
    dist_m=float(light_dist or 0.0),
    remain_s=float(max(remain_s, 0)),
    remain_go=bool(remain_go),
    stop_for_light=bool(stop_for_light),
    speed_target=float(speed_target),
    accel_target=float(accel_target),
    lane_recommend=lane_rec,
    maneuver=maneuver,
    maneuver_dir=direction,
    tbt_dist=float(turn_dist),
    road_limit_kph=float(road_kph),
    send_turn=bool(send_turn),
    enter_road=enter_road[:40],
    go_dist_m=float(max(go_dist, 0.0)),
    go_time_s=float(max(go_time, 0.0)),
    goal_name=goal_name[:40],
  )


def format_tbt_capsule(dist_m: float) -> tuple[str, str] | None:
  """Distance capsule for hero: meters or km."""
  d = float(dist_m or 0.0)
  if d < 1.0:
    return None
  if d >= 1000.0:
    return (f"{d / 1000.0:.1f}".rstrip("0").rstrip("."), KILOMETERS)
  return (str(int(round(d))), METERS)


def format_remain_km(dist_m: float) -> str:
  d = float(dist_m or 0.0)
  if d < 1.0:
    return ""
  if d >= 1000.0:
    return f"{d / 1000.0:.1f}".rstrip("0").rstrip(".")
  return f"{d / 1000.0:.2f}".rstrip("0").rstrip(".")


def format_remain_min(time_s: float) -> str:
  t = float(time_s or 0.0)
  if t < 1.0:
    return ""
  return str(max(1, int(round(t / 60.0))))


def lane_hint(snap: NavSnapshot) -> str:
  rec = (snap.lane_recommend or "none").lower()
  if rec == "left":
    return LANE_LEFT
  if rec == "right":
    return LANE_RIGHT
  if snap.send_turn and snap.maneuver_dir == "left":
    return TURN_LEFT
  if snap.send_turn and snap.maneuver_dir == "right":
    return TURN_RIGHT
  maneuver = (snap.maneuver or "none").lower()
  if maneuver == "exit" and float(snap.tbt_dist or 0.0) > 0.0:
    return EXIT_AHEAD
  if maneuver == "arrive":
    return ARRIVE_SOON
  if rec == "straight":
    return STRAIGHT_LANE if float(snap.tbt_dist or 0.0) < 1.0 else STRAIGHT_AHEAD
  # Far TBT still ahead (toast window) → show 前方直行 with distance when no turn yet.
  if float(snap.tbt_dist or 0.0) >= 1.0 and maneuver in ("none", "fork") and not snap.send_turn:
    if (snap.maneuver_dir or "none") == "none" and rec in ("none", "straight"):
      return STRAIGHT_AHEAD
  return ""
