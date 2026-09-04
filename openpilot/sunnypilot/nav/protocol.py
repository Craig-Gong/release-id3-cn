"""Carrot / amapauto flat JSON → NavSnapshot fields. Wire names from PROTOCOL.md."""
from __future__ import annotations

import math
from typing import Any

from openpilot.sunnypilot.nav.hud_copy import LANE_LEFT, LANE_RIGHT, TURN_LEFT, TURN_RIGHT
from openpilot.sunnypilot.nav.snapshot import NavSnapshot

_TURN_LEFT = {1, 12, 16, 17, 18}
_TURN_RIGHT = {2, 13, 19, 20, 21}
_LC_LEFT = {3, 7, 9, 22, 23}
_LC_RIGHT = {4, 8, 10, 24, 25}
_ROUNDABOUT = {5, 14, 15}
_EXIT = {6, 11}

_RED_LIGHT_ACCEL = -2.0
_RED_LIGHT_DECEL = 2.0
_STOP_LINE_EARLY_M = 0.0
_YELLOW_STOP_DIST_M = 30.0
LIGHT_TURN_WINDOW_M = 150.0
TURN_DESIRE_WINDOW_M = 150.0


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
    if light_dist > 0.0:
      speed_target = approach_speed_ms(light_dist + _STOP_LINE_EARLY_M, _RED_LIGHT_DECEL, cap_ms=road_ms)
      if speed_target <= 0.05:
        speed_target = 0.0
    else:
      speed_target = 0.0
    accel_target = _RED_LIGHT_ACCEL

  maneuver = "none"
  if send_turn:
    maneuver = "turn"
  elif bucket.startswith("lc"):
    maneuver = "fork"
  elif bucket == "exit":
    maneuver = "exit"

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
  )


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
  return ""
