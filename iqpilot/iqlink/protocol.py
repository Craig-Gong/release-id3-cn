"""Map CP搭子 / Carrot flat nav JSON → iqNavState field dict.

Called by: iqpilot/iqlink/bridge.py, iqpilot/iqlink/tests/test_protocol.py
"""

from __future__ import annotations

from typing import Any

from iqpilot.selfdrive.controls.lib.helpers.nav_decel import approach_speed_ms

from . import AGGRESSIVE_LC_DISTANCE_M

_TURN_LEFT = {1, 12, 16, 17, 18}
_TURN_RIGHT = {2, 13, 19, 20, 21}
_LC_LEFT = {3, 7, 9, 22, 23}
_LC_RIGHT = {4, 8, 10, 24, 25}
_ROUNDABOUT = {5, 14, 15}
_EXIT = {6, 11}

# Strong decelerate for nav red-light stop (m/s^2).
_RED_LIGHT_ACCEL = -2.0
_RED_LIGHT_DECEL_MS2 = 2.0
# Roadtest: nav red stop landed ~2 m early of the stop line.
_STOP_LINE_EARLY_COMP_M = 2.0
_YELLOW_STOP_DIST_M = 30.0
# China RTOR / left-arrow: only treat the TBT turn as "at this light" inside this window.
LIGHT_TURN_WINDOW_M = 150.0
# Nav turn desire + "Navigation: Turning Left/Right" (aligned with LIGHT_TURN_WINDOW_M).
TURN_DESIRE_WINDOW_M = 150.0
# Arrive: stop lateral desire only; keep limit / lights / snapshot (R1).
NEAR_DEST_REMAIN_M = 150.0


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


def _kph_to_ms(kph: float) -> float:
  return max(kph, 0.0) / 3.6


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


def _maneuver_type(bucket: str) -> str:
  if bucket.startswith("turn"):
    return "turn"
  if bucket.startswith("lc"):
    return "fork"
  if bucket == "exit":
    return "exit"
  if bucket == "roundabout":
    return "roundabout"
  if bucket == "arrive":
    return "arrive"
  return "none"


def nav_turn_pending(nav, *, side: str, window_m: float = LIGHT_TURN_WINDOW_M) -> bool:
  """True when APK TBT is a same-side turn at this light (not a distant later turn)."""
  if nav is None or side not in ("left", "right"):
    return False
  if isinstance(nav, dict):
    if str(nav.get("nextManeuverType") or "") != "turn":
      return False
    if str(nav.get("nextManeuverDirection") or "") != side:
      return False
    dist = float(nav.get("nextManeuverDistance") or 0.0)
    return 0.0 < dist <= float(window_m)
  mtype = getattr(nav, "nextManeuverType", None)
  mtype_name = str(getattr(mtype, "name", None) or mtype or "").lower()
  if mtype_name != "turn":
    return False
  direction = getattr(nav, "nextManeuverDirection", None)
  dir_name = str(getattr(direction, "name", None) or direction or "").lower()
  if side not in dir_name:
    return False
  dist = float(getattr(nav, "nextManeuverDistance", 0.0) or 0.0)
  return 0.0 < dist <= float(window_m)


def _dir_from_bucket(bucket: str) -> str:
  if "left" in bucket:
    return "left"
  if "right" in bucket:
    return "right"
  return "none"


def _red_light_approach_ms(light_dist_m: float, road_limit_ms: float) -> float:
  """Distance-based red stop cap; +2 m compensates measured early stop."""
  d = max(0.0, float(light_dist_m) + _STOP_LINE_EARLY_COMP_M)
  cap = approach_speed_ms(d, _RED_LIGHT_DECEL_MS2, cap_ms=road_limit_ms if road_limit_ms > 0 else 0.0)
  return 0.0 if cap <= 0.05 else cap


def flatten_payload(payload: dict[str, Any]) -> dict[str, Any]:
  if not isinstance(payload, dict):
    return {}
  if isinstance(payload.get("rgdata"), dict):
    return dict(payload["rgdata"])
  return dict(payload)


def is_nav_heartbeat(data: dict[str, Any]) -> bool:
  return "nRoadLimitSpeed" in data


def map_carrot_to_nav_fields(
  data: dict[str, Any],
  *,
  aggressive_lc: bool = True,
  command_index: int = 0,
  vision_stop: bool = False,
) -> dict[str, Any] | None:
  data = flatten_payload(data)
  if not is_nav_heartbeat(data):
    return None

  road_limit_kph = _f(data, "nRoadLimitSpeed")
  speed_ms = _kph_to_ms(road_limit_kph)

  turn_type = int(_f(data, "nTBTTurnType"))
  turn_dist = _f(data, "nTBTDist")
  bucket = _turn_bucket(turn_type)
  mtype = _maneuver_type(bucket)
  direction = _dir_from_bucket(bucket)
  desc = _s(data, "szTBTMainText") or _s(data, "szNearDirName")

  next_type = int(_f(data, "nTBTTurnTypeNext"))
  next_dist = _f(data, "nTBTDistNext")
  next_bucket = _turn_bucket(next_type)

  # goalPos* = destination; vpPosPoint* = ego GPS report only (never treat as dest).
  dest_lat = _f(data, "goalPosY")
  dest_lon = _f(data, "goalPosX")
  if abs(dest_lat) > 90 and abs(dest_lat) < 90000000:
    dest_lat /= 1e6
  if abs(dest_lon) > 180 and abs(dest_lon) < 180000000:
    dest_lon /= 1e6

  dest_name = _s(data, "szGoalName")
  go_dist = _f(data, "nGoPosDist")
  go_time = _f(data, "nGoPosTime")

  # SDI / speed-camera pressure: 本期不做（手机不上线 nSdi*；设备忽略）。
  cam_type = "none"
  cam_valid = False

  lc_window = AGGRESSIVE_LC_DISTANCE_M if aggressive_lc else 500.0
  is_lc = bucket.startswith("lc")
  is_turn = bucket.startswith("turn")
  near_turn = 0.0 < turn_dist <= TURN_DESIRE_WINDOW_M
  # B1: amapauto 13012 laneRecommend=straight → suppress auto LC (TBT HUD still valid).
  lane_rec = _s(data, "laneRecommend", "none").strip().lower()
  lane_rec_straight = lane_rec == "straight"

  send_lc = bool(is_lc and 0.0 < turn_dist <= lc_window)
  if send_lc and lane_rec_straight:
    send_lc = False

  send_turn = bool(is_turn and near_turn)
  # Urban: Gaode often labels intersection maneuvers as lc*; promote when near (not highway fork).
  if is_lc and near_turn and not lane_rec_straight:
    send_turn = True
    send_lc = False

  # TBT distance speed cap removed: turns/LC/exit leave speedTarget at road limit.
  # Curve slowdown is owned by IQ.Dynamic experimental/blended longitudinal.
  long_speed = speed_ms
  long_provider = "route"
  accel_target = 0.0

  # Green-wave: no amapauto source — do not consume nGreenWaveSpeed (C5-b).

  light = _s(data, "trafficLight", "none").strip().lower()
  light_dist = _f(data, "trafficLightDistM")
  # APK red countdown (Gaode redLightCountDownSeconds). HUD always shows remainS.
  # Explicit remainS==1 is the IQ-link go signal; omitted/0 stays red (BleCrypto omits zero).
  remain_s = int(_f(data, "trafficLightRemainS"))
  remain_go = "trafficLightRemainS" in data and remain_s == 1
  vision_stop = bool(vision_stop) or bool(data.get("visionStop"))
  # China RTOR: only skip nav red/yellow when the right turn is at this light.
  right_turn_pending = bucket == "turn_right" and 0.0 < turn_dist <= LIGHT_TURN_WINDOW_M
  left_turn_pending = bucket == "turn_left" and 0.0 < turn_dist <= LIGHT_TURN_WINDOW_M
  # remainS==1 lifts nav red-stop so standstill hold can launch like green.
  # Never fake trafficLight="green". Omitted remainS keeps the red hold.
  stop_for_light = False
  if not right_turn_pending:
    if light == "red":
      stop_for_light = not remain_go
    elif light == "yellow" and light_dist > 0 and light_dist <= _YELLOW_STOP_DIST_M:
      stop_for_light = True

  if stop_for_light:
    if light_dist > 0:
      approach = _red_light_approach_ms(light_dist, long_speed if long_speed > 0 else speed_ms)
      long_speed = approach
    else:
      long_speed = 0.0
    accel_target = _RED_LIGHT_ACCEL
    long_provider = "route"

  engaged = bool(stop_for_light or long_speed > 0.0)

  # Arrive: keep limit / lights; only drop lateral LC/turn desire.
  if 0.0 < go_dist <= NEAR_DEST_REMAIN_M:
    send_lc = False
    send_turn = False
    mtype = "arrive"

  return {
    "active": True,
    # Raw Gaode road limit for HUD (distinct from targetSpeed which may be TBT/red capped).
    "roadSpeedLimit": speed_ms,
    "roadSpeedLimitValid": speed_ms > 0.0,
    # Fullscreen Gaode often has remain distance/time but no POI title in a11y tree.
    "destinationValid": bool(
      dest_name
      or (abs(dest_lat) > 0.01 and abs(dest_lon) > 0.01)
      or go_dist > 500.0
    ),
    "distanceRemaining": go_dist,
    "timeRemaining": go_time,
    "nextManeuverValid": turn_dist > 0 and mtype != "none",
    "nextManeuverDistance": turn_dist,
    "nextManeuverType": mtype,
    "nextManeuverDirection": direction,
    "nextManeuverDescription": desc,
    "secondNextManeuverValid": next_dist > 0 and next_bucket != "none",
    "secondNextManeuverType": _maneuver_type(next_bucket),
    "secondNextManeuverDirection": _dir_from_bucket(next_bucket),
    "secondNextManeuverDistance": next_dist,
    "shouldSendTurnDesire": send_turn,
    "turnDesireDirection": direction if send_turn else "none",
    "shouldSendLaneChangeDesire": send_lc,
    "laneChangeDesireDirection": direction if send_lc else "none",
    "maneuverPhase": "turnActive" if send_turn else ("highwayCommit" if send_lc else "none"),
    "maneuverDirection": direction if (send_turn or send_lc) else "none",
    "command": "laneChange" if send_lc else "none",
    "commandDirection": direction if send_lc else "none",
    "commandIndex": command_index,
    "destinationLatitude": dest_lat,
    "destinationLongitude": dest_lon,
    "destinationName": dest_name,
    "targetSpeed": long_speed,
    "targetSpeedValid": engaged,
    "speedTarget": long_speed,
    "accelTarget": accel_target,
    "valid": engaged,
    "longitudinalEngaged": engaged,
    "longitudinalProvider": long_provider,
    "longitudinalState": "active" if engaged else "disabled",
    "navSpeedTargetActive": engaged and (stop_for_light or long_speed < speed_ms - 1e-3),
    "cameraValid": cam_valid,
    "cameraType": cam_type,
    "cameraDistance": 0.0,
    "cameraSpeedLimit": 0.0,
    "navTurnDesireDirection": direction if send_turn else "none",
    "navLaneChangeDesireDirection": direction if send_lc else "none",
    "trafficLight": light,
    "trafficLightDistM": float(light_dist or 0.0),
    "trafficLightRemainS": float(max(remain_s, 0)),
    "visionStop": vision_stop,
    "rightTurnPending": right_turn_pending,
    "leftTurnPending": left_turn_pending,
    "laneRecommend": lane_rec,
  }
