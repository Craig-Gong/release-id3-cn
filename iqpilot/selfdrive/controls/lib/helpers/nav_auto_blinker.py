"""IQ-link navigation auto blinker (default off).

Highway turn: arm at clamp(v×T_arm, D_min, D_max) inside send_turn (≤150 m).
Highway fork: same arm distance on send_lc / highwayCommit (no 150 m cap).
Urban (road limit <70 and v <65): turn path adds red-left hold, RTOR, BSM delay.
"""

from __future__ import annotations

from openpilot.common.params import Params
from openpilot.iqpilot.iqlink.protocol import LIGHT_TURN_WINDOW_M, TURN_DESIRE_WINDOW_M
from openpilot.iqpilot.selfdrive.controls.lib.helpers.turn_prep import (
  HIGHWAY_LIMIT_MS,
  NAV_NEAR_TURN_M,
  PHASE_HIGHWAY_COMMIT,
  URBAN_V_MAX_MS,
)

PARAM_ENABLED = "IQNavAutoBlinker"

T_ARM_S = 3.75
D_MIN_M = 50.0
D_MAX_M = 120.0
DEBOUNCE_FRAMES = 3
STANDSTILL_V_MS = 1.0

PHASE_TURN_ACTIVE = 2
MANEUVER_FORK = 4
DIR_LEFT = 1
DIR_RIGHT = 2


def _as_int(value) -> int:
  if value is None:
    return 0
  raw = getattr(value, "raw", value)
  try:
    return int(raw)
  except (TypeError, ValueError):
    return 0


def _side_from_attr(nav, attr: str) -> str:
  direction = getattr(nav, attr, None)
  name = str(getattr(direction, "name", None) or direction or "").lower()
  if "left" in name:
    return "left"
  if "right" in name:
    return "right"
  raw = _as_int(direction)
  if raw == DIR_LEFT:
    return "left"
  if raw == DIR_RIGHT:
    return "right"
  return "none"


def _dir_name(nav) -> str:
  return _side_from_attr(nav, "turnDesireDirection")


def _lc_dir_name(nav) -> str:
  return _side_from_attr(nav, "laneChangeDesireDirection")


def _mtype_is_fork(nav) -> bool:
  mtype = getattr(nav, "nextManeuverType", None)
  name = str(getattr(mtype, "name", None) or mtype or "").lower()
  if name == "fork":
    return True
  return _as_int(mtype) == MANEUVER_FORK


def _light_token(nav) -> str:
  light = getattr(nav, "trafficLight", None)
  return str(getattr(light, "name", None) or light or "none").strip().lower()


def _remain_go(nav) -> bool:
  remain = float(getattr(nav, "trafficLightRemainS", 0.0) or 0.0)
  return abs(remain - 1.0) < 1e-6


def _turn_pending(nav, side: str) -> bool:
  if side == "left":
    return bool(getattr(nav, "leftTurnPending", False))
  if side == "right":
    return bool(getattr(nav, "rightTurnPending", False))
  return False


def arm_distance_m(v_ego_mps: float) -> float:
  dist = float(v_ego_mps) * T_ARM_S
  return min(max(dist, D_MIN_M), D_MAX_M, float(TURN_DESIRE_WINDOW_M))


def is_urban_context(road_limit_ms: float, v_ego_mps: float) -> bool:
  """Urban gates: road limit <70 km/h and ego below 65 km/h (align turn_prep)."""
  limit = float(road_limit_ms or 0.0)
  if limit <= 0.0:
    limit = HIGHWAY_LIMIT_MS
  return limit < HIGHWAY_LIMIT_MS and float(v_ego_mps) < URBAN_V_MAX_MS


def is_highway_fast_context(road_limit_ms: float, v_ego_mps: float) -> bool:
  limit = float(road_limit_ms or 0.0)
  if limit <= 0.0:
    limit = 0.0
  return limit >= HIGHWAY_LIMIT_MS or float(v_ego_mps) >= URBAN_V_MAX_MS


def same_side_bsm_blocked(side: str, cs) -> bool:
  if side == "left":
    return bool(getattr(cs, "leftBlindspot", False))
  if side == "right":
    return bool(getattr(cs, "rightBlindspot", False))
  return False


def urban_red_left_hold(nav, cs, *, side: str) -> bool:
  """Left at red: wait until stopped or within NAV_NEAR_TURN_M (80 m)."""
  if side != "left":
    return False
  if _light_token(nav) != "red":
    return False
  if _remain_go(nav):
    return False
  if not _turn_pending(nav, "left"):
    return False
  v = float(getattr(cs, "vEgo", 0.0) or 0.0)
  dist = float(getattr(nav, "nextManeuverDistance", 0.0) or 0.0)
  if v <= STANDSTILL_V_MS:
    return False
  if 0.0 < dist <= NAV_NEAR_TURN_M:
    return False
  return True


def urban_rtor_red_hold(nav, cs, *, side: str) -> bool:
  """Right on red: arm only when trafficLightDistM is within arm window."""
  if side != "right":
    return False
  if _light_token(nav) != "red":
    return False
  if not _turn_pending(nav, "right"):
    return False
  light_dist = float(getattr(nav, "trafficLightDistM", 0.0) or 0.0)
  if light_dist <= 0.0:
    return True
  if light_dist > LIGHT_TURN_WINDOW_M:
    return True
  v = float(getattr(cs, "vEgo", 0.0) or 0.0)
  close_m = min(arm_distance_m(v), NAV_NEAR_TURN_M)
  return light_dist > close_m


def _blink_side_blocked(side: str, cs) -> bool:
  left = bool(getattr(cs, "leftBlinker", False))
  right = bool(getattr(cs, "rightBlinker", False))
  if side == "left" and right:
    return True
  if side == "right" and left:
    return True
  if (side == "left" and left) or (side == "right" and right):
    return True
  if same_side_bsm_blocked(side, cs):
    return True
  return False


class NavAutoBlinker:
  def __init__(self, params: Params | None = None) -> None:
    self._params = params
    self._enabled = False
    self._tick = 0
    self._debounce = 0
    self._active_side: str | None = None
    self.read_params()

  def _store(self) -> Params | None:
    if self._params is not None:
      return self._params
    try:
      self._params = Params()
    except Exception:
      self._params = None
    return self._params

  def read_params(self) -> None:
    store = self._store()
    if store is None:
      return
    self._enabled = bool(store.get_bool(PARAM_ENABLED))

  def reset(self) -> None:
    self._debounce = 0
    self._active_side = None

  def _iqlink_on(self, params: Params) -> bool:
    try:
      return bool(params.get_bool("IqlinkExclusive")) or bool(params.get_bool("NavigationActive"))
    except Exception:
      return False

  def _link_warn(self, params: Params) -> bool:
    try:
      return bool(params.get_bool("IqlinkLinkWarn"))
    except Exception:
      return False

  def _eligible_turn(
    self,
    *,
    nav,
    cs,
    road_ms: float,
    v_ego: float,
  ) -> str | None:
    if not bool(getattr(nav, "shouldSendTurnDesire", False)):
      return None
    if bool(getattr(nav, "shouldSendLaneChangeDesire", False)):
      return None
    if _as_int(getattr(nav, "maneuverPhase", 0)) != PHASE_TURN_ACTIVE:
      return None

    side = _dir_name(nav)
    if side not in ("left", "right"):
      return None
    if _blink_side_blocked(side, cs):
      return None

    if is_urban_context(road_ms, v_ego):
      if urban_red_left_hold(nav, cs, side=side):
        return None
      if urban_rtor_red_hold(nav, cs, side=side):
        return None

    dist = float(getattr(nav, "nextManeuverDistance", 0.0) or 0.0)
    if not 0.0 < dist <= float(TURN_DESIRE_WINDOW_M):
      return None
    if dist > arm_distance_m(v_ego):
      return None
    return side

  def _eligible_fork(
    self,
    *,
    nav,
    cs,
    road_ms: float,
    v_ego: float,
  ) -> str | None:
    if not bool(getattr(nav, "shouldSendLaneChangeDesire", False)):
      return None
    if bool(getattr(nav, "shouldSendTurnDesire", False)):
      return None
    if _as_int(getattr(nav, "maneuverPhase", 0)) != PHASE_HIGHWAY_COMMIT:
      return None
    if not _mtype_is_fork(nav):
      return None
    if not is_highway_fast_context(road_ms, v_ego):
      return None

    side = _lc_dir_name(nav)
    if side not in ("left", "right"):
      return None
    if _blink_side_blocked(side, cs):
      return None

    dist = float(getattr(nav, "nextManeuverDistance", 0.0) or 0.0)
    if dist <= 0.0 or dist > arm_distance_m(v_ego):
      return None
    return side

  def _eligible(
    self,
    *,
    nav,
    cs,
    engaged: bool,
    iqlink_on: bool,
    link_warn: bool,
  ) -> str | None:
    if not self._enabled or not engaged or not iqlink_on or link_warn:
      return None
    gear = getattr(cs, "gearShifter", None)
    gear_name = str(getattr(gear, "name", None) or gear or "").lower()
    if gear_name in ("park", "reverse", "neutral"):
      return None
    if not bool(getattr(nav, "active", False)):
      return None

    road_ms = float(getattr(nav, "roadSpeedLimit", 0.0) or 0.0)
    v_ego = float(getattr(cs, "vEgo", 0.0) or 0.0)

    side = self._eligible_turn(nav=nav, cs=cs, road_ms=road_ms, v_ego=v_ego)
    if side is None:
      side = self._eligible_fork(nav=nav, cs=cs, road_ms=road_ms, v_ego=v_ego)
    return side

  def update(
    self,
    nav,
    cs,
    *,
    engaged: bool,
    params: Params | None = None,
  ) -> tuple[bool, bool]:
    self._tick += 1
    if self._tick % 50 == 0:
      self.read_params()

    store = params or self._store()
    iqlink_on = self._iqlink_on(store) if store is not None else False
    link_warn = self._link_warn(store) if store is not None else True

    side = self._eligible(
      nav=nav,
      cs=cs,
      engaged=engaged,
      iqlink_on=iqlink_on,
      link_warn=link_warn,
    )

    if side is None:
      self.reset()
      return False, False

    self._debounce = min(self._debounce + 1, DEBOUNCE_FRAMES + 1)
    if self._debounce < DEBOUNCE_FRAMES:
      return False, False

    self._active_side = side
    return (side == "left"), (side == "right")
