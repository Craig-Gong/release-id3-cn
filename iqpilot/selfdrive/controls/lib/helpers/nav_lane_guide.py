"""IQ-link lane-recommend HUD + gentle highway prep + second-TBT lookahead.

Does not trigger lane-change desire or NavExit ALC — hints and optional weak
longitudinal min() only.
"""

from __future__ import annotations

import time

from iqpilot.common.params import Params
from iqpilot.selfdrive.controls.lib.helpers.nav_highway_context import is_highway_fast_context
from iqpilot.selfdrive.controls.lib.helpers.nav_decel import approach_speed_ms
from iqpilot.selfdrive.controls.lib.helpers.turn_prep import (
  DEFAULT_TURN_GATE_MPS,
  PHASE_HIGHWAY_COMMIT,
)

PARAM_LANE_GUIDE = "IQNavLaneGuide"

HOLD_S = 1.0
SECOND_LOOKAHEAD_M = 500.0
PREP_DECEL_MS2 = 0.8
PREP_VIRTUAL_DIST_M = 280.0
PHASE_TURN_ACTIVE = 2
MANEUVER_TURN = 1
MANEUVER_FORK = 4


def _as_int(value) -> int:
  if value is None:
    return 0
  raw = getattr(value, "raw", value)
  try:
    return int(raw)
  except (TypeError, ValueError):
    return 0


def _token(text) -> str:
  return str(text or "none").strip().lower()


def _mtype_is_turn(nav, attr: str) -> bool:
  mtype = getattr(nav, attr, None)
  name = _token(getattr(mtype, "name", None) or mtype)
  return name == "turn" or _as_int(mtype) == MANEUVER_TURN


def _side_from_nav(nav, attr: str) -> str:
  direction = getattr(nav, attr, None)
  name = _token(getattr(direction, "name", None) or direction)
  if "left" in name:
    return "left"
  if "right" in name:
    return "right"
  raw = _as_int(direction)
  if raw == 1:
    return "left"
  if raw == 2:
    return "right"
  return "none"


class _TextHold:
  def __init__(self, hold_s: float = HOLD_S) -> None:
    self._hold_s = hold_s
    self._confirmed = ""
    self._pending = ""
    self._pending_since = 0.0

  def reset(self) -> None:
    self._confirmed = ""
    self._pending = ""
    self._pending_since = 0.0

  def filter(self, raw: str, now: float) -> str:
    raw = _token(raw)
    if raw in ("", "none"):
      return self._confirmed
    if raw == self._confirmed:
      self._pending = ""
      return self._confirmed
    if raw != self._pending:
      self._pending = raw
      self._pending_since = now
      return self._confirmed
    if now - self._pending_since >= self._hold_s:
      self._confirmed = raw
      self._pending = ""
    return self._confirmed


def _hint_for_lane(side: str) -> str:
  if side == "left":
    return "靠左车道"
  if side == "right":
    return "靠右车道"
  return ""


def _hint_for_second(side: str) -> str:
  if side == "left":
    return "前方左转"
  if side == "right":
    return "前方右转"
  return ""


class NavLaneGuide:
  def __init__(self, params: Params | None = None) -> None:
    self._params = params
    self._enabled = True
    self._lane_hold = _TextHold()
    self._hint = ""
    self._tick = 0

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
    stored = store.get(PARAM_LANE_GUIDE)
    self._enabled = True if stored is None else bool(store.get_bool(PARAM_LANE_GUIDE))

  @property
  def hint(self) -> str:
    return self._hint

  def update(
    self,
    nav,
    *,
    engaged: bool,
    iqlink_on: bool,
    v_ego: float,
    posted_limit_ms: float,
    slc_enabled: bool,
  ) -> float | None:
    self._tick += 1
    if self._tick % 50 == 0:
      self.read_params()

    self._hint = ""
    if not engaged or not iqlink_on or not slc_enabled or not self._enabled:
      self._lane_hold.reset()
      return None
    if nav is None or not bool(getattr(nav, "active", False)):
      self._lane_hold.reset()
      return None

    road_ms = float(getattr(nav, "roadSpeedLimit", 0.0) or 0.0)
    if road_ms <= 0.0:
      road_ms = float(posted_limit_ms or 0.0)
    highway = is_highway_fast_context(road_ms, v_ego)
    now = time.monotonic()

    send_turn = bool(getattr(nav, "shouldSendTurnDesire", False))
    send_lc = bool(getattr(nav, "shouldSendLaneChangeDesire", False))
    phase = _as_int(getattr(nav, "maneuverPhase", 0))

    lane_rec = self._lane_hold.filter(getattr(nav, "laneRecommend", "none"), now)
    caps: list[float] = []

    # ② laneRecommend left/right — HUD always when debounced; weak cap on highway only.
    if lane_rec in ("left", "right") and not send_turn:
      self._hint = _hint_for_lane(lane_rec)
      if highway and not send_lc:
        cap = approach_speed_ms(
          PREP_VIRTUAL_DIST_M,
          PREP_DECEL_MS2,
          floor_ms=DEFAULT_TURN_GATE_MPS,
          cap_ms=float(v_ego),
        )
        road_cap = float(road_ms or 0.0)
        if road_cap > 0.0:
          cap = min(cap, road_cap)
        if cap < float(v_ego) - 0.2:
          caps.append(cap)

    # ③ secondNext maneuver — hint + cap when next segment still far.
    if bool(getattr(nav, "secondNextManeuverValid", False)):
      second_dist = float(getattr(nav, "secondNextManeuverDistance", 0.0) or 0.0)
      second_side = _side_from_nav(nav, "secondNextManeuverDirection")
      if (
        highway
        and _mtype_is_turn(nav, "secondNextManeuverType")
        and second_side in ("left", "right")
        and 0.0 < second_dist <= SECOND_LOOKAHEAD_M
        and not send_turn
        and phase != PHASE_TURN_ACTIVE
      ):
        second_hint = _hint_for_second(second_side)
        if second_hint:
          self._hint = second_hint
        cap = approach_speed_ms(
          second_dist,
          PREP_DECEL_MS2,
          floor_ms=DEFAULT_TURN_GATE_MPS,
          cap_ms=float(v_ego),
        )
        road_cap = float(road_ms or 0.0)
        if road_cap > 0.0:
          cap = min(cap, road_cap)
        if cap < float(v_ego) - 0.2:
          caps.append(cap)

    if not caps:
      return None
    return min(caps)
