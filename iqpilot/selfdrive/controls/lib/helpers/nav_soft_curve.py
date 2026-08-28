"""IQ-link nav turn soft speed cap for fast / highway approaches.

Urban turns below ~65 km/h stay with turn_prep (G-3 / 20). Highway send_turn
and highwayCommit fork share sqrt decel toward ~40 km/h at the maneuver.
"""

from __future__ import annotations

from openpilot.common.params import Params
from openpilot.iqpilot.iqlink.protocol import TURN_DESIRE_WINDOW_M
from openpilot.iqpilot.selfdrive.controls.lib.helpers.nav_auto_blinker import (
  arm_distance_m,
  is_highway_fast_context,
)
from openpilot.iqpilot.selfdrive.controls.lib.helpers.nav_decel import approach_speed_ms
from openpilot.iqpilot.selfdrive.controls.lib.helpers.turn_prep import (
  DEFAULT_TURN_GATE_MPS,
  PHASE_HIGHWAY_COMMIT,
  TURN_TRIGGER_MPS,
  URBAN_V_MAX_MS,
)

PARAM_SOFT_CURVE = "IQNavSoftCurveCap"
DECEL_MS2 = 1.2
PHASE_TURN_ACTIVE = 2
MANEUVER_FORK = 4


def _as_int(value) -> int:
  if value is None:
    return 0
  raw = getattr(value, "raw", value)
  try:
    return int(raw)
  except (TypeError, ValueError):
    return 0


def _mtype_is_fork(nav) -> bool:
  mtype = getattr(nav, "nextManeuverType", None)
  name = str(getattr(mtype, "name", None) or mtype or "").lower()
  return name == "fork" or _as_int(mtype) == MANEUVER_FORK


class NavSoftCurveCap:
  def __init__(self, params: Params | None = None) -> None:
    self._params = params
    self._enabled = True
    self._tick = 0
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
    stored = store.get(PARAM_SOFT_CURVE)
    self._enabled = True if stored is None else bool(store.get_bool(PARAM_SOFT_CURVE))

  def _cap_for_dist(self, dist: float, v_ego: float, posted_limit_ms: float) -> float | None:
    cap = approach_speed_ms(
      dist,
      DECEL_MS2,
      floor_ms=DEFAULT_TURN_GATE_MPS,
      cap_ms=float(v_ego),
    )
    road_cap = float(posted_limit_ms or 0.0)
    if road_cap > 0.0:
      cap = min(cap, road_cap)
    return cap if cap < float(v_ego) - 0.2 else None

  def update(
    self,
    *,
    iqlink_on: bool,
    enabled: bool,
    v_ego: float,
    posted_limit_ms: float,
    nav_send_turn: bool,
    nav_phase: int,
    turn_dist_m: float,
    nav_send_lc: bool,
    nav=None,
  ) -> float | None:
    self._tick += 1
    if self._tick % 50 == 0:
      self.read_params()

    if not enabled or not self._enabled or not iqlink_on:
      return None

    road_ms = float(posted_limit_ms or 0.0)
    if nav is not None:
      nav_road = float(getattr(nav, "roadSpeedLimit", 0.0) or 0.0)
      if nav_road > 0.0:
        road_ms = nav_road

    highway = is_highway_fast_context(road_ms, v_ego)
    fast_urban = float(v_ego) > URBAN_V_MAX_MS
    above_gate = float(v_ego) >= TURN_TRIGGER_MPS
    if not highway and not fast_urban and not above_gate:
      return None

    dist = float(turn_dist_m or 0.0)
    phase = _as_int(nav_phase)

    if nav_send_turn and phase == PHASE_TURN_ACTIVE:
      if not 0.0 < dist <= float(TURN_DESIRE_WINDOW_M):
        return None
      return self._cap_for_dist(dist, v_ego, road_ms)

    if (
      nav_send_lc
      and phase == PHASE_HIGHWAY_COMMIT
      and nav is not None
      and _mtype_is_fork(nav)
      and highway
      and dist > 0.0
      and dist <= arm_distance_m(v_ego)
    ):
      return self._cap_for_dist(dist, v_ego, road_ms)

    return None
