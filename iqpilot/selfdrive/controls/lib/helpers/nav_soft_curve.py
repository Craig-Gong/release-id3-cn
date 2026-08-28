"""IQ-link nav turn soft speed cap for fast / highway approaches.

Urban turns below ~65 km/h stay with turn_prep (G-3 / 20). This fills the gap
when posted limit >= 70 or ego is still above the turn gate: gradual sqrt decel
from TBT distance toward ~40 km/h at the maneuver without touching MAX.
"""

from __future__ import annotations

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.iqpilot.iqlink.protocol import TURN_DESIRE_WINDOW_M
from openpilot.iqpilot.selfdrive.controls.lib.helpers.nav_decel import approach_speed_ms
from openpilot.iqpilot.selfdrive.controls.lib.helpers.turn_prep import (
  DEFAULT_TURN_GATE_MPS,
  HIGHWAY_LIMIT_MS,
  TURN_TRIGGER_MPS,
  URBAN_V_MAX_MS,
)

PARAM_SOFT_CURVE = "IQNavSoftCurveCap"
DECEL_MS2 = 1.2
PHASE_TURN_ACTIVE = 2


def _as_bool(value) -> bool:
  if value is None:
    return False
  raw = getattr(value, "raw", value)
  return bool(raw)


def _as_int(value) -> int:
  if value is None:
    return 0
  raw = getattr(value, "raw", value)
  try:
    return int(raw)
  except (TypeError, ValueError):
    return 0


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
    # Default on: highway/ramp TBT has no turn_prep path.
    stored = store.get(PARAM_SOFT_CURVE)
    self._enabled = True if stored is None else bool(store.get_bool(PARAM_SOFT_CURVE))

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
  ) -> float | None:
    self._tick += 1
    if self._tick % 50 == 0:
      self.read_params()

    if not enabled or not self._enabled or not iqlink_on:
      return None
    if nav_send_lc or not nav_send_turn:
      return None
    if _as_int(nav_phase) != PHASE_TURN_ACTIVE:
      return None
    dist = float(turn_dist_m or 0.0)
    if not 0.0 < dist <= float(TURN_DESIRE_WINDOW_M):
      return None

    highway = float(posted_limit_ms or 0.0) >= HIGHWAY_LIMIT_MS
    fast_urban = float(v_ego) > URBAN_V_MAX_MS
    above_gate = float(v_ego) >= TURN_TRIGGER_MPS
    if not highway and not fast_urban and not above_gate:
      return None  # turn_prep owns sub-45 urban

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
