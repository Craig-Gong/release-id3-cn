"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""

from __future__ import annotations

from dataclasses import dataclass

from cereal import custom

from openpilot.common.constants import CV
from openpilot.common.params import Params

TurnDirection = custom.IQTurnSignalDirection

# Urban intersection approach with blinker on (China). Hard cap used to be 20 mph /
# 32 km/h — too low for typical turn approaches, so the blinker fell into the lane-
# change FSM and the model kept lane-centering instead of turning.
#
# Desire always uses this 45 km/h split (same as "no LC below here"). The prep
# slider default is still ~40 (G−3 approach / turn-in); using that for desire
# left a 40–45 dead band — no LC and no turn, so the car went straight.
TURN_TRIGGER_MPS = 45.0 * CV.KPH_TO_MS  # ~28 mph
TURN_SPEED_GATE_MPS = TURN_TRIGGER_MPS
LANE_CHANGE_SPEED_MIN = TURN_SPEED_GATE_MPS
DEFAULT_TURN_GATE_MPS = 40.0 * CV.KPH_TO_MS


@dataclass
class _TurnGateState:
  active: bool = True
  speed_limit_mps: float = DEFAULT_TURN_GATE_MPS
  outcome: int = TurnDirection.none
  refresh_tick: int = 0


def _mph_param_to_mps(raw_value) -> float:
  try:
    return float(raw_value) * CV.MPH_TO_MS
  except (TypeError, ValueError):
    return DEFAULT_TURN_GATE_MPS


def _resolve_signal_choice(speed_mps: float,
                           speed_limit_mps: float,
                           left_signal: bool,
                           right_signal: bool,
                           left_blocked: bool = False,
                           right_blocked: bool = False) -> int:
  """Blinker → turn desire below the gate. Same-side BSM blocks (side traffic)."""
  if speed_mps >= speed_limit_mps:
    return TurnDirection.none
  if left_signal and not right_signal and not left_blocked:
    return TurnDirection.turnLeft
  if right_signal and not left_signal and not right_blocked:
    return TurnDirection.turnRight
  return TurnDirection.none


class TurnSignalPlanner:
  _REFRESH_STRIDE = 50

  def __init__(self, desire_hub):
    self._desire_hub = desire_hub
    self._params = Params()
    self._state = _TurnGateState()
    self.reload_setup()

  def _refresh_from_params(self) -> None:
    requested_gate = _mph_param_to_mps(self._params.get("IQLaneTurnValue", return_default=True))
    # Default ON: missing key → True. Explicit off still wins.
    stored = self._params.get("IQLaneTurnDesire")
    if stored is None:
      self._state.active = True
    else:
      self._state.active = self._params.get_bool("IQLaneTurnDesire")
    self._state.speed_limit_mps = min(TURN_TRIGGER_MPS, max(requested_gate, 5.0 * CV.MPH_TO_MS))

  def _consume_legacy_kwargs(self, **legacy) -> tuple[bool, bool, bool, bool, float]:
    return (
      bool(legacy.get("blindspot_left", False)),
      bool(legacy.get("blindspot_right", False)),
      bool(legacy.get("left_blinker", False)),
      bool(legacy.get("right_blinker", False)),
      float(legacy.get("v_ego", 0.0)),
    )

  def reload_setup(self):
    self._refresh_from_params()

  def heartbeat(self) -> None:
    if self._state.refresh_tick % self._REFRESH_STRIDE == 0:
      self._refresh_from_params()
    self._state.refresh_tick += 1

  def sample(self,
             blocked_l: bool = False,
             blocked_r: bool = False,
             blink_l: bool = False,
             blink_r: bool = False,
             speed_mps: float = 0.0,
             **legacy) -> None:
    if legacy:
      blocked_l, blocked_r, blink_l, blink_r, speed_mps = self._consume_legacy_kwargs(**legacy)
    # IQLaneTurnValue is the longitudinal prep gate only. Desire follows the
    # LC/turn split so 40–45 km/h with a blinker is still a turn.
    self._state.outcome = _resolve_signal_choice(speed_mps,
                                                 TURN_TRIGGER_MPS,
                                                 blink_l,
                                                 blink_r,
                                                 blocked_l,
                                                 blocked_r)

  def output(self):
    return self._state.outcome if self._state.active else TurnDirection.none

  @property
  def enabled(self):
    return self._state.active

  @enabled.setter
  def enabled(self, value):
    self._state.active = bool(value)

  @property
  def speed_gate(self):
    return self._state.speed_limit_mps

  @speed_gate.setter
  def speed_gate(self, value):
    self._state.speed_limit_mps = float(value)

  @property
  def turn_direction(self):
    return self._state.outcome

  @turn_direction.setter
  def turn_direction(self, value):
    self._state.outcome = value


class IQNavTurnController(TurnSignalPlanner):
  def __init__(self, desire_helper):
    super().__init__(desire_helper)

  def read_params(self):
    self.reload_setup()

  def update_params(self) -> None:
    self.heartbeat()

  def update_lane_turn(self,
                       blindspot_left: bool,
                       blindspot_right: bool,
                       left_blinker: bool,
                       right_blinker: bool,
                       v_ego: float) -> None:
    self.sample(blocked_l=blindspot_left,
                blocked_r=blindspot_right,
                blink_l=left_blinker,
                blink_r=right_blinker,
                speed_mps=v_ego)

  def get_turn_direction(self):
    return self.output()

  @property
  def lane_turn_value(self):
    return self.speed_gate

  @lane_turn_value.setter
  def lane_turn_value(self, value):
    self.speed_gate = value
