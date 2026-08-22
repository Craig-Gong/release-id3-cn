from openpilot.common.constants import CV
from openpilot.iqpilot.selfdrive.controls.lib.helpers.lane_turn import (
  TURN_TRIGGER_MPS,
  _resolve_signal_choice,
)
from cereal import custom

TurnDirection = custom.IQTurnSignalDirection


def test_turn_desire_at_urban_approach_speed():
  # 40 km/h with left blinker must still request turnLeft (old hard cap was 32 km/h).
  v = 40.0 * CV.KPH_TO_MS
  assert v < TURN_TRIGGER_MPS
  assert _resolve_signal_choice(v, TURN_TRIGGER_MPS, True, False) == TurnDirection.turnLeft
  assert _resolve_signal_choice(v, TURN_TRIGGER_MPS, False, True) == TurnDirection.turnRight


def test_turn_desire_in_former_dead_band():
  # Default prep G is 40; arriving at 42 used to emit neither LC nor turn.
  v = 42.0 * CV.KPH_TO_MS
  gate_40 = 40.0 * CV.KPH_TO_MS
  assert _resolve_signal_choice(v, gate_40, True, False) == TurnDirection.none
  assert _resolve_signal_choice(v, TURN_TRIGGER_MPS, True, False) == TurnDirection.turnLeft
  assert _resolve_signal_choice(v, TURN_TRIGGER_MPS, False, True) == TurnDirection.turnRight


def test_turn_desire_clears_above_gate():
  v = TURN_TRIGGER_MPS + 0.5
  assert _resolve_signal_choice(v, TURN_TRIGGER_MPS, True, False) == TurnDirection.none


def test_highway_blinker_is_not_a_turn_desire():
  v = 80.0 * CV.KPH_TO_MS
  assert _resolve_signal_choice(v, TURN_TRIGGER_MPS, True, False) == TurnDirection.none


def test_same_side_bsm_blocks_turn():
  v = 30.0 * CV.KPH_TO_MS
  assert _resolve_signal_choice(v, TURN_TRIGGER_MPS, True, False, left_blocked=True) == TurnDirection.none
  assert _resolve_signal_choice(v, TURN_TRIGGER_MPS, False, True, right_blocked=True) == TurnDirection.none
  # Opposite-side BSM must not block.
  assert _resolve_signal_choice(v, TURN_TRIGGER_MPS, True, False, right_blocked=True) == TurnDirection.turnLeft
  assert _resolve_signal_choice(v, TURN_TRIGGER_MPS, False, True, left_blocked=True) == TurnDirection.turnRight
