from types import SimpleNamespace

import pytest

from iqpilot.cereal import custom, log
from iqpilot.common.realtime import DT_MDL
from iqpilot.selfdrive.controls.lib.desire_helper import (
  DesireHelper,
  TURN_DESIRE_STOP_CYCLE_TIME,
  TURN_DESIRE_STOP_HOLD_TIME,
)


TurnDirection = custom.IQTurnSignalDirection


def helper(v_ego=0.0, yaw_rate=0.0):
  result = DesireHelper.__new__(DesireHelper)
  result._last_carstate = SimpleNamespace(vEgo=v_ego, yawRate=yaw_rate)
  result.turn_desire_stop_timer = 0.0
  result.turn_desire_stop_active = False
  result.turn_desire_cycle_input = log.Desire.none
  result.turn_desire_committed = False
  result.nav_turn_direction = TurnDirection.none
  result.lane_turn_direction = TurnDirection.none
  result.lane_change_direction = log.LaneChangeDirection.none
  result.lane_change_state = log.LaneChangeState.off
  result.desire = log.Desire.none
  return result


@pytest.mark.parametrize("source", ["manual", "nav"])
def test_manual_and_nav_turn_desires_receive_rising_edges(source):
  h = helper()
  if source == "manual":
    h.lane_turn_direction = TurnDirection.turnLeft
  else:
    h.nav_turn_direction = TurnDirection.turnLeft

  outputs = []
  for _ in range(round((TURN_DESIRE_STOP_CYCLE_TIME + 2 * DT_MDL) / DT_MDL)):
    h._pick_desire_output()
    outputs.append(h.desire)

  gap_index = next(i for i, output in enumerate(outputs) if output == log.Desire.none)
  assert gap_index * DT_MDL == pytest.approx(TURN_DESIRE_STOP_HOLD_TIME, abs=DT_MDL * 1.1)
  assert log.Desire.turnLeft in outputs[gap_index + 1:]


def test_creeping_restarts_stopped_turn_cycle():
  h = helper()
  for _ in range(round(TURN_DESIRE_STOP_HOLD_TIME / DT_MDL)):
    h._cycle_turn_desire_when_stopped(log.Desire.turnRight)

  h._last_carstate.vEgo = 3.0
  assert h._cycle_turn_desire_when_stopped(log.Desire.turnRight) == log.Desire.turnRight
  h._last_carstate.vEgo = 0.0
  assert h._cycle_turn_desire_when_stopped(log.Desire.turnRight) == log.Desire.turnRight
  assert h.turn_desire_stop_timer == pytest.approx(DT_MDL)


def test_measured_turn_commitment_stops_cycling():
  h = helper()
  h._last_carstate.yawRate = -0.1
  assert h._cycle_turn_desire_when_stopped(log.Desire.turnLeft) == log.Desire.turnLeft
  h._last_carstate.yawRate = 0.0

  outputs = [h._cycle_turn_desire_when_stopped(log.Desire.turnLeft) for _ in range(300)]
  assert set(outputs) == {log.Desire.turnLeft}


def test_new_turn_direction_rearms_cycle_after_commitment():
  h = helper(yaw_rate=0.1)
  h._cycle_turn_desire_when_stopped(log.Desire.turnLeft)
  h._last_carstate.yawRate = 0.0
  h._cycle_turn_desire_when_stopped(log.Desire.turnRight)
  assert h.turn_desire_committed is False
  assert h.turn_desire_cycle_input == log.Desire.turnRight
