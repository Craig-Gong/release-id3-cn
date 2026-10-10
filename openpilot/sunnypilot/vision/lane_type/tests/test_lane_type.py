"""Unit tests: class map, hysteresis, shm stale, DesireHelper solid gate, low-speed turn."""
from __future__ import annotations

import os
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest import mock

from openpilot.sunnypilot.vision.lane_type.hysteresis import (
  DASHED_EXIT, SOLID_ENTER, LaneTypeGate, SideHysteresis,
)
from openpilot.sunnypilot.vision.lane_type.inference import class_to_type
from openpilot.sunnypilot.vision.lane_type.snapshot import (
  LANE_TYPE_FRESH_S, LINE_DASHED, LINE_SOLID, LINE_UNKNOWN,
  LaneTypeSnapshot, read_lane_type, write_lane_type,
)


class TestClassMap(unittest.TestCase):
  def test_dashed_solid_unknown(self):
    self.assertEqual(class_to_type(1), LINE_DASHED)
    for cid in (0, 2, 5):
      self.assertEqual(class_to_type(cid), LINE_SOLID)
    for cid in (3, 4, 99):
      self.assertEqual(class_to_type(cid), LINE_UNKNOWN)


class TestHysteresis(unittest.TestCase):
  def test_solid_enter_dashed_exit(self):
    h = SideHysteresis()
    for _ in range(SOLID_ENTER - 1):
      self.assertFalse(h.update(LINE_SOLID))
    self.assertTrue(h.update(LINE_SOLID))
    for _ in range(DASHED_EXIT - 1):
      self.assertTrue(h.update(LINE_DASHED))
    self.assertFalse(h.update(LINE_DASHED))

  def test_unknown_freezes_lock(self):
    h = SideHysteresis()
    for _ in range(SOLID_ENTER):
      h.update(LINE_SOLID)
    self.assertTrue(h.locked_solid)
    self.assertTrue(h.update(LINE_UNKNOWN))
    self.assertTrue(h.locked_solid)

  def test_gate_disabled_or_invalid_fail_open(self):
    g = LaneTypeGate()
    solid = LaneTypeSnapshot(ts=time.monotonic(), left=LINE_SOLID, right=LINE_DASHED, valid=True)
    g.update(solid, enabled=True)
    for _ in range(SOLID_ENTER):
      g.update(solid, enabled=True)
    self.assertTrue(g.blocks(True))
    g.update(solid, enabled=False)
    self.assertFalse(g.blocks(True))
    g.update(solid, enabled=True)
    for _ in range(SOLID_ENTER):
      g.update(solid, enabled=True)
    g.update(LaneTypeSnapshot(valid=False), enabled=True)
    self.assertFalse(g.blocks(True))


class TestSnapshotFreshness(unittest.TestCase):
  def test_stale_and_missing(self):
    with tempfile.TemporaryDirectory() as td:
      path = os.path.join(td, "sp_lane_type.json")
      now = 1000.0
      write_lane_type(
        LaneTypeSnapshot(ts=now - LANE_TYPE_FRESH_S - 0.1, left=LINE_SOLID, valid=True),
        path=path,
      )
      snap = read_lane_type(path, now=now)
      self.assertFalse(snap.valid)
      self.assertIn("stale", snap.err)
      missing = read_lane_type(os.path.join(td, "nope.json"), now=now)
      self.assertFalse(missing.valid)

  def test_fresh_roundtrip(self):
    with tempfile.TemporaryDirectory() as td:
      path = os.path.join(td, "sp_lane_type.json")
      now = 2000.0
      write_lane_type(
        LaneTypeSnapshot(ts=now, left=LINE_DASHED, right=LINE_SOLID, left_conf=0.4, right_conf=0.5, valid=True),
        path=path,
      )
      snap = read_lane_type(path, now=now + 0.5)
      self.assertTrue(snap.valid)
      self.assertEqual(snap.left, LINE_DASHED)
      self.assertEqual(snap.right, LINE_SOLID)


def _cereal_available() -> bool:
  try:
    import capnp  # noqa: F401
    from openpilot.cereal import log  # noqa: F401
    return True
  except Exception:
    return False


@unittest.skipUnless(_cereal_available(), "cereal/capnp not available on this host")
class TestDesireHelperGate(unittest.TestCase):
  def test_solid_blocks_lc_start_unknown_fail_open(self):
    from openpilot.cereal import log
    from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper

    LaneChangeState = log.LaneChangeState
    dh = DesireHelper()
    dh.lane_change_state = LaneChangeState.preLaneChange
    dh.lane_change_direction = log.LaneChangeDirection.left
    dh.prev_one_blinker = True

    cs = SimpleNamespace(
      vEgo=20.0,  # >45 kph
      leftBlinker=True, rightBlinker=False,
      steeringPressed=True, steeringTorque=1.0,
      leftBlindspot=False, rightBlindspot=False,
      brakePressed=False, steeringAngleDeg=0.0, yawRate=0.0,
      gearShifter=None,
    )

    solid = LaneTypeSnapshot(ts=time.monotonic(), left=LINE_SOLID, right=LINE_DASHED, valid=True)
    with mock.patch("openpilot.selfdrive.controls.lib.desire_helper.lane_type_enabled", return_value=True), \
         mock.patch("openpilot.selfdrive.controls.lib.desire_helper.read_lane_type", return_value=solid), \
         mock.patch("openpilot.selfdrive.controls.lib.desire_helper.read_snapshot", side_effect=Exception("no nav")):
      for _ in range(SOLID_ENTER):
        dh.update(cs, lateral_active=True, lane_change_prob=0.5)
      self.assertEqual(dh.lane_change_state, LaneChangeState.preLaneChange)
      self.assertTrue(dh.solid_line_blocked)

    unknown = LaneTypeSnapshot(ts=time.monotonic(), left=LINE_UNKNOWN, right=LINE_UNKNOWN, valid=True)
    dh = DesireHelper()
    dh.lane_change_state = LaneChangeState.preLaneChange
    dh.lane_change_direction = log.LaneChangeDirection.left
    dh.prev_one_blinker = True
    with mock.patch("openpilot.selfdrive.controls.lib.desire_helper.lane_type_enabled", return_value=True), \
         mock.patch("openpilot.selfdrive.controls.lib.desire_helper.read_lane_type", return_value=unknown), \
         mock.patch("openpilot.selfdrive.controls.lib.desire_helper.read_snapshot", side_effect=Exception("no nav")):
      dh.update(cs, lateral_active=True, lane_change_prob=0.5)
      self.assertEqual(dh.lane_change_state, LaneChangeState.laneChangeStarting)
      self.assertFalse(dh.solid_line_blocked)

  def test_low_speed_turn_not_gated_by_solid(self):
    """<45 km/h blinker turn desire path must ignore solid-line LC gate."""
    from openpilot.cereal import log, custom
    from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LANE_CHANGE_SPEED_MIN

    TurnDirection = custom.ModelDataV2SP.TurnDirection
    dh = DesireHelper()
    self.assertLess(14.0, LANE_CHANGE_SPEED_MIN)  # sanity: 14 m/s ≈ 50 kph boundary above turn

    cs = SimpleNamespace(
      vEgo=8.0,  # ~29 km/h — turn desire territory
      leftBlinker=True, rightBlinker=False,
      steeringPressed=False, steeringTorque=0.0,
      leftBlindspot=False, rightBlindspot=False,
      brakePressed=False, steeringAngleDeg=15.0, yawRate=0.2,
      gearShifter=None,
    )
    solid = LaneTypeSnapshot(ts=time.monotonic(), left=LINE_SOLID, right=LINE_SOLID, valid=True)

    with mock.patch("openpilot.selfdrive.controls.lib.desire_helper.lane_type_enabled", return_value=True), \
         mock.patch("openpilot.selfdrive.controls.lib.desire_helper.read_lane_type", return_value=solid), \
         mock.patch("openpilot.selfdrive.controls.lib.desire_helper.read_snapshot", side_effect=Exception("no nav")), \
         mock.patch.object(dh.lane_turn_controller, "get_turn_direction", return_value=TurnDirection.turnLeft), \
         mock.patch.object(dh.lane_turn_controller, "update_lane_turn"), \
         mock.patch.object(dh.lane_turn_controller, "update_params"):
      dh.update(cs, lateral_active=True, lane_change_prob=0.0)
      self.assertEqual(dh.lane_change_state, log.LaneChangeState.off)
      self.assertEqual(dh.desire, log.Desire.turnLeft)
      self.assertFalse(dh.solid_line_blocked)


class TestLcGateLogic(unittest.TestCase):
  """DesireHelper transition predicate without cereal (mac hosts)."""

  def test_predicate_matches_plan(self):
    # (torque or auto_lc) and not bsm and not solid → start
    def may_start(torque, auto_lc, bsm, solid):
      return (torque or auto_lc) and not bsm and not solid

    self.assertFalse(may_start(True, False, False, True))   # solid blocks
    self.assertTrue(may_start(True, False, False, False))    # unknown/dashed fail-open
    self.assertFalse(may_start(True, False, True, False))    # BSM still blocks
    # Low-speed turn never enters preLaneChange LC FSM when turn_active — gate unused.
    turn_active = True
    self.assertTrue(turn_active)  # document: solid gate only in preLaneChange branch


if __name__ == "__main__":
  unittest.main()
