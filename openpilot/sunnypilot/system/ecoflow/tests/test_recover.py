#!/usr/bin/env python3
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from openpilot.sunnypilot.system.ecoflow.recover import (
  GpuRecoverCycle, RECOVER_OFF_S, RECOVER_ON_SETTLE_S, RecoverPhase,
  clear_recover_request, recover_allowed, recover_request_pending, request_recover,
)


class TestRecoverAllowed(unittest.TestCase):
  def test_offroad_always_ok(self):
    self.assertTrue(recover_allowed(started=False, engaged=False, v_ego=20.0))

  def test_engaged_blocked(self):
    self.assertFalse(recover_allowed(started=True, engaged=True, v_ego=0.0))

  def test_parked_onroad_ok(self):
    self.assertTrue(recover_allowed(started=True, engaged=False, v_ego=0.1))

  def test_moving_blocked(self):
    self.assertFalse(recover_allowed(started=True, engaged=False, v_ego=2.0))


class TestGpuRecoverCycle(unittest.TestCase):
  def test_off_then_on_then_done(self):
    c = GpuRecoverCycle(off_s=10.0, on_settle_s=5.0)
    self.assertFalse(c.active)
    c.start(100.0)
    self.assertTrue(c.active)
    self.assertIs(c.phase, RecoverPhase.power_off)
    self.assertIsNone(c.tick(105.0))
    self.assertEqual(c.tick(110.0), "on")
    self.assertIs(c.phase, RecoverPhase.power_on)
    self.assertIsNone(c.tick(112.0))
    self.assertEqual(c.tick(115.0), "done")
    self.assertFalse(c.active)

  def test_defaults_match_constants(self):
    c = GpuRecoverCycle()
    c.start(0.0)
    self.assertEqual(c.deadline, RECOVER_OFF_S)
    self.assertEqual(c.tick(RECOVER_OFF_S), "on")
    self.assertEqual(c.deadline, RECOVER_OFF_S + RECOVER_ON_SETTLE_S)

  def test_cancel(self):
    c = GpuRecoverCycle()
    c.start(0.0)
    c.cancel()
    self.assertFalse(c.active)
    self.assertIsNone(c.tick(100.0))


class TestRecoverRequestFlag(unittest.TestCase):
  def test_file_flag_roundtrip(self):
    with tempfile.TemporaryDirectory() as td:
      flag = Path(td) / "ecoflow_gpu_recover"
      with mock.patch("openpilot.sunnypilot.system.ecoflow.recover.RECOVER_REQUEST_PATH", flag):
        self.assertFalse(recover_request_pending(lambda _k: False))
        request_recover()
        self.assertTrue(flag.exists())
        self.assertTrue(recover_request_pending(lambda _k: False))
        clear_recover_request()
        self.assertFalse(flag.exists())
        self.assertFalse(recover_request_pending(lambda _k: False))

  def test_param_alone_is_enough(self):
    self.assertTrue(recover_request_pending(lambda k: k == "EcoflowGpuRecover"))


if __name__ == "__main__":
  unittest.main()
