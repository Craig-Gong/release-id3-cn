import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from openpilot.sunnypilot.modeld_v2.egpu_loader import (
  C3XL_DOCK_SETTLE, C3XL_DOCK_UNKNOWN_GRACE, C3XL_DOCK_WAIT_TIMEOUT,
  DOCK_ABSENT, DOCK_NOT_READY, DOCK_READY, DOCK_UNKNOWN,
  clear_chestnut_dock_decided, clear_chestnut_dock_seen, dock_usable, mark_chestnut_dock_decided,
  mark_chestnut_dock_seen, should_wait_for_dock, wait_for_chestnut_dock,
)


class FakeClock:
  def __init__(self):
    self.t = 0.0

  def monotonic(self):
    return self.t

  def sleep(self, dt):
    self.t += dt


def _wait(probe, clock):
  return wait_for_chestnut_dock(probe, sleep=clock.sleep, monotonic=clock.monotonic)


def _timeline(clock, *steps):
  """steps: (start_time, state) sorted; the last one at or before now wins."""
  def probe():
    state = steps[0][1]
    for t, s in steps:
      if clock.t >= t:
        state = s
    return state
  return probe


class TestDockWait(unittest.TestCase):
  def test_ready_at_start_has_no_delay(self):
    clock = FakeClock()
    self.assertEqual(_wait(lambda: DOCK_READY, clock), DOCK_READY)
    self.assertEqual(clock.t, 0.0)

  def test_dock_enumerating_after_12v_then_link_up(self):
    clock = FakeClock()
    probe = _timeline(clock, (0, DOCK_ABSENT), (6.0, DOCK_NOT_READY), (8.0, DOCK_READY))
    self.assertEqual(_wait(probe, clock), DOCK_READY)
    self.assertGreaterEqual(clock.t, 8.0 + C3XL_DOCK_SETTLE)
    self.assertLess(clock.t, 9.0 + C3XL_DOCK_SETTLE)

  def test_bridge_on_usb_power_waits_for_gpu_link(self):
    # ASM enumerates on USB-C alone; the GPU is unusable until PCIe reaches L0.
    clock = FakeClock()
    probe = _timeline(clock, (0, DOCK_NOT_READY), (12.0, DOCK_READY))
    self.assertEqual(_wait(probe, clock), DOCK_READY)
    self.assertGreaterEqual(clock.t, 12.0)

  def test_absent_times_out(self):
    clock = FakeClock()
    state = _wait(lambda: DOCK_ABSENT, clock)
    self.assertEqual(state, DOCK_ABSENT)
    self.assertFalse(dock_usable(state))
    self.assertGreaterEqual(clock.t, C3XL_DOCK_WAIT_TIMEOUT)
    self.assertLess(clock.t, C3XL_DOCK_WAIT_TIMEOUT + 1.0)

  def test_present_but_never_ready_is_not_loaded(self):
    clock = FakeClock()
    state = _wait(lambda: DOCK_NOT_READY, clock)
    self.assertEqual(state, DOCK_NOT_READY)
    self.assertFalse(dock_usable(state))

  def test_unreadable_link_loads_after_grace(self):
    clock = FakeClock()
    state = _wait(lambda: DOCK_UNKNOWN, clock)
    self.assertEqual(state, DOCK_UNKNOWN)
    self.assertTrue(dock_usable(state))
    self.assertGreaterEqual(clock.t, C3XL_DOCK_UNKNOWN_GRACE)
    self.assertLess(clock.t, C3XL_DOCK_UNKNOWN_GRACE + 1.0)

  def test_unknown_then_readable_l0_prefers_ready(self):
    clock = FakeClock()
    probe = _timeline(clock, (0, DOCK_UNKNOWN), (3.0, DOCK_READY))
    self.assertEqual(_wait(probe, clock), DOCK_READY)


class TestDockWaitGate(unittest.TestCase):
  def setUp(self):
    self.tmp = TemporaryDirectory()
    self.decided = str(Path(self.tmp.name) / "decided")
    self.seen = str(Path(self.tmp.name) / "seen")

  def tearDown(self):
    self.tmp.cleanup()

  def _should(self, present):
    return should_wait_for_dock(present, decided_path=self.decided, seen_path=self.seen)

  def test_never_seen_dock_does_not_wait(self):
    self.assertFalse(self._should(False))

  def test_present_dock_checks_link(self):
    self.assertTrue(self._should(True))

  def test_seen_dock_missing_at_ready_waits(self):
    mark_chestnut_dock_seen(self.seen)
    self.assertTrue(self._should(False))

  def test_unplugged_dock_costs_one_wait(self):
    mark_chestnut_dock_seen(self.seen)
    clear_chestnut_dock_seen(self.seen)
    self.assertFalse(self._should(False))

  def test_restarted_modeld_in_same_onroad_never_waits(self):
    mark_chestnut_dock_seen(self.seen)
    mark_chestnut_dock_decided(self.decided)
    self.assertFalse(self._should(False))
    self.assertFalse(self._should(True))
    clear_chestnut_dock_decided(self.decided)
    self.assertTrue(self._should(False))

  def test_marker_write_failure_is_ignored(self):
    mark_chestnut_dock_seen(str(Path(self.tmp.name) / "missing_dir" / "seen"))
    mark_chestnut_dock_decided(str(Path(self.tmp.name) / "missing_dir" / "decided"))


if __name__ == "__main__":
  unittest.main()
