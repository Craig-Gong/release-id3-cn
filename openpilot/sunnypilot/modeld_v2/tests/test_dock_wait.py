import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from openpilot.sunnypilot.modeld_v2.egpu_loader import (
  C3XL_DOCK_MAX_TIMEOUT, C3XL_DOCK_OFFLINE_TIMEOUT, C3XL_DOCK_POWER_PENDING_TIMEOUT, C3XL_DOCK_POWERED_TIMEOUT,
  C3XL_DOCK_SETTLE, C3XL_DOCK_UNKNOWN_GRACE, C3XL_DOCK_WAIT_TIMEOUT,
  DOCK_ABSENT, DOCK_NOT_READY, DOCK_READY, DOCK_UNKNOWN, POWER_OFFLINE, POWER_ON, POWER_PENDING, POWER_UNKNOWN,
  clear_chestnut_dock_decided, clear_chestnut_dock_seen, dock_usable, ecoflow_dock_power,
  keep_dock_seen_after_wait, mark_chestnut_dock_absent, mark_chestnut_dock_decided,
  mark_chestnut_dock_seen, should_wait_for_dock, wait_for_chestnut_dock,
)
from openpilot.sunnypilot.system.ecoflow.status import EcoflowStatus


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


def _wait_powered(probe, power, clock):
  return wait_for_chestnut_dock(probe, power=power, sleep=clock.sleep, monotonic=clock.monotonic)


class TestDockWaitWithRail(unittest.TestCase):
  def test_cold_morning_slow_mqtt_still_loads(self):
    # 2026-10-07 09:24: 12 V confirmed ~7 s after KL15, dock ~5 s later.
    clock = FakeClock()
    power = _timeline(clock, (0, POWER_PENDING), (7.0, POWER_ON))
    probe = _timeline(clock, (0, DOCK_ABSENT), (12.0, DOCK_NOT_READY), (13.0, DOCK_READY))
    self.assertEqual(_wait_powered(probe, power, clock), DOCK_READY)

  def test_rail_slower_than_flat_timeout_still_loads(self):
    clock = FakeClock()
    power = _timeline(clock, (0, POWER_PENDING), (38.0, POWER_ON))
    probe = _timeline(clock, (0, DOCK_ABSENT), (43.0, DOCK_READY))
    self.assertEqual(_wait_powered(probe, power, clock), DOCK_READY)

  def test_rail_never_confirmed_gives_up_at_pending_cap(self):
    clock = FakeClock()
    state = _wait_powered(lambda: DOCK_ABSENT, lambda: POWER_PENDING, clock)
    self.assertEqual(state, DOCK_ABSENT)
    self.assertGreaterEqual(clock.t, C3XL_DOCK_POWER_PENDING_TIMEOUT)
    self.assertLess(clock.t, C3XL_DOCK_POWER_PENDING_TIMEOUT + 1.0)

  def test_powered_but_no_dock_gives_up_after_powered_window(self):
    clock = FakeClock()
    state = _wait_powered(lambda: DOCK_ABSENT, lambda: POWER_ON, clock)
    self.assertEqual(state, DOCK_ABSENT)
    self.assertGreaterEqual(clock.t, C3XL_DOCK_POWERED_TIMEOUT)
    self.assertLess(clock.t, C3XL_DOCK_POWERED_TIMEOUT + 1.0)

  def test_late_rail_is_capped(self):
    clock = FakeClock()
    power = _timeline(clock, (0, POWER_PENDING), (44.0, POWER_ON))
    state = _wait_powered(lambda: DOCK_ABSENT, power, clock)
    self.assertEqual(state, DOCK_ABSENT)
    self.assertGreaterEqual(clock.t, C3XL_DOCK_MAX_TIMEOUT)
    self.assertLess(clock.t, C3XL_DOCK_MAX_TIMEOUT + 1.0)

  def test_no_ecoflow_keeps_flat_timeout(self):
    clock = FakeClock()
    _wait_powered(lambda: DOCK_ABSENT, lambda: POWER_UNKNOWN, clock)
    self.assertGreaterEqual(clock.t, C3XL_DOCK_WAIT_TIMEOUT)
    self.assertLess(clock.t, C3XL_DOCK_WAIT_TIMEOUT + 1.0)

  def test_no_mqtt_all_day_gives_up_at_offline_cap(self):
    clock = FakeClock()
    state = _wait_powered(lambda: DOCK_ABSENT, lambda: POWER_OFFLINE, clock)
    self.assertEqual(state, DOCK_ABSENT)
    self.assertGreaterEqual(clock.t, C3XL_DOCK_OFFLINE_TIMEOUT)
    self.assertLess(clock.t, C3XL_DOCK_OFFLINE_TIMEOUT + 1.0)

  def test_mqtt_logging_in_extends_to_pending_cap(self):
    # 2026-10-07 09:24: MQTT up ~6 s after READY, 12 V a second later.
    clock = FakeClock()
    power = _timeline(clock, (0, POWER_OFFLINE), (6.0, POWER_PENDING), (7.0, POWER_ON))
    probe = _timeline(clock, (0, DOCK_ABSENT), (12.0, DOCK_READY))
    self.assertEqual(_wait_powered(probe, power, clock), DOCK_READY)

  def test_mqtt_flapping_after_login_keeps_pending_cap(self):
    clock = FakeClock()
    power = _timeline(clock, (0, POWER_PENDING), (25.0, POWER_OFFLINE))
    _wait_powered(lambda: DOCK_ABSENT, power, clock)
    self.assertGreaterEqual(clock.t, C3XL_DOCK_POWER_PENDING_TIMEOUT)
    self.assertLess(clock.t, C3XL_DOCK_POWER_PENDING_TIMEOUT + 1.0)

  def test_on_change_logs_each_transition_once(self):
    clock = FakeClock()
    power = _timeline(clock, (0, POWER_PENDING), (3.0, POWER_ON))
    probe = _timeline(clock, (0, DOCK_ABSENT), (5.0, DOCK_NOT_READY), (6.0, DOCK_READY))
    seen = []
    wait_for_chestnut_dock(probe, power=power, on_change=lambda dt, d, r: seen.append((dt, d, r)),
                           sleep=clock.sleep, monotonic=clock.monotonic)
    self.assertEqual([(d, r) for _, d, r in seen],
                     [(DOCK_ABSENT, POWER_PENDING), (DOCK_ABSENT, POWER_ON), (DOCK_NOT_READY, POWER_ON)])
    self.assertEqual([dt for dt, _, _ in seen], [0.0, 3.0, 5.0])

  def test_ready_at_start_logs_nothing(self):
    seen = []
    wait_for_chestnut_dock(lambda: DOCK_READY, power=lambda: POWER_ON, on_change=lambda *a: seen.append(a))
    self.assertEqual(seen, [])

  def test_ecoflowd_dropping_out_after_confirming_keeps_powered_window(self):
    clock = FakeClock()
    power = _timeline(clock, (0, POWER_ON), (3.0, POWER_UNKNOWN))
    _wait_powered(lambda: DOCK_ABSENT, power, clock)
    self.assertGreaterEqual(clock.t, C3XL_DOCK_POWERED_TIMEOUT)
    self.assertLess(clock.t, C3XL_DOCK_POWERED_TIMEOUT + 1.0)


class TestEcoflowDockPower(unittest.TestCase):
  def _power(self, status, now=100.0):
    return ecoflow_dock_power(lambda: status, monotonic=lambda: now)

  def test_states(self):
    self.assertEqual(self._power(EcoflowStatus(ts=99.0, enabled=True, mqtt=True, dc12v=True)), POWER_ON)
    self.assertEqual(self._power(EcoflowStatus(ts=99.0, enabled=True, mqtt=True, dc12v=None)), POWER_PENDING)
    self.assertEqual(self._power(EcoflowStatus(ts=99.0, enabled=True, mqtt=True, dc12v=False)), POWER_PENDING)
    self.assertEqual(self._power(EcoflowStatus(ts=99.0, enabled=True, mqtt=False, dc12v=False)), POWER_OFFLINE)
    self.assertEqual(self._power(EcoflowStatus(ts=99.0, enabled=True, mqtt=False, dc12v=None)), POWER_OFFLINE)
    # Last telemetry said on before the session dropped: the rail is still up.
    self.assertEqual(self._power(EcoflowStatus(ts=99.0, enabled=True, mqtt=False, dc12v=True)), POWER_ON)

  def test_disabled_stale_or_missing_is_unknown(self):
    self.assertEqual(self._power(EcoflowStatus(ts=99.0, enabled=False, dc12v=True)), POWER_UNKNOWN)
    self.assertEqual(self._power(EcoflowStatus(ts=50.0, enabled=True, dc12v=True)), POWER_UNKNOWN)
    self.assertEqual(self._power(EcoflowStatus()), POWER_UNKNOWN)

  def test_read_failure_is_unknown(self):
    def boom():
      raise OSError("no shm")
    self.assertEqual(ecoflow_dock_power(boom), POWER_UNKNOWN)

  def test_unplugged_only_when_rail_not_pending(self):
    self.assertTrue(keep_dock_seen_after_wait(DOCK_ABSENT, POWER_PENDING))
    self.assertTrue(keep_dock_seen_after_wait(DOCK_ABSENT, POWER_OFFLINE))
    self.assertFalse(keep_dock_seen_after_wait(DOCK_ABSENT, POWER_ON))
    self.assertFalse(keep_dock_seen_after_wait(DOCK_ABSENT, POWER_UNKNOWN))
    self.assertTrue(keep_dock_seen_after_wait(DOCK_NOT_READY, POWER_ON))
    self.assertTrue(keep_dock_seen_after_wait(DOCK_READY, POWER_ON))


class TestDockWaitGate(unittest.TestCase):
  def setUp(self):
    self.tmp = TemporaryDirectory()
    self.decided = str(Path(self.tmp.name) / "decided")
    self.seen = str(Path(self.tmp.name) / "seen")
    self.absent = str(Path(self.tmp.name) / "absent")

  def tearDown(self):
    self.tmp.cleanup()

  def _should(self, present, power_expected=False):
    return should_wait_for_dock(present, power_expected=power_expected, decided_path=self.decided,
                                seen_path=self.seen, absent_path=self.absent)

  def test_first_ever_start_waits_when_ecoflow_powers_the_dock(self):
    self.assertTrue(self._should(False, power_expected=True))

  def test_confirmed_unplugged_dock_stops_ecoflow_waits_until_seen_again(self):
    mark_chestnut_dock_absent(self.absent)
    self.assertFalse(self._should(False, power_expected=True))
    self.assertTrue(self._should(True, power_expected=True))
    mark_chestnut_dock_seen(self.seen, absent_path=self.absent)
    self.assertFalse(Path(self.absent).exists())
    clear_chestnut_dock_seen(self.seen)
    self.assertTrue(self._should(False, power_expected=True))

  def test_never_seen_dock_does_not_wait(self):
    self.assertFalse(self._should(False))

  def test_present_dock_checks_link(self):
    self.assertTrue(self._should(True))

  def test_seen_dock_missing_at_ready_waits(self):
    mark_chestnut_dock_seen(self.seen, absent_path=self.absent)
    self.assertTrue(self._should(False))

  def test_unplugged_dock_costs_one_wait(self):
    mark_chestnut_dock_seen(self.seen, absent_path=self.absent)
    clear_chestnut_dock_seen(self.seen)
    self.assertFalse(self._should(False))

  def test_restarted_modeld_in_same_onroad_never_waits(self):
    mark_chestnut_dock_seen(self.seen, absent_path=self.absent)
    mark_chestnut_dock_decided(self.decided)
    self.assertFalse(self._should(False))
    self.assertFalse(self._should(True))
    clear_chestnut_dock_decided(self.decided)
    self.assertTrue(self._should(False))

  def test_marker_write_failure_is_ignored(self):
    mark_chestnut_dock_seen(str(Path(self.tmp.name) / "missing_dir" / "seen"), absent_path=self.absent)
    mark_chestnut_dock_absent(str(Path(self.tmp.name) / "missing_dir" / "absent"))
    mark_chestnut_dock_decided(str(Path(self.tmp.name) / "missing_dir" / "decided"))


if __name__ == "__main__":
  unittest.main()
