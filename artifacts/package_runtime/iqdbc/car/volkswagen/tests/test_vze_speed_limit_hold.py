"""Camera VZE glitch filter: large jumps and ego-context nonsense need a short hold."""
from types import SimpleNamespace
from unittest.mock import patch

from iqdbc.car.common.conversions import Conversions as CV
from iqdbc.car.volkswagen.speed_limit_manager import SpeedLimitManager, VZE_HOLD_S
from iqdbc.car.volkswagen.values import VolkswagenFlags


def _mgr():
  return SpeedLimitManager(SimpleNamespace(flags=VolkswagenFlags.MEB))


def _feed(mgr, kph, v_ego_kph, t_mono):
  with patch("iqdbc.car.volkswagen.speed_limit_manager.time.monotonic", return_value=t_mono):
    mgr._receive_speed_limit_vze_meb(
      {"VZE_Verkehrszeichen_1": kph, "VZE_Anzeigemodus": 3},
      v_ego_kph * CV.KPH_TO_MS,
    )
    return mgr.get_speed_limit() * CV.MS_TO_KPH


def test_small_change_applies_immediately():
  mgr = _mgr()
  assert abs(_feed(mgr, 60, 50, 0.0) - 60) < 1e-6
  assert abs(_feed(mgr, 70, 55, 0.1) - 70) < 1e-6


def test_highway_false_40_flash_keeps_prior_limit():
  mgr = _mgr()
  assert abs(_feed(mgr, 120, 110, 0.0) - 120) < 1e-6
  # Brief false urban sign while still at highway speed.
  assert abs(_feed(mgr, 40, 110, 0.2) - 120) < 1e-6
  assert mgr.v_limit_vze_sanity_error is True
  # Flash gone — back to 120 without waiting full hold.
  assert abs(_feed(mgr, 120, 110, 0.4) - 120) < 1e-6


def test_city_false_120_flash_keeps_prior_limit():
  mgr = _mgr()
  assert abs(_feed(mgr, 40, 40, 0.0) - 40) < 1e-6
  assert abs(_feed(mgr, 120, 45, 0.2) - 40) < 1e-6
  assert mgr.v_limit_vze_sanity_error is True
  assert abs(_feed(mgr, 40, 45, 0.4) - 40) < 1e-6


def test_real_ramp_down_accepted_after_hold():
  mgr = _mgr()
  assert abs(_feed(mgr, 120, 100, 0.0) - 120) < 1e-6
  # Continuous 40 while leaving the highway — still held at first.
  assert abs(_feed(mgr, 40, 85, 0.2) - 120) < 1e-6
  assert abs(_feed(mgr, 40, 80, 0.2 + VZE_HOLD_S - 0.05) - 120) < 1e-6
  # After hold window, accept the real 40.
  assert abs(_feed(mgr, 40, 75, 0.2 + VZE_HOLD_S + 0.05) - 40) < 1e-6
  assert mgr.v_limit_vze_sanity_error is False


def test_real_highway_entry_accepted_after_hold():
  mgr = _mgr()
  assert abs(_feed(mgr, 60, 50, 0.0) - 60) < 1e-6
  assert abs(_feed(mgr, 120, 55, 0.2) - 60) < 1e-6
  assert abs(_feed(mgr, 120, 70, 0.2 + VZE_HOLD_S + 0.05) - 120) < 1e-6


def test_highway_ego_context_holds_without_prior_limit():
  mgr = _mgr()
  # First reading is an implausible urban sign at highway speed.
  assert _feed(mgr, 40, 100, 0.0) == 0
  assert mgr.v_limit_vze_sanity_error is True
  assert _feed(mgr, 40, 100, 0.5) == 0
  # After hold, accept (could be a real low limit on a slow highway stretch).
  assert abs(_feed(mgr, 40, 100, VZE_HOLD_S + 0.1) - 40) < 1e-6
