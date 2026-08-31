from iqpilot.ui.onroad.display_speed_limit import min_display_speed_limit_mps, LIMIT_MIN_SPEED_MS


def test_min_picks_lowest_valid():
  # 100 / 80 / 60 kph
  got = min_display_speed_limit_mps(100 / 3.6, 80 / 3.6, 60 / 3.6)
  assert got is not None
  assert abs(got - 60 / 3.6) < 1e-6


def test_min_ignores_zero_and_below_floor():
  got = min_display_speed_limit_mps(0.0, LIMIT_MIN_SPEED_MS - 0.1, 50 / 3.6)
  assert got is not None
  assert abs(got - 50 / 3.6) < 1e-6


def test_min_none_when_empty():
  assert min_display_speed_limit_mps(0.0, 0.0, 0.0) is None


def test_cruise_mismatch_policy_op_long_silent():
  # Mirrors selfdrived: only pcmCruise + car on + OP off.
  def is_cruise_mismatch(pcm_cruise, car_enabled, op_enabled):
    return bool(pcm_cruise and car_enabled and not op_enabled)

  assert not is_cruise_mismatch(False, True, True)   # MEB OP long engaged
  assert not is_cruise_mismatch(False, True, False)  # MEB OP long, OP off (no PCM mismatch alert)
  assert is_cruise_mismatch(True, True, False)       # PCM car cruise without OP
  assert not is_cruise_mismatch(True, True, True)    # PCM + OP engaged
  assert not is_cruise_mismatch(True, False, False)
