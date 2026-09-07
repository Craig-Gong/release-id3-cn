from opendbc.car.volkswagen.mebcan import meb_pid_hold_should_start


def test_pid_hold_negative_accel_stays_held():
  assert meb_pid_hold_should_start(True, 0.0, -0.4, False) is False


def test_pid_hold_positive_accel_releases():
  assert meb_pid_hold_should_start(True, 0.0, 1.8, False) is True


def test_gas_always_releases():
  assert meb_pid_hold_should_start(True, 0.0, -0.4, True) is True


def test_not_held_does_not_start():
  assert meb_pid_hold_should_start(False, 5.0, 1.8, False) is False
