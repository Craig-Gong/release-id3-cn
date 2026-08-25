"""MEB TSK=6/7 at standstill must not raise Cruise Faulted after a prior engage."""
from iqdbc.car.volkswagen.carstate import CarState


def test_tsk6_at_standstill_keeps_long_without_fault():
  faulted, available, enabled, was = CarState.meb_tsk_cruise_flags(
    tsk_status=3, standstill=False, esp_hold=False, was_enabled=False
  )
  assert (faulted, available, enabled, was) == (False, True, True, True)

  faulted, available, enabled, was = CarState.meb_tsk_cruise_flags(
    tsk_status=6, standstill=True, esp_hold=False, was_enabled=was
  )
  assert faulted is False
  assert available is True
  assert enabled is True
  assert was is True


def test_tsk6_while_moving_is_still_fault():
  faulted, available, enabled, was = CarState.meb_tsk_cruise_flags(
    tsk_status=6, standstill=False, esp_hold=False, was_enabled=True
  )
  assert faulted is True
  assert available is False
  assert enabled is False
  assert was is True


def test_tsk7_at_standstill_after_op_long_ok():
  faulted, available, enabled, was = CarState.meb_tsk_cruise_flags(
    tsk_status=3, standstill=True, esp_hold=True, was_enabled=False
  )
  assert was is True
  faulted, available, enabled, was = CarState.meb_tsk_cruise_flags(
    tsk_status=7, standstill=True, esp_hold=True, was_enabled=was
  )
  assert faulted is False
  assert available is True
  assert enabled is True


def test_tsk7_creep_after_hold_drop_not_fault():
  faulted, available, enabled, was = CarState.meb_tsk_cruise_flags(
    tsk_status=3, standstill=True, esp_hold=True, was_enabled=False
  )
  assert was is True
  faulted, available, enabled, was = CarState.meb_tsk_cruise_flags(
    tsk_status=7, standstill=False, esp_hold=False, was_enabled=was,
    near_standstill=True,
  )
  assert faulted is False
  assert available is True
  assert enabled is True


def test_tsk6_after_hard_brake_release_not_fault():
  faulted, available, enabled, was = CarState.meb_tsk_cruise_flags(
    tsk_status=6, standstill=False, esp_hold=False, was_enabled=False,
    driver_braking=True,
  )
  assert faulted is False
  assert available is True


def test_not_d_still_masks_fault_via_update_acc_fault():
  # 4bf1793: P / not-D must not latch Cruise Fault even if TSK 6/7 while rolling.
  cs = CarState.__new__(CarState)
  cs.frame = 500
  cs.cruise_recovery_timer = 0
  assert cs.update_acc_fault(True, parking_brake=False, drive_mode=False) is False
  assert cs.update_acc_fault(True, parking_brake=True, drive_mode=True) is False
