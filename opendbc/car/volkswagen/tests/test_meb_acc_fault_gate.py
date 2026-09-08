"""MEB Cruise Fault gate: ignore TSK 6/7 until settled in D."""
from opendbc.car.volkswagen.carstate import CarState


class _CS:
  """Minimal stand-in — only update_acc_fault + timers."""

  def __init__(self):
    self.frame = 0
    self.tsk_recovery_timer = 0

  update_acc_fault = CarState.update_acc_fault


def test_not_drive_masks_fault():
  cs = _CS()
  assert cs.update_acc_fault(True, False, False, drive_mode=False) is False
  assert cs.tsk_recovery_timer == 0


def test_parking_brake_masks_fault():
  cs = _CS()
  assert cs.update_acc_fault(True, False, False, drive_mode=True, parking_brake=True) is False


def test_settle_after_drive_about_3s():
  cs = _CS()
  # leave P: arm timer
  cs.update_acc_fault(True, False, False, drive_mode=False)
  # enter D at frame 0; fault still masked for 300 frames
  for cs.frame in range(0, 300):
    assert cs.update_acc_fault(True, False, False, drive_mode=True) is False
  cs.frame = 300
  assert cs.update_acc_fault(True, False, False, drive_mode=True) is True
  assert cs.update_acc_fault(False, False, False, drive_mode=True) is False


def test_engine_off_and_inhibit_still_mask():
  cs = _CS()
  cs.frame = 500
  assert cs.update_acc_fault(True, True, False, drive_mode=True) is False
  assert cs.update_acc_fault(True, False, True, drive_mode=True) is False
