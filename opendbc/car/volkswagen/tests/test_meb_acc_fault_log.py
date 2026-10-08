"""MEB Cruise Fault diagnostics: log raw TSK 6/7 / AEB-unavailable edges only."""
from unittest import mock

from opendbc.car.volkswagen import carstate
from opendbc.car.volkswagen.carstate import CarState


class _CS:
  ACC_FAULT_LOG_MAX = CarState.ACC_FAULT_LOG_MAX

  def __init__(self):
    self.frame = 0
    self.esp_hold_confirmation = False
    self.acc_fault_raw_frame = None
    self.esp_hold_frame = None
    self.acc_fault_log_count = 0

  log_acc_fault_edges = CarState.log_acc_fault_edges


def _run(cs, frames):
  """frames: iterable of (esp_hold, raw_fault); returns logged events."""
  with mock.patch.object(carstate, "carlog") as log:
    for esp_hold, raw in frames:
      cs.esp_hold_confirmation = esp_hold
      cs.log_acc_fault_edges(raw, {"tsk": 6 if raw else 3})
      cs.frame += 1
  return [c.args[0] for c in log.warning.call_args_list]


def test_logs_start_and_end_with_hold_and_duration():
  cs = _CS()
  # 50 s held at standstill, then a 1 s fault, then clear
  ev = _run(cs, [(True, False)] * 5000 + [(True, True)] * 100 + [(True, False)])
  assert [e["event"] for e in ev] == ["meb_acc_fault_raw_start", "meb_acc_fault_raw_end"]
  assert ev[0]["esp_hold_s"] == 50.0
  assert ev[0]["tsk"] == 6
  assert ev[1]["dur_s"] == 1.0
  assert ev[1]["tsk"] == 3


def test_no_log_without_fault_edges():
  cs = _CS()
  assert _run(cs, [(True, False)] * 300 + [(False, False)] * 300) == []


def test_hold_resets_when_moving():
  cs = _CS()
  ev = _run(cs, [(True, False)] * 300 + [(False, False)] + [(False, True)])
  assert ev[0]["esp_hold_s"] is None


def test_flapping_is_capped():
  cs = _CS()
  ev = _run(cs, [(False, bool(i % 2)) for i in range(1000)])
  assert len(ev) == CarState.ACC_FAULT_LOG_MAX
