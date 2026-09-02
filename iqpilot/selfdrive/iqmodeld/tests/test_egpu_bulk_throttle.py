"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""
from iqpilot.selfdrive.iqmodeld.egpu_helpers import (
  _bulk_tuning,
  throttle_usbgpu_bulk_reads,
  throttle_usbgpu_bulk_writes,
  wait_for_stable_usb_link,
)


def test_throttle_splits_large_bulk_write(monkeypatch):
  calls: list[int] = []

  class USB3:
    def bulk_write(self, payload, timeout=1000):
      calls.append(len(payload))

  monkeypatch.setattr("iqpilot.selfdrive.iqmodeld.egpu_helpers.iqos_linux49", lambda: True)
  assert throttle_usbgpu_bulk_writes(usb3_cls=USB3, chunk=4, pause_s=0)
  USB3().bulk_write(b"abcdefgh")
  assert calls == [4, 4]


def test_throttle_splits_large_bulk_read(monkeypatch):
  calls: list[int] = []

  class USB3:
    def bulk_read(self, length, timeout=1000):
      calls.append(length)
      return memoryview(b"x" * length)

  monkeypatch.setattr("iqpilot.selfdrive.iqmodeld.egpu_helpers.iqos_linux49", lambda: True)
  assert throttle_usbgpu_bulk_reads(usb3_cls=USB3, chunk=4, pause_s=0)
  out = USB3().bulk_read(8)
  assert bytes(out) == b"xxxxxxxx"
  assert calls == [4, 4]


def test_throttle_skips_non_linux49(monkeypatch):
  class USB3:
    def bulk_write(self, payload, timeout=1000):
      pass

    def bulk_read(self, length, timeout=1000):
      return memoryview(b"")

  monkeypatch.setattr("iqpilot.selfdrive.iqmodeld.egpu_helpers.iqos_linux49", lambda: False)
  assert throttle_usbgpu_bulk_writes(usb3_cls=USB3) is False
  assert throttle_usbgpu_bulk_reads(usb3_cls=USB3) is False


def test_wait_for_stable_link_non_linux49(monkeypatch):
  monkeypatch.setattr("iqpilot.selfdrive.iqmodeld.egpu_helpers.iqos_linux49", lambda: False)
  assert wait_for_stable_usb_link(settle_s=0.0, poll_s=0.0) is True


def test_bulk_tuning_env(monkeypatch):
  monkeypatch.setenv("IQ_EGPU_BULK_CHUNK", "65536")
  monkeypatch.setenv("IQ_EGPU_BULK_PAUSE_MS", "10")
  monkeypatch.setenv("IQ_EGPU_HCQ_WAIT_MS", "90000")
  chunk, pause_s, settle_s, hcq_ms, copyout_chunk, copyout_pause_s = _bulk_tuning()
  assert chunk == 65536
  assert pause_s == 0.01
  assert hcq_ms == 90000
  assert settle_s == 3.0
  assert copyout_chunk == 65536
