from __future__ import annotations

import os
from pathlib import Path

import pytest

from openpilot.iqpilot.selfdrive.iqmodeld.runtime import usbgpu


def _fake_usb_device(root: Path, vid: str, pid: str) -> None:
  node = root / "1-1"
  node.mkdir()
  (node / "idVendor").write_text(f"{vid}\n")
  (node / "idProduct").write_text(f"{pid}\n")


def test_usbgpu_present_detects_tinygrad_dock(tmp_path: Path):
  _fake_usb_device(tmp_path, "add1", "0001")
  assert usbgpu.usbgpu_present(tmp_path)


def test_usbgpu_present_ignores_other_usb(tmp_path: Path):
  _fake_usb_device(tmp_path, "1d6b", "0003")
  assert not usbgpu.usbgpu_present(tmp_path)


def test_usbgpu_present_missing_sysfs(tmp_path: Path):
  assert not usbgpu.usbgpu_present(tmp_path / "does-not-exist")


@pytest.mark.parametrize("value, enabled", [
  (None, False),
  ("1", True),
  ("", True),
  ("0", False),
  ("false", False),
])
def test_usbgpu_enabled_env(monkeypatch, value, enabled):
  monkeypatch.delenv("USBGPU", raising=False)
  if value is not None:
    monkeypatch.setenv("USBGPU", value)
  assert usbgpu.usbgpu_enabled() is enabled


def test_tinygrad_backend_name_strips_usb_iface():
  assert usbgpu.tinygrad_backend_name("USB+AMD") == "AMD"
  assert usbgpu.tinygrad_backend_name("QCOM") == "QCOM"
  assert usbgpu.tinygrad_backend_name("CPU") == "CPU"
  assert usbgpu.tinygrad_backend_name("") is None


def test_backend_matches_captured_usb_amd():
  assert usbgpu.backend_matches_captured({"AMD", "NPY"}, "USB+AMD")
  assert not usbgpu.backend_matches_captured({"QCOM", "NPY"}, "USB+AMD")
  assert usbgpu.backend_matches_captured({"QCOM", "NPY"}, "QCOM")


def test_configure_accelerator_usbgpu(monkeypatch):
  monkeypatch.setenv("USBGPU", "1")
  monkeypatch.setenv("AMD_IFACE", "USB")
  monkeypatch.setenv("QCOM_PRIORITY", "8")
  assert usbgpu.configure_accelerator(tici=True) is True
  assert os.environ["DEV"] == "USB+AMD"
  assert "AMD_IFACE" not in os.environ
  assert "QCOM_PRIORITY" not in os.environ


def test_configure_accelerator_off_uses_host_backend(monkeypatch):
  monkeypatch.delenv("USBGPU", raising=False)
  assert usbgpu.configure_accelerator(tici=False) is False
  assert os.environ["DEV"] == "CPU"
