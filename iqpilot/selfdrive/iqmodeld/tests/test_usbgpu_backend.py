from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

import pytest

from openpilot.iqpilot.selfdrive.iqmodeld.runtime import usbgpu


def _fake_usb_device(root: Path, vid: str, pid: str) -> None:
  node = root / "1-1"
  node.mkdir()
  (node / "idVendor").write_text(f"{vid}\n")
  (node / "idProduct").write_text(f"{pid}\n")


@dataclass
class _Artifact:
  fileName: str


@dataclass
class _Model:
  type: str = "vision"
  artifact: _Artifact = field(default_factory=lambda: _Artifact(""))
  metadata: _Artifact = field(default_factory=lambda: _Artifact(""))


@dataclass
class _Bundle:
  models: list[_Model]


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


def test_normalize_artifact_basename():
  assert usbgpu.normalize_artifact_basename("big_driving_supercombo_foo_tinygrad.pkl") == "driving_supercombo_foo_tinygrad.pkl"
  assert usbgpu.normalize_artifact_basename("driving_vision_c210m_usbgpu_tinygrad.pkl") == "driving_vision_c210m_tinygrad.pkl"


def test_usbgpu_sibling_names_prefers_big_prefix():
  names = usbgpu.usbgpu_sibling_names("driving_vision_c210m_tinygrad.pkl")
  assert names[0] == "big_driving_vision_c210m_tinygrad.pkl"
  assert "driving_vision_c210m_usbgpu_tinygrad.pkl" in names


def test_resolve_usbgpu_artifact(tmp_path: Path):
  (tmp_path / "big_driving_policy_c210m_tinygrad.pkl").write_bytes(b"x")
  assert usbgpu.resolve_usbgpu_artifact("driving_policy_c210m_tinygrad.pkl", tmp_path) == "big_driving_policy_c210m_tinygrad.pkl"


def test_bundle_has_usbgpu_weights_split_bundle(tmp_path: Path):
  (tmp_path / "big_driving_vision_c210m_tinygrad.pkl").write_bytes(b"v")
  (tmp_path / "big_driving_policy_c210m_tinygrad.pkl").write_bytes(b"p")
  bundle = _Bundle([
    _Model(artifact=_Artifact("driving_vision_c210m_tinygrad.pkl"), metadata=_Artifact("driving_vision_c210m_metadata.pkl")),
    _Model(artifact=_Artifact("driving_policy_c210m_tinygrad.pkl"), metadata=_Artifact("driving_policy_c210m_metadata.pkl")),
  ])
  assert usbgpu.bundle_has_usbgpu_weights(bundle, tmp_path)


def test_apply_usbgpu_overlay_with_dock(tmp_path: Path, monkeypatch):
  monkeypatch.delenv("USBGPU", raising=False)
  (tmp_path / "big_driving_vision_c210m_tinygrad.pkl").write_bytes(b"v")
  (tmp_path / "big_driving_policy_c210m_tinygrad.pkl").write_bytes(b"p")
  bundle = _Bundle([
    _Model(artifact=_Artifact("driving_vision_c210m_tinygrad.pkl"), metadata=_Artifact("driving_vision_c210m_metadata.pkl")),
    _Model(artifact=_Artifact("driving_policy_c210m_tinygrad.pkl"), metadata=_Artifact("driving_policy_c210m_metadata.pkl")),
  ])
  usbgpu.apply_usbgpu_overlay(bundle, tmp_path, dock_present=True)
  assert bundle.models[0].artifact.fileName == "big_driving_vision_c210m_tinygrad.pkl"
  assert bundle.models[1].artifact.fileName == "big_driving_policy_c210m_tinygrad.pkl"
  assert bundle.models[0].metadata.fileName == "driving_vision_c210m_metadata.pkl"


def test_apply_usbgpu_overlay_skips_without_siblings(tmp_path: Path, monkeypatch):
  monkeypatch.setenv("USBGPU", "1")
  bundle = _Bundle([
    _Model(artifact=_Artifact("driving_vision_c210m_tinygrad.pkl"), metadata=_Artifact("driving_vision_c210m_metadata.pkl")),
  ])
  usbgpu.apply_usbgpu_overlay(bundle, tmp_path, dock_present=True)
  assert bundle.models[0].artifact.fileName == "driving_vision_c210m_tinygrad.pkl"


def test_prepare_usbgpu_runtime_auto_enables_with_dock(tmp_path: Path, monkeypatch):
  monkeypatch.delenv("USBGPU", raising=False)
  (tmp_path / "big_driving_vision_c210m_tinygrad.pkl").write_bytes(b"v")
  bundle = _Bundle([
    _Model(artifact=_Artifact("driving_vision_c210m_tinygrad.pkl"), metadata=_Artifact("driving_vision_c210m_metadata.pkl")),
  ])
  monkeypatch.setattr(usbgpu, "usbgpu_present", lambda _root=None: True)
  assert usbgpu.prepare_usbgpu_runtime(bundle, tmp_path, tici=True) is True
  assert os.environ["DEV"] == "USB+AMD"
  assert os.environ.get("USBGPU") == "1"


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
