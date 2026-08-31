from pathlib import Path
import tempfile

from iqpilot.system.ecoflow.enabled import (
  has_credentials,
  heal_enabled,
  is_enabled,
  overlay_enabled,
  read_status,
  set_enabled,
  status_line,
  write_overlay_enabled,
  write_status,
)


class _Params:
  def __init__(self, enabled=False):
    self.enabled = enabled

  def get(self, key):
    if key == "EcoflowEnabled":
      return "1" if self.enabled else "0"
    return None

  def get_bool(self, key):
    return bool(self.enabled) if key == "EcoflowEnabled" else False

  def put_bool(self, key, value, block=True):
    if key == "EcoflowEnabled":
      self.enabled = bool(value)


def _creds(overlay: Path) -> None:
  overlay.mkdir(parents=True, exist_ok=True)
  (overlay / "EcoflowPhone").write_text("13800138000")
  (overlay / "EcoflowPassword").write_text("secret")
  (overlay / "EcoflowSn").write_text("P231CC1AZH6F0154")


def test_overlay_file_wins_over_native_off():
  with tempfile.TemporaryDirectory() as td:
    overlay = Path(td)
    write_overlay_enabled(True, overlay)
    assert is_enabled(_Params(enabled=False), overlay) is True
    write_overlay_enabled(False, overlay)
    assert is_enabled(_Params(enabled=True), overlay) is False


def test_creds_without_enabled_file_count_as_on():
  with tempfile.TemporaryDirectory() as td:
    overlay = Path(td)
    _creds(overlay)
    assert has_credentials(overlay) is True
    assert overlay_enabled(overlay) is None
    assert is_enabled(_Params(enabled=False), overlay) is True


def test_explicit_off_not_overridden_by_creds():
  with tempfile.TemporaryDirectory() as td:
    overlay = Path(td)
    _creds(overlay)
    write_overlay_enabled(False, overlay)
    assert is_enabled(_Params(enabled=False), overlay) is False


def test_heal_writes_enabled_when_only_creds_exist():
  with tempfile.TemporaryDirectory() as td:
    overlay = Path(td)
    _creds(overlay)
    params = _Params(enabled=False)
    assert heal_enabled(params, overlay) is True
    assert overlay_enabled(overlay) is True
    assert params.enabled is True


def test_heal_mirrors_native_on_into_overlay():
  with tempfile.TemporaryDirectory() as td:
    overlay = Path(td)
    params = _Params(enabled=True)
    assert heal_enabled(params, overlay) is True
    assert overlay_enabled(overlay) is True


def test_set_enabled_writes_overlay_and_native():
  with tempfile.TemporaryDirectory() as td:
    overlay = Path(td)
    params = _Params(enabled=False)
    set_enabled(True, params, overlay)
    assert overlay_enabled(overlay) is True
    assert params.enabled is True
    set_enabled(False, params, overlay)
    assert overlay_enabled(overlay) is False
    assert params.enabled is False


def test_status_line_roundtrip():
  with tempfile.TemporaryDirectory() as td:
    overlay = Path(td)
    write_status({"kl15": True, "mqtt": True, "telemetry": True, "error": ""}, overlay)
    st = read_status(overlay)
    assert st["kl15"] is True
    line = status_line(st)
    assert "KL15 on" in line
    assert "MQTT" in line
    assert "DC on" in line
