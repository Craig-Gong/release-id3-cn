"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""
import types

import pytest

from iqpilot.cereal import log, messaging
from iqpilot.cereal.services import SERVICE_LIST


class TestTelemetryContract:
  def test_service_is_published_at_stock_cadence(self):
    assert "egpuDockState" in SERVICE_LIST
    assert SERVICE_LIST["egpuDockState"].frequency == 10.

  def test_message_carries_every_stock_field(self):
    msg = messaging.new_message("egpuDockState")
    state = msg.egpuDockState
    for field in ("tempC", "memoryTempC", "powerDrawW", "powerLimitW", "gpuUsagePercent",
                  "gpuClockMhz", "fanSpeedRpm", "pcieLtssm", "supplyVoltage", "supplyCurrent"):
      setattr(state, field, 1)
      assert getattr(state, field) == 1

  def test_metrics_refresh_matches_stock(self):
    from iqpilot.selfdrive.iqmodeld.egpu_telemetry import METRICS_REFRESH_EVERY
    assert METRICS_REFRESH_EVERY == 100

  def test_send_without_a_gpu_publishes_an_invalid_message(self):
    from iqpilot.selfdrive.iqmodeld import egpu_telemetry
    sent = []
    telemetry = egpu_telemetry.EgpuDockTelemetry(types.SimpleNamespace(send=lambda n, m: sent.append((n, m))), big=True)
    telemetry._device = lambda: types.SimpleNamespace(_opened_devices=set())
    telemetry.send()
    assert sent and sent[0][0] == "egpuDockState"
    assert sent[0][1].valid is False


class TestBigFrameFlag:
  def test_model_message_carries_the_big_flag(self):
    msg = messaging.new_message("modelV2")
    msg.modelV2.big = True
    assert msg.modelV2.big


class TestStatusParams:
  def test_loading_param_exists_and_is_cleared_like_stock(self):
    from pathlib import Path
    root = Path(__file__).resolve().parents[3]
    keys = (root / "common" / "params_keys.h").read_text()
    assert '{"UsbGpuLoading"' in keys
    line = next(ln for ln in keys.splitlines() if '"UsbGpuLoading"' in ln)
    for flag in ("CLEAR_ON_MANAGER_START", "CLEAR_ON_OFFROAD_TRANSITION", "CLEAR_ON_IGNITION_ON"):
      assert flag in line


class TestAlerts:
  def test_both_stock_big_model_events_exist(self):
    assert hasattr(log.OnroadEvent.EventName, "bigModelLoading")
    assert hasattr(log.OnroadEvent.EventName, "bigModelFailed")

  def test_alerts_are_wired_with_stock_severities(self):
    from iqpilot.selfdrive.selfdrived.events import EVENTS, ET
    EventName = log.OnroadEvent.EventName
    loading = EVENTS[EventName.bigModelLoading]
    failed = EVENTS[EventName.bigModelFailed]
    assert ET.NO_ENTRY in loading
    assert ET.SOFT_DISABLE in failed and ET.PERMANENT in failed


class TestFirmwareGate:
  def test_runtime_refuses_a_dock_on_other_firmware(self, tmp_path):
    from iqpilot.selfdrive.iqmodeld.egpu_helpers import usbgpu_present
    from iqpilot.system.hardware.usb import EGPU_DOCK_FW_PRODUCT, EGPU_DOCK_USB_IDS
    vid, pid = EGPU_DOCK_USB_IDS[0]
    d = tmp_path / "1-1"
    d.mkdir()
    (d / "idVendor").write_text(f"{vid:04x}\n")
    (d / "idProduct").write_text(f"{pid:04x}\n")
    (d / "product").write_text("custom deadbeef-CLEAN\n")
    assert not usbgpu_present(str(tmp_path))
    (d / "product").write_text(EGPU_DOCK_FW_PRODUCT + "\n")
    assert usbgpu_present(str(tmp_path))


class TestAutoFlash:
  def test_hardwared_drives_the_flasher_offroad_only(self):
    from iqpilot.system.hardware.hardwared import EgpuDockFlasher
    f = EgpuDockFlasher()
    calls = []
    f.flash = lambda: calls.append(1)
    stale = [{"vendorId": 0xADD1, "productId": 0x0001, "product": "custom deadbeef-CLEAN"}]
    f.update(False, stale)
    assert f.attempts == 0, "must not flash onroad"
    f.update(True, stale)
    assert f.attempts == 1
    if f.thread is not None:
      f.thread.join(timeout=5)

  def test_matching_firmware_is_never_flashed(self):
    from iqpilot.system.hardware.egpu_dock.flash import bundled_version
    from iqpilot.system.hardware.hardwared import EgpuDockFlasher
    f = EgpuDockFlasher()
    f.flash = lambda: pytest.fail("flashed a dock that already matches")
    f.update(True, [{"vendorId": 0xADD1, "productId": 0x0001, "product": bundled_version()}])
    assert f.attempts == 0

  def test_attempts_are_bounded_like_stock(self):
    from iqpilot.system.hardware.hardwared import EgpuDockFlasher
    assert EgpuDockFlasher.MAX_ATTEMPTS == 3
    assert EgpuDockFlasher.RETRY_INTERVAL == 20.


class TestDockIsItsOwnConsent:

  def _params(self, **flags):
    class P:
      def get_bool(self, k):
        return bool(flags.get(k, False))
      def get(self, k, *a, **kw):
        return None
    return P()

  def _sysfs_with_dock(self, tmp_path, product=None):
    from iqpilot.system.hardware.usb import EGPU_DOCK_FW_PRODUCT, EGPU_DOCK_USB_IDS
    vid, pid = EGPU_DOCK_USB_IDS[0]
    d = tmp_path / "1-1"
    d.mkdir()
    (d / "idVendor").write_text(f"{vid:04x}\n")
    (d / "idProduct").write_text(f"{pid:04x}\n")
    (d / "product").write_text((product or EGPU_DOCK_FW_PRODUCT) + "\n")
    return str(tmp_path)

  def test_a_plugged_in_dock_selects_itself(self, tmp_path):
    from iqpilot.selfdrive.iqmodeld.egpu_helpers import egpu_selected
    assert egpu_selected(self._params(), self._sysfs_with_dock(tmp_path))

  def test_nothing_plugged_in_selects_nothing(self, tmp_path):
    from iqpilot.selfdrive.iqmodeld.egpu_helpers import egpu_selected
    assert not egpu_selected(self._params(), str(tmp_path))

  def test_a_dock_on_foreign_firmware_does_not_select_itself(self, tmp_path):
    from iqpilot.selfdrive.iqmodeld.egpu_helpers import egpu_selected
    assert not egpu_selected(self._params(), self._sysfs_with_dock(tmp_path, "custom deadbeef-CLEAN"))

  def test_the_user_can_force_it_off(self, tmp_path):
    from iqpilot.selfdrive.iqmodeld.egpu_helpers import egpu_selected
    root = self._sysfs_with_dock(tmp_path)
    assert not egpu_selected(self._params(IQEgpuDisabled=True), root)

  def test_the_param_can_force_it_on_without_hardware(self, tmp_path):
    from iqpilot.selfdrive.iqmodeld.egpu_helpers import egpu_selected
    assert egpu_selected(self._params(IQEgpuEnabled=True), str(tmp_path))

  def test_present_dock_wins_even_with_emac_enabled(self, tmp_path):
    from iqpilot.selfdrive.iqmodeld.egpu_helpers import egpu_selected, resolve_backend, usbgpu_present
    root = self._sysfs_with_dock(tmp_path)
    assert resolve_backend(True, egpu_selected(self._params(), root), usbgpu_present(root)) == "egpu"

  def test_force_param_without_hardware_yields_to_emac(self, tmp_path):
    from iqpilot.selfdrive.iqmodeld.egpu_helpers import egpu_selected, resolve_backend, usbgpu_present
    assert resolve_backend(True, egpu_selected(self._params(IQEgpuEnabled=True), str(tmp_path)),
                           usbgpu_present(str(tmp_path))) == "emac"
