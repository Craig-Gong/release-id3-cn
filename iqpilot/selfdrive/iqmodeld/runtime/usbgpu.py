"""tinygrad USBGPU (VID 0xADD1) backend selection for iqmodeld.

Chestnut / tinygrad USBGPU talks over USB3; C3XL can run it. Current IQ artifacts
are still QCOM JIT, so the dock is detected and logged but AMD is only used when
USBGPU is explicitly enabled (env). Large-model switching is a follow-up.

Do not set AMD_IFACE=USB: this tinygrad asserts that as deprecated. Use DEV=USB+AMD.
"""
from __future__ import annotations

import os
from pathlib import Path

USBGPU_VID = 0xADD1
USBGPU_PID = 0x0001
USBGPU_DEV = "USB+AMD"
USB_DEVICES_ROOT = Path("/sys/bus/usb/devices")


def usbgpu_present(devices_root: Path | None = None) -> bool:
  root = USB_DEVICES_ROOT if devices_root is None else devices_root
  try:
    entries = root.iterdir()
  except OSError:
    return False

  for node in entries:
    try:
      vid = int((node / "idVendor").read_text().strip(), 16)
      pid = int((node / "idProduct").read_text().strip(), 16)
    except (OSError, ValueError):
      continue
    if vid == USBGPU_VID and pid == USBGPU_PID:
      return True
  return False


def usbgpu_enabled() -> bool:
  """True only when USBGPU is set and not an explicit off value.

  Presence of the dock alone does not switch backends: QCOM-compiled bundles
  cannot run on AMD USB.
  """
  if "USBGPU" not in os.environ:
    return False
  return os.environ["USBGPU"] not in ("0", "false", "False")


def tinygrad_backend_name(dev_env: str | None = None) -> str | None:
  """Device token baked into JIT captures (QCOM / AMD / CPU), not the full DEV spec."""
  raw = os.environ.get("DEV") if dev_env is None else dev_env
  if not raw:
    return None
  after_plus = raw.split("+")[-1]
  name = after_plus.split(":")[0].strip().upper()
  return name or None


def backend_matches_captured(captured_devices: set[str], dev_env: str | None = None) -> bool:
  expected = tinygrad_backend_name(dev_env)
  if not expected or not captured_devices:
    return True
  return expected in captured_devices


def configure_accelerator(tici: bool | None = None) -> bool:
  """Point tinygrad at QCOM, CPU, or USB AMD. Must run before tinygrad is imported."""
  if tici is None:
    from openpilot.system.hardware import TICI
    tici = TICI

  if usbgpu_enabled():
    os.environ["DEV"] = USBGPU_DEV
    os.environ.pop("AMD_IFACE", None)
    os.environ.pop("QCOM_PRIORITY", None)
    return True

  os.environ["DEV"] = "QCOM" if tici else "CPU"
  if tici:
    os.environ["QCOM_PRIORITY"] = "8"
  return False
