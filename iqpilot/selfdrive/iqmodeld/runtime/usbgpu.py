"""tinygrad USBGPU (VID 0xADD1) backend selection for iqmodeld.

Chestnut / tinygrad USBGPU talks over USB3; C3XL can run it.

QCOM-compiled bundles cannot run on AMD. When the dock is present (or USBGPU=1)
and every weight pkl has an AMD sibling on disk, remap the active bundle to those
files and set DEV=USB+AMD. USBGPU=0 forces the QCOM path.

Sibling names (first existing file wins):
  driving_foo_tinygrad.pkl -> big_driving_foo_tinygrad.pkl  (official chestnut)
  driving_foo_tinygrad.pkl -> driving_foo_usbgpu_tinygrad.pkl

Do not set AMD_IFACE=USB: this tinygrad asserts that as deprecated. Use DEV=USB+AMD.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

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


def usbgpu_forced_off() -> bool:
  if "USBGPU" not in os.environ:
    return False
  return os.environ["USBGPU"] in ("0", "false", "False")


def usbgpu_enabled() -> bool:
  """True when USBGPU is set and not an explicit off value."""
  if "USBGPU" not in os.environ:
    return False
  return not usbgpu_forced_off()


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


def is_usbgpu_filename(filename: str) -> bool:
  name = Path(filename).name
  return name.startswith("big_") or "_usbgpu" in name


def normalize_artifact_basename(filename: str) -> str:
  """Strip USBGPU prefixes so bundle-type detection still matches."""
  name = Path(filename).name
  if name.startswith("big_"):
    name = name[4:]
  if "_usbgpu" in name:
    name = name.replace("_usbgpu", "")
  return name


def _is_weight_artifact(filename: str) -> bool:
  name = Path(filename).name.lower()
  if "metadata" in name:
    return False
  return name.endswith(".pkl")


def usbgpu_sibling_names(filename: str) -> list[str]:
  base = Path(filename).name
  if is_usbgpu_filename(base):
    return []
  names: list[str] = []
  if not base.startswith("big_"):
    names.append("big_" + base)
  for suffix in ("_tinygrad.pkl", "_metadata.pkl", ".pkl"):
    if base.endswith(suffix):
      names.append(base[:-len(suffix)] + "_usbgpu" + suffix)
      break
  return names


def resolve_usbgpu_artifact(filename: str, model_root: Path) -> str | None:
  for candidate in usbgpu_sibling_names(filename):
    if (model_root / candidate).is_file():
      return candidate
  return None


def _iter_named_artifacts(bundle: Any):
  for model in getattr(bundle, "models", None) or []:
    for attr in ("artifact", "metadata"):
      art = getattr(model, attr, None)
      if art is None:
        continue
      filename = getattr(art, "fileName", None) or getattr(art, "file_name", None)
      if filename:
        yield art, str(filename)


def bundle_has_usbgpu_weights(bundle: Any, model_root: Path) -> bool:
  weights = [(art, fn) for art, fn in _iter_named_artifacts(bundle) if _is_weight_artifact(fn)]
  if not weights:
    return False
  for _art, filename in weights:
    if is_usbgpu_filename(filename):
      if not (model_root / Path(filename).name).is_file():
        return False
      continue
    if resolve_usbgpu_artifact(filename, model_root) is None:
      return False
  return True


def usbgpu_overlay_wanted(bundle: Any, model_root: Path, dock_present: bool | None = None) -> bool:
  if bundle is None or usbgpu_forced_off():
    return False
  if not bundle_has_usbgpu_weights(bundle, model_root):
    return False
  if usbgpu_enabled():
    return True
  if dock_present is None:
    dock_present = usbgpu_present()
  return dock_present


def apply_usbgpu_overlay(bundle: Any, model_root: Path, dock_present: bool | None = None) -> Any:
  """Rewrite artifact filenames onto AMD siblings. Does not persist params."""
  if not usbgpu_overlay_wanted(bundle, model_root, dock_present=dock_present):
    return bundle

  remapped: list[tuple[str, str]] = []
  for art, filename in _iter_named_artifacts(bundle):
    if is_usbgpu_filename(filename):
      continue
    sibling = resolve_usbgpu_artifact(filename, model_root)
    if sibling is None:
      continue
    remapped.append((filename, sibling))
    if hasattr(art, "fileName"):
      art.fileName = sibling
    elif hasattr(art, "file_name"):
      art.file_name = sibling

  if remapped:
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.warning(f"usbgpu overlay active: {remapped}")
    except Exception:
      pass
  return bundle


def prepare_usbgpu_runtime(bundle: Any = None, model_root: Path | None = None, tici: bool | None = None) -> bool:
  """Enable USB+AMD when the env asks or when the dock and AMD weights are both present."""
  if bundle is not None:
    if model_root is None:
      from openpilot.system.hardware.hw import Paths
      model_root = Path(Paths.model_root())
    if usbgpu_overlay_wanted(bundle, model_root):
      if not usbgpu_forced_off():
        os.environ["USBGPU"] = "1"
  return configure_accelerator(tici=tici)


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
