"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""
from __future__ import annotations

import glob
import os
os.environ.setdefault("XDG_CACHE_HOME", "/data/.cache")
import time

from iqpilot.common.params import Params
from iqpilot.common.swaglog import cloudlog
from iqpilot.selfdrive.iqmodeld.egpu_helpers import download_precompiled, egpu_policy_pkl_path, egpu_selected, patch_tinygrad_fetch_fw, usbgpu_present
from iqpilot.selfdrive.iqmodeld.egpu_model import resolve_egpu_model

POLL_S = 30.0
RETRY_S = 120.0


def _selected_meta(params: Params) -> dict | None:
  key = params.get("IQEmacModel", encoding="utf8")
  try:
    return resolve_egpu_model(params, key)
  except Exception as e:
    cloudlog.warning(f"egpu_prefetch cannot resolve {key!r}: {e}")
    return None


def _drop_stale_partials(keep: str) -> None:
  root = os.path.dirname(keep)
  for path in glob.glob(os.path.join(root, "egpu_*_amd_policy.pkl.part")) + glob.glob(os.path.join(root, "big_driving_supercombo_*.onnx.part")):
    if not path.startswith(keep):
      try:
        os.remove(path)
      except OSError:
        pass


def prefetch_once(params: Params) -> bool:
  if not usbgpu_present() or not egpu_selected(params):
    return False
  meta = _selected_meta(params)
  if meta is None:
    return False
  dst = egpu_policy_pkl_path(meta)
  if os.path.isfile(dst):
    return True
  if not meta.get("egpu_policy_artifact"):
    return False
  _drop_stale_partials(dst)
  params.put("UsbGpuSetupProgress", "0.0")
  last = [-1.0]

  def _prog(p: float) -> None:
    if p - last[0] >= 0.02 or p >= 1.0:
      last[0] = p
      params.put("UsbGpuSetupProgress", f"{p:.3f}")

  cloudlog.warning(f"egpu_prefetch downloading {meta['key']} policy artifact offroad")
  out = download_precompiled(meta, progress_cb=_prog, policy=True)
  cloudlog.warning(f"egpu_prefetch ready -> {out}")
  return out is not None


_firmware_warm = False


def warm_firmware() -> None:
  global _firmware_warm
  if _firmware_warm or not usbgpu_present():
    return
  os.environ.setdefault("DEV", "USB+AMD:LLVM")
  os.environ.setdefault("GMMU", "0")
  patch_tinygrad_fetch_fw()
  from tinygrad.device import Device
  Device["AMD"]
  _firmware_warm = True
  cloudlog.warning("egpu_prefetch: dock opened offroad; firmware cached and mirrored")


def main() -> None:
  params = Params()
  while True:
    try:
      warm_firmware()
    except Exception as e:
      cloudlog.warning(f"egpu_prefetch firmware warm failed: {e}")
    try:
      prefetch_once(params)
    except Exception as e:
      cloudlog.warning(f"egpu_prefetch failed: {e}")
      params.put("UsbGpuLastError", str(e)[:512])
      time.sleep(RETRY_S)
      continue
    time.sleep(POLL_S)


if __name__ == "__main__":
  main()
