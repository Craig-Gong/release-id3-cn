"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""
from __future__ import annotations

import hashlib
import json
import os

from iqpilot.common.swaglog import cloudlog
import urllib.request
from pathlib import Path

from iqpilot.system.hardware.usb import egpu_dock_ready

USB_SYSFS_ROOT = "/sys/bus/usb/devices"
FIRMWARE_MIRROR = os.getenv("IQ_EGPU_FIRMWARE_MIRROR", "/data/firmware/tinygrad")
TINYGRAD_CACHE = Path(os.getenv("XDG_CACHE_HOME", "/data/.cache"))
TINYGRAD_FW_STORE = Path("/data/amdgpu-fw/tinygrad-cache")
USBGPU_BULK_CHUNK = 128 * 1024
USBGPU_BULK_PAUSE_S = 0.005
USBGPU_LINK_SETTLE_S = 3.0
USBGPU_LINK_POLL_S = 1.0
USBGPU_HCQ_WAIT_MS = 300_000


def _env_int(name: str, default: int) -> int:
  raw = os.getenv(name)
  if raw is None or raw == "":
    return default
  try:
    return int(raw)
  except ValueError:
    return default


def _env_float(name: str, default: float) -> float:
  raw = os.getenv(name)
  if raw is None or raw == "":
    return default
  try:
    return float(raw)
  except ValueError:
    return default


def _bulk_tuning() -> tuple[int, float, float, int]:
  chunk = _env_int("IQ_EGPU_BULK_CHUNK", USBGPU_BULK_CHUNK)
  pause_ms = _env_float("IQ_EGPU_BULK_PAUSE_MS", USBGPU_BULK_PAUSE_S * 1000.0)
  settle_s = _env_float("IQ_EGPU_LINK_SETTLE_S", USBGPU_LINK_SETTLE_S)
  hcq_wait_ms = _env_int("IQ_EGPU_HCQ_WAIT_MS", USBGPU_HCQ_WAIT_MS)
  return max(chunk, 4096), max(pause_ms, 0.0) / 1000.0, max(settle_s, 0.0), max(hcq_wait_ms, 30_000)

COMMA_LFS_BATCH_URL = "https://gitlab.com/commaai/openpilot-lfs.git/info/lfs/objects/batch"

DOWNLOAD_CHUNK = 4 * 1024 * 1024


def usbgpu_present(sysfs_root: str = USB_SYSFS_ROOT) -> bool:
  return egpu_dock_ready(Path(sysfs_root))


def egpu_present_consented(params, sysfs_root: str = USB_SYSFS_ROOT) -> bool:
  try:
    if params is not None and params.get_bool("IQEgpuDisabled"):
      return False
  except Exception:
    pass
  return usbgpu_present(sysfs_root)


def egpu_selected(params, sysfs_root: str = USB_SYSFS_ROOT) -> bool:
  try:
    if params is not None and params.get_bool("IQEgpuDisabled"):
      return False
    if params is not None and params.get_bool("IQEgpuEnabled"):
      return True
  except Exception:
    pass
  return usbgpu_present(sysfs_root)


def egpu_artifact_prefers_local_policy(params) -> bool:
  """When True, use a cached policy.pkl before downloading the OOB streamable blob."""
  try:
    return params is not None and params.get_bool("IQEgpuPreferLocalPolicy")
  except Exception:
    return False


def resolve_backend(emac_enabled: bool, egpu_enabled: bool, egpu_present: bool = False) -> str | None:
  if egpu_present:
    return "egpu"
  if emac_enabled:
    return "emac"
  if egpu_enabled:
    return "egpu"
  return None


def egpu_pkl_path(meta: dict) -> str:
  from iqpilot.system.hardware.hw import Paths
  return os.path.join(Paths.model_root(), f"egpu_{meta['key']}_{meta['sha256'][:8]}_amd_tinygrad.pkl")


def egpu_policy_pkl_path(meta: dict) -> str:
  from iqpilot.system.hardware.hw import Paths
  return os.path.join(Paths.model_root(), f"egpu_{meta['key']}_{meta['sha256'][:8]}_amd_policy.pkl")


def egpu_oob_pkl_path(meta: dict) -> str:
  from iqpilot.system.hardware.hw import Paths
  return os.path.join(Paths.model_root(), f"egpu_{meta['key']}_{meta['sha256'][:8]}_amd_policy_oob.pkl")


def onnx_cache_path(meta: dict) -> str:
  from iqpilot.system.hardware.hw import Paths
  return os.path.join(Paths.model_root(), f"{meta['model_name']}_{meta['sha256'][:8]}.onnx")


def _sha256_file(path: str) -> str:
  digest = hashlib.sha256()
  with open(path, "rb") as f:
    while chunk := f.read(DOWNLOAD_CHUNK):
      digest.update(chunk)
  return digest.hexdigest()


def quarantine_artifact(path: str, why: str) -> None:
  try:
    if os.path.isfile(path):
      os.replace(path, path + ".unusable")
  except OSError:
    try:
      os.remove(path)
    except OSError:
      pass


def local_onnx(meta: dict) -> str | None:
  path = onnx_cache_path(meta)
  if not os.path.isfile(path):
    return None
  size = int(meta.get("download", {}).get("size", 0))
  if size and os.path.getsize(path) != size:
    quarantine_artifact(path, "onnx size mismatch")
    return None
  if _sha256_file(path) != meta["sha256"]:
    quarantine_artifact(path, "onnx sha256 mismatch")
    return None
  return path


def resolve_download_url(download_url: str, sha256: str, size: int, timeout: float = 30.0) -> str:
  if download_url.startswith("commalfs:"):
    oid = download_url.split(":", 1)[1]
    body = json.dumps({"operation": "download", "transfers": ["basic"],
                       "objects": [{"oid": oid, "size": size}]}).encode()
    req = urllib.request.Request(COMMA_LFS_BATCH_URL, data=body, headers={
      "Accept": "application/vnd.git-lfs+json", "Content-Type": "application/vnd.git-lfs+json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
      d = json.load(r)
    return d["objects"][0]["actions"]["download"]["href"]
  return download_url


def download_onnx(meta: dict, progress_cb=None) -> str:
  from iqpilot.selfdrive.iqmodeld.egpu_model import download_descriptor
  download_url, size = download_descriptor(meta)
  if not download_url:
    raise RuntimeError(f"model {meta['key']} has no download source; stage the onnx at {onnx_cache_path(meta)}")

  path = onnx_cache_path(meta)
  os.makedirs(os.path.dirname(path), exist_ok=True)
  try:
    from iqpilot.selfdrive.iqmodeld.model_bundle_downloader import download_hf_file
    return download_hf_file(f"onnx/{meta['sha256']}.onnx", path, meta["sha256"], int(size or 0), progress_cb=progress_cb)
  except Exception as e:
    cloudlog.warning(f"onnx {meta['key']} unavailable from HF ({e}); falling back to {download_url.split(':', 1)[0]}")
  url = resolve_download_url(download_url, meta["sha256"], size)
  tmp = path + ".part"
  digest = hashlib.sha256()
  got = 0
  with urllib.request.urlopen(url, timeout=60) as r, open(tmp, "wb") as f:
    while chunk := r.read(DOWNLOAD_CHUNK):
      f.write(chunk)
      digest.update(chunk)
      got += len(chunk)
      if progress_cb is not None and size:
        progress_cb(got / size)
  if size and got != size:
    os.remove(tmp)
    raise RuntimeError(f"onnx download truncated: {got}/{size} bytes")
  if digest.hexdigest() != meta["sha256"]:
    os.remove(tmp)
    raise RuntimeError(f"onnx sha256 mismatch for {meta['key']}")
  os.replace(tmp, path)
  return path


def download_precompiled(meta: dict, progress_cb=None, policy: bool = False, oob: bool = False) -> str | None:
  field = "egpu_oob_artifact" if oob else "egpu_policy_artifact" if policy else "egpu_artifact"
  art = meta.get(field)
  if not art or not (art.get("objects") or art.get("hf_path")):
    return None
  from iqpilot.selfdrive.iqmodeld.model_bundle_downloader import download_hf_file, download_lfs_bundle
  dest = egpu_oob_pkl_path(meta) if oob else egpu_policy_pkl_path(meta) if policy else egpu_pkl_path(meta)
  if art.get("hf_path"):
    try:
      return download_hf_file(art["hf_path"], dest, art["sha256"], int(art.get("size", 0)), progress_cb=progress_cb)
    except Exception as e:
      cloudlog.warning(f"precompiled {meta['key']} unavailable from HF ({e}); trying LFS")
      if not art.get("objects"):
        raise
  return download_lfs_bundle(art["objects"], dest, art["sha256"], int(art.get("size", 0)), progress_cb=progress_cb)


def iqos_linux49(release: str | None = None) -> bool:
  rel = os.uname().release if release is None else release
  return rel.startswith("4.9")


def restore_tinygrad_fw_cache(*, store: Path | None = None, cache: Path | None = None) -> int:
  """Mirror /data AMD firmware blobs into tinygrad's URL-md5 cache (IQ.OS fetch_fw 403)."""
  import shutil

  store = store or TINYGRAD_FW_STORE
  cache = cache or TINYGRAD_CACHE
  if not store.is_dir():
    return 0
  cache.mkdir(parents=True, exist_ok=True)
  copied = 0
  for src in store.iterdir():
    if not src.is_file():
      continue
    dest = cache / src.name
    if dest.is_file() and dest.stat().st_size == src.stat().st_size:
      continue
    shutil.copy2(src, dest)
    copied += 1
  return copied


def throttle_usbgpu_bulk_writes(*, usb3_cls=None, chunk: int = USBGPU_BULK_CHUNK,
                                pause_s: float = USBGPU_BULK_PAUSE_S) -> bool:
  """IQ.OS 4.9 xHCI wedges on huge USB3 bulk OUT; chunk writes with a short pause."""
  import time

  if not iqos_linux49():
    return False
  if usb3_cls is None:
    try:
      from tinygrad.runtime.support.usb import USB3
      usb3_cls = USB3
    except ImportError:
      return False
  orig = usb3_cls.bulk_write
  if getattr(orig, "_iqos_throttled", False):
    return True

  def bulk_write(self, payload, timeout=1000):
    n = len(payload)
    if n <= chunk:
      return orig(self, payload, timeout)
    mv = memoryview(payload)
    for i in range(0, n, chunk):
      orig(self, bytes(mv[i:i + chunk]), timeout)
      if pause_s and i + chunk < n:
        time.sleep(pause_s)

  bulk_write._iqos_throttled = True  # type: ignore[attr-defined]
  usb3_cls.bulk_write = bulk_write
  return True


def throttle_usbgpu_bulk_reads(*, usb3_cls=None, chunk: int = USBGPU_BULK_CHUNK,
                               pause_s: float = USBGPU_BULK_PAUSE_S) -> bool:
  """IQ.OS 4.9 xHCI wedges on huge USB3 bulk IN during inference readback."""
  import time

  if not iqos_linux49():
    return False
  if usb3_cls is None:
    try:
      from tinygrad.runtime.support.usb import USB3
      usb3_cls = USB3
    except ImportError:
      return False
  orig = usb3_cls.bulk_read
  if getattr(orig, "_iqos_throttled", False):
    return True

  def bulk_read(self, length, timeout=1000):
    if length <= chunk:
      return orig(self, length, timeout)
    req_timeout = max(timeout, 30_000) if iqos_linux49() else timeout
    parts: list[bytes] = []
    remaining = length
    while remaining > 0:
      n = min(chunk, remaining)
      part = orig(self, n, req_timeout)
      blob = bytes(part)
      parts.append(blob)
      got = len(blob)
      remaining -= got
      if got < n:
        break
      if pause_s and remaining > 0:
        time.sleep(pause_s)
    return memoryview(b"".join(parts))

  bulk_read._iqos_throttled = True  # type: ignore[attr-defined]
  usb3_cls.bulk_read = bulk_read
  return True


def wait_for_stable_usb_link(*, settle_s: float = USBGPU_LINK_SETTLE_S,
                             poll_s: float = USBGPU_LINK_POLL_S) -> bool:
  """Poll ssusb portli counters; require no growth over settle_s (IQ.OS 4.9.1+)."""
  import time

  from iqpilot.system.hardware.usb import get_link_error_count

  if not iqos_linux49():
    time.sleep(settle_s)
    return True
  deadline = time.monotonic() + settle_s
  baseline = get_link_error_count()
  while time.monotonic() < deadline:
    time.sleep(poll_s)
    if get_link_error_count() > baseline:
      return False
  return True


def prepare_egpu_runtime() -> None:
  """Call before any tinygrad DEV=USB+AMD work on IQ.OS."""
  chunk, pause_s, settle_s, hcq_wait_ms = _bulk_tuning()
  if iqos_linux49() and os.getenv("HCQDEV_WAIT_TIMEOUT_MS") is None:
    os.environ["HCQDEV_WAIT_TIMEOUT_MS"] = str(hcq_wait_ms)
  try:
    from iqpilot.system.hardware.usb import ensure_host_role
    if not ensure_host_role():
      cloudlog.warning("egpu: ssusb mode is not host; dock may not enumerate")
  except Exception as e:
    cloudlog.warning(f"egpu: ensure_host_role failed ({e})")
  restore_tinygrad_fw_cache()
  patch_tinygrad_fetch_fw()
  throttle_usbgpu_bulk_writes(chunk=chunk, pause_s=pause_s)
  throttle_usbgpu_bulk_reads(chunk=chunk, pause_s=pause_s)
  if not wait_for_stable_usb_link(settle_s=settle_s):
    cloudlog.warning("egpu: USB link errors increased during settle window")


def patch_tinygrad_fetch_fw() -> None:
  import pathlib

  import zstandard
  from tinygrad import helpers
  if getattr(helpers.fetch_fw, "_iq_patched", False):
    return
  _orig = helpers.fetch_fw

  def fetch_fw(path, name, sha256):
    mirror = pathlib.Path(FIRMWARE_MIRROR) / path / name
    if mirror.is_file():
      blob = mirror.read_bytes()
      if hashlib.sha256(blob).hexdigest() == sha256:
        return blob
    p = pathlib.Path(f"/lib/firmware/{path}/{name}.zst")
    if p.is_file():
      blob = zstandard.ZstdDecompressor().stream_reader(p.read_bytes()).read()
      if hashlib.sha256(blob).hexdigest() == sha256:
        return blob
    blob = _orig(path, name, sha256)
    # The dock's GPU firmware otherwise lives only in tinygrad's per-user download cache, which is
    # a network fetch the first time a new HOME sees it; onroad the car is usually offline.
    try:
      mirror.parent.mkdir(parents=True, exist_ok=True)
      tmp = mirror.with_suffix(mirror.suffix + ".part")
      tmp.write_bytes(blob)
      os.replace(tmp, mirror)
    except OSError:
      pass
    return blob

  fetch_fw._iq_patched = True
  helpers.fetch_fw = fetch_fw
