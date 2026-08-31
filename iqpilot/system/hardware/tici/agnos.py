#!/usr/bin/env python3
import base64
import hashlib
import json
import lzma
import os
import struct
import subprocess
import time
from collections.abc import Generator

import sys
_VENV_PY = "/usr/local/venv/bin/python3"
if sys.executable != _VENV_PY and os.path.exists(_VENV_PY):
  try:
    import Crypto  # noqa: F401
  except ImportError:
    os.execv(_VENV_PY, [_VENV_PY, os.path.abspath(__file__), *sys.argv[1:]])

import requests

import iqpilot.system.updated.casync.casync as casync

try:
  from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
except Exception as exc:
  Ed25519PublicKey = None
  _CRYPTO_IMPORT_ERROR = exc
else:
  _CRYPTO_IMPORT_ERROR = None

SPARSE_CHUNK_FMT = struct.Struct('H2xI4x')
CAIBX_URL = "https://commadist.azureedge.net/agnosupdate/"
IQPILOT_MANIFEST_PUBLIC_KEY = bytes.fromhex("40ae3f81b77506ecc4982a1ca37ba1d6f8765d2ae510eae9039577206c3e5732")

AGNOS_MANIFEST_FILE = "system/hardware/tici/agnos.json"

LFS_POINTER_MAGIC = b"version https://git-lfs"

def _image_auth_module():
  try:
    from iqpilot.system.proprietary_runtime._verified_import import import_verified_module
    return import_verified_module("iqpilot_updater_private", "iqpilot_private.updater.git_remote")
  except Exception:
    pass
  try:
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    bundle_python = os.path.join(root, "artifacts", "iqpilot_updater_private", "python")
    if os.path.isdir(bundle_python):
      if bundle_python not in sys.path:
        sys.path.insert(0, bundle_python)
      import importlib
      return importlib.import_module("iqpilot_private.updater.git_remote")
  except Exception:
    pass
  return None

def _download_headers(url: str) -> dict:
  mod = _image_auth_module()
  if mod is not None:
    try:
      headers = mod.os_image_headers(url)
      if headers:
        return headers
    except Exception:
      pass
  try:
    from iqpilot.common.git_creds import get_credentials
    creds = get_credentials()
    if creds and all(creds) and "/iq.lvbs/iqos" in url.lower():
      return {"Authorization": "Basic " + base64.b64encode(f"{creds[0]}:{creds[1]}".encode()).decode()}
  except Exception:
    pass
  return {}

def _open_image_response(url: str) -> requests.Response:
  auth = _download_headers(url)
  req = requests.get(url, stream=True, headers={'Accept-Encoding': None, **auth}, timeout=60)
  req.raise_for_status()
  if int(req.headers.get('content-length') or 0) >= 1024:
    return req

  body = req.content
  if not body.startswith(LFS_POINTER_MAGIC):
    raise requests.exceptions.InvalidURL(f"unexpected tiny response ({len(body)} bytes) for {url}")
  meta = dict(line.split(" ", 1) for line in body.decode().strip().splitlines() if " " in line)
  oid = meta["oid"].split(":", 1)[1]
  size = int(meta["size"])
  lfs_base = url.split("/raw/", 1)[0] + ".git/info/lfs"

  req = requests.get(f"{lfs_base}/objects/{oid}", stream=True,
                     headers={'Accept-Encoding': None, 'Accept': 'application/vnd.git-lfs', **auth}, timeout=60)
  if req.status_code == 200:
    return req

  batch = requests.post(f"{lfs_base}/objects/batch",
                        data=json.dumps({"operation": "download", "transfers": ["basic"],
                                         "objects": [{"oid": oid, "size": size}]}),
                        headers={"Content-Type": "application/vnd.git-lfs+json",
                                 "Accept": "application/vnd.git-lfs+json", **auth},
                        timeout=60)
  batch.raise_for_status()
  action = batch.json()["objects"][0]["actions"]["download"]
  req = requests.get(action["href"], stream=True,
                     headers={'Accept-Encoding': None, **action.get("header", {})}, timeout=60)
  req.raise_for_status()
  return req


def verify_manifest_signature(manifest_path: str) -> None:
  sig_path = f"{manifest_path}.sig"
  if not os.path.exists(sig_path):
    raise RuntimeError(f"missing AGNOS manifest signature: {sig_path}")
  if Ed25519PublicKey is None:
    raise RuntimeError(f"cryptography import failed: {_CRYPTO_IMPORT_ERROR}")

  manifest_bytes = open(manifest_path, "rb").read()
  signature = base64.b64decode(open(sig_path, "rb").read().strip())
  digest = hashlib.sha256(manifest_bytes).digest()
  public_key = Ed25519PublicKey.from_public_bytes(IQPILOT_MANIFEST_PUBLIC_KEY)
  public_key.verify(signature, digest)

class _ChainedParts:
  def __init__(self, urls: list[str]) -> None:
    self.urls = urls
    self.req: requests.Response | None = None

  def raise_for_status(self) -> None:
    if self.req is not None:
      self.req.raise_for_status()

  def iter_content(self, chunk_size: int) -> Generator[bytes, None, None]:
    for u in self.urls:
      self.req = _open_image_response(u)
      yield from self.req.iter_content(chunk_size=chunk_size)

class StreamingDecompressor:
  def __init__(self, url: str, parts: int = 0) -> None:
    self.buf = b""

    if parts > 1:
      self.req = _ChainedParts([f"{url}.p{i:02d}" for i in range(parts)])
    else:
      self.req = _open_image_response(url)
    self.it = self.req.iter_content(chunk_size=1024 * 1024)
    self.decompressor = lzma.LZMADecompressor(format=lzma.FORMAT_AUTO)
    self.eof = False
    self.sha256 = hashlib.sha256()

  def read(self, length: int) -> bytes:
    while len(self.buf) < length and not self.eof:
      if self.decompressor.needs_input:
        self.req.raise_for_status()

        try:
          compressed = next(self.it)
        except StopIteration:
          self.eof = True
          break
      else:
        compressed = b''

      self.buf += self.decompressor.decompress(compressed, max_length=length)

      if self.decompressor.eof:
        self.eof = True
        break

    result = self.buf[:length]
    self.buf = self.buf[length:]

    self.sha256.update(result)
    return result

def unsparsify(f: StreamingDecompressor) -> Generator[bytes, None, None]:
  magic = struct.unpack("I", f.read(4))[0]
  assert(magic == 0xed26ff3a)

  major = struct.unpack("H", f.read(2))[0]
  minor = struct.unpack("H", f.read(2))[0]
  assert(major == 1 and minor == 0)

  f.read(2)
  f.read(2)

  block_sz = struct.unpack("I", f.read(4))[0]
  f.read(4)
  num_chunks = struct.unpack("I", f.read(4))[0]
  f.read(4)

  for _ in range(num_chunks):
    chunk_type, out_blocks = SPARSE_CHUNK_FMT.unpack(f.read(12))

    if chunk_type == 0xcac1:
      yield f.read(out_blocks * block_sz)
    elif chunk_type == 0xcac2:
      filler = f.read(4) * (block_sz // 4)
      for _ in range(out_blocks):
        yield filler
    elif chunk_type == 0xcac3:
      yield b""
    else:
      raise Exception("Unhandled sparse chunk type")

def noop(f: StreamingDecompressor) -> Generator[bytes, None, None]:
  while len(chunk := f.read(1024 * 1024)) > 0:
    yield chunk

def get_target_slot_number() -> int:
  current_slot = subprocess.check_output(["abctl", "--boot_slot"], encoding='utf-8').strip()
  return 1 if current_slot == "_a" else 0

def slot_number_to_suffix(slot_number: int) -> str:
  assert slot_number in (0, 1)
  return '_a' if slot_number == 0 else '_b'

def get_partition_path(target_slot_number: int, partition: dict) -> str:
  path = f"/dev/disk/by-partlabel/{partition['name']}"

  if partition.get('has_ab', True):
    path += slot_number_to_suffix(target_slot_number)

  return path

def get_raw_hash(path: str, partition_size: int) -> str:
  raw_hash = hashlib.sha256()
  pos, chunk_size = 0, 1024 * 1024

  with open(path, 'rb+') as out:
    while pos < partition_size:
      n = min(chunk_size, partition_size - pos)
      raw_hash.update(out.read(n))
      pos += n

  return raw_hash.hexdigest().lower()

def verify_partition(target_slot_number: int, partition: dict[str, str | int], force_full_check: bool = False) -> bool:
  full_check = partition['full_check'] or force_full_check
  path = get_partition_path(target_slot_number, partition)

  if not isinstance(partition['size'], int):
    return False

  partition_size: int = partition['size']

  if not isinstance(partition['hash_raw'], str):
    return False

  partition_hash: str = partition['hash_raw']

  if full_check:
    return get_raw_hash(path, partition_size) == partition_hash.lower()
  else:
    with open(path, 'rb+') as out:
      out.seek(partition_size)
      return out.read(64) == partition_hash.lower().encode()

def clear_partition_hash(target_slot_number: int, partition: dict) -> None:
  path = get_partition_path(target_slot_number, partition)
  with open(path, 'wb+') as out:
    partition_size = partition['size']

    out.seek(partition_size)
    out.write(b"\x00" * 64)
    os.sync()

def extract_compressed_image(target_slot_number: int, partition: dict, cloudlog):
  path = get_partition_path(target_slot_number, partition)
  downloader = StreamingDecompressor(partition['url'], parts=int(partition.get('url_parts', 0)))

  with open(path, 'wb+') as out:
    last_p = 0
    raw_hash = hashlib.sha256()
    f = unsparsify if partition['sparse'] else noop
    for chunk in f(downloader):
      raw_hash.update(chunk)
      out.write(chunk)
      p = int(out.tell() / partition['size'] * 100)
      if p != last_p:
        last_p = p
        print(f"Installing {partition['name']}: {p}", flush=True)

    if raw_hash.hexdigest().lower() != partition['hash_raw'].lower():
      raise Exception(f"Raw hash mismatch '{raw_hash.hexdigest().lower()}'")

    if downloader.sha256.hexdigest().lower() != partition['hash'].lower():
      raise Exception("Uncompressed hash mismatch")

    if out.tell() != partition['size']:
      raise Exception("Uncompressed size mismatch")

    os.sync()

def extract_casync_image(target_slot_number: int, partition: dict, cloudlog):
  path = get_partition_path(target_slot_number, partition)
  seed_path = path[:-1] + ('b' if path[-1] == 'a' else 'a')

  target = casync.parse_caibx(partition['casync_caibx'])

  sources: list[tuple[str, casync.ChunkReader, casync.ChunkDict]] = []

  try:
    raw_hash = get_raw_hash(seed_path, partition['size'])
    caibx_url = f"{CAIBX_URL}{partition['name']}-{raw_hash}.caibx"

    try:
      cloudlog.info(f"casync fetching {caibx_url}")
      sources += [('seed', casync.FileChunkReader(seed_path), casync.build_chunk_dict(casync.parse_caibx(caibx_url)))]
    except requests.RequestException:
      cloudlog.error(f"casync failed to load {caibx_url}")
  except Exception:
    cloudlog.exception("casync failed to hash seed partition")

  sources += [('target', casync.FileChunkReader(path), casync.build_chunk_dict(target))]

  sources += [('remote', casync.RemoteChunkReader(partition['casync_store']), casync.build_chunk_dict(target))]

  last_p = 0

  def progress(cur):
    nonlocal last_p
    p = int(cur / partition['size'] * 100)
    if p != last_p:
      last_p = p
      print(f"Installing {partition['name']}: {p}", flush=True)

  stats = casync.extract(target, sources, path, progress)
  cloudlog.error(f'casync done {json.dumps(stats)}')

  os.sync()
  if not verify_partition(target_slot_number, partition, force_full_check=True):
    raise Exception(f"Raw hash mismatch '{partition['hash_raw'].lower()}'")

def flash_partition(target_slot_number: int, partition: dict, cloudlog, standalone=False):
  cloudlog.info(f"Downloading and writing {partition['name']}")

  if verify_partition(target_slot_number, partition):
    cloudlog.info(f"Already flashed {partition['name']}")
    return

  full_check = partition['full_check']
  if not full_check:
    clear_partition_hash(target_slot_number, partition)

  path = get_partition_path(target_slot_number, partition)

  if ('casync_caibx' in partition) and not standalone:
    extract_casync_image(target_slot_number, partition, cloudlog)
  else:
    extract_compressed_image(target_slot_number, partition, cloudlog)

  if not full_check:
    with open(path, 'wb+') as out:
      out.seek(partition['size'])
      out.write(partition['hash_raw'].lower().encode())

def swap(manifest_path: str, target_slot_number: int, cloudlog) -> None:
  verify_manifest_signature(manifest_path)
  update = json.load(open(manifest_path))
  for partition in update:
    if not partition.get('full_check', False):
      clear_partition_hash(target_slot_number, partition)

  while True:
    out = subprocess.check_output(f"abctl --set_active {target_slot_number}", shell=True, stderr=subprocess.STDOUT, encoding='utf8')
    if ("No such file or directory" not in out) and ("lun as boot lun" in out):
      cloudlog.info(f"Swap successful {out}")
      break
    else:
      cloudlog.error(f"Swap failed {out}")

def flash_agnos_update(manifest_path: str, target_slot_number: int, cloudlog, standalone=False) -> None:
  verify_manifest_signature(manifest_path)
  update = json.load(open(manifest_path))

  cloudlog.info(f"Target slot {target_slot_number}")

  os.system(f"abctl --set_unbootable {target_slot_number}")

  for partition in update:
    success = False

    for retries in range(10):
      try:
        flash_partition(target_slot_number, partition, cloudlog, standalone)
        success = True
        break

      except requests.exceptions.RequestException:
        cloudlog.exception("Failed")
        cloudlog.info(f"Failed to download {partition['name']}, retrying ({retries})")
        time.sleep(10)

    if not success:
      cloudlog.info(f"Failed to flash {partition['name']}, aborting")
      raise Exception("Maximum retries exceeded")

  cloudlog.info(f"AGNOS ready on slot {target_slot_number}")

def verify_agnos_update(manifest_path: str, target_slot_number: int) -> bool:
  verify_manifest_signature(manifest_path)
  update = json.load(open(manifest_path))
  return all(verify_partition(target_slot_number, partition) for partition in update)

if __name__ == "__main__":
  import argparse
  import logging

  parser = argparse.ArgumentParser(description="Flash and verify AGNOS update",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("--verify", action="store_true", help="Verify and perform swap if update ready")
  parser.add_argument("--swap", action="store_true", help="Verify and perform swap, downloads if necessary")
  parser.add_argument("manifest", help="Manifest json")
  args = parser.parse_args()

  logging.basicConfig(level=logging.INFO)

  target_slot_number = get_target_slot_number()
  if args.verify:
    if verify_agnos_update(args.manifest, target_slot_number):
      swap(args.manifest, target_slot_number, logging)
      exit(0)
    exit(1)
  elif args.swap:
    while not verify_agnos_update(args.manifest, target_slot_number):
      logging.error("Verification failed. Flashing AGNOS")
      flash_agnos_update(args.manifest, target_slot_number, logging, standalone=True)

    logging.warning(f"Verification succeeded. Swapping to slot {target_slot_number}")
    swap(args.manifest, target_slot_number, logging)
  else:
    flash_agnos_update(args.manifest, target_slot_number, logging, standalone=True)
