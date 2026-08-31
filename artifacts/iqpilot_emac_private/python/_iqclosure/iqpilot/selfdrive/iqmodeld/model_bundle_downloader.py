"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""
from __future__ import annotations

import hashlib
import json
import os

MODELS_BASE_URLS = (
  "https://git.konn3kt.com/teal/IQModels/raw/branch/main",
  "https://gitlvb.teallvbs.xyz/teal/IQModels/raw/branch/main",
)
CHUNK = 4 * 1024 * 1024
HTTP_TIMEOUT_S = 60.0
STREAM_RETRIES = 6


def _requests_auth():
  import importlib
  for mod in ("iqpilot_private.models.git_auth", "iqpilot.models_private_src.git_auth",
              "iqpilot.selfdrive.iqmodeld.models.git_auth"):
    try:
      return importlib.import_module(mod).get_requests_auth()
    except Exception:
      continue
  return None


def _hf():
  import importlib
  for mod in ("iqpilot_private.models.git_auth", "iqpilot.selfdrive.iqmodeld.models.git_auth"):
    try:
      m = importlib.import_module(mod)
      return m.get_hf_headers(), m.hf_resolve_url
    except Exception:
      continue
  return None, None


def download_hf_file(hf_path: str, dst: str, sha256: str, size: int, progress_cb=None) -> str:
  import requests
  headers, resolve = _hf()
  if resolve is None:
    raise RuntimeError("no HF credentials available")
  url = resolve(hf_path)
  os.makedirs(os.path.dirname(dst), exist_ok=True)
  tmp = dst + ".hfpart"
  last_error: Exception | None = None
  for _attempt in range(STREAM_RETRIES):
    try:
      have = os.path.getsize(tmp) if os.path.isfile(tmp) else 0
      if size and have > size:
        os.remove(tmp)
        have = 0
      if not size or have < size:
        req_headers = dict(headers)
        if have:
          req_headers["Range"] = f"bytes={have}-"
        with requests.get(url, headers=req_headers, stream=True, timeout=HTTP_TIMEOUT_S, allow_redirects=True) as r:
          r.raise_for_status()
          if have and r.status_code != 206:
            have = 0
          with open(tmp, "ab" if have else "wb") as f:
            got = have
            for chunk in r.iter_content(CHUNK):
              f.write(chunk)
              got += len(chunk)
              if progress_cb is not None and size:
                progress_cb(min(1.0, got / size))
      digest = hashlib.sha256()
      with open(tmp, "rb") as f:
        for chunk in iter(lambda: f.read(CHUNK), b""):
          digest.update(chunk)
      if size and os.path.getsize(tmp) != size:
        raise RuntimeError(f"size mismatch: {os.path.getsize(tmp)}/{size} bytes")
      if sha256 and digest.hexdigest() != sha256:
        os.remove(tmp)
        raise RuntimeError("sha256 mismatch")
      os.replace(tmp, dst)
      return dst
    except Exception as e:
      last_error = e
  raise RuntimeError(f"HF download failed: {last_error}")


def _lfs_endpoint(base_url: str) -> str:
  return base_url.split("/raw/", 1)[0] + ".git/info/lfs"


def _resolve_oid(session, base_url: str, oid: str, size: int, auth):
  import requests
  batch = session.post(f"{_lfs_endpoint(base_url)}/objects/batch",
                       data=json.dumps({"operation": "download", "transfers": ["basic"],
                                        "objects": [{"oid": oid, "size": size}]}),
                       headers={"Content-Type": "application/vnd.git-lfs+json",
                                "Accept": "application/vnd.git-lfs+json"},
                       auth=auth, timeout=HTTP_TIMEOUT_S)
  batch.raise_for_status()
  entry = batch.json()["objects"][0]
  if "actions" not in entry:
    raise requests.RequestException(f"LFS object unavailable: {entry.get('error', oid)}")
  action = entry["actions"]["download"]
  return action["href"], action.get("header", {})


def _part_path(dst: str, oid: str) -> str:
  return os.path.join(dst + ".parts", oid)


def _part_complete(path: str, oid: str, size: int) -> bool:
  if not os.path.isfile(path) or os.path.getsize(path) != size:
    return False
  digest = hashlib.sha256()
  with open(path, "rb") as f:
    for chunk in iter(lambda: f.read(CHUNK), b""):
      digest.update(chunk)
  return digest.hexdigest() == oid


def _fetch_part(session, base_url: str, obj: dict, path: str, auth, progress) -> None:
  size = int(obj["size"])
  have = os.path.getsize(path) if os.path.isfile(path) else 0
  if have > size:
    os.remove(path)
    have = 0
  href, headers = _resolve_oid(session, base_url, obj["oid"], size, auth)
  obj_auth = None if headers.get("Authorization") else auth
  # LFS parts are content-addressed (oid == sha256), so a half-written part can be resumed with a
  # Range request and verified afterwards instead of being thrown away on every restart.
  if have:
    headers = {**headers, "Range": f"bytes={have}-"}
  with session.get(href, headers=headers, stream=True, timeout=HTTP_TIMEOUT_S, auth=obj_auth) as r:
    r.raise_for_status()
    if have and r.status_code != 206:
      have = 0
    with open(path, "ab" if have else "wb") as f:
      for chunk in r.iter_content(CHUNK):
        f.write(chunk)
        progress(len(chunk))


def download_lfs_bundle(objects: list, dst: str, sha256: str, size: int, progress_cb=None) -> str:
  import requests
  auth = _requests_auth()
  session = requests.Session()
  os.makedirs(dst + ".parts", exist_ok=True)
  total = int(size) or sum(int(o["size"]) for o in objects)
  done_bytes = sum(int(o["size"]) for o in objects if _part_complete(_part_path(dst, o["oid"]), o["oid"], int(o["size"])))
  got = [done_bytes]

  def progress(n: int) -> None:
    got[0] += n
    if progress_cb is not None and total:
      progress_cb(min(1.0, got[0] / total))

  last_error: Exception | None = None
  for base_url in MODELS_BASE_URLS:
    for _attempt in range(STREAM_RETRIES):
      try:
        for obj in objects:
          path = _part_path(dst, obj["oid"])
          if _part_complete(path, obj["oid"], int(obj["size"])):
            continue
          got[0] = done_bytes
          _fetch_part(session, base_url, obj, path, auth, progress)
          if not _part_complete(path, obj["oid"], int(obj["size"])):
            if os.path.getsize(path) >= int(obj["size"]):
              os.remove(path)
            raise RuntimeError(f"part {obj['oid'][:12]} incomplete or failed verification")
          done_bytes += int(obj["size"])
          got[0] = done_bytes
        break
      except Exception as e:
        last_error = e
    else:
      continue
    break
  else:
    raise RuntimeError(f"model bundle download failed: {last_error}")

  tmp = dst + ".part"
  digest = hashlib.sha256()
  with open(tmp, "wb") as out:
    for obj in objects:
      with open(_part_path(dst, obj["oid"]), "rb") as f:
        for chunk in iter(lambda: f.read(CHUNK), b""):
          out.write(chunk)
          digest.update(chunk)
  if total and os.path.getsize(tmp) != total:
    os.remove(tmp)
    raise RuntimeError(f"size mismatch: {os.path.getsize(tmp) if os.path.exists(tmp) else 0}/{total} bytes")
  if sha256 and digest.hexdigest() != sha256:
    os.remove(tmp)
    for obj in objects:
      try:
        os.remove(_part_path(dst, obj["oid"]))
      except OSError:
        pass
    raise RuntimeError("sha256 mismatch")
  os.replace(tmp, dst)
  for obj in objects:
    try:
      os.remove(_part_path(dst, obj["oid"]))
    except OSError:
      pass
  try:
    os.rmdir(dst + ".parts")
  except OSError:
    pass
  return dst
