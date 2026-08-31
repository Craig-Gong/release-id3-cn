#!/usr/bin/env python3
"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
import hashlib
import json
import platform
import threading
import time
from pathlib import Path

import requests

from iqpilot.common.params import Params
from iqpilot.common.swaglog import cloudlog
from iqpilot.ui.onroad.offline_tiles import offline_map_root

try:
  from iqpilot.iq_maps.tiles_auth import get_base_urls as _private_base_urls, get_requests_auth as _private_auth
except Exception:  # ProprietaryModuleMissing or import errors in stripped builds
  _private_base_urls = None
  _private_auth = None

# Tile bundles live as LFS objects in the PRIVATE repo IQ.Lvbs/iqmaps (R2 is gone).
# Anonymous access 404s by design; devices authenticate with the embedded read-only PAT
# carried by the closed-source updater bundle (same fetch account as the OS images).
DEFAULT_TILE_BUNDLE_BASE_URL = "https://git.konn3kt.com/IQ.Lvbs/iqmaps/raw/branch/master"
FALLBACK_TILE_BUNDLE_BASE_URL = "https://gitlvb.teallvbs.xyz/IQ.Lvbs/iqmaps/raw/branch/master"

# Gitea /raw NEVER returns LFS content -- it returns this pointer, and the real bytes come
# from the LFS batch API (see _resolve_object_url).
LFS_POINTER_MAGIC = b"version https://git-lfs"
BASE_URL_PARAM = "OfflineTilesBaseUrl"
PROGRESS_PARAM = "OfflineTilesDownloadProgress"
REQUEST_PARAM = "OfflineTilesDownloadRequest"
CHUNK_BYTES = 1 << 20
# must match scripts/iqpilot/tile_factory/upload_bundles_lfs.py
PART_BYTES = 90 * 1024 * 1024
HTTP_TIMEOUT_S = 30.0
STREAM_RETRIES = 8


def candidate_base_urls(params: Params) -> list[str]:
  override = params.get(BASE_URL_PARAM)
  if isinstance(override, bytes):
    override = override.decode("utf-8", errors="ignore")
  override = (override or "").strip()
  if override:
    return [override.rstrip("/")]
  urls: list[str] = []
  if _private_base_urls is not None:
    try:
      urls.extend(url.rstrip("/") for url in _private_base_urls())
    except Exception:
      pass
  urls.append(DEFAULT_TILE_BUNDLE_BASE_URL)
  urls.append(FALLBACK_TILE_BUNDLE_BASE_URL)
  return urls


def _maps_auth_module():
  """The read PAT lives in the compiled updater bundle (never in this open file)."""
  try:
    from iqpilot.system.proprietary_runtime._verified_import import import_verified_module
    return import_verified_module("iqpilot_updater_private", "iqpilot_private.updater.git_remote")
  except Exception:
    pass
  try:
    import importlib
    import os
    import sys
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    bundle_python = os.path.join(root, "artifacts", "iqpilot_updater_private", "python")
    if os.path.isdir(bundle_python):
      if bundle_python not in sys.path:
        sys.path.insert(0, bundle_python)
      return importlib.import_module("iqpilot_private.updater.git_remote")
  except Exception:
    pass
  return None


def request_headers(url: str) -> dict:
  mod = _maps_auth_module()
  if mod is not None:
    try:
      headers = mod.map_tiles_headers(url)
      if headers:
        return headers
    except Exception:
      pass
  try:
    from iqpilot.common.git_creds import get_credentials
    creds = get_credentials()
    if creds and all(creds) and "/iq.lvbs/iqmaps" in url.lower():
      import base64
      return {"Authorization": "Basic " + base64.b64encode(f"{creds[0]}:{creds[1]}".encode()).decode()}
  except Exception:
    pass
  return {}


def request_auth() -> tuple[str, str] | None:
  if _private_auth is None:
    return None
  try:
    return _private_auth()
  except Exception:
    return None


def _lfs_endpoint(base_url: str) -> str:
  """<host>/<owner>/<repo>/raw/branch/<b> -> <host>/<owner>/<repo>.git/info/lfs"""
  return base_url.split("/raw/", 1)[0] + ".git/info/lfs"


def _resolve_oid_url(session: requests.Session, base_url: str, oid: str, size: int,
                     headers: dict) -> tuple[str, dict]:
  """Bundles are stored as bare LFS objects addressed by oid from the index -- no pointer
  files, because committing one per part meant hundreds of concurrent commits per branch."""
  batch = session.post(f"{_lfs_endpoint(base_url)}/objects/batch",
                       data=json.dumps({"operation": "download", "transfers": ["basic"],
                                        "objects": [{"oid": oid, "size": size}]}),
                       headers={"Content-Type": "application/vnd.git-lfs+json",
                                "Accept": "application/vnd.git-lfs+json", **headers},
                       timeout=HTTP_TIMEOUT_S)
  batch.raise_for_status()
  entry = batch.json()["objects"][0]
  if "actions" not in entry:
    raise requests.RequestException(f"LFS object unavailable: {entry.get('error', oid)}")
  action = entry["actions"]["download"]
  return action["href"], action.get("header", {})


def _resolve_object_url(session: requests.Session, url: str, headers: dict) -> tuple[str, dict]:
  """Follow a Gitea LFS pointer to the real (pre-signed) object URL.

  Returns the URL to stream plus any extra headers it needs. A plain host that serves the
  bytes directly (local test server, static mirror) resolves to itself unchanged."""
  probe = session.get(url, headers={**headers, "Accept-Encoding": None}, stream=True,
                      timeout=HTTP_TIMEOUT_S)
  probe.raise_for_status()
  if int(probe.headers.get("content-length") or 0) >= 1024:
    probe.close()
    return url, {}
  body = probe.content
  probe.close()
  if not body.startswith(LFS_POINTER_MAGIC):
    return url, {}

  meta = dict(line.split(" ", 1) for line in body.decode().strip().splitlines() if " " in line)
  oid = meta["oid"].split(":", 1)[1]
  size = int(meta["size"])
  lfs_base = url.split("/raw/", 1)[0] + ".git/info/lfs"
  batch = session.post(f"{lfs_base}/objects/batch",
                       data=json.dumps({"operation": "download", "transfers": ["basic"],
                                        "objects": [{"oid": oid, "size": size}]}),
                       headers={"Content-Type": "application/vnd.git-lfs+json",
                                "Accept": "application/vnd.git-lfs+json", **headers},
                       timeout=HTTP_TIMEOUT_S)
  batch.raise_for_status()
  action = batch.json()["objects"][0]["actions"]["download"]
  return action["href"], action.get("header", {})


def fetch_index(base_url: str, session: requests.Session) -> dict:
  index_url = f"{base_url}/index.json"
  headers = request_headers(index_url)
  # requests' auth= rewrites the Authorization header, so only fall back to it when the
  # closed-source bundle gave us nothing.
  response = session.get(index_url, timeout=HTTP_TIMEOUT_S, headers=headers,
                         auth=None if headers else request_auth())
  response.raise_for_status()
  index = response.json()
  regions = index.get("regions")
  if not isinstance(regions, dict):
    raise ValueError("tile bundle index has no regions")
  return regions


def region_bundle_dir(selector: str) -> Path:
  return offline_map_root() / "regions" / selector


def region_bundle_path(selector: str) -> Path:
  return region_bundle_dir(selector) / "tiles" / "offline.mbtiles"


def region_bundle_installed(selector: str) -> bool:
  return region_bundle_path(selector).exists()


def installed_region_selectors() -> list[str]:
  regions_root = offline_map_root() / "regions"
  if not regions_root.exists():
    return []
  return sorted(
    child.name for child in regions_root.iterdir()
    if child.is_dir() and (child / "tiles" / "offline.mbtiles").exists()
  )


def _hash_existing(path: Path) -> tuple["hashlib._Hash", int]:
  digest = hashlib.sha256()
  size = 0
  with open(path, "rb") as f:
    while True:
      chunk = f.read(CHUNK_BYTES)
      if not chunk:
        break
      digest.update(chunk)
      size += len(chunk)
  return digest, size


def _write_manifest(selector: str, entry: dict) -> None:
  manifest = {
    "region": selector,
    "version": entry.get("version", ""),
    "mbtiles": {
      "bounds": entry.get("bounds", ""),
      "minzoom": entry.get("minzoom"),
      "maxzoom": entry.get("maxzoom"),
      "bytes": entry.get("bytes"),
      "sha256": entry.get("sha256", ""),
    },
  }
  if entry.get("day_path"):
    manifest["mbtiles_day"] = {
      "bytes": entry.get("day_bytes"),
      "sha256": entry.get("day_sha256", ""),
    }
  manifest_path = region_bundle_dir(selector) / "manifest.json"
  manifest_path.parent.mkdir(parents=True, exist_ok=True)
  manifest_path.write_text(json.dumps(manifest, indent=2))


class TileBundleDownloader:

  def __init__(self, params: Params | None = None, mem_params: Params | None = None,
               abort_check=None):
    self.params = params if params is not None else Params()
    if mem_params is not None:
      self.mem_params = mem_params
    else:
      self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.session = requests.Session()
    self._cancelled = threading.Event()
    self._abort_check = abort_check

  def cancel(self) -> None:
    self._cancelled.set()

  def _should_abort(self) -> bool:
    if self._cancelled.is_set():
      return True
    if not self.mem_params.get(REQUEST_PARAM):
      self._cancelled.set()
      return True
    if self._abort_check is not None and self._abort_check():
      self._cancelled.set()
      return True
    return False

  def _publish_progress(self, region: str, downloaded: int, total: int, active: bool) -> None:
    self.mem_params.put(PROGRESS_PARAM, {
      "active": active,
      "region": region,
      "downloaded_bytes": int(downloaded),
      "total_bytes": int(total),
    })

  def _download_one(self, selector: str, entry: dict, base_url: str,
                    progress_offset: int, progress_total: int) -> bool:
    night_path = region_bundle_path(selector)
    ok = self._download_file(
      selector, base_url, entry["path"], int(entry.get("bytes", 0)),
      str(entry.get("sha256", "")).strip().lower(), night_path,
      progress_offset, progress_total, int(entry.get("parts", 1)), entry.get("objects"),
    )
    if not ok:
      return False
    if entry.get("day_path"):
      day_ok = self._download_file(
        selector, base_url, entry["day_path"], int(entry.get("day_bytes", 0)),
        str(entry.get("day_sha256", "")).strip().lower(),
        night_path.with_name("offline_day.mbtiles"),
        progress_offset + int(entry.get("bytes", 0)), progress_total,
        int(entry.get("day_parts", 1)), entry.get("day_objects"),
      )
      if not day_ok:
        cloudlog.warning(f"iq_maps: day-style bundle failed for {selector}; night set installed")
    _write_manifest(selector, entry)
    cloudlog.info(f"iq_maps: installed tile bundle {selector}")
    return True

  def _download_file(self, selector: str, base_url: str, remote_path: str, expected_bytes: int,
                     expected_sha: str, final_path: Path,
                     progress_offset: int, progress_total: int, parts: int = 1,
                     objects: list | None = None) -> bool:
    # Bundles are published as <name>.pNN because Cloudflare caps proxied bodies at ~100MB.
    # They stream back-to-back into ONE .part file: concatenating afterwards would need
    # double the free space, which devices do not have.
    base = f"{base_url}/{remote_path.lstrip('/')}"
    if objects:
      urls = [None] * len(objects)   # resolved per-attempt from the oid
    else:
      urls = [base] if parts <= 1 else [f"{base}.p{i:02d}" for i in range(parts)]
    part_path = final_path.with_name(final_path.name + ".part")
    part_path.parent.mkdir(parents=True, exist_ok=True)

    downloaded = 0
    digest = hashlib.sha256()
    last_error: Exception | None = None
    for attempt in range(STREAM_RETRIES):
      if self._should_abort():
        cloudlog.warning(f"iq_maps: tile bundle download cancelled for {selector}")
        return False
      if attempt:
        time.sleep(min(30.0, 2.0 * attempt))
      try:
        digest = hashlib.sha256()
        resume_from = 0
        if part_path.exists():
          digest, resume_from = _hash_existing(part_path)
          if expected_bytes and resume_from > expected_bytes:
            part_path.unlink()
            digest = hashlib.sha256()
            resume_from = 0

        # every part but the last is exactly PART_BYTES, so a byte offset maps to a part index
        first_part = resume_from // PART_BYTES if len(urls) > 1 else 0
        skip_in_part = resume_from - first_part * PART_BYTES if len(urls) > 1 else resume_from
        downloaded = resume_from
        mode = "ab" if resume_from else "wb"
        with open(part_path, mode) as f:
          for index in range(first_part, len(urls)):
            if objects:
              url_headers = request_headers(base_url)
              auth = None if url_headers else request_auth()
              object_url, object_headers = _resolve_oid_url(
                self.session, base_url, objects[index]["oid"], int(objects[index]["size"]),
                url_headers)
            else:
              url = urls[index]
              url_headers = request_headers(url)
              auth = None if url_headers else request_auth()
              # Re-resolve per part: a pre-signed LFS object URL can expire mid-download.
              object_url, object_headers = _resolve_object_url(self.session, url, url_headers)
            headers = dict(object_headers)
            offset = skip_in_part if index == first_part else 0
            if offset:
              headers["Range"] = f"bytes={offset}-"
            response = self.session.get(object_url, headers=headers, stream=True,
                                        timeout=HTTP_TIMEOUT_S, auth=auth)
            if offset and response.status_code != 206:
              # server ignored the range: restart this whole file cleanly
              f.close()
              part_path.unlink(missing_ok=True)
              raise requests.RequestException(f"range not honoured for part {index}")
            response.raise_for_status()
            for chunk in response.iter_content(chunk_size=CHUNK_BYTES):
              if self._should_abort():
                cloudlog.warning(f"iq_maps: tile bundle download cancelled for {selector}")
                return False
              f.write(chunk)
              digest.update(chunk)
              downloaded += len(chunk)
              self._publish_progress(selector, progress_offset + downloaded, progress_total, active=True)
        break
      except requests.RequestException as exc:
        last_error = exc
        cloudlog.warning(f"iq_maps: tile bundle stream interrupted for {selector} "
                         + f"(attempt {attempt + 1}/{STREAM_RETRIES}): {exc}")
    else:
      raise requests.RequestException(f"stream failed after {STREAM_RETRIES} attempts") from last_error

    if expected_bytes and downloaded != expected_bytes:
      cloudlog.error(f"iq_maps: tile bundle size mismatch for {selector}: {downloaded} != {expected_bytes}")
      part_path.unlink(missing_ok=True)
      return False
    if expected_sha and digest.hexdigest() != expected_sha:
      cloudlog.error(f"iq_maps: tile bundle sha256 mismatch for {selector}")
      part_path.unlink(missing_ok=True)
      return False

    part_path.replace(final_path)
    return True

  def download_regions(self, selectors: list[str]) -> bool:
    self._cancelled.clear()
    ok = True
    try:
      self.mem_params.put(REQUEST_PARAM, {"regions": list(selectors)})
      regions = None
      base_url = ""
      for candidate in candidate_base_urls(self.params):
        try:
          regions = fetch_index(candidate, self.session)
          base_url = candidate
          break
        except (requests.RequestException, ValueError, json.JSONDecodeError):
          cloudlog.warning(f"iq_maps: tile bundle index unavailable at {candidate}")
      if regions is None:
        cloudlog.error("iq_maps: no tile bundle host reachable")
        return False

      wanted: list[tuple[str, dict]] = []
      for selector in selectors:
        entry = regions.get(selector)
        if entry is None:
          cloudlog.warning(f"iq_maps: no tile bundle published for {selector}")
          ok = False
          continue
        if region_bundle_installed(selector) and self._installed_matches(selector, entry):
          continue
        wanted.append((selector, entry))

      progress_total = sum(int(entry.get("bytes", 0)) + int(entry.get("day_bytes", 0)) for _, entry in wanted)
      progress_offset = 0
      for selector, entry in wanted:
        if self._should_abort():
          return False
        try:
          if not self._download_one(selector, entry, base_url, progress_offset, progress_total):
            ok = False
        except (requests.RequestException, OSError):
          cloudlog.exception(f"iq_maps: tile bundle download failed for {selector}")
          ok = False
        progress_offset += int(entry.get("bytes", 0)) + int(entry.get("day_bytes", 0))
      return ok
    finally:
      self._publish_progress("", 0, 0, active=False)
      try:
        self.mem_params.remove(REQUEST_PARAM)
      except Exception:
        pass

  @staticmethod
  def _installed_matches(selector: str, entry: dict) -> bool:
    manifest_path = region_bundle_dir(selector) / "manifest.json"
    try:
      manifest = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError):
      return False
    installed_sha = str(manifest.get("mbtiles", {}).get("sha256", "")).strip().lower()
    expected_sha = str(entry.get("sha256", "")).strip().lower()
    if not expected_sha or installed_sha != expected_sha:
      return False
    if entry.get("day_path"):
      day_file = region_bundle_dir(selector) / "tiles" / "offline_day.mbtiles"
      installed_day = str(manifest.get("mbtiles_day", {}).get("sha256", "")).strip().lower()
      expected_day = str(entry.get("day_sha256", "")).strip().lower()
      if not day_file.exists() or installed_day != expected_day:
        return False
    return True
