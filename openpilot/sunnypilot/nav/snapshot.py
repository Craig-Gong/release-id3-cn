"""Atomic nav snapshot in /dev/shm. Planner and UI read this; iqlinkd writes it."""
from __future__ import annotations

import json
import os
import tempfile
import time
from dataclasses import asdict, dataclass
from typing import Any

NAV_SHM_PATH = "/dev/shm/sp_nav.json"
HUD_SHM_PATH = "/dev/shm/sp_nav_hud.json"
CLUSTER_SHM_PATH = "/dev/shm/sp_cluster_hud.json"
INJECT_SHM_PATH = "/dev/shm/sp_nav_inject.json"
STALE_LINK_S = 2.5


@dataclass
class NavSnapshot:
  ts: float = 0.0
  link_ok: bool = False
  link_state: int = 0
  iqlink_enabled: bool = False
  traffic_light: str = "none"
  dist_m: float = 0.0
  remain_s: float = 0.0
  remain_go: bool = False
  stop_for_light: bool = False
  speed_target: float = 0.0
  accel_target: float = 0.0
  lane_recommend: str = "none"
  maneuver: str = "none"
  maneuver_dir: str = "none"
  tbt_dist: float = 0.0
  road_limit_kph: float = 0.0
  send_turn: bool = False

  @property
  def apk_green(self) -> bool:
    return self.traffic_light == "green"

  @property
  def light_token(self) -> str:
    token = str(self.traffic_light or "none").strip().lower()
    return token if token in ("red", "yellow", "green") else "none"


def _atomic_write(path: str, payload: dict[str, Any]) -> None:
  directory = os.path.dirname(path) or "."
  fd, tmp = tempfile.mkstemp(prefix=".sp_nav_", dir=directory, text=True)
  try:
    with os.fdopen(fd, "w", encoding="utf-8") as f:
      json.dump(payload, f, ensure_ascii=True, separators=(",", ":"))
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  except Exception:
    try:
      os.unlink(tmp)
    except OSError:
      pass
    raise


def write_snapshot(snap: NavSnapshot, path: str = NAV_SHM_PATH) -> None:
  _atomic_write(path, asdict(snap))


def write_cluster_hud(*, approaching: bool, standstill: bool) -> None:
  _atomic_write(CLUSTER_SHM_PATH, {
    "approaching": bool(approaching),
    "standstill": bool(standstill),
    "ts": time.monotonic(),
  })


def read_cluster_hud(path: str = CLUSTER_SHM_PATH) -> tuple[bool, bool]:
  try:
    with open(path, encoding="utf-8") as f:
      obj = json.load(f)
  except (OSError, json.JSONDecodeError, TypeError):
    return False, False
  if not isinstance(obj, dict):
    return False, False
  return bool(obj.get("approaching")), bool(obj.get("standstill"))


def read_snapshot(path: str = NAV_SHM_PATH) -> NavSnapshot:
  try:
    with open(path, encoding="utf-8") as f:
      obj = json.load(f)
  except (OSError, json.JSONDecodeError, TypeError):
    return NavSnapshot()
  if not isinstance(obj, dict):
    return NavSnapshot()
  snap = NavSnapshot()
  for key in asdict(snap):
    if key not in obj:
      continue
    try:
      setattr(snap, key, type(getattr(snap, key))(obj[key]))
    except (TypeError, ValueError):
      continue
  return snap


def snapshot_executable(snap: NavSnapshot, *, now: float | None = None) -> bool:
  """BLE down / stale HMAC: keep HUD, do not execute leftover nav speed."""
  if not snap.iqlink_enabled or not snap.link_ok:
    return False
  clock = time.monotonic() if now is None else now
  if snap.ts <= 0.0:
    return False
  # Keep HUD via link_ok; leftover speed/accel only while the last packet is fresh.
  return (clock - snap.ts) <= STALE_LINK_S
