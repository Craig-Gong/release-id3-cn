"""Unified IQ-link longitudinal policy (P0–P5).

Priority (lowest wins for v_cap; tightest a_pos_cap wins):
  camera/interval > dest approach > dual TBT > congestion > light accel gate

Hard rules:
  * Only min() speed / clamp positive accel — never fake obstacles or desire.
  * Clear on IQ-link stale / disabled / P·R / gas (soft gates only).
  * Near radar lead: skip empty-road accel gates; keep camera/interval caps.
  * No stop-gap changes (avoids fighting 3.5 m follow / red offset).
"""
from __future__ import annotations

import json
import math
import os
import tempfile
import time
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.sunnypilot.nav.hud_copy import APPROACH_DEST, CAMERA_AHEAD, INTERVAL_SPEED
from openpilot.sunnypilot.nav.protocol import _turn_bucket
from openpilot.sunnypilot.nav.snapshot import NavSnapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import snapshot_long_ok

POLICY_SHM_PATH = "/dev/shm/sp_nav_policy.json"

# --- camera / interval ---
SDI_POINT = 4
SDI_INTERVAL_START = 8
SDI_INTERVAL_END = 9
POINT_ARM_M = 300.0
INTERVAL_TIMEOUT_S = 180.0
CAMERA_TOAST_HOLD_S = 3.5

# --- destination ---
DEST_APPROACH_M = 1500.0
DEST_APPROACH_MIN_M = 200.0
DEST_LAST_M = 500.0
DEST_URBAN_LIMIT_KPH = 55.0
DEST_URBAN_V_KPH = 55.0
DEST_A_POS = 0.85
DEST_NAME_HINTS = ("停车", "停车场", "地库", "车库", "P库", "park", "parking")
DEST_ROAD_HINTS = ("匝道", "辅路", "园区", "地库", "内部路", "小区")

# --- dual TBT ---
DUAL_CUR_MAX_M = 120.0
DUAL_NEXT_MIN_M = 80.0
DUAL_NEXT_MAX_M = 350.0
DUAL_DECEL = 0.9
DUAL_FLOOR_MS = 12.0  # ~43 km/h — lighter than turn_prep 20
DUAL_HIGHWAY_KPH = 70.0

# --- congestion ETA ---
CONG_MIN_GO_M = 800.0
CONG_MAX_LIMIT_KPH = 75.0
CONG_ENTER_DT_PER_DM = 0.45   # s gained per metre lost (slow progress)
CONG_EXIT_DT_PER_DM = 0.18
CONG_A_POS = 0.70
CONG_SAMPLE_S = 4.0

# --- light accel gate ---
GREEN_SHORT_S = 2.5
GREEN_STANDSTILL_A = 0.35


@dataclass(frozen=True)
class NavCruiseDecision:
  v_cap_ms: float | None = None
  a_pos_cap: float | None = None
  toast: str = ""
  mode: str = "idle"


def _min_cap(cur: float | None, nxt: float | None) -> float | None:
  if nxt is None:
    return cur
  if cur is None:
    return float(nxt)
  return float(min(cur, nxt))


def _min_a(cur: float | None, nxt: float | None) -> float | None:
  if nxt is None:
    return cur
  if cur is None:
    return float(nxt)
  return float(min(cur, nxt))


def _urban_dest_scene(snap: NavSnapshot, v_ego: float) -> bool:
  road = float(snap.road_limit_kph or 0.0)
  v_kph = float(v_ego) * CV.MS_TO_KPH
  if 0.0 < road <= DEST_URBAN_LIMIT_KPH:
    return True
  if v_kph <= DEST_URBAN_V_KPH and road < DUAL_HIGHWAY_KPH:
    return True
  name = (snap.goal_name or "").lower()
  if any(h.lower() in name for h in DEST_NAME_HINTS):
    return True
  road_name = snap.pos_road_name or ""
  if any(h in road_name for h in DEST_ROAD_HINTS):
    return True
  return False


def _next_is_turn_or_exit(tbt_type: int) -> bool:
  bucket = _turn_bucket(int(tbt_type))
  return bucket.startswith("turn") or bucket == "exit" or bucket == "roundabout"


def write_policy_hud(*, toast: str, mode: str, now: float | None = None,
                     toast_until: float = 0.0) -> None:
  clock = time.monotonic() if now is None else float(now)
  until = float(toast_until) if toast else 0.0
  payload = {
    "toast": toast or "",
    "mode": mode or "idle",
    "ts": clock,
    "toast_until": until,
  }
  directory = os.path.dirname(POLICY_SHM_PATH) or "."
  tmp = ""
  try:
    if directory and not os.path.isdir(directory):
      return
    fd, tmp = tempfile.mkstemp(prefix=".sp_nav_pol_", dir=directory, text=True)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
      json.dump(payload, f, ensure_ascii=True, separators=(",", ":"))
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, POLICY_SHM_PATH)
  except Exception:
    if tmp:
      try:
        os.unlink(tmp)
      except OSError:
        pass


def read_policy_hud(path: str = POLICY_SHM_PATH) -> tuple[str, str]:
  """Return (toast_if_fresh, mode)."""
  try:
    with open(path, encoding="utf-8") as f:
      obj = json.load(f)
  except (OSError, json.JSONDecodeError, TypeError):
    return "", "idle"
  if not isinstance(obj, dict):
    return "", "idle"
  mode = str(obj.get("mode") or "idle")
  until = float(obj.get("toast_until") or 0.0)
  toast = str(obj.get("toast") or "")
  if toast and until > 0.0 and time.monotonic() <= until:
    return toast, mode
  return "", mode


class NavCruisePolicy:
  def __init__(self):
    self._interval_active = False
    self._interval_limit_kph = 0.0
    self._interval_until = 0.0
    self._toast = ""
    self._toast_until = 0.0
    self._toast_key: tuple | None = None
    self._cong = False
    self._eta_t = 0.0
    self._eta_go_m = 0.0
    self._eta_go_s = 0.0
    self._last_mode = "idle"

  def reset(self) -> None:
    self.__init__()

  def _arm_toast(self, key: tuple, text: str, now: float) -> None:
    if key == self._toast_key and now < self._toast_until:
      return
    self._toast_key = key
    self._toast = text
    self._toast_until = now + CAMERA_TOAST_HOLD_S

  def _camera_caps(self, snap: NavSnapshot, v_ego: float, now: float) -> tuple[float | None, str]:
    sdi = int(snap.sdi_type)
    road = float(snap.road_limit_kph or 0.0)
    toast = ""

    if sdi == SDI_INTERVAL_END:
      self._interval_active = False
      self._interval_limit_kph = 0.0
      self._interval_until = 0.0
      return None, ""

    if sdi == SDI_INTERVAL_START:
      lim = float(snap.sdi_speed_kph or 0.0)
      if lim < 20.0:
        lim = road
      if lim >= 20.0:
        self._interval_active = True
        self._interval_limit_kph = lim
        self._interval_until = now + INTERVAL_TIMEOUT_S
        self._arm_toast(("iv", int(lim)), INTERVAL_SPEED, now)
        toast = INTERVAL_SPEED

    if self._interval_active:
      if now > self._interval_until:
        self._interval_active = False
      else:
        lim = self._interval_limit_kph
        if road >= 20.0:
          lim = min(lim, road) if lim >= 20.0 else road
        if lim >= 20.0:
          return lim * CV.KPH_TO_MS, toast or (INTERVAL_SPEED if now < self._toast_until else "")

    if sdi == SDI_POINT and 0.0 < float(snap.sdi_dist_m) <= POINT_ARM_M:
      lim = float(snap.sdi_speed_kph or 0.0)
      if lim < 20.0:
        lim = road
      if road >= 20.0 and lim >= 20.0:
        lim = min(lim, road)
      if lim < 20.0:
        return None, ""
      # Cap at camera/road limit only — no fake obstacle. Planner/MPC decelerates.
      self._arm_toast(("pt", int(lim), int(snap.sdi_dist_m) // 50), CAMERA_AHEAD, now)
      return lim * CV.KPH_TO_MS, CAMERA_AHEAD

    return None, ""

  def _dest_caps(self, snap: NavSnapshot, v_ego: float) -> tuple[float | None, float | None, str]:
    go = float(snap.go_dist_m or 0.0)
    man = (snap.maneuver or "none").lower()
    if go <= 0.0 or man == "arrived":
      return None, None, ""
    if not _urban_dest_scene(snap, v_ego):
      return None, None, ""

    road = float(snap.road_limit_kph or 0.0)
    road_ms = road * CV.KPH_TO_MS if road >= 20.0 else 0.0

    # Last 500 m: dull empty-road accel; keep road limit as soft ceiling.
    if go <= DEST_LAST_M:
      return (road_ms if road_ms > 0.0 else None), DEST_A_POS, APPROACH_DEST

    # 200–1500 m: only when posted urban limit — nudge toward road, no a_cap yet.
    if DEST_APPROACH_MIN_M < go <= DEST_APPROACH_M and road_ms > 0.0 and road <= DEST_URBAN_LIMIT_KPH:
      return road_ms, None, ""
    return None, None, ""

  def _dual_tbt_cap(self, snap: NavSnapshot) -> float | None:
    cur = float(snap.tbt_dist or 0.0)
    nxt = float(snap.tbt_dist_next or 0.0)
    if not (0.0 < cur <= DUAL_CUR_MAX_M):
      return None
    if not (DUAL_NEXT_MIN_M <= nxt <= DUAL_NEXT_MAX_M):
      return None
    if not _next_is_turn_or_exit(int(snap.tbt_type_next)):
      return None
    road = float(snap.road_limit_kph or 0.0)
    # Highway soft-curve already owns far exits; avoid double meat.
    if road >= DUAL_HIGHWAY_KPH:
      return None
    man = (snap.maneuver or "none").lower()
    if man in ("arrive", "arrived"):
      return None
    cap = math.sqrt(2.0 * DUAL_DECEL * nxt)
    if road >= 20.0:
      cap = min(cap, road * CV.KPH_TO_MS)
    return max(DUAL_FLOOR_MS, float(cap))

  def _congestion_a(self, snap: NavSnapshot, now: float) -> float | None:
    go_m = float(snap.go_dist_m or 0.0)
    go_s = float(snap.go_time_s or 0.0)
    road = float(snap.road_limit_kph or 0.0)
    if go_m < CONG_MIN_GO_M or go_s < 1.0:
      self._cong = False
      return None
    if road >= CONG_MAX_LIMIT_KPH:
      self._cong = False
      return None

    if self._eta_t <= 0.0:
      self._eta_t = now
      self._eta_go_m = go_m
      self._eta_go_s = go_s
      return CONG_A_POS if self._cong else None

    dt = now - self._eta_t
    if dt < CONG_SAMPLE_S:
      return CONG_A_POS if self._cong else None

    dm = self._eta_go_m - go_m
    ds = go_s - self._eta_go_s
    self._eta_t = now
    self._eta_go_m = go_m
    self._eta_go_s = go_s

    # Progressing: distance down. Congestion if time budget grows vs progress.
    if dm > 15.0:
      ratio = ds / dm
      if ratio >= CONG_ENTER_DT_PER_DM:
        self._cong = True
      elif ratio <= CONG_EXIT_DT_PER_DM:
        self._cong = False
    elif ds > 25.0 and dm < 5.0:
      # ETA ballooning with almost no distance change (reroute / jam).
      self._cong = True

    return CONG_A_POS if self._cong else None

  def _light_a(self, snap: NavSnapshot, *, has_near_lead: bool,
               standstill: bool, v_ego: float) -> float | None:
    if has_near_lead:
      return None
    light = snap.light_token
    if light == "red" and snap.stop_for_light:
      return 0.0
    if light == "green" and float(snap.remain_s or 0.0) <= GREEN_SHORT_S:
      if standstill or float(v_ego) < 0.5:
        return GREEN_STANDSTILL_A
    return None

  def update(
    self,
    snap: NavSnapshot,
    v_ego: float,
    *,
    gear,
    has_near_lead: bool,
    standstill: bool,
    gas: bool,
    long_enabled: bool,
    now: float | None = None,
  ) -> NavCruiseDecision:
    clock = time.monotonic() if now is None else float(now)
    if not long_enabled or not snapshot_long_ok(snap, gear, now=clock):
      self.reset()
      write_policy_hud(toast="", mode="idle", now=clock)
      return NavCruiseDecision()

    v_cap: float | None = None
    a_cap: float | None = None
    modes: list[str] = []

    cam_v, cam_toast = self._camera_caps(snap, v_ego, clock)
    v_cap = _min_cap(v_cap, cam_v)
    if cam_v is not None:
      modes.append("camera")

    dest_v, dest_a, dest_toast = self._dest_caps(snap, v_ego)
    v_cap = _min_cap(v_cap, dest_v)
    # Empty-road accel dulling only — keep camera/interval v_cap with a lead.
    if not gas and not has_near_lead:
      a_cap = _min_a(a_cap, dest_a)
    if dest_a is not None or dest_v is not None:
      modes.append("dest")

    dual_v = self._dual_tbt_cap(snap)
    v_cap = _min_cap(v_cap, dual_v)
    if dual_v is not None:
      modes.append("dual_tbt")

    if has_near_lead:
      # Following: do not dull empty-road accel; freeze congestion sample clock.
      self._eta_t = 0.0
    elif not gas:
      cong_a = self._congestion_a(snap, clock)
      a_cap = _min_a(a_cap, cong_a)
      if cong_a is not None:
        modes.append("cong")

      light_a = self._light_a(
        snap, has_near_lead=False, standstill=standstill, v_ego=v_ego,
      )
      a_cap = _min_a(a_cap, light_a)
      if light_a is not None:
        modes.append("light")

    toast = ""
    if clock < self._toast_until:
      toast = self._toast
    elif cam_toast:
      toast = cam_toast
    elif dest_toast:
      toast = dest_toast
      self._arm_toast(("dest", int(snap.go_dist_m) // 100), dest_toast, clock)

    mode = "+".join(modes) if modes else "idle"
    self._last_mode = mode
    write_policy_hud(
      toast=toast, mode=mode, now=clock,
      toast_until=self._toast_until if toast else 0.0,
    )
    return NavCruiseDecision(v_cap_ms=v_cap, a_pos_cap=a_cap, toast=toast, mode=mode)


def apply_a_pos_cap(a_target: float, a_pos_cap: float | None, *, gas: bool) -> float:
  if gas or a_pos_cap is None:
    return float(a_target)
  a = float(a_target)
  if a > float(a_pos_cap):
    return float(a_pos_cap)
  return a
