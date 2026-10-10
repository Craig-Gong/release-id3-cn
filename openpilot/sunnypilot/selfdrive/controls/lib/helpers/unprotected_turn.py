"""Cautious Unprotected Turn (CUT): slow/hold/remind — never gap-accept.

Practical refinements vs first draft:
  - Speed cap only near the corner (≤50 m or creep), not from 80 m.
  - Hold only at standstill so the car can still creep to see oncoming traffic.
  - Near lead ≤12 m → follow-car mode (CUT off).
  - Gas / nav green go / right blinker / highway LC → immediate exit.
"""
from __future__ import annotations

import json
import os
import tempfile
import time
from dataclasses import asdict, dataclass

from openpilot.common.constants import CV
from openpilot.common.file_params import read_file_param
from openpilot.common.params import Params, UnknownKeyName
from openpilot.sunnypilot.nav.snapshot import NavSnapshot

PARAM_KEY = "UnprotectedTurnAssist"
CUT_SHM_PATH = "/dev/shm/sp_cut.json"

NEAR_LEAD_M = 12.0
NAV_ARM_M = 80.0          # arm intent / HUD
NAV_CAP_M = 50.0          # apply 12 km/h cap
TURN_TRIGGER_MPS = 45.0 * CV.KPH_TO_MS
WAIT_CAP_MS = 12.0 * CV.KPH_TO_MS
CREEP_V_MPS = 3.0         # below this, cap applies even without near TBT
WEAK_V_MPS = 25.0 * CV.KPH_TO_MS
HIGHWAY_LIMIT_MS = 70.0 * CV.KPH_TO_MS
HIGHWAY_SPEED_MS = 65.0 * CV.KPH_TO_MS
STEER_IN_DEG = 20.0
PATH_LEFT_M = 1.8
PATH_RANGE_MIN_M = 10.0
PATH_RANGE_MAX_M = 40.0
LC_STARTING = 2
LC_FINISHING = 3
STANDSTILL_V = 0.5


@dataclass(frozen=True)
class CutDecision:
  active: bool = False
  v_cap_ms: float | None = None
  hold: bool = False
  hud: bool = False
  reason: str = ""


@dataclass
class CutSnapshot:
  ts: float = 0.0
  active: bool = False
  hold: bool = False
  hud: bool = False
  reason: str = ""


def cut_enabled(params: Params | None = None) -> bool:
  p = params or Params()
  try:
    return bool(p.get_bool(PARAM_KEY))
  except UnknownKeyName:
    return bool(read_file_param(PARAM_KEY, True))
  except Exception:
    return bool(read_file_param(PARAM_KEY, True))


def write_cut_snapshot(snap: CutSnapshot, path: str = CUT_SHM_PATH) -> None:
  directory = os.path.dirname(path) or "."
  tmp = ""
  try:
    if directory and not os.path.isdir(directory):
      return
    fd, tmp = tempfile.mkstemp(prefix=".sp_cut_", dir=directory, text=True)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
      json.dump(asdict(snap), f, ensure_ascii=True, separators=(",", ":"))
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  except Exception:
    if tmp:
      try:
        os.unlink(tmp)
      except OSError:
        pass


def read_cut_snapshot(path: str = CUT_SHM_PATH, *, now: float | None = None) -> CutSnapshot:
  try:
    with open(path, encoding="utf-8") as f:
      obj = json.load(f)
  except (OSError, json.JSONDecodeError, TypeError):
    return CutSnapshot()
  if not isinstance(obj, dict):
    return CutSnapshot()
  snap = CutSnapshot()
  for key in asdict(snap):
    if key not in obj:
      continue
    try:
      setattr(snap, key, type(getattr(snap, key))(obj[key]))
    except (TypeError, ValueError):
      continue
  clock = time.monotonic() if now is None else float(now)
  if snap.ts <= 0.0 or (clock - snap.ts) > 2.0:
    return CutSnapshot()
  return snap


def _path_left_m(path_x, path_y) -> float | None:
  if path_x is None or path_y is None:
    return None
  xs = list(path_x)
  ys = list(path_y)
  if len(xs) < 4 or len(xs) != len(ys):
    return None
  best = None
  for x, y in zip(xs, ys):
    try:
      xf, yf = float(x), float(y)
    except (TypeError, ValueError):
      continue
    if PATH_RANGE_MIN_M <= xf <= PATH_RANGE_MAX_M:
      # model y: left positive in openpilot car frame
      if best is None or yf > best:
        best = yf
  return best


def _nav_left_near(snap: NavSnapshot | None, *, max_m: float) -> bool:
  if snap is None:
    return False
  if not bool(getattr(snap, "send_turn", False)):
    return False
  if str(getattr(snap, "maneuver_dir", "") or "").strip().lower() != "left":
    return False
  dist = float(getattr(snap, "tbt_dist", 0.0) or 0.0)
  return 0.0 < dist <= float(max_m)


def _nav_go_free(snap: NavSnapshot | None) -> bool:
  """Nav confirmed go — CUT must not pin over standstill_hold release."""
  if snap is None:
    return False
  if bool(getattr(snap, "remain_go", False)):
    return True
  light = str(getattr(snap, "traffic_light", "") or "").strip().lower()
  return light == "green"


def _highway_blocked(snap: NavSnapshot | None, v_ego: float, posted_limit_ms: float) -> bool:
  if float(posted_limit_ms) >= HIGHWAY_LIMIT_MS:
    return True
  road = float(getattr(snap, "road_limit_kph", 0.0) or 0.0) if snap is not None else 0.0
  if road >= 70.0 and not _nav_left_near(snap, max_m=NAV_ARM_M):
    return True
  if float(v_ego) > HIGHWAY_SPEED_MS and not _nav_left_near(snap, max_m=NAV_ARM_M):
    return True
  return False


class UnprotectedTurnAssist:
  def __init__(self):
    self._params = Params()
    self._last = CutDecision()

  def reset(self) -> None:
    self._last = CutDecision()
    write_cut_snapshot(CutSnapshot(ts=time.monotonic()))

  def update(
    self,
    *,
    v_ego: float,
    enabled: bool,
    standstill: bool,
    gas: bool,
    left_blinker: bool,
    right_blinker: bool,
    steering_angle_deg: float,
    lane_change_state: int,
    near_lead: bool,
    posted_limit_ms: float,
    path_x=None,
    path_y=None,
    snap: NavSnapshot | None = None,
    nav_go_latched: bool = False,
  ) -> CutDecision:
    if not enabled or not cut_enabled(self._params) or gas or right_blinker:
      return self._publish(CutDecision(reason="off"))
    if near_lead:
      return self._publish(CutDecision(reason="lead"))
    if int(lane_change_state) in (LC_STARTING, LC_FINISHING):
      return self._publish(CutDecision(reason="lane_change"))
    if float(v_ego) >= TURN_TRIGGER_MPS and not standstill:
      return self._publish(CutDecision(reason="fast"))
    if _highway_blocked(snap, v_ego, posted_limit_ms):
      return self._publish(CutDecision(reason="highway"))
    if nav_go_latched or _nav_go_free(snap):
      return self._publish(CutDecision(reason="nav_go"))

    nav_arm = _nav_left_near(snap, max_m=NAV_ARM_M)
    nav_cap = _nav_left_near(snap, max_m=NAV_CAP_M)
    path_left = _path_left_m(path_x, path_y)
    turning_in = abs(float(steering_angle_deg)) >= STEER_IN_DEG or (
      path_left is not None and path_left >= PATH_LEFT_M
    )
    weak = (
      bool(left_blinker) and not bool(right_blinker)
      and not nav_arm
      and float(v_ego) < WEAK_V_MPS
      and turning_in
    )
    if not nav_arm and not weak:
      return self._publish(CutDecision(reason="no_intent"))

    active = True
    reason = "nav_left" if nav_arm else "weak_left"
    creep = bool(standstill) or float(v_ego) <= CREEP_V_MPS
    # Cap near the corner (≤50 m) or while creeping under an armed left TBT / weak turn.
    apply_cap = bool(nav_cap or (weak and creep) or (creep and nav_arm))
    v_cap = WAIT_CAP_MS if apply_cap else None
    # Hold only when stopped under apply_cap — rolling creep stays free to peek.
    hold = bool(apply_cap and (standstill or float(v_ego) <= STANDSTILL_V))
    hud = bool(active and (hold or nav_cap or weak))
    return self._publish(CutDecision(active=active, v_cap_ms=v_cap, hold=hold, hud=hud, reason=reason))

  def _publish(self, decision: CutDecision) -> CutDecision:
    self._last = decision
    write_cut_snapshot(CutSnapshot(
      ts=time.monotonic(),
      active=bool(decision.active),
      hold=bool(decision.hold),
      hud=bool(decision.hud),
      reason=str(decision.reason or ""),
    ))
    return decision
