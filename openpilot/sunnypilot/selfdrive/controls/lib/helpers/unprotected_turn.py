"""Cautious Unprotected Turn (CUT): slow/hold/remind — never gap-accept.

Covers left (oncoming) and right (pedestrians / cross traffic / RTOR yield).

Practical:
  - Speed cap only near the corner (≤50 m or creep), not from 80 m.
  - Hold only at standstill so the car can still creep to peek.
  - Near lead ≤12 m → follow-car mode (CUT off).
  - Gas / opposite blinker / highway LC → immediate exit.
  - Right + circular red (RTOR): keep hold until gas — do not treat remain_go as clear.
  - Never re-arms nav sticky red; RTOR exemption in protocol/standstill stays.
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
NAV_CAP_M = 50.0          # apply 12 km/h cap (aligns with RTOR window)
TURN_TRIGGER_MPS = 45.0 * CV.KPH_TO_MS
WAIT_CAP_MS = 12.0 * CV.KPH_TO_MS
CREEP_V_MPS = 3.0
WEAK_V_MPS = 25.0 * CV.KPH_TO_MS
HIGHWAY_LIMIT_MS = 70.0 * CV.KPH_TO_MS
HIGHWAY_SPEED_MS = 65.0 * CV.KPH_TO_MS
STEER_IN_DEG = 20.0
PATH_SIDE_M = 1.8
PATH_RANGE_MIN_M = 10.0
PATH_RANGE_MAX_M = 40.0
LC_STARTING = 2
LC_FINISHING = 3
STANDSTILL_V = 0.5


@dataclass(frozen=True)
class CutDecision:
  active: bool = False
  side: str = ""          # "left" | "right" | ""
  v_cap_ms: float | None = None
  hold: bool = False
  hud: bool = False
  reason: str = ""


@dataclass
class CutSnapshot:
  ts: float = 0.0
  active: bool = False
  side: str = ""
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


def _path_side_m(path_x, path_y, *, left: bool) -> float | None:
  """Peak lateral offset toward the turn side in the near path window."""
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
      scored = yf if left else -yf
      if best is None or scored > best:
        best = scored
  return best


def _nav_turn_near(snap: NavSnapshot | None, *, side: str, max_m: float) -> bool:
  if snap is None:
    return False
  if not bool(getattr(snap, "send_turn", False)):
    return False
  if str(getattr(snap, "maneuver_dir", "") or "").strip().lower() != side:
    return False
  dist = float(getattr(snap, "tbt_dist", 0.0) or 0.0)
  return 0.0 < dist <= float(max_m)


def _nav_either_near(snap: NavSnapshot | None, *, max_m: float) -> bool:
  return _nav_turn_near(snap, side="left", max_m=max_m) or _nav_turn_near(snap, side="right", max_m=max_m)


def _light_token(snap: NavSnapshot | None) -> str:
  if snap is None:
    return "none"
  return str(getattr(snap, "traffic_light", "") or "").strip().lower()


def _rtor_red(snap: NavSnapshot | None) -> bool:
  """Circular/main red with a near right turn (protocol RTOR exemption window)."""
  if not _nav_turn_near(snap, side="right", max_m=NAV_CAP_M):
    return False
  if _light_token(snap) != "red":
    return False
  # Dedicated right red arrow: not RTOR — treat as normal red (standstill owns).
  light_dir = str(getattr(snap, "light_dir", "") or "").strip().lower()
  return light_dir != "right"


def _nav_go_free(snap: NavSnapshot | None, *, side: str) -> bool:
  """Nav confirmed go — CUT must not pin over standstill_hold release.

  Right + circular red (RTOR): remain_go / countdown is NOT clearance to enter
  the crosswalk — driver still yields to pedestrians / released traffic.
  """
  if snap is None:
    return False
  light = _light_token(snap)
  if side == "right" and light == "red":
    return False
  if bool(getattr(snap, "remain_go", False)):
    return True
  return light == "green"


def _highway_blocked(snap: NavSnapshot | None, v_ego: float, posted_limit_ms: float) -> bool:
  if float(posted_limit_ms) >= HIGHWAY_LIMIT_MS:
    return True
  road = float(getattr(snap, "road_limit_kph", 0.0) or 0.0) if snap is not None else 0.0
  if road >= 70.0 and not _nav_either_near(snap, max_m=NAV_ARM_M):
    return True
  if float(v_ego) > HIGHWAY_SPEED_MS and not _nav_either_near(snap, max_m=NAV_ARM_M):
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
    if not enabled or not cut_enabled(self._params) or gas:
      return self._publish(CutDecision(reason="off"))
    if left_blinker and right_blinker:
      return self._publish(CutDecision(reason="off"))
    if near_lead:
      return self._publish(CutDecision(reason="lead"))
    if int(lane_change_state) in (LC_STARTING, LC_FINISHING):
      return self._publish(CutDecision(reason="lane_change"))
    if float(v_ego) >= TURN_TRIGGER_MPS and not standstill:
      return self._publish(CutDecision(reason="fast"))
    if _highway_blocked(snap, v_ego, posted_limit_ms):
      return self._publish(CutDecision(reason="highway"))

    nav_left = _nav_turn_near(snap, side="left", max_m=NAV_ARM_M)
    nav_right = _nav_turn_near(snap, side="right", max_m=NAV_ARM_M)
    # Nav maneuver wins; else weak stalk+turn-in.
    if nav_left and not nav_right:
      side = "left"
      nav_arm = True
    elif nav_right and not nav_left:
      side = "right"
      nav_arm = True
    else:
      side = ""
      nav_arm = False

    steer = float(steering_angle_deg)
    path_left = _path_side_m(path_x, path_y, left=True)
    path_right = _path_side_m(path_x, path_y, left=False)
    turning_left = steer >= STEER_IN_DEG or (path_left is not None and path_left >= PATH_SIDE_M)
    turning_right = steer <= -STEER_IN_DEG or (path_right is not None and path_right >= PATH_SIDE_M)

    weak_left = (
      not nav_arm
      and bool(left_blinker) and not bool(right_blinker)
      and float(v_ego) < WEAK_V_MPS
      and turning_left
    )
    weak_right = (
      not nav_arm
      and bool(right_blinker) and not bool(left_blinker)
      and float(v_ego) < WEAK_V_MPS
      and turning_right
    )
    if not side:
      if weak_left:
        side = "left"
      elif weak_right:
        side = "right"

    if not side:
      return self._publish(CutDecision(reason="no_intent"))

    # Opposite blinker cancels that side (LC intent / cancel).
    if side == "left" and right_blinker:
      return self._publish(CutDecision(reason="off"))
    if side == "right" and left_blinker:
      return self._publish(CutDecision(reason="off"))

    if nav_go_latched or _nav_go_free(snap, side=side):
      return self._publish(CutDecision(reason="nav_go"))

    nav_cap = _nav_turn_near(snap, side=side, max_m=NAV_CAP_M)
    weak = (side == "left" and weak_left) or (side == "right" and weak_right)
    reason = f"nav_{side}" if nav_arm else f"weak_{side}"
    creep = bool(standstill) or float(v_ego) <= CREEP_V_MPS
    # Cap near corner, or creeping under arm / weak turn-in.
    apply_cap = bool(nav_cap or (weak and creep) or (creep and nav_arm))
    # RTOR red: always allow cap/hold once armed at ≤50 m (even if still rolling > creep).
    if side == "right" and _rtor_red(snap):
      apply_cap = True
    v_cap = WAIT_CAP_MS if apply_cap else None
    hold = bool(apply_cap and (standstill or float(v_ego) <= STANDSTILL_V))
    hud = bool(hold or nav_cap or weak or (side == "right" and _rtor_red(snap)))
    return self._publish(CutDecision(
      active=True, side=side, v_cap_ms=v_cap, hold=hold, hud=hud, reason=reason,
    ))

  def _publish(self, decision: CutDecision) -> CutDecision:
    self._last = decision
    write_cut_snapshot(CutSnapshot(
      ts=time.monotonic(),
      active=bool(decision.active),
      side=str(decision.side or ""),
      hold=bool(decision.hold),
      hud=bool(decision.hud),
      reason=str(decision.reason or ""),
    ))
    return decision
