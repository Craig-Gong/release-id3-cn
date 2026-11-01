"""MEB software blinker request (EA_02 path) + gated nav auto LC intent.

Static / product params (file-backed; default all off):
  MebForceBlinker   0=off 1=left 2=right   — Phase B static probe (auto-expire)
  NavAutoBlinker    bool                  — IQ-link near turn → request blinker
  NavAutoLaneChange bool                  — near fork/exit LC → blinker (ALC via
                                            existing AutoLaneChangeTimer; needs
                                            timer > Nudge and BSM delay recommended)

Does NOT send CAN. controlsd ORs the result into CarControl.left/rightBlinker;
opendbc carcontroller TX EA_02.

Test matrix (docstring for road/static SOP): see TEST_PLAN below.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

from openpilot.common.file_params import read_file_param, write_file_param
from openpilot.sunnypilot.nav.protocol import NAV_LATERAL_TURN_M, TURN_DESIRE_WINDOW_M
from openpilot.sunnypilot.nav.snapshot import NavSnapshot, snapshot_executable
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import (
  nav_intersection_turn,
  nav_long_blocked,
)

# --- file params ---
FORCE_BLINKER_PARAM = "MebForceBlinker"          # int 0/1/2
NAV_AUTO_BLINKER_PARAM = "NavAutoBlinker"        # bool
NAV_AUTO_LC_PARAM = "NavAutoLaneChange"          # bool

FORCE_BLINKER_HOLD_S = 3.0
# Urban intersection auto-blinker arm window (stricter than toast 150 m).
TURN_ARM_MIN_M = 40.0
TURN_ARM_MAX_M = 100.0
TURN_ARM_TIME_S = 3.5
TURN_MAX_KPH = 55.0
# Highway / fork LC: only when NavAutoLaneChange on.
LC_ARM_MIN_M = 60.0
LC_ARM_MAX_M = 250.0
LC_MIN_KPH = 45.0
LC_MAX_KPH = 120.0
LC_ROAD_MIN_KPH = 70.0  # below this, treat as urban — no auto LC

TEST_PLAN = """
STATIC (P + READY, person outside watching corner lamps)
  A Listen: hand stalk L/R — confirm Blinkmodi BM_* / carState blinker; note EA_02 bus.
  B Force:  echo -n 1 > /data/openpilot_extra_params/MebForceBlinker  (left 3s)
            echo -n 2 > .../MebForceBlinker  (right 3s); echo -n 0 to clear.
            PASS = exterior lamp + BM_* + no Cruise Fault / relay fault.
  C Stalk cancel: while force on, flick stalk — software must yield.
  D Engage @0: force again while latActive if possible — no TSK fault.
  Requires panda FW with EA_02 TX allow (rebuild safety + flash signed; NOT recover).

GATED ONROAD (only after A–D pass; defaults stay OFF)
  E NavAutoBlinker=1: urban send_turn near — blinker only, no auto LC.
  F NavAutoLaneChange=1 + AutoLaneChangeTimer=1s + BsmDelay: fork/exit only,
     road≥70, 45–120 km/h, BSM clear, not intersection turn, LinkWarn off.
  NEVER: solid-unknown + no BSM, RTOR-only, unprotected left cross-traffic,
         dual-blinker hazard, P/R, BLE stale.
"""


@dataclass(frozen=True)
class BlinkerRequest:
  left: bool = False
  right: bool = False
  source: str = "none"  # force|turn|lc|none


_force_until_mono: float = 0.0
_force_side: int = 0


def _dir(value) -> str:
  token = str(value or "none").strip().lower()
  return token if token in ("left", "right") else "none"


def read_force_blinker() -> int:
  raw = read_file_param(FORCE_BLINKER_PARAM, 0)
  try:
    return int(raw or 0)
  except (TypeError, ValueError):
    return 0


def clear_force_blinker() -> None:
  write_file_param(FORCE_BLINKER_PARAM, 0)


def nav_auto_blinker_enabled() -> bool:
  return bool(read_file_param(NAV_AUTO_BLINKER_PARAM, False))


def nav_auto_lane_change_enabled() -> bool:
  return bool(read_file_param(NAV_AUTO_LC_PARAM, False))


def _arm_turn_dist_m(v_ego_mps: float) -> float:
  d = float(v_ego_mps) * TURN_ARM_TIME_S
  return max(TURN_ARM_MIN_M, min(TURN_ARM_MAX_M, d))


def _update_force(now: float) -> BlinkerRequest:
  global _force_until_mono, _force_side
  side = read_force_blinker()
  if side not in (1, 2):
    _force_until_mono = 0.0
    _force_side = 0
    return BlinkerRequest()
  if side != _force_side or _force_until_mono <= 0.0:
    _force_side = side
    _force_until_mono = now + FORCE_BLINKER_HOLD_S
  if now >= _force_until_mono:
    clear_force_blinker()
    _force_until_mono = 0.0
    _force_side = 0
    return BlinkerRequest()
  return BlinkerRequest(left=(side == 1), right=(side == 2), source="force")


def _nav_turn_request(snap: NavSnapshot, *, v_ego_mps: float, now: float | None) -> BlinkerRequest:
  if not nav_auto_blinker_enabled():
    return BlinkerRequest()
  if not snapshot_executable(snap, now=now):
    return BlinkerRequest()
  if not nav_intersection_turn(snap):
    return BlinkerRequest()
  d = _dir(snap.maneuver_dir)
  if d == "none":
    return BlinkerRequest()
  dist = float(snap.tbt_dist or 0.0)
  if dist <= 0.0 or dist > TURN_DESIRE_WINDOW_M:
    return BlinkerRequest()
  if float(v_ego_mps) * 3.6 > TURN_MAX_KPH:
    return BlinkerRequest()
  arm = _arm_turn_dist_m(v_ego_mps)
  # Near corner only (not toast-early spam).
  if not (0.0 < dist <= max(arm, NAV_LATERAL_TURN_M)):
    return BlinkerRequest()
  return BlinkerRequest(left=(d == "left"), right=(d == "right"), source="turn")


def _nav_lc_request(snap: NavSnapshot, *, v_ego_mps: float, now: float | None,
                    left_blindspot: bool, right_blindspot: bool) -> BlinkerRequest:
  """Fork/exit lane-change blinker only — never intersection turn."""
  if not nav_auto_lane_change_enabled():
    return BlinkerRequest()
  if not snapshot_executable(snap, now=now):
    return BlinkerRequest()
  if nav_intersection_turn(snap):
    return BlinkerRequest()
  maneuver = str(snap.maneuver or "").strip().lower()
  if maneuver not in ("fork", "exit"):
    return BlinkerRequest()
  d = _dir(snap.maneuver_dir)
  if d == "none":
    return BlinkerRequest()
  if d == "left" and left_blindspot:
    return BlinkerRequest()
  if d == "right" and right_blindspot:
    return BlinkerRequest()
  road = float(getattr(snap, "road_limit_kph", 0.0) or 0.0)
  if road > 0.0 and road < LC_ROAD_MIN_KPH:
    return BlinkerRequest()
  v_kph = float(v_ego_mps) * 3.6
  if not (LC_MIN_KPH <= v_kph <= LC_MAX_KPH):
    return BlinkerRequest()
  dist = float(snap.tbt_dist or 0.0)
  if not (LC_ARM_MIN_M <= dist <= LC_ARM_MAX_M):
    return BlinkerRequest()
  return BlinkerRequest(left=(d == "left"), right=(d == "right"), source="lc")


def evaluate_blinker_request(
  *,
  snap: NavSnapshot | None,
  v_ego_mps: float,
  gear=None,
  left_blindspot: bool = False,
  right_blindspot: bool = False,
  left_blinker_active: bool = False,
  right_blinker_active: bool = False,
  now: float | None = None,
) -> BlinkerRequest:
  """Priority: stalk/BCM active → none; force → turn → lc."""
  t = time.monotonic() if now is None else float(now)
  if left_blinker_active or right_blinker_active:
    return BlinkerRequest(source="yield_stalk")
  if gear is not None and nav_long_blocked(gear):
    return BlinkerRequest()

  force = _update_force(t)
  if force.source == "force":
    return force

  if snap is None:
    return BlinkerRequest()

  turn = _nav_turn_request(snap, v_ego_mps=v_ego_mps, now=t)
  if turn.source == "turn":
    return turn

  return _nav_lc_request(
    snap, v_ego_mps=v_ego_mps, now=t,
    left_blindspot=left_blindspot, right_blindspot=right_blindspot,
  )
