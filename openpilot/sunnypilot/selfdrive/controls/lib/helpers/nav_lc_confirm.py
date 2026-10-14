"""Nav LC Confirm: button-confirmed lane change from IQ-link — no EA_02 / stalk.

Spec locks (do not re-litigate without road-test failure):
  - One confirm = one LC FSM pass; multi-lane = re-arm up to MAX_CONFIRMS=4.
  - SATISFIED = stable road edge | laneRecommend=straight | TBT out of window.
  - No BSM hardware → feature disabled. Default params off.
  - Highway: fork/exit. Urban (second toggle): lc_* fork; dir = recommend else maneuver_dir.
  - Dist: exit ≤520 m, fork ≤380 m, urban ≤200 m; LO 80 / urban 35 (see constants).
  - Exit v≥45 (fork≥55); urban v≤60; exit posted limit floor 50 (fork 70).
  - Urban fork: ignore turn_active (send_turn promote must not clear Nav LC).
  - Solid line: ignore SET + DesireHelper enters preLaneChange (not Starting).
"""
from __future__ import annotations

import json
import os
import tempfile
import time
from dataclasses import asdict, dataclass, field
from enum import Enum

from openpilot.common.constants import CV
from openpilot.common.file_params import read_file_param
from openpilot.common.params import Params, UnknownKeyName
from openpilot.sunnypilot.nav.hud_copy import (
  LC_CONFIRM, LC_EDGE_LEFT, LC_EDGE_RIGHT, LC_EXEC, LC_LEFT, LC_RIGHT, LC_SIDE, LC_WAIT,
  SOLID_LINE_BLOCK,
)
from openpilot.sunnypilot.nav.snapshot import (
  NavSnapshot, light_executable, snapshot_executable,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import nav_long_blocked
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.unprotected_turn import (
  read_cut_snapshot,
)

PARAM_KEY = "NavLcConfirm"
PARAM_URBAN_KEY = "NavLcConfirmUrban"
CONFIRM_BTN_KEY = "NavLcConfirmButton"
NAV_LC_SHM_PATH = "/dev/shm/sp_nav_lc.json"

# --- locked constants (distance budget for SET-confirm LC) ---
# Highway: one LC ≈ 3–5 s; multi-lane needs earlier first prompt + shorter
# re-arm cooldown. At 100 km/h, 520 m ≈ 19 s → ~2 solid LCs (not 4).
# Budget MAX_CONFIRMS=4 still covers slow/congested traffic.
# Do NOT arm at 1–2 km (guide-sign scale) — too early / noisy.
MAX_CONFIRMS_PER_MANEUVER = 4

HWY_LIMIT_MIN_KPH = 70.0          # fork: below → not highway band
# Exit: Gaode often drops posted limit 100→60 near ramp; 50 keeps city
# false "exit" out while allowing real decelerating approaches.
HWY_EXIT_LIMIT_MIN_KPH = 50.0
HWY_V_MIN = 55.0 * CV.KPH_TO_MS   # fork / keep-side cruise band
# Exit: align DesireHelper LANE_CHANGE_SPEED_MIN (45). Covers decelerating
# to ramp without arming crawl (<45) or urban false exits (road floor).
HWY_EXIT_V_MIN = 45.0 * CV.KPH_TO_MS
HWY_V_MAX = 120.0 * CV.KPH_TO_MS
HWY_V_HYST_KPH = 3.0              # in-window speed hysteresis (anti flicker)
HWY_DIST_ENTER_LO = 80.0          # below → too close to gore / missed exit
HWY_DIST_FORK_HI = 380.0          # fork / keep-side
HWY_DIST_EXIT_HI = 520.0          # exit: ~国内 500 m 牌 + 2×LC 时间
HWY_DIST_HYST_M = 20.0
HWY_ARMED_TIMEOUT_S = 12.0        # long approach: wait for gap / BSM without flicker
HWY_COOLDOWN_S = 10.0
MULTI_COOLDOWN_S = 6.0            # still NEED after one LC

URBAN_V_MIN = 25.0 * CV.KPH_TO_MS
# City express often sits 56–65; 55 blanked the whole approach. Cap 60 keeps
# road≥70 on the highway classifier.
URBAN_V_MAX = 60.0 * CV.KPH_TO_MS
# ~12–18 s at 40–50 km/h to see HUD + SET; Gaode lane preview ~300 m is noisier.
URBAN_DIST_ENTER_LO = 35.0
URBAN_DIST_ENTER_HI = 200.0
URBAN_DIST_HYST_M = 15.0
URBAN_ARMED_TIMEOUT_S = 10.0
URBAN_COOLDOWN_S = 10.0

# Back-compat aliases (tests / callers)
HWY_DIST_ENTER_HI = HWY_DIST_FORK_HI

SATISFIED_HOLD_S = 0.5
EDGE_BLOCK_SHOW_S = 1.5
EXEC_TIMEOUT_S = 10.0
# SET on the wheel (GRA Tip_Setzen). RES/Cancel stay forbidden.
DEFAULT_CONFIRM_BTN = "setCruise"

# Never bind resume/cancel as confirm (disengage / resume semantics).
_FORBIDDEN_BTN = frozenset({"resumeCruise", "cancel", "cancelCruise"})
# States where SET is owned by Nav LC — cruise/SLA/buttonEnable must ignore SET.
_SWALLOW_SET_STATES = frozenset({"armed", "hold", "executing"})


class NavLcState(str, Enum):
  IDLE = "idle"
  ARMED = "armed"
  HOLD = "hold"
  EXECUTING = "executing"
  COOLDOWN = "cooldown"
  EDGE_BLOCK = "edge_block"


@dataclass(frozen=True)
class NavLcHud:
  show: bool = False
  head: str = ""
  detail: str = ""
  state: str = "idle"
  direction: str = "none"


@dataclass(frozen=True)
class NavLcDecision:
  state: str = NavLcState.IDLE.value
  direction: str = "none"
  execute: bool = False
  start_pulse: bool = False
  confirms_used: int = 0
  reason: str = ""
  hud: NavLcHud = field(default_factory=NavLcHud)
  mode: str = ""  # "highway" | "urban" | ""


@dataclass
class NavLcSnapshot:
  ts: float = 0.0
  show: bool = False
  head: str = ""
  detail: str = ""
  state: str = "idle"
  direction: str = "none"
  reason: str = ""
  # When True, cruise/SLA/buttonEnable must ignore SET this frame.
  swallow_set: bool = False


def nav_lc_enabled(params: Params | None = None) -> bool:
  p = params or Params()
  try:
    return bool(p.get_bool(PARAM_KEY))
  except UnknownKeyName:
    return bool(read_file_param(PARAM_KEY, False))
  except Exception:
    return bool(read_file_param(PARAM_KEY, False))


def nav_lc_urban_enabled(params: Params | None = None) -> bool:
  if not nav_lc_enabled(params):
    return False
  p = params or Params()
  try:
    return bool(p.get_bool(PARAM_URBAN_KEY))
  except UnknownKeyName:
    return bool(read_file_param(PARAM_URBAN_KEY, False))
  except Exception:
    return bool(read_file_param(PARAM_URBAN_KEY, False))


def confirm_button_name() -> str:
  raw = read_file_param(CONFIRM_BTN_KEY, DEFAULT_CONFIRM_BTN)
  name = str(raw or DEFAULT_CONFIRM_BTN).strip()
  if not name or name in _FORBIDDEN_BTN:
    return DEFAULT_CONFIRM_BTN
  return name


def nav_lc_swallow_set(path: str = NAV_LC_SHM_PATH, *, now: float | None = None) -> bool:
  """Cruise stack: ignore SET while Nav LC owns the wheel prompt.

  modeld still sees raw buttonEvents (shm is written from the prior frame's
  ARMED/HOLD/EXEC). VW Alpha Long does not change MAX on SET while already
  engaged; this gate still blocks buttonEnable re-fire and SLA minus-confirm.
  """
  snap = read_nav_lc_snapshot(path, now=now)
  return bool(snap.swallow_set or (snap.show and snap.state in _SWALLOW_SET_STATES))


def button_event_is_set(ev) -> bool:
  return _btn_type_name(getattr(ev, "type", None)) == "setCruise"


def write_nav_lc_snapshot(snap: NavLcSnapshot, path: str = NAV_LC_SHM_PATH) -> None:
  directory = os.path.dirname(path) or "."
  tmp = ""
  try:
    if directory and not os.path.isdir(directory):
      return
    fd, tmp = tempfile.mkstemp(prefix=".sp_nav_lc_", dir=directory, text=True)
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


def read_nav_lc_snapshot(path: str = NAV_LC_SHM_PATH, *, now: float | None = None) -> NavLcSnapshot:
  try:
    with open(path, encoding="utf-8") as f:
      obj = json.load(f)
  except (OSError, json.JSONDecodeError, TypeError):
    return NavLcSnapshot()
  if not isinstance(obj, dict):
    return NavLcSnapshot()
  snap = NavLcSnapshot()
  for key in asdict(snap):
    if key not in obj:
      continue
    try:
      setattr(snap, key, type(getattr(snap, key))(obj[key]))
    except (TypeError, ValueError):
      continue
  clock = time.monotonic() if now is None else float(now)
  if snap.ts <= 0.0 or (clock - snap.ts) > 2.0:
    return NavLcSnapshot()
  return snap


def _dir(value) -> str:
  token = str(value or "none").strip().lower()
  return token if token in ("left", "right") else "none"


def _gear_drive(gear) -> bool:
  if gear is None:
    return True
  if nav_long_blocked(gear):
    return False
  name = str(getattr(gear, "name", gear) or "").split(".")[-1].lower()
  return name in ("drive", "sport", "eco", "manumatic", "low", "")


def _btn_type_name(btn_type) -> str:
  return str(getattr(btn_type, "name", btn_type) or "").split(".")[-1]


def confirm_rising_edge(button_events, *, button_name: str | None = None) -> bool:
  """True on pressed=True event for the calibrated confirm button."""
  want = button_name or confirm_button_name()
  if want in _FORBIDDEN_BTN:
    want = DEFAULT_CONFIRM_BTN
  for ev in button_events or ():
    if not bool(getattr(ev, "pressed", False)):
      continue
    if _btn_type_name(getattr(ev, "type", None)) == want:
      return True
  return False


def _maneuver_key(snap: NavSnapshot | None) -> str:
  if snap is None:
    return ""
  man = str(getattr(snap, "maneuver", "") or "none").strip().lower()
  d = _dir(getattr(snap, "maneuver_dir", None))
  return f"{man}:{d}"


def _read_enable_bsm(params: Params) -> bool | None:
  """None = unknown (do not enable). True/False from CarParams (opendbc structs)."""
  try:
    raw = params.get("CarParams")
  except Exception:
    return None
  if not raw:
    return None
  try:
    # Same import path as controlsd — cereal.car is not available in this tree.
    from opendbc.car.structs import car
    import openpilot.cereal.messaging as messaging
    cp = messaging.log_from_bytes(raw, car.CarParams)
    return bool(getattr(cp, "enableBsm", False))
  except Exception:
    return None


def classify_nav_lc_intent(
  snap: NavSnapshot | None,
  *,
  v_ego: float,
  urban_ok: bool,
  in_window: bool,
  now: float | None = None,
) -> tuple[str, str, str]:
  """Return (direction, mode, reason). mode is highway|urban|''."""
  if snap is None:
    return "none", "", "no_snap"
  if not snapshot_executable(snap, now=now):
    return "none", "", "link"
  man = str(getattr(snap, "maneuver", "") or "none").strip().lower()
  d = _dir(getattr(snap, "maneuver_dir", None))
  rec = str(getattr(snap, "lane_recommend", "") or "none").strip().lower()
  if man in ("arrive", "arrived"):
    return "none", "", "arrive"
  if man == "turn":
    return "none", "", "turn"
  # Fork may resolve dir from laneRecommend below; exit still needs maneuver_dir.
  if d == "none" and man != "fork":
    return "none", "", "no_dir"

  dist = float(getattr(snap, "tbt_dist", 0.0) or 0.0)
  road = float(getattr(snap, "road_limit_kph", 0.0) or 0.0)
  send_turn = bool(getattr(snap, "send_turn", False))
  # protocol: lc_* → maneuver=fork; urban promote sets send_turn. Highway
  # fork/exit keep send_turn False. Never arm pure intersection turns here.

  # --- Highway: exit always; fork only when NOT urban-promoted ---
  hwy_candidate = man == "exit" or (man == "fork" and not send_turn)
  if hwy_candidate:
    is_exit = man == "exit"
    if d == "none":
      return "none", "", "no_dir"
    limit_floor = HWY_EXIT_LIMIT_MIN_KPH if is_exit else HWY_LIMIT_MIN_KPH
    v_min = HWY_EXIT_V_MIN if is_exit else HWY_V_MIN
    v_max = HWY_V_MAX
    if in_window:
      # Stay armed while easing for the ramp / brief speed noise.
      v_min = max(0.0, v_min - HWY_V_HYST_KPH * CV.KPH_TO_MS)
      v_max = v_max + HWY_V_HYST_KPH * CV.KPH_TO_MS
    if road > 0.0 and road < limit_floor:
      # Low posted limit → not highway band (may still be urban below).
      pass
    elif not (v_min <= float(v_ego) <= v_max):
      pass
    else:
      hi = HWY_DIST_EXIT_HI if is_exit else HWY_DIST_FORK_HI
      lo = HWY_DIST_ENTER_LO
      if in_window:
        lo = max(0.0, lo - HWY_DIST_HYST_M)
        hi = hi + HWY_DIST_HYST_M
      if lo < dist <= hi:
        return d, "highway", "ok"
      return "none", "", "hwy_dist"
    if is_exit:
      # Exit failed hwy gates — do not fall through as urban lc.
      return "none", "", "hwy_gate"

  # --- Urban: fork (often lc_*) ; second toggle ---
  # Dir: laneRecommend L/R wins; else maneuver_dir (Gaode often omits recommend).
  # straight → not needed; pure turn never reaches here.
  if not urban_ok:
    return "none", "", "urban_off"
  if man != "fork":
    return "none", "", "urban_man"
  if rec == "straight":
    return "none", "", "urban_rec"
  if rec in ("left", "right"):
    d = rec
  elif d not in ("left", "right"):
    return "none", "", "urban_rec"
  if road >= HWY_LIMIT_MIN_KPH:
    return "none", "", "urban_limit"
  if not (URBAN_V_MIN <= float(v_ego) <= URBAN_V_MAX):
    return "none", "", "urban_speed"
  lo, hi = URBAN_DIST_ENTER_LO, URBAN_DIST_ENTER_HI
  if in_window:
    lo = max(0.0, lo - URBAN_DIST_HYST_M)
    hi = hi + URBAN_DIST_HYST_M
  if not (lo < dist <= hi):
    return "none", "", "urban_dist"
  return d, "urban", "ok"


def _hud_for(state: NavLcState, direction: str, *, solid: bool = False) -> NavLcHud:
  d = _dir(direction)
  if state == NavLcState.ARMED:
    head = LC_LEFT if d == "left" else LC_RIGHT if d == "right" else ""
    detail = SOLID_LINE_BLOCK if solid else LC_CONFIRM
    return NavLcHud(True, head, detail, state.value, d)
  if state == NavLcState.HOLD:
    return NavLcHud(True, LC_WAIT, LC_SIDE, state.value, d)
  if state == NavLcState.EXECUTING:
    if solid:
      head = LC_LEFT if d == "left" else LC_RIGHT if d == "right" else LC_EXEC
      return NavLcHud(True, head, SOLID_LINE_BLOCK, state.value, d)
    return NavLcHud(True, LC_EXEC, "", state.value, d)
  if state == NavLcState.EDGE_BLOCK:
    head = LC_EDGE_LEFT if d == "left" else LC_EDGE_RIGHT if d == "right" else ""
    return NavLcHud(bool(head), head, "", state.value, d)
  return NavLcHud(False, "", "", state.value, "none")


class NavLcConfirmAssist:
  def __init__(self):
    self._params = Params()
    self._state = NavLcState.IDLE
    self._direction = "none"
    self._mode = ""
    self._reason = ""
    self._in_window = False
    self._armed_since = 0.0
    self._cooldown_until = 0.0
    self._edge_block_until = 0.0
    self._exec_since = 0.0
    self._confirms_used = 0
    self._maneuver_key = ""
    self._budget_exhausted_key = ""
    self._sat_edge_since = 0.0
    self._sat_rec_since = 0.0
    self._prev_lc_state = 0  # cereal LaneChangeState as int-ish
    self._enable_bsm: bool | None = None
    self._bsm_checked_at = 0.0
    self._last = NavLcDecision()

  def reset(self) -> None:
    self._state = NavLcState.IDLE
    self._direction = "none"
    self._mode = ""
    self._reason = ""
    self._in_window = False
    self._armed_since = 0.0
    self._cooldown_until = 0.0
    self._edge_block_until = 0.0
    self._exec_since = 0.0
    self._confirms_used = 0
    self._maneuver_key = ""
    self._budget_exhausted_key = ""
    self._sat_edge_since = 0.0
    self._sat_rec_since = 0.0
    self._prev_lc_state = 0
    self._last = NavLcDecision()
    write_nav_lc_snapshot(NavLcSnapshot(ts=time.monotonic()))

  def update(
    self,
    *,
    snap: NavSnapshot | None,
    v_ego: float,
    lateral_active: bool,
    gear=None,
    left_blinker: bool = False,
    right_blinker: bool = False,
    left_blindspot: bool = False,
    right_blindspot: bool = False,
    left_edge: bool = False,
    right_edge: bool = False,
    brake_pressed: bool = False,
    button_events=None,
    turn_active: bool = False,
    light_bar_busy: bool = False,
    lane_change_state: int = 0,
    solid_line_blocked: bool = False,
    now: float | None = None,
  ) -> NavLcDecision:
    clock = time.monotonic() if now is None else float(now)
    start_pulse = False
    solid = bool(solid_line_blocked)

    if clock - self._bsm_checked_at > 5.0 or self._enable_bsm is None:
      self._enable_bsm = _read_enable_bsm(self._params)
      self._bsm_checked_at = clock

    if not nav_lc_enabled(self._params):
      return self._publish(NavLcState.IDLE, "none", reason="off", now=clock)
    if self._enable_bsm is not True:
      return self._publish(NavLcState.IDLE, "none", reason="no_bsm", now=clock)
    if not lateral_active:
      return self._publish(NavLcState.IDLE, "none", reason="lat", now=clock)
    if not _gear_drive(gear):
      return self._publish(NavLcState.IDLE, "none", reason="gear", now=clock)
    if left_blinker and right_blinker:
      return self._publish(NavLcState.IDLE, "none", reason="hazard", now=clock)

    urban_ok = nav_lc_urban_enabled(self._params)
    key = _maneuver_key(snap)
    if key and key != self._maneuver_key:
      self._maneuver_key = key
      self._confirms_used = 0
      if key != self._budget_exhausted_key:
        self._budget_exhausted_key = ""

    # EXECUTING: drive / abort / evaluate
    # DesireHelper: start_pulse → preLaneChange (1); promotes to Starting (2)
    # when BSM/edge/solid clear. Stay execute=True while waiting on solid.
    if self._state == NavLcState.EXECUTING:
      d = _dir(self._direction)
      opposite = (d == "left" and right_blinker and not left_blinker) or (
        d == "right" and left_blinker and not right_blinker
      )
      # Urban fork often has send_turn → nav turn desire; that must not abort LC.
      if brake_pressed or opposite or (turn_active and self._mode != "urban"):
        self._prev_lc_state = int(lane_change_state)
        return self._enter_idle(reason="abort", now=clock)
      if (clock - self._exec_since) > EXEC_TIMEOUT_S:
        self._confirms_used = min(MAX_CONFIRMS_PER_MANEUVER, self._confirms_used + 1)
        self._prev_lc_state = int(lane_change_state)
        return self._after_exec(snap, left_edge, right_edge, now=clock, timed_out=True)

      # LaneChangeState: off=0, preLaneChange=1, laneChangeStarting=2, laneChangeFinishing=3
      lc = int(lane_change_state)
      prev = self._prev_lc_state
      self._prev_lc_state = lc
      # Finished: left starting/finishing back to off, or finishing→off.
      finished = prev in (2, 3) and lc == 0
      # Inject failed only if we never reached pre/starting (solid wait uses lc=1).
      never_started = (
        prev == 0 and lc == 0 and (clock - self._exec_since) > 1.0 and not solid
      )
      if finished or never_started:
        self._confirms_used = min(MAX_CONFIRMS_PER_MANEUVER, self._confirms_used + 1)
        return self._after_exec(snap, left_edge, right_edge, now=clock, timed_out=False)

      # Waiting in preLaneChange on solid — keep execute, show solid HUD.
      waiting_solid = solid and lc in (0, 1)
      hud = _hud_for(NavLcState.EXECUTING, d, solid=waiting_solid)
      reason = "solid_wait" if waiting_solid else "exec"
      return self._publish(
        NavLcState.EXECUTING, d, execute=True, reason=reason, now=clock, hud=hud,
      )

    # EDGE_BLOCK toast window
    if self._state == NavLcState.EDGE_BLOCK:
      if clock < self._edge_block_until:
        return self._publish(
          NavLcState.EDGE_BLOCK, self._direction, reason="edge_block", now=clock,
          hud=_hud_for(NavLcState.EDGE_BLOCK, self._direction),
        )
      return self._enter_idle(reason="edge_done", now=clock)

    # COOLDOWN
    if self._state == NavLcState.COOLDOWN:
      if clock < self._cooldown_until:
        return self._publish(NavLcState.COOLDOWN, "none", reason="cooldown", now=clock)
      self._state = NavLcState.IDLE

    # Urban lc_* is promoted to send_turn for toast/turn_prep; DesireHelper then
    # sets turn_active. That must not clear Nav LC — only real turn maneuvers.
    if turn_active:
      man = str(getattr(snap, "maneuver", "") or "").strip().lower() if snap else ""
      if not (urban_ok and man == "fork"):
        return self._enter_idle(reason="turn", now=clock)

    cut = read_cut_snapshot(now=clock)
    if cut.hold:
      return self._enter_idle(reason="cut", now=clock)

    if light_bar_busy or _light_bar_busy(snap, now=clock):
      if self._state in (NavLcState.ARMED, NavLcState.HOLD):
        return self._enter_idle(reason="light_bar", now=clock)
      return self._publish(NavLcState.IDLE, "none", reason="light_bar", now=clock)

    if snap is not None and str(getattr(snap, "maneuver", "") or "").lower() in ("arrive", "arrived"):
      return self._enter_idle(reason="arrive", now=clock)

    if not snapshot_executable(snap or NavSnapshot(), now=clock):
      return self._enter_idle(reason="link", now=clock)

    if self._budget_exhausted_key and self._budget_exhausted_key == self._maneuver_key:
      return self._publish(NavLcState.IDLE, "none", reason="budget", now=clock)

    direction, mode, why = classify_nav_lc_intent(
      snap, v_ego=v_ego, urban_ok=urban_ok, in_window=self._in_window, now=clock,
    )
    if direction == "none":
      self._in_window = False
      if self._state in (NavLcState.ARMED, NavLcState.HOLD):
        return self._enter_idle(reason=why or "gate", now=clock)
      return self._publish(NavLcState.IDLE, "none", reason=why or "gate", now=clock)

    self._in_window = True
    self._mode = mode
    self._direction = direction

    # Already on outer edge toward need → EDGE_BLOCK or satisfied silence
    target_edge = left_edge if direction == "left" else right_edge
    if target_edge and self._stable_edge(direction, left_edge, right_edge, clock):
      if self._state in (NavLcState.ARMED, NavLcState.HOLD, NavLcState.IDLE):
        self._state = NavLcState.EDGE_BLOCK
        self._edge_block_until = clock + EDGE_BLOCK_SHOW_S
        return self._publish(
          NavLcState.EDGE_BLOCK, direction, reason="edge", now=clock,
          hud=_hud_for(NavLcState.EDGE_BLOCK, direction),
        )

    if self._satisfied(snap, direction, left_edge, right_edge, clock):
      self._in_window = False
      return self._enter_idle(reason="satisfied", now=clock)

    same_bsm = (direction == "left" and left_blindspot) or (
      direction == "right" and right_blindspot
    )

    # HOLD — BSM wins over solid (side traffic is harder than line paint).
    if same_bsm:
      if self._state not in (NavLcState.ARMED, NavLcState.HOLD):
        self._armed_since = clock
      self._state = NavLcState.HOLD
      # Confirm ignored while HOLD
      return self._publish(
        NavLcState.HOLD, direction, reason="bsm", now=clock,
        hud=_hud_for(NavLcState.HOLD, direction),
      )

    # ARMED
    if self._state != NavLcState.ARMED:
      self._armed_since = clock
    self._state = NavLcState.ARMED
    timeout = HWY_ARMED_TIMEOUT_S if mode == "highway" else URBAN_ARMED_TIMEOUT_S
    if (clock - self._armed_since) > timeout:
      return self._enter_idle(reason="armed_timeout", now=clock)

    # Solid (LaneTypeOnnx): ignore SET; DesireHelper also holds preLaneChange.
    # Unknown/dashed/feature-off → solid=False (fail-open, same as blinker ALC).
    if solid:
      return self._publish(
        NavLcState.ARMED, direction, reason="solid", now=clock,
        hud=_hud_for(NavLcState.ARMED, direction, solid=True),
      )

    if confirm_rising_edge(button_events):
      start_pulse = True
      self._state = NavLcState.EXECUTING
      self._exec_since = clock
      self._prev_lc_state = int(lane_change_state)
      return self._publish(
        NavLcState.EXECUTING, direction, execute=True, start_pulse=start_pulse,
        reason="confirm", now=clock, hud=_hud_for(NavLcState.EXECUTING, direction),
      )

    return self._publish(
      NavLcState.ARMED, direction, reason="armed", now=clock,
      hud=_hud_for(NavLcState.ARMED, direction),
    )

  def _stable_edge(self, direction: str, left_edge: bool, right_edge: bool, clock: float) -> bool:
    on = left_edge if direction == "left" else right_edge
    if not on:
      self._sat_edge_since = 0.0
      return False
    if self._sat_edge_since <= 0.0:
      self._sat_edge_since = clock
    return (clock - self._sat_edge_since) >= SATISFIED_HOLD_S

  def _stable_recommend(self, snap: NavSnapshot | None, clock: float) -> bool:
    rec = str(getattr(snap, "lane_recommend", "") or "").strip().lower() if snap else ""
    if rec != "straight":
      self._sat_rec_since = 0.0
      return False
    if self._sat_rec_since <= 0.0:
      self._sat_rec_since = clock
    return (clock - self._sat_rec_since) >= SATISFIED_HOLD_S

  def _satisfied(
    self,
    snap: NavSnapshot | None,
    direction: str,
    left_edge: bool,
    right_edge: bool,
    clock: float,
  ) -> bool:
    if self._stable_edge(direction, left_edge, right_edge, clock):
      return True
    if self._stable_recommend(snap, clock):
      return True
    # TBT / intent gone — classify already failed, but double-check dist floor
    if snap is not None:
      dist = float(getattr(snap, "tbt_dist", 0.0) or 0.0)
      man = str(getattr(snap, "maneuver", "") or "").lower()
      d = _dir(getattr(snap, "maneuver_dir", None))
      if man in ("none", "arrive", "arrived") or d not in ("left", "right"):
        return True
      if dist <= 0.0:
        return True
    return False

  def _after_exec(
    self,
    snap: NavSnapshot | None,
    left_edge: bool,
    right_edge: bool,
    *,
    now: float,
    timed_out: bool,
  ) -> NavLcDecision:
    d = _dir(self._direction)
    if self._confirms_used >= MAX_CONFIRMS_PER_MANEUVER:
      self._budget_exhausted_key = self._maneuver_key
      return self._enter_idle(reason="budget", now=now)

    if self._satisfied(snap, d, left_edge, right_edge, now):
      cd = HWY_COOLDOWN_S if self._mode == "highway" else URBAN_COOLDOWN_S
      self._state = NavLcState.COOLDOWN
      self._cooldown_until = now + cd
      self._in_window = False
      return self._publish(NavLcState.COOLDOWN, "none", reason="satisfied", now=now)

    # Still NEED — multi-lane shorter cooldown then may re-arm
    self._state = NavLcState.COOLDOWN
    self._cooldown_until = now + (MULTI_COOLDOWN_S if not timed_out else MULTI_COOLDOWN_S)
    return self._publish(NavLcState.COOLDOWN, "none", reason="need_more", now=now)

  def _enter_idle(self, *, reason: str, now: float) -> NavLcDecision:
    self._state = NavLcState.IDLE
    self._direction = "none"
    self._in_window = False
    self._sat_edge_since = 0.0
    self._sat_rec_since = 0.0
    return self._publish(NavLcState.IDLE, "none", reason=reason, now=now)

  def _publish(
    self,
    state: NavLcState,
    direction: str,
    *,
    reason: str,
    now: float,
    execute: bool = False,
    start_pulse: bool = False,
    hud: NavLcHud | None = None,
  ) -> NavLcDecision:
    self._state = state
    self._direction = _dir(direction)
    view = hud if hud is not None else _hud_for(state, direction)
    dec = NavLcDecision(
      state=state.value,
      direction=_dir(direction),
      execute=bool(execute),
      start_pulse=bool(start_pulse),
      confirms_used=int(self._confirms_used),
      reason=str(reason or ""),
      hud=view,
      mode=str(self._mode or ""),
    )
    self._last = dec
    st = str(view.state or state.value)
    write_nav_lc_snapshot(NavLcSnapshot(
      ts=now,
      show=bool(view.show),
      head=str(view.head or ""),
      detail=str(view.detail or ""),
      state=st,
      direction=str(view.direction or "none"),
      reason=str(reason or ""),
      swallow_set=bool(st in _SWALLOW_SET_STATES),
    ))
    return dec


def _light_bar_busy(snap: NavSnapshot | None, *, now: float | None = None) -> bool:
  if snap is None:
    return False
  if bool(getattr(snap, "stop_for_light", False)):
    return True
  token = str(getattr(snap, "traffic_light", "") or "").strip().lower()
  if token in ("red", "yellow") and light_executable(snap, now=now):
    return True
  return False
