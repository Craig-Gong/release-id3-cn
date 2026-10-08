"""Hold after a vision stop, or IQ-link red, until a stable go.

Vision-only: ~1 s dwell after the model stops asking to stop.
IQ-link head car: require confirmed traffic_light=green (APK green) plus
mmWave nose-clear — remainS==1 while still red does not launch.
Follow car: remainS==1 / green after flicker filter if GreenFollowLeadGate
agrees (radar lead moving / gap). APK green dwells ~1 s then the same gate.
Every nav go also needs the camera to have stopped asking to stop: Gaode
can report the wrong direction's light or a stale green.
Sticky red keeps pinning briefly when BLE drops executable.
While rolling, a nav red never brakes on its own — the camera / distance
helpers own the approach. At rest it pins until a confirmed go.
After a nav go, keep `_nav_go_latched` until vEgo > ~2 m/s so a one-frame
vision hitch cannot tap the brake; a sustained model stop still cancels it.
"""
from __future__ import annotations

from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_stop_safety import radar_lead_departed
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.green_follow_lead import (
  LEAD_GO_SPEED_MPS,
  STOPPED_LEAD_CREEP_M,
  STOPPED_LEAD_GAP_M,
  GreenFollowLeadGate,
  is_nav_head_car,
  lead_owns_nav_stop,
  radar_nose_clear,
  read_follow_lead,
  read_nav_queue_lead,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_follow_comfort import (
  FollowLaunchController,
)
from openpilot.sunnypilot.nav.snapshot import NavSnapshot, light_executable, read_snapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import nav_long_blocked

_STANDSTILL_HOLD_RELEASE_S = 1.0
_STANDSTILL_HOLD_LEAD_RELEASE_S = 0.15
_REMAIN_GO_CONFIRM_S = 0.15  # 3 frames @ 50 ms — filter single-packet false go
_STICKY_RED_TTL_S = 2.0
# Red that goes dark with ≤ this many seconds left on the countdown = turned
# green; Gaode often sends status 0 / stops broadcasting at the change.
_RED_END_COUNTDOWN_S = 3.0
# Camera must stop asking to stop this long before a nav green may launch.
_CAMERA_GO_S = 0.5
# Sustained model stop cancels a nav launch latch (one-frame hitches do not).
_LATCH_CANCEL_MODEL_STOP_S = 0.3
_RTOR_TURN_WINDOW_M = 50.0
# IQ-link off: model shouldStop rising edge pins standstill briefly (no creep).
_VISION_PIN_TTL_S = 5.0
# MEB needs clearly positive accel for ANFAHREN; 0.4 felt like release-without-go.
_GO_LAUNCH_FLOOR_A = 0.9
_DT_MDL = 0.05
_RELEASE_V_EGO = 2.0
_STANDSTILL_V = 0.3
# Last few km/h: a one-frame model/nav dropout used to hand back +e2e and
# roll through the line. Hold a clearly negative accel until a real go.
_APPROACH_CREEP_V = 4.0
_APPROACH_HOLD_A = -0.45


def hold_approach_accel(a_target: float, v_ego: float, *, stop_intent: bool,
                        go_latched: bool, gas: bool, right_blinker: bool) -> float:
  """No forward accel while committed to a line and still rolling slowly.

  Green latch, driver gas, and right-blinker (red-turn) stay free. Faster
  than ~14 km/h the nav/vision caps still own the approach.
  """
  if gas or go_latched or right_blinker or not stop_intent:
    return float(a_target)
  if float(v_ego) > _APPROACH_CREEP_V:
    return float(a_target)
  return min(float(a_target), _APPROACH_HOLD_A)


def left_arrow_red(snap: NavSnapshot) -> bool:
  """Left-turn TBT + red: keep pin even if approach curve still has speed."""
  return (
    str(snap.traffic_light or "").strip().lower() == "red"
    and bool(snap.send_turn)
    and str(snap.maneuver_dir or "") == "left"
    and not bool(snap.remain_go)
  )


def _right_turn_pending(snap: NavSnapshot) -> bool:
  # Match protocol RTOR: near right turn must not sticky-arm a red pin.
  return (
    str(snap.maneuver or "") == "turn"
    and str(snap.maneuver_dir or "") == "right"
    and 0.0 < float(snap.tbt_dist or 0.0) <= _RTOR_TURN_WINDOW_M
    and str(snap.light_dir or "") != "right"
  )


def nav_light_live(snap: NavSnapshot, gear=None, *, now: float | None = None) -> bool:
  """Light colour is fresh from Gaode and nav longitudinal is allowed."""
  return bool(light_executable(snap, now=now) and not nav_long_blocked(gear))


class StandstillHold:
  def __init__(self):
    self.hold = False
    self.hold_s = 0.0
    self.hold_released = False
    self._follow = GreenFollowLeadGate()
    self._nav_go_latched = False
    self.sticky_red = False
    self._sticky_until = 0.0
    self._remain_go_s = 0.0
    self.red_pin = False
    self.vision_pin = False
    self._vision_pin_until = 0.0
    self._model_stop_prev = False
    self._model_clear_s = 0.0
    self._model_stop_s = 0.0
    self._red_live_prev = False
    self._red_remain_last = 0.0

  def reset(self) -> None:
    self.hold = False
    self.hold_s = 0.0
    self.hold_released = False
    self._follow.reset()
    self._nav_go_latched = False
    self.sticky_red = False
    self._sticky_until = 0.0
    self._remain_go_s = 0.0
    self.red_pin = False
    self.vision_pin = False
    self._vision_pin_until = 0.0
    self._model_stop_prev = False
    self._model_clear_s = 0.0
    self._model_stop_s = 0.0
    self._red_live_prev = False
    self._red_remain_last = 0.0

  def _release(self, reason: str, snap: NavSnapshot) -> None:
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.event("standstill_hold_release", reason=reason, light=snap.traffic_light,
                     light_raw=int(snap.light_raw), light_dir=snap.light_dir,
                     remain_s=float(snap.remain_s), model_clear_s=round(self._model_clear_s, 2))
    except Exception:
      pass

  def _clear_sticky(self) -> None:
    self.sticky_red = False
    self._sticky_until = 0.0

  def _clear_vision_pin(self) -> None:
    self.vision_pin = False
    self._vision_pin_until = 0.0

  def _launch_a(self, a_target: float, sm, v_ego: float) -> float:
    """Positive takeoff after nav green; do not pass through a vision hitch.

    Radar range floor still runs after this and can brake a close bumper.
    """
    a_out = float(a_target)
    if v_ego <= _STANDSTILL_V + 0.5:
      return max(a_out, _GO_LAUNCH_FLOOR_A)
    lead = read_follow_lead(sm)
    if lead.present and lead.d_rel < STOPPED_LEAD_CREEP_M:
      return a_out
    return max(a_out, 0.0)

  def observe_nav(self, snap: NavSnapshot, now: float, *, gas: bool, v_ego: float,
                  gear=None, sm=None) -> None:
    """Arm / expire sticky red. Speed-limit stale rules stay separate."""
    if gas or v_ego > _RELEASE_V_EGO or not snap.iqlink_enabled or nav_long_blocked(gear):
      self._clear_sticky()
      self.red_pin = False
      self._remain_go_s = 0.0
      self._red_live_prev = False
      # Keep vision_pin when IQ-link is merely off — only gas / speed / gear clear it via reset.
      if gas or v_ego > _RELEASE_V_EGO or nav_long_blocked(gear):
        self._clear_vision_pin()
      return

    # Queue behind a stopped lead short of the light: follow lead, not nav pin.
    if sm is not None and lead_owns_nav_stop(sm, snap):
      self._clear_sticky()
      self.red_pin = False
      self._remain_go_s = 0.0
      self._red_live_prev = False
      return

    live = light_executable(snap, now=now)
    live_red = bool(live and snap.stop_for_light)
    live_left = bool(live and left_arrow_red(snap))
    red_now = live_red or live_left
    if red_now:
      self._red_remain_last = float(snap.remain_s or 0.0)
    elif self._red_live_prev and self.sticky_red:
      # Red ended. Explicit green, or dark right at the end of the countdown,
      # means the light changed — do not keep pinning on the old red.
      ended_on_countdown = 0.0 < self._red_remain_last <= _RED_END_COUNTDOWN_S
      if snap.light_token == "green" or (snap.light_token == "none" and ended_on_countdown):
        self._clear_sticky()
    self._red_live_prev = red_now
    # Never sticky-arm across an active RTOR exemption.
    if red_now and not _right_turn_pending(snap):
      self.sticky_red = True
      self._sticky_until = float(now) + _STICKY_RED_TTL_S
    elif self.sticky_red and float(now) > self._sticky_until:
      self._clear_sticky()

    self.red_pin = bool(
      (live_red or live_left or self.sticky_red) and not _right_turn_pending(snap)
    )

  def apply(self, should_stop: bool, a_target: float, v_ego: float, *,
            standstill: bool, gas: bool, model_stop: bool,
            sm=None, now: float | None = None) -> tuple[bool, float]:
    if gas or v_ego > _RELEASE_V_EGO:
      self.reset()
      return should_stop, a_target

    clock = float(now) if now is not None else 0.0
    snap = read_snapshot() if sm is not None else NavSnapshot()
    try:
      gear = sm['carState'].gearShifter if sm is not None else None
    except Exception:
      gear = None
    self.observe_nav(snap, clock, gas=False, v_ego=v_ego, gear=gear, sm=sm)

    if model_stop:
      self._model_clear_s = 0.0
      self._model_stop_s += _DT_MDL
    else:
      self._model_clear_s += _DT_MDL
      self._model_stop_s = 0.0
    camera_go = self._model_clear_s >= _CAMERA_GO_S - 1e-6
    if self._nav_go_latched and self._model_stop_s >= _LATCH_CANCEL_MODEL_STOP_S - 1e-6:
      self._nav_go_latched = False
      self.hold_released = False

    nav_live = nav_light_live(snap, gear, now=clock)
    follow_sm = sm if sm is not None else {}
    # Radar-priority queue lead: vision phantoms must not block head-car green.
    lead = read_nav_queue_lead(follow_sm)
    lead_rolling = bool(lead.present and lead.v_lead >= LEAD_GO_SPEED_MPS)
    closing_gap = bool(
      lead.present and lead.v_lead < LEAD_GO_SPEED_MPS and lead.d_rel > STOPPED_LEAD_CREEP_M
    )
    confirmed_green = bool(nav_live and snap.apk_green)
    head_car = is_nav_head_car(follow_sm)

    # remainS==1 must be live; require ~3 frames to filter a single false packet.
    if nav_live and snap.remain_go:
      self._remain_go_s += _DT_MDL
    else:
      self._remain_go_s = 0.0
    remain_go = self._remain_go_s >= _REMAIN_GO_CONFIRM_S

    # Head car: remainS==1 on a still-red light is countdown, not go.
    # Follow car: remain_go can release once radar lead moves / gap opens.
    nav_go = bool(confirmed_green or (remain_go and not head_car))
    follow_ok = self._follow.may_release(
      now=clock, nav_go=nav_go, sm=follow_sm, confirmed_green=confirmed_green,
    )
    queue_go = bool(remain_go and follow_ok and not head_car and camera_go)

    # Queue: remainS==1 + lead moving / gap — do not wait for APK green token.
    if queue_go:
      self._clear_sticky()
      self.red_pin = False
      self._clear_vision_pin()
      self.hold = False
      self.hold_s = 0.0
      self.hold_released = True
      self._nav_go_latched = True
      self._release("queue_remain_go", snap)
      return False, max(float(a_target), _GO_LAUNCH_FLOOR_A)

    at_rest = standstill or v_ego <= _STANDSTILL_V

    # Nav / sticky red at rest: pin until confirmed green (head) or queue go.
    # While green is dwelling, skip this pin so the ~1 s APK green path can run.
    # Rolling: Gaode has no light distance, so a nav red alone must not brake
    # (it may be the next intersection or another direction). The camera and
    # TrafficStopOffset own the last metres, same as with IQ-link off.
    if self.red_pin and not confirmed_green:
      self._nav_go_latched = False
      self._clear_vision_pin()
      self._model_stop_prev = bool(model_stop)
      if at_rest:
        self.hold = True
        self.hold_s = 0.0
        self.hold_released = False
        return True, min(float(a_target), -1.0)
      return should_stop, a_target

    if not at_rest:
      if self.hold:
        self.hold = False
        self.hold_s = 0.0
      if self._nav_go_latched:
        # Drop residual e2e/vision hitch; radar floor still owns a close bumper.
        return False, self._launch_a(a_target, follow_sm, v_ego)
      self.hold_released = False
      return should_stop, a_target

    # Radar, not the nav bar: lead has left the settle gap and is really moving.
    # Do not wait for 8 m or a missing green packet. still honor a live red pin.
    # A lead turning away while the camera still asks to stop is not a go.
    if radar_lead_departed(follow_sm) and not self.red_pin and not model_stop:
      self.hold = False
      self.hold_s = 0.0
      self.hold_released = True
      self._nav_go_latched = True
      self._clear_vision_pin()
      self._release("lead_departed", snap)
      return False, self._launch_a(a_target, follow_sm, v_ego)

    # Congestion: lead already rolling, or closing a too-large gap.
    if lead_rolling or closing_gap:
      # Nose-to-bumper only. 8 m was "a short lead start" and left ego sitting.
      if lead.present and lead.d_rel < STOPPED_LEAD_GAP_M:
        self.hold = True
        self.hold_released = False
        return True, min(float(a_target), 0.0)
      release_s = _STANDSTILL_HOLD_LEAD_RELEASE_S if lead_rolling else 0.0
      if self.hold:
        self.hold_s += _DT_MDL
        if self.hold_s < release_s:
          return True, min(float(a_target), 0.0)
        self.hold = False
        self.hold_s = 0.0
        self.hold_released = True
      a_out = float(a_target)
      # Lead rolling with a usable gap. Under CREEP: radar noise is not a go —
      # leave accel to gap brake, do not add a positive floor.
      if lead_rolling and lead.d_rel >= STOPPED_LEAD_CREEP_M and not model_stop:
        a_out = max(a_out, _GO_LAUNCH_FLOOR_A)
      return should_stop, a_out

    if confirmed_green:
      # Camera still sees a stop (wrong-direction / stale green): keep waiting.
      # Once latched, brief hitches are the latch's job (sustained stop cancels it).
      if not camera_go and not self._nav_go_latched:
        self.hold = True
        self.hold_s = 0.0
        self._model_stop_prev = bool(model_stop)
        return True, min(float(a_target), -1.0)
      # Head: mmWave must stay clear. Follow: gate owns lead motion / timeout.
      if head_car and (not follow_ok or not radar_nose_clear(follow_sm)):
        self.hold = True
        return True, min(float(a_target), 0.0)
      if (not head_car) and (not follow_ok):
        self.hold = True
        return True, min(float(a_target), 0.0)
      if not self.hold_released:
        if not self.hold:
          self.hold = True
          self.hold_s = 0.0
        self.hold_s += _DT_MDL
        if self.hold_s < _STANDSTILL_HOLD_RELEASE_S:
          return True, min(float(a_target), 0.0)
        self.hold = False
        self.hold_released = True
        self._nav_go_latched = True
        self._clear_sticky()
        self.red_pin = False
        self._clear_vision_pin()
        self._release("green_head" if head_car else "green_follow", snap)
        return False, max(float(a_target), _GO_LAUNCH_FLOOR_A)

    # Latched after nav/APK green: ignore vision shouldStop until rolling.
    if self._nav_go_latched:
      return False, self._launch_a(a_target, follow_sm, v_ego)

    # IQ-link off (or no live red): model shouldStop rising edge → short vision pin.
    if model_stop and not self._model_stop_prev and not self.red_pin:
      self.vision_pin = True
      self._vision_pin_until = clock + _VISION_PIN_TTL_S
    self._model_stop_prev = bool(model_stop)
    if self.vision_pin and clock > self._vision_pin_until:
      self._clear_vision_pin()
    if self.red_pin:
      self._clear_vision_pin()

    arm = should_stop if self.hold_released else (should_stop or model_stop or self.vision_pin)
    if arm:
      self.hold = True
      self.hold_s = 0.0
    elif self.hold:
      # Vision pin: do not time-release while pin is live (no creep at red/stop).
      if self.vision_pin:
        self.hold_s = 0.0
      else:
        self.hold_s += _DT_MDL
        if self.hold_s >= _STANDSTILL_HOLD_RELEASE_S:
          self.hold = False
          self.hold_released = True
    if self.hold:
      # Match nav red pin: a≤−1 so MEB never ANFAHREN on a≈0 / tiny +e2e.
      return True, min(float(a_target), -1.0)
    return should_stop, a_target


# Module-level controller for the pure-function call site in apply_stop_helpers.
# LongitudinalPlannerSP also owns one; tests may reset via FollowLaunchController.reset.
_follow_launch = FollowLaunchController()


def apply_follow_launch(sm, v_ego: float, a_target: float, dt: float = 0.05,
                        controller: FollowLaunchController | None = None) -> float:
  """Soft congestion takeoff floor + lead-decel anticipate (see lead_follow_comfort)."""
  ctrl = controller if controller is not None else _follow_launch
  return ctrl.apply(sm, v_ego, a_target, dt=dt)
