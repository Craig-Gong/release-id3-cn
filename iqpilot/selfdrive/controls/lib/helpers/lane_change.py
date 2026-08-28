"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
from cereal import custom, log

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

NAV_EXIT_COMMIT_DISTANCE = 500.0  # m before a route exit to begin moving into the exit lane
_ManeuverType = custom.IQNavState.ManeuverType
_ManeuverPhase = custom.IQNavState.ManeuverPhase
_NavDirection = custom.NavDirection


class LaneSwapPreset:
  DISABLED = -1
  STEERING_NUDGE = 0
  DIRECT = 1
  DELAY_HALF = 2
  DELAY_ONE = 3
  DELAY_TWO = 4
  DELAY_THREE = 5
  OFF = DISABLED
  NUDGE = STEERING_NUDGE
  NUDGELESS = DIRECT
  HALF_SECOND = DELAY_HALF
  ONE_SECOND = DELAY_ONE
  TWO_SECONDS = DELAY_TWO
  THREE_SECONDS = DELAY_THREE


PRESET_SECONDS = {
  LaneSwapPreset.DISABLED: 0.0,
  LaneSwapPreset.STEERING_NUDGE: 0.0,
  LaneSwapPreset.DIRECT: 0.05,
  LaneSwapPreset.DELAY_HALF: 0.5,
  LaneSwapPreset.DELAY_ONE: 1.0,
  LaneSwapPreset.DELAY_TWO: 2.0,
  LaneSwapPreset.DELAY_THREE: 3.0,
}

LANE_SWAP_SECONDS = dict(PRESET_SECONDS)
BLINDSPOT_WAIT_OFFSET = -1


class LaneSwapEngine:
  def __init__(self, desire_hub):
    self._hub = desire_hub
    self._kv = Params()
    self._mem = {
      "sec": 0.0,
      "tick": 0,
      "gate": 0.0,
      "preset": self._kv.get("IQLaneChangeTimer", return_default=True),
      "bsm_hold": False,
      "braked": False,
      "ready": False,
      "used": False,
    }
    self.reload_setup()

  def _pull_setup(self) -> None:
    self._mem["bsm_hold"] = self._kv.get_bool("IQLaneChangeBsmDelay")
    self._mem["preset"] = self._kv.get("IQLaneChangeTimer", return_default=True)

  def _idle_phase(self) -> bool:
    return (
      self._hub.lane_change_state == log.LaneChangeState.off and
      self._hub.lane_change_direction == log.LaneChangeDirection.none
    )

  def _seconds_for_preset(self) -> float:
    picked = self._mem["preset"]
    return PRESET_SECONDS.get(picked, PRESET_SECONDS[LaneSwapPreset.STEERING_NUDGE])

  def _auto_preset_active(self) -> bool:
    picked = self._mem["preset"]
    return picked not in (LaneSwapPreset.DISABLED, LaneSwapPreset.STEERING_NUDGE)

  def _advance_clock(self, blindspot_now: bool) -> None:
    wait_s = self._seconds_for_preset()
    self._mem["gate"] = wait_s
    self._mem["sec"] += DT_MDL
    if self._mem["bsm_hold"] and blindspot_now and wait_s > 0.0:
      if wait_s == PRESET_SECONDS[LaneSwapPreset.DIRECT]:
        self._mem["sec"] = BLINDSPOT_WAIT_OFFSET
      else:
        self._mem["sec"] = wait_s + BLINDSPOT_WAIT_OFFSET

  def _ready_to_fire(self) -> bool:
    return (
      self._auto_preset_active() and
      (not self._mem["braked"]) and
      (not self._mem["used"]) and
      (self._mem["sec"] > self._mem["gate"])
    )

  def reload_setup(self) -> None:
    self._pull_setup()

  def heartbeat(self) -> None:
    if (self._mem["tick"] % 50) == 0:
      self._pull_setup()
    self._mem["tick"] += 1

  def sample(self, blindspot_now: bool = False, brake_now: bool = False, **legacy) -> None:
    blindspot_now = bool(legacy.get("blindspot_detected", blindspot_now))
    brake_now = bool(legacy.get("brake_pressed", brake_now))
    self._mem["braked"] = self._mem["braked"] or brake_now
    self._advance_clock(blindspot_now)
    self._mem["ready"] = self._ready_to_fire()

  def finalize(self) -> None:
    started = self._hub.lane_change_state == log.LaneChangeState.laneChangeStarting
    self._mem["used"] = self._mem["used"] or started
    if self._idle_phase():
      self._mem["sec"] = 0.0
      self._mem["braked"] = False
      self._mem["used"] = False

  @property
  def ready(self):
    return self._mem["ready"]

  @property
  def delay(self):
    return self._mem["gate"]

  @property
  def elapsed(self):
    return self._mem["sec"]

  @property
  def preset(self):
    return self._mem["preset"]

  @preset.setter
  def preset(self, value):
    self._mem["preset"] = value

  @property
  def bsm_hold(self):
    return self._mem["bsm_hold"]

  @bsm_hold.setter
  def bsm_hold(self, value):
    self._mem["bsm_hold"] = bool(value)

  @property
  def braked(self):
    return self._mem["braked"]

  @braked.setter
  def braked(self, value):
    self._mem["braked"] = bool(value)

  @property
  def used(self):
    return self._mem["used"]

  @used.setter
  def used(self, value):
    self._mem["used"] = bool(value)


class NavExitLaneChangeController:
  def __init__(self, enable_bsm: bool):
    self._params = Params()
    self._enable_bsm = bool(enable_bsm)
    self.enabled = self._read_enabled()
    self._tick = 0
    self.active = False
    self.direction = log.LaneChangeDirection.none
    self.auto_allowed = False

  def _read_enabled(self) -> bool:
    try:
      return self._params.get_bool("NavExitLaneChange")
    except Exception:
      return False

  def update_params(self) -> None:
    if self._tick % 50 == 0:
      self.enabled = self._read_enabled()
    self._tick += 1

  @staticmethod
  def _raw(value):
    return getattr(value, "raw", value)

  def update(self, nav_state, carstate) -> None:
    self.active = False
    self.direction = log.LaneChangeDirection.none
    self.auto_allowed = False

    if not self.enabled or nav_state is None or not getattr(nav_state, "active", False):
      return
    if not getattr(nav_state, "nextManeuverValid", False):
      return
    if self._raw(getattr(nav_state, "nextManeuverType", _ManeuverType.none)) != int(_ManeuverType.exit):
      return
    distance = float(getattr(nav_state, "nextManeuverDistance", 0.0))
    if not 0.0 < distance <= NAV_EXIT_COMMIT_DISTANCE:
      return

    direction = self._raw(getattr(nav_state, "nextManeuverDirection", _NavDirection.none))
    if direction == int(_NavDirection.left):
      self.direction = log.LaneChangeDirection.left
    elif direction == int(_NavDirection.right):
      self.direction = log.LaneChangeDirection.right
    else:
      return

    self.active = True
    blindspot = carstate.leftBlindspot if self.direction == log.LaneChangeDirection.left else carstate.rightBlindspot
    self.auto_allowed = (not blindspot) if self._enable_bsm else False


PARAM_HIGHWAY_ALC = "IQNavHighwayAlc"
PARAM_HIGHWAY_ALC_DIRECT = "IQNavHighwayAlcDirect"

# protocol promotes lc* → send_turn inside this window (see LIGHT_TURN_WINDOW_M).
FORK_LC_PROMOTE_M = 150.0


def _nav_side_to_lc_direction(side: str) -> int:
  if side == "left":
    return log.LaneChangeDirection.left
  if side == "right":
    return log.LaneChangeDirection.right
  return log.LaneChangeDirection.none


class NavForkLaneChangeController:
  """IQ-link highway fork: shouldSendLaneChangeDesire → lane-change FSM (Plan B)."""

  def __init__(self, enable_bsm: bool):
    self._params = Params()
    self._enable_bsm = bool(enable_bsm)
    self._tick = 0
    self.enabled = False
    self.direct_mode = False
    self.active = False
    self.direction = log.LaneChangeDirection.none
    self.auto_allowed = False
    self.latched = False

  def _read_enabled(self) -> bool:
    try:
      return self._params.get_bool(PARAM_HIGHWAY_ALC)
    except Exception:
      return False

  def _read_direct(self) -> bool:
    try:
      return self._params.get_bool(PARAM_HIGHWAY_ALC_DIRECT)
    except Exception:
      return False

  def _iqlink_on(self) -> bool:
    try:
      return bool(self._params.get_bool("IqlinkEnabled")) or bool(self._params.get_bool("NavigationActive"))
    except Exception:
      return False

  def update_params(self) -> None:
    if self._tick % 50 == 0:
      self.enabled = self._read_enabled()
      self.direct_mode = self._read_direct()
    self._tick += 1

  @staticmethod
  def _raw(value):
    return getattr(value, "raw", value)

  @staticmethod
  def _side_from_nav(nav_state, attr: str) -> str:
    direction = getattr(nav_state, attr, None)
    name = str(getattr(direction, "name", None) or direction or "").lower()
    if "left" in name:
      return "left"
    if "right" in name:
      return "right"
    raw = NavForkLaneChangeController._raw(direction)
    if raw == 1:
      return "left"
    if raw == 2:
      return "right"
    return "none"

  def _signal_active(self, nav_state, carstate) -> bool:
    if not self.enabled or not self._iqlink_on():
      return False
    if nav_state is None or not getattr(nav_state, "active", False):
      return False
    if not bool(getattr(nav_state, "shouldSendLaneChangeDesire", False)):
      return False
    if bool(getattr(nav_state, "shouldSendTurnDesire", False)):
      return False
    if self._raw(getattr(nav_state, "maneuverPhase", _ManeuverPhase.none)) != int(_ManeuverPhase.highwayCommit):
      return False
    if self._raw(getattr(nav_state, "nextManeuverType", _ManeuverType.none)) != int(_ManeuverType.fork):
      return False

    from openpilot.iqpilot.selfdrive.controls.lib.helpers.nav_auto_blinker import is_highway_fast_context
    from openpilot.iqpilot.selfdrive.controls.lib.helpers.lane_turn import TURN_TRIGGER_MPS

    if float(carstate.vEgo) < TURN_TRIGGER_MPS:
      return False

    road_ms = float(getattr(nav_state, "roadSpeedLimit", 0.0) or 0.0)
    if road_ms <= 0.0:
      road_ms = float(getattr(carstate, "vEgo", 0.0) or 0.0)  # fallback only for gate
    if not is_highway_fast_context(road_ms, float(carstate.vEgo)):
      return False

    side = self._side_from_nav(nav_state, "laneChangeDesireDirection")
    if side not in ("left", "right"):
      return False

    lane_rec = str(getattr(nav_state, "laneRecommend", "none") or "none").strip().lower()
    if lane_rec == "straight":
      return False
    if lane_rec in ("left", "right") and lane_rec != side:
      return False

    dist = float(getattr(nav_state, "nextManeuverDistance", 0.0) or 0.0)
    if not dist > FORK_LC_PROMOTE_M:
      return False

    return True

  def note_lane_change_state(self, lane_change_state) -> None:
    if lane_change_state == log.LaneChangeState.off:
      self.latched = False

  def blocks_nav_turn_blinker(self) -> bool:
    return bool(self.latched or self.active)

  def update(self, nav_state, carstate) -> None:
    self.active = False
    self.direction = log.LaneChangeDirection.none
    self.auto_allowed = False

    signal = self._signal_active(nav_state, carstate)
    if signal:
      side = self._side_from_nav(nav_state, "laneChangeDesireDirection")
      self.direction = _nav_side_to_lc_direction(side)
      if self.direction != log.LaneChangeDirection.none:
        self.latched = True

    if not self.latched:
      return

    self.active = True
    if signal:
      side = self._side_from_nav(nav_state, "laneChangeDesireDirection")
      self.direction = _nav_side_to_lc_direction(side)

    blindspot = (
      carstate.leftBlindspot if self.direction == log.LaneChangeDirection.left else carstate.rightBlindspot
    )
    if not self.direct_mode:
      self.auto_allowed = False
    elif not self._enable_bsm:
      self.auto_allowed = True
    else:
      self.auto_allowed = not blindspot


AutoLaneChangeMode = LaneSwapPreset
AUTO_LANE_CHANGE_TIMER = LANE_SWAP_SECONDS
ONE_SECOND_DELAY = BLINDSPOT_WAIT_OFFSET


class IQLaneSwapController(LaneSwapEngine):
  def __init__(self, desire_helper):
    super().__init__(desire_helper)

  def reset(self) -> None:
    self.finalize()

  def update_params(self) -> None:
    self.heartbeat()

  def update_lane_change(self, blindspot_detected: bool, brake_pressed: bool) -> None:
    self.sample(blindspot_now=blindspot_detected, brake_now=brake_pressed)

  def update_state(self) -> None:
    self.finalize()

  @property
  def lane_change_wait_timer(self):
    return self.elapsed

  @lane_change_wait_timer.setter
  def lane_change_wait_timer(self, value):
    self._mem["sec"] = float(value)

  @property
  def lane_change_delay(self):
    return self.delay

  @lane_change_delay.setter
  def lane_change_delay(self, value):
    self._mem["gate"] = float(value)

  @property
  def lane_change_set_timer(self):
    return self.preset

  @lane_change_set_timer.setter
  def lane_change_set_timer(self, value):
    self.preset = value

  @property
  def lane_change_bsm_delay(self):
    return self.bsm_hold

  @lane_change_bsm_delay.setter
  def lane_change_bsm_delay(self, value):
    self.bsm_hold = value

  @property
  def prev_brake_pressed(self):
    return self.braked

  @prev_brake_pressed.setter
  def prev_brake_pressed(self, value):
    self.braked = value

  @property
  def auto_lane_change_allowed(self):
    return self.ready

  @auto_lane_change_allowed.setter
  def auto_lane_change_allowed(self, value):
    self._mem["ready"] = bool(value)

  @property
  def prev_lane_change(self):
    return self.used

  @prev_lane_change.setter
  def prev_lane_change(self, value):
    self.used = value
