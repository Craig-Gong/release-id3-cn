"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
import time
from datetime import datetime

import numpy as np

from cereal import messaging, custom
from iqdbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.iqpilot.selfdrive.controls.lib.custom_stop_distance import CustomStopDistance
from openpilot.iqpilot.selfdrive.controls.lib.traffic_stop_offset import TrafficStopOffset
from openpilot.iqpilot.selfdrive.controls.lib.iq_dynamic.engine import IQDynamicController
from openpilot.iqpilot.selfdrive.controls.lib.iq_dynamic.imahelper import IQConstants
from openpilot.iqpilot.selfdrive.controls.lib.helpers.e2e_alerts import EndToEndAlertEngine
from openpilot.iqpilot.selfdrive.controls.lib.helpers.green_follow_lead import GreenFollowLeadGate
from openpilot.iqpilot.selfdrive.controls.lib.helpers.junction_hud import junction_hud_active, light_token
from openpilot.iqpilot.selfdrive.controls.lib.helpers.turn_prep import UrbanTurnPrep
from openpilot.iqpilot.iqlink.protocol import nav_turn_pending
from openpilot.iqpilot.selfdrive.controls.lib.slc_vcruise import SLCVCruise
from openpilot.iqpilot.selfdrive.controls.lib.speed_limit_controller import LIMIT_ADAPT_ACC
from openpilot.iqpilot.selfdrive.selfdrived.events import IQEvents
from openpilot.iqpilot.selfdrive.iqmodeld.models.helpers import get_active_bundle

IQDynamicState = custom.IQPlan.IQDynamicControl.IQDynamicControlState
LongitudinalPlanSource = custom.IQPlan.LongitudinalPlanSource
SpeedLimitAssistState = custom.IQPlan.SpeedLimit.AssistState
SpeedLimitSource = custom.IQPlan.SpeedLimit.Source
NavProvider = custom.IQNavState.LongitudinalProvider
NavLongitudinalState = custom.IQNavState.LongitudinalState

# IQ-link: non-stop nav execution floor (red-light stop is exempt).
_NAV_EXEC_MIN_MS = 60.0 * CV.KPH_TO_MS
# Red-stop signature from protocol.py (_RED_LIGHT_ACCEL).
_NAV_RED_DECEL = -1.9
# Once stopped for a light / model stop, require this long of a clean "go" before launch.
_STANDSTILL_HOLD_RELEASE_S = 1.0
_STANDSTILL_HOLD_SPEED = 1.0  # m/s: below this, a red-decel approach becomes a hard hold


def nav_long_blocked_by_gear(gear) -> bool:
  """Park/reverse must not inherit leftover IQlink speedTarget."""
  token = str(getattr(gear, "name", None) or "").split(".")[-1].lower()
  if token in ("park", "reverse"):
    return True
  if token:
    return False
  try:
    return int(gear) in (1, 4)  # cereal GearShifter.park / reverse
  except (TypeError, ValueError):
    return False


def nav_long_blocked(gear, *, link_warn: bool = False) -> bool:
  """Drop nav long while BLE is stale (snapshot kept, speed no longer live) or in P/R."""
  return nav_long_blocked_by_gear(gear) or bool(link_warn)


class LongitudinalPlannerIQ:
  def __init__(self, CP: structs.CarParams, CP_IQ: structs.IQCarParams, mpc):
    self.events_iq = IQEvents()
    self.iq_dynamic = IQDynamicController(CP, mpc)
    self.custom_stop_distance = CustomStopDistance()
    self.traffic_stop_offset = TrafficStopOffset()
    self.slimit = SLCVCruise()
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts = EndToEndAlertEngine()
    self.output_v_target = 0.
    self.output_a_target = 0.
    self.speed_limit_last = 0.
    self.speed_limit_final_last = 0.
    self.speed_limit_source = SpeedLimitSource.none
    self.nav_engaged = False
    self.nav_provider = NavProvider.none
    self.nav_state = NavLongitudinalState.disabled
    self.nav_speed_target = 0.
    self.nav_accel_target = 0.
    self.nav_valid = False
    self.nav_stop_request = False
    try:
      self._params = Params()
    except Exception:
      self._params = None
    self.force_stop_timer = 0.0
    self.forcing_stop = False
    self.override_force_stop = False
    self.override_force_stop_timer = 0.0
    self.tracked_model_length = 0.0
    self._standstill_hold = False
    self._standstill_hold_s = 0.0
    self._green_launch = False
    self._hold_released = False
    self._green_follow_gate = GreenFollowLeadGate()
    self.junction_hud = False
    self.turn_prep = UrbanTurnPrep(params=self._params)

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.iq_dynamic.active():
      return experimental_mode

    return experimental_mode and self.iq_dynamic.mode() == "blended"

  def _iqlink_on(self) -> bool:
    try:
      return bool(self._params and self._params.get_bool("IqlinkEnabled"))
    except Exception:
      return False

  def _nav_device_offset_ms(self) -> float:
    """Device speed offset for nav execution (m/s). Prefer SLC band offset; fall back to value offset."""
    offset = float(getattr(self.slimit, "slc_offset", 0.0) or 0.0)
    if offset != 0.0:
      return offset
    if self._params is None:
      return 0.0
    try:
      raw = self._params.get("IQSpeedAssistValueOffset", return_default=True)
      value = float(raw) if raw is not None else 0.0
      is_metric = bool(self._params.get_bool("IsMetric"))
      return value * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)
    except Exception:
      return 0.0

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS
    # SLC should apply whenever IQ.Pilot is engaged, even on stock-longitudinal cars
    # where carControl.longActive stays false.
    slc_apply_enabled = bool(getattr(sm['selfdriveState'], "enabled", False))

    nav_state = sm['iqNavState']
    self.nav_engaged = bool(getattr(nav_state, "longitudinalEngaged", False))
    self.nav_provider = getattr(nav_state, "longitudinalProvider", NavProvider.none)
    self.nav_state = getattr(nav_state, "longitudinalState", NavLongitudinalState.disabled)
    self.nav_speed_target = float(getattr(nav_state, "speedTarget", 0.0))
    self.nav_accel_target = float(getattr(nav_state, "accelTarget", 0.0))
    # Allow speedTarget==0 (nav red-light stop) while engaged/valid.
    self.nav_valid = bool(getattr(nav_state, "valid", False) and self.nav_engaged)
    self.nav_stop_request = bool(self.nav_valid and self.nav_speed_target <= 0.0)
    # Approach curve still has a few m/s left at the line; once crawling, hold instead of creeping.
    if self.nav_valid and self.nav_accel_target <= _NAV_RED_DECEL and v_ego <= _STANDSTILL_HOLD_SPEED:
      self.nav_stop_request = True

    link_warn = False
    if self._params is not None:
      try:
        link_warn = bool(self._params.get_bool("IqlinkLinkWarn"))
      except Exception:
        link_warn = False
    block_nav = nav_long_blocked(CS.gearShifter, link_warn=link_warn)
    if block_nav:
      # Keep HUD snapshot; do not execute leftover nav speed / red-stop.
      self.nav_stop_request = False

    has_follow_lead = False
    try:
      lead = sm['radarState'].leadOne
      has_follow_lead = bool(getattr(lead, "status", False))
    except Exception:
      has_follow_lead = False
    # A flickering lead must not drop a red-light hold.

    iqlink_on = self._iqlink_on()

    # IQ.Pilot custom Speed Limit Controller
    now = datetime.now()
    if hasattr(sm, "alive"):
      time_validated = sm.alive.get('clocks', False) and getattr(sm['clocks'], 'timeValid', False)
    else:
      clocks = sm.get('clocks', None) if isinstance(sm, dict) else None
      time_validated = bool(getattr(clocks, 'timeValid', False))
    slc_v_cruise = self.slimit.update(slc_apply_enabled, now, time_validated, v_cruise, v_ego, sm)
    self.iq_dynamic.set_slc_experimental_mode(self.slimit.slc_experimental_mode)
    self.iq_dynamic.update(sm)
    # Prefer confirmed controller output for UI/planner rendering.
    # Fall back to active (policy-resolved) target/source when confirmed is unavailable.
    display_speed_limit = self.slimit.slc_target if self.slimit.slc_target > 0 else self.slimit.slc_active_target
    display_source = self.slimit.slc_source if self.slimit.slc_source != "None" else self.slimit.slc_active_source

    if display_speed_limit > 0:
      self.speed_limit_last = display_speed_limit
      self.speed_limit_final_last = display_speed_limit + self.slimit.slc_offset
    elif display_source == "None":
      self.speed_limit_last = 0.0
      self.speed_limit_final_last = 0.0
    # Respect user-defined max cruise speed when applying SLC.
    if v_cruise_cluster > 0 and self.speed_limit_final_last > 0:
      self.speed_limit_final_last = min(self.speed_limit_final_last, v_cruise_cluster)
    source_map = {
      "Dashboard": SpeedLimitSource.car,
      "Map Data": SpeedLimitSource.map,
      "Mapbox": SpeedLimitSource.map,
      "Iqlink": SpeedLimitSource.map,  # Gaode via IQ-link BLE
      "None": SpeedLimitSource.none,
    }
    self.speed_limit_source = source_map.get(display_source, SpeedLimitSource.none)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.speedLimitAssist: (slc_v_cruise, a_ego),
    }
    # Follow-lead owns cruise pressure; a nav red-stop still publishes so the car stays held.
    if self.nav_valid and (self.nav_stop_request or not has_follow_lead) and not block_nav:
      if iqlink_on:
        if self.nav_stop_request:
          nav_v = 0.0
        elif self.nav_accel_target <= _NAV_RED_DECEL:
          nav_v = max(self.nav_speed_target, 0.0)
        else:
          nav_v = max(self.nav_speed_target, 0.0, _NAV_EXEC_MIN_MS) + self._nav_device_offset_ms()
        targets[LongitudinalPlanSource.nav] = (nav_v, self.nav_accel_target)
      else:
        targets[LongitudinalPlanSource.nav] = (self.nav_speed_target, self.nav_accel_target)

    if iqlink_on and self.nav_valid and (not has_follow_lead or self.nav_stop_request) and not block_nav:
      targets.pop(LongitudinalPlanSource.cruise, None)
      if LongitudinalPlanSource.speedLimitAssist in targets and LongitudinalPlanSource.nav in targets:
        nav_exec = targets[LongitudinalPlanSource.nav][0]
        if self.nav_stop_request or targets[LongitudinalPlanSource.speedLimitAssist][0] >= nav_exec:
          targets.pop(LongitudinalPlanSource.speedLimitAssist, None)

    # MEB predicative curve: take stricter than nav when enabled (ID.3 / evo-release parity).
    try:
      pred_on = bool(self._params and self._params.get_bool("EnableSLPredReactToCurves"))
      pred_v = float(getattr(CS.cruiseState, "speedLimitPredicative", 0.0) or 0.0)
      if pred_on and pred_v > 0.0 and LongitudinalPlanSource.nav in targets and not self.nav_stop_request:
        nav_v, nav_a = targets[LongitudinalPlanSource.nav]
        if pred_v < nav_v:
          targets[LongitudinalPlanSource.nav] = (pred_v, nav_a)
    except Exception:
      pass

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    after_force = self._apply_force_stop(self.output_v_target, v_ego, sm, slc_apply_enabled)
    if iqlink_on and self.nav_stop_request and not block_nav:
      self.output_v_target = min(after_force, 0.0)
      if self.nav_accel_target < self.output_a_target:
        self.output_a_target = self.nav_accel_target
    else:
      self.output_v_target = after_force
    prep_v = self._turn_prep_speed(sm, float(getattr(CS, "vEgo", v_ego)), slc_apply_enabled)
    if prep_v is not None:
      self.output_v_target = min(float(self.output_v_target), float(prep_v))
    # envelope shaping only in Assist mode: info/warn must never change the plan
    self._envelope_enabled = (slc_apply_enabled and bool(getattr(self.slimit, "controller_enabled", False))
                              and bool(getattr(self.slimit, "mode_assist", False)))
    return self.output_v_target, self.output_a_target

  def _turn_prep_speed(self, sm: messaging.SubMaster, v_ego: float, apply_enabled: bool) -> float | None:
    """Cap planned speed for an urban blinker turn. Never writes MAX; caller must min()."""
    try:
      cs = sm['carState']
      model = sm['modelV2']
      nav = sm['iqNavState']
    except Exception:
      return None
    posted = float(self.speed_limit_last or 0.0)
    if posted <= 0.0:
      posted = float(getattr(getattr(cs, "cruiseState", None), "speedLimit", 0.0) or 0.0)
    try:
      path_x = model.position.x
      path_y = model.position.y
    except Exception:
      path_x, path_y = None, None
    try:
      lane_change_state = model.meta.laneChangeState
    except Exception:
      lane_change_state = 0
    return self.turn_prep.update(
      v_ego=float(v_ego),
      enabled=bool(apply_enabled),
      left_blinker=bool(getattr(cs, "leftBlinker", False)),
      right_blinker=bool(getattr(cs, "rightBlinker", False)),
      gas_pressed=bool(getattr(cs, "gasPressed", False)),
      steering_angle_deg=float(getattr(cs, "steeringAngleDeg", 0.0) or 0.0),
      posted_limit_ms=posted,
      lane_change_state=lane_change_state,
      path_x=path_x,
      path_y=path_y,
      nav_phase=getattr(nav, "maneuverPhase", 0),
      nav_maneuver_type=getattr(nav, "nextManeuverType", 0),
      nav_maneuver_dir=getattr(nav, "nextManeuverDirection", 0),
      nav_phase_dir=getattr(nav, "maneuverDirection", 0),
      nav_turn_dist_m=float(getattr(nav, "nextManeuverDistance", 0.0) or 0.0),
      nav_send_lc=bool(getattr(nav, "shouldSendLaneChangeDesire", False)),
    )

  def cruise_envelope(self, v_target: float, v_ego: float, t_idxs) -> np.ndarray:
    """Per-timestep cruise speed over the MPC horizon: the scalar target, shaped down
    ahead of an upcoming lower speed limit so the solver decelerates before the sign
    instead of at it."""
    env = np.full(len(t_idxs), max(float(v_target), 0.0))
    if not getattr(self, "_envelope_enabled", False):
      return env
    slc = getattr(self.slimit, "slc", None)
    next_limit = float(getattr(slc, "next_speed_limit", 0.0) or 0.0)
    next_dist = float(getattr(slc, "next_speed_distance", 0.0) or 0.0)
    if next_limit <= 0.0 or next_dist <= 0.0:
      return env
    next_target = max(next_limit + float(getattr(self.slimit, "slc_offset", 0.0) or 0.0), 0.0)
    if next_target >= env[0]:
      return env
    travel = np.maximum(v_ego, 1.0) * np.asarray(t_idxs)
    v_allowed = np.sqrt(np.maximum(next_target ** 2 + 2.0 * abs(LIMIT_ADAPT_ACC) * (next_dist - travel), next_target ** 2))
    return np.minimum(env, v_allowed)

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_iq.clear()
    for event_name in getattr(self.slimit, 'pending_events', []):
      self.events_iq.add(event_name)
    self.custom_stop_distance.update()
    self.traffic_stop_offset.update()
    self.e2e_alerts.update(sm, self.events_iq)
    if bool(getattr(sm["iqCarState"], "alcOverrideAlert", False)):
      self.events_iq.add(custom.IQOnroadEvent.EventName.steeringOverrideReengageAlc)

  def apply_e2e_stop_distance(self, sm: messaging.SubMaster, v_ego: float, a_target: float, should_stop: bool) -> tuple[float, bool]:
    if not self.is_e2e(sm):
      return a_target, should_stop
    a_target, should_stop = self.custom_stop_distance.adjust_e2e_stop(a_target, should_stop, v_ego, sm['modelV2'])
    has_lead = bool(getattr(self.iq_dynamic, "tracking_lead", False))
    try:
      has_lead = has_lead or bool(getattr(sm['radarState'].leadOne, "status", False))
    except Exception:
      pass
    right_blinker = False
    try:
      right_blinker = bool(getattr(sm['carState'], "rightBlinker", False))
    except Exception:
      pass
    return self.traffic_stop_offset.adjust(
      a_target, should_stop, v_ego, sm['modelV2'],
      stop_light=bool(getattr(self.iq_dynamic, "stop_light_detected", False)),
      has_lead=has_lead,
      right_blinker=right_blinker,
      nav_red=bool(self.nav_stop_request),
    )

  def _release_nav_go(self, should_stop: bool, a_target: float, sm: messaging.SubMaster, *, nav_go: bool) -> tuple[bool, float]:
    """Release standstill hold for navigation go, optionally waiting on a close lead."""
    if self._green_follow_gate.may_release(now=time.monotonic(), nav_go=nav_go, sm=sm):
      self._standstill_hold = False
      self._standstill_hold_s = 0.0
      self._hold_released = True
      self._green_launch = True
      return False, a_target
    self._standstill_hold = True
    self._standstill_hold_s = 0.0
    self._green_launch = False
    self._hold_released = False
    return True, min(float(a_target), 0.0)

  def apply_standstill_hold(self, should_stop: bool, a_target: float, v_ego: float, sm: messaging.SubMaster) -> tuple[bool, float]:
    """Keep the car held after a light / model stop until go is stable.

    Arm on nav red-stop, Force Stop, planner shouldStop, or vision model-stop.
    Release does not wait for sticky vision-stop at the line (short E2E path).
    Explicit remainS==1 launches immediately for head car (no close lead); with a
    close lead, wait until the lead moves or FOLLOW_TIMEOUT_S. APK green still waits
    ~1 s of stable go before the same follow-lead gate. Gas always wins.
    """
    cs = sm['carState']
    standstill = bool(getattr(cs, "standstill", False) or v_ego <= 0.3)
    accel_pressed = False
    try:
      accel_pressed = bool(getattr(sm['iqCarState'], "accelPressed", False))
    except Exception:
      accel_pressed = False
    gas = bool(getattr(cs, "gasPressed", False) or accel_pressed)
    remain_go = False
    try:
      remain_go = abs(float(getattr(sm['iqNavState'], "trafficLightRemainS", 0.0) or 0.0) - 1.0) < 1e-6
    except Exception:
      remain_go = False
    nav_hold = bool(self.nav_stop_request)
    try:
      # Left-arrow red: keep hold even if the approach curve still has speedTarget > 0.
      # remainS==1 / APK green still releases below.
      left_red = (
        (not remain_go)
        and nav_turn_pending(sm['iqNavState'], side="left")
        and light_token(getattr(sm['iqNavState'], "trafficLight", None)) == "red"
      )
      nav_hold = nav_hold or left_red
    except Exception:
      pass
    force = bool(self.forcing_stop)
    model_hold = bool(
      getattr(self.iq_dynamic, "stop_light_detected", False)
      or getattr(self.iq_dynamic, "model_stopped", False)
    )
    nav_green = False
    try:
      nav_green = light_token(getattr(sm['iqNavState'], "trafficLight", None)) == "green"
    except Exception:
      nav_green = False
    nav_go = bool(nav_green or remain_go)

    if not hasattr(self, "_green_launch"):
      self._green_launch = False
    if not hasattr(self, "_hold_released"):
      self._hold_released = False

    if gas or v_ego > 2.0:
      self._standstill_hold = False
      self._standstill_hold_s = 0.0
      self._green_launch = False
      self._hold_released = False
      self._green_follow_gate.reset()
      return should_stop, a_target

    # IQ-link: remainS==1 immediate for head car; follow-lead gate when queued.
    if remain_go and not force:
      return self._release_nav_go(should_stop, a_target, sm, nav_go=True)

    if standstill:
      if nav_go:
        arm = force
      elif self._hold_released or self._green_launch:
        arm = should_stop or nav_hold or force
      else:
        arm = should_stop or nav_hold or force or model_hold
      if arm:
        self._standstill_hold = True
        self._green_launch = False
        self._hold_released = False

    if self._green_launch and nav_go and not nav_hold and not force:
      return False, a_target

    if not self._standstill_hold:
      self._standstill_hold_s = 0.0
      return should_stop, a_target

    if nav_go and not force:
      can_go = True
    else:
      can_go = (not should_stop) and (not nav_hold) and (not force)

    if can_go:
      self._standstill_hold_s += DT_MDL
    else:
      self._standstill_hold_s = 0.0

    if self._standstill_hold_s < _STANDSTILL_HOLD_RELEASE_S:
      return True, min(float(a_target), 0.0)

    self._standstill_hold = False
    self._standstill_hold_s = 0.0
    self._hold_released = True
    if nav_go:
      return self._release_nav_go(should_stop, a_target, sm, nav_go=True)
    return should_stop, a_target

  def _apply_force_stop(self, v_target: float, v_ego: float, sm: messaging.SubMaster, apply_enabled: bool) -> float:
    force_stop = self.iq_dynamic.force_stop_requested() and apply_enabled and self.override_force_stop_timer <= 0.0
    self.force_stop_timer = self.force_stop_timer + DT_MDL if force_stop else 0.0
    force_stop_enabled = self.force_stop_timer >= 1.0
    force_stop_ramp_time = max(float(getattr(self.iq_dynamic, "model_stop_time", IQConstants.FORCE_STOP_PLANNER_TIME)), DT_MDL)

    accel_pressed = bool(getattr(sm["iqCarState"], "accelPressed", False))
    self.override_force_stop |= sm["carState"].gasPressed or accel_pressed
    self.override_force_stop &= force_stop_enabled

    if self.override_force_stop:
      self.override_force_stop_timer = 10.0
    elif self.override_force_stop_timer > 0.0:
      self.override_force_stop_timer = max(0.0, self.override_force_stop_timer - DT_MDL)
    else:
      self.override_force_stop = False

    if force_stop_enabled and not self.override_force_stop:
      self.forcing_stop = True
      self.tracked_model_length = max(self.tracked_model_length - (v_ego * DT_MDL), 0.0)
      if sm["carState"].standstill:
        return 0.0
      return min(self.tracked_model_length / force_stop_ramp_time, v_target)

    self.forcing_stop = False
    self.tracked_model_length = max(
      float(getattr(self.iq_dynamic, "model_length", 0.0)),
      float(getattr(self.iq_dynamic, "minimum_force_stop_length", 0.0)),
      0.0,
    )
    return v_target

  def publish_longitudinal_plan_iq(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    has_follow_lead = False
    try:
      has_follow_lead = bool(getattr(sm['radarState'].leadOne, "status", False))
    except Exception:
      has_follow_lead = False
    nav_light = "none"
    try:
      nav_light = light_token(getattr(sm['iqNavState'], "trafficLight", None))
    except Exception:
      nav_light = "none"
    self.junction_hud = junction_hud_active(
      has_lead=has_follow_lead,
      nav_red_decel=bool(self.nav_valid and self.nav_accel_target <= _NAV_RED_DECEL),
      stop_light=bool(getattr(self.iq_dynamic, "stop_light_detected", False)),
      standstill_hold=bool(self._standstill_hold),
      light=nav_light,
    )

    def fill_plan(plan_msg) -> None:
      plan_msg.longitudinalPlanSource = self.source
      plan_msg.vTarget = float(self.output_v_target)
      plan_msg.aTarget = float(self.output_a_target)
      plan_msg.events = self.events_iq.to_msg()

      # IQ.Dynamic control state
      iq_dynamic = plan_msg.iqDynamic
      iq_dynamic.state = IQDynamicState.blended if self.iq_dynamic.mode() == 'blended' else IQDynamicState.acc
      iq_dynamic.enabled = self.iq_dynamic.enabled()
      iq_dynamic.active = self.iq_dynamic.active()
      iq_dynamic.curveDetected = bool(self.iq_dynamic.active() and self.iq_dynamic.curve_slowdown_active)

      nav_summary = plan_msg.iqNavState.nav
      nav_summary.engaged = self.nav_engaged
      nav_summary.provider = self.nav_provider
      nav_summary.state = self.nav_state
      nav_summary.speedTarget = float(self.nav_speed_target)
      nav_summary.accelTarget = float(self.nav_accel_target)
      nav_summary.valid = self.nav_valid

      # Speed Limit
      speedLimit = plan_msg.speedLimit
      resolver = speedLimit.resolver
      speed_limit = float(self.slimit.slc_target if self.slimit.slc_target > 0 else self.slimit.slc_active_target)
      speed_limit_offset = float(self.slimit.slc_offset)
      speed_limit_final = speed_limit + speed_limit_offset if speed_limit > 0 else 0.
      speed_limit_valid = speed_limit > 0.
      speed_limit_last_valid = self.speed_limit_last > 0.

      resolver.speedLimit = speed_limit
      resolver.speedLimitLast = float(self.speed_limit_last)
      resolver.speedLimitFinal = float(speed_limit_final)
      resolver.speedLimitFinalLast = float(self.speed_limit_final_last)
      resolver.speedLimitValid = speed_limit_valid
      resolver.speedLimitLastValid = speed_limit_last_valid
      resolver.speedLimitOffset = speed_limit_offset
      resolver.distToSpeedLimit = 0.
      resolver.source = self.speed_limit_source

      assist = speedLimit.assist
      slc_assist_state = self.slimit.assist_state
      assist.enabled = bool(self.slimit.slc_target > 0 or self.slimit.slc_unconfirmed > 0)
      assist.active = self.source == LongitudinalPlanSource.speedLimitAssist and self.slimit.slc_target > 0
      if slc_assist_state is not None:
        assist.state = slc_assist_state
      elif not assist.enabled:
        assist.state = SpeedLimitAssistState.disabled
      elif self.slimit.slc_unconfirmed > 0:
        assist.state = SpeedLimitAssistState.preActive
      elif assist.active:
        assist.state = SpeedLimitAssistState.active
      else:
        assist.state = SpeedLimitAssistState.inactive
      assist.vTarget = float(self.output_v_target if assist.active else 255.)
      assist.aTarget = float(self.slimit.slc_a_target if assist.active else 0.)

      e2eAlerts = plan_msg.e2eAlerts
      e2eAlerts.pathOpen = self.e2e_alerts.path_alert
      e2eAlerts.leadPullaway = self.e2e_alerts.lead_alert
      e2eAlerts.junctionStop = bool(self.junction_hud)

    valid = sm.all_checks(service_list=['carState', 'controlsState'])

    plan_iq_send = messaging.new_message('iqPlan')
    plan_iq_send.valid = valid
    fill_plan(plan_iq_send.iqPlan)
    pm.send('iqPlan', plan_iq_send)
