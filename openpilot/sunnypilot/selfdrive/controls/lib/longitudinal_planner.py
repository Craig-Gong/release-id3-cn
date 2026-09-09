"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import time

from openpilot.cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.junction_hud import junction_stop_active
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_soft_curve import nav_soft_curve_ms
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.green_follow_lead import (
  apply_stopped_lead_gap, follow_lead_present,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_stop_safety import apply_lead_stop_safety
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.standstill_hold import StandstillHold, apply_follow_launch
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.traffic_stop_offset import TrafficStopOffset
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.turn_prep import UrbanTurnPrep
from openpilot.sunnypilot.nav.protocol import nav_red_speed_ms, nav_stop_margin_m
from openpilot.sunnypilot.nav.snapshot import read_snapshot, write_cluster_hud
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import snapshot_long_ok
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc, *, enable_dec: bool = True):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc) if enable_dec else None
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()
    self.turn_prep = UrbanTurnPrep()
    self.traffic_stop_offset = TrafficStopOffset()
    self.standstill_hold = StandstillHold()

    self.output_v_target = 0.
    self.output_a_target = 0.

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    # Never blend e2e into a present lead — ACC/MPC owns the gap.
    try:
      if sm['radarState'].leadOne.present:
        return False
    except Exception:
      pass
    # Nav red / sticky red / crawling vision stop: e2e positive accel causes creep→slam.
    if getattr(self.standstill_hold, "red_pin", False):
      return False
    try:
      snap = read_snapshot()
      gear = sm['carState'].gearShifter
      if snapshot_long_ok(snap, gear) and snap.stop_for_light:
        return False
    except Exception:
      pass
    try:
      CS = sm['carState']
      if (CS.standstill or float(CS.vEgo) <= 0.6) and bool(sm['modelV2'].action.shouldStop):
        return False
    except Exception:
      pass
    experimental_mode = sm['selfdriveState'].experimentalMode
    if self.dec is None or not self.dec.active():
      return experimental_mode

    return experimental_mode and self.dec.mode() == "blended"

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    prep_v = self._turn_prep_speed(sm, v_ego, long_enabled)
    if prep_v is not None:
      self.output_v_target = min(float(self.output_v_target), float(prep_v))
    snap = read_snapshot()
    if snapshot_long_ok(snap, CS.gearShifter):
      curve = nav_soft_curve_ms(snap, v_ego)
      if curve is not None:
        self.output_v_target = min(float(self.output_v_target), float(curve))
      if snap.stop_for_light:
        margin = nav_stop_margin_m(self.traffic_stop_offset.distance)
        v_nav = nav_red_speed_ms(snap.dist_m, 0.0, margin)
        self.output_v_target = min(float(self.output_v_target), v_nav)
        self.output_a_target = min(float(self.output_a_target), float(snap.accel_target))
    return self.output_v_target, self.output_a_target

  def _turn_prep_speed(self, sm: messaging.SubMaster, v_ego: float, enabled: bool) -> float | None:
    try:
      CS = sm['carState']
      model = sm['modelV2']
    except Exception:
      return None
    posted = float(self.resolver.speed_limit or 0.0)
    try:
      path_x = model.position.x
      path_y = model.position.y
    except Exception:
      path_x, path_y = None, None
    try:
      lane_change_state = model.meta.laneChangeState
    except Exception:
      lane_change_state = 0
    big = bool(getattr(model, "big", False))
    try:
      snap = read_snapshot()
    except Exception:
      snap = None
    return self.turn_prep.update(
      v_ego=float(v_ego),
      enabled=bool(enabled),
      left_blinker=bool(CS.leftBlinker),
      right_blinker=bool(CS.rightBlinker),
      gas_pressed=bool(CS.gasPressed),
      steering_angle_deg=float(CS.steeringAngleDeg or 0.0),
      posted_limit_ms=posted,
      lane_change_state=lane_change_state,
      path_x=path_x,
      path_y=path_y,
      big=big,
      snap=snap,
    )

  def apply_stop_helpers(self, sm: messaging.SubMaster, v_ego: float, a_target: float,
                         should_stop: bool) -> tuple[float, bool]:
    try:
      CS = sm['carState']
      model = sm['modelV2']
      lead = sm['radarState'].leadOne
    except Exception:
      return a_target, should_stop
    now = time.monotonic()
    snap = read_snapshot()
    long_ok = snapshot_long_ok(snap, CS.gearShifter, now=now)
    # Arm sticky red before lead-gap / e2e so a stale link cannot re-enable creep.
    self.standstill_hold.observe_nav(
      snap, now, gas=bool(CS.gasPressed), v_ego=float(v_ego), gear=CS.gearShifter,
    )
    red_pin = bool(self.standstill_hold.red_pin)

    has_lead = follow_lead_present(sm) or bool(getattr(lead, "present", False))
    try:
      lead_d_rel = float(getattr(lead, "dRel", 0.0) or 0.0) if has_lead else None
    except (TypeError, ValueError):
      lead_d_rel = None
    model_stop = bool(getattr(model.action, "shouldStop", False))
    self.traffic_stop_offset.update()
    nav_red = bool(red_pin or (long_ok and snap.stop_for_light))
    a_target, should_stop = self.traffic_stop_offset.adjust(
      a_target, should_stop, v_ego, model,
      stop_light=model_stop, has_lead=has_lead, right_blinker=bool(CS.rightBlinker),
      lead_d_rel=lead_d_rel, nav_red=nav_red,
    )
    a_target, should_stop = apply_stopped_lead_gap(
      sm, v_ego, a_target, should_stop, red_pin=red_pin, model_stop=model_stop,
    )
    a_target, should_stop = apply_lead_stop_safety(sm, v_ego, a_target, should_stop)
    if red_pin or (long_ok and snap.stop_for_light):
      # Never let lead-gap creep / e2e leave should_stop=False under a red.
      # Approach speed is only for high-speed braking; once crawling or near
      # the line, hard-stop or MEB RELEASE → creep → slam.
      a_target = min(float(a_target), float(snap.accel_target if snap.stop_for_light else -2.0))
      near_line = snap.dist_m <= 0.0 or 0.0 < snap.dist_m <= 12.0 or snap.speed_target <= 0.5
      if v_ego <= 1.5 or near_line or red_pin:
        should_stop = True
        if v_ego <= 0.6 or red_pin:
          a_target = min(float(a_target), -1.0)
    should_stop, a_target = self.standstill_hold.apply(
      should_stop, a_target, v_ego,
      standstill=bool(CS.standstill), gas=bool(CS.gasPressed), model_stop=model_stop,
      sm=sm, now=now,
    )
    a_target = apply_follow_launch(sm, v_ego, a_target)
    approaching = junction_stop_active(
      has_lead=has_lead,
      nav_red=bool(self.standstill_hold.red_pin or snap.stop_for_light),
      model_stop=model_stop,
      standstill_hold=self.standstill_hold.hold, light=snap.light_token,
    )
    write_cluster_hud(approaching=approaching and not bool(CS.standstill),
                      standstill=bool(CS.standstill) and approaching)
    return a_target, should_stop

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self._update_backend(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def _update_backend(self, sm: messaging.SubMaster) -> None:
    """Backend extension point; the upstream provider keeps DEC unchanged."""
    if self.dec is not None:
      self.dec.update(sm)

  def _publish_backend_state(self, longitudinal_plan_sp) -> None:
    """Publish backend-specific diagnostics without forking the common plan."""
    if self.dec is None:
      return
    dec = longitudinal_plan_sp.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    self._publish_backend_state(longitudinalPlanSP)

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)
