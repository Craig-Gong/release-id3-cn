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
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.standstill_hold import StandstillHold, apply_follow_launch
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.traffic_stop_offset import TrafficStopOffset
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.turn_prep import UrbanTurnPrep
from openpilot.sunnypilot.nav.snapshot import read_snapshot, snapshot_executable, write_cluster_hud
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc)
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
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
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
    if snapshot_executable(snap):
      curve = nav_soft_curve_ms(snap, v_ego)
      if curve is not None:
        self.output_v_target = min(float(self.output_v_target), float(curve))
      if snap.stop_for_light:
        self.output_v_target = min(float(self.output_v_target), float(snap.speed_target))
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
    )

  def apply_stop_helpers(self, sm: messaging.SubMaster, v_ego: float, a_target: float,
                         should_stop: bool) -> tuple[float, bool]:
    try:
      CS = sm['carState']
      model = sm['modelV2']
      lead = sm['radarState'].leadOne
    except Exception:
      return a_target, should_stop
    has_lead = bool(getattr(lead, "status", False))
    model_stop = bool(getattr(model.action, "shouldStop", False))
    self.traffic_stop_offset.update()
    a_target, should_stop = self.traffic_stop_offset.adjust(
      a_target, should_stop, v_ego, model,
      stop_light=model_stop, has_lead=has_lead, right_blinker=bool(CS.rightBlinker),
    )
    snap = read_snapshot()
    if snapshot_executable(snap) and snap.stop_for_light:
      a_target = min(float(a_target), float(snap.accel_target))
      if snap.speed_target <= 0.05:
        should_stop = True
    should_stop, a_target = self.standstill_hold.apply(
      should_stop, a_target, v_ego,
      standstill=bool(CS.standstill), gas=bool(CS.gasPressed), model_stop=model_stop,
      sm=sm, now=time.monotonic(),
    )
    a_target = apply_follow_launch(sm, v_ego, a_target)
    approaching = junction_stop_active(
      has_lead=has_lead, nav_red=bool(snap.stop_for_light), model_stop=model_stop,
      standstill_hold=self.standstill_hold.hold, light=snap.light_token,
    )
    write_cluster_hud(approaching=approaching and not bool(CS.standstill),
                      standstill=bool(CS.standstill) and approaching)
    return a_target, should_stop

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self.dec.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

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
