"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import time

from openpilot.cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.junction_hud import junction_stop_active
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_soft_curve import nav_soft_curve_ms
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_cruise_policy import (
  NavCruisePolicy, apply_a_pos_cap,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.green_follow_lead import (
  apply_stopped_lead_gap, follow_lead_present, lead_owns_nav_stop, read_nav_queue_lead,
  read_nav_red_lead, radar_state_readable,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_comfort_stop import (
  BrakeOnsetLimiter, lead_brake_urgent, read_lead_kinematics,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_follow_comfort import (
  AccelOnsetLimiter, FollowLaunchController,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_stop_safety import (
  apply_lead_stop_safety,
  apply_radar_range_floor,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.standstill_hold import (
  StandstillHold, apply_follow_launch, hold_approach_accel, nav_light_live,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.traffic_stop_offset import TrafficStopOffset
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.turn_prep import UrbanTurnPrep
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.unprotected_turn import (
  UnprotectedTurnAssist, CutDecision,
)
from openpilot.sunnypilot.nav.protocol import (
  nav_red_accel_cap,
  nav_red_force_stop,
  nav_red_remaining_m,
  nav_red_speed_ms,
  traffic_stop_margin_m,
)
from openpilot.sunnypilot.nav.snapshot import nav_light_dist, read_snapshot, snapshot_executable, write_cluster_hud
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import snapshot_long_ok

# Yellow with a real distance: if stopping needs more than this, go through.
_YELLOW_DILEMMA_DECEL = 2.5
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
    self.nav_cruise = NavCruisePolicy()
    self.cut = UnprotectedTurnAssist()
    self.traffic_stop_offset = TrafficStopOffset()
    self.standstill_hold = StandstillHold()
    self.follow_launch = FollowLaunchController()
    self.accel_onset = AccelOnsetLimiter()
    self.brake_onset = BrakeOnsetLimiter()
    # Red-light stops keep their own slew (vision 0.95/3.2, nav red).
    self._light_stop_active = False
    # Nav-red a_cap slew state (jerk limit across planner frames).
    self._nav_red_a_prev: float | None = None
    self._nav_red_a_t = 0.0
    self._nav_policy_a_pos: float | None = None
    self._cut_decision = CutDecision()

    self.output_v_target = 0.
    self.output_a_target = 0.

  def _nav_red_plan(self, sm: messaging.SubMaster, snap, v_ego: float,
                    accel_target: float) -> tuple[float, float, float]:
    """Return (remaining_m, margin, a_cap) for head-car red.

    Fuses mmWave bumper gap when a track sits short of the light; slews a_cap.
    Same-frame re-entry (update_targets + apply_stop_helpers) reuses the cap.
    """
    now = time.monotonic()
    if (self._nav_red_a_prev is not None and self._nav_red_a_t > 0.0
        and (now - self._nav_red_a_t) < 0.02
        and getattr(self, "_nav_red_cache_rem", None) is not None):
      return float(self._nav_red_cache_rem), float(self._nav_red_cache_margin), float(self._nav_red_a_prev)

    margin = float(traffic_stop_margin_m())
    light_d = float(snap.dist_m or 0.0)
    lead = read_nav_red_lead(sm, light_d)
    lead_d = float(lead.d_rel) if lead.present else None
    remaining = nav_red_remaining_m(light_d, margin, lead_d_rel=lead_d)
    # Stop Line Extra (live slider), not folded into the 10 m offset cap.
    brake_rem = float(remaining) - float(self.traffic_stop_offset.lead_m)
    dt = 0.05
    if self._nav_red_a_t > 0.0:
      dt = max(1e-3, min(0.2, now - self._nav_red_a_t))
    a_cap = nav_red_accel_cap(
      v_ego, light_d, margin, float(accel_target),
      remaining_m=brake_rem, prev_a=self._nav_red_a_prev, dt=dt,
    )
    self._nav_red_a_prev = float(a_cap)
    self._nav_red_a_t = now
    self._nav_red_cache_rem = float(remaining)
    self._nav_red_cache_margin = float(margin)
    return remaining, margin, a_cap

  def _nav_red_dist_active(self, snap, v_ego: float, gear, now: float | None = None) -> bool:
    """Nav red/yellow with a real light distance, fresh from Gaode."""
    if not (snap.stop_for_light and snap.dist_ok and nav_light_live(snap, gear, now=now)):
      return False
    if snap.light_token == "yellow":
      d = max(1.0, float(snap.dist_m or 0.0))
      if (float(v_ego) ** 2) / (2.0 * d) > _YELLOW_DILEMMA_DECEL:
        return False
    return True

  def _clear_nav_red_a(self) -> None:
    self._nav_red_a_prev = None
    self._nav_red_a_t = 0.0
    self._nav_red_cache_rem = None
    self._nav_red_cache_margin = None

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    # Real bumper → ACC/MPC. Vision phantoms at empty lights must NOT kill e2e
    # (same radar-first rule as nav green / queue gates).
    try:
      if radar_state_readable(sm):
        if read_nav_queue_lead(sm).present:
          return False
      elif sm['radarState'].leadOne.present:
        return False
    except Exception:
      pass
    # CTM / experimental parity: do NOT disable e2e on nav/sticky red.
    # Model owns the no-lead approach brake; standstill_hold pins a≤−1 at rest
    # until confirmed green (MEB must not ANFAHREN on e2e creep).
    try:
      CS = sm['carState']
      # While latched after a confirmed go, do not let a vision shouldStop
      # hitch force cruise-only. A raw nav green token is not enough — it may
      # be another direction's light; the latch already required the camera.
      nav_go_guard = bool(getattr(self.standstill_hold, "_nav_go_latched", False))
      if (not nav_go_guard and (CS.standstill or float(CS.vEgo) <= 0.6)
          and bool(sm['modelV2'].action.shouldStop)):
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
    # Posted limit → MAX (cruise_ext). SLA must not min() a gas/button-raised
    # cruise target back to the same limit (feels like "snap back to limit"
    # the instant the accelerator is released — including BLE blips that
    # briefly clear snapshot_executable while Assist is still active).
    snap_gate = read_snapshot()
    sla_v = float(self.sla.output_v_target)
    # iqlinkd mirrors IqlinkEnabled into the snapshot at 5 Hz; no per-frame Params read.
    neutralize_sla = bool(snapshot_executable(snap_gate) or snap_gate.iqlink_enabled)
    # Gas Sync / SET raised MAX above SLA's posted target.
    if (not neutralize_sla) and sla_v < float(V_CRUISE_UNSET) and v_cruise > sla_v + 0.5:
      neutralize_sla = True
    if neutralize_sla:
      targets[LongitudinalPlanSource.speedLimitAssist] = (float(V_CRUISE_UNSET), a_ego)

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    prep_v = self._turn_prep_speed(sm, v_ego, long_enabled)
    if prep_v is not None:
      self.output_v_target = min(float(self.output_v_target), float(prep_v))
    snap = snap_gate
    self._nav_policy_a_pos = None
    try:
      queue = read_nav_queue_lead(sm)
      near_lead = bool(queue.present and float(queue.d_rel) <= 12.0)
    except Exception:
      near_lead = False
    if snapshot_long_ok(snap, CS.gearShifter):
      curve = nav_soft_curve_ms(snap, v_ego)
      if curve is not None:
        self.output_v_target = min(float(self.output_v_target), float(curve))
      # Nav red = safety floor only (min with cruise/e2e). CTM owns far-field
      # feel when experimental e2e is active; soft FAR/COAST must not replace it.
      # Needs a real light distance: a guessed one braked for phantom lines.
      if (self._nav_red_dist_active(snap, v_ego, CS.gearShifter)
          and not lead_owns_nav_stop(sm, snap)):
        remaining, margin, a_cap = self._nav_red_plan(
          sm, snap, v_ego, float(snap.accel_target or -2.0),
        )
        road_ms = 0.0
        if float(snap.road_limit_kph or 0.0) >= 20.0:
          road_ms = float(snap.road_limit_kph) * CV.KPH_TO_MS
        v_nav = nav_red_speed_ms(snap.dist_m, road_ms, margin, remaining_m=remaining)
        self.output_v_target = min(float(self.output_v_target), v_nav)
        self.output_a_target = min(float(self.output_a_target), a_cap)
      else:
        self._clear_nav_red_a()
      # Camera / dest / dual-TBT / congestion / light accel — after soft curve.
      decision = self.nav_cruise.update(
        snap, float(v_ego),
        gear=CS.gearShifter,
        has_near_lead=near_lead,
        standstill=bool(CS.standstill),
        gas=bool(CS.gasPressed),
        long_enabled=bool(long_enabled),
      )
      if decision.v_cap_ms is not None:
        self.output_v_target = min(float(self.output_v_target), float(decision.v_cap_ms))
      self._nav_policy_a_pos = decision.a_pos_cap
    else:
      self._clear_nav_red_a()
      self.nav_cruise.reset()
    # Cautious unprotected left: near-corner 12 km/h cap (never gap-accept).
    self._cut_decision = self._cut_update(sm, v_ego, long_enabled, near_lead, snap)
    if self._cut_decision.v_cap_ms is not None:
      self.output_v_target = min(float(self.output_v_target), float(self._cut_decision.v_cap_ms))
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

  def _cut_update(self, sm: messaging.SubMaster, v_ego: float, enabled: bool,
                  near_lead: bool, snap) -> CutDecision:
    try:
      CS = sm['carState']
      model = sm['modelV2']
    except Exception:
      self.cut.reset()
      return CutDecision()
    try:
      path_x = model.position.x
      path_y = model.position.y
    except Exception:
      path_x, path_y = None, None
    try:
      lane_change_state = int(model.meta.laneChangeState)
    except Exception:
      lane_change_state = 0
    posted = float(self.resolver.speed_limit or 0.0)
    return self.cut.update(
      v_ego=float(v_ego),
      enabled=bool(enabled),
      standstill=bool(CS.standstill),
      gas=bool(CS.gasPressed),
      left_blinker=bool(CS.leftBlinker),
      right_blinker=bool(CS.rightBlinker),
      steering_angle_deg=float(CS.steeringAngleDeg or 0.0),
      lane_change_state=lane_change_state,
      near_lead=bool(near_lead),
      posted_limit_ms=posted,
      path_x=path_x,
      path_y=path_y,
      snap=snap,
      nav_go_latched=bool(getattr(self.standstill_hold, "_nav_go_latched", False)),
    )

  def limit_accel_onset(self, sm: messaging.SubMaster, v_ego: float, a_target: float, dt: float,
                        *, reset: bool) -> float:
    """Slew positive accel increases; standstill go / gas / reset pass through."""
    bypass = bool(reset)
    if not bypass:
      try:
        CS = sm['carState']
        # One-frame MEB ANFAHREN floor must clear standstill without being slewed.
        bypass = bool(CS.gasPressed or CS.standstill)
      except Exception:
        bypass = True
    out = self.accel_onset.update(a_target, v_ego, dt, bypass=bypass)
    # Keep brake limiter's memory aligned so the two half-axes share one trail.
    if self.brake_onset.a_prev is None:
      self.brake_onset.a_prev = self.accel_onset.a_prev
    return out

  def limit_brake_onset(self, sm: messaging.SubMaster, v_ego: float, a_target: float, dt: float,
                        *, reset: bool) -> float:
    """Slew brake increases of the final accel; physics-urgent cases pass through."""
    bypass = bool(reset or self._light_stop_active)
    if not bypass:
      try:
        CS = sm['carState']
        bypass = bool(CS.gasPressed or CS.standstill)
      except Exception:
        bypass = True
    if not bypass:
      kin = read_lead_kinematics(sm)
      if kin is not None:
        bypass = lead_brake_urgent(kin[0], kin[1], kin[2], v_ego)
    out = self.brake_onset.update(a_target, v_ego, dt, bypass=bypass)
    self.accel_onset.a_prev = self.brake_onset.a_prev
    return out

  def apply_stop_helpers(self, sm: messaging.SubMaster, v_ego: float, a_target: float,
                         should_stop: bool) -> tuple[float, bool]:
    self._light_stop_active = False
    try:
      CS = sm['carState']
      model = sm['modelV2']
      lead = sm['radarState'].leadOne
    except Exception:
      return a_target, should_stop
    now = time.monotonic()
    snap = read_snapshot()
    light_live = nav_light_live(snap, CS.gearShifter, now=now)
    nav_dist_red = self._nav_red_dist_active(snap, v_ego, CS.gearShifter, now=now)
    # Arm sticky red before lead-gap / e2e so a stale link cannot re-enable creep.
    self.standstill_hold.observe_nav(
      snap, now, gas=bool(CS.gasPressed), v_ego=float(v_ego), gear=CS.gearShifter, sm=sm,
    )
    red_pin = bool(self.standstill_hold.red_pin)
    lead_owns = lead_owns_nav_stop(sm, snap)

    # Offset / HUD "has lead": radar-first so vision phantoms at empty lights
    # do not cancel TrafficStopOffset (same as is_e2e / green queue).
    queue = read_nav_queue_lead(sm)
    if radar_state_readable(sm):
      has_lead = bool(queue.present)
      lead_d_rel = float(queue.d_rel) if queue.present else None
    else:
      has_lead = follow_lead_present(sm) or bool(getattr(lead, "present", False))
      try:
        lead_d_rel = float(getattr(lead, "dRel", 0.0) or 0.0) if has_lead else None
      except (TypeError, ValueError):
        lead_d_rel = None
    model_stop = bool(getattr(model.action, "shouldStop", False))
    self.traffic_stop_offset.update()
    # CTM parity: live IQ-link light color must NOT skip TrafficStopOffset.
    # Skip only while the go latch holds (avoid hitch); a sustained model stop
    # cancels the latch in standstill_hold — lead/RTOR handled inside adjust.
    skip_vision_stop = bool(getattr(self.standstill_hold, "_nav_go_latched", False))
    a_target, should_stop = self.traffic_stop_offset.adjust(
      a_target, should_stop, v_ego, model,
      stop_light=model_stop, has_lead=has_lead, right_blinker=bool(CS.rightBlinker),
      lead_d_rel=lead_d_rel, nav_red=skip_vision_stop,
      steering_angle_deg=float(CS.steeringAngleDeg or 0.0),
    )
    a_target, should_stop = apply_stopped_lead_gap(
      sm, v_ego, a_target, should_stop, red_pin=red_pin, model_stop=model_stop,
    )
    a_target, should_stop = apply_lead_stop_safety(sm, v_ego, a_target, should_stop)
    if nav_dist_red and not lead_owns:
      # Safety net only: min() with model/e2e — near lamp if CTM has not shouldStop yet.
      remaining, margin, a_cap = self._nav_red_plan(
        sm, snap, v_ego, float(snap.accel_target or -2.0),
      )
      a_target = min(float(a_target), a_cap)
      # Only force LongControl.stopping near/past the line (standstill_hold pins at rest).
      if nav_red_force_stop(v_ego, float(snap.dist_m), margin, remaining_m=remaining):
        should_stop = True
        if v_ego <= 0.6 or remaining <= 0.0:
          a_target = min(float(a_target), -1.5)
    should_stop, a_target = self.standstill_hold.apply(
      should_stop, a_target, v_ego,
      standstill=bool(CS.standstill), gas=bool(CS.gasPressed), model_stop=model_stop,
      sm=sm, now=now,
    )
    a_target = apply_follow_launch(sm, v_ego, a_target, controller=self.follow_launch)
    # Radar dRel is the last word — green / launch floors cannot punch a close bumper.
    a_target, should_stop = apply_radar_range_floor(
      sm, v_ego, a_target, should_stop, gas=bool(CS.gasPressed),
    )
    lead_going = False
    if lead_owns:
      try:
        own = read_nav_red_lead(sm, nav_light_dist(snap))
        lead_going = bool(own.present and own.v_lead >= 1.0 and own.d_rel >= 5.0)
      except Exception:
        lead_going = False
    # Nav red without a real distance is not a stop intent: it may be the
    # next intersection, and -0.45 below 14 km/h would stall the car short.
    stop_intent = bool(
      (model_stop or self.traffic_stop_offset._engaged or (nav_dist_red and not lead_owns))
      and not lead_going
    )
    self._light_stop_active = bool(
      (self.traffic_stop_offset._engaged or nav_dist_red or red_pin or (model_stop and not has_lead))
      and not lead_owns
    )
    a_target = hold_approach_accel(
      a_target, v_ego, stop_intent=stop_intent,
      go_latched=bool(self.standstill_hold._nav_go_latched),
      gas=bool(CS.gasPressed), right_blinker=bool(CS.rightBlinker),
    )
    a_target = apply_a_pos_cap(
      a_target, self._nav_policy_a_pos, gas=bool(CS.gasPressed),
    )
    # CUT standstill pin last — blocks follow-launch / e2e creep. Gas exits in helper.
    try:
      near_cut_lead = bool(has_lead and lead_d_rel is not None and float(lead_d_rel) <= 12.0)
    except (TypeError, ValueError):
      near_cut_lead = bool(has_lead)
    self._cut_decision = self._cut_update(
      sm, v_ego, bool(sm['carControl'].enabled), near_cut_lead, snap,
    )
    if self._cut_decision.hold and not bool(CS.gasPressed):
      should_stop = True
      a_target = min(float(a_target), -1.0)
    approaching = junction_stop_active(
      has_lead=has_lead,
      nav_red=bool((self.standstill_hold.red_pin or (light_live and snap.stop_for_light)) and not lead_owns),
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
