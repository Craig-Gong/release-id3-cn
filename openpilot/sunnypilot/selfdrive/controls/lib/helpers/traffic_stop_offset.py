"""Vision red / model-stop offset (IQ-link OFF / no live nav light).

TrafficStopOffset (meters, 0..10 in 0.5 steps, default 3): when the model
wants to stop and there is no real lead short of that point, brake toward a
point this far short of a *filtered* model.position.x[-1] and hold there.
Larger = stop sooner (before the line). 0 disables.

IQ-link nav red uses the same slider via traffic_stop_margin_m() in
nav/protocol.py (floored at 3 m, capped at 6 m so vision-only 8–10 m does
not make head-car nav absurdly early). Lead follow gap is LeadStopDistance /
stopped-lead helpers.

Carrot-inspired (without TrafficState / fake ACC obstacles):
  - rate-limit xStop so the stop point cannot jump nearer faster than ego closes
  - far stops (>~50 m): fade in the offset so first contact is not a hard slam
  - |steer| above ~50° blocks *entry* into a new vision offset (turning)
  - end-velocity gate skips stop-sign cruise-through plans (sunnypilot #1864)

A radar track past the stop point (phantom / cross-traffic) must not cancel
this — that used to disable the offset and let stopped-lead creep push past
the line.
"""
from __future__ import annotations

from opendbc.car.interfaces import ACCEL_MIN
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_MDL

TRAFFIC_STOP_OFFSET_PARAM = "TrafficStopOffset"
MIN_OFFSET_M = 0.0
MAX_OFFSET_M = 10.0
DEFAULT_OFFSET_M = 3.0
OFFSET_STEP_M = 0.5

# Model "shouldStop" plans often end ~1–2 m/s while still stopping; only skip
# clearly non-stopping trajectories (stop-sign cruise-through).
E2E_STOP_PLAN_VEL_THRESHOLD = 2.5
E2E_STOP_HOLD_BUFFER = 2.0
E2E_STOP_MIN_SAMPLES = 4

# Carrot-style: do not let filtered stop jump nearer faster than closing rate.
_STOP_CLOSE_SLACK_M = 0.5
# Fade vision offset in between this and the raw (un-offset) stop distance.
_RELEASE_DISTANCE_M = 50.0
# Suppress *new* vision-offset entry while steering hard (carrot ~50°).
_STEER_ENTRY_LIMIT_DEG = 50.0


def _sanitize_offset_m(raw) -> float:
  try:
    value = float(raw)
  except (TypeError, ValueError):
    return DEFAULT_OFFSET_M
  bounded = min(max(value, MIN_OFFSET_M), MAX_OFFSET_M)
  return round(bounded / OFFSET_STEP_M) * OFFSET_STEP_M


def _lead_owns_stop(has_lead: bool, lead_d_rel: float | None, stop_distance: float) -> bool:
  """True only when a lead sits short of the model stop (real queue)."""
  if not has_lead:
    return False
  if lead_d_rel is None:
    return True
  try:
    d = float(lead_d_rel)
  except (TypeError, ValueError):
    return True
  if d <= 0.5:
    return False
  return d < float(stop_distance)


def soft_release_remaining(hard_remaining: float, soft_remaining: float,
                           release_distance: float = _RELEASE_DISTANCE_M) -> float:
  """Blend toward the offset stop so far contacts do not slam (carrot-like).

  hard_remaining = filtered_stop - offset (intended).
  soft_remaining = filtered_stop (no offset yet).
  When hard is beyond release_distance, interpolate soft→hard as we approach
  release_distance; at/under release_distance use full hard.
  """
  hard = max(0.0, float(hard_remaining))
  soft = max(hard, float(soft_remaining))
  release = max(0.0, float(release_distance))
  if hard <= release or soft <= release:
    return hard
  # hard in (release, soft]: fade from 0 at soft → 1 at release
  t = (soft - hard) / max(soft - release, 1e-3)
  t = min(max(t, 0.0), 1.0)
  return soft + t * (hard - soft)


class TrafficStopOffset:
  def __init__(self, params: Params | None = None):
    self.params = params if params is not None else Params()
    self.frame = 0
    self.distance = float(DEFAULT_OFFSET_M)
    self._filtered_stop: float | None = None
    self._engaged = False
    self.read_params()

  def read_params(self) -> None:
    try:
      stored = self.params.get(TRAFFIC_STOP_OFFSET_PARAM, return_default=True)
      snapped = _sanitize_offset_m(stored if stored is not None else DEFAULT_OFFSET_M)
      if stored is not None and snapped != float(stored):
        self.params.put(TRAFFIC_STOP_OFFSET_PARAM, snapped)
      self.distance = snapped
    except (TypeError, ValueError, UnknownKeyName):
      self.distance = float(DEFAULT_OFFSET_M)

  def update(self) -> None:
    if self.frame % int(3 / DT_MDL) == 0:
      self.read_params()
    self.frame += 1

  def _reset_session(self) -> None:
    self._filtered_stop = None
    self._engaged = False

  def _filter_stop(self, raw_stop: float, v_ego: float) -> float:
    """Allow stop to jump farther instantly; limit noisy jump-near.

    Large drops (model replan to a much nearer stop) snap through so we do
    not under-brake when the horizon suddenly shortens.
    """
    raw = max(0.0, float(raw_stop))
    if self._filtered_stop is None:
      self._filtered_stop = raw
      return raw
    prev = float(self._filtered_stop)
    if raw >= prev:
      self._filtered_stop = raw
    elif (prev - raw) > 8.0:
      self._filtered_stop = raw
    else:
      max_close = max(0.0, float(v_ego)) * float(DT_MDL) + _STOP_CLOSE_SLACK_M
      self._filtered_stop = max(prev - max_close, raw)
    return float(self._filtered_stop)

  def adjust(self, a_target: float, should_stop: bool, v_ego: float, model_msg,
             *, stop_light: bool, has_lead: bool, right_blinker: bool,
             lead_d_rel: float | None = None, nav_red: bool = False,
             steering_angle_deg: float = 0.0) -> tuple[float, bool]:
    if self.distance <= 0. or not stop_light or right_blinker or nav_red:
      self._reset_session()
      return a_target, should_stop

    x = model_msg.position.x
    v = model_msg.velocity.x
    if len(x) < E2E_STOP_MIN_SAMPLES or len(v) < E2E_STOP_MIN_SAMPLES or len(x) != len(v):
      self._reset_session()
      return a_target, should_stop

    raw_stop = float(x[-1])
    if _lead_owns_stop(has_lead, lead_d_rel, raw_stop):
      self._reset_session()
      return a_target, should_stop

    # Stop-sign / cruise-through: model still plans speed at the horizon.
    if float(v[-1]) > E2E_STOP_PLAN_VEL_THRESHOLD:
      self._reset_session()
      return a_target, should_stop

    # Entry gate only — once engaged, finish the stop even if wheel turns.
    if not self._engaged:
      if abs(float(steering_angle_deg)) >= _STEER_ENTRY_LIMIT_DEG:
        return a_target, should_stop
      self._engaged = True

    filtered_stop = self._filter_stop(raw_stop, v_ego)
    hard_remaining = filtered_stop - self.distance
    remaining = soft_release_remaining(hard_remaining, filtered_stop)

    if remaining <= E2E_STOP_HOLD_BUFFER:
      should_stop = True
      brake_d = max(remaining, 0.3)
      a_required = max(-(v_ego ** 2) / (2.0 * brake_d), ACCEL_MIN)
      a_target = min(float(a_target), float(a_required))
    else:
      a_required = max(-(v_ego ** 2) / (2.0 * remaining), ACCEL_MIN)
      if a_required < a_target:
        a_target = float(a_required)

    return a_target, should_stop
