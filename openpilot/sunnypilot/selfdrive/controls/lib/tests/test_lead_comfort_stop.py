"""Comfort stop behind a stopped lead: no zoned steps, late stopping state, slewed onset."""
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.green_follow_lead import apply_stopped_lead_gap
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_comfort_stop import (
  MIN_GAP_M, SETTLED_HOLD_A, STOP_V_MPS, TAPER_A, BrakeOnsetLimiter, comfort_stop_accel,
  lead_brake_urgent, settle_cap,
)
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_stop_safety import (
  apply_lead_stop_safety, apply_radar_range_floor,
)

DT = 0.05


class _Lead:
  def __init__(self, d_rel, v_lead=0.0, present=True):
    self.present = present
    self.status = present
    self.dRel = d_rel
    self.vLead = v_lead
    self.aLeadK = 0.0


def _sm(d_rel, v_lead=0.0):
  return {"radarState": type("R", (), {"leadOne": _Lead(d_rel, v_lead)})()}


def _chain(v_ego, d_rel, a_target=0.0, should_stop=False):
  sm = _sm(d_rel)
  a, stop = apply_stopped_lead_gap(sm, v_ego, a_target, should_stop)
  a, stop = apply_lead_stop_safety(sm, v_ego, a, stop)
  return apply_radar_range_floor(sm, v_ego, a, stop)


def test_profile_is_continuous_across_old_zone_edges():
  # Old code stepped −1.5 at 8 m, −2.2 at 5 m, −2.5 at 3.5 m in one frame.
  for v in (1.0, 2.0, 3.0):
    prev = None
    d = 9.0
    while d > 3.0:
      if comfort_stop_accel(v, d, 0.0)[1]:
        break  # physically urgent: kinematic escalation is intended
      a, _ = _chain(v, d)
      if prev is not None:
        assert abs(a - prev) <= 0.2, (v, d, a, prev)
      prev = a
      d -= 0.05


def test_old_jab_point_is_soft_and_not_stopping():
  # Measured jab: ~7 km/h at ~4.4 m → was −2.2 plus forced LongControl stopping.
  a, stop = _chain(2.0, 4.4)
  assert a >= -1.6
  assert stop is False


def test_stopping_state_only_below_stop_speed():
  _, stop_fast = _chain(STOP_V_MPS + 0.2, 4.2)
  _, stop_slow = _chain(STOP_V_MPS - 0.1, 4.2)
  assert stop_fast is False
  assert stop_slow is True


def test_urgent_when_min_gap_needs_hard_brake():
  a, urgent = comfort_stop_accel(8.0, 12.0, 0.0)
  assert urgent is True
  assert a <= -2.5


def test_inside_min_gap_closing_fast_brakes_hard():
  a, stop = _chain(1.5, MIN_GAP_M - 0.3)
  assert stop is True
  assert a <= -2.2


def test_settled_holds_and_rolling_eases():
  assert settle_cap(0.05) == SETTLED_HOLD_A
  assert settle_cap(0.3) == -TAPER_A
  a, stop = _chain(0.05, 3.6)
  assert stop is True and a <= SETTLED_HOLD_A


def test_taper_releases_before_standstill():
  a_fast, _ = comfort_stop_accel(1.2, 4.5, 0.0)
  a_slow, _ = comfort_stop_accel(0.6, 3.9, 0.0)
  assert a_slow > a_fast
  assert a_slow >= -1.0


def test_far_lead_not_closing_is_untouched():
  assert comfort_stop_accel(1.0, 20.0, 2.0) == (None, False)


def test_limiter_slews_onset_and_passes_urgent():
  lim = BrakeOnsetLimiter()
  lim.update(0.0, 10.0, DT, bypass=False)
  first = lim.update(-2.0, 10.0, DT, bypass=False)
  assert -0.06 <= first < 0.0
  for _ in range(40):
    out = lim.update(-2.0, 10.0, DT, bypass=False)
  assert out == -2.0
  lim.update(0.0, 10.0, DT, bypass=False)
  assert lim.update(-3.0, 10.0, DT, bypass=True) == -3.0


def test_limiter_lifts_throttle_freely_and_ignores_low_speed():
  lim = BrakeOnsetLimiter()
  lim.update(1.2, 3.0, DT, bypass=False)
  out = lim.update(-1.0, 3.0, DT, bypass=False)
  assert -0.06 <= out <= 0.0
  lim.update(0.0, 0.3, DT, bypass=False)
  assert lim.update(-1.2, 0.3, DT, bypass=False) == -1.2


def test_urgency_checks():
  assert lead_brake_urgent(10.0, 0.0, 0.0, 5.0) is True     # TTC 2 s
  assert lead_brake_urgent(60.0, 18.0, 0.0, 20.0) is False  # slow close, far
  assert lead_brake_urgent(25.0, 10.0, -3.0, 10.0) is True  # lead braking hard


def _closed_loop(v0, d0, deliver=1.0, tau=0.3):
  v, d, a_act = v0, d0, 0.0
  lim = BrakeOnsetLimiter()
  lim.a_prev = 0.0
  peak = 0.0
  min_d = d0
  for _ in range(1200):
    a, _stop = _chain(v, d)
    a = lim.update(a, v, DT, bypass=lead_brake_urgent(d, 0.0, 0.0, v) or v < 0.5)
    if v > 0.5:
      peak = min(peak, a)
    a_act += (a * (deliver if a < 0 else 1.0) - a_act) * DT / tau
    v = max(0.0, v + a_act * DT)
    d -= v * DT
    min_d = min(min_d, d)
    if v == 0.0:
      break
  return peak, min_d, v


def test_closed_loop_city_stop_is_gentle():
  peak, min_d, v = _closed_loop(30 / 3.6, 60.0)
  assert v == 0.0
  assert peak >= -1.2
  assert 3.0 <= min_d <= 4.5


def test_closed_loop_under_delivery_stays_bounded():
  peak, min_d, v = _closed_loop(50 / 3.6, 80.0, deliver=0.85)
  assert v == 0.0
  assert peak >= -2.2
  assert min_d >= MIN_GAP_M


def test_hot_queue_arrival_settles_closer_instead_of_stamping():
  # ~13 km/h with a stopped car 6.6 m ahead: the old zones asked for ~-2.2 to -2.6.
  a, urgent = comfort_stop_accel(3.5, 6.6, 0.0)
  assert not urgent
  assert a >= -1.6
  a, urgent = comfort_stop_accel(3.9, 6.6, 0.0)
  assert not urgent
  assert a >= -2.6
