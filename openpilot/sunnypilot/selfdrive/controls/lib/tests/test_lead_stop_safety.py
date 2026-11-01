"""Unit checks for kinematic lead-stop safety overlay."""
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.lead_stop_safety import (
  STOP_GAP_M, apply_lead_stop_safety,
)


class _Lead:
  def __init__(self, present, d_rel, v_lead):
    self.present = present
    self.dRel = d_rel
    self.vLead = v_lead


def _sm(present=True, d_rel=20.0, v_lead=0.0):
  return {"radarState": type("R", (), {"leadOne": _Lead(present, d_rel, v_lead)})()}


def test_far_stationary_lead_forces_early_brake():
  # 50 km/h into a stopped bumper at 40 m with 6 m gap → slack 34 m
  # a_need = -(13.9^2)/(2*34) ≈ -2.84
  a, stop = apply_lead_stop_safety(_sm(d_rel=40.0, v_lead=0.0), v_ego=13.9, a_target=0.5, should_stop=False)
  assert a < -2.0
  assert stop is False


def test_inside_gap_holds_at_standstill():
  a, stop = apply_lead_stop_safety(_sm(d_rel=3.0, v_lead=0.0), v_ego=0.1, a_target=0.8, should_stop=False)
  assert stop is True
  assert a <= -1.0


def test_no_lead_passthrough():
  a, stop = apply_lead_stop_safety(_sm(present=False), v_ego=10.0, a_target=0.4, should_stop=False)
  assert (a, stop) == (0.4, False)


def test_stop_gap_constant_matches_tuning_default():
  assert STOP_GAP_M == 3.5
