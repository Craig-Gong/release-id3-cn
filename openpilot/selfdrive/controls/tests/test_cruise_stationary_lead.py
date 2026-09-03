from openpilot.selfdrive.controls.radard import KalmanParams, Track


def _track(d_rel, y_rel, v_lead, frames=8):
  t = Track(1, v_lead, KalmanParams(0.05))
  v_rel = v_lead - 16.0
  for _ in range(frames):
    t.update(d_rel, y_rel, v_rel, v_lead)
  return t


def test_cruise_stationary_in_lane_at_50kph():
  t = _track(d_rel=28.0, y_rel=0.2, v_lead=0.0)
  assert t.potential_cruise_stationary_lead(14.0)
  assert t.potential_radar_only_lead(14.0)
  assert not t.potential_low_speed_lead(14.0)


def test_cruise_stationary_rejects_moving_or_offset():
  moving = _track(d_rel=28.0, y_rel=0.2, v_lead=8.0)
  offset = _track(d_rel=28.0, y_rel=2.0, v_lead=0.0)
  assert not moving.potential_cruise_stationary_lead(14.0)
  assert not offset.potential_cruise_stationary_lead(14.0)


def test_low_speed_path_unchanged():
  t = _track(d_rel=12.0, y_rel=0.2, v_lead=0.0)
  assert t.potential_low_speed_lead(2.0)
  assert not t.potential_cruise_stationary_lead(2.0)
  assert t.potential_radar_only_lead(2.0)
