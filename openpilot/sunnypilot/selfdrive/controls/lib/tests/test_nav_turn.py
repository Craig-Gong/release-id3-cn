from openpilot.common.constants import CV
from openpilot.sunnypilot.nav.snapshot import NavSnapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import (
  eval_nav_turn_desire,
  nav_intersection_turn,
  nav_led_approach,
  nav_long_blocked,
  snapshot_long_ok,
)


def _cs(v_kph, *, left=False, right=False, bsl=False, bsr=False):
  return dict(
    v_ego_mps=v_kph * CV.KPH_TO_MS,
    left_blinker=left,
    right_blinker=right,
    left_blindspot=bsl,
    right_blindspot=bsr,
  )


def test_nav_turn_blocked_far_and_fast():
  # Outside toast window, or still ≥45 km/h → no unblinkered desire
  assert eval_nav_turn_desire(direction="left", turn_dist_m=200.0, **_cs(50.0)) == "none"
  assert eval_nav_turn_desire(direction="left", turn_dist_m=120.0, **_cs(50.0)) == "none"


def test_nav_turn_near_and_slow():
  assert eval_nav_turn_desire(direction="left", turn_dist_m=50.0, **_cs(40.0)) == "left"


def test_nav_turn_no_desire_in_toast_only_window():
  # Toast / prep still ≤150 m, but unblinkered lateral desire only ≤120 m
  assert eval_nav_turn_desire(direction="right", turn_dist_m=140.0, **_cs(40.0)) == "none"
  assert eval_nav_turn_desire(direction="right", turn_dist_m=120.0, **_cs(40.0)) == "right"
  assert eval_nav_turn_desire(direction="right", turn_dist_m=80.0, **_cs(40.0)) == "right"


def test_nav_turn_keep_pulse_speeds_up_near_corner():
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import (
    nav_turn_keep_pulse_s, NAV_CORNER_PULSE_S, NAV_APPROACH_PULSE_S, NAV_DEFAULT_PULSE_S,
  )
  assert nav_turn_keep_pulse_s(30.0) == NAV_CORNER_PULSE_S
  assert nav_turn_keep_pulse_s(80.0) == NAV_APPROACH_PULSE_S
  assert nav_turn_keep_pulse_s(200.0) == NAV_DEFAULT_PULSE_S


def test_nav_turn_blinker_confirms_when_fast():
  assert eval_nav_turn_desire(direction="left", turn_dist_m=120.0, **_cs(55.0, left=True)) == "left"


def test_nav_turn_bsm_blocks():
  assert eval_nav_turn_desire(direction="left", turn_dist_m=50.0, **_cs(40.0, bsl=True)) == "none"


def test_fork_without_send_turn_is_not_intersection():
  snap = NavSnapshot(send_turn=False, maneuver="fork", maneuver_dir="left", tbt_dist=80.0)
  assert nav_intersection_turn(snap) is False
  assert nav_led_approach(snap) is False


def test_urban_lc_send_turn_is_intersection():
  snap = NavSnapshot(
    ts=1.0, link_ok=True, iqlink_enabled=True,
    send_turn=True, maneuver="fork", maneuver_dir="left", tbt_dist=100.0,
  )
  assert nav_intersection_turn(snap) is True
  assert nav_led_approach(snap, now=1.1) is True


def test_stale_link_does_not_nav_led():
  snap = NavSnapshot(
    ts=1.0, link_ok=False, iqlink_enabled=True,
    send_turn=True, maneuver="turn", maneuver_dir="right", tbt_dist=100.0,
  )
  assert nav_intersection_turn(snap) is True
  assert nav_led_approach(snap, now=1.1) is False


def test_urban_turn_led():
  snap = NavSnapshot(
    ts=1.0, link_ok=True, iqlink_enabled=True,
    send_turn=True, maneuver="turn", maneuver_dir="right", tbt_dist=100.0,
  )
  assert nav_intersection_turn(snap) is True
  assert nav_led_approach(snap, now=1.1) is True


def test_park_blocks_long():
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, stop_for_light=True)
  assert nav_long_blocked("park") is True
  assert snapshot_long_ok(snap, "park", now=10.1) is False
  assert snapshot_long_ok(snap, "drive", now=10.1) is True
