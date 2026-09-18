from openpilot.common.constants import CV
from openpilot.sunnypilot.nav.snapshot import NavSnapshot
from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import (
  eval_nav_turn_desire,
  nav_intersection_turn,
  nav_led_approach,
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
  assert eval_nav_turn_desire(direction="left", turn_dist_m=200.0, **_cs(50.0)) == "none"
  assert eval_nav_turn_desire(direction="left", turn_dist_m=120.0, **_cs(50.0)) == "none"


def test_nav_turn_near_and_slow():
  assert eval_nav_turn_desire(direction="left", turn_dist_m=50.0, **_cs(40.0)) == "left"
  assert eval_nav_turn_desire(direction="left", turn_dist_m=30.0, **_cs(40.0)) == "left"
  assert eval_nav_turn_desire(direction="left", turn_dist_m=60.0, **_cs(40.0)) == "none"


def test_nav_turn_no_desire_in_toast_only_window():
  # Toast / prep still ≤150 m; lateral desire only ≤50 m (CTM v2).
  assert eval_nav_turn_desire(direction="right", turn_dist_m=140.0, **_cs(40.0)) == "none"
  assert eval_nav_turn_desire(direction="right", turn_dist_m=80.0, **_cs(40.0)) == "none"
  assert eval_nav_turn_desire(direction="right", turn_dist_m=60.0, **_cs(40.0)) == "none"
  assert eval_nav_turn_desire(direction="right", turn_dist_m=50.0, **_cs(40.0)) == "right"


def test_nav_turn_keep_pulse_speeds_up_near_corner():
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import (
    nav_turn_keep_pulse_s, NAV_CORNER_PULSE_S, NAV_DEFAULT_PULSE_S,
  )
  assert nav_turn_keep_pulse_s(30.0) == NAV_CORNER_PULSE_S
  assert nav_turn_keep_pulse_s(50.0) == NAV_CORNER_PULSE_S
  assert nav_turn_keep_pulse_s(80.0) == NAV_DEFAULT_PULSE_S
  assert nav_turn_keep_pulse_s(200.0) == NAV_DEFAULT_PULSE_S


def test_nav_turn_commit_hold_near_corner():
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import nav_turn_commit_hold
  assert nav_turn_commit_hold(turn_dist_m=30.0, committed=False) is True
  assert nav_turn_commit_hold(turn_dist_m=50.0, committed=False) is True
  assert nav_turn_commit_hold(turn_dist_m=80.0, committed=False) is False
  assert nav_turn_commit_hold(turn_dist_m=80.0, committed=True) is True
  assert nav_turn_commit_hold(turn_dist_m=0.0, committed=False) is False


def test_nav_blinker_matches_turn():
  from openpilot.sunnypilot.selfdrive.controls.lib.helpers.nav_turn import nav_blinker_matches_turn
  snap = NavSnapshot(send_turn=True, maneuver="turn", maneuver_dir="left", tbt_dist=100.0)
  assert nav_blinker_matches_turn(snap, left_blinker=True, right_blinker=False) is True
  assert nav_blinker_matches_turn(snap, left_blinker=False, right_blinker=True) is False
  snap2 = NavSnapshot(send_turn=False, maneuver="fork", maneuver_dir="left", tbt_dist=80.0)
  assert nav_blinker_matches_turn(snap2, left_blinker=True, right_blinker=False) is False


def test_nav_turn_blinker_does_not_widen_window():
  # Early / fast stalk must not inject turn desire (stay LC or wait for ≤50/<45).
  assert eval_nav_turn_desire(direction="left", turn_dist_m=120.0, **_cs(55.0, left=True)) == "none"
  assert eval_nav_turn_desire(direction="left", turn_dist_m=60.0, **_cs(40.0, left=True)) == "none"
  assert eval_nav_turn_desire(direction="left", turn_dist_m=50.0, **_cs(55.0, left=True)) == "none"
  assert eval_nav_turn_desire(direction="left", turn_dist_m=50.0, **_cs(40.0, left=True)) == "left"


def test_nav_turn_bsm_blocks():
  assert eval_nav_turn_desire(direction="left", turn_dist_m=50.0, **_cs(40.0, bsl=True)) == "none"


def test_fork_without_send_turn_is_not_intersection():
  snap = NavSnapshot(send_turn=False, maneuver="fork", maneuver_dir="left", tbt_dist=80.0)
  assert nav_intersection_turn(snap) is False


def test_urban_lc_send_turn_is_intersection():
  snap = NavSnapshot(
    send_turn=True, maneuver="fork", maneuver_dir="left", tbt_dist=100.0,
  )
  assert nav_intersection_turn(snap) is True


def test_nav_led_approach_uses_toast_window():
  import time
  now = time.monotonic()
  snap = NavSnapshot(
    ts=now, link_ok=True, iqlink_enabled=True,
    send_turn=True, maneuver="turn", maneuver_dir="right", tbt_dist=100.0,
  )
  assert nav_led_approach(snap, now=now) is True
  far = NavSnapshot(
    ts=now, link_ok=True, iqlink_enabled=True,
    send_turn=True, maneuver="turn", maneuver_dir="right", tbt_dist=200.0,
  )
  assert nav_led_approach(far, now=now) is False


def test_nav_led_needs_executable():
  import time
  now = time.monotonic()
  snap = NavSnapshot(
    ts=now, link_ok=False, iqlink_enabled=True,
    send_turn=True, maneuver="turn", maneuver_dir="right", tbt_dist=100.0,
  )
  assert nav_led_approach(snap, now=now) is False
