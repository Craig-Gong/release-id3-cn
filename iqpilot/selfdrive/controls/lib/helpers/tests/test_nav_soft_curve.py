from openpilot.common.constants import CV
from openpilot.iqpilot.selfdrive.controls.lib.helpers.nav_decel import approach_speed_ms
from openpilot.iqpilot.selfdrive.controls.lib.helpers.nav_soft_curve import NavSoftCurveCap


class _Params:
  def get_bool(self, key: str) -> bool:
    return True

  def get(self, key: str):
    return None


def test_approach_speed_monotonic():
  v1 = approach_speed_ms(150.0, 1.2, floor_ms=40 * CV.KPH_TO_MS)
  v2 = approach_speed_ms(80.0, 1.2, floor_ms=40 * CV.KPH_TO_MS)
  assert v1 > v2


def test_soft_curve_highway_only():
  cap = NavSoftCurveCap(_Params())
  v = cap.update(
    iqlink_on=True,
    enabled=True,
    v_ego=25.0,
    posted_limit_ms=80 * CV.KPH_TO_MS,
    nav_send_turn=True,
    nav_phase=2,
    turn_dist_m=120.0,
    nav_send_lc=False,
  )
  assert v is not None and v < 25.0


def test_soft_curve_skips_slow_urban():
  cap = NavSoftCurveCap(_Params())
  v = cap.update(
    iqlink_on=True,
    enabled=True,
    v_ego=12.0,
    posted_limit_ms=50 * CV.KPH_TO_MS,
    nav_send_turn=True,
    nav_phase=2,
    turn_dist_m=80.0,
    nav_send_lc=False,
  )
  assert v is None
