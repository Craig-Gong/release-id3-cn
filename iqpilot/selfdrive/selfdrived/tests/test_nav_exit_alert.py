"""navExit* gated (arrive/reverse/dedup)."""
from iqpilot.selfdrive.selfdrived.nav_exit_alert import nav_exit_alert_allowed

# cereal ordinals (see nav_exit_alert.py): exit=2 arrive=6 turn=1; park=1 reverse=4
_EXIT, _ARRIVE, _TURN = 2, 6, 1
_PARK, _REVERSE = 1, 4


class Dummy:
  def __init__(self, **kw):
    self.__dict__.update(kw)


def _nav(**kw):
  base = dict(
    active=True,
    nextManeuverValid=True,
    nextManeuverType=_EXIT,
    nextManeuverDistance=300.0,
    nextManeuverDirection=1,
    distanceRemaining=5000.0,
  )
  base.update(kw)
  return Dummy(**base)


def _cs(**kw):
  return Dummy(**kw)


def test_allowed_first_exit():
  ok, d = nav_exit_alert_allowed(_nav(), _cs(), 100.0, -1e9, 0)
  assert ok and d == 1


def test_blocked_reverse():
  ok, d = nav_exit_alert_allowed(_nav(), _cs(gearShifter=_REVERSE), 100.0, -1e9, 0)
  assert not ok and d == 0


def test_blocked_park():
  ok, d = nav_exit_alert_allowed(_nav(), _cs(gearShifter=_PARK), 100.0, -1e9, 0)
  assert not ok and d == 0


def test_blocked_remain_62():
  ok, d = nav_exit_alert_allowed(_nav(distanceRemaining=62), _cs(), 100.0, -1e9, 0)
  assert not ok and d == 0


def test_remain_zero_treated_as_unset():
  ok, d = nav_exit_alert_allowed(_nav(distanceRemaining=0), _cs(), 100.0, -1e9, 0)
  assert ok and d == 1


def test_blocked_type_arrive():
  ok, d = nav_exit_alert_allowed(_nav(nextManeuverType=_ARRIVE), _cs(), 100.0, -1e9, 0)
  assert not ok and d == 0


def test_blocked_type_turn():
  ok, d = nav_exit_alert_allowed(_nav(nextManeuverType=_TURN), _cs(), 100.0, -1e9, 0)
  assert not ok and d == 0


def test_dedup_same_dir_within_8s():
  ok, d = nav_exit_alert_allowed(_nav(), _cs(), 100.0, -1e9, 0)
  assert ok and d == 1
  ok2, d2 = nav_exit_alert_allowed(_nav(), _cs(), 107.9, 100.0, 1)
  assert not ok2 and d2 == 0


def test_dedup_same_dir_after_8s():
  ok, d = nav_exit_alert_allowed(_nav(), _cs(), 108.0, 100.0, 1)
  assert ok and d == 1
