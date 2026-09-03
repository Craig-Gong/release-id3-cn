from opendbc.car.common.conversions import Conversions as CV
from opendbc.sunnypilot.car.volkswagen.vze_speed_limit_hold import VZE_HOLD_S, VzeSpeedLimitHold


def test_accepts_stable_urban_limit():
  h = VzeSpeedLimitHold()
  assert h.update(50, 14.0, now=0.0) == 50  # ~50 kph ego


def test_holds_highway_false_urban():
  h = VzeSpeedLimitHold()
  h.update(120, 30.0, now=0.0)  # ~108 kph ego, accept 120
  held = h.update(40, 30.0, now=0.1)
  assert held == 120
  still = h.update(40, 30.0, now=VZE_HOLD_S)
  assert still == 120
  accepted = h.update(40, 30.0, now=VZE_HOLD_S + 0.1)
  assert accepted == 40


def test_holds_city_false_highway():
  h = VzeSpeedLimitHold()
  h.update(40, 10.0, now=0.0)
  assert h.update(120, 10.0, now=0.1) == 40


def test_jump_of_40_holds():
  h = VzeSpeedLimitHold()
  h.update(80, 22.0, now=0.0)
  assert h.update(40, 22.0, now=0.1) == 80
