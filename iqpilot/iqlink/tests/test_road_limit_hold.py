from iqpilot.iqlink.road_limit_hold import IqlinkRoadLimitHold


def test_hold_rejects_flutter():
  h = IqlinkRoadLimitHold()
  t = 1000.0
  assert h.filter_kph(60.0, t) == 0.0
  assert h.filter_kph(60.0, t + 0.6) == 60.0
  assert h.filter_kph(80.0, t + 0.7) == 60.0
  assert h.filter_kph(80.0, t + 1.8) == 80.0


def test_decrease_adopts_faster():
  h = IqlinkRoadLimitHold()
  t = 2000.0
  h.filter_kph(80.0, t)
  h.filter_kph(80.0, t + 0.6)
  assert h.confirmed_kph == 80.0
  h.filter_kph(60.0, t + 0.7)
  assert h.filter_kph(60.0, t + 1.0) == 80.0
  assert h.filter_kph(60.0, t + 1.35) == 60.0


def test_large_jump_needs_longer_hold():
  h = IqlinkRoadLimitHold()
  t = 3000.0
  h.filter_kph(100.0, t)
  h.filter_kph(100.0, t + 0.6)
  h.filter_kph(50.0, t + 0.7)
  assert h.filter_kph(50.0, t + 2.0) == 100.0
  assert h.filter_kph(50.0, t + 2.25) == 50.0
