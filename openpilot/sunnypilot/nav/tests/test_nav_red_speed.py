from openpilot.sunnypilot.nav.protocol import nav_red_speed_ms, nav_stop_margin_m


def test_margin_uses_slider_when_set():
  assert nav_stop_margin_m(8.5) == 8.5
  assert nav_stop_margin_m(10.0) == 10.0
  assert nav_stop_margin_m(0.0) == 3.0


def test_already_inside_margin_is_hard_stop():
  assert nav_red_speed_ms(8.0, 13.9, 10.0) == 0.0
  assert nav_red_speed_ms(0.0, 13.9, 3.0) == 0.0
  assert nav_red_speed_ms(3.0, 13.9, 3.0) == 0.0


def test_far_light_brakes_short_of_amap_distance():
  v_old = nav_red_speed_ms(20.0, 0.0, 3.0)
  v_new = nav_red_speed_ms(20.0, 0.0, 10.0)
  assert v_new < v_old
  assert v_new > 0.0
