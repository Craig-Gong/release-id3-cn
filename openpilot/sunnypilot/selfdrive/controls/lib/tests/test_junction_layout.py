from openpilot.sunnypilot.nav.hud_layout import (
  SET_SPEED_H,
  junction_bar_rect,
  limit_chip_rect,
  max_chip_rect,
)


def test_metric_bar_aligns_max_left_and_limit_right():
  bar = junction_bar_rect(0, 0, metric=True)
  max_r = max_chip_rect(0, 0, metric=True)
  limit_r = limit_chip_rect(0, 0, metric=True)
  assert bar.x == max_r.x
  assert abs(bar.right - limit_r.right) < 0.01
  assert bar.w > 400
  assert bar.h >= 80
  assert bar.y >= max_r.y + SET_SPEED_H


def test_bar_is_below_chips():
  bar = junction_bar_rect(10, 20, metric=True)
  max_r = max_chip_rect(10, 20, metric=True)
  limit_r = limit_chip_rect(10, 20, metric=True)
  assert bar.y > max_r.y + max_r.h
  assert bar.y > limit_r.y + limit_r.h
