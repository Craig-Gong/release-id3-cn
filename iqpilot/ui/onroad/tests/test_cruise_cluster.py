from iqpilot.ui.onroad.cruise_cluster import (
  CARD_GAP,
  CLUSTER_GAP,
  GUIDE_CARD_H,
  JUNC_CARD_H,
  SET_SPEED_H,
  SET_SPEED_W_IMPERIAL,
  SET_SPEED_W_METRIC,
  cluster_box,
  stack_layout,
)


def test_cluster_matches_official_max_box_metric():
  x, y, w, h = cluster_box(0, 0, metric=True)
  assert x == 60 + (SET_SPEED_W_IMPERIAL - SET_SPEED_W_METRIC) // 2
  assert y == 45
  assert w == SET_SPEED_W_METRIC
  assert h == SET_SPEED_H


def test_stack_junction_then_guide():
  jy, gy = stack_layout(100, junction=True, guide=True)
  assert jy == 100 + CLUSTER_GAP
  assert gy == jy + JUNC_CARD_H + CARD_GAP


def test_stack_guide_only_uses_same_slot():
  jy, gy = stack_layout(100, junction=False, guide=True)
  assert jy is None
  assert gy == 100 + CLUSTER_GAP
  assert GUIDE_CARD_H > 0
