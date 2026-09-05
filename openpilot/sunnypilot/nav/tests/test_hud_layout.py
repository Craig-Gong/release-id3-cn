#!/usr/bin/env python3
import unittest

from openpilot.sunnypilot.nav.hud_layout import (
  EGPU_HUD_GAP, EGPU_HUD_HEIGHT, EGPU_HUD_HEIGHT_COMPACT, LANE_GUIDE_GAP, LANE_GUIDE_HEIGHT,
  egpu_status_rect, junction_bar_rect, lane_guide_rect,
)


class TestEgpuHudLayout(unittest.TestCase):
  def test_egpu_strip_sits_under_junction_when_lane_guide_is_hidden(self):
    j = junction_bar_rect(0, 0, metric=True)
    e = egpu_status_rect(0, 0, metric=True, lane_guide=False)
    self.assertEqual(e.x, j.x)
    self.assertEqual(e.w, j.w)
    self.assertEqual(e.h, EGPU_HUD_HEIGHT)
    self.assertEqual(e.y, j.y + j.h + EGPU_HUD_GAP)

  def test_compact_height_when_no_metrics(self):
    e = egpu_status_rect(0, 0, metric=True, compact=True)
    self.assertEqual(e.h, EGPU_HUD_HEIGHT_COMPACT)
    self.assertLess(e.h, EGPU_HUD_HEIGHT)

  def test_egpu_pad_matches_right_chip_inset(self):
    from openpilot.sunnypilot.nav.hud_layout import CAPSULE_RIGHT_PAD, EGPU_PAD, EGPU_RAIL_W, EGPU_RAIL_X
    self.assertEqual(EGPU_PAD, CAPSULE_RIGHT_PAD)
    self.assertGreater(EGPU_RAIL_X + EGPU_RAIL_W + EGPU_PAD, CAPSULE_RIGHT_PAD)

  def test_dc_pill_grows_for_long_label(self):
    from openpilot.sunnypilot.nav.hud_layout import (
      EGPU_DC_PILL_GAP, EGPU_DC_PILL_MIN, EGPU_DC_PILL_PAD, dc_pill_width,
    )
    short = dc_pill_width(28.0, 26.0)
    long = dc_pill_width(28.0, 90.0)
    self.assertGreaterEqual(short, EGPU_DC_PILL_MIN)
    self.assertGreater(long, short)
    self.assertGreaterEqual(long - 90.0 - 28.0, EGPU_DC_PILL_GAP + EGPU_DC_PILL_PAD * 2)

  def test_egpu_strip_sits_under_lane_guide_when_present(self):
    lane = lane_guide_rect(0, 0, metric=True)
    e = egpu_status_rect(0, 0, metric=True, lane_guide=True)
    self.assertEqual(e.x, lane.x)
    self.assertEqual(e.w, lane.w)
    self.assertEqual(e.y, lane.y + lane.h + EGPU_HUD_GAP)
    j = junction_bar_rect(0, 0, metric=True)
    self.assertEqual(lane.y, j.y + j.h + LANE_GUIDE_GAP)
    self.assertEqual(lane.h, LANE_GUIDE_HEIGHT)


  def test_lane_and_egpu_type_matches_junction_chinese(self):
    from openpilot.sunnypilot.nav.hud_layout import (
      EGPU_DC_VALUE, EGPU_DETAIL_SIZE, EGPU_HEAD_SIZE, HUD_CN_DETAIL, HUD_CN_HEAD,
      LANE_TEXT_SIZE,
    )
    self.assertEqual(LANE_TEXT_SIZE, HUD_CN_HEAD)
    self.assertEqual(EGPU_HEAD_SIZE, HUD_CN_HEAD)
    self.assertEqual(EGPU_DC_VALUE, HUD_CN_DETAIL)
    self.assertEqual(EGPU_DETAIL_SIZE, HUD_CN_DETAIL)
    self.assertGreaterEqual(LANE_GUIDE_HEIGHT, 110)
    self.assertGreaterEqual(EGPU_HUD_HEIGHT, 350)
    self.assertGreaterEqual(EGPU_HUD_HEIGHT_COMPACT, 130)


if __name__ == "__main__":
  unittest.main()

