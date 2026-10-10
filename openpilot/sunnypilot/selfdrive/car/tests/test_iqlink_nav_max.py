"""IQ-link ON: MAX follows nav road limit; gas sync may raise above until limit changes."""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import numpy as np

from openpilot.sunnypilot.nav.snapshot import NavSnapshot
from openpilot.sunnypilot.selfdrive.car.cruise_ext import VCruiseHelperSP, V_CRUISE_MIN


def _helper() -> VCruiseHelperSP:
  CP = MagicMock()
  CP_SP = MagicMock()
  CP_SP.pcmCruiseSpeed = False
  h = VCruiseHelperSP(CP, CP_SP)
  h.v_cruise_min = V_CRUISE_MIN
  h.v_cruise_kph = 30.0
  h.params = MagicMock()
  h.params.get_bool = MagicMock(return_value=True)
  return h


def test_iqlink_raises_max_to_nav_limit():
  h = _helper()
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 50.0


def test_iqlink_limit_unchanged_leaves_gas_raised_max():
  h = _helper()
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      h.update_speed_limit_assist_v_cruise_non_pcm()
      h.v_cruise_kph = 58.0  # gas sync raised above limit
      h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 58.0


def test_iqlink_limit_drop_lowers_max():
  h = _helper()
  snap50 = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  snap40 = NavSnapshot(ts=11.0, link_ok=True, iqlink_enabled=True, road_limit_kph=40.0)
  with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
    with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap50):
      h.update_speed_limit_assist_v_cruise_non_pcm()
    with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap40):
      h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 40.0


def test_stale_link_does_not_apply_nav_max():
  h = _helper()
  h.v_cruise_kph = 35.0
  snap = NavSnapshot(ts=10.0, link_ok=False, iqlink_enabled=True, road_limit_kph=50.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=False):
      applied = h._apply_iqlink_nav_to_max()
  assert applied is False
  assert h.v_cruise_kph == 35.0
