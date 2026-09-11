"""IQ-link ON: MAX follows nav road limit; gas sync may raise above until limit changes."""
from __future__ import annotations

import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

# cruise_ext pulls cereal/opendbc/params; stub for host unit tests without Cap'n Proto.
sys.modules.setdefault("capnp", MagicMock())
sys.modules.setdefault("zmq", MagicMock())
_cereal = MagicMock()
_cereal.custom = SimpleNamespace(
  LongitudinalPlanSP=SimpleNamespace(
    SpeedLimit=SimpleNamespace(AssistState=SimpleNamespace(disabled=0, active=1, adapting=2)),
  ),
)
sys.modules.setdefault("openpilot.cereal", _cereal)
sys.modules.setdefault("cereal", _cereal)
sys.modules.setdefault("opendbc.car.structs", MagicMock())
sys.modules.setdefault("opendbc.car", MagicMock())
sys.modules.setdefault("openpilot.common.swaglog", MagicMock())
_params_mod = MagicMock()
_params_mod.Params = MagicMock
_params_mod.UnknownKeyName = type("UnknownKeyName", (Exception,), {})
sys.modules.setdefault("openpilot.common.params", _params_mod)
sys.modules.setdefault(
  "openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers",
  MagicMock(get_minimum_set_speed=lambda is_metric: 8),
)
sys.modules.setdefault(
  "openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist",
  MagicMock(ACTIVE_STATES=(1, 2)),
)
sys.modules.setdefault(
  "openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers",
  MagicMock(compare_cluster_target=lambda *a, **k: (False, False)),
)

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
      # dt capped at 0.2 s → +1.6 km/h/step; ~13 steps from 30 → 50
      for _ in range(15):
        h._iqlink_max_t = __import__("time").monotonic() - 1.0
        h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 50.0


def test_iqlink_limit_unchanged_leaves_gas_raised_max():
  h = _helper()
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      for _ in range(15):
        h._iqlink_max_t = __import__("time").monotonic() - 1.0
        h.update_speed_limit_assist_v_cruise_non_pcm()
      h.v_cruise_kph = 58.0  # gas sync raised above limit
      h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 58.0


def test_iqlink_raises_max_when_below_unchanged_limit():
  h = _helper()
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      for _ in range(15):
        h._iqlink_max_t = __import__("time").monotonic() - 1.0
        h.update_speed_limit_assist_v_cruise_non_pcm()
      h.v_cruise_kph = 35.0  # manual SET below nav
      for _ in range(12):
        h._iqlink_max_t = __import__("time").monotonic() - 1.0
        h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 50.0


def test_iqlink_raise_is_slewed_not_instant():
  h = _helper()
  h.v_cruise_kph = 30.0
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=60.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      h._iqlink_max_t = __import__("time").monotonic() - 1.0
      h.update_speed_limit_assist_v_cruise_non_pcm()
  # dt capped at 0.2 → ~+1.6 km/h, not jump to 60
  assert 31.0 <= h.v_cruise_kph <= 34.0
  assert h.v_cruise_kph < 60.0


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
