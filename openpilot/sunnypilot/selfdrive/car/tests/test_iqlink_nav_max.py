"""IQ-link ON: MAX follows nav road limit; gas sync may raise above until limit drops."""
from __future__ import annotations

import sys
import time
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


def _slew_step(h: VCruiseHelperSP) -> None:
  """Advance one nav-MAX slew frame with a capped 0.2 s dt (host clocks vary)."""
  now = 1000.0
  h._iqlink_max_t = now - 1.0
  with patch("openpilot.sunnypilot.selfdrive.car.cruise_ext.time.monotonic", return_value=now):
    h.update_speed_limit_assist_v_cruise_non_pcm()


def test_iqlink_raises_max_to_nav_limit():
  h = _helper()
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      # dt capped at 0.2 s → +1.6 km/h/step; ~13 steps from 30 → 50
      for _ in range(15):
        _slew_step(h)
  assert h.v_cruise_kph == 50.0


def test_iqlink_limit_unchanged_leaves_gas_raised_max():
  h = _helper()
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      for _ in range(15):
        _slew_step(h)
      h.v_cruise_kph = 58.0  # gas sync raised above limit
      h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 58.0


def test_iqlink_raises_max_when_below_unchanged_limit():
  h = _helper()
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      for _ in range(15):
        _slew_step(h)
      h.v_cruise_kph = 35.0  # manual SET below nav
      for _ in range(12):
        _slew_step(h)
  assert h.v_cruise_kph == 50.0


def test_iqlink_raise_is_slewed_not_instant():
  h = _helper()
  h.v_cruise_kph = 30.0
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=60.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      _slew_step(h)
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


def test_iqlink_limit_raise_leaves_gas_raised_max():
  """Posted limit going 50→60 must not yank a gas-raised 70 back down."""
  h = _helper()
  snap50 = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  snap60 = NavSnapshot(ts=11.0, link_ok=True, iqlink_enabled=True, road_limit_kph=60.0)
  with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
    with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap50):
      for _ in range(15):
        _slew_step(h)
      h.v_cruise_kph = 70.0
    with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap60):
      h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 70.0


def test_iqlink_brief_gap_keeps_gas_raised_max():
  """Executable blip must not clear prev or hand MAX to SLA."""
  h = _helper()
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      for _ in range(15):
        _slew_step(h)
      h.v_cruise_kph = 58.0
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=False):
      assert h._apply_iqlink_nav_to_max() is True
      assert h.v_cruise_kph == 58.0
      assert h.prev_iqlink_road_limit_kph == 50.0
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 58.0


def test_iqlink_manual_override_blocks_auto_raise():
  h = _helper()
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      for _ in range(15):
        _slew_step(h)
      h.v_cruise_kph = 35.0
      h.mark_iqlink_cruise_override()
      for _ in range(12):
        _slew_step(h)
  assert h.v_cruise_kph == 35.0


def test_iqlink_limit_drop_clears_override():
  h = _helper()
  snap50 = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  snap40 = NavSnapshot(ts=11.0, link_ok=True, iqlink_enabled=True, road_limit_kph=40.0)
  with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
    with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap50):
      for _ in range(15):
        _slew_step(h)
      h.v_cruise_kph = 35.0
      h.mark_iqlink_cruise_override()
    with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap40):
      h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 40.0
  assert h._iqlink_max_override is False


def test_iqlink_long_gap_releases_ownership():
  h = _helper()
  from openpilot.sunnypilot.selfdrive.car.cruise_ext import IQLINK_GAP_CLEAR_S
  snap = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
      h.update_speed_limit_assist_v_cruise_non_pcm()
      h.v_cruise_kph = 58.0
      h.mark_iqlink_cruise_override()
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=False):
      assert h._apply_iqlink_nav_to_max() is True
      now = 1000.0
      h._iqlink_none_since = now - (IQLINK_GAP_CLEAR_S + 1.0)
      with patch("openpilot.sunnypilot.selfdrive.car.cruise_ext.time.monotonic", return_value=now):
        assert h._apply_iqlink_nav_to_max() is False
  assert h.prev_iqlink_road_limit_kph < 0.0
  # Gas-raised MAX / override must survive handing ownership back to SLA.
  assert h.v_cruise_kph == 58.0
  assert h._iqlink_max_override is True


def test_sla_does_not_yank_gas_raised_max():
  """Assist active + limit unchanged must not pull MAX back after Gas Sync."""
  h = _helper()
  h.sla_state = 1  # active
  h.prev_sla_state = 1
  h.has_speed_limit = True
  h.speed_limit_final_last_kph = 50.0
  h.prev_speed_limit_final_last_kph = 50.0
  h.v_cruise_kph = 50.0
  h.params.get_bool = MagicMock(return_value=False)  # IQ-link off → SLA path
  # Enter Assist: adopt limit
  h.prev_sla_state = 0
  h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 50.0
  # Gas Sync raise + override latch
  h.v_cruise_kph = 62.0
  h.mark_iqlink_cruise_override()
  h.speed_limit_final_last_kph = 50.0001  # float flutter → "changed"
  h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 62.0
  assert h._iqlink_max_override is True


def test_sla_limit_drop_clears_gas_override():
  h = _helper()
  h.sla_state = 1
  h.prev_sla_state = 1
  h.has_speed_limit = True
  h.speed_limit_final_last_kph = 50.0
  h.prev_speed_limit_final_last_kph = 50.0
  h.v_cruise_kph = 62.0
  h.mark_iqlink_cruise_override()
  h.params.get_bool = MagicMock(return_value=False)
  h.speed_limit_final_last_kph = 40.0
  h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 40.0
  assert h._iqlink_max_override is False


def test_iqlink_tiny_limit_flutter_not_a_drop():
  """Sub-1 km/h Gaode flutter must not clear a gas-raised MAX."""
  h = _helper()
  snap50 = NavSnapshot(ts=10.0, link_ok=True, iqlink_enabled=True, road_limit_kph=50.0)
  snap49 = NavSnapshot(ts=11.0, link_ok=True, iqlink_enabled=True, road_limit_kph=49.6)
  with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=True):
    with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap50):
      for _ in range(15):
        _slew_step(h)
      h.v_cruise_kph = 58.0
      h.mark_iqlink_cruise_override()
    with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap49):
      h.update_speed_limit_assist_v_cruise_non_pcm()
  assert h.v_cruise_kph == 58.0
  assert h._iqlink_max_override is True


def test_stale_link_does_not_apply_nav_max():
  h = _helper()
  h.v_cruise_kph = 35.0
  # Never established a prev limit → do not claim ownership on a dead link.
  snap = NavSnapshot(ts=10.0, link_ok=False, iqlink_enabled=True, road_limit_kph=50.0)
  with patch("openpilot.sunnypilot.nav.snapshot.read_snapshot", return_value=snap):
    with patch("openpilot.sunnypilot.nav.snapshot.snapshot_executable", return_value=False):
      applied = h._apply_iqlink_nav_to_max()
  assert applied is False
  assert h.v_cruise_kph == 35.0
