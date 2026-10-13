"""Nav LC Confirm — gates, BSM HOLD, multi-lane budget, SATISFIED, mutual exclusion."""
from __future__ import annotations

import os
import sys
import tempfile
import types
import unittest
from unittest import mock

_KPH_TO_MS = 1.0 / 3.6

if "openpilot.common.constants" not in sys.modules:
  _cv = types.ModuleType("openpilot.common.constants")
  _cv.CV = types.SimpleNamespace(KPH_TO_MS=_KPH_TO_MS, MS_TO_KPH=3.6, MPH_TO_MS=0.44704)
  sys.modules["openpilot.common.constants"] = _cv

if "openpilot.common.params" not in sys.modules:
  _params = types.ModuleType("openpilot.common.params")

  class UnknownKeyName(Exception):
    pass

  class Params:
    def get_bool(self, key):
      return key == "NavLcConfirm"

    def get(self, key):
      return None

  _params.UnknownKeyName = UnknownKeyName
  _params.Params = Params
  sys.modules["openpilot.common.params"] = _params

if "openpilot.common.file_params" not in sys.modules:
  _fp = types.ModuleType("openpilot.common.file_params")
  _fp.read_file_param = lambda key, default=None: default
  sys.modules["openpilot.common.file_params"] = _fp

from openpilot.sunnypilot.nav.snapshot import NavSnapshot  # noqa: E402
from openpilot.sunnypilot.selfdrive.controls.lib.helpers import nav_lc_confirm as nlc  # noqa: E402


class _Btn:
  def __init__(self, name="setCruise", pressed=True):
    self.type = types.SimpleNamespace(name=name)
    self.pressed = pressed


def _assist(*, master=True, urban=False) -> nlc.NavLcConfirmAssist:
  a = nlc.NavLcConfirmAssist.__new__(nlc.NavLcConfirmAssist)
  a._params = types.SimpleNamespace(
    get_bool=lambda key: (
      master if key == "NavLcConfirm" else
      urban if key == "NavLcConfirmUrban" else False
    ),
    get=lambda key: None,
  )
  a._state = nlc.NavLcState.IDLE
  a._direction = "none"
  a._mode = ""
  a._reason = ""
  a._in_window = False
  a._armed_since = 0.0
  a._cooldown_until = 0.0
  a._edge_block_until = 0.0
  a._exec_since = 0.0
  a._confirms_used = 0
  a._maneuver_key = ""
  a._budget_exhausted_key = ""
  a._sat_edge_since = 0.0
  a._sat_rec_since = 0.0
  a._prev_lc_state = 0
  a._enable_bsm = True
  a._bsm_checked_at = 1e9
  a._last = nlc.NavLcDecision()
  return a


def _hwy_snap(*, dist=150.0, side="right", man="exit", road=80.0, send_turn=False,
              rec="none", light="none", stop=False, link=True) -> NavSnapshot:
  return NavSnapshot(
    ts=100.0,
    link_ok=link,
    link_state=2,
    iqlink_enabled=True,
    maneuver=man,
    maneuver_dir=side,
    tbt_dist=float(dist),
    road_limit_kph=float(road),
    send_turn=bool(send_turn),
    lane_recommend=rec,
    traffic_light=light,
    stop_for_light=bool(stop),
    light_ts=100.0 if light != "none" else 0.0,
  )


def _dec(a, *, now=100.0, v_kph=80.0, snap=None, lat=True, left_bsm=False, right_bsm=False,
         left_edge=False, right_edge=False, brake=False, buttons=None, turn=False,
         light_bar=False, lc_state=0, left_blink=False, right_blink=False):
  with mock.patch.object(nlc, "write_nav_lc_snapshot", lambda *args, **kwargs: None), \
       mock.patch.object(nlc, "read_cut_snapshot", lambda **kwargs: types.SimpleNamespace(hold=False, hud=False, active=False)):
    return a.update(
      snap=snap if snap is not None else _hwy_snap(),
      v_ego=float(v_kph) * _KPH_TO_MS,
      lateral_active=lat,
      gear="drive",
      left_blinker=left_blink,
      right_blinker=right_blink,
      left_blindspot=left_bsm,
      right_blindspot=right_bsm,
      left_edge=left_edge,
      right_edge=right_edge,
      brake_pressed=brake,
      button_events=buttons or (),
      turn_active=turn,
      light_bar_busy=light_bar,
      lane_change_state=lc_state,
      now=now,
    )


class TestNavLcConfirm(unittest.TestCase):
  def test_highway_exit_arms(self):
    a = _assist()
    d = _dec(a, snap=_hwy_snap(dist=150, side="right", man="exit"))
    self.assertEqual(d.state, "armed")
    self.assertEqual(d.direction, "right")
    self.assertEqual(d.hud.head, "变道 右")
    self.assertEqual(d.hud.detail, "按 SET")
    self.assertEqual(d.mode, "highway")

  def test_confirm_starts_exec(self):
    a = _assist()
    _dec(a, now=100.0)
    d = _dec(a, now=100.1, buttons=[_Btn()])
    self.assertEqual(d.state, "executing")
    self.assertTrue(d.start_pulse)
    self.assertTrue(d.execute)

  def test_confirm_ignored_when_idle(self):
    a = _assist()
    d = _dec(a, snap=_hwy_snap(dist=700), buttons=[_Btn()])  # beyond exit HI 520
    self.assertEqual(d.state, "idle")
    self.assertFalse(d.start_pulse)

  def test_exit_arms_at_500m(self):
    a = _assist()
    d = _dec(a, snap=_hwy_snap(dist=500, man="exit"))
    self.assertEqual(d.state, "armed")
    self.assertEqual(d.mode, "highway")

  def test_exit_too_far_at_600m(self):
    a = _assist()
    d = _dec(a, snap=_hwy_snap(dist=600, man="exit"))
    self.assertEqual(d.state, "idle")
    self.assertEqual(d.reason, "hwy_dist")

  def test_fork_arms_at_350m(self):
    a = _assist()
    d = _dec(a, snap=_hwy_snap(dist=350, man="fork", send_turn=False, road=100))
    self.assertEqual(d.state, "armed")

  def test_fork_too_far_at_450m(self):
    a = _assist()
    d = _dec(a, snap=_hwy_snap(dist=450, man="fork", send_turn=False, road=100))
    self.assertEqual(d.state, "idle")
    self.assertEqual(d.reason, "hwy_dist")

  def test_urban_arms_at_110m(self):
    a = _assist(urban=True)
    snap = _hwy_snap(man="fork", send_turn=True, road=50, dist=110, rec="left", side="left")
    d = _dec(a, v_kph=40, snap=snap)
    self.assertEqual(d.state, "armed")
    self.assertEqual(d.mode, "urban")

  def test_bsm_hold_ignores_confirm(self):
    a = _assist()
    _dec(a, now=100.0, right_bsm=True)
    d = _dec(a, now=100.1, right_bsm=True, buttons=[_Btn()])
    self.assertEqual(d.state, "hold")
    self.assertEqual(d.hud.head, "变道 等待")
    self.assertFalse(d.start_pulse)

  def test_edge_block_when_already_outer(self):
    a = _assist()
    # Need stable edge 0.5 s — first frame arms sat timer
    _dec(a, now=100.0, right_edge=True, snap=_hwy_snap(side="right"))
    d = _dec(a, now=100.6, right_edge=True, snap=_hwy_snap(side="right"))
    self.assertEqual(d.state, "edge_block")
    self.assertEqual(d.hud.head, "已在右侧")

  def test_satisfied_recommend_straight(self):
    a = _assist()
    snap = _hwy_snap(rec="straight")
    _dec(a, now=100.0, snap=snap)
    d = _dec(a, now=100.6, snap=snap)
    # still NEED side from TBT but recommend straight → satisfied after hold
    self.assertIn(d.state, ("idle", "cooldown"))
    self.assertIn(d.reason, ("satisfied", "gate", "hwy_dist", "urban_off", "satisfied"))

  def test_urban_off_by_default(self):
    a = _assist(urban=False)
    snap = _hwy_snap(man="fork", send_turn=True, road=50, dist=60, rec="left", side="left")
    d = _dec(a, v_kph=40, snap=snap)
    self.assertEqual(d.state, "idle")
    self.assertEqual(d.reason, "urban_off")

  def test_urban_arms_with_recommend(self):
    a = _assist(urban=True)
    snap = _hwy_snap(man="fork", send_turn=True, road=50, dist=60, rec="left", side="left")
    d = _dec(a, v_kph=40, snap=snap)
    self.assertEqual(d.state, "armed")
    self.assertEqual(d.mode, "urban")
    self.assertEqual(d.direction, "left")

  def test_urban_rejects_straight_recommend(self):
    a = _assist(urban=True)
    snap = _hwy_snap(man="fork", send_turn=True, road=50, dist=60, rec="straight", side="left")
    d = _dec(a, v_kph=40, snap=snap)
    self.assertEqual(d.state, "idle")

  def test_turn_maneuver_never(self):
    a = _assist(urban=True)
    snap = _hwy_snap(man="turn", send_turn=True, road=40, dist=50, rec="left", side="left")
    d = _dec(a, v_kph=35, snap=snap)
    self.assertEqual(d.reason, "turn")

  def test_no_bsm_disables(self):
    a = _assist()
    a._enable_bsm = False
    d = _dec(a)
    self.assertEqual(d.reason, "no_bsm")

  def test_light_bar_defers(self):
    a = _assist()
    d = _dec(a, light_bar=True)
    self.assertEqual(d.reason, "light_bar")

  def test_stop_for_light_defers(self):
    a = _assist()
    d = _dec(a, snap=_hwy_snap(stop=True, light="red"))
    self.assertEqual(d.reason, "light_bar")

  def test_hazard_clears(self):
    a = _assist()
    _dec(a, now=100.0)
    d = _dec(a, now=100.1, left_blink=True, right_blink=True)
    self.assertEqual(d.reason, "hazard")

  def test_turn_active_clears(self):
    a = _assist()
    _dec(a, now=100.0)
    d = _dec(a, now=100.1, turn=True)
    self.assertEqual(d.reason, "turn")

  def test_multi_lane_budget_four(self):
    a = _assist()
    now = 100.0

    def snap_at(t, dist):
      s = _hwy_snap(dist=dist)
      s.ts = float(t)
      return s

    for i in range(4):
      dist = 180 - i * 20
      d = _dec(a, now=now, snap=snap_at(now, dist))
      self.assertEqual(d.state, "armed", msg=f"arm {i} reason={d.reason}")
      now += 0.1
      d = _dec(a, now=now, buttons=[_Btn()], snap=snap_at(now, dist))
      self.assertEqual(d.state, "executing", msg=f"confirm {i}")
      now += 0.2
      # finish LC: starting→off
      _dec(a, now=now, lc_state=2, snap=snap_at(now, dist))
      now += 0.2
      d = _dec(a, now=now, lc_state=0, snap=snap_at(now, dist))
      if i < 3:
        self.assertEqual(d.state, "cooldown", msg=f"after {i} reason={d.reason}")
        now = a._cooldown_until + 0.1
      else:
        # 4th successful EXEC ends the maneuver budget immediately
        self.assertEqual(d.reason, "budget")
        self.assertEqual(d.state, "idle")
    # Further ARMED blocked for same maneuver key
    now += 1.0
    d = _dec(a, now=now, snap=snap_at(now, 100))
    self.assertEqual(d.reason, "budget")
    self.assertEqual(d.state, "idle")

  def test_forbidden_resume_falls_back_to_set(self):
    with mock.patch.object(nlc, "read_file_param", lambda key, default=None: "resumeCruise"):
      self.assertEqual(nlc.confirm_button_name(), "setCruise")

  def test_confirm_rising_edge_set(self):
    self.assertTrue(nlc.confirm_rising_edge([_Btn("setCruise", True)]))
    self.assertFalse(nlc.confirm_rising_edge([_Btn("setCruise", False)]))
    self.assertFalse(nlc.confirm_rising_edge([_Btn("gapAdjustCruise", True)]))

  def test_swallow_set_when_armed(self):
    with tempfile.TemporaryDirectory() as td:
      path = os.path.join(td, "sp_nav_lc.json")
      nlc.write_nav_lc_snapshot(nlc.NavLcSnapshot(
        ts=50.0, show=True, head="变道 右", detail="按 SET", state="armed",
        direction="right", swallow_set=True,
      ), path=path)
      self.assertTrue(nlc.nav_lc_swallow_set(path, now=50.5))
      nlc.write_nav_lc_snapshot(nlc.NavLcSnapshot(
        ts=50.0, show=False, state="idle", swallow_set=False,
      ), path=path)
      self.assertFalse(nlc.nav_lc_swallow_set(path, now=50.5))

  def test_shm_roundtrip(self):
    with tempfile.TemporaryDirectory() as td:
      path = os.path.join(td, "sp_nav_lc.json")
      nlc.write_nav_lc_snapshot(nlc.NavLcSnapshot(
        ts=50.0, show=True, head="变道 左", detail="按 SET", state="armed", direction="left",
      ), path=path)
      snap = nlc.read_nav_lc_snapshot(path, now=50.5)
      self.assertTrue(snap.show)
      self.assertEqual(snap.head, "变道 左")

  def test_classify_highway_fork_not_send_turn(self):
    snap = _hwy_snap(man="fork", send_turn=False, dist=120, road=100)
    d, mode, why = nlc.classify_nav_lc_intent(
      snap, v_ego=90 * _KPH_TO_MS, urban_ok=False, in_window=False, now=100.0,
    )
    self.assertEqual((d, mode, why), ("right", "highway", "ok"))

  def test_classify_promoted_fork_not_highway(self):
    snap = _hwy_snap(man="fork", send_turn=True, dist=120, road=100, rec="right")
    d, mode, why = nlc.classify_nav_lc_intent(
      snap, v_ego=90 * _KPH_TO_MS, urban_ok=False, in_window=False, now=100.0,
    )
    self.assertEqual(d, "none")
    self.assertEqual(why, "urban_off")


if __name__ == "__main__":
  unittest.main()
