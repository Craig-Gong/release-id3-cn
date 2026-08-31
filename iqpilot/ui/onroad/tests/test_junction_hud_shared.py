from iqpilot.ui.onroad.junction_hud_shared import (
  GreenFlashState,
  JunctionHudSnapshot,
  GREEN_FLASH_S,
  junction_accent_rgb,
  merge_green_flash,
  read_junction_snapshot,
)


class _Plan:
  junctionStop = True


class _E2E:
  junctionStop = True


class _IQPlan:
  e2eAlerts = _E2E()


class _Nav:
  trafficLight = "red"
  trafficLightDistM = 42.0
  trafficLightRemainS = 8.0


class _SM:
  def __init__(self, junction=True, light="red"):
    self._junction = junction
    self._nav = _Nav()
    self._nav.trafficLight = light

  def __getitem__(self, key):
    if key == "iqPlan":
      plan = _IQPlan()
      plan.e2eAlerts.junctionStop = self._junction
      return plan
    if key == "iqNavState":
      return self._nav
    raise KeyError(key)


def test_snapshot_red():
  snap = read_junction_snapshot(_SM(), engaged=True)
  assert snap.active
  assert snap.headline == "红灯"
  assert "42" in snap.detail and "8" in snap.detail


def test_snapshot_plain_stop():
  snap = read_junction_snapshot(_SM(light="none"), engaged=True)
  assert snap.headline == "前方停车"


def test_inactive_when_disengaged():
  snap = read_junction_snapshot(_SM(), engaged=False)
  assert not snap.active


def test_accent_colors():
  assert junction_accent_rgb("red")[0] > 200
  assert junction_accent_rgb("yellow")[1] > 150
  assert junction_accent_rgb("green")[1] > 150


def test_green_headline():
  snap = JunctionHudSnapshot(True, "green", 0.0, 0.0)
  assert snap.headline == "绿灯"
  assert snap.detail == "可通行"


def test_green_flash_after_stop():
  state = GreenFlashState()
  stop = JunctionHudSnapshot(True, "red", 30.0, 5.0)
  idle = JunctionHudSnapshot(False, "none", 0.0, 0.0)
  t0 = 100.0

  merge_green_flash(stop, engaged=True, has_lead=False, light="red",
                    dist_m=30.0, remain_s=5.0, state=state, now=t0)
  snap = merge_green_flash(idle, engaged=True, has_lead=False, light="green",
                           dist_m=0.0, remain_s=0.0, state=state, now=t0 + 0.1)
  assert snap.active and snap.light == "green" and snap.headline == "绿灯"

  snap_late = merge_green_flash(idle, engaged=True, has_lead=False, light="green",
                                dist_m=0.0, remain_s=0.0, state=state, now=t0 + GREEN_FLASH_S + 0.1)
  assert not snap_late.active


def test_green_flash_skips_without_prior_stop():
  state = GreenFlashState()
  idle = JunctionHudSnapshot(False, "none", 0.0, 0.0)
  snap = merge_green_flash(idle, engaged=True, has_lead=False, light="green",
                           dist_m=0.0, remain_s=0.0, state=state, now=1.0)
  assert not snap.active


def test_green_flash_hidden_with_lead():
  state = GreenFlashState()
  stop = JunctionHudSnapshot(True, "red", 10.0, 3.0)
  idle = JunctionHudSnapshot(False, "none", 0.0, 0.0)
  merge_green_flash(stop, engaged=True, has_lead=False, light="red",
                    dist_m=10.0, remain_s=3.0, state=state, now=10.0)
  snap = merge_green_flash(idle, engaged=True, has_lead=True, light="green",
                           dist_m=0.0, remain_s=0.0, state=state, now=10.1)
  assert not snap.active
