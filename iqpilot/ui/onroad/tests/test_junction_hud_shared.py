from openpilot.iqpilot.ui.onroad.junction_hud_shared import (
  JunctionHudSnapshot,
  junction_accent_rgb,
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
