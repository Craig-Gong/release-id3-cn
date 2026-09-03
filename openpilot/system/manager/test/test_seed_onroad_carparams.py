from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.system.manager.helpers import seed_onroad_carparams


class TestSeedOnroadCarparams(OpenpilotTestCase):
  def test_seeds_from_persistent_when_live_missing(self):
    params = Params()
    params.put("CarParamsPersistent", b"cp")
    params.put("CarParamsSPPersistent", b"sp")
    params.remove("CarParams")
    params.remove("CarParamsSP")
    seed_onroad_carparams(params)
    assert params.get("CarParams") == b"cp"
    assert params.get("CarParamsSP") == b"sp"

  def test_does_not_overwrite_live(self):
    params = Params()
    params.put("CarParams", b"live")
    params.put("CarParamsPersistent", b"old")
    seed_onroad_carparams(params)
    assert params.get("CarParams") == b"live"
