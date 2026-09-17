from enum import Enum
from types import ModuleType
from types import SimpleNamespace as ns
import sys

from openpilot.cereal import custom


class FakeChestnutState(Enum):
  DISCONNECTED = "disconnected"
  UNCOMPILED = "uncompiled"
  READY = "ready"
  LOADING = "loading"
  ACTIVE = "active"
  FAILED = "failed"


fake_ui_state_module = ModuleType("openpilot.selfdrive.ui.ui_state")
fake_ui_state_module.ui_state = None
fake_ui_state_module.ChestnutState = FakeChestnutState
sys.modules["openpilot.selfdrive.ui.ui_state"] = fake_ui_state_module

from openpilot.selfdrive.ui.sunnypilot import model_info
from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION
from openpilot.sunnypilot.models.tests.test_selection import FakeParams


def _bundle(name: str):
  return custom.ModelManagerSP.ModelBundle(
    internalName=name,
    displayName=f"{name} display",
    ref=f"{name}-ref",
    minimumSelectorVersion=REQUIRED_JSON_VERSION,
  )


def _state(params, bundles, *, present: bool, active: bool | None, loading: bool, offroad: bool):
  return ns(
    params=params,
    sm={"modelManagerSP": ns(availableBundles=bundles)},
    chestnut_present=present,
    chestnut_active=active,
    chestnut_loading=loading,
    chestnut_state=FakeChestnutState.ACTIVE if active else FakeChestnutState.FAILED,
    is_offroad=lambda: offroad,
  )


def test_model_info_uses_chestnut_slot_while_egpu_is_active(monkeypatch):
  qcom, chestnut = _bundle("QCOM"), _bundle("BIG")
  params = FakeParams({
    "ModelManager_ActiveBundle": qcom.to_dict(),
    "ModelManager_ActiveBundleChestnut": chestnut.to_dict(),
  })
  monkeypatch.setattr(model_info, "ui_state", _state(params, [chestnut], present=True, active=True, loading=False, offroad=False))

  assert model_info.active_source() == "chestnut"
  assert model_info.model_info() == ("chestnut", "BIG display", "QCOM display")
  assert model_info.carrying_model() == ("chestnut", "BIG", "BIG display")


def test_custom_big_failure_does_not_claim_a_small_bundle_fallback(monkeypatch):
  chestnut = _bundle("BIG")
  params = FakeParams({"ModelManager_ActiveBundleChestnut": chestnut.to_dict()})
  monkeypatch.setattr(model_info, "ui_state", _state(params, [chestnut], present=True, active=False, loading=False, offroad=False))

  assert model_info.active_source() == "qcom"
  assert model_info.carrying_model() == (None, None, None)


def test_refresh_model_list_invalidates_both_hardware_catalogs(monkeypatch):
  params = FakeParams({
    "ModelManager_LastSyncTime": 123,
    "ModelManager_LastSyncTime_Chestnut": 456,
  })
  monkeypatch.setattr(model_info, "ui_state", ns(params=params))

  model_info.refresh_model_list()

  assert params.get("ModelManager_LastSyncTime") == 0
  assert params.get("ModelManager_LastSyncTime_Chestnut") == 0


def test_refresh_progress_completes_on_restamp_or_timeout(monkeypatch):
  params = FakeParams({
    "ModelManager_LastSyncTime": 0,
    "ModelManager_LastSyncTime_Chestnut": 0,
  })
  monkeypatch.setattr(model_info, "ui_state", ns(params=params))
  monkeypatch.setattr(model_info.time, "monotonic", lambda: 100.0)

  assert model_info.refresh_in_progress(90.0)
  params.put("ModelManager_LastSyncTime", 1)
  params.put("ModelManager_LastSyncTime_Chestnut", 2)
  assert not model_info.refresh_in_progress(90.0)
  assert not model_info.refresh_in_progress(70.0)
  assert not model_info.refresh_in_progress(None)
