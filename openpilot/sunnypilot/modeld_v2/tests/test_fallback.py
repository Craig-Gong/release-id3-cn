import numpy as np
import pytest

from openpilot.sunnypilot.modeld_v2 import modeld as modeld_module
from openpilot.sunnypilot.models import manager as manager_module
from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION


class FakeParams:
  def __init__(self):
    self.values = {}
    self.blocking_bool_writes = []

  def put_bool(self, key, value, block=False):
    self.values[key] = bool(value)
    if block:
      self.blocking_bool_writes.append((key, bool(value)))

  def put(self, key, value, block=False):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key, False))


def test_initial_big_model_failure_falls_back_to_small():
  params = FakeParams()
  small_model = object()

  def load_big():
    raise RuntimeError("USB AMD unavailable")

  model, fallback, keep_loading = modeld_module.load_models_with_fallback(
    chestnut=True,
    load_big=load_big,
    load_small=lambda: small_model,
    params=params,
    update_loading_progress=lambda _progress: None,
  )

  assert model is small_model
  assert fallback is small_model
  assert keep_loading is False
  assert params.values["ChestnutActive"] is False
  assert params.values["ChestnutLoading"] is False
  assert params.values["ChestnutModelError"] is True


def test_skip_drive_holds_loading_while_qcom_reloads():
  params = FakeParams()
  small_model = object()
  loading_during_small = []

  def load_small():
    loading_during_small.append(params.values.get("ChestnutLoading"))
    return small_model

  model, fallback, keep_loading = modeld_module.load_models_with_fallback(
    chestnut=False,
    load_big=lambda: (_ for _ in ()).throw(AssertionError("must not load USB")),
    load_small=load_small,
    params=params,
    update_loading_progress=lambda _progress: None,
    hold_loading=True,
  )

  assert model is small_model
  assert fallback is small_model
  assert keep_loading is True
  assert loading_during_small != [False]
  assert params.values.get("ChestnutLoading") is True


def test_successful_big_model_keeps_preloaded_small_for_runtime_fallback(monkeypatch):
  params = FakeParams()
  big_model = object()
  small_model = object()
  calls = {"big": 0, "small": 0}
  monkeypatch.setattr(modeld_module, "load_with_timeout", lambda load, timeout: load())

  def load_big():
    calls["big"] += 1
    return big_model

  def load_small():
    calls["small"] += 1
    return small_model

  model, fallback, keep_loading = modeld_module.load_models_with_fallback(
    chestnut=True,
    load_big=load_big,
    load_small=load_small,
    params=params,
    update_loading_progress=lambda _progress: None,
  )

  assert model is big_model
  assert fallback is small_model
  assert keep_loading is True
  assert calls == {"big": 1, "small": 1}
  assert params.values["ChestnutActive"] is True
  assert "ChestnutLoading" not in params.values
  assert "ChestnutModelError" not in params.values


def test_successful_big_model_survives_missing_small_fallback(monkeypatch):
  params = FakeParams()
  big_model = object()
  monkeypatch.setattr(modeld_module, "load_with_timeout", lambda load, timeout: load())

  def load_small():
    raise AssertionError("No driving pkl found — qcom slot empty")

  model, fallback, keep_loading = modeld_module.load_models_with_fallback(
    chestnut=True,
    load_big=lambda: big_model,
    load_small=load_small,
    params=params,
    update_loading_progress=lambda _progress: None,
  )

  assert model is big_model
  assert fallback is None
  assert keep_loading is True
  assert params.values["ChestnutActive"] is True
  assert "ChestnutLoading" not in params.values


def test_runtime_big_model_failure_switches_to_preloaded_small():
  params = FakeParams()
  params.values["ChestnutActive"] = True
  small_model = object()
  chestnut_state = type("ChestnutState", (), {"big": True})()

  class FailingBigModel:
    def run(self, *_args, **_kwargs):
      raise RuntimeError("non-finite model output")

  active, output, fell_back = modeld_module.run_model_with_fallback(
    FailingBigModel(), small_model, params, chestnut_state, (), {}, {},
  )

  assert active is small_model
  assert output is None
  assert fell_back
  assert params.values["ChestnutActive"] is False
  assert params.values["ChestnutModelError"] is True
  assert chestnut_state.big is False
  assert ("ChestnutActive", False) in params.blocking_bool_writes


def test_runtime_big_model_failure_without_small_fallback_is_explicit():
  params = FakeParams()
  params.values["ChestnutActive"] = True

  class FailingBigModel:
    def run(self, *_args, **_kwargs):
      raise RuntimeError("USB stream stopped")

  with pytest.raises(RuntimeError, match="small fallback unavailable"):
    modeld_module.run_model_with_fallback(
      FailingBigModel(), None, params, None, (), {}, {},
    )

  assert params.values["ChestnutActive"] is False
  assert params.values["ChestnutModelError"] is True


def test_non_finite_big_model_plan_becomes_fallback_error():
  outputs = {"plan": np.array([np.nan])}

  with pytest.raises(RuntimeError, match="not finite"):
    modeld_module.validate_model_outputs(chestnut=True, outputs=outputs)


def test_runtime_forwards_enqueue_callback_without_losing_fallback():
  params = FakeParams()
  calls = []

  class Model:
    def run(self, *args, after_enqueue=None):
      after_enqueue()
      return {"plan": np.array([1.0])}

  model = Model()
  active, output, fell_back = modeld_module.run_model_with_fallback(
    model, None, params, None, (), {}, {}, after_enqueue=lambda: calls.append("telemetry"),
  )
  assert active is model
  assert not fell_back
  assert calls == ["telemetry"]
  assert output["plan"][0] == 1.0


def test_missing_qcom_selection_queues_exact_default_fallback_ref():
  params = FakeParams()
  params.values["ModelManager_ActiveBundleChestnut"] = {
    "internalName": "BMV4",
    "minimumSelectorVersion": REQUIRED_JSON_VERSION,
  }

  manager_module.ensure_default_qcom_fallback(params)

  assert params.values["ModelManager_DownloadRef"] == "5b6436a90cf6902b8aaa71c2b6f3d7164d8ae391"


@pytest.mark.parametrize("existing", (
  {"ModelManager_ActiveBundle": {"internalName": "USER", "minimumSelectorVersion": REQUIRED_JSON_VERSION}},
  {"ModelManager_DownloadRef": "user-request"},
))
def test_default_fallback_never_overwrites_user_model_or_download(existing):
  params = FakeParams()
  params.values.update(existing)
  params.values["ModelManager_ActiveBundleChestnut"] = {
    "internalName": "BMV4",
    "minimumSelectorVersion": REQUIRED_JSON_VERSION,
  }

  manager_module.ensure_default_qcom_fallback(params)

  if "ModelManager_DownloadRef" in existing:
    assert params.values["ModelManager_DownloadRef"] == "user-request"
  else:
    assert "ModelManager_DownloadRef" not in params.values


def test_chestnut_skip_drive_roundtrip(tmp_path):
  from openpilot.sunnypilot.modeld_v2.egpu_loader import (
    chestnut_skip_drive, clear_chestnut_skip_drive, set_chestnut_skip_drive,
  )

  path = str(tmp_path / "chestnut_skip_drive")
  assert chestnut_skip_drive(path) is False
  set_chestnut_skip_drive(path)
  assert chestnut_skip_drive(path) is True
  clear_chestnut_skip_drive(path)
  assert chestnut_skip_drive(path) is False
  clear_chestnut_skip_drive(path)
