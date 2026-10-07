from types import SimpleNamespace

from openpilot.sunnypilot.models import helpers, manager


def test_installed_default_avoids_download_after_reset(tmp_path, monkeypatch):
  model = tmp_path / 'driving_tinygrad.pkl'
  monkeypatch.setattr(helpers, 'modeld_pkl_path', lambda big: model)
  model.with_name(model.name + '.chunkmanifest').write_text('2')
  model.with_name(model.name + '.chunk01of02').write_bytes(b'compiled model')
  model.with_name(model.name + '.chunk02of02').touch()  # legitimate SCons padding
  fallback = helpers.bundled_qcom_fallback()
  assert fallback.models[0].artifact.fileName == str(model)
  calls = []
  params = SimpleNamespace(get=lambda key: None, put=lambda *args: calls.append(args))
  monkeypatch.setattr(manager, 'get_selected_bundle', lambda p, source: object() if source == 'chestnut' else None)
  manager.ensure_default_qcom_fallback(params)
  assert calls == []
  model.with_name(model.name + '.chunk01of02').unlink()
  assert helpers.bundled_qcom_fallback() is None
  manager.ensure_default_qcom_fallback(params)
  assert len(calls) == 1


def test_empty_slots_keep_stock_runner_and_explicit_small_choice_wins(monkeypatch):
  monkeypatch.setattr(helpers, 'get_active_source', lambda **kw: 'qcom')
  params = object()
  monkeypatch.setattr(helpers, 'get_selected_bundle', lambda p, source: None)
  monkeypatch.setattr(helpers, 'bundled_qcom_fallback', lambda: 'bundled')
  assert helpers.get_active_bundle(params, chestnut=False) is None
  monkeypatch.setattr(helpers, 'get_selected_bundle', lambda p, source: 'big' if source == 'chestnut' else None)
  assert helpers.get_active_bundle(params, chestnut=False) == 'bundled'
  monkeypatch.setattr(helpers, 'get_selected_bundle', lambda p, source: 'user-selected')
  assert helpers.get_active_bundle(params, chestnut=False) == 'user-selected'
