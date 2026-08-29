from openpilot.iqpilot.iqlink.konn3kt_ble_exclusive import (
  KONN3KT_BLE_PARAM,
  konn3kt_ble_for_iqlink,
  sync_konn3kt_ble_for_iqlink,
)


class FakeParams:
  def __init__(self, data: dict | None = None):
    self.data = dict(data or {})

  def get_bool(self, key: str) -> bool:
    val = self.data.get(key)
    if val is None:
      raise KeyError(key)
    return bool(val) if not isinstance(val, str) else val in ("1", "true", "True")

  def put_bool(self, key: str, val: bool) -> None:
    self.data[key] = val


def test_mirror_mapping():
  assert konn3kt_ble_for_iqlink(True) is False
  assert konn3kt_ble_for_iqlink(False) is True


def test_sync_writes_on_mismatch():
  p = FakeParams({"IqlinkEnabled": True, KONN3KT_BLE_PARAM: True})
  assert sync_konn3kt_ble_for_iqlink(p, True) is True
  assert p.get_bool(KONN3KT_BLE_PARAM) is False


def test_sync_idempotent():
  p = FakeParams({"IqlinkEnabled": False, KONN3KT_BLE_PARAM: True})
  assert sync_konn3kt_ble_for_iqlink(p, False) is False
