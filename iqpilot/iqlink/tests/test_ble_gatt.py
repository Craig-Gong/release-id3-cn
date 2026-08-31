"""Unit tests for iqlink BLE envelope HMAC / seq / skew.

Run: pytest iqpilot/iqlink/tests/test_ble_gatt.py
"""

import pytest

from iqpilot.iqlink.ble_gatt import (
  BleAuthError,
  SeqTracker,
  compute_hmac_hex,
  extract_nav_payload,
  verify_envelope,
)


def test_status_notify_payload_includes_device_ms():
  from iqpilot.iqlink.ble_gatt import status_notify_payload
  import json

  payload = status_notify_payload(1_720_000_000_123)
  obj = json.loads(payload.decode("ascii"))
  assert obj == {"ok": True, "t": 1_720_000_000_123}


def test_compute_hmac_stable():
  psk = "123456"
  data = {"nRoadLimitSpeed": 80, "nTBTDist": 100}
  mac = compute_hmac_hex(psk, 7, 1_720_000_000_000, data)
  assert len(mac) == 32
  assert mac == mac.lower()
  assert mac == compute_hmac_hex(psk, 7, 1_720_000_000_000, data)
  # key order in data must not matter (canonical sort_keys)
  data2 = {"nTBTDist": 100, "nRoadLimitSpeed": 80}
  assert compute_hmac_hex(psk, 7, 1_720_000_000_000, data2) == mac


def test_verify_envelope_ok_and_replay():
  psk = "654321"
  data = {"nRoadLimitSpeed": 60}
  seq, ts = 10, 1_720_000_000_000
  env = {"v": 1, "seq": seq, "ts": ts, "data": data, "hmac": compute_hmac_hex(psk, seq, ts, data)}
  tracker = SeqTracker()
  out = verify_envelope(env, psk, tracker, now_ms=ts)
  assert out == data
  with pytest.raises(BleAuthError, match="seq_replay"):
    verify_envelope(env, psk, tracker, now_ms=ts)


def test_verify_rejects_bad_hmac_and_skew():
  psk = "111111"
  data = {"nRoadLimitSpeed": 50}
  seq, ts = 1, 1_720_000_000_000
  env = {"v": 1, "seq": seq, "ts": ts, "data": data, "hmac": "0" * 32}
  with pytest.raises(BleAuthError, match="bad_hmac"):
    verify_envelope(env, psk, SeqTracker(), now_ms=ts)
  env["hmac"] = compute_hmac_hex(psk, seq, ts, data)
  with pytest.raises(BleAuthError, match="ts_skew"):
    verify_envelope(env, psk, SeqTracker(), now_ms=ts + 200_000)
  with pytest.raises(BleAuthError, match="missing_psk"):
    verify_envelope(env, "", SeqTracker(), now_ms=ts)


def test_verify_allows_clock_broken_skew():
  """Device RTC days wrong until NTP — accept plausible phone ts (seq+HMAC bind)."""
  psk = "222222"
  data = {"nRoadLimitSpeed": 80}
  seq, ts = 3, 1_720_000_000_000  # plausible 2024 phone ts
  env = {"v": 1, "seq": seq, "ts": ts, "data": data, "hmac": compute_hmac_hex(psk, seq, ts, data)}
  # Device clock ~52 days behind phone (same class of boot skew seen on device).
  device_now = ts - 52 * 24 * 3600 * 1000
  out = verify_envelope(env, psk, SeqTracker(), now_ms=device_now)
  assert out == data


def test_extract_nav_payload():
  flat = {"nRoadLimitSpeed": 40, "x": 1}
  assert extract_nav_payload(flat) is flat
  wrapped = {"rgdata": {"nRoadLimitSpeed": 40}}
  assert extract_nav_payload(wrapped) == {"nRoadLimitSpeed": 40}


def test_any_device_connected():
  from iqpilot.iqlink.ble_gatt import DEVICE_IFACE, any_device_connected

  adapter = "/org/bluez/hci0"
  empty = {}
  assert any_device_connected(empty, adapter) is False
  assert any_device_connected(empty, None) is False

  managed = {
    f"{adapter}/dev_AA_BB": {DEVICE_IFACE: {"Connected": False, "Address": "AA:BB"}},
    f"{adapter}/dev_CC_DD": {DEVICE_IFACE: {"Connected": True, "Address": "CC:DD"}},
    "/org/bluez/hci1/dev_EE": {DEVICE_IFACE: {"Connected": True}},
  }
  assert any_device_connected(managed, adapter) is True
  managed[f"{adapter}/dev_CC_DD"][DEVICE_IFACE]["Connected"] = False
  assert any_device_connected(managed, adapter) is False


def test_set_ble_link_state_writes_params():
  from iqpilot.iqlink.ble_gatt import (
    ADV_WINDOW_S,
    BLE_CONNECTED_PARAM,
    BLE_LINK_STATE_PARAM,
    LINK_CONNECTED,
    LINK_CONNECTING,
    LINK_OFF,
    set_ble_connected,
    set_ble_link_state,
  )

  class _FakeParams:
    def __init__(self):
      self.vals = {}

    def put(self, key, value):
      self.vals[key] = value

    def put_bool(self, key, value):
      self.vals[key] = bool(value)

  p = _FakeParams()
  set_ble_link_state(p, LINK_CONNECTING)
  assert p.vals[BLE_LINK_STATE_PARAM] == LINK_CONNECTING
  assert p.vals[BLE_CONNECTED_PARAM] is False
  set_ble_link_state(p, LINK_CONNECTED)
  assert p.vals[BLE_CONNECTED_PARAM] is True
  set_ble_connected(p, False)
  assert p.vals[BLE_LINK_STATE_PARAM] == LINK_OFF
  assert ADV_WINDOW_S == 120.0


def test_discover_window_vs_advertising_policy():
  """ADV_WINDOW_S gates PSK/discovering UI only; ADV off only when SoftBus+HMAC both up."""
  from iqpilot.iqlink.ble_gatt import ADV_WINDOW_S, LINK_CONNECTED, LINK_OFF, ble_should_advertise

  assert ADV_WINDOW_S == 120.0

  assert ble_should_advertise(enabled=True, link_state=LINK_OFF, peer_connected=False) is True
  assert ble_should_advertise(enabled=True, link_state=LINK_CONNECTED, peer_connected=True) is False
  # SoftBus flap: LinkState=2 but peer down → ADV so phone remounts without demote.
  assert ble_should_advertise(enabled=True, link_state=LINK_CONNECTED, peer_connected=False) is True
  assert ble_should_advertise(enabled=False, link_state=LINK_OFF, peer_connected=False) is False


def test_next_adv_retry_delay():
  from iqpilot.iqlink.ble_gatt import ADV_RETRY_BACKOFF_S, next_adv_retry_delay

  assert next_adv_retry_delay(0) == 0.0
  assert next_adv_retry_delay(1) == ADV_RETRY_BACKOFF_S[0]
  assert next_adv_retry_delay(2) == ADV_RETRY_BACKOFF_S[1]
  assert next_adv_retry_delay(3) == ADV_RETRY_BACKOFF_S[2]
  assert next_adv_retry_delay(99) == ADV_RETRY_BACKOFF_S[-1]
  assert next_adv_retry_delay(2, backoff=(0.5, 1.5)) == 1.5


def test_peer_is_zombie():
  """Pure helper for SoftBus-up / no-WriteValue zombie drop (run_ble_gatt_loop)."""
  # Callers: ble_gatt.IqlinkBleGatt.maybe_drop_zombie_peers → peer_is_zombie
  # Existing file: iqpilot/iqlink/tests/test_ble_gatt.py (edit, not new)
  from iqpilot.iqlink.ble_gatt import LINK_CONNECTED, LINK_OFF, peer_is_zombie

  assert peer_is_zombie(
    link_state=LINK_CONNECTED, peer_connected_mono=1.0, last_nav_rx_mono=0.0, now_mono=100.0,
  ) is False
  # HMAC connected but WriteValue went stale (SoftBus may already be down).
  assert peer_is_zombie(
    link_state=LINK_CONNECTED, peer_connected_mono=0.0, last_nav_rx_mono=20.0, now_mono=31.0, zombie_s=12.0,
    softbus_down_zombie_s=12.0,
  ) is False
  assert peer_is_zombie(
    link_state=LINK_CONNECTED, peer_connected_mono=0.0, last_nav_rx_mono=20.0, now_mono=32.0, zombie_s=12.0,
    softbus_down_zombie_s=12.0,
  ) is True
  # Fresh HMAC connect: never zombie even if last_nav_rx looks stale (Disconnect race).
  assert peer_is_zombie(
    link_state=LINK_CONNECTED, peer_connected_mono=0.0, last_nav_rx_mono=20.0, now_mono=100.0,
    zombie_s=12.0, link_connected_mono=90.0, hmac_grace_s=60.0,
  ) is False
  assert peer_is_zombie(
    link_state=LINK_OFF, peer_connected_mono=10.0, last_nav_rx_mono=0.0, now_mono=100.0,
    zombie_s=12.0, link_connected_mono=95.0, hmac_grace_s=60.0,
  ) is False
  # SoftBus down short window (45s): no demote — remount protect (SOFTBUS_DOWN_ZOMBIE_S=100).
  assert peer_is_zombie(
    link_state=LINK_CONNECTED, peer_connected_mono=0.0, last_nav_rx_mono=50.0, now_mono=100.0,
    zombie_s=45.0, softbus_down_zombie_s=100.0, link_connected_mono=10.0, hmac_grace_s=60.0,
  ) is False
  # SoftBus up + Status=0: still demotes at zombie_s (45).
  assert peer_is_zombie(
    link_state=LINK_CONNECTED, peer_connected_mono=10.0, last_nav_rx_mono=50.0, now_mono=100.0,
    zombie_s=45.0, softbus_down_zombie_s=100.0, link_connected_mono=10.0, hmac_grace_s=60.0,
  ) is True
  # Past SoftBus-down stale + SoftBus already down + stale WriteValue → demote.
  assert peer_is_zombie(
    link_state=LINK_CONNECTED, peer_connected_mono=0.0, last_nav_rx_mono=20.0, now_mono=130.0,
    zombie_s=45.0, softbus_down_zombie_s=100.0, link_connected_mono=10.0, hmac_grace_s=60.0,
  ) is True
  # Past grace + no WriteValue ever → zombie (anchor = connect mono; SoftBus-down stale).
  assert peer_is_zombie(
    link_state=LINK_CONNECTED, peer_connected_mono=0.0, last_nav_rx_mono=0.0, now_mono=140.0,
    zombie_s=45.0, softbus_down_zombie_s=100.0, link_connected_mono=30.0, hmac_grace_s=60.0,
  ) is True
  # Inside grace + no WriteValue → never demote.
  assert peer_is_zombie(
    link_state=LINK_CONNECTED, peer_connected_mono=0.0, last_nav_rx_mono=0.0, now_mono=50.0,
    zombie_s=45.0, link_connected_mono=30.0, hmac_grace_s=60.0,
  ) is False

  assert peer_is_zombie(
    link_state=LINK_OFF, peer_connected_mono=0.0, last_nav_rx_mono=0.0, now_mono=100.0,
  ) is False
  assert peer_is_zombie(
    link_state=LINK_OFF, peer_connected_mono=10.0, last_nav_rx_mono=0.0, now_mono=21.0, zombie_s=12.0,
  ) is False
  assert peer_is_zombie(
    link_state=LINK_OFF, peer_connected_mono=10.0, last_nav_rx_mono=0.0, now_mono=22.0, zombie_s=12.0,
  ) is True
  assert peer_is_zombie(
    link_state=LINK_OFF, peer_connected_mono=10.0, last_nav_rx_mono=20.0, now_mono=31.0, zombie_s=12.0,
  ) is False
  assert peer_is_zombie(
    link_state=LINK_OFF, peer_connected_mono=10.0, last_nav_rx_mono=20.0, now_mono=32.0, zombie_s=12.0,
  ) is True


def test_stale_hmac_allows_advertise_after_demote():
  """SoftBus-down ADV without demote; long stale still demotes."""
  from iqpilot.iqlink.ble_gatt import LINK_CONNECTED, LINK_OFF, ble_should_advertise, peer_is_zombie

  # SoftBus down + link=2 short window: ADV on, no demote.
  assert peer_is_zombie(
    link_state=LINK_CONNECTED, peer_connected_mono=0.0, last_nav_rx_mono=50.0, now_mono=100.0,
    zombie_s=45.0, softbus_down_zombie_s=100.0, link_connected_mono=10.0, hmac_grace_s=60.0,
  ) is False
  assert ble_should_advertise(enabled=True, link_state=LINK_CONNECTED, peer_connected=False) is True

  # Inside HMAC grace: no demote; SoftBus-up keeps ADV off.
  assert peer_is_zombie(
    link_state=LINK_CONNECTED, peer_connected_mono=1.0, last_nav_rx_mono=20.0, now_mono=50.0,
    zombie_s=45.0, softbus_down_zombie_s=100.0, link_connected_mono=40.0, hmac_grace_s=60.0,
  ) is False
  assert ble_should_advertise(enabled=True, link_state=LINK_CONNECTED, peer_connected=True) is False

  # Past SoftBus-down stale → demote → ADV on.
  assert peer_is_zombie(
    link_state=LINK_CONNECTED, peer_connected_mono=0.0, last_nav_rx_mono=20.0, now_mono=130.0,
    zombie_s=45.0, softbus_down_zombie_s=100.0, link_connected_mono=10.0, hmac_grace_s=60.0,
  ) is True
  assert ble_should_advertise(enabled=True, link_state=LINK_OFF, peer_connected=False) is True


def test_hmac_connect_is_fresh():
  from iqpilot.iqlink.ble_gatt import hmac_connect_is_fresh

  assert hmac_connect_is_fresh(link_connected_mono=0.0, now_mono=100.0) is False
  assert hmac_connect_is_fresh(link_connected_mono=90.0, now_mono=100.0, grace_s=60.0) is True
  assert hmac_connect_is_fresh(link_connected_mono=40.0, now_mono=100.0, grace_s=60.0) is False


def test_softbus_down_should_clear_link():
  """SoftBus flap must not clear HMAC LinkState while WriteValue is fresh."""
  from iqpilot.iqlink.ble_gatt import (
    LINK_CONNECTED,
    LINK_CONNECTING,
    LINK_OFF,
    softbus_down_should_clear_link,
  )

  assert softbus_down_should_clear_link(
    link_state=LINK_OFF, last_nav_rx_mono=0.0, now_mono=100.0,
  ) is True
  assert softbus_down_should_clear_link(
    link_state=LINK_CONNECTING, last_nav_rx_mono=99.0, now_mono=100.0,
  ) is True
  assert softbus_down_should_clear_link(
    link_state=LINK_CONNECTED, last_nav_rx_mono=0.0, now_mono=100.0,
  ) is True
  # Just HMAC-connected: SoftBus flap must not clear even if last_nav_rx unset.
  assert softbus_down_should_clear_link(
    link_state=LINK_CONNECTED, last_nav_rx_mono=0.0, now_mono=100.0,
    link_connected_mono=95.0, grace_s=15.0,
  ) is False
  assert softbus_down_should_clear_link(
    link_state=LINK_CONNECTED, last_nav_rx_mono=95.0, now_mono=100.0, grace_s=15.0,
  ) is False
  assert softbus_down_should_clear_link(
    link_state=LINK_CONNECTED, last_nav_rx_mono=80.0, now_mono=100.0, grace_s=15.0,
  ) is True


def test_connected_device_paths():
  from iqpilot.iqlink.ble_gatt import DEVICE_IFACE, connected_device_paths

  adapter = "/org/bluez/hci0"
  managed = {
    f"{adapter}/dev_AA": {DEVICE_IFACE: {"Connected": True}},
    f"{adapter}/dev_BB": {DEVICE_IFACE: {"Connected": False}},
    "/org/bluez/hci1/dev_CC": {DEVICE_IFACE: {"Connected": True}},
  }
  assert connected_device_paths(managed, adapter) == [f"{adapter}/dev_AA"]
  assert connected_device_paths(managed, None) == []


def test_connecting_is_stale():
  """F4: LinkState stuck in connecting without HMAC → recover."""
  from iqpilot.iqlink.ble_gatt import CONNECTING_TIMEOUT_S, LINK_CONNECTING, LINK_OFF, connecting_is_stale

  assert connecting_is_stale(
    link_state=LINK_OFF, connecting_since_mono=1.0, now_mono=100.0,
  ) is False
  assert connecting_is_stale(
    link_state=LINK_CONNECTING, connecting_since_mono=0.0, now_mono=100.0,
  ) is False
  assert connecting_is_stale(
    link_state=LINK_CONNECTING, connecting_since_mono=10.0, now_mono=10.0 + CONNECTING_TIMEOUT_S - 0.1,
  ) is False
  assert connecting_is_stale(
    link_state=LINK_CONNECTING, connecting_since_mono=10.0, now_mono=10.0 + CONNECTING_TIMEOUT_S,
  ) is True


def test_latest_envelope_slot_coalesce():
  """Rapid puts keep only the last env; take clears (nav = latest-only)."""
  from iqpilot.iqlink.ble_gatt import LatestEnvelopeSlot

  slot = LatestEnvelopeSlot()
  assert slot.take() is None
  slot.put({"seq": 1})
  slot.put({"seq": 2})
  slot.put({"seq": 3})
  assert slot.take() == {"seq": 3}
  assert slot.take() is None
  slot.put({"seq": 4})
  assert slot.take() == {"seq": 4}

