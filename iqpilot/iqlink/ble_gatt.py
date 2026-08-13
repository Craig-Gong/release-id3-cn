#!/usr/bin/env python3
"""iqlink BLE GATT (nav write + status notify) via BlueZ / gi.

Wire contract must match the phone app — see PROTOCOL.md.
Patterned after system/ui/lib/setup_ble.py but smaller (no setup auth/framing).
"""

from __future__ import annotations

import hashlib
import hmac
import json
import os
import socket
import threading
import time
from typing import Any, Callable

from openpilot.common.params import Params


class _LazyCloudlog:
  """Defer swaglog; on Windows host cereal/capnp aborts, so use stdlib logging."""

  _real = None

  def _get(self):
    if self._real is None:
      import sys
      if sys.platform == "win32":
        import logging
        self._real = logging.getLogger("iqlink")
      else:
        from openpilot.common.swaglog import cloudlog as real
        self._real = real
    return self._real

  def __getattr__(self, name):
    return getattr(self._get(), name)


cloudlog = _LazyCloudlog()

# --- wire UUIDs (MUST match phone; do NOT use setup 73f2c700-…) ---
IQLINK_SERVICE_UUID = "73f2c710-5e40-4d0d-8b7f-fde61f729100"
IQLINK_NAV_CHAR_UUID = "73f2c711-5e40-4d0d-8b7f-fde61f729100"
IQLINK_STATUS_CHAR_UUID = "73f2c712-5e40-4d0d-8b7f-fde61f729100"

PSK_PARAM = "IqlinkBlePsk"
FIXED_BLE_PSK = "999999"
BLE_CONNECTED_PARAM = "IqlinkBleConnected"
BLE_PEER_CONNECTED_PARAM = "IqlinkBlePeerConnected"  # SoftBus Device1 Connected (UI green)
BLE_LINK_STATE_PARAM = "IqlinkBleLinkState"  # 0=off, 1=connecting, 2=connected
BLE_DISCOVERING_PARAM = "IqlinkBleDiscovering"
BLE_PAIR_FAILED_PARAM = "IqlinkBlePairFailed"
LINK_OFF = 0
LINK_CONNECTING = 1
LINK_CONNECTED = 2
ADV_WINDOW_S = 120.0  # discover/PSK UI window after IqlinkEnabled rising edge
MAX_SKEW_MS = 120_000
# Device RTC is often days wrong until NTP (timed log: -52d). Phone ts is authoritative
# when skew is absurd; seq+HMAC still bind the session.
CLOCK_BROKEN_SKEW_MS = 3_600_000  # 1h
_PLAUSIBLE_TS_MS_MIN = 1_704_067_200_000  # 2024-01-01 UTC
_PLAUSIBLE_TS_MS_MAX = 1_893_456_000_000  # 2030-01-01 UTC
SEQ_REPLAY_WINDOW = 128
MAX_ENVELOPE_BYTES = 64 * 1024
ADAPTER_WAIT_TIMEOUT_S = 8.0
BLUEZ_REGISTER_TIMEOUT_MS = 15_000
ADV_REGISTER_TIMEOUT_MS = 5_000  # re-adv: short timeout + backoff (avoid 15s GLib stall)
ADV_UNREGISTER_TIMEOUT_MS = 3_000
ADV_POST_UNREGISTER_DELAY_S = 0.4  # let BlueZ settle before RegisterAdvertisement
ADV_RETRY_BACKOFF_S = (1.0, 2.0, 5.0)
ADV_FAIL_RECOVER_S = 30.0  # continuous re-adv fail → restart GATT app
# SoftBus Connected=false must not clear HMAC LinkState if WriteValue was recent
# (BlueZ flap / late Disconnect after phone already reconnected+HMAC).
# First ~10 min SoftBus flaps are common on Huawei; keep grace > cancel timeout.
SOFTBUS_HMAC_GRACE_S = 60.0
# SoftBus Connected but no GATT WriteValue (phone Status=0 zombie) → Disconnect peers.
# Keep > typical Huawei MTU+discover+first-write (often 5–15s); phone NO_ACK is 8s.
ZOMBIE_PEER_S = 45.0
# SoftBus already down + LinkState=2: phone may remount ATT; demote later than SoftBus-up
# Status=0 (force-stop still ADV via ble_should_advertise, demote ≤ ~100s).
SOFTBUS_DOWN_ZOMBIE_S = 100.0
# Post-HMAC protect: SoftBus-zombie Disconnect must not race a new LINK_CONNECTED (~1–3s flash).
# ≥ ZOMBIE_PEER_S; reuse SoftBus grace so flap+zombie share one window.
ZOMBIE_HMAC_GRACE_S = SOFTBUS_HMAC_GRACE_S
# After zombie Disconnect, hold ADV briefly so call_sync settles before phone re-pairs.
ZOMBIE_DROP_COOLDOWN_S = 1.5
# TODO: SoftBus flap can leave dual Device1 Connected; prefer Disconnect oldest-only (one-liner later).
# F4: LinkState=connecting without HMAC success → fall back to off (UI leaves yellow).
CONNECTING_TIMEOUT_S = 30.0
ENABLED_POLL_S = 1.0

BLUEZ_SERVICE = "org.bluez"
DBUS_OM_IFACE = "org.freedesktop.DBus.ObjectManager"
DBUS_PROPS_IFACE = "org.freedesktop.DBus.Properties"
GATT_MANAGER_IFACE = "org.bluez.GattManager1"
LE_ADV_MANAGER_IFACE = "org.bluez.LEAdvertisingManager1"
GATT_SERVICE_IFACE = "org.bluez.GattService1"
GATT_CHRC_IFACE = "org.bluez.GattCharacteristic1"
LE_ADV_IFACE = "org.bluez.LEAdvertisement1"
ADAPTER_IFACE = "org.bluez.Adapter1"
DEVICE_IFACE = "org.bluez.Device1"
AGENT_IFACE = "org.bluez.Agent1"
AGENT_MANAGER_IFACE = "org.bluez.AgentManager1"

# Gate facts (edit existing ble_gatt.py, not a new file):
# 1. Callers: iqpilot/iqlink/bridge.py → run_ble_gatt_loop / ensure_ble_psk
# 2. No other BlueZ Agent in tree (grep AgentManager empty outside this edit)
# 3. Params: IqlinkBlePsk (6-digit string), IqlinkEnabled, IqlinkBleLinkState — unchanged schema
# 4. User: 「配对过一次就变成已配对的设备，自动连接」+ fixed 2min PSK / plaintext APK / fix pair fail


def set_ble_link_state(params: Params, state: int) -> None:
  """UI polls IqlinkBleLinkState (0/1/2). Also mirrors IqlinkBleConnected=(state==2)."""
  state = int(state)
  if state not in (LINK_OFF, LINK_CONNECTING, LINK_CONNECTED):
    state = LINK_OFF
  try:
    params.put(BLE_LINK_STATE_PARAM, state)
  except Exception:
    pass
  try:
    params.put_bool(BLE_CONNECTED_PARAM, state == LINK_CONNECTED)
  except Exception:
    pass


def set_ble_connected(params: Params, connected: bool) -> None:
  """Compat: True → link=2, False → link=0 (clears connecting). Prefer set_ble_link_state."""
  set_ble_link_state(params, LINK_CONNECTED if connected else LINK_OFF)


def set_ble_discovering(params: Params, discovering: bool) -> None:
  try:
    params.put_bool(BLE_DISCOVERING_PARAM, bool(discovering))
  except Exception:
    pass


def set_ble_pair_failed(params: Params, failed: bool) -> None:
  """UI red while IqlinkEnabled: HMAC/PSK mismatch or connecting timeout (F4)."""
  try:
    params.put_bool(BLE_PAIR_FAILED_PARAM, bool(failed))
  except Exception:
    pass


def set_ble_peer_connected(params: Params, connected: bool) -> None:
  """UI green when SoftBus peer is up (product C); independent of HMAC LinkState."""
  try:
    params.put_bool(BLE_PEER_CONNECTED_PARAM, bool(connected))
  except Exception:
    pass


def connecting_is_stale(
  *,
  link_state: int,
  connecting_since_mono: float,
  now_mono: float,
  timeout_s: float = CONNECTING_TIMEOUT_S,
) -> bool:
  """True when LinkState stuck in connecting without reaching HMAC connected."""
  if int(link_state) != LINK_CONNECTING:
    return False
  if connecting_since_mono <= 0:
    return False
  return (now_mono - connecting_since_mono) >= float(timeout_s)


def _variant_bool(value: Any) -> bool:
  if value is None:
    return False
  if hasattr(value, "unpack"):
    try:
      value = value.unpack()
    except Exception:
      return False
  return bool(value)


def any_device_connected(managed: dict, adapter_path: str | None) -> bool:
  """True if any BlueZ Device1 under adapter reports Connected."""
  return bool(connected_device_paths(managed, adapter_path))


def connected_device_paths(managed: dict, adapter_path: str | None) -> list[str]:
  """BlueZ Device1 object paths under adapter with Connected=true."""
  if not adapter_path:
    return []
  prefix = adapter_path.rstrip("/") + "/"
  out: list[str] = []
  for path, ifaces in managed.items():
    if not str(path).startswith(prefix):
      continue
    props = ifaces.get(DEVICE_IFACE)
    if not isinstance(props, dict):
      continue
    if _variant_bool(props.get("Connected")):
      out.append(str(path))
  return out


def hmac_connect_is_fresh(
  *,
  link_connected_mono: float,
  now_mono: float,
  grace_s: float = ZOMBIE_HMAC_GRACE_S,
) -> bool:
  """True for a short window after HMAC entered LINK_CONNECTED."""
  if link_connected_mono <= 0:
    return False
  return (now_mono - link_connected_mono) < float(grace_s)


def ble_should_advertise(*, enabled: bool, link_state: int, peer_connected: bool) -> bool:
  """ADV while enabled unless HMAC session still has a live SoftBus peer.

  SoftBus Connected=false + LinkState=2: keep ADV so phone can remount ATT without demoting
  (avoids 45–70s link_state=off flash). Demote only after long WriteValue stale.
  """
  if not enabled:
    return False
  if int(link_state) == LINK_CONNECTED and bool(peer_connected):
    return False
  return True


def peer_is_zombie(
  *,
  link_state: int,
  peer_connected_mono: float,
  last_nav_rx_mono: float,
  now_mono: float,
  zombie_s: float = ZOMBIE_PEER_S,
  softbus_down_zombie_s: float = SOFTBUS_DOWN_ZOMBIE_S,
  link_connected_mono: float = 0.0,
  hmac_grace_s: float = ZOMBIE_HMAC_GRACE_S,
) -> bool:
  """True when LE peer is up / HMAC was up but our GATT app has no recent WriteValue.

  SoftBus Connected can flap on phone stacks while ATT still works; HMAC LinkState=2
  must not be cleared on that edge. Instead, stale WriteValue demotes the session.
  Fresh HMAC connect is never a zombie (avoids Disconnect racing re-pair).
  SoftBus-already-down uses a longer stale so remount can finish before demote.
  """
  if hmac_connect_is_fresh(
    link_connected_mono=link_connected_mono, now_mono=now_mono, grace_s=hmac_grace_s,
  ):
    return False
  if int(link_state) == LINK_CONNECTED:
    # No WriteValue yet: after grace, anchor on connect mono (force-stop before first
    # nav write must not leave LinkState=2 forever — ADV already on via SoftBus-down).
    anchor = last_nav_rx_mono if last_nav_rx_mono > 0 else link_connected_mono
    if anchor <= 0:
      return False
    stale_s = softbus_down_zombie_s if peer_connected_mono <= 0 else zombie_s
    return (now_mono - anchor) >= float(stale_s)
  if peer_connected_mono <= 0:
    return False
  anchor = last_nav_rx_mono if last_nav_rx_mono > 0 else peer_connected_mono
  return (now_mono - anchor) >= float(zombie_s)


def softbus_down_should_clear_link(
  *,
  link_state: int,
  last_nav_rx_mono: float,
  now_mono: float,
  grace_s: float = SOFTBUS_HMAC_GRACE_S,
  link_connected_mono: float = 0.0,
) -> bool:
  """SoftBus Connected=false: clear LinkState unless HMAC WriteValue / connect was recent."""
  if int(link_state) != LINK_CONNECTED:
    return True
  if hmac_connect_is_fresh(
    link_connected_mono=link_connected_mono, now_mono=now_mono, grace_s=grace_s,
  ):
    return False
  if last_nav_rx_mono <= 0:
    return True
  return (now_mono - last_nav_rx_mono) >= float(grace_s)


def next_adv_retry_delay(fail_count: int, backoff: tuple[float, ...] = ADV_RETRY_BACKOFF_S) -> float:
  """Seconds to wait after N consecutive RegisterAdvertisement failures (fail_count >= 1)."""
  if fail_count <= 0:
    return 0.0
  return float(backoff[min(int(fail_count) - 1, len(backoff) - 1)])


def _import_gi():
  import sys
  # Windows host has no BlueZ/gi; importing broken PyGObject can abort the process.
  if sys.platform == "win32":
    raise RuntimeError("gi_unavailable_on_win32")
  try:
    import gi  # noqa: F401
  except Exception:
    for extra in ("/usr/lib/python3/dist-packages", "/usr/lib/python3.12/dist-packages"):
      if os.path.isdir(os.path.join(extra, "gi")) and extra not in sys.path:
        sys.path.append(extra)
    import gi  # noqa: F401
  gi.require_version("Gio", "2.0")
  from gi.repository import Gio, GLib
  return gi, Gio, GLib


try:
  gi, Gio, GLib = _import_gi()
  _GI_AVAILABLE = True
except Exception:
  _GI_AVAILABLE = False
  Gio = GLib = None  # type: ignore
  gi = None  # type: ignore

# ---------------------------------------------------------------------------
# PSK + HMAC (pure; unit-tested without BlueZ)
# ---------------------------------------------------------------------------
def mask_psk(psk: str) -> str:
  psk = str(psk or "")
  if len(psk) < 2:
    return "******"
  return f"****{psk[-2:]}"


def ensure_ble_psk(params: Params | None = None) -> str:
  """Return fixed 6-digit PSK; write Params if missing/wrong."""
  params = params or Params()
  raw = params.get(PSK_PARAM)
  if isinstance(raw, (bytes, bytearray)):
    raw = raw.decode("utf-8", errors="ignore")
  psk = ("" if raw is None else str(raw)).strip()
  if psk == FIXED_BLE_PSK:
    return psk
  params.put(PSK_PARAM, FIXED_BLE_PSK)
  cloudlog.info(f"iqlink ble: set PSK {mask_psk(FIXED_BLE_PSK)}")
  return FIXED_BLE_PSK


def canonical_json(data: Any) -> bytes:
  return json.dumps(data, ensure_ascii=True, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def compute_hmac_hex(psk: str, seq: int, ts: int, data: dict[str, Any]) -> str:
  msg = f"{int(seq)}:{int(ts)}:".encode("utf-8") + canonical_json(data)
  return hmac.new(psk.encode("utf-8"), msg, hashlib.sha256).hexdigest()[:32]


class SeqTracker:
  """Sliding-window seq replay guard (window=128)."""

  def __init__(self, window: int = SEQ_REPLAY_WINDOW):
    self.window = window
    self.highest = 0
    self.mask = 0
    self._lock = threading.Lock()

  def check_and_consume(self, seq: int) -> bool:
    seq = int(seq)
    if seq <= 0:
      return False
    with self._lock:
      if seq > self.highest:
        shift = seq - self.highest
        self.mask = ((self.mask << shift) | 1) & ((1 << self.window) - 1)
        self.highest = seq
        return True
      offset = self.highest - seq
      if offset >= self.window or (self.mask >> offset) & 1:
        return False
      self.mask |= 1 << offset
      return True


class LatestEnvelopeSlot:
  """Single-slot coalesce: put overwrites; take clears. Nav only needs latest.

  Pure helper — unit-tested without BlueZ.
  """

  def __init__(self) -> None:
    self._lock = threading.Lock()
    self._env: dict[str, Any] | None = None
    self._event = threading.Event()

  def put(self, env: dict[str, Any]) -> None:
    with self._lock:
      self._env = env
      self._event.set()

  def take(self) -> dict[str, Any] | None:
    with self._lock:
      env = self._env
      self._env = None
      if env is None:
        self._event.clear()
      return env

  def wait(self, timeout: float | None = None) -> bool:
    return self._event.wait(timeout)

  def wake(self) -> None:
    """Unblock waiters on shutdown."""
    self._event.set()


class BleAuthError(Exception):
  pass


def verify_envelope(env: dict[str, Any], psk: str, seq_tracker: SeqTracker, *, now_ms: int | None = None) -> dict[str, Any]:
  """Validate envelope; return `data` dict. Raises BleAuthError on reject."""
  if not psk or not (len(psk) == 6 and psk.isdigit()):
    raise BleAuthError("missing_psk")
  try:
    seq = int(env["seq"])
    ts = int(env["ts"])
  except (KeyError, TypeError, ValueError) as e:
    raise BleAuthError("bad_envelope") from e
  data = env.get("data")
  if not isinstance(data, dict):
    raise BleAuthError("bad_data")
  mac = str(env.get("hmac") or "").strip().lower()
  expected = compute_hmac_hex(psk, seq, ts, data)
  if not hmac.compare_digest(expected, mac):
    raise BleAuthError("bad_hmac")
  now = int(time.time() * 1000) if now_ms is None else int(now_ms)
  skew = abs(now - ts)
  if skew > MAX_SKEW_MS:
    clock_broken = skew > CLOCK_BROKEN_SKEW_MS and _PLAUSIBLE_TS_MS_MIN <= ts <= _PLAUSIBLE_TS_MS_MAX
    if not clock_broken:
      raise BleAuthError("ts_skew")
    cloudlog.warning(f"iqlink ble: accepting envelope despite skew={skew}ms (device clock unsynced)")
  if not seq_tracker.check_and_consume(seq):
    raise BleAuthError("seq_replay")
  return data


def extract_nav_payload(data: dict[str, Any]) -> dict[str, Any]:
  """Prefer flat carrot dict; unwrap rgdata if present and no heartbeat key."""
  if "nRoadLimitSpeed" in data:
    return data
  rg = data.get("rgdata")
  if isinstance(rg, dict):
    return rg
  return data


# ---------------------------------------------------------------------------
# BlueZ GATT (optional)
# ---------------------------------------------------------------------------
OM_XML = '<node><interface name="org.freedesktop.DBus.ObjectManager"><method name="GetManagedObjects"><arg type="a{oa{sa{sv}}}" name="objects" direction="out"/></method></interface></node>'
PROPS_XML = '<node><interface name="org.freedesktop.DBus.Properties"><method name="Get"><arg type="s" direction="in"/><arg type="s" direction="in"/><arg type="v" direction="out"/></method><method name="GetAll"><arg type="s" direction="in"/><arg type="a{sv}" direction="out"/></method></interface></node>'
SERVICE_XML = '<node><interface name="org.bluez.GattService1"><property name="UUID" type="s" access="read"/><property name="Primary" type="b" access="read"/><property name="Characteristics" type="ao" access="read"/></interface></node>'
CHAR_XML = '<node><interface name="org.bluez.GattCharacteristic1"><method name="ReadValue"><arg type="a{sv}" direction="in"/><arg type="ay" direction="out"/></method><method name="WriteValue"><arg type="ay" direction="in"/><arg type="a{sv}" direction="in"/></method><method name="StartNotify"/><method name="StopNotify"/><property name="UUID" type="s" access="read"/><property name="Service" type="o" access="read"/><property name="Flags" type="as" access="read"/><property name="Value" type="ay" access="read"/><property name="Notifying" type="b" access="read"/></interface></node>'
ADV_XML = '<node><interface name="org.bluez.LEAdvertisement1"><method name="Release"/><property name="Type" type="s" access="read"/><property name="ServiceUUIDs" type="as" access="read"/><property name="LocalName" type="s" access="read"/></interface></node>'
# Auto-accept Just Works so phone OS shows comma as bonded after first pair.
AGENT_XML = (
  '<node><interface name="org.bluez.Agent1">'
  '<method name="Release"/>'
  '<method name="RequestPinCode"><arg direction="in" type="o"/><arg direction="out" type="s"/></method>'
  '<method name="DisplayPinCode"><arg direction="in" type="o"/><arg direction="in" type="s"/></method>'
  '<method name="RequestPasskey"><arg direction="in" type="o"/><arg direction="out" type="u"/></method>'
  '<method name="DisplayPasskey"><arg direction="in" type="o"/><arg direction="in" type="u"/><arg direction="in" type="q"/></method>'
  '<method name="RequestConfirmation"><arg direction="in" type="o"/><arg direction="in" type="u"/></method>'
  '<method name="RequestAuthorization"><arg direction="in" type="o"/></method>'
  '<method name="AuthorizeService"><arg direction="in" type="o"/><arg direction="in" type="s"/></method>'
  '<method name="Cancel"/>'
  '</interface></node>'
)


def _variant(sig: str, val: Any):
  return GLib.Variant(sig, val)


class _Exported:
  def __init__(self, path, xml, methods=None, properties=None):
    self.path = path
    self.node = Gio.DBusNodeInfo.new_for_xml(xml)
    self.methods = methods or {}
    self.properties = properties or {}
    self.ids: list[int] = []

  def register(self, bus):
    for iface in self.node.interfaces:
      self.ids.append(bus.register_object(self.path, iface, self._call, self._get, None))

  def unregister(self, bus):
    for i in self.ids:
      try:
        bus.unregister_object(i)
      except Exception:
        pass
    self.ids.clear()

  def _call(self, conn, sender, path, iface, method, params, invocation):
    handler = self.methods.get((iface, method))
    if handler is None:
      invocation.return_dbus_error("org.iqlink.Error", f"unsupported:{iface}.{method}")
      return
    try:
      result = handler(params)
      if result is None:
        invocation.return_value(GLib.Variant("()", ()))
      else:
        invocation.return_value(result)
    except Exception as e:
      invocation.return_dbus_error("org.iqlink.Error", str(e))

  def _get(self, conn, sender, path, iface, name):
    props = self.properties.get(iface, {})
    return props[name]() if name in props else None


class IqlinkBleGatt:
  """Advertise iqlink GATT while running; nav writes → verify → ingest_cb."""

  def __init__(self, ingest_cb: Callable[[dict[str, Any]], None], *, local_name: str | None = None):
    self.ingest_cb = ingest_cb
    self.local_name = (local_name or socket.gethostname() or "iqlink")[:20] or "iqlink"
    self.params = Params()
    self.seq_tracker = SeqTracker()
    self.bus = None
    self.adapter_path: str | None = None
    self.context = None
    self.loop = None
    self._thread: threading.Thread | None = None
    self._ready = threading.Event()
    self._error: Exception | None = None
    self.running = False
    self._root = f"/io/iqlink/ble/p{os.getpid()}"
    self._objects: list[_Exported] = []
    self._notify_status = False
    self._values = {"nav": b"", "status": b""}
    self._nav_buf = bytearray()
    self._link_state = LINK_OFF
    self._sig_ids: list[int] = []
    self._adv_registered = False
    self._adv_fail_count = 0
    self._adv_next_try_mono = 0.0
    self._adv_fail_since_mono = 0.0  # 0 = no active failure streak
    self._adv_cooldown_until = 0.0
    self._agent_registered = False
    self._peer_connected_mono = 0.0  # SoftBus Connected rising edge
    self._last_nav_rx_mono = 0.0  # last GATT nav WriteValue
    self._link_connected_mono = 0.0  # HMAC entered LINK_CONNECTED
    self._connecting_since_mono = 0.0  # F4: enter LINK_CONNECTING mono
    self._peer_drop_cooldown_until = 0.0  # after zombie Disconnect settle
    self._env_slot = LatestEnvelopeSlot()
    self._env_worker: threading.Thread | None = None
    self._env_worker_stop = threading.Event()
    set_ble_link_state(self.params, LINK_OFF)
    set_ble_pair_failed(self.params, False)

  @property
  def _app_path(self): return self._root
  @property
  def _service_path(self): return self._root + "/service0"
  @property
  def _nav_path(self): return self._service_path + "/char0"
  @property
  def _status_path(self): return self._service_path + "/char1"
  @property
  def _adv_path(self): return self._root + "/advertisement0"
  @property
  def _agent_path(self): return self._root + "/agent"

  def start(self, timeout_s: float = 30.0) -> None:
    if not _GI_AVAILABLE:
      raise RuntimeError("gi_unavailable")
    if self.running:
      return
    self._ready.clear()
    self._error = None
    self._env_worker_stop.clear()
    self._env_worker = threading.Thread(target=self._envelope_worker, name="iqlink_ble_env", daemon=True)
    self._env_worker.start()
    self._thread = threading.Thread(target=self._run, name="iqlink_ble_gatt", daemon=True)
    self._thread.start()
    if not self._ready.wait(timeout=timeout_s):
      self._env_worker_stop.set()
      self._env_slot.wake()
      raise RuntimeError("iqlink_ble_start_timeout")
    if self._error is not None:
      self._env_worker_stop.set()
      self._env_slot.wake()
      raise self._error

  def stop(self) -> None:
    self._env_worker_stop.set()
    self._env_slot.wake()
    try:
      if self.context is not None:
        GLib.idle_add(self._stop_on_loop)
      if self._thread is not None:
        self._thread.join(timeout=3.0)
      if self._env_worker is not None:
        self._env_worker.join(timeout=2.0)
    except Exception:
      pass
    self.running = False
    set_ble_connected(self.params, False)
    self._ble_connected = False

  def _run(self):
    self.context = GLib.MainContext()
    self.loop = GLib.MainLoop.new(self.context, False)
    self.context.push_thread_default()
    try:
      src = GLib.idle_source_new()
      src.set_callback(self._startup)
      src.attach(self.context)
      self.loop.run()
    except Exception as e:
      self._error = e
      self._ready.set()
    finally:
      try:
        self.context.pop_thread_default()
      except Exception:
        pass

  def _managed_objects(self) -> dict:
    reply = self.bus.call_sync(
      BLUEZ_SERVICE, "/", DBUS_OM_IFACE, "GetManagedObjects", None,
      GLib.VariantType.new("(a{oa{sa{sv}}})"), Gio.DBusCallFlags.NONE, 5000, None,
    )
    u = reply.unpack()
    return u[0] if isinstance(u, tuple) else u

  def _find_adapter(self) -> str | None:
    for path, ifaces in self._managed_objects().items():
      if GATT_MANAGER_IFACE in ifaces and LE_ADV_MANAGER_IFACE in ifaces and ADAPTER_IFACE in ifaces:
        return path
    return None

  def _startup(self, *_):
    try:
      self.bus = Gio.bus_get_sync(Gio.BusType.SYSTEM, None)
      start = time.monotonic()
      self.adapter_path = None
      while time.monotonic() - start < ADAPTER_WAIT_TIMEOUT_S:
        self.adapter_path = self._find_adapter()
        if self.adapter_path:
          break
        time.sleep(0.5)
      if not self.adapter_path:
        raise RuntimeError("bluetooth_adapter_not_found")
      try:
        self.bus.call_sync(
          BLUEZ_SERVICE, self.adapter_path, DBUS_PROPS_IFACE, "Set",
          GLib.Variant("(ssv)", (ADAPTER_IFACE, "Powered", GLib.Variant("b", True))),
          None, Gio.DBusCallFlags.NONE, 5000, None,
        )
      except Exception:
        pass
      # Allow phone OS-level createBond (Just Works) so Settings shows 已配对.
      for prop, val in (("Pairable", True), ("Discoverable", True)):
        try:
          self.bus.call_sync(
            BLUEZ_SERVICE, self.adapter_path, DBUS_PROPS_IFACE, "Set",
            GLib.Variant("(ssv)", (ADAPTER_IFACE, prop, GLib.Variant("b", val))),
            None, Gio.DBusCallFlags.NONE, 3000, None,
          )
        except Exception:
          pass
      self._register_pair_agent()
      self._register_objects()
      self._register_with_bluez()
      self._watch_connections()
    except Exception as e:
      self._error = e
      self._stop_on_loop()
      if self.loop:
        self.loop.quit()
      self._ready.set()
    return False

  def _char_props(self, uuid, flags, kind):
    return {GATT_CHRC_IFACE: {
      "UUID": lambda: _variant("s", uuid),
      "Service": lambda: _variant("o", self._service_path),
      "Flags": lambda: _variant("as", flags),
      "Value": lambda: _variant("ay", list(self._values[kind])),
      "Notifying": lambda: _variant("b", self._notify_status if kind == "status" else False),
    }}

  def _register_pair_agent(self):
    """BlueZ Agent NoInputNoOutput — auto-accept LE bond so phone lists comma as paired."""
    if self.bus is None:
      return

    def _ok(_params):
      return None

    def _pin(_params):
      return GLib.Variant("(s)", ("000000",))

    def _passkey(_params):
      return GLib.Variant("(u)", (0,))

    methods = {
      (AGENT_IFACE, "Release"): _ok,
      (AGENT_IFACE, "Cancel"): _ok,
      (AGENT_IFACE, "DisplayPinCode"): _ok,
      (AGENT_IFACE, "DisplayPasskey"): _ok,
      (AGENT_IFACE, "RequestConfirmation"): _ok,
      (AGENT_IFACE, "RequestAuthorization"): _ok,
      (AGENT_IFACE, "AuthorizeService"): _ok,
      (AGENT_IFACE, "RequestPinCode"): _pin,
      (AGENT_IFACE, "RequestPasskey"): _passkey,
    }
    agent = _Exported(self._agent_path, AGENT_XML, methods=methods)
    agent.register(self.bus)
    self._objects.append(agent)
    try:
      self.bus.call_sync(
        BLUEZ_SERVICE, "/org/bluez", AGENT_MANAGER_IFACE, "RegisterAgent",
        GLib.Variant("(os)", (self._agent_path, "NoInputNoOutput")),
        None, Gio.DBusCallFlags.NONE, 5000, None,
      )
      self.bus.call_sync(
        BLUEZ_SERVICE, "/org/bluez", AGENT_MANAGER_IFACE, "RequestDefaultAgent",
        GLib.Variant("(o)", (self._agent_path,)),
        None, Gio.DBusCallFlags.NONE, 5000, None,
      )
      self._agent_registered = True
      cloudlog.info("iqlink ble: pair agent registered (NoInputNoOutput)")
    except Exception as e:
      cloudlog.warning(f"iqlink ble: pair agent: {e}")
      self._agent_registered = False

  def _unregister_pair_agent(self):
    if not self._agent_registered or self.bus is None:
      return
    try:
      self.bus.call_sync(
        BLUEZ_SERVICE, "/org/bluez", AGENT_MANAGER_IFACE, "UnregisterAgent",
        GLib.Variant("(o)", (self._agent_path,)),
        None, Gio.DBusCallFlags.NONE, 3000, None,
      )
    except Exception:
      pass
    self._agent_registered = False

  def _mk_props(self, path, pmap):
    def get_prop(params):
      iface, name = params.unpack()
      getter = pmap.get(iface, {}).get(name)
      if getter is None:
        raise RuntimeError(f"unknown_property:{iface}.{name}")
      return GLib.Variant("(v)", (getter(),))

    def get_all(params):
      (iface,) = params.unpack()
      pm = pmap.get(iface)
      if pm is None:
        raise RuntimeError(f"unknown_interface:{iface}")
      return GLib.Variant("(a{sv})", ({k: g() for k, g in pm.items()},))

    return _Exported(path, PROPS_XML, methods={(DBUS_PROPS_IFACE, "Get"): get_prop, (DBUS_PROPS_IFACE, "GetAll"): get_all})

  def _register_objects(self):
    chars = [self._nav_path, self._status_path]
    service_props = {GATT_SERVICE_IFACE: {
      "UUID": lambda: _variant("s", IQLINK_SERVICE_UUID),
      "Primary": lambda: _variant("b", True),
      "Characteristics": lambda: _variant("ao", chars),
    }}
    adv_props = {LE_ADV_IFACE: {
      "Type": lambda: _variant("s", "peripheral"),
      "LocalName": lambda: _variant("s", self.local_name),
      "ServiceUUIDs": lambda: _variant("as", [IQLINK_SERVICE_UUID]),
    }}
    nav_flags = ["write", "write-without-response"]
    status_flags = ["notify"]

    app = _Exported(self._app_path, OM_XML, methods={(DBUS_OM_IFACE, "GetManagedObjects"): self._get_managed})
    service = _Exported(self._service_path, SERVICE_XML, properties=service_props)
    nav = self._mk_nav_char()
    status = self._mk_status_char()
    adv = _Exported(self._adv_path, ADV_XML, methods={(LE_ADV_IFACE, "Release"): lambda _: None}, properties=adv_props)

    self._objects = [
      app, self._mk_props(self._app_path, {}),
      service, self._mk_props(self._service_path, service_props),
      nav, self._mk_props(self._nav_path, self._char_props(IQLINK_NAV_CHAR_UUID, nav_flags, "nav")),
      status, self._mk_props(self._status_path, self._char_props(IQLINK_STATUS_CHAR_UUID, status_flags, "status")),
      adv, self._mk_props(self._adv_path, adv_props),
    ]
    for o in self._objects:
      o.register(self.bus)

  def _mk_nav_char(self):
    def read_value(_params):
      return GLib.Variant("(ay)", (list(self._values["nav"]),))

    def write_value(params):
      value, _options = params.unpack()
      chunk = bytes(int(x) & 0xFF for x in value) if isinstance(value, (list, tuple)) else bytes(value or b"")
      self._last_nav_rx_mono = time.monotonic()
      if self._link_state == LINK_OFF:
        cloudlog.info(f"iqlink ble: nav write {len(chunk)}B (first)")
      self._on_nav_bytes(chunk)
      return None

    return _Exported(self._nav_path, CHAR_XML, methods={
      (GATT_CHRC_IFACE, "ReadValue"): read_value,
      (GATT_CHRC_IFACE, "WriteValue"): write_value,
      (GATT_CHRC_IFACE, "StartNotify"): lambda _: None,
      (GATT_CHRC_IFACE, "StopNotify"): lambda _: None,
    }, properties=self._char_props(IQLINK_NAV_CHAR_UUID, ["write", "write-without-response"], "nav"))

  def _mk_status_char(self):
    def read_value(_params):
      return GLib.Variant("(ay)", (list(self._values["status"]),))

    def start_notify(_params):
      self._notify_status = True
      return None

    def stop_notify(_params):
      self._notify_status = False
      return None

    return _Exported(self._status_path, CHAR_XML, methods={
      (GATT_CHRC_IFACE, "ReadValue"): read_value,
      (GATT_CHRC_IFACE, "WriteValue"): lambda _: None,
      (GATT_CHRC_IFACE, "StartNotify"): start_notify,
      (GATT_CHRC_IFACE, "StopNotify"): stop_notify,
    }, properties=self._char_props(IQLINK_STATUS_CHAR_UUID, ["notify"], "status"))

  def _get_managed(self, _params):
    managed = {
      self._service_path: {GATT_SERVICE_IFACE: {
        "UUID": _variant("s", IQLINK_SERVICE_UUID), "Primary": _variant("b", True),
        "Characteristics": _variant("ao", [self._nav_path, self._status_path]),
      }},
      self._nav_path: {GATT_CHRC_IFACE: {
        "UUID": _variant("s", IQLINK_NAV_CHAR_UUID), "Service": _variant("o", self._service_path),
        "Flags": _variant("as", ["write", "write-without-response"]),
      }},
      self._status_path: {GATT_CHRC_IFACE: {
        "UUID": _variant("s", IQLINK_STATUS_CHAR_UUID), "Service": _variant("o", self._service_path),
        "Flags": _variant("as", ["notify"]),
      }},
    }
    return GLib.Variant("(a{oa{sa{sv}}})", (managed,))

  def _on_nav_bytes(self, chunk: bytes) -> None:
    if not chunk:
      return
    # GATT traffic seen — leave "寻找中" for connecting (HMAC still required for connected).
    if self._link_state == LINK_OFF:
      self._set_link_state(LINK_CONNECTING)
    # New envelope often starts with '{'; reset stale partial if so.
    if chunk[:1] == b"{" and self._nav_buf and not self._nav_buf.startswith(b"{"):
      self._nav_buf.clear()
    self._nav_buf.extend(chunk)
    if len(self._nav_buf) > MAX_ENVELOPE_BYTES:
      self._nav_buf.clear()
      cloudlog.warning("iqlink ble: envelope too large, dropped")
      return
    try:
      env = json.loads(self._nav_buf.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
      return
    self._nav_buf.clear()
    if not isinstance(env, dict):
      return
    # Coalesce: overwrite slot; single worker serializes verify/ingest (nav = latest only).
    self._env_slot.put(env)

  def _envelope_worker(self) -> None:
    while not self._env_worker_stop.is_set():
      env = self._env_slot.take()
      if env is None:
        self._env_slot.wait(0.5)
        continue
      self._handle_envelope(env)

  def _handle_envelope(self, env: dict[str, Any]) -> None:
    try:
      psk = ensure_ble_psk(self.params)
      data = verify_envelope(env, psk, self.seq_tracker)
      # Publish phone wall clock for timed (NTP/HTTP may fail on WWAN-only boot).
      try:
        with open("/dev/shm/iqlink_phone_ts_ms", "w", encoding="utf-8") as f:
          f.write(str(int(env["ts"])))
      except Exception:
        pass
      payload = extract_nav_payload(data)
      self.ingest_cb(payload)
      self._set_link_state(LINK_CONNECTED)
      self._notify_ok()
    except BleAuthError as e:
      cloudlog.warning(f"iqlink ble reject: {e}")
      # F4: bad HMAC / seq / skew must not leave UI stuck on yellow "connecting"
      self._recover_from_match_failure(reason=str(e))
    except Exception as e:
      cloudlog.warning(f"iqlink ble handle: {e}")

  def _notify_ok(self) -> None:
    if not self.running or self.bus is None or not self._notify_status:
      return
    payload = b'{"ok":true}'
    self._values["status"] = payload
    try:
      self.bus.emit_signal(
        None, self._status_path, DBUS_PROPS_IFACE, "PropertiesChanged",
        GLib.Variant("(sa{sv}as)", (GATT_CHRC_IFACE, {"Value": _variant("ay", list(payload))}, [])),
      )
    except Exception:
      pass

  def _set_link_state(self, state: int) -> None:
    state = int(state)
    if state == self._link_state:
      return
    prev = self._link_state
    self._link_state = state
    set_ble_link_state(self.params, state)
    label = {LINK_OFF: "off", LINK_CONNECTING: "connecting", LINK_CONNECTED: "connected"}.get(state, str(state))
    cloudlog.info(f"iqlink ble: link_state={label}")
    if state == LINK_CONNECTING:
      self._connecting_since_mono = time.monotonic()
    else:
      self._connecting_since_mono = 0.0
    if state == LINK_CONNECTED:
      self._link_connected_mono = time.monotonic()
    elif prev == LINK_CONNECTED:
      self._link_connected_mono = 0.0
    # Power: stop ADV when HMAC-connected. On disconnect, do NOT RegisterAdvertisement
    # on the GLib thread (call_sync can hang ~timeout and stall the main loop). Reconcile
    # in run_ble_gatt_loop re-ADVs after cooldown with short-timeout + backoff.
    if state == LINK_CONNECTED:
      set_ble_pair_failed(self.params, False)
      set_ble_discovering(self.params, False)
      try:
        self.stop_advertising()
      except Exception:
        pass
    elif state == LINK_CONNECTING:
      # Phone GATT writing — stop showing 寻找中 (PSK already entered on phone).
      set_ble_discovering(self.params, False)
    elif state == LINK_OFF and self.running and prev == LINK_CONNECTED:
      self._adv_cooldown_until = max(
        self._adv_cooldown_until, time.monotonic() + ADV_POST_UNREGISTER_DELAY_S,
      )

  def _recover_from_match_failure(self, *, reason: str) -> None:
    """Leave connecting, clear seq, disconnect — auto-retry (no sticky red / no manual toggle)."""
    cloudlog.warning(f"iqlink ble: match failure recover ({reason}) — auto retry")
    # Yellow reconnecting, not sticky red: phone + advertising retry without user tap.
    set_ble_pair_failed(self.params, False)
    set_ble_discovering(self.params, True)
    self.seq_tracker = SeqTracker()
    self._nav_buf.clear()
    self._last_nav_rx_mono = 0.0
    if self._link_state != LINK_OFF:
      self._set_link_state(LINK_OFF)
    try:
      self.disconnect_connected_peers()
    except Exception as e:
      cloudlog.warning(f"iqlink ble: match-fail Disconnect: {e}")

  def maybe_recover_stale_connecting(self) -> bool:
    """F4: connecting without HMAC within CONNECTING_TIMEOUT_S → off + pair_failed."""
    if not connecting_is_stale(
      link_state=self._link_state,
      connecting_since_mono=self._connecting_since_mono,
      now_mono=time.monotonic(),
    ):
      return False
    self._recover_from_match_failure(reason="connecting_timeout")
    return True

  def _set_connected(self, connected: bool) -> None:
    """SoftBus Connected → peer flag (UI green). HMAC LinkState survives brief SoftBus flaps."""
    set_ble_peer_connected(self.params, bool(connected))
    if connected:
      if self._peer_connected_mono <= 0:
        self._peer_connected_mono = time.monotonic()
      return
    self._peer_connected_mono = 0.0
    # Do not clear HMAC LinkState=2 on SoftBus falling edge if WriteValue / HMAC was recent.
    if softbus_down_should_clear_link(
      link_state=self._link_state,
      last_nav_rx_mono=self._last_nav_rx_mono,
      now_mono=time.monotonic(),
      link_connected_mono=self._link_connected_mono,
    ):
      self._set_link_state(LINK_OFF)
    else:
      cloudlog.info("iqlink ble: SoftBus down ignored (HMAC WriteValue recent)")

  def _refresh_connected(self) -> None:
    try:
      managed = self._managed_objects()
    except Exception:
      return
    self._set_connected(any_device_connected(managed, self.adapter_path))

  def disconnect_connected_peers(self) -> int:
    """Drop SoftBus LE links so phone rediscovers GATT (clears write Status=0 zombies)."""
    if self.bus is None or self.adapter_path is None:
      return 0
    # Match-fail clears _link_connected_mono first; zombie path must not kill fresh HMAC.
    if hmac_connect_is_fresh(link_connected_mono=self._link_connected_mono, now_mono=time.monotonic()):
      cloudlog.info("iqlink ble: skip Disconnect (HMAC connect fresh)")
      return 0
    try:
      paths = connected_device_paths(self._managed_objects(), self.adapter_path)
    except Exception:
      return 0
    n = 0
    for path in paths:
      # call_sync can process GLib; HMAC may land mid-loop — abort remaining.
      if hmac_connect_is_fresh(link_connected_mono=self._link_connected_mono, now_mono=time.monotonic()):
        cloudlog.info("iqlink ble: abort Disconnect (HMAC connect fresh)")
        break
      try:
        # Sync: async Disconnect often completes after phone already reconnected+HMAC
        # and then kills the new session (~1–3s later) → 30s zombie loop.
        self.bus.call_sync(
          BLUEZ_SERVICE, path, DEVICE_IFACE, "Disconnect", None,
          None, Gio.DBusCallFlags.NONE, 3000, None,
        )
        n += 1
      except Exception as e:
        cloudlog.warning(f"iqlink ble: Disconnect {path}: {e}")
    if n:
      self._peer_connected_mono = 0.0
      self._last_nav_rx_mono = 0.0
      self._peer_drop_cooldown_until = time.monotonic() + ZOMBIE_DROP_COOLDOWN_S
      self._adv_cooldown_until = max(self._adv_cooldown_until, self._peer_drop_cooldown_until)
      cloudlog.warning(f"iqlink ble: dropped {n} zombie peer(s)")
    return n

  def maybe_drop_zombie_peers(self) -> bool:
    """If SoftBus/HMAC session without recent nav WriteValue, demote + Disconnect.

    SoftBus may already be down (phone force-stop) while LinkState stayed 2 because
    SoftBus-down was ignored under WriteValue/HMAC grace — still demote so ADV resumes.
    """
    now = time.monotonic()
    if not peer_is_zombie(
      link_state=self._link_state,
      peer_connected_mono=self._peer_connected_mono,
      last_nav_rx_mono=self._last_nav_rx_mono,
      now_mono=now,
      link_connected_mono=self._link_connected_mono,
    ):
      return False
    # Disconnect first (refuses / aborts if HMAC becomes fresh). Demote even if no
    # peers left — otherwise LinkState=2 keeps ADV off after SoftBus-already-down.
    self.disconnect_connected_peers()
    if hmac_connect_is_fresh(link_connected_mono=self._link_connected_mono, now_mono=time.monotonic()):
      return False
    if self._link_state == LINK_CONNECTED:
      self._set_link_state(LINK_OFF)
      return True
    return False

  def _watch_connections(self) -> None:
    if self.bus is None:
      return
    self._refresh_connected()
    try:
      self._sig_ids.append(self.bus.signal_subscribe(
        BLUEZ_SERVICE, DBUS_PROPS_IFACE, "PropertiesChanged", None, None,
        Gio.DBusSignalFlags.NONE, self._on_props_changed, None,
      ))
      self._sig_ids.append(self.bus.signal_subscribe(
        BLUEZ_SERVICE, DBUS_OM_IFACE, "InterfacesAdded", None, None,
        Gio.DBusSignalFlags.NONE, self._on_ifaces_added, None,
      ))
      self._sig_ids.append(self.bus.signal_subscribe(
        BLUEZ_SERVICE, DBUS_OM_IFACE, "InterfacesRemoved", None, None,
        Gio.DBusSignalFlags.NONE, self._on_ifaces_removed, None,
      ))
    except Exception as e:
      cloudlog.warning(f"iqlink ble: connection watch failed: {e}")

  def _unwatch_connections(self) -> None:
    if self.bus is None:
      return
    for sid in self._sig_ids:
      try:
        self.bus.signal_unsubscribe(sid)
      except Exception:
        pass
    self._sig_ids.clear()

  def _on_props_changed(self, _conn, _sender, path, _iface, _signal, params, _ud):
    try:
      iface, changed, _invalidated = params.unpack()
    except Exception:
      return
    if iface != DEVICE_IFACE or "Connected" not in changed:
      return
    if self.adapter_path and not str(path).startswith(self.adapter_path.rstrip("/") + "/"):
      return
    if _variant_bool(changed.get("Connected")):
      # New SoftBus session: clear seq window so phone restart cannot seq_replay forever.
      self.seq_tracker = SeqTracker()
      self._set_connected(True)
    else:
      self._refresh_connected()

  def _on_ifaces_added(self, _conn, _sender, path, _iface, _signal, params, _ud):
    try:
      obj_path, ifaces = params.unpack()
    except Exception:
      return
    path = obj_path or path
    if DEVICE_IFACE not in ifaces:
      return
    if self.adapter_path and not str(path).startswith(self.adapter_path.rstrip("/") + "/"):
      return
    props = ifaces.get(DEVICE_IFACE) or {}
    if _variant_bool(props.get("Connected")):
      self.seq_tracker = SeqTracker()
      self._set_connected(True)

  def _on_ifaces_removed(self, _conn, _sender, path, _iface, _signal, params, _ud):
    try:
      obj_path, ifaces = params.unpack()
    except Exception:
      return
    path = obj_path or path
    if DEVICE_IFACE not in ifaces:
      return
    if self.adapter_path and not str(path).startswith(self.adapter_path.rstrip("/") + "/"):
      return
    self._refresh_connected()

  def _register_with_bluez(self):
    self.bus.call(
      BLUEZ_SERVICE, self.adapter_path, GATT_MANAGER_IFACE, "RegisterApplication",
      GLib.Variant("(oa{sv})", (self._app_path, {})), None, Gio.DBusCallFlags.NONE,
      BLUEZ_REGISTER_TIMEOUT_MS, None, self._on_app_registered, None,
    )

  def _on_app_registered(self, conn, result, _ud):
    try:
      (conn or self.bus).call_finish(result)
    except GLib.GError as e:
      if "AlreadyExists" not in str(e):
        self._error = e
        self._stop_on_loop()
        if self.loop:
          self.loop.quit()
        self._ready.set()
        return
    self.bus.call(
      BLUEZ_SERVICE, self.adapter_path, LE_ADV_MANAGER_IFACE, "RegisterAdvertisement",
      GLib.Variant("(oa{sv})", (self._adv_path, {})), None, Gio.DBusCallFlags.NONE,
      BLUEZ_REGISTER_TIMEOUT_MS, None, self._on_adv_registered, None,
    )

  def _on_adv_registered(self, conn, result, _ud):
    try:
      (conn or self.bus).call_finish(result)
    except GLib.GError as e:
      if "AlreadyExists" not in str(e):
        self._error = e
        self._stop_on_loop()
        if self.loop:
          self.loop.quit()
        self._ready.set()
        return
    self.running = True
    self._error = None
    self._note_adv_ok()
    self._ready.set()
    cloudlog.info(f"iqlink ble: advertising as {self.local_name!r}")

  def _note_adv_ok(self) -> None:
    self._adv_registered = True
    self._adv_fail_count = 0
    self._adv_fail_since_mono = 0.0
    self._adv_next_try_mono = 0.0

  def _note_adv_failure(self, err: str) -> None:
    """Clear registered flag and schedule backoff retry (reconcile must keep trying)."""
    self._adv_registered = False
    now = time.monotonic()
    if self._adv_fail_since_mono <= 0:
      self._adv_fail_since_mono = now
    self._adv_fail_count += 1
    delay = next_adv_retry_delay(self._adv_fail_count)
    self._adv_next_try_mono = now + delay
    cloudlog.warning(f"iqlink ble: start_advertising: {err} (retry in {delay:.0f}s)")

  def adv_needs_gatt_recover(self) -> bool:
    """True if re-adv has been failing long enough to restart the GATT app."""
    if self._adv_registered:
      return False
    # SoftBus-down + LinkState=2 still wants ADV — allow recover if register keeps failing.
    if not ble_should_advertise(
      enabled=True, link_state=self._link_state, peer_connected=self._peer_connected_mono > 0,
    ):
      return False
    if self._adv_fail_since_mono <= 0:
      return False
    return (time.monotonic() - self._adv_fail_since_mono) >= ADV_FAIL_RECOVER_S

  def start_advertising(self) -> None:
    """Register LE advertisement. No-op if registered / cooling down / backoff."""
    if not self.running or self.bus is None or self.adapter_path is None:
      return
    if self._adv_registered:
      return
    now = time.monotonic()
    if now < self._adv_cooldown_until or now < self._adv_next_try_mono:
      return
    try:
      # Short sync timeout from poll thread only — never from GLib disconnect path.
      self.bus.call_sync(
        BLUEZ_SERVICE, self.adapter_path, LE_ADV_MANAGER_IFACE, "RegisterAdvertisement",
        GLib.Variant("(oa{sv})", (self._adv_path, {})), None, Gio.DBusCallFlags.NONE,
        ADV_REGISTER_TIMEOUT_MS, None,
      )
      self._note_adv_ok()
      cloudlog.info(f"iqlink ble: advertising as {self.local_name!r}")
    except Exception as e:
      if "AlreadyExists" in str(e):
        self._note_adv_ok()
        return
      self._note_adv_failure(str(e))

  def stop_advertising(self) -> None:
    """Unregister LE advertisement only; GATT app stays for bonded reconnects."""
    if self.bus is None or self.adapter_path is None:
      self._adv_registered = False
      return
    if not self._adv_registered:
      return
    try:
      self.bus.call_sync(
        BLUEZ_SERVICE, self.adapter_path, LE_ADV_MANAGER_IFACE, "UnregisterAdvertisement",
        GLib.Variant("(o)", (self._adv_path,)), None, Gio.DBusCallFlags.NONE,
        ADV_UNREGISTER_TIMEOUT_MS, None,
      )
    except Exception:
      pass
    self._adv_registered = False
    self._adv_cooldown_until = time.monotonic() + ADV_POST_UNREGISTER_DELAY_S
    set_ble_discovering(self.params, False)
    cloudlog.info("iqlink ble: advertising stopped")

  def _clear_registrations(self):
    self._unregister_pair_agent()
    if self.bus is None or self.adapter_path is None:
      return
    for iface, method, arg in (
      (LE_ADV_MANAGER_IFACE, "UnregisterAdvertisement", self._adv_path),
      (GATT_MANAGER_IFACE, "UnregisterApplication", self._app_path),
    ):
      try:
        self.bus.call_sync(
          BLUEZ_SERVICE, self.adapter_path, iface, method, GLib.Variant("(o)", (arg,)),
          None, Gio.DBusCallFlags.NONE, 3000, None,
        )
      except Exception:
        pass
    self._adv_registered = False

  def _stop_on_loop(self):
    self._unwatch_connections()
    if self.bus is not None and self.adapter_path is not None:
      self._clear_registrations()
    if self.bus is not None:
      for o in reversed(self._objects):
        try:
          o.unregister(self.bus)
        except Exception:
          pass
    self._objects.clear()
    self.running = False
    self._adv_registered = False
    set_ble_discovering(self.params, False)
    set_ble_peer_connected(self.params, False)
    self._set_link_state(LINK_OFF)
    if self.loop:
      try:
        self.loop.quit()
      except Exception:
        pass


def run_ble_gatt_loop(ingest_cb: Callable[[dict[str, Any]], None]) -> None:
  """GATT while IqlinkEnabled.

  Discover/PSK UI (`IqlinkBleDiscovering`) lasts ADV_WINDOW_S after enable rising edge only.
  Connectable ADV while enabled unless LinkState=2 *and* SoftBus peer is up (power). SoftBus
  flap with LinkState=2 keeps ADV so phone remounts without demoting. Long WriteValue stale
  still demotes (SoftBus-down uses SOFTBUS_DOWN_ZOMBIE_S).
  """
  if not _GI_AVAILABLE:
    cloudlog.warning("iqlink ble: gi/BlueZ unavailable — BLE off")
    return

  params = Params()
  set_ble_link_state(params, LINK_OFF)
  set_ble_discovering(params, False)
  set_ble_pair_failed(params, False)
  set_ble_peer_connected(params, False)
  server: IqlinkBleGatt | None = None
  was_enabled = False
  discover_deadline = 0.0
  while True:
    try:
      enabled = params.get_bool("IqlinkEnabled")
    except Exception:
      enabled = False

    # Rising edge: open 2-minute discover/PSK UI window (advertising is separate)
    if enabled and not was_enabled:
      discover_deadline = time.monotonic() + ADV_WINDOW_S
      set_ble_discovering(params, True)
      set_ble_pair_failed(params, False)
      cloudlog.info(f"iqlink ble: discover window {ADV_WINDOW_S:.0f}s")

    if enabled and (server is None or not server.running):
      try:
        ensure_ble_psk(params)
        server = IqlinkBleGatt(ingest_cb)
        server.start(timeout_s=30.0)
        # Ensure discover flag if we (re)started mid-window
        if time.monotonic() < discover_deadline:
          set_ble_discovering(params, True)
      except Exception as e:
        cloudlog.warning(f"iqlink ble: start failed: {e}")
        server = None
        set_ble_link_state(params, LINK_OFF)
        time.sleep(5.0)
        was_enabled = enabled
        continue

    if enabled and server is not None and server.running:
      # End discover/PSK UI: deadline OR already HMAC-connected
      if time.monotonic() >= discover_deadline or server._link_state == LINK_CONNECTED:
        set_ble_discovering(params, False)
      # Nuclear: re-adv stuck for ADV_FAIL_RECOVER_S → restart GATT (better than forever)
      if server.adv_needs_gatt_recover():
        cloudlog.warning("iqlink ble: advertising stuck — restarting GATT")
        try:
          server.stop()
        except Exception:
          pass
        server = None
        was_enabled = enabled
        continue
      # F4: connecting stuck (no HMAC) → pair_failed + off + drop zombies
      try:
        if server.maybe_recover_stale_connecting():
          # Brief rediscover so PSK shows again after match failure
          discover_deadline = time.monotonic() + min(ADV_WINDOW_S, 60.0)
          set_ble_discovering(params, True)
      except Exception as e:
        cloudlog.warning(f"iqlink ble: connecting recover: {e}")
      # Phone ATT "write Status=0" zombie: SoftBus up, our WriteValue never fires → kick peer
      try:
        server.maybe_drop_zombie_peers()
      except Exception as e:
        cloudlog.warning(f"iqlink ble: zombie drop: {e}")
      # Reconcile ADV: SoftBus-up+HMAC → off; SoftBus-down or link!=2 → on (no demote needed)
      try:
        want_adv = ble_should_advertise(
          enabled=True,
          link_state=server._link_state,
          peer_connected=server._peer_connected_mono > 0,
        )
        if want_adv:
          if not server._adv_registered:
            server.start_advertising()
        elif server._adv_registered:
          server.stop_advertising()
      except Exception as e:
        cloudlog.warning(f"iqlink ble: advertising reconcile: {e}")

    if not enabled and server is not None:
      try:
        server.stop()
      except Exception:
        pass
      server = None
      set_ble_link_state(params, LINK_OFF)
      set_ble_discovering(params, False)
      set_ble_pair_failed(params, False)
      set_ble_peer_connected(params, False)
      cloudlog.info("iqlink ble: stopped (IqlinkEnabled=0)")

    was_enabled = enabled
    time.sleep(ENABLED_POLL_S)
