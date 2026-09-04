"""BlueZ LE GATT for IQ-link phone UUIDs (PROTOCOL.md). Compact, not an IQ.Pilot port."""
from __future__ import annotations

import json
import threading
import time
from typing import Callable

IQLINK_SERVICE_UUID = "73f2c710-5e40-4d0d-8b7f-fde61f729100"
IQLINK_NAV_CHAR_UUID = "73f2c711-5e40-4d0d-8b7f-fde61f729100"
IQLINK_STATUS_CHAR_UUID = "73f2c712-5e40-4d0d-8b7f-fde61f729100"
ADV_PATH = "/org/bluez/iqlink/advertisement0"
APP_PATH = "/org/bluez/iqlink"
SVC_PATH = "/org/bluez/iqlink/service0"
NAV_PATH = "/org/bluez/iqlink/service0/char0"
STATUS_PATH = "/org/bluez/iqlink/service0/char1"


def _log(msg: str) -> None:
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.info(msg)
  except Exception:
    print(msg)


class BleGattServer:
  def __init__(self, on_write: Callable[[bytes], None]):
    self.on_write = on_write
    self._thread: threading.Thread | None = None
    self._stop = threading.Event()
    self._loop = None
    self.running = False

  def start(self) -> bool:
    if self._thread and self._thread.is_alive():
      return True
    self._stop.clear()
    self._thread = threading.Thread(target=self._run, name="iqlink-ble", daemon=True)
    self._thread.start()
    for _ in range(40):
      if self.running or self._stop.is_set():
        break
      time.sleep(0.05)
    return self.running

  def stop(self) -> None:
    self._stop.set()
    loop = self._loop
    if loop is not None:
      try:
        loop.quit()
      except Exception:
        pass
    if self._thread is not None:
      self._thread.join(timeout=2.0)
    self.running = False

  def notify_ok(self) -> None:
    # Status characteristic Notify is best-effort; phone accepts missing notify.
    pass

  def _run(self) -> None:
    try:
      self._run_bluez()
    except Exception:
      _log("iqlink BLE GATT failed to start")
      self.running = False

  def _run_bluez(self) -> None:
    import dbus
    import dbus.mainloop.glib
    import dbus.service
    from gi.repository import GLib

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    adapter = _find_adapter(bus)
    if adapter is None:
      _log("iqlink: no BlueZ adapter")
      return

    class Advertisement(dbus.service.Object):
      def __init__(self):
        super().__init__(bus, ADV_PATH)

      @dbus.service.method("org.freedesktop.DBus.Properties", in_signature="s", out_signature="a{sv}")
      def GetAll(self, interface):
        return {
          "Type": "peripheral",
          "ServiceUUIDs": dbus.Array([IQLINK_SERVICE_UUID], signature="s"),
          "LocalName": "IQ-link",
          "IncludeTxPower": dbus.Boolean(True),
        }

      @dbus.service.method("org.bluez.LEAdvertisement1", in_signature="", out_signature="")
      def Release(self):
        pass

    class Characteristic(dbus.service.Object):
      def __init__(self, path, uuid, flags, write_cb=None):
        super().__init__(bus, path)
        self.uuid = uuid
        self.flags = flags
        self.write_cb = write_cb
        self.value = dbus.Array([], signature="y")

      @dbus.service.method("org.freedesktop.DBus.Properties", in_signature="ss", out_signature="v")
      def Get(self, interface, prop):
        if prop == "UUID":
          return self.uuid
        if prop == "Service":
          return dbus.ObjectPath(SVC_PATH)
        if prop == "Flags":
          return dbus.Array(self.flags, signature="s")
        if prop == "Value":
          return self.value
        raise dbus.exceptions.DBusException("unknown")

      @dbus.service.method("org.freedesktop.DBus.Properties", in_signature="s", out_signature="a{sv}")
      def GetAll(self, interface):
        return {
          "UUID": self.uuid,
          "Service": dbus.ObjectPath(SVC_PATH),
          "Flags": dbus.Array(self.flags, signature="s"),
        }

      @dbus.service.method("org.bluez.GattCharacteristic1", in_signature="a{sv}", out_signature="ay")
      def ReadValue(self, options):
        if self.uuid == IQLINK_STATUS_CHAR_UUID:
          payload = json.dumps({"ok": True, "t": int(time.time() * 1000)}).encode()
          return dbus.Array(payload, signature="y")
        return self.value

      @dbus.service.method("org.bluez.GattCharacteristic1", in_signature="aya{sv}", out_signature="")
      def WriteValue(self, value, options):
        raw = bytes(bytearray(value))
        if self.write_cb is not None:
          self.write_cb(raw)

      @dbus.service.method("org.bluez.GattCharacteristic1", in_signature="", out_signature="")
      def StartNotify(self):
        pass

      @dbus.service.method("org.bluez.GattCharacteristic1", in_signature="", out_signature="")
      def StopNotify(self):
        pass

    class Service(dbus.service.Object):
      def __init__(self):
        super().__init__(bus, SVC_PATH)

      @dbus.service.method("org.freedesktop.DBus.Properties", in_signature="s", out_signature="a{sv}")
      def GetAll(self, interface):
        return {
          "UUID": IQLINK_SERVICE_UUID,
          "Primary": dbus.Boolean(True),
          "Characteristics": dbus.Array([
            dbus.ObjectPath(NAV_PATH),
            dbus.ObjectPath(STATUS_PATH),
          ], signature="o"),
        }

    class Application(dbus.service.Object):
      def __init__(self):
        super().__init__(bus, APP_PATH)

      @dbus.service.method("org.freedesktop.DBus.ObjectManager", out_signature="a{oa{sa{sv}}}")
      def GetManagedObjects(self):
        return {
          SVC_PATH: {"org.bluez.GattService1": {
            "UUID": IQLINK_SERVICE_UUID,
            "Primary": dbus.Boolean(True),
          }},
          NAV_PATH: {"org.bluez.GattCharacteristic1": {
            "UUID": IQLINK_NAV_CHAR_UUID,
            "Service": dbus.ObjectPath(SVC_PATH),
            "Flags": dbus.Array(["write", "write-without-response"], signature="s"),
          }},
          STATUS_PATH: {"org.bluez.GattCharacteristic1": {
            "UUID": IQLINK_STATUS_CHAR_UUID,
            "Service": dbus.ObjectPath(SVC_PATH),
            "Flags": dbus.Array(["read", "notify"], signature="s"),
          }},
        }

    _adv = Advertisement()
    _app = Application()
    _svc = Service()
    _nav = Characteristic(NAV_PATH, IQLINK_NAV_CHAR_UUID, ["write", "write-without-response"], self.on_write)
    _status = Characteristic(STATUS_PATH, IQLINK_STATUS_CHAR_UUID, ["read", "notify"])

    gatt = dbus.Interface(bus.get_object("org.bluez", adapter), "org.bluez.GattManager1")
    adv_mgr = dbus.Interface(bus.get_object("org.bluez", adapter), "org.bluez.LEAdvertisingManager1")
    gatt.RegisterApplication(APP_PATH, {}, reply_handler=lambda: None,
                             error_handler=lambda e: _log(f"iqlink RegisterApplication: {e}"))
    adv_mgr.RegisterAdvertisement(ADV_PATH, {}, reply_handler=lambda: None,
                                  error_handler=lambda e: _log(f"iqlink RegisterAdvertisement: {e}"))

    loop = GLib.MainLoop()
    self._loop = loop
    self.running = True
    _log("iqlink BLE GATT advertising")
    while not self._stop.is_set():
      try:
        GLib.MainContext.default().iteration(True)
      except Exception:
        break
    try:
      adv_mgr.UnregisterAdvertisement(ADV_PATH)
    except Exception:
      pass
    self.running = False


def _find_adapter(bus) -> str | None:
  import dbus
  om = dbus.Interface(bus.get_object("org.bluez", "/"), "org.freedesktop.DBus.ObjectManager")
  objects = om.GetManagedObjects()
  for path, ifaces in objects.items():
    if "org.bluez.GattManager1" in ifaces and "org.bluez.LEAdvertisingManager1" in ifaces:
      return str(path)
  return None
