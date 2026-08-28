import os
import threading

# IQ.Pilot ships a prebuilt params_pyx.so without Iqlink* keys. Python iqlinkd/UI
# store those keys as files beside the native params directory.
IQLINK_PARAM_DEFAULTS = {
  "IqlinkEnabled": "1",
  "IqlinkExclusive": "0",
  "IqlinkLinkWarn": "0",
  "IqlinkSsidWhitelist": None,
  "IqlinkWarnTimeoutS": "5",
  "IqlinkCancelTimeoutS": "5",
  "IqlinkHotspotSsid": "comma-cplink",
  "IqlinkPinnedGaodeVersion": None,
  "IqlinkPinnedCpVersion": None,
  "IqlinkAggressiveLaneChange": "1",
  "IqlinkBlePsk": "999999",
  "IqlinkBleLinkState": "0",
  "IqlinkBleDiscovering": "0",
  "IqlinkBleConnected": "0",
  "IqlinkBlePeerConnected": "0",
  "IqlinkBlePairFailed": "0",
  "IqlinkProductCruiseDefaultsV1": "0",
}
IQLINK_BOOL_KEYS = {
  "IqlinkEnabled", "IqlinkExclusive", "IqlinkLinkWarn", "IqlinkAggressiveLaneChange",
  "IqlinkBleDiscovering", "IqlinkBleConnected", "IqlinkBlePeerConnected",
  "IqlinkBlePairFailed", "IqlinkProductCruiseDefaultsV1",
}
IQLINK_INT_KEYS = {"IqlinkBleLinkState"}

# Keys not in the prebuilt params_pyx.so. Default ON: hold gas raises MAX.
# IQTrafficStopOffset: meters short of a vision red/model stop (not follow gap).
# Stored as a float so the UI can step 0.5 m. Old integer files ("3") still read as 3.0.
# IQLeadStopDistance: parked follow gap behind a stopped lead (Lead MPC STOP_DISTANCE).
EXTRA_PARAM_DEFAULTS = {
  "AutoGasSyncSpeed": "1",
  "IQTrafficStopOffset": "3",
  "IQLeadStopDistance": "4",
  "IQNavSoftCurveCap": "1",
  "IQNavLaneGuide": "1",
  # EcoFlow Delta 3 12V DC (ecoflowd). Default OFF — enable after setting credentials.
  "EcoflowEnabled": "0",
  "EcoflowPhone": None,
  "EcoflowEmail": None,
  "EcoflowPassword": None,
  "EcoflowSn": None,
  "EcoflowApiBase": None,
}
EXTRA_BOOL_KEYS = {
  "AutoGasSyncSpeed",
  "EcoflowEnabled",
  "IQNavSoftCurveCap",
  "IQNavLaneGuide",
}
EXTRA_FLOAT_KEYS = {
  "IQTrafficStopOffset",
  "IQLeadStopDistance",
}


def _iqlink_key_name(key):
  return key.decode() if isinstance(key, bytes) else key


def _iqlink_is_extra(key) -> bool:
  name = _iqlink_key_name(key)
  return name in IQLINK_PARAM_DEFAULTS or name in EXTRA_PARAM_DEFAULTS


def _extra_default(name):
  if name in EXTRA_PARAM_DEFAULTS:
    return EXTRA_PARAM_DEFAULTS[name]
  return IQLINK_PARAM_DEFAULTS.get(name)


def _iqlink_decode(key, dat, encoding=None):
  name = _iqlink_key_name(key)
  if dat is None:
    default = _extra_default(name)
    if name in IQLINK_BOOL_KEYS or name in EXTRA_BOOL_KEYS:
      return default == "1"
    if name in IQLINK_INT_KEYS:
      return int(default or 0)
    if name in EXTRA_FLOAT_KEYS:
      return float(default or 0)
    return default
  if encoding is not None:
    text = dat.decode(encoding) if isinstance(dat, (bytes, bytearray)) else str(dat)
  else:
    try:
      text = dat.decode("utf-8") if isinstance(dat, (bytes, bytearray)) else str(dat)
    except UnicodeDecodeError:
      return dat
  if name in IQLINK_BOOL_KEYS or name in EXTRA_BOOL_KEYS:
    return text == "1"
  if name in IQLINK_INT_KEYS:
    try:
      return int(text)
    except (TypeError, ValueError):
      return 0
  if name in EXTRA_FLOAT_KEYS:
    try:
      return float(text)
    except (TypeError, ValueError):
      return 0.0
  return text


def _iqlink_encode(key, dat) -> bytes:
  name = _iqlink_key_name(key)
  if name in IQLINK_BOOL_KEYS or name in EXTRA_BOOL_KEYS:
    if isinstance(dat, bool):
      return b"1" if dat else b"0"
    if dat in (b"1", b"0"):
      return dat
    return b"1" if str(dat) in ("1", "True", "true") else b"0"
  if isinstance(dat, bytes):
    return dat
  return str(dat).encode("utf-8")


def _iqlink_write(path: str, dat: bytes) -> None:
  os.makedirs(os.path.dirname(path), exist_ok=True)
  tmp = path + ".tmp"
  with open(tmp, "wb") as f:
    f.write(dat)
    f.flush()
    os.fsync(f.fileno())
  os.rename(tmp, path)


def _iqlink_read(path: str):
  try:
    with open(path, "rb") as f:
      return f.read()
  except (FileNotFoundError, NotADirectoryError, IsADirectoryError):
    return None

try:
  from openpilot.common.params_pyx import Params as _NativeParams, ParamKeyFlag, ParamKeyType, UnknownKeyName

  class Params(_NativeParams):
    def _extra_path(self, key) -> str:
      name = _iqlink_key_name(key)
      # /data/params/d is rebuilt on boot; only native PERSISTENT keys return.
      # Keep Ecoflow* outside that overlay so credentials survive reboot.
      if name.startswith("Ecoflow"):
        return os.path.join("/data/ecoflow_params", name)
      return os.path.join(self.get_param_path(""), name)

    def check_key(self, key):
      if _iqlink_is_extra(key):
        return key.encode() if isinstance(key, str) else key
      return super().check_key(key)

    def get(self, key, block: bool = False, return_default: bool = False, encoding=None):
      if _iqlink_is_extra(key):
        dat = _iqlink_read(self._extra_path(key))
        if dat is None and not return_default:
          # Match native missing-file behavior for most keys, but IqlinkEnabled
          # defaults ON like the iq-link product key.
          if _iqlink_key_name(key) == "IqlinkEnabled":
            return _iqlink_decode(key, None, encoding)
          return None
        return _iqlink_decode(key, dat, encoding)
      return super().get(key, block=block, return_default=return_default, encoding=encoding)

    def get_bool(self, key, block: bool = False) -> bool:
      if _iqlink_is_extra(key):
        val = self.get(key, block=block, return_default=True)
        return bool(val)
      return super().get_bool(key, block=block)

    def put(self, key, dat):
      if _iqlink_is_extra(key):
        _iqlink_write(self._extra_path(key), _iqlink_encode(key, dat))
        return
      return super().put(key, dat)

    def put_bool(self, key, val: bool):
      if _iqlink_is_extra(key):
        self.put(key, bool(val))
        return
      return super().put_bool(key, val)

    def put_nonblocking(self, key, dat):
      if _iqlink_is_extra(key):
        self.put(key, dat)
        return
      return super().put_nonblocking(key, dat)

    def put_bool_nonblocking(self, key, val: bool):
      if _iqlink_is_extra(key):
        self.put_bool(key, val)
        return
      return super().put_bool_nonblocking(key, val)

    def remove(self, key):
      if _iqlink_is_extra(key):
        try:
          os.remove(self._extra_path(key))
        except FileNotFoundError:
          pass
        return
      return super().remove(key)

except ImportError:
  from enum import IntEnum, IntFlag

  class UnknownKeyName(Exception):
    pass

  class ParamKeyFlag(IntFlag):
    # must stay in lockstep with enum ParamKeyFlag in common/params.h
    PERSISTENT = 0x02
    CLEAR_ON_MANAGER_START = 0x04
    CLEAR_ON_ONROAD_TRANSITION = 0x08
    CLEAR_ON_OFFROAD_TRANSITION = 0x10
    DONT_LOG = 0x20
    DEVELOPMENT_ONLY = 0x40
    CLEAR_ON_IGNITION_ON = 0x80
    ALL = 0xFFFFFFFF

  class ParamKeyType(IntEnum):
    STRING = 0
    BOOL = 1
    INT = 2
    FLOAT = 3
    TIME = 4
    JSON = 5
    BYTES = 6

  class Params:
    def __init__(self, path: str = ""):
      root = path or os.environ.get("PARAMS_ROOT", "/data/params")
      # keys live under <root>/d (comma.sh sets up the d -> d_tmp symlink on a fresh boot)
      self._d = os.path.join(root, "d")
      self._lock = threading.Lock()

    def _p(self, key):
      if isinstance(key, bytes):
        key = key.decode()
      return os.path.join(self._d, key)

    def check_key(self, key):
      return True

    def get(self, key, block: bool = False, return_default: bool = False, encoding=None):
      dat = _iqlink_read(self._p(key))
      if dat is None:
        if _iqlink_is_extra(key) and (return_default or _iqlink_key_name(key) == "IqlinkEnabled"):
          return _iqlink_decode(key, None, encoding)
        return None
      if _iqlink_is_extra(key):
        return _iqlink_decode(key, dat, encoding)
      if encoding is not None:
        return dat.decode(encoding)
      try:
        return dat.decode("utf-8")
      except UnicodeDecodeError:
        return dat

    def get_bool(self, key, block: bool = False) -> bool:
      if _iqlink_is_extra(key):
        return bool(self.get(key, block=block, return_default=True))
      try:
        with open(self._p(key), "rb") as f:
          return f.read() == b"1"
      except (FileNotFoundError, NotADirectoryError, IsADirectoryError):
        return False

    def get_int(self, key, block: bool = False) -> int:
      value = self.get(key, block=block)
      return int(value) if value else 0

    def get_float(self, key, block: bool = False) -> float:
      value = self.get(key, block=block)
      return float(value) if value else 0.0

    def put(self, key, dat):
      if _iqlink_is_extra(key):
        dat = _iqlink_encode(key, dat)
      elif isinstance(dat, str):
        dat = dat.encode("utf-8")
      with self._lock:
        _iqlink_write(self._p(key), dat)

    def put_bool(self, key, val: bool):
      self.put(key, b"1" if val else b"0")

    def put_int(self, key, val: int):
      self.put(key, str(val))

    def put_float(self, key, val: float):
      self.put(key, str(val))

    def put_nonblocking(self, key, dat):
      self.put(key, dat)

    def put_bool_nonblocking(self, key, val: bool):
      self.put_bool(key, val)

    def put_int_nonblocking(self, key, val: int):
      self.put_int(key, val)

    def put_float_nonblocking(self, key, val: float):
      self.put_float(key, val)

    def remove(self, key):
      try:
        os.remove(self._p(key))
      except FileNotFoundError:
        pass

    def clear_all(self, tx_type=None):
      pass

    def get_param_path(self, key: str = "") -> str:
      return self._p(key) if key else self._d

    def all_keys(self):
      try:
        return [k.encode() for k in os.listdir(self._d)]
      except FileNotFoundError:
        return []

assert Params
assert ParamKeyFlag
assert ParamKeyType
assert UnknownKeyName

if __name__ == "__main__":
  import sys

  params = Params()
  key = sys.argv[1]
  assert params.check_key(key), f"unknown param: {key}"

  if len(sys.argv) == 3:
    val = sys.argv[2]
    print(f"SET: {key} = {val}")
    params.put(key, val)
  elif len(sys.argv) == 2:
    print(f"GET: {key} = {params.get(key)}")
