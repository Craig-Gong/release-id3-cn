"""EcoFlow Delta 3 cloud client (App-login MQTT).

P231 (classic Delta 3) uses protobuf SET (tolwi hassio-ecoflow-cloud):
  Delta3SetCommand.cfg_dc12v_out_open = 18 → wrapped in Delta3SendHeaderMsg
  cmd_func=254, cmd_id=17 → MQTT /app/{userId}/{sn}/thing/property/set

Credentials: Params (EcoflowPhone/Password/Sn) or ECOFLOW_* env. Never commit secrets.
"""
from __future__ import annotations

import base64
import json
import os
import random
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

DEFAULT_API_BASE = "https://api.ecoflow.com"
CN_API_BASE = "https://api-cn.ecoflow.com"  # China App Store / phone accounts
USER_AGENT = "okhttp/3.14.9"

DC_MODE_PROTO = "proto"              # P231 / classic Delta 3 (protobuf)
DC_MODE_MPPT_CAR = "mpptCar"         # D361 JSON live-tested
DC_MODE_CFG = "cfgDc12vOutOpen"      # D361 JSON ioBroker fallback

# HA sniff: flow_info_12v == 14 → DC on, == 4 → off
_FLOW_DC_ON = 14
_FLOW_DC_OFF = 4


def _load_local_env_files() -> None:
  """Load gitignored ecoflow.env into os.environ if keys are missing. Never commit these files."""
  candidates = [
    Path(__file__).resolve().parent / "ecoflow.env",
    Path(__file__).resolve().parent / ".env",
    Path("/data/ecoflow.env"),
    Path("/data/params/ecoflow.env"),
  ]
  # Monorepo: iq-pilot-id3/demos/ecoflow/ecoflow.env when developing on Mac
  try:
    repo_demos = Path(__file__).resolve().parents[4] / "demos" / "ecoflow" / "ecoflow.env"
    candidates.append(repo_demos)
  except IndexError:
    pass
  for path in candidates:
    if not path.is_file():
      continue
    try:
      for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
          continue
        key, _, val = line.partition("=")
        key = key.strip()
        val = val.strip().strip("'").strip('"')
        if key and key not in os.environ:
          os.environ[key] = val
    except OSError:
      continue
    break


class EcoflowError(RuntimeError):
  pass


def _looks_like_phone(value: str) -> bool:
  s = value.strip().replace(" ", "").replace("-", "")
  if s.startswith("+"):
    s = s[1:]
  return s.isdigit() and 8 <= len(s) <= 15


def _normalize_cn_phone(value: str) -> str:
  """api-cn (ha-ef-ble PR#335): phone WITHOUT +86 prefix, e.g. 15589967080."""
  s = value.strip().replace(" ", "").replace("-", "")
  if s.startswith("+86"):
    return s[3:]
  if s.startswith("86") and len(s) >= 13 and s[2:].isdigit():
    return s[2:]
  if s.startswith("+"):
    return s[1:]
  return s


def _http_json(method: str, url: str, headers: dict[str, str], body: dict | None = None) -> dict:
  data = None if body is None else json.dumps(body).encode()
  req = urllib.request.Request(url, data=data, headers=headers, method=method)
  try:
    with urllib.request.urlopen(req, timeout=30) as resp:
      payload = json.loads(resp.read().decode())
  except urllib.error.HTTPError as e:
    raise EcoflowError(f"HTTP {e.code} {url}: {e.read()[:400]!r}") from e
  except urllib.error.URLError as e:
    raise EcoflowError(f"network error {url}: {e}") from e
  if str(payload.get("code")) not in ("0", "0.0"):
    raise EcoflowError(f"API error {url}: {payload}")
  return payload


def _import_delta3_pb2():
  # demos/ecoflow/proto/ef_delta3_pb2.py (tolwi generated)
  root = Path(__file__).resolve().parent
  if str(root) not in sys.path:
    sys.path.insert(0, str(root))
  try:
    from proto import ef_delta3_pb2 as pb2  # type: ignore
  except ImportError as e:
    raise EcoflowError(
      "Need protobuf + demos/ecoflow/proto/ef_delta3_pb2.py "
      "(pip install 'protobuf>=5,<7')"
    ) from e
  return pb2


def _gen_seq() -> int:
  return 999900000 + random.randint(10000, 99999)


def _derive_dc_from_flow(telemetry: dict[str, Any]) -> None:
  flow = telemetry.get("flow_info_12v")
  if flow == _FLOW_DC_ON:
    telemetry["cfg_dc12v_out_open"] = 1
  elif flow == _FLOW_DC_OFF:
    telemetry["cfg_dc12v_out_open"] = 0


@dataclass
class EcoflowSession:
  api_base: str
  account: str
  password: str
  sn: str
  use_phone: bool = False
  token: str = ""
  user_id: str = ""
  mqtt_url: str = ""
  mqtt_port: int = 8883
  mqtt_user: str = ""
  mqtt_pass: str = ""
  _seq: int = field(default=3000, repr=False)
  _telemetry: dict[str, Any] = field(default_factory=dict, repr=False)
  _ack: dict | None = field(default=None, repr=False)
  _ack_event: threading.Event = field(default_factory=threading.Event, repr=False)
  _client: Any = field(default=None, repr=False)
  _pb2: Any = field(default=None, repr=False)

  @classmethod
  def from_env(cls) -> "EcoflowSession":
    _load_local_env_files()
    phone = os.environ.get("ECOFLOW_PHONE", "").strip()
    email = os.environ.get("ECOFLOW_EMAIL", "").strip()
    password = os.environ.get("ECOFLOW_PASSWORD", "").strip()
    sn = os.environ.get("ECOFLOW_SN", "").strip()
    api_base_env = os.environ.get("ECOFLOW_API_BASE", "").strip().rstrip("/")

    if phone:
      account = _normalize_cn_phone(phone)
      use_phone = True
    elif email and _looks_like_phone(email):
      account = _normalize_cn_phone(email)
      use_phone = True
    else:
      account = email
      use_phone = False

    if use_phone:
      api_base = api_base_env or CN_API_BASE
    else:
      api_base = api_base_env or DEFAULT_API_BASE

    if not account or not password or not sn:
      raise EcoflowError(
        "Set ECOFLOW_PASSWORD, ECOFLOW_SN, and either:\n"
        "  ECOFLOW_PHONE='13800138000'   # China App (default api-cn.ecoflow.com)\n"
        "  or ECOFLOW_EMAIL='you@ex.com' # international App\n"
        "Optional ECOFLOW_API_BASE to override."
      )
    return cls(api_base=api_base, account=account, password=password, sn=sn, use_phone=use_phone)

  @classmethod
  def from_params(cls, params: Any = None) -> "EcoflowSession":
    """Load credentials from Params first, then fall back to ECOFLOW_* env."""
    if params is None:
      try:
        from openpilot.common.params import Params
        params = Params()
      except Exception:
        params = None

    def _pget(key: str) -> str:
      if params is None:
        return ""
      try:
        raw = params.get(key)
      except Exception:
        return ""
      if raw is None:
        return ""
      if isinstance(raw, (bytes, bytearray)):
        return raw.decode("utf-8", errors="replace").strip()
      return str(raw).strip()

    phone = _pget("EcoflowPhone") or os.environ.get("ECOFLOW_PHONE", "").strip()
    email = _pget("EcoflowEmail") or os.environ.get("ECOFLOW_EMAIL", "").strip()
    password = _pget("EcoflowPassword") or os.environ.get("ECOFLOW_PASSWORD", "").strip()
    sn = _pget("EcoflowSn") or os.environ.get("ECOFLOW_SN", "").strip()
    api_base_env = _pget("EcoflowApiBase") or os.environ.get("ECOFLOW_API_BASE", "").strip().rstrip("/")

    # Temporarily seed env so from_env() reuses normalization / base selection.
    saved = {k: os.environ.get(k) for k in ("ECOFLOW_PHONE", "ECOFLOW_EMAIL", "ECOFLOW_PASSWORD", "ECOFLOW_SN", "ECOFLOW_API_BASE")}
    try:
      if phone:
        os.environ["ECOFLOW_PHONE"] = phone
        os.environ.pop("ECOFLOW_EMAIL", None)
      elif email:
        os.environ["ECOFLOW_EMAIL"] = email
        os.environ.pop("ECOFLOW_PHONE", None)
      if password:
        os.environ["ECOFLOW_PASSWORD"] = password
      if sn:
        os.environ["ECOFLOW_SN"] = sn
      if api_base_env:
        os.environ["ECOFLOW_API_BASE"] = api_base_env
      return cls.from_env()
    finally:
      for k, v in saved.items():
        if v is None:
          os.environ.pop(k, None)
        else:
          os.environ[k] = v

  @staticmethod
  def default_dc_mode_for_sn(sn: str) -> str:
    # P231 = classic Delta 3 protobuf; D361 = newer JSON protocol.
    if sn.upper().startswith("P231"):
      return DC_MODE_PROTO
    return DC_MODE_MPPT_CAR

  def login(self) -> None:
    b64pw = base64.b64encode(self.password.encode()).decode()
    headers = {
      "Content-Type": "application/json",
      "User-Agent": USER_AGENT,
      "lang": "zh_CN" if self.use_phone else "en_US",
    }
    body: dict[str, Any] = {
      "password": b64pw,
      "scene": "IOT_APP",
      "userType": "ECOFLOW",
      "appVersion": "1.0.0",
      "os": "android",
      "osVersion": "30",
      "oauth": {"bundleId": "com.ef.EcoFlow"},
    }
    if self.use_phone:
      body["phone"] = self.account
    else:
      body["email"] = self.account

    login = _http_json("POST", f"{self.api_base}/auth/login", headers, body)
    data = login.get("data") or {}
    self.token = data.get("token") or ""
    user = data.get("user") or {}
    self.user_id = str(user.get("userId") or data.get("userId") or "")
    if not self.token or not self.user_id:
      raise EcoflowError(f"login missing token/userId: {login}")

    cert = _http_json(
      "GET",
      f"{self.api_base}/iot-auth/app/certification?userId={urllib.parse.quote(self.user_id)}",
      {**headers, "Authorization": f"Bearer {self.token}"},
    )
    cdata = cert.get("data") or {}
    self.mqtt_url = cdata.get("url") or ""
    self.mqtt_port = int(cdata.get("port") or 8883)
    self.mqtt_user = cdata.get("certificateAccount") or ""
    self.mqtt_pass = cdata.get("certificatePassword") or ""
    if not self.mqtt_url or not self.mqtt_user:
      raise EcoflowError(f"certification missing mqtt fields: {cert}")

  def _next_id(self) -> int:
    self._seq += 1
    return self._seq

  def _ensure_pb2(self):
    if self._pb2 is None:
      self._pb2 = _import_delta3_pb2()
    return self._pb2

  def _merge_proto_fields(self, msg) -> dict[str, Any]:
    """Flatten protobuf message fields that are set into a plain dict."""
    out: dict[str, Any] = {}
    for desc, value in msg.ListFields():
      name = desc.name
      if desc.type == desc.TYPE_MESSAGE:
        continue
      out[name] = value
    return out

  def _ingest_proto_bytes(self, raw: bytes, topic: str) -> None:
    pb2 = self._ensure_pb2()
    try:
      wrapper = pb2.Delta3HeaderMessage()
      wrapper.ParseFromString(raw)
      headers = list(wrapper.header)
    except Exception:
      try:
        wrapper2 = pb2.Delta3SendHeaderMsg()
        wrapper2.ParseFromString(raw)
        headers = list(wrapper2.msg)
      except Exception:
        return

    for header in headers:
      cmd_func = int(getattr(header, "cmd_func", 0) or 0)
      cmd_id = int(getattr(header, "cmd_id", 0) or 0)
      pdata = bytes(getattr(header, "pdata", b"") or b"")
      if not pdata:
        continue

      parsed: dict[str, Any] = {}
      try:
        if cmd_func == 254 and cmd_id == 21:
          m = pb2.Delta3DisplayPropertyUpload()
          m.ParseFromString(pdata)
          parsed = self._merge_proto_fields(m)
          _derive_dc_from_flow(parsed)
        elif cmd_func == 254 and cmd_id == 18:
          m = pb2.Delta3SetReply()
          m.ParseFromString(pdata)
          parsed = self._merge_proto_fields(m)
        elif cmd_func == 254 and cmd_id == 17:
          m = pb2.Delta3SetCommand()
          m.ParseFromString(pdata)
          parsed = self._merge_proto_fields(m)
      except Exception:
        continue

      if parsed:
        self._telemetry.update(parsed)
        _derive_dc_from_flow(self._telemetry)

      if "set_reply" in topic:
        self._ack = {"proto": True, "cmd_func": cmd_func, "cmd_id": cmd_id, **parsed}
        self._ack_event.set()

  def _on_message(self, _client, _userdata, msg) -> None:
    topic = msg.topic or ""
    raw = bytes(msg.payload or b"")

    # Prefer protobuf (P231); fall through to JSON for D361.
    if raw and raw[:1] not in (b"{", b"["):
      self._ingest_proto_bytes(raw, topic)
      return

    try:
      obj = json.loads(raw.decode())
    except (UnicodeDecodeError, json.JSONDecodeError):
      if raw:
        self._ingest_proto_bytes(raw, topic)
      return

    if topic == f"/app/device/property/{self.sn}":
      params = obj.get("params") or {}
      if isinstance(params, dict):
        self._telemetry.update(params)
    if "get_reply" in topic:
      quota = ((obj.get("data") or {}).get("quotaMap")) or {}
      if isinstance(quota, dict):
        self._telemetry.update(quota)
    if "set_reply" in topic:
      self._ack = obj
      self._ack_event.set()

  def connect_mqtt(self) -> None:
    try:
      import paho.mqtt.client as mqtt
    except ImportError as e:
      raise EcoflowError("Need paho-mqtt: pip install paho-mqtt") from e

    if not self.token:
      self.login()

    # Warm protobuf import early so SET/status don't fail mid-flight.
    if self.sn.upper().startswith("P231"):
      self._ensure_pb2()

    client_id = f"ANDROID_{random.randint(10_000_000, 99_999_999)}_{self.user_id}"
    client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)
    client.username_pw_set(self.mqtt_user, self.mqtt_pass)
    client.tls_set()
    client.on_message = self._on_message
    client.connect(self.mqtt_url, self.mqtt_port, keepalive=60)
    client.loop_start()
    self._client = client

    tel = f"/app/device/property/{self.sn}"
    get_reply = f"/app/{self.user_id}/{self.sn}/thing/property/get_reply"
    set_reply = f"/app/{self.user_id}/{self.sn}/thing/property/set_reply"
    client.subscribe([(tel, 0), (get_reply, 1), (set_reply, 1)])

  def disconnect(self) -> None:
    if self._client is not None:
      self._client.loop_stop()
      self._client.disconnect()
      self._client = None

  def _publish_set_json(self, module_type: int, operate_type: str, params: dict) -> dict | None:
    if self._client is None:
      raise EcoflowError("MQTT not connected")
    self._ack = None
    self._ack_event.clear()
    payload = {
      "id": self._next_id(),
      "version": "1.0",
      "sn": self.sn,
      "moduleType": module_type,
      "operateType": operate_type,
      "from": "Android",
      "params": params,
    }
    topic = f"/app/{self.user_id}/{self.sn}/thing/property/set"
    self._client.publish(topic, json.dumps(payload), qos=1)
    if not self._ack_event.wait(8.0):
      return None
    return self._ack

  def _build_dc12v_proto_packet(self, on: bool) -> bytes:
    pb2 = self._ensure_pb2()
    payload = pb2.Delta3SetCommand()
    payload.cfg_dc12v_out_open = 1 if on else 0
    pdata = payload.SerializeToString()

    packet = pb2.Delta3SendHeaderMsg()
    message = packet.msg.add()
    message.src = 32
    message.dest = 2
    message.d_src = 1
    message.d_dest = 1
    message.cmd_func = 254
    message.cmd_id = 17
    message.need_ack = 1
    message.seq = _gen_seq()
    message.product_id = 1
    message.version = 19
    message.payload_ver = 1
    message.device_sn = self.sn
    message.data_len = len(pdata)
    message.pdata = pdata
    return packet.SerializeToString()

  def _publish_set_proto_dc12v(self, on: bool, wait_s: float = 10.0) -> dict | None:
    if self._client is None:
      raise EcoflowError("MQTT not connected")
    self._ack = None
    self._ack_event.clear()
    before = dict(self._telemetry)
    want = 1 if on else 0

    topic = f"/app/{self.user_id}/{self.sn}/thing/property/set"
    self._client.publish(topic, self._build_dc12v_proto_packet(on), qos=1)

    deadline = time.time() + wait_s
    while time.time() < deadline:
      cur = self._telemetry.get("cfg_dc12v_out_open")
      if cur == want and (cur != before.get("cfg_dc12v_out_open") or self._ack is not None):
        return self._ack or {"ok": True, "cfg_dc12v_out_open": want, "via": "telemetry"}
      if self._ack is not None:
        ack_val = self._ack.get("cfg_dc12v_out_open")
        if ack_val is None or int(ack_val) == want:
          time.sleep(0.5)
          if self._telemetry.get("cfg_dc12v_out_open") == want or ack_val is not None:
            return self._ack
      time.sleep(0.2)

    if self._telemetry.get("cfg_dc12v_out_open") == want:
      return self._ack or {"ok": True, "cfg_dc12v_out_open": want, "via": "telemetry_late"}
    return self._ack  # may be None → TIMEOUT

  def refresh_quotas(self, wait_s: float = 5.0) -> dict[str, Any]:
    if self._client is None:
      raise EcoflowError("MQTT not connected")
    # JSON latestQuotas is mostly for D361; P231 pushes protobuf telemetry.
    topic = f"/app/{self.user_id}/{self.sn}/thing/property/get"
    payload = {
      "id": self._next_id(),
      "version": "1.0",
      "sn": self.sn,
      "moduleType": 0,
      "operateType": "latestQuotas",
      "params": {},
    }
    self._client.publish(topic, json.dumps(payload), qos=1)
    time.sleep(wait_s)
    return dict(self._telemetry)

  def set_dc12v(self, on: bool, mode: str | None = None, wait_s: float = 10.0) -> dict | None:
    if mode is None:
      mode = self.default_dc_mode_for_sn(self.sn)
    if mode == DC_MODE_PROTO:
      return self._publish_set_proto_dc12v(on, wait_s=wait_s)
    enabled = 1 if on else 0
    if mode == DC_MODE_MPPT_CAR:
      return self._publish_set_json(5, "mpptCar", {"enabled": enabled})
    if mode == DC_MODE_CFG:
      return self._publish_set_json(1, "setDp3", {"cfgDc12vOutOpen": enabled})
    raise EcoflowError(f"unknown DC mode {mode!r}")

  def dc_state_hint(self) -> dict[str, Any]:
    keys = (
      "cfg_dc12v_out_open",
      "flow_info_12v",
      "pow_get_12v",
      "dc_out_open",
      "mppt.carState",
      "flowInfo_12v",
      "powGet_12v",
      "powGetDc",
      "cfgDc12vOutOpen",
      "bms_batt_soc",
      "pow_out_sum_w",
      "pow_in_sum_w",
    )
    return {k: self._telemetry[k] for k in keys if k in self._telemetry}
