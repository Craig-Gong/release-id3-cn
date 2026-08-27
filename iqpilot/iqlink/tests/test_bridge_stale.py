"""R1: no-write warns only; sticky exec snapshot until content change or disable."""

from __future__ import annotations

import threading
import time

from openpilot.iqpilot.iqlink.bridge import IqlinkBridge, clear_stale_nav_params


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key, block=False, encoding=None):
    _ = block, encoding
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_bool(self, key, value):
    self.values[key] = bool(value)

  def put(self, key, value):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


def _bridge(params: FakeParams, *, age_s: float) -> IqlinkBridge:
  b = IqlinkBridge.__new__(IqlinkBridge)
  b.params = params
  b._lock = threading.Lock()
  b._latest = {"active": True, "valid": True, "speedTarget": 16.67}
  b._raw_payload = {"nRoadLimitSpeed": 60}
  b._raw_fp = None
  b._last_rx = time.monotonic() - age_s
  b._command_index = 0
  b._last_lc_cmd = False
  b._warn_logged = False
  b.sm = None
  return b


def test_clear_stale_keeps_exclusive_by_default():
  """Disable/keepalive cleanup clears Active, not Exclusive (R1: not called on timeout)."""
  p = FakeParams({"NavigationActive": True, "IqlinkExclusive": True, "IqlinkLinkWarn": True,
                  "NavigationDestination": {"name": "x"}})
  clear_stale_nav_params(p)
  assert p.get_bool("NavigationActive") is False
  assert p.get_bool("IqlinkLinkWarn") is False
  assert p.get_bool("IqlinkExclusive") is True
  assert "NavigationDestination" not in p.values


def test_clear_stale_can_drop_exclusive_on_disable():
  p = FakeParams({"IqlinkExclusive": True, "NavigationActive": True})
  clear_stale_nav_params(p, clear_exclusive=True)
  assert p.get_bool("IqlinkExclusive") is False


def test_maybe_timeout_keeps_envelope():
  """R1: long no-write warns but does not clear sticky snapshot."""
  p = FakeParams({"IqlinkWarnTimeoutS": "1", "IqlinkCancelTimeoutS": "2",
                  "NavigationActive": True, "IqlinkExclusive": True})
  b = _bridge(p, age_s=5.0)
  b._maybe_timeout()
  assert b._latest is not None
  assert b._raw_payload is not None
  assert p.get_bool("IqlinkLinkWarn") is True
  assert p.get_bool("NavigationActive") is True
  assert p.get_bool("IqlinkExclusive") is True


def test_soft_warn_before_long_silence():
  p = FakeParams({"IqlinkWarnTimeoutS": "1", "IqlinkCancelTimeoutS": "10"})
  b = _bridge(p, age_s=2.0)
  b._maybe_timeout()
  assert b._latest is not None
  assert p.get_bool("IqlinkLinkWarn") is True


def test_ingest_ignores_unchanged_payload():
  p = FakeParams({})
  b = IqlinkBridge.__new__(IqlinkBridge)
  b.params = p
  b._lock = threading.Lock()
  b._latest = None
  b._raw_payload = None
  b._raw_fp = None
  b._last_rx = 0.0
  b._command_index = 0
  b._last_lc_cmd = False
  b._warn_logged = False
  b.sm = type("SM", (), {"update": lambda *a, **k: None, "alive": {}})()

  payload = {"nRoadLimitSpeed": 60}
  b.ingest(payload)
  assert b._latest is not None
  first = dict(b._latest)
  t0 = b._last_rx
  time.sleep(0.02)
  b.ingest(dict(payload))  # same content
  assert b._latest == first
  assert b._last_rx > t0  # heartbeat refreshed
  b.ingest({"nRoadLimitSpeed": 80})
  assert abs(b._latest["speedTarget"] - 80 / 3.6) < 1e-3


def test_ingest_disables_nav_exit_alc():
  p = FakeParams({"NavExitLaneChange": True, "AutoLaneChangeTimer": 1})
  b = IqlinkBridge.__new__(IqlinkBridge)
  b.params = p
  b._lock = threading.Lock()
  b._latest = None
  b._raw_payload = None
  b._raw_fp = None
  b._last_rx = 0.0
  b._command_index = 0
  b._last_lc_cmd = False
  b._warn_logged = False
  b.sm = type("SM", (), {"update": lambda *a, **k: None, "alive": {}})()
  b.ingest({"nRoadLimitSpeed": 60})
  assert p.get_bool("NavExitLaneChange") is False
  assert p.values.get("AutoLaneChangeTimer") == 1  # not forced to DIRECT
