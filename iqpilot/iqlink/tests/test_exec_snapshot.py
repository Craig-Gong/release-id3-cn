from iqpilot.iqlink.exec_snapshot import preserve_exec_long


def test_preserve_keeps_exec_when_limit_drops_to_zero():
  prev = {
    "longitudinalEngaged": True,
    "speedTarget": 16.67,
    "roadSpeedLimit": 16.67,
    "valid": True,
  }
  incoming = {
    "longitudinalEngaged": False,
    "speedTarget": 0.0,
    "nextManeuverDistance": 80.0,
    "destinationValid": True,
  }
  merged = preserve_exec_long(incoming, prev)
  assert merged["longitudinalEngaged"] is True
  assert abs(merged["speedTarget"] - 16.67) < 1e-6
  assert abs(merged["roadSpeedLimit"] - 16.67) < 1e-6
  assert merged["nextManeuverDistance"] == 80.0
  assert merged["destinationValid"] is True


def test_preserve_skips_when_already_engaged():
  prev = {"longitudinalEngaged": True, "speedTarget": 16.67}
  incoming = {"longitudinalEngaged": True, "speedTarget": 22.2}
  assert preserve_exec_long(incoming, prev)["speedTarget"] == 22.2


def test_preserve_skips_without_prior_exec():
  incoming = {"longitudinalEngaged": False, "speedTarget": 0.0}
  assert preserve_exec_long(incoming, None)["speedTarget"] == 0.0
