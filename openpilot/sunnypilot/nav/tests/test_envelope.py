from openpilot.sunnypilot.nav.envelope import EnvelopeVerifier, envelope_hmac


def _env(seq, data, ts=1_720_000_000_000, psk="999999"):
  digest = envelope_hmac(psk, seq, ts, data)
  return {"v": 1, "seq": seq, "ts": ts, "data": data, "hmac": digest}


def test_accept_then_replay():
  v = EnvelopeVerifier("999999")
  raw = __import__("json").dumps(_env(1, {"nRoadLimitSpeed": 50})).encode()
  status, data = v.inspect(raw, now_ms=1_720_000_000_000)
  assert status == "ok"
  assert data["nRoadLimitSpeed"] == 50
  status, data = v.inspect(raw, now_ms=1_720_000_000_000)
  assert status == "replay"
  assert data["nRoadLimitSpeed"] == 50
  assert v.accept(raw, now_ms=1_720_000_000_000) is None


def test_bad_hmac():
  v = EnvelopeVerifier("999999")
  env = _env(2, {"nRoadLimitSpeed": 40})
  env["hmac"] = "0" * 32
  raw = __import__("json").dumps(env).encode()
  status, data = v.inspect(raw, now_ms=1_720_000_000_000)
  assert status == "bad"
  assert data is None
