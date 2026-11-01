from openpilot.sunnypilot.nav.envelope import EnvelopeVerifier, envelope_hmac
from openpilot.sunnypilot.nav.udp_nav import UDP_ACK_PAYLOAD, UdpNavServer


def test_udp_server_receives_hmac_envelope():
  import json
  import socket
  import time

  got = []

  def on_dg(raw: bytes, addr):
    got.append((raw, addr))

  probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  probe.bind(("127.0.0.1", 0))
  port = probe.getsockname()[1]
  probe.close()

  srv = UdpNavServer(on_dg, port=port)
  assert srv.start()
  try:
    data = {"nRoadLimitSpeed": 60, "trafficLight": "red", "trafficLightDistM": 40}
    ts = 1_720_000_000_000
    seq = 9
    env = {
      "v": 1,
      "seq": seq,
      "ts": ts,
      "data": data,
      "hmac": envelope_hmac("999999", seq, ts, data),
    }
    raw = json.dumps(env, separators=(",", ":")).encode()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(raw, ("127.0.0.1", port))
    sock.close()
    deadline = time.time() + 2.0
    while not got and time.time() < deadline:
      time.sleep(0.05)
    assert len(got) == 1
    status, payload = EnvelopeVerifier("999999").inspect(got[0][0], now_ms=ts)
    assert status == "ok"
    assert payload["nRoadLimitSpeed"] == 60
  finally:
    srv.stop()


def test_udp_server_reply_ack():
  import json
  import socket
  import time

  def on_dg(raw: bytes, addr):
    srv.reply(addr)

  probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  probe.bind(("127.0.0.1", 0))
  port = probe.getsockname()[1]
  probe.close()

  srv = UdpNavServer(on_dg, port=port)
  assert srv.start()
  try:
    data = {"nRoadLimitSpeed": 40}
    ts = 1_720_000_000_100
    seq = 11
    env = {
      "v": 1,
      "seq": seq,
      "ts": ts,
      "data": data,
      "hmac": envelope_hmac("999999", seq, ts, data),
    }
    raw = json.dumps(env, separators=(",", ":")).encode()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.5)
    sock.sendto(raw, ("127.0.0.1", port))
    ack, _from = sock.recvfrom(256)
    sock.close()
    assert ack == UDP_ACK_PAYLOAD
    assert json.loads(ack.decode())["ok"] == 1
  finally:
    srv.stop()
