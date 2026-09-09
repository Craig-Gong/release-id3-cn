from openpilot.sunnypilot.nav.gatt_json import pop_complete_json


def test_pop_one_object():
  buf = bytearray(b'{"a":1}')
  assert pop_complete_json(buf) == b'{"a":1}'
  assert buf == b""


def test_pop_concatenated():
  buf = bytearray(b'{"a":1}{"b":2}')
  assert pop_complete_json(buf) == b'{"a":1}'
  assert pop_complete_json(buf) == b'{"b":2}'
  assert buf == b""


def test_pop_partial_waits():
  buf = bytearray(b'{"a":')
  assert pop_complete_json(buf) is None
  assert buf == b'{"a":'
  buf.extend(b'1}')
  assert pop_complete_json(buf) == b'{"a":1}'
