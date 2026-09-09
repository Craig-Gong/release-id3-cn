"""Reassemble fragmented ATT writes into complete JSON objects."""
from __future__ import annotations

import json

MAX_GATT_BUF = 64 * 1024


def pop_complete_json(buf: bytearray) -> bytes | None:
  """Pull the first complete JSON object. Keeps a partial tail for the next ATT write."""
  if not buf:
    return None
  try:
    text = bytes(buf).decode("utf-8")
  except UnicodeDecodeError:
    return None
  stripped = text.lstrip()
  if not stripped:
    buf.clear()
    return None
  lead = len(text) - len(stripped)
  try:
    _, end = json.JSONDecoder().raw_decode(stripped)
  except json.JSONDecodeError:
    return None
  consumed = len(text[:lead + end].encode("utf-8"))
  blob = bytes(buf[:consumed])
  del buf[:consumed]
  return blob
