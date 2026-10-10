"""LAN UDP nav transport for IQ-link (EcoFlow hotspot). Same HMAC envelope as BLE."""
from __future__ import annotations

import socket
import threading
from typing import Callable

# Not the deprecated Carrot 7705/7706 plain JSON ports — HMAC envelope only.
IQLINK_UDP_PORT = 17710
MAX_DATAGRAM = 64 * 1024


def _log(msg: str) -> None:
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.info(msg)
  except Exception:
    print(msg)


class UdpNavServer:
  """Bind 0.0.0.0:IQLINK_UDP_PORT and forward complete datagrams to on_datagram."""

  def __init__(self, on_datagram: Callable[[bytes], None], port: int = IQLINK_UDP_PORT):
    self.on_datagram = on_datagram
    self.port = int(port)
    self._thread: threading.Thread | None = None
    self._stop = threading.Event()
    self._sock: socket.socket | None = None
    self.running = False

  def start(self) -> bool:
    if self._thread and self._thread.is_alive() and self.running:
      return True
    self.stop()
    self._stop.clear()
    try:
      sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      sock.bind(("0.0.0.0", self.port))
      sock.settimeout(0.5)
    except OSError as e:
      _log(f"iqlink UDP bind :{self.port} failed: {e}")
      return False
    self._sock = sock
    self._thread = threading.Thread(target=self._run, name="iqlink-udp", daemon=True)
    self._thread.start()
    self.running = True
    _log(f"iqlink UDP listening on 0.0.0.0:{self.port}")
    return True

  def stop(self) -> None:
    self._stop.set()
    sock = self._sock
    self._sock = None
    if sock is not None:
      try:
        sock.close()
      except OSError:
        pass
    if self._thread is not None:
      self._thread.join(timeout=2.0)
      self._thread = None
    self.running = False

  def _run(self) -> None:
    sock = self._sock
    if sock is None:
      self.running = False
      return
    try:
      while not self._stop.is_set():
        try:
          data, _addr = sock.recvfrom(MAX_DATAGRAM)
        except socket.timeout:
          continue
        except OSError:
          break
        if not data:
          continue
        try:
          self.on_datagram(data)
        except Exception:
          _log("iqlink UDP datagram handler error")
    finally:
      self.running = False
      try:
        sock.close()
      except OSError:
        pass
