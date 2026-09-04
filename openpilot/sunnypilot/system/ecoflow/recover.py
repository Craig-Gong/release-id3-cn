"""Safe EcoFlow 12V cycle for chestnut GPU recovery (C3XL always-on host).

KL15 still owns normal on/off. This path only cuts 12V when parked and not
engaged, so USB can stay plugged while XT60 power-cycles the GPU.
"""
from __future__ import annotations

from enum import Enum, auto
from pathlib import Path


# EcoFlow cuts DC quickly; dock input caps need a few seconds, not minutes.
RECOVER_OFF_S = 15.0
RECOVER_ON_SETTLE_S = 5.0
# Parked gate: walking-speed and below, or offroad.
RECOVER_MAX_V_EGO = 0.5
# File flag works even when libparams was not rebuilt with EcoflowGpuRecover.
RECOVER_REQUEST_PATH = Path("/data/ecoflow_gpu_recover")


class RecoverPhase(Enum):
  idle = auto()
  power_off = auto()
  power_on = auto()


def recover_allowed(*, started: bool, engaged: bool, v_ego: float) -> bool:
  """True when a 12V cycle will not interrupt an active drive."""
  if engaged:
    return False
  if not started:
    return True
  return v_ego < RECOVER_MAX_V_EGO


def recover_request_pending(params_get_bool) -> bool:
  """True if UI/param or file flag asked for a recover cycle.

  params_get_bool: callable(key) -> bool that must not raise on unknown keys.
  """
  try:
    if params_get_bool("EcoflowGpuRecover"):
      return True
  except Exception:
    pass
  try:
    return RECOVER_REQUEST_PATH.exists()
  except OSError:
    return False


def request_recover() -> None:
  """UI / tools: latch a recover request (param optional; file always)."""
  try:
    RECOVER_REQUEST_PATH.parent.mkdir(parents=True, exist_ok=True)
    RECOVER_REQUEST_PATH.write_text("1", encoding="utf-8")
  except OSError:
    pass


def clear_recover_request(params=None) -> None:
  try:
    RECOVER_REQUEST_PATH.unlink(missing_ok=True)
  except OSError:
    pass
  if params is None:
    return
  try:
    params.remove("EcoflowGpuRecover")
  except Exception:
    try:
      params.put_bool("EcoflowGpuRecover", False, block=True)
    except Exception:
      pass


class GpuRecoverCycle:
  """Timed OFF → ON sequence. Caller sends MQTT; this only tracks phase/deadline."""

  def __init__(self, *, off_s: float = RECOVER_OFF_S, on_settle_s: float = RECOVER_ON_SETTLE_S):
    self.off_s = off_s
    self.on_settle_s = on_settle_s
    self.phase = RecoverPhase.idle
    self.deadline = 0.0

  @property
  def active(self) -> bool:
    return self.phase is not RecoverPhase.idle

  def start(self, now: float) -> None:
    self.phase = RecoverPhase.power_off
    self.deadline = now + self.off_s

  def cancel(self) -> None:
    self.phase = RecoverPhase.idle
    self.deadline = 0.0

  def tick(self, now: float) -> str | None:
    """Advance the cycle. Returns 'on', 'done', or None (waiting)."""
    if self.phase is RecoverPhase.idle:
      return None
    if now < self.deadline:
      return None
    if self.phase is RecoverPhase.power_off:
      self.phase = RecoverPhase.power_on
      self.deadline = now + self.on_settle_s
      return "on"
    # power_on settle finished
    self.cancel()
    return "done"
