from collections.abc import Callable
from enum import StrEnum
import time

from panda import Panda

from openpilot.common.hardware import HARDWARE
from openpilot.sunnypilot.hardware.profile import HardwareProfile, get_hardware_profile


class PandaStartupResult(StrEnum):
  RESET = "reset"
  RECOVER = "recover"
  APP_READY = "app_ready"
  BOOTSTUB_READY = "bootstub_ready"
  RECOVER_AFTER_TIMEOUT = "recover_after_timeout"
  INTERRUPTED = "interrupted"


class PandaStartupIO:
  def reset_internal(self) -> None:
    HARDWARE.reset_internal_panda()

  def recover_internal(self) -> None:
    HARDWARE.recover_internal_panda()

  def list_internal(self) -> list[str]:
    # C3XL / AGNOS: internal tres is SPI. Non-C3XL tip tici may prefer USB-only list.
    if HARDWARE.get_device_type() == "tici" and get_hardware_profile() != HardwareProfile.C3XL:
      return Panda.list(usb_only=True)
    return Panda.spi_list()

  def is_bootstub(self, serial: str) -> bool:
    with Panda(serial) as panda:
      return panda.bootstub

  def monotonic(self) -> float:
    return time.monotonic()

  def sleep(self, seconds: float) -> None:
    time.sleep(seconds)


class PandaStartup:
  C3XL_EMPTY_WAIT_S = 5.0

  def __init__(self, io: PandaStartupIO | None = None, profile: HardwareProfile | None = None):
    self.io = io or PandaStartupIO()
    self.profile = profile or get_hardware_profile()

  def prepare(self, attempt: int, should_exit: Callable[[], bool]) -> PandaStartupResult:
    # Non-C3XL: tip alternating reset / DFU recover.
    if self.profile != HardwareProfile.C3XL:
      if (attempt % 2) == 0:
        self.io.reset_internal()
        return PandaStartupResult.RESET
      self.io.recover_internal()
      return PandaStartupResult.RECOVER

    # C3XL (ID.3 daily parity): never GPIO-reset a live internal panda (clock_init
    # deadlock) and never DFU-recover (erases sectors 0+1). Wait if the list is
    # empty instead of recover().
    if should_exit():
      return PandaStartupResult.INTERRUPTED
    try:
      serials = self.io.list_internal()
    except Exception:
      serials = []
    for serial in serials:
      try:
        if not self.io.is_bootstub(serial):
          return PandaStartupResult.APP_READY
      except Exception:
        continue
    if serials:
      return PandaStartupResult.BOOTSTUB_READY
    self.io.sleep(self.C3XL_EMPTY_WAIT_S)
    return PandaStartupResult.RESET
