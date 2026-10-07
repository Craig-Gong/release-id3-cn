#!/usr/bin/env python3
import time
import json
import jwt
from typing import cast
from pathlib import Path

from datetime import datetime, timedelta, UTC
from openpilot.common.api import api_get, get_key_pair
from openpilot.common.params import Params
from openpilot.common.spinner import Spinner
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.common.hardware import HARDWARE, PC
from openpilot.common.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog


UNREGISTERED_DONGLE_ID = "UnregisteredDevice"
REGISTRATION_TIMEOUT = 30.0
IMEI_TIMEOUT = 5.0

def is_registered_device() -> bool:
  dongle = Params().get("DongleId")
  return dongle not in (None, UNREGISTERED_DONGLE_ID)


def register(show_spinner=False) -> str | None:
  """
  All devices built since March 2024 come with all
  info stored in /persist/. This is kept around
  only for devices built before then.

  With a backend update to take serial number instead
  of dongle ID to some endpoints, this can be removed
  entirely.
  """
  params = Params()

  dongle_id: str | None = params.get("DongleId") or None
  # Offline first boots must be able to retry once connectivity returns.
  if dongle_id == UNREGISTERED_DONGLE_ID:
    dongle_id = None
  if dongle_id is None and Path(Paths.persist_root()+"/comma/dongle_id").is_file():
    # not all devices will have this; added early in comma 3X production (2/28/24)
    with open(Paths.persist_root()+"/comma/dongle_id") as f:
      dongle_id = f.read().strip() or None

  # Create registration token, in the future, this key will make JWTs directly
  jwt_algo, private_key, public_key = get_key_pair()

  if not public_key:
    dongle_id = UNREGISTERED_DONGLE_ID
    cloudlog.warning("missing public key")
  elif dongle_id is None:
    spinner = Spinner() if show_spinner else None
    if spinner is not None:
      spinner.update("registering device")
    try:
      dongle_id = _register_online(jwt_algo, private_key, public_key)
    finally:
      if spinner is not None:
        spinner.close()

  if dongle_id:
    params.put("DongleId", dongle_id, block=True)
    set_offroad_alert("Offroad_UnregisteredHardware", (dongle_id == UNREGISTERED_DONGLE_ID) and not PC)
  return dongle_id


def _register_online(jwt_algo, private_key, public_key):
  serial = HARDWARE.get_serial()
  started = time.monotonic()
  deadline = started + REGISTRATION_TIMEOUT
  imei = None
  while imei is None:
    try:
      imei = HARDWARE.get_imei()
    except Exception:
      cloudlog.exception("Error getting imei")
    if imei is not None or time.monotonic() >= started + IMEI_TIMEOUT:
      break
    time.sleep(1)

  # Wi-Fi-only hardware may have no modem. Do not invent an IMEI.
  backoff = 0
  while (remaining := deadline - time.monotonic()) > 0:
    try:
      register_token = jwt.encode({'register': True, 'exp': datetime.now(UTC).replace(tzinfo=None) + timedelta(hours=1)},
                                  cast(str, private_key), algorithm=jwt_algo)
      resp = api_get("v2/pilotauth/", method='POST', timeout=min(10, remaining),
                     imei=imei or "", imei2="", serial=serial, public_key=public_key, register_token=register_token)
      if resp.status_code in (402, 403):
        return UNREGISTERED_DONGLE_ID
      dongle_id = json.loads(resp.text)["dongle_id"]
      if not isinstance(dongle_id, str) or not dongle_id.strip():
        raise ValueError("Empty device registration response")
      return dongle_id
    except NotImplementedError:
      raise
    except Exception:
      cloudlog.exception("failed to authenticate")
      backoff = min(backoff + 1, 5)
      time.sleep(min(backoff, max(0, deadline - time.monotonic())))
  cloudlog.warning("Device registration timed out; continuing offline, retrying on next startup")
  return UNREGISTERED_DONGLE_ID


if __name__ == "__main__":
  print(register())
