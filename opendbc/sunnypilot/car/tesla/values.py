"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntFlag


class TeslaFlagsSP(IntFlag):
  HAS_VEHICLE_BUS = 1  # Multi-finger infotainment press signal is present on the VEHICLE bus with the deprecated Tesla harness installed
  COOP_STEERING = 2  # Coop steering
  MADS_SCREEN_BUTTON_3_FINGER = 4
  MADS_SCREEN_BUTTON_5_FINGER = 16
  STOCK_LONGITUDINAL_ACTIVE = 32  # Runtime toggle: 4-finger touch switches to stock (OEM) longitudinal control
  DYNAMIC_AUTO_STOCK = 64
  AP_HYBRID = 256  # Config: keep an AP session active with sunnypilot lateral control
  AP_HYBRID_ACTIVE = 512  # Runtime: AP hybrid session active; longitudinal may be SP or Tesla
  DYNAMIC_STOCK_ACTIVE = 1024  # Runtime source: Dynamic ACC selected stock longitudinal
  MANUAL_STOCK_ACTIVE = 2048  # Runtime source: 4-finger selection chose stock longitudinal
  DYNAMIC_AP_LONGITUDINAL = 4096  # Config: switch AP longitudinal between SP and OEM using speed hysteresis
  AP_HYBRID_STOCK_LATERAL_ACTIVE = 8192  # Runtime: Tesla AP owns lateral control
  AP_HYBRID_EXIT_RECOVERY_ACTIVE = 16384  # Runtime: preserve MADS while Tesla AP/LKAS settles after exit
  TURN_SIGNAL_VALIDATION = 32768  # Config: one-shot DAS body-control validation while disengaged
  SPEED_BUTTON_VALIDATION = 65536  # Config: one-shot replay of a fresh original 0x3C2 vehicle template while disengaged
  AUTO_SPEED_LIMIT = 131072  # Config: synchronize Tesla set speed to SP's resolved speed limit


class MadsScreenButtonType:
  OFF = 0
  THREE_FINGER = 1
  FIVE_FINGER = 2


class TeslaSafetyFlagsSP:
  HAS_VEHICLE_BUS = 1
  MADS_SCREEN_BUTTON_3_FINGER = 2
  MADS_SCREEN_BUTTON_5_FINGER = 8
  DYNAMIC_AUTO_STOCK = 16
  AP_HYBRID_HANDOFF = 64
  AP_HYBRID_LATERAL_HANDOFF = 128
  TURN_SIGNAL_VALIDATION = 256
  SPEED_BUTTON_VALIDATION = 512
  AUTO_SPEED_LIMIT = 1024
