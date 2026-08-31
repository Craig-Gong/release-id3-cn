"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""

import pytest

from iqpilot.selfdrive.longitudinal_settings import (
  LONGITUDINAL_MODE_CHILL,
  LONGITUDINAL_MODE_DYNAMIC,
  LONGITUDINAL_MODE_STOCK,
  PERSONALITY_AGGRESSIVE,
  PERSONALITY_RELAXED,
  PERSONALITY_STANDARD,
  PERSONALITY_VALUES,
  apply_longitudinal_mode,
  get_follow_distance_state,
  get_longitudinal_mode,
  get_runtime_personality,
  set_valid_personality,
)


class Params:
  def __init__(self, personality=PERSONALITY_STANDARD):
    self.values = {
      "AlphaLongitudinalEnabled": True,
      "ExperimentalMode": True,
      "IQDynamicMode": True,
      "LongitudinalPersonality": personality,
    }
    self.personality_writes = []

  def get(self, key, return_default=False):
    return self.values[key]

  def get_bool(self, key):
    return bool(self.values[key])

  def put(self, key, value):
    self.values[key] = value
    if key == "LongitudinalPersonality":
      self.personality_writes.append(value)

  def put_bool(self, key, value):
    self.values[key] = bool(value)


def test_personality_writer_rejects_stock_value():
  params = Params()

  with pytest.raises(ValueError):
    set_valid_personality(params, 3)

  assert params.personality_writes == []


def test_mode_paths_only_write_valid_personalities():
  params = Params(PERSONALITY_AGGRESSIVE)

  for mode in range(4):
    apply_longitudinal_mode(params, mode)

  assert all(value in PERSONALITY_VALUES for value in params.personality_writes)


def test_stock_mode_preserves_personality_and_dynamic_restores_it():
  params = Params(PERSONALITY_AGGRESSIVE)

  apply_longitudinal_mode(params, LONGITUDINAL_MODE_STOCK)

  assert get_follow_distance_state(params) == (None, False)
  assert get_runtime_personality(params) == PERSONALITY_AGGRESSIVE
  assert params.values["LongitudinalPersonality"] == PERSONALITY_AGGRESSIVE
  assert params.personality_writes == []

  apply_longitudinal_mode(params, LONGITUDINAL_MODE_DYNAMIC)

  assert get_longitudinal_mode(params) == LONGITUDINAL_MODE_DYNAMIC
  assert get_follow_distance_state(params) == (PERSONALITY_AGGRESSIVE, True)


def test_stock_mode_sanitizes_legacy_stock_personality_value():
  params = Params(3)

  apply_longitudinal_mode(params, LONGITUDINAL_MODE_STOCK)

  assert get_follow_distance_state(params) == (None, False)
  assert params.values["LongitudinalPersonality"] == PERSONALITY_RELAXED
  assert params.personality_writes == [PERSONALITY_RELAXED]


def test_chill_mode_forces_relaxed_personality():
  params = Params(PERSONALITY_AGGRESSIVE)

  apply_longitudinal_mode(params, LONGITUDINAL_MODE_CHILL)

  assert get_follow_distance_state(params) == (PERSONALITY_RELAXED, False)
  assert get_runtime_personality(params) == PERSONALITY_RELAXED
  assert params.values["LongitudinalPersonality"] == PERSONALITY_RELAXED
  assert params.personality_writes == [PERSONALITY_RELAXED]


def test_dynamic_and_pilot_enable_valid_personality_selection():
  params = Params(PERSONALITY_STANDARD)

  assert get_follow_distance_state(params) == (PERSONALITY_STANDARD, True)

  params.values["IQDynamicMode"] = False

  assert get_follow_distance_state(params) == (PERSONALITY_STANDARD, True)
