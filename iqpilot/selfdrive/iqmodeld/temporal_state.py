"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""

from __future__ import annotations

import math

import numpy as np

DEFAULT_FRAME_SKIP = 4

MODEL_INPUT_SPEC: dict[str, tuple[tuple[int, ...], str]] = {
  "img": ((1, 12, 128, 256), "uint8"),
  "big_img": ((1, 12, 128, 256), "uint8"),
  "desire_pulse": ((1, 25, 8), "float32"),
  "traffic_convention": ((1, 2), "float32"),
  "features_buffer": ((1, 24, 512), "float32"),
  "action_t": ((1, 2), "float32"),
}


def spec_from_meta(meta: dict) -> dict[str, tuple[tuple[int, ...], str]] | None:
  shapes = meta.get("input_shapes")
  if not shapes:
    return None
  return {name: (tuple(shape), "uint8" if name in ("img", "big_img") else "float32")
          for name, shape in shapes.items()}


class TemporalInputState:
  def __init__(self, frame_skip: int, spec: dict[str, tuple[tuple[int, ...], str]] = MODEL_INPUT_SPEC):
    self.frame_skip = frame_skip
    img = spec["img"][0]
    fb = spec["features_buffer"][0]
    dp = spec["desire_pulse"][0]

    self.n_frames = img[1] // 6
    img_q_shape = (frame_skip * (self.n_frames - 1) + 1, 6, img[2], img[3])
    self._img_shape = img
    self._fb_shape = fb
    self._dp_shape = dp
    feat_dim = math.prod(fb[2:])

    self.img_q = np.zeros(img_q_shape, dtype=np.uint8)
    self.big_img_q = np.zeros(img_q_shape, dtype=np.uint8)
    self.feat_q = np.zeros((frame_skip * fb[1], fb[0], feat_dim), dtype=np.float32)
    self.desire_q = np.zeros((frame_skip * dp[1], dp[0], dp[2]), dtype=np.float32)
    self.prev_desire = np.zeros(dp[2], dtype=np.float32)
    self.prev_feat = np.zeros((fb[0], feat_dim), dtype=np.float32)

  @staticmethod
  def _shift_append(q: np.ndarray, new_val: np.ndarray) -> None:
    q[:-1] = q[1:]
    q[-1] = new_val

  def push_and_materialize(self, warped: np.ndarray, desire_pulse: np.ndarray,
                           traffic_convention: np.ndarray, action_t: np.ndarray,
                           ) -> dict[str, np.ndarray]:
    fs = self.frame_skip

    cur = desire_pulse.astype(np.float32).copy()
    cur[0] = 0
    pulse = np.where(cur - self.prev_desire > 0.99, cur, 0).astype(np.float32)
    self.prev_desire[:] = cur

    self._shift_append(self.img_q, warped[0])
    self._shift_append(self.big_img_q, warped[1])
    self._shift_append(self.desire_q, pulse.reshape(self._dp_shape[0], self._dp_shape[2]))
    self._shift_append(self.feat_q, self.prev_feat)

    dp = self._dp_shape
    return {
      "img": np.ascontiguousarray(self.img_q[::fs]).reshape(self._img_shape),
      "big_img": np.ascontiguousarray(self.big_img_q[::fs]).reshape(self._img_shape),
      "features_buffer": np.ascontiguousarray(self.feat_q[::fs]).reshape(self._fb_shape),
      "desire_pulse": self.desire_q.reshape(dp[1], fs, dp[0], dp[2]).max(axis=1).reshape(dp),
      "traffic_convention": traffic_convention.astype(np.float32).reshape(1, -1),
      "action_t": action_t.astype(np.float32).reshape(1, -1),
    }

  def note_hidden_state(self, model_output: np.ndarray, hidden_slice: slice) -> None:
    self.prev_feat[:] = model_output[hidden_slice].reshape(self.prev_feat.shape)


class SplitTemporalState:

  def __init__(self, frame_skip: int, img_shape: tuple[int, ...],
               feature_shape: tuple[int, ...], desire_shape: tuple[int, ...]):
    self.frame_skip = frame_skip
    self._img_shape = tuple(img_shape)
    self._fb_shape = tuple(feature_shape)
    self._dp_shape = tuple(desire_shape)

    n_frames = img_shape[1] // 6
    img_q_shape = (frame_skip * (n_frames - 1) + 1, 6, img_shape[2], img_shape[3])
    self.img_q = np.zeros(img_q_shape, dtype=np.uint8)
    self.big_img_q = np.zeros(img_q_shape, dtype=np.uint8)
    self.feat_q = np.zeros((frame_skip * (feature_shape[1] - 1) + 1, feature_shape[0], feature_shape[2]),
                           dtype=np.float32)
    self.desire_q = np.zeros((frame_skip * desire_shape[1], desire_shape[0], desire_shape[2]), dtype=np.float32)
    self.prev_desire = np.zeros(desire_shape[2], dtype=np.float32)

  def materialize_vision(self, warped: np.ndarray, desire: np.ndarray) -> dict[str, np.ndarray]:
    fs = self.frame_skip
    cur = desire.astype(np.float32).copy()
    cur[0] = 0
    pulse = np.where(cur - self.prev_desire > 0.99, cur, 0).astype(np.float32)
    self.prev_desire[:] = cur

    TemporalInputState._shift_append(self.img_q, warped[0])
    TemporalInputState._shift_append(self.big_img_q, warped[1])
    TemporalInputState._shift_append(self.desire_q, pulse.reshape(self._dp_shape[0], self._dp_shape[2]))
    return {
      "img": np.ascontiguousarray(self.img_q[::fs]).reshape(self._img_shape),
      "big_img": np.ascontiguousarray(self.big_img_q[::fs]).reshape(self._img_shape),
    }

  def materialize_policy(self, vision_feature: np.ndarray, traffic_convention: np.ndarray,
                         action_t: np.ndarray | None = None) -> dict[str, np.ndarray]:
    fs = self.frame_skip
    TemporalInputState._shift_append(self.feat_q, vision_feature.reshape(self._fb_shape[0], self._fb_shape[2]))
    dp = self._dp_shape
    out = {
      "features_buffer": np.ascontiguousarray(self.feat_q[::fs]).reshape(self._fb_shape),
      "desire_pulse": self.desire_q.reshape(dp[1], fs, dp[0], dp[2]).max(axis=1).reshape(dp),
      "traffic_convention": traffic_convention.astype(np.float32).reshape(1, -1),
    }
    if action_t is not None:
      out["action_t"] = action_t.astype(np.float32).reshape(1, -1)
    return out
