"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""
from __future__ import annotations

import io
import math
import os
import pickle
import shutil
import struct
import tempfile

import numpy as np

POLICY_FORMAT = 2
OOB_MAGIC = b"IQEGPUOOB1"
QUEUE_NAMES = ("img_q", "big_img_q", "feat_q", "desire_q")
PACKED_ORDER = ("desire", "traffic_convention", "action_t", "prev_feat")


def packed_layout(input_spec: dict) -> tuple[dict[str, tuple[int, ...]], list[int]]:
  dp = input_spec["desire_pulse"][0]
  fb = input_spec["features_buffer"][0]
  shapes = {
    "desire": (dp[2],),
    "traffic_convention": tuple(input_spec["traffic_convention"][0]),
    "action_t": tuple(input_spec["action_t"][0]),
    "prev_feat": (fb[0], math.prod(fb[2:])),
  }
  return shapes, [math.prod(s) for s in shapes.values()]


def queue_shapes(input_spec: dict, frame_skip: int) -> dict[str, tuple[tuple[int, ...], str]]:
  img = input_spec["img"][0]
  fb = input_spec["features_buffer"][0]
  dp = input_spec["desire_pulse"][0]
  n_frames = img[1] // 6
  img_buf = (frame_skip * (n_frames - 1) + 1, 6, img[2], img[3])
  return {
    "img_q": (img_buf, "uint8"),
    "big_img_q": (img_buf, "uint8"),
    "feat_q": ((frame_skip * fb[1], fb[0], math.prod(fb[2:])), "float32"),
    "desire_q": ((frame_skip * dp[1], dp[0], dp[2]), "float32"),
  }


def make_queues(input_spec: dict, frame_skip: int, device: str) -> dict:
  from tinygrad.tensor import Tensor
  return {name: Tensor(np.zeros(shape, dtype=dtype), device=device).contiguous().realize()
          for name, (shape, dtype) in queue_shapes(input_spec, frame_skip).items()}


class PackedInputs:
  def __init__(self, input_spec: dict):
    from tinygrad.tensor import Tensor
    self.shapes, self.sizes = packed_layout(input_spec)
    self.array = np.zeros(sum(self.sizes), dtype=np.float32)
    self.views = dict(zip(self.shapes, [v.reshape(s) for s, v in zip(self.shapes.values(), np.split(self.array, np.cumsum(self.sizes[:-1])))], strict=True))
    self.tensor = Tensor(self.array, device="NPY").realize()


def make_run_policy(model_runner, input_spec: dict, frame_skip: int, device: str):
  from tinygrad.tensor import Tensor
  shapes, sizes = packed_layout(input_spec)
  fb = input_spec["features_buffer"][0]

  def shift_and_sample(buf, new_val, sample_fn):
    buf.assign(buf[1:].cat(new_val, dim=0).contiguous())
    return sample_fn(buf)

  def sample_skip(buf):
    return buf[::frame_skip].contiguous().flatten(0, 1).unsqueeze(0)

  def sample_desire(buf):
    return buf.reshape(-1, frame_skip, *buf.shape[1:]).max(1).flatten(0, 1).unsqueeze(0)

  def run_policy(warped, img_q, big_img_q, feat_q, desire_q, packed_npy_inputs):
    packed_npy_inputs = packed_npy_inputs.to(device)
    warped = warped.to(device)
    Tensor.realize(packed_npy_inputs, warped)
    img = shift_and_sample(img_q, warped[0:1], sample_skip)
    big_img = shift_and_sample(big_img_q, warped[1:2], sample_skip)
    desire, traffic_convention, action_t, prev_feat = (t.reshape(s) for t, s in zip(packed_npy_inputs.split(sizes), shapes.values(), strict=True))
    desire_buf = shift_and_sample(desire_q, desire.reshape(1, 1, -1), sample_desire)
    feat_buf = shift_and_sample(feat_q, prev_feat.reshape(1, 1, -1), sample_skip)
    inputs = {
      "img": img,
      "big_img": big_img,
      "features_buffer": feat_buf.reshape(fb),
      "desire_pulse": desire_buf,
      "traffic_convention": traffic_convention,
      "action_t": action_t,
    }
    out = next(iter(model_runner(inputs).values())).cast("float32")
    return out.reshape(-1),

  return run_policy


class PolicyRunner:
  def __init__(self, jit, input_spec: dict, frame_skip: int, hidden_slice: slice, device: str):
    from tinygrad.tensor import Tensor
    self._Tensor = Tensor
    self._jit = jit
    self._queues = make_queues(input_spec, frame_skip, device)
    self._packed = PackedInputs(input_spec)
    self._hidden = hidden_slice
    self._prev_desire = np.zeros(input_spec["desire_pulse"][0][2], dtype=np.float32)
    self._warped_shape = (2, 6, *input_spec["img"][0][2:])

  def run(self, warped: np.ndarray, desire_pulse: np.ndarray, traffic_convention: np.ndarray,
          action_t: np.ndarray) -> np.ndarray:
    cur = desire_pulse.astype(np.float32, copy=False)
    v = self._packed.views
    v["desire"][:] = np.where(cur - self._prev_desire > 0.99, cur, 0)
    self._prev_desire[:] = cur
    v["traffic_convention"][:] = np.asarray(traffic_convention, dtype=np.float32).reshape(v["traffic_convention"].shape)
    v["action_t"][:] = np.asarray(action_t, dtype=np.float32).reshape(v["action_t"].shape)
    warped_t = self._Tensor(np.ascontiguousarray(warped, dtype=np.uint8).reshape(self._warped_shape), device="NPY").realize()
    out, = self._jit(warped=warped_t, packed_npy_inputs=self._packed.tensor, **self._queues)
    flat = out.numpy().reshape(-1)
    v["prev_feat"][:] = flat[self._hidden].reshape(v["prev_feat"].shape)
    return flat


def dump_oob(obj, f) -> None:
  # Out-of-band pickle buffers keep the host peak at one tensor while the weights stream to the
  # dock; a plain pickle keeps every weight referenced in the memo until load() returns (~1.7GB).
  f.write(OOB_MAGIC)
  with tempfile.TemporaryFile(dir=os.path.dirname(os.path.abspath(f.name)) or ".") as tmp:
    def buffer_callback(pb: pickle.PickleBuffer):
      m = pb.raw()
      tmp.write(struct.pack("<q", m.nbytes))
      tmp.write(m)
      pb.release()
    stream = io.BytesIO()
    pickle.Pickler(stream, protocol=5, buffer_callback=buffer_callback).dump(obj)
    opcodes = stream.getvalue()
    f.write(struct.pack("<q", len(opcodes)))
    f.write(opcodes)
    tmp.seek(0)
    shutil.copyfileobj(tmp, f)


def is_oob(path: str) -> bool:
  with open(path, "rb") as f:
    return f.read(len(OOB_MAGIC)) == OOB_MAGIC


def load_oob(f):
  if f.read(len(OOB_MAGIC)) != OOB_MAGIC:
    raise ValueError("not an out-of-band bundle")
  opcodes = f.read(struct.unpack("<q", f.read(8))[0])

  def buffers():
    while (h := f.read(8)):
      pb = pickle.PickleBuffer(bytearray(struct.unpack("<q", h)[0]))
      f.readinto(pb)
      yield pb

  return pickle.load(io.BytesIO(opcodes), buffers=buffers())


def load_bundle(path: str):
  with open(path, "rb") as f:
    if f.read(len(OOB_MAGIC)) == OOB_MAGIC:
      f.seek(0)
      return load_oob(f)
    f.seek(0)
    return pickle.load(f)
