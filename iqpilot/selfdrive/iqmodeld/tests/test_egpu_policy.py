"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos/
"""
import os

import numpy as np

os.environ["DEV"] = "CPU"

from iqpilot.selfdrive.iqmodeld.egpu_policy import PolicyRunner, make_run_policy, packed_layout, queue_shapes
from iqpilot.selfdrive.iqmodeld.temporal_state import TemporalInputState

SPEC = {
  "img": ((1, 12, 8, 16), "uint8"),
  "big_img": ((1, 12, 8, 16), "uint8"),
  "desire_pulse": ((1, 25, 8), "float32"),
  "traffic_convention": ((1, 2), "float32"),
  "action_t": ((1, 2), "float32"),
  "features_buffer": ((1, 24, 512), "float32"),
}
FS = 4
OUT_LEN = 2580
HIDDEN = slice(1064, 1576)


def _pack(inputs):
  from tinygrad.tensor import Tensor
  parts = [inputs[k].cast("float32").reshape(-1) for k in ("img", "big_img", "features_buffer", "desire_pulse", "traffic_convention", "action_t")]
  flat = Tensor.cat(*parts)
  hidden = (flat[:512] * 0.001).reshape(1, 512)
  return flat, hidden


def _fake_model(inputs):
  from tinygrad.tensor import Tensor
  flat, hidden = _pack(inputs)
  n = flat.shape[0]
  head = flat[:min(n, HIDDEN.start)]
  out = Tensor.cat(head.pad((0, HIDDEN.start - head.shape[0])), hidden.reshape(-1), Tensor.zeros(OUT_LEN - HIDDEN.stop, device="CPU"))
  return {"outputs": out.reshape(1, -1)}


class _Reference:
  def __init__(self):
    self.state = TemporalInputState(FS, SPEC)

  def run(self, warped, desire, traffic, action_t):
    inputs = self.state.push_and_materialize(warped, desire, traffic, action_t)
    from tinygrad.tensor import Tensor
    t = {k: Tensor(np.ascontiguousarray(v), device="CPU") for k, v in inputs.items()}
    out = _fake_model(t)["outputs"].numpy().reshape(-1)
    self.state.note_hidden_state(out, HIDDEN)
    return out


def test_policy_queues_match_temporal_state():
  from tinygrad.engine.jit import TinyJit
  jit = TinyJit(make_run_policy(_fake_model, SPEC, FS, "CPU"), prune=True)
  runner = PolicyRunner(jit, SPEC, FS, HIDDEN, "CPU")
  ref = _Reference()
  rng = np.random.default_rng(3)
  desire = np.zeros(8, dtype=np.float32)
  for i in range(14):
    warped = rng.integers(0, 256, (2, 6, 8, 16), dtype=np.int64).astype(np.uint8)
    if i in (2, 3, 9):
      desire[:] = 0
      desire[1 + (i % 3)] = 1
    elif i == 5:
      desire[:] = 0
    traffic = np.array([1.0, 0.0], dtype=np.float32) if i % 2 else np.array([0.0, 1.0], dtype=np.float32)
    action_t = np.array([0.1 * i, 0.2], dtype=np.float32)
    got = runner.run(warped, desire, traffic, action_t)
    want = ref.run(warped, desire, traffic, action_t)
    np.testing.assert_array_equal(got, want, err_msg=f"frame {i}")


def test_layouts():
  shapes, sizes = packed_layout(SPEC)
  assert list(shapes) == ["desire", "traffic_convention", "action_t", "prev_feat"]
  assert sum(sizes) == 8 + 2 + 2 + 512
  q = queue_shapes(SPEC, FS)
  assert q["img_q"][0] == (5, 6, 8, 16) and q["feat_q"][0] == (96, 1, 512) and q["desire_q"][0] == (100, 1, 8)
