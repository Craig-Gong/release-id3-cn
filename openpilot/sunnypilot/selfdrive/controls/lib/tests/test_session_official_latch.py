"""ID.3 product latch must land on Official even when tip Mode=1 is persisted."""
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends import session as sess
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import BackendId


class _FakeParams:
  def __init__(self, store: dict):
    self.store = store

  def get(self, key, return_default=False):
    if key in self.store:
      return self.store[key]
    if return_default and key == sess.DESIRED_BACKEND_PARAM:
      return 0
    return None

  def put(self, key, value, block=True):
    self.store[key] = value

  def remove(self, key):
    self.store.pop(key, None)


class TestSessionOfficialLatch(unittest.TestCase):
  def setUp(self):
    self._tmpdir = tempfile.TemporaryDirectory()
    self.addCleanup(self._tmpdir.cleanup)
    self._lock = Path(self._tmpdir.name) / "backend.lock"
    self._patch = mock.patch.object(sess, "BACKEND_LOCK_PATH", str(self._lock))
    self._patch.start()
    self.addCleanup(self._patch.stop)

  def test_mode_one_heals_to_official(self):
    params = _FakeParams({sess.DESIRED_BACKEND_PARAM: int(BackendId.EXPERIMENTAL)})
    backend = sess.latch_active_backend(params)
    self.assertEqual(backend.id, BackendId.OFFICIAL)
    self.assertEqual(params.store[sess.DESIRED_BACKEND_PARAM], int(BackendId.OFFICIAL))
    self.assertEqual(params.store[sess.ACTIVE_BACKEND_PARAM], int(BackendId.OFFICIAL))

  def test_stale_active_experimental_prefers_official(self):
    params = _FakeParams({
      sess.DESIRED_BACKEND_PARAM: int(BackendId.EXPERIMENTAL),
      sess.ACTIVE_BACKEND_PARAM: int(BackendId.EXPERIMENTAL),
    })
    backend = sess.latch_active_backend(params)
    self.assertEqual(backend.id, BackendId.OFFICIAL)
    self.assertEqual(params.store[sess.ACTIVE_BACKEND_PARAM], int(BackendId.OFFICIAL))
    self.assertEqual(params.store[sess.DESIRED_BACKEND_PARAM], int(BackendId.OFFICIAL))

  def test_official_passthrough(self):
    params = _FakeParams({sess.DESIRED_BACKEND_PARAM: int(BackendId.OFFICIAL)})
    backend = sess.latch_active_backend(params)
    self.assertEqual(backend.id, BackendId.OFFICIAL)

  def test_tn_no_dec_passthrough(self):
    # tip Tesla path; must not be force-remapped by the Mode=1 heal.
    params = _FakeParams({sess.DESIRED_BACKEND_PARAM: int(BackendId.TN_NO_DEC)})
    backend = sess.latch_active_backend(params)
    self.assertEqual(backend.id, BackendId.TN_NO_DEC)
    self.assertEqual(params.store[sess.ACTIVE_BACKEND_PARAM], int(BackendId.TN_NO_DEC))


if __name__ == "__main__":
  unittest.main()
