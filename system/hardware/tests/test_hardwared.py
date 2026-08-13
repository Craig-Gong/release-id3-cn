from types import SimpleNamespace

from openpilot.system.hardware.hardwared import (
  ALLOWED_TICI_BRANCHES,
  CAN_STARTUP_RECOVERY_COOLDOWN,
  CAN_STARTUP_RECOVERY_DELAY,
  CAN_STARTUP_RECOVERY_MAX_ATTEMPTS,
  CanStartupRecovery,
  is_supported_tici_branch,
  meb_ignition_from_can,
)


def test_beta_pq_allowed_for_tici():
  metadata = SimpleNamespace(channel="beta-pq", channel_type="dev")
  assert "beta-pq" in ALLOWED_TICI_BRANCHES
  assert is_supported_tici_branch(metadata)


def test_tici_channel_type_allowed():
  metadata = SimpleNamespace(channel="random-branch", channel_type="tici")
  assert is_supported_tici_branch(metadata)


def test_unsupported_branch_allowed_on_c3xl_fork():
  metadata = SimpleNamespace(channel="random-branch", channel_type="dev")
  assert is_supported_tici_branch(metadata)


def test_release_id3_cn_allowed_for_tici():
  metadata = SimpleNamespace(channel="release-id3-cn", channel_type="feature")
  assert "release-id3-cn" in ALLOWED_TICI_BRANCHES
  assert is_supported_tici_branch(metadata)


def test_meb_ignition_from_klemmens_status():
  pkt = SimpleNamespace(can=[SimpleNamespace(address=0x3C0, dat=bytes([0, 0, 0x02, 0]), src=0)])
  on, ts = meb_ignition_from_can([pkt], 10.0, None)
  assert on and ts == 10.0
  on, ts = meb_ignition_from_can([], 11.5, ts)
  assert on
  on, _ = meb_ignition_from_can([], 13.0, ts)
  assert not on


def recovery_update(recovery: CanStartupRecovery, now: float, **kwargs) -> bool:
  defaults = {
    "ignition": True,
    "started": True,
    "engaged": False,
    "car_state_alive": True,
    "can_timeout": True,
    "v_ego": 0.,
  }
  return recovery.update(now, **(defaults | kwargs))


def test_can_startup_recovery_requires_persistent_timeout():
  recovery = CanStartupRecovery()
  assert not recovery_update(recovery, 10.)
  assert not recovery_update(recovery, 10. + CAN_STARTUP_RECOVERY_DELAY - 0.1)
  assert recovery_update(recovery, 10. + CAN_STARTUP_RECOVERY_DELAY)


def test_can_startup_recovery_only_when_safe():
  for unsafe_state in (
    {"started": False},
    {"engaged": True},
    {"car_state_alive": False},
    {"can_timeout": False},
    {"v_ego": 0.2},
  ):
    recovery = CanStartupRecovery()
    assert not recovery_update(recovery, 10., **unsafe_state)
    assert not recovery_update(recovery, 10. + CAN_STARTUP_RECOVERY_DELAY, **unsafe_state)


def test_can_startup_recovery_is_bounded_and_resets_next_ignition():
  recovery = CanStartupRecovery()
  now = 10.
  for _ in range(CAN_STARTUP_RECOVERY_MAX_ATTEMPTS):
    assert not recovery_update(recovery, now)
    now += CAN_STARTUP_RECOVERY_DELAY
    assert recovery_update(recovery, now)
    now += CAN_STARTUP_RECOVERY_COOLDOWN

  assert not recovery_update(recovery, now)
  assert not recovery_update(recovery, now + CAN_STARTUP_RECOVERY_DELAY)

  assert not recovery_update(recovery, now + 10., ignition=False)
  assert not recovery_update(recovery, now + 11.)
  assert recovery_update(recovery, now + 11. + CAN_STARTUP_RECOVERY_DELAY)
