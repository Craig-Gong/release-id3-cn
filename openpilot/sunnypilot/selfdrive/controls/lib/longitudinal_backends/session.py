import fcntl

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import (
  BackendId, BackendSpec, get_backend,
)


DESIRED_BACKEND_PARAM = "LongitudinalPlannerMode"
ACTIVE_BACKEND_PARAM = "ActiveLongitudinalBackend"
BACKEND_LOCK_PATH = "/tmp/longitudinal_backend_session.lock"


def _prefer_official(backend: BackendSpec) -> BackendSpec:
  """ID.3 product: heal tip Mode=1 (Experimental) → Official.

  Experimental skips AccelBoost / empty-road soft cap (helpers alone are not
  enough). TN_NO_DEC and other tip backends stay selectable for tip tests /
  Tesla tooling — they are not the Mode=1 trap this car hits.
  """
  if backend.id != BackendId.EXPERIMENTAL:
    return backend
  return get_backend(BackendId.OFFICIAL)


def latch_active_backend(params) -> BackendSpec:
  """Select one installed provider for the lifetime of an onroad session."""
  # A one-time startup lock covers both reads and the durable typed write.
  # Individual Params operations are atomic, the read/select/write sequence is not.
  with open(BACKEND_LOCK_PATH, "a") as lock_file:
    fcntl.flock(lock_file, fcntl.LOCK_EX)
    active = params.get(ACTIVE_BACKEND_PARAM)
    if active is not None:
      backend = _prefer_official(get_backend(active))
      if int(active) != int(backend.id):
        params.put(ACTIVE_BACKEND_PARAM, int(backend.id), block=True)
        try:
          params.put(DESIRED_BACKEND_PARAM, int(BackendId.OFFICIAL), block=True)
        except Exception:
          pass
      return backend

    desired = get_backend(params.get(DESIRED_BACKEND_PARAM, return_default=True))
    backend = _prefer_official(desired)
    if backend.id != desired.id:
      try:
        from openpilot.common.swaglog import cloudlog
        cloudlog.warning(
          f"longitudinal backend {desired.slug} → official (ID.3 AccelBoost latch)"
        )
      except Exception:
        pass
      try:
        # Heal persistent tip Mode=1 so UI / next boots match Official.
        params.put(DESIRED_BACKEND_PARAM, int(BackendId.OFFICIAL), block=True)
      except Exception:
        pass
    params.put(ACTIVE_BACKEND_PARAM, int(backend.id), block=True)
    return backend


def active_backend_id(params) -> BackendId:
  return latch_active_backend(params).id


def end_longitudinal_session(params, consumers) -> None:
  """Join all old latch writers before clearing the completed session."""
  consumers = tuple(consumers)
  for process in consumers:
    process.stop(block=False)
  for process in consumers:
    process.stop(block=True)
  # Never hold this lock while stopping: a gracefully exiting consumer may
  # still need it to finish its startup write. Join/SIGKILL closes that window.
  with open(BACKEND_LOCK_PATH, "a") as lock_file:
    fcntl.flock(lock_file, fcntl.LOCK_EX)
    params.remove(ACTIVE_BACKEND_PARAM)


def legacy_experimental_enabled(CP, params) -> bool:
  if CP is None or CP.brand != "tesla":
    return False
  active = params.get(ACTIVE_BACKEND_PARAM)
  value = active if active is not None else params.get(DESIRED_BACKEND_PARAM, return_default=True)
  return get_backend(value).id != BackendId.OFFICIAL
