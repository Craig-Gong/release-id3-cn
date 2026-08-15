from opendbc.sunnypilot.car.tesla.ars408.models import (
  AssembledObject, CycleResult, ObjectExtended, ObjectGeneral, ObjectQuality, ObjectStatus,
)


class ARS408CycleAssembler:
  def __init__(self) -> None:
    self._status: ObjectStatus | None = None
    self._general: dict[int, ObjectGeneral] = {}
    self._quality: dict[int, ObjectQuality] = {}
    self._extended: dict[int, ObjectExtended] = {}
    self._duplicate_ids: set[int] = set()

  def push(self, frame: ObjectStatus | ObjectGeneral | ObjectQuality | ObjectExtended) -> CycleResult | None:
    if isinstance(frame, ObjectStatus):
      result = self._close_cycle()
      self._status = frame
      self._general.clear()
      self._quality.clear()
      self._extended.clear()
      self._duplicate_ids.clear()
      return result

    if self._status is None:
      return None
    if isinstance(frame, ObjectGeneral):
      self._store(self._general, frame.raw_id, frame)
    elif isinstance(frame, ObjectQuality):
      self._store(self._quality, frame.raw_id, frame)
    else:
      self._store(self._extended, frame.raw_id, frame)
    return None

  def _store(self, collection: dict, raw_id: int, frame: object) -> None:
    if raw_id in collection:
      self._duplicate_ids.add(raw_id)
      return
    collection[raw_id] = frame

  def _close_cycle(self) -> CycleResult | None:
    if self._status is None:
      return None

    paired_ids = (set(self._general) & set(self._quality)) - self._duplicate_ids
    objects = tuple(
      AssembledObject(self._general[raw_id], self._quality[raw_id], self._extended.get(raw_id))
      for raw_id in sorted(paired_ids)
    )
    expected = self._status.object_count
    exact = self._status.protocol_valid and not self._duplicate_ids and len(self._general) == expected and \
      len(self._quality) == expected and set(self._general) == set(self._quality)
    return CycleResult(
      status=self._status, objects=objects, exact=exact, invalid=not self._status.protocol_valid,
      general_count=len(self._general), quality_count=len(self._quality), extended_count=len(self._extended),
      duplicate_ids=tuple(sorted(self._duplicate_ids)),
    )
