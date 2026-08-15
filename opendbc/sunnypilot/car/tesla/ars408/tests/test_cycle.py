from opendbc.sunnypilot.car.tesla.ars408.cycle import ARS408CycleAssembler
from opendbc.sunnypilot.car.tesla.ars408.models import ObjectExtended, ObjectGeneral, ObjectQuality, ObjectStatus


def general(raw_id: int) -> ObjectGeneral:
  return ObjectGeneral(raw_id, 30.0 + raw_id, 0.2, -1.0, 0.0, 5.0, 0)


def quality(raw_id: int) -> ObjectQuality:
  return ObjectQuality(raw_id, 4, 2)


def close(assembler: ARS408CycleAssembler, next_count: int = 0):
  return assembler.push(ObjectStatus(next_count, 2, 1))


def test_complete_cycle_pairs_all_parts_by_raw_id() -> None:
  assembler = ARS408CycleAssembler()
  assert assembler.push(ObjectStatus(2, 1, 1)) is None
  assembler.push(general(2))
  assembler.push(quality(1))
  assembler.push(ObjectExtended(1, 0.3, 1))
  assembler.push(general(1))
  assembler.push(quality(2))

  result = close(assembler)
  assert result is not None and result.exact and not result.invalid
  assert [obj.raw_id for obj in result.objects] == [1, 2]
  assert result.objects[0].extended == ObjectExtended(1, 0.3, 1)
  assert result.objects[1].extended is None


def test_partial_cycle_salvages_only_general_quality_intersection() -> None:
  assembler = ARS408CycleAssembler()
  assembler.push(ObjectStatus(3, 1, 1))
  assembler.push(general(1))
  assembler.push(quality(1))
  assembler.push(general(2))
  assembler.push(quality(3))

  result = close(assembler)
  assert result is not None and not result.exact and not result.invalid
  assert [obj.raw_id for obj in result.objects] == [1]


def test_duplicate_core_part_is_not_silently_selected() -> None:
  assembler = ARS408CycleAssembler()
  assembler.push(ObjectStatus(1, 1, 1))
  assembler.push(general(1))
  assembler.push(general(1))
  assembler.push(quality(1))

  result = close(assembler)
  assert result is not None and not result.exact
  assert result.objects == ()
  assert result.duplicate_ids == (1,)


def test_invalid_status_and_zero_object_cycle_are_distinct() -> None:
  assembler = ARS408CycleAssembler()
  assembler.push(ObjectStatus(101, 1, 1))
  invalid = close(assembler)
  assert invalid is not None and invalid.invalid

  empty = close(assembler)
  assert empty is not None and empty.exact and not empty.invalid and empty.objects == ()


def test_object_parts_before_status_are_ignored() -> None:
  assembler = ARS408CycleAssembler()
  assert assembler.push(general(1)) is None
  assert assembler.push(quality(1)) is None
  assert assembler.push(ObjectStatus(0, 1, 1)) is None
  result = close(assembler)
  assert result is not None and result.objects == ()
