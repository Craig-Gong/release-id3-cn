import math
from dataclasses import dataclass

from opendbc.sunnypilot.car.tesla.ars408.constants import ARS408_MAX_OBJECTS


def _finite(*values: float) -> None:
  if not all(math.isfinite(value) for value in values):
    raise ValueError("ARS408 values must be finite")


def _raw_id(raw_id: int) -> None:
  if not 0 <= raw_id <= 255:
    raise ValueError(f"invalid ARS408 raw object ID: {raw_id}")


@dataclass(frozen=True, slots=True)
class ObjectStatus:
  object_count: int
  measurement_counter: int
  interface_version: int

  def __post_init__(self) -> None:
    if not 0 <= self.object_count <= 255:
      raise ValueError(f"invalid ARS408 object count: {self.object_count}")
    if not 0 <= self.measurement_counter <= 65535:
      raise ValueError(f"invalid ARS408 measurement counter: {self.measurement_counter}")
    if not 0 <= self.interface_version <= 15:
      raise ValueError(f"invalid ARS408 interface version: {self.interface_version}")

  @property
  def protocol_valid(self) -> bool:
    return self.object_count <= ARS408_MAX_OBJECTS and self.interface_version == 1


@dataclass(frozen=True, slots=True)
class ObjectGeneral:
  raw_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  yv_rel: float
  rcs: float
  dynamic_property: int

  def __post_init__(self) -> None:
    _raw_id(self.raw_id)
    _finite(self.d_rel, self.y_rel, self.v_rel, self.yv_rel, self.rcs)
    if not 0 <= self.dynamic_property <= 7:
      raise ValueError(f"invalid ARS408 dynamic property: {self.dynamic_property}")


@dataclass(frozen=True, slots=True)
class ObjectQuality:
  raw_id: int
  probability: int
  measurement_state: int

  def __post_init__(self) -> None:
    _raw_id(self.raw_id)
    if not 0 <= self.probability <= 7:
      raise ValueError(f"invalid ARS408 existence probability: {self.probability}")
    if not 0 <= self.measurement_state <= 7:
      raise ValueError(f"invalid ARS408 measurement state: {self.measurement_state}")


@dataclass(frozen=True, slots=True)
class ObjectExtended:
  raw_id: int
  a_rel: float
  object_class: int

  def __post_init__(self) -> None:
    _raw_id(self.raw_id)
    _finite(self.a_rel)
    if not 0 <= self.object_class <= 7:
      raise ValueError(f"invalid ARS408 object class: {self.object_class}")


@dataclass(frozen=True, slots=True)
class AssembledObject:
  general: ObjectGeneral
  quality: ObjectQuality
  extended: ObjectExtended | None = None

  def __post_init__(self) -> None:
    ids = {self.general.raw_id, self.quality.raw_id}
    if self.extended is not None:
      ids.add(self.extended.raw_id)
    if len(ids) != 1:
      raise ValueError("ARS408 object parts have different raw IDs")

  @property
  def raw_id(self) -> int:
    return self.general.raw_id


@dataclass(frozen=True, slots=True)
class CycleResult:
  status: ObjectStatus
  objects: tuple[AssembledObject, ...]
  exact: bool
  invalid: bool
  general_count: int
  quality_count: int
  extended_count: int
  duplicate_ids: tuple[int, ...] = ()


@dataclass(frozen=True, slots=True)
class RadarStateSnapshot:
  interference: bool
  voltage_error: bool
  temporary_error: bool
  temperature_error: bool
  persistent_error: bool
  sensor_id: int
  output_type: int
  quality_enabled: bool
  extended_enabled: bool
  motion_rx_state: int
  max_distance_m: int
  nvm_read_status: int
  nvm_write_status: int
  sort_index: int
  ctrl_relay_enabled: bool
  rcs_threshold: int

  def __post_init__(self) -> None:
    if not 0 <= self.sensor_id <= 7:
      raise ValueError(f"invalid ARS408 sensor ID: {self.sensor_id}")
    if not 0 <= self.output_type <= 3:
      raise ValueError(f"invalid ARS408 output type: {self.output_type}")
    if not 0 <= self.motion_rx_state <= 3:
      raise ValueError(f"invalid ARS408 motion RX state: {self.motion_rx_state}")
    if not 0 <= self.max_distance_m <= 2046:
      raise ValueError(f"invalid ARS408 maximum distance: {self.max_distance_m}")
    if not 0 <= self.nvm_read_status <= 1 or not 0 <= self.nvm_write_status <= 1:
      raise ValueError("invalid ARS408 NVM status")
    if not 0 <= self.sort_index <= 2 or not 0 <= self.rcs_threshold <= 7:
      raise ValueError("invalid ARS408 sort index or RCS threshold")


@dataclass(frozen=True, slots=True)
class FilterStateHeader:
  cluster_filter_count: int
  object_filter_count: int

  def __post_init__(self) -> None:
    if not 0 <= self.cluster_filter_count <= 255 or not 0 <= self.object_filter_count <= 255:
      raise ValueError("invalid ARS408 filter count")


@dataclass(frozen=True, slots=True)
class FilterStateRecord:
  index: int
  active: bool
  minimum: float
  maximum: float

  def __post_init__(self) -> None:
    _finite(self.minimum, self.maximum)
    if not 0 <= self.index <= 14:
      raise ValueError(f"invalid ARS408 object filter index: {self.index}")
    if self.minimum > self.maximum:
      raise ValueError("ARS408 filter minimum exceeds maximum")


@dataclass(frozen=True, slots=True)
class TrackedObject:
  raw_id: int
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  yv_rel: float
  a_rel: float
  measured: bool
  object_class: int
  probability: int
  dynamic_property: int


@dataclass(frozen=True, slots=True)
class TrackerResult:
  tracks: tuple[TrackedObject, ...]
  accepted_raw_ids: tuple[int, ...]
  rejection_reasons: tuple[tuple[int, str], ...]
  handover_count: int
  duplicate_suppression_count: int


@dataclass(frozen=True, slots=True)
class DiagnosticErrors:
  can_error: bool = False
  radar_fault: bool = False
  radar_unavailable_temporary: bool = False
  wrong_config: bool = False


@dataclass(frozen=True, slots=True)
class DiagnosticsSnapshot:
  errors: DiagnosticErrors
  radar_state_ready: bool
  radar_state_count: int
  interference_count: int
  exact_cycles: int
  partial_cycles: int
  invalid_cycles: int
  raw_object_count: int
  class_counts: tuple[tuple[int, int], ...]
  probability_counts: tuple[tuple[int, int], ...]
  grace_held_tracks: int
  filter_header: FilterStateHeader | None
  filter_records: tuple[FilterStateRecord, ...]


ParsedFrame = ObjectStatus | ObjectGeneral | ObjectQuality | ObjectExtended | RadarStateSnapshot | FilterStateHeader | FilterStateRecord
