"""Contact-inertial sensor stream for the heading-rotation experiment."""

from __future__ import annotations
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Iterator, Sequence
from preprocessing.contact_settings import FROZEN_CONTACT_DWELL_SECONDS, FROZEN_CONTACT_OFF_THRESHOLDS, FROZEN_CONTACT_ON_THRESHOLDS, NATIVE_TO_CANONICAL, PRODUCTION_BY2_PREFIX_END_EXCLUSIVE
PRODUCTION_H5_RECORD_COUNT = 63277
H5_RECORD_COUNT = PRODUCTION_H5_RECORD_COUNT
H5_SENSOR_TO_BODY_ROLL_DEG = -1.0

class HartleyH5Error(ValueError):
    pass

@dataclass(frozen=True)
class H5Record:
    timestamp_ns: int
    gyro_body: tuple[float, float, float]
    accel_body: tuple[float, float, float]
    foot_force_canonical: tuple[float, float, float, float]
    foot_position_body_canonical: tuple[tuple[float, float, float], ...]
    contact_mask: int
    add_mask: int
    remove_mask: int

@dataclass(frozen=True)
class SemanticReadCounters:
    timestamp_values: int = 0
    gyro_values: int = 0
    accel_values: int = 0
    foot_force_values: int = 0
    foot_position_values: int = 0
    forbidden_or_audit_values_decoded: int = 0
    lexically_skipped_lines: int = 0

def _finite_number(text: str, label: str) -> float:
    try:
        value = float(text.strip().split('#', 1)[0])
    except ValueError as exc:
        raise HartleyH5Error(f'malformed allowed numeric field {label}') from exc
    if not math.isfinite(value):
        raise HartleyH5Error(f'nonfinite allowed numeric field {label}')
    return value

def _integer(text: str, label: str) -> int:
    value = text.strip().split('#', 1)[0]
    if not value or any((character not in '+-0123456789' for character in value)):
        raise HartleyH5Error(f'malformed allowed integer field {label}')
    return int(value)

def _inline_array(text: str, label: str) -> list[float]:
    value = text.strip().split('#', 1)[0].strip()
    if not value.startswith('[') or not value.endswith(']'):
        raise HartleyH5Error(f'allowed field {label} is not an inline array')
    body = value[1:-1].strip()
    return [] if not body else [_finite_number(item, label) for item in body.split(',')]

def _rotation_x_minus_one_degree(values: Sequence[float]) -> tuple[float, float, float]:
    angle = math.radians(H5_SENSOR_TO_BODY_ROLL_DEG)
    (c, s) = (math.cos(angle), math.sin(angle))
    (x, y, z) = values
    return (x, c * y - s * z, s * y + c * z)

def _parse_allowed_record(lines: Iterable[str], record_index: int) -> tuple[int, list[float], list[float], list[float], list[float], SemanticReadCounters]:
    sec: int | None = None
    nanosec: int | None = None
    gyro: list[float] = []
    accel: list[float] = []
    force: list[float] = []
    feet: list[float] = []
    top = ''
    active = ''
    counts = {'timestamp': 0, 'gyro': 0, 'accel': 0, 'force': 0, 'feet': 0, 'skip': 0}
    targets = {'gyroscope': gyro, 'accelerometer': accel, 'foot_force': force, 'foot_position_body': feet}
    for raw in lines:
        stripped = raw.strip()
        if not stripped or stripped.startswith('#') or stripped in ('---', '...'):
            continue
        indent = len(raw) - len(raw.lstrip(' '))
        if indent == 0 and ':' in stripped and (not stripped.startswith('-')):
            (key, value) = stripped.split(':', 1)
            key = key.strip()
            (top, active) = (key, '')
            if key in ('stamp', 'imu_state'):
                continue
            if key in ('foot_force', 'foot_position_body'):
                if value.strip():
                    targets[key].extend(_inline_array(value, key))
                else:
                    active = key
                continue
            top = ''
            counts['skip'] += 1
            continue
        if top == 'stamp' and ':' in stripped and (not stripped.startswith('-')):
            (key, value) = stripped.split(':', 1)
            if key.strip() == 'sec':
                sec = _integer(value, 'stamp.sec')
                counts['timestamp'] += 1
            elif key.strip() == 'nanosec':
                nanosec = _integer(value, 'stamp.nanosec')
                counts['timestamp'] += 1
            else:
                counts['skip'] += 1
            continue
        if top == 'imu_state' and ':' in stripped and (not stripped.startswith('-')):
            (key, value) = stripped.split(':', 1)
            key = key.strip()
            active = ''
            if key in ('gyroscope', 'accelerometer'):
                if value.strip():
                    targets[key].extend(_inline_array(value, key))
                else:
                    active = key
            else:
                counts['skip'] += 1
            continue
        if active and stripped.startswith('-'):
            targets[active].append(_finite_number(stripped[1:], active))
        else:
            counts['skip'] += 1
    shapes = (len(gyro), len(accel), len(force), len(feet))
    if sec is None or nanosec is None or shapes != (3, 3, 4, 12):
        raise HartleyH5Error(f'H5 record {record_index} allowed-field shape failure: {shapes}')
    if not 0 <= nanosec < 1000000000:
        raise HartleyH5Error(f'H5 record {record_index} nanosec outside [0,1e9)')
    counter = SemanticReadCounters(timestamp_values=counts['timestamp'], gyro_values=len(gyro), accel_values=len(accel), foot_force_values=len(force), foot_position_values=len(feet), forbidden_or_audit_values_decoded=0, lexically_skipped_lines=counts['skip'])
    return (sec * 1000000000 + nanosec, gyro, accel, force, feet, counter)

def _sum_counters(lhs: SemanticReadCounters, rhs: SemanticReadCounters) -> SemanticReadCounters:
    return SemanticReadCounters(**{key: getattr(lhs, key) + getattr(rhs, key) for key in asdict(lhs)})

class ContactHysteresis:

    def __init__(self) -> None:
        self.active = [False] * 4
        self.initialized = False
        self.candidate: list[bool | None] = [None] * 4
        self.candidate_start_seconds: list[float | None] = [None] * 4

    def update(self, timestamp_ns: int, force: Sequence[float]) -> tuple[int, int, int]:
        before = list(self.active)
        timestamp_seconds = timestamp_ns / 1000000000.0
        if not self.initialized:
            self.active = [value >= 0.5 * (FROZEN_CONTACT_ON_THRESHOLDS[index] + FROZEN_CONTACT_OFF_THRESHOLDS[index]) for (index, value) in enumerate(force)]
            self.initialized = True
            mask = sum((1 << index for (index, value) in enumerate(self.active) if value))
            return (mask, mask, 0)
        for (index, value) in enumerate(force):
            desired: bool | None = None
            if self.active[index] and value <= FROZEN_CONTACT_OFF_THRESHOLDS[index]:
                desired = False
            elif not self.active[index] and value >= FROZEN_CONTACT_ON_THRESHOLDS[index]:
                desired = True
            if desired is None:
                self.candidate[index] = None
                self.candidate_start_seconds[index] = None
                continue
            if self.candidate[index] != desired:
                self.candidate[index] = desired
                self.candidate_start_seconds[index] = timestamp_seconds
            assert self.candidate_start_seconds[index] is not None
            elapsed = timestamp_seconds - self.candidate_start_seconds[index]
            if elapsed + 1e-12 >= FROZEN_CONTACT_DWELL_SECONDS:
                self.active[index] = desired
                self.candidate[index] = None
                self.candidate_start_seconds[index] = None
        mask = sum((1 << index for (index, value) in enumerate(self.active) if value))
        add = sum((1 << index for index in range(4) if self.active[index] and (not before[index])))
        remove = sum((1 << index for index in range(4) if before[index] and (not self.active[index])))
        return (mask, add, remove)

def stream_h5_prefix(source: Path, *, prefix_end_exclusive: int=PRODUCTION_BY2_PREFIX_END_EXCLUSIVE, expected_records: int=H5_RECORD_COUNT) -> tuple[Iterator[H5Record], dict[str, Any]]:
    counters = SemanticReadCounters()
    summary: dict[str, Any] = {'semantic_read_counters': None}

    def iterator() -> Iterator[H5Record]:
        nonlocal counters
        detector = ContactHysteresis()
        previous_ns: int | None = None
        record_lines: list[str] = []
        byte_offset = 0
        emitted = 0
        with source.open('rb') as handle:
            while byte_offset < prefix_end_exclusive:
                binary = handle.readline()
                if not binary:
                    raise HartleyH5Error('source ended inside locked prefix')
                byte_offset += len(binary)
                if byte_offset > prefix_end_exclusive:
                    raise HartleyH5Error('prefix boundary splits a physical line')
                try:
                    line = binary.decode('utf-8', errors='strict')
                except UnicodeDecodeError as exc:
                    raise HartleyH5Error('raw prefix is not strict UTF-8') from exc
                if line.strip() == 'stamp:' and (not line[:1].isspace()):
                    if record_lines:
                        raise HartleyH5Error('record start encountered before delimiter')
                    record_lines = [line]
                    continue
                if not record_lines:
                    continue
                record_lines.append(line)
                if line.strip() != '---':
                    continue
                emitted += 1
                (timestamp_ns, gyro, accel, force_native, feet_native, row_counts) = _parse_allowed_record(record_lines, emitted)
                counters = _sum_counters(counters, row_counts)
                if previous_ns is not None and timestamp_ns <= previous_ns:
                    raise HartleyH5Error('timestamps are not strictly increasing integer nanoseconds')
                previous_ns = timestamp_ns
                force = tuple((force_native[index] for index in NATIVE_TO_CANONICAL))
                grouped = tuple((tuple(feet_native[index:index + 3]) for index in range(0, 12, 3)))
                feet = tuple((grouped[index] for index in NATIVE_TO_CANONICAL))
                (mask, add, remove) = detector.update(timestamp_ns, force_native)

                def canonical_mask(native_mask: int) -> int:
                    return sum((1 << canonical for (canonical, native) in enumerate(NATIVE_TO_CANONICAL) if native_mask & 1 << native))
                yield H5Record(timestamp_ns, _rotation_x_minus_one_degree(gyro), _rotation_x_minus_one_degree(accel), force, feet, canonical_mask(mask), canonical_mask(add), canonical_mask(remove))
                record_lines = []
        if record_lines:
            raise HartleyH5Error('locked prefix ended with an undelimited record')
        if byte_offset != prefix_end_exclusive or emitted != expected_records:
            raise HartleyH5Error('locked prefix byte/record count mismatch')
        summary['semantic_read_counters'] = asdict(counters)
        summary['record_count'] = emitted
        summary['first_timestamp_ns'] = None if emitted == 0 else summary.get('first_timestamp_ns')
    return (iterator(), summary)
