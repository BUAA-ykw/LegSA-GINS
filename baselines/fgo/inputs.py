"""Timing, geodesy, and input arrays for OiSAM."""

from __future__ import annotations
from dataclasses import dataclass
import hashlib
from pathlib import Path
import numpy as np
from preprocessing.robot_log import _array, _scalar

def llh_to_ecef(llh_deg_m):
    (lat, lon) = np.deg2rad(np.asarray(llh_deg_m, float)[:2])
    height = float(llh_deg_m[2])
    (a, e2) = (6378137.0, 0.0066943799901413165)
    n = a / np.sqrt(1.0 - e2 * np.sin(lat) ** 2)
    return np.array([(n + height) * np.cos(lat) * np.cos(lon), (n + height) * np.cos(lat) * np.sin(lon), (n * (1.0 - e2) + height) * np.sin(lat)])

def ecef_to_ned_rotation(llh_deg_m):
    (lat, lon) = np.deg2rad(np.asarray(llh_deg_m, float)[:2])
    (s, c, sl, cl) = (np.sin(lat), np.cos(lat), np.sin(lon), np.cos(lon))
    return np.array([[-s * cl, -s * sl, c], [-sl, cl, 0.0], [-c * cl, -c * sl, -s]])

def ecef_to_llh(ecef):
    """WGS84 inverse for the ordinary terrestrial navigation domain."""
    (x, y, z) = np.asarray(ecef, float)
    (a, e2) = (6378137.0, 0.0066943799901413165)
    rho = np.hypot(x, y)
    latitude = np.arctan2(z, rho * (1.0 - e2))
    for _ in range(10):
        radius = a / np.sqrt(1.0 - e2 * np.sin(latitude) ** 2)
        updated = np.arctan2(z + e2 * radius * np.sin(latitude), rho)
        if abs(updated - latitude) < 1e-14:
            latitude = updated
            break
        latitude = updated
    radius = a / np.sqrt(1.0 - e2 * np.sin(latitude) ** 2)
    height = rho / np.cos(latitude) - radius
    return np.array([np.rad2deg(latitude), np.rad2deg(np.arctan2(y, x)), height])

def select_nodes(gnss: np.ndarray, end_s: float, tolerance_s: float):
    """Nearest original observation at each integer second; never retime it.

    Missing seconds remain explicit expected nodes, with row=None. Selecting a
    timestamp precedes any validity/accuracy checks, so support is not selected
    for a favorable measurement value.
    """
    times = gnss[:, 0]
    nodes = []
    for second in range(int(np.ceil(times[0])), int(np.floor(end_s)) + 1):
        i = int(np.searchsorted(times, second))
        candidates = [j for j in (i - 1, i) if 0 <= j < len(times)]
        j = min(candidates, key=lambda x: (abs(times[x] - second), x))
        if abs(times[j] - second) <= tolerance_s:
            nodes.append((float(times[j]), gnss[j].copy(), second))
        else:
            nodes.append((float(second), None, second))
    return nodes

def exact_imu_intervals(imu, raw_times, max_dt=0.1):
    """Match the frozen builder's retained rows to its unrounded raw stamps."""
    raw_times = np.asarray(raw_times, float)
    raw_dt = np.diff(raw_times)
    if not np.isfinite(raw_times).all() or np.any(raw_dt < 0):
        raise ValueError('RAW_IMU_TIME_INVALID')
    retained = np.flatnonzero((raw_dt > 0) & (raw_dt <= max_dt)) + 1
    if len(retained) != len(imu) or np.any(np.abs(imu[:, 0] - raw_times[retained]) > 5.6e-07):
        raise ValueError('RAW_PROVIDER_IMU_TIME_IDENTITY_MISMATCH')
    return np.column_stack((raw_times[retained - 1], raw_times[retained]))

def read_raw_times(path, base_time):
    """Time-only counterpart of wen_ahrs.read_raw_quaternions.

    Gyro/accelerometer fields are checked for record completeness exactly as in
    the common raw builder. Their values, quaternions, and Go2 odometry never
    become OiSAM measurements here. SHA256 is computed from the same read.
    """
    (digest, times) = (hashlib.sha256(), [])
    counts = {'message_count': 0, 'incomplete_imu_messages': 0}

    def consume(lines):
        if not any((line.strip() for line in lines)):
            return
        counts['message_count'] += 1
        try:
            sec = int(_scalar(lines, ['stamp', 'sec']))
            nsec = int(_scalar(lines, ['stamp', 'nanosec']))
            gyro = np.asarray(_array(lines, ['imu_state', 'gyroscope'], 3), float)
            accel = np.asarray(_array(lines, ['imu_state', 'accelerometer'], 3), float)
            complete = np.isfinite(gyro).all() and np.isfinite(accel).all() and (0 <= nsec < 1000000000)
        except (TypeError, ValueError, OverflowError):
            complete = False
        if complete:
            times.append(sec + nsec * 1e-09 - float(base_time))
        else:
            counts['incomplete_imu_messages'] += 1
    with Path(path).open('rb') as stream:
        message = []
        for payload in stream:
            digest.update(payload)
            line = payload.decode('utf-8-sig').rstrip('\r\n')
            if line.strip() == '---':
                consume(message)
                message = []
            else:
                message.append(line)
        consume(message)
    if len(times) < 2:
        raise ValueError('NO_COMPLETE_RAW_IMU_TIMELINE')
    return (np.asarray(times), digest.hexdigest(), counts)

def imu_pieces(imu: np.ndarray, begin_s: float, end_s: float, max_dt: float, intervals=None):
    """Return (dt, dtheta, dvel) for exact overlap of original intervals.

    Splitting an endpoint uses its frozen current-sample rate. A retained-row
    gap > max_dt is NOT the duration of its last increment: such an interval is
    unsupported and raises. No interpolation or zero/held-rate gap fill occurs.
    """
    if end_s <= begin_s:
        raise ValueError('NONPOSITIVE_PREINTEGRATION_INTERVAL')
    if intervals is None:
        intervals = np.column_stack((imu[:-1, 0], imu[1:, 0]))
        measured = imu[1:]
    else:
        intervals = np.asarray(intervals)
        measured = imu
    if begin_s < intervals[0, 0] or end_s > intervals[-1, 1]:
        raise ValueError('IMU_SUPPORT_UNAVAILABLE')
    first = int(np.searchsorted(intervals[:, 1], begin_s, side='right'))
    last = int(np.searchsorted(intervals[:, 1], end_s, side='left'))
    pieces = []
    covered = 0.0
    cursor = begin_s
    for i in range(first, last + 1):
        (left, right) = intervals[i]
        dt = float(right - left)
        overlap = float(min(right, end_s) - max(left, begin_s))
        if overlap <= 0:
            continue
        if not 0.0 < dt <= max_dt:
            raise ValueError(f'IMU_GAP:{left:.9f}:{right:.9f}')
        if left > cursor + 1e-07:
            raise ValueError(f'IMU_GAP:{cursor:.9f}:{left:.9f}')
        if not np.isfinite(measured[i, 1:]).all():
            raise ValueError('IMU_NONFINITE')
        fraction = overlap / dt
        pieces.append((overlap, measured[i, 1:4] * fraction, measured[i, 4:7] * fraction))
        covered += overlap
        cursor = min(right, end_s)
    if abs(covered - (end_s - begin_s)) > 1e-07:
        raise ValueError('IMU_GAP_INTERVAL_NOT_COVERED')
    return pieces

def initialization_body_rate(imu, timestamp, max_dt, intervals=None):
    """Nearest completed calibrated current-sample rate, never a future rate.

    Production supplies each increment's source-bound, unrounded interval. A
    missing, stale, or gap-adjacent rate leaves initialization unavailable; no
    antenna velocity is then passed off as an IMU velocity.
    """
    if intervals is None:
        intervals = np.column_stack((imu[:-1, 0], imu[1:, 0]))
        (measured, provider_offset) = (imu[1:], 1)
        duration_source = 'exact_synthetic_sample_stamps'
    else:
        intervals = np.asarray(intervals)
        (measured, provider_offset) = (imu, 0)
        duration_source = 'unrounded_source_raw_stamps'
    index = int(np.searchsorted(intervals[:, 1], timestamp, side='right')) - 1
    if index < 0:
        raise ValueError('IMU_INITIAL_BODY_RATE_NO_COMPLETED_INTERVAL')
    (left, right) = intervals[index]
    (dt, age) = (float(right - left), float(timestamp - right))
    if not np.isfinite([left, right, timestamp]).all() or not 0.0 < dt <= max_dt or age > max_dt or (not np.isfinite(measured[index, 1:4]).all()):
        raise ValueError('IMU_INITIAL_BODY_RATE_INVALID_OR_STALE')
    if age > 1e-09:
        if index + 1 == len(intervals):
            raise ValueError('IMU_INITIAL_BODY_RATE_SUPPORT_UNAVAILABLE')
        (next_left, next_right) = intervals[index + 1]
        if next_left > right + 1e-07 or not 0.0 < next_right - next_left <= max_dt:
            raise ValueError('IMU_INITIAL_BODY_RATE_GAP')
    rate = measured[index, 1:4] / dt
    return (rate, {'source': 'CALIBRATED_IMU_dtheta_div_original_current_sample_dt', 'duration_source': duration_source, 'provider_row_index_zero_based': index + provider_offset, 'provider_timestamp_rel_s': float(measured[index, 0]), 'interval_start_rel_s': float(left), 'interval_end_rel_s': float(right), 'effective_timestamp_rel_s': float(right), 'age_at_initialization_s': age, 'body_rate_frd_radps': rate.tolist(), 'earth_rate_or_installation_reapplied': False})

def position_valid(row):
    return row is not None and row[15] > 0.5 and np.isfinite(row[1:7]).all() and np.all(row[4:7] > 0.0)

def initialization_valid(row):
    return position_valid(row) and row[17] > 0.5 and np.isfinite(row[13:15]).all() and (row[14] > 0.0)

@dataclass
class SequenceInput:
    sequence: str
    gnss: np.ndarray
    imu: np.ndarray
    nodes: list
    metadata: dict
    imu_intervals: np.ndarray | None = None

def sha256(path):
    import hashlib
    from pathlib import Path
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
