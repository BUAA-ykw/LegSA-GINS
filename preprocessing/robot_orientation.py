"""Quaternion and inclination conventions in robot messages."""

from __future__ import annotations
import math
from typing import Any

def _as_float(value: Any, default: float=0.0) -> float:
    if value is None or str(value).strip() == '':
        return default
    return float(value)

def quaternion_wxyz_to_rpy(q: tuple[float, float, float, float] | list[float]) -> tuple[float, float, float]:
    (w, x, y, z) = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)

def _wrap_rad(value: float) -> float:
    wrapped = (value + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped == math.pi:
        return -math.pi
    return wrapped

def check_quaternion_rpy_consistency(row: dict[str, Any]) -> dict[str, Any]:
    quat = [_as_float(row.get('quat_w', row.get('quat_0'))), _as_float(row.get('quat_x', row.get('quat_1'))), _as_float(row.get('quat_y', row.get('quat_2'))), _as_float(row.get('quat_z', row.get('quat_3')))]
    norm = math.sqrt(sum((value * value for value in quat)))
    if norm <= 0.0:
        return {'quaternion_rpy_consistency_status': 'evidence_missing', 'max_quat_rpy_delta_rad': None}
    quat = [value / norm for value in quat]
    (roll, pitch, yaw) = quaternion_wxyz_to_rpy(quat)
    deltas = [abs(_wrap_rad(roll - _as_float(row.get('roll_rad')))), abs(_wrap_rad(pitch - _as_float(row.get('pitch_rad')))), abs(_wrap_rad(yaw - _as_float(row.get('yaw_rad'))))]
    max_delta = max(deltas)
    return {'quaternion_rpy_consistency_status': 'passed' if max_delta < 0.05 else 'suspicious', 'max_quat_rpy_delta_rad': max_delta}
