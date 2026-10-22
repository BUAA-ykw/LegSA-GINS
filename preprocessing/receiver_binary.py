"""Receiver-position decoding and WGS84 conversion."""

from __future__ import annotations
import math
import struct
from dataclasses import dataclass
from typing import Sequence
import numpy as np

class RawBackendError(ValueError):
    pass

@dataclass(frozen=True)
class NavHpPosEcefEpoch:
    """UBX-NAV-HPPOSECEF v0 with the exact documented integer components."""
    itow_ms: int
    version: int
    ecef_cm: tuple[int, int, int]
    ecef_hp_0p1mm: tuple[int, int, int]
    position_accuracy_0p1mm: int
    reserved1: bytes = b'\x00\x00\x00'
    reserved2: int = 0

    @property
    def position_ecef_m(self) -> np.ndarray:
        return np.asarray(self.ecef_cm, dtype=float) * 0.01 + np.asarray(self.ecef_hp_0p1mm, dtype=float) * 0.0001

    @property
    def position_accuracy_m(self) -> float:
        return self.position_accuracy_0p1mm * 0.0001

def decode_nav_hpposecef(payload: bytes) -> NavHpPosEcefEpoch:
    """Decode UBX-NAV-HPPOSECEF (0x01 0x13), protocol-29 version 0.

    Units and high-precision bounds follow UBX-18053584 R02 section 5.14.6.
    The receiver position remains diagnostic-only; this function merely
    preserves the source fields and applies their documented unit conversion.
    """
    if len(payload) != 28:
        raise RawBackendError('NAV-HPPOSECEF record boundary mismatch')
    version = payload[0]
    if version != 0:
        raise RawBackendError('unsupported NAV-HPPOSECEF version')
    reserved1 = payload[1:4]
    (itow_ms, x_cm, y_cm, z_cm, x_hp, y_hp, z_hp, reserved2, p_acc) = struct.unpack_from('<IiiibbbBI', payload, 4)
    high_precision = (x_hp, y_hp, z_hp)
    if any((value < -99 or value > 99 for value in high_precision)):
        raise RawBackendError('NAV-HPPOSECEF high-precision component outside -99..99')
    epoch = NavHpPosEcefEpoch(itow_ms=itow_ms, version=version, ecef_cm=(x_cm, y_cm, z_cm), ecef_hp_0p1mm=high_precision, position_accuracy_0p1mm=p_acc, reserved1=reserved1, reserved2=reserved2)
    position = epoch.position_ecef_m
    radius = float(np.linalg.norm(position))
    if np.any(~np.isfinite(position)) or not 1000000.0 <= radius <= 100000000.0:
        raise RawBackendError('NAV-HPPOSECEF ECEF position outside physical bounds')
    return epoch

def ecef_to_geodetic(receiver_ecef_m: Sequence[float]) -> tuple[float, float, float]:
    receiver = np.asarray(receiver_ecef_m, dtype=float)
    if receiver.shape != (3,) or np.any(~np.isfinite(receiver)):
        raise RawBackendError('invalid receiver ECEF')
    (x, y, z) = receiver
    longitude = math.atan2(y, x)
    semi_major = 6378137.0
    eccentricity_sq = 0.0066943799901413165
    horizontal = math.hypot(x, y)
    if horizontal < 1.0:
        raise RawBackendError('receiver ECEF is too close to polar axis/origin')
    latitude = math.atan2(z, horizontal * (1.0 - eccentricity_sq))
    height = 0.0
    for _ in range(12):
        sin_lat = math.sin(latitude)
        radius = semi_major / math.sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat)
        height = horizontal / math.cos(latitude) - radius
        updated = math.atan2(z, horizontal * (1.0 - eccentricity_sq * radius / (radius + height)))
        if abs(updated - latitude) < 1e-13:
            latitude = updated
            break
        latitude = updated
    return (latitude, longitude, height)
