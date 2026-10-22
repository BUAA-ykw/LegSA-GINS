"""WGS84 transformations used to construct controlled position perturbations."""

from __future__ import annotations
import math

def _ecef(lat_deg: float, lon_deg: float, height_m: float) -> tuple[float, float, float]:
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    return ((n + height_m) * cos_lat * cos_lon, (n + height_m) * cos_lat * sin_lon, (n * (1.0 - e2) + height_m) * sin_lat)

def _ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert WGS84 ECEF to latitude/longitude degrees and ellipsoidal height."""
    a = 6378137.0
    f = 1.0 / 298.257223563
    b = a * (1.0 - f)
    e2 = f * (2.0 - f)
    ep2 = (a * a - b * b) / (b * b)
    p = math.hypot(x, y)
    if p < 1e-12:
        latitude = math.copysign(math.pi / 2.0, z)
        longitude = 0.0
        height = abs(z) - b
        return (math.degrees(latitude), math.degrees(longitude), height)
    longitude = math.atan2(y, x)
    theta = math.atan2(z * a, p * b)
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    latitude = math.atan2(z + ep2 * b * sin_theta ** 3, p - e2 * a * cos_theta ** 3)
    sin_latitude = math.sin(latitude)
    prime_vertical = a / math.sqrt(1.0 - e2 * sin_latitude * sin_latitude)
    height = p / math.cos(latitude) - prime_vertical
    return (math.degrees(latitude), math.degrees(longitude), height)
