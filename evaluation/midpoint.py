"""Declared IMU-to-antenna-midpoint transform used for position evaluation."""

from __future__ import annotations
from io import StringIO
import math
from pathlib import Path
import numpy as np
import pandas as pd
from preprocessing.receiver_binary import ecef_to_geodetic

def body_to_ned(rpy_deg):
    """Active Rz(yaw) Ry(pitch) Rx(roll), body FRD to navigation NED."""
    (r, p, y) = np.deg2rad(np.asarray(rpy_deg, dtype=float)).T
    (cr, sr, cp, sp, cy, sy) = (np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y))
    return np.stack((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr), axis=-1).reshape((-1, 3, 3))

def transform_nav(nav, baseline_median_m):
    """Return a new NAV; only LLH columns change under the human physical contract."""
    source = np.asarray(nav, dtype=float)
    if source.ndim != 2 or source.shape[1] < 11 or (not np.isfinite(source).all()):
        raise ValueError('NAV must be finite with time, LLH and roll/pitch/yaw columns')
    if not math.isfinite(baseline_median_m) or baseline_median_m <= 0:
        raise ValueError('Frozen A1 baseline median must be positive and finite')
    out = source.copy()
    lever = np.array([0.03, 0.03 - 0.5 * baseline_median_m, -0.3])
    offset = np.einsum('nij,j->ni', body_to_ned(source[:, 8:11]), lever)
    (lat, lon) = (np.deg2rad(source[:, 2]), np.deg2rad(source[:, 3]))
    (sl, cl, so, co) = (np.sin(lat), np.cos(lat), np.sin(lon), np.cos(lon))
    radius = 6378137.0 / np.sqrt(1 - 0.0066943799901413165 * sl * sl)
    xyz = np.column_stack(((radius + source[:, 4]) * cl * co, (radius + source[:, 4]) * cl * so, (radius * (1 - 0.0066943799901413165) + source[:, 4]) * sl))
    (north, east, down) = offset.T
    xyz += np.column_stack((-sl * co * north - so * east - cl * co * down, -sl * so * north + co * east - cl * so * down, cl * north - sl * down))
    llh = np.asarray([ecef_to_geodetic(p) for p in xyz])
    out[:, 2:4] = np.rad2deg(llh[:, :2])
    out[:, 4] = llh[:, 2]
    return out

def write_transformed_nav(source_path, target_path, transformed):
    """Accept percent/hash comments; validate every numeric row before exclusive write.

    Non-position numeric tokens are preserved verbatim. Numeric layout and values
    must match the transformed array outside the three authorized LLH columns.
    """
    (source_path, target_path) = (Path(source_path), Path(target_path))
    if source_path.is_symlink() or not source_path.is_file():
        raise ValueError('Missing/symlink source NAV')
    changed = np.asarray(transformed, float)
    if changed.ndim != 2 or changed.shape[1] < 11 or (not np.isfinite(changed).all()):
        raise ValueError('Invalid transformed NAV matrix')
    numeric = [line.split() for line in source_path.read_text().splitlines() if line.strip() and (not line.lstrip().startswith(('#', '%')))]
    if len(numeric) != len(changed):
        raise ValueError('NAV row parser disagreement')
    if any((len(tokens) != changed.shape[1] for tokens in numeric)):
        raise ValueError('NAV field count mismatch')
    original = pd.read_csv(StringIO('\n'.join((' '.join(tokens) for tokens in numeric))), sep='\\s+', engine='python', header=None, comment='%').to_numpy(float)
    if not np.isfinite(original).all():
        raise ValueError('NAV contains nonfinite values')
    retained = [k for k in range(changed.shape[1]) if k not in (2, 3, 4)]
    if not np.array_equal(original[:, retained], changed[:, retained]):
        raise ValueError('Unauthorized non-position NAV change')
    if target_path == source_path or target_path.resolve() == source_path.resolve():
        raise ValueError('Source NAV must remain immutable')
    if target_path.is_symlink() or any((p.is_symlink() for p in target_path.parents)):
        raise ValueError('Symlink transformed NAV target')
    with target_path.open('x', encoding='utf-8') as handle:
        for (tokens, row) in zip(numeric, changed):
            tokens[2:5] = [format(x, '.17g') for x in row[2:5]]
            handle.write(' '.join(tokens) + '\n')
