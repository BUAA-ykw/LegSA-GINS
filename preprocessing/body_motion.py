"""Robot inclination and heading-dependent horizontal velocity."""

from __future__ import annotations
import copy
import numpy as np
K_HV = 1.0 / 0.962142
SIGMA_HV = 0.132838

def interpolate_a1(times, a1_times, a1_yaw_deg, *, maximum_gap_s=1.2):
    """Unwrapped linear interpolation, closed observations and open long gaps."""
    (times, at, yaw) = (np.asarray(times, float), np.asarray(a1_times, float), np.asarray(a1_yaw_deg, float))
    if len(at) != len(yaw) or not np.isfinite(at).all() or (not np.isfinite(yaw).all()) or np.any(np.diff(at) <= 0):
        raise ValueError('A1 must be finite, ordered, unique observation epochs')
    if not len(at):
        return (None, np.zeros(len(times), bool))
    values = np.interp(times, at, np.unwrap(np.deg2rad(yaw)))
    support = (times >= at[0]) & (times <= at[-1])
    if len(at) > 1:
        index = np.clip(np.searchsorted(at, times, side='right') - 1, 0, len(at) - 2)
        support &= ~((np.diff(at)[index] > maximum_gap_s) & (times > at[index]) & (times < at[index + 1]))
    return (values, support)

def rotate_flu_to_ned(velocity, roll, pitch, yaw):
    v = np.asarray(velocity, float) * [1.0, -1.0, -1.0]
    (cr, sr, cp, sp) = (np.cos(roll), np.sin(roll), np.cos(-np.asarray(pitch)), np.sin(-np.asarray(pitch)))
    (cy, sy) = (np.cos(yaw), np.sin(yaw))
    return np.column_stack((cy * cp * v[:, 0] + (cy * sp * sr - sy * cr) * v[:, 1] + (cy * sp * cr + sy * sr) * v[:, 2], sy * cp * v[:, 0] + (sy * sp * sr + cy * cr) * v[:, 1] + (sy * sp * cr - cy * sr) * v[:, 2]))

def correct_hv(table, body, yaw_rows, *, timing_fault=False):
    """Preserve row identities/payloads; invalid measurements never update."""
    result = copy.deepcopy(table)
    extra = ('valid', 'go2_source_valid', 'a1_heading_valid')
    result.fields = tuple(dict.fromkeys((*result.fields, *extra)))
    indices = np.array([int(r['__canonical541_original_row_id']) if timing_fault else i for (i, r) in enumerate(result.rows)])
    if len(indices) != len(body) or set(indices) != set(range(len(body))):
        raise ValueError('HV original row identity mismatch')
    a1 = [r for r in yaw_rows if str(r.get('valid', '1')).lower() in {'1', 'true'}]
    at = np.array([float(r['time']) for r in a1])
    if not timing_fault:
        at = np.rint(at * 1000) / 1000.0
    times = np.array([float(r['time']) for r in result.rows])
    (yaw, support) = interpolate_a1(times, at, [float(r['yaw_deg']) for r in a1])
    transformed = None if yaw is None else K_HV * rotate_flu_to_ned(body[indices, 3:6], body[indices, 1], body[indices, 2], yaw)
    for (i, row) in enumerate(result.rows):
        source_valid = str(row.get('go2_source_valid', str(row.get('valid', '1')).lower() in {'1', 'true'})).lower() in {'true', '1'}
        if transformed is not None:
            (row['vn'], row['ve']) = (repr(float(x)) for x in transformed[i])
        row.update(std_vn=str(SIGMA_HV), std_ve=str(SIGMA_HV), valid=str(int(source_valid and support[i])), update_flag=str(bool(source_valid and support[i])), go2_source_valid=str(source_valid), a1_heading_valid=str(bool(support[i])), frame_candidate='FLU_to_FRD_Go2_RP_injected_A1_NED', prior_policy='SENSOR_MODEL_V21_horizontal_weak_prior')
        if not support[i]:
            row['reason_codes'] = 'a1_heading_unavailable_no_update'
    return result

def correct_rp(table, raw):
    result = copy.deepcopy(table)
    times = np.array([float(r['time']) for r in result.rows])
    indices = np.searchsorted(raw[:, 0], times)
    right = np.minimum(indices, len(raw) - 1)
    left = np.maximum(right - 1, 0)
    indices = np.where(abs(raw[left, 0] - times) <= abs(raw[right, 0] - times), left, right)
    if np.max(abs(raw[indices, 0] - times)) >= 1e-09:
        raise ValueError('RP raw row identity mismatch')
    for (row, source) in zip(result.rows, raw[indices]):
        if abs(float(row['roll_rad']) - source[1]) > 1e-12 or abs(float(row['pitch_rad']) - source[2]) > 1e-12:
            raise ValueError('RP source frame tokens differ from pinned raw')
        row['pitch_rad'] = repr(float(-source[2]))
        if any((abs(float(row[k]) - np.deg2rad(1.6)) > 1e-12 for k in ('std_roll_rad', 'std_pitch_rad'))):
            raise ValueError('RP frozen std differs from 1.6 degrees')
    return result
