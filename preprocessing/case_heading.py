"""Lift the fixed controlled heading disturbances to the exact receiver epoch grid."""

from __future__ import annotations
import bisect
import hashlib
import numpy as np
from preprocessing import heading as hp
(YAW, VALID) = (13, 17)
PERTURBED = {'D32', 'D33', 'D34', 'D35', 'D38', 'D41', 'D58', 'D60'}
CELL_DROPOUT = {'D11', 'D12'}
OUTAGE = {'D06', 'D30', 'D31', 'D39', 'D61'}

def digest(payload):
    return hashlib.sha256(payload).hexdigest()

def byte_gate(before, after):
    """Independently prove every non-heading token and whitespace byte identical."""
    (old, width) = hp._rows(before)
    (new, new_width) = hp._rows(after)
    if width != 18 or new_width != 18 or len(old) != len(new):
        raise ValueError('HARD_STOP_V3_PROVIDER_SHAPE')
    if len(before.splitlines(keepends=True)) != len(after.splitlines(keepends=True)):
        raise ValueError('HARD_STOP_V3_PROVIDER_LINE_COUNT')
    changed = [0] * 18
    normalized = after.splitlines(keepends=True)
    for (left, right) in zip(old, new):
        if left['line_index'] != right['line_index']:
            raise ValueError('HARD_STOP_V3_PROVIDER_ROW_IDENTITY')
        for (column, (a, b)) in enumerate(zip(left['tokens'], right['tokens'])):
            changed[column] += a != b
            if column not in (YAW, VALID) and a != b:
                raise ValueError(f'HARD_STOP_V3_NON_HEADING_COLUMN_{column + 1}')
        normalized[right['line_index']] = hp._replace_tokens(right, {YAW: left['tokens'][YAW], VALID: left['tokens'][VALID]})
    if b''.join(normalized) != before:
        raise ValueError('HARD_STOP_V3_NON_HEADING_WHITESPACE')
    return dict(passed=True, rows=len(old), changed_tokens_by_column=changed, allowed_columns_zero_based=[YAW, VALID], all_other_bytes_identical=True, before_sha256=digest(before), after_sha256=digest(after))

def frozen_cells(clean_bytes, fault_rows):
    """Recover realized scalar input perturbations, never estimator errors."""
    (clean, _) = hp._rows(clean_bytes)
    base = [r for r in clean if float(r['tokens'][VALID]) == 1]
    times = [float(r['tokens'][0]) for r in base]
    if len(times) < 2 or any((abs(b - a - 1.0) > 1e-06 for (a, b) in zip(times, times[1:]))):
        raise ValueError('HARD_STOP_V3_FROZEN_A1_CELL_SCHEDULE')
    if len(fault_rows) != len(base):
        raise ValueError('HARD_STOP_V3_FROZEN_A1_AUDIT_ROW_COUNT')
    result = []
    for (index, (reference, fault)) in enumerate(zip(base, fault_rows)):
        if abs(float(fault['time']) - times[index]) > 1e-06:
            raise ValueError('HARD_STOP_V3_FROZEN_A1_AUDIT_TIME')
        delta = (float(fault['yaw_deg']) - float(reference['tokens'][YAW]) + 180.0) % 360.0 - 180.0
        result.append(dict(start_s=times[index], end_s=times[index + 1] if index + 1 < len(times) else times[index] + 1.0, delta_deg=delta, valid=str(fault['valid']).lower() in ('1', 'true')))
    return result

def fault_intervals(components, type_id):
    intervals = []
    if type_id not in OUTAGE | {'D58', 'D60'}:
        return intervals
    for component in components:
        details = component.get('details', {})
        source = component.get('affected_source', '')
        if type_id == 'D39' or 'yaw' in source or (type_id == 'D58' and source == 'gnss_position'):
            values = details.get('intervals', [])
            if 'interval' in details:
                values = [details['interval']]
            for value in values:
                if isinstance(value, dict):
                    value = (value['start_s'], value['end_s'])
                pair = tuple(map(float, value))
                if pair not in intervals:
                    intervals.append(pair)
    if not intervals:
        raise ValueError('HARD_STOP_V3_MISSING_FROZEN_HEADING_INTERVAL_' + type_id)
    return sorted(intervals)

def lift_heading(template, clean_r5, clean_frozen, *, type_id='CLEAN', fault_rows=(), components=(), base_time=1772784000.0, anchor_time_s=None):
    """Overlay yaw/valid only on an immutable case table with exact raw keys.

    The cell lift preserves original seeded event realizations and half-open
    durations. It is not independently sampled 5 Hz noise. D57 never matches a
    neighbour or repairs a timestamp. Inherited case std exposure stays intact.
    """
    (original, width) = hp._rows(template)
    (raw_rows, _) = hp._rows(clean_r5)
    if width != 18:
        raise ValueError('HARD_STOP_V3_EXPECTED_GNSS18')
    raw_map = {hp.time_to_itow_ms(r['tokens'][0], gps_week=2408, base_time=base_time): r for r in raw_rows}
    cells = frozen_cells(clean_frozen, fault_rows) if type_id in PERTURBED | CELL_DROPOUT else []
    starts = [c['start_s'] for c in cells]
    intervals = fault_intervals(components, type_id)
    keep_keys = None
    if type_id in {'D08', 'D09', 'D10'}:
        if anchor_time_s is None:
            raise ValueError('HARD_STOP_V3_MISSING_DOWNSAMPLE_ANCHOR')
        rate = {'D08': 5.0, 'D09': 2.0, 'D10': 1.0}[type_id]
        valid_items = [(k, r) for (k, r) in raw_map.items() if float(r['tokens'][VALID]) == 1.0]
        times = np.array([float(r['tokens'][0]) for (k, r) in valid_items])
        keep_keys = {k for (k, r) in valid_items}
        if len(times) >= 2 and 1.0 / float(np.median(np.diff(times))) > rate + 1e-09:
            phase = float(anchor_time_s) % (1.0 / rate)
            slots = np.floor((times - phase) * rate + 1e-09).astype(np.int64)
            first = np.unique(slots, return_index=True)[1]
            keep_keys = {valid_items[i][0] for i in first}
    lines = template.splitlines(keepends=True)
    active = matched = changed_by_fault = 0
    for row in original:
        t = float(row['tokens'][0])
        try:
            key = hp.time_to_itow_ms(row['tokens'][0], gps_week=2408, base_time=base_time)
        except hp.HeadingProviderError:
            if type_id != 'D57':
                raise
            key = None
        raw = raw_map.get(key)
        valid = raw is not None and float(raw['tokens'][VALID]) == 1.0
        replacements = {VALID: b'1' if valid else b'0'}
        if valid:
            matched += 1
            replacements[YAW] = raw['tokens'][YAW]
        i = bisect.bisect_right(starts, t + 1e-07) - 1
        cell = cells[i] if i >= 0 and t < cells[i]['end_s'] - 1e-07 else None
        inside = any((a <= t < b for (a, b) in intervals))
        if valid and (type_id in OUTAGE and inside or (type_id in CELL_DROPOUT and cell and (not cell['valid'])) or (keep_keys is not None and key not in keep_keys)):
            valid = False
            replacements[VALID] = b'0'
            changed_by_fault += 1
        if valid and type_id in PERTURBED and cell and (type_id not in {'D58', 'D60'} or inside):
            delta = cell['delta_deg']
            if delta != 0.0:
                value = (float(raw['tokens'][YAW]) + delta) % 360.0
                token = f'{value:.6f}'.encode()
                replacements[YAW] = b'0.000000' if token == b'360.000000' else token
                changed_by_fault += 1
        active += valid
        lines[row['line_index']] = hp._replace_tokens(row, replacements)
    payload = b''.join(lines)
    gate = byte_gate(template, payload)
    if type_id == 'D57' and active:
        raise ValueError('HARD_STOP_V3_D57_NONZERO_VALID_HEADING')
    return (payload, {**gate, 'type_id': type_id, 'valid_heading_rows': active, 'raw_matched_valid_rows': matched, 'rows_affected_by_heading_fault': changed_by_fault, 'frozen_fault_intervals': intervals, 'fault_cells': cells, 'time_mapping': 'FROZEN_A1_HALF_OPEN_TIME_CELLS_NO_RNG_REDRAW', 'all_case_std_bytes_preserved': True, 'HV_regenerated': False})
