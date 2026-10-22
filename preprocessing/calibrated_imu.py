"""Current-sample IMU increment preparation with the fixed acceleration scale."""

from __future__ import annotations
import math
import numpy as np
from preprocessing.imu import euler_rpy_deg_to_matrix, _matvec

def _token(value, precision):
    return format(float(format(float(value), '.12g')), '.' + str(precision) + 'f')

def scale_current_sample_payload(*, baseline_rows, baseline_text, frames, scale):
    """Use unrounded installed FRD current force, preserving original gyro/time tokens."""
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('Invalid frozen scale')
    fields = ('dtheta_x', 'dtheta_y', 'dtheta_z', 'dvel_x', 'dvel_y', 'dvel_z')
    reconstructed = ''.join((_token(r['time'], 6) + ' ' + ' '.join((_token(r[k], 8) for k in fields)) + '\n' for r in baseline_rows))
    if reconstructed != baseline_text:
        raise ValueError('Frozen current-sample IMU byte reconstruction failed')
    indices = [i for i in range(1, len(frames)) if 0 < frames[i]['timestamp'] - frames[i - 1]['timestamp'] <= 0.1]
    tokens = [line.split() for line in baseline_text.splitlines()]
    if len(indices) != len(tokens) or len(indices) != len(baseline_rows):
        raise ValueError('Raw/frozen IMU interval mapping mismatch')
    rotation = euler_rpy_deg_to_matrix(-1, 0, 0)
    output = []
    for (row, original, i) in zip(baseline_rows, tokens, indices):
        dt = frames[i]['timestamp'] - frames[i - 1]['timestamp']
        if dt != row['dt']:
            raise ValueError('Frozen measured-dt identity mismatch')
        a = frames[i]['accelerometer']
        force = np.asarray(_matvec(rotation, [a[0], -a[1], -a[2]]), float)
        dvel = force * scale * dt
        output.append(' '.join(original[:4] + [_token(v, 8) for v in dvel]) + '\n')
    text = ''.join(output)
    if any((a.split()[:4] != b.split()[:4] for (a, b) in zip(text.splitlines(), baseline_text.splitlines()))):
        raise ValueError('Scale changed time/gyro tokens')
    return (text, {'frozen_unscaled_imu_byte_identity': True, 'timestamp_and_gyro_tokens_unchanged': True, 'row_count': len(output), 'raw_frame_count': len(frames), 'skipped_interval_count': len(frames) - 1 - len(indices), 's': scale, 'scale_fitted_here': False, 'external_preprocessing_called': False, 'integration_convention': 'frozen current sample, measured dt, FLU->FRD, Rx(-1 degree)', 'first_1000_gyro_bias': 'unchanged frozen builder input-side calibration', 'accel_scale_applied_before_dt': True, 'previous_sample_ZOH_used': False})
