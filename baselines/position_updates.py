"""Position update scheduling for the invariant EKF."""

from __future__ import annotations
from typing import Any
import numpy as np
from baselines.iekf import PavlasekIEKF, rotation_to_rpy_ned_frd_deg
BASELINE_BODY_FRD_M = np.array([0.0, -0.350, 0.0])

def _h02_gnss_update(filter_: PavlasekIEKF, arrays, index, *, state_time, base_time, two_receiver, method_id, innovation_rows, nis_rows, measurement_update=None):
    """Unchanged PHASE5 covariance/innovation arithmetic for one GNSS event."""
    if not bool(arrays['valid1'][index]) or (two_receiver and (not bool(arrays['valid2'][index]))):
        return False
    pacc1 = float(arrays['pacc1'][index])
    R1 = np.eye(3) * pacc1 * pacc1
    kwargs: dict[str, Any] = {}
    if two_receiver:
        pacc2 = float(arrays['pacc2'][index])
        kwargs = {'receiver2_position_ned_m': arrays['p2'][index], 'receiver2_covariance_ned_m2': np.eye(3) * pacc2 * pacc2}
    update = filter_.update if measurement_update is None else measurement_update
    diagnostics = update(arrays['p1'][index], R1, **kwargs)
    degrees_of_freedom = len(diagnostics.innovation)
    (roll, pitch, yaw) = rotation_to_rpy_ned_frd_deg(filter_.pose.C_nb)
    innovation = diagnostics.innovation.tolist() + [''] * (6 - len(diagnostics.innovation))
    relative = diagnostics.relative_residual_ned_m.tolist() if diagnostics.relative_residual_ned_m is not None else ['', '', '']
    estimated = (filter_.pose.C_nb @ BASELINE_BODY_FRD_M).tolist() if two_receiver else ['', '', '']
    measured = (arrays['p2'][index] - arrays['p1'][index]).tolist() if two_receiver else ['', '', '']
    innovation_rows.append({'method_id': method_id, 'provider_epoch_index': index, 'absolute_time_unix_seconds': state_time, 'time_seconds': state_time - base_time, **{f'innovation_{axis}': innovation[axis] for axis in range(6)}, 'receiver1_residual_n': diagnostics.receiver1_residual_ned_m[0], 'receiver1_residual_e': diagnostics.receiver1_residual_ned_m[1], 'receiver1_residual_d': diagnostics.receiver1_residual_ned_m[2], 'relative_residual_n': relative[0], 'relative_residual_e': relative[1], 'relative_residual_d': relative[2], 'measured_baseline_n': measured[0], 'measured_baseline_e': measured[1], 'measured_baseline_d': measured[2], 'estimated_baseline_n': estimated[0], 'estimated_baseline_e': estimated[1], 'estimated_baseline_d': estimated[2], 'roll_deg': roll, 'pitch_deg': pitch, 'yaw_ned_deg': yaw, 'nis': diagnostics.nis, 'degrees_of_freedom': degrees_of_freedom})
    nis_rows.append({'method_id': method_id, 'provider_epoch_index': index, 'absolute_time_unix_seconds': state_time, 'time_seconds': state_time - base_time, 'nis': diagnostics.nis, 'degrees_of_freedom': degrees_of_freedom, 'normalized_nis': diagnostics.nis / degrees_of_freedom})
    return True
