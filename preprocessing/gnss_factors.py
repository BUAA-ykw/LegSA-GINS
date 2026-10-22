"""Corrected pseudorange and Doppler inputs for GNSS factor graphs."""

from __future__ import annotations
import collections
import ctypes
import numpy as np
from preprocessing import raw_gnss as raw
C = 299792458.0
OMEGA = 7.2921151467e-05

class Corrections:

    def __init__(self, path):
        self.lib = ctypes.CDLL(str(path))
        ptr = ctypes.POINTER(ctypes.c_double)
        self.fun = self.lib.fgo_corrections
        self.fun.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_int, ptr, ptr, ctypes.c_double, ptr]
        self.fun.restype = ctypes.c_int

    def compute(self, provider, epoch, identity, point, azel):
        p = (ctypes.c_double * 3)(*point)
        a = (ctypes.c_double * 2)(*azel)
        out = (ctypes.c_double * 6)()
        ok = self.fun(provider._handle, epoch.gps_week, epoch.gps_tow_seconds, provider.satellite_number(identity), p, a, raw.signal_frequency_hz(identity), out)
        if not ok:
            raise raw.RawBackendError('Atmosphere/TGD unavailable')
        return np.asarray(out)

def code_sigma(elevation, cno, parameters):
    """goGPS cofactor q; q is a variance, not an information weight.

    Wen2021 Eq15 labels q W then inverts it in Eq16. The author's executable
    convention (GNSS_Tools::cofactorMatrixCal_WLS then sqrt(1/weight)) resolves
    this as sigma=sqrt(q), so lower elevation/CNO means less information.
    """
    (T, A, a, F) = (float(parameters[k]) for k in ('T', 'A', 'a', 'F'))
    q = 1 / np.sin(elevation) ** 2 * 10 ** (-(np.asarray(cno) - T) / a) * ((A / 10 ** (-(F - T) / a) - 1) * (np.asarray(cno) - T) / (F - T) + 1)
    if np.any(~np.isfinite(q)) or np.any(q <= 0):
        raise ValueError('Invalid code cofactor')
    return np.sqrt(q)

def solve_code_wls(satellites, codes, sigma, systems, initial):
    """Independent code-only WLS, with only observed constellation clocks."""
    (sat, codes, sigma) = map(np.asarray, (satellites, codes, sigma))
    used = np.unique(systems)
    if len(codes) < 3 + len(used):
        return None
    system_design = np.column_stack([np.asarray(systems) == s for s in used]).astype(float)
    point = np.asarray(initial, float).copy()
    clocks = np.zeros(len(used))
    for it in range(50):
        d = point - sat
        r = np.linalg.norm(d, axis=1)
        J = np.column_stack((d / r[:, None], system_design)) / sigma[:, None]
        residual = (codes - r - system_design @ clocks) / sigma
        (step, _, rank, _) = np.linalg.lstsq(J, residual, rcond=None)
        if rank != J.shape[1]:
            return None
        point += step[:3]
        clocks += step[3:]
        if not np.isfinite(point).all() or np.linalg.norm(point) > 100000000.0:
            return None
        if np.max(np.abs(step)) < 0.0001:
            all_clocks = np.full(2, np.nan)
            all_clocks[used] = clocks
            return (point, all_clocks, it + 1)
    return None

def doppler_wls(satellites, velocities, clock_rates, frequencies, doppler, sigma_range_rate, point):
    """RAWX D is negative range rate: rho_dot = -lambda*D.

    Use broadcast satellite velocity/clock drift and the derivative of the
    first-order Sagnac term (GNC Eq9), with unrotated transmit positions here.
    Position factors independently use a rotated satellite, exactly once.
    """
    (sv, vv) = (np.asarray(satellites), np.asarray(velocities))
    los = sv - np.asarray(point)
    los /= np.linalg.norm(los, axis=1)[:, None]
    H = np.column_stack((-los, np.ones(len(sv))))
    H[:, 0] += OMEGA / C * sv[:, 1]
    H[:, 1] -= OMEGA / C * sv[:, 0]
    pred0 = np.sum(los * vv, axis=1) + OMEGA / C * (vv[:, 1] * point[0] - vv[:, 0] * point[1])
    observed = -C / np.asarray(frequencies) * np.asarray(doppler)
    rhs = observed - pred0 + C * np.asarray(clock_rates)
    Hwhite = H / np.asarray(sigma_range_rate)[:, None]
    if len(sv) < 4 or np.linalg.matrix_rank(Hwhite) < 4:
        return None
    (x, *_) = np.linalg.lstsq(Hwhite, rhs / np.asarray(sigma_range_rate), rcond=None)
    cov = np.linalg.inv(Hwhite.T @ Hwhite)
    return (x[:3], cov[:3, :3], x[3], rhs - H @ x)

def selected_measurements(epoch, config, counts=None):
    counts = counts if counts is not None else collections.Counter()
    best = {}
    for m in epoch.measurements:
        counts['raw_signal_count'] += 1
        ident = m.identity
        allowed = (ident.gnss_id, ident.sig_id, ident.freq_id) in ((0, 0, 0), (3, 0, 0), (3, 1, 0))
        if not allowed:
            counts['unsupported_signal'] += 1
            continue
        if not m.pseudorange_valid or not 1000000.0 < m.pr_mes_m < 100000000.0:
            counts['invalid_code'] += 1
            continue
        if m.cno_dbhz < config['minimum_cno_dbhz']:
            counts['below_cno'] += 1
            continue
        key = (ident.gnss_id, ident.sv_id)
        if key in best:
            counts['duplicate_signal'] += 1
        if key not in best or (m.cno_dbhz, -ident.sig_id) > (best[key].cno_dbhz, -best[key].identity.sig_id):
            best[key] = m
    counts['selected_signal_count'] += len(best)
    return [best[k] for k in sorted(best)]
