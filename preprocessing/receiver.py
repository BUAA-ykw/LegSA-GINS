"""Time-aligned receiver position and velocity observations."""

from __future__ import annotations
import csv
import struct
from pathlib import Path
from preprocessing.ubx import iter_ubx_frames, parse_bytes_cell
from preprocessing.receiver_binary import decode_nav_hpposecef

def csv_rows(path):
    with Path(path).open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))

def stamp(row, prefix):
    return int(row[prefix + 'secs']) + int(row[prefix + 'nsecs']) * 1e-09

def decode_receiver(path):
    (hp, pvt) = ({}, {})
    for (line, row) in enumerate(csv_rows(path), 2):
        name = row.get('name')
        if name not in ('UBX-NAV-HPPOSECEF', 'UBX-NAV-PVT'):
            continue
        frames = list(iter_ubx_frames(parse_bytes_cell(row['data'])))
        if len(frames) != 1:
            raise ValueError(f'Invalid UBX record {line}')
        frame = frames[0]
        payload = frame[6:-2]
        if name == 'UBX-NAV-HPPOSECEF':
            decoded = decode_nav_hpposecef(payload)
            key = decoded.itow_ms
            target = hp
            entry = {'ecef_m': decoded.position_ecef_m.tolist(), 'pAcc_m': decoded.position_accuracy_m}
        else:
            if frame[2:4] != bytes((1, 7)) or len(payload) != 92:
                raise ValueError('PVT class/length mismatch')
            key = struct.unpack_from('<I', payload)[0]
            target = pvt
            entry = {'velocity_mps': [v * 0.001 for v in struct.unpack_from('<iii', payload, 48)], 'hAcc_m': struct.unpack_from('<I', payload, 40)[0] * 0.001, 'vAcc_m': struct.unpack_from('<I', payload, 44)[0] * 0.001, 'receipt_stamp': stamp(row, 'stamp.')}
        if key in target:
            raise ValueError('Duplicate UBX epoch')
        target[key] = {**entry, 'itow_ms': key, 'csv_line': line}
    if not hp or set(hp) != set(pvt):
        raise ValueError('HP/PVT exact epoch coverage mismatch')
    return (hp, pvt)
