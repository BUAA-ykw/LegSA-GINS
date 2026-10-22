"""UBX frame and payload decoding."""

from __future__ import annotations
import ast
import re
from typing import Iterable

def parse_bytes_cell(value: str) -> bytes:
    text = (value or '').strip()
    if not text:
        return b''
    if text.startswith("b'") or text.startswith('b"'):
        parsed = ast.literal_eval(text)
        if isinstance(parsed, bytes):
            return parsed
    if text.startswith('[') and text.endswith(']'):
        parsed = ast.literal_eval(text)
        if isinstance(parsed, (list, tuple)):
            return bytes((int(x) & 255 for x in parsed))
    matches = re.findall('0x[0-9a-fA-F]+|[0-9a-fA-F]{2}', text)
    if matches:
        return bytes((int(item, 16) for item in matches))
    return b''

def ubx_checksum(payload: bytes) -> tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for byte in payload:
        ck_a = ck_a + byte & 255
        ck_b = ck_b + ck_a & 255
    return (ck_a, ck_b)

def is_valid_ubx_frame(frame: bytes) -> bool:
    if len(frame) < 8 or frame[:2] != b'\xb5b':
        return False
    length = int.from_bytes(frame[4:6], 'little')
    if len(frame) != length + 8:
        return False
    return ubx_checksum(frame[2:-2]) == (frame[-2], frame[-1])

def iter_ubx_frames(blob: bytes) -> Iterable[bytes]:
    index = 0
    while index + 8 <= len(blob):
        start = blob.find(b'\xb5b', index)
        if start < 0 or start + 8 > len(blob):
            return
        length = int.from_bytes(blob[start + 4:start + 6], 'little')
        end = start + length + 8
        if end > len(blob):
            return
        frame = blob[start:end]
        if is_valid_ubx_frame(frame):
            yield frame
        index = max(end, start + 2)
