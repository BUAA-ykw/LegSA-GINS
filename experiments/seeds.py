"""Fixed random substreams and interruption placement."""

from __future__ import annotations
from typing import Sequence
import numpy as np
from experiments.specification import WINDOW_END_S, WINDOW_START_S

class AnchorPolicyError(ValueError):
    """Source-only anchor inputs are incomplete or non-finite."""

def stable_component_rngs(seed_value: int, component_names: Sequence[str]) -> dict[str, np.random.Generator]:
    """Derive substreams by sorted component identity, never call order.

    中文说明：组件名先排序，再由 SeedSequence.spawn 分配子流，避免新增一个
    handler 分支后改变其他组件的随机序列。
    """
    names = tuple(sorted(set(component_names)))
    if len(names) != len(component_names):
        raise AnchorPolicyError('component substream names must be unique')
    spawned = np.random.SeedSequence(int(seed_value)).spawn(len(names))
    return {name: np.random.Generator(np.random.PCG64(sequence)) for (name, sequence) in zip(names, spawned)}

def place_interval(anchor_time_s: float, duration_s: float, window: tuple[float, float]=(WINDOW_START_S, WINDOW_END_S)) -> tuple[float, float]:
    """Place a half-open interval, shifting before any unavoidable clipping."""
    (start_bound, end_bound) = map(float, window)
    duration = float(duration_s)
    if duration <= 0 or end_bound <= start_bound:
        raise AnchorPolicyError('invalid interval/window')
    if duration >= end_bound - start_bound:
        return (start_bound, end_bound)
    start = float(anchor_time_s) - duration / 2.0
    end = start + duration
    if start < start_bound:
        end += start_bound - start
        start = start_bound
    if end > end_bound:
        start -= end - end_bound
        end = end_bound
    return (start, end)

def place_repeated_d07(anchor_time_s: float) -> tuple[tuple[float, float], ...]:
    centers = np.asarray([anchor_time_s - 6.0, anchor_time_s, anchor_time_s + 6.0], dtype=float)
    starts = centers - 1.5
    ends = centers + 1.5
    if starts[0] < WINDOW_START_S:
        shift = WINDOW_START_S - starts[0]
        starts += shift
        ends += shift
    if ends[-1] > WINDOW_END_S:
        shift = ends[-1] - WINDOW_END_S
        starts -= shift
        ends -= shift
    return tuple(((float(start), float(end)) for (start, end) in zip(starts, ends)))
