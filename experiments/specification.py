"""Types and evaluation window for the paper disturbance experiments."""

from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Mapping
CLEAN_CASE_ID = 'C00_clean_normal'
WINDOW_START_S = 66.0
WINDOW_END_S = 340.0

@dataclass(frozen=True)
class DegradationType:
    type_id: str
    name: str
    family: str
    parameters: Mapping[str, Any]
    affected_sources: tuple[str, ...]
    claim_level: str = 'controlled_degradation_descriptive'
    effect_validation_rule_id: str = ''

    @property
    def requires_randomness(self) -> bool:
        return self.type_id not in DETERMINISTIC_TYPES
DETERMINISTIC_TYPES = frozenset({'D01', 'D02', 'D03', 'D04', 'D05', 'D06', 'D07', 'D08', 'D09', 'D10', 'D23', 'D24', 'D25', 'D26', 'D28', 'D29', 'D30', 'D31', 'D36', 'D37', 'D42', 'D46'})

def load_type_registry():
    import json
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / 'configs/experiments/disturbances.json'
    return tuple(DegradationType(**row) for row in json.loads(path.read_text()))
