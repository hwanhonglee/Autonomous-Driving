"""HH_261002 - Fresh V2 identity only; preserve the failed V1 pilot and every physical/membership statement."""
from __future__ import annotations

import ast
import copy
from pathlib import Path

from scripts.e2e import town01_low_speed_left45_task_20261002 as baseline

FROZEN_V1_SHA = "9406fc63f77b61225eb2aa16ccaeeb5219a4b29da4582e82dfb4aefc769869b2"
IDENTITIES = {
    "carla.actual_town01_left45_teacher.20261002.v1": "carla.actual_town01_left45_teacher.20261002.v2",
    "v1": "v2",
    "storage/experiments/validation/2026-10-02/town01_low_speed_left45_teacher_v1":
        "storage/experiments/validation/2026-10-02/town01_low_speed_left45_teacher_v2",
}


def identity_ast():
    baseline.require(baseline.sha(Path(baseline.__file__)) == FROZEN_V1_SHA, "frozen LEFT45 V1 task changed")
    original = ast.parse(Path(baseline.__file__).read_bytes()); changed = copy.deepcopy(original)
    counts = dict.fromkeys(IDENTITIES, 0)
    for node in ast.walk(changed):
        if isinstance(node, ast.Constant) and type(node.value) is str and node.value in IDENTITIES:
            counts[node.value] += 1; node.value = IDENTITIES[node.value]
    baseline.require(all(value == 1 for value in counts.values()), "exact three task identities required")
    inverse = copy.deepcopy(changed)
    for node in ast.walk(inverse):
        if isinstance(node, ast.Constant) and type(node.value) is str and node.value in IDENTITIES.values():
            node.value = next(old for old, new in IDENTITIES.items() if new == node.value)
    baseline.require(ast.dump(inverse) == ast.dump(original), "LEFT45 task physical AST changed")
    return changed


exec(compile(ast.fix_missing_locations(identity_ast()), __file__, "exec"), globals())
