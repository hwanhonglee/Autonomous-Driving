"""HH_261002 - Fresh V2 teacher namespace only; actual control, pixels, phases and all original hazards stay unchanged."""
from __future__ import annotations

import ast
import copy
from pathlib import Path

from scripts.e2e import collect_town01_low_speed_left45_20261002 as baseline
from scripts.e2e import town01_low_speed_left45_task_v2_20261002 as new_task

FROZEN_V1_SHA = "e87b64c9f1df8348561c5a5ee3625e8bc6a90fde22e490b88dc45647dae36cb3"


def identity_ast():
    new_task.require(new_task.sha(Path(baseline.__file__)) == FROZEN_V1_SHA, "frozen LEFT45 V1 collector changed")
    original = ast.parse(Path(baseline.__file__).read_bytes()); changed = copy.deepcopy(original)
    imports = times = 0
    for node in ast.walk(changed):
        if isinstance(node, ast.ImportFrom) and node.module == "scripts.e2e":
            for alias in node.names:
                if alias.name == "town01_low_speed_left45_task_20261002":
                    alias.name = "town01_low_speed_left45_task_v2_20261002"; imports += 1
        if isinstance(node, ast.Constant) and node.value == "v1": node.value = "v2"; times += 1
    new_task.require((imports, times) == (1, 2), "exact collector provider and two protocol identities required")
    inverse = copy.deepcopy(changed)
    for node in ast.walk(inverse):
        if isinstance(node, ast.ImportFrom) and node.module == "scripts.e2e":
            for alias in node.names:
                if alias.name == "town01_low_speed_left45_task_v2_20261002": alias.name = "town01_low_speed_left45_task_20261002"
        if isinstance(node, ast.Constant) and node.value == "v2": node.value = "v1"
    new_task.require(ast.dump(inverse) == ast.dump(original), "LEFT45 collector physical AST changed")
    return changed


exec(compile(ast.fix_missing_locations(identity_ast()), __file__, "exec"), globals())
