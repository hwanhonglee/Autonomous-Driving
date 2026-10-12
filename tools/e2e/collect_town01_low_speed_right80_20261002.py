"""HH_261002 - RIGHT80 task identity only; original manual control, hazards, Green fixture and coast remain exact."""
from __future__ import annotations

import ast
import copy

from scripts.e2e import collect_town01_low_speed_right70_v2_20261002 as previous
from scripts.e2e import town01_low_speed_right80_task_20261002 as declared_task

PREVIOUS_SHA = "af27e39776ccee1f5431a9c2307da58c8b03c9445ac9a9c7a2dc4bf9dff9d835"


def identity_ast():
    declared_task.require(declared_task.sha(previous.__file__) == PREVIOUS_SHA, "frozen RIGHT70 V2 worker changed")
    old = previous.identity_ast(); new = copy.deepcopy(old); changes = []; imports = versions = 0
    for node in ast.walk(new):
        if isinstance(node, ast.ImportFrom):
            for alias in node.names:
                if alias.name == "town01_low_speed_right70_task_v2_20261002":
                    changes.append((alias, "name", alias.name)); alias.name = "town01_low_speed_right80_task_20261002"; imports += 1
        if isinstance(node, ast.Constant) and type(node.value) is str:
            before = node.value
            if before == "v2": node.value = "v1"; versions += 1
            else: node.value = before.replace("RIGHT70", "RIGHT80").replace("right70", "right80")
            if node.value != before: changes.append((node, "value", before))
    declared_task.require(imports == 1 and versions == 2, "one task and two capture protocol identities required")
    inverse = copy.deepcopy(new); nodes = list(ast.walk(new)); inverse_nodes = list(ast.walk(inverse))
    for node, key, before in changes: setattr(inverse_nodes[nodes.index(node)], key, before)
    declared_task.require(ast.dump(inverse) == ast.dump(old), "RIGHT80 changed teacher/fixture/transport/cleanup physics")
    return new


exec(compile(ast.fix_missing_locations(identity_ast()), __file__, "exec"), globals())
