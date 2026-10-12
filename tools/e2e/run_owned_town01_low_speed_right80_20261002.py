"""HH_261002 - One fresh horizon-only RIGHT80 owner; unchanged global turn/ownership/physical gates, never retrospective RIGHT70 approval."""
from __future__ import annotations

import ast
import copy

from scripts.e2e import run_owned_town01_low_speed_right70_v2_20261002 as previous
from scripts.e2e import town01_low_speed_right80_task_20261002 as declared_task
from scripts.e2e import collect_town01_low_speed_right80_20261002 as declared_worker
from scripts.e2e import audit_town01_right80_closing_control_20261002 as declared_closing

V2_SOURCE_SHA = "01f919763a05eab2f0b6b7df002f70e933859eac7c0d7687274673d94c600a3d"
NEW_MODULE = "scripts.e2e.run_owned_town01_low_speed_right80_20261002"
NEW_CLOSING_SHA = "f724029fb1ce44147d0586d474ba209ebfc5a199048fb1f1d560c45a9596ebb9"
NAMES = {"town01_low_speed_right70_task_v2_20261002": "town01_low_speed_right80_task_20261002",
    "collect_town01_low_speed_right70_v2_20261002": "collect_town01_low_speed_right80_20261002",
    "audit_town01_right70_closing_control_20261002": "audit_town01_right80_closing_control_20261002"}
COUNTS = {1400: 1600, 1401: 1601, 1424: 1624}


def identity_ast():
    declared_task.require(declared_task.sha(previous.__file__) == V2_SOURCE_SHA, "frozen RIGHT70 V2 owner changed")
    old = previous.revision_ast(); new = copy.deepcopy(old); changes = []; imports = 0; counts = dict.fromkeys(COUNTS, 0)
    for node in ast.walk(new):
        if isinstance(node, ast.ImportFrom):
            for alias in node.names:
                if alias.name in NAMES:
                    changes.append((alias, "name", alias.name)); alias.name = NAMES[alias.name]; imports += 1
        if isinstance(node, ast.Constant) and type(node.value) is int and node.value in COUNTS:
            before = node.value; counts[before] += 1; changes.append((node, "value", before)); node.value = COUNTS[before]
        elif isinstance(node, ast.Constant) and type(node.value) is str:
            before = node.value
            if before in NAMES: node.value = NAMES[before]
            elif before == previous.CLOSING_SHA: node.value = NEW_CLOSING_SHA
            elif before == "v2": node.value = "v1"
            else: node.value = before.replace("RIGHT70", "RIGHT80").replace("right70", "right80").replace("plus1400", "plus1600")
            if node.value != before: changes.append((node, "value", before))
    declared_task.require(imports == 3 and all(counts.values()), "exact task/worker/closing identities and driving census required")
    inverse = copy.deepcopy(new); nodes = list(ast.walk(new)); inverse_nodes = list(ast.walk(inverse))
    for node, key, before in changes: setattr(inverse_nodes[nodes.index(node)], key, before)
    declared_task.require(ast.dump(inverse) == ast.dump(old), "RIGHT80 changed turn/physics/ownership/source predicates")
    return new


revision_ast = identity_ast
exec(compile(ast.fix_missing_locations(revision_ast()), __file__, "exec"), globals())
require(task is declared_task and worker_module is declared_worker and closing is declared_closing,
    "actual RIGHT80 task/worker/strict closing reader must be source-bound")
MODULE = NEW_MODULE
_originals["MODULE"] = MODULE
require(task.fixed_contract("v1", "pilot_00")["closing_receipt_reader_schema"] == closing.SCHEMA,
    "all owner fields must be authenticated by the unchanged strict capture parser")


if __name__ == "__main__": raise SystemExit(main())
