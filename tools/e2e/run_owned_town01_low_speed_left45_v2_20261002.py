"""HH_261002 - One fresh V2 pilot with an authenticated final-closing ACK reader; failed V1 data stays denied."""
from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
from pathlib import Path

from scripts.e2e import run_owned_town01_low_speed_left45_20261002 as baseline
from scripts.e2e import town01_low_speed_left45_task_v2_20261002 as new_task
from scripts.e2e import collect_town01_low_speed_left45_v2_20261002 as declared_worker
from scripts.e2e import audit_town01_left45_closing_control_20261002 as closing
from scripts.e2e import audit_carla_acknowledged_control as original_ack

FROZEN_V1_SHA = "87afc96b1da9e2ac71bf11e3b6a04583aaaea09c43a0a1e9d1af33f196c40eba"
IDENTITIES = {"town01_low_speed_left45_task_20261002": "town01_low_speed_left45_task_v2_20261002",
    "collect_town01_low_speed_left45_20261002": "collect_town01_low_speed_left45_v2_20261002"}
OLD_MODULE = "scripts.e2e.run_owned_town01_low_speed_left45_20261002"
NEW_MODULE = "scripts.e2e.run_owned_town01_low_speed_left45_v2_20261002"


def identity_ast():
    new_task.require(new_task.sha(Path(baseline.__file__)) == FROZEN_V1_SHA, "frozen LEFT45 V1 owner changed")
    tree = ast.parse(Path(baseline.__file__).read_bytes())
    original = ast.Module(body=[node for node in tree.body if not isinstance(node, ast.If)], type_ignores=[])
    new_task.require(len(tree.body) - len(original.body) == 1, "one original main guard required")
    changed = copy.deepcopy(original); counts = dict.fromkeys((*IDENTITIES, OLD_MODULE, "v1"), 0)
    for node in ast.walk(changed):
        if isinstance(node, ast.ImportFrom) and node.module == "scripts.e2e":
            for alias in node.names:
                if alias.name in IDENTITIES: counts[alias.name] += 1; alias.name = IDENTITIES[alias.name]
        if isinstance(node, ast.Constant) and type(node.value) is str and node.value in (OLD_MODULE, "v1"):
            old = node.value; counts[old] += 1; node.value = NEW_MODULE if old == OLD_MODULE else "v2"
    new_task.require(all(value == 1 for value in counts.values()), "exact four owner identities required")
    inverse = copy.deepcopy(changed)
    for node in ast.walk(inverse):
        if isinstance(node, ast.ImportFrom) and node.module == "scripts.e2e":
            for alias in node.names:
                if alias.name in IDENTITIES.values(): alias.name = next(old for old, new in IDENTITIES.items() if new == alias.name)
        if isinstance(node, ast.Constant) and node.value in (NEW_MODULE, "v2"): node.value = OLD_MODULE if node.value == NEW_MODULE else "v1"
    new_task.require(ast.dump(inverse) == ast.dump(original), "LEFT45 ownership/physical AST changed")
    return changed


exec(compile(ast.fix_missing_locations(identity_ast()), __file__, "exec"), globals())
require(worker_module is declared_worker, "actual V2 worker must be present in the static source closure")
_original_activated, _original_contract = activated, execution_contract


def execution_contract(run_id, case_id):
    return dict(_original_contract(run_id, case_id), closing_receipt_reader_schema=closing.SCHEMA,
        original_V1_failed_capture_not_requalified=True)


@contextmanager
def activated():
    require(lifecycle.independent_audit is original_ack, "unknown or nested original receipt reader binding")
    saved = lifecycle.independent_audit
    with _original_activated():
        try:
            lifecycle.independent_audit = closing.provider()
            yield
        finally:
            lifecycle.independent_audit = saved


if __name__ == "__main__": raise SystemExit(main())
