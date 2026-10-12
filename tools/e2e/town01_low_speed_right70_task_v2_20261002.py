"""HH_261002 - Fresh RIGHT70 task identity and two truthful owner fields; the original strict parser and all physical membership math stay exact."""
from __future__ import annotations

import ast
import copy
from pathlib import Path

from scripts.e2e import town01_low_speed_right70_task_20261002 as previous
from scripts.e2e import prepare_town01_right70_capture_route_20261002 as container

PREVIOUS_SHA="49704b8634e887c7d760a0c7b57ddf9726f8d9d8eef822a7493c475ce831f3ea"
IDENTITIES={previous.SCHEMA:"carla.actual_town01_right70_teacher.20261002.v2",
    "storage/experiments/validation/2026-10-02/town01_low_speed_right70_teacher_v1":
        "storage/experiments/validation/2026-10-02/town01_low_speed_right70_teacher_v2",
    "v1":"v2"}


def identity_ast():
    previous.require(previous.sha(previous.__file__)==PREVIOUS_SHA,"frozen RIGHT70 V1 task changed")
    original=previous.definition();changed=copy.deepcopy(original);counts=dict.fromkeys(IDENTITIES,0)
    for node in ast.walk(changed):
        if isinstance(node,ast.Constant) and type(node.value) is str and node.value in IDENTITIES:
            counts[node.value]+=1;node.value=IDENTITIES[node.value]
    previous.require(all(n==1 for n in counts.values()),"exact schema/path/run identities required")
    inverse=copy.deepcopy(changed)
    for node in ast.walk(inverse):
        if isinstance(node,ast.Constant) and type(node.value) is str and node.value in IDENTITIES.values():
            node.value=next(old for old,new in IDENTITIES.items() if new==node.value)
    previous.require(ast.dump(inverse)==ast.dump(original),"RIGHT70 V2 changed physical or configuration predicates")
    return changed


exec(compile(ast.fix_missing_locations(identity_ast()),__file__,"exec"),globals())
_original_fixed_contract=fixed_contract
_base_verify_parent=verify_parent
_right_fixed=next(n for n in ast.parse(Path(previous.__file__).read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name=="fixed_contract")
exec(compile(ast.fix_missing_locations(ast.Module(body=[copy.deepcopy(_right_fixed)],type_ignores=[])),__file__,"exec"),globals())
_right_fixed_contract=fixed_contract


def verify_parent():
    _base_verify_parent();previous.verify_parent()
    require(sha(previous.__file__)==PREVIOUS_SHA,"frozen RIGHT70 V1 task changed")


def fixed_contract(run_id,case_id):
    # HH_261002 - These exact values are already declared by the owner. Adding them to fixed makes the unchanged strict parser authenticate them.
    return dict(_right_fixed_contract(run_id,case_id),
        closing_receipt_reader_schema="carla.actual_town01_right70_closing_control.20261002.v1",
        original_LEFT_and_failed_RIGHT_candidates_not_requalified=True)
