"""HH_261002 - Prospective horizon-only RIGHT80; failed RIGHT70 stays immutable and no admission is implied."""
from __future__ import annotations

import ast
import copy
from pathlib import Path

from scripts.e2e import town01_low_speed_right70_task_v2_20261002 as previous
from scripts.e2e import prepare_town01_right70_capture_route_20261002 as container

PREVIOUS_SHA = "529955f75bdede135d99ce093dc8f00b675d60785277b031fa81740b9860f04f"
COUNTS = {1272: 1472, 597: 697, 63_600_000_000: 73_600_000_000, 1555: 1755, 778: 878}


def identity_ast():
    previous.require(previous.sha(previous.__file__) == PREVIOUS_SHA, "frozen RIGHT70 V2 task changed")
    old = previous.identity_ast(); new = copy.deepcopy(old); changes = []; counts = dict.fromkeys(COUNTS, 0)
    for node in ast.walk(new):
        if isinstance(node, ast.Constant) and type(node.value) is int and node.value in COUNTS:
            before = node.value; counts[before] += 1; changes.append((node, before)); node.value = COUNTS[before]
        elif isinstance(node, ast.Constant) and type(node.value) is str:
            before = node.value
            if before == str(previous.ROUTE.relative_to(previous.ROOT)): node.value = before
            elif before == previous.SCHEMA: node.value = "carla.actual_town01_right80_teacher.20261002.v1"
            elif before == "v2": node.value = "v1"
            else:
                node.value = before.replace("RIGHT70", "RIGHT80").replace("right70", "right80")
                node.value = node.value.replace("teacher_v2", "teacher_v1").replace("ALL597", "ALL697").replace("80..1272", "80..1472").replace("4.0..63.6", "4.0..73.6")
            if node.value != before: changes.append((node, before))
    previous.require(all(counts.values()), "all exact RIGHT80 membership/census substitutions required")
    inverse = copy.deepcopy(new); nodes = list(ast.walk(new)); inverse_nodes = list(ast.walk(inverse))
    for node, before in changes: inverse_nodes[nodes.index(node)].value = before
    previous.require(ast.dump(inverse) == ast.dump(old), "RIGHT80 changed physical/history/selection predicates")
    return new


exec(compile(ast.fix_missing_locations(identity_ast()), __file__, "exec"), globals())
_original_fixed_contract = fixed_contract
_base_verify_parent = verify_parent
_right_fixed = copy.deepcopy(previous._right_fixed)
_old_right_fixed = copy.deepcopy(_right_fixed)
_fixed_changes = []
for _node in ast.walk(_right_fixed):
    if isinstance(_node, ast.Constant) and type(_node.value) is float and _node.value == 70.:
        _fixed_changes.append((_node, _node.value)); _node.value = 80.
    elif isinstance(_node, ast.Constant) and type(_node.value) is int and _node.value == 1400:
        _fixed_changes.append((_node, _node.value)); _node.value = 1600
require(len(_fixed_changes) == 2, "only exact driving seconds and interval count may change")
_inverse = copy.deepcopy(_right_fixed); _nodes = list(ast.walk(_right_fixed)); _inverse_nodes = list(ast.walk(_inverse))
for _node, _before in _fixed_changes: _inverse_nodes[_nodes.index(_node)].value = _before
require(ast.dump(_inverse) == ast.dump(_old_right_fixed), "RIGHT80 changed original physical fixed contract")
exec(compile(ast.fix_missing_locations(ast.Module(body=[_right_fixed], type_ignores=[])), __file__, "exec"), globals())
_right_fixed_contract = fixed_contract


def verify_parent():
    _base_verify_parent(); previous.verify_parent()
    require(sha(previous.__file__) == PREVIOUS_SHA, "frozen RIGHT70 V2 task changed")


def fixed_contract(run_id, case_id):
    return dict(_right_fixed_contract(run_id, case_id),
        closing_receipt_reader_schema="carla.actual_town01_right80_closing_control.20261002.v1",
        original_LEFT_and_failed_RIGHT_candidates_not_requalified=True)
