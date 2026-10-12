"""HH_261002 - Prospective RIGHT70 fixed membership; original LEFT/recovery mathematics and failed candidates remain immutable."""
from __future__ import annotations

import ast
import copy
import hashlib
from pathlib import Path

from scripts.e2e import town01_low_speed_left45_task_20261002 as previous
from scripts.e2e import prepare_town01_right70_capture_route_20261002 as container

PREVIOUS_SHA="9406fc63f77b61225eb2aa16ccaeeb5219a4b29da4582e82dfb4aefc769869b2"
CONTAINER_SHA="bb8fff98e029692efd8ef44156ebef90d831fc6bf21691f8d71b2cc8c473ef96"
ROUTE_SHA_EXPECTED="0561dc59c89a37aa318cd8548c4b8912ed3eb25680522006244b77e27edaf9f5"
_STRINGS={previous.SCHEMA:"carla.actual_town01_right70_teacher.20261002.v1",
    str(previous.ROUTE.relative_to(previous.ROOT)):str(container.OUTPUT.relative_to(previous.ROOT)),
    previous.ROUTE_SHA:ROUTE_SHA_EXPECTED,
    "left":"right"}
_COUNTS={772:1272,347:597,38_600_000_000:63_600_000_000,1055:1555,528:778,2026102500:2026102600}


def definition():
    raw=Path(previous.__file__).read_bytes()
    previous.require(hashlib.sha256(raw).hexdigest()==PREVIOUS_SHA,"frozen LEFT membership source changed")
    previous.require(hashlib.sha256(Path(container.__file__).read_bytes()).hexdigest()==CONTAINER_SHA,"capture container source changed")
    tree=ast.parse(raw); old=copy.deepcopy(tree); changes=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.Constant) and type(node.value) is str:
            before=node.value
            node.value=_STRINGS.get(before,before).replace("LEFT45","RIGHT70").replace("LEFT","RIGHT").replace("left45","right70")
            node.value=node.value.replace("ALL347","ALL597").replace("80..772","80..1272").replace("4.0..38.6","4.0..63.6")
            if node.value!=before:changes.append((node,before))
        elif isinstance(node,ast.Constant) and type(node.value) is int and node.value in _COUNTS:
            before=node.value;node.value=_COUNTS[before];changes.append((node,before))
        elif isinstance(node,ast.keyword) and node.arg=="max_duration_sec":
            previous.require(isinstance(node.value,ast.Constant) and node.value.value==60.,"exact original duration literal required")
            changes.append((node.value,node.value.value));node.value.value=90.
    inverse=copy.deepcopy(tree)
    # HH_261002 - Position-indexed inverse proves that only the enumerated identities/membership/duration literals differ.
    new_nodes=list(ast.walk(tree)); inverse_nodes=list(ast.walk(inverse))
    for node,before in changes:inverse_nodes[new_nodes.index(node)].value=before
    previous.require(ast.dump(inverse)==ast.dump(old),"RIGHT70 changed original membership/control mathematics")
    return tree


exec(compile(ast.fix_missing_locations(definition()),__file__,"exec"),globals())
_original_fixed_contract=fixed_contract
_original_verify_parent=verify_parent


def verify_parent():
    _original_verify_parent()
    require(sha(Path(previous.__file__))==PREVIOUS_SHA and sha(Path(container.__file__))==CONTAINER_SHA,
        "original membership/container providers changed")


def fixed_contract(run_id,case_id):
    result=_original_fixed_contract(run_id,case_id)
    result.update(perturbation="No additional lateral/yaw perturbation; NEW pinned XODR centreline rear spawn/goal declarations, not observed poses or original failed route rewrite.",
        actual_turn_minimum_signed_yaw_deg=-60.,planned_turn_yaw_tolerance_deg=15.,
        actual_turn_exit_plus10_stable_seconds=3.,full_generated_navigation_route_sha256=container.SOURCE_SHA,
        whole_nominal_summary_sha256=container.SUMMARY_SHA,capture_container_sha256=ROUTE_SHA,
        capture_transforms_are_prospective_rear_reference_not_measured=True,
        unchanged_collector_rear_to_actor_half_wheelbase_shift_required=True,
        original32_and_generated_p00_failure_not_reclassified=True,
        capture_duration_seconds=70.,normal_teacher_native_intervals=1400,
        whole_capture_is_bounded_turn_not_full_route_goal_stop=True)
    return result
