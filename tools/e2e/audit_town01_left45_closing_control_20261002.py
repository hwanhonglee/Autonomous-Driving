"""HH_261002 - Explicit prospective LEFT45 closing receipt; no V1 requalification or observed STOP claim."""
from __future__ import annotations

import ast
import copy
import hashlib
from pathlib import Path
from types import SimpleNamespace

from scripts.e2e import audit_carla_acknowledged_control as original

SCHEMA = "carla.actual_town01_left45_closing_control_audit.20261002.v2"
ORIGINAL_SHA = "9d6fccb6874a5cdfc85e8bf5d9229435e4ba6fe6850eb63a9e83488f39bcc5bd"
CLOSING_REASON = "left45_post_capture_cleanup_full_brake"
analyze_camera_alignment = original.analyze_camera_alignment
require = original.base.require


def verify_frozen():
    path = Path(original.__file__)
    require(hashlib.sha256(path.read_bytes()).hexdigest() == ORIGINAL_SHA,
        "immutable original ACK audit changed")


def audit_definition():
    # HH_261002 - Compile exactly the original numerical/frame algorithm in a private namespace, not a global allowlist patch.
    verify_frozen()
    tree = ast.parse(Path(original.__file__).read_bytes())
    found = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "analyze_transport"]
    require(len(found) == 1, "one immutable transport algorithm required")
    return copy.deepcopy(found[0])


def private_analyze(coast_frame=None):
    node = audit_definition()
    inverse = copy.deepcopy(node)
    matches = [item for item in ast.walk(node) if isinstance(item, ast.If)
        and isinstance(item.test, ast.Compare) and isinstance(item.test.left, ast.Name) and item.test.left.id == "reason"
        and len(item.test.ops) == 1 and isinstance(item.test.ops[0], ast.In)
        and isinstance(item.test.comparators[0], ast.Tuple)
        and any(isinstance(value, ast.Constant) and value.value == "driving_end_brake" for value in item.test.comparators[0].elts)]
    require(len(matches) == 1, "one original boundary full-brake predicate required")
    old_test = matches[0].test
    matches[0].test = ast.BoolOp(op=ast.And(), values=[old_test,
        ast.parse('not (_verified_coast_frame is not None and reason == "driving_end_brake" and before == _verified_coast_frame)', mode="eval").body])
    restored = copy.deepcopy(node)
    restored_match = next(item for item in ast.walk(restored) if isinstance(item, ast.If)
        and isinstance(item.test, ast.BoolOp) and ast.dump(item.test.values[0]) == ast.dump(old_test))
    restored_match.test = restored_match.test.values[0]
    require(ast.dump(restored) == ast.dump(inverse), "unrelated original transport checks changed")
    namespace = dict(original.__dict__, __file__=__file__, REASONS=original.REASONS | {CLOSING_REASON},
        _verified_coast_frame=coast_frame)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), __file__, "exec"), namespace)
    return namespace["analyze_transport"]


def coast_context(states, receipts):
    boundaries = [row for row in receipts if row.get("reason") == "driving_end_brake"]
    if not boundaries or boundaries[0].get("requested_control", {}).get("brake") != 0.:
        return None  # HH_261002 - Every non-contextual original full-brake expectation remains effective.
    require(len(boundaries) == 1 and len(states) == 1055, "one boundary and all1055 declared LEFT45 observations required")
    require([row.get("capture_phase") for row in states] == ["stationary_warmup"]*24+["driving"]*901+["stationary_tail"]*130,
        "exact24 warmup/901 driving/130 diagnostic coast phase census required")
    require(all(type(row.get("frame")) is int for row in states)
        and all(b["frame"] == a["frame"]+1 for a,b in zip(states,states[1:])), "coast observations must preserve every native frame")
    driving, tail, boundary = states[24:925], states[925:], boundaries[0]
    last = driving[-1]; record = last.get("moving_task", {})
    require(last["frame"] == driving[0]["frame"]+900 and boundary.get("before_frame") == last["frame"]
        and record.get("future_support_complete") is True and record.get("termination_reason") == "recovery_future_support_complete",
        "zero-pedal boundary must follow the exact declared true-future support observation")
    zero = dict(throttle=0., brake=0., steer=0., gear=1, manual_gear_shift=True, reverse=False, hand_brake=False)
    by_sequence = {row["sequence"]: row for row in receipts}
    actor = boundary.get("actor_id")
    def accepted(row, frame):
        require(type(actor) is int and actor > 0 and row.get("actor_id") == row.get("response_actor_id") == actor
            and row.get("status") == "ACKNOWLEDGED" and row.get("server_accepted") is True
            and row.get("mode") == "acknowledged_batch" and row.get("do_tick") is False
            and row.get("physical_actuation_proven") is False and "error" not in row
            and row.get("before_frame") == row.get("after_ack_frame") == frame,
            "each coast control must be accepted by the same actor without a native tick")
        require(original.full_control(row["requested_control"]) == zero, "coast ACK must be exact zero-pedal manual1")
    accepted(boundary, last["frame"])
    require(original.full_control(last["next_control"]) == zero
        and last["control_transport"]["next_command_receipt_sequence"] == boundary["sequence"],
        "final true-future support pose must retain its real zero-pedal boundary next-command link")
    before_boundary = [row for row in receipts if row.get("reason") == "next_driving_control"
        and row.get("before_frame") == last["frame"]]
    require(len(before_boundary) == 1 and before_boundary[0]["sequence"]+1 == boundary["sequence"],
        "final support observation needs its own unchanged next-driving command followed by the coast boundary")
    accepted(before_boundary[0], last["frame"])
    previous = boundary
    for row in tail:
        alignment = row["control_transport"]
        require(original.full_control(row["current_control"]) == zero and original.full_control(row["next_control"]) == zero
            and alignment["expected_receipt_sequence"] == previous["sequence"]
            and previous["before_frame"] == row["frame"]-1, "all130 observed coast controls require their actual preceding ACK")
        next_receipt = by_sequence[alignment["next_command_receipt_sequence"]]
        require(next_receipt.get("reason") == "next_stationary_tail_control", "every coast observation needs its own next-tail receipt")
        accepted(next_receipt, row["frame"]); previous = next_receipt
    return dict(boundary_frame=last["frame"], boundary_receipt_sequence=boundary["sequence"],
        observed_coast_native_count=130, final_support_native_offset=900,
        diagnostic_context_only=True, STOP_or_stationary_labels_claimed=False)


def closing_receipt(states, receipts):
    require(isinstance(states, list) and isinstance(receipts, list), "actual state and receipt journals required")
    matches = [row for row in receipts if row.get("reason") == CLOSING_REASON]
    require(len(matches) <= 1, "one post-capture closing command maximum")
    if not matches:
        return None  # HH_261002 - Original failed/partial evidence remains assessed by the unchanged algorithm.
    row = matches[0]
    require(bool(states) and row is receipts[-1] and row["sequence"] == len(receipts),
        "closing command must be the final receipt after an actual last recorded pose")
    last = states[-1]
    require(type(last.get("frame")) is int and type(row.get("before_frame")) is int
        and row["before_frame"] == last["frame"] and type(row.get("after_ack_frame")) is int
        and row["after_ack_frame"] == last["frame"] and row.get("do_tick") is False,
        "closing ACK cannot advance the last recorded native frame")
    require(row.get("mode") == "acknowledged_batch" and row.get("status") == "ACKNOWLEDGED"
        and row.get("server_accepted") is True and row.get("physical_actuation_proven") is False
        and "error" not in row, "closing command needs the original accepted no-actuation-proof protocol")
    actor = row.get("actor_id")
    require(type(actor) is int and actor > 0 and type(row.get("response_actor_id")) is int
        and row["response_actor_id"] == actor and all(item.get("actor_id") == actor for item in receipts),
        "closing command must belong to the same owned actor")
    expected = dict(throttle=0., brake=1., steer=0., gear=1, manual_gear_shift=True, reverse=False, hand_brake=False)
    require(original.full_control(row["requested_control"]) == expected,
        "closing request must be exact manual1 full brake with zero throttle/steering")
    link = last.get("control_transport", {}).get("next_command_receipt_sequence")
    require(type(link) is int and 0 < link < row["sequence"],
        "closing command is separate from the last recorded pose's own next-command link")
    return dict(receipt_sequence=row["sequence"], recorded_native_frame=last["frame"],
        actor_id=actor, no_tick_requested=True, no_post_command_pose_recorded=True,
        observed_STOP_or_stationary_completion_claimed=False)


def analyze_transport(states, receipts, bootstrap, *, wheelbase=2.85):
    proof = closing_receipt(states, receipts)
    coast = coast_context(states, receipts)
    result = private_analyze(None if coast is None else coast["boundary_frame"])(states, receipts, bootstrap, wheelbase=wheelbase)
    result.update(prospective_context_reader_schema=SCHEMA, post_capture_closing_receipt=proof,
        declared_zero_pedal_coast_context=coast,
        failed_V1_capture_requalified=False, observed_cleanup_stop_claimed=False)
    verify_frozen()
    return result


def provider():
    verify_frozen()
    return SimpleNamespace(**dict(original.__dict__, __file__=__file__, SCHEMA=SCHEMA,
        analyze_transport=analyze_transport, analyze_camera_alignment=analyze_camera_alignment))
