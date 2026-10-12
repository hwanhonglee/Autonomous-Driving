"""HH_261002 - Synthetic RIGHT70 exact context and cleanup receipt tests; no raw capture or admission."""
import ast
import copy
import hashlib
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import audit_town01_right70_closing_control_20261002 as audit
from test_audit_carla_acknowledged_control import chain, control, raw_row
from test_audit_town01_left45_closing_control_20261002 import coast_chain as old_left_chain


def coast_chain():
    receipts, states = [], []
    brake = control(brake=1., gear=1, manual_gear_shift=True)
    zero = control(gear=1, manual_gear_shift=True)
    moving = control(throttle=.15, steer=.02, gear=1, manual_gear_shift=True)
    def send(frame, reason, requested):
        receipts.append(dict(sequence=len(receipts)+1, actor_id=42, response_actor_id=42, before_frame=frame,
            after_ack_frame=frame, mode="acknowledged_batch", do_tick=False, status="ACKNOWLEDGED",
            server_accepted=True, physical_actuation_proven=False, reason=reason, requested_control=copy.deepcopy(requested)))
        return len(receipts)
    send(99, "pre_capture_bootstrap", brake)
    bootstrap = raw_row(100, "stationary_warmup", brake, 1)
    bootstrap["alignment"] = bootstrap.pop("control_transport"); bootstrap["recorded_training_state"] = False
    send(100, "stationary_warmup_start", brake)
    for index in range(1555):
        frame = 101 + index
        phase = "stationary_warmup" if index < 24 else "driving" if index < 1425 else "stationary_tail"
        row = raw_row(frame, phase, receipts[-1]["requested_control"], receipts[-1]["sequence"])
        value = brake if phase == "stationary_warmup" else moving if phase == "driving" and index < 1424 else zero
        row["next_control"] = copy.deepcopy(value)
        row["control_transport"]["next_command_receipt_sequence"] = send(frame, "next_" + phase + "_control", value)
        if index == 23:
            row["next_control"] = copy.deepcopy(moving)
            row["control_transport"]["next_command_receipt_sequence"] = send(frame, "initial_drive_control", moving)
        if index == 1424:
            row["moving_task"] = dict(future_support_complete=True, termination_reason="recovery_future_support_complete")
            row["control_transport"]["next_command_receipt_sequence"] = send(frame, "driving_end_brake", zero)
        states.append(row)
    send(states[-1]["frame"], audit.CLOSING_REASON, brake)
    return states, receipts, bootstrap


def test_inverse_AST_four_count_values_and_only_count_error_labels():
    changed = audit.definition(); counts = dict.fromkeys(audit.COUNTS.values(), 0)
    for node in ast.walk(changed):
        if isinstance(node, ast.Constant) and type(node.value) is int and node.value in counts:
            counts[node.value] += 1
            node.value = next(old for old, new in audit.COUNTS.items() if new == node.value)
        elif isinstance(node, ast.Constant) and type(node.value) is str and node.value in audit.ERROR_IDENTITIES.values():
            node.value = next(old for old, new in audit.ERROR_IDENTITIES.items() if new == node.value)
    assert counts == {1555: 1, 1401: 1, 1425: 2, 1400: 2}
    old = ast.parse(Path(audit.previous.__file__).read_bytes())
    expected = ast.Module(body=[n for n in old.body if isinstance(n, ast.FunctionDef) and n.name != "verify_frozen"], type_ignores=[])
    assert ast.dump(changed) == ast.dump(expected)


def test_entire_exact_RIGHT70_context_all_original_flags_without_cleanup_stop():
    value = coast_chain(); before = copy.deepcopy(value)
    with pytest.raises(audit.original.base.EvidenceError, match="unknown"):
        audit.original.analyze_transport(*value)
    result = audit.analyze_transport(*value)
    assert result["status"] == "PASS" and all(result["flags"].values())
    assert result["recorded_state_count"] == 1555 and result["receipt_count"] == 1560
    assert result["prospective_context_reader_schema"] == audit.SCHEMA
    assert result["declared_zero_pedal_coast_context"] == dict(boundary_frame=1525, boundary_receipt_sequence=1429,
        observed_coast_native_count=130, final_support_native_offset=1400,
        diagnostic_context_only=True, STOP_or_stationary_labels_claimed=False)
    assert result["post_capture_closing_receipt"] == dict(receipt_sequence=1560, recorded_native_frame=1655,
        actor_id=42, no_tick_requested=True, no_post_command_pose_recorded=True,
        observed_STOP_or_stationary_completion_claimed=False)
    assert result["physical_actuation_proven"] is False and result["observed_cleanup_stop_claimed"] is False
    assert value == before


@pytest.mark.parametrize("change", ["short", "phase", "frame", "no_support", "failure", "boundary_manual", "boundary_tick",
    "boundary_actor", "support_next", "support_link", "own_next", "tail_current", "tail_next", "tail_prior",
    "tail_reason", "tail_tick", "tail_actor"])
def test_incomplete_or_unbound_coast_never_relaxes_brake_guard(change):
    states, receipts, bootstrap = coast_chain()
    boundary = next(row for row in receipts if row["reason"] == "driving_end_brake")
    if change == "short": states.pop(1425)
    elif change == "phase": states[1425]["capture_phase"] = "driving"
    elif change == "frame": states[1426]["frame"] += 1
    elif change == "no_support": states[1424]["moving_task"]["future_support_complete"] = False
    elif change == "failure": states[1424]["moving_task"]["termination_reason"] = "collision"
    elif change == "boundary_manual": boundary["requested_control"]["manual_gear_shift"] = False
    elif change == "boundary_tick": boundary["do_tick"] = True
    elif change == "boundary_actor": boundary["response_actor_id"] = 43
    elif change == "support_next": states[1424]["next_control"]["throttle"] = .01
    elif change == "support_link": states[1424]["control_transport"]["next_command_receipt_sequence"] -= 1
    elif change == "own_next": receipts[boundary["sequence"]-2]["reason"] = "next_stationary_tail_control"
    elif change == "tail_current": states[1450]["current_control"]["brake"] = 1.
    elif change == "tail_next": states[1450]["next_control"]["throttle"] = .01
    elif change == "tail_prior": states[1450]["control_transport"]["expected_receipt_sequence"] -= 1
    else:
        row = receipts[states[1450]["control_transport"]["next_command_receipt_sequence"]-1]
        if change == "tail_reason": row["reason"] = "next_driving_control"
        elif change == "tail_tick": row["do_tick"] = True
        else: row["actor_id"] = 43
    with pytest.raises((audit.original.base.EvidenceError, KeyError)):
        audit.analyze_transport(states, receipts, bootstrap)


@pytest.mark.parametrize("change", ["early", "duplicate", "tick", "before", "after", "actor", "response", "failed", "rejected",
    "throttle", "steer", "brake", "auto", "gear", "hand", "reverse", "linked"])
def test_closing_is_one_final_same_actor_fullbrake_without_tick(change):
    states, receipts, bootstrap = coast_chain(); row = receipts[-1]
    if change == "early": receipts[-1], receipts[-2] = receipts[-2], receipts[-1]
    elif change == "duplicate": receipts[-2]["reason"] = audit.CLOSING_REASON
    elif change == "tick": row["do_tick"] = True
    elif change == "before": row["before_frame"] += 1
    elif change == "after": row["after_ack_frame"] += 1
    elif change == "actor": row["actor_id"] += 1
    elif change == "response": row["response_actor_id"] += 1
    elif change == "failed": row.update(status="FAILED", error="failed")
    elif change == "rejected": row["server_accepted"] = False
    elif change in ("throttle", "steer"): row["requested_control"][change] = .01
    elif change == "brake": row["requested_control"]["brake"] = .99
    elif change == "auto": row["requested_control"]["manual_gear_shift"] = False
    elif change == "gear": row["requested_control"]["gear"] = 2
    elif change == "hand": row["requested_control"]["hand_brake"] = True
    elif change == "reverse": row["requested_control"]["reverse"] = True
    else: states[-1]["control_transport"]["next_command_receipt_sequence"] = row["sequence"]
    with pytest.raises((audit.original.base.EvidenceError, KeyError)):
        audit.analyze_transport(states, receipts, bootstrap)


def test_left_context_or_unknown_reason_still_rejected():
    value = old_left_chain()
    value[1][-1]["reason"] = audit.CLOSING_REASON
    with pytest.raises(audit.original.base.EvidenceError, match="all1555"):
        audit.analyze_transport(*value)
    states, receipts, bootstrap = coast_chain(); receipts[-1]["reason"] = "foreign_closing_reason"
    with pytest.raises(audit.original.base.EvidenceError, match="unknown"):
        audit.analyze_transport(states, receipts, bootstrap)


def test_bootstrap_and_failed_None_boundary_keep_original_fullbrake_guard(chain):
    states, receipts, bootstrap = copy.deepcopy(chain)
    boundary = next(row for row in receipts if row["reason"] == "driving_end_brake")
    boundary.update(before_frame=None, status="FAILED", server_accepted=False, error="no frame")
    boundary["requested_control"] = control()
    result = audit.private_analyze()(states, receipts, bootstrap)
    assert result["status"] == "FAIL" and "phase_command_ledger" in result["failed_flags"]
    states, receipts, bootstrap = coast_chain(); receipts[0]["requested_control"]["brake"] = 0.
    result = audit.analyze_transport(states, receipts, bootstrap)
    assert result["status"] == "FAIL" and "phase_command_ledger" in result["failed_flags"]


def test_unchanged_original_helper_namespace_and_frozen_bytes():
    audit.verify_frozen(); provider = audit.provider()
    assert provider.__file__ == audit.__file__ and provider.SCHEMA == audit.SCHEMA
    assert provider.analyze_transport is audit.analyze_transport
    assert provider.analyze_camera_alignment is audit.original.analyze_camera_alignment
    for name in ("full_control", "mismatched_controls", "recompute_snapshot_state", "primary_source_evidence"):
        assert getattr(provider, name) is getattr(audit.original, name)
    assert audit.CLOSING_REASON not in audit.original.REASONS
    assert audit.previous.CLOSING_REASON not in audit.private_analyze().__globals__["REASONS"]
    assert hashlib.sha256(Path(audit.previous.__file__).read_bytes()).hexdigest() == audit.PREVIOUS_SHA
    with mock.patch.object(audit, "PREVIOUS_SHA", "0" * 64):
        with pytest.raises(audit.original.base.EvidenceError): audit.definition()
