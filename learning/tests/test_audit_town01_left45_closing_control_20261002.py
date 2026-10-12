"""HH_261002 - Synthetic exact closing receipt/frame checks; no capture, model, admission or source rewriting."""
import ast
import copy
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import audit_town01_left45_closing_control_20261002 as audit
from test_audit_carla_acknowledged_control import chain, control, raw_row


def closed(chain):
    states, receipts, bootstrap = copy.deepcopy(chain)
    row = dict(receipts[-1], sequence=len(receipts)+1, reason=audit.CLOSING_REASON,
        requested_control=control(brake=1., gear=1, manual_gear_shift=True))
    receipts.append(row)
    return states, receipts, bootstrap


def test_original_source_ast_and_globals_unchanged(chain):
    before = copy.deepcopy(audit.original.REASONS)
    source = ast.parse(Path(audit.original.__file__).read_bytes())
    original = next(node for node in source.body if isinstance(node, ast.FunctionDef) and node.name == "analyze_transport")
    assert ast.dump(audit.audit_definition()) == ast.dump(original)
    fn = audit.private_analyze()
    assert fn.__globals__ is not audit.original.__dict__
    assert fn.__globals__["REASONS"] == before | {audit.CLOSING_REASON}
    assert audit.original.REASONS == before
    assert audit.original.analyze_transport(*chain) == fn(*chain)


def test_exact_final_no_tick_manual_brake_is_accepted_without_stop_claim(chain):
    value = closed(chain); before = copy.deepcopy(value)
    with pytest.raises(audit.original.base.EvidenceError, match="unknown"):
        audit.original.analyze_transport(*value)
    result = audit.analyze_transport(*value)
    assert result["status"] == "PASS" and result["receipt_count"] == 12
    assert result["post_capture_closing_receipt"] == dict(receipt_sequence=12, recorded_native_frame=107,
        actor_id=42, no_tick_requested=True, no_post_command_pose_recorded=True,
        observed_STOP_or_stationary_completion_claimed=False)
    assert result["physical_actuation_proven"] is False and result["observed_cleanup_stop_claimed"] is False
    assert result["failed_V1_capture_requalified"] is False and value == before


@pytest.mark.parametrize("change", ["early", "duplicate", "tick", "wrong_before", "wrong_after", "wrong_actor",
    "wrong_response", "failed", "rejected", "throttle", "steer", "brake", "auto", "gear", "hand", "reverse", "linked", "no_states"])
def test_every_closing_precondition_fails_closed(chain, change):
    states, receipts, bootstrap = closed(chain); row = receipts[-1]
    if change == "early": receipts[-1], receipts[-2] = receipts[-2], receipts[-1]
    elif change == "duplicate": receipts[-2]["reason"] = audit.CLOSING_REASON
    elif change == "tick": row["do_tick"] = True
    elif change == "wrong_before": row["before_frame"] += 1
    elif change == "wrong_after": row["after_ack_frame"] += 1
    elif change == "wrong_actor": row["actor_id"] += 1
    elif change == "wrong_response": row["response_actor_id"] += 1
    elif change == "failed": row.update(status="FAILED", error="transport failed")
    elif change == "rejected": row["server_accepted"] = False
    elif change == "throttle": row["requested_control"]["throttle"] = .01
    elif change == "steer": row["requested_control"]["steer"] = .01
    elif change == "brake": row["requested_control"]["brake"] = .99
    elif change == "auto": row["requested_control"]["manual_gear_shift"] = False
    elif change == "gear": row["requested_control"]["gear"] = 2
    elif change == "hand": row["requested_control"]["hand_brake"] = True
    elif change == "reverse": row["requested_control"]["reverse"] = True
    elif change == "linked": states[-1]["control_transport"]["next_command_receipt_sequence"] = row["sequence"]
    else: states.clear()
    with pytest.raises((audit.original.base.EvidenceError, KeyError)):
        audit.analyze_transport(states, receipts, bootstrap)


def test_other_unknown_reasons_still_denied(chain):
    states, receipts, bootstrap = closed(chain)
    receipts[3]["reason"] = "foreign_post_capture_cleanup"
    with pytest.raises(audit.original.base.EvidenceError, match="unknown"):
        audit.analyze_transport(states, receipts, bootstrap)


def test_partial_zero_pedal_driving_end_does_not_relax_original_guard(chain):
    states, receipts, bootstrap = closed(chain)
    boundary = next(row for row in receipts if row["reason"] == "driving_end_brake")
    boundary["requested_control"] = control(gear=1, manual_gear_shift=True)
    with pytest.raises(audit.original.base.EvidenceError, match="all1055"):
        audit.analyze_transport(states, receipts, bootstrap)


def test_failed_original_alignment_or_snapshot_cannot_be_hidden(chain):
    states, receipts, bootstrap = closed(chain); states[3]["x"] += 1.
    result = audit.analyze_transport(states, receipts, bootstrap)
    assert result["status"] == "FAIL" and "all_snapshot_conversions_match" in result["failed_flags"]


def test_absent_closing_keeps_original_algorithm_and_denominators(chain):
    before = audit.original.analyze_transport(*chain); result = audit.analyze_transport(*chain)
    assert all(result[key] == value for key, value in before.items())
    assert result["post_capture_closing_receipt"] is None


def test_provider_preserves_original_helpers_and_new_identity():
    provider = audit.provider()
    assert provider.__file__ == audit.__file__ and provider.SCHEMA == audit.SCHEMA
    assert provider.analyze_transport is audit.analyze_transport
    assert provider.analyze_camera_alignment is audit.original.analyze_camera_alignment
    for name in ("full_control", "mismatched_controls", "recompute_snapshot_state", "primary_source_evidence"):
        assert getattr(provider, name) is getattr(audit.original, name)


def test_parent_source_tamper_rejects_before_private_compilation():
    with mock.patch.object(audit, "ORIGINAL_SHA", "0"*64):
        with pytest.raises(audit.original.base.EvidenceError, match="immutable"):
            audit.private_analyze()


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
    for index in range(1055):
        frame = 101+index
        phase = "stationary_warmup" if index < 24 else "driving" if index < 925 else "stationary_tail"
        row = raw_row(frame, phase, receipts[-1]["requested_control"], receipts[-1]["sequence"])
        value = brake if phase == "stationary_warmup" else moving if phase == "driving" and index < 924 else zero
        row["next_control"] = copy.deepcopy(value)
        row["control_transport"]["next_command_receipt_sequence"] = send(frame, "next_"+phase+"_control", value)
        if index == 23:
            row["next_control"] = copy.deepcopy(moving)
            row["control_transport"]["next_command_receipt_sequence"] = send(frame, "initial_drive_control", moving)
        if index == 924:
            row["moving_task"] = dict(future_support_complete=True, termination_reason="recovery_future_support_complete")
            row["control_transport"]["next_command_receipt_sequence"] = send(frame, "driving_end_brake", zero)
        states.append(row)
    send(states[-1]["frame"], audit.CLOSING_REASON, brake)
    return states, receipts, bootstrap


def test_complete_actual_coast_context_binds130_native_and_original_all_flags():
    value = coast_chain(); before = copy.deepcopy(value)
    with pytest.raises(audit.original.base.EvidenceError, match="unknown"):
        audit.original.analyze_transport(*value)
    result = audit.analyze_transport(*value)
    assert result["status"] == "PASS" and all(result["flags"].values())
    assert result["recorded_state_count"] == 1055 and result["receipt_count"] == 1060
    assert result["declared_zero_pedal_coast_context"] == dict(boundary_frame=1025, boundary_receipt_sequence=929,
        observed_coast_native_count=130, final_support_native_offset=900,
        diagnostic_context_only=True, STOP_or_stationary_labels_claimed=False)
    assert result["post_capture_closing_receipt"]["recorded_native_frame"] == 1155
    assert result["physical_actuation_proven"] is False and value == before


@pytest.mark.parametrize("change", ["short", "wrong_phase", "wrong_frame", "no_support", "failure", "wrong_boundary",
    "boundary_tick", "boundary_actor", "wrong_support_next", "wrong_support_link", "missing_own_next", "tail_current",
    "tail_next", "tail_prior_link", "tail_next_reason", "tail_ack_tick", "tail_actor"])
def test_context_cannot_hide_incomplete_or_unbound_coast(change):
    states, receipts, bootstrap = coast_chain()
    boundary = next(row for row in receipts if row["reason"] == "driving_end_brake")
    if change == "short": states.pop(925)
    elif change == "wrong_phase": states[925]["capture_phase"] = "driving"
    elif change == "wrong_frame": states[926]["frame"] += 1
    elif change == "no_support": states[924]["moving_task"]["future_support_complete"] = False
    elif change == "failure": states[924]["moving_task"]["termination_reason"] = "collision"
    elif change == "wrong_boundary": boundary["requested_control"]["manual_gear_shift"] = False
    elif change == "boundary_tick": boundary["do_tick"] = True
    elif change == "boundary_actor": boundary["response_actor_id"] = 43
    elif change == "wrong_support_next": states[924]["next_control"]["throttle"] = .01
    elif change == "wrong_support_link": states[924]["control_transport"]["next_command_receipt_sequence"] -= 1
    elif change == "missing_own_next": receipts[boundary["sequence"]-2]["reason"] = "next_stationary_tail_control"
    elif change == "tail_current": states[950]["current_control"]["brake"] = 1.
    elif change == "tail_next": states[950]["next_control"]["throttle"] = .01
    elif change == "tail_prior_link": states[950]["control_transport"]["expected_receipt_sequence"] -= 1
    else:
        receipt = receipts[states[950]["control_transport"]["next_command_receipt_sequence"]-1]
        if change == "tail_next_reason": receipt["reason"] = "next_driving_control"
        elif change == "tail_ack_tick": receipt["do_tick"] = True
        else: receipt["actor_id"] = 43
    with pytest.raises((audit.original.base.EvidenceError, KeyError)):
        audit.analyze_transport(states, receipts, bootstrap)


def test_coast_exception_is_only_one_authenticated_boundary_predicate():
    fn = audit.private_analyze(1025)
    assert fn.__globals__["_verified_coast_frame"] == 1025
    unchanged = audit.audit_definition()
    assert "_verified_coast_frame" not in ast.unparse(unchanged)
    # HH_261002 - A genuine non-contextual abort or bootstrap still requires exact original full brake.
    states, receipts, bootstrap = coast_chain()
    receipts[0]["requested_control"]["brake"] = 0.
    result = audit.analyze_transport(states, receipts, bootstrap)
    assert result["status"] == "FAIL" and "phase_command_ledger" in result["failed_flags"]


def test_none_failed_boundary_keeps_original_full_brake_predicate(chain):
    states, receipts, bootstrap = copy.deepcopy(chain)
    boundary = next(row for row in receipts if row["reason"] == "driving_end_brake")
    boundary.update(before_frame=None, status="FAILED", server_accepted=False, error="no frame")
    boundary["requested_control"] = control()
    result = audit.private_analyze()(states, receipts, bootstrap)
    assert result["status"] == "FAIL"
    assert any(row["check"] == "phase_command_ledger" and row["detail"] == "bootstrap/tail/abort is not the declared full-brake command"
        for row in result["failures"])
