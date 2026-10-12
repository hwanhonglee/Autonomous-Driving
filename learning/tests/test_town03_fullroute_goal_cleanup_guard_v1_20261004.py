"""HH_261004 - Check actual catalog cleanup authentication on CPU without simulator or policy activation."""

import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import town03_fullroute_goal_cleanup_guard_v1_20261004 as guard


class FakeControl:
    # HH_261004 - Mirror only the original VehicleControl scalar ABI; do not simulate physical effects.
    def __init__(self, throttle=0., steer=0., brake=0., hand_brake=False, reverse=False,
                 gear=0, manual_gear_shift=False):
        self.throttle, self.steer, self.brake = throttle, steer, brake
        self.hand_brake, self.reverse = hand_brake, reverse
        self.gear, self.manual_gear_shift = gear, manual_gear_shift


def fixture(tmp_path):
    carla = SimpleNamespace(VehicleControl=FakeControl)
    teacher = guard.collector._blank_brake_control(carla)
    requested = guard.collector.control_dict(teacher)
    route = dict(town="Town03", scenario="straight", route=[
        dict(x=0., y=0., distance_m=0., vad_command=0),
        dict(x=10., y=0., distance_m=10., vad_command=0)])
    alignment = dict(status="PASS", frame_binding_pass=True, observed_frame=42,
        control_read_frame_before=42, control_read_frame_after=42, expected_receipt_sequence=10,
        mismatched_fields=[], next_command_receipt_sequence=10)
    state = dict(frame=42, timestamp=2.1, capture_phase="driving", x=8., y=0., z=0., yaw=0.,
        route_progress_m=8., world_velocity_carla=(2., 0., 0.), current_control=requested,
        control_transport=alignment, collision=[], lane_invasion=[])
    prior = dict(sequence=10, status="ACKNOWLEDGED", server_accepted=True, actor_id=17,
        response_actor_id=17, mode="acknowledged_batch", do_tick=False, after_ack_frame=41, before_frame=41,
        requested_control=dict(requested))
    timing = dict(ACK_sequence=10, ACK_server_accepted=True, learned_control_proposed=True,
        control_source="learned_trajectory_tracker", command_frame=41,
        source_checkpoint_sha256="a" * 64, stream_source_sha256="b" * 64)
    session = SimpleNamespace(states=[state], args=SimpleNamespace(goal_tolerance_m=2.5),
        handed_off=True, latched_emergency=None, goal_cleanup_catalog_route=route,
        route=((0., 0.), (10., 0.)), wall_timing_records=[timing], checkpoint_sha256="a" * 64,
        source_sha256="b" * 64, output=tmp_path)
    transport = SimpleNamespace(carla=carla, ego=SimpleNamespace(id=17, is_alive=True),
        frame_reader=lambda: 42, receipts=[prior])
    goal = guard.collector.catalog_goal_status(8., 10., 8., 0., 10., 0., 2.5)
    return session, transport, teacher, goal


def authenticate(values, **changes):
    session, transport, teacher, goal = values
    return guard.authenticate_goal_cleanup(session, transport, changes.pop("teacher", teacher),
        changes.pop("expected_before_frame", 42), changes.pop("goal_status", goal),
        changes.pop("event_snapshot", ({}, {})), changes.pop("source_check", lambda: None), **changes)


def native_guard(session, free=None):
    if free is None:
        free = guard.frozen.DISK_FLOOR_BYTES + guard.frozen.CLOSING_RESERVE_BYTES
    with mock.patch.object(guard.shutil, "disk_usage", return_value=SimpleNamespace(free=free)):
        return guard.run_original_native_guard(session)


def claim(values):
    session, transport, teacher, _ = values
    return guard.claim_goal_cleanup_send(session, transport, teacher, 42)


def actual_cleanup_ack(values):
    session, transport, teacher, _ = values
    receipt = dict(sequence=11, status="ACKNOWLEDGED", server_accepted=True, reason=guard.GOAL_REASON,
        mode="acknowledged_batch", do_tick=False, actor_id=17, response_actor_id=17,
        expected_before_frame=42, before_frame=42, after_ack_frame=42,
        requested_control=guard.collector.control_dict(teacher))
    transport.receipts.append(receipt)
    return receipt


def test_frozen_dependencies_and_no_ordinary_policy_edit():
    guard.verify_frozen()
    for module, expected in ((guard.collector, guard.COLLECTOR_SHA), (guard.frozen, guard.TEMPORAL_COLLECTOR_SHA)):
        assert hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest() == expected
    assert "teacher.brake > .05" in Path(guard.frozen.__file__).read_text()


@pytest.mark.parametrize("phase", ["driving", "stationary_warmup", "stationary_tail"])
def test_reason_ordinary_strings_and_alignment_first(phase):
    goal = guard.collector.CatalogGoalStatus(True, 2., 2.)
    assert guard.choose_goal_control_reason({"status": "FAIL"}, phase, "catalog_goal_tolerance", goal) == "alignment_failure_abort"
    assert guard.choose_goal_control_reason({"status": "PASS"}, phase, None, goal) == "next_" + phase + "_control"
    assert guard.choose_goal_control_reason(None, phase, "basic_agent_done_before_catalog_goal", goal) == "next_" + phase + "_control"
    expected = guard.GOAL_REASON if phase == "driving" else "next_" + phase + "_control"
    assert guard.choose_goal_control_reason({"status": "PASS"}, phase, "catalog_goal_tolerance", goal) == expected


def test_goal_false_and_ordinary_hazard_reason_are_not_rewritten():
    false = guard.collector.CatalogGoalStatus(False, 3., 1.)
    assert guard.choose_goal_control_reason(None, "driving", "catalog_goal_tolerance", false) == "next_driving_control"
    assert guard.choose_goal_control_reason(None, "driving", None, None) == "next_driving_control"
    assert guard.choose_goal_control_reason(None, "driving", "collision event", false) == "next_driving_control"


def test_true_goal_pending_guard_claim_and_actual_original_ack(tmp_path):
    values = fixture(tmp_path)
    session, transport, _, _ = values
    pending = authenticate(values)
    assert pending["status"] == "PREPARED" and pending["learned_control_proposed"] is False
    assert pending["current_tick_hazard_detector_clear_proven"] is False
    pending["requested_original_cleanup_control"]["steer"] = .9
    assert session._goal_cleanup_pending["requested_original_cleanup_control"]["steer"] == 0.
    checked = native_guard(session)
    assert checked["required_free_bytes"] == 5 * 1024**3 + 16 * 1024**2
    assert claim(values)["status"] == "SEND_CLAIMED"
    session.states[-1]["control_transport"]["next_command_receipt_sequence"] = 11
    receipt = actual_cleanup_ack(values)
    result = guard.complete_goal_cleanup_receipt(session, transport, receipt)
    assert result["status"] == "ACKNOWLEDGED" and result["cleanup_ACK_sequence"] == 11
    assert result["actual_prior_learned_ACK_effect_proven"] is True
    assert result["physical_cleanup_actuation_proven"] is False and result["learned_STOP_proven"] is False
    assert session.states[-1]["control_transport"]["expected_receipt_sequence"] == 10
    assert json.loads((tmp_path / "native_guard.jsonl").read_text()) == checked
    with pytest.raises(Exception, match="create-only"):
        authenticate(values)
    with pytest.raises(Exception, match="only once"):
        claim(values)
    with pytest.raises(Exception, match="claimed"):
        guard.complete_goal_cleanup_receipt(session, transport, receipt)


@pytest.mark.parametrize("change", ["remaining", "planar", "false", "wrong_goal", "tolerance", "catalog", "navigation"])
def test_actual_original_two_goal_predicates_and_geometry(tmp_path, change):
    values = fixture(tmp_path)
    session, _, _, goal = values
    overrides = {}
    if change == "remaining": session.states[-1]["route_progress_m"] = 7.
    elif change == "planar": session.states[-1]["y"] = 3.
    elif change == "false": overrides["goal_status"] = guard.collector.CatalogGoalStatus(False, 2., 2.)
    elif change == "wrong_goal": overrides["goal_status"] = guard.collector.CatalogGoalStatus(True, 1., 2.)
    elif change == "tolerance": session.args.goal_tolerance_m = 3.
    elif change == "catalog": session.goal_cleanup_catalog_route["town"] = "Town07"
    elif change == "navigation": session.route = ((0., 0.), (11., 0.))
    with pytest.raises(Exception): authenticate(values, **overrides)
    assert getattr(session, "_goal_cleanup_pending", None) is None


@pytest.mark.parametrize("change", ["phase", "frame", "world_frame", "bool_world_frame", "alignment", "binding",
    "observed_frame", "readback", "mismatch", "handoff", "latch", "actor", "dead_actor", "nonfinite", "bool_pose"])
def test_authentication_rejects_wrong_actual_state_or_alignment(tmp_path, change):
    values = fixture(tmp_path)
    session, transport, _, _ = values
    state, alignment = session.states[-1], session.states[-1]["control_transport"]
    overrides = {}
    if change == "phase": state["capture_phase"] = "stationary_tail"
    elif change == "frame": overrides["expected_before_frame"] = 41
    elif change == "world_frame": transport.frame_reader = lambda: 43
    elif change == "bool_world_frame": transport.frame_reader = lambda: True
    elif change == "alignment": alignment["status"] = "FAIL"
    elif change == "binding": alignment["frame_binding_pass"] = False
    elif change == "observed_frame": alignment["observed_frame"] = 41
    elif change == "readback": alignment["control_read_frame_after"] = 43
    elif change == "mismatch": alignment["mismatched_fields"] = ["brake"]
    elif change == "handoff": session.handed_off = False
    elif change == "latch": session.latched_emergency = "collision event"
    elif change == "actor": transport.ego.id = 18
    elif change == "dead_actor": transport.ego.is_alive = False
    elif change == "nonfinite": state["x"] = float("nan")
    elif change == "bool_pose": state["x"] = True
    with pytest.raises(Exception): authenticate(values, **overrides)


@pytest.mark.parametrize("field,value", [("status", "FAILED"), ("server_accepted", False), ("sequence", 9),
    ("actor_id", 18), ("response_actor_id", 18), ("mode", "legacy_async"), ("do_tick", True),
    ("after_ack_frame", 40), ("before_frame", 40)])
def test_prior_ack_must_be_the_actual_previous_owned_interval(tmp_path, field, value):
    values = fixture(tmp_path)
    values[1].receipts[-1][field] = value
    with pytest.raises(Exception, match="previous accepted"): authenticate(values)


@pytest.mark.parametrize("field,value", [("ACK_server_accepted", False), ("learned_control_proposed", False),
    ("control_source", "expert_startup"), ("command_frame", 40), ("source_checkpoint_sha256", "c" * 64),
    ("stream_source_sha256", "c" * 64)])
def test_previous_command_must_be_source_bound_learned_effect(tmp_path, field, value):
    values = fixture(tmp_path)
    values[0].wall_timing_records[0][field] = value
    with pytest.raises(Exception, match="source-bound"): authenticate(values)


def test_prior_duplicate_timing_and_wrong_expected_sequence(tmp_path):
    values = fixture(tmp_path)
    values[0].wall_timing_records *= 2
    with pytest.raises(Exception, match="unique"): authenticate(values)
    values[0].wall_timing_records = values[0].wall_timing_records[:1]
    values[0].states[-1]["control_transport"]["expected_receipt_sequence"] = 9
    with pytest.raises(Exception, match="previous accepted"): authenticate(values)


def test_current_prior_control_uses_original_tolerance_without_mutating_transport(tmp_path):
    values = fixture(tmp_path)
    values[0].states[-1]["current_control"]["steer"] = 5.e-7
    authenticate(values)
    assert not hasattr(values[1], "alignment_failures")
    other = fixture(tmp_path)
    other[0].states[-1]["current_control"]["steer"] = 2.e-6
    with pytest.raises(Exception, match="prior ACK request"): authenticate(other)


@pytest.mark.parametrize("events", [({42: [{}]}, {}), ({}, {41: [{}]}), ({42: []}, {}), [{}, {}], None])
def test_actual_event_census_not_initial_empty_state_arrays(tmp_path, events):
    values = fixture(tmp_path)
    assert values[0].states[-1]["collision"] == values[0].states[-1]["lane_invasion"] == []
    with pytest.raises(Exception, match="event-recorder"): authenticate(values, event_snapshot=events)


@pytest.mark.parametrize("source", [None, lambda: False, lambda: "truthy"])
def test_source_guard_must_be_callable_and_succeed(tmp_path, source):
    with pytest.raises(Exception, match="owner/source/PID"): authenticate(fixture(tmp_path), source_check=source)


def test_source_guard_exception_or_source_frame_change_is_not_swallowed(tmp_path):
    values = fixture(tmp_path)
    def denied(): raise RuntimeError("PID generation changed")
    with pytest.raises(RuntimeError, match="PID generation"): authenticate(values, source_check=denied)
    def crossed(): values[1].frame_reader = lambda: 43
    with pytest.raises(Exception, match="source frame"): authenticate(values, source_check=crossed)


@pytest.mark.parametrize("field,value", [("throttle", .01), ("brake", .9), ("steer", .01),
    ("hand_brake", True), ("reverse", True), ("gear", 1), ("manual_gear_shift", True)])
def test_cleanup_requires_exact_original_blank_brake(tmp_path, field, value):
    values = fixture(tmp_path)
    setattr(values[2], field, value)
    with pytest.raises(Exception, match="blank brake"): authenticate(values)


@pytest.mark.parametrize("change", ["speed", "disk"])
def test_original_native_guard_failures_are_journaled_and_cannot_claim(tmp_path, change):
    values = fixture(tmp_path)
    session = values[0]
    if change == "speed": session.states[-1]["world_velocity_carla"] = (2.700001, 0., 0.)
    authenticate(values)
    free = 5 * 1024**3 + 16 * 1024**2 - int(change == "disk")
    with pytest.raises(Exception, match="speed or storage"): native_guard(session, free)
    assert len((tmp_path / "native_guard.jsonl").read_text().splitlines()) == 1
    assert session._goal_cleanup_pending["native_guard_passed"] is False
    with pytest.raises(Exception, match="one unchanged"): native_guard(session)
    with pytest.raises(Exception, match="native-guarded"): claim(values)


def test_native_guard_uses_unclipped_actor_planar_speed_and_original_threshold(tmp_path):
    values = fixture(tmp_path)
    values[0].states[-1]["world_velocity_carla"] = (0., 2.7, 0.)
    authenticate(values)
    row = native_guard(values[0])
    assert row["measured_actor_planar_speed_mps"] == row["maximum_actor_planar_speed_mps"] == 2.7
    with pytest.raises(Exception, match="one unchanged"): native_guard(values[0])


@pytest.mark.parametrize("change", ["frame", "pose", "actor", "transport", "prior_ACK", "brake", "latch"])
def test_claim_is_one_use_and_rejects_stale_permission(tmp_path, change):
    values = fixture(tmp_path)
    session, transport, teacher, _ = values
    authenticate(values); native_guard(session)
    if change == "frame": transport.frame_reader = lambda: 43
    elif change == "pose": session.states[-1]["x"] = 8.1
    elif change == "actor": transport.ego.id = 18
    elif change == "transport": values = (session, copy.copy(transport), teacher, values[3])
    elif change == "prior_ACK": transport.receipts[-1]["sequence"] = 11
    elif change == "brake": teacher.brake = .9
    elif change == "latch": session.latched_emergency = "late collision event"
    with pytest.raises(Exception): claim(values)


@pytest.mark.parametrize("field,value", [("sequence", 12), ("status", "FAILED"), ("server_accepted", False),
    ("reason", "next_driving_control"), ("mode", "legacy_async"), ("do_tick", True), ("actor_id", 18),
    ("response_actor_id", 18), ("expected_before_frame", 41), ("before_frame", 41), ("after_ack_frame", 43)])
def test_completed_receipt_requires_actual_original_no_tick_actor_ack(tmp_path, field, value):
    values = fixture(tmp_path)
    authenticate(values); native_guard(values[0]); claim(values)
    receipt = actual_cleanup_ack(values); receipt[field] = value
    with pytest.raises(Exception, match="no-tick owned"): guard.complete_goal_cleanup_receipt(values[0], values[1], receipt)
    assert getattr(values[0], "_goal_cleanup_closed", False) is False


def test_copied_receipt_control_tampering_or_late_latch_cannot_complete(tmp_path):
    values = fixture(tmp_path)
    authenticate(values); native_guard(values[0]); claim(values)
    receipt = actual_cleanup_ack(values)
    with pytest.raises(Exception, match="no-tick owned"):
        guard.complete_goal_cleanup_receipt(values[0], values[1], dict(receipt))
    receipt["requested_control"]["steer"] = .01
    with pytest.raises(Exception, match="no-tick owned"):
        guard.complete_goal_cleanup_receipt(values[0], values[1], receipt)
    receipt["requested_control"]["steer"] = 0.
    values[0].latched_emergency = "late collision event"
    with pytest.raises(Exception, match="emergency latch"):
        guard.complete_goal_cleanup_receipt(values[0], values[1], receipt)
