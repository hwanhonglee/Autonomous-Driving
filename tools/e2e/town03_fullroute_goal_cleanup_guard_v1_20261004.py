"""HH_261004 - Authenticate genuine catalog arrival without changing ordinary control or hazard decisions."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
from types import SimpleNamespace

from . import collect_carla_vad_expert as collector
from . import collect_temporal_v2_simulation_research_20261001 as frozen


SCHEMA = "portable_e2e.town03_fullroute_goal_cleanup_guard.20261004.v1"
GOAL_REASON = "catalog_goal_terminal_cleanup"
COLLECTOR_SHA = "a41eac9019c644fec03396a50645fa021b63230a4ca86a664e8044dfd170ae08"
TEMPORAL_COLLECTOR_SHA = "e1978a21f77b5f4c7f9bdcadfbaf8157ab1baad07c88cc83c1aecef6ea8356a0"


def _require(condition, message):
    if not condition:
        raise collector.CollectionError(message)


def verify_frozen():
    """HH_261004 - The new helper must use the original goal, control and native-guard dependencies."""
    for module, expected in ((collector, COLLECTOR_SHA), (frozen, TEMPORAL_COLLECTOR_SHA)):
        _require(hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest() == expected,
                 "frozen goal-cleanup dependency changed")
    _require(frozen.DISK_FLOOR_BYTES == 5 * 1024**3 and frozen.CLOSING_RESERVE_BYTES == 16 * 1024**2,
             "original native disk floor or closing reserve changed")


def choose_goal_control_reason(alignment, phase, stop_reason, goal_status):
    """HH_261004 - Preserve alignment-abort priority and all ordinary reason strings; selection is not permission."""
    if alignment is not None and alignment.get("status") != "PASS":
        return "alignment_failure_abort"
    if phase == "driving" and stop_reason == "catalog_goal_tolerance" \
            and getattr(goal_status, "reached", False) is True:
        return GOAL_REASON
    return "next_" + phase + "_control"


def _actual_state(session):
    _require(isinstance(session.states, list) and session.states and isinstance(session.states[-1], dict),
             "actual current native state required")
    return session.states[-1]


def _state_binding(state):
    """HH_261004 - Bind observed inputs only; later next-command annotations are not physical state."""
    names = ("frame", "timestamp", "capture_phase", "x", "y", "z", "yaw", "route_progress_m",
             "world_velocity_carla", "current_control")
    value = {name: state[name] for name in names}
    value["prior_control_alignment"] = dict(state["control_transport"])
    # HH_261004 - Generic capture later adds the outgoing sequence; that is not the prior ACK effect.
    value["prior_control_alignment"].pop("next_command_receipt_sequence", None)
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _blank_brake(transport, teacher):
    expected = collector.AcknowledgedControlTransport.checked_control(collector._blank_brake_control(transport.carla))
    actual = collector.AcknowledgedControlTransport.checked_control(teacher)
    _require(actual == expected, "goal cleanup requires the unchanged original blank brake")
    return expected


def _actor_id(transport):
    actor_id = transport.ego.id
    _require(type(actor_id) is int and actor_id > 0 and transport.ego.is_alive is True,
             "live exact owned actor required for goal cleanup")
    return actor_id


def _native_frame(transport, frame):
    actual = transport.frame_reader()
    _require(type(frame) is int and frame >= 0 and type(actual) is int and actual == frame,
             "goal cleanup native source frame changed")


def authenticate_goal_cleanup(session, transport, teacher, expected_before_frame,
                              goal_status, event_snapshot, source_check):
    """HH_261004 - Create one pending scalar receipt from actual catalog/pose/prior-NN ACK provenance.

    Integration must preserve the original XY ``session.route`` and separately retain
    the original catalog payload as ``session.goal_cleanup_catalog_route``.  The
    supplied source check must verify that payload's frozen asset and exact live
    owner/source/PID generations.  It may return None (the original guard ABI) or
    True, but not False or an arbitrary truthy value.
    """
    verify_frozen()
    _require(getattr(session, "_goal_cleanup_pending", None) is None
             and getattr(session, "_goal_cleanup_closed", False) is False,
             "goal cleanup receipt is create-only and cannot repeat")
    _require(callable(source_check), "actual owner/source/PID check must be callable")
    state = _actual_state(session)
    frame = state["frame"]
    _require(all(type(state.get(name)) in (int, float) and math.isfinite(state[name])
                 for name in ("timestamp", "x", "y", "z", "yaw", "route_progress_m")),
             "finite nonboolean actual goal state required")
    _require(state["timestamp"] >= 0, "actual goal simulation timestamp must be nonnegative")
    _require(state["capture_phase"] == "driving" and type(expected_before_frame) is int
             and expected_before_frame == frame, "goal cleanup requires the current driving frame")
    _native_frame(transport, frame)
    actor_id = _actor_id(transport)
    checked_source = source_check()
    _require(checked_source is None or checked_source is True, "actual owner/source/PID check rejected goal cleanup")
    _native_frame(transport, frame)
    _require(session.handed_off is True and session.latched_emergency is None,
             "goal cleanup cannot bypass startup or an existing emergency latch")
    _require(type(event_snapshot) is tuple and len(event_snapshot) == 2
             and all(type(group) is dict and not group for group in event_snapshot),
             "actual full event-recorder snapshot must contain no observed collision or lane event")
    alignment = state.get("control_transport", {})
    _require(alignment.get("status") == "PASS" and alignment.get("frame_binding_pass") is True
             and alignment.get("observed_frame") == frame
             and alignment.get("control_read_frame_before") == alignment.get("control_read_frame_after") == frame
             and alignment.get("mismatched_fields") == [], "actual current control/frame alignment must pass")
    tolerance = session.args.goal_tolerance_m
    _require(type(tolerance) in (int, float) and tolerance == 2.5,
             "original catalog goal tolerance must remain exactly 2.5m")
    route = getattr(session, "goal_cleanup_catalog_route", None)
    _require(isinstance(route, dict) and route.get("town") == "Town03" and route.get("scenario") == "straight",
             "source-checked original Town03 straight catalog payload required")
    projector = collector.RouteProjector(route)
    _require(tuple((float(point["x"]), float(point["y"])) for point in projector.points) == session.route,
             "original catalog geometry differs from the actual navigation route")
    final = projector.points[-1]
    recomputed = collector.catalog_goal_status(state["route_progress_m"], projector.length_m,
        state["x"], state["y"], float(final["x"]), float(final["y"]), tolerance)
    _require(type(goal_status) is collector.CatalogGoalStatus and goal_status == recomputed
             and recomputed.reached is True, "both original catalog goal predicates must genuinely pass")
    sequence = alignment.get("expected_receipt_sequence")
    _require(type(sequence) is int and sequence > 0 and transport.receipts,
             "actual previous ACK receipt required")
    prior = transport.receipts[-1]
    _require(prior.get("sequence") == sequence and prior.get("status") == "ACKNOWLEDGED"
             and prior.get("server_accepted") is True and prior.get("actor_id") == actor_id
             and prior.get("response_actor_id") == actor_id and prior.get("mode") == "acknowledged_batch"
             and prior.get("do_tick") is False and prior.get("after_ack_frame") == frame - 1
             and prior.get("before_frame") == frame - 1,
             "previous accepted owned ACK must produce exactly the current native interval")
    # HH_261004 - Reuse the original pure comparison math on a private counter, not on the actual transport.
    comparison_receiver = SimpleNamespace(alignment_failures=0,
        checked_control=collector.AcknowledgedControlTransport.checked_control)
    compared = collector.AcknowledgedControlTransport.compare_observation(comparison_receiver,
        SimpleNamespace(**state["current_control"]), prior, frame,
        alignment["control_read_frame_before"], alignment["control_read_frame_after"])
    _require(compared["status"] == "PASS" and compared["expected_receipt_sequence"] == sequence,
             "actual current control differs from its original prior ACK request")
    timings = [row for row in session.wall_timing_records if row.get("ACK_sequence") == sequence]
    _require(len(timings) == 1, "unique actual prior learned ACK wall receipt required")
    timing = timings[0]
    _require(timing.get("ACK_server_accepted") is True and timing.get("learned_control_proposed") is True
             and timing.get("control_source") == "learned_trajectory_tracker"
             and timing.get("command_frame") == frame - 1
             and timing.get("source_checkpoint_sha256") == session.checkpoint_sha256
             and timing.get("stream_source_sha256") == session.source_sha256,
             "prior ACK must be an actual source-bound learned-control effect")
    requested = _blank_brake(transport, teacher)
    _native_frame(transport, frame)
    pending = dict(schema=SCHEMA, status="PREPARED", command_reason=GOAL_REASON,
        command_frame=frame, source_timestamp_simulation_seconds=state["timestamp"], actor_id=actor_id,
        transport_object_id=id(transport), prior_learned_ACK_sequence=sequence,
        prior_learned_ACK_after_frame=prior["after_ack_frame"], original_goal_tolerance_m=tolerance,
        original_catalog_length_m=projector.length_m, remaining_catalog_route_m=recomputed.remaining_route_m,
        actual_planar_catalog_goal_distance_m=recomputed.planar_distance_m,
        actual_state_binding_sha256=_state_binding(state), requested_original_cleanup_control=requested,
        original_catalog_two_predicates_passed=True, actual_prior_learned_ACK_effect_proven=True,
        owner_source_PID_check_passed=True, observed_event_snapshot_empty=True,
        existing_emergency_latch_clear=True, current_tick_hazard_detector_clear_proven=False,
        learned_control_proposed=False, learned_STOP_proven=False, physical_cleanup_actuation_proven=False,
        native_guard_attempted=False, native_guard_passed=False, cleanup_ACK_received=False, vehicle_control_approved=False)
    session._goal_cleanup_pending = pending
    return copy.deepcopy(pending)


def run_original_native_guard(session):
    """HH_261004 - Run the original actor-speed/disk predicate once for the newly named cleanup reason only."""
    verify_frozen()
    state = _actual_state(session)
    pending = getattr(session, "_goal_cleanup_pending", None)
    _require(isinstance(pending, dict) and pending["status"] == "PREPARED"
             and pending["native_guard_attempted"] is False and pending["native_guard_passed"] is False
             and pending["actual_state_binding_sha256"] == _state_binding(state),
             "one unchanged authenticated goal state required before its native guard")
    pending["native_guard_attempted"] = True
    # HH_261004 - This is the exact original ResearchTransport.send native-guard arithmetic and journal ABI.
    velocity = state["world_velocity_carla"]
    speed = math.hypot(float(velocity[0]), float(velocity[1]))
    free = shutil.disk_usage(frozen.ROOT).free
    guard = dict(frame=state["frame"], measured_actor_planar_speed_mps=speed,
        maximum_actor_planar_speed_mps=2.7, free_disk_bytes=free,
        required_free_bytes=frozen.DISK_FLOOR_BYTES + frozen.CLOSING_RESERVE_BYTES)
    with (session.output / "native_guard.jsonl").open("a") as stream:
        stream.write(json.dumps(guard, allow_nan=False) + "\n")
    _require(math.isfinite(speed) and speed <= 2.7
             and free >= frozen.DISK_FLOOR_BYTES + frozen.CLOSING_RESERVE_BYTES,
             "native temporal speed or storage guard exceeded")
    pending["native_guard_passed"] = True
    pending["native_guard"] = dict(guard)
    return guard


def claim_goal_cleanup_send(session, transport, teacher, expected_before_frame):
    """HH_261004 - Consume permission once immediately before the unchanged original no-tick ACK send."""
    pending = getattr(session, "_goal_cleanup_pending", None)
    state = _actual_state(session)
    _require(isinstance(pending, dict) and pending["status"] == "PREPARED"
             and pending["native_guard_passed"] is True,
             "authenticated native-guarded cleanup send may be claimed only once")
    _require(pending["transport_object_id"] == id(transport) and pending["actor_id"] == _actor_id(transport)
             and type(expected_before_frame) is int and expected_before_frame == pending["command_frame"] == state["frame"]
             and pending["actual_state_binding_sha256"] == _state_binding(state)
             and session.latched_emergency is None
             and transport.receipts[-1]["sequence"] == pending["prior_learned_ACK_sequence"],
             "goal cleanup permission became stale or its owner/actor/prior ACK changed")
    _require(_blank_brake(transport, teacher) == pending["requested_original_cleanup_control"],
             "authenticated cleanup control changed")
    _native_frame(transport, expected_before_frame)
    pending["status"] = "SEND_CLAIMED"
    return copy.deepcopy(pending)


def complete_goal_cleanup_receipt(session, transport, receipt):
    """HH_261004 - Verify the actual original transport receipt; ACK never proves unobserved brake actuation."""
    pending = getattr(session, "_goal_cleanup_pending", None)
    _require(isinstance(pending, dict) and pending["status"] == "SEND_CLAIMED"
             and pending["transport_object_id"] == id(transport)
             and pending["actor_id"] == _actor_id(transport), "one claimed owned cleanup receipt required")
    frame, actor = pending["command_frame"], pending["actor_id"]
    _require(transport.receipts and receipt is transport.receipts[-1]
             and receipt.get("sequence") == pending["prior_learned_ACK_sequence"] + 1
             and receipt.get("status") == "ACKNOWLEDGED" and receipt.get("server_accepted") is True
             and receipt.get("reason") == GOAL_REASON and receipt.get("mode") == "acknowledged_batch"
             and receipt.get("do_tick") is False and receipt.get("actor_id") == receipt.get("response_actor_id") == actor
             and receipt.get("expected_before_frame") == receipt.get("before_frame") == receipt.get("after_ack_frame") == frame
             and receipt.get("requested_control") == pending["requested_original_cleanup_control"],
             "actual unchanged no-tick owned cleanup ACK receipt differs")
    _native_frame(transport, frame)
    _require(_state_binding(_actual_state(session)) == pending["actual_state_binding_sha256"]
             and session.latched_emergency is None, "goal state or emergency latch changed during cleanup ACK")
    pending.update(status="ACKNOWLEDGED", cleanup_ACK_received=True, cleanup_ACK_sequence=receipt["sequence"])
    session._goal_cleanup_closed = True
    return copy.deepcopy(pending)
