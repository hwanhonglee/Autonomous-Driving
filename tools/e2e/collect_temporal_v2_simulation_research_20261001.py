"""HH_261001 - Connect measured six-camera CARLA observations to isolated temporal V2 cruise control."""

from __future__ import annotations

import argparse
from collections import deque
from contextlib import contextmanager
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import shutil
import time

import numpy as np
import torch

from portable_e2e.contract import ContractError, _canonical_route_in_base
from portable_e2e.runtime import RuntimeCameraFrame, RuntimeInputs
from portable_e2e.simulation_research import application_receipt
from portable_e2e.temporal_simulation_research_v2_20261001 import (
    ObservedTemporalFrameV2, TemporalSimulationResearchPolicyV2, load_npz)
from . import collect_ah_simulation_research as original


collector = original.collector
SCHEMA = "portable_e2e.owned_temporal_v2_cruise.20261001.v1"
DISK_FLOOR_BYTES, CLOSING_RESERVE_BYTES = 5 * 1024**3, 16 * 1024**2
ROOT = Path(__file__).resolve().parents[2]


def _require(condition, message):
    if not condition:
        raise collector.CollectionError(message)


def _write_new(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write("\n")


class TemporalLiveResearchSessionV2(original.LiveResearchSession):
    """HH_261001 - Use observed causal history and disclose expert startup; never fall back after handoff."""

    def __init__(self, model, args, route, specs, states, output):
        super().__init__(model, args, route, specs, states, output)
        self.observed_frames = deque(maxlen=10)
        self.last_prefill_frame = None
        self.counts.update(prefill_count=0)
        self.rig_id = hashlib.sha256(json.dumps(self.calibration, allow_nan=False).encode()).hexdigest()
        self.episode_id = "temporal_v2_" + args.output.parent.name
        self.source_sha256 = args.research_source_sha256
        self.checkpoint_sha256 = model._research_temporal_checkpoint_sha256
        self.bundle_wall_timing = {}
        self.wall_timing_records = []

    def observe_bundle(self, bundle, frame):
        _require(self.states and self.states[-1]["frame"] == frame,
                 "temporal camera bundle must match the actual native state frame")
        state = self.states[-1]
        stamp = int(round(float(state["timestamp"]) * 1.e9))
        _require(set(bundle) == set(collector.MODEL_CAMERA_ORDER) and all(
            int(round(float(bundle[name].timestamp) * 1.e9)) == stamp
            and int(bundle[name].frame) == frame for name in collector.MODEL_CAMERA_ORDER),
            "temporal six-camera timestamps/frames differ from measured native snapshot")
        if self.observed_frames:
            _require(abs(stamp - self.observed_frames[-1].timestamp_ns - 100_000_000) <= 1_000,
                     "temporal observed camera stream is not consecutive 10 Hz")
        super().observe_bundle(bundle, frame)
        yaw = float(state["yaw"])
        quaternion = (0., 0., math.sin(yaw / 2.), math.cos(yaw / 2.))
        position = tuple(float(state[name]) for name in ("x", "y", "z"))
        local_route, _, _ = _canonical_route_in_base(self.route, position_m=position,
            orientation_xyzw=quaternion, contract=self.contract, context="owned temporal V2 causal navigation")
        cameras = tuple(RuntimeCameraFrame(stamp,
            np.frombuffer(bundle[name].raw_data, dtype=np.uint8).reshape(360, 640, 4).copy(), "bgra8")
            for name in collector.MODEL_CAMERA_ORDER)
        real_history = tuple(row for _, row, _ in self.history)
        missing = 10 - len(real_history)
        inputs = RuntimeInputs(cameras, self.calibration, ((0.,) * 13,) * missing + real_history,
                               (False,) * missing + (True,) * len(real_history), local_route)
        self.observed_frames.append(ObservedTemporalFrameV2(stamp, inputs, position, quaternion, stamp,
            self.episode_id, "train", self.rig_id, self.source_sha256))

    def _emergency(self, reason, state):
        self.latched_emergency = reason
        if self.policy is not None:
            self.policy.reset("collector_latched_emergency")
        return dict(schema=SCHEMA, source_checkpoint_sha256=self.checkpoint_sha256,
            simulation_only=True, vehicle_control_approved=False, training_data_approved=False,
            learned_control_proposed=False, learned_control_applied=False, control_source="emergency_brake",
            emergency_reason=reason, inference_frame=self.last_inference_frame, command_frame=state["frame"],
            control=dict(throttle=0., brake=1., steer=0., reverse=False, hand_brake=False))

    def propose(self, transport, teacher, reason):
        # HH_261001 - Original bootstrap/warmup/tail/abort authority is unchanged and is never learned.
        if reason not in ("initial_drive_control", "next_driving_control") or not self.states:
            return teacher, None
        state = self.states[-1]
        now_ns = int(round(float(state["timestamp"]) * 1.e9))
        if self.policy is None:
            config = original.actor_controller_config(transport.ego.get_physics_control(), 2.5)
            self.policy = TemporalSimulationResearchPolicyV2(self.model, config,
                episode_id=self.episode_id, split="train", rig_id=self.rig_id, source_sha256=self.source_sha256,
                deadline_s=.1, longitudinal_pi=self.args.research_longitudinal_pi)
        if abs(state["route_cte_m"]) > self.args.research_max_cte_m:
            self.latched_emergency = "route cross-track limit exceeded"
        if self.handed_off and teacher.brake > .05:
            self.latched_emergency = "expert brake safety override, not learned control"
        current = self.observed_frames[-1] if self.observed_frames else None
        fresh = current is not None and self.bundle_frame == state["frame"]
        if self.latched_emergency:
            decision = self._emergency(self.latched_emergency, state)
        elif not self.handed_off:
            # HH_261001 - The third fresh source frame can hand off; the two actual parity prefills own no actuation.
            can_handoff = (fresh and self.policy._prefill_count >= 2 and self.last_prefill_frame != self.bundle_frame
                           and state["vx"] > .5 and len(self.history) == 10)
            if can_handoff:
                self.handed_off = True
            elif fresh and self.last_prefill_frame != self.bundle_frame:
                try:
                    record = self.policy.prefill(current, past_frames=tuple(self.observed_frames)[:-1],
                        now_ns=now_ns, ego_stamp_ns=self.history[-1][2], route_stamp_ns=current.timestamp_ns)
                    record.update(native_frame=self.bundle_frame, checkpoint_sha256=self.checkpoint_sha256)
                    with (self.output / "temporal_prefill.jsonl").open("a") as stream:
                        stream.write(json.dumps(record, allow_nan=False) + "\n")
                    self.last_prefill_frame = self.bundle_frame
                    self.counts["prefill_count"] += 1
                except (ContractError, ValueError, RuntimeError) as error:
                    self.latched_emergency = "temporal startup prefill rejected: " + str(error)
                    decision = self._emergency(self.latched_emergency, state)
            if not self.handed_off and not self.latched_emergency:
                self.counts["expert_startup_commands"] += 1
                return teacher, dict(schema=SCHEMA, control_source="expert_startup",
                    learned_control_proposed=False, source_checkpoint_sha256=self.checkpoint_sha256,
                    startup_prefill_count=self.policy._prefill_count, candidate_count=1,
                    runtime_intent="explicit_cruise_GO_not_head")
        if self.handed_off and not self.latched_emergency:
            if current is None or not 0 <= now_ns - current.timestamp_ns <= 150_000_000:
                decision = self._emergency("camera age exceeded while holding control", state)
            elif fresh and self.last_inference_frame != self.bundle_frame:
                self.last_decision = self.policy.step(current, past_frames=tuple(self.observed_frames)[:-1],
                    now_ns=now_ns, ego_stamp_ns=self.history[-1][2], route_stamp_ns=current.timestamp_ns)
                self.last_inference_frame = self.bundle_frame
                self.counts["inference_count"] += 1
                if not self.last_decision["learned_control_proposed"]:
                    self.latched_emergency = "temporal learned proposal rejected: " + str(self.last_decision["emergency_reason"])
                decision = dict(self.last_decision)
            elif self.last_decision is not None:
                decision = dict(self.last_decision)
            else:
                decision = self._emergency("no fresh temporal handoff inference", state)
        decision.update(inference_frame=self.last_inference_frame, command_frame=state["frame"],
                        held_between_camera_frames=self.last_inference_frame != state["frame"],
                        candidate_count=1, runtime_intent="explicit_cruise_GO_not_head")
        command = decision["control"]
        chosen = transport.carla.VehicleControl(**{key: command[key] for key in
            ("throttle", "brake", "steer", "reverse", "hand_brake")})
        return chosen, decision


def validate_capture(args, route):
    _require(args.host == "127.0.0.1" and args.port == 2440 and not args.allow_map_load
        and args.control_transport == "acknowledged_batch" and args.goal_stop_profile == "disabled"
        and args.physics_hz == 20 and args.capture_hz == 10 and args.stationary_warmup_sec == 1.2
        and args.max_duration_sec == 20 and args.target_speed_kmh == 9
        and args.weather == "ClearNoon" and args.seed == 0 and args.spawn_z_offset_m == .5
        and args.research_speed_cap_mps == 2.5 and args.research_max_cte_m == 1.
        and route["town"] == "Town07" and route["scenario"] == "straight"
        and isinstance(args.research_source_sha256, str) and len(args.research_source_sha256) == 64
        and all(x in "0123456789abcdef" for x in args.research_source_sha256)
        and type(args.research_longitudinal_pi) is bool,
        "fixed isolated temporal Town07 twenty-second precursor contract differs")


def wall_timing_receipt(session, decision, receipt, *, command_frame, reason,
                        proposal_started_ns, proposal_finished_ns, ack_started_ns, ack_finished_ns):
    """HH_261001 - Keep simulated exposure time distinct from observed monotonic delivery-to-ACK latency."""
    inference_frame = decision.get("inference_frame") if decision is not None else None
    bundle = session.bundle_wall_timing.get(inference_frame)
    if bundle is None and session.bundle_frame is not None:
        bundle = session.bundle_wall_timing.get(session.bundle_frame)
    delivery = bundle["exact_bundle_delivery_monotonic_ns"] if bundle else None
    return dict(schema=SCHEMA + ".wall_timing", clock="time.perf_counter_ns_monotonic",
        source_checkpoint_sha256=session.checkpoint_sha256, stream_source_sha256=session.source_sha256,
        command_frame=command_frame, inference_frame=inference_frame, command_reason=reason,
        held_between_camera_frames=decision.get("held_between_camera_frames") if decision else None,
        learned_control_proposed=bool(decision and decision.get("learned_control_proposed", False)),
        ACK_sequence=receipt["sequence"], ACK_server_accepted=receipt["server_accepted"],
        control_source=decision.get("control_source") if decision else "original_nonlearned_capture_control",
        observed_bundle=bundle, proposal_started_monotonic_ns=proposal_started_ns,
        proposal_finished_monotonic_ns=proposal_finished_ns, ACK_started_monotonic_ns=ack_started_ns,
        ACK_finished_monotonic_ns=ack_finished_ns,
        propose_seconds=(proposal_finished_ns - proposal_started_ns) / 1.e9,
        ACK_send_seconds=(ack_finished_ns - ack_started_ns) / 1.e9,
        exact_bundle_delivery_to_proposal_seconds=(proposal_finished_ns - delivery) / 1.e9 if delivery else None,
        exact_bundle_delivery_to_ACK_seconds=(ack_finished_ns - delivery) / 1.e9 if delivery else None,
        sensor_callback_entry_observed=False, sensor_callback_entry_to_ACK_seconds=None,
        exposure_timestamp_clock="CARLA_simulation_seconds_not_monotonic_wall_time",
        exposure_to_ACK_wall_latency_proven=False, ten_hz_end_to_end_proven=False,
        policy_deadline_seconds=.1, policy_deadline_relaxed=False)


def wall_timing_summary(records):
    """HH_261001 - Report every learned proposal separately from held native ACKs, without FPS certification."""
    fresh = [row for row in records if row["learned_control_proposed"]
             and row["held_between_camera_frames"] is False
             and row["exact_bundle_delivery_to_ACK_seconds"] is not None]
    result = dict(record_count=len(records), fresh_learned_proposal_ACK_count=len(fresh),
        sensor_callback_entry_observed=False, exposure_to_ACK_wall_latency_proven=False,
        ten_hz_end_to_end_proven=False, policy_deadline_relaxed=False)
    for key in ("propose_seconds", "ACK_send_seconds", "exact_bundle_delivery_to_proposal_seconds",
                "exact_bundle_delivery_to_ACK_seconds"):
        values = [row[key] for row in fresh]
        result[key] = dict(count=len(values), p95=float(np.percentile(values, 95)) if values else None,
                           maximum=max(values) if values else None)
    return result


@contextmanager
def activated(model, args):
    """HH_261001 - Scope only this worker's collector hooks and preserve frozen source files."""
    saved = dict(collect_episode=collector.collect_episode, exact_camera_bundle=collector.exact_camera_bundle,
                 AcknowledgedControlTransport=collector.AcknowledgedControlTransport)
    original_collision, original_lane = collector.EventRecorder.on_collision, collector.EventRecorder.on_lane_invasion
    session = None

    def observe(queues, frame, *positional, **kwargs):
        started_ns = time.perf_counter_ns()
        bundle = saved["exact_camera_bundle"](queues, frame, *positional, **kwargs)
        delivery_ns = time.perf_counter_ns()
        if session is not None:
            session.observe_bundle(bundle, frame)
            finished_ns = time.perf_counter_ns()
            session.bundle_wall_timing[frame] = dict(native_frame=frame,
                source_image_timestamp_simulation_ns=session.observed_frames[-1].timestamp_ns,
                exact_bundle_wait_started_monotonic_ns=started_ns,
                exact_bundle_delivery_monotonic_ns=delivery_ns,
                observed_frame_build_finished_monotonic_ns=finished_ns,
                exact_bundle_wait_seconds=(delivery_ns - started_ns) / 1.e9,
                observed_frame_build_seconds=(finished_ns - delivery_ns) / 1.e9,
                sensor_callback_entry_observed=False)
            while len(session.bundle_wall_timing) > 10:
                del session.bundle_wall_timing[next(iter(session.bundle_wall_timing))]
        return bundle

    def collision(recorder, event):
        if session is not None: session.latched_emergency = "collision event"
        return original_collision(recorder, event)

    def lane(recorder, event):
        if session is not None: session.latched_emergency = "lane invasion event"
        return original_lane(recorder, event)

    class ResearchTransport(saved["AcknowledgedControlTransport"]):
        def send(self, control, reason, expected_before_frame=None):
            teacher = collector.control_dict(control)
            # HH_261001 - Keep actor-center safety measurement distinct from the decoder's base-link v0.
            if session is not None and session.states and reason in ("initial_drive_control", "next_driving_control",
                    "stationary_warmup_start", "next_stationary_warmup_control", "next_stationary_tail_control"):
                state = session.states[-1]
                velocity = state["world_velocity_carla"]
                speed = math.hypot(float(velocity[0]), float(velocity[1]))
                free = shutil.disk_usage(ROOT).free
                guard = dict(frame=state["frame"], measured_actor_planar_speed_mps=speed,
                    maximum_actor_planar_speed_mps=2.7, free_disk_bytes=free,
                    required_free_bytes=DISK_FLOOR_BYTES + CLOSING_RESERVE_BYTES)
                with (session.output / "native_guard.jsonl").open("a") as stream:
                    stream.write(json.dumps(guard, allow_nan=False) + "\n")
                _require(math.isfinite(speed) and speed <= 2.7 and free >= DISK_FLOOR_BYTES + CLOSING_RESERVE_BYTES,
                         "native temporal speed or storage guard exceeded")
            proposal_started_ns = time.perf_counter_ns()
            chosen, decision = session.propose(self, control, reason) if session else (control, None)
            proposal_finished_ns = time.perf_counter_ns()
            if decision is not None and session.states:
                session.states[-1].update(expert_proposed_next_control=teacher, next_control=collector.control_dict(chosen),
                    research_control_source=decision["control_source"], research_inference_frame=decision.get("inference_frame"))
            ack_started_ns = time.perf_counter_ns()
            receipt = super().send(chosen, reason, expected_before_frame)
            ack_finished_ns = time.perf_counter_ns()
            if session is not None:
                timing = wall_timing_receipt(session, decision, receipt,
                    command_frame=session.states[-1]["frame"] if session.states else None, reason=reason,
                    proposal_started_ns=proposal_started_ns, proposal_finished_ns=proposal_finished_ns,
                    ack_started_ns=ack_started_ns, ack_finished_ns=ack_finished_ns)
                session.wall_timing_records.append(timing)
                with (session.output / "wall_timing.jsonl").open("a") as stream:
                    stream.write(json.dumps(timing, allow_nan=False) + "\n")
            if decision is not None:
                source = decision["control_source"]
                ack = application_receipt(decision, acknowledged=receipt["server_accepted"],
                    applied_source="expert" if source == "expert_startup" else source)
                decision.update(application=ack, ack_receipt_sequence=receipt["sequence"], teacher_proposal=teacher,
                                command_reason=reason)
                if ack["learned_control_applied"]: session.counts["learned_commands_acknowledged"] += 1
                elif source == "emergency_brake": session.counts["emergency_commands"] += 1
                with (session.output / "learned_control.jsonl").open("a") as stream:
                    stream.write(json.dumps(decision, allow_nan=False) + "\n")
            return receipt

    def collect(*positional, **kwargs):
        nonlocal session
        capture_args, route, specs, partial, states, _, manifest = positional
        validate_capture(capture_args, route)
        session = TemporalLiveResearchSessionV2(model, args, route, specs, states, partial)
        manifest.update(training_data_approved=False, vehicle_control_approved=False,
                        control_source="temporal_v2_cruise_with_disclosed_expert_startup_and_emergency")
        manifest.setdefault("capture_contract", {})["learned_research"] = dict(schema=SCHEMA,
            checkpoint_sha256=session.checkpoint_sha256, candidate_count=1, runtime_intent="explicit_cruise_GO_not_head",
            stop_intent_trained=False, automatic_training_admission=False, expert_startup_until_vx_above_mps=.5,
            two_real_adjacent_startup_prefills_required=True, no_teacher_fallback_after_handoff=True,
            learned_deadline_s=.1, callback_and_ACK_outside_policy_timing=True,
            wall_timing_receipts="observed_exact_bundle_delivery_propose_ACK_monotonic_not_exposure",
            sensor_callback_entry_observed=False, exposure_to_ACK_wall_latency_proven=False,
            stream_identity=dict(episode_id=session.episode_id, split="train", rig_id=session.rig_id,
                                 source_sha256=session.source_sha256))
        try:
            return saved["collect_episode"](*positional, **kwargs)
        finally:
            schedule = manifest.get("capture_contract", {}).get("capture_phase_schedule", {})
            if "driving" in schedule:
                schedule["driving"]["control_source"] = "temporal_v2_with_logged_expert_startup_and_emergency"
            manifest.setdefault("result", {})["learned_research_counts"] = dict(session.counts)
            _write_new(partial / "learned_summary.json", dict(schema=SCHEMA, counts=session.counts,
                source_checkpoint_sha256=session.checkpoint_sha256, candidate_count=1,
                handoff_occurred=session.handed_off, latched_emergency=session.latched_emergency,
                controller=asdict(session.policy.controller) if session.policy else None,
                prefill_records=session.policy.prefill_records if session.policy else [],
                wall_timing=wall_timing_summary(session.wall_timing_records),
                vehicle_control_approved=False, training_data_approved=False))

    collector.collect_episode, collector.exact_camera_bundle = collect, observe
    collector.AcknowledgedControlTransport = ResearchTransport
    collector.EventRecorder.on_collision, collector.EventRecorder.on_lane_invasion = collision, lane
    try:
        yield
    finally:
        for key, value in saved.items(): setattr(collector, key, value)
        collector.EventRecorder.on_collision, collector.EventRecorder.on_lane_invasion = original_collision, original_lane


def run(model, args):
    with activated(model, args):
        return collector.run(args)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, add_help=False, allow_abbrev=False)
    parser.add_argument("--research-weights", type=Path, required=True)
    parser.add_argument("--research-metadata", type=Path, required=True)
    parser.add_argument("--research-weights-sha256", required=True)
    parser.add_argument("--research-metadata-sha256", required=True)
    parser.add_argument("--research-source-sha256", required=True)
    parser.add_argument("--research-longitudinal-pi", action="store_true")
    research, remaining = parser.parse_known_args(argv)
    args = collector.parse_args(remaining)
    for key, value in vars(research).items(): setattr(args, key, value)
    args.research_speed_cap_mps, args.research_max_cte_m = 2.5, 1.
    return args


def main(argv=None):
    args = parse_args(argv)
    validate_capture(args, collector.load_route(args.route_file))
    torch.set_num_threads(1)
    model = load_npz(args.research_weights, args.research_metadata,
        weights_sha256=args.research_weights_sha256, metadata_sha256=args.research_metadata_sha256)
    print(run(model, args))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
