"""HH_261001 - Owned simulated AH control with retained expert proposals and raw failures."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import torch

from portable_e2e.contract import _canonical_route_in_base, load_contract
from portable_e2e.runtime import RuntimeCameraFrame, RuntimeInputs
from portable_e2e.simulation_research import (AH_CHECKPOINT_SHA256, ControllerConfig,
    SimulationResearchPolicy, application_receipt, load_ah_weights)
from . import collect_carla_vad_expert as collector
from .prepare_carla_common10_dataset import _optical_transform


def state_features(state):
    """HH_261001 - Match the trained ego feature order and the collector's VAD command ABI."""
    command = int(state["command"])
    if command not in range(6):
        raise ValueError("six-way causal command required")
    return (1., float(state["vx"]), float(state["vy"]), float(state["ax"]),
            float(state["ay"]), float(state["yaw_rate"]), float(state["steering_tire_angle_rad"]),
            *(float(command == index) for index in range(6)))


def actor_controller_config(physics, speed_cap):
    """HH_261001 - Use actual front/rear wheel centers and front steering limits."""
    if len(physics.wheels) != 4:
        raise ValueError("four-wheel actor physics required")
    wheels = physics.wheels
    centers = [tuple(sum(getattr(w.position, axis) for w in pair) / 2 for axis in ("x", "y", "z"))
               for pair in (wheels[:2], wheels[2:])]
    wheelbase = math.dist(*centers) / 100.
    steer = math.radians(min(float(w.max_steer_angle) for w in wheels[:2]))
    if not 1.5 <= wheelbase <= 4.5:
        raise ValueError("actual wheelbase outside supported passenger-car geometry")
    result = ControllerConfig(wheelbase, steer, speed_cap_mps=speed_cap)
    result.validate()
    return result


def calibration_from_specs(specs):
    """HH_261001 - Derive optical calibration from the exact spawned six-camera specifications."""
    rows = []
    for index, spec in enumerate(specs):
        if (spec.name != collector.MODEL_CAMERA_ORDER[index] or spec.width != 640
                or spec.height != 360 or spec.enable_postprocess_effects):
            raise ValueError("research cameras must retain the trained source ABI")
        k = spec.intrinsic
        rows.append((k["fx"] / spec.width, k["fy"] / spec.height,
                     k["cx"] / spec.width, k["cy"] / spec.height,
                     *_optical_transform(spec.ros_extrinsic)[:12]))
    return tuple(rows)


class LiveResearchSession:
    """HH_261001 - Hand off once after expert launch; never hide subsequent fallback."""

    def __init__(self, model, args, route, specs, states, output):
        self.model, self.args, self.states = model, args, states
        self.output = output
        self.route = tuple((float(p["x"]), float(p["y"])) for p in route["route"])
        self.calibration = calibration_from_specs(specs)
        self.contract = load_contract()
        self.history = deque(maxlen=10)
        self.bundle = None
        self.bundle_frame = None
        self.policy = None
        self.handed_off = False
        self.latched_emergency = None
        self.last_decision = None
        self.last_inference_frame = None
        self.counts = dict(learned_commands_acknowledged=0, expert_startup_commands=0,
                           emergency_commands=0, inference_count=0)

    def observe_bundle(self, bundle, frame):
        self.bundle, self.bundle_frame = bundle, frame
        if self.states and self.states[-1]["frame"] == frame:
            self.history.append((frame, state_features(self.states[-1]),
                                 int(round(self.states[-1]["timestamp"] * 1e9))))

    def propose(self, transport, teacher, reason):
        """HH_261001 - Preserve owner setup/cleanup braking and never rewrite expert labels as learned."""
        if reason not in ("initial_drive_control", "next_driving_control") or not self.states:
            return teacher, None
        state = self.states[-1]
        if self.policy is None:
            cfg = actor_controller_config(transport.ego.get_physics_control(), self.args.research_speed_cap_mps)
            self.policy = SimulationResearchPolicy(self.model, cfg)
        # HH_261001 - Start learned control only on the exact fresh camera/state frame.
        if (not self.handed_off and state["vx"] > 0.5 and len(self.history) == 10
                and self.bundle_frame == state["frame"]):
            self.handed_off = True
        if self.latched_emergency and not self.handed_off:
            return transport.carla.VehicleControl(brake=1.), dict(
                control_source="emergency_brake", learned_control_proposed=False,
                emergency_reason=self.latched_emergency, source_checkpoint_sha256=AH_CHECKPOINT_SHA256)
        if not self.handed_off:
            self.counts["expert_startup_commands"] += 1
            return teacher, dict(control_source="expert_startup", learned_control_proposed=False,
                                 source_checkpoint_sha256=AH_CHECKPOINT_SHA256)
        if abs(state["route_cte_m"]) > self.args.research_max_cte_m:
            self.latched_emergency = "route cross-track limit exceeded"
        # HH_261001 - A teacher brake is an explicit safety override, never a learned success.
        emergency = self.latched_emergency or ("expert brake override" if teacher.brake > 0.05 else None)
        now_ns = int(round(state["timestamp"] * 1e9))
        if self.bundle is None:
            emergency = "no current camera bundle"
        if self.bundle is not None and (self.last_inference_frame != self.bundle_frame or emergency):
            yaw = state["yaw"]
            local_route, _, _ = _canonical_route_in_base(self.route,
                position_m=(state["x"], state["y"], state["z"]),
                orientation_xyzw=(0., 0., math.sin(yaw / 2), math.cos(yaw / 2)),
                contract=self.contract, context="owned AH simulation")
            frames = tuple(RuntimeCameraFrame(int(round(self.bundle[name].timestamp * 1e9)),
                np.frombuffer(self.bundle[name].raw_data, dtype=np.uint8).reshape(360, 640, 4).copy(),
                "bgra8") for name in collector.MODEL_CAMERA_ORDER)
            history = tuple(row for _, row, _ in self.history)
            inputs = RuntimeInputs(frames, self.calibration, history, (True,) * 10, local_route)
            self.last_decision = self.policy.step(inputs, now_ns=now_ns,
                ego_stamp_ns=self.history[-1][2],
                route_stamp_ns=now_ns, emergency_reason=emergency)
            self.last_inference_frame = self.bundle_frame
            self.counts["inference_count"] += 1
        decision = dict(self.last_decision) if self.last_decision is not None else dict(
            control_source="emergency_brake", learned_control_proposed=False,
            emergency_reason=emergency or "no learned decision",
            control=dict(throttle=0., brake=1., steer=0., reverse=False, hand_brake=False))
        if self.bundle is None or now_ns - int(round(next(iter(self.bundle.values())).timestamp * 1e9)) > 150_000_000:
            decision.update(control_source="emergency_brake", learned_control_proposed=False,
                emergency_reason="camera age exceeded while holding control",
                control=dict(throttle=0., brake=1., steer=0., reverse=False, hand_brake=False))
        decision.update(inference_frame=self.last_inference_frame, command_frame=state["frame"],
                        held_between_camera_frames=self.last_inference_frame != state["frame"])
        command = decision["control"]
        control = transport.carla.VehicleControl(**{k: command[k] for k in
            ("throttle", "brake", "steer", "reverse", "hand_brake")})
        return control, decision


def run(model, args):
    """HH_261001 - Install hooks only in this worker; shared expert collector files are untouched."""
    original_collect = collector.collect_episode
    original_bundle = collector.exact_camera_bundle
    original_transport = collector.AcknowledgedControlTransport
    original_collision = collector.EventRecorder.on_collision
    original_lane = collector.EventRecorder.on_lane_invasion
    session = None

    def observe_bundle(queues, frame, *positional, **kwargs):
        bundle = original_bundle(queues, frame, *positional, **kwargs)
        if session is not None:
            session.observe_bundle(bundle, frame)
        return bundle

    def collision(recorder, event):
        if session is not None:
            session.latched_emergency = "collision event"
        return original_collision(recorder, event)

    def lane(recorder, event):
        if session is not None:
            session.latched_emergency = "lane invasion event"
        return original_lane(recorder, event)

    class ResearchTransport(original_transport):
        def send(self, control, reason, expected_before_frame=None):
            teacher = collector.control_dict(control)
            chosen, decision = session.propose(self, control, reason) if session else (control, None)
            if decision is not None and session.states:
                session.states[-1].update(expert_proposed_next_control=teacher,
                    next_control=collector.control_dict(chosen),
                    research_control_source=decision["control_source"],
                    research_inference_frame=decision.get("inference_frame"))
            receipt = super().send(chosen, reason, expected_before_frame)
            if decision is not None:
                source = decision["control_source"]
                applied_source = "expert" if source == "expert_startup" else source
                ack = application_receipt(decision, acknowledged=receipt["server_accepted"], applied_source=applied_source)
                decision.update(application=ack, ack_receipt_sequence=receipt["sequence"],
                                teacher_proposal=teacher, command_reason=reason)
                if ack["learned_control_applied"]:
                    session.counts["learned_commands_acknowledged"] += 1
                elif source == "emergency_brake":
                    session.counts["emergency_commands"] += 1
                with (session.output / "learned_control.jsonl").open("a") as stream:
                    stream.write(json.dumps(decision, allow_nan=False) + "\n")
            return receipt

    def collect(*positional, **kwargs):
        nonlocal session
        capture_args, route, specs, partial, states, _, manifest = positional
        session = LiveResearchSession(model, args, route, specs, states, partial)
        manifest.update(training_data_approved=False, vehicle_control_approved=False,
                        control_source="simulation_research_AH_with_disclosed_expert_startup_and_emergency")
        manifest.setdefault("capture_contract", {})["learned_research"] = dict(
            schema="portable_e2e.owned_simulation_research.v1", checkpoint_sha256=AH_CHECKPOINT_SHA256,
            automatic_training_admission=False, expert_startup_until_vx_above_mps=0.5,
            repeated_dev_evaluation=False, source_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in (Path(__file__), Path(collector.__file__))})
        try:
            return original_collect(*positional, **kwargs)
        finally:
            schedule = manifest.get("capture_contract", {}).get("capture_phase_schedule", {})
            if "driving" in schedule:
                schedule["driving"]["control_source"] = "AH_research_with_logged_expert_startup_and_emergency"
            manifest.setdefault("result", {})["learned_research_counts"] = dict(session.counts)
            with (partial / "learned_summary.json").open("x") as stream:
                json.dump(dict(schema="portable_e2e.owned_simulation_research.v1",
                    counts=session.counts, handoff_occurred=session.handed_off,
                    latched_emergency=session.latched_emergency,
                    controller=asdict(session.policy.controller) if session.policy else None,
                    vehicle_control_approved=False, training_data_approved=False), stream, indent=2)

    collector.collect_episode, collector.exact_camera_bundle = collect, observe_bundle
    collector.AcknowledgedControlTransport = ResearchTransport
    collector.EventRecorder.on_collision, collector.EventRecorder.on_lane_invasion = collision, lane
    try:
        return collector.run(args)
    finally:
        collector.collect_episode, collector.exact_camera_bundle = original_collect, original_bundle
        collector.AcknowledgedControlTransport = original_transport
        collector.EventRecorder.on_collision, collector.EventRecorder.on_lane_invasion = original_collision, original_lane


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, add_help=False)
    parser.add_argument("--research-weights", type=Path, required=True)
    parser.add_argument("--research-metadata", type=Path, required=True)
    parser.add_argument("--research-weights-sha256", required=True)
    parser.add_argument("--research-metadata-sha256", required=True)
    parser.add_argument("--research-speed-cap-mps", type=float, default=2.5)
    parser.add_argument("--research-max-cte-m", type=float, default=1.)
    research, remaining = parser.parse_known_args(argv)
    args = collector.parse_args(remaining)
    for key, value in vars(research).items():
        setattr(args, key, value)
    if (args.host not in ("localhost", "127.0.0.1") or args.allow_map_load
            or args.control_transport != "acknowledged_batch" or args.goal_stop_profile != "disabled"
            or args.physics_hz != 20 or args.capture_hz != 10 or args.stationary_warmup_sec < 1.2
            or args.max_duration_sec > 30 or not 0 < args.research_speed_cap_mps <= 2.5
            or args.target_speed_kmh > args.research_speed_cap_mps * 3.6
            or not 0 < args.research_max_cte_m <= 1.):
        raise ValueError("owned local research requires ack control, 20/10 Hz, >=1.2 s warmup, <=30 s, <=9 km/h, <=1 m CTE")
    route = json.loads(args.route_file.read_text())
    if route["town"] not in ("Town01", "Town07", "c_track_1_0_7") or route["scenario"] != "straight":
        raise ValueError("initial learned research is limited to TRAIN-town straight routes")
    torch.set_num_threads(2)
    model = load_ah_weights(args.research_weights, args.research_metadata,
        weights_sha256=args.research_weights_sha256, metadata_sha256=args.research_metadata_sha256)
    print(run(model, args))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
