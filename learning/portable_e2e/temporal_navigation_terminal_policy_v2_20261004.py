"""HH_261004 - Bind prepared terminal V2 to authenticated actual current research inputs.

This new intervention keeps the frozen policy step, NN, ordinary decoder,
physical gates, tracker and PI unchanged.  Only the below18m decoder branch
receives a COMPLETE-mission/current-pose witness and navigation-owned actual
float64 arc.  Proof and decoding execute inside the inherited100ms step timer.
This is navigation/controller research, not learned STOP or vehicle approval.
"""

from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import math

import torch

from .contract import ContractError, _canonical_route_in_base
from .navigation_terminal_retiming_v2_20261004 import NavigationTerminalRetimingV2
from . import terminal_witness_v1_20261002 as proof


SCHEMA = "portable_e2e.actual_temporal_navigation_terminal_research.20261004.v1"
ROUTE_SHA256 = "8fc442ed6cc6f6e40a4f346c2cfbda26105bd5a53c0992f7c6eb78964838cf06"
CANONICAL_NAVIGATION = dict(navigation=dict(maximum_anchor_distance_m=5., route_resample_spacing_m=1., route_lookahead_m=120.))


def require(condition, message):
    if not condition:
        raise ContractError(message)


def encoded(document):
    payload = json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return payload, hashlib.sha256(payload).hexdigest()


def complete_route_geometry(route_payload, expected_sha256=ROUTE_SHA256):
    # HH_261004 - Load the entire authentic catalog; only literal consecutive geometry duplicates are removed.
    require(type(route_payload) is bytes and hashlib.sha256(route_payload).hexdigest() == expected_sha256,
            "original COMPLETE catalog route bytes differ")
    document = json.loads(route_payload)
    require(document.get("town") == "Town03" and document.get("scenario") == "straight"
            and len(document.get("route", [])) == 211 and document.get("route_length_m") == 209.370221122845,
            "original Town03 p07 full catalog identity differs")
    points = []
    for row in document["route"]:
        point = (float(row["x"]), float(row["y"]))
        require(all(math.isfinite(v) for v in point), "nonfinite actual mission geometry")
        if not points or math.dist(points[-1], point) > 1.e-9:
            points.append(point)
    total = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    require(len(points) >= 2 and total > 0, "actual COMPLETE mission has no genuine arc")
    return document, tuple(points), total


class ActualNavigationTerminalDecoderBridge:
    """HH_261004 - Dispatch only authenticated current geometry; ordinary calls preserve exact inputs/outputs."""

    def __init__(self, policy, route_payload, source_binding):
        require(callable(source_binding), "actual owner/native pose binding callback required")
        self.policy, self.source_binding = policy, source_binding
        self.route, self.points, self.total = complete_route_geometry(route_payload)
        self.ordinary_decoder = policy.decoder
        self.terminal_decoder = NavigationTerminalRetimingV2()
        self.current = None
        self.previous_witness = None
        self.mission_payload = self.mission_sha256 = None
        self.last_receipt = None

    def bind_current(self, frame, tensors, window, *, now_ns, ego_stamp_ns, route_stamp_ns):
        # HH_261004 - Keep the existing actual frame/tensors/window; never reconstruct images, infer future states or alter NN inputs.
        self.current = frame, tensors, window, now_ns, ego_stamp_ns, route_stamp_ns
        self.last_receipt = None

    def __call__(self, curvature, preferred, current, intent, stop, supported):
        require(self.current is not None, "decoder lacks this step's actual input binding")
        frame, tensors, window, now, ego_stamp, route_stamp = self.current
        require(supported.shape == (1,) and supported.device.type == "cpu"
                and bool(torch.isfinite(supported).all()), "actual inherited support ABI differs")
        if float(supported[0]) >= 18.:
            self.last_receipt = dict(schema=SCHEMA, branch="ordinary_exact_delegate", frame_timestamp_ns=frame.timestamp_ns,
                inherited_tensor_support_m=float(supported[0]), terminal_controller_intervention=False,
                learned_STOP_claim=False, original_NN_fields_changed=False, vehicle_control_approved=False)
            return self.ordinary_decoder(curvature, preferred, current, intent, stop, supported)

        # HH_261004 - Proof work is invoked only at actual short support, within the inherited hard100ms policy timer.
        audit = self.policy._last_owned_input_audit
        require(isinstance(audit, dict) and audit["frame_timestamp_ns"] == frame.timestamp_ns
                and audit["every_source_camera_SHA_preserved"] is True
                and audit["source_pixels_changed"] is False and window.frames[-1] is not None
                and window.frames[-1].timestamp_ns == frame.timestamp_ns,
                "current actual owned-camera/window source proof is missing")
        pose_provenance, motion_provenance = self.source_binding(frame, tensors, window)
        require(pose_provenance.get("actual_observed_native_pose") is True
                and motion_provenance.get("actual_observed_native_motion") is True
                and pose_provenance.get("counterfactual_not_observed") is False
                and tuple(pose_provenance["position_map_m"]) == frame.position_map_m
                and tuple(pose_provenance["orientation_xyzw"]) == frame.orientation_xyzw
                and pose_provenance["timestamp_ns"] == frame.pose_timestamp_ns
                and motion_provenance["timestamp_ns"] == ego_stamp,
                "actual native pose/motion provenance differs from this causal input")
        _, pose_sha = encoded(pose_provenance)
        _, motion_sha = encoded(motion_provenance)
        provided, goal, anchor = _canonical_route_in_base(self.points, position_m=frame.position_map_m,
            orientation_xyzw=frame.orientation_xyzw, contract=CANONICAL_NAVIGATION,
            context="actual owned Town03 COMPLETE mission terminal navigation")
        require(tuple(frame.inputs.route_xy_base_m) == tuple(provided),
                "terminal geometry differs from the actual original NN navigation input")
        provided_arc = sum(math.dist(a, b) for a, b in zip(provided, provided[1:]))
        if self.mission_payload is None:
            mission = dict(schema=proof.MISSION_SCHEMA, coordinates=dict(proof.COORDINATES),
                mission_id=frame.episode_id + ":original_town03_p07_COMPLETE", map_id="Town03", route_id="town03_straight_s0000_p07",
                source_route_artifact_sha256=ROUTE_SHA256, source_available_timestamp_ns=min(frame.timestamp_ns, ego_stamp, route_stamp),
                complete_mission_route=True, polyline_map_xy_m=self.points, goal_map_xy_m=self.points[-1], stop_reference_offset_m=0.)
            self.mission_payload, self.mission_sha256 = encoded(mission)
        mission = json.loads(self.mission_payload)
        observation = dict(schema=proof.OBSERVATION_SCHEMA, coordinates=dict(proof.COORDINATES),
            mission_id=mission["mission_id"], map_id=mission["map_id"], route_id=mission["route_id"], source_route_artifact_sha256=ROUTE_SHA256,
            sequence_index=0 if self.previous_witness is None else self.previous_witness.sequence_index + 1,
            anchor_timestamp_ns=frame.timestamp_ns, ego_timestamp_ns=ego_stamp, pose_timestamp_ns=frame.pose_timestamp_ns,
            navigation_timestamp_ns=route_stamp, camera_timestamp_ns=[camera.timestamp_ns for camera in frame.inputs.camera_frames],
            camera_source_sha256=list(window.frames[-1].camera_sha256),
            ego_source_sha256=audit["normalized_input_tensor_sha256"]["ego_history"], pose_source_sha256=pose_sha,
            position_map_m=frame.position_map_m, orientation_xyzw=frame.orientation_xyzw,
            provided_route_xy_base_m=provided, provided_route_support_m=provided_arc, route_anchor_arc_m=anchor,
            remaining_mission_arc_m=self.total - anchor, goal_base_m=goal)
        payload, observation_sha = encoded(observation)
        witness = proof.evaluate_terminal_witness(mission_payload=self.mission_payload,
            expected_mission_sha256=self.mission_sha256, expected_route_artifact_sha256=ROUTE_SHA256,
            observation_payload=payload, expected_observation_sha256=observation_sha, now_ns=now,
            requested_mode=proof.VERIFIED_TERMINAL, previous=self.previous_witness)
        require(witness.status == "VERIFIED", witness.reason_code + ": " + witness.reason)
        distance = min(witness.witness.provided_route_arc_m, witness.witness.remaining_mission_arc_m, 64.)
        self.last_receipt = dict(schema=SCHEMA, branch="authenticated_navigation_terminal_V2", frame_timestamp_ns=frame.timestamp_ns,
            inherited_tensor_support_m=float(supported[0]), actual_provided_arc_m=witness.witness.provided_route_arc_m,
            actual_remaining_mission_arc_m=witness.witness.remaining_mission_arc_m, navigation_owned_D_m=distance,
            mission_document=mission, mission_sha256=self.mission_sha256, observation_document=observation,
            observation_sha256=observation_sha, actual_pose_provenance=pose_provenance, actual_motion_provenance=motion_provenance,
            actual_motion_provenance_sha256=motion_sha, witness_decision=asdict(witness),
            witness_and_decoder_inside_original_policy_deadline=True, model_weights_changed=False,
            terminal_controller_intervention=True, learned_STOP_claim=False, original_NN_fields_changed=False,
            raw_semantic_heads_used=False, actual_observed_speed_clipped=False, actual_support_extrapolated=False,
            vehicle_control_approved=False, training_data_approved=False)
        result = self.terminal_decoder(curvature, preferred, current, intent, stop,
            torch.tensor([distance], dtype=torch.float64), terminal_witness=witness,
            terminal_anchor_timestamp_ns=frame.timestamp_ns, terminal_now_ns=now,
            terminal_observation_sha256=observation_sha, terminal_mission_sha256=self.mission_sha256,
            terminal_route_artifact_sha256=ROUTE_SHA256)
        self.previous_witness = witness.witness
        self.last_receipt["physical_navigation_requests"] = {name: value.tolist() for name, value in result.items() if name.startswith("navigation_")}
        self.last_receipt["terminal_arrival_time_s"] = result["terminal_arrival_time_s"].tolist()
        self.last_receipt["terminal_stop_complete_within_horizon"] = result["terminal_stop_complete_within_horizon"].tolist()
        return result


def navigation_terminal_policy_class(base_policy, route_payload, source_binding):
    """HH_261004 - Create a new MRO wrapper; inherited step and causal input math remain unchanged."""
    class ActualNavigationTerminalPolicy(base_policy):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            require(self.identity[1] == "val", "full-route terminal research is known-DEV val only, never TRAIN")
            self.navigation_terminal_bridge = ActualNavigationTerminalDecoderBridge(self, route_payload, source_binding)
            self.decoder = self.navigation_terminal_bridge

        def build_inputs(self, frame, *, past_frames=(), **kwargs):
            tensors, window = super().build_inputs(frame, past_frames=past_frames, **kwargs)
            self.navigation_terminal_bridge.bind_current(frame, tensors, window, **kwargs)
            return tensors, window

        def step(self, *args, **kwargs):
            self.navigation_terminal_bridge.last_receipt = None
            result = super().step(*args, **kwargs)
            # HH_261004 - Receipt attachment parallels the inherited owned-input audit; all actual proof/control work was already timed.
            result["navigation_terminal_research"] = self.navigation_terminal_bridge.last_receipt
            result["navigation_terminal_controller_intervention_declared"] = True
            result["learned_STOP_claim"] = False
            return result

    return ActualNavigationTerminalPolicy
