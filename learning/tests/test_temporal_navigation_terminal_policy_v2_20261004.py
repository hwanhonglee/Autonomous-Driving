"""HH_261004 - Check actual-policy bridge contracts with clearly synthetic CPU current-pose fixtures."""

from dataclasses import replace
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.path_time_decoder_v2 import GO
from portable_e2e.temporal_navigation_terminal_policy_v2_20261004 import ActualNavigationTerminalDecoderBridge, complete_route_geometry
from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("frozen_counterfactual_current_pose_fixture_helpers", ROOT / "tools/e2e/benchmark_navigation_terminal_retiming_v1_20261004.py")
fixture_helpers = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fixture_helpers)


def bridge_fixture(support=10., stamp=1_000_000_000, speed=2.4):
    payload = fixture_helpers.read_canonical_regular(ROOT / fixture_helpers.ROUTE_REL)
    geometry = fixture_helpers.route_geometry(json.loads(payload))
    case = dict(source_index=0, actual_state_sha256="a" * 64, actual_state=dict(z=.1), stamp_ns=stamp,
        original_camera_sha256=[str(i) * 64 for i in range(1, 7)], record=dict(inference_frame=1,
        owned_camera_history=dict(normalized_input_tensor_sha256=dict(ego_history="b" * 64))))
    _, _, op, _, _ = fixture_helpers.counterfactual_payloads(case, support, geometry)
    obs = json.loads(op)
    frame = SimpleNamespace(timestamp_ns=stamp, pose_timestamp_ns=stamp,
        position_map_m=tuple(obs["position_map_m"]), orientation_xyzw=tuple(obs["orientation_xyzw"]),
        episode_id="SYNTHETIC_CPU_NOT_ACTUAL_CAPTURE", inputs=SimpleNamespace(
        route_xy_base_m=tuple(tuple(p) for p in obs["provided_route_xy_base_m"]),
        camera_frames=[SimpleNamespace(timestamp_ns=stamp) for _ in range(6)]))
    current = SimpleNamespace(timestamp_ns=stamp, camera_sha256=tuple(case["original_camera_sha256"]))
    window = SimpleNamespace(frames=(None, None, None, None, current))
    calls = []
    def ordinary(*args):
        calls.append(args)
        return {"SYNTHETIC_ORDINARY_OBJECT_IDENTITY": object()}
    policy = SimpleNamespace(decoder=ordinary, _last_owned_input_audit=dict(frame_timestamp_ns=stamp,
        every_source_camera_SHA_preserved=True, source_pixels_changed=False,
        normalized_input_tensor_sha256=dict(ego_history="b" * 64)))
    def actual_binding_faked_only_in_explicit_synthetic_test(frame, tensors, window):
        return (dict(synthetic_CPU_fixture=True, actual_observed_native_pose=True, counterfactual_not_observed=False,
            position_map_m=frame.position_map_m, orientation_xyzw=frame.orientation_xyzw, timestamp_ns=frame.pose_timestamp_ns),
            dict(synthetic_CPU_fixture=True, actual_observed_native_motion=True, timestamp_ns=stamp))
    bridge = ActualNavigationTerminalDecoderBridge(policy, payload, actual_binding_faked_only_in_explicit_synthetic_test)
    bridge.bind_current(frame, {}, window, now_ns=stamp, ego_stamp_ns=stamp, route_stamp_ns=stamp)
    args = [torch.zeros((1, 33)), torch.full((1, 33), 2.4), torch.tensor([speed]), torch.tensor([GO]),
            torch.zeros(1), torch.tensor([support])]
    return bridge, frame, policy, args, calls


def test_ordinary_delegation_preserves_original_argument_objects_and_never_calls_binding():
    bridge, _, _, args, calls = bridge_fixture(support=18.)
    bridge.source_binding = lambda *a: pytest.fail("ordinary branch must not synthesize or authenticate terminal geometry")
    result = bridge(*args)
    assert len(calls) == 1 and all(a is b for a, b in zip(args, calls[0]))
    assert result.keys() == {"SYNTHETIC_ORDINARY_OBJECT_IDENTITY"}
    assert bridge.last_receipt["branch"] == "ordinary_exact_delegate"
    assert bridge.previous_witness is None


@pytest.mark.parametrize("support", [17.9, 10., 5., 2.5])
def test_short_real_geometry_synthetic_pose_uses_authentic_arc_and_original_gates(support):
    bridge, _, _, args, calls = bridge_fixture(support=support)
    result = bridge(*args)
    assert calls == []
    receipt = bridge.last_receipt
    assert receipt["branch"] == "authenticated_navigation_terminal_V2"
    assert receipt["model_weights_changed"] is receipt["learned_STOP_claim"] is False
    assert receipt["raw_semantic_heads_used"] is receipt["actual_observed_speed_clipped"] is False
    assert receipt["navigation_owned_D_m"] == min(receipt["actual_provided_arc_m"], receipt["actual_remaining_mission_arc_m"], 64.)
    assert result["instantaneous_speed"][0, 0].item() == args[2].item()
    validate_and_select_trajectory(result["xy"][0].tolist(), result["speed"][0].tolist(), [0.],
        RuntimeGateConfig(candidate_count=1), current_speed_mps=args[2].item())


def test_unknown_or_counterfactual_pose_binding_cannot_authorize_actual_terminal_branch():
    bridge, frame, _, args, _ = bridge_fixture()
    bridge.source_binding = lambda *a: (dict(actual_observed_native_pose=False), dict(actual_observed_native_motion=False))
    with pytest.raises(ContractError, match="actual native pose"):
        bridge(*args)


def test_stale_current_time_and_mutated_NN_route_reject_without_ordinary_fallback():
    bridge, frame, _, args, calls = bridge_fixture()
    bridge.bind_current(frame, {}, bridge.current[2], now_ns=frame.timestamp_ns + 150_000_001,
                        ego_stamp_ns=frame.timestamp_ns, route_stamp_ns=frame.timestamp_ns)
    with pytest.raises(ContractError, match="STALE_OR_FUTURE_INPUT"):
        bridge(*args)
    assert calls == []
    bridge, frame, _, args, calls = bridge_fixture()
    frame.inputs.route_xy_base_m = ((0., 0.), (64., 0.))
    with pytest.raises(ContractError, match="NN navigation input"):
        bridge(*args)
    assert calls == []


def test_missing_owned_camera_or_wrong_pose_geometry_is_denied():
    bridge, frame, policy, args, _ = bridge_fixture()
    policy._last_owned_input_audit["frame_timestamp_ns"] += 1
    with pytest.raises(ContractError, match="owned-camera/window"):
        bridge(*args)
    bridge, frame, _, args, _ = bridge_fixture()
    original = bridge.source_binding
    def wrong(*a):
        pose, motion = original(*a)
        pose["position_map_m"] = [0., 0., 0.]
        return pose, motion
    bridge.source_binding = wrong
    with pytest.raises(ContractError, match="actual native pose"):
        bridge(*args)


def test_duplicate_terminal_frame_sequence_is_rejected_not_reset_to_sequence_zero():
    bridge, _, _, args, _ = bridge_fixture()
    bridge(*args)
    with pytest.raises(ContractError, match="NONMONOTONIC_OBSERVATION"):
        bridge(*args)


def test_complete_route_sha_and_identity_are_fail_closed():
    payload = fixture_helpers.read_canonical_regular(ROOT / fixture_helpers.ROUTE_REL)
    with pytest.raises(ContractError, match="catalog route bytes"):
        complete_route_geometry(payload + b" ")
    route, points, arc = complete_route_geometry(payload)
    assert len(route["route"]) == 211 and len(points) < 211
    assert arc == pytest.approx(209.3699998276922, abs=1.e-9)
