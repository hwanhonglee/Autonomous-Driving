"""HH_261001 - Verify one brake-only intervention and preserved stopped/emergency decisions."""

from copy import deepcopy
from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import town07_moving_brake_ab_control_v2_20261001 as candidate
from portable_e2e import simulation_research as research
from portable_e2e.runtime_contract import SelectedTrajectory


def proposal(desired=1.4, brake=.023):
    return dict(throttle=0., brake=brake, steer=.04, reverse=False, hand_brake=False,
                desired_speed_mps=desired, pursuit_xy_base_m=(1.8, -.04),
                steering_convention="CARLA_right_positive_from_base_link_y_left")


@pytest.mark.parametrize("arm", candidate.ARMS)
def test_original_moving_proposal_is_never_mutated(arm):
    original = proposal()
    before = deepcopy(original)
    result = candidate.normal_tracker_control(original, arm)
    assert original == before and result is not original
    assert set(result) == set(before)
    assert {key: value for key, value in result.items() if key != "brake"} == {
        key: value for key, value in before.items() if key != "brake"}
    assert result["brake"] == (before["brake"] if arm == candidate.ARMS[0] else 0.)


@pytest.mark.parametrize("speed", [0., .01, .05])
@pytest.mark.parametrize("brake", [.3, .7, 1.])
def test_original_stop_branch_is_exactly_preserved(speed, brake):
    original = proposal(speed, brake)
    assert candidate.normal_tracker_control(original, candidate.ARMS[1]) == original


def test_strict_threshold_and_no_throttle_floor():
    original = proposal(.0500000001, .01)
    result = candidate.normal_tracker_control(original, candidate.ARMS[1])
    assert result["brake"] == 0. and result["throttle"] == 0.
    assert candidate.intervention_audit(original, result, candidate.ARMS[1])[
        "normal_brake_suppression_proposed"] is True
    zero_brake = proposal(brake=0.)
    assert candidate.normal_tracker_control(zero_brake, candidate.ARMS[1]) == zero_brake
    assert candidate.intervention_audit(zero_brake, zero_brake, candidate.ARMS[1])[
        "normal_brake_suppression_proposed"] is False


@pytest.mark.parametrize("field,value", [("desired_speed_mps", float("nan")),
    ("desired_speed_mps", float("inf")), ("desired_speed_mps", -.1),
    ("desired_speed_mps", 2.500001), ("desired_speed_mps", True),
    ("brake", -.01), ("brake", 1.01), ("brake", float("nan")), ("brake", False)])
def test_malformed_original_normal_proposal_is_rejected(field, value):
    original = proposal()
    original[field] = value
    with pytest.raises(ValueError):
        candidate.normal_tracker_control(original, candidate.ARMS[1])


def test_no_other_control_change_passes_audit():
    original = proposal()
    result = candidate.normal_tracker_control(original, candidate.ARMS[1])
    result["throttle"] = .01
    with pytest.raises(ValueError, match="another tracker field"):
        candidate.intervention_audit(original, result, candidate.ARMS[1])
    for bad in ["B", None, "feedforward"]:
        with pytest.raises(ValueError):
            candidate.normal_tracker_control(original, bad)


def fake_research():
    calls = []
    raw = dict(candidate_xy=[[[.1, 0.]]], candidate_speed=[[1.4]], candidate_logits=[1.])
    module = SimpleNamespace()

    def control(selected, speed, config):
        calls.append((selected, speed, config))
        return proposal(selected)

    class Policy:
        def step(self, desired=1.4, *, emergency=None, deadline_failure=False):
            decision = dict(control_source="emergency_brake", learned_control_proposed=False,
                            emergency_reason=emergency,
                            control=dict(throttle=0., brake=1., steer=0., reverse=False, hand_brake=False),
                            raw_output=raw, total_seconds=.012, selected=dict(candidate_index=5),
                            source_checkpoint_sha256="fixed_W1_checkpoint")
            if emergency:
                return module._json_finite(decision)
            output = module.trajectory_control(desired, 1.45, "original_config")
            if deadline_failure:
                decision["emergency_reason"] = "inference/control deadline exceeded"
                return module._json_finite(decision)
            decision.update(control_source="learned_trajectory_tracker", learned_control_proposed=True,
                            control=output)
            return module._json_finite(decision)

    module.trajectory_control, module.SimulationResearchPolicy = control, Policy
    module._json_finite = research._json_finite
    return module, calls, raw


@pytest.mark.parametrize("arm", candidate.ARMS)
def test_scoped_hook_calls_original_once_preserves_prediction_and_timing(arm):
    module, calls, raw = fake_research()
    original = module.trajectory_control, module.SimulationResearchPolicy.step
    policy = module.SimulationResearchPolicy()
    with candidate.tracker_scope(module, arm):
        result = policy.step()
        assert result["raw_output"] == raw and result["total_seconds"] == .012
        assert result["selected"] == dict(candidate_index=5)
        assert result["source_checkpoint_sha256"] == "fixed_W1_checkpoint"
        assert result["control"] == research._json_finite(candidate.normal_tracker_control(proposal(), arm))
        assert isinstance(result["control"]["pursuit_xy_base_m"], list)
        assert result["moving_brake_ab"]["normal_tracker_called"] is True
        assert result["moving_brake_ab"]["actuator_acknowledged"] is False
        assert len(calls) == 1
    assert (module.trajectory_control, module.SimulationResearchPolicy.step) == original


def actual_policy_fixture(monkeypatch, desired):
    """Use the actual policy, controller and serializer; isolate only tensors/model/gate inputs."""
    selected = SelectedTrajectory(candidate_index=2,
        xy_base_m=tuple(((index + 1) * .1, -.01) for index in range(64)),
        speed_mps=(desired,) * 64, timestep_s=.1, logit_margin=1., planar_extent_m=6.4)
    monkeypatch.setattr(research, "validate_and_select_trajectory", lambda *_args, **_kwargs: selected)
    monkeypatch.setattr(research, "build_runtime_tensors", lambda *_args: {
        name: torch.zeros(1) for name in research.FIELDS})
    monkeypatch.setattr(research.time, "perf_counter", lambda: 1.)

    class Model:
        config = SimpleNamespace(model_id=research.PHYSICAL_STOPMIX_MODEL_ID, candidate_count=12)
        _research_source_checkpoint_sha256 = research.AH_CHECKPOINT_SHA256

        def eval(self):
            return self

        def __call__(self, *_tensors):
            return torch.zeros(1, 12, 64, 2), torch.full((1, 12, 64), desired), torch.zeros(1, 12)

    state = [0.] * len(research.FEATURE_NAMES)
    state[research.FEATURE_NAMES.index("velocity_x_mps")] = desired + .04
    inputs = SimpleNamespace(camera_frames=(SimpleNamespace(timestamp_ns=1_000_000_000),) * 6,
                             ego_history=(state,))
    policy = research.SimulationResearchPolicy(Model(), research.ControllerConfig(2.7, .6))
    return policy, inputs, dict(now_ns=1_000_000_000, ego_stamp_ns=1_000_000_000,
                                route_stamp_ns=1_000_000_000)


@pytest.mark.parametrize("arm", candidate.ARMS)
@pytest.mark.parametrize("desired", [0., .05, .0500000001, 1.4])
def test_actual_policy_serialized_tuple_handoff_and_stop_boundary(monkeypatch, arm, desired):
    policy, inputs, timing = actual_policy_fixture(monkeypatch, desired)
    baseline = policy.step(inputs, **timing)
    assert baseline["learned_control_proposed"] is True
    assert isinstance(baseline["control"]["pursuit_xy_base_m"], list)
    with candidate.tracker_scope(research, arm):
        result = policy.step(inputs, **timing)
    audit = result.pop("moving_brake_ab")
    expected = deepcopy(baseline)
    if arm == candidate.ARMS[1] and desired > candidate.STOP_SPEED_MPS:
        expected["control"]["brake"] = 0.
    assert result == expected
    assert audit["normal_control_returned"] is True
    assert audit["normal_brake_suppression_returned"] is (
        arm == candidate.ARMS[1] and desired > candidate.STOP_SPEED_MPS)
    assert isinstance(audit["original_normal_tracker_control"]["pursuit_xy_base_m"], tuple)
    if desired <= candidate.STOP_SPEED_MPS:
        assert result["control"]["brake"] >= .3


@pytest.mark.parametrize("arm", candidate.ARMS)
def test_actual_policy_external_teacher_emergency_is_exactly_preserved(monkeypatch, arm):
    policy, inputs, timing = actual_policy_fixture(monkeypatch, 1.4)
    timing["emergency_reason"] = "expert brake override"
    baseline = policy.step(inputs, **timing)
    with candidate.tracker_scope(research, arm):
        result = policy.step(inputs, **timing)
    audit = result.pop("moving_brake_ab")
    assert result == baseline and result["control"]["brake"] == 1.
    assert audit["normal_tracker_called"] is False
    assert audit["normal_brake_suppression_returned"] is False


@pytest.mark.parametrize("arm", candidate.ARMS)
def test_actual_policy_deadline_after_tracker_preserves_original_emergency(monkeypatch, arm):
    policy, inputs, timing = actual_policy_fixture(monkeypatch, 1.4)
    clock = iter((1., 1.2, 1.21))
    monkeypatch.setattr(research.time, "perf_counter", lambda: next(clock))
    with candidate.tracker_scope(research, arm):
        result = policy.step(inputs, **timing)
    assert result["control"]["brake"] == 1. and result["raw_output"] is not None
    assert "deadline" in result["emergency_reason"]
    assert result["moving_brake_ab"]["normal_tracker_called"] is True
    assert result["moving_brake_ab"]["normal_control_returned"] is False
    assert result["moving_brake_ab"]["normal_brake_suppression_returned"] is False


def test_scoped_hook_preserves_emergency_and_discards_stale_tracker_audit():
    module, calls, _ = fake_research()
    policy = module.SimulationResearchPolicy()
    with candidate.tracker_scope(module, candidate.ARMS[1]):
        assert policy.step()["moving_brake_ab"]["normal_brake_suppression_returned"] is True
        emergency = policy.step(emergency="expert brake override")
        assert emergency["control"]["brake"] == 1.
        assert emergency["emergency_reason"] == "expert brake override"
        assert emergency["moving_brake_ab"]["normal_tracker_called"] is False
        assert emergency["moving_brake_ab"]["normal_brake_suppression_returned"] is False
        assert len(calls) == 1


def test_original_deadline_emergency_after_tracker_remains_full_brake():
    module, _, _ = fake_research()
    with candidate.tracker_scope(module, candidate.ARMS[1]):
        result = module.SimulationResearchPolicy().step(deadline_failure=True)
    assert result["control"]["brake"] == 1.
    assert result["moving_brake_ab"]["normal_brake_suppression_proposed"] is True
    assert result["moving_brake_ab"]["normal_brake_suppression_returned"] is False


def test_hooks_restore_on_exception_and_reject_nested_scopes():
    module, _, _ = fake_research()
    original = module.trajectory_control, module.SimulationResearchPolicy.step
    with pytest.raises(RuntimeError):
        with candidate.tracker_scope(module, candidate.ARMS[1]):
            with pytest.raises(ValueError, match="nested"):
                with candidate.tracker_scope(module, candidate.ARMS[0]):
                    pass
            raise RuntimeError("injected worker interruption")
    assert (module.trajectory_control, module.SimulationResearchPolicy.step) == original
