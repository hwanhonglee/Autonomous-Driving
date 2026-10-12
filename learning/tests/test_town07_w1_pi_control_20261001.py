"""HH_261001 - Verify controller-only PI scoping against the actual policy deadline and serializer."""

from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import pytest
import torch

from portable_e2e import simulation_research as research
from portable_e2e.runtime_contract import SelectedTrajectory
from scripts.e2e import town07_w1_pi_control_20261001 as candidate


def fixture(monkeypatch, desired=1.5, observed=1.):
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

        def __call__(self, *_args):
            return torch.zeros(1, 12, 64, 2), torch.full((1, 12, 64), desired), torch.zeros(1, 12)

    state = [0.] * len(research.FEATURE_NAMES)
    state[research.FEATURE_NAMES.index("velocity_x_mps")] = observed
    inputs = SimpleNamespace(camera_frames=(SimpleNamespace(timestamp_ns=1_000_000_000),) * 6,
                             ego_history=(state,))
    policy = research.SimulationResearchPolicy(Model(), research.ControllerConfig(2.7, .6))
    return policy, inputs


def call(policy, inputs, stamp=1_000_000_000, **overrides):
    for frame in inputs.camera_frames:
        frame.timestamp_ns = stamp
    return policy.step(inputs, now_ns=stamp, ego_stamp_ns=stamp, route_stamp_ns=stamp, **overrides)


@pytest.mark.parametrize("arm", candidate.ARMS)
def test_actual_policy_preserves_all_nonlongitudinal_outputs(monkeypatch, arm):
    policy, inputs = fixture(monkeypatch)
    baseline = call(policy, inputs)
    before = deepcopy(baseline)
    with candidate.tracker_scope(research, arm):
        result = call(policy, inputs)
    audit = result.pop("longitudinal_pi_research")
    for key in result:
        if key != "control":
            assert result[key] == before[key]
    for key in result["control"]:
        if key not in ("throttle", "brake"):
            assert result["control"][key] == before["control"][key]
    assert baseline == before
    assert audit["normal_control_returned"] is True
    assert audit["actuator_acknowledged"] is False
    assert audit["source_checkpoint_sha256"] == research.AH_CHECKPOINT_SHA256
    if arm == candidate.ARMS[0]:
        assert result == before and audit["committed_integral_after"] == 0.
    else:
        assert result["control"]["throttle"] == pytest.approx(.36)
        assert audit["committed_integral_after"] == pytest.approx(.05)


@pytest.mark.parametrize("arm", candidate.ARMS)
@pytest.mark.parametrize("desired", [0., .01, .05])
def test_model_stop_branch_is_bytevalue_preserved(monkeypatch, arm, desired):
    policy, inputs = fixture(monkeypatch, desired)
    baseline = call(policy, inputs)
    with candidate.tracker_scope(research, arm):
        result = call(policy, inputs)
    audit = result.pop("longitudinal_pi_research")
    assert result == baseline and result["control"]["brake"] >= .3
    assert audit["committed_integral_after"] == 0.
    assert audit["stop_branch_preserved"] is True


def test_updates_once_per_camera_call_not_per_ack(monkeypatch):
    policy, inputs = fixture(monkeypatch)
    with candidate.tracker_scope(research, candidate.ARMS[1]):
        first = call(policy, inputs)
        second = call(policy, inputs, 1_100_000_000)
    assert first["control"]["throttle"] == pytest.approx(.36)
    assert second["control"]["throttle"] == pytest.approx(.37)
    assert second["longitudinal_pi_research"]["previous_camera_policy_timestamp_ns"] == 1_000_000_000
    assert second["longitudinal_pi_research"]["held_control_native_timestep_s"] == .05


@pytest.mark.parametrize("arm", candidate.ARMS)
@pytest.mark.parametrize("step_ns", [0, 50_000_000, 100_001_001, 200_000_000, -100_000_000])
def test_cadence_discontinuity_fails_closed_and_resets(monkeypatch, arm, step_ns):
    policy, inputs = fixture(monkeypatch)
    with candidate.tracker_scope(research, arm):
        call(policy, inputs)
        failed = call(policy, inputs, 1_000_000_000 + step_ns)
        recovered = call(policy, inputs, 1_500_000_000)
    assert failed["control"]["brake"] == 1.
    assert "cadence" in failed["emergency_reason"]
    assert failed["longitudinal_pi_research"]["committed_integral_after"] == 0.
    assert recovered["learned_control_proposed"] is True
    if arm == candidate.ARMS[1]:
        assert recovered["control"]["throttle"] == pytest.approx(.36)


def test_rounding_tolerance_is_bounded(monkeypatch):
    policy, inputs = fixture(monkeypatch)
    with candidate.tracker_scope(research, candidate.ARMS[1]):
        call(policy, inputs)
        assert call(policy, inputs, 1_100_001_000)["learned_control_proposed"] is True


@pytest.mark.parametrize("arm", candidate.ARMS)
def test_external_emergency_keeps_exact_original_decision_and_resets(monkeypatch, arm):
    policy, inputs = fixture(monkeypatch)
    baseline = call(policy, inputs, 1_100_000_000, emergency_reason="expert brake override")
    with candidate.tracker_scope(research, arm):
        call(policy, inputs)
        result = call(policy, inputs, 1_100_000_000, emergency_reason="expert brake override")
        next_result = call(policy, inputs, 1_200_000_000)
    audit = result.pop("longitudinal_pi_research")
    assert result == baseline and result["control"]["brake"] == 1.
    assert audit["normal_tracker_called"] is False and audit["committed_integral_after"] == 0.
    if arm == candidate.ARMS[1]:
        assert next_result["control"]["throttle"] == pytest.approx(.36)


@pytest.mark.parametrize("arm", candidate.ARMS)
def test_deadline_failure_after_tracker_retains_full_brake_and_discards_integral(monkeypatch, arm):
    policy, inputs = fixture(monkeypatch)
    with candidate.tracker_scope(research, arm):
        clock = iter((1., 1.2, 1.21))
        monkeypatch.setattr(research.time, "perf_counter", lambda: next(clock))
        failed = call(policy, inputs)
        monkeypatch.setattr(research.time, "perf_counter", lambda: 1.)
        next_result = call(policy, inputs, 1_100_000_000)
    assert "deadline" in failed["emergency_reason"]
    assert failed["control"]["brake"] == 1. and failed["raw_output"] is not None
    audit = failed["longitudinal_pi_research"]
    assert audit["normal_tracker_called"] is True and audit["normal_control_returned"] is False
    assert audit["committed_integral_after"] == 0.
    if arm == candidate.ARMS[1]:
        assert next_result["control"]["throttle"] == pytest.approx(.36)


def test_policy_objects_and_new_handoffs_have_separate_state(monkeypatch):
    first, inputs = fixture(monkeypatch)
    second = research.SimulationResearchPolicy(first.model, first.controller)
    with candidate.tracker_scope(research, candidate.ARMS[1]):
        call(first, inputs)
        call(first, inputs, 1_100_000_000)
        result = call(second, inputs, 1_100_000_000)
    assert result["control"]["throttle"] == pytest.approx(.36)
    assert result["longitudinal_pi_research"]["previous_camera_policy_timestamp_ns"] is None


@pytest.mark.parametrize("field,value", [("throttle_gain", .71), ("brake_gain", .71),
    ("speed_cap_mps", 2.), ("maximum_throttle", .4), ("speed_preview_s", .4)])
def test_changed_original_control_gains_fail_closed(monkeypatch, field, value):
    policy, inputs = fixture(monkeypatch)
    policy.controller = replace(policy.controller, **{field: value})
    with candidate.tracker_scope(research, candidate.ARMS[1]):
        result = call(policy, inputs)
    assert result["control"]["brake"] == 1.
    assert "settings differ" in result["emergency_reason"]


def test_scope_restores_frozen_hooks_and_rejects_nesting(monkeypatch):
    fixture(monkeypatch)
    original = research.trajectory_control, research.SimulationResearchPolicy.step
    with pytest.raises(RuntimeError):
        with candidate.tracker_scope(research, candidate.ARMS[1]):
            with pytest.raises(ValueError, match="nested"):
                with candidate.tracker_scope(research, candidate.ARMS[0]):
                    pass
            raise RuntimeError("synthetic worker interruption")
    assert (research.trajectory_control, research.SimulationResearchPolicy.step) == original


def test_tracker_direct_call_without_camera_policy_is_forbidden(monkeypatch):
    policy, _ = fixture(monkeypatch)
    with candidate.tracker_scope(research, candidate.ARMS[1]):
        with pytest.raises(ValueError, match="within"):
            research.trajectory_control(None, 1., policy.controller)


def test_rejects_unknown_arm(monkeypatch):
    fixture(monkeypatch)
    with pytest.raises(ValueError, match="arm"):
        with candidate.tracker_scope(research, "constant_throttle_floor"):
            pass
