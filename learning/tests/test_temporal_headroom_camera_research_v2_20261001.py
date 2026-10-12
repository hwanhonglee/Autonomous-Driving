"""HH_261001 - Check that the separate controller calibration cannot change model/gate/deadline semantics."""

from dataclasses import replace
import math

import numpy as np
import pytest
import torch

from portable_e2e.runtime import RuntimeCameraFrame, RuntimeInputs
from portable_e2e.simulation_research import ControllerConfig
from portable_e2e.temporal_cached_encoder_v2 import CachedTemporalModelV2
from portable_e2e.temporal_model_v2 import TemporalModelConfigV2
from portable_e2e.temporal_headroom_camera_research_v2_20261001 import HeadroomOwnedCameraTemporalSimulationResearchPolicyV2
from portable_e2e import temporal_owned_camera_history_v2_20261001 as owned
from portable_e2e import temporal_simulation_research_v2_20261001 as frozen
from scripts.e2e import collect_temporal_v2_headroom_research_20261001 as collector


IDENTITY = dict(episode_id="unit-headroom-fixture", split="train", rig_id="rig", source_sha256="a"*64)


@pytest.fixture(autouse=True)
def one_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def make_policy(kind=HeadroomOwnedCameraTemporalSimulationResearchPolicyV2, preferred_speed=2.5, pi=True):
    torch.manual_seed(7)
    model = CachedTemporalModelV2(TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128)).eval()
    with torch.no_grad():
        model.curvature_head.weight.zero_(); model.curvature_head.bias.zero_()
        model.speed_head.weight.zero_()
        model.speed_head.bias.fill_(math.log(math.expm1(preferred_speed)))
    model._research_temporal_checkpoint_sha256 = "1"*64
    model._research_temporal_export_npz_sha256 = "2"*64
    model._research_temporal_metadata_sha256 = "3"*64
    return kind(model, ControllerConfig(2.7, .6), **IDENTITY, longitudinal_pi=pi)


def frame(index, speed=1.):
    stamp = 1_000_000_000+index*100_000_000
    calibration = (.7, 1.2, .5, .5, 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.)
    state = (1., speed, 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.)
    cameras = tuple(RuntimeCameraFrame(stamp,
        np.full((360, 640, 4), index+i, dtype=np.uint8), "bgra8") for i in range(6))
    inputs = RuntimeInputs(cameras, (calibration,)*6, (state,)*10, (True,)*10, ((0., 0.), (100., 0.)))
    source = frozen.ObservedTemporalFrameV2(stamp, inputs, (index*.1, 0., 0.), (0., 0., 0., 1.), stamp, **IDENTITY)
    return owned.own_observed_camera_bytes(source)


def call(policy, method, current, past, **extra):
    return getattr(policy, method)(current, past_frames=tuple(past), now_ns=current.timestamp_ns,
        ego_stamp_ns=current.timestamp_ns, route_stamp_ns=current.timestamp_ns, **extra)


def ready(policy, frames):
    call(policy, "prefill", frames[8], frames[:8])
    call(policy, "prefill", frames[9], frames[:9])


def test_headroom_changes_only_tracker_throttle_not_raw_path_or_single_selected_trajectory(monkeypatch):
    monkeypatch.setattr(frozen.time, "perf_counter", lambda: 1.)
    frames = [frame(i, 2.45) for i in range(11)]
    original = make_policy(owned.OwnedCameraTemporalSimulationResearchPolicyV2)
    proposed = make_policy()
    ready(original, frames); ready(proposed, frames)
    first, second = call(original, "step", frames[10], frames[1:10]), call(proposed, "step", frames[10], frames[1:10])
    assert first["learned_control_proposed"] is True and second["learned_control_proposed"] is True
    for key in ("raw_output", "decoded_output", "selected", "candidate_gates", "controller"):
        assert first[key] == second[key], key
    audit = second["longitudinal_pi_research"]
    assert audit["original_model_target_mps"] == first["control"]["desired_speed_mps"]
    assert audit["governed_controller_target_mps"] == 2.4
    assert second["control"]["desired_speed_mps"] == first["control"]["desired_speed_mps"]
    assert second["control"]["steer"] == first["control"]["steer"]
    assert second["measured_current_speed_mps"] == first["measured_current_speed_mps"]
    # HH_261001 - The original normal tracker remains documented even when the PI target is governed.
    assert audit["original_normal_tracker_control"] == first["longitudinal_pi_research"]["original_normal_tracker_control"]
    assert second["control"]["throttle"] < first["control"]["throttle"]
    assert second["control"]["brake"] > first["control"]["brake"]
    assert proposed.pi.integral_error == audit["integral_after"]


@pytest.mark.parametrize("cause", ["over_speed", "external", "cadence", "deadline"])
def test_unchanged_fail_closed_gates_and_reset_authority(monkeypatch, cause):
    monkeypatch.setattr(frozen.time, "perf_counter", lambda: 1.)
    frames = [frame(i) for i in range(12)]
    policy = make_policy(); ready(policy, frames)
    policy.pi.integral_error = .3
    current, past, extra = frames[10], frames[1:10], {}
    if cause == "over_speed": current = frame(10, 2.505561754)
    elif cause == "external": extra["emergency_reason"] = "independent safety guard"
    elif cause == "cadence": current, past = frames[11], frames[2:11]
    else:
        original = policy.build_inputs
        def slow(*args, **kwargs):
            result = original(*args, **kwargs)
            monkeypatch.setattr(frozen.time, "perf_counter", lambda: 1.101)
            return result
        monkeypatch.setattr(policy, "build_inputs", slow)
    result = call(policy, "step", current, past, **extra)
    assert result["learned_control_proposed"] is False and result["control"]["brake"] == 1.
    assert policy.pi.integral_error == 0. and policy._prefill_count == 0
    if cause == "over_speed": assert "speed exceeds unchanged low-speed envelope; no clipping" in result["emergency_reason"]
    if cause == "deadline": assert result["emergency_reason"] == "temporal inference/control deadline exceeded"


def test_explicit_PI_required_and_rejections_do_not_expose_controller_fallback():
    with pytest.raises(ValueError, match="explicit longitudinal PI"):
        make_policy(pi=False)


def test_startup_prefills_have_no_actuation_or_integrator_update_and_reset_discards_authority(monkeypatch):
    monkeypatch.setattr(frozen.time, "perf_counter", lambda: 1.)
    policy = make_policy()
    frames = [frame(i) for i in range(10)]
    ready(policy, frames)
    assert policy.pi.integral_error == 0. and policy._prefill_count == 2
    assert all(row["control_proposed"] is False for row in policy.prefill_records)
    policy.pi.integral_error = .3
    policy.reset("explicit_handoff_or_emergency_reset")
    assert policy.pi.integral_error == 0. and policy._prefill_count == 0
    assert HeadroomOwnedCameraTemporalSimulationResearchPolicyV2.step is owned.OwnedCameraTemporalSimulationResearchPolicyV2.step


def test_collector_activation_scopes_same_observed_session_and_restores_all_bindings():
    saved_policy = collector.owned.OwnedCameraTemporalSimulationResearchPolicyV2
    saved_session = collector.owned.frozen.TemporalLiveResearchSessionV2
    saved_generic_policy = collector.owned.frozen.TemporalSimulationResearchPolicyV2
    with pytest.raises(RuntimeError, match="stop"):
        with collector.activated(None, None):
            assert collector.owned.frozen.TemporalLiveResearchSessionV2 is collector.owned.OwnedCameraTemporalLiveResearchSessionV2
            assert collector.owned.frozen.TemporalSimulationResearchPolicyV2 is HeadroomOwnedCameraTemporalSimulationResearchPolicyV2
            with pytest.raises(RuntimeError, match="nest"):
                with collector.activated(None, None): pass
            raise RuntimeError("stop")
    assert collector.owned.OwnedCameraTemporalSimulationResearchPolicyV2 is saved_policy
    assert collector.owned.frozen.TemporalLiveResearchSessionV2 is saved_session
    assert collector.owned.frozen.TemporalSimulationResearchPolicyV2 is saved_generic_policy
