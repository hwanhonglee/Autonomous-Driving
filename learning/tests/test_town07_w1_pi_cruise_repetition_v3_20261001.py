"""HH_261001 - Synthetic contracts/observer/ownership tests; no simulator or physical control is launched."""

from copy import deepcopy
import errno
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from scripts.e2e import research_town07_w1_pi_cruise_repetition_v3_20261001 as run
from scripts.e2e import town07_w1_pi_cruise_repetition_contract_v3_20261001 as contract
from scripts.e2e import capture_exact_simulation_inputs_20261001 as exact


def test_fixed_three_trials_and_unreduced_gates():
    fixed = contract.fixed_contract()
    assert len(fixed["trial_order"]) == fixed["attempts_per_arm"] == 3
    assert [row["index"] for row in fixed["trial_order"]] == [0, 1, 2]
    assert {row["arm"] for row in fixed["trial_order"]} == {"B_pi_only"}
    assert len({row["output"] for row in fixed["trial_order"]}) == 3
    assert fixed["total_capture_seconds"] == 45.
    assert (fixed["expected_native_states"], fixed["expected_camera_bundles"]) == (900, 450)
    screen = fixed["prospective_motion_screen"]
    assert screen["net_progress_minimum_m"] == 90.
    assert (screen["cte_p95_maximum_m"], screen["cte_absolute_maximum_m"]) == (.35, .75)
    assert screen["all_native_held_acknowledgements_required"]
    assert screen["all_three_must_pass"]
    assert not fixed["automatic_retry"] and not fixed["full_feature_approval"]
    assert not fixed["end_to_end_realtime_fps_approved"]


@pytest.mark.parametrize("index", [-1, 3, True, 1., None])
def test_no_selected_or_additional_trials(index):
    with pytest.raises(ValueError):
        contract.trial(index)


def test_storage_only_sampling_revision_fits_without_shortening():
    fixed = contract.fixed_contract()
    assert fixed["sparse_exact_inputs_every"] == 20
    assert fixed["maximum_exact_npz_samples"] == len(range(0, 450, 20)) == 23
    assert 23*4150804 < contract.SAMPLE_LIMIT == 128*1024**2
    assert contract.TRIAL_RESERVE == 512*1024**2
    assert contract.FLOOR == 5*1024**3
    assert fixed["all_original_JPEG_native_ACK_predictions_retained"]


def test_explicit_closure_excludes_frozen_running_new_training_and_v2_runtime():
    names = contract.SOURCE_FILES
    assert len(names) == len(set(names))
    assert "learning/portable_e2e/longitudinal_pi_research_20261001.py" in names
    assert "tools/e2e/collect_ah_simulation_research.py" in names
    assert not any("temporal_v2_low_speed_joint" in name or "temporal_simulation_research_v2" in name for name in names)
    assert contract.source_pins()[str(contract.assets_v2.FREEZE.relative_to(contract.ROOT))] == contract.assets_v2.FREEZE_SHA


def test_disk_floor_accounts_all_remaining_attempts():
    required = contract.FLOOR+3*contract.TRIAL_RESERVE+contract.SHARED_RESERVE
    assert contract.disk_check(3, free=required)["required_bytes"] == required
    with pytest.raises(ValueError):
        contract.disk_check(3, free=required-1)
    for value in (0, 4, True):
        with pytest.raises(ValueError):
            contract.disk_check(value, free=required)


def test_natural_bind_wait_uses_original_probe_and_bounded_poll():
    tick, calls = [0.], []

    def probe(port):
        calls.append(port)
        if len(calls) < 4:
            raise OSError(errno.EADDRINUSE, "in use")

    value = run.wait_for_original_bind_checks(2440, probe,
        pause=lambda amount: tick.__setitem__(0, tick[0]+amount), clock=lambda: tick[0])
    assert calls == [2440]*4 and tick[0] == pytest.approx(.6)
    assert value["original_all_tcp_udp_bind_check"]
    assert not value["sockets_forced_or_reused"]
    assert value["maximum_wait_s"] == 75.


def test_natural_bind_timeout_never_sleeps_past_seventy_five_seconds():
    tick, sleeps = [0.], []

    def pause(amount):
        sleeps.append(amount)
        tick[0] += amount

    def probe(port):
        raise OSError(errno.EADDRINUSE, "retained ownership")

    with pytest.raises(OSError):
        run.wait_for_original_bind_checks(2440, probe, pause=pause, clock=lambda: tick[0])
    assert tick[0] == 75. and max(sleeps) <= .2


def test_other_socket_errors_are_never_retried_or_bypassed():
    sleeps = []

    def probe(port):
        raise OSError(errno.EACCES, "denied")

    with pytest.raises(OSError):
        run.wait_for_original_bind_checks(2440, probe, pause=sleeps.append, clock=lambda: 0.)
    assert not sleeps


def observer_fixture():
    fields = ("images", "calibration", "ego_history", "ego_history_mask", "route_xy", "route_mask")
    tensors = {name: torch.arange(6, dtype=torch.float32).reshape(1, 6) for name in fields}
    tensors["ego_history_mask"] = torch.ones((1, 6), dtype=torch.bool)
    tensors["route_mask"] = torch.ones((1, 6), dtype=torch.bool)
    pixels = np.zeros((360, 640, 4), dtype=np.uint8)
    pixels[0, 0, :] = (1, 2, 3, 4)
    inputs = SimpleNamespace(camera_frames=tuple(SimpleNamespace(
        payload=pixels, encoding="bgra8", timestamp_ns=100) for _ in range(6)))
    received = []
    research = SimpleNamespace(FIELDS=fields)
    research.build_runtime_tensors = lambda *args, **kwargs: tensors

    class Policy:
        def step(self, value, **kwargs):
            built = research.build_runtime_tensors(value)
            assert built is tensors
            decision = dict(source_checkpoint_sha256=contract.assets_v2.CHECKPOINT,
                raw_output=dict(candidate_logits=list(range(12))), total_seconds=.04,
                control=dict(throttle=.2, brake=0., steer=.001))
            received.append(decision)
            return decision

    research.SimulationResearchPolicy = Policy
    return research, inputs, received, tensors


def test_unified_observer_hashes_all_bgraw_inputs_samples_twentieth_and_never_changes_decisions(tmp_path):
    research, inputs, received, tensors = observer_fixture()
    original_step, original_builder = research.SimulationResearchPolicy.step, research.build_runtime_tensors
    originals = {name: value.clone() for name, value in tensors.items()}
    with run.observed_runtime_inputs(tmp_path, research, exact, SimpleNamespace(CollectionError=RuntimeError)):
        policy = research.SimulationResearchPolicy()
        for i in range(41):
            decision = policy.step(inputs, now_ns=100+i*100000000, ego_stamp_ns=100, route_stamp_ns=100)
            assert decision is received[-1]
    assert research.SimulationResearchPolicy.step is original_step
    assert research.build_runtime_tensors is original_builder
    assert all(torch.equal(value, originals[name]) for name, value in tensors.items())
    hashes = run.rows(tmp_path / "all_inference_input_hashes.jsonl")
    samples = run.rows(tmp_path / "sampled_exact_runtime_inputs/index.jsonl")
    assert len(hashes) == 41
    assert [row["policy_call_index"] for row in samples] == [0, 20, 40]
    assert all(len(row["capture"]["native_pixels"]) == 6 for row in hashes)
    assert all(row["capture"]["native_pixels"][0]["encoding"] == "bgra8" for row in hashes)
    assert all(row["capture"]["native_pixels"][0]["pixels_sha256"] == exact.array_digest(inputs.camera_frames[0].payload)
               for row in hashes)
    assert all(row["observer_cost_in_original_deadline"] and not row["inputs_outputs_controls_modified"] for row in hashes)
    for row in samples:
        with np.load(tmp_path / "sampled_exact_runtime_inputs" / row["tensor_file"], allow_pickle=False) as saved:
            assert set(saved.files) == set(tensors)
            assert all(np.array_equal(saved[name], value.numpy()) for name, value in tensors.items())


def test_sample_quota_abort_retains_unacknowledged_original_raw_decision_and_restores(tmp_path, monkeypatch):
    research, inputs, received, _ = observer_fixture()
    original_step, original_builder = research.SimulationResearchPolicy.step, research.build_runtime_tensors
    monkeypatch.setattr(contract, "SAMPLE_LIMIT", 1024)
    with pytest.raises(RuntimeError):
        with run.observed_runtime_inputs(tmp_path, research, exact, SimpleNamespace(CollectionError=RuntimeError)):
            research.SimulationResearchPolicy().step(inputs, now_ns=100, ego_stamp_ns=100, route_stamp_ns=100)
    assert research.SimulationResearchPolicy.step is original_step
    assert research.build_runtime_tensors is original_builder
    witness = json.loads((tmp_path / "sample_allowance_abort.json").read_text())
    assert witness["original_unmodified_unacknowledged_decision"] == received[-1]
    assert witness["proposed_decision_sent"] is False and witness["output_or_control_modified"] is False


def state(frame, sequence, speed=2., cte=.1):
    return dict(frame=frame, timestamp=frame*.05, capture_phase="driving",
        route_progress_m=frame*.1, route_cte_m=cte, world_velocity_carla=[speed, 0., 0.],
        control_transport=dict(expected_receipt_sequence=sequence), collision=[], lane_invasion=[])


def simple_decision(sequence, source="learned_trajectory_tracker"):
    return dict(ack_receipt_sequence=sequence, control_source=source,
        learned_control_proposed=source == "learned_trajectory_tracker", emergency_reason=None,
        application=dict(acknowledged=True, learned_control_applied=source == "learned_trajectory_tracker"))


def test_observed_interval_uses_prior_ack_and_keeps_later_failures_including_held_ticks():
    states = [state(1, 10), state(2, 11), state(3, 12), state(4, 13)]
    decisions = [simple_decision(10, "expert_startup"), simple_decision(11),
                 simple_decision(12), simple_decision(13, "emergency_brake")]
    selected, witnesses, failures = run.native_learned_interval(states, decisions)
    assert [row["frame"] for row in selected] == [2, 3, 4]
    assert [row["ack_receipt_sequence"] for row in witnesses] == [11, 12, 13]
    assert failures == ["nonlearned_or_rejected_native_tick:4"]


def test_duplicate_decision_ack_or_absent_handoff_cannot_pass():
    with pytest.raises(ValueError):
        run.native_learned_interval([state(1, 10)], [simple_decision(10), simple_decision(10)])
    assert run.native_learned_interval([state(1, 10)], [simple_decision(10, "expert_startup")])[2]


def test_native_stationary_strict_limit_and_cte_nearest_rank_over_all_rows():
    states = [state(i, i, speed=0., cte=.1 if i < 19 else .8) for i in range(20)]
    metrics = run.motion_numbers(states)
    assert metrics["longest_unintended_stationary_dwell_seconds"] == 1.
    assert metrics["cte_p95_m"] == .1 and metrics["cte_abs_maximum_m"] == .8


def normal_decision(sequence=1, winner=1):
    control = dict(throttle=.2, brake=0., steer=.001, desired_speed_mps=2.4, reverse=False, hand_brake=False)
    original = dict(control, throttle=.1)
    audit = dict(arm="B_pi_only", integral_added=True, normal_control_returned=True,
        original_normal_tracker_control=original, proposed_normal_tracker_control=control)
    audit.update({name: False for name in ("selected_speed_or_path_changed", "steering_changed", "stop_branch_changed",
        "emergency_authority_changed", "control_cadence_changed", "feedforward_added", "throttle_floor_added")})
    return dict(simple_decision(sequence), source_checkpoint_sha256=contract.assets_v2.CHECKPOINT,
        raw_output=dict(candidate_logits=[10. if i == winner else 0. for i in range(12)],
                        candidate_xy=[[[.2*(step+1), 0.] for step in range(64)] for _ in range(12)],
                        candidate_speed=[[2.4]*64 for _ in range(12)]),
        selected=dict(candidate_index=winner, xy_base_m=[[.2*(step+1), 0.] for step in range(64)], speed_mps=[2.4]*64),
        control=control, longitudinal_pi_research=audit, total_seconds=.04)


def test_normal_pi_unchanged_raw_winner_is_a_valid_diagnostic_control():
    result = run.decision_checks([normal_decision()])
    assert result["unexpected_stop_commands"] == result["emergency_commands"] == 0
    assert result["control_invariant_failures"] == []
    assert result["max_policy_seconds"] == .04


@pytest.mark.parametrize("winner", [6, 7, 8, 9, 10, 11])
def test_all_explicit_stop_families_fail_cruise_even_before_vehicle_stops(winner):
    value = normal_decision(winner=winner)
    assert run.decision_checks([value])["unexpected_stop_commands"] == 1
    assert value["selected"]["candidate_index"] == winner


def test_other_steering_change_or_rerank_is_never_allowed():
    changed = normal_decision()
    changed["control"]["steer"] = .2
    assert any("expanded_controller" in value for value in run.decision_checks([changed])["control_invariant_failures"])
    reranked = normal_decision()
    reranked["selected"]["candidate_index"] = 2
    assert any("raw_argmax_changed" in value for value in run.decision_checks([reranked])["control_invariant_failures"])


def test_main_contract_does_not_declare_launch_or_create_run(monkeypatch, capsys):
    monkeypatch.setattr(run, "preflight", lambda *a: pytest.fail("no preflight requested"))
    monkeypatch.setattr(run, "execute", lambda *a: pytest.fail("no physical execution requested"))
    assert run.main(["--mode", "contract"]) == 0
    assert json.loads(capsys.readouterr().out)["status"] == "PREPARED_NOT_DECLARED_NOT_EXECUTED"


def test_summary_retains_all_failures_and_rejects_missing_third_trial(tmp_path, monkeypatch):
    monkeypatch.setattr(contract, "RUN_ROOT", tmp_path)
    monkeypatch.setattr(run, "declaration", lambda expected: {})
    for index in range(3):
        folder = contract.output(index)
        folder.mkdir()
        identity = dict(trial=contract.trial(index), declaration_sha256="a"*64)
        contract.write_new(folder / "owner_result.json", identity)
        contract.write_new(folder / "cruise_motion_screen.json", dict(identity,
            status="FAIL_RETAINED" if index == 1 else "PASS_LOW_SPEED_CRUISE_RESEARCH_SCREEN_ONLY",
            retained_in_three_attempt_denominator=True, failures=["emergency"] if index == 1 else []))
    value = run.repetition_summary("a"*64)
    assert value["passed_attempts"] == 2 and value["failed_attempts"] == 1
    assert value["total_predeclared_attempts"] == 3 and value["failed_attempts_excluded"] == 0
    assert not value["motion_reproducibility_research_screen_passed"]
    assert not value["full_feature_approval"] and not value["generalization_approved"]


def test_worker_argument_contract_is_45_seconds_and_no_goal_speed_substitution():
    value = run.capture_argv(0)
    assert value[value.index("--max-duration-sec")+1] == "45"
    assert value[value.index("--goal-stop-profile")+1] == "disabled"
    assert value[value.index("--target-speed-kmh")+1] == "9"
    assert run.worker_command(0, "a"*64)[-len(value):] == value


def test_ack_fields_keep_original_float32_conversion_tolerance():
    value = dict(throttle=.12145845856853077, brake=0., steer=.001487821438853414,
                 reverse=False, hand_brake=False)
    serialized = dict(value, throttle=float(np.float32(value["throttle"])), steer=float(np.float32(value["steer"])))
    assert run.acknowledged_actuator_fields_match(serialized, value)
    assert not run.acknowledged_actuator_fields_match(dict(serialized, steer=value["steer"]+.0001), value)
    assert not run.acknowledged_actuator_fields_match(dict(serialized, reverse=0), value)
