"""HH_261001 - Exercise SI anti-windup, strict causal gates and unchanged owner/emergency paths without CARLA."""

import copy
import inspect
from types import SimpleNamespace

import pytest

from scripts.e2e import calibrate_town12_j15824_pi_coast_hold_20261001 as worker
from scripts.e2e import run_owned_town12_j15824_pi_coast_hold_20261001 as owner


def control(throttle=.2, brake=0.):
    return SimpleNamespace(throttle=throttle, brake=brake, steer=.07, gear=1,
        hand_brake=False, reverse=False, manual_gear_shift=False)


def apply(policy, index, speed):
    return policy.apply(control(), speed=speed, timestamp=index * .05, frame=100 + index)


def enter_coast(policy):
    apply(policy, 0, 0.)
    for index in range(1, 11):
        result = apply(policy, index, 1.5)
    return result


def test_first_command_is_exact_zero_then_slew_and_anti_windup_hold_integral():
    pi = worker.PersistentPI()
    assert pi.step(0., 0.) == 0. and pi.integral == 0.
    assert pi.step(0., .05) == pytest.approx(.001)
    assert pi.integral == 0. and pi.last["integral_update_rejected"]
    assert pi.last["integral_candidate"] == pytest.approx(.003)
    assert pi.last["candidate_unclipped_throttle"] == pytest.approx(.183)
    assert pi.last["unclipped_throttle_after_anti_windup"] == pytest.approx(.18)


def test_integral_persists_beyond_ten_samples_without_sliding_window():
    pi = worker.PersistentPI()
    pi.throttle = .012
    for _ in range(25):
        pi.step(1.4, .05)
        assert not pi.last["integral_update_rejected"]
    assert pi.integral == pytest.approx(.005)
    assert pi.throttle == pytest.approx(.017)


def test_upward_saturation_and_downward_slew_both_prevent_further_windup():
    pi = worker.PersistentPI()
    pi.integral, pi.throttle = .2, .2
    assert pi.step(0., .05) == .2 and pi.integral == .2
    pi.integral = .1
    assert pi.step(2., .05) == pytest.approx(.1975)
    assert pi.integral == .1 and pi.last["integral_update_rejected"]
    assert pi.last["allowed_throttle_lower"] == pytest.approx(.1975)


def test_integral_unwinds_when_error_moves_back_inside_feasible_range():
    pi = worker.PersistentPI()
    pi.integral, pi.throttle = .1, .088
    pi.step(1.6, .05)
    assert not pi.last["integral_update_rejected"]
    assert pi.integral == pytest.approx(.0998)
    assert pi.throttle == pytest.approx(.0878)


@pytest.mark.parametrize("speed", [-.01, 2.551, float("nan"), float("inf")])
def test_invalid_raw_measurement_changes_no_control(speed):
    policy, value = worker.StartStopPolicy(), control()
    before = vars(value).copy()
    with pytest.raises(worker.scout.ScoutError):
        policy.apply(value, speed=speed, timestamp=0., frame=100)
    assert vars(value) == before


def test_actual_ten_consecutive_entry_observations_are_recorded_without_filtering():
    policy = worker.StartStopPolicy()
    apply(policy, 0, 0.)
    for index in range(1, 10):
        apply(policy, index, 1.5)
        assert policy.entry is None
    value = apply(policy, 10, 1.5)
    assert policy.last["stage"] == worker.COAST_STAGE and value.throttle == value.brake == 0.
    assert [x["frame"] for x in policy.entry["preceding_ten_native_observations"]] == list(range(101, 111))
    assert value.steer == .07 and value.gear == 1 and not value.manual_gear_shift


def test_out_of_band_sample_resets_entry_and_coast_does_not_slew():
    policy = worker.StartStopPolicy()
    for index in range(12):
        apply(policy, index, 1.43 if index % 2 else 1.51)
    assert policy.entry is None and len(policy.entry_witnesses) == 0
    policy.pi.throttle = .20
    for index in range(12, 22):
        value = apply(policy, index, 1.5)
    assert policy.entry is not None and value.throttle == 0.
    assert not policy.last["pi_updated_this_tick"]


def test_launch_has_no_timeout_extension_or_target_as_measurement():
    policy = worker.StartStopPolicy()
    for index in range(400):
        apply(policy, index, 1.449)
    with pytest.raises(worker.scout.ScoutError, match="20-second"):
        apply(policy, 400, 1.449)
    assert policy.entry is None


def test_hold_requires_two_consecutive_actual_low_samples_and_keeps_pi_state():
    policy = worker.StartStopPolicy()
    enter_coast(policy)
    integral = policy.pi.integral
    assert apply(policy, 11, .04).brake == 0.
    assert apply(policy, 12, .051).brake == 0.
    assert apply(policy, 13, .05).brake == 0.
    value = apply(policy, 14, .05)
    assert value.throttle == 0. and value.brake == 1.
    assert policy.hold_entry["preceding_two_native_observations"] == [
        {"frame": 113, "timestamp": .65, "raw_actor_planar_speed_mps": .05},
        {"frame": 114, "timestamp": pytest.approx(.70), "raw_actor_planar_speed_mps": .05}]
    assert policy.pi.integral == integral and policy.hold_observations == 0
    assert not policy.observe(speed=.03, frame=115)


def test_hold_needs_forty_native_ack_bound_physical_observations():
    policy = worker.StartStopPolicy()
    enter_coast(policy)
    apply(policy, 11, .04)
    for number, index in enumerate(range(12, 52), 1):
        value = apply(policy, index, .04)
        assert value.brake == 1.
        assert policy.observe(speed=.01, frame=101 + index) is (number == 40)
    assert policy.hold_commands == policy.hold_observations == 40
    with pytest.raises(worker.scout.ScoutError, match="completion"):
        apply(policy, 52, .01)


@pytest.mark.parametrize("frame,speed,message", [(999, .01, "bound"), (113, .051, "hold speed")])
def test_bad_hold_observation_cannot_count_as_success(frame, speed, message):
    policy = worker.StartStopPolicy()
    enter_coast(policy)
    apply(policy, 11, .04)
    apply(policy, 12, .04)
    with pytest.raises(worker.scout.ScoutError, match=message):
        policy.observe(speed=speed, frame=frame)
    assert policy.hold_observations == 0


def test_coast_has_no_low_speed_feedback_and_expires_after_ten_seconds():
    policy = worker.StartStopPolicy()
    enter_coast(policy)
    for index in range(11, 210):
        value = apply(policy, index, .06)
        assert value.throttle == value.brake == 0.
    with pytest.raises(worker.scout.ScoutError, match="10-second"):
        apply(policy, 210, .06)
    assert policy.hold_entry is None


@pytest.mark.parametrize("timestamp,frame", [(.1, 101), (.05, 100), (.05, 102), (0., 101)])
def test_stale_skipped_or_duplicate_native_clock_is_rejected(timestamp, frame):
    policy = worker.StartStopPolicy()
    apply(policy, 0, 0.)
    with pytest.raises(worker.scout.ScoutError, match="cadence"):
        policy.apply(control(), speed=0., timestamp=timestamp, frame=frame)


def test_original_emergency_return_is_not_modified_or_rate_limited():
    policy = worker.StartStopPolicy()
    emergency = control(throttle=0., brake=.5)
    def original_agent():
        apply(policy, 0, 0.)
        return emergency
    command, decision, overridden = worker.checked_agent_command(SimpleNamespace(run_step=original_agent), policy, 100)
    assert command is emergency and command.brake == .5 and overridden
    assert decision["normal_control_before_original_emergency"]["brake"] == 0.
    assert decision["original_agent_overrode_normal_control"]
    source = inspect.getsource(worker.run_trial)
    assert source.index("observed = advance(command, decision)") < source.index("scout.require(not emergency")
    assert source.index("scout.require(not emergency") < source.index("if policy.observe(")


def test_original_normal_brake_cap_and_missing_source_frame_still_fail():
    with pytest.raises(worker.scout.ScoutError, match="capped"):
        worker.StartStopPolicy().apply(control(brake=.15), speed=0., timestamp=0., frame=100)
    with pytest.raises(worker.scout.ScoutError, match="missing or stale"):
        worker.checked_agent_command(SimpleNamespace(run_step=lambda: control()), worker.StartStopPolicy(), 100)


def test_original_native_bounds_fail_without_clipping_and_cleanup_stays_separate():
    bounds = {"physical_decoder": {"maximum_acceleration_mps2": 2.9, "maximum_deceleration_mps2": 2.9},
              "runtime_speed_rate_gate": {"maximum_acceleration_mps2": 3., "maximum_deceleration_mps2": 6.}}
    rows = [{"frame": index, "speed_rate_mps2": rate, "applied_policy": {"stage": stage}}
            for index, (rate, stage) in enumerate(((3.6, worker.PI_STAGE), (-3., worker.COAST_STAGE),
                (-.5, worker.HOLD_STAGE), (-20., "termination_service_brake_nontraining")))]
    original = copy.deepcopy(rows)
    result = worker.native_quality(rows, bounds)
    assert rows == original and result["status"] == "FAIL" and result["assessed_native_intervals"] == 3
    assert len(result["failed_native_intervals"]) == 2
    assert result["failed_native_intervals"][0]["failed_bounds"] == ["physical_decoder", "runtime_speed_rate_gate"]
    assert result["failed_native_intervals"][1]["failed_bounds"] == ["physical_decoder"]
    assert not result["full_trajectory_quality_proven"]


def test_exact_inherited_guards_cleanup_and_explicit_one_trial_nontraining_scope():
    assert worker.scalar_guards is worker.previous.scalar_guards
    assert worker.checked_geometry is worker.previous.checked_geometry
    assert worker.destroy_owned is worker.previous.destroy_owned
    contract = worker.protocol()
    assert contract["candidate_trials"] == 1 and contract["maximum_launch_seconds"] == 20.
    assert contract["maximum_coast_seconds"] == 10. and contract["hold_observed_ticks"] == 40
    assert contract["entry_band_mps"] == [1.45, 1.55]
    assert contract["initial_throttle"] == 0. and contract["target_speed_mps"] == 1.5
    assert not contract["training_data_admitted"] and not contract["six_camera_capture"]
    assert not contract["future_labels_created"] and not contract["smoother_launch_claimed"]
    assert not contract["traffic_light_states_written"]


def test_prior_v4_evidence_and_preregistration_are_frozen_inputs():
    owner.validate_prior_evidence()
    assert owner.PREREGISTRATION.is_file()
    assert len(owner.SOURCE_FILES) == len(set(owner.SOURCE_FILES))
    assert all((owner.ROOT / name).is_file() for name in owner.SOURCE_FILES)
    assert all(path.is_file() for path in owner.EXTERNAL_EVIDENCE.values())
    assert owner.OUTPUT != owner.prior_owner.OUTPUT and not owner.OUTPUT.exists()


def test_all_owner_bindings_and_deep_callback_scopes_restore_after_exception():
    inherited = owner.inherited
    before = inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker
    scout_owner = inherited.inherited
    external = scout_owner.EXTERNAL_SOURCES
    root_owner = scout_owner.diagnostic.owner
    callbacks = root_owner.preflight, root_owner._read_live_xodr, root_owner._write_new_json
    with pytest.raises(RuntimeError):
        with owner.configured_owner():
            assert inherited.OUTPUT == owner.OUTPUT and inherited.worker is worker
            assert owner.EXTERNAL_EVIDENCE.items() <= scout_owner.EXTERNAL_SOURCES.items()
            with inherited.configured_owner() as actual:
                assert actual is root_owner and actual._read_live_xodr is not callbacks[1]
                raise RuntimeError("restore")
    assert (inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker) == before
    assert scout_owner.EXTERNAL_SOURCES is external
    assert (root_owner.preflight, root_owner._read_live_xodr, root_owner._write_new_json) == callbacks


def test_worker_dry_run_cannot_contact_source_api(monkeypatch, capsys):
    monkeypatch.setattr(worker.scout, "source_api", lambda: pytest.fail("CARLA contact forbidden"))
    assert worker.main([]) == 0
    assert '"simulator_contacted": false' in capsys.readouterr().out


@pytest.mark.parametrize("delta", [-1, 0, 1])
def test_five_gib_disk_floor_is_exact_and_never_deletes(monkeypatch, delta):
    free = worker.MINIMUM_FREE_BYTES + delta
    monkeypatch.setattr(worker.shutil, "disk_usage", lambda path: SimpleNamespace(free=free))
    if delta < 0:
        with pytest.raises(worker.scout.ScoutError, match="5 GiB"):
            worker.require_free_space(owner.ROOT)
    else:
        assert worker.require_free_space(owner.ROOT) == free
