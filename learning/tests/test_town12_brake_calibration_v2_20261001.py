"""HH_261001 - Timed caps preserve normal PID, real entry measurements and every inherited safety boundary."""

from types import SimpleNamespace

import pytest

from scripts.e2e import calibrate_town12_j15824_brake_pulses_v2_20261001 as worker
from scripts.e2e import run_owned_town12_j15824_brake_calibration_v2_20261001 as owner


def control(throttle=.2, brake=0., steer=.07):
    return SimpleNamespace(throttle=throttle, brake=brake, steer=steer,
        hand_brake=False, reverse=False, manual_gear_shift=False, gear=1)


def test_timed_cap_uses_native_elapsed_time_without_speed_handoff():
    policy = worker.TimedThrottleCap()
    for elapsed, cap in [(0., .13), (1., .14), (6., .19), (7., .20), (8., .20), (19.95, .20)]:
        command = policy.apply(control(), speed=.01, timestamp=100. + elapsed,
                               frame=2000 + round(20 * elapsed), normal_proposal={})
        assert command.throttle == pytest.approx(cap) and command.brake == 0. and command.steer == .07
        assert not policy.last["speed_handoff_latched"] and not policy.last["smoother_launch_claimed"]
    with pytest.raises(worker.scout.ScoutError, match="20-second"):
        policy.apply(control(), speed=.01, timestamp=120., frame=2400, normal_proposal={})


def test_timed_cap_cannot_force_throttle_above_original_pid():
    policy = worker.TimedThrottleCap()
    assert policy.apply(control(.03), speed=.2, timestamp=5., frame=100, normal_proposal={}).throttle == .03
    assert policy.apply(control(0.), speed=.2, timestamp=5.05, frame=101, normal_proposal={}).throttle == 0.


@pytest.mark.parametrize("speed,time,brake", [(float("nan"), 0., 0.), (-1., 0., 0.),
    (0., float("nan"), 0.), (0., 0., .01)])
def test_invalid_normal_input_fails_before_mutation(speed, time, brake):
    value = control(brake=brake)
    with pytest.raises(worker.scout.ScoutError):
        worker.TimedThrottleCap().apply(value, speed=speed, timestamp=time, frame=1, normal_proposal={})
    assert value.throttle == .2 and value.brake == brake


def test_measured_pulse_prerequisite_and_20_second_native_timeout_remain_bounded():
    with worker.configured_worker():
        policy = worker.inherited.PulsePolicy(.001)
        for index in range(170):
            output = policy.apply(control(), speed=.1, timestamp=index * .05, frame=index)
        assert output.throttle == .2 and policy.entry is None
        assert policy.last["stage"] == "timed_cap_launch_and_stabilize"
        for index in range(170, 179):
            policy.apply(control(), speed=2.4, timestamp=index * .05, frame=index)
            assert policy.entry is None
        output = policy.apply(control(), speed=2.4, timestamp=179 * .05, frame=179)
        assert policy.entry["frame"] == 179 and policy.pulse_ticks == 1
        assert output.throttle == 0. and output.brake == .001
        assert policy.last["stage"] == "measured_brake_pulse"
        assert policy.completion(.5) == "EARLY_MEASURED_SPEED_FLOOR"
        unready = worker.inherited.PulsePolicy(0.)
        for index in range(400):
            unready.apply(control(), speed=.1, timestamp=index * .05, frame=index)
        with pytest.raises(worker.scout.ScoutError, match="stabilization timed out"):
            unready.apply(control(), speed=.1, timestamp=20., frame=400)


@pytest.mark.parametrize("level", [0., .0001, .001, .005])
def test_all_levels_still_allow_only_forty_native_pulse_commands(level):
    with worker.configured_worker():
        policy = worker.inherited.PulsePolicy(level)
        for index in range(49):
            output = policy.apply(control(), speed=2.4, timestamp=index * .05, frame=index)
        assert policy.pulse_ticks == 40 and output.brake == level and output.throttle == 0.
        assert policy.completion(2.4) == "FIXED_PULSE_DURATION"


def test_worker_hooks_restore_after_error_and_do_not_touch_gates_or_physics_loop():
    previous = worker.inherited.LaunchControl, worker.inherited.PulsePolicy, worker.inherited.protocol
    untouched = worker.inherited.scalar_guards, worker.inherited.run_level, worker.inherited.checked_geometry
    with pytest.raises(RuntimeError):
        with worker.configured_worker():
            assert worker.inherited.LaunchControl is worker.TimedThrottleCap
            raise RuntimeError("restore")
    assert (worker.inherited.LaunchControl, worker.inherited.PulsePolicy, worker.inherited.protocol) == previous
    assert (worker.inherited.scalar_guards, worker.inherited.run_level, worker.inherited.checked_geometry) == untouched


def test_protocol_changes_only_approved_launch_fields_and_rejects_smoothness_claim():
    old, new = worker.ORIGINAL_PROTOCOL(), worker.protocol()
    for name in ("brake_levels", "entry_band_mps", "consecutive_entry_band_ticks", "pulse_ticks",
                 "minimum_all_body_corner_stop_distance_m", "maximum_route_progress_m",
                 "minimum_instantaneous_body_lane_margin_m", "base_min_distance_m", "distance_ratio_s",
                 "original_basic_agent_red_emergency_unchanged", "traffic_light_states_written"):
        assert new[name] == old[name]
    assert "launch_handoff_mps" not in new and "post_handoff_ramp_per_second" not in new
    assert not new["training_data_admitted"] and not new["smoother_launch_claimed"]


def test_new_owner_output_worker_sources_and_protocol_are_scoped():
    previous = owner.inherited.OUTPUT, owner.inherited.WORKER, owner.inherited.SOURCE_FILES, owner.inherited.worker
    with owner.configured_owner():
        assert owner.inherited.OUTPUT == owner.OUTPUT and owner.inherited.WORKER == owner.WORKER
        assert owner.inherited.worker.protocol()["policy_id"] == worker.POLICY_ID
        assert all((owner.ROOT / name).is_file() for name in owner.SOURCE_FILES)
    assert (owner.inherited.OUTPUT, owner.inherited.WORKER, owner.inherited.SOURCE_FILES, owner.inherited.worker) == previous


def test_nonexecuting_worker_dispatch_uses_new_scope_and_restores(monkeypatch):
    def main(arguments):
        assert arguments == []
        assert worker.inherited.protocol()["policy_id"] == worker.POLICY_ID
        assert worker.inherited.PulsePolicy is worker.TimedPulsePolicy
        return 0
    monkeypatch.setattr(worker.inherited, "main", main)
    assert worker.main([]) == 0
    assert worker.inherited.PulsePolicy is worker.ORIGINAL_PULSE_POLICY
