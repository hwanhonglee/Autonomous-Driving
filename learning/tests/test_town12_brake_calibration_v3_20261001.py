"""HH_261001 - Low-speed entry changes are explicit; the complete native level loop and safety gates stay frozen."""

import inspect
from types import SimpleNamespace

import pytest

from scripts.e2e import calibrate_town12_j15824_brake_pulses_v3_20261001 as worker
from scripts.e2e import run_owned_town12_j15824_brake_calibration_v3_20261001 as owner


def control():
    return SimpleNamespace(throttle=.2, brake=0., steer=.07,
        hand_brake=False, reverse=False, manual_gear_shift=False, gear=1)


def test_level_loop_diff_is_only_actual_pid_target_and_its_live_assertion():
    original = inspect.getsource(worker.inherited.run_level)
    revised = inspect.getsource(worker.run_level)
    expected = original.replace("8.64", "5.58").replace("        agent.set_global_plan(plan)\n",
        "        agent.set_global_plan(plan)\n"
        "        scout.require(agent._target_speed == 5.58 and agent.get_local_planner()._target_speed == 5.58,\n"
        "                      \"actual agent and local PID targets differ from 1.55 m/s\")\n"
        "        record[\"effective_target_speed_kmh\"] = 5.58\n")
    assert revised == expected


def test_pulse_logic_diff_is_only_measured_entry_band_and_honest_stage_label():
    original = inspect.getsource(worker.inherited.PulsePolicy)
    assert inspect.getsource(worker.PulsePolicy) == original.replace("2.35 <= speed <= 2.45", "1.45 <= speed <= 1.55").replace(
        '"gentle_launch_and_stabilize"', '"timed_cap_launch_and_stabilize"')
    assert worker.LaunchControl is worker.previous.TimedThrottleCap


@pytest.mark.parametrize("level", [0., .0001, .001, .005])
def test_actual_ten_tick_low_speed_entry_then_fixed_pulse_budget(level):
    policy = worker.PulsePolicy(level)
    for index in range(9):
        policy.apply(control(), speed=1.5, timestamp=index * .05, frame=index)
        assert policy.entry is None
    result = policy.apply(control(), speed=1.5, timestamp=.45, frame=9)
    assert policy.entry["speed_mps"] == 1.5 and result.brake == level and result.throttle == 0.
    for index in range(10, 49):
        policy.apply(control(), speed=1.4, timestamp=index * .05, frame=index)
    assert policy.pulse_ticks == 40 and policy.completion(1.4) == "FIXED_PULSE_DURATION"


def test_entry_never_uses_pid_target_in_place_of_actual_speed_and_timeout_unchanged():
    policy = worker.PulsePolicy(0.)
    for index in range(400):
        policy.apply(control(), speed=1.449, timestamp=index * .05, frame=index)
    assert policy.entry is None and policy.stable_ticks == 0
    with pytest.raises(worker.scout.ScoutError, match="timed out"):
        policy.apply(control(), speed=1.449, timestamp=20., frame=400)


def test_metadata_and_scopes_match_the_explicit_new_nontraining_protocol():
    metadata = worker.protocol()
    assert metadata["entry_band_mps"] == [1.45, 1.55]
    assert metadata["original_pid_target_speed_kmh"] == 5.58
    assert metadata["original_pid_target_speed_mps"] == 1.55
    assert not metadata["training_data_admitted"] and not metadata["higher_speed_extrapolation_claimed"]
    assert not metadata["smoother_launch_claimed"]
    old = worker.inherited.run_level, worker.inherited.protocol
    with worker.configured_worker():
        assert worker.inherited.run_level is worker.run_level
        assert worker.inherited.protocol()["policy_id"] == worker.POLICY_ID
    assert (worker.inherited.run_level, worker.inherited.protocol) == old
    before = owner.inherited.OUTPUT, owner.inherited.WORKER, owner.inherited.SOURCE_FILES, owner.inherited.worker
    with owner.configured_owner():
        assert owner.inherited.OUTPUT == owner.OUTPUT and owner.inherited.WORKER == owner.WORKER
        assert all((owner.ROOT / name).is_file() for name in owner.SOURCE_FILES)
    assert (owner.inherited.OUTPUT, owner.inherited.WORKER, owner.inherited.SOURCE_FILES, owner.inherited.worker) == before
