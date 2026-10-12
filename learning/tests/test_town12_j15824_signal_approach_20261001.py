"""HH_261001 - Bound-signal stopping changes only normal target speed and restores every scope."""

from types import SimpleNamespace

import pytest

from scripts.e2e import capture_town12_j15824_signal_approach_20261001 as worker
from scripts.e2e import run_owned_town12_j15824_signal_approach_20261001 as owner


@pytest.mark.parametrize("state,distance,expected", [
    ("Yellow", 7.5, True), ("Red", 12., True), ("Red", 0., True),
    ("Yellow", 12.001, False), ("Red", -0.01, False), ("Green", 7.5, False)])
def test_policy_distance_and_state_are_bounded(state, distance, expected):
    assert worker.should_stop(state, distance) is expected


def test_invalid_state_or_distance_fails_before_command():
    with pytest.raises(worker.scout.ScoutError):
        worker.should_stop("Off", 3.)
    with pytest.raises(worker.scout.ScoutError):
        worker.should_stop("Red", float("nan"))


def test_normal_target_change_keeps_original_control_and_restores_target():
    planner = SimpleNamespace(_target_speed=9.)
    agent = SimpleNamespace(get_local_planner=lambda: planner,
        set_target_speed=lambda value: setattr(planner, "_target_speed", value))
    emergency = SimpleNamespace(brake=.5, throttle=0., steer=.7)
    calls = []
    def original():
        calls.append(planner._target_speed)
        return emergency
    control, record = worker.apply_target_policy(agent, original, "Red", 7.)
    assert control is emergency and control.brake == .5 and control.steer == .7
    assert calls == [0.] and planner._target_speed == 9.
    assert record["active"] and record["target_restored_after_step"]
    worker.apply_target_policy(agent, original, "Green", 7.)
    assert calls == [0., 9.]


def test_original_step_failure_cannot_leave_target_or_module_patched():
    planner = SimpleNamespace(_target_speed=1.2)
    agent = SimpleNamespace(get_local_planner=lambda: planner,
        set_target_speed=lambda value: setattr(planner, "_target_speed", value))
    def fail():
        raise RuntimeError("original error")
    with pytest.raises(RuntimeError):
        worker.apply_target_policy(agent, fail, "Yellow", 5.)
    assert planner._target_speed == 1.2
    original = worker.base.instrumented_collector
    with worker.configured_policy():
        assert worker.base.instrumented_collector is not original
    assert worker.base.instrumented_collector is original


def test_new_owner_output_worker_and_policy_are_explicit_and_restored():
    before = owner.inherited.OUTPUT, owner.inherited.WORKER, owner.inherited.protocol_record
    with owner.configured_owner():
        assert owner.inherited.OUTPUT == owner.OUTPUT and owner.inherited.WORKER == owner.WORKER
        policy = owner.inherited.protocol_record()["additional_signal_policy_revision"]
        assert policy["policy_id"] == worker.POLICY_ID
        assert policy["original_basic_agent_red_emergency_unchanged"] is True
        assert policy["traffic_light_states_written"] is False
        assert policy["strict_boundary_gate_changed"] is False
    assert (owner.inherited.OUTPUT, owner.inherited.WORKER, owner.inherited.protocol_record) == before
