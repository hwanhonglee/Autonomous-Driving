"""HH_261001 - Fixed moving-task membership/clock/history, unchanged old authority, and normal ramp."""

import pytest
import sys
import types

from scripts.e2e import carla_moving_capture as original
from scripts.e2e import town07_low_speed_moving_task_20261001 as task
from scripts.e2e.collect_town07_low_speed_moving_20261001 import TargetRamp
from scripts.e2e.carla_goal_stop_profile import ComfortableGoalStopConfig, GoalStopGovernor


@pytest.fixture(autouse=True)
def adequate_disk_for_pure_tests(monkeypatch):
    monkeypatch.setattr(task, 'available_disk_bytes', lambda: 6 * 1024**3)


def camera(frame, phase='driving'):
    return dict(frame=frame, timestamp=frame * .05, capture_phase=phase,
        camera_order=list(original.CAMERAS), images={name: name for name in original.CAMERAS},
        source_timestamps={name: frame * .05 for name in original.CAMERAS})


def observation(instance, frame, progress, phase='driving', events=0):
    value, reason = instance.observe(frame=frame, timestamp=frame * .05, phase=phase,
        progress=progress, base_stop_reason=None, event_count=events)
    if frame % 2 == 0:
        instance.saved_camera(camera(frame, phase))
    return value, reason


def test_fixed_membership_full_history_future_and_tail():
    instance = task.MovingTask(task.plan_contract())
    for frame in range(70):
        observation(instance, frame, 0., 'stationary_warmup')
    for frame in range(70, 1600):
        value, reason = observation(instance, frame, (frame - 70) * .125)
        if reason:
            assert reason == 'moving_task_future_support_complete'
            break
    assert instance.first['frame'] == 390
    assert instance.last['frame'] == 1030
    assert len(instance.selected) == 321
    assert instance.support['frame'] == 1158
    assert instance.history_support['camera_observations'][0]['frame'] == 372
    assert instance.history_support['preceding_native_guard']['frame'] == 371
    assert not instance.result(tail_states=129)['task_complete']
    result = instance.result(tail_states=130, goal_reached=False)
    assert result['task_complete'] and not result['actual_catalog_goal_reached']
    assert result['admission_status'] == 'PENDING_QA'
    assert 'training_data_approved' not in result


def test_first_anchor_without_complete_history_fails_no_later_trimming():
    instance = task.MovingTask(task.plan_contract())
    observation(instance, 0, 40.)
    assert instance.failure == 'moving_first_anchor_full_history_missing'
    assert len(instance.selected) == 1
    assert not instance.result(tail_states=130)['task_complete']


def test_short_geometric_window_span_rejected_and_event_preserved():
    instance = task.MovingTask(task.plan_contract())
    for frame in range(500):
        observation(instance, frame, frame * .4)
    assert instance.failure == 'moving_selected_span_under_30_seconds'
    failing = task.MovingTask(task.plan_contract())
    observation(failing, 0, 0., events=1)
    assert failing.failure == 'moving_observed_collision_or_lane_event'


def test_clock_gap_and_progress_reentry_fail():
    instance = task.MovingTask(task.plan_contract())
    observation(instance, 0, 0.)
    observation(instance, 2, 1.)
    assert instance.failure == 'moving_native_cadence_failure'
    instance = task.MovingTask(task.plan_contract())
    for frame in range(24):
        observation(instance, frame, 39. + frame * .05)
    observation(instance, 24, 39.)
    assert instance.failure == 'moving_progress_decreased_or_reentered'


def test_native_disk_guard_and_replay_use_original_recorded_bytes(monkeypatch):
    instance = task.MovingTask(task.plan_contract())
    monkeypatch.setattr(task, 'available_disk_bytes', lambda: task.FREE_SPACE_FLOOR_BYTES + task.CLOSING_RESERVE_BYTES - 1)
    _, reason = observation(instance, 0, 0.)
    assert reason == 'moving_disk_floor_guard'
    assert not instance.result(tail_states=130)['task_complete']
    replay = task.MovingTask(task.plan_contract())
    _, reason = replay.observe(frame=0, timestamp=0., phase='driving', progress=0.,
        base_stop_reason=None, event_count=0, observed_free_disk_bytes=6 * 1024**3)
    assert reason is None and replay.minimum_observed_free_bytes == 6 * 1024**3


def test_owner_preflight_resource_budget_fails_before_server(monkeypatch):
    from scripts.e2e import run_owned_town07_low_speed_moving_20261001 as owner
    monkeypatch.setattr(task, 'available_disk_bytes', lambda: task.FREE_SPACE_FLOOR_BYTES + task.ESTIMATED_MAX_RAW_BYTES - 1)
    with pytest.raises(ValueError, match='preflight disk budget blocked'):
        owner.prepare()


def test_new_scope_does_not_expand_old_sealed_authority():
    with pytest.raises(ValueError):
        original.require_15kph_speed(9.)
    with pytest.raises(ValueError):
        original.slot_spec(task.SLOT)
    plan = task.plan_contract()
    assert plan['selected_progress_m'] == [40., 120.]
    assert plan['minimum_selected_camera_span_ns'] == 30_000_000_000
    assert plan['fixed_arguments']['goal_stop_profile'] == 'disabled'
    assert plan['controller']['normal_brake_cap'] == 0.
    assert plan['controller']['BasicAgent_emergency_unchanged']
    assert 'training_data_approved' not in plan and 'development_only' not in plan


def test_target_ramp_equals_original_governor_far_from_goal_after_initial_proposal():
    ramp = TargetRamp()
    assert ramp.update(100) == 2.5
    governor = GoalStopGovernor(ComfortableGoalStopConfig(desired_deceleration_mps2=.5), 2.5, 20)
    for frame in range(101, 151):
        original_target = governor.update(100., 100., 2., -100.)['target_speed_mps']
        assert ramp.update(frame) == original_target
    with pytest.raises(ValueError):
        ramp.update(152)


def test_new_wrapper_preserves_emergency_after_normal_caps(monkeypatch, tmp_path):
    from scripts.e2e import collect_town07_low_speed_moving_20261001 as worker
    def control():
        return types.SimpleNamespace(throttle=.7, brake=.2, steer=.4, hand_brake=False,
            reverse=False, gear=0, manual_gear_shift=False)
    world = types.SimpleNamespace(frame=100)
    world.get_snapshot = lambda: types.SimpleNamespace(frame=world.frame,
        timestamp=types.SimpleNamespace(elapsed_seconds=world.frame * .05))
    vehicle = types.SimpleNamespace(get_world=lambda: world,
        get_velocity=lambda: types.SimpleNamespace(x=0., y=0.))
    class Agent:
        def __init__(self, vehicle):
            self._vehicle, self._max_brake, self.emergency = vehicle, 1., False
            self.planner = types.SimpleNamespace(_vehicle=vehicle,
                _vehicle_controller=types.SimpleNamespace(max_brake=.3, max_throt=.75), run_step=control)
        def get_local_planner(self):
            return self.planner
        def set_target_speed(self, value):
            self.target = value
        def run_step(self):
            result = self.planner.run_step()
            if self.emergency:
                result.throttle, result.brake = 0., 1.
            return result
    agents = types.ModuleType('agents')
    navigation = types.ModuleType('agents.navigation')
    module = types.ModuleType('agents.navigation.basic_agent')
    module.BasicAgent = Agent
    agents.navigation, navigation.basic_agent = navigation, module
    for name, value in [('agents', agents), ('agents.navigation', navigation), ('agents.navigation.basic_agent', module)]:
        monkeypatch.setitem(sys.modules, name, value)
    def original_collect(*args, **kwargs):
        agent = module.BasicAgent(vehicle)
        normal = agent.run_step()
        assert normal.throttle == .13 and normal.brake == 0. and normal.steer == 0.
        world.frame += 1
        agent.emergency = True
        emergency = agent.run_step()
        assert emergency.throttle == 0. and emergency.brake == 1.
        return 'captured'
    monkeypatch.setattr(worker.base, 'collect_episode', original_collect)
    old_provider = worker.base.moving_capture_module
    manifest = {'capture_contract': {}}
    with worker.scope(types.SimpleNamespace()):
        result = worker.base.collect_episode(None, None, None, tmp_path, [], [], manifest)
        assert result == 'captured'
        assert manifest['admission_status'] == 'PENDING_QA'
        assert 'training_data_approved' not in manifest
    assert worker.base.collect_episode is original_collect
    assert worker.base.moving_capture_module is old_provider
    assert module.BasicAgent is Agent
