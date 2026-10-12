"""HH_260906 - Preserve original engagement/emergency checks around the new simulation-only steering intervention."""
import ast
import inspect
from types import SimpleNamespace

import pytest

from scripts.e2e import collect_carla_vad_expert as capture
from scripts.e2e import carla_goal_stop_profile as profiles


class Actors:
    def __init__(self, pattern): self.pattern = pattern
    def filter(self, pattern): return [object()] if pattern == self.pattern else []


@pytest.mark.parametrize('pattern', ['walker.pedestrian.0001', 'walker.*', 'controller.ai.walker'])
def test_walker_guard_rejects_existing_pedestrians(pattern):
    lookup = 'walker.*' if pattern.startswith('walker.') else pattern
    with pytest.raises(capture.CollectionError, match='walkers'):
        capture._preflight_no_walkers(SimpleNamespace(get_actors=lambda: Actors(lookup)))


def test_empty_walker_guard_does_not_mutate_world():
    capture._preflight_no_walkers(SimpleNamespace(get_actors=lambda: Actors('not_present')))


def test_existing_profile_does_not_read_new_event_source():
    governor = profiles.DevelopmentGoalStopGovernor(profiles.BrakeHoldDevelopmentGoalStopConfig(), 8., 20.)
    record = {'original': True}
    capture.observe_low_speed_steering_events(governor, record,
        SimpleNamespace(snapshot=lambda: pytest.fail('legacy event monitor changed')))
    assert record == {'original': True}


@pytest.mark.parametrize('collision,lane', [(False,False), (True,False), (False,True)])
def test_v6_observed_event_abort_is_recorded_without_synchronous_delivery_claim(collision, lane):
    governor = profiles.DevelopmentGoalStopGovernor(profiles.LowSpeedSteeringDevelopmentGoalStopConfig(), 8., 20.)
    record = governor.update(180., 180., 0., -180., timestamp=1., longitudinal_speed_mps=0., driving=True)
    recorder = SimpleNamespace(snapshot=lambda: ({123:[{'event':'collision'}]} if collision else {},
                                                  {123:[{'event':'lane'}]} if lane else {}))
    capture.observe_low_speed_steering_events(governor, record, recorder)
    observed = record['low_speed_steering_event_monitor']
    assert observed['observed_event_counts'] == {'collision':int(collision), 'lane_invasion':int(lane)}
    assert observed['asynchronous_event_delivery'] is True and observed['zero_latency_abort_proven'] is False
    assert bool(governor.failure_reason) is (collision or lane)
    if collision or lane: assert record['pilot_failure_reason'].endswith('_observed_event_abort')


def test_hook_placement_preserves_original_first_proposal_check_and_emergency_annotation():
    # HH_260906 - The first proposed steering check must see the original BasicAgent output, never the zeroed output.
    tree = ast.parse(inspect.getsource(capture.collect_episode))
    hooks = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name) and node.func.id == 'apply_low_speed_steering']
    assert len(hooks) == 2
    initial = next(node for node in hooks if isinstance(node.args[0],ast.Name) and node.args[0].id == 'initial_drive_control')
    regular = next(node for node in hooks if isinstance(node.args[0],ast.Name) and node.args[0].id == 'next_control')
    pid_check = next(node for node in ast.walk(tree) if isinstance(node,ast.Constant)
                     and node.value == 'first BasicAgent steering step exceeds the unchanged PID delta limit')
    assert pid_check.lineno < initial.lineno
    annotations = [node.lineno for node in ast.walk(tree) if isinstance(node,ast.Call)
                   and isinstance(node.func,ast.Name) and node.func.id == 'annotate_development_control']
    assert max(line for line in annotations if line < regular.lineno) < regular.lineno
    guards = [node.lineno for node in ast.walk(tree) if isinstance(node,ast.Call)
              and isinstance(node.func,ast.Name) and node.func.id == 'suppress_stopped_brake_steering']
    assert any(regular.lineno < line < initial.lineno for line in guards)
    assert any(line > initial.lineno for line in guards)


def test_legacy_parser_rejects_new_arm_without_new_profile():
    with pytest.raises(SystemExit): capture.parse_args(['out', 'route.json', '--low-speed-steering-mode', 'zero'])
