"""HH_260906 - Check fixed pedal interventions and unchanged emergency/quality boundaries without CARLA."""
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace
import pytest
from scripts.e2e import collect_carla_pedal_caps_aj as aj
from scripts.e2e import prepare_carla_common10_dataset as adapter
from scripts.e2e.carla_goal_stop_profile import install_normal_brake_cap

ROOT=Path(__file__).resolve().parents[2]
ROUTE=None


@pytest.fixture(autouse=True)
def route_fixture(tmp_path,monkeypatch):
    # HH_260906 - Unit tests do not depend on private simulator recordings or change the real route seal.
    global ROUTE
    ROUTE=tmp_path/'route.json';ROUTE.write_text('{"town":"Town01"}')
    monkeypatch.setattr(aj,'ROUTE_SHA',aj.hashlib.sha256(ROUTE.read_bytes()).hexdigest())


def arguments(tmp_path,arm='launch'):
    return [str(tmp_path/'episode'),str(ROUTE),'--pedal-arm',arm,'--goal-stop-profile','comfortable_v2',
        '--target-speed-kmh','9','--physics-hz','20','--capture-hz','10','--max-duration-sec','180',
        '--stationary-warmup-sec','3.5','--stationary-tail-sec','6.5','--goal-tolerance-m','1',
        '--agent-initialization','after_bootstrap','--control-transport','acknowledged_batch',
        '--weather','ClearNoon','--seed','2026092601','--spawn-z-offset-m','0',
        '--basic-agent-base-min-distance-m','2','--basic-agent-distance-ratio','.2']


@pytest.mark.parametrize('arm,caps',[('launch',(.13,.15)),('launch_brake',(.13,.02))])
def test_caps_keep_completion_and_reject_training(tmp_path,arm,caps):
    args=aj.parse_args(arguments(tmp_path,arm));config=aj.validate_args(args)
    assert (config.normal_throttle_cap,config.normal_brake_cap)==caps
    assert config.goal_tolerance_m==1 and config.hold_seconds==2 and config.stopped_speed_mps==.1
    assert config.desired_deceleration_mps2==1 and config.target_acceleration_limit_mps2==1.5
    with pytest.raises(adapter.AdapterError):
        adapter._require_native_admission_not_denied({'capture_contract':{'goal_stop_profile':asdict(config)}})


@pytest.mark.parametrize('flag,value',[('--weather','WetNoon'),('--target-speed-kmh','10'),('--seed','1'),
    ('--agent-initialization','before_bootstrap'),('--basic-agent-distance-ratio','.3')])
def test_protocol_changes_refused(tmp_path,flag,value):
    with pytest.raises(ValueError):aj.parse_args(arguments(tmp_path)+[flag,value])


def test_scope_restores_original_on_failure():
    original=aj.base.configuration_from_args
    with pytest.raises(RuntimeError):
        with aj.configuration_scope():
            assert aj.base.configuration_from_args is aj.validate_args
            raise RuntimeError('fixture')
    assert aj.base.configuration_from_args is original


def test_normal_caps_leave_lateral_and_later_emergency_override_intact():
    controller=SimpleNamespace(max_brake=.5,max_throt=.75)
    planner=SimpleNamespace(_vehicle_controller=controller,
        run_step=lambda:SimpleNamespace(throttle=.5,brake=.4,steer=.3))
    metadata=install_normal_brake_cap(planner,.02,.13)
    control=planner.run_step()
    assert (control.throttle,control.brake,control.steer)==(.13,.02,.3)
    control.throttle=0;control.brake=.5  # HH_260906 - BasicAgent applies hazard braking after local-planner control.
    assert control.brake==.5 and metadata['basic_agent_emergency_brake_modified'] is False


def test_staged_launch_handoff_is_latched_without_overriding_emergency():
    velocity=SimpleNamespace(x=0.,y=0.)
    planner=SimpleNamespace(_vehicle_controller=SimpleNamespace(max_brake=.5,max_throt=.75),
        _vehicle=SimpleNamespace(get_velocity=lambda:velocity),
        run_step=lambda:SimpleNamespace(throttle=.4,brake=0.,steer=.3))
    aj.staged_installer(planner,.15,.2)
    assert planner.run_step().throttle==.13
    velocity.x=.5
    assert planner.run_step().throttle==.2
    velocity.x=0
    assert planner.run_step().throttle==.2
    control=planner.run_step();control.brake=.5
    assert control.brake==.5


def test_early_brake_changes_target_envelope_not_quality_or_completion(tmp_path):
    c=aj.validate_args(aj.parse_args(arguments(tmp_path,'staged_early_brake')))
    assert c.normal_throttle_cap==.2 and c.normal_brake_cap==.02 and c.launch_only_cap==.13
    assert c.desired_deceleration_mps2==.4
    assert c.goal_tolerance_m==1 and c.stopped_speed_mps==.1 and c.hold_seconds==2


def test_coast_changes_only_normal_brake_cap(tmp_path):
    early=asdict(aj.validate_args(aj.parse_args(arguments(tmp_path,'staged_early_brake'))))
    coast=asdict(aj.validate_args(aj.parse_args(arguments(tmp_path,'staged_coast'))))
    assert coast.pop('normal_brake_cap')==0 and early.pop('normal_brake_cap')==.02
    early.pop('profile_id');coast.pop('profile_id')
    assert early==coast


def test_earlier_coast_changes_only_distance_speed_envelope(tmp_path):
    coast=asdict(aj.validate_args(aj.parse_args(arguments(tmp_path,'staged_coast'))))
    early=asdict(aj.validate_args(aj.parse_args(arguments(tmp_path,'staged_early_coast'))))
    assert early.pop('desired_deceleration_mps2')==.12 and coast.pop('desired_deceleration_mps2')==.4
    early.pop('profile_id');coast.pop('profile_id')
    assert early==coast


def test_coast_buffer_is_not_a_relaxed_goal_window(tmp_path):
    c=aj.validate_args(aj.parse_args(arguments(tmp_path,'staged_coast_buffer')))
    assert c.stop_buffer_m==2 and c.goal_tolerance_m==1 and c.hold_seconds==2
    assert c.stopped_speed_mps==.1 and c.minimum_tail_seconds==6.5 and c.development_only
