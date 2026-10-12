"""HH_260906 - Exercise optional timing through the complete fake collector, never a real world, GPU, or image codec."""

import copy
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from scripts.e2e import collect_carla_vad_expert as capture
from scripts.e2e import carla_wall_timing as wall


def control(**updates):
    result = dict(throttle=0., brake=0., steer=0., hand_brake=False, reverse=False, gear=0, manual_gear_shift=False)
    result.update(updates)
    return SimpleNamespace(**result)


@pytest.fixture
def fake_run(tmp_path, monkeypatch):
    def execute(name, *, enabled, failure=None):
        root = tmp_path / name
        root.mkdir()
        events, sensors, state = [], [], dict(frame=0, observed=control(), sent=control())
        vector = lambda x=0., y=0., z=0.: SimpleNamespace(x=x, y=y, z=z)
        def transform():
            x = max(0., min(2., state['frame'] - 3))
            return SimpleNamespace(location=vector(x + 1.425), rotation=SimpleNamespace(roll=0., pitch=0., yaw=0.))
        actor = SimpleNamespace(id=42, get_transform=transform, get_velocity=vector,
            get_acceleration=vector, get_angular_velocity=vector)
        ego = SimpleNamespace(id=42, type_id='vehicle.toyota.prius', attributes={},
            get_control=lambda: copy.deepcopy(state['observed']), destroy=lambda: events.append('destroy_ego') or True,
            apply_control=lambda _: pytest.fail('unexpected async transport'),
            get_transform=lambda: pytest.fail('unexpected live kinematics'), get_velocity=lambda: pytest.fail('unexpected live velocity'))
        blueprint = SimpleNamespace(has_attribute=lambda _: False)
        def spawn(*_, **__):
            sensor = SimpleNamespace(id=50 + len(sensors), callback=None)
            sensor.listen = lambda callback: setattr(sensor, 'callback', callback)
            sensor.stop = lambda: None
            sensor.destroy = lambda: events.append('destroy_sensor') or True
            sensors.append(sensor)
            return sensor
        def tick(_timeout):
            if failure == 'world' and state['frame'] == 1:
                raise TimeoutError('synthetic world timeout')
            state['frame'] += 1
            state['observed'] = copy.deepcopy(state['sent'])
            events.append(('tick', state['frame']))
            for index, sensor in enumerate(sensors[:6]):
                if failure == 'camera' and state['frame'] == 2 and index == 0:
                    continue
                sensor.callback(SimpleNamespace(frame=state['frame'], timestamp=state['frame'] * .05))
            return state['frame']
        settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=.05, no_rendering_mode=False,
            substepping=True, max_substep_delta_time=.01, max_substeps=5)
        world = SimpleNamespace(get_actors=lambda: SimpleNamespace(filter=lambda _: []),
            get_map=lambda: SimpleNamespace(name='Town07'), get_settings=lambda: copy.copy(settings),
            get_weather=lambda: 'original', apply_settings=lambda _: events.append('settings'),
            set_weather=lambda _: events.append('weather'), get_blueprint_library=lambda: SimpleNamespace(find=lambda _: blueprint),
            try_spawn_actor=lambda *_: ego, spawn_actor=spawn, tick=tick,
            get_snapshot=lambda: SimpleNamespace(frame=state['frame'], timestamp=SimpleNamespace(elapsed_seconds=state['frame'] * .05),
                find=lambda actor_id: actor if actor_id == 42 else None))
        rpc_count = [0]
        def batch(commands, do_tick):
            assert len(commands) == 1 and commands[0][0] == 42 and do_tick is False
            rpc_count[0] += 1
            events.append(('rpc', rpc_count[0]))
            if failure == 'rpc' and rpc_count[0] == 3:
                raise RuntimeError('synthetic command timeout')
            state['sent'] = copy.deepcopy(commands[0][1])
            return [SimpleNamespace(actor_id=42, error='', has_error=lambda: False)]
        client = SimpleNamespace(get_world=lambda: world, set_timeout=lambda _: None,
            get_server_version=lambda: 'synthetic', get_client_version=lambda: 'synthetic', apply_batch_sync=batch)
        carla = ModuleType('carla')
        carla.Client, carla.VehicleControl, carla.Transform = lambda *_: client, control, transform
        carla.WeatherParameters = SimpleNamespace(ClearNoon='clear')
        carla.AttachmentType = SimpleNamespace(Rigid='rigid')
        carla.command = SimpleNamespace(ApplyVehicleControl=lambda actor_id, command: (actor_id, command))
        monkeypatch.setitem(sys.modules, 'carla', carla)
        for module in ('agents', 'agents.navigation', 'agents.navigation.basic_agent', 'agents.navigation.global_route_planner'):
            monkeypatch.setitem(sys.modules, module, ModuleType(module))
        agent = SimpleNamespace(set_global_plan=lambda _: None, get_local_planner=lambda: SimpleNamespace(_vehicle_controller=object()),
            done=lambda: False, run_step=lambda: control(throttle=.15))
        sys.modules['agents.navigation.basic_agent'].BasicAgent = lambda *_a, **_k: agent
        sys.modules['agents.navigation.global_route_planner'].GlobalRoutePlanner = lambda *_: SimpleNamespace(trace_route=lambda *_: [1, 2])
        monkeypatch.setattr(capture, '_python_class_source_provenance', lambda _: {'synthetic': True})
        monkeypatch.setattr(capture, '_carla_transform', lambda *_: transform())
        monkeypatch.setattr(capture, '_carla_location', lambda *_: vector())
        monkeypatch.setattr(capture, '_configure_camera_blueprint', lambda *_: None)
        monkeypatch.setattr(capture, 'measured_vehicle_fields', lambda *_: {})
        monkeypatch.setattr(capture, '_validate_image_geometry', lambda *_: None)
        def jpeg(_image, output, _quality):
            events.append(('jpeg', output.parent.name, state['frame']))
            if failure == 'jpeg' and state['frame'] == 2 and output.parent.name == capture.MODEL_CAMERA_ORDER[2]:
                raise OSError('synthetic encoding failure')
            output.write_bytes(b'synthetic_not_a_real_jpeg')
        monkeypatch.setattr(capture, '_save_jpeg', jpeg)
        route = {'town': 'Town07', 'start_carla_transform': dict(x=0., y=0., z=0.),
            'goal_carla_transform': dict(x=2., y=0., z=0.),
            'route': [dict(x=0., y=0., distance_m=0., vad_command=3), dict(x=2., y=0., distance_m=2., vad_command=3)]}
        route_path, mapping, calibration = [root / name for name in ('route.json', 'mapping.yaml', 'calibration.yaml')]
        route_path.write_text(json.dumps(route))
        mapping.write_text('synthetic: true\n'); calibration.write_text('synthetic: true\n')
        specs = [capture.CameraSpec(camera, index, 'camera', 1, 1, 90., .1, False, camera, {}, {}, {})
                 for index, camera in enumerate(capture.MODEL_CAMERA_ORDER)]
        monkeypatch.setattr(capture, 'load_route', lambda _: copy.deepcopy(route))
        monkeypatch.setattr(capture, 'load_camera_specs', lambda *_: specs)
        output = root / 'episode'
        arguments = [str(output), str(route_path), '--mapping', str(mapping), '--calibration', str(calibration),
            '--control-transport', 'acknowledged_batch', '--stationary-warmup-sec', '.1', '--stationary-tail-sec', '.1',
            '--goal-tolerance-m', '.2', '--max-duration-sec', '1', '--sensor-timeout-sec', '.01']
        if enabled:
            arguments += ['--wall-timing']
        if failure == 'persist':
            original_open = Path.open
            def bad_open(path, mode='r', *args, **kwargs):
                if path.name == 'wall_timing.jsonl' and mode == 'a':
                    raise OSError('synthetic timing journal failure')
                return original_open(path, mode, *args, **kwargs)
            monkeypatch.setattr(Path, 'open', bad_open)
        error = None
        try:
            capture.run(capture.parse_args(arguments))
        except Exception as caught:
            error = caught
        folder = output if output.exists() else Path(str(output) + '.partial')
        manifest = json.loads((folder / 'manifest.json').read_text())
        rows = [json.loads(line) for line in (folder / 'states.jsonl').read_text().splitlines()]
        cameras = [json.loads(line) for line in (folder / 'camera_frames.jsonl').read_text().splitlines()]
        return SimpleNamespace(folder=folder, events=events, error=error, manifest=manifest, states=rows, cameras=cameras)
    return execute


def test_complete_enabled_and_disabled_runs_preserve_original_state_camera_and_call_order(fake_run):
    disabled = fake_run('disabled', enabled=False)
    enabled = fake_run('enabled', enabled=True)
    assert disabled.error is enabled.error is None
    assert disabled.states == enabled.states and disabled.cameras == enabled.cameras and disabled.events == enabled.events
    assert len(enabled.states) == 6 and len(enabled.cameras) == 3
    assert not list(disabled.folder.glob('wall_timing*'))
    assert 'wall_timing' not in disabled.manifest['capture_contract']
    rows = [json.loads(line) for line in (enabled.folder / 'wall_timing.jsonl').read_text().splitlines()]
    assert len(rows) == 6 and all(row['status'] == 'COMPLETE' for row in rows)
    report = json.loads((enabled.folder / 'wall_timing_summary.json').read_text())
    assert report['status'] == 'COMPLETE_DIAGNOSTIC' and all(report['flags'].values())
    assert report['phase_counts'] == {phase: dict(attempts=2, states=2, camera_anchors=1) for phase in capture.CAPTURE_PHASE_ORDER}
    assert enabled.manifest['result']['wall_timing']['journal_exactly_matches_memory'] is True
    assert enabled.manifest['provenance']['wall_timing_helper_sha256'] == capture.sha256_file(Path(wall.__file__))


@pytest.mark.parametrize('failure,stage,states,cameras', [('world', 'world_tick_snapshot', 0, 0),
    ('camera', 'camera_queue_wait', 1, 0), ('jpeg', 'jpeg_encode_write', 1, 0), ('rpc', 'control_rpc', 1, 1)])
def test_mid_tick_failure_keeps_partial_timing_and_original_failure_and_owned_cleanup(fake_run, failure, stage, states, cameras):
    result = fake_run(failure, enabled=True, failure=failure)
    assert result.error is not None and result.manifest['status'] == 'failed'
    assert len(result.states) == states and len(result.cameras) == cameras
    rows = [json.loads(line) for line in (result.folder / 'wall_timing.jsonl').read_text().splitlines()]
    assert len(rows) == 1 and rows[0]['status'] == 'PARTIAL_OR_FAILED'
    assert rows[0]['stages'][stage]['status'] == 'FAILED'
    assert result.manifest['cleanup']['completed'] is True
    assert result.events.count('destroy_ego') == 1 and result.events.count('destroy_sensor') == 8
    assert result.events.count('settings') == result.events.count('weather') == 2
    assert result.manifest['result']['wall_timing']['status'] == 'PARTIAL_OR_FAILED_DIAGNOSTIC'


def test_timing_journal_failure_does_not_change_capture_but_is_failed_and_recovered(fake_run):
    result = fake_run('persist', enabled=True, failure='persist')
    assert result.error is None and result.manifest['status'] == 'complete' and result.manifest['cleanup']['completed'] is True
    assert (result.folder / 'wall_timing.jsonl').read_bytes() == b''
    recovered = [json.loads(line) for line in (result.folder / 'wall_timing_recovery.jsonl').read_text().splitlines()]
    assert len(recovered) == len(result.states) == 6
    assert result.manifest['result']['wall_timing']['status'] == 'PARTIAL_OR_FAILED_DIAGNOSTIC'
    assert result.manifest['result']['wall_timing']['journal_exactly_matches_memory'] is False


def test_disabled_mode_never_constructs_timing_recorder(fake_run, monkeypatch):
    monkeypatch.setattr(wall, 'WallTimingRecorder', lambda **_: pytest.fail('disabled path imported or constructed timing'))
    assert fake_run('default', enabled=False).error is None


def test_timing_finalization_failure_does_not_skip_cleanup_or_fake_metrics(fake_run, monkeypatch):
    def broken(*args, **kwargs):
        raise OSError('synthetic summary failure')
    monkeypatch.setattr(wall.WallTimingRecorder, 'summarize', broken)
    result = fake_run('summary_failure', enabled=True)
    assert result.error is None and result.manifest['cleanup']['completed'] is True
    assert result.manifest['result']['wall_timing'] == dict(status='FAILED_DIAGNOSTIC', dataset_admission=False, error_type='OSError')
    assert result.events.count('destroy_ego') == 1 and result.events.count('settings') == 2


def test_cli_timing_requires_explicit_boolean_flag():
    assert capture.parse_args(['output', 'route']).wall_timing is False
    assert capture.parse_args(['output', 'route', '--wall-timing']).wall_timing is True
    with pytest.raises(SystemExit):
        capture.parse_args(['output', 'route', '--wall-timing=false'])


def test_disabled_native_collector_matches_frozen_pre_timing_function(fake_run, monkeypatch):
    # HH_260906 - Load only the historical collector function AST; its executed helper/control semantics are unchanged.
    import ast
    import subprocess
    from types import FunctionType
    result = subprocess.run(['git', 'show', '9d3dcab:scripts/e2e/collect_carla_vad_expert.py'],
        cwd=Path(capture.__file__).resolve().parents[2], capture_output=True, text=True, timeout=10)
    if result.returncode:
        pytest.skip('Historical source is unavailable in this shallow checkout; no fetch is attempted.')
    old_function = next(node for node in ast.parse(result.stdout).body if isinstance(node, ast.FunctionDef) and node.name == 'collect_episode')
    current = fake_run('current_default', enabled=False)
    namespace = dict(capture.__dict__)
    exec(compile(ast.Module(body=[old_function], type_ignores=[]), '<frozen-pre-timing-collector>', 'exec'), namespace)
    historical_function = FunctionType(namespace['collect_episode'].__code__, capture.__dict__)
    monkeypatch.setattr(capture, 'collect_episode', historical_function)
    # HH_260906 - Resolve the fresh fake-world helpers through live fixture globals, not a simulator installation.
    historical = fake_run('historical_default', enabled=False)
    assert current.error is historical.error is None
    assert current.states == historical.states and current.cameras == historical.cameras
    assert current.events == historical.events
