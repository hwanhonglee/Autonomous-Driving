# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Moving-task protocol tests; no simulator or changed quality thresholds."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

from scripts.e2e import carla_moving_capture as m
from scripts.e2e import collect_carla_vad_expert as capture
from test_carla_wall_timing_integration import fake_run, control

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path

ROOT=next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
ROUTE=_hh_source_path(ROOT, 'artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town07/catalog/straight/routes/town07/straight/town07_straight_s0000_p00.json')


@pytest.fixture
def declared(tmp_path,monkeypatch):
    for name in m.SOURCE_PATHS:
        path=tmp_path/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes((_hh_source_path(ROOT, name)).read_bytes())
    cohort=tmp_path/m.COHORT_PATH;cohort.parent.mkdir(parents=True,exist_ok=True);cohort.write_bytes((_hh_source_path(ROOT, m.COHORT_PATH)).read_bytes())
    route=tmp_path/'route.json';route.write_bytes(ROUTE.read_bytes())
    mapping=tmp_path/'mapping.yaml';mapping.write_text('mapping: fixture\n')
    calibration=tmp_path/'calibration.yaml';calibration.write_text('calibration: fixture\n')
    monkeypatch.setattr(m,'ROOT',tmp_path)
    output=tmp_path/m.CAPTURE_PATH
    plan=m.build_plan(output,route,mapping,calibration,'2026-09-15T00:00:00Z')
    path=tmp_path/'plan.json';path.write_bytes(m.encoded(plan))
    args=SimpleNamespace(**m.FIXED_ARGS,output=output,route_file=route,mapping=mapping,calibration=calibration,
        moving_task_plan=path,moving_task_plan_sha256=m.sha(path))
    return args,plan,path


def generic_workspace(tmp_path,monkeypatch,slot_id):
    spec=m.slot_spec(slot_id)
    for name in m.SOURCE_PATHS:
        path=tmp_path/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes((_hh_source_path(ROOT, name)).read_bytes())
    cohort=tmp_path/m.COHORT_PATH;cohort.parent.mkdir(parents=True,exist_ok=True);cohort.write_bytes((_hh_source_path(ROOT, m.COHORT_PATH)).read_bytes())
    route=tmp_path/spec['route_path'];route.parent.mkdir(parents=True,exist_ok=True)
    route.write_bytes((_hh_source_path(ROOT, spec['route_path'])).read_bytes())
    mapping=tmp_path/'mapping.yaml';mapping.write_text('mapping: fixture\n')
    calibration=tmp_path/'calibration.yaml';calibration.write_text('calibration: fixture\n')
    monkeypatch.setattr(m,'ROOT',tmp_path)
    return spec,route,mapping,calibration


def camera(frame,timestamp):
    return dict(frame=frame,timestamp=timestamp,capture_phase='driving',camera_order=list(m.CAMERAS),
        images={name:f'{name}/{frame}.jpg' for name in m.CAMERAS},source_timestamps={name:timestamp for name in m.CAMERAS})


def test_exact_pending_plan_and_no_legacy_denial(declared):
    args,plan,path=declared
    assert m.configuration_from_args(args)==plan
    assert 'training_data_approved' not in m.encoded(plan).decode()
    assert 'development_only' not in m.encoded(plan).decode()
    assert plan['admission_status']=='PENDING_QA' and plan['dataset_conversion_performed'] is False
    assert m.configuration_from_args(SimpleNamespace()) is None
    assert capture.moving_capture_module(SimpleNamespace()) is None


def test_legacy_t07_s01_fixed_contract_remains_byte_exact():
    historical=m.read(_hh_source_path(ROOT, m.PLAN_PATH))
    dynamic={'declared_at_utc','output','input_paths','input_sha256','source_sha256'}
    fixed={key:value for key,value in historical.items() if key not in dynamic}
    assert m.encoded(m.plan_contract())==m.encoded(fixed)
    assert m.CAPTURE_PATH=='artifacts/training/2026-09-15/moving_cohort_v1/01_moving_t07_s01/episode'
    assert m.PLAN_PATH=='artifacts/training/2026-09-15/moving_cohort_v1/t07_s01_plan.json'
    assert m.ROUTE_SHA=='804dc69cec5ebbc2edba70946694f312c77328b5cb5d7260b096af0775f34848'


@pytest.mark.parametrize('slot_id',m.RECOVERY_SLOT_IDS)
def test_five_sealed_train_straight_slots_declare_create_only(tmp_path,monkeypatch,slot_id):
    spec,route,mapping,calibration=generic_workspace(tmp_path,monkeypatch,slot_id)
    target=tmp_path/spec['plan_path'];output=tmp_path/spec['capture_path']
    argv=['--declare',str(target),'--slot-id',slot_id,'--target-speed-kmh','15',
        '--mapping',str(mapping),'--calibration',str(calibration)]
    assert m.main(argv)==0
    assert target.is_file() and not output.exists() and not output.parent.exists()
    plan=m.read(target)
    assert plan['schema']=='carla.moving_train_straight_capture_plan.v1'
    assert plan['cohort_slot_id']==slot_id and plan['dataset_split_intent']=='train'
    assert plan['cohort_plan']==dict(path=m.COHORT_PATH,sha256=m.COHORT_SHA)
    assert plan['route_group_independence']=='NOT_CERTIFIED'
    assert plan['corridor_conflicts']==spec['corridor_conflicts']
    assert plan['full_30kph_route_budget_available'] is False
    assert plan['task_id']==spec['task_id'] and plan['case_id']==spec['case_id']
    assert plan['town']==spec['town'] and plan['scenario']=='straight'
    assert plan['route_sha256']==spec['route_sha256']==m.sha(route)
    assert plan['fixed_arguments']['target_speed_kmh']==15.
    assert plan['fixed_arguments']['weather']==spec['weather']
    assert plan['fixed_arguments']['seed']==spec['fixed_seed']
    assert plan['admission_status']=='PENDING_QA' and plan['dataset_conversion_performed'] is False
    args=SimpleNamespace(**plan['fixed_arguments'],output=output,route_file=route,mapping=mapping,
        calibration=calibration,moving_task_plan=target,moving_task_plan_sha256=m.sha(target))
    assert m.configuration_from_args(args)==plan
    with pytest.raises(ValueError,match='fresh fixed'):
        m.main(argv)


def test_c_track_straight_slots_disclose_their_nonindependent_shared_corridor():
    first=m.plan_contract('CT-S01')['corridor_conflicts']
    second=m.plan_contract('CT-S02')['corridor_conflicts']
    first_pair=next(row for row in first if row['other_slot_id']=='CT-S02')
    second_pair=next(row for row in second if row['other_slot_id']=='CT-S01')
    assert first_pair['shared_road_sections']==second_pair['shared_road_sections']
    assert len(first_pair['shared_road_sections'])==7


@pytest.mark.parametrize('slot_id',['T07-S01','V03-S01','T04-S01','T15-S01','T07-L01'])
def test_explicit_legacy_dev_test_and_nonstraight_slots_are_refused_without_writes(tmp_path,monkeypatch,slot_id):
    monkeypatch.setattr(m,'ROOT',tmp_path)
    target=tmp_path/'candidate.json'
    with pytest.raises(ValueError,match='five uncollected'):
        m.main(['--declare',str(target),'--slot-id',slot_id])
    assert not target.exists()


def test_30kph_declaration_is_explicitly_refused_before_writes(tmp_path,monkeypatch):
    monkeypatch.setattr(m,'ROOT',tmp_path)
    target=tmp_path/'candidate.json'
    with pytest.raises(ValueError,match='30 kph.*anchor-span budget'):
        m.main(['--declare',str(target),'--slot-id','T07-S02','--target-speed-kmh','30'])
    assert not target.exists()
    with pytest.raises(ValueError,match='30 kph'):
        m.plan_contract('T07-S02',30.)


def test_generic_configuration_rejects_changed_cohort_and_cross_slot_route(tmp_path,monkeypatch):
    slot_id='CT-S01';spec,route,mapping,calibration=generic_workspace(tmp_path,monkeypatch,slot_id)
    output=tmp_path/spec['capture_path']
    wrong=tmp_path/'wrong_route.json';wrong.write_bytes(ROUTE.read_bytes())
    with pytest.raises(ValueError,match='wrong moving-task route'):
        m.build_plan(output,wrong,mapping,calibration,'2026-09-21T00:00:00Z',slot_id=slot_id)
    plan=m.build_plan(output,route,mapping,calibration,'2026-09-21T00:00:00Z',slot_id=slot_id)
    path=tmp_path/spec['plan_path'];path.write_bytes(m.encoded(plan))
    args=SimpleNamespace(**plan['fixed_arguments'],output=output,route_file=route,mapping=mapping,
        calibration=calibration,moving_task_plan=path,moving_task_plan_sha256=m.sha(path))
    changed=deepcopy(plan);changed['cohort_slot_id']='T07-S02';path.write_bytes(m.encoded(changed))
    args.moving_task_plan_sha256=m.sha(path)
    with pytest.raises(ValueError,match='fixed moving-task contract'):
        m.configuration_from_args(args)
    path.write_bytes(m.encoded(plan));args.moving_task_plan_sha256=m.sha(path)
    (tmp_path/m.COHORT_PATH).write_text('{}')
    with pytest.raises(ValueError,match='cohort changed'):
        m.configuration_from_args(args)


@pytest.mark.parametrize('field,value',[
    ('target_speed_kmh',15.1),('physics_hz',10.),('capture_hz',5.),('control_transport','legacy_async'),
    ('agent_initialization','before_bootstrap'),('goal_stop_profile','comfortable_v9_terminal_feedback'),
    ('physics_substep_profile','fine_5ms'),('stationary_warmup_sec',3.),('stationary_tail_sec',6.4),
    ('max_duration_sec',121.),('allow_map_load',True),('weather','CloudyNoon'),('low_speed_steering_mode','zero')])
def test_different_settings_rejected_before_world(declared,field,value):
    args,_,_=declared;setattr(args,field,value)
    with pytest.raises(ValueError):m.configuration_from_args(args)


@pytest.mark.parametrize('mutation',['source','input','route','plan','future','output','missingflag','symlink'])
def test_source_input_plan_and_paths_fail_closed(declared,mutation):
    args,plan,path=declared
    if mutation=='source':(_hh_source_path(m.ROOT, m.SOURCE_PATHS[0])).write_text('changed')
    elif mutation=='input':args.mapping.write_text('changed')
    elif mutation=='route':args.route_file.write_text('{}')
    elif mutation=='plan':path.write_text('{}')
    elif mutation=='future':
        plan['required_future_support_ns']-=1;path.write_bytes(m.encoded(plan));args.moving_task_plan_sha256=m.sha(path)
    elif mutation=='output':args.output=args.output.parent/'other'
    elif mutation=='missingflag':args.moving_task_plan_sha256=None
    else:
        link=path.parent/'link';link.symlink_to(path);args.moving_task_plan=link
    with pytest.raises(ValueError):m.configuration_from_args(args)


def test_complete_camera_window_requires_full_actual_support_then_tail():
    task=m.MovingTask(m.plan_contract());states=[];cameras=[]
    points=[dict(x=0.,y=0.,distance_m=0.),dict(x=250.,y=0.,distance_m=250.)]
    for frame in range(1,1000):
        timestamp=frame*.05;x=max(0.,4*(timestamp-1.))
        # HH_260906 - Use the identical declared projection, retaining its actual arithmetic rounding.
        progress=m.measured_progress(points,x,0.)
        ledger,reason=task.observe(frame=frame,timestamp=timestamp,phase='driving',progress=progress,
            base_stop_reason=None,event_count=0)
        ledger.update(original_termination_reason=None,observed_event_count=0)
        states.append(dict(frame=frame,timestamp=timestamp,capture_phase='driving',x=x,y=0.,moving_task=ledger))
        if frame%2==0:
            item=camera(frame,timestamp);task.saved_camera(item);cameras.append(item)
        if reason is not None:
            assert reason=='moving_task_future_support_complete';break
    assert task.last['timestamp_ns']-task.first['timestamp_ns']>=30_000_000_000
    assert task.closed['measured_route_progress_m']>165
    assert task.support['timestamp_ns']>=task.last['timestamp_ns']+6_400_000_000
    assert task.result(tail_states=129)['task_complete'] is False
    result=task.result(tail_states=130,goal_reached=False)
    assert result['task_complete'] is True and result['actual_catalog_goal_reached'] is False
    replay=m.reconstruct(m.plan_contract(),states,cameras,points)
    assert replay.selected==task.selected and replay.support==task.support
    assert all(not c['moving_task_selected'] for c in cameras if c['moving_task_measured_route_progress_m']>165)
    bad=deepcopy(states);bad[300]['x']+=.1
    with pytest.raises(ValueError,match='immutable pose'):m.reconstruct(m.plan_contract(),bad,cameras,points)
    bad_cameras=deepcopy(cameras);bad_cameras[-1]['frame']=99999
    with pytest.raises(ValueError,match='no retained native'):m.reconstruct(m.plan_contract(),states,bad_cameras,points)


def test_window_never_reopens_and_fast_window_never_passes():
    task=m.MovingTask(m.plan_contract())
    for frame,progress in enumerate((29.,31.,166.,164.),1):
        ledger,reason=task.observe(frame=frame,timestamp=frame*.05,phase='driving',progress=progress,
            base_stop_reason=None,event_count=0)
        item=camera(frame,frame*.05);task.saved_camera(item)
    assert task.failure=='moving_task_selected_span_under_30_seconds'
    assert task.closed['frame']==3 and len(task.selected)==1
    assert not ledger['camera_selection_eligible'] and not item['moving_task_selected']
    assert not task.result(tail_states=130)['task_complete']


def test_decreasing_only_after_first_saved_anchor_and_future_never_bridged():
    task=m.MovingTask(m.plan_contract())
    for frame,progress in enumerate((1.,.9,30.,30.1),1):
        task.observe(frame=frame,timestamp=frame*.05,phase='driving',progress=progress,base_stop_reason=None,event_count=0)
    assert task.failure is None
    task.saved_camera(camera(4,.2))
    task.observe(frame=5,timestamp=.25,phase='driving',progress=30.099,base_stop_reason=None,event_count=0)
    assert task.failure=='moving_task_decreasing_progress_or_reentry'
    other=m.MovingTask(m.plan_contract())
    other.observe(frame=1,timestamp=.05,phase='driving',progress=0.,base_stop_reason=None,event_count=0)
    other.observe(frame=2,timestamp=.2,phase='driving',progress=0.,base_stop_reason=None,event_count=0)
    assert other.failure=='moving_task_native_timestamp_cadence_failure'


@pytest.mark.parametrize('base,event,reason',[(None,1,'moving_task_observed_collision_or_lane_event'),
    ('catalog_goal_reached',0,'moving_task_catalog_or_agent_end_before_support')])
def test_events_and_original_termination_abort_without_trimming_camera(base,event,reason):
    task=m.MovingTask(m.plan_contract())
    ledger,stop=task.observe(frame=1,timestamp=.05,phase='driving',progress=35.,base_stop_reason=base,event_count=event)
    item=camera(1,.05);task.saved_camera(item)
    assert stop==reason and task.result(tail_states=130)['task_complete'] is False
    assert item['moving_task_selected'] is True and len(task.selected)==1


def test_exact_eleven_field_physics_readback():
    settings=SimpleNamespace(synchronous_mode=False,no_rendering_mode=False,substepping=True,
        deterministic_ragdolls=True,spectator_as_ego=True,fixed_delta_seconds=None,max_substep_delta_time=.01,
        max_culling_distance=0.,tile_stream_distance=3000.,actor_active_distance=2000.,max_substeps=10)
    record=m.validate_original_physics(settings);assert len(record['before'])==11
    settings.synchronous_mode=True;settings.fixed_delta_seconds=.05
    m.validate_applied_physics(record,settings);assert record['status']=='PASS'
    settings.max_substeps=5
    with pytest.raises(ValueError):m.validate_original_physics(settings)


def test_existing_afterbootstrap_default_is_not_rewritten():
    assert capture.parse_args(['output','route']).moving_task_plan is None
    with pytest.raises(SystemExit):capture.parse_args(['output','route','--moving-task-plan','missing'])
    assert m.measured_progress([dict(x=0.,y=0.,distance_m=0.),dict(x=10.,y=0.,distance_m=10.)],2.,1.)==2.


def test_moving_progress_is_planar_arc_not_serialized_three_dimensional_arc():
    points=[dict(x=0.,y=0.,z=0.,distance_m=0.),dict(x=3.,y=4.,z=12.,distance_m=13.),
        dict(x=6.,y=8.,z=24.,distance_m=26.)]
    assert m.measured_progress(points,3.,4.)==5.
    assert m.measured_progress(points,4.5,6.)==7.5


@pytest.mark.parametrize('backtrack',[False,True])
def test_full_fake_collector_goal_false_support_then_tail_or_honest_failure(fake_run,monkeypatch,backtrack):
    # HH_260906 - Simulated poses/1-pixel files test orchestration only; strict source/CLI preflight is tested independently above.
    active={};original_parse=capture.parse_args
    route=dict(town='Town07',start_carla_transform=dict(x=0.,y=0.,z=0.),goal_carla_transform=dict(x=250.,y=0.,z=0.),
        route=[dict(x=0.,y=0.,distance_m=0.,vad_command=3),dict(x=250.,y=0.,distance_m=250.,vad_command=3)])
    def parse(argv):
        args=original_parse(argv)
        for key,value in m.FIXED_ARGS.items():setattr(args,key,value)
        path=Path(argv[0]).parent/'moving_plan.json';path.write_bytes(m.encoded(m.plan_contract()))
        args.moving_task_plan=path;args.moving_task_plan_sha256=m.sha(path)
        return args
    monkeypatch.setattr(capture,'parse_args',parse)
    def configure(args):
        if not active:
            active['installed']=True
            world=sys.modules['carla'].Client('',0).get_world()
            current=dict(synchronous_mode=False,no_rendering_mode=False,substepping=True,deterministic_ragdolls=True,
                spectator_as_ego=True,fixed_delta_seconds=None,max_substep_delta_time=.01,max_culling_distance=0.,
                tile_stream_distance=3000.,actor_active_distance=2000.,max_substeps=10)
            original_apply=world.apply_settings
            def apply(settings):current.update(vars(settings));original_apply(settings)
            world.get_settings=lambda:SimpleNamespace(**current);world.apply_settings=apply
            actor=world.get_snapshot().find(42)
            def transform():
                frame=world.get_snapshot().frame;x=max(0.,.2*(frame-71))
                if backtrack and frame==500:x-=1.
                return SimpleNamespace(location=SimpleNamespace(x=x+1.425,y=0.,z=0.),
                    rotation=SimpleNamespace(roll=0.,pitch=0.,yaw=0.))
            actor.get_transform=transform
            planner=SimpleNamespace(_vehicle_controller=SimpleNamespace(past_steering=0.))
            agent=SimpleNamespace(set_global_plan=lambda _:None,get_local_planner=lambda:planner,
                done=lambda:False,run_step=lambda:control(throttle=.15))
            sys.modules['agents.navigation.basic_agent'].BasicAgent=lambda *_a,**_k:agent
            monkeypatch.setattr(capture,'load_route',lambda _:deepcopy(route))
        return m.plan_contract()
    monkeypatch.setattr(m,'configuration_from_args',configure)
    result=fake_run('moving_failure' if backtrack else 'moving_complete',enabled=False)
    assert result.manifest['result']['goal_reached'] is False
    assert result.manifest['cleanup']['completed'] is True
    assert result.manifest['capture_contract']['moving_task']['physics_settings']['status']=='PASS'
    assert result.manifest['capture_contract']['agent_initialization']['admission_status']=='PENDING_QA'
    assert 'training_data_approved' not in json_text(result.manifest)
    assert result.manifest['result']['admission_status']=='PENDING_QA'
    proof=result.manifest['result']['moving_task']
    if backtrack:
        assert result.error is not None and result.manifest['status']=='failed'
        assert not proof['task_complete'] and proof['failure_reason']=='moving_task_decreasing_progress_or_reentry'
        witness=next(r for r in result.cameras if r['frame']==500)
        assert witness['moving_task_selected'] is True
        assert not any(row['capture_phase']=='stationary_tail' for row in result.states)
    else:
        assert result.error is None and result.manifest['status']=='complete'
        assert proof['task_complete'] and proof['observed_tail_state_count']==130
        assert proof['future_support']['timestamp_ns']-proof['last_selected_camera']['timestamp_ns']>=6_400_000_000
        final_driving=[r for r in result.states if r['capture_phase']=='driving'][-1]
        assert final_driving['next_control']['brake']==1.
        assert final_driving['moving_task']['future_support_complete'] is True
        assert len([r for r in result.states if r['capture_phase']=='stationary_warmup'])==70
        assert proof['selected_camera_anchor_count']==sum(c['moving_task_selected'] for c in result.cameras)


def json_text(value):
    import json
    return json.dumps(value,sort_keys=True)


def test_reproducible_create_only_declaration_cli(declared):
    args,_,_=declared
    target=_hh_source_path(m.ROOT, m.PLAN_PATH)
    argv=['--declare',str(target),'--capture-output',str(args.output),'--route-file',str(args.route_file),
        '--mapping',str(args.mapping),'--calibration',str(args.calibration)]
    assert m.main(argv)==0
    assert m.read(target)['output']==m.CAPTURE_PATH
    assert m.read(target)['admission_status']=='PENDING_QA'
    with pytest.raises(ValueError,match='fresh fixed'):m.main(argv)
