"""HH_260906 - Prospective moving-task capture support; no dataset admission or label changes."""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Any

ROOT=Path(__file__).resolve().parents[2]
CAPTURE_PATH='artifacts/training/2026-09-15/moving_cohort_v1/01_moving_t07_s01/episode'
PLAN_PATH='artifacts/training/2026-09-15/moving_cohort_v1/t07_s01_plan.json'
ROUTE_PATH='artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town07/catalog/straight/routes/town07/straight/town07_straight_s0000_p00.json'
ROUTE_SHA='804dc69cec5ebbc2edba70946694f312c77328b5cb5d7260b096af0775f34848'
COHORT_PATH='artifacts/training/2026-09-15/moving_cohort_v1/cohort.json'
COHORT_SHA='32765291218e35ce1ea002db68f8b2e0b694889252c82e5e38bc8cde8f271863'
SOURCE_PATHS=('scripts/e2e/collect_carla_vad_expert.py','scripts/e2e/carla_moving_capture.py',
    'scripts/e2e/carla_goal_stop_profile.py','scripts/e2e/carla_physics_substeps.py')
CAMERAS=('CAM_FRONT','CAM_BACK','CAM_FRONT_LEFT','CAM_BACK_LEFT','CAM_FRONT_RIGHT','CAM_BACK_RIGHT')
FIXED_ARGS=dict(host='127.0.0.1',port=2100,physics_hz=20.,capture_hz=10.,target_speed_kmh=15.,
    max_duration_sec=120.,stationary_warmup_sec=3.5,stationary_tail_sec=6.5,
    control_transport='acknowledged_batch',agent_initialization='after_bootstrap',goal_stop_profile='disabled',
    physics_substep_profile='inherited',low_speed_steering_mode=None,hold_brake_mode=None,terminal_approach_mode=None,
    weather='ClearNoon',seed=0,vehicle_type='vehicle.toyota.prius',role_name='autoware_e2e_expert',
    wheelbase_m=2.85,spawn_z_offset_m=2.,goal_tolerance_m=2.5,command_lookahead_m=2.,command_exit_lookahead_m=2.5,
    allow_map_load=False,allow_stopped_steering=False,wall_timing=False,jpeg_quality=95,timeout=20.,sensor_timeout_sec=5.,
    basic_agent_base_min_distance_m=3.,basic_agent_distance_ratio=.5,basic_agent_lateral_kp=1.95,
    basic_agent_lateral_ki=.05,basic_agent_lateral_kd=.2,
    basic_agent_max_steering=.8,basic_agent_lane_offset_m=0.)
DEFAULT_SLOT_ID='T07-S01'
RECOVERY_SLOT_IDS=('T07-S02','CT-S01','CT-S02','T01-S01','T01-S02')
SLOT_SPECS={
    'T07-S01':dict(task_id='t07_s01_moving_15kph_v1',case_id='01_moving_t07_s01',town='Town07',
        weather='ClearNoon',cohort_seed=101,fixed_seed=0,route_id='town07_straight_s0000_p00',
        route_path=ROUTE_PATH,route_sha256=ROUTE_SHA,capture_path=CAPTURE_PATH,plan_path=PLAN_PATH),
    'T07-S02':dict(task_id='t07_s02_moving_15kph_v1',case_id='02_moving_t07_s02',town='Town07',
        weather='CloudyNoon',cohort_seed=102,fixed_seed=102,route_id='town07_straight_s0000_p02',
        corridor_conflicts=[dict(other_slot_id='T07-R02',other_split='train',shared_road_sections=[[14,0]])],
        route_path='artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town07/catalog/straight/routes/town07/straight/town07_straight_s0000_p02.json',
        route_sha256='099d224e1c810c5d58395deafc3d331fd7fdd683892c43a7bdd9664bb19f1b46',
        capture_path='artifacts/training/2026-09-15/moving_cohort_v1/02_moving_t07_s02/episode',
        plan_path='artifacts/training/2026-09-15/moving_cohort_v1/t07_s02_plan.json'),
    'CT-S01':dict(task_id='ct_s01_moving_15kph_v1',case_id='03_moving_ct_s01',town='C_track_1_0_7',
        weather='ClearNoon',cohort_seed=101,fixed_seed=101,route_id='c_track_1_0_7_straight_s0000_p06',
        corridor_conflicts=[
            dict(other_slot_id='CT-S02',other_split='train',shared_road_sections=[[40,0],[52,0],[56,0],[79,0],[95,0],[116,0],[117,0]]),
            dict(other_slot_id='CT-R02',other_split='train',shared_road_sections=[[52,0],[56,0]])],
        route_path='artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/c_track_1_0_7/catalog/straight/routes/c_track_1_0_7/straight/c_track_1_0_7_straight_s0000_p06.json',
        route_sha256='c0f57c12a7a74dbf0d4269f5cda976a38b3601b9dffee23d4b14938c175c18ba',
        capture_path='artifacts/training/2026-09-15/moving_cohort_v1/03_moving_ct_s01/episode',
        plan_path='artifacts/training/2026-09-15/moving_cohort_v1/ct_s01_plan.json'),
    'CT-S02':dict(task_id='ct_s02_moving_15kph_v1',case_id='04_moving_ct_s02',town='C_track_1_0_7',
        weather='CloudyNoon',cohort_seed=102,fixed_seed=102,route_id='c_track_1_0_7_straight_s0000_p03',
        corridor_conflicts=[
            dict(other_slot_id='CT-S01',other_split='train',shared_road_sections=[[40,0],[52,0],[56,0],[79,0],[95,0],[116,0],[117,0]]),
            dict(other_slot_id='CT-R02',other_split='train',shared_road_sections=[[52,0],[56,0],[1112,0]])],
        route_path='artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/c_track_1_0_7/catalog/straight/routes/c_track_1_0_7/straight/c_track_1_0_7_straight_s0000_p03.json',
        route_sha256='cb8632fa48598389a948fe552826ab05a0d2c737126dd3f39e4dde12d5e3b70e',
        capture_path='artifacts/training/2026-09-15/moving_cohort_v1/04_moving_ct_s02/episode',
        plan_path='artifacts/training/2026-09-15/moving_cohort_v1/ct_s02_plan.json'),
    'T01-S01':dict(task_id='t01_s01_moving_15kph_v1',case_id='05_moving_t01_s01',town='Town01',
        weather='ClearNoon',cohort_seed=101,fixed_seed=101,route_id='town01_straight_s0000_p02',
        corridor_conflicts=[],
        route_path='artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town01/catalog/straight/routes/town01/straight/town01_straight_s0000_p02.json',
        route_sha256='cc39e43ba6fdc37a1c9261d015e9053b927eaaf4c3d71ff4debb1f50e53baf12',
        capture_path='artifacts/training/2026-09-15/moving_cohort_v1/05_moving_t01_s01/episode',
        plan_path='artifacts/training/2026-09-15/moving_cohort_v1/t01_s01_plan.json'),
    'T01-S02':dict(task_id='t01_s02_moving_15kph_v1',case_id='06_moving_t01_s02',town='Town01',
        weather='CloudyNoon',cohort_seed=102,fixed_seed=102,route_id='town01_straight_s0000_p01',
        corridor_conflicts=[],
        route_path='artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town01/catalog/straight/routes/town01/straight/town01_straight_s0000_p01.json',
        route_sha256='913b44d56755d226f3e2957814924b021ad6913400a880c8aac19bb934fa4194',
        capture_path='artifacts/training/2026-09-15/moving_cohort_v1/06_moving_t01_s02/episode',
        plan_path='artifacts/training/2026-09-15/moving_cohort_v1/t01_s02_plan.json'),
}


def require(value,message):
    if not value:raise ValueError(message)


def canonical(path):
    path=Path(path).absolute()
    require('..' not in path.parts and path==path.resolve() and not any(p.is_symlink() for p in (path,*path.parents)),
        'moving-task paths must be canonical and nonsymlink')
    return path


def sha(path):
    return hashlib.sha256(canonical(path).read_bytes()).hexdigest()


def encoded(value):
    return (json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()


def read(path):
    def pairs(items):
        result={}
        for key,value in items:
            require(key not in result,'duplicate moving-plan JSON key');result[key]=value
        return result
    result=json.loads(canonical(path).read_bytes(),object_pairs_hook=pairs,
        parse_constant=lambda _:require(False,'nonfinite moving-plan JSON'))
    encoded(result)
    return result


def pending_scope():
    # HH_260906 - A fresh candidate has no Boolean training-admission marker; old explicit denials are never removed.
    return dict(admission_status='PENDING_QA',dataset_conversion_performed=False,vehicle_control_approved=False,
        qualification_notice='Only a future explicit, source-bound qualification can admit a newly collected moving task; no current adapter or training approval.')


def slot_spec(slot_id=DEFAULT_SLOT_ID):
    require(type(slot_id) is str and slot_id in SLOT_SPECS,'moving slot must be one sealed TRAIN straight slot')
    return SLOT_SPECS[slot_id]


def fixed_args_for_slot(slot_id=DEFAULT_SLOT_ID):
    spec=slot_spec(slot_id)
    return dict(FIXED_ARGS,weather=spec['weather'],seed=spec['fixed_seed'])


def require_15kph_speed(value):
    require(type(value) in (int,float) and not isinstance(value,bool) and math.isfinite(value),
        'moving declaration speed must be finite')
    if float(value)==30.:
        raise ValueError('30 kph moving declaration refused: every sealed straight route lacks the required 30-second anchor-span budget')
    require(float(value)==15.,'only the sealed 15 kph moving declaration is supported')
    return 15.


def verify_cohort_slot(slot_id):
    # HH_260906 - Bind only the six sealed TRAIN straight rows without opening DEV or release-TEST route payloads.
    spec=slot_spec(slot_id);path=canonical(ROOT/COHORT_PATH)
    require(path.is_file() and path.stat().st_size<=16*1024*1024 and sha(path)==COHORT_SHA,
        'sealed moving cohort changed or is missing')
    cohort=read(path)
    require(cohort.get('schema')=='portable_e2e.moving_cohort_plan.v1'
        and cohort.get('status')=='PROVISIONAL_NON_EXECUTING_COHORT'
        and cohort.get('slot_count')==24 and cohort.get('route_bound_count')==24
        and cohort.get('geometry_budget_available_slots')==7
        and cohort.get('full_30kph_geometry_budget_available_slots')==0,
        'sealed moving cohort header differs')
    slots=cohort.get('slots');require(type(slots) is list,'sealed moving cohort slots differ')
    matches=[row for row in slots if type(row) is dict and row.get('slot_id')==slot_id]
    require(len(matches)==1,'sealed moving slot identity differs')
    row=matches[0];route=row.get('route',{});horizon=route.get('horizon',{});full30=route.get('full_30kph_horizon',{})
    recorded_route=route.get('route_path')
    require(row.get('split')=='train' and row.get('scenario')=='straight'
        and row.get('weather')==spec['weather'] and row.get('traffic_seed')==spec['cohort_seed']
        and row.get('execution_allowed') is False and row.get('training_data_approved') is False
        and row.get('route_group_independence')=='NOT_CERTIFIED'
        and (slot_id==DEFAULT_SLOT_ID or row.get('corridor_conflicts')==spec['corridor_conflicts'])
        and route.get('canonical_name')==spec['town'] and route.get('scenario')=='straight'
        and route.get('route_id')==spec['route_id'] and route.get('route_sha256')==spec['route_sha256']
        and type(recorded_route) is str and recorded_route.endswith('/'+spec['route_path'])
        and horizon.get('status')=='CONSERVATIVE_ARC_BUDGET_AVAILABLE'
        and horizon.get('reference_speed_ceiling_kph')==15.
        and horizon.get('minimum_reference_anchor_span_seconds')==30.
        and horizon.get('reference_anchor_span_seconds',0.)>=30.
        and full30.get('status')=='ROUTE_EXTENSION_OR_SPEED_PLAN_NEEDED'
        and full30.get('reference_speed_ceiling_kph')==30.
        and full30.get('reference_anchor_span_seconds',30.)<30.
        and 'SHORT_ANCHOR_SPAN_AT_DECLARED_SPEED_REFERENCE' in full30.get('reasons',()),
        'sealed TRAIN straight slot contract differs')
    return row


def slot_id_from_plan(plan):
    matches=[slot_id for slot_id,spec in SLOT_SPECS.items() if plan.get('task_id')==spec['task_id']]
    require(len(matches)==1,'moving plan task is not one sealed TRAIN straight slot')
    return matches[0]


def plan_contract(slot_id=DEFAULT_SLOT_ID,target_speed_kmh=15.):
    require_15kph_speed(target_speed_kmh);spec=slot_spec(slot_id)
    contract=dict(schema='carla.moving_task_capture_plan.v1',task_id=spec['task_id'],
        case_id=spec['case_id'],town=spec['town'],scenario='straight',route_sha256=spec['route_sha256'],
        fixed_arguments=fixed_args_for_slot(slot_id),selected_progress_m=[30.,165.],minimum_selected_camera_span_ns=30_000_000_000,
        required_future_support_ns=6_400_000_000,maximum_attempts=1,automatic_retry=False,owner_wall_timeout_sec=300,
        native_cadence_ns=50_000_000,native_timestamp_rounding_tolerance_ns=1,
        selected_anchor_timestamp='Latest of the six saved camera source timestamps, individually rounded to nanoseconds.',
        selection_rule='All saved six-camera driving anchors in the inclusive measured route-progress interval; permanently close on first native progress >165m.',
        progress_measurement='Unclamped nearest-segment projection of immutable ROS base_link XY onto ROS route XY; arc is cumulative hypot(dx,dy), not serialized XYZ distance_m. Original monotone collector progress remains unchanged.',
        decreasing_progress_policy='Reject decreases from first selected camera anchor through final moving support; pre-entry launch and post-support brake tail remain fully retained.',
        raw_outside_selection_retained=True,goal_completion_required=False,catalog_goal_success_claim=False,
        stop_or_dwell_success_claim=False,original_qa_or_runtime_gate_changed=False,**pending_scope())
    if slot_id in RECOVERY_SLOT_IDS:
        contract.update(schema='carla.moving_train_straight_capture_plan.v1',cohort_slot_id=slot_id,
            dataset_split_intent='train',route_id=spec['route_id'],
            cohort_plan=dict(path=COHORT_PATH,sha256=COHORT_SHA),
            route_group_independence='NOT_CERTIFIED',corridor_conflicts=spec['corridor_conflicts'],
            full_30kph_route_budget_available=False)
    return contract


def build_plan(output,route_path,mapping_path,calibration_path,declared_at_utc=None,*,slot_id=DEFAULT_SLOT_ID,
        target_speed_kmh=15.):
    require_15kph_speed(target_speed_kmh);spec=slot_spec(slot_id);verify_cohort_slot(slot_id)
    paths={k:canonical(v) for k,v in dict(route_file=route_path,mapping=mapping_path,calibration=calibration_path).items()}
    target=canonical(output)
    require(target==ROOT/spec['capture_path'],'moving output must be the sealed slot capture case')
    require(sha(paths['route_file'])==spec['route_sha256'],'wrong moving-task route')
    route=read(paths['route_file']);require(route['town']==spec['town'] and route['scenario']=='straight','wrong route identity')
    return plan_contract(slot_id,target_speed_kmh)|dict(declared_at_utc=declared_at_utc or datetime.now(timezone.utc).isoformat(),
        output=target.relative_to(ROOT).as_posix(),input_paths={k:p.relative_to(ROOT).as_posix() for k,p in paths.items()},
        input_sha256={k:sha(p) for k,p in paths.items()},source_sha256={n:sha(ROOT/n) for n in SOURCE_PATHS})


def configuration_from_args(args:Any):
    path=getattr(args,'moving_task_plan',None);expected=getattr(args,'moving_task_plan_sha256',None)
    if path is None and expected is None:return None
    require(path is not None and isinstance(expected,str) and len(expected)==64
        and all(c in '0123456789abcdef' for c in expected),'moving task requires paired plan/path SHA flags')
    path=canonical(path);require(sha(path)==expected,'moving task prospective plan SHA differs')
    plan=read(path);slot_id=slot_id_from_plan(plan);verify_cohort_slot(slot_id);spec=slot_spec(slot_id);fixed=plan_contract(slot_id)
    require(set(plan)==set(fixed)|{'declared_at_utc','output','input_paths','input_sha256','source_sha256'},'moving plan fields differ')
    require(encoded({k:plan[k] for k in fixed})==encoded(fixed),'fixed moving-task contract changed')
    stamp=datetime.fromisoformat(plan['declared_at_utc'].replace('Z','+00:00'))
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds()==0 and stamp<=datetime.now(timezone.utc),
        'prospective UTC moving declaration required')
    for key,expected_value in fixed['fixed_arguments'].items():
        require(getattr(args,key,None)==expected_value and (type(expected_value) is not bool or type(getattr(args,key,None)) is bool),
            f'moving task requires {key}={expected_value}')
    require(plan['output']==spec['capture_path'] and canonical(args.output)==canonical(ROOT/plan['output']) and
        canonical(args.output).is_relative_to(ROOT/'artifacts/training/2026-09-15/moving_cohort_v1'),'moving output differs from plan')
    require(not path.is_relative_to(canonical(args.output)) and not path.is_relative_to(Path(str(canonical(args.output))+'.partial')),
        'moving plan must be outside capture output')
    require(set(plan['source_sha256'])==set(SOURCE_PATHS),'moving source closure changed')
    for name,expected_hash in plan['source_sha256'].items():require(sha(ROOT/name)==expected_hash,'moving execution source changed: '+name)
    require(set(plan['input_paths'])==set(plan['input_sha256'])=={'route_file','mapping','calibration'},'moving input closure changed')
    for key in plan['input_paths']:
        source=canonical(ROOT/plan['input_paths'][key])
        require(source.is_relative_to(ROOT) and canonical(getattr(args,key))==source
            and sha(source)==plan['input_sha256'][key],'moving input differs: '+key)
    require(plan['input_sha256']['route_file']==spec['route_sha256'],'moving route pin changed')
    return plan


def physics_snapshot(settings):
    if __package__:from .carla_physics_substeps import settings_snapshot
    else:from carla_physics_substeps import settings_snapshot
    return settings_snapshot(settings)


def validate_original_physics(settings):
    before=physics_snapshot(settings)
    require(before['substepping'] is True and before['max_substep_delta_time']==.01
        and before['max_substeps']==10 and before['no_rendering_mode'] is False,'moving original physics must be true/.01/10 with rendering')
    return dict(status='BEFORE_VALIDATED',before=before,requested=dict(before,synchronous_mode=True,fixed_delta_seconds=.05),after=None)


def validate_applied_physics(record,settings):
    record['after']=physics_snapshot(settings)
    require(record['after']==record['requested'],'moving inherited world-setting readback differs')
    record['status']='PASS'


def measured_progress(points,x,y):
    require(all(math.isfinite(v) for v in (x,y)),'moving projection requires finite pose')
    best=None;arc=0.
    for index,(a,b) in enumerate(zip(points,points[1:])):
        dx,dy=float(b['x'])-float(a['x']),float(b['y'])-float(a['y'])
        length2=dx*dx+dy*dy
        if length2<=1e-12:continue
        length=math.sqrt(length2)
        ratio=max(0.,min(1.,((x-float(a['x']))*dx+(y-float(a['y']))*dy)/length2))
        progress=arc+ratio*length
        error=math.hypot(x-float(a['x'])-ratio*dx,y-float(a['y'])-ratio*dy)
        candidate=(error,-progress,index,progress)
        if best is None or candidate<best:best=candidate
        arc+=length
    require(best is not None,'moving route projection unavailable')
    return best[3]


class MovingTask:
    def __init__(self,plan):
        self.plan=plan;self.first=None;self.last=None;self.closed=None;self.entered=False
        self.previous_progress=None;self.previous_time=None;self.selected=[];self.support=None;self.failure=None;self.latest=None

    def fail(self,reason):
        if self.failure is None:self.failure=reason

    def observe(self,*,frame,timestamp,phase,progress,base_stop_reason,event_count):
        require(type(frame) is int and frame>=0 and phase in ('stationary_warmup','driving','stationary_tail'),'invalid moving observation identity')
        require(all(type(v) in (int,float) and math.isfinite(v) for v in (timestamp,progress)),'nonfinite moving observation')
        now=round(timestamp*1e9)
        if self.previous_time is not None and abs(now-self.previous_time-50_000_000)>1:
            self.fail('moving_task_native_timestamp_cadence_failure')
        self.previous_time=now
        if event_count:self.fail('moving_task_observed_collision_or_lane_event')
        if phase=='driving' and self.support is None:
            if self.entered and self.previous_progress is not None and progress<self.previous_progress:
                self.fail('moving_task_decreasing_progress_or_reentry')
            self.previous_progress=progress
            if progress>165. and self.closed is None:
                self.closed=dict(frame=frame,timestamp_ns=now,measured_route_progress_m=progress)
                if self.first is None or self.last['timestamp_ns']-self.first['timestamp_ns']<30_000_000_000:
                    self.fail('moving_task_selected_span_under_30_seconds')
            if base_stop_reason is not None:self.fail('moving_task_catalog_or_agent_end_before_support')
            if self.closed is not None and self.last is not None and now>=self.last['timestamp_ns']+6_400_000_000 and self.failure is None:
                self.support=dict(frame=frame,timestamp_ns=now,required_timestamp_ns=self.last['timestamp_ns']+6_400_000_000)
        reason=self.failure or ('moving_task_future_support_complete' if self.support is not None and phase=='driving' else None)
        self.latest=dict(schema='carla.moving_task_observation.v1',measured_route_progress_m=progress,
            source_frame=frame,source_timestamp_ns=now,selection_open=self.closed is None,selected_camera_saved=False,
            camera_selection_eligible=phase=='driving' and self.closed is None and 30.<=progress<=165.,
            window_closed_at=self.closed,future_support_complete=self.support is not None,termination_reason=reason)
        return self.latest,reason

    def saved_camera(self,camera):
        require(self.latest is not None and camera['frame']==self.latest['source_frame']
            and round(camera['timestamp']*1e9)==self.latest['source_timestamp_ns'],
            'moving camera must bind current native observation')
        selected=self.latest['camera_selection_eligible']
        camera['moving_task_selected']=selected
        camera['moving_task_measured_route_progress_m']=self.latest['measured_route_progress_m']
        if selected:
            require(tuple(camera['camera_order'])==CAMERAS and set(camera['images'])==set(CAMERAS)
                and set(camera['source_timestamps'])==set(CAMERAS),'moving camera bundle incomplete')
            stamps=list(camera['source_timestamps'].values())
            require(all(type(v) in (int,float) and math.isfinite(v) for v in stamps),'nonfinite camera timestamp')
            require(max(stamps)-min(stamps)<=1e-6 and all(abs(v-camera['timestamp'])<=1e-6 for v in stamps),
                'moving camera/native timestamps unaligned')
            anchor=dict(frame=camera['frame'],timestamp_ns=max(round(v*1e9) for v in stamps),
                measured_route_progress_m=self.latest['measured_route_progress_m'])
            self.selected.append(anchor)
            if self.first is None:self.first=anchor;self.entered=True
            self.last=anchor;self.latest['selected_camera_saved']=True

    def result(self,*,tail_states=0,goal_reached=False):
        complete=self.support is not None and self.failure is None and tail_states==130
        return dict(schema='carla.moving_task_result.v1',task_complete=complete,actual_catalog_goal_reached=goal_reached,
            selected_camera_anchor_count=len(self.selected),first_selected_camera=self.first,last_selected_camera=self.last,
            selected_camera_span_ns=None if self.first is None else self.last['timestamp_ns']-self.first['timestamp_ns'],
            first_closing_observation=self.closed,future_support=self.support,observed_tail_state_count=tail_states,
            failure_reason=self.failure,raw_outside_selection_retained=True,**pending_scope())


def reconstruct(plan,states,cameras,route_points):
    # HH_260906 - Replay prospective membership from retained measured poses/stamps, not a favorable post-hoc slice.
    by_frame={r['frame']:r for r in cameras};require(len(by_frame)==len(cameras),'duplicate camera frame')
    require(set(by_frame).issubset({r['frame'] for r in states}),'camera has no retained native observation')
    task=MovingTask(plan)
    for row in states:
        stored=row['moving_task']
        progress=measured_progress(route_points,row['x'],row['y'])
        require(progress==stored['measured_route_progress_m'],'moving projection differs from immutable pose')
        measured,reason=task.observe(frame=row['frame'],timestamp=row['timestamp'],phase=row['capture_phase'],
            progress=progress,base_stop_reason=stored['original_termination_reason'],
            event_count=stored['observed_event_count'])
        if row['frame'] in by_frame:
            camera=by_frame[row['frame']];copy=dict(camera)
            require(camera['capture_phase']==row['capture_phase'],'camera/native phase mismatch')
            task.saved_camera(copy)
            require(camera['moving_task_selected']==copy['moving_task_selected'],'saved camera membership differs')
        require(all(stored[k]==v for k,v in measured.items()),'moving observation replay differs')
    return task


def main(argv=None):
    # HH_260906 - Reproducible CPU-only declaration writes one new external plan; it never creates capture data or contacts CARLA.
    import argparse
    if __package__:from . import collect_carla_vad_expert as collector
    else:import collect_carla_vad_expert as collector
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--declare',type=Path,required=True)
    parser.add_argument('--slot-id')
    parser.add_argument('--target-speed-kmh',type=float,default=15.)
    parser.add_argument('--capture-output',type=Path)
    parser.add_argument('--route-file',type=Path)
    parser.add_argument('--mapping',type=Path,default=collector.DEFAULT_MAPPING)
    parser.add_argument('--calibration',type=Path,default=collector.DEFAULT_CALIBRATION)
    args=parser.parse_args(argv);require_15kph_speed(args.target_speed_kmh)
    if args.slot_id is None:
        slot_id=DEFAULT_SLOT_ID
    else:
        require(args.slot_id in RECOVERY_SLOT_IDS,
            'explicit moving slot must be one of the five uncollected sealed TRAIN straight slots')
        slot_id=args.slot_id
    spec=slot_spec(slot_id);verify_cohort_slot(slot_id)
    target=canonical(args.declare)
    capture_output=canonical(args.capture_output or ROOT/spec['capture_path'])
    route_file=canonical(args.route_file or ROOT/spec['route_path'])
    require(target==ROOT/spec['plan_path'] and not target.exists(),'fresh fixed prospective plan required')
    require(not capture_output.exists() and not Path(str(capture_output)+'.partial').exists(),
        'one fresh moving capture required before declaration')
    if slot_id in RECOVERY_SLOT_IDS:
        require(not capture_output.parent.exists(),'fresh uncollected moving case directory required before declaration')
    plan=build_plan(capture_output,route_file,args.mapping,args.calibration,slot_id=slot_id,
        target_speed_kmh=args.target_speed_kmh)
    target.parent.mkdir(parents=True,exist_ok=True)
    with target.open('xb') as stream:
        stream.write(encoded(plan));stream.flush()
        __import__('os').fsync(stream.fileno())
    print(json.dumps(dict(plan=str(target),sha256=sha(target),slot_id=slot_id,capture_output=str(capture_output),
        target_speed_kmh=15.,admission_status='PENDING_QA',dataset_conversion_performed=False)),flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
