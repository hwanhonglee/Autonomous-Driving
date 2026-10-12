"""HH_260906 - Fixed Town01 pedal-cap experiment; preserve the original collector and all quality gates."""
from __future__ import annotations
import argparse
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
import sys
from . import collect_carla_vad_expert as base
from .carla_goal_stop_profile import ComfortableGoalStopConfig

ROUTE_SHA='3f94016e8e86da050f86fb90bf4deaff668ecf59e38357f53018036dbfdf005a'
ARMS={'launch':(.13,.15),'launch_brake':(.13,.02),
      'staged_launch':(.20,.15),'staged_early_brake':(.20,.02),'staged_coast':(.20,0.),
      'staged_early_coast':(.20,0.),'staged_coast_buffer':(.20,0.)}
ORIGINAL_CONFIGURATION=base.configuration_from_args
ORIGINAL_INSTALLER=base.install_normal_brake_cap


@dataclass(frozen=True)
class PedalCapsConfig(ComfortableGoalStopConfig):
    # HH_260906 - Scalar success alone cannot approve this experimental collector for training.
    development_only: bool=True
    training_data_approved: bool=False
    policy_source_sha256: str=''
    parent_profile: str='comfortable_v2'
    launch_only_cap: float | None=None
    launch_handoff_speed_mps: float=.5


def validate_args(args):
    """HH_260906 - Hold route, initialization, timing and sensors fixed before simulator access."""
    expected=dict(goal_stop_profile='comfortable_v2',target_speed_kmh=9.,physics_hz=20.,capture_hz=10.,
        stationary_warmup_sec=3.5,stationary_tail_sec=6.5,max_duration_sec=180.,goal_tolerance_m=1.,
        agent_initialization='after_bootstrap',control_transport='acknowledged_batch',weather='ClearNoon',
        seed=2026092601,spawn_z_offset_m=0.,vehicle_type='vehicle.toyota.prius',physics_substep_profile='inherited',
        basic_agent_base_min_distance_m=2.,basic_agent_distance_ratio=.2,allow_stopped_steering=False)
    if any(getattr(args,k)!=v for k,v in expected.items()):raise ValueError('AJ fixed experiment settings differ')
    if args.moving_task_plan is not None or args.moving_task_plan_sha256 is not None:
        raise ValueError('AJ does not authorize a moving-task plan')
    if args.pedal_arm not in ARMS:raise ValueError('unknown AJ arm')
    if hashlib.sha256(args.route_file.read_bytes()).hexdigest()!=ROUTE_SHA:raise ValueError('AJ route changed')
    original=ORIGINAL_CONFIGURATION(args)
    values=asdict(original)
    values.update(profile_id='comfortable_v2_aj_'+args.pedal_arm,
        normal_throttle_cap=ARMS[args.pedal_arm][0],normal_brake_cap=ARMS[args.pedal_arm][1],
        policy_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    if args.pedal_arm.startswith('staged_'):values['launch_only_cap']=.13
    # HH_260906 - Coast differs from the early-brake arm only in its normal brake cap; goal and hazard rules stay fixed.
    if args.pedal_arm in ('staged_early_brake','staged_coast'):values['desired_deceleration_mps2']=.4
    # HH_260906 - Observed coast overshoot motivates an earlier target envelope, not a changed goal tolerance or bound.
    if args.pedal_arm in ('staged_early_coast','staged_coast_buffer'):values['desired_deceleration_mps2']=.12
    # HH_260906 - Residual 0.345 m/s endpoint speed motivates additional coasting distance; measured 1 m goal window stays unchanged.
    if args.pedal_arm=='staged_coast_buffer':values['stop_buffer_m']=2.
    return PedalCapsConfig(**values)


def parse_args(argv=None):
    p=argparse.ArgumentParser(add_help=False,allow_abbrev=False)
    p.add_argument('--pedal-arm',choices=tuple(ARMS),required=True)
    own,rest=p.parse_known_args(argv)
    args=base.parse_args(rest);args.pedal_arm=own.pedal_arm
    validate_args(args)
    return args


def staged_installer(planner,cap,throttle_cap=None):
    """HH_260906 - Latch launch handoff from measured planar actor speed; emergency override remains downstream."""
    metadata=ORIGINAL_INSTALLER(planner,cap,throttle_cap)
    normal=planner.run_step
    handoff=False
    def step(*args,**kwargs):
        nonlocal handoff
        velocity=planner._vehicle.get_velocity()
        speed=math.hypot(float(velocity.x),float(velocity.y))
        if not math.isfinite(speed):raise ValueError('nonfinite launch speed')
        handoff=handoff or speed>=.5
        control=normal(*args,**kwargs)
        if not handoff:control.throttle=min(float(control.throttle),.13)
        return control
    planner.run_step=step
    metadata.update(launch_only_cap=.13,launch_handoff_planar_actor_speed_mps=.5,
        handoff_latched=True,launch_speed_source='CARLA actor measured XY velocity; not commanded speed',
        policy_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    return metadata


@contextmanager
def configuration_scope(staged=False):
    """HH_260906 - Replace only this process's explicit config provider, never files or safety/measurement functions."""
    original=base.configuration_from_args
    installer=base.install_normal_brake_cap
    if original is not ORIGINAL_CONFIGURATION or installer is not ORIGINAL_INSTALLER:
        raise ValueError('unexpected collector configuration/control provider')
    base.configuration_from_args=validate_args
    if staged:base.install_normal_brake_cap=staged_installer
    try:yield
    finally:
        base.configuration_from_args=original
        base.install_normal_brake_cap=installer


def main(argv=None):
    try:
        args=parse_args(argv)
        with configuration_scope(args.pedal_arm.startswith('staged_')):output=base.run(args)
    except (base.CollectionError,OSError,ValueError) as error:
        print(f'ERROR: {error}',file=sys.stderr);return 1
    print(f'capture={output}; development only, NOT admitted')
    return 0


if __name__=='__main__':raise SystemExit(main())
