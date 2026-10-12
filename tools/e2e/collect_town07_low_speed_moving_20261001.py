"""HH_261001 - Fresh fixed moving task with v2 normal controls, not a goal-success override."""

from contextlib import contextmanager
import json
import math
from pathlib import Path
import sys

from scripts.e2e import collect_carla_vad_expert as base
from scripts.e2e import collect_town07_low_speed_start_v2_20261001 as v2
from scripts.e2e import town07_low_speed_moving_task_20261001 as task
from scripts.e2e.carla_goal_stop_profile import ComfortableGoalStopConfig


def moving_provider(args):
    """HH_261001 - Route only the exact new plan in this standalone worker process."""
    task.configuration_from_args(args)
    return task


@contextmanager
def parsing_scope():
    original = base.moving_capture_module
    base.moving_capture_module = moving_provider
    try:
        yield
    finally:
        base.moving_capture_module = original


class TargetRamp:
    """HH_261001 - Preserve the v2 normal target ramp while keeping catalog-stop quality separate."""
    def __init__(self):
        self.initial_call = True
        self.last_frame = None
        self.target = 0.

    def update(self, frame):
        if self.last_frame is not None and frame <= self.last_frame:
            raise ValueError('one BasicAgent proposal per increasing native frame required')
        if self.last_frame is not None and frame != self.last_frame + 1:
            raise ValueError('normal target ramp requires contiguous native frames')
        self.last_frame = frame
        if self.initial_call:
            # HH_261001 - Original initial-drive proposal precedes the first governor update.
            self.initial_call = False
            return 2.5
        self.target = min(2.5, self.target + 1.5 * .05)
        return self.target


@contextmanager
def scope(args):
    from agents.navigation import basic_agent as agent_module
    original_agent = agent_module.BasicAgent
    original_collect, original_transport = base.collect_episode, base.AcknowledgedControlTransport
    original_moving = base.moving_capture_module
    session = dict(output=None, states=None, manifest=None, policy=None, target=None)

    class FixedMovingAgent(original_agent):
        """HH_261001 - Wrap normal LocalPlanner only; inherited BasicAgent emergency return is untouched."""
        def __init__(self, *positional, **kwargs):
            super().__init__(*positional, **kwargs)
            planner = self.get_local_planner()
            original_normal, observed = planner.run_step, {'value': None}

            def observe_normal(*p, **kw):
                control = original_normal(*p, **kw)
                observed['value'] = base.control_dict(control)
                return control

            planner.run_step = observe_normal
            metadata = base.install_normal_brake_cap(planner, 0., .20)
            normal, policy = planner.run_step, v2.CombinedControl()
            session['policy'] = policy

            def normal_step(*p, **kw):
                control = normal(*p, **kw)
                velocity = planner._vehicle.get_velocity()
                snapshot = planner._vehicle.get_world().get_snapshot()
                return policy.apply(control, speed=math.hypot(velocity.x, velocity.y),
                    timestamp=snapshot.timestamp.elapsed_seconds, frame=snapshot.frame,
                    normal_proposal=observed['value'])

            planner.run_step = normal_step
            self._moving_target_ramp = TargetRamp()
            session['manifest']['capture_contract']['fixed_moving_normal_control']['effective_control'] = dict(metadata,
                inherited_basic_agent_max_brake=self._max_brake, **task.pending_scope())

        def run_step(self, *positional, **kwargs):
            snapshot = self._vehicle.get_world().get_snapshot()
            target = self._moving_target_ramp.update(snapshot.frame)
            self.set_target_speed(target * 3.6)
            session['target'] = dict(source_frame=snapshot.frame, target_speed_mps=target,
                target_ramp_mps2=1.5, initial_drive_proposal_precedes_first_ramp_update=True)
            return super().run_step(*positional, **kwargs)

    class Transport(original_transport):
        def send(self, control, reason, expected_before_frame=None):
            receipt = super().send(control, reason, expected_before_frame)
            if session['output'] is not None and reason in ('initial_drive_control', 'next_driving_control'):
                policy = session['policy']
                normal = policy.last if policy and policy.last and policy.last['source_frame'] == receipt['before_frame'] else None
                record = dict(schema='carla.town07_low_speed_moving_control.v1', reason=reason,
                    normal_policy=normal, target_ramp=session['target'],
                    final_basic_agent_proposal=base.control_dict(control), ack_receipt=receipt,
                    learned_control=False, training_data_approved=False)
                with (session['output'] / 'expert_moving_control.jsonl').open('a') as stream:
                    stream.write(json.dumps(record, allow_nan=False) + '\n')
            return receipt

    def collect(*positional, **kwargs):
        _, _, _, partial, states, cameras, manifest = positional
        session.update(output=partial, states=states, manifest=manifest)
        # HH_261001 - Fresh prospective metadata: no prior manifest or denial marker is loaded or removed.
        manifest.update(task.pending_scope(), control_source='BasicAgent_fixed_v2_normal_policy_new_moving_task')
        manifest['capture_contract']['fixed_moving_normal_control'] = dict(
            task.plan_contract()['controller'], v2_normal_policy_source_sha256=task.sha(Path(v2.__file__)),
            task_worker_sha256=task.sha(Path(__file__)), effective_control={'status': 'NOT_REACHED'},
            terminal_goal_governor_enabled=False,
            shutdown='Original moving-task full-brake/130-tick tail strictly after selected future support; not stop supervision.',
            **task.pending_scope())
        try:
            return original_collect(*positional, **kwargs)
        finally:
            # HH_261001 - Preserve original whole-recording failed goal/native QA independently of moving-task completion.
            if states:
                config = ComfortableGoalStopConfig(profile_id='v2_normal_control_only_new_moving_task',
                    desired_deceleration_mps2=.5, normal_brake_cap=0., normal_throttle_cap=.20)
                try:
                    quality = base.measured_stop_quality(states, [row['frame'] for row in cameras], config,
                        base.source_motion_bounds(), manifest.get('result', {}).get('goal_reached', False))
                except (ValueError, KeyError, IndexError) as error:
                    quality = dict(status='UNAVAILABLE_RETAINED', reason=str(error),
                                   notice='Cannot qualify missing evidence; original capture exception is not replaced.')
                manifest.setdefault('result', {})['whole_recording_goal_stop_quality_diagnostic'] = quality
                manifest['result']['whole_recording_quality_not_overridden_by_task_completion'] = True

    base.collect_episode, base.AcknowledgedControlTransport = collect, Transport
    base.moving_capture_module, agent_module.BasicAgent = moving_provider, FixedMovingAgent
    try:
        yield
    finally:
        base.collect_episode, base.AcknowledgedControlTransport = original_collect, original_transport
        base.moving_capture_module, agent_module.BasicAgent = original_moving, original_agent


def parse_args(argv=None):
    with parsing_scope():
        args = base.parse_args(argv)
    task.configuration_from_args(args)
    if base.configuration_from_args(args) is not None:
        raise ValueError('catalog goal-stop governor must remain disabled for this moving task')
    return args


def main(argv=None):
    try:
        args = parse_args(argv)
        with scope(args):
            output = base.run(args)
    except (base.CollectionError, OSError, ValueError) as error:
        print('ERROR: ' + str(error), file=sys.stderr)
        return 1
    print('capture=' + str(output) + '; moving task complete only; PENDING_QA, no existing qualification or admission')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
