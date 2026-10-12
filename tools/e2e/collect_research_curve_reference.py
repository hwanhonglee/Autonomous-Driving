"""HH_260906 - Isolated simulation-only curve-speed cap pilot; never approved training data."""
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.e2e import collect_carla_vad_expert as collector
from scripts.e2e.offline_turn_speed_envelope import EnvelopeConfig, plan
from scripts.e2e.research_speed_reference import SpeedReference


def parse_args(argv=None):
    """HH_260906 - Reuse strict owned collector arguments without changing shared defaults."""
    args = collector.parse_args(argv)
    route = json.loads(args.route_file.read_text())
    if route['town'] not in ('Town01', 'Town07') or route['scenario'] != 'right':
        raise ValueError('TRAIN right-turn pilot only')
    if args.control_transport != 'acknowledged_batch':
        raise ValueError('acknowledged control required')
    return args


def main(argv=None):
    """HH_260906 - Patch the imported agent only in this worker process and preserve emergency logic."""
    args = parse_args(argv)
    from agents.navigation import basic_agent
    original = basic_agent.BasicAgent
    route = json.loads(args.route_file.read_text())
    cruise = args.target_speed_kmh / 3.6
    # HH_260906 - Test curvature anticipation separately; existing goal-stop logic owns terminal braking.
    envelope = plan(route['route'], EnvelopeConfig(cruise_mps=cruise, start_mps=cruise, end_mps=cruise))
    reference = SpeedReference(envelope)
    projector = collector.RouteProjector(route)
    journal = args.output.parent / 'curve_reference.jsonl'
    with journal.open('x') as stream:
        class CurveReferenceAgent(original):
            """HH_260906 - Change only requested speed before unchanged hazard detection and PID control."""
            def __init__(self, vehicle, target_speed=20, **kwargs):
                self.research_vehicle = vehicle
                self.research_cap = target_speed
                self.research_progress = 0.0
                super().__init__(vehicle, target_speed=target_speed, **kwargs)

            def set_target_speed(self, speed):
                self.research_cap = speed
                super().set_target_speed(speed)

            def _affected_by_traffic_light(self, *args, **kwargs):
                # HH_260906 - Observe only the existing hazard call; never add a second cache-changing query.
                result = super()._affected_by_traffic_light(*args, **kwargs)
                self.research_hazards['traffic_light'] = bool(result[0])
                self.research_hazards['traffic_light_actor_id'] = getattr(result[1], 'id', None)
                return result

            def _vehicle_obstacle_detected(self, *args, **kwargs):
                # HH_260906 - Return the original obstacle tuple and record its decision without intervention.
                result = super()._vehicle_obstacle_detected(*args, **kwargs)
                self.research_hazards['vehicle_obstacle'] = bool(result[0])
                self.research_hazards['vehicle_actor_id'] = getattr(result[1], 'id', None)
                return result

            def add_emergency_stop(self, control):
                # HH_260906 - Preserve the exact upstream emergency override, including its brake strength.
                self.research_hazards['emergency_override_called'] = True
                return super().add_emergency_stop(control)

            def run_step(self):
                transform = self.research_vehicle.get_transform()
                yaw = math.radians(transform.rotation.yaw)
                x = transform.location.x - args.wheelbase_m * .5 * math.cos(yaw)
                y = -(transform.location.y - args.wheelbase_m * .5 * math.sin(yaw))
                projection = projector.project(x, y, self.research_progress)
                self.research_progress = projection.progress_m
                cap = reference.local_speed(self.research_progress) * 3.6
                requested = min(self.research_cap, cap)
                super().set_target_speed(requested)
                # HH_260906 - Do not pre-call traffic-light checks or modify their internal hazard cache.
                self.research_hazards = dict(emergency_override_called=False)
                control = super().run_step()
                velocity = self.research_vehicle.get_velocity()
                record = dict(frame=self.research_vehicle.get_world().get_snapshot().frame,
                    progress_m=self.research_progress, cross_track_error_m=projection.cross_track_error_m,
                    requested_speed_kph=requested, upstream_cap_kph=self.research_cap,
                    curve_cap_kph=cap, measured_speed_mps=math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2),
                    throttle=control.throttle, brake=control.brake, steer=control.steer,
                    traffic_light_state=str(self.research_vehicle.get_traffic_light_state()),
                    at_traffic_light=self.research_vehicle.is_at_traffic_light(),
                    upstream_hazard_decisions=dict(self.research_hazards),
                    training_data_approved=False, emergency_logic_unchanged=True)
                stream.write(json.dumps(record, allow_nan=False) + '\n')
                stream.flush()
                return control

        basic_agent.BasicAgent = CurveReferenceAgent
        original_collect = collector.collect_episode
        def marked_collect(*positional, **kwargs):
            # HH_260906 - Record research exclusion before collection, including failed partial episodes.
            positional[6]['training_data_approved'] = False
            positional[6].setdefault('capture_contract', {})['curve_reference_research'] = dict(
                training_data_approved=False, envelope=envelope,
                control_source='CARLA BasicAgent expert with spatial curvature speed cap; not learned model')
            return original_collect(*positional, **kwargs)
        collector.collect_episode = marked_collect
        try:
            output = collector.run(args)
            print(f'capture={output}')
        finally:
            basic_agent.BasicAgent = original
            collector.collect_episode = original_collect
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
