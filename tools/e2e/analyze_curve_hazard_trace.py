"""HH_260906 - Bind measured camera braking to prior native expert decisions without causal overclaim."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from scripts.e2e.localize_curve_speed_disagreement import interval_metrics


def main():
    """HH_260906 - Distinguish emergency overrides from tail braking using actual per-frame decisions."""
    p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    e=args.trial/'episode'
    if not e.exists():e=args.trial/'episode.partial'
    states=[json.loads(s) for s in (e/'states.jsonl').read_text().splitlines()]
    by_frame={s['frame']:s for s in states}
    camera=[by_frame[c['frame']] for c in map(json.loads,(e/'camera_frames.jsonl').read_text().splitlines())]
    decisions={s['frame']:s for s in map(json.loads,(args.trial/'curve_reference.jsonl').read_text().splitlines())}
    intervals=[]
    for a,b in zip(camera,camera[1:]):
        metrics=interval_metrics(a,b)
        if metrics['measured_speed_rate_mps2']>=-6 and abs(metrics['endpoint_error_mps'])<=.25:continue
        frames=range(a['frame'],b['frame'])
        relevant=[decisions[f] for f in frames if f in decisions]
        metrics.update(phase=b['capture_phase'],remaining_route_m=None,
            decision_frames=[d['frame'] for d in relevant],
            emergency_overrides=[d['frame'] for d in relevant if d['upstream_hazard_decisions']['emergency_override_called']],
            traffic_light_hazards=[d['upstream_hazard_decisions'].get('traffic_light_actor_id') for d in relevant
                                  if d['upstream_hazard_decisions'].get('traffic_light')],
            vehicle_hazards=[d['upstream_hazard_decisions'].get('vehicle_actor_id') for d in relevant
                             if d['upstream_hazard_decisions'].get('vehicle_obstacle')],
            controls=[dict(frame=f,current=by_frame[f]['current_control'],next=by_frame[f]['next_control'])
                      for f in range(a['frame'],b['frame']+1)],
            all_native_ack_bound=all(by_frame[f].get('control_transport',{}).get('frame_binding_pass') is True
                                     for f in range(a['frame']+1,b['frame']+1)))
        intervals.append(metrics)
    report=dict(trial=str(args.trial),intervals=intervals,
        measured_deceleration_violations=sum(r['measured_speed_rate_mps2'] < -6 for r in intervals),
        endpoint_disagreements=sum(abs(r['endpoint_error_mps'])>.25 for r in intervals),
        source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in
                       (e/'states.jsonl',e/'camera_frames.jsonl',args.trial/'curve_reference.jsonl')},
        training_admitted=False,emergency_logic_changed=False,
        limitations=['Temporal control/decision association is not an isolated actuator causal experiment.',
                     'No live-model evaluation; this is expert data instrumentation.'])
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps(dict(intervals=len(intervals),deceleration_violations=report['measured_deceleration_violations'],
        rows=[{k:r[k] for k in ('from_frame','to_frame','phase','measured_speed_rate_mps2','emergency_overrides','traffic_light_hazards','vehicle_hazards')} for r in intervals])))


if __name__=='__main__':main()
