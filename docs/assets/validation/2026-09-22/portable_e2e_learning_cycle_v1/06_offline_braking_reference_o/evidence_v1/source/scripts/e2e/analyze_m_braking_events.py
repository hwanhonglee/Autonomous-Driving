#!/usr/bin/env python3
"""HH_260906 - Localize measured braking violations without inventing signal-state telemetry."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def analyze(trial):
    """HH_260906 - Separate native and camera cadence and preserve predecessor controls."""
    episode=trial/'episode'
    if not episode.is_dir():episode=trial/'episode.partial'
    manifest=json.loads((episode/'manifest.json').read_text())
    states=[json.loads(s) for s in (episode/'states.jsonl').read_text().splitlines()]
    cameras=[json.loads(s) for s in (episode/'camera_frames.jsonl').read_text().splitlines()]
    by_frame={row['frame']:row for row in states}
    if len(by_frame)!=len(states):raise ValueError('duplicate state frame')
    if any(b['frame']!=a['frame']+1 for a,b in zip(states,states[1:])):
        raise ValueError('native frame gap')
    output={}
    for cadence,rows in [('native20hz',states),('camera10hz',[by_frame[c['frame']] for c in cameras])]:
        violations=[];rates=[]
        for a,b in zip(rows,rows[1:]):
            dt=b['timestamp']-a['timestamp']
            if abs(dt-(.05 if cadence=='native20hz' else .1))>1e-6:raise ValueError('unexpected time gap')
            va=math.hypot(a['vx'],a['vy']);vb=math.hypot(b['vx'],b['vy'])
            rate=(vb-va)/dt;rates.append(rate)
            if rate>=-6:continue
            phase=b['capture_phase'];remaining=manifest['result']['route_length_m']-b['route_progress_m']
            region=('tail_transition_or_hold' if phase=='stationary_tail' else
                    'terminal_approach' if remaining<=5 else 'interior_route')
            violations.append(dict(from_frame=a['frame'],to_frame=b['frame'],
                relative_time_s=b['timestamp']-states[0]['timestamp'],speed_before_mps=va,speed_after_mps=vb,
                measured_rate_mps2=rate,phase=phase,region=region,remaining_route_m=remaining,
                current_control=b['current_control'],previous_next_control=a['next_control'],
                goal_stop_phase=b.get('goal_stop',{}).get('phase'),
                traffic_signal_cause='unknown_not_recorded'))
        output[cadence]=dict(violation_intervals=violations,violation_count=len(violations),
            region_counts=dict(Counter(v['region'] for v in violations)),min_rate_mps2=min(rates),
            max_rate_mps2=max(rates))
    return dict(trial=str(trial),cadences=output,source_sha256={name:hashlib.sha256((episode/name).read_bytes()).hexdigest()
        for name in ('manifest.json','states.jsonl','camera_frames.jsonl')},
        limitations=['Controls are observed correlations, not isolated causal actuator tests.',
                     'Traffic-light state and stop-line distance are not recorded; red-light cause is not established.',
                     'Camera-cadence previous command spans two native ticks; inspect native trace for intermediate commands.'])


def main():
    """HH_260906 - Write isolated evidence without altering source captures or shared logs."""
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    report=dict(schema='portable_e2e.braking_localization.v1',trials=[analyze(t) for t in args.trial],
                training_executed=False,live_control_modified=False)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    for trial in report['trials']:
        print(trial['trial'], {k:dict(count=v['violation_count'],regions=v['region_counts'],minimum=v['min_rate_mps2'])
                              for k,v in trial['cadences'].items()})


if __name__=='__main__':main()
