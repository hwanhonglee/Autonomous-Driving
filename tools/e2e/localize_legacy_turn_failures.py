#!/usr/bin/env python3
"""HH_260906 - Collapse overlapping failed futures onto recorded native times without altering labels."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def main():
    """HH_260906 - Report repeated-window multiplicity separately from independent physical events."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit',type=Path,required=True);parser.add_argument('--samples',type=Path,required=True)
    parser.add_argument('--native',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=json.loads(args.audit.read_text())
    episodes=[e for e in report['episodes'] if e['episode_id'].startswith('carla_town01_town01_right_')]
    if len(episodes)!=1:raise ValueError('one selected TRAIN right episode required')
    ep=episodes[0]
    if hashlib.sha256(args.samples.read_bytes()).hexdigest()!=ep['samples_sha256']:raise ValueError('sample hash mismatch')
    samples={r['sample_id']:r for r in map(json.loads,args.samples.read_text().splitlines())}
    native=[json.loads(line) for line in (args.native/'states.jsonl').read_text().splitlines()]
    manifest=json.loads((args.native/'manifest.json').read_text())
    if manifest['runtime']['town']!='Town01':raise ValueError('TRAIN map mismatch')
    wheelbase=manifest['coordinate_contract']['wheelbase_m']
    anchors=[]
    for failure in ep['failures']:
        if failure.get('diagnostic_group')!='future_turn_ge15deg':continue
        row=samples[failure['sample_id']];index=failure['gate_locals']['index']
        time=row['labels']['planning']['target_timestamp_ns'][index]/1e9
        nearest=min(range(len(native)),key=lambda i:abs(native[i]['timestamp']-time))
        current=native[nearest]
        if abs(current['timestamp']-time)>1e-6:raise ValueError('future target not matched to native frame')
        speed=math.hypot(current['vx'],current['vy'])
        anchors.append(dict(sample_id=row['sample_id'],native_frame=current['frame'],native_timestamp=current['timestamp'],
            first_failure=failure['category'],gate_locals=failure['gate_locals'],
            measured_speed_mps=speed,yaw_rate_over_speed_inv_m=current['yaw_rate']/speed if speed>.1 else None,
            tan_steering_over_wheelbase_inv_m=math.tan(current['steering_tire_angle_rad'])/wheelbase,
            current_control=current['current_control']))
    groups={}
    for row in anchors:
        groups.setdefault(row['first_failure'],Counter())[row['native_frame']]+=1
    output=dict(schema='portable_e2e.legacy_turn_overlap.v1',failed_windows=len(anchors),
        distinct_first_failure_native_frames=len({r['native_frame'] for r in anchors}),
        grouped_frame_multiplicities={k:dict(v) for k,v in groups.items()},rows=anchors,
        source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in
                       (args.audit,args.samples,args.native/'states.jsonl',args.native/'manifest.json')},
        limitations=['Overlapping 6.4-second futures are not independent driving trials.',
            'Yaw-rate/speed and bicycle steering curvature are contextual proxies, not interchangeable with the runtime chord-curvature gate.',
            'Legacy capture lacks raw actor snapshots and acknowledged control receipts; exact source of low-speed jitter is not established.'],
        labels_modified=False,live_gate_modified=False)
    args.output.mkdir(parents=True,exist_ok=False)
    with (args.output/'summary.json').open('x') as stream:json.dump(output,stream,indent=2,allow_nan=False)
    curvature=[r for r in anchors if 'curvature' in r['first_failure']]
    t=min(r['native_timestamp'] for r in curvature)
    selected=[r for r in native if t-.5<=r['timestamp']<=t+.5]
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    x=[r['timestamp']-t for r in selected]
    axes[0].plot(x,[math.hypot(r['vx'],r['vy']) for r in selected]);axes[0].set_ylabel('Speed (m/s)')
    axes[1].plot(x,[r['yaw_rate'] for r in selected],label='Reported yaw rate (rad/s)')
    axes[1].plot(x,[r['steering_tire_angle_rad'] for r in selected],label='Tire steering angle (rad)');axes[1].legend()
    axes[2].plot(x,[r['current_control']['throttle'] for r in selected],label='Throttle')
    axes[2].plot(x,[r['current_control']['brake'] for r in selected],label='Brake');axes[2].legend()
    axes[2].set_xlabel('Seconds relative to first low-speed curvature failure')
    fig.suptitle('Legacy TRAIN low-speed restart — measured records, not a new driving test')
    fig.tight_layout();fig.savefig(args.output/'low_speed_restart.png',dpi=130);plt.close(fig)
    print(json.dumps({k:v for k,v in output.items() if k in ('failed_windows','distinct_first_failure_native_frames','grouped_frame_multiplicities')}))


if __name__=='__main__':main()
