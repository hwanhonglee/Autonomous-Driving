#!/usr/bin/env python3
"""HH_260906 - Summarize immutable expert captures without admitting them for training."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def summarize(trial):
    """HH_260906 - Keep completion, event quality and conversion authorization separate."""
    episode = trial / 'episode'
    if not episode.exists():
        episode = trial / 'episode.partial'
    manifest = json.loads((episode / 'manifest.json').read_text())
    states = [json.loads(line) for line in (episode / 'states.jsonl').read_text().splitlines()]
    cameras = [json.loads(line) for line in (episode / 'camera_frames.jsonl').read_text().splitlines()]
    result = manifest['result']
    driving = [row for row in states if row['capture_phase'] == 'driving']
    moving = [row for row in driving if math.hypot(row['vx'], row['vy']) >= 1.0]
    denials = []

    def inspect(value, path):
        """HH_260906 - Report explicit training denials without modifying source flags."""
        if isinstance(value, dict):
            for key, item in value.items():
                if key == 'training_data_approved' and item is False:
                    denials.append(path + '.' + key)
                inspect(item, path + '.' + key)
        elif isinstance(value, list):
            for index, item in enumerate(value):
                inspect(item, f'{path}[{index}]')

    inspect(manifest, 'manifest')
    events = [dict(frame=row['frame'], speed_kph=math.hypot(row['vx'], row['vy'])*3.6,
                   route_cte_m=row['route_cte_m'], events=row['lane_invasion'])
              for row in states if row['lane_invasion']]
    summary = dict(trial=trial.name, goal_reached=result['goal_reached'],
        native_status=manifest['status'], partial_source=episode.name.endswith('.partial'),
        collision_events=result['collision_event_count'], lane_events=result['lane_invasion_event_count'],
        states=len(states), camera_bundles=len(cameras), duration_sec=result['duration_sec'],
        speed_max_kph=max(math.hypot(row['vx'], row['vy'])*3.6 for row in driving),
        speed_mean_driving_kph=sum(math.hypot(row['vx'], row['vy'])*3.6 for row in driving)/len(driving),
        max_abs_cte_m=max(abs(row['route_cte_m']) for row in driving),
        yaw_rate_over_speed_max_abs_inv_m=max((abs(row['yaw_rate'])/math.hypot(row['vx'],row['vy']) for row in moving), default=None),
        physics_interval_max_error_sec=max(abs(b['timestamp']-a['timestamp']-.05) for a,b in zip(states,states[1:])),
        explicit_training_denials=denials, lane_event_details=events,
        training_admitted=False, learned_vehicle_control=False,
        source_sha256={name:hashlib.sha256((episode/name).read_bytes()).hexdigest()
                       for name in ('manifest.json','states.jsonl','camera_frames.jsonl','route.json')})
    summary['native_event_gate_pass'] = (manifest['status'] == 'complete' and summary['goal_reached'] and not summary['collision_events']
                                        and not summary['lane_events'] and not denials)
    return summary, states


def main():
    """HH_260906 - Render measured traces, not generated screenshots or success claims."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', action='append', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args=parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    reports=[]
    for trial in args.trial:
        report, rows=summarize(trial)
        reports.append(report)
        fig, axes=plt.subplots(1,2,figsize=(12,5))
        origin=next(row for row in rows if row['capture_phase']=='driving')
        c,s=math.cos(origin['yaw']), math.sin(origin['yaw'])
        forward=[c*(r['x']-origin['x'])+s*(r['y']-origin['y']) for r in rows]
        left=[-s*(r['x']-origin['x'])+c*(r['y']-origin['y']) for r in rows]
        axes[0].plot([-v for v in left],forward,label='Recorded expert trajectory')
        axes[0].scatter([0],[0],marker='^',label='Ego at start',color='black')
        for i,r in enumerate(rows):
            if r['lane_invasion']:
                axes[0].scatter([-left[i]],[forward[i]],color='red',marker='x')
        limit=max(10,max(map(abs,forward))+5,max(map(abs,left))+5)
        axes[0].set(xlim=(-limit,limit),ylim=(-limit,limit),xlabel='Right from initial ego (m)',ylabel='Forward from initial ego (m)')
        axes[0].set_aspect('equal');axes[0].legend()
        axes[1].plot([r['timestamp']-rows[0]['timestamp'] for r in rows],
                     [math.hypot(r['vx'],r['vy'])*3.6 for r in rows])
        axes[1].set(xlabel='Simulation time (s)',ylabel='Measured speed (km/h)')
        fig.suptitle(trial.name+' — expert data; NOT learned driving')
        fig.tight_layout();fig.savefig(args.output/(trial.name+'.png'),dpi=130);plt.close(fig)
    (args.output/'summary.json').write_text(json.dumps(dict(schema='portable_e2e.right_diversity_m_audit.v1',
        trials=reports, training_admitted=False),indent=2,allow_nan=False)+'\n')
    print(json.dumps(reports,indent=2))


if __name__ == '__main__':
    main()
