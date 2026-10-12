"""HH_260906 - Summarize bounded stop pilots without hiding failed partial episodes."""
import argparse
import json
import math
from pathlib import Path


def summarize(trial):
    """HH_260906 - Report capture outcomes separately from training or model capability."""
    e=trial/'episode'
    if not e.exists():e=trial/'episode.partial'
    m=json.loads((e/'manifest.json').read_text())
    states=[json.loads(s) for s in (e/'states.jsonl').read_text().splitlines()]
    frames={s['frame']:s for s in states}
    cameras=[frames[c['frame']] for c in map(json.loads,(e/'camera_frames.jsonl').read_text().splitlines())]
    records=[json.loads(s) for s in (trial/'curve_reference.jsonl').read_text().splitlines()]
    speed=lambda s:math.hypot(s['vx'],s['vy'])
    rates=[(speed(b)-speed(a))/(b['timestamp']-a['timestamp']) for a,b in zip(cameras,cameras[1:])]
    driving=[s for s in states if s['capture_phase']=='driving']
    return dict(trial=str(trial),status=m['status'],error=m.get('error'),
        goal_reached=m.get('result',{}).get('goal_reached'),
        collision_events=m.get('result',{}).get('collision_event_count'),
        lane_invasion_events=m.get('result',{}).get('lane_invasion_event_count'),
        last_driving_speed_mps=speed(driving[-1]),last_goal_record=driving[-1].get('goal_stop'),
        minimum_camera_speed_rate_mps2=min(rates),deceleration_violation_intervals=sum(v < -6 for v in rates),
        emergency_override_ticks=sum(s['upstream_hazard_decisions']['emergency_override_called'] for s in records),
        preview_ticks=sum(s.get('signal_stop_preview') is not None for s in records),
        tail_states=sum(s['capture_phase']=='stationary_tail' for s in states),
        source_archived_unchanged=json.loads((trial/'owner_result.json').read_text())['source_bytes_unchanged_and_archived'],
        training_admitted=False,learned_model_evaluated=False)


def main():
    """HH_260906 - Write new summary evidence without overwriting past trial outcomes."""
    p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    reports=[summarize(t) for t in a.trial]
    with a.output.open('x') as stream:json.dump(reports,stream,indent=2,allow_nan=False)
    for r in reports: print(json.dumps({k:v for k,v in r.items() if k!='last_goal_record'}))


if __name__=='__main__':main()
