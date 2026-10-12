"""HH_260906 - Audit raw research camera horizons without preparing or admitting training data."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
from portable_e2e.contract import ContractError
from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory


def audit(trial):
    """HH_260906 - Apply unchanged runtime bounds to exact 10 Hz recorded futures, not model predictions."""
    episode = trial / 'episode'
    if not episode.exists():
        episode = trial / 'episode.partial'
    states = [json.loads(s) for s in (episode/'states.jsonl').read_text().splitlines()]
    cameras = [json.loads(s) for s in (episode/'camera_frames.jsonl').read_text().splitlines()]
    by_frame = {s['frame']: s for s in states}
    rows = [by_frame[c['frame']] for c in cameras]
    errors = []
    for i in range(max(0, len(rows)-64)):
        ego = rows[i]
        future = rows[i+1:i+65]
        if any(abs(s['timestamp']-ego['timestamp']-(j+1)*.1)>1e-6 for j,s in enumerate(future)):
            raise ValueError('camera cadence mismatch')
        c, s = math.cos(ego['yaw']), math.sin(ego['yaw'])
        xy = [[c*(f['x']-ego['x'])+s*(f['y']-ego['y']),
               -s*(f['x']-ego['x'])+c*(f['y']-ego['y'])] for f in future]
        try:
            validate_and_select_trajectory([xy], [[math.hypot(f['vx'],f['vy']) for f in future]], [0.],
                RuntimeGateConfig(candidate_count=1), current_speed_mps=ego['vx'])
        except ContractError as exc:
            errors.append(dict(frame=ego['frame'], reason=str(exc)))
    manifest = json.loads((episode/'manifest.json').read_text())
    return dict(trial=str(trial), status=manifest['status'], result=manifest.get('result'),
        candidate_horizons=max(0,len(rows)-64), passing=max(0,len(rows)-64)-len(errors),
        first_failures=dict(Counter(e['reason'] for e in errors)), failures=errors,
        training_admitted=False, model_evaluated=False,
        scope='All complete camera horizons including warmup/tail; not common10 eligibility or admission.',
        source_sha256={name:hashlib.sha256((episode/name).read_bytes()).hexdigest()
                       for name in ('manifest.json','states.jsonl','camera_frames.jsonl')})


def main():
    """HH_260906 - Preserve every failed interval and refuse overwriting an audit."""
    p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    reports=[audit(t) for t in args.trial]
    with args.output.open('x') as stream:json.dump(reports,stream,indent=2,allow_nan=False)
    for r in reports:
        print(json.dumps({k:r[k] for k in ('trial','status','candidate_horizons','passing','first_failures')}))


if __name__ == '__main__':
    main()
