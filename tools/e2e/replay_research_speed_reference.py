#!/usr/bin/env python3
"""HH_260906 - Replay recorded observations through an isolated reference policy, not vehicle dynamics."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
from scripts.e2e.offline_turn_speed_envelope import plan
from scripts.e2e.research_speed_reference import ReferenceRequest,SpeedReference


def replay(trial):
    """HH_260906 - Preserve measured speed and flag incompatibility without fabricating new outcomes."""
    episode=trial/'episode'
    route=json.loads((episode/'route.json').read_text());states=[json.loads(l) for l in (episode/'states.jsonl').read_text().splitlines()]
    if route['town'] not in ('Town01','Town07') or route['scenario']!='right':raise ValueError('TRAIN right routes only')
    envelope=plan(route['route']);reference=SpeedReference(envelope);rows=[]
    for state in states:
        if state['capture_phase']!='driving':continue
        progress=state['route_progress_m'];speed=math.hypot(state['vx'],state['vy'])
        response=reference.update(ReferenceRequest(progress,speed,.05))
        rows.append(dict(frame=state['frame'],t=state['timestamp']-states[0]['timestamp'],
            progress_m=progress,measured_speed_mps=speed,command=state['command'],
            observed_brake=state['current_control']['brake'],**response))
    return dict(trial=str(trial),route_proposal=envelope,replay_count=len(rows),
        status_counts=dict(Counter(r['status'] for r in rows)),
        traffic_stop_constraint_available=False,
        source_sha256={name:hashlib.sha256((episode/name).read_bytes()).hexdigest() for name in ('route.json','states.jsonl','manifest.json')},
        source_files_unchanged=True,training_executed=False,model_inference_executed=False,
        measured_outcome_after_applying_reference=None,
        notice='Observation replay only. Counterfactual vehicle response, improved labels and driving success are NOT measured.',
        rows=rows)


def main():
    """HH_260906 - Create a self-contained offline report and plot in a fresh output directory."""
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    reports=[replay(t) for t in args.trial]
    args.output.mkdir(parents=True,exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for report in reports:
        name=Path(report['trial']).name
        with (args.output/(name+'.json')).open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
        rows=report['rows'];fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
        axes[0].plot([r['t'] for r in rows],[r['measured_speed_mps']*3.6 for r in rows],label='Recorded expert speed')
        axes[0].plot([r['t'] for r in rows],[r['target_speed_mps']*3.6 for r in rows],label='Offline proposed reference (not driven)')
        axes[0].set_ylabel('km/h');axes[0].legend()
        axes[1].plot([r['t'] for r in rows],[r['observed_brake'] for r in rows],label='Recorded brake')
        axes[1].plot([r['t'] for r in rows],[int(not r['normal_reference_allowed']) for r in rows],label='Reference infeasible from observation')
        axes[1].set(xlabel='Recorded simulation time (s)',ylabel='Brake / flag');axes[1].legend()
        fig.suptitle(name+' — OFFLINE observation replay, not closed-loop');fig.tight_layout()
        fig.savefig(args.output/(name+'.png'),dpi=130);plt.close(fig)
        print(name,report['status_counts'])
    sources=('scripts/e2e/offline_turn_speed_envelope.py','scripts/e2e/research_speed_reference.py',
             'scripts/e2e/replay_research_speed_reference.py')
    summary=dict(trials=[{k:v for k,v in r.items() if k not in ('rows','route_proposal')} for r in reports],
                 code_sha256={name:hashlib.sha256(Path(name).read_bytes()).hexdigest() for name in sources})
    with (args.output/'summary.json').open('x') as stream:json.dump(summary,stream,indent=2)


if __name__=='__main__':main()
