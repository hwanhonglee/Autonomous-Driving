"""HH_260906 - Verify paired TRAIN-only memorization evidence without treating it as DEV improvement."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def paired_groups(before,after):
    """HH_260906 - Require identical unique sample identities before comparing training-fit metrics."""
    if len(before)!=12 or len(after)!=12:raise ValueError('twelve samples required')
    ids=[r['token'] for r in before]
    if len(set(ids))!=12 or ids!=[r['token'] for r in after]:raise ValueError('paired sample identity mismatch')
    metrics=('selected_ade_m','oracle_ade_m','speed_mae_mps','constant_speed_route_ade_m')
    for a,b in zip(before,after):
        if (a['group'],a['episode'])!=(b['group'],b['episode']):raise ValueError('sample group changed')
        if a['constant_speed_route_ade_m']!=b['constant_speed_route_ade_m']:raise ValueError('reference changed')
        if any(not math.isfinite(row[k]) or row[k]<0 for row in (a,b) for k in metrics):
            raise ValueError('invalid metric')
    out={}
    for group in ('straight','left','right'):
        a=[r for r in before if r['group']==group];b=[r for r in after if r['group']==group]
        if len(a)!=4:raise ValueError('four samples per group required')
        out[group]={k:dict(before=sum(r[k] for r in a)/4,after=sum(r[k] for r in b)/4) for k in metrics}
    return out


def main():
    """HH_260906 - Verify source/checkpoint hashes and publish a non-promotable paired report."""
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    read=lambda name:json.loads((a.run/name).read_text())
    sha=lambda name:hashlib.sha256((a.run/name).read_bytes()).hexdigest()
    summary=read('summary.json');started=read('started.json')
    if sha('research_checkpoint.pt')!=summary['checkpoint_sha256']:raise ValueError('checkpoint checksum mismatch')
    if sha('source.py')!=started['source_sha256']:raise ValueError('source checksum mismatch')
    progress=[json.loads(line) for line in (a.run/'progress.jsonl').read_text().splitlines()]
    if [r['step'] for r in progress]!=list(range(100,1501,100)):raise ValueError('incomplete progress')
    groups=paired_groups(read('before.json'),read('after.json'))
    report=dict(groups=groups,steps=summary['steps'],elapsed_training_s=summary['elapsed_training_s'],
        coverage=summary['coverage'],checkpoint_sha256=summary['checkpoint_sha256'],
        measured_generalization=False,closed_loop_performed=False,promotion_eligible=False,
        same_train_fit_improved_all_groups=all(g['selected_ade_m']['after']<g['selected_ade_m']['before'] for g in groups.values()))
    with a.output.open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
