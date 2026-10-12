"""HH_260906 - Plot label-conditioned latent fitting without presenting it as deployable inference."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    """HH_260906 - Preserve per-sample failures and distinguish geometry-only from joint fitting."""
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    plan=json.loads((a.run/'plan.json').read_text());result=json.loads((a.run/'results.json').read_text())
    if hashlib.sha256((a.run/'source.py').read_bytes()).hexdigest()!=plan['source_sha256']:
        raise ValueError('source hash differs')
    if not result['model_weights_unchanged'] or result['release_approved']:raise ValueError('diagnostic scope changed')
    a.output.mkdir(parents=True,exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,3,figsize=(15,9));compact=[]
    for row,ax in zip(result['rows'],axes.flat):
        if len(row['fits'])!=4:raise ValueError('incomplete objective/initialization matrix')
        geometry=min((f for f in row['fits'] if f['objective']=='geometry'),key=lambda f:f['objective_value'])
        joint=min((f for f in row['fits'] if f['objective']=='joint'),key=lambda f:f['objective_value'])
        compact.append(dict(token=row['token'],group=row['group'],yaw_deg=row['future_yaw_deg'],
            target_gate=row['target_gate'],baseline_selected_ade_m=row['baseline_selected_ade_m'],
            baseline_oracle_ade_m=row['baseline_oracle_ade_m'],
            geometry={k:v for k,v in geometry.items() if k not in ('xy','speed')},
            joint={k:v for k,v in joint.items() if k not in ('xy','speed')}))
        for label,points in [('Recorded target',row['target_xy']),('K1 selected',row['baseline_xy']),
                             ('Label-fitted DRIVE',joint['xy'])]:
            ax.plot([p[0] for p in points],[p[1] for p in points],label=label)
        ax.set_title(f"{row['group']} {row['future_yaw_deg']:.1f} deg");ax.axis('equal');ax.grid(alpha=.3)
        ax.set_xlabel('Base X (m)');ax.set_ylabel('Base Y (m)');ax.legend(fontsize=8)
    fig.suptitle('TRAIN decoder diagnostic — target-conditioned fits are NOT model driving')
    fig.tight_layout();fig.savefig(a.output/'trajectories.png',dpi=130)
    report=dict(rows=compact,elapsed_s=result['elapsed_s'],model_weights_unchanged=True,
        deployable=False,global_optimum_proven=False,uses_target_future=True,
        source_results_sha256=hashlib.sha256((a.run/'results.json').read_bytes()).hexdigest())
    with (a.output/'comparison.json').open('x') as stream:json.dump(report,stream,indent=2)
    for row in compact:print(json.dumps(row))


if __name__=='__main__':main()
