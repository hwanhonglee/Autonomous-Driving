"""HH_260906 - Plot measured local expert trials without presenting them as model validation."""
import argparse
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    """HH_260906 - Compare complete traces and retain the unmodified camera sources separately."""
    p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    fig, axes=plt.subplots(3,1,figsize=(13,12))
    for trial in args.trial:
        episode=trial/'episode'
        if not episode.exists(): episode=trial/'episode.partial'
        rows=[json.loads(line) for line in (episode/'states.jsonl').read_text().splitlines()]
        t=[r['timestamp']-rows[0]['timestamp'] for r in rows]
        v=[math.hypot(r['vx'],r['vy']) for r in rows]
        label=trial.name
        axes[0].plot([r['x'] for r in rows],[r['y'] for r in rows],label=label)
        axes[1].plot(t,[s*3.6 for s in v],label=label)
        axes[2].plot(t[1:],[(b-a)/(tb-ta) for a,b,ta,tb in zip(v,v[1:],t,t[1:])],label=label)
    axes[0].set(xlabel='World X (m)',ylabel='ROS world Y (m)',title='Measured path; expert data, NOT learned driving')
    axes[0].axis('equal')
    axes[1].set(xlabel='Time from capture start (s)',ylabel='Measured speed (km/h)')
    axes[1].axhline(30,color='red',linestyle='--')
    axes[2].set(xlabel='Time from capture start (s)',ylabel='Native 20 Hz speed rate (m/s²)')
    axes[2].axhline(-6,color='red',linestyle='--');axes[2].axhline(3,color='red',linestyle='--')
    for ax in axes: ax.grid(alpha=.3);ax.legend()
    fig.tight_layout();args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.output.exists():raise ValueError('refuse overwrite')
    fig.savefig(args.output,dpi=150)


if __name__=='__main__':main()
