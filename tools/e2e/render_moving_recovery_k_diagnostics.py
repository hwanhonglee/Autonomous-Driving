#!/usr/bin/env python3
"""HH_260906 - Render fixed-index offline K traces with the ego origin centered in every panel."""
import argparse
import json
from pathlib import Path
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def render(source, output):
    """HH_260906 - Show selected, oracle and target paths without implying autonomous vehicle control."""
    summary = json.loads((source/'summary.json').read_text())
    if summary['status'] != 'POSTHOC_REPLAY_MATCHED_ORIGINAL_METRICS':
        raise ValueError('verified diagnostic replay required')
    arms = ('K0_current1475', 'K1_current_plus_successful4')
    reports = [json.loads((source/(arm+'.json')).read_text()) for arm in arms]
    output.mkdir(parents=True, exist_ok=False)
    for direction in ('right', 'straight'):
        figure, axes = plt.subplots(3, 2, figsize=(12, 16))
        for row_index, sample_index in enumerate((0,164,327)):
            pair = [next(trace for trace in report['traces']
                         if '_'+direction+'_' in trace['episode_id'] and trace['episode_index']==sample_index)
                    for report in reports]
            limit = max(10, math.ceil(max(abs(value) for trace in pair
                for name in ('selected_xy','oracle_xy','target_xy') for point in trace[name] for value in point)/10)*10)
            for column, trace in enumerate(pair):
                axis = axes[row_index,column]
                for name, label, color, style in (
                    ('route_xy','Navigation route','#999999',':'),
                    ('target_xy','Recorded future target','#222222','-'),
                    ('oracle_xy','Best candidate by future labels','#2ca02c','--'),
                    ('selected_xy','Model-selected candidate','#d62728','-')):
                    points = trace[name]
                    axis.plot([-point[1] for point in points], [point[0] for point in points],
                              label=label, color=color, linestyle=style, linewidth=2)
                axis.scatter([0],[0],c='#1f77b4',marker='^',s=90,label='Ego at anchor',zorder=5)
                axis.set(xlim=(-limit,limit),ylim=(-limit,limit),xlabel='Right (m)',ylabel='Forward (m)',
                    title=f"K{column} / {direction} index {sample_index}\nSelected c{trace['selected_index']} ADE {trace['selected_ade_m']:.2f} m; oracle c{trace['oracle_index']} {trace['oracle_ade_m']:.2f} m")
                axis.set_aspect('equal')
                axis.grid(alpha=0.2)
                if row_index == 0:
                    axis.legend(fontsize=8)
        figure.suptitle('Offline fixed-index DEV replay — ego centered; NOT closed-loop driving', fontsize=14)
        figure.tight_layout(rect=(0,0,1,0.97))
        figure.savefig(output/(direction+'_trajectory_diagnostics.png'), dpi=120)
        plt.close(figure)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args=parser.parse_args()
    render(args.source,args.output)
