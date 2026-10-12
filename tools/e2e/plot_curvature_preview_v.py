"""HH_260906 - Publish offline path and speed evidence without depicting a simulated drive."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    """HH_260906 - Plot the two previously failing right-turn references on equal-scale axes."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    traces = json.loads((args.run/'traces.json').read_text())
    results = json.loads((args.run/'results.json').read_text())['rows']
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    for row, suffix in enumerate((':000464', ':000487')):
        token = next(t for t in traces if t.endswith(suffix))
        trace = traces[token]
        report = next(r for r in results if r['token'] == token)
        xy = np.asarray(trace['xy'])
        path, speed = axes[row]
        path.plot(xy[:, 0], xy[:, 1], label='Ideal reference path')
        path.scatter(*xy[0], color='green', label='Start')
        path.scatter(*xy[-1], color='red', label='Stop')
        path.set_aspect('equal', adjustable='datalim')
        path.set(xlabel='Forward x (m)', ylabel='Left y (m)', title=f'{suffix[1:]}: full available route')
        path.legend()
        distance = np.asarray(trace['distance_m'])[1:]
        speed.plot(distance, np.asarray(trace['segment_speed_mps'])*3.6, label='Reference speed')
        speed.axvline(report['turn_exit_distance_m'], color='purple', linestyle='--', label='85 deg + 3 m coverage')
        speed.set(xlabel='Progress (m)', ylabel='Speed (km/h)', title=f"{report['windows_checked']} windows checked; 0 rejected")
        speed.legend()
        path.grid(alpha=.3)
        speed.grid(alpha=.3)
    fig.suptitle('OFFLINE GEOMETRIC REFERENCE — NOT LEARNED / NOT VEHICLE DRIVING')
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
