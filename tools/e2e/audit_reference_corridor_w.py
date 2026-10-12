"""HH_260906 - Measure reference displacement without confusing proximity with lane containment."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def distances_to_polyline(points, route):
    """HH_260906 - Compute exact point-to-segment distance, including endpoints and duplicate points."""
    points, route = np.asarray(points, dtype=float), np.asarray(route, dtype=float)
    if (points.ndim != 2 or route.ndim != 2 or points.shape[1] != 2 or route.shape[1] != 2
            or len(route) < 2 or not np.isfinite(points).all() or not np.isfinite(route).all()):
        raise ValueError('finite two-dimensional points and a route of at least two points required')
    start, delta = route[:-1], np.diff(route, axis=0)
    squared = np.sum(delta*delta, axis=1)
    offset = points[:, None, :]-start
    ratio = np.clip(np.sum(offset*delta, axis=2)/np.maximum(squared, 1e-20), 0., 1.)
    return np.linalg.norm(offset-ratio[:, :, None]*delta, axis=2).min(axis=1)


def main():
    """HH_260906 - Audit all references and explicitly withhold lane and training approval."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    metadata = json.loads(args.metadata.read_text())
    traces = json.loads((args.run/'traces.json').read_text())
    rows = []
    for token, trace in traces.items():
        row = dict(token=token, lane_containment='UNVERIFIED', training_admitted=False)
        if trace is None:
            row['status'] = 'initial_state_rejected_by_V'
        else:
            # HH_260906 - Audit dense geometry, not only the sparse 10 Hz temporal points.
            points = np.asarray(trace['route_xy'])
            deviation = distances_to_polyline(points, metadata[token]['route'])
            worst = int(deviation.argmax())
            row.update(status='proximity_measured_only', maximum_deviation_m=float(deviation.max()),
                       p95_deviation_m=float(np.percentile(deviation, 95)),
                       worst_progress_m=trace['arc_m'][worst], worst_xy=points[worst].tolist(),
                       initial_deviation_m=float(deviation[0]))
        rows.append(row)
    sources = [args.metadata, args.run/'traces.json', Path(__file__)]
    result = dict(rows=rows, source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                  missing_evidence=['Matching map version and lane-boundary geometry',
                                    'Per-sample map-to-ego pose and coordinate convention',
                                    'Vehicle footprint and swept-envelope clearance',
                                    'Lane identity and junction transition correspondence'],
                  model_trained=False, live_simulator_accessed=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    for row in rows:
        print(json.dumps(row))


if __name__ == '__main__':
    main()
