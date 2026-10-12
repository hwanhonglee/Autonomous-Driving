"""HH_260906 - Test true-arc curvature preview offline, never as learned or closed-loop driving."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from portable_e2e.contract import ContractError
from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory
from portable_e2e.torch_dataset import _linear_route_samples


def smooth_route(points):
    """HH_260906 - Recompute actual arc length after Hermite interpolation and the ego-origin join."""
    route = np.asarray(_linear_route_samples(points, limit=128), dtype=float)
    arc = np.r_[0., np.linalg.norm(np.diff(route, axis=0), axis=1).cumsum()]
    tangent = np.gradient(route, arc, axis=0)
    sample = np.linspace(0., arc[-1], max(3, int(arc[-1] / .02) + 1))
    ix = np.clip(np.searchsorted(arc, sample, side='right') - 1, 0, len(arc)-2)
    h = arc[ix+1] - arc[ix]
    t = ((sample-arc[ix])/h)[:, None]
    xy = (2*t**3-3*t**2+1)*route[ix] + (t**3-2*t**2+t)*h[:, None]*tangent[ix]
    xy += (-2*t**3+3*t**2)*route[ix+1] + (t**3-t**2)*h[:, None]*tangent[ix+1]
    join = np.clip(sample/5., 0., 1.)[:, None]
    xy -= route[0]*(1.-3*join**2+2*join**3)
    keep = np.r_[True, np.linalg.norm(np.diff(xy, axis=0), axis=1) > 1e-8]
    xy = xy[keep]
    arc = np.r_[0., np.linalg.norm(np.diff(xy, axis=0), axis=1).cumsum()]
    derivative = np.gradient(xy, arc, axis=0)
    yaw = np.unwrap(np.arctan2(derivative[:, 1], derivative[:, 0]))
    curvature = np.abs(np.gradient(yaw, arc))
    return xy, arc, yaw, curvature


def speed_envelope(arc, curvature, current):
    """HH_260906 - Reject infeasible measured initial speed instead of silently lowering it."""
    if not np.isfinite(current) or current < 0:
        raise ValueError('invalid initial speed')
    speed = np.minimum(8., np.sqrt(1.5/np.maximum(curvature, 1e-9)))
    speed[-1] = 0.
    ds = np.diff(arc)
    for i in range(len(speed)-2, -1, -1):
        speed[i] = min(speed[i], np.sqrt(speed[i+1]**2 + 4.*ds[i]))
    if current > speed[0] + 1e-8:
        raise ValueError(f'initial speed {current:.6f} exceeds feasible envelope {speed[0]:.6f}')
    speed[0] = current
    for i in range(1, len(speed)):
        speed[i] = min(speed[i], np.sqrt(speed[i-1]**2 + 3.*ds[i-1]))
    return speed


def evaluate(points, current):
    """HH_260906 - Check every 6.4-second window of an ideal geometric rollout, including endpoint stopping."""
    xy, arc, yaw, curvature = smooth_route(points)
    report = dict(learned_model=False, closed_loop=False, training_admitted=False,
                  maximum_curvature_inv_m=float(curvature.max()), initial_speed_mps=current)
    try:
        speed = speed_envelope(arc, curvature, current)
    except ValueError as error:
        return dict(report, feasible=False, rejection=str(error)), None
    times = np.r_[0., np.cumsum(2*np.diff(arc)/(speed[:-1]+speed[1:]))]
    ticks = np.arange(int(np.ceil(times[-1]/.1))+2)*.1
    # HH_260906 - Integrate each constant-acceleration segment exactly, including the final stop.
    ix = np.clip(np.searchsorted(times, ticks, side='right')-1, 0, len(times)-2)
    elapsed = np.minimum(ticks, times[-1])-times[ix]
    spatial_acceleration = np.diff(speed**2)/(2*np.diff(arc))
    distance = arc[ix]+speed[ix]*elapsed+.5*spatial_acceleration[ix]*elapsed**2
    distance = np.clip(distance, 0., arc[-1])
    driven = np.column_stack([np.interp(distance, arc, xy[:, d]) for d in (0, 1)])
    # HH_260906 - Report chord-derived speed to match the runtime ABI; audit acceleration independently.
    segments = np.diff(driven, axis=0)
    speeds = np.linalg.norm(segments, axis=1)/.1
    headings = np.interp(distance, arc, yaw)
    padded_xy = np.vstack([driven, np.repeat(driven[-1:], 64, axis=0)])
    padded_speed = np.r_[speeds, np.zeros(64)]
    failures = []
    for start in range(len(driven)-1):
        heading = 0. if start == 0 else headings[start]
        c, s = np.cos(heading), np.sin(heading)
        rotation = np.array([[c, -s], [s, c]])
        local = (padded_xy[start+1:start+65]-driven[start]) @ rotation
        try:
            validate_and_select_trajectory([local.tolist()], [padded_speed[start:start+64].tolist()], [0.],
                RuntimeGateConfig(candidate_count=1), current_speed_mps=current if start == 0 else float(speeds[start-1]))
        except ContractError as error:
            failures.append(dict(start_tick=start, reason=str(error)))
    # HH_260906 - Near-right-angle coverage needs 85 degrees plus three metres beyond its first crossing.
    turn_indices = np.flatnonzero(np.abs(yaw-yaw[0]) >= np.deg2rad(85.))
    exit_distance = float(arc[turn_indices[0]]+3.) if len(turn_indices) else None
    covered = exit_distance is not None and distance[-1] >= exit_distance
    acceleration = np.diff(np.r_[current, speeds, 0.])/.1
    report.update(feasible=True, duration_s=float(ticks[-1]), distance_m=float(distance[-1]),
                  endpoint_speed_mps=float(speeds[-1]), maximum_speed_mps=float(speeds.max()),
                  maximum_acceleration_mps2=float(acceleration.max()),
                  maximum_deceleration_mps2=float(-acceleration.min()),
                  turn_exit_distance_m=exit_distance, geometric_turn_covered=bool(covered),
                  turn_threshold_deg=85., after_turn_margin_m=3.,
                  windows_checked=len(driven)-1, windows_failed=len(failures), failures=failures,
                  all_windows_pass=not failures, full_turn_reference_pass=bool(covered and not failures))
    trace = dict(time_s=ticks.tolist(), xy=driven.tolist(), distance_m=distance.tolist(),
                 segment_speed_mps=speeds.tolist(), route_xy=xy.tolist(), arc_m=arc.tolist(),
                 curvature_inv_m=curvature.tolist(), envelope_mps=speed.tolist())
    return report, trace


def main():
    """HH_260906 - Freeze all six inputs and retain every failure without changing production gates."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = args.metadata.read_bytes()
    rows, traces = [], {}
    for token, item in json.loads(source).items():
        report, trace = evaluate(item['route'], item['current_speed'])
        rows.append(dict(token=token, **report))
        traces[token] = trace
        print(json.dumps({k: v for k, v in rows[-1].items() if k != 'failures'}))
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(rows=rows, metadata_sha256=hashlib.sha256(source).hexdigest(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  limitation='Ideal path traversal only: no vehicle dynamics, perception, obstacles or lane-boundary verification.')
    (args.output/'results.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    (args.output/'traces.json').write_text(json.dumps(traces, allow_nan=False))


if __name__ == '__main__':
    main()
