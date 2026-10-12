"""HH_261001 - Render recorded geometry only, preserving inset failure and raw-lane distinction."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path

from scripts.e2e import audit_future_actor_lane_containment as body


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    import carla
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from shapely.geometry import Polygon

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists()
    episode = args.run / 'episode.partial'
    if not episode.exists():
        episode = args.run / 'episode'
    report = json.loads((args.audit / 'summary.json').read_text())
    for name in ('manifest.json', 'route.json', 'states.jsonl', 'active_map_opendrive.xodr'):
        assert sha(episode / name) == report['source_sha256'][str(episode / name)]
    with gzip.open(args.audit / 'all_native_body_sweeps.jsonl.gz', 'rt') as stream:
        rows = [json.loads(line) for line in stream]
    possible = [row for row in rows if row['previous_sweep']]
    witness = max(possible, key=lambda row: row['previous_sweep']['regions']['inset_10cm_lane']['outside_m2'])
    before = next(row for row in rows if row['frame'] == witness['previous_sweep']['from_frame'])
    route = json.loads((episode / 'route.json').read_text())
    points = []
    for point in route['route']:
        if not points or point['distance_m'] != points[-1]['distance_m']:
            points.append(point)
    xml = (episode / 'active_map_opendrive.xodr').read_bytes()
    carla_map = carla.Map('archived-body-witness', xml.decode())
    ids = {tuple(lane) for lane in witness['previous_sweep']['intended_lane_ids']}
    relevant = {i for i, point in enumerate(points) if body._triple(point) in ids}
    corridors = body.lane_corridors(carla_map, carla, points, xml, ids, relevant)
    raw = body.authorized_lane_region(corridors, ids, 0.)
    inset = body.authorized_lane_region(corridors, ids, body.LANE_INSET_M)
    envelope = Polygon(witness['previous_sweep']['envelope_carla_xy'])
    outside = envelope.difference(inset)
    expected = witness['previous_sweep']['regions']['inset_10cm_lane']['outside_m2']
    assert abs(outside.area - expected) < 1e-9
    figure, axis = plt.subplots(figsize=(8, 9))

    def fill(geometry, color, alpha, label=None, edgecolor=None, linewidth=1):
        pieces = list(geometry.geoms) if hasattr(geometry, 'geoms') else [geometry]
        for i, polygon in enumerate(pieces):
            if polygon.is_empty or not hasattr(polygon, 'exterior'):
                continue
            x, y = polygon.exterior.xy
            axis.fill(x, y, color=color, alpha=alpha, label=label if i == 0 else None,
                      edgecolor=edgecolor, linewidth=linewidth)

    fill(raw, 'lightgray', 1, 'Recorded intended lane')
    fill(inset, '#dcf3df', 1, 'Lane inset by unchanged 0.10 m')
    fill(envelope, '#e9a72e', .60, 'Bounded 20 Hz sweep envelope')
    fill(Polygon(before['body_polygon_carla_xy']), '#61b5e8', .6, f"Body frame {before['frame']}")
    fill(Polygon(witness['body_polygon_carla_xy']), '#15507a', .55, f"Body frame {witness['frame']}")
    fill(outside, '#de172c', 1, 'Outside inset (not outside raw lane)')
    xmin, ymin, xmax, ymax = envelope.bounds
    axis.set_xlim(xmin - .65, xmax + .65)
    axis.set_ylim(ymin - .65, ymax + .65)
    axis.set_aspect('equal')
    axis.set_xlabel('Archived CARLA world x (m)')
    axis.set_ylabel('Archived CARLA world y (m)')
    raw_measurement = witness['previous_sweep']['regions']['raw_lane']
    elapsed = witness['timestamp'] - rows[0]['timestamp']
    axis.set_title(f"W1 recorded sweep witness: frame {before['frame']} → {witness['frame']}\n"
        f"t = {elapsed:.2f} s | inset outside area {expected:.6f} m²\n"
        f"raw-lane clearance {raw_measurement['minimum_boundary_clearance_m']:.4f} m < 0.10 m")
    axis.legend(loc='upper center', bbox_to_anchor=(.5, -.085), ncol=1, fontsize=9)
    figure.text(.5, .015, 'Bounded snapshot-linear geometry only; original budget failure retained. No promotion or data admission.',
                ha='center', fontsize=8)
    figure.subplots_adjust(bottom=.24, top=.88)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=160)
    plt.close(figure)
    proof = dict(schema='portable_e2e.recorded_body_margin_witness.v1',
        audit_summary_sha256=sha(args.audit / 'summary.json'),
        native_geometry_sha256=sha(args.audit / 'all_native_body_sweeps.jsonl.gz'),
        renderer_sha256=sha(Path(__file__)), png_sha256=sha(args.output),
        source_checkpoint_sha256=report['checkpoint_sha256'], frame_pair=[before['frame'], witness['frame']],
        relative_time_s=elapsed, independent_actor_arc_m=witness['independent_actor_arc_m'],
        inset_outside_area_m2=expected, raw_lane_minimum_clearance_m=raw_measurement['minimum_boundary_clearance_m'],
        original_manifest_status=report['original_manifest_status'], training_data_approved=False,
        physical_continuous_time_proven=False)
    args.output.with_suffix('.json').write_text(json.dumps(proof, indent=2) + '\n')
    print(json.dumps(proof, indent=2))


if __name__ == '__main__':
    main()
