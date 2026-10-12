"""HH_261001 - Census failed-capture windows without conversion, relabeling, or admission."""

import argparse
from bisect import bisect_left
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path

from scripts.e2e import audit_carla_acknowledged_control as ack
from scripts.e2e import audit_carla_raw_pre_admission as raw
from scripts.e2e import audit_future_actor_lane_containment as body
from scripts.e2e import audit_portable_target_feasibility as targets
from scripts.e2e import prepare_carla_common10_dataset as adapter


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_rows(path):
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt') as stream:
        return [json.loads(line) for line in stream]


def native_span_indices(stamps, camera_state_index, future_end_ns):
    """HH_261001 - Include a prior native guard and the real right interpolation bracket."""
    return max(0, camera_state_index - 1), min(len(stamps) - 1, bisect_left(stamps, future_end_ns))


def exclusions(history_complete, future_complete, span, camera_rows, future):
    reasons = []
    if not history_complete:
        reasons.append('incomplete_ten_camera_history')
    if not future_complete:
        reasons.append('incomplete_64_point_future')
    for key in ('native_speed_bound', 'native_rate_bound', 'ack_bound', 'body_bound',
                'sweep_bound', 'native_clock_bound', 'snapshot_conversion_bound',
                'no_recorded_event', 'upstream_of_terminal', 'driving_phase'):
        if not all(row[key] for row in span):
            reasons.append(key)
    if not all(row['same_recorded_frame_and_timestamp'] for row in camera_rows):
        reasons.append('camera_state_timestamp_binding')
    diagnostic = future.get('discrete_bound_diagnostic', {})
    if not diagnostic.get('steps') or any(failed for step in diagnostic.get('steps', [])
                                         for _, failed in step['metrics'].values()):
        reasons.append('nine_discrete_future_bounds')
    return reasons


def geometry_rows(manifest, route, states, xml):
    """HH_261001 - Pure geometry only; never forge complete status to enter audit_episode."""
    import carla
    assert hashlib.sha256(xml).hexdigest() == manifest['capture_contract']['active_map_opendrive']['sha256']
    assert len(xml) == manifest['capture_contract']['active_map_opendrive']['size_bytes']
    record = manifest['capture_contract']['actor_body_geometry']
    assert record['status'] == 'RECORDED' and record['source'] == 'carla.Vehicle.bounding_box'
    actor = record['actor_id']
    construction = manifest['capture_contract']['agent_initialization']['construction']
    assert construction['status'] == 'PASS'
    assert all(construction[side]['actor_id'] == actor for side in ('before', 'after'))
    points = []
    for point in route['route']:
        if not points or point['distance_m'] != points[-1]['distance_m']:
            points.append(point)
        else:
            assert point == points[-1] or all(point[key] == points[-1][key]
                for key in ('x', 'y', 'z', 'road_id', 'section_id', 'lane_id'))
    lane_ids = {body._triple(point) for point in points}
    carla_map = carla.Map('archived-failed-capture-diagnostic', xml.decode())
    corridors = body.lane_corridors(carla_map, carla, points, xml, lane_ids, set(range(len(points))))
    box, radius = body._body_box(manifest, carla)
    cache, rows, previous = {}, [], None

    def allowed(lo, hi):
        lanes = frozenset(body._route_lanes_for_arc_interval(points, lo, hi))
        if lanes not in cache:
            cache[lanes] = body.authorized_lane_region(corridors, lanes, body.LANE_INSET_M)
        return cache[lanes], sorted(lanes)

    for state in states:
        pose = state['actor_snapshot_transform_carla']
        arc, segment, distance = body._route_projection((pose['x'], pose['y']), points)
        footprint = body.body_footprint(box, pose, carla)
        region, lanes = allowed(arc - radius, arc + radius)
        outside = footprint.difference(region).area
        sweep_outside = None
        if previous:
            before, before_arc, before_footprint, before_segment = previous
            assert -.25 <= arc - before_arc <= body.MAX_FRAME_PROGRESS_M
            assert 0 <= segment - before_segment <= 3
            sweep = body.swept_body(before_footprint, footprint, before, pose, radius)
            sweep_region, _ = allowed(before_arc - radius, arc + radius)
            sweep_outside = sweep.difference(sweep_region).area
        rows.append(dict(frame=state['frame'], independent_actor_progress_m=arc,
            independent_actor_route_distance_m=distance, intended_lanes=lanes,
            body_outside_inset_m2=outside, prior_sweep_outside_inset_m2=sweep_outside,
            body_polygon_carla_xy=list(footprint.exterior.coords),
            body_bound=outside <= 1e-6, sweep_bound=sweep_outside is None or sweep_outside <= 1e-6))
        previous = pose, arc, footprint, segment
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--raw-audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists(), 'create-only diagnostic output required'
    episode = args.run / 'episode.partial'
    manifest = json.loads((episode / 'manifest.json').read_text())
    assert manifest['status'] == 'failed' and manifest['training_data_approved'] is False
    assert manifest['capture_contract']['low_speed_start_research']['profile']['profile_id'] == 'town07_low_speed_start_20261001_v2'
    states, cameras, receipts = [read_rows(episode / name) for name in
        ('states.jsonl', 'camera_frames.jsonl', 'control_receipts.jsonl')]
    route = json.loads((episode / 'route.json').read_text())
    proof = json.loads((args.raw_audit / 'summary.json').read_text())
    for name, digest in proof['input_sha256'].items():
        assert sha(episode / name) == digest, 'original raw audit input changed'
    for name, digest in proof['source_hashes']['files'].items():
        assert sha(Path(name)) == digest, 'original raw audit helper changed'
    source_files = [episode / name for name in ('manifest.json', 'route.json', 'states.jsonl',
        'camera_frames.jsonl', 'control_receipts.jsonl', 'expert_launch_control.jsonl', 'active_map_opendrive.xodr')]
    source_files += [args.run / 'owner_result.json', args.raw_audit / 'summary.json',
        args.raw_audit / 'all_raw_futures.jsonl.gz', Path(__file__),
        *[Path(module.__file__) for module in (raw, ack, body, targets, adapter)]]
    hashes = {str(path): sha(path) for path in source_files}
    futures = read_rows(args.raw_audit / 'all_raw_futures.jsonl.gz')
    assert [row['frame'] for row in futures] == [row['frame'] for row in cameras]
    timeline = raw.measured_timeline(states)
    pixels, images, anchors = raw.camera_audit(episode, manifest, cameras, timeline)
    assert pixels['all_pixel_and_metadata_checks_clear']
    bootstrap = manifest['capture_contract']['control_transport']['bootstrap_observation']
    transport = ack.analyze_transport(states, receipts, bootstrap, wheelbase=2.85)
    geometries = geometry_rows(manifest, route, states, (episode / 'active_map_opendrive.xodr').read_bytes())
    by_sequence = {row['sequence']: row for row in receipts}
    limit = targets.PHYSICAL_MAXIMUM_ACCELERATION_MPS2
    tol = targets.ROUNDING_TOLERANCE
    native = []
    for index, (state, geometric) in enumerate(zip(states, geometries)):
        speed = math.hypot(state['vx'], state['vy'])
        before = states[index - 1] if index else None
        dt = state['timestamp'] - before['timestamp'] if before else None
        rate = (speed - math.hypot(before['vx'], before['vy'])) / dt if before else None
        binding = state['control_transport']
        receipt = by_sequence[binding['expected_receipt_sequence']]
        reconstructed = ack.recompute_snapshot_state(state, 2.85)
        ack_bound = (receipt['status'] == 'ACKNOWLEDGED' and receipt['server_accepted']
            and receipt['before_frame'] == receipt['after_ack_frame'] == state['frame'] - 1
            and binding['status'] == 'PASS' and binding['frame_binding_pass']
            and not ack.mismatched_controls(state['current_control'], receipt['requested_control']))
        native.append(dict(**geometric, timestamp_ns=timeline[index][0], speed_mps=speed,
            native_scalar_rate_mps2=rate, raw_api_ax_mps2=state['ax'],
            recorded_rear_route_progress_m=state['route_progress_m'],
            capture_phase=state['capture_phase'], current_control=state['current_control'],
            native_speed_bound=speed <= targets.PHYSICAL_MAXIMUM_SPEED_MPS + tol,
            native_rate_bound=rate is None or abs(rate) <= limit + tol,
            native_clock_bound=before is None or (state['frame'] == before['frame'] + 1 and abs(dt - .05) <= 1e-6),
            ack_bound=ack_bound, snapshot_conversion_bound=max(abs(state[k] - v) for k, v in reconstructed.items()) <= 1e-9,
            no_recorded_event=not state['collision'] and not state['lane_invasion'],
            upstream_of_terminal=state.get('goal_stop', {}).get('terminal_overshoot_m', -1) <= 0,
            driving_phase=state['capture_phase'] == 'driving'))
    state_index = {row['frame']: i for i, row in enumerate(states)}
    stamps = [row[0] for row in timeline]
    windows = []
    for i, (camera, future) in enumerate(zip(cameras, futures)):
        history_start = max(0, i - 9)
        future_complete = future['valid_future_points'] == 64
        lo, hi = native_span_indices(stamps, state_index[cameras[history_start]['frame']],
                                    future['anchor_timestamp_ns'] + 6_400_000_000)
        span = native[lo:hi + 1]
        history_future_camera = anchors[history_start:min(len(cameras), i + 65)]
        reasons = exclusions(i >= 9, future_complete, span, history_future_camera, future)
        # HH_261001 - History edges use existing first-step checks; no custom native XY threshold.
        if any(failed for row in futures[history_start:i] for step in row.get('discrete_bound_diagnostic', {}).get('steps', [])[:1]
               for _, failed in step['metrics'].values()):
            reasons.append('nine_discrete_history_edge_bounds')
        current = states[state_index[camera['frame']]]
        windows.append(dict(camera_index=i, frame=camera['frame'], capture_phase=camera['capture_phase'],
            history_camera_index_range=[history_start, i], history_plus_future_native_frame_range=[span[0]['frame'], span[-1]['frame']],
            history_plus_future_timestamp_ns=[span[0]['timestamp_ns'], span[-1]['timestamp_ns']],
            input_vx_mps=current['vx'], input_ax_mps2=current['ax'], route_progress_m=current['route_progress_m'],
            full_future=future_complete, exclusion_reasons=reasons, physical_window_screen_clear=not reasons,
            minimum_native_rate_mps2=min(row['native_scalar_rate_mps2'] for row in span if row['native_scalar_rate_mps2'] is not None),
            maximum_native_rate_mps2=max(row['native_scalar_rate_mps2'] for row in span if row['native_scalar_rate_mps2'] is not None),
            future_speed_0p3_mps=future['diagnostic_endpoint_planar_speed_mps'][2] if future['valid_future_points'] >= 3 else None,
            training_data_approved=False, route_success_claimed=False))
    try:
        adapter._require_native_admission_not_denied(manifest)
    except adapter.AdapterError as error:
        admission_block = str(error)
    else:
        raise AssertionError('native source must remain explicitly prohibited')
    clear = [row for row in windows if row['physical_window_screen_clear']]
    band = [row for row in clear if 1.5 <= row['input_vx_mps'] <= 2.5]
    summary = dict(schema='carla.failed_episode_window_census.research.v1',
        status='DIAGNOSTIC_ONLY_NOT_ADMITTED', episode_status=manifest['status'], episode_error=manifest.get('error'),
        original_owner_result=json.loads((args.run / 'owner_result.json').read_text()),
        source_sha256=hashes, source_image_sha256={row['path']: row['sha256'] for row in images},
        camera_anchors=len(cameras), native_states=len(states), full_future_anchors=sum(row['full_future'] for row in windows),
        physical_screen_clear_windows=len(clear), exclusion_reason_counts=dict(Counter(reason for row in windows for reason in row['exclusion_reasons'])),
        physical_clear_camera_index_range=[clear[0]['camera_index'], clear[-1]['camera_index']] if clear else None,
        physical_clear_anchor_progress_range_m=[clear[0]['route_progress_m'], clear[-1]['route_progress_m']] if clear else None,
        speed_1p5_to_2p5_clear_count=len(band),
        speed_1p5_to_2p5_abs_near_future_dv_at_most_0p1_count=sum(abs(row['future_speed_0p3_mps'] - row['input_vx_mps']) <= .1 for row in band),
        maximum_body_outside_inset_m2=max(row['body_outside_inset_m2'] for row in native),
        maximum_sweep_outside_inset_m2=max(row['prior_sweep_outside_inset_m2'] or 0 for row in native),
        original_transport_reconstruction=transport, pixel_checks=pixels,
        native_rate_violations=[row for row in native if not row['native_rate_bound']],
        native_admission_block=admission_block, admission_blocked_windows=len(windows),
        contract_notice='Existing adapter rejects explicit denial, failed status and absent catalog goal; sealed moving exception is not applicable. No original admission or complete-episode body validator was patched or bypassed.',
        policy=dict(history_camera_observations=10, history_seconds=.9, future_points=64, future_seconds=6.4,
            preceding_native_guard_seconds=.05, native_scalar_rate_bound_mps2=limit,
            discrete_nine_metrics=list(targets.METRICS), body_inset_m=body.LANE_INSET_M,
            membership='all camera anchors, with overlapping windows and every exclusion retained',
            screen='entire history/native guard/future bracket must be driving, event-free and upstream; no score selection'),
        training_data_approved=False, common10_created=False, route_success_claimed=False,
        physical_continuous_time_proven=False,
        limitations=['Necessary recorded-window checks, not admission or a full route success.',
            'Overlapping windows from one route are not independent episodes.',
            'Nine discrete checks preserve original limits; body sweep is bounded snapshot-linear, not continuous physical proof.',
            'ACK establishes server acceptance and observed-control agreement, not independently measured physical actuation.',
            'Pixels decode and calibration bind; visual asset/semantic quality is not thereby proven.'])
    assert all(sha(Path(path)) == digest for path, digest in hashes.items())
    assert all(sha(episode / path) == digest for path, digest in summary['source_image_sha256'].items())
    args.output.mkdir(parents=True)
    for name, rows in [('all_windows.jsonl.gz', windows), ('all_native_geometry_and_bounds.jsonl.gz', native)]:
        with gzip.open(args.output / name, 'wt') as stream:
            for row in rows:
                stream.write(json.dumps(row, allow_nan=False) + '\n')
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: summary[key] for key in ('camera_anchors', 'full_future_anchors',
        'physical_screen_clear_windows', 'exclusion_reason_counts', 'physical_clear_camera_index_range',
        'speed_1p5_to_2p5_clear_count', 'speed_1p5_to_2p5_abs_near_future_dv_at_most_0p1_count',
        'native_admission_block')}, indent=2))


if __name__ == '__main__':
    main()
