"""HH_261001 - Read-only raw/swept body and all-candidate bounds for owned learned diagnostics."""

import argparse
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


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def region_measurement(shape, region):
    """HH_261001 - Report actual exterior area and interior clearance, never hide intrusion in a margin."""
    outside = shape.difference(region).area
    covered = region.covers(shape)
    return dict(outside_m2=outside, area_gate_clear=outside <= 1e-6,
                strictly_covered=covered,
                minimum_boundary_clearance_m=shape.distance(region.boundary) if covered else None)


def body_census(manifest, route, states, xml, actor_id):
    import carla
    capture = manifest['capture_contract']
    record = capture['actor_body_geometry']
    require(record['status'] == 'RECORDED' and record['source'] == 'carla.Vehicle.bounding_box'
            and record['reference_frame'] == 'CARLA actor-local; not ROS rear-axle base_link'
            and record['actor_id'] == actor_id, 'recorded body is not bound to command actor')
    map_record = capture['active_map_opendrive']
    require(map_record['status'] == 'RECORDED' and map_record['source'] == 'carla.Map.to_opendrive'
            and map_record['file'] == 'active_map_opendrive.xodr'
            and map_record['sha256'] == hashlib.sha256(xml).hexdigest()
            and map_record['size_bytes'] == len(xml), 'archived active-map identity differs')
    require(route['coordinate_reference'] == 'base_link', 'route reference differs')
    points = []
    for point in route['route']:
        body._triple(point)
        if not points or point['distance_m'] != points[-1]['distance_m']:
            points.append(point)
        else:
            require(all(point[key] == points[-1][key] for key in ('x', 'y', 'z', 'road_id', 'section_id', 'lane_id')),
                    'duplicate route distance changes geometry or lane')
    lanes = {body._triple(point) for point in points}
    carla_map = carla.Map('archived-learned-diagnostic', xml.decode())
    corridors = body.lane_corridors(carla_map, carla, points, xml, lanes, set(range(len(points))))
    box, radius = body._body_box(manifest, carla)
    region_cache = {}

    def regions(lo, hi):
        ids = frozenset(body._route_lanes_for_arc_interval(points, lo, hi))
        if ids not in region_cache:
            region_cache[ids] = {name: body.authorized_lane_region(corridors, ids, inset)
                                 for name, inset in [('raw_lane', 0.), ('inset_10cm_lane', body.LANE_INSET_M)]}
        return region_cache[ids], [list(item) for item in sorted(ids)]

    rows, previous = [], None
    for state in states:
        pose = state['actor_snapshot_transform_carla']
        arc, segment, distance = body._route_projection((pose['x'], pose['y']), points)
        footprint = body.body_footprint(box, pose, carla)
        allowed, ids = regions(arc - radius, arc + radius)
        row = dict(frame=state['frame'], timestamp=state['timestamp'], capture_phase=state['capture_phase'],
            research_control_source=state.get('research_control_source'),
            pose_carla=pose, independent_actor_arc_m=arc, independent_actor_route_distance_m=distance,
            intended_lane_ids=ids, body_polygon_carla_xy=list(footprint.exterior.coords),
            sampled_body={name: region_measurement(footprint, region) for name, region in allowed.items()},
            previous_sweep=None)
        if previous:
            old_state, old_arc, old_segment, old_footprint = previous
            require(state['frame'] == old_state['frame'] + 1
                    and abs(state['timestamp'] - old_state['timestamp'] - .05) <= 1e-6,
                    'native sweep requires contiguous 20 Hz states')
            require(-.25 <= arc - old_arc <= body.MAX_FRAME_PROGRESS_M
                    and 0 <= segment - old_segment <= 3, 'route projection jumps or reverses')
            envelope = body.swept_body(old_footprint, footprint, old_state['actor_snapshot_transform_carla'], pose, radius)
            allowed_sweep, sweep_ids = regions(old_arc - radius, arc + radius)
            row['previous_sweep'] = dict(from_frame=old_state['frame'], to_frame=state['frame'],
                intended_lane_ids=sweep_ids, envelope_carla_xy=list(envelope.exterior.coords),
                regions={name: region_measurement(envelope, region) for name, region in allowed_sweep.items()})
        rows.append(row)
        previous = state, arc, segment, footprint
    summary = {}
    for region in ('raw_lane', 'inset_10cm_lane'):
        for kind in ('sampled_body', 'previous_sweep'):
            values = [(row['frame'], row[kind][region] if kind == 'sampled_body' else row[kind]['regions'][region])
                      for row in rows if row[kind] is not None]
            violations = [frame for frame, value in values if not value['area_gate_clear']]
            margins = [(frame, value['minimum_boundary_clearance_m']) for frame, value in values
                       if value['minimum_boundary_clearance_m'] is not None]
            summary[kind + '_' + region] = dict(assessed_count=len(values), violation_count=len(violations),
                violation_frames=violations, maximum_outside_m2=max(value['outside_m2'] for _, value in values),
                minimum_interior_clearance_m=min(value for _, value in margins) if margins else None,
                minimum_clearance_frame=min(margins, key=lambda item: item[1])[0] if margins else None,
                status='PASS' if values and not violations else 'FAIL')
    return rows, summary


def candidate_bound_rows(commands, states, route, checkpoint):
    """HH_261001 - Math diagnostics of NN output; synthetic heading is not observed body yaw or expert truth."""
    by_frame = {state['frame']: state for state in states}
    seen, rows = set(), []
    for command in commands:
        output = command.get('raw_output')
        if output is None:
            continue
        frame = command['inference_frame']
        identity = hashlib.sha256(json.dumps(output, sort_keys=True, allow_nan=False).encode()).hexdigest()
        require(command['source_checkpoint_sha256'] == checkpoint, 'command checkpoint identity differs')
        if frame in seen:
            require(any(row['inference_frame'] == frame and row['raw_output_sha256'] == identity for row in rows),
                    'held command changed original NN output')
            continue
        seen.add(frame)
        state = by_frame[frame]
        anchor = raw.adapter._timestamp_ns(state['timestamp'], 'learned anchor')
        points = [[point['x'], point['y']] for point in route['route']]
        _, goal, arc = raw.contract._canonical_route_in_base(points,
            position_m=[state['x'], state['y'], state['z']],
            orientation_xyzw=[0., 0., math.sin(state['yaw'] / 2), math.cos(state['yaw'] / 2)],
            contract=raw.contract.load_contract(), context='NN-output mathematical diagnostic')
        selected = command.get('selected', {}).get('candidate_index') if command.get('selected') else None
        require(len(output['candidate_xy']) == len(output['candidate_speed']) == len(output['candidate_logits']) == 12,
                'learned output does not retain all 12 candidates')
        for index, (xy, speeds) in enumerate(zip(output['candidate_xy'], output['candidate_speed'])):
            require(len(xy) == len(speeds) == 64, 'learned candidate horizon differs')
            finite = (all(isinstance(point, list) and len(point) == 2 for point in xy)
                and all(type(value) in (int, float) and math.isfinite(value)
                        for value in [*speeds, *[coordinate for point in xy for coordinate in point]]))
            if not finite:
                rows.append(dict(inference_frame=frame, candidate_index=index, selected=index == selected,
                    raw_output_sha256=identity, source_checkpoint_sha256=checkpoint,
                    violation_point_indices={metric: [] for metric in targets.METRICS},
                    all_nine_mathematical_checks_clear=False, diagnostic=None,
                    unavailable_reason='nonfinite original candidate; no substitution or clipping',
                    training_data_approved=False))
                continue
            headings, previous, last_heading = [], [0., 0.], 0.
            for point in xy:
                if math.dist(point, previous) > targets.HEADING_MINIMUM_STEP_M:
                    last_heading = math.atan2(point[1] - previous[1], point[0] - previous[0])
                headings.append(last_heading)
                previous = point
            sample = dict(sample_id=f'learned_{frame}_{index}', anchor_timestamp_ns=anchor,
                ego=dict(linear_velocity_base_mps=[state['vx'], state['vy'], 0.]),
                navigation=dict(goal_base_m=list(goal), route_anchor_arc_m=arc),
                labels=dict(planning=dict(available=True, dt_s=.1, positions_base_xy_m=xy,
                    speed_mps=speeds, yaw_rad=headings, valid=[True] * 64,
                    target_timestamp_ns=[anchor + (i + 1) * 100_000_000 for i in range(64)], invalid_reason=[None] * 64)))
            diagnostic = targets.audit_sample(sample, route_length_m=route['route_length_m'],
                episode_start_ns=anchor, episode_end_ns=anchor + 6_400_000_000)
            failed = {metric: [step['index'] for step in diagnostic['steps'] if step['metrics'][metric][1]]
                      for metric in targets.METRICS}
            rows.append(dict(inference_frame=frame, candidate_index=index, selected=index == selected,
                raw_output_sha256=identity, source_checkpoint_sha256=checkpoint,
                violation_point_indices=failed, all_nine_mathematical_checks_clear=not any(failed.values()),
                diagnostic=diagnostic, training_data_approved=False))
    summaries = {}
    for name, selected_rows in [('all_candidates', rows), ('selected_candidates', [row for row in rows if row['selected']])]:
        summaries[name] = dict(candidate_count=len(selected_rows),
            clear_count=sum(row['all_nine_mathematical_checks_clear'] for row in selected_rows),
            unassessed_nonfinite_count=sum(row['diagnostic'] is None for row in selected_rows),
            metric_violation_candidate_counts={metric: sum(bool(row['violation_point_indices'][metric]) for row in selected_rows)
                                               for metric in targets.METRICS})
    return rows, summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists(), 'create-only diagnostic output required')
    episodes = [args.run / name for name in ('episode', 'episode.partial') if (args.run / name).is_dir()]
    require(len(episodes) == 1, 'exactly one raw episode required')
    episode = episodes[0]
    manifest = json.loads((episode / 'manifest.json').read_text())
    require(manifest['training_data_approved'] is False and manifest['vehicle_control_approved'] is False,
            'only explicitly unapproved simulation diagnostics are supported')
    capture = manifest['capture_contract']
    require(capture['physics_hz'] == 20 and capture['control_transport']['mode'] == 'acknowledged_batch',
            '20 Hz acknowledged snapshot source required')
    checkpoint = capture['learned_research']['checkpoint_sha256']
    owner_plan = json.loads((args.run / 'owner_plan.json').read_text())
    require(owner_plan['source_checkpoint_sha256'] == checkpoint, 'owner and raw checkpoint identity differ')
    files = [episode / name for name in ('manifest.json', 'states.jsonl', 'route.json', 'control_receipts.jsonl',
        'active_map_opendrive.xodr', 'learned_control.jsonl', 'learned_summary.json')]
    files += [args.run / 'owner_plan.json', args.run / 'owner_result.json', Path(__file__),
              *[Path(module.__file__) for module in (raw, ack, body, targets)]]
    for name in ('owner_diagnostic_result.json', 'owner_source_closure_verification.json'):
        if (args.run / name).exists():
            files.append(args.run / name)
    archived = args.run / 'source/tools/e2e/collect_carla_vad_expert.py'
    require(sha(archived) == manifest['provenance']['collector_sha256'], 'archived collector identity differs')
    files.append(archived)
    hashes = {str(path): sha(path) for path in files}
    require(sha(episode / 'route.json') == manifest['provenance']['route_sha256'], 'raw route identity differs')
    states, receipts, commands = [read_rows(episode / name) for name in
                                ('states.jsonl', 'control_receipts.jsonl', 'learned_control.jsonl')]
    route = json.loads((episode / 'route.json').read_text())
    actors = {row['actor_id'] for row in receipts}
    require(len(actors) == 1, 'multiple command actors in one body record')
    transport = ack.analyze_transport(states, receipts, capture['control_transport']['bootstrap_observation'], wheelbase=2.85)
    geometry, body_summary = body_census(manifest, route, states,
        (episode / 'active_map_opendrive.xodr').read_bytes(), next(iter(actors)))
    candidates, candidate_summary = candidate_bound_rows(commands, states, route, checkpoint)
    report = dict(schema='portable_e2e.owned_simulation_body_lane_diagnostic.v1',
        status='PASS' if transport['status'] == 'PASS' and all(row['status'] == 'PASS' for row in body_summary.values()) else 'FAIL',
        status_scope='recorded actor body and bounded snapshot-linear sweeps; not route or learned driving success',
        original_manifest_status=manifest['status'], original_capture_error=manifest.get('error'),
        original_owner_result=json.loads((args.run / 'owner_result.json').read_text()),
        checkpoint_sha256=checkpoint, source_sha256=hashes, body_geometry=body_summary,
        native_state_count=len(states), actual_state_control_source_counts=dict(Counter(row.get('research_control_source', 'unrecorded') for row in states)),
        observed_collision_events=sum(len(row['collision']) for row in states),
        observed_lane_invasion_events=sum(len(row['lane_invasion']) for row in states),
        transport_reconstruction=transport, candidate_nine_bound_diagnostic=candidate_summary,
        geometry_policy=dict(lane_sample_m=body.LANE_SAMPLE_M, lane_inset_m=body.LANE_INSET_M,
            seam_close_m=body.SEAM_GAP_CLOSE_M, sweep_position_guard_m=body.SWEEP_POSITION_GUARD_M,
            actor_box='all eight vertices with recorded full roll/pitch/yaw',
            actor_identity='recorded bbox actor_id matches every acknowledged command actor_id'),
        candidate_notice='All 12 original raw predictions retained per distinct inference. Targets.audit_sample is reused only as math; these are NOT expert labels. Ninth heading-envelope value uses XY-derived tangent, not an independently predicted or measured body-yaw label. Hypothetical 6.4s plan support is not actual observed future.',
        geometry_notice='No complete-episode validator bypass: diagnostic invokes pure geometry functions and independently binds failed-source metadata; original failed status is retained. Interior clearance is null on intrusion, never a false positive margin.',
        physical_continuous_time_proven=False, training_data_approved=False, vehicle_control_approved=False,
        learned_success_claimed=False)
    require(all(sha(Path(path)) == digest for path, digest in hashes.items()), 'input changed during audit')
    args.output.mkdir(parents=True)
    for name, rows in [('all_native_body_sweeps.jsonl.gz', geometry), ('all_candidate_nine_bounds.jsonl.gz', candidates)]:
        with gzip.open(args.output / name, 'wt') as stream:
            for row in rows:
                stream.write(json.dumps(row, allow_nan=False) + '\n')
    (args.output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: report[key] for key in ('status', 'native_state_count', 'body_geometry',
        'candidate_nine_bound_diagnostic', 'original_manifest_status', 'checkpoint_sha256')}, indent=2))


if __name__ == '__main__':
    main()
