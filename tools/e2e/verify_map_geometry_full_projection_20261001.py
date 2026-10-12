#!/usr/bin/env python3
"""HH_261001 - No-forward full240 projection replay with independent frame-vote fusion."""

from io import BytesIO
import json
import math
from pathlib import Path
import signal
import time
import numpy as np
from PIL import Image
from scripts.e2e import project_map_geometry_full_20261001 as p


def expected_artifacts():
    names = {'started.json'}
    for scenario in p.SCENARIOS:
        names.update(f'{scenario}_{i:03d}_frame.json' for i in range(240))
        names.update(f'{scenario}_{i:03d}_{arm}_confidence.npz' for i in range(240) for arm in p.ARMS)
        names.update(f'{scenario}_{key}_grid.npz' for key in ('oracle',*p.ARMS))
        names.update(f'{scenario}_{i:03d}_bev.png' for i in p.INDICES)
        names.add(f'{scenario}_full_route_bev.png')
    return names


def validate_summary(summary, registration, directory_names):
    p.c.require(summary.get('schema') == 'portable_e2e.map_geometry_full_projection.v1'
        and summary.get('status') == 'FULL_RECORDING_ORACLE_ASSISTED_ACCUMULATION_DIAGNOSTIC'
        and summary.get('preregistration_sha256') == p.c.digest(p.PREREG)
        and p.encoded(summary.get('policy')) == p.encoded(p.policy())
        and summary.get('bindings') == registration['bindings'] and summary.get('sources') == registration['sources']
        and set(summary.get('scenarios',{})) == set(p.SCENARIOS)
        and summary.get('attempted_forwards') == 960 and summary.get('confidence_forwards') == 960
        and summary.get('oracle_frames') == 480
        and summary.get('learned_map_updates') == 960 and summary.get('completed_frame_count') == 480
        and summary.get('source_and_inputs_unchanged') is True and summary.get('evaluation_is_not_live') is True
        and summary.get('online_offline_equality') == 'PENDING_SEPARATE_NO_FORWARD_REPLAY',
        'complete frozen full240 study required; no partial or live-success substitution')
    names = expected_artifacts()
    p.c.require(set(summary['artifact_sha256']) == names and set(directory_names) == names|{'summary.json'},
                'complete exact artifact set required; no omitted/extra/failure/old-verifier files')


def independent_update(grid, observation):
    """HH_261001 - Recount max-confidence per cell/class per frame without core update()."""
    p.c.require(observation.frame_id == grid.frame_id and observation.source == grid.source,
                'observation frame/source differs')
    stamp = observation.timestamp_s
    p.c.require(math.isfinite(stamp) and (grid.timestamp_s is None or stamp > grid.timestamp_s),
                'strict chronological independent update required')
    votes = {}
    for x,y,label,confidence in observation.points:
        p.c.require(all(math.isfinite(v) for v in (x,y,confidence)) and 0 <= confidence <= 1
            and label in p.TABLE.values(), 'finite six-static-class evidence required')
        if confidence == 0:
            continue
        key = ((math.floor(x/.2),math.floor(y/.2)),label)
        if key not in votes or confidence > votes[key]:
            votes[key] = confidence
    p.c.require(len(set(grid.cells)|{key[0] for key in votes}) <= 200000, 'fixed map capacity exceeded')
    for (cell,label),score in votes.items():
        labels = grid.cells.setdefault(cell,{})
        if label not in labels:
            labels[label] = [1,float(score)]
        else:
            labels[label][0] += 1
            labels[label][1] += float(score)
    grid.timestamp_s, grid.dynamic = stamp, []


def verify_incremental(row, maps):
    p.c.require(row['accumulated_cells'] == {key:len(grid.cells) for key,grid in maps.items()},
                'incremental cell count differs')
    p.c.require(row['accumulated_snapshot_sha256'] == {key:p.sha(p.encoded(grid.snapshot())) for key,grid in maps.items()},
                'exact incremental confidence/count/timestamp state differs')


def restore_saved_confidence(path, projection):
    p.c.require(path.name == projection['confidence_file'], 'fixed confidence filename differs')
    with np.load(path,allow_pickle=False) as data:
        p.c.require(data.files == ['confidence_stride3'], 'exact confidence archive key required')
        sampled = data['confidence_stride3'].copy()
    full = p.restore_confidence(sampled)
    p.c.require(p.sha(sampled.tobytes()) == projection['confidence_numeric_sha256'], 'exact confidence numeric hash differs')
    p.c.require(projection['exact_saved_argmax'] is True, 'source-bound online argmax assertion absent')
    timing = projection['forward_and_projection_ms']
    p.c.require(not isinstance(timing,bool) and math.isfinite(timing) and timing >= 0, 'finite recorded timing required')
    return full


def verify_final_grid(arrays, meta, actual):
    recovered = p.unpack_grid(arrays,meta)
    expected = p.pack_grid(actual)
    p.c.require(set(arrays) == set(expected), 'final grid fields differ')
    for name in expected:
        left,right = arrays[name],expected[name]
        p.c.require(left.dtype == right.dtype and left.shape == right.shape and left.tobytes() == right.tobytes(),
                    'final numeric grid/order/tie/padding differs: '+name)
    p.c.require(recovered.snapshot() == actual.snapshot() and meta['cells'] == len(actual.cells)
        and meta['snapshot_sha256'] == p.sha(p.encoded(actual.snapshot()))
        and meta['unobserved_is_unknown'] is True and meta['hd_map_approved'] is False,
        'final snapshot/provenance differs')


def independent_overview(records,maps):
    p.c.require(len(records) == 240, 'overview must preserve all240 poses')
    xs,ys = [r['ego_x'] for r in records],[r['ego_y'] for r in records]
    for grid in maps.values():
        for ix,iy in grid.cells:
            xs += [ix*.2,(ix+1)*.2]; ys += [iy*.2,(iy+1)*.2]
    p.c.require(all(math.isfinite(v) for v in xs+ys), 'finite overview support required')
    bounds = [min(xs)-1,min(ys)-1,max(xs)+1,max(ys)+1]
    scale = min(600/(bounds[2]-bounds[0]),560/(bounds[3]-bounds[1]))
    return dict(bounds_xy_m=bounds,pixels_per_m=scale,pose_count=240,all_stored_cell_extents_included=True)


def compare_png(path, expected, shape):
    with Image.open(path) as image:
        actual = np.asarray(image)
    with Image.open(BytesIO(expected)) as image:
        repeated = np.asarray(image)
    p.c.require(actual.shape == shape and actual.dtype == np.uint8 and np.array_equal(actual,repeated),
                'every native map PNG pixel must replay exactly')
    return actual


def verify():
    # HH_261001 - No model construction, Torch import, checkpoint load or forward in this verifier.
    previous_handler = signal.signal(signal.SIGALRM,p._timeout)
    signal.setitimer(signal.ITIMER_REAL,1800)
    begin = time.monotonic()
    learned = oracle_count = snapshots = grids = pngs = completed = 0
    try:
        registration = p.validate()
        p.c.require(p.OUTPUT.is_dir() and p.OUTPUT.resolve() == p.OUTPUT, 'existing canonical output required')
        p.used_bytes()
        summary_path = p.OUTPUT/'summary.json'
        summary_sha = p.c.digest(summary_path)
        summary = p.c.read_json(summary_path)
        validate_summary(summary,registration,[path.name for path in p.OUTPUT.iterdir()])
        for name,expected in summary['artifact_sha256'].items():
            p.c.require(p.c.digest(p.OUTPUT/name) == expected, 'recorded artifact changed: '+name)
        started = p.c.read_json(p.OUTPUT/'started.json')
        p.c.require(started['schema'] == 'portable_e2e.map_geometry_full_projection_started.v1'
            and p.encoded(started['policy']) == p.encoded(p.policy())
            and started['preregistration_sha256'] == p.c.digest(p.PREREG)
            and started['bindings'] == registration['bindings'] and started['sources'] == registration['sources'],
            'started source/study receipt differs')
        for scenario in p.SCENARIOS:
            binding = registration['bindings']['scenarios'][scenario]
            published = summary['scenarios'][scenario]
            p.c.require(published['rows'] == [f'{scenario}_{i:03d}_frame.json' for i in range(240)]
                and published['pngs'] == [*(f'{scenario}_{i:03d}_bev.png' for i in p.INDICES),f'{scenario}_full_route_bev.png']
                and set(published['grids']) == {'oracle',*p.ARMS}, 'full fixed row/grid/PNG sequence differs')
            for key in ('evaluation_role','source_qa_status','original_owner_status','original_live_replay_exact_match'):
                p.c.require(published[key] == binding[key], 'original QA/LEFT forensic boundary changed')
            source = Path(binding['source']); records = p.c.read_json(source/'records.json')
            p.c.require(len(records) == 240 and p.c.digest(source/'records.json') == binding['records_sha256'], 'full source sequence differs')
            predictions = p.load_predictions(scenario)
            frame_id = 'carla_ue:'+binding['town']
            maps = {'oracle':p.new_grid(frame_id,'oracle_sensor')}
            maps.update({arm:p.new_grid(frame_id,'learned_prediction') for arm in p.ARMS})
            for index,record in enumerate(records):
                row = p.c.read_json(p.OUTPUT/published['rows'][index])
                p.c.require(row['index'] == index and row['sensor_frame'] == record['frame']
                    and row['timestamp_s'] == record['timestamp_s'] and set(row['projections']) == {'oracle',*p.ARMS},
                    'chronological frame identity/projection set differs')
                raw_path = source/(record['stem']+'.npz')
                p.c.require(p.c.digest(raw_path) == binding['frames'][index]['artifact_sha256']['.npz'], 'raw oracle geometry changed')
                with np.load(raw_path,allow_pickle=False) as raw:
                    depth,semantic,pose = (raw[key] for key in ('depth','semantic','world_from_camera'))
                observations = {'oracle':p.project_frame(depth,semantic,pose,frame_id=frame_id,
                    timestamp_s=record['timestamp_s'],fov_deg=90,label_table=p.RAW_TABLE,stride=3,max_depth_m=40)}
                p.c.require(p.point_receipt(observations['oracle']) == row['projections']['oracle'], 'oracle point identity differs')
                oracle_count += 1
                for arm in p.ARMS:
                    projection = row['projections'][arm]
                    path = p.OUTPUT/f'{scenario}_{index:03d}_{arm}_confidence.npz'
                    confidence = restore_saved_confidence(path,projection)
                    observations[arm] = p.project_predicted_frame(p.metric_depth(depth),predictions[arm][index],confidence,pose,
                        frame_id=frame_id,timestamp_s=record['timestamp_s'],fov_deg=90,
                        label_table=p.TABLE,stride=3,max_depth_m=40,min_confidence=.5)
                    receipt = p.point_receipt(observations[arm])
                    p.c.require(all(projection[key] == value for key,value in receipt.items()), 'learned projected point identity differs')
                    learned += 1
                for key,observation in observations.items():
                    independent_update(maps[key],observation)
                verify_incremental(row,maps); snapshots += len(maps)
                if index in p.INDICES:
                    name = f'{scenario}_{index:03d}_bev.png'
                    p.c.require(row['png'] == name and row['comparisons'] == {arm:p.comparison(maps['oracle'],maps[arm]) for arm in p.ARMS},
                                'fixed incremental comparison differs')
                    canvas = compare_png(p.OUTPUT/name,p.render_fixed(scenario,index,record,maps),(490,1920,3))
                    for column in range(3):
                        p.c.require(tuple(canvas[255,column*640+320]) == (255,110,65)
                            and tuple(canvas[61,column*640+1]) == (22,31,43), 'ego center/unknown background differs')
                    pngs += 1
                else:
                    p.c.require('png' not in row and 'comparisons' not in row, 'no adaptive intermediate panel selection')
                completed += 1
                if (index+1)%40 == 0:
                    print(json.dumps(dict(scenario=scenario,replayed_frames=index+1,learned_observations=learned)),flush=True)
            for key,grid in maps.items():
                meta = published['grids'][key]
                name = f'{scenario}_{key}_grid.npz'
                p.c.require(meta['file'] == name and meta['frame_id'] == frame_id
                    and meta['source'] == grid.source and meta['timestamp_s'] == records[-1]['timestamp_s'], 'final grid identity differs')
                with np.load(p.OUTPUT/name,allow_pickle=False) as data:
                    p.c.require(len(data.files) == len(set(data.files)), 'duplicate grid archive key')
                    arrays = {name:data[name].copy() for name in data.files}
                verify_final_grid(arrays,meta,grid); grids += 1
            overview,overview_meta = p.render_overview(scenario,records,maps)
            p.c.require(published['overview'] == independent_overview(records,maps) == overview_meta,
                        'overview bounds must include all stored footprints/poses with common scale')
            compare_png(p.OUTPUT/f'{scenario}_full_route_bev.png',overview,(710,1920,3)); pngs += 1
            p.c.require(published['comparisons'] == {arm:p.comparison(maps['oracle'],maps[arm]) for arm in p.ARMS},
                        'final grid comparisons differ')
        p.c.require((learned,oracle_count,snapshots,grids,pngs,completed) == (960,480,1440,6,8,480),
                    'complete fixed denominator required')
        p.validate()
        p.c.require(p.c.digest(summary_path) == summary_sha, 'summary changed during independent replay')
        for name,expected in summary['artifact_sha256'].items():
            p.c.require(p.c.digest(p.OUTPUT/name) == expected, 'artifact changed during independent replay')
        result = dict(schema='portable_e2e.map_geometry_full_projection_independent_replay.v1',status='PASS',
            summary_sha256=summary_sha,preregistration_sha256=p.c.digest(p.PREREG),
            learned_observations_replayed=learned,oracle_observations_replayed=oracle_count,
            incremental_grid_states_replayed=snapshots,final_grids_replayed=grids,
            all_png_pixels_replayed=pngs,frames_per_route=240,skipped_frames=0,
            chronological_recorded_update_vs_offline_equality=True,evaluation_is_not_live=True,
            independent_max_per_frame_cell_class_fusion=True,exact_final_label_order_ties_verified=True,
            argmax_forward_recomputed=False,recorded_exact_argmax_assertions_verified=True,
            recorded_attempted_and_completed_forwards=960,recorded_learned_map_updates=960,
            independent_replay_inference_count=0,training_steps=0,gpu_used=False,simulator_started=False,
            oracle_depth_and_pose=True,unknown_remains_unknown=True,source_and_input_hashes_unchanged=True,
            ctrack_train_eligible=False,model_promotion=False,hd_map_approved=False,
            elapsed_seconds=time.monotonic()-begin,bytes_before_receipt=p.used_bytes())
        p.write_artifact('independent_replay.json',p.encoded(result))
        return result
    except BaseException as error:
        signal.setitimer(signal.ITIMER_REAL,0)
        failure = dict(schema='portable_e2e.map_geometry_full_projection_verification_failure.v1',
            status='VERIFICATION_FAILED_NO_PARTIAL_PASS',error=repr(error),completed_frames=completed,
            learned_observations=learned,original_artifacts_preserved=True,no_new_forward=True,
            source_and_input_hashes_unchanged='NOT_CLAIMED_ON_FAILURE')
        if p.OUTPUT.is_dir() and not (p.OUTPUT/'verification_failure.json').exists():
            try:
                p.write_artifact('verification_failure.json',p.encoded(failure),failure=True)
            except BaseException:
                print(json.dumps(failure),flush=True)
        else:
            print(json.dumps(failure),flush=True)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL,0); signal.signal(signal.SIGALRM,previous_handler)


if __name__ == '__main__':
    print(json.dumps(verify(),sort_keys=True),flush=True)
