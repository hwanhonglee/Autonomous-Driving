#!/usr/bin/env python3
"""HH_261007 - Verify all stored480 oracle-assisted mapping frames without NN or training.

This is an additional coordinate/fusion diagnostic of the historical control/geometry
models, NOT the later RGB7 native-fusion model, independent DEV, or an HD-map approval.
Actual replay requires a fresh fixed ROOT review and its externally supplied SHA.
HH_261007 - V2 solves the actual stored near-rigid rotation and retains numerical
witnesses before fixed gates; frozen V1's first-frame failure is preserved.
"""

import argparse
import ast
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
HISTORICAL = ROOT / 'docs/assets/validation/2026-10-01/map_geometry_full_projection'
OLD = HISTORICAL / 'run_v1'
ENDPOINT = ROOT / 'storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1'
FIT = ROOT / 'storage/experiments/training/2026-10-01/map_route_geometry_matched_20261001_v1/fit'
OUTPUT = ROOT / 'storage/experiments/diagnostics/2026-10-07/mapping_no_training_v2'
REVIEW = ROOT / 'storage/experiments/plans/2026-10-07/mapping_no_training_v2/ROOT_review.json'
TEST = ROOT / 'learning/tests/test_verify_mapping_no_training_20261007_v2.py'
SCENARIOS, ARMS = ('ctrack_left', 'ctrack_straight'), ('control', 'geometry')
LABELS = ('other', 'road', 'sidewalk', 'lane_marking', 'traffic_light_fixture', 'traffic_sign', 'pole')
TABLE = dict(enumerate(LABELS[1:], 1))
RAW_TABLE = {1: 'road', 2: 'sidewalk', 24: 'lane_marking', 7: 'traffic_light_fixture', 8: 'traffic_sign', 6: 'pole'}
PINS = {'preregistration.json': 'ec54254aa275e9bee03b31cc1ba3325baa6ff2921a422f210a71c473b96c7721',
        'run_v1/summary.json': 'b848700f12f740d1afb424187e4e5ef8fe3eb8af6819629b625da03da9397fd1',
        'run_v1/independent_replay.json': 'bb8149546f098cbdfaab5a35187a237bf51c6be81df4d04007bcf2900fc0a574'}
MATH_SOURCE = ROOT / 'tools/e2e/project_map_photometric_fixed_20261001.py'
VOTE_SOURCE = ROOT / 'tools/e2e/verify_map_geometry_full_projection_20261001.py'
MATH_SHA = 'a229abee7fc4a62bfe37e738ffdf212174b28c3a70f2e4e5d9fade47ee9a0bf3'
VOTE_SHA = 'af1e264adfeddd09ca3a0242198ae9615a66a84aec349565317818e7c4adf4f5'
LIMIT, FLOOR, DEADLINE = 4 * 1024**2, 64 * 1024**2, 600
# HH_261007 - Seal the complete failed V1 witness and both original sources; never retry or overwrite it.
V1_DIRECTORY = ROOT / 'storage/experiments/diagnostics/2026-10-07/mapping_no_training_v1'
V1_PINS = {
    str(ROOT / 'tools/e2e/verify_mapping_no_training_20261007_v1.py'): 'df6118bdfc425f5b1abab7d05258556cdfc51b46b98726f21605f044c559ba5e',
    str(ROOT / 'learning/tests/test_verify_mapping_no_training_20261007_v1.py'): 'bbdd23cd9264d593fcf336e2f22838f45e0944ddb7c2debb957af196ab5337b7',
    str(V1_DIRECTORY / 'verify_mapping_no_training_20261007_v1.py'): 'df6118bdfc425f5b1abab7d05258556cdfc51b46b98726f21605f044c559ba5e',
    str(V1_DIRECTORY / 'test_verify_mapping_no_training_20261007_v1.py'): 'bbdd23cd9264d593fcf336e2f22838f45e0944ddb7c2debb957af196ab5337b7',
    str(V1_DIRECTORY / 'result.json'): '06f4aa862a4a59741994ce77baf70f268045ab6b105d5a421240f1961378d144',
    str(V1_DIRECTORY / 'started.json'): '560ff56b8a7d193658f96b9bb45849d99b3bc197f321c5989e3606fdf84cdc12',
    str(V1_DIRECTORY / 'ROOT_review.json'): '743960fb112dee5538b9b6eb48c0ef7c2f72df63035ed6f10fb2b2a9e184342f',
    str(ROOT / 'storage/experiments/plans/2026-10-07/mapping_no_training_v1/ROOT_review.json'): '743960fb112dee5538b9b6eb48c0ef7c2f72df63035ed6f10fb2b2a9e184342f',
}


def require(value, message):
    if not value:
        raise ValueError(message)


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()


def digest(path):
    path = Path(path)
    require(path == path.resolve(strict=True) and path.is_file(), 'canonical regular file required: ' + str(path))
    before = path.stat()
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024**2), b''):
            result.update(chunk)
    after = path.stat()
    require((before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns) ==
            (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns), 'file changed while hashing')
    return result.hexdigest()


def read_json(path):
    require(Path(path).stat().st_size <= 2 * 1024**2, 'bounded metadata required')
    return json.loads(Path(path).read_text(encoding='utf-8'))


def policy():
    return dict(schema='portable_e2e.mapping_no_training.v2', scenarios=list(SCENARIOS),
                frames_per_scenario=240, frames=480, learned_observations=960, oracle_observations=480,
                stride=3, minimum_confidence=.5, depth_strict_range_m=[.1, 40], resolution_m=.2,
                max_cells=200000, coordinate_xy_max_error_m=1e-8, roundtrip_pixel_max_error=1e-6,
                maximum_output_bytes=LIMIT, free_disk_floor_bytes=FLOOR, deadline_seconds=DEADLINE,
                scope='STORED_PREDICTION_COORDINATE_FUSION_DIAGNOSTIC_ONLY',
                oracle_depth=True, oracle_pose=True, oracle_semantics_reference_only=True,
                latest_native_fusion_v3_model_verified=False, ctrack_train_eligible=False,
                left_original_owner_status='FAIL', native_frames_dropped=0, unknown_is_not_free_space=True,
                inference_count=0, training_steps=0, gpu_used=False, simulator_started=False,
                independent_validation=False, hd_map_approved=False, model_promotion=False,
                new_budget='small read-only diagnostic witness; does not replace historical capture/export budgets')


def input_manifest():
    """HH_261007 - Bind genuine existing records and whole stored artifacts before any numeric import."""
    hashes = {str(HISTORICAL / name): expected for name, expected in PINS.items()}
    hashes.update(V1_PINS)
    for path, expected in hashes.items():
        require(digest(path) == expected, 'historical root metadata changed')
    registration, summary, replay = (read_json(HISTORICAL / name) for name in PINS)
    require(summary['bindings'] == registration['bindings'] and summary['sources'] == registration['sources']
            and summary['policy'] == registration['policy'] and summary['completed_frame_count'] == 480
            and summary['learned_map_updates'] == 960 and summary['oracle_frames'] == 480
            and replay['status'] == 'PASS' and replay['summary_sha256'] == PINS['run_v1/summary.json'],
            'complete original480/replay identity required')
    names = set(summary['artifact_sha256']) | {'summary.json', 'independent_replay.json'}
    require({p.name for p in OLD.iterdir()} == names and len(summary['artifact_sha256']) == 1455,
            'whole historical output census differs; failures/extras cannot be hidden')
    hashes.update({str(OLD / name): sha for name, sha in summary['artifact_sha256'].items()})
    hashes.update(registration['sources'])
    for name, sha in registration['bindings']['fit_artifacts'].items():
        require(Path(name).as_posix() == name and '..' not in Path(name).parts, 'unsafe fit-relative name')
        hashes[str(FIT / name)] = sha  # Byte seal only: never deserialize weights/checkpoints/training payloads.
    for scenario in SCENARIOS:
        binding = registration['bindings']['scenarios'][scenario]
        source = Path(binding['source'])
        require(source.is_relative_to(ROOT / 'storage/experiments/validation/2026-10-01'), 'wrong raw source scope')
        hashes[str(source / 'records.json')] = binding['records_sha256']
        hashes[str(source / 'summary.json')] = binding['source_summary_sha256']
        hashes[str(ENDPOINT / scenario / 'summary.json')] = binding['endpoint_summary_sha256']
        hashes[str(ENDPOINT / scenario / 'prediction_audit.json')] = binding['endpoint_audit_sha256']
        hashes.update({str(ENDPOINT / scenario / name): sha for name, sha in binding['predictions'].items()})
        require(len(binding['frames']) == 240, 'all240 raw frames required')
        for index, frame in enumerate(binding['frames']):
            require(frame['index'] == index and frame['stem'] == f'frames/{index:04d}', 'fixed raw frame order differs')
            hashes.update({str(source / (frame['stem'] + extension)): sha for extension, sha in frame['artifact_sha256'].items()})
    for path, expected in hashes.items():
        require(digest(path) == expected, 'historical source/input changed: ' + path)
    return hashes, registration, summary


def sources():
    paths = [Path(__file__).resolve(), TEST, ROOT / 'tools/workspace_python.py', ROOT / 'tools/canonical_workspace.py',
             ROOT / 'learning/portable_e2e/__init__.py', ROOT / 'learning/portable_e2e/contract.py',
             ROOT / 'learning/portable_e2e/mapping.py', ROOT / 'learning/portable_e2e/mapping_projection.py',
             ROOT / 'tools/e2e/verify_mapping_no_training_20261007_v1.py',
             ROOT / 'learning/tests/test_verify_mapping_no_training_20261007_v1.py']
    return {str(path): digest(path) for path in paths}


def review_binding():
    hashes, _, _ = input_manifest()
    return dict(schema='portable_e2e.mapping_no_training_ROOT_review.v2', policy=policy(), sources=sources(),
                input_manifest_sha256=hashlib.sha256(encoded(hashes)).hexdigest(), input_count=len(hashes),
                output=str(OUTPUT), authorized_cpu_stored_replay=True)


def extracted_functions(path, expected_sha, names, namespace):
    """HH_261007 - Compile only exact pinned pure functions, never the old NN evaluator/import chain."""
    require(digest(path) == expected_sha, 'pure-function source changed')
    tree = ast.parse(path.read_text(encoding='utf-8'))
    selected = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    require({n.name for n in selected} == set(names) and len(selected) == len(names), 'pure function selection differs')
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


def numerical():
    import numpy as np
    from portable_e2e.mapping import SemanticEvidenceMap
    from portable_e2e.mapping_projection import project_frame, project_predicted_frame
    namespace = dict(np=np, json=json, hashlib=hashlib, math=math, LABELS=LABELS,
                     TABLE=TABLE, STRIDE=3, c=SimpleNamespace(require=require), SemanticEvidenceMap=SemanticEvidenceMap)
    names = ('encoded', 'sha', 'metric_depth', 'restore_confidence', 'pack_grid', 'new_grid', 'point_receipt', 'comparison')
    extracted_functions(MATH_SOURCE, MATH_SHA, names, namespace)
    namespace['p'] = SimpleNamespace(c=SimpleNamespace(require=require), TABLE=TABLE,
                                   encoded=namespace['encoded'], sha=namespace['sha'])
    extracted_functions(VOTE_SOURCE, VOTE_SHA, ('independent_update', 'independent_overview'), namespace)
    return np, SimpleNamespace(**namespace), project_frame, project_predicted_frame


def coordinate_metrics(np, depth, classes, confidence, pose, observation, table, *, witness=None):
    """HH_261007 - Independently invert genuine oracle-assisted XYZ, not physical localization accuracy."""
    z, tags, certainty = depth[::3, ::3], classes[::3, ::3], confidence[::3, ::3]
    v, u = np.mgrid[0:depth.shape[0]:3, 0:depth.shape[1]:3]
    valid = np.isfinite(z) & (z > .1) & (z < 40) & (certainty >= .5) & np.isin(tags, list(table))
    z = z[valid].astype(np.float64)
    metrics = dict(points=len(z), xy_max_error_m=None, roundtrip_pixel_max_error=None, cell_center_max_distance_m=None)
    if witness is not None:
        witness.update(metrics)
    if len(z) == 0:
        require(len(observation.points) == 0, 'empty projected support differs')
        return metrics
    h, w = depth.shape
    focal = w / (2 * math.tan(math.radians(90) / 2))
    camera = np.column_stack((z, (u[valid] - w / 2) * z / focal, -(v[valid] - h / 2) * z / focal))
    matrix = np.asarray(pose, dtype=np.float64)
    world = camera @ matrix[:3, :3].T + matrix[:3, 3]
    observed = np.asarray([(p[0], p[1]) for p in observation.points], dtype=np.float64)
    require(observed.shape == world[:, :2].shape, 'point selection/order differs')
    xy_error = float(np.max(np.abs(observed - world[:, :2])))
    # HH_261007 - Stored float32-origin values are near rigid, not exactly orthogonal: transpose is not their inverse.
    recovered = np.linalg.solve(matrix[:3, :3], (world - matrix[:3, 3]).T).T
    uv = np.column_stack((w / 2 + focal * recovered[:, 1] / recovered[:, 0],
                          h / 2 - focal * recovered[:, 2] / recovered[:, 0]))
    uv_error = float(np.max(np.abs(uv - np.column_stack((u[valid], v[valid])))))
    center = (np.floor(observed / .2) + .5) * .2
    quantization = float(np.max(np.linalg.norm(observed - center, axis=1)))
    metrics.update(xy_max_error_m=xy_error, roundtrip_pixel_max_error=uv_error,
                   cell_center_max_distance_m=quantization)
    if witness is not None:
        witness.update(metrics)  # HH_261007 - Keep measured errors even when an unchanged gate raises below.
    require(math.isfinite(xy_error) and xy_error <= 1e-8 and math.isfinite(uv_error) and uv_error <= 1e-6,
            'fixed coordinate roundtrip screen failed')
    require(quantization <= math.sqrt(2) * .1 + 1e-8, 'floor-cell raster quantization differs')
    return metrics


def coverage(observation, prior_cells):
    cells, labels = set(), {}
    for x, y, label, score in observation.points:
        if score > 0:
            cell = (math.floor(x / .2), math.floor(y / .2))
            cells.add(cell)
            labels.setdefault(cell, set()).add(label)
    return dict(observed_cells=len(cells), newly_observed_cells=len(cells - prior_cells),
                reobserved_cells=len(cells & prior_cells), multi_label_cells_in_this_frame=sum(len(v) > 1 for v in labels.values()))


def write_new(name, value):
    raw = encoded(value)
    require(Path(name).name == name, 'simple witness basename required')
    used = sum(p.stat().st_size for p in OUTPUT.iterdir())
    require(used + len(raw) <= LIMIT and shutil.disk_usage(OUTPUT.parent).free - len(raw) >= FLOOR, 'new witness budget exceeded')
    with (OUTPUT / name).open('xb') as stream:
        stream.write(raw)


def execute(review_sha):
    require(isinstance(review_sha, str) and len(review_sha) == 64 and all(c in '0123456789abcdef' for c in review_sha),
            'externally supplied ROOT review SHA required')
    require(digest(REVIEW) == review_sha and read_json(REVIEW) == review_binding(), 'fresh fixed ROOT CPU review differs')
    require(os.environ.get('CUDA_VISIBLE_DEVICES') == '' and os.environ.get('OPENBLAS_NUM_THREADS') == '1', 'CPU-only/BLAS1 environment required')
    hashes, registration, summary = input_manifest()
    source_seals = sources()
    # HH_261007 - A new dated parent may be absent; reserve on its existing ancestor before create-only mkdir.
    ancestor = OUTPUT.parent
    while not ancestor.exists():
        ancestor = ancestor.parent
    require(OUTPUT == OUTPUT.resolve(strict=False) and not OUTPUT.exists()
            and shutil.disk_usage(ancestor).free >= FLOOR + LIMIT, 'fresh canonical output/reserve required')
    OUTPUT.mkdir(parents=True, exist_ok=False)
    begin = time.monotonic()
    rows = [dict(scenario=s, index=i, status='UNEXECUTED') for s in SCENARIOS for i in range(240)]
    result = dict(schema=policy()['schema'], status='FAILED_WITH_FULL_DENOMINATOR', policy=policy(),
                  rows=rows, scenarios={}, source_and_inputs_unchanged=False, root_review_sha256=review_sha)
    handler = signal.getsignal(signal.SIGALRM)
    def timeout(signum, frame):
        raise TimeoutError('fixed600s diagnostic deadline exceeded')
    signal.signal(signal.SIGALRM, timeout)
    signal.setitimer(signal.ITIMER_REAL, DEADLINE)
    try:
        write_new('started.json', dict(policy=policy(), sources=source_seals, input_count=len(hashes),
                                      input_manifest_sha256=hashlib.sha256(encoded(hashes)).hexdigest(),
                                      created_at_utc=datetime.now(timezone.utc).isoformat()))
        # HH_261007 - Archive small exact reviewed bytes only; never duplicate raw images or model payloads.
        for path in (REVIEW, Path(__file__).resolve(), TEST):
            raw = path.read_bytes()
            require(sum(p.stat().st_size for p in OUTPUT.iterdir()) + len(raw) <= LIMIT,
                    'bounded immutable witness copy required')
            with (OUTPUT / path.name).open('xb') as stream:
                stream.write(raw)
        np, n, project_frame, project_learned = numerical()
        for scenario in SCENARIOS:
            binding = registration['bindings']['scenarios'][scenario]
            records = read_json(Path(binding['source']) / 'records.json')
            require(len(records) == 240, 'whole native recording required')
            predictions = {}
            for arm in ARMS:
                with np.load(ENDPOINT / scenario / (arm + '_true_all240_predictions.npz'), allow_pickle=False) as data:
                    require(data.files == ['prediction'], 'prediction archive fields differ')
                    predictions[arm] = data['prediction'].copy()
                require(predictions[arm].shape == (240, 360, 640) and predictions[arm].dtype == np.uint8
                        and int(predictions[arm].max()) < 7, 'all240 native learned masks required')
            maps = {key: n.new_grid('carla_ue:' + binding['town'], 'oracle_sensor' if key == 'oracle' else 'learned_prediction')
                    for key in ('oracle', *ARMS)}
            independent = {key: n.new_grid(grid.frame_id, grid.source) for key, grid in maps.items()}
            for i, record in enumerate(records):
                row = rows[SCENARIOS.index(scenario) * 240 + i]
                row.update(status='FAIL', sensor_frame=record['frame'], timestamp_s=record['timestamp_s'])
                fixed = binding['frames'][i]
                require(record['stem'] == fixed['stem'] and record['frame'] == fixed['sensor_frame']
                        and record['timestamp_s'] == fixed['timestamp_s']
                        and (i == 0 or record['frame'] == records[i - 1]['frame'] + 1), 'exact native chronology differs')
                oldrow = read_json(OLD / f'{scenario}_{i:03d}_frame.json')
                with np.load(Path(binding['source']) / (record['stem'] + '.npz'), allow_pickle=False) as raw:
                    depth, semantic, pose = (raw[key].copy() for key in ('depth', 'semantic', 'world_from_camera'))
                options = dict(frame_id=maps['oracle'].frame_id, timestamp_s=record['timestamp_s'], fov_deg=90, stride=3, max_depth_m=40)
                observations = {'oracle': project_frame(depth, semantic, pose, label_table=RAW_TABLE, **options)}
                d = depth.astype(np.float64)
                axis64 = (d[..., 2] + d[..., 1] * 256 + d[..., 0] * 65536) * (1000 / 16777215)
                row['maps'] = {'oracle': {}}
                coordinate_metrics(np, axis64, semantic[..., 2], np.ones((360, 640)), pose, observations['oracle'],
                                   RAW_TABLE, witness=row['maps']['oracle'])
                for arm in ARMS:
                    receipt = oldrow['projections'][arm]
                    name = f'{scenario}_{i:03d}_{arm}_confidence.npz'
                    require(receipt['confidence_file'] == name and receipt['exact_saved_argmax'] is True, 'fixed saved confidence/argmax provenance differs')
                    with np.load(OLD / name, allow_pickle=False) as data:
                        require(data.files == ['confidence_stride3'], 'confidence fields differ')
                        sampled = data['confidence_stride3'].copy()
                    require(n.sha(sampled.tobytes()) == receipt['confidence_numeric_sha256'], 'exact confidence bytes differ')
                    confidence = n.restore_confidence(sampled)
                    axis32 = n.metric_depth(depth)
                    observations[arm] = project_learned(axis32, predictions[arm][i], confidence, pose, label_table=TABLE, min_confidence=.5, **options)
                    row['maps'][arm] = {}
                    coordinate_metrics(np, axis32, predictions[arm][i], confidence, pose, observations[arm],
                                       TABLE, witness=row['maps'][arm])
                for key, observation in observations.items():
                    require(all(oldrow['projections'][key][field] == value for field, value in n.point_receipt(observation).items()), 'exact original projected point receipt differs')
                    row['maps'][key].update(coverage(observation, set(maps[key].cells)))
                    maps[key].update(observation)
                    n.independent_update(independent[key], observation)
                    require(maps[key].snapshot() == independent[key].snapshot(), 'independent frame-vote fusion differs')
                    require(n.sha(n.encoded(maps[key].snapshot())) == oldrow['accumulated_snapshot_sha256'][key]
                            and len(maps[key].cells) == oldrow['accumulated_cells'][key], 'original full incremental state differs')
                row['status'] = 'PASS'
            oldscenario = summary['scenarios'][scenario]
            for key, grid in maps.items():
                with np.load(OLD / oldscenario['grids'][key]['file'], allow_pickle=False) as data:
                    expected = n.pack_grid(grid)
                    require(set(data.files) == set(expected) and len(data.files) == len(expected), 'final grid fields differ')
                    for field, array in expected.items():
                        actual = data[field]
                        require(actual.dtype == array.dtype and actual.shape == array.shape and actual.tobytes() == array.tobytes(), 'exact final grid differs: ' + field)
            extent = n.independent_overview(records, maps)
            require(extent == oldscenario['overview'], 'shared raster extent/all240 poses differs')
            comparisons = {arm: n.comparison(maps['oracle'], maps[arm]) for arm in ARMS}
            require(comparisons == oldscenario['comparisons'], 'original final semantic comparison differs')
            result['scenarios'][scenario] = dict(raster_extent=extent, comparisons=comparisons,
                cells_with_multiple_label_evidence={key: sum(len(v) > 1 for v in grid.cells.values()) for key, grid in maps.items()},
                original_owner_status=binding['original_owner_status'], evaluation_role=binding['evaluation_role'],
                source_qa_status=binding['source_qa_status'], original_live_replay_exact_match=binding['original_live_replay_exact_match'])
        require(sum(row['status'] == 'PASS' for row in rows) == 480, 'full480 denominator required')
        require(input_manifest()[0] == hashes and sources() == source_seals and digest(REVIEW) == review_sha, 'closing source/input/review differs')
        result.update(status='COMPLETE_STORED_COORDINATE_FUSION_DIAGNOSTIC', source_and_inputs_unchanged=True)
    except BaseException as error:
        result['error'] = repr(error)
        try:
            result['source_and_inputs_unchanged'] = input_manifest()[0] == hashes and sources() == source_seals
        except BaseException as closing:
            result['closing_error'] = repr(closing)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, handler)
        result.update(elapsed_seconds=time.monotonic() - begin, completed_frames=sum(r['status'] == 'PASS' for r in rows),
                      failed_frames=sum(r['status'] == 'FAIL' for r in rows), unexecuted_frames=sum(r['status'] == 'UNEXECUTED' for r in rows),
                      physical_localization_error_verified=False, png_pixels_replayed=False, latest_model_verified=False,
                      train_exposure='Historical geometry/control TRAIN; CTrack previously exposed EVAL only, never TRAIN. Three-frame Town01 bridge includes TRAIN frames and is not reused.')
        result['coordinate_error_summary'] = {}
        for key in ('oracle', *ARMS):
            available = [row['maps'][key] for row in rows if key in row.get('maps', {})]
            result['coordinate_error_summary'][key] = dict(available_frames=len(available),
                projected_points=sum(x['points'] for x in available),
                **{field: max((x[field] for x in available if x[field] is not None), default=None)
                   for field in ('xy_max_error_m', 'roundtrip_pixel_max_error', 'cell_center_max_distance_m')})
        write_new('result.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root-review-sha256')
    parser.add_argument('--inspect', action='store_true', help='read-only metadata/hashes for ROOT review; no numeric imports')
    args = parser.parse_args()
    value = review_binding() if args.inspect else execute(args.root_review_sha256)
    print(json.dumps({k: v for k, v in value.items() if k not in ('rows', 'scenarios', 'sources')}, sort_keys=True))
    raise SystemExit(0 if args.inspect or value['status'] == 'COMPLETE_STORED_COORDINATE_FUSION_DIAGNOSTIC' else 2)
