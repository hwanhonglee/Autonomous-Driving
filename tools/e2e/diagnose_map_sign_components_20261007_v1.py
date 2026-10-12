#!/usr/bin/env python3
"""HH_261007 - Describe every stored native RGB7 sign component without inference.

Connected pixels are not guaranteed semantic objects. Size/border correlations
cannot establish crop, labeling, or model-training causes. Actual replay requires
the fixed external ROOT authority; importing this module imports stdlib only.
"""
import argparse
import ast
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import re
import resource
import signal
import struct
import time
from types import SimpleNamespace
import zipfile

ROOT = Path(__file__).resolve().parents[2]
NAME = 'map_sign_components_v1'
SCHEMA = 'portable_e2e.map_sign_components.20261007.v1'
OUTPUT = ROOT / 'storage/experiments/diagnostics/2026-10-07' / NAME
AUTHORITY = ROOT / 'storage/experiments/plans/2026-10-07' / NAME / 'ROOT_authority.json'
LOCK = ROOT / 'storage/assets/locks/autoware_e2e_runtime.lock'
TEST = ROOT / 'learning/tests/test_diagnose_map_sign_components_20261007_v1.py'
PLAN = ROOT / 'docs/assets/validation/2026-10-07/mapping_review/MAP_SIGN_COMPONENTS_PLAN_v1.md'
RUN = ROOT / 'storage/experiments/training/2026-10-02/map_native_fusion_pair_v3'
DATA = ROOT / 'storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1'
RESULTS = ROOT / 'docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/25_map_native_fusion_v3_results/results.json'
MANIFEST = RUN / 'inputs/manifest.json'
LABELS = ('other', 'road', 'sidewalk', 'lane_marking', 'traffic_light_fixture', 'traffic_sign', 'pole')
COHORTS = (('dev_town01', 118), ('holdout_town02_p02', 232))
ARMS = ('parent', 'control', 'candidate')
BINS = ((1, 1), (2, 4), (5, 16), (17, 64), (65, 256), (257, 360 * 640))
LIMIT, FLOOR, RESERVE, RSS, DEADLINE = 2 * 1024**2, 2 * 1024**3, 64 * 1024**2, 512 * 1024**2, 900
SCREEN_SOURCES = tuple(ROOT / 'tools/e2e' / name for name in (
    'publish_map_sign_sampler_v2_comparison_20261002.py', 'publish_map_sign_margin_loss_results_20261002.py'))
METRIC_SOURCE = ROOT / 'tools/e2e/publish_map_paired_balanced_results_20261002.py'
INPUTS = {str(RESULTS): 'a367c1f594cc5518ac6d6a0df8bff8aa51cd946395108967870fc948fe7fcd7b',
          str(MANIFEST): '614840471e938a0e9ecd169368d6240e96c01fdf76c74b56f82863c9d49d2b1b'}
for cohort, source_sha, prediction_shas in (
    ('dev_town01', '417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3', (
        'c335375371ef30b07fb5042fbfbed902bb0758c0d15b307ccf683fdc3d79472a',
        'f05096b04655762543f82e5ed7390de917beaa88f4c100ba83bc39aed3c50988',
        '0f9f6a3f2db8aea63b5778e2c2e57444dd141c75149ebe5c626cd1310ebf94f1')),
    ('holdout_town02_p02', '858f92ccd874c154576d3cf6a85132508801a22671baf6d0c07abe3936eb16cb', (
        '8d8f47c4ec709acd19f0f49d0ecd214b068725125d34f5e8f8309597f1ad3b4b',
        'e31b9f4ec787870a855b6b2ca285bfbaa6c04ac431f0691331ab620bf17929fb',
        'ff55003d3f71aa8c83e62cfaff1673a028549b49170d12d9e68cd415f3973ee5'))):
    INPUTS[str(DATA / (cohort + '.npz'))] = source_sha
    for arm, sha in zip(ARMS, prediction_shas):
        INPUTS[str(RUN / ('parent_diagnostic' if arm == 'parent' else arm) / (cohort + '_predictions.npz'))] = sha


def require(value, message):
    if not value:
        raise ValueError(message)


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()


def canonical(path):
    path = Path(path)
    require(path == path.resolve(strict=True) and path.is_file(), 'canonical regular file required')
    return path


def digest(path):
    path = canonical(path); before = path.stat(); h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024**2), b''):
            h.update(chunk)
    after = path.stat()
    require((before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns) ==
            (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns), 'hash read changed')
    return h.hexdigest()


def read_json(path):
    require(canonical(path).stat().st_size <= 512 * 1024, 'metadata cap exceeded')
    def pairs(items):
        value = {}
        for key, item in items:
            require(key not in value, 'duplicate JSON field'); value[key] = item
        return value
    return json.loads(path.read_bytes(), object_pairs_hook=pairs,
                      parse_constant=lambda value: require(False, 'nonfinite JSON denied'))


def sources():
    return {str(p): digest(p) for p in (Path(__file__).resolve(), TEST, PLAN, *SCREEN_SOURCES,
        METRIC_SOURCE, ROOT / 'tools/workspace_python.py', ROOT / 'tools/canonical_workspace.py')}


def contract():
    return dict(schema=SCHEMA, source_sha256=sources(), input_sha256=INPUTS,
        labels=list(LABELS), sign_index=5, cohort_counts=dict(COHORTS), arms=list(ARMS),
        complete_arm_frame_denominator=1050, native_hw=[360, 640], connectivity=8,
        descriptive_area_bins=[list(v) for v in BINS], border_definition='touches image outermost row or column',
        maximum_output_bytes=LIMIT, free_floor_bytes=FLOOR, output_reserve_bytes=RESERVE,
        sampled_peak_RSS_limit_bytes=RSS, deadline_seconds=DEADLINE, runtime_lock=str(LOCK),
        runtime_packages={'numpy': '1.21.5', 'scipy': '1.8.0'}, one_CPU=True,
        decoded_zip_members=['target.npy', 'prediction.npy'], RGB_decoded=False,
        inference_count=0, training_steps=0, GPU_used=False, original_27_criteria_unchanged=True,
        independent_new_DEV_sites=0, semantic_object_identity_verified=False, crop_causation_verified=False,
        full_HD_AMG_verified=False, formal_main_approved=0, formal_sub_approved=0)


def verify_pins(pins, checkpoint=lambda: None):
    for name, sha in pins.items():
        checkpoint()
        require(re.fullmatch('[0-9a-f]{64}', sha) and digest(Path(name)) == sha, 'source/input seal changed: ' + name)


def admission(sha):
    require(type(sha) is str and re.fullmatch('[0-9a-f]{64}', sha), 'external ROOT raw SHA required')
    require(digest(AUTHORITY) == sha, 'fixed ROOT authority raw bytes differ')
    authority = read_json(AUTHORITY)
    require(authority.get('schema') == SCHEMA + '.ROOT_authority' and authority.get('contract') == contract()
        and authority.get('stored_mask_CPU_diagnostic_authorized') is True
        and authority.get('output') == str(OUTPUT), 'fixed ROOT scope denied')
    cpu = authority.get('CPU_affinity')
    require(type(cpu) is list and len(cpu) == 1 and type(cpu[0]) is int
        and cpu[0] in os.sched_getaffinity(0), 'one currently available declared CPU required')
    for key, wanted in {'CUDA_VISIBLE_DEVICES': '', 'PYTHONNOUSERSITE': '1', 'PYTHONDONTWRITEBYTECODE': '1',
                         'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1'}.items():
        require(os.environ.get(key) == wanted, 'clean single-CPU environment denied: ' + key)
    for package, version in authority['contract']['runtime_packages'].items():
        require(importlib.metadata.version(package) == version, 'existing system package version differs')
    return authority, cpu


def guard(start):
    require(time.monotonic() - start < DEADLINE, 'whole diagnostic deadline exceeded')
    peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    require(peak <= RSS, 'sampled peak RSS cap exceeded')
    fs = os.statvfs(OUTPUT.parent)
    require(fs.f_bavail * fs.f_frsize >= FLOOR + RESERVE, 'shared floor/output reserve unavailable')
    return peak


def write_bytes(path, payload):
    require(OUTPUT == OUTPUT.resolve(strict=True), 'output directory changed')
    entries = list(OUTPUT.iterdir())
    require(all(p == p.resolve(strict=True) and p.is_file() for p in entries), 'unexpected output type/alias')
    used = sum(p.stat().st_size for p in entries)
    require(used + len(payload) <= LIMIT, 'complete output cap exceeded')
    with path.open('xb') as stream:
        stream.write(payload)


def write_new(path, value):
    write_bytes(path, encoded(value))


def npy_header(stream, shape):
    require(stream.read(6) == b'\x93NUMPY', 'NPY magic differs')
    version = stream.read(2)
    require(version in (b'\x01\x00', b'\x02\x00', b'\x03\x00'), 'NPY version denied')
    width = 2 if version == b'\x01\x00' else 4
    raw = stream.read(width); require(len(raw) == width, 'truncated NPY header length')
    length = int.from_bytes(raw, 'little'); require(0 < length <= 16384, 'NPY header cap exceeded')
    raw = stream.read(length); require(len(raw) == length, 'truncated NPY header')
    header = ast.literal_eval(raw.decode('utf-8' if version == b'\x03\x00' else 'latin1'))
    require(type(header) is dict and set(header) == {'descr', 'fortran_order', 'shape'}
        and header['descr'] == '|u1' and header['fortran_order'] is False
        and type(header['shape']) is tuple and all(type(v) is int for v in header['shape'])
        and header['shape'] == shape, 'exact native uint8 C-order NPY ABI required')


def load_mask(path, member, count, np):
    shape = (count, 360, 640); total = count * 360 * 640
    with zipfile.ZipFile(canonical(path)) as archive:
        expected = ['rgb.npy', 'target.npy'] if member == 'target.npy' else ['prediction.npy']
        require(sorted(archive.namelist()) == expected, 'exact stored ZIP member census required')
        entry = archive.getinfo(member)
        require(not entry.flag_bits & 1 and total < entry.file_size <= total + 16400, 'bounded unencrypted entry required')
        with archive.open(member) as stream:
            npy_header(stream, shape)
            pixels = stream.read(total + 1)
            require(len(pixels) == total, 'truncated/extra NPY pixels')
    value = np.frombuffer(pixels, dtype=np.uint8).reshape(shape)
    require(int(value.max()) < 7, 'class outside exact seven-label table')
    return value, hashlib.sha256(pixels).hexdigest()


def metrics(matrix):
    # HH_261007 - Independently retain the original integer confusion and null-IoU formulas.
    require(len(matrix) == 7 and all(len(r) == 7 and all(type(v) is int and v >= 0 for v in r) for r in matrix), '49 confusion integers required')
    actual = [sum(r) for r in matrix]; predicted = [sum(r[i] for r in matrix) for i in range(7)]
    diagonal = [matrix[i][i] for i in range(7)]; union = [a + p - d for a, p, d in zip(actual, predicted, diagonal)]
    require(sum(actual) > 0, 'empty metric denominator')
    return dict(confusion=matrix, target_pixels=dict(zip(LABELS, actual)), predicted_pixels=dict(zip(LABELS, predicted)),
        true_positive_pixels=dict(zip(LABELS, diagonal)), false_negative_pixels=dict(zip(LABELS, [a-d for a,d in zip(actual,diagonal)])),
        false_positive_pixels=dict(zip(LABELS, [p-d for p,d in zip(predicted,diagonal)])),
        iou={k: diagonal[i]/union[i] if actual[i] else None for i,k in enumerate(LABELS)},
        precision={k: diagonal[i]/predicted[i] if predicted[i] else None for i,k in enumerate(LABELS)},
        recall={k: diagonal[i]/actual[i] if actual[i] else None for i,k in enumerate(LABELS)},
        pixel_accuracy=sum(diagonal)/sum(actual), absent_target_class_iou='null; false positives retained',
        other_to_sign_false_positive_pixels=matrix[0][5])


def original_screens(evaluation):
    previous = None
    for path in SCREEN_SOURCES:
        functions = [n for n in ast.parse(path.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == 'diagnostic_screen']
        require(len(functions) == 1, 'unique genuine diagnostic_screen required')
        namespace = {'require': require, 'previous': SimpleNamespace(diagnostic_screen=previous)}
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), 'exec'), namespace)
        previous = namespace['diagnostic_screen']
    return previous(evaluation)


def bin_index(area):
    require(type(area) is int and 1 <= area <= 360 * 640, 'component area out of bounds')
    return next(i for i, (lo, hi) in enumerate(BINS) if lo <= area <= hi)


def summarize_components(rows):
    bins = [[0] * 9 for _ in BINS]; digest_rows = hashlib.sha256()
    for area, tp, error, border in rows:
        require(type(area) is type(tp) is type(error) is int and type(border) is bool
            and area > 0 and tp >= 0 and error >= 0 and tp + error == area, 'complete component partition differs')
        row = bins[bin_index(area)]
        for i, value in enumerate((1, area, tp, error, int(tp == 0), int(border), area if border else 0,
                                   tp if border else 0, error if border else 0)):
            row[i] += value
        digest_rows.update(struct.pack('<qqqq', area, tp, error, int(border)))
    return bins, digest_rows.hexdigest()


def components(mask, correct, np, ndimage):
    labeled, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=np.uint8))
    areas = np.bincount(labeled.ravel(), minlength=count + 1)
    hit = np.bincount(labeled[correct], minlength=count + 1)
    border = set(int(v) for edge in (labeled[0], labeled[-1], labeled[:, 0], labeled[:, -1]) for v in edge if v)
    return summarize_components((int(areas[i]), int(hit[i]), int(areas[i]-hit[i]), i in border) for i in range(1, count + 1))


def ledger():
    return [dict(cohort=c, arm=a, ordinal=i, status='UNEXECUTED') for c,n in COHORTS for a in ARMS for i in range(n)]


def execute(authority_sha):
    authority, cpu = admission(authority_sha)
    with canonical(LOCK).open('r') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        require(not OUTPUT.exists(), 'fresh output only; no retry/resume')
        OUTPUT.mkdir(parents=True, exist_ok=False)
        start = time.monotonic(); rows = ledger(); cells = {}; current = None; error = None; pins_ok = False
        result = dict(schema=SCHEMA, contract=authority['contract'], authority_sha256=authority_sha,
            status='FAILED_WITH_FULL_DENOMINATOR', frames=rows, cells=cells, original_criteria_verified=False,
            source_and_inputs_unchanged=False, interpretation='DESCRIPTIVE_KNOWN_EXPOSED_STORED_MASK_DIAGNOSTIC_ONLY',
            RGB_decoded=False, model_loaded=False, inference_count=0, training_steps=0, GPU_used=False,
            GT_depth_or_pose_read=False, independent_validation=False, model_performance_improved=False,
            crop_causation_verified=False, semantic_component_object_identity_verified=False,
            RSS_limit_is_sampled_peak_guard_not_OS_memory_sandbox=True, package_binary_closure_verified=False)
        old_handler = signal.signal(signal.SIGALRM, lambda *args: require(False, 'whole diagnostic deadline exceeded'))
        signal.setitimer(signal.ITIMER_REAL, DEADLINE)
        try:
            guard(start); verify_pins(authority['contract']['source_sha256'], lambda: guard(start)); verify_pins(INPUTS, lambda: guard(start))
            for i, (name, sha) in enumerate(authority['contract']['source_sha256'].items()):
                payload = Path(name).read_bytes(); require(hashlib.sha256(payload).hexdigest() == sha, 'archive source changed')
                require(sum(p.stat().st_size for p in OUTPUT.iterdir()) + len(payload) < LIMIT, 'source archive cap')
                write_bytes(OUTPUT / ('source_%02d_' % i + Path(name).name), payload)
            authority_raw = AUTHORITY.read_bytes()
            require(hashlib.sha256(authority_raw).hexdigest() == authority_sha, 'raw authority archive differs')
            write_bytes(OUTPUT / 'ROOT_authority.json', authority_raw)
            write_new(OUTPUT / 'started.json', dict(created_at_UTC=datetime.now(timezone.utc).isoformat(),
                monotonic_start=start, PID=os.getpid(), original_affinity=sorted(os.sched_getaffinity(0)),
                declared_affinity=cpu, input_seals=dict(INPUTS)))
            manifest, saved = read_json(MANIFEST), read_json(RESULTS)
            require(manifest['labels'] == list(LABELS) and manifest['raw_model_inputs'] == ['RGB']
                and manifest['depth_or_pose_model_input'] is False and manifest['dev_or_test_used_for_fit'] is False
                and manifest['independent_new_dev_sites'] == 0 and saved['full_cohort_counts'] == dict(COHORTS), 'original native-label/scope manifest differs')
            os.sched_setaffinity(0, cpu); require(sorted(os.sched_getaffinity(0)) == cpu, 'one CPU affinity not applied')
            import numpy as np
            from scipy import ndimage
            result['runtime_versions'] = dict(numpy=np.__version__, scipy=importlib.metadata.version('scipy'))
            peak = guard(start); position = 0; evaluation = {}
            for cohort, count in COHORTS:
                current = rows[position]; target, target_sha = load_mask(DATA / (cohort + '.npz'), 'target.npy', count, np)
                evaluation[cohort] = {}; peak = max(peak, guard(start))
                for arm in ARMS:
                    current = rows[position]
                    path = RUN / ('parent_diagnostic' if arm == 'parent' else arm) / (cohort + '_predictions.npz')
                    prediction, prediction_sha = load_mask(path, 'prediction.npy', count, np)
                    matrix = np.zeros((7, 7), dtype=np.int64); totals = [[[0]*9 for _ in BINS] for _ in range(2)]
                    peak = max(peak, guard(start))
                    for i in range(count):
                        current = rows[position]; guard(start); before = time.monotonic()
                        truth, pred = target[i], prediction[i]
                        confusion = np.bincount((truth * 7 + pred).ravel(), minlength=49).reshape(7,7)
                        matrix += confusion; correct = (truth == 5) & (pred == 5)
                        true_bins, true_sha = components(truth == 5, correct, np, ndimage)
                        pred_bins, pred_sha = components(pred == 5, correct, np, ndimage)
                        for aggregate, bins in zip(totals, (true_bins, pred_bins)):
                            for destination, values in zip(aggregate, bins):
                                for k, value in enumerate(values): destination[k] += value
                        require(sum(r[1] for r in true_bins) == int((truth == 5).sum())
                            and sum(r[2] for r in true_bins) == sum(r[2] for r in pred_bins) == int(correct.sum())
                            and sum(r[3] for r in pred_bins) == int(confusion[:,5].sum()-confusion[5,5]), 'component/49-cell partition differs')
                        current.update(status='PASS', true_components=sum(r[0] for r in true_bins), predicted_components=sum(r[0] for r in pred_bins),
                            true_component_rows_sha256=true_sha, predicted_component_rows_sha256=pred_sha,
                            confusion=confusion.tolist(),
                            sign_TP=int(confusion[5,5]), sign_FN=int(confusion[5].sum()-confusion[5,5]),
                            sign_FP=int(confusion[:,5].sum()-confusion[5,5]), other_to_sign_FP=int(confusion[0,5]), seconds=time.monotonic()-before)
                        position += 1; peak = max(peak, guard(start))
                    calculated = metrics(matrix.tolist()); reference = saved['evaluation'][cohort][arm]
                    require(reference['count'] == reference['evaluated_frames'] == count and reference['full_FOV'] is True
                        and reference['source_sha256'] == INPUTS[str(DATA / (cohort+'.npz'))]
                        and reference['prediction_file_sha256'] == INPUTS[str(path)], 'original full interval/reference identity differs')
                    for key, value in calculated.items(): require(reference[key] == value, 'original full metric differs: ' + key)
                    evaluation[cohort][arm] = calculated
                    cells[cohort+'/'+arm] = dict(count=count, metrics=calculated, target_pixel_bytes_sha256=target_sha,
                        prediction_pixel_bytes_sha256=prediction_sha, true_component_bins=totals[0], predicted_component_bins=totals[1])
                    del prediction
                del target
            screen = original_screens(evaluation)
            require(screen == saved['exploratory_screen'], 'all original27 screens differ')
            result.update(original_criteria_verified=True, original_exploratory_screen=screen, peak_RSS_bytes=peak,
                component_bin_columns=['components','pixels','TP_pixels','FN_or_FP_pixels','zero_TP_components',
                    'border_components','border_component_pixels','border_TP_pixels','border_FN_or_FP_pixels'],
                component_digest_rows='scan-order labels; little-endian int64 [area,TP,FN_or_FP,border]; includes every component')
        except Exception as exc:
            error = type(exc).__name__ + ': ' + str(exc)
            if current is not None:
                current['status'] = 'FAIL'; current['error'] = error
        finally:
            try:
                verify_pins(authority['contract']['source_sha256'], lambda: guard(start)); verify_pins(INPUTS, lambda: guard(start))
                require(digest(AUTHORITY) == authority_sha, 'ROOT authority changed'); pins_ok = True
            except Exception as exc:
                error = (error + '; ' if error else '') + 'closing: ' + type(exc).__name__ + ': ' + str(exc)
            signal.setitimer(signal.ITIMER_REAL, 0); signal.signal(signal.SIGALRM, old_handler)
            result.update(error=error, source_and_inputs_unchanged=pins_ok, elapsed_seconds=time.monotonic()-start,
                peak_RSS_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
                deadline_scope='numeric work and closing input/source seals; small failure receipt can follow deadline',
                completed_arm_frames=sum(r['status']=='PASS' for r in rows), failed_arm_frames=sum(r['status']=='FAIL' for r in rows),
                unexecuted_arm_frames=sum(r['status']=='UNEXECUTED' for r in rows))
            if error is None and pins_ok and result['completed_arm_frames'] == 1050:
                result['status'] = 'COMPLETE_DESCRIPTIVE_DIAGNOSTIC_NOT_MODEL_APPROVAL'
            write_new(OUTPUT / 'result.json', result)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('contract', 'run'), default='contract')
    parser.add_argument('--authority-sha')
    args = parser.parse_args()
    if args.mode == 'contract':
        print(encoded(contract()).decode(), end=''); return 0
    result = execute(args.authority_sha)
    print(json.dumps({k: result[k] for k in ('status','completed_arm_frames','failed_arm_frames','unexecuted_arm_frames','elapsed_seconds','error')}, allow_nan=False))
    return 0 if result['status'].startswith('COMPLETE_') else 2


if __name__ == '__main__':
    raise SystemExit(main())
