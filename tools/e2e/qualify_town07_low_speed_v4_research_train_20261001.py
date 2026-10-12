"""HH_261001 - Authenticate one V4 steady-cruise research TRAIN conversion without release approval."""

from __future__ import annotations

import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path

from scripts.e2e import prepare_carla_common10_dataset as adapter
from scripts.e2e import audit_town07_low_speed_moving_v4_20261001 as v4
from scripts.e2e import qualify_carla_moving_common10 as reconstruction
from scripts.e2e import town07_low_speed_moving_v4_task_20261001 as task
from portable_e2e import contract, dataset


ROOT = task.ROOT
CASE = task.RUN
EPISODE = CASE / 'episode'
PREPARED = ROOT / 'storage/datasets/prepared/town07-low-speed-cruise-v4-research-train-20261001-v1'
QA = ROOT / 'docs/assets/validation/2026-10-01/town07_low_speed_moving_v4_qa_v1'
SCHEMA = 'carla.town07_low_speed_v4_research_train_receipt.20261001.v1'
RESULT_SCHEMA = 'carla.town07_low_speed_v4_research_train_conversion_result.20261001.v1'
ADAPTER_ID = 'carla_town07_low_speed_v4_research_train_to_common10_20261001_v1'
DATASET_ID = 'town07-low-speed-cruise-v4-research-train-20261001-v1'
PLAN_SHA = '92b95eb6f7c535a4d6d0c4bc463db0960aeb86834bc8d6c02c3ae8a6ac8e2015'
ADDENDUM_SHA = 'd4a2a2e2ec9f72ddd9b8410570852c1e302eb227d3401b190f61b69754370989'
OWNER_SHA = '6c5937580e16c4b316e9806ad19366febbffd48930a1be84314d84d9ad5d60a7'
NATIVE_MANIFEST_SHA = 'a43a0a5f98dad1a3c777ec0896b2b97de20ac4846e5edb80ff80097cf0163658'
QA_SHA = '51e646cb56bab73c4e6120b7ab5a331cca250992c2ca90b1f3f5ba24ca9f7dab'
LEDGER_SHA = {
    'all_camera_futures.jsonl.gz': 'b734dd230561eb2891b09061d011fd6ce35716e6e6244cbe0b2190f127608d63',
    'all_control_checks.jsonl.gz': '84d3ab62c415733e49a86e4848e17b0b9a14645011d4adc01515344f2617638c',
    'all_image_checks.jsonl.gz': '8f4118988607214b9247522a542bf8803363c6d0f4d84da5865f1400088a72fa',
    'all_native.jsonl.gz': '52d4420dba75a7938a5507b26ee335b564e1e0051a7ba450180cf932502d6a84',
    'all_projection_rows.jsonl.gz': '112d766507547b0cee8c3cc92ad4951a5c5f2ac9fc83207fa0bc1917b8da7820',
    'all_selected_windows.jsonl.gz': '17d9320a0029cd24b49b6aff05a2a4587e12504083600b05052bfd48cef838cb',
}
SOURCE_PATHS = (
    'tools/e2e/qualify_town07_low_speed_v4_research_train_20261001.py',
    'tools/e2e/prepare_town07_low_speed_v4_common10_20261001.py',
    'tools/e2e/prepare_carla_common10_dataset.py',
    'tools/e2e/qualify_carla_moving_common10.py',
    'tools/e2e/audit_portable_target_feasibility.py',
    'learning/portable_e2e/contract.py', 'learning/portable_e2e/dataset.py',
    'learning/portable_e2e/torch_dataset.py', 'learning/portable_e2e/temporal_dataset_v2.py',
    'learning/portable_e2e/config/common_10hz_v1.contract.json',
)


def require(condition, message):
    # HH_261001 - Retain the converter's fail-closed exception type for every research guard.
    if not condition:
        raise adapter.AdapterError(message)


def safe(path, *, under_root=True):
    # HH_261001 - Reject lexical traversal and every symlink, including ancestors.
    path = Path(path).absolute()
    require('..' not in path.parts and path == path.resolve()
            and not any(parent.is_symlink() for parent in (path, *path.parents)),
            'canonical nonsymlink path required')
    require(not under_root or path.is_relative_to(ROOT), 'research path escapes workspace')
    return path


def sha(path):
    return adapter._sha256(safe(path, under_root=False))


def document(path):
    return adapter._json(safe(path))


def hash_value(value):
    require(isinstance(value, str) and contract.SHA256_PATTERN.fullmatch(value) is not None,
            'external SHA256 must be lowercase 64-hex')
    return value


def ledger(name):
    with gzip.open(safe(QA / name), 'rt') as stream:
        return [adapter._loads_json(line, name) for line in stream if line.strip()]


def scope_manifest(manifest):
    # HH_261001 - PENDING_QA is not an explicit denial; do not erase or invert any real denial.
    adapter._require_native_admission_not_denied(manifest)
    contract._require_admission_not_denied(manifest, 'native manifest', include_native_metadata=True)
    result = manifest.get('result', {})
    require(manifest.get('status') == 'complete' and manifest.get('cleanup', {}).get('completed') is True,
            'complete native capture and cleanup required')
    require(result.get('task_complete') is True and result.get('goal_reached') is False,
            'truthful completed moving task with false catalog goal required')
    for item in (manifest, result, result.get('moving_task', {}),
                 manifest.get('capture_contract', {}).get('agent_initialization', {})):
        require(item.get('admission_status') == 'PENDING_QA'
                and item.get('dataset_conversion_performed') is False
                and item.get('vehicle_control_approved') is False,
                'original fresh PENDING metadata must remain unchanged')
    require(result.get('collision_event_count') == result.get('lane_invasion_event_count') == 0,
            'native collision or lane event prevents conversion')
    moving = manifest.get('capture_contract', {}).get('moving_task', {})
    require(moving.get('plan_sha256') == PLAN_SHA and moving.get('cohort_slot_id') == task.SLOT
            and moving.get('dataset_split_intent') == 'train'
            and moving.get('selected_progress_m') == [40., 120.], 'V4 capture scope differs')


def evidence(*, decode_images=False):
    # HH_261001 - Bind the completed audit and every journal before inspecting derived authority.
    inputs = {}

    def pin(path, expected=None):
        path = safe(path, under_root=False)
        digest = sha(path)
        require(expected is None or digest == expected, 'pinned input changed: ' + str(path))
        require(str(path) not in inputs or inputs[str(path)] == digest,
                'input changed between repeated reads: ' + str(path))
        inputs[str(path)] = digest
        return digest

    pin(QA / 'summary.json', QA_SHA)
    summary = document(QA / 'summary.json')
    require(summary['status'] == v4.CLEAR and summary['fixed_denominator_clear'] is True
            and summary['selected_window_count'] == 327 and summary['thresholds_relaxed'] is False
            and summary['window_filtering_performed'] is False
            and summary['training_data_approved'] is False,
            'unchanged complete 327-window necessary-check result required')
    numerical = summary['numerical_audit']
    require(all(value is True for value in numerical['flags'].values())
            and numerical['window_failure_counts'] == {}
            and numerical['scoped_native_failures']['protected'] == [],
            'a selected or protected numerical check failed')
    for name, expected in LEDGER_SHA.items():
        pin(QA / name, expected)
    for group in ('input_sha256',):
        for name, digest in numerical[group].items():
            pin(Path(name), digest)
    for group in ('v4_source_input_sha256', 'audit_source_sha256'):
        for name, digest in summary[group].items():
            pin(Path(name), digest)
    source, source_inputs = v4.source_audit(CASE, PLAN_SHA, ADDENDUM_SHA, OWNER_SHA, NATIVE_MANIFEST_SHA)
    require(source == summary['v4_source_evidence'] and source['status'] == 'PASS'
            and source_inputs == summary['v4_source_input_sha256'],
            'independent V4 source postcheck differs')
    manifest = document(EPISODE / 'manifest.json')
    pin(EPISODE / 'manifest.json', NATIVE_MANIFEST_SHA)
    scope_manifest(manifest)
    plan = task.verify_plan(PLAN_SHA)
    states = adapter._jsonl(EPISODE / 'states.jsonl')
    cameras = adapter._jsonl(EPISODE / 'camera_frames.jsonl')
    route = document(EPISODE / 'route.json')
    require(len(states) == 1395 and len(cameras) == 698, 'fixed native denominators changed')
    with task.activated():
        replay = task.original.reconstruct(plan, states, cameras, route['route'])
        require(replay.result(tail_states=130, goal_reached=False) == manifest['result']['moving_task'],
                'independent geometric membership replay differs')
    selected = numerical['membership']['selected']
    windows = ledger('all_selected_windows.jsonl.gz')
    require(len(selected) == len(windows) == 327
            and all(window['necessary_window_checks_clear'] is True and window['failure_reasons'] == []
                    for window in windows), 'exact selected window census differs')
    for expected, window in zip(selected, windows):
        require(all(window[key] == expected[key] for key in ('frame', 'timestamp_ns', 'progress_m')),
                'selected journal membership changed')
    selected_frames = [row['frame'] for row in selected]
    require(selected_frames == list(range(1648, 2301, 2)), 'fixed camera frame denominator changed')
    require(Counter(row['capture_phase'] for row in states)
            == dict(stationary_warmup=70, driving=1195, stationary_tail=130),
            'original startup/driving/tail denominators changed')
    image_rows = ledger('all_image_checks.jsonl.gz')
    require(len(image_rows) == 4188 and len({row['path'] for row in image_rows}) == 4188,
            'full six-camera image provenance denominator differs')
    for row in image_rows:
        require(row['decoded'] is True and row['referenced'] is True, 'original pixel QA failed')
        path = adapter._safe_source_file(EPISODE, row['path'], 'original pixel source')
        pin(path, row['sha256'])
        if decode_images:
            from PIL import Image
            with Image.open(path) as image:
                image.load()
                require(image.size == (640, 360) and image.mode == 'RGB'
                        and hashlib.sha256(image.tobytes()).hexdigest() == row['decoded_pixel_sha256'],
                        'pixel content differs from original QA')
    source_pins = {name: sha(ROOT / name) for name in SOURCE_PATHS}
    for name, digest in inputs.items():
        require(sha(Path(name)) == digest, 'input changed during research qualification')
    return dict(schema=SCHEMA, status='QUALIFIED_FOR_RESEARCH_TRAIN_CONVERSION',
        case=str(CASE), prepared_output=str(PREPARED), dataset_id=DATASET_ID,
        qa_summary_sha256=QA_SHA, task_plan_sha256=PLAN_SHA, addendum_sha256=ADDENDUM_SHA,
        owner_plan_sha256=OWNER_SHA, native_manifest_sha256=NATIVE_MANIFEST_SHA,
        source_sha256=source_pins, input_sha256=inputs,
        selected_camera_frames=selected_frames, selected_anchors=selected,
        selected_camera_anchors=327, raw_camera_anchors=698, context_only_camera_anchors=371,
        raw_images=4188, selected_images=1962, native_states=1395,
        selected_span_ns=32_600_000_486,
        route_identity=dict(route_id='town07_straight_v4_research_train',
            route_sha256=pin(EPISODE / 'route.json', task.original.ROUTE_SHA),
            route_group_independence='NOT_CERTIFIED: same existing TRAIN Town07 corridor'),
        research_training_authorized=True, authority='Versioned technical research TRAIN qualification; no external human approval asserted.',
        conversion_only=True, prepared_dataset_validation_performed=False, training_run_performed=False,
        raw_native_denials_overridden=False, original_pending_metadata_unchanged=True,
        whole_episode_goal_reached=False, whole_episode_goal_stop_quality=manifest['result']['whole_recording_goal_stop_quality_diagnostic'],
        outside_protected_native_failures=numerical['scoped_native_failures']['outside_protected'],
        vehicle_control_approved=False, model_or_vehicle_release_approved=False,
        notice='One overlapping-window cruise TRAIN episode; no DEV/TEST, launch, recovery, goal-stop, route success, generalization or release claim. Genuine unavailable prepared history is masked, never copied from the current image.')


def recheck(qualification):
    require(sha(qualification['receipt_path']) == qualification['receipt_sha256'], 'research receipt changed')
    value = qualification['document']
    for name, digest in value['input_sha256'].items():
        require(sha(Path(name)) == digest, 'research input changed: ' + name)
    for name, digest in value['source_sha256'].items():
        require(sha(ROOT / name) == digest, 'research converter source changed: ' + name)


def authorize(args):
    # HH_261001 - Require root-reviewed external receipt hash and an explicit research-only CLI flag.
    receipt = getattr(args, 'moving_qualification', None)
    digest = getattr(args, 'moving_qualification_sha256', None)
    require(receipt is not None and digest is not None, 'paired research receipt and external SHA required')
    require(getattr(args, 'execute_reviewed_research_conversion', False) is True,
            'explicit reviewed research conversion flag required')
    receipt = safe(receipt)
    require(sha(receipt) == hash_value(digest), 'research receipt external SHA differs')
    require(len(args.episode) == 1 and args.episode[0].split == 'train'
            and safe(args.episode[0].path) == EPISODE, 'one exact V4 TRAIN episode required')
    require(args.dataset_id == DATASET_ID and safe(args.output) == PREPARED
            and args.image_mode == 'copy' and args.validate_mode == 'planning',
            'fixed fresh research dataset identity and copy/planning mode required')
    value = document(receipt)
    require(value == evidence(decode_images=False), 'research receipt differs from rederived evidence')
    qualification = dict(receipt_path=str(receipt), receipt_sha256=digest, document=value)
    recheck(qualification)
    return qualification


def verify_native_scope(spec, manifest, qualification):
    scope_manifest(manifest)
    require(spec.split == 'train' and safe(spec.path) == EPISODE
            and qualification['document']['native_manifest_sha256'] == sha(EPISODE / 'manifest.json')
            and qualification['document']['research_training_authorized'] is True,
            'native input differs from authenticated research scope')


def native_route_id(spec, manifest, route, qualification):
    verify_native_scope(spec, manifest, qualification)
    require(sha(spec.path / 'route.json') == qualification['document']['route_identity']['route_sha256'],
            'native route bytes differ')
    return qualification['document']['route_identity']['route_id']


def validate_prepared(prepared, native, qualification, native_report):
    # HH_261001 - Reconstruct all 20,928 unchanged native targets and all copied pixels before publication.
    from scripts.e2e import audit_portable_target_feasibility as targets
    from portable_e2e import temporal_dataset_v2 as temporal
    recheck(qualification)
    prepared = safe(prepared)
    loaded = dataset.load_training_examples(prepared, split='train', mode='planning', check_image_hashes=True)
    require(len(loaded.examples) == native_report['sample_count'] == 327
            and native_report['episode_count'] == 1, 'unchanged Common10 TRAIN denominator differs')
    manifest = document(prepared / 'dataset.json')
    episode_path = prepared / manifest['episodes'][0]['manifest']
    episode_root = episode_path.parent
    episode = document(episode_path)
    samples = adapter._jsonl(episode_root / 'samples.jsonl')
    collection = document(episode_root / 'collection_config.json')
    require(episode['split'] == 'train' and episode['source_provenance']['adapter_id'] == ADAPTER_ID
            and collection['adapter_id'] == ADAPTER_ID, 'research adapter/split identity differs')
    require(collection['native_capture_contract'] == native.manifest['capture_contract']
            and collection['native_runtime'] == native.manifest['runtime']
            and collection['native_result'] == native.manifest['result'], 'native metadata was normalized')
    for name, expected in (('native_manifest.json', NATIVE_MANIFEST_SHA),
                           ('native_route.json', native.source_hashes['route.json']),
                           ('moving_qualification.json', qualification['receipt_sha256'])):
        require(sha(episode_root / name) == expected, 'immutable raw/receipt copy differs')
    timeline = adapter._state_timeline(native)
    state_by_stamp = dict(timeline)
    stamps = list(state_by_stamp)
    polyline = []
    import math
    for point in native.route['route']:
        xy = [float(point['x']), float(point['y'])]
        if not polyline or math.dist(polyline[-1], xy) > 1e-9:
            polyline.append(xy)
    require(document(episode_root / 'route_geometry.json')
            == dict(route_id=native.route_id, frame_id='map', polyline_m=polyline),
            'prepared route geometry differs from original measured route')
    source = document(episode_root / 'source_manifest.json')
    require(len(source['camera_frames']) == 4188, 'raw context image provenance was lost')
    for image in source['camera_frames']:
        require(image['source_payload_sha256'] == qualification['document']['input_sha256'][str(EPISODE / image['source_uri'])],
                'raw context image hash differs')
    by_frame = {row['frame']: row for row in native.camera_frames}
    audits, copied = [], []
    require(len(samples) == len(qualification['document']['selected_anchors']) == 327,
            'fixed selected anchor count differs')
    contract_data = contract.load_contract()
    for index, (sample, selected) in enumerate(zip(samples, qualification['document']['selected_anchors'])):
        require(sample['sequence_index'] == index and sample['anchor_timestamp_ns'] == selected['timestamp_ns'],
                'fixed selected ordering/timestamp differs')
        reconstruction.independently_expected(native, sample, state_by_stamp, stamps, polyline,
            episode['route_geometry_sha256'], contract_data)
        for item, camera_name in zip(sample['camera_bundle'], contract.CAMERA_ORDER):
            raw_path = EPISODE / by_frame[selected['frame']]['images'][camera_name]
            output_path = episode_root / item['path']
            require(item['name'] == camera_name and sha(raw_path) == sha(output_path) == item['sha256']
                    and raw_path.stat().st_ino != output_path.stat().st_ino
                    and output_path.stat().st_nlink == 1, 'selected image is not an independent identical copy')
            copied.append(output_path)
        audit = targets.audit_sample(sample, route_length_m=native.route['route_length_m'],
            episode_start_ns=episode['start_timestamp_ns'], episode_end_ns=episode['end_timestamp_ns'],
            tail_start_ns=next(stamp for stamp, state in timeline if state['capture_phase'] == 'stationary_tail'))
        audit['episode_id'] = native.episode_id
        audits.append(audit)
    require(len(copied) == 1962 and set(copied) == {path for path in (episode_root / 'images').rglob('*') if path.is_file()},
            'selected pixel copy denominator differs')
    require(episode['capture_accounting']['ineligible_anchor_reasons']
            == {'moving_task_fixed_window_context_only': 371}, 'startup/tail context selection differs')
    measured = targets.summarize_samples(audits, 64)
    require(measured['valid_step_count'] == 20_928 and measured['invalid_step_count'] == 0
            and all(metric['violating_step_count'] == 0 for metric in measured['metrics'].values()),
            'unchanged prepared target feasibility audit failed')
    raw_metadata = temporal.load_common10_temporal_metadata(prepared, split='train',
        expected_tokens=tuple(example.token for example in loaded.examples))
    view = temporal.TemporalCommon10TorchDataset(loaded.examples, raw_metadata=raw_metadata,
        split='train', verify_image_sha256=True)
    # HH_261001 - Check real tensor history for the leading boundary and the first fully supported anchor.
    history_masks = [view[index]['time_valid'].tolist() for index in range(9)]
    require(sum(history_masks[0]) == 1 and all(history_masks[8]), 'boundary temporal mask differs')
    full_camera_history = sum(all(view._past_index(example, offset) is not None
        for offset in temporal.TIME_OFFSETS_NS) for example in view.examples)
    full_ego_history = sum(bool(view._ego_history(example)[1].all()) for example in view.examples)
    require(full_camera_history == 319 and full_ego_history == 318,
            'actual selected-only history denominator differs')
    recheck(qualification)
    return dict(schema=RESULT_SCHEMA, status='QUALIFIED_RESEARCH_TRAIN_COMMON10',
        research_training_authorized=True, vehicle_control_approved=False,
        qualification_receipt_sha256=qualification['receipt_sha256'],
        dataset_manifest_sha256=sha(prepared / 'dataset.json'),
        validation_report_sha256=sha(prepared / 'validation_report.json'),
        train_loader_fingerprint_sha256=loaded.fingerprint_sha256,
        temporal_dataset_fingerprint_sha256=view.fingerprint_sha256,
        train_examples=327, episode_count=1, val_examples=0, test_examples=0,
        rederived_target_points=20_928, exact_native_ego_navigation_and_target_reconstruction=True,
        raw_native_denials_overridden=False, original_pending_metadata_unchanged=True,
        raw_camera_anchors=698, selected_camera_anchors=327, context_only_camera_anchors=371,
        original_raw_image_hashes_preserved=4188, independently_copied_selected_images=1962,
        first_nine_temporal_history_masks=history_masks,
        full_five_camera_history_examples=full_camera_history, full_ten_ego_history_examples=full_ego_history,
        synthetic_history_created=False, unchanged_target_audit=measured,
        outside_protected_native_failures=qualification['document']['outside_protected_native_failures'],
        whole_episode_goal_stop_quality=qualification['document']['whole_episode_goal_stop_quality'],
        whole_episode_goal_reached=False, training_run_performed=False,
        model_or_vehicle_release_approved=False,
        notice='One prospective fixed-window TRAIN-only cruise episode; first boundary histories masked, raw startup and forced-tail failures retained, no release or independent validation claim.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--decode-images', action='store_true')
    args = parser.parse_args(argv)
    output = safe(args.output)
    require(not output.exists(), 'create-only research qualification receipt required')
    for protected in (CASE, QA, PREPARED):
        require(not output.is_relative_to(protected) and not protected.is_relative_to(output),
                'research receipt overlaps original/derived dataset evidence')
    value = evidence(decode_images=args.decode_images)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(status=value['status'], research_training_authorized=True,
        vehicle_control_approved=False, receipt_sha256=sha(output), output=str(output))))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
