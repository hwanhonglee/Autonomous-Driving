# HH_260906 - Public source view only: replace the workspace placeholder before use; not the executed source bytes.
"""HH_260906 - Illustrate one archived input-ambiguity pair with unaltered expert images and original future labels."""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil

ROOT = Path('<WORKSPACE_ROOT>')
BASE = ROOT / 'artifacts/training/2026-09-09'
EPISODE_ID = 'carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f'
EPISODE = ROOT / 'datasets/prepared/carla-common10-30kph-five-episodes-20260907-v3/episodes' / EPISODE_ID
DIAG = BASE / 'stopmix_training_oracles_v1'
OUT = BASE / 'identical_input_illustration_v1'
PAIRS = ((907, 1047),)
SAMPLES_SHA = '4ec758139cc54f55f25a071a723298045648c165e864bd34e1b8604b17a71318'
REPORT_SHA = 'cc97748804dcb433973335f7b6c1ea3fb1d5118ba6247306a388aa88d42970fc'
MANIFEST_SHA = 'd0a678b3a7dfebe09754de92dda6f5f5c4d7d3794a4588811a80b9d6353886b0'
CAMERAS = ('CAM_FRONT', 'CAM_BACK', 'CAM_FRONT_LEFT', 'CAM_BACK_LEFT', 'CAM_FRONT_RIGHT', 'CAM_BACK_RIGHT')


def require(value, message):
    if not value:
        raise ValueError(message)


def sha(path):
    require(path.is_file() and not any(p.is_symlink() for p in (path, *path.parents)), 'regular nonsymlink file required')
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_pair(first, second, witnesses):
    # HH_260906 - The first discovered pair is an explanatory, post-hoc example; the full eleven-pair census is reported separately.
    require(first['sequence_index'] == 294 and second['sequence_index'] == 434, 'fixed first pair changed')
    require(first['episode_id'] == second['episode_id'] == EPISODE_ID, 'episode mismatch')
    require(second['anchor_timestamp_ns'] - first['anchor_timestamp_ns'] == 14000000209, 'native timestamps changed')
    require(all(first['ego'][k] == second['ego'][k] for k in ('position_m', 'orientation_xyzw')), 'same-pose illustration requires exact original pose')
    require(first['navigation']['route_polyline_base_m'] == second['navigation']['route_polyline_base_m'], 'same causal route required')
    for row in (first, second):
        plan = row['labels']['planning']
        require(plan['available'] is True and plan['dt_s'] == 0.1, 'original planning ABI required')
        require(len(plan['valid']) == 64 and all(v is True for v in plan['valid']) and len(plan['positions_base_xy_m']) == len(plan['speed_mps']) == 64, 'full fixed horizon required')
        require(all(len(xy) == 2 and all(finite_number(v) for v in xy) for xy in plan['positions_base_xy_m']), 'finite XY required')
        require(all(finite_number(v) for v in plan['speed_mps']), 'finite speed required')
        require(all(type(t) is int for t in plan['target_timestamp_ns']) and plan['target_timestamp_ns'] == [row['anchor_timestamp_ns'] + (i + 1) * 100000000 for i in range(64)], 'native targets changed')
        require(len(row['ego']['position_m']) == 3 and len(row['ego']['orientation_xyzw']) == 4
            and all(finite_number(v) for k in ('position_m', 'orientation_xyzw') for v in row['ego'][k]), 'finite original pose required')
        require(bool(row['navigation']['route_polyline_base_m']) and all(len(xy) == 2 and all(finite_number(v) for v in xy)
            for xy in row['navigation']['route_polyline_base_m']), 'finite nonempty causal route required')
        require(tuple(cam['name'] for cam in row['camera_bundle']) == CAMERAS, 'camera order differs')
    require(len(witnesses) == 6, 'all six original diagnostic passes required')
    input_hash = None
    for pair in witnesses:
        a, b = pair
        require(a['model_input_sha256'] == b['model_input_sha256'], 'different model inputs')
        if input_hash is None:
            input_hash = a['model_input_sha256']
        require(a['model_input_sha256'] == input_hash, 'cross-checkpoint input differs')
        require(a['candidate_logits'] == b['candidate_logits'], 'original logits differ')
        require(a['target_sha256'] != b['target_sha256'], 'targets are not different')
        require(not set(a['exact_minimum_indices']) & set(b['exact_minimum_indices']), 'oracle sets overlap')
        for raw, archived in zip((first, second), pair):
            require(raw['sample_id'] == archived['sample_id'] and raw['anchor_timestamp_ns'] == archived['anchor_timestamp_ns'], 'sample identity differs')
            require(raw['labels']['planning']['speed_mps'] == archived['raw_target_speed_mps'], 'raw speed labels differ')
            require([cam['sha256'] for cam in raw['camera_bundle']] == archived['camera_sha256'], 'camera witness differs')
    require([cam['sha256'] for cam in first['camera_bundle']] == [cam['sha256'] for cam in second['camera_bundle']], 'raw camera bytes differ')
    return input_hash


def main():
    started = datetime.now(timezone.utc).isoformat()
    own_sha = sha(Path(__file__))
    require(not OUT.exists(), 'refuse to overwrite any prior attempt')
    require(not any(p.is_symlink() for p in (OUT, *OUT.parents)) and OUT.resolve().is_relative_to(BASE.resolve()), 'private nonsymlink output required')
    require(not OUT.resolve().is_relative_to(DIAG.resolve()) and not OUT.resolve().is_relative_to((ROOT / 'datasets').resolve()), 'output must not overlap original evidence or dataset')
    require(sha(EPISODE / 'samples.jsonl') == SAMPLES_SHA, 'original sample file changed')
    require(sha(DIAG / 'report.json') == REPORT_SHA and sha(DIAG / 'SHA256SUMS') == MANIFEST_SHA, 'diagnostic provenance changed')
    original_inventory = {}
    for line in (DIAG / 'SHA256SUMS').read_text().splitlines():
        expected, relative = line.split('  ', 1)
        path = DIAG / relative
        require(path.resolve().is_relative_to(DIAG.resolve()) and sha(path) == expected, 'diagnostic checksum mismatch')
        original_inventory[relative] = expected
    source_lines = (EPISODE / 'samples.jsonl').read_bytes().splitlines(keepends=True)
    require(len(source_lines) == 534, 'original sample count changed')
    first, second = (json.loads(source_lines[i]) for i in (294, 434))
    witnesses, diagnostic_files = [], {}
    for seed in (20260903, 20260904, 20260905):
        for stage in ('parent1540', 'continued2870'):
            path = DIAG / f'seed_{seed}/{stage}/samples.jsonl'
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            require(len(rows) == 1147, 'diagnostic row count changed')
            witnesses.append([rows[907], rows[1047]])
            diagnostic_files[str(path.relative_to(DIAG))] = sha(path)
    input_hash = validate_pair(first, second, witnesses)
    image_sources = []
    for row in (first, second):
        for cam in row['camera_bundle']:
            path = EPISODE / cam['path']
            require(path.resolve().is_relative_to(EPISODE.resolve()) and sha(path) == cam['sha256'], 'original camera file mismatch')
            image_sources.append({'sample_id': row['sample_id'], 'path_relative_to_episode': cam['path'], 'sha256': cam['sha256']})
    # HH_260906 - Matplotlib renders recorded numeric labels; source JPEGs are copied byte-for-byte without synthetic scene generation.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    OUT.mkdir()
    (OUT / 'original_images').mkdir()
    for cam in first['camera_bundle']:
        shutil.copyfile(EPISODE / cam['path'], OUT / 'original_images' / (cam['name'] + '.jpg'))
    (OUT / 'original_pair_samples.jsonl').write_bytes(source_lines[294] + source_lines[434])
    shutil.copyfile(Path(__file__), OUT / Path(__file__).name)
    fig, axes = plt.subplots(2, 3, figsize=(16, 7.4), constrained_layout=True)
    order = ('CAM_FRONT_LEFT', 'CAM_FRONT', 'CAM_FRONT_RIGHT', 'CAM_BACK_LEFT', 'CAM_BACK', 'CAM_BACK_RIGHT')
    for ax, name in zip(axes.flat, order):
        with Image.open(OUT / 'original_images' / (name + '.jpg')) as im:
            im.load()
            require(im.size == (640, 360), 'source camera dimensions changed')
            ax.imshow(im)
        ax.set_title(name + ' | same raw JPEG SHA256', fontsize=10)
        ax.axis('off')
    fig.suptitle('Recorded CARLA expert cameras: identical inputs at Town01 sequences 294 and 434\nOnboard views, not a learned-model drive; traffic-signal ground truth was not recorded', fontsize=14)
    fig.savefig(OUT / 'same_six_camera_inputs.png', dpi=120)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), constrained_layout=True)
    for row, color, label in ((first, '#1765b8', 'sequence 294: remains stopped'), (second, '#dd731a', 'sequence 434: future departure')):
        plan = row['labels']['planning']
        axes[0].plot([xy[0] for xy in plan['positions_base_xy_m']], [xy[1] for xy in plan['positions_base_xy_m']], color=color, label=label, linewidth=2.2)
        axes[1].plot([(t-row['anchor_timestamp_ns']) / 1e9 for t in plan['target_timestamp_ns']], [v * 3.6 for v in plan['speed_mps']], color=color, label=label, linewidth=2.2)
    route = first['navigation']['route_polyline_base_m']
    axes[0].plot([xy[0] for xy in route], [xy[1] for xy in route], ':', color='#777777', label='causal navigation route')
    axes[0].scatter([0], [0], marker='^', s=100, c='black', label='ego anchor (same pose)', zorder=5)
    axes[0].set(xlim=(-30, 30), ylim=(-30, 30), xlabel='Forward x (m)', ylabel='Left y (m)', title='Original future labels: ego at plot center')
    axes[0].set_aspect('equal', adjustable='box')
    axes[1].set(xlim=(0, 6.4), ylim=(-0.5, 33), xlabel='Native target offset (s)', ylabel='Original target speed (km/h)', title='6.4-second future speed labels')
    for ax in axes:
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8, loc='upper left')
    fig.suptitle('Same present model input, different future labels\nOffline expert-data illustration; no label repair, model rollout or driving approval', fontsize=14)
    fig.savefig(OUT / 'different_future_labels.png', dpi=140)
    plt.close(fig)
    for relative, expected in original_inventory.items():
        require(sha(DIAG / relative) == expected, 'diagnostic changed while rendering')
    require(sha(DIAG / 'SHA256SUMS') == MANIFEST_SHA, 'original diagnostic manifest changed while rendering')
    require(sha(EPISODE / 'samples.jsonl') == SAMPLES_SHA, 'source samples changed while rendering')
    for source in image_sources:
        require(sha(EPISODE / source['path_relative_to_episode']) == source['sha256'], 'source image changed while rendering')
    for cam in first['camera_bundle']:
        require(sha(OUT / 'original_images' / (cam['name'] + '.jpg')) == cam['sha256'], 'copied original image bytes differ')
    require((OUT / 'original_pair_samples.jsonl').read_bytes() == source_lines[294] + source_lines[434], 'copied original sample lines differ')
    require(sha(Path(__file__)) == own_sha == sha(OUT / Path(__file__).name), 'illustration source changed while rendering')
    report = {'comment': 'HH_260906 - Post-hoc explanatory illustration of the first of eleven verified duplicate-input pairs; not an independent evaluation.',
        'started_at_utc': started, 'completed_at_utc': datetime.now(timezone.utc).isoformat(),
        'status': 'ORIGINAL_INPUT_AND_TARGET_ILLUSTRATION_COMPLETE_NOT_PROMOTED',
        'episode_id': EPISODE_ID, 'sequence_indices': [294, 434], 'global_train_indices': [907, 1047],
        'model_input_sha256': input_hash, 'source_samples_sha256': SAMPLES_SHA,
        'illustration_source_sha256': own_sha,
        'original_diagnostic_report_sha256': REPORT_SHA, 'original_diagnostic_manifest_sha256': MANIFEST_SHA,
        'six_diagnostic_jsonl_sha256': diagnostic_files, 'twelve_original_image_sources': image_sources,
        'native_anchor_timestamp_ns': [first['anchor_timestamp_ns'], second['anchor_timestamp_ns']],
        'native_anchor_difference_ns': second['anchor_timestamp_ns'] - first['anchor_timestamp_ns'],
        'original_sample_line_sha256': [hashlib.sha256(source_lines[i]).hexdigest() for i in (294, 434)],
        'source_and_diagnostic_unchanged': True, 'source_images_copied_byte_exact': True,
        'labels_or_timestamps_modified': False, 'new_training_or_inference': False,
        'traffic_signal_ground_truth_available': False, 'vehicle_control_approved': False,
        'interpretation': 'A deterministic model with this identical input ABI cannot match both different future labels. This is not proof of incorrect labels, the sole error cause, or observed signal timing.'}
    (OUT / 'report.json').write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n')
    files = sorted(p for p in OUT.rglob('*') if p.is_file())
    (OUT / 'SHA256SUMS').write_text(''.join(f'{sha(p)}  {p.relative_to(OUT)}\n' for p in files))
    print(json.dumps({'status': report['status'], 'file_count': len(files) + 1, 'report_sha256': sha(OUT / 'report.json')}))


if __name__ == '__main__':
    main()
