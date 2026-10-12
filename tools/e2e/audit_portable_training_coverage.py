#!/usr/bin/env python3
"""HH_260906 - Audit metadata coverage and saved optimizer exposure, never dataset payloads or models."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import re

RUNS = tuple(f'seed_{seed}/{arm}' for seed in (20260903, 20260904, 20260905)
             for arm in ('A_original_loss', 'B_cost_aware'))
COUNTS = {'train': 1147, 'val': 337}
CAMERAS = ['CAM_FRONT', 'CAM_BACK', 'CAM_FRONT_LEFT', 'CAM_BACK_LEFT', 'CAM_FRONT_RIGHT', 'CAM_BACK_RIGHT']
LOSS_FIELDS = ('loss', 'regression_loss', 'candidate_score_loss', 'selected_ade_m', 'selected_fde_m',
               'selected_speed_mae_mps', 'selected_yaw_mae_rad', 'selected_kinematic_speed_mae_mps', 'gradient_norm')
DENIALS = dict(dataset_samples_read=False, dataset_images_read=False, dataset_provenance_payloads_read=False,
    test_payloads_read=False, checkpoint_payloads_read=False, neural_inference=False, training_executed=False,
    data_admission_approved=False, convergence_established=False, all_maps_covered=False,
    learned_closed_loop_established=False, vehicle_control_approved=False, automatic_promotion=False)


def require(value, message):
    if not value:
        raise ValueError(message)


def canonical(path, *, directory=False):
    path = Path(path).absolute()
    require('..' not in path.parts and path == path.resolve()
        and not any(p.is_symlink() for p in (path, *path.parents)), 'canonical nonsymlink path required')
    require(path.is_dir() if directory else path.is_file(), 'expected regular input file/directory')
    return path


def child(root, name):
    require(isinstance(name, str) and name and not Path(name).is_absolute()
        and '..' not in Path(name).parts and Path(name).as_posix() == name, 'unsafe metadata reference')
    path = canonical(root / name)
    require(path.is_relative_to(root), 'metadata reference escaped input root')
    return path


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def hash_value(value):
    require(isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value), 'SHA256 required')


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), 'finite numeric value required')
    return value


def integer(value, minimum=0):
    require(type(value) is int and value >= minimum, 'bounded integer value required')
    return value


def loads(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'duplicate JSON key')
            result[key] = value
        return result
    def finite_float(value):
        return number(float(value))
    return json.loads(raw, object_pairs_hook=pairs, parse_float=finite_float,
        parse_constant=lambda _: require(False, 'nonfinite JSON'))


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)+'\n').encode()


class Inputs:
    def __init__(self):
        self.hashes = {}

    def raw(self, path):
        path = canonical(path)
        require(path.stat().st_size <= 16*1024*1024, 'bounded metadata/history input required')
        raw = path.read_bytes()
        value = sha(raw)
        require(str(path) not in self.hashes or self.hashes[str(path)] == value, 'input changed between reads')
        self.hashes[str(path)] = value
        return raw

    def read(self, path):
        return loads(self.raw(path))

    def verify(self):
        for name, expected in self.hashes.items():
            require(sha(canonical(name).read_bytes()) == expected, 'input changed during audit')


def coverage(root, inputs):
    # HH_260906 - This entire function opens only dataset.json and episode.json; no payload path is resolved.
    manifest_path = child(root, 'dataset.json')
    manifest = inputs.read(manifest_path)
    require(manifest['schema_id'] == 'autoware-e2e.common10.dataset.v1'
        and manifest['contract_id'] == 'common_10hz_v1' and manifest['camera_order'] == CAMERAS,
        'fixed Common10 metadata contract required')
    refs = manifest['episodes']
    require(isinstance(refs, list) and 1 <= len(refs) <= 1000, 'bounded nonempty episode references required')
    episodes, seen = [], set()
    for ref in refs:
        eid = ref['episode_id']
        require(isinstance(eid, str) and eid and eid not in seen, 'unique episode identity required')
        require(ref['manifest'] == f'episodes/{eid}/episode.json', 'only contained episode metadata references permitted')
        path = child(root, ref['manifest']); episode = inputs.read(path)
        # HH_260906 - Reject TEST/unknown split immediately, without touching its samples, cameras or provenance.
        require(episode.get('split') in COUNTS, 'TEST and unknown splits forbidden in selected view')
        hash_value(ref['sha256'])
        require(inputs.hashes[str(path)] == ref['sha256'], 'episode metadata hash differs')
        require(episode['schema_id'] == 'autoware-e2e.common10.episode.v1'
            and episode['episode_id'] == eid and episode['domain'] == 'carla', 'episode metadata contract differs')
        start = integer(episode['start_timestamp_ns']); end = integer(episode['end_timestamp_ns'])
        require(end > start, 'episode timestamp duration must be positive')
        count = integer(episode['sample_count'], 1)
        raw_count = integer(episode['capture_accounting']['raw_anchor_count'], count)
        fields = ('source_session_id', 'split_group_id', 'map_id', 'site_id', 'route_id', 'weather')
        require(all(isinstance(episode[k], str) and episode[k] for k in fields), 'episode grouping metadata missing')
        episodes.append(dict(episode_id=eid, split=episode['split'], **{k:episode[k] for k in fields},
            traffic_seed=integer(episode['traffic_seed']), sample_count=count, raw_anchor_count=raw_count,
            start_timestamp_ns=start, end_timestamp_ns=end, recorded_duration_seconds=(end-start)/1e9))
        seen.add(eid)
    require({ep['split'] for ep in episodes} == {'train', 'val'}, 'both TRAIN and VAL metadata required')
    for field in ('source_session_id', 'split_group_id'):
        assignment = {}
        for ep in episodes:
            require(ep[field] not in assignment or assignment[ep[field]] == ep['split'], 'episode/session/route-group split leakage')
            assignment[ep[field]] = ep['split']
    results = {}
    for split in ('train', 'val'):
        group = [ep for ep in episodes if ep['split'] == split]
        require(sum(ep['sample_count'] for ep in group) == COUNTS[split], 'frozen TRAIN/VAL sample count differs')
        duration = math.fsum(ep['recorded_duration_seconds'] for ep in group)
        results[split] = dict(sample_count=COUNTS[split], recorded_episode_count=len(group),
            unique_source_sessions=len({ep['source_session_id'] for ep in group}),
            unique_route_groups=len({ep['split_group_id'] for ep in group}),
            unique_map_count=len({ep['map_id'] for ep in group}), maps=sorted({ep['map_id'] for ep in group}),
            unique_route_count=len({(ep['map_id'], ep['route_id']) for ep in group}),
            unique_weather_count=len({ep['weather'] for ep in group}), weather=sorted({ep['weather'] for ep in group}),
            unique_traffic_seeds=sorted({ep['traffic_seed'] for ep in group}),
            episode_counts_by_map=dict(sorted(Counter(ep['map_id'] for ep in group).items())),
            raw_anchor_count=sum(ep['raw_anchor_count'] for ep in group), recorded_duration_seconds=duration,
            recorded_duration_minutes=duration/60, episodes=group)
    return manifest, results


def exposure(rows, sample_count, batch_size, steps, arm):
    require(type(steps) is int and steps > 0 and len(rows) == steps, 'complete optimizer history required')
    integer(sample_count, 1); integer(batch_size, 1)
    batches = (sample_count+batch_size-1)//batch_size
    metric_fields = LOSS_FIELDS + (('candidate_regret_loss',) if arm == 'B_cost_aware' else ())
    require(arm in ('A_original_loss', 'B_cost_aware'), 'unknown objective arm')
    fields = set(metric_fields) | {'global_step', 'epoch', 'samples_seen', 'batch_domain_sample_counts', 'domain_samples_seen'}
    seen, by_epoch = 0, {}
    for step, row in enumerate(rows, 1):
        require(set(row) == fields, 'history objective fields differ')
        epoch, index = divmod(step-1, batches)
        size = min(batch_size, sample_count-index*batch_size); seen += size
        require(integer(row['global_step'], 1) == step and integer(row['epoch']) == epoch
            and integer(row['samples_seen'], 1) == seen and row['domain_samples_seen'] == {'carla':seen}
            and row['batch_domain_sample_counts'] == {'carla':size}, 'optimizer step/exposure/order differs')
        require(all(number(row[k]) >= 0 for k in metric_fields), 'negative optimizer metric')
        by_epoch.setdefault(epoch, []).append((row, size))
    epochs = []
    for epoch, values in by_epoch.items():
        count = sum(size for _, size in values)
        epochs.append(dict(epoch_number=epoch+1, optimizer_steps=len(values), sample_exposures=count,
            complete=count == sample_count, exposure_weighted_metrics={key:math.fsum(row[key]*size for row,size in values)/count
                for key in metric_fields if key != 'gradient_norm'}))
    return dict(optimizer_steps=steps, batches_per_complete_epoch=batches, sample_exposures=seen,
        complete_epochs=sum(ep['complete'] for ep in epochs), partial_epoch_sample_exposures=epochs[-1]['sample_exposures'] if not epochs[-1]['complete'] else 0,
        equivalent_dataset_passes=seen/sample_count, epochs=epochs,
        gradient_norm_range=[min(row['gradient_norm'] for row in rows), max(row['gradient_norm'] for row in rows)],
        gradient_norm_scope='Saved pre-clipping norms; values above the fixed clipping limit do not establish clipping failure.',
        learning_curve_scope='Exposure-weighted optimizer-time batch means, not fixed-checkpoint TRAIN evaluation or per-epoch VAL curves.')


def audit(dataset_root, campaign_root, plan_path, plan_sha256):
    dataset_root = canonical(dataset_root, directory=True); campaign_root = canonical(campaign_root, directory=True)
    plan_path = canonical(plan_path); hash_value(plan_sha256)
    require(not dataset_root.is_relative_to(campaign_root) and not campaign_root.is_relative_to(dataset_root), 'dataset and campaign must be disjoint')
    source = canonical(__file__); source_sha = sha(source.read_bytes()); inputs = Inputs()
    manifest, measured = coverage(dataset_root, inputs)
    plan = inputs.read(plan_path)
    require(inputs.hashes[str(plan_path)] == plan_sha256, 'prospective plan SHA differs')
    require(plan['schema'] == 'portable_e2e.stopmix_regret_research_plan.v1' and plan['runs'] == list(RUNS)
        and plan['steps'] == 1540 and plan['batch_size'] == 4 and plan['samples_seen'] == 6155
        and plan['arms'] == {'A_original_loss':0., 'B_cost_aware':.1}
        and plan['resume'] is False and plan['test_read_or_use'] is False, 'fixed six-fresh-fit plan required')
    require(inputs.hashes[str(dataset_root/'dataset.json')] == plan['dataset_manifest_sha256'], 'selected view differs from training plan')
    for path, value in inputs.hashes.items():
        if Path(path).is_relative_to(dataset_root):
            require(plan['view_file_sha256'].get(Path(path).relative_to(dataset_root).as_posix()) == value, 'dataset metadata differs from planned inventory')
    archived_plan = inputs.raw(child(campaign_root, 'plan.json'))
    require(archived_plan == encoded(plan), 'archived prospective plan normalization differs')
    result = inputs.read(child(campaign_root, 'summary.json'))
    require(result['status'] == 'COMPLETE_NOT_PROMOTED' and result['plan_sha256'] == plan_sha256
        and result['runs'] == {name:'COMPLETE' for name in RUNS}
        and result['test_read_or_use'] is False and result['automatic_promotion'] is False, 'complete research result required')
    stages = {(row['run'], row['stage']):row for row in result['stages']}
    require(len(stages) == len(result['stages']) == 24
        and set(stages) == {(name, stage) for name in RUNS for stage in ('train','evaluate','audit','behavior')}, 'complete unique stage ledger required')
    fits = []
    train_ids = {ep['episode_id'] for ep in measured['train']['episodes']}
    for name in RUNS:
        reports = {}
        for stage, relative in (('train','training/run.json'), ('evaluate','evaluation/metrics.json')):
            path = child(campaign_root, name+'/'+relative); reports[stage] = inputs.read(path)
            receipt = inputs.read(child(campaign_root, name+'/'+stage+'_completed.json'))
            require(receipt == stages[(name, stage)] and receipt['status'] == 'COMPLETE'
                and receipt['path'] == name+'/'+relative and receipt['sha256'] == inputs.hashes[str(path)], 'stage report/hash ledger differs')
        train, evaluation = reports['train'], reports['evaluate']
        checkpoint_sha = stages[(name,'train')]['checkpoint_sha256']; hash_value(checkpoint_sha)
        require(all(stages[(name,stage)]['checkpoint_sha256'] == checkpoint_sha for stage in ('evaluate','audit','behavior'))
            and evaluation['checkpoint_sha256'] == checkpoint_sha, 'saved final checkpoint identities disagree')
        require(train['status'] == 'TRAINING_TARGET_REACHED' and train['training_split'] == 'train'
            and set(train['training_episode_ids']) == train_ids and len(train['training_episode_ids']) == len(train_ids)
            and train['dataset_size'] == COUNTS['train'] and 'resumed_at_utc' not in train, 'fresh training coverage differs')
        require(train['dataset_fingerprint_sha256'] == plan['split_contract']['train']['fingerprint_sha256']
            and train['corpus_fingerprint_sha256'] == evaluation['corpus_fingerprint_sha256'] == plan['corpus_fingerprint_sha256'], 'dataset fingerprint binding differs')
        require(evaluation['status'] == 'OPEN_LOOP_EVALUATION_COMPLETE' and evaluation['evaluation_split'] == 'val'
            and evaluation['sample_count'] == COUNTS['val']
            and evaluation['dataset_fingerprint_sha256'] == plan['split_contract']['val']['fingerprint_sha256'], 'final VAL denominator differs')
        raw = inputs.raw(child(campaign_root, name+'/training/metrics.jsonl'))
        rows = [loads(line) for line in raw.splitlines()]
        values = exposure(rows, COUNTS['train'], 4, 1540, name.split('/')[1])
        require(values['sample_exposures'] == plan['samples_seen']
            and train['last_metrics'] == rows[-1] and train['state']['global_step'] == 1540
            and train['state']['samples_seen'] == values['sample_exposures'], 'final training history/state differs')
        require(train['model_config']['future_points'] == 64 and train['model_config']['camera_count'] == 6, 'fixed horizon/camera contract required')
        require(all(math.isfinite(number(v)) for v in evaluation['metrics'].values()), 'nonfinite final evaluation')
        fits.append(dict(run=name, checkpoint_sha256_from_saved_receipts=checkpoint_sha, **values,
            final_val_metrics=evaluation['metrics'], final_val_metric_counts=evaluation['metric_counts'],
            per_epoch_val_curve_available=False))
    inputs.verify(); require(sha(source.read_bytes()) == source_sha, 'auditor source changed')
    return dict(schema='portable_e2e.training_coverage_audit.v1', status='METADATA_AND_HISTORY_VERIFIED_NOT_SUFFICIENT_FOR_DEPLOYMENT',
        dataset_id=manifest['dataset_id'], plan_sha256=plan_sha256, source_sha256=source_sha,
        input_sha256_before=inputs.hashes, input_sha256_after=dict(inputs.hashes), coverage=measured, fits=fits,
        total_optimizer_steps=sum(fit['optimizer_steps'] for fit in fits), total_sample_exposures=sum(fit['sample_exposures'] for fit in fits),
        nominal_future_window_overlap=dict(timestep_seconds=.1, future_points=64, horizon_seconds=6.4,
            adjacent_shared_future_time_points=63, adjacent_shared_fraction=63/64,
            scope='Conditional on consecutive regular 10 Hz anchors with all 64 targets valid; cadence/masks are not inspected by this metadata-only audit.'),
        limitations=['Episode durations are metadata end-minus-start spans, including warmup and future-label tail, not moving-only time.',
            'Unique episode/session/route counts describe recorded units; statistical independence is not proved.',
            'Repeated optimizer exposures and random seeds add no new maps, weather, episodes or camera data.',
            'Decreasing TRAIN loss does not prove VAL improvement, convergence or adequate training; only final VAL reports are read.',
            'No corpus payload admission, checkpoint bytes, trajectory math, behavior groups or runtime deployment is independently verified here.',
            'Candidate count is not a count of implemented driving functions; unseen maps and learned closed-loop performance remain unproved.'],
        **DENIALS)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ('dataset-root','campaign-root','plan','output-json'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--plan-sha256', required=True)
    args = parser.parse_args(argv); output = args.output_json.absolute()
    require(not output.exists() and not output.is_symlink() and output == output.resolve()
        and not any(p.is_symlink() for p in output.parents), 'fresh canonical nonsymlink output required')
    for path in (args.dataset_root.absolute(), args.campaign_root.absolute(), args.plan.absolute()):
        require(not output.is_relative_to(path) and not path.is_relative_to(output), 'output must be disjoint from inputs')
    report = audit(args.dataset_root, args.campaign_root, args.plan, args.plan_sha256)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream:
        stream.write(encoded(report)); stream.flush(); os.fsync(stream.fileno())
    print(json.dumps(dict(status=report['status'], total_optimizer_steps=report['total_optimizer_steps'], output_json=str(output))))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
