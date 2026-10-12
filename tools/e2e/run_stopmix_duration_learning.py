#!/usr/bin/env python3
"""HH_260906 - Exact same-view STOPMIX continuation, epochwise VAL, never promotion."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import sys
import time

# HH_260906 - Importing a sealed archived helper must not add __pycache__ files to its parent campaign.
sys.dont_write_bytecode = True

WORKSPACE = Path.home() / 'personal/hwanhong/portable_e2e'
PARENT = 'runs/diagnostics/stopmix_regret_20260914_v1'
OUTPUT = 'runs/diagnostics/stopmix_duration_20260915_v1'
BASE_NAME = 'run_stopmix_regret_research.py'
BASE_SHA = '170ddeea85d4a3330229cb0850ec919e8edf9b44a07947f76ebac2d7b1b66e7f'
RUNS = tuple(f'seed_{seed}/{arm}' for seed in (20260903, 20260904, 20260905)
             for arm in ('A_original_loss', 'B_cost_aware'))
EPOCHS = tuple(range(6, 21))
RESUME_FILES = ('run.json', 'metrics.jsonl', 'checkpoints/latest.pt')
VIEW_KEYS = ('source_sha256', 'dataset', 'dataset_manifest_sha256', 'view_report_path',
             'view_report_sha256', 'view_file_sha256', 'split_contract', 'corpus_fingerprint_sha256')
DENIALS = dict(training_data_approved=False, vehicle_control_approved=False,
    automatic_promotion=False, automatic_retry=False, test_read_or_use=False,
    model_export=False, runtime_gate_changes=False, convergence_claim=False,
    all_map_claim=False, closed_loop_claim=False)


def require(value, message):
    if not value:
        raise ValueError(message)


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()


def finite(value):
    if isinstance(value, dict):
        for item in value.values(): finite(item)
    elif isinstance(value, list):
        for item in value: finite(item)
    elif isinstance(value, float):
        require(math.isfinite(value), 'nonfinite JSON value')


def decode(value):
    def pairs(items):
        result = {}
        for key, item in items:
            require(key not in result, 'duplicate JSON key')
            result[key] = item
        return result
    result = json.loads(value, object_pairs_hook=pairs,
        parse_constant=lambda _: require(False, 'nonfinite JSON value'))
    finite(result)
    return result


def canonical(path):
    path = Path(path).absolute()
    require('..' not in path.parts and path == path.resolve()
        and not any(p.is_symlink() for p in (path, *path.parents)), 'canonical nonsymlink path required')
    return path


def digest(path):
    path = canonical(path)
    require(path.is_file(), 'regular file required')
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''): value.update(block)
    return value.hexdigest()


def read(path):
    path = canonical(path)
    require(path.is_file(), 'regular JSON file required')
    return decode(path.read_bytes())


def write(path, value):
    with canonical(path).open('xb') as stream:
        stream.write(encoded(value)); stream.flush(); os.fsync(stream.fileno())


def utc():
    return datetime.now(timezone.utc).isoformat()


def progress(output, value):
    # HH_260906 - Only this explicitly mutable status is replaced; scientific stage receipts stay create-only.
    temporary = output / 'progress.json.partial'
    write(temporary, value)
    canonical(output / 'progress.json')
    os.replace(temporary, output / 'progress.json')


def load_base(parent):
    path = canonical(parent) / 'source' / BASE_NAME
    require(digest(path) == BASE_SHA, 'sealed Sep14 runner changed')
    spec = importlib.util.spec_from_file_location('_duration_sealed_regret', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module; spec.loader.exec_module(module)
    require(digest(path) == BASE_SHA, 'sealed runner changed during import')
    return module


def inventory(root, expected=None):
    root = canonical(root); require(root.is_dir(), 'inventory root missing')
    result = {}
    for parent, directories, files in os.walk(root, followlinks=False):
        for name in [*directories, *files]: canonical(Path(parent) / name)
        for name in files:
            path = Path(parent) / name; key = path.relative_to(root).as_posix()
            if expected is not None: require(key in expected, 'unplanned parent file; refuse read')
            result[key] = digest(path)
    if expected is not None: require(result == expected, 'parent inventory changed')
    return result


def plan_contract(base):
    return dict(schema='portable_e2e.stopmix_duration_learning_plan.v1',
        campaign_id='hh260915-stopmix-duration-3seeds-v1', parent_root=PARENT, output_root=OUTPUT,
        source_commit=base.SOURCE_COMMIT, gpu_uuid=base.GPU_UUID, model_config=base.MODEL_PATH,
        model_config_sha256=base.MODEL_SHA, owner_path=base.OWNER_PATH, owner_sha256=base.OWNER_SHA,
        sealed_runner_sha256=BASE_SHA, runs=list(RUNS), epochs=list(EPOCHS), order='epoch_major',
        behavior_epochs=[10, 20], batches_per_epoch=287, samples_per_epoch=1147,
        parent_steps=1540, parent_samples_seen=6155, final_steps=5740, final_samples_seen=22940,
        additional_steps_per_lineage=4200, additional_samples_per_lineage=16785,
        stage_timeout_seconds=base.TIMEOUTS, whole_timeout_seconds=21600,
        continuation_authorization=dict(exact_same_view_six_parents_only=True,
            model_optimizer_rng_and_sampler_resume=True, only_train_config_change='max_steps',
            resume_original_corpus=False, original_fresh_fit_proof_unchanged=True,
            original_fresh_only_scope='Historical Sep14 initial-fit admission; this new declaration authorizes only exact same-view continuation.'),
        decision=dict(all_epochs_retained=True, validation_is_reused_development_only=True,
            no_early_stopping_or_selection=True, total_loss_not_comparable_between_arms=True,
            retain_all_absolute_gate_and_subgroup_failures=True), **DENIALS)


def validate_plan(plan, worker_sha, base, observed=None):
    fixed = plan_contract(base)
    variable = set(VIEW_KEYS) | {'declared_at_utc', 'worker_sha256', 'parent_file_sha256',
        'parent_plan_sha256', 'parent_summary_sha256'}
    require(set(plan) == set(fixed) | variable, 'duration plan fields differ')
    require(encoded({k: plan[k] for k in fixed}) == encoded(fixed), 'fixed duration settings differ')
    stamp = datetime.fromisoformat(plan['declared_at_utc'].replace('Z', '+00:00'))
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds() == 0
        and stamp <= (observed or datetime.now(timezone.utc)), 'prospective UTC declaration required')
    require(plan['worker_sha256'] == worker_sha, 'duration worker SHA differs')
    for key in ('worker_sha256', 'parent_plan_sha256', 'parent_summary_sha256'):
        base.hash_value(plan[key])
    require(set(plan['source_sha256']) == set(base.SOURCE_PATHS), 'native source closure differs')
    for value in plan['source_sha256'].values(): base.hash_value(value)
    require(plan['source_sha256'][base.MODEL_PATH] == base.MODEL_SHA, 'STOPMIX config differs')
    require(base.relative(plan['dataset']).startswith('datasets/prepared/'), 'view must be personal prepared data')
    base.relative(plan['view_report_path'])
    for key in ('dataset_manifest_sha256', 'view_report_sha256', 'corpus_fingerprint_sha256'):
        base.hash_value(plan[key])
    require(set(plan['split_contract']) == {'train', 'val'}, 'only TRAIN and VAL permitted')
    for split, count in (('train',1147), ('val',337)):
        item = plan['split_contract'][split]
        require(set(item) == {'sample_count', 'fingerprint_sha256'} and type(item['sample_count']) is int
            and item['sample_count'] == count, 'split denominator changed')
        base.hash_value(item['fingerprint_sha256'])
    require(len(plan['view_file_sha256']) == 8926, 'complete same-view inventory required')
    require(plan['view_file_sha256'].get('dataset.json') == plan['dataset_manifest_sha256'], 'view manifest pins disagree')
    for name, value in plan['view_file_sha256'].items():
        base.relative(name); base.hash_value(value)
        require(not any('test' in p.lower() for p in Path(name).parts), 'TEST path forbidden')
    pins = plan['parent_file_sha256']
    require(len(pins) == 293, 'complete sealed parent inventory required')
    for name, value in pins.items(): base.relative(name); base.hash_value(value)
    require(pins.get('plan.json') == plan['parent_plan_sha256']
        and pins.get('summary.json') == plan['parent_summary_sha256']
        and pins.get('source/' + BASE_NAME) == BASE_SHA, 'parent pins disagree')
    for run in RUNS:
        for name in RESUME_FILES: require(f'{run}/training/{name}' in pins, 'parent resume input missing')
    return plan


def verify_parent(plan, parent, base):
    observed = inventory(parent, plan['parent_file_sha256'])
    original = base.validate_plan(read(parent / 'plan.json'), BASE_SHA)
    require(all(plan[k] == original[k] for k in VIEW_KEYS), 'continuation changed the original data or source contract')
    summary = read(parent / 'summary.json')
    require(summary['status'] == 'COMPLETE_NOT_PROMOTED'
        and summary['runs'] == {r: 'COMPLETE' for r in RUNS}
        and len(summary['stages']) == 24 and summary['source_and_view_postcheck_pass'] is True,
        'all six original parents must be complete')
    require(all(summary[k] is False for k in DENIALS if k in summary), 'parent denial changed')
    require(datetime.fromisoformat(summary['ended_at_utc'].replace('Z', '+00:00'))
        <= datetime.fromisoformat(plan['declared_at_utc'].replace('Z', '+00:00')), 'parent completed after declaration')
    for name, value in original['source_sha256'].items():
        require(observed['source/' + name] == value, 'parent archived source differs')
    require(observed['source/profile_portable_training.py'] == base.OWNER_SHA, 'parent owner archive differs')
    for run in RUNS:
        for stage in base.TIMEOUTS:
            proof = base.verify_stage(original, parent, run, stage)
            receipts = [r for r in summary['stages'] if r['run'] == run and r['stage'] == stage]
            require(len(receipts) == 1 and receipts[0]['status'] == 'COMPLETE'
                and all(receipts[0][k] == v for k, v in proof.items()), 'parent stage binding differs')
        report = read(parent / run / 'training/run.json')
        require(report['state'] == dict(epoch=5, next_batch_index=105, global_step=1540,
            samples_seen=6155, domain_samples_seen={'carla':6155}), 'parent exact resume cursor differs')
        history(parent / run / 'training/metrics.jsonl', 1540, report['last_metrics'])
    return original


def build_plan(parent_root, script=None, now=None):
    # HH_260906 - Declaration reads/hashes existing CPU files only; it never imports Torch or probes a GPU.
    parent = canonical(parent_root); base = load_base(parent)
    original = base.validate_plan(read(parent / 'plan.json'), BASE_SHA)
    pins = inventory(parent)
    plan = plan_contract(base) | {k: original[k] for k in VIEW_KEYS} | dict(
        declared_at_utc=now.isoformat() if isinstance(now,datetime) else (now or utc()),
        worker_sha256=digest(script or __file__), parent_file_sha256=pins,
        parent_plan_sha256=pins['plan.json'], parent_summary_sha256=pins['summary.json'])
    validate_plan(plan, digest(script or __file__), base)
    verify_parent(plan, parent, base)
    return plan


def history(path, steps, last_metrics, prefix=None):
    # HH_260906 - Gradient norm is recorded before clipping; finite nonnegative values need not be <= 5.
    count = 0; previous_bytes = 0; prefix_hash = hashlib.sha256(); latest = None
    with canonical(path).open('rb') as stream:
        for raw in stream:
            require(raw.endswith(b'\n'), 'torn history row')
            row = decode(raw); count += 1
            require(type(row['global_step']) is int and row['global_step'] == count
                and type(row['epoch']) is int and row['epoch'] == (count - 1) // 287, 'history step/epoch differs')
            samples = count * 4 - count // 287
            require(type(row['samples_seen']) is int and row['samples_seen'] == samples
                and row['domain_samples_seen'] == {'carla':samples}
                and row['batch_domain_sample_counts'] == {'carla':3 if count % 287 == 0 else 4}, 'history sample exposures differ')
            for key in ('loss', 'regression_loss', 'candidate_score_loss', 'gradient_norm'):
                require(type(row[key]) in (int, float) and math.isfinite(row[key]) and row[key] >= 0,
                    'nonfinite or negative training metric')
            if prefix and count <= prefix['steps']:
                prefix_hash.update(raw); previous_bytes += len(raw)
            latest = row
    require(count == steps and latest == last_metrics, 'history endpoint/last_metrics differs')
    if prefix:
        require(previous_bytes == prefix['bytes'] and prefix_hash.hexdigest() == prefix['sha256'],
            'inherited history byte prefix changed')
    return dict(row_count=count, samples_seen=count*4-count//287, complete_epochs=count//287,
        history_sha256=digest(path), inherited_prefix_verified=prefix is not None,
        inherited_steps=prefix['steps'] if prefix else 0,
        new_steps=count-(prefix['steps'] if prefix else 0))


def copy_exact(source, target, expected):
    source = canonical(source); target = canonical(target)
    require(source.stat().st_nlink == 1 and digest(source) == expected, 'copy source changed or hardlinked')
    target.parent.mkdir(parents=True, exist_ok=True)
    with source.open('rb') as src, target.open('xb') as dst:
        for block in iter(lambda: src.read(1024*1024), b''): dst.write(block)
        dst.flush(); os.fsync(dst.fileno())
    require(digest(source) == expected and digest(target) == expected
        and target.stat().st_nlink == 1 and not os.path.samestat(source.stat(), target.stat()), 'independent exact copy failed')


def epoch_root(output, epoch):
    require(type(epoch) is int and epoch in EPOCHS, 'undeclared epoch')
    return output / 'epochs' / f'epoch_{epoch:03d}'


def prepare_resume(plan, output, parent, run, epoch):
    require(run in RUNS, 'undeclared lineage')
    source = parent / run / 'training' if epoch == 6 else epoch_root(output, epoch-1) / run / 'training'
    if epoch == 6:
        pins = {name: plan['parent_file_sha256'][f'{run}/training/{name}'] for name in RESUME_FILES}
    else:
        prior = read(source.parent / 'train_completed.json')
        pins = prior['resume_file_sha256']
        require(prior['status'] == 'COMPLETE' and prior['epoch'] == epoch-1 and prior['run'] == run,
            'previous epoch training receipt differs')
        for stage in ('evaluate', 'audit'):
            require(read(source.parent / f'{stage}_completed.json')['status'] == 'COMPLETE', 'previous validation incomplete')
    report = read(source / 'run.json')
    steps = 1540 if epoch == 6 else (epoch-1)*287
    require(report['state']['global_step'] == steps, 'resume source step differs')
    history(source / 'metrics.jsonl', steps, report['last_metrics'])
    target = epoch_root(output, epoch) / run
    target.mkdir(parents=True, exist_ok=False)
    for name in RESUME_FILES: copy_exact(source / name, target / 'training' / name, pins[name])
    receipt = dict(schema='portable_e2e.duration_resume_copy.v1', run=run, epoch=epoch,
        source_training_path=str(source), source_file_sha256=pins, copied_file_sha256=pins,
        source_global_step=steps, target_global_step=epoch*287,
        inherited_history=dict(steps=steps, bytes=(source/'metrics.jsonl').stat().st_size, sha256=pins['metrics.jsonl']),
        independent_byte_exact_copies=True, parent_checkpoint_not_modified=True,
        checkpoint_cursor_note='Native checkpoint may retain completed epoch cursor (epoch-1,287); native trainer validates and normalizes it.')
    write(target / 'resume_source.json', receipt)
    return receipt


def command(base, plan, workspace, repo, output, run, epoch, stage, plan_path, script):
    require(run in RUNS and epoch in EPOCHS, 'undeclared run/epoch')
    root = epoch_root(output, epoch)
    if stage == 'behavior':
        require(epoch in (10,20), 'undeclared behavior checkpoint')
        return [str(workspace/'venvs/py312/bin/python'), str(script), '--plan', str(plan_path),
            '--output-dir', str(output), '--behavior-run', run, '--behavior-epoch', str(epoch)]
    args = base.command(plan, workspace, repo, root, run, stage, plan_path, script)
    if stage == 'train':
        args[args.index('--max-steps')+1] = str(epoch*287)
        args.append('--resume')
    return args


def verify_training(base, plan, output, parent, run, epoch, resume):
    item = epoch_root(output, epoch) / run; report = read(item/'training/run.json')
    original = read(parent/run/'training/run.json')
    immutable = ('trainer_id','created_at_utc','device','hardware','runtime','dataset_fingerprint_sha256',
        'corpus_fingerprint_sha256','training_split','training_episode_ids','dataset_size','sampling_plan',
        'sampling_plan_sha256','model_parameter_count','model_config','loss_config','warning')
    require(all(report[k] == original[k] for k in immutable), 'resumed training ABI/config/data differs')
    expected = dict(original['train_config'], max_steps=epoch*287)
    require(report['train_config'] == expected, 'only max_steps may change during continuation')
    require(report['status'] == 'TRAINING_TARGET_REACHED' and isinstance(report.get('resumed_at_utc'),str),
        'native exact resume did not complete')
    require(report['state'] == dict(epoch=epoch, next_batch_index=0, global_step=epoch*287,
        samples_seen=epoch*1147, domain_samples_seen={'carla':epoch*1147}), 'normalized epoch endpoint differs')
    proof = history(item/'training/metrics.jsonl', epoch*287, report['last_metrics'], resume['inherited_history'])
    proof.update(new_samples_seen=epoch*1147-(6155 if epoch==6 else (epoch-1)*1147))
    return dict(path=(item/'training/run.json').relative_to(output).as_posix(),
        sha256=digest(item/'training/run.json'), checkpoint_sha256=digest(item/'training/checkpoints/latest.pt'),
        resume_file_sha256={n:digest(item/'training'/n) for n in RESUME_FILES}, history=proof)


def verify_archive(plan, output, base):
    for name, expected in plan['source_sha256'].items():
        require(digest(output/'source'/name) == expected, 'archived native source changed')
    for name, expected in ((BASE_NAME,BASE_SHA),('profile_portable_training.py',base.OWNER_SHA),
                           ('run_stopmix_duration_learning.py',plan['worker_sha256'])):
        require(digest(output/'source'/name) == expected, 'archived external source changed')


def run(plan_path, output_dir, behavior_run=None, behavior_epoch=None):
    script = canonical(__file__); plan_path = canonical(plan_path); plan_sha = digest(plan_path)
    workspace = WORKSPACE; repo = workspace/'autoware_e2e'; parent = workspace/PARENT
    base = load_base(parent); plan = validate_plan(read(plan_path),digest(script),base)
    output = canonical(output_dir)
    require(output == workspace/OUTPUT, 'fixed duration output required')
    base.verify_sources(plan,repo); base.verify_view(plan,workspace)
    owner = base.load_module(workspace/base.OWNER_PATH,base.OWNER_SHA,'_duration_gpu_owner')
    owner.check_environment(workspace,repo)
    if behavior_run is not None:
        require(behavior_run in RUNS and behavior_epoch in (10,20)
            and os.environ.get('PORTABLE_DURATION_OWNER_PID') == str(os.getppid()), 'owned behavior child required')
        root = epoch_root(output,behavior_epoch); item = root/behavior_run
        receipt = read(item/'behavior_started.json')
        require(receipt['status']=='RUNNING' and receipt['epoch']==behavior_epoch and receipt['run']==behavior_run
            and receipt['owner_pid']==os.getppid() and receipt['plan_sha256']==plan_sha,
            'behavior ownership receipt differs')
        verify_archive(plan,output,base)
        require(digest(item/'training/checkpoints/latest.pt')==receipt['input_checkpoint_sha256'], 'behavior input changed')
        base.behavior(plan,root,behavior_run,repo)
        require(digest(item/'training/checkpoints/latest.pt')==receipt['input_checkpoint_sha256'], 'behavior input changed')
        base.verify_sources(plan,repo); base.verify_view(plan,workspace); verify_archive(plan,output,base)
        require(digest(plan_path)==plan_sha and digest(script)==plan['worker_sha256'], 'behavior source changed')
        return
    require(behavior_epoch is None and not output.exists(), 'fresh campaign output required; automatic rerun forbidden')
    verify_parent(plan,parent,base)
    pipeline = base.load_module(repo/'scripts/e2e/run_portable_training_campaign.py',
        plan['source_sha256']['scripts/e2e/run_portable_training_campaign.py'],'_duration_stage_owner')
    with owner.gpu_lease(workspace):
        idle = owner.gpu_idle(repo)
        output.mkdir(parents=True,exist_ok=False); write(output/'plan.json',plan)
        for name, expected in plan['source_sha256'].items(): copy_exact(repo/name,output/'source'/name,expected)
        for source,name,expected in ((script,'run_stopmix_duration_learning.py',plan['worker_sha256']),
            (parent/'source'/BASE_NAME,BASE_NAME,BASE_SHA),
            (workspace/base.OWNER_PATH,'profile_portable_training.py',base.OWNER_SHA)):
            copy_exact(source,output/'source'/name,expected)
        # HH_260906 - Parent reports/history are portable evidence; parent weights remain immutable in their original run.
        for run_name in RUNS:
            for name in ('training/run.json','training/metrics.jsonl','evaluation/metrics.json','gate_v8.json','behavior/summary.json'):
                copy_exact(parent/run_name/name,output/'parents'/run_name/name,
                    plan['parent_file_sha256'][f'{run_name}/{name}'])
        write(output/'parent_evidence.json',dict(parent_root=PARENT,
            parent_plan_sha256=plan['parent_plan_sha256'],parent_summary_sha256=plan['parent_summary_sha256'],
            checkpoint_sha256={r:plan['parent_file_sha256'][f'{r}/training/checkpoints/latest.pt'] for r in RUNS}))
        status = dict(schema='portable_e2e.stopmix_duration_learning.v1',status='RUNNING',started_at_utc=utc(),
            plan_sha256=plan_sha,pre_cuda_gpu=idle,stages=[],epochs={str(e):{r:'NOT_RUN' for r in RUNS} for e in EPOCHS},**DENIALS)
        write(output/'started.json',status); started=time.monotonic()
        try:
            with owner.bounded_signals(plan['whole_timeout_seconds']):
                for epoch in EPOCHS:
                    root=epoch_root(output,epoch)
                    copy_exact(output/'source'/base.MODEL_PATH,root/'source'/base.MODEL_PATH,base.MODEL_SHA)
                    base.verify_view(plan,workspace)
                    for run_name in RUNS:
                        resume=prepare_resume(plan,output,parent,run_name,epoch)
                        item=root/run_name; status['epochs'][str(epoch)][run_name]='RUNNING'
                        checkpoint_sha=resume['source_file_sha256']['checkpoints/latest.pt']
                        for stage in ('train','evaluate','audit',*(['behavior'] if epoch in (10,20) else [])):
                            timeout=plan['stage_timeout_seconds'][stage]
                            require(time.monotonic()-started+timeout+30<plan['whole_timeout_seconds'],'insufficient bounded stage/cleanup time')
                            base.verify_sources(plan,repo); verify_archive(plan,output,base)
                            require(digest(script)==plan['worker_sha256'] and digest(plan_path)==plan_sha
                                and digest(workspace/base.OWNER_PATH)==base.OWNER_SHA,'execution source changed')
                            require(digest(item/'training/checkpoints/latest.pt')==checkpoint_sha,'checkpoint changed before stage')
                            stage_idle=owner.gpu_idle(repo)
                            args=command(base,plan,workspace,repo,output,run_name,epoch,stage,plan_path,script)
                            receipt=dict(epoch=epoch,run=run_name,stage=stage,command=args,timeout_seconds=timeout,
                                started_at_utc=utc(),status='RUNNING',owner_pid=os.getpid(),plan_sha256=plan_sha,
                                input_checkpoint_sha256=checkpoint_sha,pre_cuda_gpu=stage_idle)
                            write(item/f'{stage}_started.json',receipt)
                            progress(output,dict(status='RUNNING',epoch=epoch,run=run_name,stage=stage,
                                completed_stages=len(status['stages']),updated_at_utc=utc()))
                            print(json.dumps(dict(event='stage_started',epoch=epoch,run=run_name,stage=stage)),flush=True)
                            with (item/f'{stage}.log').open('xb') as log:
                                rc=pipeline.run_owned_stage(args,repo,dict(os.environ,PORTABLE_DURATION_OWNER_PID=str(os.getpid())),log,timeout=timeout)
                            require(rc==0,f'{epoch} {run_name} {stage} exited {rc}')
                            if stage=='train':
                                proof=verify_training(base,plan,output,parent,run_name,epoch,resume)
                            else:
                                require(digest(item/'training/checkpoints/latest.pt')==checkpoint_sha,'checkpoint changed during stage')
                                proof=base.verify_stage(plan,root,run_name,stage)
                                proof['path']=(root/proof['path']).relative_to(output).as_posix()
                            receipt.update(status='COMPLETE',ended_at_utc=utc(),**proof)
                            checkpoint_sha=receipt['checkpoint_sha256']
                            write(item/f'{stage}_completed.json',receipt); status['stages'].append(receipt)
                            print(json.dumps(dict(event='stage_complete',epoch=epoch,run=run_name,stage=stage,
                                checkpoint_sha256=checkpoint_sha)),flush=True)
                        status['epochs'][str(epoch)][run_name]='COMPLETE'
                    base.verify_view(plan,workspace)
                base.verify_sources(plan,repo); verify_archive(plan,output,base); verify_parent(plan,parent,base)
                for receipt in status['stages']:
                    item=epoch_root(output,receipt['epoch'])/receipt['run']
                    require(digest(output/receipt['path'])==receipt['sha256']
                        and digest(item/'training/checkpoints/latest.pt')==receipt['checkpoint_sha256'],'completed evidence changed')
                    if receipt['stage']=='train':
                        for n,h in receipt['resume_file_sha256'].items(): require(digest(item/'training'/n)==h,'training evidence changed')
                    elif receipt['stage']=='behavior': base.verify_stage(plan,epoch_root(output,receipt['epoch']),receipt['run'],'behavior')
                require(digest(script)==plan['worker_sha256'] and digest(plan_path)==plan_sha,'final source changed')
                require(len(status['stages'])==282 and all(v=='COMPLETE' for cases in status['epochs'].values() for v in cases.values()),
                    'duration campaign stage denominator incomplete')
                for run_name in RUNS:
                    for name in ('training/run.json','training/metrics.jsonl','evaluation/metrics.json','gate_v8.json','behavior/summary.json'):
                        require(digest(output/'parents'/run_name/name)==plan['parent_file_sha256'][f'{run_name}/{name}'],
                            'archived parent evidence changed')
                status.update(status='COMPLETE_NOT_PROMOTED',ended_at_utc=utc(),final_gpu=owner.gpu_idle(repo),
                    source_view_parent_postcheck_pass=True,total_new_optimizer_steps=25200,total_new_sample_exposures=100710,
                    stage_counts={s:sum(r['stage']==s for r in status['stages']) for s in base.TIMEOUTS})
                write(output/'summary.json',status)
                progress(output,dict(status=status['status'],completed_stages=len(status['stages']),updated_at_utc=utc()))
        except BaseException as error:
            for cases in status['epochs'].values():
                for name in cases:
                    if cases[name]=='RUNNING': cases[name]='FAIL'
            status.update(status='FAIL',error_type=type(error).__name__,error=str(error),ended_at_utc=utc())
            with owner.finalization_signals():
                write(output/'failed.json',status)
                progress(output,dict(status='FAIL',error=str(error),completed_stages=len(status['stages']),updated_at_utc=utc()))
            raise


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--plan',type=Path); parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--declare-from-parent',type=Path); parser.add_argument('--declaration-output',type=Path)
    parser.add_argument('--behavior-run',choices=RUNS,help=argparse.SUPPRESS)
    parser.add_argument('--behavior-epoch',type=int,choices=(10,20),help=argparse.SUPPRESS)
    args=parser.parse_args(argv)
    if args.declare_from_parent is not None:
        require(args.declaration_output is not None and all(x is None for x in
            (args.plan,args.output_dir,args.behavior_run,args.behavior_epoch)), 'declaration arguments differ')
        plan=build_plan(args.declare_from_parent)
        target=canonical(args.declaration_output)
        require(not target.is_relative_to(canonical(args.declare_from_parent)), 'declaration must not alter parent')
        write(target,plan)
        print(json.dumps(dict(plan=str(target),sha256=digest(target))),flush=True)
    else:
        require(args.plan is not None and args.output_dir is not None and args.declaration_output is None,'plan and output required')
        run(args.plan,args.output_dir,args.behavior_run,args.behavior_epoch)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
