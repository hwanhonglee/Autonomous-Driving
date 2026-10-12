#!/usr/bin/env python3
"""HH_260906 - Continue all six frozen STOPMIX lineages to ten epochs in a new namespace; never retry or promote."""

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import sys

WORKSPACE = Path.home() / 'personal/hwanhong/portable_e2e'
SOURCE_COMMIT = 'b478f02e42b94bf04bffec5c8170e05edc33b0f8'
OWNER_SHA = 'db7fd6764cfc28162400f6e86dc38fd5c99bfacb57b4c4bf0cd28eeebbf074a4'
PARENT_ID = 'hh260909-stopmix-development-ab-3seeds-v1'
CAMPAIGN_ID = 'hh260909-stopmix-duration-10epochs-v1'
PARENT_PLAN_SHA = 'f06c68ce47aaee6941a3ca42559460ac5837b53f669bf195fa15b5d5e80a05d9'
PARENT_STATUS_SHA = '9f40946bde5395c61e68a5ac5fa476a210569a7fe8f6f6d58d4588037e4ff2bc'
PARENT_BEHAVIOR_STATUS_SHA = '92f46b8030900f64f81a646f5efdef02e71c8bc62989e755ff8e69396a599871'
PARENT_BEHAVIOR_MANIFEST_SHA = 'bd8a698f03fb953d7ebe152fb7283d7ba0580a2d1805d2ae02c599b602c1fe0e'
GPU_UUID = 'GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5'
DEADLINE = '2026-09-09T01:00:00Z'
ARMS = ('A_physical_drive', 'B_drive_stop_mix')
SEEDS = (20260903, 20260904, 20260905)
RUNS = tuple(f'seed_{seed}/{arm}' for seed in SEEDS for arm in ARMS)
STAGES = ('train', 'evaluate', 'audit', 'behavior')
TIMEOUTS = {'train': 450, 'evaluate': 120, 'audit': 120, 'behavior': 180}
ORDER = tuple((run, stage) for run in RUNS for stage in STAGES)
LOSS = {'candidate_score_weight': .1, 'final_displacement_weight': .5, 'kinematic_speed_weight': .05,
        'speed_weight': .2, 'xy_weight': 1., 'yaw_weight': .1}
PARENT_FILES = {
 'seed_20260903/A_physical_drive': {'run.json':'8b6fd178ba890087ceb5221af804f48f944640b0f78f155662cf2b35576bb354',
  'metrics.jsonl':'f35cdbb0c6d577a89e3660294aab867c1ad86310e81746d0af87d88f721bb7a0',
  'checkpoints/latest.pt':'3fbde7873de6f5af72b9e301916b36006b0ae84478e906fcfa1f51c065739b9d'},
 'seed_20260903/B_drive_stop_mix': {'run.json':'c40bdf1879ee1c796e16bf623ce0939a9a616d714d4ffa68f67aaad99ef2ce38',
  'metrics.jsonl':'a008586ca82264148c2eee5a4589ddb8c4abf7a716ec1966c3152b7bec4eedb4',
  'checkpoints/latest.pt':'9b949b17b3708bed5ba25eb5721187a56f4f77c11c8461e685cbbd995259f8bf'},
 'seed_20260904/A_physical_drive': {'run.json':'697c4f94e1efd52fb4a7bb68c9926b3b49becbccc7145cb8b25788512b62b050',
  'metrics.jsonl':'be1c37b4c27f0f9883b7f746b1d44a0cc70a8821e012c24260e831adcfcd69ea',
  'checkpoints/latest.pt':'343b2cf6b256f57c37f7d21fe4297775fbc921ad8350fc946510df2505c33d81'},
 'seed_20260904/B_drive_stop_mix': {'run.json':'f32ada2843717469a7e240ec2f2b0f48a081944948196b8ce7b91b0f220a1db3',
  'metrics.jsonl':'83144380d7fd5aab6cf81159212c9978565da94b3566f766c167eebcf6b5347e',
  'checkpoints/latest.pt':'2789a33c313923ee6084ddaa6dc1a3937ebedd22d5e52db360c0e45fc841e851'},
 'seed_20260905/A_physical_drive': {'run.json':'c6735f85e8aa18154ecac81f70034a44caf46b015cf86bc8383541a9b92cf7f1',
  'metrics.jsonl':'22c67848d733d622bf58a30c2228b044e04579f847e991df6ce02ace41f5dacf',
  'checkpoints/latest.pt':'fe7ada85cefc9d6eff07bb7ba42855f675aa5c92fe0156508759e20924b1863d'},
 'seed_20260905/B_drive_stop_mix': {'run.json':'a424eb3af486d62ac73f63dd51ea15a64cc6e6198a1bc8e38f5c6e0cdb2161a6',
  'metrics.jsonl':'60ccde49e10424fc3406bd13eeb4d44d6e5770add1bc1cd197c30411c97a0bfe',
  'checkpoints/latest.pt':'ce6ffa0bbd80db39d0c9952d1a1e8d67543ff1c69ccc9a7e76d110ffefd5d9df'},
}


def require(value, message):
    if not value: raise RuntimeError(message)


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(path):
    require(path.is_file() and all(not p.is_symlink() for p in (path, *path.parents)), 'regular nonsymlink input required')
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''): value.update(block)
    return value.hexdigest()


def read(path):
    digest(path)
    return json.loads(path.read_text(), parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite JSON')))


def now():
    return datetime.now(timezone.utc)


def timestamp(value):
    require(isinstance(value, str), 'UTC timestamp required')
    result = datetime.fromisoformat(value.replace('Z', '+00:00'))
    require(result.tzinfo is not None and result.utcoffset().total_seconds() == 0, 'UTC timestamp required')
    return result


def plan_contract(worker_sha):
    require(isinstance(worker_sha, str) and re.fullmatch('[0-9a-f]{64}', worker_sha), 'worker SHA required')
    return {'schema':'portable_e2e.stopmix_duration_campaign.v1', 'campaign_id':CAMPAIGN_ID,
        'source_commit':SOURCE_COMMIT, 'worker_sha256':worker_sha, 'owner_sha256':OWNER_SHA,
        'parent_campaign_id':PARENT_ID, 'parent_plan_sha256':PARENT_PLAN_SHA,
        'parent_status_sha256':PARENT_STATUS_SHA, 'parent_training_file_sha256':PARENT_FILES,
        'parent_behavior_status_sha256':PARENT_BEHAVIOR_STATUS_SHA,
        'parent_behavior_manifest_sha256':PARENT_BEHAVIOR_MANIFEST_SHA,
        'gpu_uuid':GPU_UUID, 'finish_before_utc':DEADLINE, 'runs':list(RUNS),
        'stage_order':list(STAGES), 'expected_stage_count':24, 'stage_timeout_seconds':TIMEOUTS,
        'cleanup_reserve_seconds':300, 'integrity_overhead_reserve_seconds':600,
        'initial_full_budget_seconds':6120, 'parent_steps':1540, 'parent_samples_seen':6155,
        'total_steps':2870, 'total_samples_seen':11470, 'additional_steps':1330,
        'additional_samples_seen':5315, 'total_complete_epochs':10, 'train_samples':1147,
        'validation_samples':337, 'model_config_paths':dict(zip(ARMS, (
            'portable_e2e/config/perspective_trajectory_physical_v1.model.json',
            'portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json'))),
        'unchanged_loss_config':LOSS, 'checkpoint_interval':154, 'num_workers':0,
        'behavior_render_indices':[0,31,61,92,122,153,183,214,244,275,305,336],
        'only_train_config_change':'max_steps:1540->2870', 'automatic_retry':False,
        'best_checkpoint_selection':False, 'automatic_promotion':False, 'training_data_approved':False,
        'test_nn_inference_or_training_or_selection':False,
        'test_integrity_scope':'The unchanged full-corpus validator may read test contract/JSON/JPEG bytes only for integrity.',
        'interpretation':'Six continuation lineages, not six fresh initializations or new independent episodes. Compare both fixed endpoints; no repeat-until-pass.'}


def validate_plan(plan, worker_sha, observed=None):
    observed = now() if observed is None else observed
    require(isinstance(plan, dict) and 'declared_at_utc' in plan, 'prospective plan required')
    fixed = {k:v for k,v in plan.items() if k != 'declared_at_utc'}
    require(encoded(fixed) == encoded(plan_contract(worker_sha)), 'unapproved duration campaign settings')
    require(timestamp(plan['declared_at_utc']) <= observed < timestamp(DEADLINE), 'plan is future-dated or deadline passed')


def remaining_budget(index, observed=None):
    require(type(index) is int and 0 <= index <= len(ORDER), 'invalid remaining stage index')
    observed = now() if observed is None else observed
    seconds = sum(TIMEOUTS[stage] for _, stage in ORDER[index:]) + 300 + 600
    require((timestamp(DEADLINE) - observed).total_seconds() >= seconds, 'insufficient remaining deadline budget')
    return seconds


def load_owner():
    # HH_260906 - Reuse the executed, source-pinned stdlib ownership helper rather than a mutable repository import.
    path = WORKSPACE / 'runs/diagnostics/stopmix_behavior_v1/provenance/stdin_worker.py'
    require(digest(path) == OWNER_SHA, 'historical ownership helper changed')
    spec = importlib.util.spec_from_file_location('frozen_stopmix_duration_parent_owner', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    require(module.WORKSPACE == WORKSPACE and module.SOURCE_COMMIT == SOURCE_COMMIT
            and module.GPU_UUID == GPU_UUID and tuple(r for r,_ in module.RUNS) == RUNS, 'owner identity differs')
    return module


def train_config(seed, steps):
    return {'batch_size':4, 'checkpoint_interval':154, 'domain_ratios':[], 'learning_rate':.0001,
        'max_steps':steps, 'maximum_gradient_norm':5., 'num_workers':0,
        'sampling_policy':'uniform_without_replacement', 'seed':seed, 'verify_image_sha256':True, 'weight_decay':.0001}


def training_state(step, *, final_report=False):
    epoch, batch = divmod(step, 287)
    if not final_report and batch == 0 and step:
        epoch -= 1; batch = 287
    seen = epoch * 1147 + min(batch * 4, 1147)
    return {'epoch':epoch, 'next_batch_index':batch, 'global_step':step,
            'samples_seen':seen, 'domain_samples_seen':{'carla':seen}}


def parent_snapshot(owner, runner, repo, parent):
    require(digest(parent/'plan.json') == PARENT_PLAN_SHA and digest(parent/'status.json') == PARENT_STATUS_SHA,
            'fixed parent campaign identity changed')
    result = owner.normal_snapshot(runner, repo, parent)
    behavior = parent_behavior_proof(owner, result)
    result['history_sha256'] = {}
    for run in RUNS:
        item = parent/run/'training'
        for name, expected in PARENT_FILES[run].items():
            require(digest(item/name) == expected, 'parent training bytes changed')
        report = read(item/'run.json')
        seed = int(run.split('/')[0].removeprefix('seed_'))
        require(encoded(report['train_config']) == encoded(train_config(seed,1540))
                and encoded(report['loss_config']) == encoded(LOSS)
                and report['state'] == training_state(1540), 'parent immutable training settings differ')
        result['history_sha256'][run] = digest(item/'metrics.jsonl')
    result['behavior_completion'] = behavior
    return result


def parent_behavior_proof(owner, normal):
    # HH_260906 - All eighteen normal stages AND six existing behavior passes must finish before continuation.
    root = WORKSPACE/'runs/diagnostics/stopmix_behavior_v1'
    require(digest(root/'workflow_status.json') == PARENT_BEHAVIOR_STATUS_SHA
            and digest(root/'WORKFLOW_SHA256SUMS') == PARENT_BEHAVIOR_MANIFEST_SHA,
            'parent behavior workflow identity changed')
    state = read(root/'workflow_status.json')
    require(state.get('status') == 'COMPLETE_NOT_PROMOTED' and state.get('completed_behavior_count') == 6
            and state.get('source_commit') == SOURCE_COMMIT and state.get('worker_source_sha256') == OWNER_SHA
            and state.get('source_and_normal_inputs_unchanged') is True
            and state.get('vehicle_control_approved') is False
            and state.get('before') == state.get('after') == normal, 'all six parent behavior passes are not complete')
    stages = state.get('stages',[])
    require([record.get('run') for record in stages] == list(RUNS)
            and all(record.get('status') == 'COMPLETE' and type(record.get('returncode')) is int
                    and record['returncode'] == 0 for record in stages), 'parent behavior order or exit differs')
    proofs = {}
    for record,(run,model_id) in zip(stages,owner.RUNS):
        proof = owner.verify_result(root/run,model_id,PARENT_FILES[run]['checkpoints/latest.pt'],normal['source_sha256'])
        require(proof == record.get('report'), 'parent behavior payload changed')
        proofs[run] = proof
    require(digest(root/'workflow_status.json') == PARENT_BEHAVIOR_STATUS_SHA, 'parent behavior status changed during check')
    return {'status_sha256':PARENT_BEHAVIOR_STATUS_SHA,'checksums_sha256':PARENT_BEHAVIOR_MANIFEST_SHA,
            'finished_at_utc':state['finished_at_utc'],'reports':proofs}


def copy_training(parent, output, run):
    source, destination = parent/run/'training', output/run/'training'
    require(all(not p.is_symlink() for p in (destination,*destination.parents))
            and not output.resolve().is_relative_to(parent.resolve())
            and not parent.resolve().is_relative_to(output.resolve()), 'parent/output alias forbidden')
    require(not destination.exists(), 'new training namespace required')
    destination.mkdir(parents=True)
    for name, expected in PARENT_FILES[run].items():
        original, copied = source/name, destination/name
        require(digest(original) == expected, 'parent changed before copy')
        copied.parent.mkdir(parents=True, exist_ok=True)
        with original.open('rb') as src, copied.open('xb') as dst:
            for block in iter(lambda: src.read(8 * 1024 * 1024), b''): dst.write(block)
        require(digest(copied) == expected == digest(original) and copied.stat().st_nlink == 1
                and (copied.stat().st_dev,copied.stat().st_ino) != (original.stat().st_dev,original.stat().st_ino),
                'resume copy is not independent exact bytes')
    return {name:digest(destination/name) for name in PARENT_FILES[run]}


def history_proof(parent_training, training, steps):
    original = (parent_training/'metrics.jsonl').read_bytes()
    value = (training/'metrics.jsonl').read_bytes()
    require(original.endswith(b'\n') and value.startswith(original), 'training history original prefix changed')
    rows = [json.loads(line) for line in value.splitlines()]
    require(len(rows) == steps, 'training history step count differs')
    for index, row in enumerate(rows,1):
        expected = training_state(index)
        require(all(row.get(key) == expected[key] for key in ('epoch','global_step','samples_seen','domain_samples_seen'))
                and row.get('batch_domain_sample_counts') == {'carla':3 if index % 287 == 0 else 4},
                'history does not match the fixed sampling cursor')
        require(all(type(row.get(key)) in (int,float) and math.isfinite(row[key]) for key in
            ('loss','regression_loss','candidate_score_loss','selected_ade_m','selected_fde_m','selected_speed_mae_mps','gradient_norm')),
            'history contains missing/nonfinite metrics')
    return {'sha256':digest(training/'metrics.jsonl'), 'parent_prefix_sha256':hashlib.sha256(original).hexdigest(),
            'parent_prefix_bytes':len(original), 'rows':steps}, rows[-1]


def commands(owner, runner, repo, parent, output):
    base = list(runner.commands(read(parent/'plan.json'), parent, repo))
    result = []
    for run, model_id in owner.RUNS:
        for stage in STAGES:
            if stage == 'behavior':
                result.append((run,stage,model_id,None)); continue
            matches = [command for item,_,command,name in base if str(item.relative_to(parent)) == run and name == stage]
            require(len(matches) == 1, 'parent stage command is not unique')
            command = [str(output)+value[len(str(parent)):] if value.startswith(str(parent)+'/') else value for value in matches[0]]
            if stage == 'train':
                index = command.index('--max-steps')+1
                require(command[index] == '1540' and '--resume' not in command, 'unexpected parent train command')
                command[index] = '2870'; command.append('--resume')
            result.append((run,stage,model_id,command))
    require([(run,stage)for run,stage,_,_ in result] == list(ORDER), 'duration stage order differs')
    return result


def verify_report(owner, parent, output, run, stage, initial):
    item, training = output/run, output/run/'training'
    checkpoint = training/'checkpoints/latest.pt'
    checkpoint_sha = digest(checkpoint)
    baseline = read(parent/run/'training/run.json')
    model_id = baseline['model_config']['model_id']
    if stage == 'behavior':
        return owner.verify_result(item/'behavior', model_id, checkpoint_sha, initial['source_sha256'])
    name = {'train':'training/run.json','evaluate':'evaluation/metrics.json','audit':'gate_v8.json'}[stage]
    report = read(item/name)
    require(report.get('corpus_fingerprint_sha256') == baseline['corpus_fingerprint_sha256'], 'continued corpus changed')
    if stage == 'train':
        require(report.get('status') == 'TRAINING_TARGET_REACHED'
                and report.get('state') == training_state(2870,final_report=True), 'continued training did not complete ten epochs')
        seed = int(run.split('/')[0].removeprefix('seed_'))
        require(encoded(report.get('train_config')) == encoded(train_config(seed,2870)), 'immutable continued train setting changed')
        for key in ('trainer_id','created_at_utc','dataset_size','dataset_fingerprint_sha256','training_split',
                    'training_episode_ids','model_config','model_parameter_count','loss_config','runtime','hardware',
                    'sampling_plan','sampling_plan_sha256','device'):
            require(report.get(key) == baseline.get(key), 'continued training provenance differs: '+key)
        require('resumed_at_utc' in report, 'continuation did not record resume')
        history, last = history_proof(parent/run/'training', training,2870)
        require(report.get('last_metrics') == last, 'last metrics do not match complete appended history')
        return {'path':name,'sha256':digest(item/name),'checkpoint_sha256':checkpoint_sha,'history':history,
                'checkpoint_cursor_validation':'Owner does not load tensors; existing secure evaluation checks the valid saved 9/287 cursor.'}
    old = read(parent/run/name)
    require(report.get('status') == old['status'] and report.get('evaluation_split') == 'val'
            and report.get('checkpoint_sha256') == checkpoint_sha
            and report.get('vehicle_control_approved') is False, 'continued validation identity differs')
    for key in ('model_config_sha256','model_parameter_count','dataset_fingerprint_sha256',
                'training_dataset_fingerprint_sha256','training_episode_ids','evaluation_episode_ids'):
        require(report.get(key) == old.get(key), 'validation provenance differs: '+key)
    require(not any(key in report for key in ('auxiliary_loss_metrics','auxiliary_loss_metric_counts','loss_config')),
            'unapproved auxiliary objective in validation')
    count = report.get('sample_count') if stage == 'evaluate' else report.get('geometry',{}).get('sample_count')
    require(count == 337, 'full validation337 required')
    if stage == 'audit':
        require(report.get('gate') == old.get('gate') and report['gate']['threshold_overrides'] is False,
                'unchanged runtime gate required')
    return {'path':name,'sha256':digest(item/name),'checkpoint_sha256':checkpoint_sha}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--expected-plan-sha256',required=True)
    args = parser.parse_args(argv)
    source, source_sha = globals().get('VERIFIED_STDIN_WORKER_SOURCE'), globals().get('VERIFIED_STDIN_WORKER_SHA256')
    require(isinstance(source,bytes) and hashlib.sha256(source).hexdigest() == source_sha, 'verified stdin bootstrap required')
    require(Path(sys.prefix).resolve() == WORKSPACE/'venvs/py312'
            and os.environ.get('CUDA_VISIBLE_DEVICES') == GPU_UUID and os.environ.get('PYTHONNOUSERSITE') == '1'
            and not any(os.environ.get(k) for k in ('PYTHONHOME','PYTHONSTARTUP','PYTHONUSERBASE','PYTHONPATH')),
            'clean personal py312 environment and physical GPU0 UUID required')
    require(args.plan.is_absolute() and args.plan.is_relative_to(WORKSPACE/'runs/plans')
            and digest(args.plan) == args.expected_plan_sha256, 'pinned personal prospective plan required')
    plan = read(args.plan); validate_plan(plan,source_sha)
    repo, parent = WORKSPACE/'autoware_e2e', WORKSPACE/'runs/campaigns'/PARENT_ID
    output = WORKSPACE/'runs/campaigns'/CAMPAIGN_ID
    require(not output.exists() and all(not p.is_symlink() for p in (output,*output.parents)), 'fresh nonalias output required')
    remaining_budget(0); owner = load_owner(); runner = owner.load_runner(repo)
    lease_path = WORKSPACE/'runs/campaigns/.gpu0_training.lock'
    digest(lease_path)
    with lease_path.open('r+') as lease, runner.termination_guard():
        fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
        runner.assert_gpu_idle(repo)
        initial = parent_snapshot(owner,runner,repo,parent)
        require(all(timestamp(read(parent/run/'training/run.json')['completed_at_utc']) < timestamp(plan['declared_at_utc'])
                    for run in RUNS) and timestamp(initial['behavior_completion']['finished_at_utc']) < timestamp(plan['declared_at_utc']),
                    'plan was not declared after all parent fits and behavior passes')
        remaining_budget(0)
        output.mkdir(parents=True,exist_ok=False); (output/'provenance').mkdir()
        with (output/'provenance/stdin_worker.py').open('xb') as stream: stream.write(source)
        with (output/'plan.json').open('xb') as stream: stream.write(args.plan.read_bytes())
        state = {'schema':'portable_e2e.stopmix_duration_workflow.v1','status':'RUNNING','started_at_utc':now().isoformat(),
            'source_commit':SOURCE_COMMIT,'worker_sha256':source_sha,'plan_sha256':args.expected_plan_sha256,
            'plan':plan,'before':initial,'stages':[],'copies':{},'automatic_promotion':False,'training_data_approved':False,
            'vehicle_control_approved':False,'original_parent_modified':False,'additional_fresh_initializations':0}
        def save():
            temporary=output/'status.json.new'
            temporary.write_text(json.dumps(state,indent=2,allow_nan=False)+'\n'); temporary.replace(output/'status.json')
        def intact():
            require(digest(args.plan) == args.expected_plan_sha256 == digest(output/'plan.json'), 'prospective plan changed')
            require(digest(output/'provenance/stdin_worker.py') == source_sha, 'duration worker archive changed')
            require(digest(WORKSPACE/'runs/diagnostics/stopmix_behavior_v1/provenance/stdin_worker.py') == OWNER_SHA,
                    'ownership source changed')
            require(parent_snapshot(owner,runner,repo,parent) == initial, 'source or parent inputs changed')
        save()
        try:
            for index,(run,stage,model_id,command) in enumerate(commands(owner,runner,repo,parent,output)):
                remaining_budget(index); intact(); runner.assert_gpu_idle(repo)
                if stage == 'train':
                    state['copies'][run] = copy_training(parent,output,run); save()
                checkpoint=output/run/'training/checkpoints/latest.pt'
                if stage == 'audit': command += ['--checkpoint-sha256',digest(checkpoint)]
                if stage == 'behavior':
                    command=owner.command_for(run,model_id,digest(checkpoint),output/'behavior_outputs',
                        WORKSPACE/read(parent/'plan.json')['dataset'],output)
                    command[command.index('--output-dir')+1]=str(output/run/'behavior')
                record={'run':run,'stage':stage,'status':'RUNNING','started_at_utc':now().isoformat(),
                        'command':command,'timeout_seconds':TIMEOUTS[stage]}
                state['stages'].append(record);save();remaining_budget(index)
                with (output/(run.replace('/','_')+'_'+stage+'.log')).open('x') as log:
                    code=runner.run_owned_stage(command,repo,runner.stage_environment(),log,timeout=TIMEOUTS[stage])
                record['returncode']=code
                runner.assert_gpu_idle(repo);intact()
                require(code == 0,'owned stage failed; no automatic retry')
                record['report']=verify_report(owner,parent,output,run,stage,initial)
                record.update(status='COMPLETE',finished_at_utc=now().isoformat());save()
            require([(r['run'],r['stage']) for r in state['stages']] == list(ORDER)
                    and all(r['status']=='COMPLETE' for r in state['stages']), 'full24 stage completion required')
            state['status']='COMPLETE_NOT_PROMOTED'
        except BaseException as error:
            if state['stages'] and state['stages'][-1]['status']=='RUNNING':state['stages'][-1].update(status='FAILED',finished_at_utc=now().isoformat())
            state.update(status='STOPPED_FAILURE_NO_PROMOTION',error=f'{type(error).__name__}: {error}')
        finally:
            with owner.ignore_cleanup_signals():
                try:
                    remaining_budget(24); runner.assert_gpu_idle(repo); intact()
                    for record in state['stages']:
                        if record['status']=='COMPLETE':
                            require(verify_report(owner,parent,output,record['run'],record['stage'],initial)==record['report'],
                                    'completed output changed during later stages')
                    state['parent_source_and_output_postcheck_pass']=True
                except BaseException as error:
                    state.update(status='STOPPED_FAILURE_NO_PROMOTION',final_integrity_error=f'{type(error).__name__}: {error}')
                state['completed_stage_count']=sum(r['status']=='COMPLETE'for r in state['stages'])
                state['finished_at_utc']=now().isoformat();save()
                paths=sorted(p for p in output.rglob('*') if p.is_file())
                with (output/'WORKFLOW_SHA256SUMS').open('x') as stream:
                    for path in paths:stream.write(f'{digest(path)}  {path.relative_to(output).as_posix()}\n')
                print(json.dumps({'status':state['status'],'completed_stage_count':state['completed_stage_count']}),flush=True)
        return 0 if state['status']=='COMPLETE_NOT_PROMOTED' else 1


if __name__=='__main__':
    raise SystemExit(main())
