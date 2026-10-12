#!/usr/bin/env python3
"""HH_260906 - Audit the fixed six-lineage ten-epoch continuation using saved evidence only."""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
import re
from types import ModuleType

from portable_e2e.contract import ContractError
from scripts.e2e import summarize_portable_stopmix as old

ROOT = Path(__file__).resolve().parents[2]
WORKER_SHA = 'e51d2c4e02ba15a24a376944c08a387d616b8d749585562732ee483b913958e3'
PLAN_SHA = 'bcdf31f09838750cd24fcabf063a9baeac8fb317ed045e6a41d734fff2e6233b'
READER_SHA = '93966ee7bdc738b3ab2d1065014af10c529519d90626e935ee3e3facf3ff6378'
SOURCE = 'b478f02e42b94bf04bffec5c8170e05edc33b0f8'
SCHEMA = 'portable_e2e.stopmix_duration_summary.v1'
require, read, digest, canonical = old.require, old.read, old.checked_hash, old.canonical
STAGE_FILES = dict(old.expansion.STAGE_FILES, behavior='behavior/summary.json')


def frozen_worker(root):
    # HH_260906 - Import only the authenticated stdlib owner module; no entry point, subprocess, GPU or checkpoint load is called.
    path = root / 'provenance/stdin_worker.py'
    require(digest(path) == WORKER_SHA, 'duration worker archive differs from reviewed execution')
    # HH_260906 - Compile authenticated bytes in memory so the read-only input archive never acquires a __pycache__ file.
    payload = path.read_bytes()
    require(old.base._sha(payload) == WORKER_SHA, 'duration worker changed before in-memory import')
    worker = ModuleType('hh_frozen_duration_evidence_contract')
    worker.__file__ = str(path)
    exec(compile(payload,str(path),'exec'),worker.__dict__)
    require(worker.SOURCE_COMMIT == SOURCE, 'frozen training source differs')
    return worker


def tree_pins(root):
    require(root.is_dir() and all(not p.is_symlink() for p in (root, *root.parents)), 'regular input root required')
    return {p.relative_to(root).as_posix(): digest(p) for p in sorted(root.rglob('*')) if p.is_file() or p.is_symlink()}


def manifest(root, name, *, allow_missing_checkpoints=False):
    result = {}
    for line in (root / name).read_text().splitlines():
        checksum, relative = line.split('  ')
        require(re.fullmatch('[0-9a-f]{64}', checksum) and relative not in result
            and not Path(relative).is_absolute() and '..' not in Path(relative).parts, 'unsafe checksum inventory')
        path = root / relative
        if not path.exists() and allow_missing_checkpoints and relative.endswith('/training/checkpoints/latest.pt'):
            result[relative] = checksum
            continue
        require(digest(path) == checksum, 'checksum payload differs')
        result[relative] = checksum
    return result


def behavior_receipt(item):
    inventory = manifest(item, 'SHA256SUMS')
    expected = {'started.json', 'summary.json', 'samples.jsonl', *(f'trajectories/val_{i:03d}.png' for i in old.RENDER_INDICES)}
    require(set(inventory) == expected, 'behavior exact fifteen-file inventory differs')
    return dict(summary_sha256=inventory['summary.json'], checksums_sha256=digest(item / 'SHA256SUMS'), payload_sha256=inventory)


def parent_proof(parent, behavior, worker, parent_summary):
    require(parent_summary['status'] == 'COMPLETE_NOT_PROMOTED', 'all original eighteen plus six stages are required')
    require(digest(parent/'plan.json') == worker.PARENT_PLAN_SHA and digest(parent/'status.json') == worker.PARENT_STATUS_SHA,
        'fixed parent campaign identity differs')
    require(digest(behavior/'workflow_status.json') == worker.PARENT_BEHAVIOR_STATUS_SHA
        and digest(behavior/'WORKFLOW_SHA256SUMS') == worker.PARENT_BEHAVIOR_MANIFEST_SHA
        and digest(behavior/'provenance/stdin_worker.py') == worker.OWNER_SHA, 'fixed original behavior workflow differs')
    manifest(behavior, 'WORKFLOW_SHA256SUMS')
    state = read(behavior/'workflow_status.json')
    require(state.get('status') == 'COMPLETE_NOT_PROMOTED' and state.get('completed_behavior_count') == 6
        and state.get('source_commit') == SOURCE and state.get('worker_source_sha256') == worker.OWNER_SHA
        and state.get('source_and_normal_inputs_unchanged') is True and state.get('before') == state.get('after'),
        'original behavior completion proof differs')
    normal = state['before']
    expected_sources = {n: old.base._sha(old.git_bytes(SOURCE, n)) for n in old.BEHAVIOR_SOURCES}
    require(normal['source_sha256'] == expected_sources and normal['dataset_manifest_sha256'] == old.expansion.MANIFEST_SHA256,
        'parent full source or corpus proof differs')
    prefix = 'runs/campaigns/' + worker.PARENT_ID + '/'
    for relative, checksum in normal['normal_input_sha256'].items():
        require(relative.startswith(prefix), 'parent receipt path escapes original campaign')
        path = parent / relative[len(prefix):]
        if path.exists() or path.is_symlink(): require(digest(path) == checksum, 'parent receipt bytes changed')
        else: require(relative.endswith('/training/checkpoints/latest.pt'), 'parent noncheckpoint receipt is missing')
    expected = deepcopy(normal)
    expected['history_sha256'] = {}
    for run in worker.RUNS:
        for name, checksum in worker.PARENT_FILES[run].items():
            path = parent/run/'training'/name
            if path.exists() or path.is_symlink(): require(digest(path) == checksum, 'pinned parent training bytes changed')
            else: require(name == 'checkpoints/latest.pt', 'parent training report/history is missing')
        require(normal['checkpoint_sha256'][run] == worker.PARENT_FILES[run]['checkpoints/latest.pt'], 'parent checkpoint receipt differs')
        expected['history_sha256'][run] = worker.PARENT_FILES[run]['metrics.jsonl']
    records = state['stages']
    require([r.get('run') for r in records] == list(worker.RUNS)
        and all(r.get('status') == 'COMPLETE' and type(r.get('returncode')) is int and r['returncode'] == 0 for r in records),
        'original behavior stage ledger differs')
    reports = {r['run']: behavior_receipt(behavior/r['run']) for r in records}
    require(all(r.get('report') == reports[r['run']] for r in records), 'parent behavior receipt changed')
    expected['behavior_completion'] = dict(status_sha256=worker.PARENT_BEHAVIOR_STATUS_SHA,
        checksums_sha256=worker.PARENT_BEHAVIOR_MANIFEST_SHA, finished_at_utc=state['finished_at_utc'], reports=reports)
    return expected


def stage_command(record, worker, parent_state, checkpoint_sha):
    run, stage = record['run'], record['stage']
    original = next(r for r in parent_state['stages'] if r['run'] == run and r['stage'] == ('train' if stage == 'behavior' else stage))
    # HH_260906 - The already validated parent command provides the exact personal interpreter and workspace, without exposing either in output.
    python = Path(original['command'][0]); workspace = python.parents[3]
    parent_root = workspace/'runs/campaigns'/worker.PARENT_ID
    output = workspace/'runs/campaigns'/worker.CAMPAIGN_ID
    if stage == 'behavior':
        arm = run.split('/')[1]
        expected = [str(python), '-m', old.BEHAVIOR_SCRIPT.removesuffix('.py').replace('/', '.'),
            str(workspace/old.common.DATASET), '--checkpoint', str(output/run/'training/checkpoints/latest.pt'),
            '--checkpoint-sha256', checkpoint_sha, '--expected-source-commit', SOURCE,
            '--expected-model-id', old.MODEL_IDS[arm], '--output-dir', str(output/run/'behavior'),
            '--device', 'cuda:0', '--batch-size', '4']
    else:
        expected = [str(output)+value[len(str(parent_root)):] if value.startswith(str(parent_root)+'/') else value for value in original['command']]
        if stage == 'train': expected[expected.index('--max-steps')+1] = '2870'; expected.append('--resume')
        if stage == 'audit': expected[expected.index('--checkpoint-sha256')+1] = checkpoint_sha
    require(record.get('command') == expected, 'exact duration stage command differs')
    return [value.replace(str(workspace), '<PERSONAL_WORKSPACE>') for value in expected]


def continued_run(root, parent, relative, baseline, worker, sources):
    item = root/relative; original = read(parent/relative/'training/run.json')
    train, evaluation, audit = (read(item/n) for n in old.expansion.STAGE_FILES.values())
    seed = baseline['seed']; arm = baseline['arm']
    require(train.get('status') == 'TRAINING_TARGET_REACHED' and train.get('state') == worker.training_state(2870, final_report=True)
        and canonical(train.get('train_config')) == canonical(worker.train_config(seed, 2870)), 'continued full ten-epoch budget/config differs')
    for key in ('trainer_id', 'created_at_utc', 'dataset_size', 'dataset_fingerprint_sha256', 'corpus_fingerprint_sha256',
        'training_split', 'training_episode_ids', 'model_config', 'model_parameter_count', 'loss_config', 'runtime', 'hardware',
        'sampling_plan', 'sampling_plan_sha256', 'device'):
        require(train.get(key) == original.get(key), 'immutable parent training provenance differs: '+key)
    require('resumed_at_utc' in train and canonical(train['loss_config']) == canonical(worker.LOSS), 'resume or original loss proof differs')
    history, last = worker.history_proof(parent/relative/'training', item/'training', 2870)
    require(train.get('last_metrics') == last and history['parent_prefix_sha256'] == worker.PARENT_FILES[relative]['metrics.jsonl'],
        'last metrics or inherited prefix differs')
    for report, name in ((evaluation, 'evaluation/metrics.json'), (audit, 'gate_v8.json')):
        previous = read(parent/relative/name)
        require(report.get('status') == previous['status'] and report.get('evaluation_split') == 'val'
            and report.get('vehicle_control_approved') is False and report.get('training_domain_samples_seen') == {'carla':11470},
            'continued validation status, split or exposure differs')
        for key in ('model_config_sha256', 'model_parameter_count', 'dataset_fingerprint_sha256', 'corpus_fingerprint_sha256',
            'training_dataset_fingerprint_sha256', 'training_episode_count', 'evaluation_episode_count', 'runtime', 'hardware',
            'device', 'batch_size', 'training_sampling_plan_sha256', 'training_sampling_policy', 'domain_sample_counts'):
            require(report.get(key) == previous.get(key), 'continued validation provenance differs: '+key)
        require(not any(k in report for k in ('loss_config', 'auxiliary_loss_metrics', 'auxiliary_loss_metric_counts')),
            'unexpected changed objective')
    require(evaluation.get('sample_count') == 337 and evaluation.get('domain_sample_counts') == {'carla':337}, 'complete val337 required')
    checksum = evaluation.get('checkpoint_sha256')
    require(re.fullmatch('[0-9a-f]{64}', checksum or '') and audit.get('checkpoint_sha256') == checksum
        and checksum != baseline['checkpoint_sha256'], 'continued checkpoint binding differs')
    for name in ('audit_runtime', 'runtime_contract'):
        require(audit.get('implementation', {}).get(name+'_sha256') == sources[f'portable_e2e/{name}.py'], 'audit source differs')
    checkpoint = item/'training/checkpoints/latest.pt'
    if checkpoint.exists() or checkpoint.is_symlink(): require(digest(checkpoint) == checksum, 'local continued checkpoint bytes differ')
    result = deepcopy(baseline)
    result.update(checkpoint_sha256=checksum, checkpoint_bytes_locally_verified=checkpoint.exists(),
        geometry=old.geometry(audit, old.COUNTS[arm]), history=history,
        checkpoint_validation_reference={k:audit[k] for k in baseline['checkpoint_validation_reference']})
    return result


def summarize_campaign(campaign, parent, parent_behavior):
    roots = {'continued':Path(campaign).absolute(), 'parent':Path(parent).absolute(), 'parent_behavior':Path(parent_behavior).absolute()}
    result = dict(schema=SCHEMA, status='INCOMPLETE', completed_stage_count=0, expected_stage_count=24,
        completed_lineage_count=0, expected_lineage_count=6, automatic_promotion=False, training_data_approved=False,
        vehicle_control_approved=False, parent_reevaluated=False, checkpoint_tensors_loaded=False, lineages=[], stages=[],
        limitations=['Six continuations are not six fresh initializations; inherited 1540 updates are counted only once.',
            'Fixed endpoint comparison, not best epoch, seed selection, retry until pass or automatic promotion.',
            'Legacy v3 targets and all 337 overlapping val windows remain unchanged; no new admitted training data.',
            'Full-corpus integrity may read test bytes; no test prediction, optimization or model selection.',
            'Saved behavior rows are checked, not replayed from raw future arrays; target-defined groups do not establish traffic intent.',
            'Checkpoint bytes are hashed only if mirrored locally; absent tensors use the archived owner receipt and matching evaluation/behavior hashes.'])
    missing = [f'continued/{n}' for n in ('status.json', 'plan.json', 'provenance/stdin_worker.py') if not (roots['continued']/n).exists()]
    if missing: return dict(result, missing_artifacts=missing)
    require(digest(Path(old.__file__)) == READER_SHA, 'reviewed original summary helper changed')
    initial = {name:tree_pins(root) for name,root in roots.items()}
    helper_pins = {str(Path(__file__).relative_to(ROOT)):digest(Path(__file__)), str(Path(old.__file__).relative_to(ROOT)):READER_SHA}
    worker = frozen_worker(roots['continued']); state = read(roots['continued']/'status.json'); plan = read(roots['continued']/'plan.json')
    require(initial['continued']['plan.json'] == PLAN_SHA and state.get('plan_sha256') == PLAN_SHA and state.get('plan') == plan,
        'exact prospective duration plan binding differs')
    worker.validate_plan(plan, WORKER_SHA, observed=worker.timestamp(state['started_at_utc']))
    require(state.get('schema') == 'portable_e2e.stopmix_duration_workflow.v1' and state.get('worker_sha256') == WORKER_SHA
        and state.get('source_commit') == SOURCE and state.get('additional_fresh_initializations') == 0
        and all(state.get(k) is False for k in ('automatic_promotion', 'training_data_approved', 'vehicle_control_approved', 'original_parent_modified')),
        'duration workflow scope differs')
    original = old.summarize_campaign(roots['parent'], expected_source_commit=SOURCE, expected_plan_sha256=worker.PARENT_PLAN_SHA,
        behavior_root=roots['parent_behavior'])
    before = parent_proof(roots['parent'], roots['parent_behavior'], worker, original)
    require(state.get('before') == before, 'continued parent/source preflight receipt differs')
    require(worker.timestamp(before['behavior_completion']['finished_at_utc']) < worker.timestamp(plan['declared_at_utc']),
        'duration declaration predates original completion')
    baseline = {r['run']:r for pair in original['pairs'] for r in (pair['baseline'], pair['candidate'])}
    for run in worker.RUNS:
        require(worker.timestamp(read(roots['parent']/run/'training/run.json')['completed_at_utc']) < worker.timestamp(plan['declared_at_utc']),
            'duration declaration predates parent training')
    records = state.get('stages', [])
    require(isinstance(records, list) and len(records) <= 24
        and [(r.get('run'),r.get('stage')) for r in records] == list(worker.ORDER[:len(records)]), 'duration stage order differs')
    parent_state = read(roots['parent']/'status.json'); previous = worker.timestamp(state['started_at_utc'])
    report_paths, finished_runs = [], {}
    for index, record in enumerate(records):
        run, stage, status = record['run'], record['stage'], record.get('status')
        require(status in ('COMPLETE','RUNNING','FAILED') and (status == 'COMPLETE' or index == len(records)-1), 'unfinished duration stage followed by another')
        began = worker.timestamp(record['started_at_utc'])
        require(began >= previous and record.get('timeout_seconds') == worker.TIMEOUTS[stage], 'stage chronology/timeout differs')
        worker.remaining_budget(index, observed=began)
        if status != 'RUNNING':
            previous = worker.timestamp(record['finished_at_utc'])
            require(previous >= began, 'negative duration')
        if status == 'COMPLETE':
            require(type(record.get('returncode')) is int and record['returncode'] == 0
                and previous <= worker.timestamp(worker.DEADLINE), 'completed duration stage exit/deadline differs')
            item = roots['continued']/run; path = item/STAGE_FILES[stage]
            require(path.is_file(), 'completed stage report is missing')
            report = read(path); proof = record['report']
            checksum = proof.get('checkpoint_sha256') if stage != 'behavior' else report.get('checkpoint_sha256')
            if stage == 'train':
                require(state.get('copies', {}).get(run) == worker.PARENT_FILES[run], 'exact independent resume copy receipt missing')
                history, last = worker.history_proof(roots['parent']/run/'training', item/'training',2870)
                require(proof.get('history') == history and report.get('last_metrics') == last, 'history prefix/completion receipt differs')
            if stage == 'behavior': require(proof == behavior_receipt(item/'behavior'), 'behavior workflow receipt differs')
            else: require(proof.get('path') == STAGE_FILES[stage] and proof.get('sha256') == digest(path), 'stage report receipt differs')
            require(re.fullmatch('[0-9a-f]{64}', checksum or ''), 'stage checkpoint SHA missing')
            if stage in ('evaluate','audit'): require(report.get('checkpoint_sha256') == checksum, 'stage checkpoint receipt contradicts report')
            command = stage_command(record, worker, parent_state, checksum)
            if stage == 'behavior':
                run_result = continued_run(roots['continued'],roots['parent'],run,baseline[run],worker,before['source_sha256'])
                matching = records[index-3:index]
                require(all(r['report']['checkpoint_sha256'] == run_result['checkpoint_sha256'] for r in matching), 'three normal stages used different checkpoints')
                pins = {}; local_state = dict(stages=matching, val_fingerprint_sha256=parent_state['val_fingerprint_sha256'])
                behavior_result = old.behavior_evidence(item/'behavior',dict(run_result,run='.'),local_state,SOURCE,pins)
                require(behavior_result['status'] == 'COMPLETE_NOT_PROMOTED', 'continued behavior incomplete')
                behavior_result['run'] = run; run_result['behavior'] = behavior_result; finished_runs[run] = run_result
                report_paths.extend((roots['parent']/run/'evaluation/metrics.json', item/'evaluation/metrics.json'))
        else:
            checksum = old.base._option(record.get('command', []), '--checkpoint-sha256') if stage in ('audit','behavior') else None
            command = stage_command(record, worker, parent_state, checksum)
        result['stages'].append({k:record.get(k) for k in ('run','stage','status','returncode','started_at_utc','finished_at_utc','report')} | {'command':command})
    if report_paths:
        comparisons = old.base.compare_reports(report_paths)['reports']
        original_behaviors = {b['run']:b for b in original['behaviors']}
        for index, (run, value) in enumerate(finished_runs.items()):
            a = deepcopy(baseline[run]); a['behavior'] = original_behaviors[run]
            a['metrics'], value['metrics'] = comparisons[2*index]['metrics'], comparisons[2*index+1]['metrics']
            require(a['behavior']['sample_identity_sha256'] == value['behavior']['sample_identity_sha256'], 'continued validation sample/group identity changed')
            checks = dict(ade_improves=value['metrics']['selected_ade_m'] < a['metrics']['selected_ade_m'],
                fde_improves=value['metrics']['selected_fde_m'] < a['metrics']['selected_fde_m'],
                geometry_does_not_regress=value['geometry']['selected_pass_count'] >= a['geometry']['selected_pass_count'],
                speed_mae_within_5_percent=value['metrics']['selected_speed_mae_mps'] <= 1.05*a['metrics']['selected_speed_mae_mps'])
            absolute = {k:value['metrics'][k] <= limit for k,limit in old.base.ABSOLUTE_LIMITS.items()}
            result['lineages'].append(dict(run=run,parent=a,continued=value,relative_checks=checks,absolute_checks=absolute))
    complete = len(records) == 24 and all(r['status'] == 'COMPLETE' for r in records)
    if complete:
        require(state.get('status') == 'COMPLETE_NOT_PROMOTED' and state.get('completed_stage_count') == 24
            and state.get('parent_source_and_output_postcheck_pass') is True and len(finished_runs) == 6, 'full workflow completion proof differs')
        require(previous <= worker.timestamp(state['finished_at_utc']) <= worker.timestamp(worker.DEADLINE),
            'workflow finish is outside the stage/deadline interval')
        inventory = manifest(roots['continued'],'WORKFLOW_SHA256SUMS',allow_missing_checkpoints=True)
        require(set(inventory) == set(initial['continued'])-{'WORKFLOW_SHA256SUMS'} |
            {f'{run}/training/checkpoints/latest.pt' for run in worker.RUNS}, 'full workflow file inventory differs')
        for run,value in finished_runs.items(): require(inventory[f'{run}/training/checkpoints/latest.pt'] == value['checkpoint_sha256'], 'workflow checkpoint inventory differs')
        result['status'] = 'COMPLETE_NOT_PROMOTED'
    else: require(state.get('status') in ('RUNNING','STOPPED_FAILURE_NO_PROMOTION'), 'incomplete workflow falsely claims completion')
    result.update(source_commit=SOURCE, worker_sha256=WORKER_SHA, plan_sha256=PLAN_SHA,
        runner_status=state['status'], completed_stage_count=sum(r['status']=='COMPLETE' for r in records),
        completed_lineage_count=len(finished_runs), source_proof=before['source_sha256'],
        budget=dict(additional_fresh_initializations=0,continuation_lineages=6,parent_steps=1540,additional_steps=1330,total_steps=2870,
            parent_samples_seen=6155,additional_samples_seen=5315,total_samples_seen=11470,train_samples=1147,val_samples=337,
            total_complete_epochs=10,additional_steps_all_six=7980,additional_samples_seen_all_six=31890),
        all_lineages_relative_checks_pass=complete and all(all(r['relative_checks'].values()) for r in result['lineages']),
        all_lineages_absolute_checks_pass=complete and all(all(r['absolute_checks'].values()) for r in result['lineages']),
        reader_source_sha256=helper_pins, test_evaluated=False, test_used_for_training_or_selection=False)
    require(all(tree_pins(roots[n]) == pins for n,pins in initial.items()), 'input bytes changed during duration summary')
    require(all(digest(ROOT/n) == checksum for n,checksum in helper_pins.items()), 'summary helper source changed')
    result['input_manifest'] = {n:[dict(path=p,sha256=s) for p,s in pins.items()] for n,pins in initial.items()}
    return result


def checked_output(output, inputs):
    output = Path(output).absolute()
    require(not output.exists() and all(not p.is_symlink() for p in (output,*output.parents))
        and not any(output.resolve().is_relative_to(Path(p).resolve()) for p in [*inputs,ROOT/'datasets'])
        and not output.resolve().is_relative_to(ROOT/'datasets'), 'fresh output must be outside all inputs and dataset aliases')
    return output


def loss_bins(values):
    # HH_260906 - Preserve fixed 50-update bins, including the mixed 1501–1550 boundary bin and final 20 updates.
    result=[]
    for index in range(0,len(values),50):
        rows=values[index:index+50]
        weights=[sum(row['batch_domain_sample_counts'].values()) for row in rows]
        result.append(dict(first_step=rows[0]['global_step'],last_step=rows[-1]['global_step'],updates=len(rows),
            exposures=sum(weights),mean_loss=sum(row['loss']*weight for row,weight in zip(rows,weights))/sum(weights)))
    return result


def plot_results(report, campaign, output):
    # HH_260906 - Every recorded update and all six fixed endpoints remain represented; no camera or driving imagery is generated.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    require(report['status'] == 'COMPLETE_NOT_PROMOTED' and len(report['lineages']) == 6, 'plots require all six completed lineages')
    figure, axes = plt.subplots(2,3,figsize=(15,8),sharex=True)
    for row, axis in zip(report['lineages'],axes.flat):
        path = Path(campaign)/row['run']/'training/metrics.jsonl'
        require(digest(path) == row['continued']['history']['sha256'], 'plot history changed')
        values = [json.loads(line) for line in path.read_text().splitlines()]
        require(len(values) == 2870, 'plot must retain every update')
        axis.plot([v['global_step'] for v in values],[v['loss'] for v in values],alpha=.3,lw=.5,color='#246a94',label='Every recorded batch')
        bins = loss_bins(values)
        axis.plot([b['last_step'] for b in bins],[b['mean_loss'] for b in bins],
            lw=1.5,color='#142b48',label='Sample-weighted 50-update bins (last 20)')
        axis.axvspan(1,1540,color='#999999',alpha=.12);axis.axvline(1540,color='#b55623',ls='--')
        axis.set(title=row['run'].replace('seed_','').replace('/',' / '),xlabel='Global optimizer update',ylabel='Training composite loss')
        axis.grid(alpha=.2)
    axes.flat[0].legend(fontsize=6);figure.suptitle('SAVED TRAINING METRICS: gray = inherited 1–1540; new = 1541–2870\nDifferent K/capacity changes total-loss interpretation; this is not driving evidence')
    figure.tight_layout();figure.savefig(output/'01_continued_learning_curves.png',dpi=150);plt.close(figure)
    figure, axes = plt.subplots(1,3,figsize=(15,5))
    labels = [r['run'].replace('seed_202609','').replace('/A_physical_drive',' A6').replace('/B_drive_stop_mix',' B12') for r in report['lineages']]
    for axis,key,title in zip(axes,('selected_ade_m','selected_fde_m','selected_speed_mae_mps'),('Selected ADE (m)','Selected FDE (m)','Speed MAE (m/s)')):
        for index,row in enumerate(report['lineages']):
            axis.bar(index-.18,row['parent']['metrics'][key],width=.36,color='#8b98a9',label='1540 parent' if index==0 else None)
            axis.bar(index+.18,row['continued']['metrics'][key],width=.36,color='#226f98',label='2870 continuation' if index==0 else None)
        axis.set(title=title,xticks=range(6),xticklabels=labels);axis.grid(axis='y',alpha=.2);axis.legend(fontsize=8)
    figure.suptitle('Same held-out development VAL337: all six fixed lineage endpoints\nNo best checkpoint selection, new data admission, or model promotion')
    figure.tight_layout();figure.savefig(output/'02_fixed_endpoint_validation.png',dpi=150);plt.close(figure)
    return {'matplotlib_version':matplotlib.__version__, 'learning_history_points':6*2870,
        'inherited_updates_per_lineage':1540,'new_updates_per_lineage':1330,'training_bin_size':50,'last_bin_size':20,
        'mixed_boundary_bin':{'first_step':1501,'last_step':1550,'inherited_updates':40,'new_updates':10},
        'files':['01_continued_learning_curves.png','02_fixed_endpoint_validation.png']}


def markdown(report):
    lines = ['# STOPMIX 6개 계보 · 총 10 epoch까지 고정 길이 계속 학습', '',f"상태: `{report['status']}` · 완료 단계 {report['completed_stage_count']}/24", '',
        '새 모델 6개를 처음부터 학습한 것이 아닙니다. 각 기존 1540 step/6155회 노출에서 1330 step/5315회를 추가해 총 2870 step/11470회로 이어갑니다.', '',
        '| 계보 | ADE 1540→2870 m | FDE 1540→2870 m | Gate 1540→2870 /337 | 상대조건 | 절대조건 |',
        '| --- | ---: | ---: | ---: | --- | --- |']
    for row in report['lineages']:
        a,b=row['parent'],row['continued']
        lines.append(f"| {row['run']} | {a['metrics']['selected_ade_m']:.5f} → {b['metrics']['selected_ade_m']:.5f} | {a['metrics']['selected_fde_m']:.5f} → {b['metrics']['selected_fde_m']:.5f} | {a['geometry']['selected_pass_count']} → {b['geometry']['selected_pass_count']} | {all(row['relative_checks'].values())} | {all(row['absolute_checks'].values())} |")
    lines += ['', '그림은 실제 저장된 수치이며 주행 영상이 아닙니다. 기존 데이터·모델·loss·학습률·sampling·gate를 유지한 고정 길이 비교입니다.',
        '학습 곡선의 50-step 요약 중 1501–1550 구간만 기존 40개와 새 10개 update가 섞입니다. 원래 batch 점과 1540 경계를 모두 표시하며 마지막 bin은 20개입니다.',
        '모든 seed와 실패를 보존합니다. 정지 의도 학습·도로 주행·새 데이터 승인·자동 승격을 입증하지 않습니다.',
        'test 파일은 전체 corpus 무결성 검사에서 읽힐 수 있지만 test 추론·학습·모델 선택에는 사용하지 않습니다.', '']
    return '\n'.join(lines)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('campaign',type=Path);parser.add_argument('--parent',type=Path,required=True)
    parser.add_argument('--parent-behavior',type=Path,required=True);parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--plots',action='store_true');args=parser.parse_args(argv)
    try:
        roots={'continued':args.campaign,'parent':args.parent,'parent_behavior':args.parent_behavior}
        output=checked_output(args.output_dir,list(roots.values()))
        report=summarize_campaign(args.campaign,args.parent,args.parent_behavior)
        output.mkdir(parents=True,exist_ok=False)
        if args.plots and report['status']=='COMPLETE_NOT_PROMOTED':report['rendering']=plot_results(report,args.campaign,output)
        for name,entries in report.get('input_manifest',{}).items():
            require(tree_pins(roots[name])=={e['path']:e['sha256']for e in entries},'inputs changed during publication')
        for name,checksum in report.get('reader_source_sha256',{}).items():require(digest(ROOT/name)==checksum,'reader changed during publication')
        for name,value in [('summary.json',json.dumps(report,indent=2,allow_nan=False)+'\n'),('README.md',markdown(report))]:
            with (output/name).open('x') as stream:stream.write(value)
        with (output/'SHA256SUMS').open('x') as stream:
            for path in sorted(output.iterdir()):
                if path.name!='SHA256SUMS':stream.write(f'{digest(path)}  {path.name}\n')
    except (ContractError,RuntimeError,OSError,ValueError,TypeError,KeyError,IndexError) as error:
        print('DURATION_SUMMARY_ERROR: '+type(error).__name__);return 2
    print(json.dumps({k:report[k]for k in ('status','completed_stage_count','completed_lineage_count')}))
    return 0 if report['status']=='COMPLETE_NOT_PROMOTED' else 1


if __name__=='__main__':
    raise SystemExit(main())
