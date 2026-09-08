"""HH_260906 - Test K6/K12 result binding using synthetic files, never actual training, images or checkpoint decoding."""

from copy import deepcopy
from dataclasses import asdict
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import shutil

import pytest

from portable_e2e.contract import ContractError
from scripts.e2e import summarize_portable_stopmix as module
import test_summarize_portable_no_accel_development as old
from test_summarize_portable_no_accel_development import no_accel, expansion, campaign  # noqa: F401


@pytest.fixture
def stopmix(no_accel, tmp_path, monkeypatch):
    root = tmp_path / 'stopmix'
    previous = module.read(no_accel / 'status.json')
    plan = module.read(module.ROOT / 'config/portable_e2e_stopmix_20260909.json')
    assert plan['source_commit'] == 'b478f02e42b94bf04bffec5c8170e05edc33b0f8'
    sources = {n: (module.ROOT / n).read_bytes() for n in set(module.SOURCE_PATHS + module.BEHAVIOR_SOURCES)}
    monkeypatch.setattr(module, 'git_bytes', lambda commit, name: sources[name])
    old.first._write(root / 'plan.json', plan)
    state = deepcopy(previous)
    state.update(plan=plan, source_commit=plan['source_commit'], plan_sha256=module.sha_file(root / 'plan.json'),
        created_at_utc='2026-09-08T18:15:00+00:00', stages=[], runner_sha256=module.base._sha(sources[module.common.WORKER]),
        source_files={n: {'sha256': module.base._sha(sources[n]), 'archive_path': module.archive_name(n)} for n in module.SOURCE_PATHS},
        behavior_analysis={'status': 'NOT_RUN_SEPARATE_WORKFLOW', 'expected_count': 6,
            'per_analysis_timeout_seconds': 180, 'included_in_completed_stage_count': False}, model_configs={})
    mapping = dict(zip(old.module.ARMS, module.ARMS))
    for arm, name in module.MODELS.items():
        config = json.loads(sources[name])
        state['model_configs'][arm] = {'path': name, 'sha256': module.MODEL_SHA256[arm], 'model_config': config,
            'canonical_sha256': module.base._sha(module.canonical(config).encode())}
    for seed in module.base.SEEDS:
        for old_arm, arm in mapping.items():
            relative = f'seed_{seed}/{arm}'; shutil.copytree(no_accel / f'seed_{seed}/{old_arm}', root / relative)
            model = state['model_configs'][arm]
            for stage, name in module.expansion.STAGE_FILES.items():
                path = root / relative / name; value = module.read(path)
                value['model_parameter_count'] = module.PARAMETERS[arm]
                if stage == 'train': value['model_config'] = model['model_config']
                else:
                    value['model_config_sha256'] = model['canonical_sha256']; value['checkpoint_id'] = 'portable_e2e.pytorch_checkpoint.v1'
                if stage == 'audit':
                    value['implementation'] = {n + '_sha256': module.base._sha(sources[f'portable_e2e/{n}.py']) for n in ('audit_runtime', 'runtime_contract')}
                    g = value['geometry']; k = module.COUNTS[arm]
                    value['gate']['thresholds'] = asdict(module.RuntimeGateConfig(candidate_count=k))
                    g['candidate_results'] = [dict(deepcopy(g['selected_result']), candidate_index=i) for i in range(k)]
                    g.update(all_candidates_geometry_pass_count=336, all_candidates_geometry_pass_rate=336/337,
                        any_candidate_geometry_pass_count=336, any_candidate_geometry_pass_rate=336/337)
                    g['selector']['selection_counts'] = {str(i): 337 if i == k-1 else 0 for i in range(k)}
                old.first._write(path, value)
                original = deepcopy(next(r for r in previous['stages'] if r['run'] == f'seed_{seed}/{old_arm}' and r['stage'] == stage))
                when = datetime(2026, 9, 8, 18, 15, tzinfo=timezone.utc) + timedelta(seconds=2*len(state['stages']))
                original.update(run=relative, command=[v.replace(old.module.CAMPAIGN_ID, module.CAMPAIGN_ID).replace(old_arm, arm)
                    .replace(old.module.MODELS[old_arm], module.MODELS[arm]) for v in original['command']],
                    started_at_utc=when.isoformat(), finished_at_utc=(when+timedelta(seconds=1)).isoformat())
                original['report']['sha256'] = module.sha_file(path); state['stages'].append(original)
    for name in module.SOURCE_PATHS:
        path = root / module.archive_name(name); path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(sources[name])
    old.first._write(root / 'status.json', state)
    return root


def summarize(root, **kwargs):
    return module.summarize_campaign(root, expected_source_commit=module.read(root/'plan.json')['source_commit'],
        expected_plan_sha256=module.sha_file(root/'plan.json'), **kwargs)


def mutate(root, stage, callback, arm='B_drive_stop_mix', seed=20260903):
    relative = f'seed_{seed}/{arm}'; path = relative + '/' + module.expansion.STAGE_FILES[stage]
    old.first._mutate(root, path, callback)
    state = module.read(root/'status.json')
    next(r for r in state['stages'] if r['run'] == relative and r['stage'] == stage)['report']['sha256'] = module.sha_file(root/path)
    old.first._write(root/'status.json', state)


def test_complete_normal_stages_never_claim_missing_behavior_complete(stopmix):
    result = summarize(stopmix)
    assert result['status'] == 'COMPLETE_NORMAL_STAGES' and result['normal_stage_completion'] == 'COMPLETE'
    assert result['behavior_completion'] == 'INCOMPLETE' and result['completed_behavior_count'] == 0
    assert result['candidate_screen'] == result['absolute_quality'] == 'PASS'
    assert result['completed_stage_count'] == 18 and len(result['input_manifest']) == 31
    assert result['automatic_promotion'] is result['vehicle_control_approved'] is False
    assert result['budget']['samples_seen_per_run'] == 6155
    for pair in result['pairs']:
        for label, k in [('baseline',6),('candidate',12)]:
            g=pair[label]['geometry']; assert g['candidate_count']==len(g['raw_geometry']['candidate_results'])==len(g['selection_histogram'])==k
            assert g['selected_failure_counts']=={'speed':1}
    assert '/synthetic/' not in json.dumps(result) and str(stopmix) not in json.dumps(result)
    assert '파라미터 용량' in module.render_markdown(result)


@pytest.mark.parametrize('stage,callback', [
    ('train',lambda d:d.update(model_parameter_count=954590)),
    ('train',lambda d:d['state'].update(domain_samples_seen={'carla':6160})),
    ('train',lambda d:d['loss_config'].update(candidate_regret_weight=0.0)),
    ('train',lambda d:d['train_config'].update(seed=20260904)),
    ('train',lambda d:d.update(training_episode_ids=['one'])),
    ('evaluate',lambda d:d.update(auxiliary_loss_metrics={})),
    ('evaluate',lambda d:d.update(evaluation_split='test')),
    ('evaluate',lambda d:d['hardware'].update(device_uuid='other')),
    ('audit',lambda d:d.update(checkpoint_sha256='f'*64)),
    ('audit',lambda d:d['gate']['thresholds'].update(candidate_count=6)),
    ('audit',lambda d:d['gate']['thresholds'].update(maximum_speed_mps=20)),
    ('audit',lambda d:d['geometry']['candidate_results'].pop()),
    ('audit',lambda d:d['geometry']['candidate_results'][8].update(candidate_index=7)),
    ('audit',lambda d:d['geometry']['selector']['selection_counts'].update({'11':336})),
    ('audit',lambda d:d['geometry']['selected_result'].update(failure_counts={})),
    ('audit',lambda d:d['implementation'].update(audit_runtime_sha256='f'*64)),
])
def test_resealed_semantic_failures_not_hidden(stopmix,stage,callback):
    mutate(stopmix,stage,callback)
    with pytest.raises(ContractError): summarize(stopmix)


@pytest.mark.parametrize('fault',['source','plan','archive','stage_order','command','budget','behavior_status'])
def test_explicit_pins_source_commands_and_chronology_fail_closed(stopmix,fault):
    kwargs={}
    if fault=='source': kwargs['expected_source_commit']='a'*40
    elif fault=='plan': kwargs['expected_plan_sha256']='f'*64
    elif fault=='archive': (stopmix/'provenance/portable_e2e/stop_primitive_research.py').write_text('changed')
    else:
        state=module.read(stopmix/'status.json')
        if fault=='stage_order': state['stages'].reverse()
        elif fault=='command': state['stages'][0]['command']+=['--resume','old.pt']
        elif fault=='budget': state['stages'][0]['started_at_utc']='2026-09-09T00:59:00+00:00'
        else: state['behavior_analysis']['status']='COMPLETE'
        old.first._write(stopmix/'status.json',state)
    with pytest.raises(ContractError):
        module.summarize_campaign(stopmix, **dict(expected_source_commit=module.read(stopmix/'plan.json')['source_commit'],
            expected_plan_sha256=module.sha_file(stopmix/'plan.json'),**{}) | kwargs)


def test_incomplete_ledger_and_missing_archive_retain_denominators(stopmix):
    old.first._mutate(stopmix,'status.json',lambda d:d.update(stages=d['stages'][:3],status='RUNNING'))
    result=summarize(stopmix)
    assert result['status']=='INCOMPLETE' and result['completed_stage_count']==3 and len(result['not_started_stages'])==15
    assert result['pairs']==[]


@pytest.fixture
def behavior_root(stopmix,tmp_path):
    # HH_260906 - Build deterministic scalar prediction fixtures; these are not real learned checkpoint outputs.
    from scripts.e2e import audit_portable_stopmix_behavior as behavior
    from test_audit_portable_stopmix_behavior import sample_arguments
    root=tmp_path/'behaviors'; state=module.read(stopmix/'status.json')
    normal=summarize(stopmix)
    for pair in normal['pairs']:
        for label in ('baseline','candidate'):
            run=pair[label]; item=root/run['run']; item.mkdir(parents=True)
            template=behavior.analyze_sample(**sample_arguments(model_id=module.MODEL_IDS[run['arm']]))
            rows=[]
            for i in range(337):
                row=deepcopy(template); row.update(index=i,sample_id=f'sample-{i}',sequence_index=i,anchor_timestamp_ns=1000000000+i*100000000)
                rows.append(row)
            (item/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            started=dict(schema=behavior.SCHEMA,status='RUNNING',source_commit=state['source_commit'],
                started_at_utc='2026-09-08T18:17:00Z',
                source_sha256={n:module.base._sha(module.git_bytes(state['source_commit'],n)) for n in module.BEHAVIOR_SOURCES},
                checkpoint_sha256=run['checkpoint_sha256'],model_id=module.MODEL_IDS[run['arm']],candidate_count=run['candidate_count'],
                dataset_manifest_sha256=module.expansion.MANIFEST_SHA256,corpus_fingerprint_sha256=module.expansion.CORPUS_SHA256,
                dataset_fingerprint_sha256=state['val_fingerprint_sha256'],evaluation_split='val',expected_sample_count=337,
                vehicle_control_approved=False,training_data_approved=False,test_inference_or_optimization_or_selection=False,
                batch_size=4,device='cuda:0',fixed_render_indices=list(module.RENDER_INDICES))
            old.first._write(item/'started.json',started)
            renders=[]
            for i in module.RENDER_INDICES:
                name=f'trajectories/val_{i:03d}.png'; path=item/name;path.parent.mkdir(exist_ok=True);path.write_bytes(b'synthetic fixture image digest only')
                renders.append(dict(index=i,file=name,sha256=module.sha_file(path),**{k:rows[i][k] for k in ('sample_id','anchor_timestamp_ns','camera_sha256')}))
            report=dict(started,status='COMPLETE_NOT_PROMOTED',checkpoint_validation=run['checkpoint_validation_reference'],
                ended_at_utc='2026-09-08T18:17:30Z',
                source_checkpoint_and_corpus_postcheck_pass=True,gate={'source':module.RUNTIME_GATE_ID,'thresholds':asdict(module.RuntimeGateConfig(candidate_count=run['candidate_count'])),'threshold_overrides':False},
                samples_sha256=module.sha_file(item/'samples.jsonl'),renders=renders,**behavior.summarize_rows(rows,run['candidate_count']))
            old.first._write(item/'summary.json',report); seal_behavior(item)
    return root


def seal_behavior(item):
    paths=[item/'started.json',item/'samples.jsonl',item/'summary.json',*sorted((item/'trajectories').iterdir())]
    (item/'SHA256SUMS').write_text(''.join(f'{module.sha_file(p)}  {p.relative_to(item)}\n' for p in paths))


def test_all_six_behavior_reports_rows_and_fixed_images_bound(stopmix,behavior_root):
    result=summarize(stopmix,behavior_root=behavior_root)
    assert result['status']=='COMPLETE_NOT_PROMOTED' and result['completed_behavior_count']==6
    assert result['normal_stage_completion']==result['behavior_completion']=='COMPLETE'
    assert len(result['behavior_input_manifest'])==6*16 and result['automatic_promotion'] is False
    assert all(r['sample_count']==337 for r in result['behaviors'])


@pytest.mark.parametrize('fault',['checkpoint','count','aggregate','image_link','source','nested_checkpoint','selection','timeout','before_normal','gate','speed_behavior'])
def test_behavior_resealed_bad_evidence_fails(stopmix,behavior_root,fault):
    item=behavior_root/'seed_20260903/B_drive_stop_mix'; report=module.read(item/'summary.json')
    if fault=='checkpoint': report['checkpoint_sha256']='f'*64
    elif fault=='count': report['all_samples']['sample_count']=336
    elif fault=='aggregate': report['target_motion_groups']['continuing_motion']['metrics']['selected_ade_m']=999
    elif fault=='image_link': report['renders'][0]['sample_id']='wrong'
    elif fault=='source': report['source_sha256'][module.BEHAVIOR_SCRIPT]='f'*64
    elif fault=='nested_checkpoint': report['checkpoint_validation']['training_domain_samples_seen']={'carla':1}
    elif fault=='timeout': report['ended_at_utc']='2026-09-08T18:20:01Z'
    elif fault=='before_normal':
        report['started_at_utc']='2026-09-08T18:14:00Z'
        old.first._mutate(item,'started.json',lambda d:d.update(started_at_utc=report['started_at_utc']))
    else:
        rows=[json.loads(x) for x in (item/'samples.jsonl').read_text().splitlines()]
        if fault=='gate': rows[0]['runtime_geometry']['selected_geometry_pass']=False
        elif fault=='speed_behavior': rows[0]['selected_speed_behavior']['terminal_exact_zero']=True
        else: rows[0]['selected_ade_m']=999
        (item/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows));report['samples_sha256']=module.sha_file(item/'samples.jsonl')
    old.first._write(item/'summary.json',report);seal_behavior(item)
    with pytest.raises(ContractError): summarize(stopmix,behavior_root=behavior_root)


def test_missing_behavior_preserves_five_completed_plus_normal18(stopmix,behavior_root):
    item=behavior_root/'seed_20260905/B_drive_stop_mix'; (item/'summary.json').rename(item/'unfinished-summary.json')
    result=summarize(stopmix,behavior_root=behavior_root)
    assert result['status']=='COMPLETE_NORMAL_STAGES' and result['completed_behavior_count']==5
    assert result['behaviors'][-1]['status']=='INCOMPLETE'


def test_cli_explicit_actual_plan_bindings_and_new_output_only(stopmix,tmp_path):
    output=tmp_path/'summary';args=[str(stopmix),'--expected-source-commit','b478f02e42b94bf04bffec5c8170e05edc33b0f8',
        '--expected-plan-sha256',module.sha_file(stopmix/'plan.json'),'--output-dir',str(output)]
    assert module.main(args)==0
    assert module.main(args)==2
    for line in (output/'SHA256SUMS').read_text().splitlines():
        checksum,name=line.split('  ');assert module.sha_file(output/name)==checksum
    # HH_260906 - The real prospective file is independently pinned, not replaced by the fixture's JSON formatting.
    assert module.sha_file(module.ROOT/'config/portable_e2e_stopmix_20260909.json')=='f06c68ce47aaee6941a3ca42559460ac5837b53f669bf195fa15b5d5e80a05d9'
