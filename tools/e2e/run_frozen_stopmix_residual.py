#!/usr/bin/env python3
"""HH_260906 - Compare matched residual scorers from authenticated caches without another generator forward."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
from types import ModuleType

WORKSPACE = Path.home() / 'personal/hwanhong/portable_e2e'
SOURCE_COMMIT = 'b478f02e42b94bf04bffec5c8170e05edc33b0f8'
PARENT_ROOT = WORKSPACE / 'runs/diagnostics/frozen_stopmix_selector_v1'
PARENT_PLAN_SHA = 'a11c5c94e94077fbbd6eb3d270fe54e62ccec5590082c28b9f76c8552479cdfa'
PARENT_REPORT_SHA = '22e26bcc2b1db98594b973127bdb8c4144eaab801ec8ff53384b3bc44a9fd7cf'
PARENT_MANIFEST_SHA = '7d6552d0dc242915806e36e700a8190890c6d1a01fa1c96991798535641288b9'
FROZEN_WORKER_SHA = '469bf474d63dbaa0c7556d77af981e12eee42df8f63318fc9f522a0b38e4bb8a'
FROZEN_CORE_SHA = '4716c22d29b8e306b86f8835bf2a5ec2bf956bbb6859c6b6f001a261865b8de3'
ORACLE_SHA = '35d3d0a464fb43c9789f6e24a35e453736c99fdceb36e3292d085c5ceb960d37'
OWNER_SHA = 'a995722f1a60f192a059433afb44e6297937d97166877c5cc4de23220d15b436'
OWNER_PATH = WORKSPACE / 'benchmarks/training_profile_v1/profile_portable_training.py'
ARMS = ('prior_only_residual', 'candidate_residual')
HEAD_PARAMETERS = 115457
HEAD_TIMEOUT, AUDIT_TIMEOUT, CACHE_TIMEOUT = 120, 90, 30
INTERNAL_TIMEOUT, EXTERNAL_TIMEOUT, RESERVE = 1800, 2100, 180
SCHEMA = 'portable_e2e.external_frozen_stopmix_residual.v1'
PLAN_SCHEMA = 'portable_e2e.external_frozen_stopmix_residual_plan.v1'
DENIALS = dict(generator_constructed=False, generator_neural_forwards=0, generator_training=False,
    new_candidate_decoding=False, cached_candidates_regenerated=False, original_parent_checkpoints_loaded=False,
    runtime_checkpoint_export=False, automatic_promotion=False, vehicle_control_approved=False,
    training_data_approved=False, source_or_label_changes=False, test_neural_inference=False, test_used_for_selection=False)


def require(value, message):
    if not value: raise ValueError(message)


def sha(raw): return hashlib.sha256(raw).hexdigest()


def encode(value): return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()


def regular(path):
    path = Path(path).absolute()
    require('..' not in path.parts and path == path.resolve(), 'canonical path without parent traversal required')
    require(path.is_file() and not any(p.is_symlink() for p in (path, *path.parents)), 'regular nonsymlink source required')
    return path


def checked_output(path, dataset, base):
    output = base.checked_output(path, dataset, WORKSPACE)
    # HH_260906 - A fresh diagnostic must neither escape its namespace nor add files inside the authenticated original experiment.
    require('..' not in output.parts and output == output.resolve(), 'canonical output without parent traversal required')
    original = PARENT_ROOT.resolve()
    require(not output.is_relative_to(original) and not original.is_relative_to(output), 'output must be separate from original cache experiment')
    return output


def load_external(path, expected, name):
    path = regular(path); raw = path.read_bytes(); require(sha(raw) == expected, 'external source SHA differs')
    require(name not in sys.modules, 'external module already loaded')
    module = ModuleType(name); module.__file__ = str(path); module.__package__ = name.rpartition('.')[0]; sys.modules[name] = module
    try: exec(compile(raw, str(path), 'exec'), module.__dict__)
    except BaseException:
        del sys.modules[name]; raise
    require(sha(path.read_bytes()) == expected, 'external bytes changed while importing')
    return module


def plan_contract(base, old, oracle, projection):
    return dict(schema=PLAN_SCHEMA, experiment_id='hh260909-frozen-stopmix-matched-residual-six-heads-v1',
        source_commit=SOURCE_COMMIT, gpu_uuid=base.GPU_UUID, dataset=base.DATASET,
        dataset_manifest_sha256=base.MANIFEST_SHA, corpus_fingerprint_sha256=base.CORPUS_SHA,
        split_contract={k:dict(sample_count=v[0], fingerprint_sha256=v[1]) for k,v in old.SPLITS.items()},
        parent_cache_plan_sha256=PARENT_PLAN_SHA, parent_cache_report_sha256=PARENT_REPORT_SHA,
        parent_cache_manifest_sha256=PARENT_MANIFEST_SHA, parent_cache_source_sha256=FROZEN_WORKER_SHA,
        parent_checkpoint_sha256=old.CHECKPOINTS, original_receipt_sha256=oracle.RECEIPTS,
        model_config=old.MODEL_PATH, model_config_sha256=old.MODEL_SHA, model_id=old.MODEL_ID,
        seeds=list(old.CHECKPOINTS), arms=list(ARMS), expected_head_fit_count=6, head_parameter_count=HEAD_PARAMETERS,
        candidate_count=12, future_points=64, fused_width=256, score_input_width=449,
        steps=1540, batch_size=4, samples_seen_per_head=6155, learning_rate=1e-4, weight_decay=1e-4,
        candidate_score_weight=.1, maximum_gradient_norm=5., loss_config=base.LOSS_CONFIG, loss_projection=projection,
        num_workers=0, fixed_val_render_indices=list(old.RENDER_INDICES), expected_val_png_count=108,
        cache_files_loaded=6, original_pt_files_hash_checked=18, original_pt_files_copied=0,
        safe_cache_loading='Unchanged train._read_checkpoint_file: CPU, weights_only=True, same-inode pre/post; no unsafe fallback.',
        cache_validation='Original artifact ID/keys/ABI/digest and ordered source IDs; targets and routes reconstructed exactly by unchanged dataset._targets/_route. Fused/candidate provenance is authenticated, not independently regenerated.',
        model_input_hashes_recomputed=False,
        scorer_contract='Original detached score plus 449->256->1 residual; output weight/bias zero. A replaces only 192 normalized geometry inputs with zeros, B keeps XY/120 and speed/(30/3.6). Same seed, full parameter count and sampling order.',
        cache_timeout_seconds=CACHE_TIMEOUT, head_timeout_seconds=HEAD_TIMEOUT, paired_audit_timeout_seconds=AUDIT_TIMEOUT,
        internal_wall_timeout_seconds=INTERNAL_TIMEOUT, external_wall_timeout_seconds=EXTERNAL_TIMEOUT,
        safety_reserve_seconds=RESERVE, finish_before_utc='2026-09-09T01:00:00Z',
        frozen_core_sha256=FROZEN_CORE_SHA, frozen_worker_sha256=FROZEN_WORKER_SHA,
        oracle_helper_sha256=ORACLE_SHA, owner_helper_sha256=OWNER_SHA,
        integrity_scope='The unchanged whole-corpus validator may read held-out test bytes for integrity. No test NN, optimization, metrics or model selection. No additional image decoding for cache reconstruction.',
        interpretation='Six head-only fits, not new full-model training. Feature access differs; no improvement, deployment, driving or safety approval is assumed.', **DENIALS)


def validate_plan(plan, base, old, oracle, projection):
    expected = plan_contract(base, old, oracle, projection)
    require(set(plan) == set(expected) | {'declared_at_utc','source_sha256','worker_source_sha256','residual_core_source_sha256'}, 'exact prospective plan fields required')
    require(encode({k:plan[k] for k in expected}) == encode(expected), 'unreviewed residual experiment setting')
    require(set(plan['source_sha256']) == set(old.source_paths(base)), 'complete native source inventory required')
    for value in [*plan['source_sha256'].values(), plan['worker_source_sha256'], plan['residual_core_source_sha256']]:
        require(type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value), 'SHA256 required')
    now = datetime.now(timezone.utc)
    require(base.timestamp(plan['declared_at_utc']) <= now and (base.timestamp(plan['finish_before_utc'])-now).total_seconds() >= EXTERNAL_TIMEOUT+RESERVE, 'whole external budget and reserve required')


def source_identity(plan, parent_plan, base, old, repo, args):
    require(parent_plan['source_sha256'] == plan['source_sha256'], 'old/new native source pins differ')
    result = old.source_identity(parent_plan, base, repo, args.frozen_core, args.oracle_helper)
    require(base.digest(args.frozen_worker) == FROZEN_WORKER_SHA and base.digest(args.frozen_core) == FROZEN_CORE_SHA, 'frozen helper source differs')
    for name,path,value in (('external_residual_worker.py', Path(__file__), plan['worker_source_sha256']),
        ('external_residual_core.py', args.core, plan['residual_core_source_sha256'])):
        require(base.digest(path) == value, 'residual source changed: '+name); result[name] = value
    return result


def parent_snapshot(plan, base, old, oracle, repo, args):
    require(base.digest(PARENT_ROOT/'plan.json') == PARENT_PLAN_SHA and base.digest(PARENT_ROOT/'report.json') == PARENT_REPORT_SHA
        and base.digest(PARENT_ROOT/'SHA256SUMS') == PARENT_MANIFEST_SHA, 'completed cache experiment identity differs')
    parent_plan = base.json_read(PARENT_ROOT/'plan.json'); report = base.json_read(PARENT_ROOT/'report.json')
    require(report['status'] == 'SIX_HEAD_FITS_COMPLETE_NOT_PROMOTED' and report['completed_head_fit_count'] == 6
        and report['source_and_inputs_unchanged'] is True and report['postcheck_errors'] == [] and report['deadline_met'] is True, 'completed frozen cache source required')
    require(base.timestamp(report['completed_at_utc']) <= base.timestamp(plan['declared_at_utc']), 'residual plan must follow completed cache run')
    actual = old.output_inventory(PARENT_ROOT, base); require(len(actual) == 205 and sum(n.endswith('.pt') for n in actual) == 18, 'all205 original files and18 PT required')
    manifest = {}
    for line in regular(PARENT_ROOT/'SHA256SUMS').read_text().splitlines():
        value,name = line.split('  ')
        require(name not in manifest and name in actual and name != 'SHA256SUMS' and '..' not in Path(name).parts and not Path(name).is_absolute(), 'invalid original payload manifest')
        manifest[name] = value
    require(manifest == {n:v for n,v in actual.items() if n != 'SHA256SUMS'}, 'original cache-run bytes changed')
    source = source_identity(plan,parent_plan,base,old,repo,args)
    require(report['source_sha256'] == {k:v for k,v in source.items() if not k.startswith('external_residual_')}, 'original executed source chain differs')
    require(all(actual['source/'+n] == value for n,value in report['source_sha256'].items()), 'original source archive changed')
    require([s['seed'] for s in report['seeds']] == list(old.CHECKPOINTS), 'three original cache seeds required')
    for state in report['seeds']:
        require(state['status'] == 'TWO_HEADS_COMPLETE_NOT_PROMOTED' and state['parent_checkpoint_sha256'] == old.CHECKPOINTS[state['seed']]
            and state['generator_state_sha256'] == state['generator_state_after_sha256'], 'original generator/cache proof differs')
        require(all(actual[state['directory']+'/'+n] == value for n,value in state['completed_files'].items()), 'original complete seed inventory differs')
    originals = old.input_receipts(base,oracle,plan)
    require(originals == report['original_input_sha256'], 'original 2870 parents or lineage reports changed')
    return dict(files=actual, original_parent_inputs=originals, source_sha256=source), parent_plan, report


def validate_cache_payload(payload, dataset, seed, state, base, old, frozen_core, torch):
    expected = {'artifact_id','split','digest','sample_ids','episode_ids','source_checkpoint_sha256','tensors','extra'} | set(old.DENIALS)
    require(type(payload) is dict and set(payload) == expected and payload['artifact_id'] == old.CACHE_ID
        and payload['split'] == dataset.split and payload['source_checkpoint_sha256'] == old.CHECKPOINTS[seed]
        and all(payload[n] is False for n in old.DENIALS), 'research cache ID, fields, source or denials differ')
    require(payload['sample_ids'] == [e.token for e in dataset.examples] and payload['episode_ids'] == [e.episode_id for e in dataset.examples], 'cache/source ordered IDs differ')
    require(type(payload['tensors']) is dict and set(payload['tensors']) == set(old.CACHE_FIELDS), 'exact cache tensor fields required')
    require(all(isinstance(t,torch.Tensor) and t.device.type == 'cpu' and not t.requires_grad for t in payload['tensors'].values()), 'secure CPU detached tensor cache required')
    cache = frozen_core.CachedStopmixSelectorData(split=dataset.split,sample_ids=tuple(payload['sample_ids']),episode_ids=tuple(payload['episode_ids']),
        source_checkpoint_sha256=old.CHECKPOINTS[seed],corpus_fingerprint_sha256=base.CORPUS_SHA,
        dataset_fingerprint_sha256=dataset.fingerprint_sha256,**payload['tensors'])
    require(cache.digest() == payload['digest'] == state['cache_digest'][dataset.split], 'cache tensor/ABI/fingerprint digest differs')
    extra = payload['extra'];require(type(extra) is dict and set(extra) == {'route_xy','route_mask'}, 'only original route extras allowed')
    for name,shape,dtype in (('route_xy',(len(dataset),dataset.config.route_points,2),torch.float32),('route_mask',(len(dataset),dataset.config.route_points),torch.bool)):
        value=extra[name];require(isinstance(value,torch.Tensor) and value.device.type=='cpu' and not value.requires_grad
            and tuple(value.shape)==shape and value.dtype==dtype and bool(torch.isfinite(value).all()), 'route cache ABI differs')
    # HH_260906 - Reuse exact frozen tensorization methods; do not decode images or construct the generator to validate these metadata fields.
    for index,example in enumerate(dataset.examples):
        xy,yaw,speed,valid = dataset._targets(example);route,mask=dataset._route(example)
        for actual,expected in ((cache.target_xy[index],xy),(cache.target_yaw_rad[index],yaw),(cache.target_speed_mps[index],speed),
            (cache.target_valid[index],valid),(extra['route_xy'][index],route),(extra['route_mask'][index],mask)):
            require(torch.equal(actual,expected), 'cached target/mask/route differs from original metadata tensorization')
    return cache,extra


def load_cache(dataset,seed,state,snapshot,base,old,oracle,frozen_core,torch,reader):
    path=PARENT_ROOT/f'seed_{seed}/{dataset.split}_cache.pt';name=path.relative_to(PARENT_ROOT).as_posix();expected=snapshot['files'][name]
    require(base.digest(path)==expected,'cache file changed before secure decode')
    payload,observed=reader(path,torch.device('cpu'))
    require(observed==expected==base.digest(path),'securely decoded cache SHA differs')
    cache,extra=validate_cache_payload(payload,dataset,seed,state,base,old,frozen_core,torch)
    journal=PARENT_ROOT/f'seed_{seed}/{dataset.split}_cache_progress.jsonl'
    require(base.digest(journal)==snapshot['files'][journal.relative_to(PARENT_ROOT).as_posix()],'original extraction journal changed')
    identities=[json.loads(line)for line in regular(journal).read_bytes().splitlines()]
    require(len(identities)==len(cache) and [r['index']for r in identities]==list(range(len(cache)))
        and [r['sample_id']for r in identities]==list(cache.sample_ids),'complete original cache journal required')
    for i,row in enumerate(identities):
        tensors={k:getattr(cache,k)[i:i+1]for k in('target_xy','target_speed_mps','target_yaw_rad','target_valid')}
        require(row['target_sha256']==oracle.tensor_sha(tensors,0,tuple(tensors)),'original target journal SHA differs')
    return cache,extra,identities,dict(path=name,sha256=expected,cache_digest=cache.digest(),sample_count=len(cache),
        weights_only=True,map_location='cpu',targets_and_routes_reconstructed_exactly=True,
        fused_candidates_independently_regenerated=False,model_input_sha_recomputed=False)


def run(args,base,old,oracle):
    repo=WORKSPACE/'autoware_e2e';base.check_environment(WORKSPACE,repo)
    for path in (Path(__file__),args.core,args.frozen_worker,args.frozen_core,args.oracle_helper):
        require(regular(path).is_relative_to(WORKSPACE/'benchmarks'),'all external modules must stay outside frozen repository')
    plan_path=regular(args.plan);plan=base.json_read(plan_path);plan_sha=base.digest(plan_path)
    _,projection=oracle.projection_definition(regular(repo/'portable_e2e/losses.py').read_bytes());validate_plan(plan,base,old,oracle,projection)
    dataset_root=(WORKSPACE/base.DATASET).resolve(strict=True)
    require(dataset_root.is_relative_to((WORKSPACE.parent/'dataset').resolve()),'dataset escaped personal storage')
    output=checked_output(args.output_dir,dataset_root,base)
    snapshot,parent_plan,parent_report=parent_snapshot(plan,base,old,oracle,repo,args);before=snapshot['source_sha256']
    report=dict(schema=SCHEMA,status='STARTED',started_at_utc=base.utc(),plan_sha256=plan_sha,source_sha256=before,
        parent_cache_report_sha256=PARENT_REPORT_SHA,parent_cache_manifest_sha256=PARENT_MANIFEST_SHA,loss_projection=projection,seeds=[],**DENIALS)
    loaded={};datasets={};loader=None;caches={};cache_sha={};records=[]
    with base.gpu_lease(WORKSPACE),base.bounded_signals(INTERNAL_TIMEOUT):
        report['gpu_preflight']=base.gpu_idle(repo);output.mkdir(parents=True,exist_ok=False)
        try:
            with (output/'plan.json').open('xb')as stream:stream.write(plan_path.read_bytes())
            external={'external_worker.py':args.frozen_worker,'external_core.py':args.frozen_core,
                'external_oracle_helper.py':args.oracle_helper,'external_owner_helper.py':OWNER_PATH,
                'external_residual_worker.py':Path(__file__),'external_residual_core.py':args.core}
            for name,value in before.items():
                path=external.get(name,repo/name);dest=output/'source'/name;dest.parent.mkdir(parents=True,exist_ok=True)
                with dest.open('xb')as stream:stream.write(regular(path).read_bytes())
                require(base.digest(dest)==value,'executed source archive differs')
            sys.path.insert(0,str(repo));import torch
            from portable_e2e import losses as loss_module
            from portable_e2e.dataset import load_training_examples
            from portable_e2e.model import ModelConfig
            from portable_e2e.torch_dataset import Common10TorchDataset
            from portable_e2e.train import _read_checkpoint_file,_seed_everything
            from scripts.e2e import audit_portable_stopmix_behavior as behavior
            require(torch.cuda.device_count()==1 and torch.get_num_threads()==16,'one GPU and fixed16 CPU threads required')
            report['torch_device_uuid']=base.verify_torch_uuid(torch.cuda.get_device_properties(0));report['torch_version']=str(torch.__version__)
            frozen_core=load_external(args.frozen_core,FROZEN_CORE_SHA,'portable_e2e.frozen_stopmix_selector')
            core=load_external(args.core,plan['residual_core_source_sha256'],'portable_e2e._frozen_stopmix_residual_external')
            require(tuple(core.ARMS)==ARMS,'residual arm definition differs');report['imported_native_modules']=old.verify_import_paths(repo)
            projected,proof=oracle.projected_costs(loss_module);require(proof==projection,'original loss projection differs')
            _seed_everything(0,torch.device('cuda:0'));config=ModelConfig.from_mapping(base.json_read(repo/old.MODEL_PATH))
            loader=load_training_examples
            for split in old.SPLITS:
                loaded[split]=loader(dataset_root,split=split,mode='planning',check_image_hashes=True)
                require(loaded[split].validation_report['dataset_fingerprint_sha256']==base.CORPUS_SHA,'whole corpus changed')
                datasets[split]=Common10TorchDataset(loaded[split].examples,config,verify_image_sha256=True,split=split)
            old.validate_datasets(datasets,base)
            for seed,parent in old.CHECKPOINTS.items():
                require(parent_snapshot(plan,base,old,oracle,repo,args)[0]==snapshot,'original cache/source/parent changed')
                item=output/f'seed_{seed}';item.mkdir();state=dict(seed=seed,directory=item.name,status='STARTED',parent_checkpoint_sha256=parent,cache_proofs={},arms=[])
                report['seeds'].append(state);records.append(state);caches={};cache_sha={};extras={};identities={};baselines={}
                parent_state=next(s for s in parent_report['seeds']if s['seed']==seed)
                for split in old.SPLITS:
                    with old.stage_alarm(CACHE_TIMEOUT):
                        cache,extras[split],identities[split],state['cache_proofs'][split]=load_cache(datasets[split],seed,parent_state,snapshot,base,old,oracle,frozen_core,torch,_read_checkpoint_file)
                    caches[split]=cache.to('cuda:0')
                cache_sha={k:v.digest()for k,v in caches.items()};state['cache_digest']=cache_sha
                with old.stage_alarm(AUDIT_TIMEOUT):
                    for split in old.SPLITS:
                        baselines[split]=old.audit_cache(caches[split],extras[split],datasets[split],identities[split],caches[split].original_logits,
                            item/'original'/split,oracle,behavior,loss_module,projected,torch)
                        original=base.json_read(PARENT_ROOT/f'seed_{seed}/original/{split}/summary.json')
                        require(baselines[split]['oracle_and_cost_sha256']==original['oracle_and_cost_sha256'],'cached original composite oracle differs from previous execution')
                state['completed_files']=old.output_inventory(item,base);order_sha=None;initial_sha=None
                for arm in ARMS:
                    require(source_identity(plan,parent_plan,base,old,repo,args)==before and old.completed_outputs_intact(output,records,base),'source or prior output changed between heads')
                    arm_dir=item/arm;arm_dir.mkdir();arm_state=dict(arm=arm,status='STARTED');state['arms'].append(arm_state)
                    with (arm_dir/'metrics.jsonl').open('x')as history,old.stage_alarm(HEAD_TIMEOUT):
                        def check_deadline():require(datetime.now(timezone.utc)<base.timestamp(plan['finish_before_utc']),'user deadline reached')
                        def progress(row):
                            check_deadline();history.write(encode(row).decode());history.flush()
                            if row.get('global_step',0)%154==0:print(json.dumps(dict(event='RESIDUAL_HEAD_PROGRESS',seed=seed,arm=arm,record=row)),flush=True)
                        result=core.fit_frozen_residual(caches['train'],caches['val'],arm=arm,seed=int(seed),device='cuda:0',callback=progress,check_deadline=check_deadline)
                    old.verify_history(arm_dir/'metrics.jsonl',result['report']);old.verify_import_paths(repo)
                    require({k:v.digest()for k,v in caches.items()}==cache_sha,'frozen cache changed during residual training')
                    head=result['report'];require(head['head_parameter_count']==HEAD_PARAMETERS and head['final_state']['global_step']==1540
                        and head['final_state']['samples_seen']==6155,'fixed matched head capacity/exposure required')
                    if order_sha is None:order_sha=head['sampling_order_sha256'];initial_sha=head['initial_head_sha256']
                    require(head['sampling_order_sha256']==order_sha and head['initial_head_sha256']==initial_sha,'paired residual heads differ in initialization or sample order')
                    with old.stage_alarm(AUDIT_TIMEOUT):
                        for split in old.SPLITS:
                            require(torch.equal(result['baseline_logits_cpu'][split],caches[split].original_logits.cpu()),'zero residual failed original logit equality')
                            measured=old.audit_cache(caches[split],extras[split],datasets[split],identities[split],result['final_logits_cpu'][split],
                                arm_dir/'analysis'/split,oracle,behavior,loss_module,projected,torch)
                            require(measured['oracle_and_cost_sha256']==baselines[split]['oracle_and_cost_sha256']
                                and measured['oracle_indices']==result['oracle_indices_cpu'][split].tolist(),'fixed original candidates/costs/oracles changed')
                    old.save_tensors(arm_dir/'head_only.pt',dict(artifact_id=head['artifact_id'],head_state_dict=result['head_state_dict'],
                        parent_checkpoint_sha256=parent,parent_cache_report_sha256=PARENT_REPORT_SHA,**DENIALS),torch)
                    old.save_tensors(arm_dir/'final_logits.pt',dict(artifact_id='portable_e2e.frozen_stopmix_residual_logits.research.v1',logits=result['final_logits_cpu'],**DENIALS),torch)
                    base.write_json(arm_dir/'report.json',head)
                    require(old.completed_outputs_intact(output,records,base),'earlier completed residual result changed')
                    arm_state.update(status='HEAD_COMPLETE_NOT_PROMOTED',completed_at_utc=base.utc(),files=old.output_inventory(arm_dir,base))
                    state['completed_files']=old.output_inventory(item,base)
                require({k:v.digest()for k,v in caches.items()}==cache_sha,'final frozen cache changed')
                state.update(status='TWO_RESIDUAL_HEADS_COMPLETE_NOT_PROMOTED',completed_at_utc=base.utc())
                del result,cache,caches;caches={};torch.cuda.empty_cache()
            require(len(report['seeds'])==3 and all(s['status']=='TWO_RESIDUAL_HEADS_COMPLETE_NOT_PROMOTED'for s in report['seeds']),'all six matched residual fits required')
            report['status']='SIX_RESIDUAL_HEAD_FITS_COMPLETE_NOT_PROMOTED'
        except BaseException as error:report.update(status='FAILED_OR_PARTIAL_NOT_PROMOTED',error_type=type(error).__name__,error=str(error))
        finally:
            with base.finalization_signals():
                errors=[]
                checks={'source':lambda:source_identity(plan,parent_plan,base,old,repo,args)==before,
                    'all_original_205_files_18_pt_and_parent_sources':lambda:parent_snapshot(plan,base,old,oracle,repo,args)[0]==snapshot,
                    'source_archives':lambda:all(base.digest(output/'source'/n)==v for n,v in before.items()),
                    'plan':lambda:base.digest(plan_path)==plan_sha==base.digest(output/'plan.json'),
                    'completed_outputs':lambda:old.completed_outputs_intact(output,records,base),
                    'manifest':lambda:base.digest(dataset_root/'dataset.json')==base.MANIFEST_SHA}
                if caches and cache_sha:checks['active_cache']=lambda:{k:v.digest()for k,v in caches.items()}==cache_sha
                if 'imported_native_modules'in report:checks['native_import_paths']=lambda:old.verify_import_paths(repo)==report['imported_native_modules']
                if loader is not None:
                    for split in loaded:
                        def corpus(split=split):
                            after=loader(dataset_root,split=split,mode='planning',check_image_hashes=True)
                            return after.validation_report['dataset_fingerprint_sha256']==base.CORPUS_SHA and after.fingerprint_sha256==loaded[split].fingerprint_sha256
                        checks['whole_corpus_'+split]=corpus
                for name,operation in checks.items():
                    try:require(operation(),name+' changed')
                    except BaseException as error:errors.append(dict(check=name,error_type=type(error).__name__,error=str(error)))
                report.update(completed_at_utc=base.utc(),postcheck_errors=errors,source_and_inputs_unchanged=not errors,
                    completed_head_fit_count=sum(a['status']=='HEAD_COMPLETE_NOT_PROMOTED'for s in report['seeds']for a in s['arms']),
                    original_cache_run_file_sha256=snapshot['files'],original_parent_input_sha256=snapshot['original_parent_inputs'],
                    deadline_met=datetime.now(timezone.utc)<base.timestamp(plan['finish_before_utc']))
                if errors or not report['deadline_met']:report['status']='FAILED_OR_PARTIAL_NOT_PROMOTED'
                base.write_json(output/'report.json',report);inventory=old.output_inventory(output,base)
                with (output/'SHA256SUMS').open('x')as stream:
                    for name,value in inventory.items():stream.write(value+'  '+name+'\n')
    return 0 if report['status']=='SIX_RESIDUAL_HEAD_FITS_COMPLETE_NOT_PROMOTED' else 2


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in('plan','core','frozen-worker','frozen-core','oracle-helper','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args(argv)
    base=load_external(OWNER_PATH,OWNER_SHA,'_residual_owner_helper')
    old=load_external(args.frozen_worker,FROZEN_WORKER_SHA,'_residual_frozen_worker')
    oracle=load_external(args.oracle_helper,ORACLE_SHA,'_residual_oracle_helper')
    return run(args,base,old,oracle)


if __name__=='__main__':raise SystemExit(main())
