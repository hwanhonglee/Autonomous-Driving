#!/usr/bin/env python3
"""HH_261001 - Fit two fresh, exposure-matched geometry-source endpoints on personal GPU0 only."""

import argparse
from contextlib import contextmanager
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time
import numpy as np

sys.dont_write_bytecode = True
try:
    from . import map_route_geometry_contract_20261001 as c
except ImportError:
    import map_route_geometry_contract_20261001 as c
base,package = c.base,c.package
WORKSPACE = package.PERSONAL/'portable_e2e'
SOURCE = WORKSPACE/'source_worktrees'/c.NAME
OUTPUT = WORKSPACE/'runs/diagnostics'/c.NAME
EVAL_DATA = package.PERSONAL/'dataset/map_town02_p00_route_holdout_20260929_v1'
GPU_UUID = 'GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5'
EVAL_MANIFEST_SHA = 'f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834'
EVAL_FILES = {'dev_town01':(118,'417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3'),
    'holdout_town02_p02':(232,'858f92ccd874c154576d3cf6a85132508801a22671baf6d0c07abe3936eb16cb')}


def environment_contract(validate_only):
    base.require(os.uname().nodename == 'pro6000' and Path(sys.prefix) == WORKSPACE/'venvs/py312',
                 'Exact personal host and existing py312 venv required')
    base.require(Path.cwd() == SOURCE and Path(__file__).resolve().parent == SOURCE,'Exact frozen source directory required')
    required = {'PYTHONNOUSERSITE':'1','OMP_NUM_THREADS':'16','MKL_NUM_THREADS':'16'}
    if validate_only:
        base.require(os.environ.get('CUDA_VISIBLE_DEVICES') == '', 'CPU-only preflight must hide all GPUs')
    else:
        required.update({'CUDA_VISIBLE_DEVICES':GPU_UUID,'CUBLAS_WORKSPACE_CONFIG':':4096:8'})
    base.require(all(os.environ.get(k) == v for k,v in required.items()),'Fixed process environment differs')
    base.require(not any(os.environ.get(k) for k in ('PYTHONPATH','PYTHONHOME','PYTHONSTARTUP','PYTHONUSERBASE')),
                 'Python import environment override forbidden')
    for p in (WORKSPACE,SOURCE,package.PACKAGE,package.REFERENCE,EVAL_DATA,OUTPUT.parent):
        base.require(p.is_dir() and p.resolve() == p, 'Canonical personal directory required: '+str(p))
    base.require(not OUTPUT.exists() and not OUTPUT.is_symlink(),'Fresh run required; no resume or overwrite')


@contextmanager
def bounded_phase(seconds,label):
    """HH_261001 - Bound owned Python work; never signal another job or select partial endpoints."""
    def expired(signum,frame):
        raise TimeoutError(label+' wall-time budget exhausted')
    old = signal.signal(signal.SIGALRM,expired)
    signal.setitimer(signal.ITIMER_REAL,seconds)
    try:
        yield time.monotonic()+seconds
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        signal.signal(signal.SIGALRM,old)


def deadline_check(deadline):
    if time.monotonic() >= deadline:
        raise TimeoutError('Owned phase wall-time budget exhausted')


def load_weights(model,path,expected_sha):
    import torch
    base.require(base.digest(path) == expected_sha,'Learned weights changed')
    template = model.state_dict()
    with np.load(path,allow_pickle=False) as data:
        base.require(set(data.files) == set(template),'Safe learned-tensor ABI keys differ')
        state = {}
        for name,tensor in template.items():
            a = data[name]
            base.require(a.shape == tuple(tensor.shape) and a.dtype == np.float32 and np.isfinite(a).all(),
                         'Safe learned-tensor shape/dtype/finite check failed')
            state[name] = torch.from_numpy(a.copy())
    model.load_state_dict(state,strict=True)
    base.require(all(p.requires_grad for p in model.parameters()),'Unexpected frozen parameter')


def save_tensor_npz(model,path):
    values = {name:t.detach().cpu().contiguous().numpy() for name,t in model.state_dict().items()}
    base.require(all(a.dtype == np.float32 and np.isfinite(a).all() for a in values.values()),'Nonfinite endpoint tensor')
    with path.open('xb') as stream:
        np.savez_compressed(stream,**values)
    return {name:{'shape':list(a.shape),'dtype':str(a.dtype),'sha256':hashlib.sha256(a.tobytes()).hexdigest()}
            for name,a in values.items()}


def metrics(confusion):
    truth,pred = confusion.sum(1),confusion.sum(0)
    tp = np.diag(confusion)
    union = truth+pred-tp
    return {'confusion':confusion.tolist(),'target_pixels':dict(zip(base.LABELS,truth.tolist())),
        'predicted_pixels':dict(zip(base.LABELS,pred.tolist())),
        'false_positive_pixels':dict(zip(base.LABELS,(pred-tp).tolist())),
        'iou':{label:float(tp[i]/union[i]) if truth[i] else None for i,label in enumerate(base.LABELS)},
        'pixel_accuracy':float(tp.sum()/confusion.sum()),'absent_target_class_iou':'null; false positives retained'}


def validate_endpoint_pair(arms,output):
    base.require(set(arms) == set(c.ARMS),'Both fixed endpoints required before opening DEV')
    for arm,info in arms.items():
        base.require(info['steps'] == 10000 and info['complete'] is True
            and base.digest(output/arm/'weights.npz') == info['weights_sha256']
            and base.digest(output/arm/'checkpoint.pt') == info['checkpoint_sha256'],
            'Endpoint incomplete/changed; DEV must stay unopened')


def fit_arm(arm,arrays,indices,crops,replay,plan,model_class,guard,progress):
    import torch
    from map_perception import semantic_loss
    root = OUTPUT/arm
    root.mkdir()
    guard.seed_everything(base.SEED)
    model = model_class()
    load_weights(model,package.REFERENCE/'parent_weights.npz',base.PARENT_SHA)
    initial_sha = guard.state_digest(model)
    base.require(sum(p.numel() for p in model.parameters()) == 245831,'Architecture size differs')
    model.to('cuda:0').train()
    optimizer = torch.optim.AdamW(model.parameters(),lr=base.LR,weight_decay=base.WEIGHT_DECAY)
    base.require(len(optimizer.state) == 0,'Optimizer must start empty')
    weight = torch.tensor(c.CLASS_WEIGHTS,dtype=torch.float32,device='cuda:0')
    rng,photo_trace = np.random.default_rng(20261003),c.PhotoTrace()
    samples_seen,crops_seen = hashlib.sha256(),hashlib.sha256()
    actual_slots = np.zeros(1689,np.int64)
    completed = 0
    started = time.monotonic()
    progress.update(stage='fit',arm=arm,steps_completed=0)
    try:
        with bounded_phase(c.FIT_SECONDS,arm+' fitting/final-save') as deadline:
            total = 0.
            with (root/'history.jsonl').open('x') as log:
                for step in range(10000):
                    deadline_check(deadline)
                    ids,boxes = indices[step],crops[step]
                    rgb,target = c.batch_arrays(arrays,ids,boxes,arm,replay)
                    rgb = c.augment_both_arms(rgb,ids,rng,guard.photometric,photo_trace)
                    x = torch.tensor(rgb.transpose(0,3,1,2).copy(),dtype=torch.float32,device='cuda:0')/255
                    y = torch.tensor(target.astype(np.int64),device='cuda:0')
                    optimizer.zero_grad(set_to_none=True)
                    loss = semantic_loss(model(x),y,weight)
                    base.require(bool(torch.isfinite(loss)),'Nonfinite training loss')
                    loss.backward()
                    norm = torch.nn.utils.clip_grad_norm_(model.parameters(),5.)
                    base.require(bool(torch.isfinite(norm)),'Nonfinite training gradient')
                    optimizer.step()
                    completed = step+1
                    progress['steps_completed'] = completed
                    samples_seen.update(ids.tobytes()); crops_seen.update(boxes.tobytes())
                    actual_slots += np.bincount(ids,minlength=1689)
                    total += float(loss.detach())
                    if completed%100 == 0:
                        row = {'arm':arm,'step':completed,'mean_loss_100':total/100,'wall_seconds':time.monotonic()-started}
                        log.write(json.dumps(row)+'\n');log.flush()
                        print(json.dumps(row),flush=True)
                        total = 0.
            torch.cuda.synchronize()
            tensors = save_tensor_npz(model,root/'weights.npz')
            checkpoint = {'model_state_dict':model.state_dict(),'optimizer_state_dict':optimizer.state_dict(),
                'steps':completed,'study_id':c.NAME,'arm':arm,'labels':base.LABELS,
                'parent_weights_sha256':base.PARENT_SHA,'deployable':False,'dev_used_for_fit':False}
            with (root/'checkpoint.pt').open('xb') as stream:
                torch.save(checkpoint,stream)
            exposure = c.exposure_receipt(indices,replay,arm)
            base.require(exposure['slot_exposures'] == actual_slots.tolist(),'Actual slot counts differ')
            info = {'complete':True,'steps':completed,'parameter_count':245831,'initial_state_sha256':initial_sha,
                'final_state_sha256':guard.state_digest(model),'training_wall_seconds':time.monotonic()-started,
                'sample_sequence_sha256':samples_seen.hexdigest(),'crop_sequence_sha256':crops_seen.hexdigest(),
                'weights_sha256':base.digest(root/'weights.npz'),'checkpoint_sha256':base.digest(root/'checkpoint.pt'),
                'optimizer_initial_state_entries':0,'final_tensors':tensors,'photo_trace':photo_trace.receipt(rng),
                'common_class_weights':c.CLASS_WEIGHTS,'exposure':exposure}
            # HH_261001 - Preserve actual receipts even when an exact schedule/trace guard fails.
            guard.write_new(root/'training.json',info)
            expected_photo = {k:plan['photometric'][k] for k in info['photo_trace']}
            base.require(info['photo_trace'] == expected_photo,'Actual common photometric trace differs')
            base.require(info['sample_sequence_sha256'] == plan['schedule']['indices_int64_sha256']
                and info['crop_sequence_sha256'] == plan['schedule']['crops_int16_sha256'],'Actual sample/crop sequence differs')
            deadline_check(deadline)
        return info
    except BaseException as error:
        failed = {'status':'FAILED_PARTIAL_NOT_AN_ENDPOINT','arm':arm,'steps_completed':completed,
                  'error':repr(error),'wall_seconds':time.monotonic()-started,'dev_opened':False,
                  'completed_sample_sequence_sha256':samples_seen.hexdigest(),
                  'completed_crop_sequence_sha256':crops_seen.hexdigest(),
                  'completed_slot_exposures':actual_slots.tolist(),
                  'photo_trace_including_attempted_batch':photo_trace.receipt(rng)}
        try:
            with bounded_phase(60,arm+' failed-state preservation'):
                failed['partial_tensors'] = save_tensor_npz(model,root/'aborted_state.npz')
                failed['aborted_state_sha256'] = base.digest(root/'aborted_state.npz')
        except BaseException as save_error:
            failed['partial_save_error'] = repr(save_error)
        guard.write_new(root/'failure.json',failed)
        raise
    finally:
        del model,optimizer
        torch.cuda.empty_cache()


def evaluate_endpoints(arms,model_class,guard,progress):
    import torch
    validate_endpoint_pair(arms,OUTPUT)
    base.require(base.digest(EVAL_DATA/'manifest.json') == EVAL_MANIFEST_SHA,'Known split manifest changed')
    result = {split:{} for split in EVAL_FILES}
    for arm,info in arms.items():
        progress.update(stage='known_evaluation',arm=arm)
        with bounded_phase(c.EVALUATION_SECONDS,arm+' known evaluation') as deadline:
            model = model_class()
            load_weights(model,OUTPUT/arm/'weights.npz',info['weights_sha256'])
            model.to('cuda:0').eval()
            for split,(count,sha) in EVAL_FILES.items():
                path = EVAL_DATA/(split+'.npz')
                base.require(base.digest(path) == sha,'Known evaluation bytes changed')
                with np.load(path,allow_pickle=False) as data:
                    base.require(set(data.files) == {'rgb','target'},'Known split ABI keys')
                    rgb,target = data['rgb'],data['target']
                base.require(rgb.shape == (count,360,640,3) and target.shape == (count,360,640)
                    and rgb.dtype == target.dtype == np.uint8 and int(target.max()) < 7,'Known split ABI')
                prediction = np.empty_like(target)
                confusion = np.zeros((7,7),np.int64)
                with torch.no_grad():
                    for begin in range(0,count,8):
                        deadline_check(deadline)
                        x = torch.tensor(rgb[begin:begin+8].transpose(0,3,1,2).copy(),dtype=torch.float32,device='cuda:0')/255
                        pred = model(x).argmax(1).cpu().numpy().astype(np.uint8)
                        prediction[begin:begin+len(pred)] = pred
                        confusion += np.bincount((target[begin:begin+len(pred)].astype(np.int64)*7+pred).ravel(),minlength=49).reshape(7,7)
                with (OUTPUT/arm/(split+'_predictions.npz')).open('xb') as stream:
                    np.savez_compressed(stream,prediction=prediction)
                result[split][arm] = {'count':count,'source_sha256':sha,**metrics(confusion)}
                base.require(base.digest(path) == sha,'Known split changed during inference')
            del model
            torch.cuda.empty_cache()
    guard.write_new(OUTPUT/'known_evaluation.json',result)
    return result


def run(args):
    environment_contract(args.validate_only)
    bundle = c.source_binding(SOURCE,args.expected_source_sha256)
    manifest,reference,plan,checks,arrays,indices,crops,replay = c.load_arrays(package.PACKAGE,package.REFERENCE,args.expected_data_sha256)
    receipt = {'study_id':c.NAME,'source_bundle_sha256':args.expected_source_sha256,
        'prepared_manifest_sha256':c.DATA_SHA,'reference_plan_sha256':c.PLAN_SHA,
        'package_authorization_sha256':c.PACKAGE_APPROVAL_SHA,'parent_weights_sha256':base.PARENT_SHA,
        'common_class_weights':c.CLASS_WEIGHTS,'class_weight_provenance':'unchanged preceding original729+paired480 target-frequency weights',
        'schedule':plan['schedule'],'photometric':plan['photometric'],'fresh_fits':2,'steps_per_arm':10000,
        'batch':8,'learning_rate':base.LR,'weight_decay':base.WEIGHT_DECAY,'gradient_clip_norm':5.,
        'fit_plus_known_eval_budget_seconds_per_arm':c.FIT_SECONDS+c.EVALUATION_SECONDS,
        'same_architecture':True,'sole_factor':'extraFALSE480 old Town01 replay versus full p02/p04 route sources',
        'ctrack_train_eligible':False,'broad_registry_modified':False,'model_promotion':False}
    if args.validate_only:
        print(json.dumps({'status':'CPU_PREFLIGHT_PASS_NO_FIT',**receipt}),flush=True)
        return
    fit_approval = package.PACKAGE/'fit_authorization.json'
    base.require(args.expected_fit_approval_sha256 and base.digest(fit_approval) == args.expected_fit_approval_sha256,
                 'Separate fit approval missing/changed')
    c.validate_fit_authorization(base.read_json(fit_approval),c.DATA_SHA,args.expected_source_sha256)
    receipt['fit_authorization_sha256'] = args.expected_fit_approval_sha256
    checks[fit_approval] = args.expected_fit_approval_sha256
    import torch
    import train_map_photometric_pilot as guard
    from map_context_recovery import MapContextRecovery
    torch.set_num_threads(16)
    torch.set_num_interop_threads(1)
    progress = {'stage':'prelaunch'}
    with guard.gpu0_lease():
        gpu = guard.preflight_gpu()
        OUTPUT.mkdir()
        try:
            for name in (*bundle['files'],'bundle.json'):
                guard.copy_new(SOURCE/name,OUTPUT/'source'/name)
            for src,dst in [(package.PACKAGE/'prepared/manifest.json','prepared_manifest.json'),
                (package.PACKAGE/'reference_plan.json','reference_plan.json'),
                (package.PACKAGE/'candidate_manifest.json','candidate_manifest.json'),
                (package.PACKAGE/'package_authorization.json','package_authorization.json'),
                (fit_approval,'fit_authorization.json'),(package.REFERENCE/'manifest.json','reference_manifest.json')]:
                guard.copy_new(src,OUTPUT/'inputs'/dst)
            guard.write_new(OUTPUT/'started.json',{**receipt,'gpu':gpu,'created_at_utc':datetime.now(timezone.utc).isoformat()})
            arms = {}
            for arm in c.ARMS:
                arms[arm] = fit_arm(arm,arrays,indices,crops,replay,plan,MapContextRecovery,guard,progress)
            base.require(arms['control']['initial_state_sha256'] == arms['geometry']['initial_state_sha256'],
                         'Matched initialization differs')
            validate_endpoint_pair(arms,OUTPUT)
            guard.write_new(OUTPUT/'fit_complete.json',{'status':'BOTH_FIXED_ENDPOINTS_SAVED_BEFORE_DEV','arms':arms})
            evaluated = evaluate_endpoints(arms,MapContextRecovery,guard,progress)
            c.source_binding(SOURCE,args.expected_source_sha256)
            for path,sha in checks.items():
                base.require(base.digest(path) == sha,'Source/data/authority changed during fitting')
            result = {'status':'GEOMETRY_SOURCE_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED',**receipt,
                'arms':arms,'known_evaluation':evaluated,'source_and_data_unchanged':True,'dev_used_during_optimization':False}
            guard.write_new(OUTPUT/'summary.json',result)
            print(json.dumps({'status':result['status'],'output':str(OUTPUT)}),flush=True)
        except BaseException as error:
            guard.write_new(OUTPUT/'failure.json',{'status':'FAILED_PRESERVED_NOT_PROMOTED','error':repr(error),
                'progress':progress,'existing_endpoints_not_selected':True,'no_resume':True,'receipt':receipt})
            raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--expected-source-sha256',required=True)
    parser.add_argument('--expected-data-sha256',required=True)
    parser.add_argument('--expected-fit-approval-sha256')
    parser.add_argument('--validate-only',action='store_true')
    run(parser.parse_args())
