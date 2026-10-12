"""HH_260906 - Bounded RGB semantic learning in the personal GPU0 environment only."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--frozen-repo',type=Path,required=True)
    p.add_argument('--data',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--wait-seconds',type=int,default=3600)
    p.add_argument('--validate-only',action='store_true')
    p.add_argument('--loss-mode',choices=('sqrt_frequency','class_balanced'),default='sqrt_frequency')
    a=p.parse_args()
    if not 0<=a.wait_seconds<=7200:raise ValueError('wait budget must be 0..7200 seconds')
    sys.path.insert(0,str(a.frozen_repo));sys.path.insert(0,str(Path(__file__).resolve().parent))
    import numpy as np
    import torch
    from torch.nn import functional as F
    from map_perception import MapPerception,LABELS,semantic_loss
    from map_learning_balance import class_weights
    from scripts.e2e import run_moving_multiroute_recovery_k as r
    workspace=r.check_run_environment(dict(repo_root=str(a.frozen_repo),output_root=str(a.output)))
    if a.output.exists():raise ValueError('fresh output required')
    if not a.data.resolve().is_relative_to(workspace.parent):raise ValueError('personal data only')
    manifest=json.loads((a.data/'manifest.json').read_text())
    if tuple(manifest['labels'])!=LABELS:raise ValueError('label schema differs')
    loaded={}
    for split in ('train','dev'):
        ref=manifest['splits'][split];path=a.data/ref['file']
        if not path.resolve().is_relative_to(a.data.resolve()) or r.digest(path)!=ref['sha256']:
            raise ValueError('dataset hash/path differs')
        with np.load(path,allow_pickle=False) as d:
            loaded[split]=(d['rgb'].copy(),d['target'].copy())
        if loaded[split][0].shape!=(ref['count'],180,320,3) or loaded[split][1].shape!=(ref['count'],180,320):
            raise ValueError('shape differs')
        if loaded[split][0].dtype!=np.uint8 or loaded[split][1].dtype!=np.uint8:
            raise ValueError('uint8 data required')
        if int(loaded[split][1].max())>=len(LABELS):raise ValueError('invalid label')
    if a.validate_only:
        print('MAPPING_DATA_VALIDATED_CPU_ONLY',flush=True);return
    # HH_260906 - Wait only on the shared personal lease; never evict another job.
    lease=None;deadline=time.monotonic()+a.wait_seconds
    while True:
        try:
            candidate=r.gpu0_lease(workspace);candidate.__enter__();lease=candidate;break
        except ValueError as error:
            if 'already leased' not in str(error) or time.monotonic()>=deadline:raise
            print('WAITING_FOR_PERSONAL_GPU0_LEASE',flush=True);time.sleep(30)
    try:
        # HH_260906 - Allow utilization accounting to settle after the previous owned job exits.
        idle_deadline=time.monotonic()+90
        while True:
            try:
                gpu=r.preflight_gpu();break
            except ValueError as error:
                if 'GPU0 must be idle' not in str(error) or time.monotonic()>=idle_deadline:raise
                time.sleep(5)
        device=torch.device('cuda:0')
        r._seed_everything(20260922,device)
        a.output.mkdir(parents=True)
        for path in (Path(__file__),Path(__file__).with_name('map_perception.py'),Path(__file__).with_name('map_learning_balance.py')):
            r.copy_new(path,a.output/'source'/path.name)
        r.copy_new(a.data/'manifest.json',a.output/'data_manifest.json')
        r.write_new(a.output/'started.json',dict(gpu=gpu,started_at_utc=r.utc_now(),steps=2000,batch_size=8,
            model_input='RGB only',new_map_model=True,loss_mode=a.loss_mode,source_sha256=r.digest(Path(__file__)),
            data_manifest_sha256=r.digest(a.data/'manifest.json'),automatic_promotion=False))
        model=MapPerception().to(device)
        optimizer=torch.optim.AdamW(model.parameters(),lr=3e-4,weight_decay=1e-4)
        x,y=loaded['train'];pixel=np.bincount(y.ravel(),minlength=len(LABELS))
        weights=class_weights(pixel,a.loss_mode)
        r.write_new(a.output/'loss_contract.json',dict(mode=a.loss_mode,train_class_pixels=pixel.tolist(),
            normalized_class_weights=weights.tolist(),dev_used_for_weights=False,
            changes='loss class weights only; same architecture, seed, steps, sampler and dataset'))
        weights=torch.tensor(weights,dtype=torch.float32,device=device)
        sampler=np.random.default_rng(20260922);start=time.monotonic()
        model.train()
        with (a.output/'history.jsonl').open('x') as log:
            for step in range(1,2001):
                ids=sampler.integers(0,len(x),size=8)
                images=torch.tensor(x[ids].transpose(0,3,1,2).copy(),device=device,dtype=torch.float32)/255
                target=torch.tensor(y[ids].astype(np.int64),device=device)
                optimizer.zero_grad(set_to_none=True)
                loss=semantic_loss(model(images),target,weights)
                if not bool(torch.isfinite(loss)):raise ValueError('nonfinite loss')
                loss.backward()
                grad=torch.nn.utils.clip_grad_norm_(model.parameters(),5.)
                if not bool(torch.isfinite(grad)):raise ValueError('nonfinite gradients')
                optimizer.step()
                if step%100==0:
                    row=dict(step=step,loss=float(loss.detach()))
                    log.write(json.dumps(row)+'\n');log.flush();print(json.dumps(row),flush=True)
        r.save_new(a.output/'checkpoint.pt',dict(model_state_dict=model.state_dict(),labels=LABELS,
            model_id='portable_e2e.map_semantics.pilot.v1',optimizer_state_dict=optimizer.state_dict(),
            steps=2000,input_shape=[3,180,320],source_kind=manifest['source_kind'],deployable=False))
        model.eval();confusion=np.zeros((len(LABELS),len(LABELS)),dtype=np.int64)
        x,y=loaded['dev'];predictions=[]
        with torch.no_grad():
            for begin in range(0,len(x),8):
                images=torch.tensor(x[begin:begin+8].transpose(0,3,1,2).copy(),device=device,dtype=torch.float32)/255
                pred=model(images).argmax(1).cpu().numpy().astype(np.uint8);predictions.append(pred)
                confusion+=np.bincount((y[begin:begin+8].astype(np.int64)*len(LABELS)+pred).ravel(),
                                       minlength=len(LABELS)**2).reshape(len(LABELS),len(LABELS))
        union=confusion.sum(0)+confusion.sum(1)-np.diag(confusion)
        ious={name:float(confusion[i,i]/union[i]) if union[i] else None for i,name in enumerate(LABELS)}
        np.savez_compressed(a.output/'dev_predictions.npz',prediction=np.concatenate(predictions))
        r.write_new(a.output/'summary.json',dict(status='PILOT_COMPLETE_NOT_PROMOTED',iou=ious,
            mean_iou_present=float(np.mean([v for v in ious.values() if v is not None])),
            pixel_accuracy=float(np.trace(confusion)/confusion.sum()),confusion=confusion.tolist(),
            majority_class_baseline_pixel_accuracy=float((y==int(pixel.argmax())).mean()),
            wall_seconds=time.monotonic()-start,model_parameters=sum(p.numel() for p in model.parameters()),
            steps=2000,loss_mode=a.loss_mode,checkpoint_sha256=r.digest(a.output/'checkpoint.pt'),
            limitations='single Town/weather; only two physical routes; no independent TEST, no metric-map or traffic-rule approval',
            deployment_approved=False,completed_at_utc=r.utc_now()))
        print('MAPPING_PILOT_COMPLETE_NOT_PROMOTED',flush=True)
    finally:
        if lease is not None:lease.__exit__(None,None,None)


if __name__=='__main__':main()
