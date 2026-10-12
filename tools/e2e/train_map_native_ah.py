"""HH_260906 - Native RGB resolution plus TRAIN-only zoom; no map or driving approval."""
import argparse
import json
from pathlib import Path
import sys
import time


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--frozen-repo',type=Path,required=True)
    p.add_argument('--data',type=Path,required=True)
    p.add_argument('--parent',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--validate-only',action='store_true')
    # HH_260906 - AI is an additive TRAIN data experiment; defaults preserve the archived AH protocol.
    p.add_argument('--expanded-train',action='store_true')
    # HH_260906 - A same-data sampler ablation must not be combined with dataset expansion.
    p.add_argument('--small-sign-exposure',action='store_true')
    a=p.parse_args()
    sys.path.insert(0,str(a.frozen_repo));sys.path.insert(0,str(Path(__file__).resolve().parent))
    import numpy as np
    import torch
    from PIL import Image
    from map_perception import MapPerception,LABELS,semantic_loss
    from map_learning_balance import class_weights,SignScaleStratifiedSampler
    from dual_recovery_objectives import paired_crop
    from scripts.e2e import run_moving_multiroute_recovery_k as r
    workspace=r.check_run_environment(dict(repo_root=str(a.frozen_repo),output_root=str(a.output)))
    r.require(not a.output.exists(),'fresh output required')
    r.require(a.data.resolve().is_relative_to(workspace.parent),'personal data only')
    manifest=json.loads((a.data/'manifest.json').read_text())
    parent_summary=r.read_json(a.parent/'summary.json')
    parent_checkpoint=a.parent/'checkpoint.pt'
    r.require(r.digest(parent_checkpoint)==parent_summary['checkpoint_sha256'],'parent checkpoint changed')
    old_manifest=r.read_json(a.parent/'data_manifest.json')
    r.require(not (a.small_sign_exposure and a.expanded_train),'sampler and data changes need separate experiments')
    if a.small_sign_exposure:
        # HH_260906 - Bind this A/B to the exact parent TRAIN and DEV dataset, not just matching record names.
        r.require(r.digest(a.data/'manifest.json')==r.digest(a.parent/'data_manifest.json'),
                  'small-sign A/B requires exactly the parent dataset manifest')
    if a.expanded_train:
        from map_expansion_contract import validate_expansion
        validate_expansion(manifest,old_manifest,r.digest(a.parent/'data_manifest.json'))
    r.require(tuple(manifest['labels'])==LABELS and manifest['image_size_wh']==[640,360],'native label schema required')
    loaded={}
    for split in ('train','dev'):
        ref=manifest['splits'][split];path=a.data/ref['file']
        r.require(path.resolve().is_relative_to(a.data.resolve()) and r.digest(path)==ref['sha256'],'data hash/path changed')
        # HH_260906 - Same raw files and records keep resolution comparisons from hiding changed splits.
        if not a.expanded_train or split=='dev':
            for key in ('source_records','count'):
                r.require(ref[key]==old_manifest['splits'][split][key],'raw split differs from original pilot')
        # HH_260906 - Root aliases were retired; exact raw byte hashes, not stale path spellings, bind this comparison.
            r.require(sorted(ref['source_hashes'].values())==sorted(old_manifest['splits'][split]['source_hashes'].values()),
                      'raw source bytes differ from original pilot')
        with np.load(path,allow_pickle=False) as d: x,y=d['rgb'].copy(),d['target'].copy()
        r.require(x.shape==(ref['count'],360,640,3) and y.shape==(ref['count'],360,640),'shape differs')
        r.require(x.dtype==np.uint8 and y.dtype==np.uint8 and y.max()<len(LABELS),'invalid arrays')
        loaded[split]=(x,y)
    scale_sampler=None
    if a.small_sign_exposure:
        # HH_260906 - Reject missing TRAIN strata during CPU preflight, before acquiring GPU0.
        sign_counts=np.count_nonzero(loaded['train'][1]==LABELS.index('traffic_sign'),axis=(1,2))
        scale_sampler=SignScaleStratifiedSampler(sign_counts)
    if a.validate_only:
        print('NATIVE_MAPPING_CPU_PREFLIGHT_PASS',flush=True);return
    with r.gpu0_lease(workspace):
        # HH_260906 - Previous owned CUDA context may take a few seconds to disappear.
        deadline=time.monotonic()+90
        while True:
            try: gpu=r.preflight_gpu();break
            except ValueError as error:
                if 'GPU0 must be idle' not in str(error) or time.monotonic()>deadline:raise
                time.sleep(5)
        device=torch.device('cuda:0');r._seed_everything(20260922,device)
        a.output.mkdir(parents=True)
        for name in ('train_map_native_ah.py','map_perception.py','map_learning_balance.py','dual_recovery_objectives.py'):
            r.copy_new(Path(__file__).with_name(name),a.output/'source'/name)
        if a.expanded_train:r.copy_new(Path(__file__).with_name('map_expansion_contract.py'),a.output/'source/map_expansion_contract.py')
        r.copy_new(a.data/'manifest.json',a.output/'data_manifest.json')
        declaration=dict(gpu=gpu,started_at_utc=r.utc_now(),steps=2000,batch_size=8,
            seed=20260922,learning_rate=3e-4,input_shape=[3,360,640],initialization='scratch',
            source_sha256=r.digest(Path(__file__)),data_manifest_sha256=r.digest(a.data/'manifest.json'),
            parent_checkpoint_sha256=r.digest(parent_checkpoint),loss_mode='sqrt_frequency',
            augmentation='TRAIN only: probability 0.5 aligned crop, scale 0.65..1.0, resize to native; no flip',
            changes='native 640x360 resolution plus TRAIN crop; combined intervention, not single-factor A/B',
            new_physical_routes=a.expanded_train,automatic_promotion=False)
        if a.expanded_train:
            declaration.update(changes='AI: additive physical TRAIN route; same native architecture, seed, steps, augmentation; class weights recomputed on expanded TRAIN',
                expansion=manifest['expansion'],train_count=manifest['splits']['train']['count'],
                dev_count=manifest['splits']['dev']['count'])
        model=MapPerception().to(device)
        optimizer=torch.optim.AdamW(model.parameters(),lr=3e-4,weight_decay=1e-4)
        x,y=loaded['train'];pixels=np.bincount(y.ravel(),minlength=len(LABELS))
        if a.small_sign_exposure:
            # HH_260906 - Use TRAIN labels only; preserve all model/loss/crop/optimizer settings.
            declaration.update(changes='Same frozen dataset and native model; TRAIN-only sign-scale frame exposure is the sole A/B change',
                               sampling_contract=scale_sampler.contract(),parent_data_manifest_sha256=r.digest(a.parent/'data_manifest.json'),
                               dev_used_for_sampler=False)
        r.write_new(a.output/'started.json',declaration)
        weights=class_weights(pixels,'sqrt_frequency')
        r.write_new(a.output/'loss_contract.json',dict(train_pixels=pixels.tolist(),weights=weights.tolist(),dev_used=False))
        weights=torch.tensor(weights,dtype=torch.float32,device=device)
        sampler=np.random.default_rng(20260922)
        augmentation=np.random.default_rng(20260926)
        start=time.monotonic();model.train()
        with (a.output/'history.jsonl').open('x') as log:
            for step in range(1,2001):
                ids=(scale_sampler.sample_batch(sampler) if scale_sampler is not None
                     else sampler.integers(0,len(x),size=8))
                pairs=[paired_crop(x[i],y[i],augmentation) for i in ids]
                images=torch.tensor(np.stack([v[0] for v in pairs]).transpose(0,3,1,2).copy(),device=device,dtype=torch.float32)/255
                target=torch.tensor(np.stack([v[1] for v in pairs]).astype(np.int64),device=device)
                optimizer.zero_grad(set_to_none=True)
                loss=semantic_loss(model(images),target,weights)
                r.require(bool(torch.isfinite(loss)),'nonfinite loss');loss.backward()
                norm=torch.nn.utils.clip_grad_norm_(model.parameters(),5.)
                r.require(bool(torch.isfinite(norm)),'nonfinite gradient');optimizer.step()
                if step%100==0:
                    row=dict(step=step,loss=float(loss.detach()),wall_seconds=time.monotonic()-start)
                    log.write(json.dumps(row)+'\n');log.flush();print(json.dumps(row),flush=True)
        train_seconds=time.monotonic()-start
        r.save_new(a.output/'checkpoint.pt',dict(model_state_dict=model.state_dict(),labels=LABELS,
            model_id='portable_e2e.map_semantics.native_'+('sign_scale_ab' if a.small_sign_exposure else 'ai' if a.expanded_train else 'ah')+'.v1',optimizer_state_dict=optimizer.state_dict(),
            steps=2000,input_shape=[3,360,640],deployable=False,research_contract=declaration))
        model.eval();x,y=loaded['dev']

        def evaluate(network,images,labels):
            """HH_260906 - Unaugmented fixed DEV, identical pixels for both model comparisons."""
            network.eval();confusion=np.zeros((len(LABELS),len(LABELS)),dtype=np.int64);predictions=[]
            with torch.no_grad():
                for begin in range(0,len(images),8):
                    batch=torch.tensor(images[begin:begin+8].transpose(0,3,1,2).copy(),device=device,dtype=torch.float32)/255
                    pred=network(batch).argmax(1).cpu().numpy().astype(np.uint8);predictions.append(pred)
                    confusion+=np.bincount((labels[begin:begin+8].astype(np.int64)*len(LABELS)+pred).ravel(),minlength=len(LABELS)**2).reshape(len(LABELS),len(LABELS))
            union=confusion.sum(0)+confusion.sum(1)-np.diag(confusion)
            iou={name:float(confusion[i,i]/union[i]) if union[i] else None for i,name in enumerate(LABELS)}
            return dict(iou=iou,mean_iou_present=float(np.mean([v for v in iou.values() if v is not None])),
                pixel_accuracy=float(np.trace(confusion)/confusion.sum()),confusion=confusion.tolist()),np.concatenate(predictions)

        native,pred=evaluate(model,x,y)
        np.savez_compressed(a.output/'dev_predictions.npz',prediction=pred)
        resample=getattr(Image,'Resampling',Image)
        small_x=np.stack([np.asarray(Image.fromarray(v).resize((320,180),resample.BILINEAR)) for v in x])
        small_y=np.stack([np.asarray(Image.fromarray(v).resize((320,180),resample.NEAREST)) for v in y])
        common,_=evaluate(model,small_x,small_y)
        parent=MapPerception().to(device)
        parent.load_state_dict(torch.load(parent_checkpoint,map_location='cpu',weights_only=False)['model_state_dict'],strict=True)
        baseline_native,_=evaluate(parent,x,y);baseline_common,_=evaluate(parent,small_x,small_y)
        r.write_new(a.output/'summary.json',dict(status='PILOT_COMPLETE_NOT_PROMOTED',**native,
            steps=2000,training_wall_seconds=train_seconds,wall_seconds=time.monotonic()-start,
            checkpoint_sha256=r.digest(a.output/'checkpoint.pt'),parent_native=baseline_native,
            candidate_common_320=common,parent_common_320=baseline_common,
            model_parameters=sum(p.numel() for p in model.parameters()),
            sampling_contract=declaration.get('sampling_contract'),
            limitations=('Additional physical TRAIN route, unchanged repeated DEV; no independent TEST or complete HD map' if a.expanded_train
                         else 'Same frozen TRAIN/DEV, repeatedly inspected DEV; sign-size ablation is exploratory, no independent TEST or complete HD map' if a.small_sign_exposure
                         else 'Same two routes/Town/weather; no new physical data; no independent TEST or complete HD map'),
            deployment_approved=False,completed_at_utc=r.utc_now()))
        print('NATIVE_MAPPING_COMPLETE_NOT_PROMOTED',flush=True)


if __name__=='__main__':main()
