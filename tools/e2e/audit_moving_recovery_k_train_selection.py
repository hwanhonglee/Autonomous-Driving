#!/usr/bin/env python3
"""HH_260906 - Audit TRAIN-only selection targets of completed K models without updating weights."""
import argparse
from collections import defaultdict
from pathlib import Path
import sys
import json

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import torch
from torch.utils.data import DataLoader
from portable_e2e.losses import trajectory_loss
from portable_e2e.model import ModelConfig, PerspectiveTrajectoryModel
from portable_e2e.torch_dataset import Common10TorchDataset
from scripts.e2e import run_moving_multiroute_recovery_k as r


def summarize(rows):
    """HH_260906 - Separate score misclassification from composite-versus-ADE target disagreement."""
    count = len(rows)
    return dict(count=count,
        selected_matches_composite=sum(x['selected']==x['composite'] for x in rows),
        selected_matches_ade=sum(x['selected']==x['ade_oracle'] for x in rows),
        composite_differs_from_ade=sum(x['composite']!=x['ade_oracle'] for x in rows),
        selected_ade_m=sum(x['selected_ade_m'] for x in rows)/count,
        composite_ade_m=sum(x['composite_ade_m'] for x in rows)/count,
        oracle_ade_m=sum(x['oracle_ade_m'] for x in rows)/count,
        selection_gap_above_1m=sum(x['selected_ade_m']-x['oracle_ade_m']>1 for x in rows))


def audit(run_root, output):
    """HH_260906 - Use the personal GPU0 lease and immutable final checkpoints for one TRAIN pass."""
    plan, summary = [r.read_json(run_root/name) for name in ('plan.json','summary.json')]
    r.require(summary['status']=='COMPLETE_NOT_PROMOTED', 'completed K run required')
    r.require(r.digest(run_root/'plan.json')==summary['plan_sha256'], 'plan mismatch')
    r.validate_plan(plan)
    r.verify_sources(plan)
    workspace=r.check_run_environment(plan)
    r.require(output.absolute().is_relative_to(workspace/'runs/diagnostics') and not output.exists(), 'fresh personal output required')
    with r.gpu0_lease(workspace):
        gpu=r.preflight_gpu()
        device=torch.device('cuda:0')
        r._seed_everything(r.SEED,device)
        config=ModelConfig.from_mapping(r.read_json(ROOT/r.MODEL_RELATIVE))
        output.mkdir(parents=True)
        r.write_new(output/'started.json',dict(gpu=gpu,source_sha256=r.digest(Path(__file__)),
            train_only_inference=True, optimizer_created=False, dev_inference=False))
        results={}
        for arm in r.ARMS:
            loaded, examples, _=r._load_view(Path(plan['views'][arm]['path']),'train',arm)
            r._verify_loaded_view(plan['views'][arm],loaded,examples)
            ref=summary['training'][arm]
            checkpoint=run_root/ref['checkpoint']
            r.require(r.digest(checkpoint)==ref['checkpoint_sha256'],'checkpoint mismatch')
            payload=torch.load(checkpoint,map_location='cpu')
            model=PerspectiveTrajectoryModel(config).to(device)
            model.load_state_dict(payload['model_state_dict'],strict=True)
            model.eval().requires_grad_(False)
            dataset=Common10TorchDataset(examples,config,verify_image_sha256=True,split='train')
            rows=[]
            with torch.no_grad():
                for raw in DataLoader(dataset,batch_size=4,shuffle=False,num_workers=0):
                    batch=r._move_batch(raw,device)
                    xy,speed,logits=model(batch['images'],batch['calibration'],batch['ego_history'],
                        batch['ego_history_mask'],batch['route_xy'],batch['route_mask'])
                    loss=trajectory_loss(xy,speed,logits,batch['target_xy'],batch['target_speed_mps'],
                        batch['target_valid'],r.LOSS_CONFIG,target_yaw=batch['target_yaw_rad'])
                    mask=batch['target_valid'][:,None].to(xy.dtype)
                    errors=(torch.linalg.vector_norm(xy-batch['target_xy'][:,None],dim=-1)*mask).sum(-1)/mask.sum(-1).clamp_min(1)
                    chosen=logits.argmax(-1); oracle=errors.argmin(-1)
                    composite=loss['oracle_candidate_index']
                    for i,episode in enumerate(raw['episode_id']):
                        c,o,t=int(chosen[i]),int(oracle[i]),int(composite[i])
                        direction='left' if episode==r.LEFT_EPISODE else 'right' if episode==r.RIGHT_EPISODE else 'straight'
                        rows.append(dict(sample_id=raw['sample_id'][i],episode_id=episode,direction=direction,
                            selected=c,composite=t,ade_oracle=o,selected_ade_m=float(errors[i,c]),
                            composite_ade_m=float(errors[i,t]),oracle_ade_m=float(errors[i,o])))
            groups={}
            for field in ('direction','episode_id'):
                partition=defaultdict(list)
                for row in rows: partition[row[field]].append(row)
                groups[field]={key:summarize(values) for key,values in partition.items()}
            r.write_new(output/(arm+'.json'),dict(groups=groups,rows=rows))
            results[arm]=groups
            r.require(r.tensor_digest(model.state_dict())==ref['final_model_state_sha256'],'weights changed during audit')
            del model
            torch.cuda.empty_cache()
        report=dict(status='TRAIN_SELECTION_AUDIT_COMPLETE',results=results,dev_inference=False,
            training_performed=False,source_summary_sha256=r.digest(run_root/'summary.json'))
        r.write_new(output/'summary.json',report)
        return {arm:value['direction'] for arm,value in results.items()}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(audit(args.run_root,args.output)))
