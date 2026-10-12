"""HH_260906 - Bounded TRAIN-only memorization diagnostic; never export a deployable model."""
import argparse
import json
import math
from pathlib import Path
import sys
import time


def main():
    """HH_260906 - Reuse frozen K training math while explicitly separating this diagnostic from generalization."""
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    plan=json.loads((a.baseline/'plan.json').read_text());repo=Path(plan['repo_root'])
    sys.path.insert(0,str(repo))
    import torch
    from torch.utils.data import default_collate
    from scripts.e2e import run_moving_multiroute_recovery_k as r
    from portable_e2e.contract import ContractError
    from portable_e2e.dataset import FEATURE_NAMES
    from portable_e2e.runtime_contract import RuntimeGateConfig,validate_and_select_trajectory
    workspace=r.check_run_environment(dict(repo_root=str(repo),output_root=str(a.output)))
    r.require(not a.output.exists(),'fresh diagnostic directory required')
    r.verify_sources(plan)
    arm=r.ARMS[1];summary=r.read_json(a.baseline/'summary.json')
    r.require(r.digest(a.baseline/'plan.json')==summary['plan_sha256'],'baseline plan changed')
    checkpoint=a.baseline/summary['training'][arm]['checkpoint']
    r.require(r.digest(checkpoint)==summary['training'][arm]['checkpoint_sha256'],'checkpoint changed')
    with r.gpu0_lease(workspace):
        gpu=r.preflight_gpu();device=torch.device('cuda:0')
        a.output.mkdir(parents=True)
        r.copy_new(Path(__file__),a.output/'source.py')
        r.write_new(a.output/'started.json',dict(gpu=gpu,steps=1500,split='train',dev_test_opened=False,
            checkpoint_sha256=r.digest(checkpoint),source_sha256=r.digest(Path(__file__)),release_approved=False))
        loaded,examples,_=r._load_view(Path(plan['views'][arm]['path']),'train',arm)
        r._verify_loaded_view(plan['views'][arm],loaded,examples)
        payload=torch.load(checkpoint,map_location='cpu',weights_only=False)
        cfg=r.ModelConfig.from_mapping(payload['model_config'])
        dataset=r.Common10TorchDataset(examples,cfg,verify_image_sha256=True,split='train')
        selections=[];coverage={};velocity_index=FEATURE_NAMES.index('velocity_x_mps')
        for group,episode in [('straight',r.T07_15_EPISODE),('left',r.LEFT_EPISODE),('right',r.RIGHT_EPISODE)]:
            candidates=[];moving=0;strong=0
            for i,e in enumerate(examples):
                if e.episode_id!=episode or any(x is None for x in e.targets_xy):continue
                if max(e.target_speed_mps)<.5:continue
                moving+=1
                try:
                    validate_and_select_trajectory([e.targets_xy],[e.target_speed_mps],[0.],
                        RuntimeGateConfig(candidate_count=1),current_speed_mps=e.features[velocity_index])
                except ContractError:continue
                yaw=max(abs(y) for y in e.target_yaw_rad)
                strong+=yaw>=math.radians(15)
                candidates.append((yaw,i))
            r.require(len(candidates)>=4,'not enough feasible moving examples: '+group)
            candidates.sort(reverse=True)
            # HH_260906 - Deterministic angle quantiles expose feasible coverage instead of hand-picking easy losses.
            chosen=[candidates[round(q*(len(candidates)-1)/3)] for q in range(4)]
            coverage[group]=dict(moving=moving,feasible=len(candidates),feasible_turn_ge15deg=strong)
            selections.extend(dict(group=group,index=i,token=examples[i].token,episode=episode,
                future_max_abs_yaw_deg=math.degrees(yaw)) for yaw,i in chosen)
        r.write_new(a.output/'selection.json',dict(coverage=coverage,samples=selections,
            selection_uses_train_labels_only=True,dataset_fingerprint=loaded.fingerprint_sha256,
            history_built_from_full_train_dataset=True,training_admission=False))
        batches=[r._move_batch(default_collate([dataset[s['index']] for s in selections[i:i+4]]),device)
                 for i in range(0,12,4)]
        r._seed_everything(r.SEED,device)
        model=r.PerspectiveTrajectoryModel(cfg).to(device)
        model.load_state_dict(payload['model_state_dict'],strict=True)
        r.require(r.tensor_digest(model.state_dict())==payload['final_model_state_sha256'],'state hash changed')
        def forward(b):
            return model(b['images'],b['calibration'],b['ego_history'],b['ego_history_mask'],b['route_xy'],b['route_mask'])
        def evaluate():
            model.eval();rows=[]
            with torch.no_grad():
                for group_index,b in enumerate(batches):
                    xy,speed,logits=forward(b)
                    ade=torch.linalg.vector_norm(xy-b['target_xy'][:,None],dim=-1).mean(-1)
                    for j in range(4):
                        chosen=int(logits[j].argmax());oracle=int(ade[j].argmin())
                        route=b['route_xy'][j][b['route_mask'][j].bool()]
                        # HH_260906 - Constant-speed route interpolation is a diagnostic reference, not a controller.
                        arc=torch.cat((torch.zeros(1,device=device),torch.linalg.vector_norm(route[1:]-route[:-1],dim=-1).cumsum(0)))
                        distances=(torch.arange(1,65,device=device)*.1*b['ego_history'][j,-1,velocity_index].clamp(min=0)).clamp(max=arc[-1])
                        ix=torch.searchsorted(arc,distances).clamp(1,len(arc)-1)
                        ratio=((distances-arc[ix-1])/(arc[ix]-arc[ix-1]).clamp(min=1e-8))[:,None]
                        reference=route[ix-1]+ratio*(route[ix]-route[ix-1])
                        rows.append(dict(selections[group_index*4+j],selected_ade_m=float(ade[j,chosen]),
                            oracle_ade_m=float(ade[j,oracle]),selected_candidate=chosen,oracle_candidate=oracle,
                            speed_mae_mps=float((speed[j,chosen]-b['target_speed_mps'][j]).abs().mean()),
                            constant_speed_route_ade_m=float(torch.linalg.vector_norm(reference-b['target_xy'][j],dim=-1).mean())))
            return rows
        r.write_new(a.output/'before.json',evaluate())
        optimizer=torch.optim.AdamW(model.parameters(),lr=r.LEARNING_RATE,weight_decay=r.WEIGHT_DECAY)
        start=time.monotonic();model.train()
        with (a.output/'progress.jsonl').open('x') as log:
            for step in range(1500):
                b=batches[step%3];optimizer.zero_grad(set_to_none=True)
                xy,speed,logits=forward(b)
                losses=r.trajectory_loss(xy,speed,logits,b['target_xy'],b['target_speed_mps'],b['target_valid'],
                    r.LOSS_CONFIG,target_yaw=b['target_yaw_rad'])
                loss=losses['loss'];r.require(bool(torch.isfinite(loss)),'nonfinite loss')
                loss.backward();norm=torch.nn.utils.clip_grad_norm_(model.parameters(),r.MAXIMUM_GRADIENT_NORM)
                r.require(bool(torch.isfinite(norm)),'nonfinite gradients');optimizer.step()
                if (step+1)%100==0:
                    record=dict(step=step+1,loss=float(loss.detach()),elapsed_s=time.monotonic()-start)
                    log.write(json.dumps(record)+'\n');log.flush();print(json.dumps(record),flush=True)
        after=evaluate();r.write_new(a.output/'after.json',after)
        r.save_new(a.output/'research_checkpoint.pt',dict(schema='portable_e2e.train_memorization_s.research_only',
            model_state_dict=r._clone_state(model.state_dict()),model_config=cfg.to_dict(),steps=1500,
            train_tokens=[s['token'] for s in selections],release_approved=False))
        r.write_new(a.output/'summary.json',dict(status='COMPLETE_RESEARCH_ONLY',steps=1500,samples_seen=6000,
            elapsed_training_s=time.monotonic()-start,full_model_trained=True,dev_test_evaluated=False,
            checkpoint_sha256=r.digest(a.output/'research_checkpoint.pt'),coverage=coverage,
            model_state_sha256=r.tensor_digest(model.state_dict()),release_approved=False))


if __name__=='__main__':main()
