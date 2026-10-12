"""HH_260906 - New full-model objective experiment; frozen K evidence stays unchanged."""
import argparse
import json
from pathlib import Path
import sys
import time


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--validate-only',action='store_true')
    a=p.parse_args()
    plan=json.loads((a.baseline/'plan.json').read_text())
    sys.path.insert(0,plan['repo_root'])
    sys.path.insert(0,str(Path(__file__).resolve().parent))
    import torch
    from torch.utils.data import DataLoader
    from dual_recovery_objectives import near_horizon_loss
    from scripts.e2e import run_moving_multiroute_recovery_k as r
    workspace=r.check_run_environment(dict(repo_root=plan['repo_root'],output_root=str(a.output)))
    r.require(not a.output.exists(),'fresh output required')
    r.verify_sources(plan)
    summary=r.read_json(a.baseline/'summary.json');arm=r.ARMS[1]
    r.require(r.digest(a.baseline/'plan.json')==summary['plan_sha256'],'plan changed')
    ref=summary['training'][arm];checkpoint=a.baseline/ref['checkpoint']
    r.require(r.digest(checkpoint)==ref['checkpoint_sha256'],'checkpoint changed')
    ev=summary['evaluations'][arm];evpath=a.baseline/ev['path']
    r.require(r.digest(evpath)==ev['sha256'],'baseline evaluation changed')
    _,loaded,dev=r.verify_views(plan)
    if a.validate_only:
        print('AH_CPU_PREFLIGHT_PASS train='+str(len(loaded.examples)),flush=True);return
    with r.gpu0_lease(workspace):
        gpu=r.preflight_gpu();device=torch.device('cuda:0')
        r.LEARNING_RATE=3e-5;r.CHECKPOINT_SCHEMA='portable_e2e.near_horizon_ah.v1'
        a.output.mkdir(parents=True)
        for path in (Path(__file__),Path(__file__).with_name('dual_recovery_objectives.py')):
            r.copy_new(path,a.output/'source'/path.name)
        r.copy_new(a.baseline/'plan.json',a.output/'baseline_plan.json')
        declaration=dict(gpu=gpu,steps=r.MAX_STEPS,learning_rate=r.LEARNING_RATE,
            seed=r.SEED,batch_size=r.BATCH_SIZE,base_loss=r.LOSS_CONFIG.to_dict(),
            near_horizon_points=20,near_horizon_seconds=2.,near_horizon_weight=.5,
            mode_assignment='unchanged full-horizon composite oracle',
            baseline_checkpoint_sha256=r.digest(checkpoint),full_model_trainable=True,
            new_head=False,training_data_unchanged=True,train_count=len(loaded.examples),
            evaluation_count=len(dev['loaded'].examples),optimizer_reset=True,
            dev_during_training=False,automatic_promotion=False,
            comparison='AD continuation: same K1 initialization/LR/sampling; add 0.5 near-horizon loss',
            limitations='Repeated DEV; no independent TEST or closed-loop approval')
        r.write_new(a.output/'declaration.json',declaration)
        payload=torch.load(checkpoint,map_location='cpu',weights_only=False)
        cfg=r.ModelConfig.from_mapping(payload['model_config'])
        initial=payload['model_state_dict'];initial_hash=r.tensor_digest(initial)
        r.require(initial_hash==payload['final_model_state_sha256'],'initial state changed')
        r._seed_everything(r.SEED,device)
        model=r.PerspectiveTrajectoryModel(cfg).to(device);model.load_state_dict(initial,strict=True)
        dataset=r.Common10TorchDataset(loaded.examples,cfg,verify_image_sha256=True,split='train')
        optimizer=torch.optim.AdamW(model.parameters(),lr=r.LEARNING_RATE,weight_decay=r.WEIGHT_DECAY)
        model.train();step=0;start=time.monotonic()
        folder=a.output/arm/'training';folder.mkdir(parents=True)
        r.write_new(a.output/'started.json',dict(status='FULL_MODEL_TRAINING',started_at_utc=r.utc_now(),**declaration))
        fields=('images','calibration','ego_history','ego_history_mask','route_xy','route_mask')
        with (folder/'history.jsonl').open('x') as log:
            for epoch in range(r.LOGICAL_EPOCHS):
                batches,_=r.build_epoch_batches(dataset.examples,arm,epoch)
                for raw in DataLoader(dataset,batch_sampler=batches,num_workers=0):
                    batch=r._move_batch(raw,device)
                    optimizer.zero_grad(set_to_none=True)
                    xy,speed,logits=model(*(batch[k] for k in fields))
                    base=r.trajectory_loss(xy,speed,logits,batch['target_xy'],batch['target_speed_mps'],
                        batch['target_valid'],r.LOSS_CONFIG,target_yaw=batch['target_yaw_rad'])
                    near=near_horizon_loss(xy,speed,batch['target_xy'],batch['target_speed_mps'],
                                           batch['target_valid'],base['oracle_candidate_index'])
                    loss=base['loss']+.5*near
                    r.require(bool(torch.isfinite(loss)),'nonfinite loss');loss.backward()
                    norm=torch.nn.utils.clip_grad_norm_(model.parameters(),r.MAXIMUM_GRADIENT_NORM)
                    r.require(bool(torch.isfinite(norm)),'nonfinite gradient');optimizer.step();step+=1
                    r.require(r._nested_tensors_are_finite(model.state_dict()),'nonfinite model')
                    metrics=dict(step=step,loss=float(loss.detach()),near_loss=float(near.detach()),wall_seconds=time.monotonic()-start)
                    if step%250==0:
                        log.write(json.dumps(metrics)+'\n');log.flush();print(json.dumps(metrics),flush=True)
        r.require(step==r.MAX_STEPS,'incomplete training')
        final=r._checkpoint_payload(arm,model,cfg,initial_hash,dataset,r.schedule_contract(dataset.examples,arm),metrics)
        final['research_objective']=declaration
        r.save_new(folder/'final_checkpoint.pt',final)
        r.save_new(folder/'optimizer_state.pt',optimizer.state_dict())
        training=dict(steps=step,wall_seconds=time.monotonic()-start,checkpoint=str((folder/'final_checkpoint.pt').relative_to(a.output)),checkpoint_sha256=r.digest(folder/'final_checkpoint.pt'))
        del model,optimizer;torch.cuda.empty_cache()
        evaluation=r.evaluate_final(arm,folder/'final_checkpoint.pt',dev['loaded'],cfg,a.output,device)
        r.write_new(a.output/'summary.json',dict(status='COMPLETE_NOT_PROMOTED',training=training,
            evaluation=evaluation,versus_original_k1=r.paired_nonregression(r.read_json(evpath),evaluation),
            strict_gate=r.strict_gate(evaluation),completed_at_utc=r.utc_now(),release_approved=False))
        print('AH_TRAINING_AND_DEV_COMPLETE_NOT_PROMOTED',flush=True)


if __name__=='__main__':main()
