"""HH_260906 - Full-model K1 continuation with explicit TRAIN-only retention/ranking."""
import argparse
import json
from pathlib import Path
import sys
import time


def continuation_reference(directory, expected_sha256, baseline_plan_sha256):
    """HH_260906 - Admit only a completed, hash-pinned AF parent with the same data plan."""
    import hashlib
    digest=lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    summary=json.loads((directory/'summary.json').read_text())
    declaration=json.loads((directory/'declaration.json').read_text())
    checkpoint=directory/'K1_current_plus_successful4/training/final_checkpoint.pt'
    if (summary.get('status')!='COMPLETE_NOT_PROMOTED'
        or declaration.get('objective')!='rank_only'
        or declaration.get('preservation_weight')!=0
        or summary['training']['steps']!=15000
        or digest(directory/'baseline_plan.json')!=baseline_plan_sha256
        or digest(checkpoint)!=expected_sha256
        or summary['training']['checkpoint_sha256']!=expected_sha256
        or summary['evaluation']['checkpoint_sha256']!=expected_sha256):
        raise ValueError('completed AF parent, matching plan and checkpoint hashes required')
    return checkpoint,summary


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    # HH_260906 - AF removes only retention to isolate its effect against the archived AE run.
    p.add_argument('--objective',choices=('rank_retention','rank_only'),default='rank_retention')
    # HH_260906 - AG is an explicit weights-only continuation, never an exact optimizer resume.
    p.add_argument('--continue-from',type=Path)
    p.add_argument('--parent-checkpoint-sha256')
    a=p.parse_args()
    if bool(a.continue_from)!=bool(a.parent_checkpoint_sha256):
        p.error('continuation requires both parent directory and its pinned checkpoint SHA256')
    if a.continue_from and a.objective!='rank_only':
        p.error('AG continuation retains the AF rank-only objective')
    preservation_weight=.5 if a.objective=='rank_retention' else 0.
    plan=json.loads((a.baseline/'plan.json').read_text())
    # HH_260906 - Frozen baseline dependencies and standalone objective are separately archived.
    sys.path.insert(0,plan['repo_root'])
    sys.path.insert(0,str(Path(__file__).resolve().parent))
    import torch
    from torch.utils.data import DataLoader
    from recovery_objective import recovery_terms
    from scripts.e2e import run_moving_multiroute_recovery_k as r
    workspace=r.check_run_environment(dict(repo_root=plan['repo_root'],output_root=str(a.output)))
    r.require(not a.output.exists(),'fresh output required')
    r.verify_sources(plan)
    summary=r.read_json(a.baseline/'summary.json');arm=r.ARMS[1]
    r.require(r.digest(a.baseline/'plan.json')==summary['plan_sha256'],'plan changed')
    ref=summary['training'][arm]; checkpoint=a.baseline/ref['checkpoint']
    r.require(r.digest(checkpoint)==ref['checkpoint_sha256'],'checkpoint changed')
    evref=summary['evaluations'][arm]; evpath=a.baseline/evref['path']
    r.require(r.digest(evpath)==evref['sha256'],'evaluation changed')
    parent_summary=None
    if a.continue_from:
        r.require(a.continue_from.resolve().is_relative_to(workspace/'runs/diagnostics'),
                  'parent must remain inside personal diagnostic runs')
        checkpoint,parent_summary=continuation_reference(a.continue_from,a.parent_checkpoint_sha256,
                                                          r.digest(a.baseline/'plan.json'))
    with r.gpu0_lease(workspace):
        gpu=r.preflight_gpu(); device=torch.device('cuda:0')
        a.output.mkdir(parents=True)
        for path in (Path(__file__),Path(__file__).with_name('recovery_objective.py')):
            r.copy_new(path,a.output/'source'/path.name)
        r.copy_new(a.baseline/'plan.json',a.output/'baseline_plan.json')
        if parent_summary:
            r.copy_new(a.continue_from/'summary.json',a.output/'parent_summary.json')
            r.copy_new(a.continue_from/'declaration.json',a.output/'parent_declaration.json')
        r.LEARNING_RATE=1e-5 if parent_summary else 3e-5
        r.CHECKPOINT_SCHEMA='portable_e2e.full_recovery_'+('ag' if parent_summary else 'ae' if preservation_weight else 'af')+'.v1'
        declaration=dict(gpu=gpu,steps=r.MAX_STEPS,learning_rate=r.LEARNING_RATE,
            base_loss=r.LOSS_CONFIG.to_dict(),ranking_weight=.2,preservation_weight=preservation_weight,
            objective=a.objective,ablation=(
                'AF weights plus lower LR; convergence extension, not a single-factor K1 ablation'
                if parent_summary else 'Only preservation weight differs from AE; same K1 initialization and schedule'),
            ranking_temperature_m=.5,teacher_ade_threshold_m=.75,teacher_speed_threshold_mps=.5,
            baseline_checkpoint_sha256=ref['checkpoint_sha256'],full_model_trainable=True,
            new_head=False,training_data_unchanged=True,teacher_train_only=True,
            dev_during_training=False,automatic_promotion=False,
            limitations='One seed, repeated DEV, no left-turn/independent STOP verification; no deployment')
        declaration.update(weights_only_continuation=bool(parent_summary),optimizer_resume=False,
            rng_resume=False,sampler_restarts=True,parent_run=str(a.continue_from) if parent_summary else None,
            additional_optimizer_steps=r.MAX_STEPS,initialization_checkpoint_sha256=r.digest(checkpoint))
        r.write_new(a.output/'declaration.json',declaration)
        _,loaded,dev=r.verify_views(plan)
        payload=torch.load(checkpoint,map_location='cpu',weights_only=False)
        cfg=r.ModelConfig.from_mapping(payload['model_config'])
        initial=payload['model_state_dict']; initial_hash=r.tensor_digest(initial)
        r.require(initial_hash==payload['final_model_state_sha256'],'initial hash differs')
        r._seed_everything(r.SEED,device)
        model=r.PerspectiveTrajectoryModel(cfg).to(device)
        model.load_state_dict(initial,strict=True)
        teacher=r.PerspectiveTrajectoryModel(cfg).to(device)
        teacher.load_state_dict(initial,strict=True);teacher.eval().requires_grad_(False)
        dataset=r.Common10TorchDataset(loaded.examples,cfg,verify_image_sha256=True,split='train')
        if parent_summary:
            r.require(payload['training_dataset_fingerprint_sha256']==dataset.fingerprint_sha256,
                      'continuation training data differs from parent')
        optimizer=torch.optim.AdamW(model.parameters(),lr=r.LEARNING_RATE,weight_decay=r.WEIGHT_DECAY)
        model.train(); step=0; started=time.monotonic()
        folder=a.output/arm/'training';folder.mkdir(parents=True)
        r.write_new(a.output/'started.json',dict(status='FULL_MODEL_TRAINING',started_at_utc=r.utc_now(),
                    train_count=len(dataset),full_model_trainable=True,**r.DENIALS))
        fields=('images','calibration','ego_history','ego_history_mask','route_xy','route_mask')
        with (folder/'history.jsonl').open('x') as history:
            for epoch in range(r.LOGICAL_EPOCHS):
                batches,proof=r.build_epoch_batches(dataset.examples,arm,epoch)
                for raw in DataLoader(dataset,batch_sampler=batches,num_workers=0):
                    batch=r._move_batch(raw,device)
                    args=[batch[k] for k in fields]
                    optimizer.zero_grad(set_to_none=True)
                    xy,speed,logits=model(*args)
                    with torch.no_grad(): taught=teacher(*args)
                    base=r.trajectory_loss(xy,speed,logits,batch['target_xy'],batch['target_speed_mps'],
                        batch['target_valid'],r.LOSS_CONFIG,target_yaw=batch['target_yaw_rad'])
                    rank,preserve,eligible=recovery_terms(xy,speed,logits,taught,batch['target_xy'],
                                                   batch['target_speed_mps'],batch['target_valid'])
                    loss=base['loss']+.2*rank+preservation_weight*preserve
                    r.require(bool(torch.isfinite(loss)),'nonfinite loss')
                    loss.backward()
                    norm=torch.nn.utils.clip_grad_norm_(model.parameters(),r.MAXIMUM_GRADIENT_NORM)
                    r.require(bool(torch.isfinite(norm)),'nonfinite gradient')
                    optimizer.step(); step+=1
                    r.require(r._nested_tensors_are_finite(model.state_dict()),'nonfinite model')
                    metrics=dict(step=step,loss=float(loss.detach()),ranking=float(rank.detach()),
                                 preservation=float(preserve.detach()),eligible=int(eligible))
                    if step%250==0:
                        history.write(json.dumps(metrics)+'\n');history.flush()
                        print(json.dumps(metrics),flush=True)
        r.require(step==r.MAX_STEPS,'incomplete training')
        final=r._checkpoint_payload(arm,model,cfg,initial_hash,dataset,r.schedule_contract(dataset.examples,arm),metrics)
        final['research_objective']=declaration
        r.save_new(folder/'final_checkpoint.pt',final)
        # HH_260906 - Optimizer kept separately for future reproducibility, not misreported as resumed.
        r.save_new(folder/'optimizer_state.pt',optimizer.state_dict())
        training=dict(steps=step,wall_seconds=time.monotonic()-started,
                      checkpoint_sha256=r.digest(folder/'final_checkpoint.pt'))
        del model,teacher,optimizer;torch.cuda.empty_cache()
        evaluation=r.evaluate_final(arm,folder/'final_checkpoint.pt',dev['loaded'],cfg,a.output,device)
        comparisons={'versus_parent_af':r.paired_nonregression(parent_summary['evaluation'],evaluation)} if parent_summary else {}
        r.write_new(a.output/'summary.json',dict(status='COMPLETE_NOT_PROMOTED',training=training,
            evaluation=evaluation,versus_original_k1=r.paired_nonregression(r.read_json(evpath),evaluation),
            strict_gate=r.strict_gate(evaluation),completed_at_utc=r.utc_now(),release_approved=False,**comparisons))
        print('FULL_RECOVERY_COMPLETE_NOT_PROMOTED '+a.objective,flush=True)


if __name__=='__main__':main()
