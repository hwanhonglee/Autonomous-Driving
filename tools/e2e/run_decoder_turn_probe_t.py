"""HH_260906 - Optimize decoder latents on TRAIN turns; not a model, controller, or global bound."""
import argparse
import json
import math
from pathlib import Path
import sys
import time


def drive_latents(head_output, batch_size):
    """HH_260906 - Infer DRIVE output count; stopmix includes separate STOP candidates outside this head."""
    if head_output.ndim!=2 or head_output.shape[0]!=batch_size or head_output.shape[1]%(64*2):
        raise ValueError('invalid DRIVE head shape')
    return head_output.reshape(batch_size,head_output.shape[1]//(64*2),64,2)


def main():
    """HH_260906 - Freeze all model weights and expose label-conditioned decoder fitting explicitly."""
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    plan=json.loads((a.baseline/'plan.json').read_text());sys.path.insert(0,plan['repo_root'])
    import torch
    from torch.utils.data import default_collate
    from scripts.e2e import run_moving_multiroute_recovery_k as r
    from portable_e2e.contract import ContractError
    from portable_e2e.dataset import FEATURE_NAMES
    from portable_e2e.runtime_contract import RuntimeGateConfig,validate_and_select_trajectory
    workspace=r.check_run_environment(dict(repo_root=plan['repo_root'],output_root=str(a.output)))
    r.require(not a.output.exists(),'fresh output required');r.verify_sources(plan)
    summary=r.read_json(a.baseline/'summary.json');arm=r.ARMS[1]
    r.require(r.digest(a.baseline/'plan.json')==summary['plan_sha256'],'plan changed')
    checkpoint=a.baseline/summary['training'][arm]['checkpoint']
    r.require(r.digest(checkpoint)==summary['training'][arm]['checkpoint_sha256'],'checkpoint changed')
    with r.gpu0_lease(workspace):
        gpu=r.preflight_gpu();device=torch.device('cuda:0');r._seed_everything(r.SEED,device)
        a.output.mkdir(parents=True);r.copy_new(Path(__file__),a.output/'source.py')
        r.write_new(a.output/'plan.json',dict(gpu=gpu,steps_per_run=500,initializations=['model','random'],
            objectives=['geometry','joint'],split='train',dev_test_opened=False,model_weights_frozen=True,
            source_sha256=r.digest(Path(__file__)),checkpoint_sha256=r.digest(checkpoint),release_approved=False))
        loaded,examples,_=r._load_view(Path(plan['views'][arm]['path']),'train',arm)
        r._verify_loaded_view(plan['views'][arm],loaded,examples)
        payload=torch.load(checkpoint,map_location='cpu',weights_only=False)
        cfg=r.ModelConfig.from_mapping(payload['model_config'])
        ds=r.Common10TorchDataset(examples,cfg,verify_image_sha256=True,split='train')
        chosen=[]
        for group,episode,n in [('right',r.RIGHT_EPISODE,4),('left',r.LEFT_EPISODE,2)]:
            indexes=[i for i,e in enumerate(examples) if e.episode_id==episode
                and all(x is not None for x in e.targets_xy) and max(e.target_speed_mps)>=.5
                and max(abs(y) for y in e.target_yaw_rad)>=math.radians(15)]
            r.require(len(indexes)>=n,'insufficient strong turns')
            chosen.extend((group,indexes[round(q*(len(indexes)-1)/(n-1))]) for q in range(n))
        b=r._move_batch(default_collate([ds[i] for _,i in chosen]),device)
        model=r.PerspectiveTrajectoryModel(cfg).to(device).eval().requires_grad_(False)
        model.load_state_dict(payload['model_state_dict'],strict=True)
        original_sha=r.tensor_digest(model.state_dict());captured=[]
        hook=model.trajectory_head.register_forward_hook(lambda module,inputs,out:captured.append(out.detach()))
        with torch.no_grad():
            base_xy,base_speed,logits=model(b['images'],b['calibration'],b['ego_history'],b['ego_history_mask'],b['route_xy'],b['route_mask'])
        hook.remove();initial=drive_latents(captured[0],len(chosen))
        velocity_index=FEATURE_NAMES.index('velocity_x_mps')
        def gate(xy,speed,j):
            try:
                validate_and_select_trajectory([xy],[speed],[0.],RuntimeGateConfig(candidate_count=1),
                    current_speed_mps=float(b['ego_history'][j,-1,velocity_index]))
                return 'PASS'
            except ContractError as exc:return str(exc)
        base_ade=torch.linalg.vector_norm(base_xy-b['target_xy'][:,None],dim=-1).mean(-1)
        rows=[]
        for j,(group,i) in enumerate(chosen):
            e=examples[i];sel=int(logits[j].argmax());oracle=int(base_ade[j].argmin())
            rows.append(dict(group=group,token=e.token,episode=e.episode_id,
                future_yaw_deg=max(abs(y) for y in e.target_yaw_rad)*180/math.pi,
                target_gate=gate(e.targets_xy,e.target_speed_mps,j),
                baseline_selected_ade_m=float(base_ade[j,sel]),baseline_oracle_ade_m=float(base_ade[j,oracle]),
                target_xy=e.targets_xy,target_speed=e.target_speed_mps,baseline_xy=base_xy[j,sel].tolist(),
                baseline_speed=base_speed[j,sel].tolist(),fits=[]))
        r.write_new(a.output/'before.json',rows)
        start=time.monotonic()
        for objective in ('geometry','joint'):
            for seed_mode in ('model','random'):
                raw=torch.nn.Parameter(initial.clone() if seed_mode=='model' else torch.randn_like(initial)*.1)
                optimizer=torch.optim.Adam([raw],lr=.03)
                best=torch.full((len(chosen),),float('inf'),device=device);best_xy=base_xy[:,0].clone();best_speed=base_speed[:,0].clone()
                for step in range(501):
                    xy,speed=model._decode_physical_v1(raw,b['ego_history'],b['route_xy'],b['route_mask'])
                    ade=torch.linalg.vector_norm(xy-b['target_xy'][:,None],dim=-1).mean(-1)
                    speed_mae=(speed-b['target_speed_mps'][:,None]).abs().mean(-1)
                    score=ade+(.2*speed_mae if objective=='joint' else 0.)
                    with torch.no_grad():
                        values,ix=score.min(-1);mask=values<best;batch=torch.arange(len(chosen),device=device)
                        best[mask]=values[mask];best_xy[mask]=xy[batch,ix][mask];best_speed[mask]=speed[batch,ix][mask]
                    if step==500:break
                    loss=score.mean();r.require(bool(torch.isfinite(loss)),'nonfinite latent objective')
                    optimizer.zero_grad();loss.backward();optimizer.step()
                for j,row in enumerate(rows):
                    x=best_xy[j].tolist();s=best_speed[j].tolist()
                    row['fits'].append(dict(objective=objective,initialization=seed_mode,
                        ade_m=float(torch.linalg.vector_norm(best_xy[j]-b['target_xy'][j],dim=-1).mean()),
                        speed_mae_mps=float((best_speed[j]-b['target_speed_mps'][j]).abs().mean()),
                        objective_value=float(best[j]),runtime_gate=gate(x,s,j),xy=x,speed=s))
                print(json.dumps(dict(objective=objective,initialization=seed_mode,elapsed_s=time.monotonic()-start)),flush=True)
        r.require(r.tensor_digest(model.state_dict())==original_sha,'model weights changed')
        r.write_new(a.output/'results.json',dict(rows=rows,elapsed_s=time.monotonic()-start,
            model_weights_unchanged=True,dev_test_evaluated=False,training_admitted=False,
            release_approved=False,global_optimum_proven=False,uses_target_future=True))


if __name__=='__main__':main()
