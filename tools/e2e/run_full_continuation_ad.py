"""HH_260906 - Continue full K1 training on unchanged TRAIN data, then evaluate fixed DEV once."""
import argparse
import json
from pathlib import Path
import sys


def main():
    """HH_260906 - Keep personal GPU lease, frozen dependencies and nonpromotion safeguards."""
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    plan=json.loads((a.baseline/'plan.json').read_text());sys.path.insert(0,plan['repo_root'])
    import torch
    from scripts.e2e import run_moving_multiroute_recovery_k as r
    workspace=r.check_run_environment(dict(repo_root=plan['repo_root'],output_root=str(a.output)))
    r.require(not a.output.exists(),'fresh output required');r.verify_sources(plan)
    summary=r.read_json(a.baseline/'summary.json');arm=r.ARMS[1]
    r.require(r.digest(a.baseline/'plan.json')==summary['plan_sha256'],'baseline plan changed')
    checkpoint=a.baseline/summary['training'][arm]['checkpoint']
    r.require(r.digest(checkpoint)==summary['training'][arm]['checkpoint_sha256'],'baseline checkpoint changed')
    baseline_evaluation=a.baseline/summary['evaluations'][arm]['path']
    r.require(r.digest(baseline_evaluation)==summary['evaluations'][arm]['sha256'],'baseline evaluation changed')
    original_evaluation=r.read_json(baseline_evaluation)
    with r.gpu0_lease(workspace):
        gpu=r.preflight_gpu();device=torch.device('cuda:0')
        a.output.mkdir(parents=True);r.copy_new(Path(__file__),a.output/'source.py')
        r.copy_new(a.baseline/'plan.json',a.output/'baseline_plan.json')
        r.LEARNING_RATE=3e-5
        declaration=dict(status='PREFLIGHT',gpu=gpu,additional_steps=r.MAX_STEPS,
            cumulative_weight_training_steps=30000,learning_rate=r.LEARNING_RATE,
            optimizer_reset=True,optimizer_state_resumed=False,full_model_trainable=True,
            baseline_checkpoint=str(checkpoint),baseline_checkpoint_sha256=r.digest(checkpoint),
            source_sha256=r.digest(Path(__file__)),synthetic_references_used=False,
            data_and_loss_unchanged=True,release_approved=False,expected_evaluation_count=665,
            limitation='Repeatedly observed DEV has no left-turn subgroup; no generalization or deployment approval.')
        r.write_new(a.output/'declaration.json',declaration)
        _,loaded,dev=r.verify_views(plan)
        payload=torch.load(checkpoint,map_location='cpu',weights_only=False)
        config=r.ModelConfig.from_mapping(payload['model_config']);initial=payload['model_state_dict']
        initial_hash=r.tensor_digest(initial)
        r.require(initial_hash==payload['final_model_state_sha256'],'initial model hash differs')
        r.write_new(a.output/'started.json',dict(status='FULL_MODEL_TRAINING',started_at_utc=r.utc_now(),
                                               train_count=len(loaded.examples),additional_steps=r.MAX_STEPS))
        print('AD_FULL_MODEL_TRAINING_STARTED',flush=True)
        training=r.fit_arm(arm,loaded,config,initial,initial_hash,a.output,device)
        evaluation=r.evaluate_final(arm,a.output/training['checkpoint'],dev['loaded'],config,a.output,device)
        comparison=r.paired_nonregression(original_evaluation,evaluation)
        r.write_new(a.output/'summary.json',dict(status='COMPLETE_NOT_PROMOTED',training=training,
            evaluation=evaluation,versus_original_k1=comparison,strict_gate=r.strict_gate(evaluation),
            additional_steps=r.MAX_STEPS,cumulative_weight_training_steps=30000,
            optimizer_reset=True,release_approved=False,completed_at_utc=r.utc_now()))
        print('AD_TRAINING_AND_DEV_COMPLETE_NOT_PROMOTED',flush=True)


if __name__=='__main__':main()
