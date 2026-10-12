#!/usr/bin/env python3
"""HH_260906 - Seal and run one whole-model geometry-scoring experiment against completed K1."""
from contextlib import contextmanager
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import torch
from portable_e2e.model import ModelConfig,parameter_count
from portable_e2e.candidate_geometry_recovery import CandidateGeometryRecoveryModel
from scripts.e2e import run_moving_multiroute_recovery_k as r

SCHEMA='portable_e2e.geometry_recovery_l.v1'
CHECKPOINT_SCHEMA='portable_e2e.geometry_recovery_l_checkpoint.v1'
SOURCES=(*r.SOURCE_RELATIVES,r.MODEL_RELATIVE,'portable_e2e/config/common_10hz_v1.contract.json',
         'portable_e2e/candidate_geometry_recovery.py','scripts/e2e/run_candidate_geometry_recovery_l.py')
ARM=r.ARMS[1]
PARAMETERS=1_113_963


def validate_experiment_contract(plan, old):
    """HH_260906 - Reject changes beyond the declared architecture ablation."""
    expected=dict(steps=15000,samples_seen=60000,seed=r.SEED,batch_size=r.BATCH_SIZE,
        loss_config=r.LOSS_CONFIG.to_dict(),learning_rate=r.LEARNING_RATE,weight_decay=r.WEIGHT_DECAY,
        gradient_clip=r.MAXIMUM_GRADIENT_NORM,parameter_count=PARAMETERS,checkpoint_schema=CHECKPOINT_SCHEMA,
        same_dataset_sampler_loss_optimizer_seed_steps=True,final_dev_only=True,
        original_decision_modified=False,release_approved=False)
    for key,value in expected.items(): r.require(plan.get(key)==value,'L fixed contract differs: '+key)
    r.require(plan['views']=={ARM:old['views'][ARM],'Town03_DEV':old['views']['Town03_DEV']},'baseline views changed')
    r.require(plan['sampling']==old['sampling'][ARM],'baseline sampling changed')


@contextmanager
def research_model_binding():
    """HH_260906 - Reuse K optimization math in-process while giving L a nondeployable checkpoint schema."""
    replacements=dict(PerspectiveTrajectoryModel=CandidateGeometryRecoveryModel,
        EXPECTED_PARAMETER_COUNT=PARAMETERS,CHECKPOINT_SCHEMA=CHECKPOINT_SCHEMA,REPORT_SCHEMA=SCHEMA)
    previous={key:getattr(r,key) for key in replacements}
    try:
        for key,value in replacements.items(): setattr(r,key,value)
        yield
    finally:
        for key,value in previous.items(): setattr(r,key,value)


def validated_baseline(root):
    """HH_260906 - Bind the previously completed K1 baseline without changing its decisions."""
    plan=r.read_json(root/'plan.json'); summary=r.read_json(root/'summary.json')
    r.require(summary['status']=='COMPLETE_NOT_PROMOTED','completed K baseline required')
    r.require(r.digest(root/'plan.json')==summary['plan_sha256'],'baseline plan digest differs')
    r.require(r.digest(root/'decision.json')==summary['decision_sha256'],'baseline decision changed')
    for name,expected in plan['source_sha256'].items():
        r.require(r.digest(ROOT/name)==expected,'original optimization source differs: '+name)
    ref=summary['evaluations'][ARM]
    r.require(r.digest(root/ref['path'])==ref['sha256'],'baseline evaluation changed')
    r.require(summary['training'][ARM]['global_step']==15000,'baseline training budget differs')
    return plan,summary


def prepare(baseline,output):
    """HH_260906 - Freeze the single architecture change before L training or DEV inference."""
    old,summary=validated_baseline(baseline)
    train,examples,_=r._load_view(Path(old['views'][ARM]['path']),'train',ARM)
    r._verify_loaded_view(old['views'][ARM],train,examples)
    dev,dev_examples,_=r._load_view(Path(old['views']['Town03_DEV']['path']),'val','Town03_DEV')
    r._verify_loaded_view(old['views']['Town03_DEV'],dev,dev_examples)
    r.require(r.schedule_contract(examples,ARM)==old['sampling'][ARM],'sampling differs from K1')
    config=ModelConfig.from_mapping(r.read_json(ROOT/r.MODEL_RELATIVE))
    r._seed_everything(r.SEED,torch.device('cpu'))
    model=CandidateGeometryRecoveryModel(config)
    r.require(parameter_count(model)==PARAMETERS,'L parameter count differs')
    base={key:value for key,value in model.state_dict().items() if not key.startswith('geometry_ranker.')}
    r.require(r.tensor_digest(base)==old['model']['seeded_initial_model_state_sha256'],'base initialization differs')
    return dict(schema=SCHEMA,declared_at_utc=r.utc_now(),repo_root=str(ROOT),output_root=str(output.absolute()),
        baseline_root=str(baseline.absolute()),baseline_summary_sha256=r.digest(baseline/'summary.json'),
        source_sha256={name:r.digest(ROOT/name) for name in SOURCES},
        initial_model_state_sha256=r.tensor_digest(model.state_dict()),base_initial_state_sha256=r.tensor_digest(base),
        model_class='CandidateGeometryRecoveryModel',parameter_count=PARAMETERS,checkpoint_schema=CHECKPOINT_SCHEMA,
        single_change='zero-initialized shared candidate geometry residual on existing logits',
        same_dataset_sampler_loss_optimizer_seed_steps=True,steps=15000,samples_seen=60000,
        seed=r.SEED,batch_size=r.BATCH_SIZE,loss_config=r.LOSS_CONFIG.to_dict(),
        learning_rate=r.LEARNING_RATE,weight_decay=r.WEIGHT_DECAY,gradient_clip=r.MAXIMUM_GRADIENT_NORM,
        views={ARM:old['views'][ARM],'Town03_DEV':old['views']['Town03_DEV']},sampling=old['sampling'][ARM],
        final_dev_only=True,original_decision_modified=False,release_approved=False)


def run(plan_path):
    """HH_260906 - Train all L parameters once, then evaluate and keep every failed result."""
    plan=r.read_json(plan_path)
    r.require(plan['schema']==SCHEMA and plan['repo_root']==str(ROOT),'L plan scope differs')
    r.require(plan['steps']==15000 and plan['seed']==r.SEED and plan['loss_config']==r.LOSS_CONFIG.to_dict(),
              'L training contract differs')
    r.require(plan['source_sha256']=={name:r.digest(ROOT/name) for name in SOURCES},'L source changed')
    baseline=Path(plan['baseline_root']); old,summary=validated_baseline(baseline)
    validate_experiment_contract(plan,old)
    r.require(r.digest(baseline/'summary.json')==plan['baseline_summary_sha256'],'baseline summary changed')
    workspace=r.check_run_environment(plan)
    output=Path(plan['output_root'])
    r.require(not output.exists(),'fresh L output required')
    with r.gpu0_lease(workspace):
        gpu=r.preflight_gpu()
        loaded,examples,_=r._load_view(Path(plan['views'][ARM]['path']),'train',ARM)
        r._verify_loaded_view(plan['views'][ARM],loaded,examples)
        r.require(r.schedule_contract(examples,ARM)==plan['sampling']==old['sampling'][ARM],'sampler mismatch')
        dev,dev_examples,_=r._load_view(Path(plan['views']['Town03_DEV']['path']),'val','Town03_DEV')
        r._verify_loaded_view(plan['views']['Town03_DEV'],dev,dev_examples)
        config=ModelConfig.from_mapping(r.read_json(ROOT/r.MODEL_RELATIVE))
        device=torch.device('cuda:0')
        r._seed_everything(r.SEED,device)
        model=CandidateGeometryRecoveryModel(config)
        initial=r._clone_state(model.state_dict()); initial_sha=r.tensor_digest(initial)
        r.require(initial_sha==plan['initial_model_state_sha256'],'L initialization changed')
        del model
        output.mkdir(parents=True)
        r.copy_new(plan_path,output/'plan.json')
        for name in SOURCES: r.copy_new(ROOT/name,output/'source'/name)
        r.write_new(output/'started.json',dict(schema=SCHEMA,started_at_utc=r.utc_now(),gpu=gpu,
            research_only=True,base_initial_state_sha256=plan['base_initial_state_sha256']))
        try:
            with research_model_binding():
                training=r.fit_arm(ARM,loaded,config,initial,initial_sha,output,device)
                evaluation=r.evaluate_final(ARM,output/training['checkpoint'],dev,config,output,device)
            baseline_eval=r.read_json(baseline/summary['evaluations'][ARM]['path'])
            strict=r.strict_gate(evaluation)
            relative=r.paired_nonregression(baseline_eval,evaluation)
            passed=strict['pass'] and relative['pass']
            decision=dict(status='PASS_L_RESEARCH_SIGNAL_NOT_PROMOTED' if passed else 'FAIL_L_RESEARCH_SIGNAL_NOT_PROMOTED',
                pass_all=passed,strict=strict,relative_to_K1=relative,
                comparison_field_mapping={'K0':'completed K1 baseline','K1':'L geometry-scoring model'},
                promotion_eligible=False,closed_loop_performed=False,dev_is_repeatedly_observed=True)
            r.write_new(output/'decision.json',decision)
            result=dict(schema=SCHEMA,status='COMPLETE_NOT_PROMOTED',completed_at_utc=r.utc_now(),
                training=training,evaluation_sha256=r.digest(output/ARM/'final_dev_evaluation.json'),
                plan_sha256=r.digest(output/'plan.json'),decision_sha256=r.digest(output/'decision.json'))
            r.write_new(output/'summary.json',result)
            return decision
        except Exception as error:
            r.write_new(output/'failed.json',dict(error_type=type(error).__name__,error=str(error),
                failed_at_utc=r.utc_now(),automatic_retry=False))
            raise


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    preflight=commands.add_parser('preflight')
    preflight.add_argument('--baseline-root',type=Path,required=True)
    preflight.add_argument('--output-root',type=Path,required=True)
    preflight.add_argument('--plan',type=Path,required=True)
    execute=commands.add_parser('run'); execute.add_argument('--plan',type=Path,required=True)
    args=parser.parse_args()
    if args.command=='preflight':
        plan=prepare(args.baseline_root,args.output_root)
        r.write_new(args.plan,plan)
        print(json.dumps(dict(status='L_PLAN_DECLARED',plan=str(args.plan),sha256=r.digest(args.plan))))
    else:
        print(json.dumps(run(args.plan)))
