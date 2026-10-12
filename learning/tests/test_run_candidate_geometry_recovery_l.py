"""HH_260906 - Keep the geometry ablation bounded and restore historical runner bindings."""
import pytest
from scripts.e2e import run_candidate_geometry_recovery_l as m


def test_binding_restores_after_error():
    """HH_260906 - Do not leave the original K model or checkpoint schema modified."""
    original=(m.r.PerspectiveTrajectoryModel,m.r.CHECKPOINT_SCHEMA,m.r.EXPECTED_PARAMETER_COUNT)
    with pytest.raises(RuntimeError):
        with m.research_model_binding():
            assert m.r.CHECKPOINT_SCHEMA==m.CHECKPOINT_SCHEMA
            assert m.r.PerspectiveTrajectoryModel==m.CandidateGeometryRecoveryModel
            raise RuntimeError('test')
    assert original==(m.r.PerspectiveTrajectoryModel,m.r.CHECKPOINT_SCHEMA,m.r.EXPECTED_PARAMETER_COUNT)


def plan_pair():
    """HH_260906 - Model all fixed scalar fields without loading any dataset or checkpoint."""
    old={'views':{m.ARM:{'sha':'a'},'Town03_DEV':{'sha':'b'}},'sampling':{m.ARM:{'steps':15000}}}
    plan=dict(steps=15000,samples_seen=60000,seed=m.r.SEED,batch_size=m.r.BATCH_SIZE,
        loss_config=m.r.LOSS_CONFIG.to_dict(),learning_rate=m.r.LEARNING_RATE,weight_decay=m.r.WEIGHT_DECAY,
        gradient_clip=m.r.MAXIMUM_GRADIENT_NORM,parameter_count=m.PARAMETERS,checkpoint_schema=m.CHECKPOINT_SCHEMA,
        same_dataset_sampler_loss_optimizer_seed_steps=True,final_dev_only=True,
        original_decision_modified=False,release_approved=False,views=old['views'],sampling=old['sampling'][m.ARM])
    return plan,old


@pytest.mark.parametrize('key,value',[('steps',10),('release_approved',True),('parameter_count',5),
    ('learning_rate',1.0),('final_dev_only',False),('views',{}),('sampling',{})])
def test_rejects_extra_changes(key,value):
    """HH_260906 - Prevent turning one architecture experiment into an untracked multi-variable search."""
    plan,old=plan_pair()
    plan[key]=value
    with pytest.raises(ValueError): m.validate_experiment_contract(plan,old)


def test_fixed_contract_passes():
    """HH_260906 - Accept only the declared fixed comparison contract."""
    m.validate_experiment_contract(*plan_pair())
