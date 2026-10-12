"""HH_260906 - Verify causal geometry scoring, zero-residual parity and candidate permutation equivariance."""
import torch
from portable_e2e.model import ModelConfig, PerspectiveTrajectoryModel, PHYSICAL_STOPMIX_MODEL_ID
from portable_e2e.candidate_geometry_recovery import CandidateGeometryRecoveryModel


def config():
    """HH_260906 - Keep the production-shaped STOPMIX research ABI."""
    return ModelConfig(model_id=PHYSICAL_STOPMIX_MODEL_ID,candidate_count=12,maximum_step_m=1.0)


def test_zero_residual_and_shared_initial_weights():
    """HH_260906 - Ensure the only initial difference is dormant extra scoring capacity."""
    torch.manual_seed(20260906)
    base=PerspectiveTrajectoryModel(config())
    torch.manual_seed(20260906)
    model=CandidateGeometryRecoveryModel(config())
    for key,value in base.state_dict().items():
        assert torch.equal(value,model.state_dict()[key])
    residual=model.geometry_residual(torch.randn(2,256),torch.randn(2,12,64,2),torch.randn(2,12,64))
    assert torch.count_nonzero(residual)==0


def test_shared_scoring_permutation_and_detached_geometry():
    """HH_260906 - Score trajectories rather than index identities without leaking rank gradients into geometry."""
    model=CandidateGeometryRecoveryModel(config())
    torch.nn.init.normal_(model.geometry_ranker[-1].weight)
    context=torch.randn(2,256,requires_grad=True)
    xy=torch.randn(2,12,64,2,requires_grad=True)
    speed=torch.randn(2,12,64,requires_grad=True)
    permutation=torch.randperm(12)
    scores=model.geometry_residual(context,xy,speed)
    reordered=model.geometry_residual(context,xy[:,permutation],speed[:,permutation])
    assert torch.allclose(reordered,scores[:,permutation],atol=1e-6)
    scores.sum().backward()
    assert context.grad is not None
    assert xy.grad is None and speed.grad is None
    assert model.geometry_ranker[-1].weight.grad is not None


def test_forward_parity_and_full_trainability():
    """HH_260906 - Preserve complete initial forward outputs with no extra inference inputs."""
    torch.set_num_threads(2)
    cfg=config()
    torch.manual_seed(20260906); base=PerspectiveTrajectoryModel(cfg).eval()
    torch.manual_seed(20260906); model=CandidateGeometryRecoveryModel(cfg).eval()
    images=torch.zeros(1,6,3,180,320)
    calibration=torch.zeros(1,6,16)
    ego=torch.zeros(1,10,13)
    history_mask=torch.ones(1,10,dtype=torch.bool)
    route=torch.stack((torch.arange(128),torch.zeros(128)),dim=-1).float()[None]
    route_mask=torch.ones(1,128,dtype=torch.bool)
    args=(images,calibration,ego,history_mask,route,route_mask)
    with torch.no_grad():
        for a,b in zip(base(*args),model(*args)): assert torch.equal(a,b)
    assert all(p.requires_grad for p in model.parameters())
    assert model._rank_context is None
