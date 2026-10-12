"""HH_260906 - Pin the endpoint/interval distinction without weakening runtime acceptance."""
import pytest
from scripts.e2e.localize_curve_speed_disagreement import interval_metrics
from portable_e2e.contract import ContractError
from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory


def test_constant_deceleration_endpoint_bias():
    """HH_260906 - Exact constant deceleration creates half-step endpoint bias with zero trapezoid error."""
    a=dict(timestamp=0.,frame=0,x=0.,y=0.,vx=8.,vy=0.)
    b=dict(timestamp=.1,frame=2,x=.7725,y=0.,vx=7.45,vy=0.)
    metrics=interval_metrics(a,b)
    assert metrics['endpoint_error_mps']==pytest.approx(.275)
    assert metrics['trapezoid_error_mps']==pytest.approx(0.)
    assert metrics['measured_speed_rate_mps2']==pytest.approx(-5.5)


def test_existing_gate_rejects_physically_bounded_endpoint_fixture():
    """HH_260906 - Preserve the current gate result while exposing a contract-semantic counterexample."""
    times=[.1*(i+1) for i in range(64)]
    stop=8./5.5
    positions=[[8.*min(t,stop)-.5*5.5*min(t,stop)**2,0.] for t in times]
    speeds=[max(0.,8.-5.5*t) for t in times]
    with pytest.raises(ContractError,match='speed 0 disagrees with its displacement'):
        validate_and_select_trajectory([positions],[speeds],[0.],RuntimeGateConfig(candidate_count=1),
                                       current_speed_mps=8.)


def test_nonpositive_interval_is_rejected():
    with pytest.raises(ValueError,match='nonpositive'):
        interval_metrics({'timestamp':1.},{'timestamp':1.})
