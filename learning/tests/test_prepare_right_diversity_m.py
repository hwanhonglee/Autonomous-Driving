"""HH_260906 - Keep native identity support separate from admission and geometry checks."""
from types import SimpleNamespace
from pathlib import Path
import pytest
from scripts.e2e import prepare_right_diversity_m as m


def test_identity_restored_after_exception():
    """HH_260906 - Never leave global adapter behavior changed after a failed conversion."""
    original=m.base._native_route_id
    with pytest.raises(RuntimeError):
        with m.native_identity([({'town':'Town01'},'town01_right_s0000_p02')]):
            assert m.base._native_route_id({'town':'Town01'}) == 'town01_right_s0000_p02'
            raise RuntimeError('intentional')
    assert m.base._native_route_id is original


@pytest.mark.parametrize('routes,query', [([],{}),([({'x':1},'a')],{'x':2}),
    ([({'x':1},'a'),({'x':1},'b')],{'x':1})])
def test_unknown_or_ambiguous_route_rejected(routes,query):
    """HH_260906 - Require one exact authenticated route match."""
    with m.native_identity(routes),pytest.raises(m.base.AdapterError):
        m.base._native_route_id(query)


@pytest.mark.parametrize('split,path', [('val',m.TRIAL_ROOT/m.TRIALS[0]/'episode'),
    ('test',m.TRIAL_ROOT/m.TRIALS[0]/'episode'),('train',Path('/tmp/unapproved'))])
def test_unapproved_scope_rejected(split,path):
    """HH_260906 - Reject DEV, TEST and unknown sources before reading payloads."""
    with pytest.raises(m.base.AdapterError):
        m.authorized_routes([SimpleNamespace(split=split,path=path)])


def test_denial_stays_enforced():
    """HH_260906 - Naming support must not override an explicit diagnostic-only marker."""
    with m.native_identity([({},'a')]),pytest.raises(m.base.AdapterError):
        m.base._require_native_admission_not_denied({'capture_contract':
            {'agent_initialization':{'training_data_approved':False}}})
