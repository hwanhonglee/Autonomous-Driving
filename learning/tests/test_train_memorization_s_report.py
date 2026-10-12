"""HH_260906 - Reject mismatched diagnostic pairs and invalid metrics before claiming training-fit changes."""
import copy
import pytest
from scripts.e2e.summarize_train_memorization_s import paired_groups


def rows():
    return [dict(token=f'{g}:{i}',episode=g,group=g,selected_ade_m=1.,oracle_ade_m=.5,
        speed_mae_mps=.2,constant_speed_route_ade_m=2.) for g in ('straight','left','right') for i in range(4)]


def test_identical_pairs():
    a=rows();b=copy.deepcopy(a);b[0]['selected_ade_m']=.5
    assert paired_groups(a,b)['straight']['selected_ade_m']['after']==.875


@pytest.mark.parametrize('fault',['token','duplicate','reference','nan'])
def test_invalid_pairs(fault):
    a=rows();b=copy.deepcopy(a)
    if fault=='token':b[0]['token']='wrong'
    if fault=='duplicate':a[0]['token']=a[1]['token'];b[0]['token']=b[1]['token']
    if fault=='reference':b[0]['constant_speed_route_ade_m']=3.
    if fault=='nan':b[0]['selected_ade_m']=float('nan')
    with pytest.raises(ValueError):paired_groups(a,b)
