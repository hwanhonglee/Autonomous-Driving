"""HH_261001 - Permit only the predeclared cap pair and preserve the common teacher contract."""

import pytest
from scripts.e2e import research_w1_speed_cap_ab_20261001 as study
from scripts.e2e import collect_w1_town07_long_20261001 as long
from scripts.e2e import collect_ah_simulation_research as short
from types import SimpleNamespace
from portable_e2e.simulation_research import ControllerConfig


def valid(arm):
    return SimpleNamespace(host='127.0.0.1',port=2440,allow_map_load=False,
        control_transport='acknowledged_batch',goal_stop_profile='disabled',physics_hz=20,capture_hz=10,
        stationary_warmup_sec=1.2,max_duration_sec=20,research_speed_cap_mps=study.CAPS[arm],
        target_speed_kmh=30,research_max_cte_m=1.,weather='ClearNoon',seed=0,
        output=study.output(arm)/'episode')


@pytest.mark.parametrize('arm', ['C9','C30'])
def test_both_caps_within_original_runtime(arm):
    study.validate(valid(arm),dict(town='Town07',scenario='straight'),arm)
    ControllerConfig(2.82,1.22,speed_cap_mps=study.CAPS[arm]).validate()


@pytest.mark.parametrize('field,value', [('target_speed_kmh',9),('research_speed_cap_mps',60/3.6),
    ('research_max_cte_m',2),('max_duration_sec',21),('goal_stop_profile','comfortable_v2')])
def test_other_regimes_forbidden(field,value):
    args=valid('C9');setattr(args,field,value)
    with pytest.raises(ValueError):
        study.validate(args,dict(town='Town07',scenario='straight'),'C9')


def test_scope_does_not_change_long_worker():
    assert long.BUDGET == 150 and short.__name__.endswith('collect_ah_simulation_research')
    with pytest.raises(ValueError):
        study.output('C60')
