"""HH_261001 - Pin the longer protocol and ensure sampled observation never changes policy outputs."""

from contextlib import contextmanager
from types import SimpleNamespace
import pytest
from scripts.e2e import collect_w1_town07_long_20261001 as worker


def valid():
    return SimpleNamespace(host='127.0.0.1', port=2440, allow_map_load=False,
        control_transport='acknowledged_batch', goal_stop_profile='disabled',
        physics_hz=20, capture_hz=10, stationary_warmup_sec=1.2,
        max_duration_sec=150., research_speed_cap_mps=2.5, target_speed_kmh=9,
        research_max_cte_m=1., weather='ClearNoon', seed=0,
        research_weights_sha256=worker.WEIGHTS_SHA,
        research_metadata_sha256=worker.METADATA_SHA, output=worker.OUTPUT/'episode')


@pytest.mark.parametrize('field,value', [('max_duration_sec',151), ('target_speed_kmh',30),
    ('research_max_cte_m',2), ('allow_map_load',True), ('research_weights_sha256','0'*64)])
def test_prospective_bounds_fixed(field, value):
    args = valid()
    worker.validate(args, dict(town='Town07',scenario='straight'))
    setattr(args,field,value)
    with pytest.raises(ValueError):
        worker.validate(args, dict(town='Town07',scenario='straight'))


def test_no_turn_or_unseen_map_scope():
    for route in (dict(town='Town03',scenario='straight'),dict(town='Town07',scenario='turn')):
        with pytest.raises(ValueError):
            worker.validate(valid(),route)


def test_fixed_sparse_schedule_and_exception_restore(monkeypatch, tmp_path):
    calls, captures = [], []
    objects = [object() for _ in range(25)]

    def original(index):
        calls.append(index)
        return objects[index]

    monkeypatch.setattr(worker.research.SimulationResearchPolicy,'step',original)

    @contextmanager
    def archive(directory):
        def observe(index):
            captures.append(index)
            return original(index)
        worker.research.SimulationResearchPolicy.step = observe
        try:
            yield
        finally:
            worker.research.SimulationResearchPolicy.step = original

    monkeypatch.setattr(worker,'exact_tensor_archive',archive)
    with pytest.raises(RuntimeError):
        with worker.sampled_inputs(tmp_path):
            for index in range(25):
                assert worker.research.SimulationResearchPolicy.step(index) is objects[index]
            raise RuntimeError('expected')
    assert calls == list(range(25)) and captures == [0,10,20]
    assert worker.research.SimulationResearchPolicy.step is original
