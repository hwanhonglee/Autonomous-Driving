"""HH_261001 - Independent full240 replay rejects count-only, unsafe and partial evidence."""

import ast
from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
from portable_e2e.mapping import Observation
from scripts.e2e import verify_map_geometry_full_projection_20261001 as v


def grid():
    return v.p.new_grid('test','learned_prediction')


def test_exact_artifact_census():
    names = v.expected_artifacts()
    assert len(names) == 1455
    assert sum(name.endswith('_confidence.npz') for name in names) == 960
    assert sum(name.endswith('_frame.json') for name in names) == 480
    assert sum(name.endswith('_grid.npz') for name in names) == 6
    assert sum(name.endswith('.png') for name in names) == 8


def summary_fixture(monkeypatch):
    monkeypatch.setattr(v.p.c,'digest',lambda _: 'a'*64)
    registration={'bindings':{},'sources':{}}
    summary=dict(schema='portable_e2e.map_geometry_full_projection.v1',
        status='FULL_RECORDING_ORACLE_ASSISTED_ACCUMULATION_DIAGNOSTIC',preregistration_sha256='a'*64,
        policy=v.p.policy(),bindings={},sources={},scenarios={key:{} for key in v.p.SCENARIOS},
        attempted_forwards=960,confidence_forwards=960,oracle_frames=480,learned_map_updates=960,
        completed_frame_count=480,source_and_inputs_unchanged=True,evaluation_is_not_live=True,
        online_offline_equality='PENDING_SEPARATE_NO_FORWARD_REPLAY',
        artifact_sha256={name:'b'*64 for name in v.expected_artifacts()})
    return summary,registration,v.expected_artifacts()|{'summary.json'}


@pytest.mark.parametrize('key,value',[('attempted_forwards',961),('confidence_forwards',959),
    ('oracle_frames',479),('learned_map_updates',959),('completed_frame_count',479),
    ('online_offline_equality','PASS')])
def test_full_success_rejects_missing_or_inflated_stage_counters(monkeypatch,key,value):
    args=summary_fixture(monkeypatch)
    v.validate_summary(*args)
    args[0][key]=value
    with pytest.raises(ValueError): v.validate_summary(*args)


def test_manifest_missing_extra_or_failed_artifact_never_passes(monkeypatch):
    args=summary_fixture(monkeypatch)
    args[2].add('failure.json')
    with pytest.raises(ValueError): v.validate_summary(*args)
    args=summary_fixture(monkeypatch)
    del args[0]['artifact_sha256']['ctrack_left_239_geometry_confidence.npz']
    with pytest.raises(ValueError): v.validate_summary(*args)


def test_independent_frame_vote_not_dense_pixel_count_and_tie_order():
    actual,reference = grid(),grid()
    observations = [Observation(1.,'test','learned_prediction',((.01,.01,'road',.4),(.02,.02,'road',.8),
        (.03,.03,'sidewalk',.8),(.04,.04,'pole',0.))),
        Observation(2.,'test','learned_prediction',((.01,.01,'road',.6),(.02,.02,'sidewalk',.6)))]
    for observation in observations:
        v.independent_update(actual,observation); reference.update(observation)
        assert actual.snapshot() == reference.snapshot()
    assert actual.cells[(0,0)] == {'road':[2,1.4],'sidewalk':[2,1.4]}
    assert list(actual.cells[(0,0)]) == ['road','sidewalk']
    assert max(actual.cells[(0,0)],key=lambda label:actual.cells[(0,0)][label][1]) == 'road'


@pytest.mark.parametrize('observation',[
    Observation(1.,'wrong','learned_prediction',()),Observation(1.,'test','oracle_sensor',()),
    Observation(float('nan'),'test','learned_prediction',()),
    Observation(1.,'test','learned_prediction',((0.,0.,'vehicle',.8),)),
    Observation(1.,'test','learned_prediction',((0.,0.,'road',1.1),)),
])
def test_independent_update_rejects_invalid_context(observation):
    with pytest.raises(ValueError): v.independent_update(grid(),observation)


def test_incremental_hash_detects_equal_count_changed_weight():
    g = grid(); v.independent_update(g,Observation(1.,'test','learned_prediction',((0.,0.,'road',.8),)))
    row = dict(accumulated_cells={'control':1},accumulated_snapshot_sha256={'control':v.p.sha(v.p.encoded(g.snapshot()))})
    v.verify_incremental(row,{'control':g})
    g.cells[(0,0)]['road'][1] = .7
    with pytest.raises(ValueError,match='incremental confidence'): v.verify_incremental(row,{'control':g})


def test_full240_grid_counts_and_final_numeric_order_checked():
    g = grid()
    for i in range(240):
        v.independent_update(g,Observation(float(i),'test','learned_prediction',((0.,0.,'road',.5),(0.,0.,'sidewalk',.5))))
    arrays = v.p.pack_grid(g); meta = v.p.grid_meta(g,'test.npz')
    v.verify_final_grid(arrays,meta,g)
    assert arrays['observed_frames'][0,0] == 240
    altered = {key:value.copy() for key,value in arrays.items()}
    altered['label_order'][0,:2] = altered['label_order'][0,:2][::-1]
    with pytest.raises(ValueError,match='order/tie'): v.verify_final_grid(altered,meta,g)
    altered = {key:value.copy() for key,value in arrays.items()}; altered['observed_frames'][0,0] = 241
    with pytest.raises(ValueError): v.verify_final_grid(altered,meta,g)


def test_confidence_exact_numeric_storage_not_quantized(tmp_path):
    path = tmp_path/'confidence.npz'
    sampled = np.full((120,214),.7234567,np.float32)
    np.savez_compressed(path,confidence_stride3=sampled)
    projection = dict(confidence_file=path.name,confidence_numeric_sha256=v.p.sha(sampled.tobytes()),
        exact_saved_argmax=True,forward_and_projection_ms=1.)
    restored = v.restore_saved_confidence(path,projection)
    assert np.array_equal(restored[::3,::3],sampled) and np.count_nonzero(restored)==sampled.size
    with pytest.raises(ValueError): v.restore_saved_confidence(path,{**projection,'confidence_numeric_sha256':'wrong'})
    with pytest.raises(ValueError): v.restore_saved_confidence(path,{**projection,'exact_saved_argmax':False})


def test_overview_includes_far_earlier_pose_and_full_cell_extents():
    records = [dict(ego_x=0.,ego_y=0.) for _ in range(240)]
    records[0] = dict(ego_x=-100.,ego_y=-50.)
    g=grid();g.cells[(1000,1000)]={'road':[1,1.]}
    result=v.independent_overview(records,{'control':g})
    assert result['bounds_xy_m'] == [-101.,-51.,201.20000000000002,201.20000000000002]
    assert result['bounds_xy_m'] == v.p.overview_bounds(records,{'control':g})
    with pytest.raises(ValueError): v.independent_overview(records[1:],{'control':g})


def test_verifier_has_no_model_loader_forward_or_core_fusion_call():
    tree=ast.parse(Path(v.__file__).read_text())
    calls=[node.func for node in ast.walk(tree) if isinstance(node,ast.Call)]
    assert not any(isinstance(node,ast.Attribute) and node.attr in ('load_models','load_ah_weights','forward') for node in calls)
    updates=[node for node in calls if isinstance(node,ast.Attribute) and node.attr=='update']
    assert all(isinstance(node.value,ast.Name) and node.value.id in ('names','maps') for node in updates)
    assert not any(isinstance(node,(ast.Import,ast.ImportFrom)) and ('torch' in ast.unparse(node)) for node in ast.walk(tree))
