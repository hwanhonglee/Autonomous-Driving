"""HH_261001 - Bound full-recording map evidence without changing projection or fitting models."""

from io import BytesIO
from types import SimpleNamespace
import numpy as np
from PIL import Image
import pytest
from portable_e2e.mapping import Observation
from scripts.e2e import project_map_geometry_full_20261001 as p


def test_exact_fixed_denominators_and_nonqualification_boundary():
    value = p.policy()
    assert value['frame_indices'] == list(range(240))
    assert value['confidence_forwards'] == 960 and value['oracle_frames'] == 480
    assert value['learned_map_updates'] == 960 and value['final_grids'] == 6 and value['pngs'] == 8
    assert value['fixed_png_indices'] == [0,119,239] and value['skipped_frames'] == 0
    assert value['confidence_raw_bytes'] == 98611200
    assert value['maximum_output_bytes'] == 128*1024**2 and value['free_disk_floor_bytes'] == 5*1024**3
    assert value['execute_deadline_seconds'] == value['independent_replay_deadline_seconds'] == 1800
    assert value['minimum_confidence'] == .5 and value['resolution_m'] == .2
    assert value['stride'] == 3 and value['max_cells'] == 200000 and value['ground_height_filter'] is None
    assert not any(value[key] for key in ('gpu_used','training_steps','simulator_started','ctrack_train_eligible',
        'model_promotion','independent_validation','hd_map_approved'))


def test_historical_core_objects_are_reused_unchanged():
    assert p.project_predicted_frame is p.previous.project_predicted_frame
    assert p.project_frame is p.previous.project_frame and p.new_grid is p.previous.new_grid
    assert p.pack_grid is p.previous.pack_grid and p.restore_confidence is p.previous.restore_confidence


def test_full240_codec_preserves_ties_all_counts_and_float64_sums():
    grid = p.new_grid('test','learned_prediction')
    for index in range(240):
        grid.update(Observation(float(index),'test','learned_prediction',
            ((.1,.1,'sidewalk',.5),(.1,.1,'road',.5),(.1,.1,'road',.4))))
    packed = p.pack_grid(grid); recovered = p.unpack_grid(packed,p.grid_meta(grid,'grid.npz'))
    assert recovered.snapshot() == grid.snapshot()
    assert list(recovered.cells[(0,0)]) == ['sidewalk','road']
    assert recovered.cells[(0,0)]['road'] == [240,120.]
    assert packed['observed_frames'].dtype == np.int16 and packed['confidence_sum'].dtype == np.float64


@pytest.mark.parametrize('mutation',('over240','negative','dtype','nan','missing','duplicate_order','unused_sum'))
def test_grid_codec_rejects_changed_or_unsafe_arrays(mutation):
    grid = p.new_grid('test','learned_prediction')
    grid.update(Observation(1.,'test','learned_prediction',((.1,.1,'road',.8),)))
    arrays = p.pack_grid(grid)
    if mutation == 'over240': arrays['observed_frames'][0,0] = 241
    elif mutation == 'negative': arrays['observed_frames'][0,0] = -1
    elif mutation == 'dtype': arrays['confidence_sum'] = arrays['confidence_sum'].astype(np.float32)
    elif mutation == 'nan': arrays['confidence_sum'][0,0] = np.nan
    elif mutation == 'missing': del arrays['label_order']
    elif mutation == 'duplicate_order': arrays['label_order'][0,1] = 0
    elif mutation == 'unused_sum': arrays['confidence_sum'][0,1] = .1
    with pytest.raises(ValueError): p.unpack_grid(arrays,p.grid_meta(grid,'grid.npz'))


def test_every_pixel_argmax_must_match_not_just_projection_lattice():
    prediction = np.zeros((360,640),np.uint8)
    p.assert_argmax(prediction.astype(np.int64),prediction.astype(np.int64),prediction)
    altered = prediction.copy(); altered[1,1] = 1
    with pytest.raises(ValueError): p.assert_argmax(altered,prediction,prediction)
    with pytest.raises(ValueError): p.assert_argmax(prediction,altered,prediction)


def test_lattice_float32_recovery_preserves_every_projected_point():
    rng = np.random.default_rng(261001)
    confidence = rng.random((360,640),dtype=np.float32)
    labels = rng.integers(0,7,(360,640),np.uint8)
    depth = np.full((360,640),10,np.float32)
    kwargs = dict(frame_id='test',timestamp_s=1.,fov_deg=90,label_table=p.TABLE,stride=3,max_depth_m=40,min_confidence=.5)
    actual = p.project_predicted_frame(depth,labels,confidence,np.eye(4),**kwargs)
    replay = p.project_predicted_frame(depth,labels,p.restore_confidence(confidence[::3,::3].copy()),np.eye(4),**kwargs)
    assert actual == replay
    with pytest.raises(ValueError): p.restore_confidence(confidence[::3,::3].astype(np.float16))


def test_capacity_failure_preserves_preceding_map_without_eviction():
    grid = p.new_grid('test','learned_prediction'); grid.max_cells = 1
    grid.update(Observation(1.,'test','learned_prediction',((.1,.1,'road',.7),)))
    before = grid.snapshot()
    with pytest.raises(ValueError):
        grid.update(Observation(2.,'test','learned_prediction',((100.,100.,'road',.9),)))
    assert grid.snapshot() == before


def test_overview_bounds_include_all_cell_footprints_and_all_poses():
    grid = p.new_grid('test','oracle_sensor')
    grid.update(Observation(1.,'test','oracle_sensor',((-100.,-200.,'road',1.),(500.,600.,'sidewalk',1.))))
    records = [dict(ego_x=0.,ego_y=0.,ego_yaw_deg=0.) for _ in range(240)]
    records[100]['ego_x'] = -600.; records[101]['ego_y'] = 1000.
    bounds = p.overview_bounds(records,{'oracle':grid})
    assert bounds == [-601.,-201.,501.20000000000005,1001.]
    assert bounds[0] < -600 and bounds[1] < -200 and bounds[2] > 500.2 and bounds[3] > 1000
    with pytest.raises(ValueError): p.overview_bounds(records[:-1],{'oracle':grid})


def test_both_visualizers_are_deterministic_use_three_maps_and_preserve_unknown():
    records = [dict(ego_x=i*.2,ego_y=0.,ego_yaw_deg=0.) for i in range(240)]
    maps = {'oracle':p.new_grid('test','oracle_sensor')}
    maps.update({arm:p.new_grid('test','learned_prediction') for arm in p.ARMS})
    fixed = p.render_fixed('test',119,records[119],maps)
    assert fixed == p.render_fixed('test',119,records[119],maps)
    image = np.asarray(Image.open(BytesIO(fixed)))
    assert image.shape == (490,1920,3)
    for column in range(3):
        assert tuple(image[255,column*640+320]) == (255,110,65)
        assert tuple(image[61,column*640+1]) == (22,31,43)
    png,meta = p.render_overview('test',records,maps)
    assert (png,meta) == p.render_overview('test',records,maps)
    assert Image.open(BytesIO(png)).size == (1920,710)
    assert meta['pose_count'] == 240 and meta['all_stored_cell_extents_included']


def _writable(monkeypatch,tmp_path,free):
    output = tmp_path/'run'; output.mkdir()
    monkeypatch.setattr(p,'OUTPUT',output)
    monkeypatch.setattr(p.shutil,'disk_usage',lambda path:SimpleNamespace(free=free))
    return output


def test_writes_are_fresh_bounded_and_no_parent_path_escape(monkeypatch,tmp_path):
    output = _writable(monkeypatch,tmp_path,p.FLOOR+p.LIMIT)
    p.write_artifact('evidence.json',b'123')
    with pytest.raises(FileExistsError): p.write_artifact('evidence.json',b'456')
    with pytest.raises(ValueError): p.write_artifact('../escape.json',b'1')
    assert (output/'evidence.json').read_bytes() == b'123'


def test_aggregate_reserve_and_disk_floor_never_silently_relax(monkeypatch,tmp_path):
    output = _writable(monkeypatch,tmp_path,p.FLOOR+p.LIMIT)
    monkeypatch.setattr(p,'LIMIT',100); monkeypatch.setattr(p,'FAILURE_RESERVE',20)
    p.write_artifact('partial.npz',b'x'*80)
    with pytest.raises(ValueError): p.write_artifact('beyond.json',b'1')
    p.write_artifact('failure.json',b'x'*20,failure=True)
    assert p.used_bytes() == 100 and (output/'partial.npz').exists()
    with pytest.raises(ValueError): p.write_artifact('extra.json',b'1',failure=True)


def test_failure_receipt_cannot_cross_five_gib_floor(monkeypatch,tmp_path):
    output = _writable(monkeypatch,tmp_path,p.FLOOR)
    with pytest.raises(ValueError): p.write_artifact('failure.json',b'1',failure=True)
    assert list(output.iterdir()) == []


def test_output_symlinks_are_rejected(monkeypatch,tmp_path):
    output = _writable(monkeypatch,tmp_path,p.FLOOR+p.LIMIT)
    (tmp_path/'data').write_bytes(b'x'); (output/'link').symlink_to(tmp_path/'data')
    with pytest.raises(ValueError): p.used_bytes()


def test_run_failure_is_retained_not_returned_as_success(monkeypatch,tmp_path):
    docs = tmp_path/'docs'; docs.mkdir(); prereg = docs/'prereg.json'; prereg.write_bytes(b'{}')
    output = docs/'run'
    monkeypatch.setattr(p,'DOCS',docs); monkeypatch.setattr(p,'PREREG',prereg); monkeypatch.setattr(p,'OUTPUT',output)
    monkeypatch.setattr(p.shutil,'disk_usage',lambda path:SimpleNamespace(free=p.FLOOR+p.LIMIT))
    monkeypatch.setattr(p,'validate',lambda:dict(bindings=dict(fit_artifacts={}),sources={}))
    def no_models(binding): raise RuntimeError('injected pre-inference failure')
    monkeypatch.setattr(p.endpoint,'load_models',no_models)
    with pytest.raises(RuntimeError): p.run()
    receipt = p.c.read_json(output/'failure.json')
    assert (output/'started.json').exists() and not (output/'summary.json').exists()
    assert receipt['status'] == 'FAILED_PRESERVED_NO_PARTIAL_PASS' and receipt['confidence_forwards'] == 0
    assert receipt['partial_artifacts_retained'] and receipt['no_resume']


def test_timer_raises_without_changing_recipe():
    with pytest.raises(TimeoutError): p._timeout(None,None)


def test_completed_forward_is_counted_before_subsequent_argmax_or_write_failure():
    progress = dict(attempted_forwards=0,completed_forwards=0,current={})
    assert p.counted_forward(lambda value:value+1,4,progress) == 5
    assert progress['attempted_forwards'] == progress['completed_forwards'] == 1
    prediction = np.zeros((360,640),np.uint8); altered = prediction.copy(); altered[0,0] = 1
    with pytest.raises(ValueError): p.assert_argmax(altered,prediction,prediction)
    assert progress['completed_forwards'] == 1 and progress['current']['stage'] == 'model_call_returned'


def test_failed_model_call_counts_attempt_but_not_completed_forward():
    progress = dict(attempted_forwards=0,completed_forwards=0,current={})
    def failed(value): raise RuntimeError('model call failed')
    with pytest.raises(RuntimeError): p.counted_forward(failed,None,progress)
    assert progress['attempted_forwards'] == 1 and progress['completed_forwards'] == 0
    assert progress['current']['stage'] == 'model_call_attempted'
