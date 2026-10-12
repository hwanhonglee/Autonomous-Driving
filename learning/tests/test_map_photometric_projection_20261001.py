"""HH_261001 - Test unchanged confidence filtering, sparse storage and unknown-preserving evidence."""

import numpy as np
import pytest
from portable_e2e.mapping import Observation
from scripts.e2e import project_map_photometric_fixed_20261001 as p


def test_fixed_policy_has_no_new_semantic_or_height_gate():
    policy = p.policy()
    assert policy['indices'] == [0,119,239] and policy['forwards'] == 36
    assert policy['minimum_confidence'] == .5 and policy['depth_strict_range_m'] == [.1,40]
    assert policy['ground_height_filter'] is None and policy['skipped_frames_per_capture'] == 237
    assert not policy['ctrack_train_eligible'] and not policy['hd_map_approved']


def test_sparse_confidence_projects_exactly_same_points():
    rng = np.random.default_rng(261001)
    confidence = rng.random((360,640),dtype=np.float32)
    classes = rng.integers(0,7,(360,640),np.uint8)
    depth = np.full((360,640),10,np.float32)
    args = dict(frame_id='test',timestamp_s=1.,fov_deg=90,label_table=p.TABLE,stride=3,max_depth_m=40,min_confidence=.5)
    full = p.project_predicted_frame(depth,classes,confidence,np.eye(4),**args)
    sparse = p.project_predicted_frame(depth,classes,p.restore_confidence(confidence[::3,::3].copy()),np.eye(4),**args)
    assert full == sparse
    assert all(point[3] >= .5 for point in full.points)


def test_grid_codec_preserves_counts_weights_and_tie_insertion_order():
    grid = p.new_grid('test','learned_prediction')
    grid.update(Observation(1.,'test','learned_prediction',((.1,.1,'sidewalk',.6),(.1,.1,'road',.6),(.1,.1,'road',.5))))
    grid.update(Observation(2.,'test','learned_prediction',((.1,.1,'road',.7),)))
    meta = dict(frame_id='test',source='learned_prediction',timestamp_s=2.)
    recovered = p.unpack_grid(p.pack_grid(grid),meta)
    assert recovered.snapshot() == grid.snapshot()
    assert list(recovered.cells[(0,0)]) == ['sidewalk','road']
    assert recovered.cells[(0,0)]['road'] == [2,1.2999999999999998]


def test_missing_observations_stay_unknown_in_union_comparison():
    oracle = p.new_grid('test','oracle_sensor'); learned = p.new_grid('test','learned_prediction')
    oracle.update(Observation(1.,'test','oracle_sensor',((.1,.1,'road',1.),)))
    learned.update(Observation(1.,'test','learned_prediction',((1.,1.,'sidewalk',.8),)))
    value = p.comparison(oracle,learned)
    assert value['common_cells'] == 0 and value['union_cells'] == 2
    assert value['confusion_unknown_then_six_classes'][1][0] == 1
    assert value['confusion_unknown_then_six_classes'][0][2] == 1
    assert value['classes']['sidewalk']['iou'] is None
    assert value['classes']['sidewalk']['false_positive_cells'] == 1


def test_bad_confidence_cannot_be_replaced_or_quantized():
    with pytest.raises(ValueError): p.restore_confidence(np.ones((120,214),np.float16))
    with pytest.raises(ValueError): p.restore_confidence(np.full((120,214),np.nan,np.float32))


def test_planar_pixel_projection_known_axis():
    depth = np.full((360,640),10,np.float32); classes = np.zeros((360,640),np.uint8)
    confidence = np.zeros((360,640),np.float32); classes[180,318] = 1; confidence[180,318] = .75
    observation = p.project_predicted_frame(depth,classes,confidence,np.eye(4),frame_id='test',timestamp_s=1.,
        fov_deg=90,label_table=p.TABLE,stride=3,max_depth_m=40,min_confidence=.5)
    assert len(observation.points) == 1
    assert observation.points[0] == pytest.approx((10.,-.0625,'road',.75))
