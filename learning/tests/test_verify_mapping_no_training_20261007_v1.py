"""HH_261007 - Synthetic/AST mapping-coordinate checks are not measured mapping performance."""

import ast
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scripts.e2e import verify_mapping_no_training_20261007_v1 as v
from portable_e2e.mapping import Observation, SemanticEvidenceMap
from portable_e2e.mapping_projection import project_predicted_frame


def numeric():
    return v.numerical()[1]


def fixture():
    depth = np.full((5, 8), 10, np.float32)
    classes = np.ones((5, 8), np.uint8)
    confidence = np.ones((5, 8), np.float32)
    pose = np.array([[0, -1, 0, -5], [1, 0, 0, -3], [0, 0, 1, 2], [0, 0, 0, 1]], np.float64)
    return depth, classes, confidence, pose


def observe(depth, classes, confidence, pose, stamp=1):
    return project_predicted_frame(depth, classes, confidence, pose, frame_id='synthetic_only',
                                   timestamp_s=stamp, fov_deg=90, label_table=v.TABLE,
                                   stride=3, max_depth_m=40, min_confidence=.5)


def test_rotated_translated_coordinate_roundtrip():
    values = fixture()
    result = v.coordinate_metrics(np, *values, observe(*values), v.TABLE)
    assert result['points'] == 6
    assert result['xy_max_error_m'] <= 1e-8
    assert result['roundtrip_pixel_max_error'] <= 1e-6
    assert result['cell_center_max_distance_m'] <= np.sqrt(2) * .1 + 1e-8


@pytest.mark.parametrize('mutation', ['x', 'point_order', 'count'])
def test_coordinate_fault_is_not_relabelled_success(mutation):
    values = fixture()
    observed = observe(*values)
    points = list(observed.points)
    if mutation == 'x':
        x, y, label, score = points[0]
        points[0] = (x + .01, y, label, score)
    elif mutation == 'point_order':
        points.reverse()
    else:
        points.pop()
    bad = Observation(observed.timestamp_s, observed.frame_id, observed.source, tuple(points))
    with pytest.raises(ValueError):
        v.coordinate_metrics(np, *values, bad, v.TABLE)


def test_empty_unobserved_remains_unknown():
    depth, classes, certainty, pose = fixture()
    certainty.fill(.49)
    observed = observe(depth, classes, certainty, pose)
    result = v.coordinate_metrics(np, depth, classes, certainty, pose, observed, v.TABLE)
    assert result == dict(points=0, xy_max_error_m=None, roundtrip_pixel_max_error=None, cell_center_max_distance_m=None)
    assert v.coverage(observed, set()) == dict(observed_cells=0, newly_observed_cells=0,
        reobserved_cells=0, multi_label_cells_in_this_frame=0)


def test_exact_confidence_gate_and_depth_endpoints_are_preserved():
    depth, classes, certainty, pose = fixture()
    depth[0, 0], depth[0, 3] = .1, 40
    certainty[0, 6], certainty[3, 0] = .5, .49
    classes[3, 3] = 0
    observed = observe(depth, classes, certainty, pose)
    result = v.coordinate_metrics(np, depth, classes, certainty, pose, observed, v.TABLE)
    assert result['points'] == 2


def test_fusion_counts_frames_not_dense_points_and_preserves_negative_floor():
    n = numeric()
    grid = SemanticEvidenceMap('synthetic_only', 'learned_prediction', .2, 200000)
    independent = SemanticEvidenceMap('synthetic_only', 'learned_prediction', .2, 200000)
    first = Observation(1, 'synthetic_only', 'learned_prediction',
                        ((-.001, -.001, 'road', .3), (-.001, -.001, 'road', .8), (-.001, -.001, 'sidewalk', .6)))
    second = Observation(2, 'synthetic_only', 'learned_prediction', ((-.001, -.001, 'road', .7),))
    for observation in (first, second):
        grid.update(observation)
        n.independent_update(independent, observation)
        assert grid.snapshot() == independent.snapshot()
    assert grid.cells[(-1, -1)]['road'] == [2, 1.5]
    assert grid.cells[(-1, -1)]['sidewalk'] == [1, .6]
    assert v.coverage(first, {(-1, -1)}) == dict(observed_cells=1, newly_observed_cells=0,
        reobserved_cells=1, multi_label_cells_in_this_frame=1)


@pytest.mark.parametrize('fault', ['timestamp', 'frame', 'source'])
def test_fusion_bad_context_retains_failure(fault):
    n = numeric()
    grid = SemanticEvidenceMap('synthetic_only', 'learned_prediction', .2, 200000)
    good = Observation(1, 'synthetic_only', 'learned_prediction', ((0, 0, 'road', .6),))
    n.independent_update(grid, good)
    old = grid.snapshot()
    bad = Observation(1 if fault == 'timestamp' else 2, 'wrong' if fault == 'frame' else good.frame_id,
                      'oracle_sensor' if fault == 'source' else good.source, good.points)
    with pytest.raises(ValueError):
        n.independent_update(grid, bad)
    assert grid.snapshot() == old


def test_raster_extent_includes_every_cell_and_all240_poses():
    n = numeric()
    records = [dict(ego_x=i, ego_y=0) for i in range(240)]
    grid = SemanticEvidenceMap('synthetic_only', 'learned_prediction', .2, 200000)
    grid.cells[(-1000, -1000)] = {'road': [1, 1.]}
    grid.cells[(3000, 3000)] = {'road': [1, 1.]}
    extent = n.independent_overview(records, {'learned': grid})
    assert extent['bounds_xy_m'] == [-201., -201., 601.2, 601.2]
    assert extent['pose_count'] == 240
    assert extent['all_stored_cell_extents_included'] is True
    with pytest.raises(ValueError):
        n.independent_overview(records[:-1], {'learned': grid})


def test_exact_serialization_retains_label_insertion_order_for_ties():
    n = numeric()
    grid = SemanticEvidenceMap('synthetic_only', 'learned_prediction', .2, 200000)
    grid.cells[(0, 0)] = {'sidewalk': [1, .5], 'road': [1, .5]}
    arrays = n.pack_grid(grid)
    assert arrays['label_order'][0].tolist() == [1, 0, -1, -1, -1, -1]
    assert max(grid.cells[(0, 0)], key=lambda label: grid.cells[(0, 0)][label][1]) == 'sidewalk'


def test_only_pinned_functions_execute_not_original_module_top_level(tmp_path):
    path = tmp_path / 'pure.py'
    path.write_text('raise AssertionError("module top must not execute")\ndef pure(x):\n    return x + 1\n')
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    result = v.extracted_functions(path, sha, ('pure',), {})
    assert result['pure'](2) == 3
    with pytest.raises(ValueError):
        v.extracted_functions(path, '0' * 64, ('pure',), {})
    with pytest.raises(ValueError):
        v.extracted_functions(path, sha, ('missing',), {})


def test_private_functions_keep_genuine_frozen_ast():
    n = numeric()
    original = ast.parse(v.MATH_SOURCE.read_text())
    originals = {node.name: node for node in original.body if isinstance(node, ast.FunctionDef)}
    for name in ('metric_depth', 'restore_confidence', 'pack_grid', 'comparison'):
        assert n.__dict__[name].__code__.co_filename == str(v.MATH_SOURCE)
        assert originals[name].name == name
    assert n.independent_update.__code__.co_filename == str(v.VOTE_SOURCE)


def test_canonical_source_seal_rejects_symlink(tmp_path):
    path = tmp_path / 'source'
    path.write_bytes(b'unchanged')
    link = tmp_path / 'alias'
    link.symlink_to(path)
    assert v.digest(path) == hashlib.sha256(b'unchanged').hexdigest()
    with pytest.raises(ValueError):
        v.digest(link)


@pytest.mark.parametrize('sha', [None, '', 'g' * 64, 'a' * 63])
def test_actual_entry_denies_without_external_root_sha_before_any_reads(monkeypatch, sha):
    monkeypatch.setattr(v, 'digest', lambda path: pytest.fail('must deny before read/hash'))
    monkeypatch.setattr(v, 'numerical', lambda: pytest.fail('no numerical imports'))
    with pytest.raises(ValueError):
        v.execute(sha)


def test_valid_looking_sha_cannot_bypass_fixed_root_review(monkeypatch):
    monkeypatch.setattr(v, 'digest', lambda path: 'b' * 64)
    monkeypatch.setattr(v, 'numerical', lambda: pytest.fail('must deny before numerical import'))
    with pytest.raises(ValueError):
        v.execute('a' * 64)


def test_schema_full_denominator_and_boundaries_are_prospective():
    policy = v.policy()
    assert (policy['frames'], policy['learned_observations'], policy['oracle_observations']) == (480, 960, 480)
    assert policy['oracle_depth'] and policy['oracle_pose']
    assert not policy['ctrack_train_eligible'] and policy['left_original_owner_status'] == 'FAIL'
    for flag in ('latest_native_fusion_v3_model_verified', 'independent_validation', 'hd_map_approved', 'model_promotion', 'gpu_used', 'simulator_started'):
        assert policy[flag] is False
    assert policy['inference_count'] == policy['training_steps'] == policy['native_frames_dropped'] == 0


def test_source_has_no_nn_remote_simulator_or_pickle_execution():
    tree = ast.parse(Path(v.__file__).read_text())
    modules = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.extend(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            modules.append((node.module or '').split('.')[0])
    assert not set(modules) & {'torch', 'carla', 'subprocess', 'pickle', 'socket', 'paramiko'}
    assert {'numpy', 'portable_e2e'} <= set(modules)
    source = Path(v.__file__).read_text()
    assert 'allow_pickle=False' in source
    assert "status='UNEXECUTED'" in source and "status='FAIL'" in source
    assert "'source_and_inputs_unchanged'] = input_manifest()[0] == hashes" in source
    assert 'png_pixels_replayed=False' in source


def test_metadata_and_create_only_witness_bounds(tmp_path, monkeypatch):
    directory = tmp_path / 'fresh'
    directory.mkdir()
    monkeypatch.setattr(v, 'OUTPUT', directory)
    monkeypatch.setattr(v.shutil, 'disk_usage', lambda path: SimpleNamespace(free=v.FLOOR + v.LIMIT))
    v.write_new('result.json', {'status': 'SYNTHETIC_ONLY'})
    assert json.loads((directory / 'result.json').read_text())['status'] == 'SYNTHETIC_ONLY'
    with pytest.raises(FileExistsError):
        v.write_new('result.json', {})
    with pytest.raises(ValueError):
        v.write_new('../escape.json', {})
    monkeypatch.setattr(v, 'LIMIT', 1)
    with pytest.raises(ValueError):
        v.write_new('oversize.json', {})
    assert not (directory / 'oversize.json').exists()
