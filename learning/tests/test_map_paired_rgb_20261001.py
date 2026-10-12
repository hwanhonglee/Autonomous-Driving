"""HH_261001 - Fixed TRAIN scope, four-sensor synchronization and owner no-launch default."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from scripts.e2e import collect_map_paired_rgb_20261001 as capture


def test_exact_old_train_routes_and_no_holdout_or_ctrack():
    assert len(capture.ROUTES) == 2
    assert set(capture.ROUTES.values()) == {("Town01","right"),("Town02_Opt","straight")}
    assert "cad81f4cdfbc9dbcad8c3cc53092e6fd80f6d84dc4766b6d73758fa88dec6810" not in capture.ROUTES
    assert "2299e5bf2bc86789650da72336fdcfe7c11c585df0b9e0a0fe29a44730bae79b" not in capture.ROUTES


def test_single_sensor_attribute_difference_only():
    assert capture.SENSOR_SPECS == (("rgb_false","rgb","false"),("rgb_true","rgb","true"),
        ("depth","depth","false"),("semantic","semantic_segmentation","false"))
    assert capture.ATTRIBUTES == {"image_size_x":"640","image_size_y":"360","fov":"90",
        "sensor_tick":"0","lens_k":"0","lens_kcube":"0"}


def test_pair_validation_rejects_frame_timestamp_pose_or_shape_mismatch():
    images = [SimpleNamespace(frame=100,timestamp=10.0) for _ in range(4)]
    matrices = [np.eye(4) for _ in range(4)]
    arrays = [np.zeros((360,640,4),dtype=np.uint8) for _ in range(4)]
    capture.validate_pair_metadata(images,matrices,arrays)
    images[1].frame = 101
    with pytest.raises(ValueError,match="same-frame"):
        capture.validate_pair_metadata(images,matrices,arrays)
    images[1].frame = 100;images[2].timestamp = 10.1
    with pytest.raises(ValueError,match="timestamps"):
        capture.validate_pair_metadata(images,matrices,arrays)
    images[2].timestamp = 10;matrices[3][0,3] = .1
    with pytest.raises(ValueError,match="matrices"):
        capture.validate_pair_metadata(images,matrices,arrays)
    matrices[3][0,3] = 0;arrays[0] = arrays[0][:1]
    with pytest.raises(ValueError,match="native"):
        capture.validate_pair_metadata(images,matrices,arrays)


def test_unregistered_route_is_rejected_before_capture(tmp_path):
    path = tmp_path/"route.json"
    path.write_text('{"town":"C_track_1_0_7","scenario":"left"}')
    with pytest.raises(ValueError,match="exact Town01"):
        capture.load_bound_route(SimpleNamespace(route_file=path))


def test_owner_defaults_to_preflight_and_archives_new_worker():
    owner = capture.ROOT/"tools/e2e/run_owned_map_paired_rgb_20261001.sh"
    text = owner.read_text()
    assert "execute=0" in text and "--execute) execute=1" in text
    assert text.index("if (( ! execute ))") < text.index('mkdir -- "${output_root}"')
    assert "worker_module=scripts.e2e.collect_map_paired_rgb_20261001" in text
    assert "scripts/e2e/run_owned_map_paired_rgb_20261001.sh" in text
    assert "scripts/e2e/audit_map_paired_rgb_20261001.py" in text


def test_every_owner_python_heredoc_parses_and_null_split_is_real():
    import ast
    import re
    text = (capture.ROOT/"tools/e2e/run_owned_map_paired_rgb_20261001.sh").read_text()
    blocks = re.findall(r"<<'PY'\n(.*?)\nPY",text,re.DOTALL)
    assert len(blocks) >= 8
    for block in blocks:
        ast.parse(block)
    assert ".split(bytes([0]))" in text
