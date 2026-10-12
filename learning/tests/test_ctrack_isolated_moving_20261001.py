"""HH_261001 - Pin moving roles/routes and independently replay raw sensor identity."""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from scripts.e2e import collect_ctrack_isolated_moving_20261001 as capture
from scripts.e2e import run_owned_ctrack_moving_20261001 as owner
from scripts.e2e.audit_ctrack_isolated_moving_20261001 import verify_sensor_metadata


def test_external_diagnostic_role_does_not_promote_training_or_dev():
    assert capture.INTENDED_ROLE == "external_map_diagnostic_evaluation"
    contract = capture.qa_summary_contract()
    assert contract["dataset_admitted"] is False
    assert contract["train_eligible"] is False
    assert contract["independent_dev_approved"] is False
    assert capture.FRAMES == 240 and capture.FIXED_DELTA_SECONDS == .1


@pytest.mark.parametrize("scenario", ("left", "straight"))
def test_route_bytes_and_scenario_remain_exact(scenario):
    path, expected = capture.ROUTES[scenario]
    route, digest = capture._route_binding(path, scenario)
    assert route["scenario"] == scenario and digest == expected
    assert route["coordinate_reference"] == "base_link"
    assert owner.source_files(scenario, owner.stage.DEFAULT_OUTPUT).count(path) == 1


def test_each_raw_sensor_digest_timestamp_and_pose_are_checked():
    arrays = [np.full((360,640,4), tag, dtype=np.uint8) for tag in (7,15,24)]
    pose = np.eye(4)
    kinds = ("rgb", "depth", "semantic_segmentation")
    record = {"frame": 30, "timestamp_s": 3.0,
              "sensors": {name: {"actor_id": i+1, "frame": 30, "timestamp_s": 3.0,
                                  "width": 640, "height":360,
                                  "world_from_camera":pose.tolist(),
                                  "raw_bgra_sha256":hashlib.sha256(array.tobytes()).hexdigest()}
                          for i,(name,array) in enumerate(zip(kinds,arrays))}}
    verify_sensor_metadata(record, *arrays, pose)
    record["sensors"]["depth"]["timestamp_s"] = 3.1
    with pytest.raises(ValueError, match="depth"):
        verify_sensor_metadata(record, *arrays, pose)
    record["sensors"]["depth"]["timestamp_s"] = 3.0
    arrays[2][0,0,2] = 1
    with pytest.raises(ValueError, match="semantic_segmentation"):
        verify_sensor_metadata(record, *arrays, pose)


def test_source_archive_covers_actual_agent_and_mapping_dependencies():
    sources = owner.source_files("left", owner.stage.DEFAULT_OUTPUT)
    names = {path.name for path in sources}
    assert {"basic_agent.py", "controller.py", "local_planner.py", "misc.py",
            "mapping.py", "mapping_projection.py", "site_split_guard_v2.py",
            "collect_ctrack_isolated_moving_20261001.py"} <= names
    assert all(path.is_file() and not path.is_symlink() for path in sources)
