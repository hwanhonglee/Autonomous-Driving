"""HH_260906 - Verify bounded target-envelope diagnosis without reading held-out sample data."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

torch = pytest.importorskip("torch")

from portable_e2e.contract import ContractError, DEFAULT_CONTRACT_PATH, contract_fingerprint
from portable_e2e.dataset import FEATURE_NAMES
from portable_e2e.model import ModelConfig, PHYSICAL_MODEL_ID, PerspectiveTrajectoryModel
from scripts.e2e import audit_portable_target_feasibility as audit


def _sample(*, speed=4.0, token="sample", valid_points=64):
    return {"sample_id": token, "episode_id": "episode", "sequence_index": 0,
        "anchor_timestamp_ns": 0, "ego": {"linear_velocity_base_mps": [speed, 0.0, 0.0]},
        "navigation": {"goal_base_m": [100.0, 0.0], "route_anchor_arc_m": 0.0},
        "labels": {"planning": {"available": True, "dt_s": 0.1,
            "positions_base_xy_m": [[speed * (index + 1) * 0.1, 0.0] if index < valid_points else [None, None] for index in range(64)],
            "speed_mps": [speed if index < valid_points else None for index in range(64)],
            "yaw_rad": [0.0 if index < valid_points else None for index in range(64)],
            "valid": [index < valid_points for index in range(64)],
            "target_timestamp_ns": [(index + 1) * 100_000_000 if index < valid_points else None for index in range(64)],
            "invalid_reason": [None if index < valid_points else "episode_end" for index in range(64)]}}}


def _audit(sample, **kwargs):
    row = audit.audit_sample(sample, route_length_m=100.0, episode_start_ns=0, episode_end_ns=6_400_000_000, **kwargs)
    row["episode_id"] = sample["episode_id"]
    return row


def test_constant_straight_targets_pass_all_assessed_scalar_envelopes():
    row = _audit(_sample())
    summary = audit.summarize_samples([row], 64)
    assert summary["valid_step_count"] == 64
    assert summary["horizon_eligible_anchor_count"] == 1
    assert all(value["violating_step_count"] == 0 for value in summary["metrics"].values())
    assert summary["speed_xy_consistency_max_absolute_error_mps"] < 1.0e-12


@pytest.mark.parametrize("initial_speed", [0.0, 4.0, audit.PHYSICAL_MAXIMUM_SPEED_MPS])
def test_actual_float64_decoder_outputs_satisfy_the_same_audit_bounds(initial_speed):
    # HH_260906 - Exercise the unchanged decoder directly without creating image models or using a GPU.
    config = ModelConfig(model_id=PHYSICAL_MODEL_ID, maximum_step_m=1.0)
    raw = torch.randn(1, config.candidate_count, 64, 2,
                      generator=torch.Generator().manual_seed(260906), dtype=torch.float64) * 2.0
    ego = torch.zeros(1, config.ego_history_frames, config.ego_features, dtype=torch.float64)
    ego[0, -1, FEATURE_NAMES.index("velocity_x_mps")] = initial_speed
    route = torch.tensor([[[0.0, 0.0], [10.0, 0.0], [20.0, 3.0], [30.0, 10.0]]], dtype=torch.float64)
    xy, speeds = PerspectiveTrajectoryModel._decode_physical_v1(
        SimpleNamespace(config=config), raw, ego, route, torch.ones(1, 4, dtype=torch.bool))
    for candidate in range(config.candidate_count):
        sample = _sample(speed=initial_speed)
        planning = sample["labels"]["planning"]
        planning["positions_base_xy_m"] = xy[0, candidate].tolist()
        planning["speed_mps"] = speeds[0, candidate].tolist()
        previous, heading, yaws = (0.0, 0.0), 0.0, []
        for point in planning["positions_base_xy_m"]:
            if math.dist(previous, point) > 1.0e-12:
                heading = math.atan2(point[1] - previous[1], point[0] - previous[0])
            yaws.append(heading)
            previous = point
        planning["yaw_rad"] = yaws
        summary = audit.summarize_samples([_audit(sample)], 64)
        assert all(value["violating_step_count"] == 0 for value in summary["metrics"].values())


def test_initial_acceleration_uses_decoder_longitudinal_speed_not_planar_norm():
    sample = _sample(speed=5.0)
    sample["ego"]["linear_velocity_base_mps"] = [3.0, 4.0, 0.0]
    row = _audit(sample)
    assert row["decoder_initial_speed_mps"] == 3.0
    assert row["measured_planar_speed_mps"] == 5.0
    assert row["steps"][0]["metrics"]["speed_acceleration"] == (20.0, True)
    assert row["steps"][0]["measured_planar_initial_acceleration_mps2"] == 0.0


def test_current_speed_clamp_matches_actual_decoder():
    sample = _sample(speed=audit.PHYSICAL_MAXIMUM_SPEED_MPS)
    sample["ego"]["linear_velocity_base_mps"][0] = 20.0
    row = _audit(sample)
    assert row["raw_longitudinal_speed_mps"] == 20.0
    assert row["decoder_initial_speed_mps"] == audit.PHYSICAL_MAXIMUM_SPEED_MPS
    assert row["steps"][0]["metrics"]["speed_acceleration"][0] == 0.0


def test_consecutive_deceleration_is_independently_checked_from_speed_and_xy():
    sample = _sample()
    planning = sample["labels"]["planning"]
    planning["speed_mps"] = [4.0] + [0.0] * 63
    planning["positions_base_xy_m"] = [[0.4, 0.0]] * 64
    summary = audit.summarize_samples([_audit(sample)], 64)
    for name in ("speed_deceleration", "xy_deceleration"):
        assert summary["metrics"][name]["violating_anchor_count"] == 1
        assert summary["metrics"][name]["violating_step_count"] == 1
        assert summary["metrics"][name]["minimum"]["value"] == pytest.approx(-40.0)


def test_speed_label_conflict_does_not_automatically_flag_xy_conflict():
    sample = _sample()
    sample["labels"]["planning"]["speed_mps"][1] = 0.0
    summary = audit.summarize_samples([_audit(sample)], 64)
    assert summary["metrics"]["speed_deceleration"]["violating_anchor_count"] == 1
    assert summary["metrics"]["xy_deceleration"]["violating_anchor_count"] == 0


def test_curvature_lateral_and_yaw_proxy_diagnostics_are_distinct():
    sample = _sample()
    sample["labels"]["planning"]["positions_base_xy_m"] = [[0.0, (index + 1) * 0.4] for index in range(64)]
    sample["labels"]["planning"]["yaw_rad"] = [math.pi / 2] * 64
    summary = audit.summarize_samples([_audit(sample)], 64)
    for name in ("xy_curvature", "xy_lateral_acceleration", "yaw_label_heading_envelope"):
        assert summary["metrics"][name]["violating_anchor_count"] == 1
    sample = _sample()
    sample["labels"]["planning"]["yaw_rad"] = [1.0] * 64
    summary = audit.summarize_samples([_audit(sample)], 64)
    assert summary["metrics"]["yaw_label_heading_envelope"]["violating_anchor_count"] == 1
    assert summary["metrics"]["xy_curvature"]["violating_anchor_count"] == 0


def test_stationary_pose_noise_is_unassessed_for_heading_not_counted_as_passing():
    sample = _sample(speed=0.0)
    sample["labels"]["planning"]["positions_base_xy_m"] = [[(-1) ** index * 1.0e-6, 0.0] for index in range(64)]
    summary = audit.summarize_samples([_audit(sample)], 64)
    assert summary["metrics"]["xy_curvature"]["assessed_step_count"] == 0
    assert summary["metrics"]["yaw_label_heading_envelope"]["assessed_step_count"] == 0
    assert summary["metrics"]["xy_curvature"]["maximum"] is None


def test_partial_prefix_masks_keep_horizon_denominators_explicit():
    row = _audit(_sample(valid_points=10))
    assert audit.summarize_samples([row], 10)["horizon_eligible_anchor_count"] == 1
    later = audit.summarize_samples([row], 30)
    assert later["horizon_eligible_anchor_count"] == 0
    assert later["valid_step_count"] == 10 and later["invalid_step_count"] == 20


def test_overlapping_windows_are_not_reported_as_independent_braking_events():
    sample = _sample()
    sample["labels"]["planning"]["speed_mps"][0] = 0.0
    first = _audit(sample, tail_start_ns=100_000_000)
    second = copy.deepcopy(first)
    second["sample_id"] = "second-overlapping-anchor"
    result = audit.summarize_samples([first, second], 64)["metrics"]["speed_deceleration"]
    assert result["violating_anchor_count"] == result["violating_step_count"] == 2
    assert result["unique_episode_relative_100ms_violation_tick_count"] == 1
    assert result["violating_steps_by_capture_phase"] == {"stationary_tail": 2}


def test_goal_region_and_capture_phase_are_separate_from_physical_envelopes():
    sample = _sample()
    sample["navigation"] = {"goal_base_m": [1.0, 0.0], "route_anchor_arc_m": 99.0}
    row = _audit(sample, tail_start_ns=500_000_000)
    assert row["anchor_region"] == "near_goal"
    assert row["steps"][0]["future_region"] == "near_goal"
    assert row["steps"][3]["phase"] == "pre_tail"
    assert row["steps"][4]["phase"] == "stationary_tail"


@pytest.mark.parametrize("mutate,match", [
    (lambda s: s["labels"]["planning"].update(dt_s=0.2), "dt_s"),
    (lambda s: s["labels"]["planning"]["target_timestamp_ns"].__setitem__(0, 100_000_001), "future grid"),
    (lambda s: s["labels"]["planning"]["speed_mps"].__setitem__(0, float("nan")), "finite"),
    (lambda s: s["labels"]["planning"]["speed_mps"].__setitem__(0, -1.0), "negative"),
    (lambda s: s["labels"]["planning"]["valid"].__setitem__(0, 1), "Boolean"),
    (lambda s: s["labels"]["planning"]["speed_mps"].pop(), "64 points"),
    (lambda s: s["ego"]["linear_velocity_base_mps"].__setitem__(0, True), "finite"),
])
def test_malformed_timing_values_or_masks_fail_closed(mutate, match):
    sample = _sample()
    mutate(sample)
    with pytest.raises(ContractError, match=match):
        _audit(sample)


def test_nonprefix_mask_and_zero_valid_points_are_rejected():
    sample = _sample(valid_points=10)
    planning = sample["labels"]["planning"]
    planning["valid"][11] = True
    with pytest.raises(ContractError, match="contiguous prefix"):
        _audit(sample)
    with pytest.raises(ContractError, match="at least one"):
        _audit(_sample(valid_points=0))


def _write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) + "\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def dataset(tmp_path):
    root = tmp_path / "dataset"
    refs = []
    for split in ("train", "val", "test"):
        episode_root = root / split
        sample = _sample(token=split + "-sample")
        sample["episode_id"] = split
        episode = {"episode_id": split, "split": split, "sample_jsonl": "samples.jsonl", "sample_count": 1,
            "start_timestamp_ns": 0, "end_timestamp_ns": 6_400_000_000,
            "route_geometry_file": "route.json", "source_provenance": {"collection_config_file": "collection.json"}}
        if split != "test":
            episode["sample_jsonl_sha256"] = _write(episode_root / "samples.jsonl", sample)
            episode["route_geometry_sha256"] = _write(episode_root / "route.json", {"polyline_m": [[0.0, 0.0], [100.0, 0.0]]})
            episode["source_provenance"]["collection_config_sha256"] = _write(episode_root / "collection.json", {})
        # HH_260906 - Test sample and route files deliberately do not exist; an accidental test read must fail.
        ep_sha = _write(episode_root / "episode.json", episode)
        refs.append({"episode_id": split, "manifest": f"{split}/episode.json", "sha256": ep_sha})
    manifest = {"dataset_id": "fixture", "contract_id": "common_10hz_v1", "episodes": refs,
        "contract_sha256": contract_fingerprint(json.loads(DEFAULT_CONTRACT_PATH.read_text()))}
    return root, _write(root / "dataset.json", manifest)


def test_dataset_opens_only_train_val_targets_and_emits_relative_hashed_paths(dataset):
    root, expected_sha = dataset
    result = audit.audit_dataset(root, expected_sha)
    assert result["status"] == "TARGET_ENVELOPE_AUDIT_COMPLETE"
    assert result["splits"]["train"]["sample_count"] == result["splits"]["val"]["sample_count"] == 1
    assert result["scope"]["test_sample_data_opened"] is False
    assert result["scope"]["test_episode_metadata_read_to_identify_split"] is True
    assert result["scope"]["labels_modified"] is False
    assert str(root) not in json.dumps(result)
    assert all(not Path(item["path"]).is_absolute() for item in result["input_manifest"])
    assert result["physical_decoder_limits"]["acceleration_and_deceleration_mps2"] == audit.PHYSICAL_MAXIMUM_ACCELERATION_MPS2


def test_manifest_or_sample_hash_mismatch_is_rejected(dataset):
    root, expected_sha = dataset
    with pytest.raises(ContractError, match="manifest SHA"):
        audit.audit_dataset(root, "0" * 64)
    path = root / "train/samples.jsonl"
    path.write_text(path.read_text() + " ")
    with pytest.raises(ContractError, match="SHA-?256"):
        audit.audit_dataset(root, expected_sha)


def test_cli_creates_new_json_without_silent_overwrite(dataset, tmp_path):
    root, expected_sha = dataset
    output = tmp_path / "report.json"
    args = [str(root), "--dataset-manifest-sha256", expected_sha, "--output-json", str(output)]
    assert audit.main(args) == 0
    original = output.read_bytes()
    assert audit.main(args) == 2
    assert output.read_bytes() == original
    with pytest.raises(SystemExit):
        audit.main([*args, "--split", "test"])


def test_cli_refuses_to_create_output_inside_read_only_dataset(dataset):
    root, expected_sha = dataset
    output = root / "audit.json"
    assert audit.main([str(root), "--dataset-manifest-sha256", expected_sha,
                       "--output-json", str(output)]) == 2
    assert not output.exists()
