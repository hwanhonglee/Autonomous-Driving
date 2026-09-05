# HH_260906 - Verify exhaustive candidate accounting and atomic runtime audit evidence.

import json
import math
from dataclasses import replace
from fractions import Fraction
from types import SimpleNamespace

import pytest
import torch

from portable_e2e import ContractError
from portable_e2e import audit_runtime as audit_runtime_module
from portable_e2e.audit_runtime import _AuditAccumulator
from portable_e2e.audit_runtime import _atomic_new_json
from portable_e2e.audit_runtime import audit_checkpoint
from portable_e2e.audit_runtime import audit_prediction
from portable_e2e.audit_runtime import parse_args
from portable_e2e.model import ModelConfig
from portable_e2e.runtime_contract import RuntimeGateConfig
from portable_e2e.runtime_contract import validate_and_select_trajectory


def _valid_predictions():
    candidate_xy = []
    candidate_speed = []
    for candidate_index in range(6):
        lateral_step = 0.0005 * candidate_index
        candidate_xy.append(
            [
                (0.1 * (point_index + 1), lateral_step * point_index)
                for point_index in range(64)
            ]
        )
        candidate_speed.append([1.0] * 64)
    return candidate_xy, candidate_speed, [0.0, 0.1, 0.2, 1.0, -1.0, -2.0]


def test_audit_prediction_checks_every_candidate_and_learned_selection():
    xy, speed, logits = _valid_predictions()
    xy[3][0] = (2.0, 0.0)
    speed[3][4] = 9.0

    audit = audit_prediction(xy, speed, logits)

    assert len(audit["candidates"]) == 6
    assert [item["candidate_index"] for item in audit["candidates"]] == list(range(6))
    assert audit["candidates"][0]["geometry_pass"] is True
    assert audit["candidates"][3]["geometry_pass"] is False
    assert {"first_distance", "step", "speed"} <= set(
        audit["candidates"][3]["failure_codes"]
    )
    assert audit["all_candidates_geometry_pass"] is False
    assert audit["any_candidate_geometry_pass"] is True
    assert audit["selected_candidate_index"] == 3
    assert audit["selected_geometry_pass"] is False
    assert set(audit["selected_failure_codes"]) == set(
        audit["candidates"][3]["failure_codes"]
    )


@pytest.mark.parametrize("malformed_speed", ([], [1.0] * 63))
def test_audit_rejects_empty_and_ragged_selected_speed_rows(malformed_speed):
    # HH_260906 - Keep malformed live predictions as deterministic audit failures.
    xy, speed, logits = _valid_predictions()
    speed[3] = malformed_speed

    with pytest.raises(ContractError, match="must contain 64 future points"):
        validate_and_select_trajectory(xy, speed, logits)

    audit = audit_prediction(xy, speed, logits)

    assert audit["selected_candidate_index"] == 3
    assert audit["selected_geometry_pass"] is False
    assert audit["selected_failure_codes"] == ["nonfinite_or_shape"]
    assert audit["candidates"][3] == {
        "candidate_index": 3,
        "geometry_pass": False,
        "failure_codes": ["nonfinite_or_shape"],
    }


@pytest.mark.parametrize("malformed_xy", ({}, {3: {}}))
def test_audit_rejects_mapping_shaped_xy_without_key_error(malformed_xy):
    # HH_260906 - Convert mapping-shaped predictions into deterministic audit failures.
    xy, speed, logits = _valid_predictions()
    if malformed_xy:
        xy[3][2] = malformed_xy[3]
        candidate_xy = xy
    else:
        candidate_xy = malformed_xy

    with pytest.raises(ContractError):
        validate_and_select_trajectory(candidate_xy, speed, logits)

    audit = audit_prediction(candidate_xy, speed, logits)

    assert audit["selected_geometry_pass"] is False
    assert "nonfinite_or_shape" in audit["selected_failure_codes"]


def test_audit_rejects_extreme_finite_speed_without_overflow():
    # HH_260906 - Keep exhaustive audit fail-closed for finite values beyond float32.
    xy, speed, logits = _valid_predictions()
    speed[3] = [1.0e308] * 64

    with pytest.raises(ContractError, match="speed gate"):
        validate_and_select_trajectory(xy, speed, logits)

    audit = audit_prediction(xy, speed, logits)

    assert audit["selected_candidate_index"] == 3
    assert audit["selected_geometry_pass"] is False
    assert "speed" in audit["selected_failure_codes"]


def test_audit_classifies_boolean_current_speed_as_invalid_input():
    # HH_260906 - Preserve the live numeric type contract in exhaustive audit.
    xy, speed, logits = _valid_predictions()

    with pytest.raises(ContractError, match="must be a number"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            current_speed_mps=False,
        )

    audit = audit_prediction(xy, speed, logits, current_speed_mps=False)

    assert audit["selected_geometry_pass"] is False
    assert audit["selected_failure_codes"] == ["nonfinite_or_shape"]


@pytest.mark.parametrize("numeric_factory", (int, Fraction))
@pytest.mark.parametrize("field", ("current_speed", "xy", "speed", "logit"))
def test_audit_rejects_unrepresentable_python_numerics_without_overflow(
    numeric_factory,
    field,
):
    # HH_260906 - Keep live and audit paths fail-closed for Python values beyond float range.
    xy, speed, logits = _valid_predictions()
    oversized = numeric_factory(10**10000)
    current_speed = 1.0
    if field == "current_speed":
        current_speed = oversized
    elif field == "xy":
        xy[3][2] = (oversized, 0.0)
    elif field == "speed":
        speed[3][2] = oversized
    else:
        logits[3] = oversized

    with pytest.raises(ContractError, match="must be (?:a number|finite)"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            current_speed_mps=current_speed,
        )

    audit = audit_prediction(
        xy,
        speed,
        logits,
        current_speed_mps=current_speed,
    )

    assert audit["selected_geometry_pass"] is False
    assert audit["selected_failure_codes"] == ["nonfinite_or_shape"]


def test_accumulator_reports_selector_collapse_and_multiple_gate_failures():
    xy, speed, logits = _valid_predictions()
    xy[1][0] = (2.0, 0.0)
    xy[1][2] = (2.1, 2.0)
    logits[1] = 3.0
    accumulator = _AuditAccumulator(RuntimeGateConfig())

    accumulator.add(audit_prediction(xy, speed, logits))
    accumulator.add(audit_prediction(xy, speed, logits))
    report = accumulator.report()

    assert report["sample_count"] == 2
    assert report["all_candidates_geometry_pass_count"] == 0
    assert report["any_candidate_geometry_pass_count"] == 2
    assert report["candidate_results"][1]["geometry_pass_count"] == 0
    assert report["candidate_results"][1]["failure_counts"] == {
        "first_distance": 2,
        "step": 2,
        "heading": 2,
        "curvature": 2,
        "backward_step": 2,
        "geometric_speed": 2,
        "geometric_speed_rate": 2,
        "lateral_acceleration": 2,
        "speed_disagreement": 2,
        "distance_disagreement": 2,
    }
    assert report["selected_result"]["geometry_pass_count"] == 0
    assert report["selector"]["selection_counts"]["1"] == 2
    assert report["selector"]["distinct_selected_candidates"] == 1
    assert report["selector"]["dominant_candidate_fraction"] == 1.0
    assert report["selector"]["normalized_selection_entropy"] == 0.0


def test_audit_reports_stationary_drift_with_stable_failure_code():
    xy, speed, logits = _valid_predictions()
    xy[3] = [(0.009 * (index + 1), 0.0) for index in range(64)]
    speed[3] = [0.0] * 64

    audit = audit_prediction(xy, speed, logits)

    assert audit["selected_geometry_pass"] is False
    assert "stationary_drift" in audit["selected_failure_codes"]


def test_audit_reports_confined_oscillation_with_stable_failure_code():
    xy, speed, logits = _valid_predictions()
    xy[3] = [((index % 2) * 0.009, 0.0) for index in range(64)]
    speed[3] = [0.101] * 64

    audit = audit_prediction(xy, speed, logits)

    assert audit["selected_geometry_pass"] is False
    assert "stationary_drift" in audit["selected_failure_codes"]


def test_audit_accepts_geometry_consistent_crawl_motion():
    # HH_260906 - Keep offline classification aligned with the live explicit-stop threshold.
    xy, speed, logits = _valid_predictions()
    xy[3] = [(0.005 * (index + 1), 0.0) for index in range(64)]
    speed[3] = [0.05] * 64

    audit = audit_prediction(
        xy,
        speed,
        logits,
        current_speed_mps=0.05,
    )

    assert audit["selected_geometry_pass"] is True
    assert "stationary_drift" not in audit["selected_failure_codes"]


def test_audit_reports_integrated_distance_disagreement():
    # HH_260906 - Keep the offline audit aligned with the full-horizon live check.
    xy, speed, logits = _valid_predictions()
    xy[3] = [(0.1 * (index + 1), 0.0) for index in range(64)]
    speed[3] = [0.0001001] * 64

    audit = audit_prediction(
        xy,
        speed,
        logits,
        current_speed_mps=0.0001001,
    )

    assert audit["selected_geometry_pass"] is False
    assert "distance_disagreement" in audit["selected_failure_codes"]


def test_audit_reports_smooth_loop_with_curvature_failure_code():
    # HH_260906 - Keep audit classification identical to the live circular-loop rejection.
    xy, speed, logits = _valid_predictions()
    radius_m = 0.0255
    angle_step_rad = 0.6
    xy[3] = [
        (
            radius_m * math.sin((index + 1) * angle_step_rad),
            radius_m * (1.0 - math.cos((index + 1) * angle_step_rad)),
        )
        for index in range(64)
    ]
    step_m = 2.0 * radius_m * math.sin(angle_step_rad / 2.0)
    speed[3] = [step_m / 0.1] * 64

    audit = audit_prediction(
        xy,
        speed,
        logits,
        current_speed_mps=speed[3][0],
    )

    assert audit["selected_geometry_pass"] is False
    assert "curvature" in audit["selected_failure_codes"]


def test_audit_reports_unresolved_moving_heading_outside_stationary_radius():
    xy, speed, logits = _valid_predictions()
    xy[3] = [
        (0.049 if index == 0 or index % 2 == 0 else 0.051, 0.0)
        for index in range(64)
    ]
    speed[3] = [0.101] * 64

    audit = audit_prediction(xy, speed, logits)

    assert audit["selected_geometry_pass"] is False
    assert "heading" in audit["selected_failure_codes"]


def test_audit_rejects_low_speed_pure_lateral_deadband_shuttle():
    # HH_260906 - Mirror retained subcentimeter side-slip travel in exhaustive audit.
    xy, speed, logits = _valid_predictions()
    y_m = 0.0
    points = []
    selected_speed = []
    for index in range(64):
        dy_m = (0.0099, -0.0002)[index % 2]
        y_m += dy_m
        points.append((0.0, y_m))
        selected_speed.append(abs(dy_m) / 0.1)
    xy[3] = points
    speed[3] = selected_speed

    audit = audit_prediction(
        xy,
        speed,
        logits,
        current_speed_mps=0.099,
    )

    assert audit["selected_geometry_pass"] is False
    assert "heading" in audit["selected_failure_codes"]


def test_audit_reports_current_speed_transition_with_stable_failure_code():
    xy, speed, logits = _valid_predictions()

    audit = audit_prediction(
        xy,
        speed,
        logits,
        current_speed_mps=8.0,
    )

    assert audit["selected_geometry_pass"] is False
    assert "speed_rate" in audit["selected_failure_codes"]


def test_audit_accepts_short_dynamically_consistent_braking_stop():
    # HH_260906 - Mirror the live exemption for a legal monotonic stop under 5 cm.
    xy, speed, logits = _valid_predictions()
    selected_speed = [0.11] + [0.0] * 63
    x_m = 0.0
    selected_xy = []
    for speed_mps in selected_speed:
        x_m += speed_mps * 0.1
        selected_xy.append((x_m, 0.0))
    xy[3] = selected_xy
    speed[3] = selected_speed

    audit = audit_prediction(
        xy,
        speed,
        logits,
        current_speed_mps=0.4,
    )

    assert audit["selected_geometry_pass"] is True
    assert audit["selected_failure_codes"] == []


@pytest.mark.parametrize(
    "selected_speed",
    (
        [0.11] + [0.0002] * 63,
        [0.11, 0.0, 0.01] + [0.0] * 61,
    ),
)
def test_audit_short_stop_exemption_rejects_nonstop_or_reacceleration(
    selected_speed,
):
    # HH_260906 - Mirror rejection of short profiles that do not brake monotonically to zero.
    xy, speed, logits = _valid_predictions()
    x_m = 0.0
    selected_xy = []
    for speed_mps in selected_speed:
        x_m += speed_mps * 0.1
        selected_xy.append((x_m, 0.0))
    xy[3] = selected_xy
    speed[3] = selected_speed

    audit = audit_prediction(
        xy,
        speed,
        logits,
        current_speed_mps=0.4,
    )

    assert audit["selected_geometry_pass"] is False
    assert "extent" in audit["selected_failure_codes"]


def test_atomic_json_creates_one_new_parseable_result_and_refuses_overwrite(tmp_path):
    output = tmp_path / "nested" / "audit.json"
    payload = {"audit_id": "fixture", "vehicle_control_approved": False}

    written = _atomic_new_json(output, payload)

    assert written == output.absolute()
    assert json.loads(output.read_text(encoding="utf-8")) == payload
    assert list(output.parent.glob(".*.tmp.*")) == []
    with pytest.raises(ContractError, match="already exists"):
        _atomic_new_json(output, payload)


def test_atomic_json_preserves_a_concurrent_winner(monkeypatch, tmp_path):
    output = tmp_path / "audit.json"
    winner = '{"writer":"other"}\n'

    def competing_link(*args, **kwargs):
        # HH_260906 - Simulate another process winning after the initial absence check.
        output.write_text(winner, encoding="utf-8")
        raise FileExistsError

    monkeypatch.setattr(audit_runtime_module.os, "link", competing_link)

    with pytest.raises(ContractError, match="already exists"):
        _atomic_new_json(output, {"writer": "this-process"})

    assert output.read_text(encoding="utf-8") == winner
    assert list(output.parent.glob(".*.tmp.*")) == []


def test_cli_exposes_no_runtime_gate_threshold_override():
    arguments = parse_args(
        [
            "dataset",
            "--checkpoint",
            "model.pt",
            "--checkpoint-sha256",
            "a" * 64,
            "--output-json",
            "audit.json",
        ]
    )

    assert arguments.device == "cpu"
    assert arguments.split == "val"
    assert set(vars(arguments)) == {
        "dataset",
        "checkpoint",
        "checkpoint_sha256",
        "output_json",
        "split",
        "device",
        "batch_size",
    }


def test_audit_checkpoint_builds_dataset_from_checkpoint_model_config(
    monkeypatch, tmp_path
):
    custom_config = replace(ModelConfig(), hidden_width=48)
    loaded = SimpleNamespace(
        examples=(SimpleNamespace(episode_id="eval-episode", domain="carla"),),
        validation_report={
            "dataset_fingerprint_sha256": "c" * 64,
            "dataset_id": "fixture-dataset",
        },
    )
    events = []

    def fake_load_training_examples(*args, **kwargs):
        events.append("load_dataset_manifest")
        return loaded

    def fake_read_checkpoint_for_audit(**kwargs):
        events.append("read_checkpoint_config")
        return (
            {"model_state_dict": {}},
            custom_config,
            ("train-episode",),
            {
                "checkpoint_sha256": "a" * 64,
                "training_dataset_fingerprint_sha256": "b" * 64,
            },
        )

    class FakeDataset:
        def __init__(self, examples, model_config, **kwargs):
            events.append("construct_tensor_dataset")
            assert model_config == custom_config
            self.config = model_config
            self.examples = examples
            self.fingerprint_sha256 = "d" * 64

    def fake_validate_checkpoint_and_model(dataset, **kwargs):
        events.append("validate_model")
        assert dataset.config == custom_config
        assert kwargs["model_config"] == custom_config
        return SimpleNamespace(config=custom_config), {
            "checkpoint_sha256": "a" * 64,
            "training_dataset_fingerprint_sha256": "b" * 64,
        }

    monkeypatch.setattr(
        audit_runtime_module, "load_training_examples", fake_load_training_examples
    )
    monkeypatch.setattr(
        audit_runtime_module,
        "_read_checkpoint_for_audit",
        fake_read_checkpoint_for_audit,
    )
    monkeypatch.setattr(audit_runtime_module, "Common10TorchDataset", FakeDataset)
    monkeypatch.setattr(
        audit_runtime_module,
        "_validate_checkpoint_and_model",
        fake_validate_checkpoint_and_model,
    )
    monkeypatch.setattr(audit_runtime_module, "DataLoader", lambda *args, **kwargs: ())
    monkeypatch.setattr(
        audit_runtime_module,
        "_audit_model_batches",
        lambda *args, **kwargs: ({"sample_count": 1}, {"audit_wall_seconds": 0.1}),
    )
    monkeypatch.setattr(audit_runtime_module, "_runtime_abi", lambda: {})
    monkeypatch.setattr(audit_runtime_module, "_source_sha256", lambda path: "e" * 64)
    monkeypatch.setattr(
        audit_runtime_module,
        "_atomic_new_json",
        lambda path, value: tmp_path / "audit.json",
    )
    monkeypatch.setattr(
        audit_runtime_module,
        "_select_runtime_device",
        lambda name: torch.device("cpu"),
    )
    monkeypatch.setattr(audit_runtime_module, "_seed_everything", lambda *args: None)

    report = audit_checkpoint(
        tmp_path / "dataset",
        checkpoint_path=tmp_path / "custom-model.pt",
        expected_checkpoint_sha256="a" * 64,
        output_json=tmp_path / "audit.json",
    )

    assert events == [
        "load_dataset_manifest",
        "read_checkpoint_config",
        "construct_tensor_dataset",
        "validate_model",
    ]
    assert report["gate"]["thresholds"]["candidate_count"] == (
        custom_config.candidate_count
    )
