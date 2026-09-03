from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from portable_e2e import ContractError
from portable_e2e import compare as comparison_module
from portable_e2e.compare import EVALUATION_ID, REQUIRED_METRICS, compare_reports, main


def _report(
    path: Path,
    *,
    checkpoint: str,
    ade: float,
    device: str = "cpu",
    overrides: dict[str, Any] | None = None,
) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    metrics = {
        "loss": ade + 0.5,
        "regression_loss": ade + 0.4,
        "candidate_score_loss": 0.1,
        "oracle_ade_m": ade * 0.8,
        "selected_ade_m": ade,
        "selected_fde_m": ade * 2.0,
        "selected_speed_mae_mps": ade * 0.2,
        "selected_yaw_mae_rad": ade * 0.05,
        "selected_kinematic_speed_mae_mps": ade * 0.12,
        "ade_1p0s_m": ade * 0.4,
        "fde_1p0s_m": ade * 0.5,
        "speed_mae_1p0s_mps": ade * 0.1,
        "ade_3p0s_m": ade * 0.7,
        "fde_3p0s_m": ade * 1.1,
        "speed_mae_3p0s_mps": ade * 0.15,
    }
    report = {
        "evaluation_id": EVALUATION_ID,
        "status": "OPEN_LOOP_EVALUATION_COMPLETE",
        "checkpoint": checkpoint,
        "checkpoint_sha256": ("a" if checkpoint == "a.pt" else "b") * 64,
        "model_config_sha256": "c" * 64,
        "dataset_fingerprint_sha256": "1" * 64,
        "training_dataset_fingerprint_sha256": "3" * 64,
        "corpus_fingerprint_sha256": "2" * 64,
        "sample_count": 20,
        "evaluation_split": "val",
        "batch_size": 4,
        "device": device,
        "runtime": {
            "python": "3.12.3",
            "torch": "2.6.0",
            "numpy": "2.1.0",
            "pillow": "11.0.0",
            "torch_num_threads": 8,
            "torch_num_interop_threads": 4,
            "omp_num_threads": None,
            "mkl_num_threads": "8",
        },
        "hardware": {
            "device_type": "cpu" if device == "cpu" else "cuda",
            "device_name": "test-device",
        },
        "model_parameter_count": 1234,
        "timing": {
            "warmup_batches": 1,
            "warmup_samples": 4,
            "warmup_seconds": 0.01,
            "evaluation_wall_seconds": 0.2,
            "model_forward_seconds": 0.09,
            "model_forward_ms_per_sample": 4.5,
        },
        "metrics": metrics,
        "metric_counts": {name: 20 for name in metrics},
    }
    if overrides:
        report.update(overrides)
    path.write_text(
        json.dumps(report),
        encoding="utf-8",
    )
    return path


def test_compare_requires_same_split_samples_and_device(tmp_path: Path) -> None:
    first = _report(tmp_path / "first" / "metrics.json", checkpoint="a.pt", ade=1.0)
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)

    comparison = compare_reports((first, second))

    assert comparison["status"] == "FAIR_OPEN_LOOP_COMPARISON_READY"
    assert [report["metrics"]["selected_ade_m"] for report in comparison["reports"]] == [
        1.0,
        0.8,
    ]
    assert set(REQUIRED_METRICS) <= set(
        json.loads(first.read_text(encoding="utf-8"))["metrics"]
    )
    assert comparison["warmup_batches"] == 1
    assert comparison["warmup_samples"] == 4
    assert comparison["training_dataset_fingerprint_sha256"] == "3" * 64
    assert "LATENCY IS REFERENCE-ONLY" in comparison["warning"]
    assert "NO SHARED-SYSTEM RESOURCE OR SCHEDULER RESERVATION" in comparison["warning"]

    _report(second, checkpoint="b.pt", ade=0.8, device="cuda:0")
    with pytest.raises(ContractError, match="different device"):
        compare_reports((first, second))


def test_compare_cli_writes_json_and_markdown_without_overwrite(tmp_path: Path) -> None:
    first = _report(tmp_path / "first" / "metrics.json", checkpoint="a.pt", ade=1.0)
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)
    output_json = tmp_path / "comparison" / "summary.json"
    output_markdown = tmp_path / "comparison" / "summary.md"

    assert main(
        [
            str(first),
            str(second),
            "--output-json",
            str(output_json),
            "--output-markdown",
            str(output_markdown),
        ]
    ) == 0
    assert json.loads(output_json.read_text(encoding="utf-8"))["sample_count"] == 20
    markdown = output_markdown.read_text(encoding="utf-8")
    assert "| model |" in markdown
    assert "forward latency is reference-only" in markdown
    assert "scheduler reservation" in markdown
    assert main(
        [
            str(first),
            str(second),
            "--output-json",
            str(output_json),
            "--output-markdown",
            str(output_markdown),
        ]
    ) == 2


@pytest.mark.parametrize(
    ("field", "invalid"),
    [
        ("dataset_fingerprint_sha256", "not-a-sha256"),
        ("training_dataset_fingerprint_sha256", "3" * 63),
        ("corpus_fingerprint_sha256", ""),
        ("checkpoint_sha256", "A" * 64),
        ("model_config_sha256", "f" * 63),
        ("sample_count", True),
        ("model_parameter_count", 0),
        ("batch_size", False),
        ("device", "  "),
        ("checkpoint", ""),
        ("evaluation_split", ""),
        ("runtime", {}),
        ("hardware", {}),
    ],
)
def test_compare_rejects_malformed_required_provenance(
    tmp_path: Path, field: str, invalid: Any
) -> None:
    first = _report(
        tmp_path / "first" / "metrics.json",
        checkpoint="a.pt",
        ade=1.0,
        overrides={field: invalid},
    )
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)

    with pytest.raises(ContractError, match=field):
        compare_reports((first, second))


def test_compare_rejects_incomplete_metrics_and_counts(tmp_path: Path) -> None:
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)
    base_path = _report(tmp_path / "first" / "metrics.json", checkpoint="a.pt", ade=1.0)
    base = json.loads(base_path.read_text(encoding="utf-8"))

    missing_metric = dict(base["metrics"])
    missing_metric.pop("fde_3p0s_m")
    _report(
        base_path,
        checkpoint="a.pt",
        ade=1.0,
        overrides={"metrics": missing_metric},
    )
    with pytest.raises(ContractError, match="missing required metrics"):
        compare_reports((base_path, second))

    invalid_counts = {name: 20 for name in base["metrics"]}
    invalid_counts["selected_ade_m"] = True
    _report(
        base_path,
        checkpoint="a.pt",
        ade=1.0,
        overrides={"metric_counts": invalid_counts},
    )
    with pytest.raises(ContractError, match="positive integer"):
        compare_reports((base_path, second))


def test_compare_requires_identical_metric_keys_and_counts(tmp_path: Path) -> None:
    first = _report(tmp_path / "first" / "metrics.json", checkpoint="a.pt", ade=1.0)
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)
    second_report = json.loads(second.read_text(encoding="utf-8"))
    second_report["metrics"]["optional_new_metric"] = 1.0
    second_report["metric_counts"]["optional_new_metric"] = 20
    second.write_text(json.dumps(second_report), encoding="utf-8")

    with pytest.raises(ContractError, match="different metric key set"):
        compare_reports((first, second))

    second_report["metrics"].pop("optional_new_metric")
    second_report["metric_counts"].pop("optional_new_metric")
    second_report["metric_counts"]["ade_3p0s_m"] = 19
    second.write_text(json.dumps(second_report), encoding="utf-8")
    with pytest.raises(ContractError, match="different metric_counts"):
        compare_reports((first, second))


@pytest.mark.parametrize(
    ("field", "different"),
    [
        ("training_dataset_fingerprint_sha256", "4" * 64),
        ("evaluation_split", "test"),
        ("batch_size", 2),
        (
            "runtime",
            {
                "python": "3.12.3",
                "torch": "2.7.0",
                "numpy": "2.1.0",
                "pillow": "11.0.0",
                "torch_num_threads": 8,
                "torch_num_interop_threads": 4,
                "omp_num_threads": None,
                "mkl_num_threads": "8",
            },
        ),
        ("hardware", {"device_type": "cpu", "device_name": "another-cpu"}),
    ],
)
def test_compare_requires_same_split_batch_runtime_and_hardware(
    tmp_path: Path, field: str, different: Any
) -> None:
    first = _report(tmp_path / "first" / "metrics.json", checkpoint="a.pt", ade=1.0)
    overrides = {field: different}
    if field == "batch_size":
        overrides["timing"] = {
            "warmup_batches": 1,
            "warmup_samples": different,
            "warmup_seconds": 0.01,
            "evaluation_wall_seconds": 0.2,
            "model_forward_seconds": 0.09,
            "model_forward_ms_per_sample": 4.5,
        }
    second = _report(
        tmp_path / "second" / "metrics.json",
        checkpoint="b.pt",
        ade=0.8,
        overrides=overrides,
    )

    with pytest.raises(ContractError, match=f"different {field}"):
        compare_reports((first, second))


@pytest.mark.parametrize(
    ("field", "invalid", "message"),
    [
        ("torch_num_threads", 0, "positive integer"),
        ("torch_num_threads", True, "positive integer"),
        ("torch_num_interop_threads", None, "positive integer"),
        ("omp_num_threads", 8, "string or null"),
        ("mkl_num_threads", [], "string or null"),
    ],
)
def test_compare_rejects_malformed_thread_runtime_fields(
    tmp_path: Path, field: str, invalid: Any, message: str
) -> None:
    first = _report(tmp_path / "first" / "metrics.json", checkpoint="a.pt", ade=1.0)
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)
    report = json.loads(first.read_text(encoding="utf-8"))
    report["runtime"][field] = invalid
    first.write_text(json.dumps(report), encoding="utf-8")

    with pytest.raises(ContractError, match=message):
        compare_reports((first, second))


def test_compare_rejects_missing_thread_runtime_field(tmp_path: Path) -> None:
    first = _report(tmp_path / "first" / "metrics.json", checkpoint="a.pt", ade=1.0)
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)
    report = json.loads(first.read_text(encoding="utf-8"))
    del report["runtime"]["omp_num_threads"]
    first.write_text(json.dumps(report), encoding="utf-8")

    with pytest.raises(ContractError, match="runtime field 'omp_num_threads' is missing"):
        compare_reports((first, second))


@pytest.mark.parametrize(
    ("field", "different"),
    [
        ("torch_num_threads", 2),
        ("torch_num_interop_threads", 2),
        ("omp_num_threads", "2"),
        ("mkl_num_threads", None),
    ],
)
def test_compare_requires_same_thread_runtime_conditions(
    tmp_path: Path, field: str, different: Any
) -> None:
    first = _report(tmp_path / "first" / "metrics.json", checkpoint="a.pt", ade=1.0)
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)
    report = json.loads(second.read_text(encoding="utf-8"))
    report["runtime"][field] = different
    second.write_text(json.dumps(report), encoding="utf-8")

    with pytest.raises(ContractError, match="different runtime"):
        compare_reports((first, second))


@pytest.mark.parametrize(
    "timing",
    [
        {
            "warmup_batches": 1,
            "warmup_samples": 4,
            "warmup_seconds": 0.01,
            "model_forward_seconds": 0.09,
            "model_forward_ms_per_sample": 4.5,
        },
        {
            "warmup_batches": 1,
            "warmup_samples": 4,
            "warmup_seconds": 0.01,
            "evaluation_wall_seconds": 0.2,
            "model_forward_seconds": 0.0,
            "model_forward_ms_per_sample": 4.5,
        },
        {
            "warmup_batches": 1,
            "warmup_samples": 4,
            "warmup_seconds": 0.01,
            "evaluation_wall_seconds": 0.2,
            "model_forward_seconds": 0.09,
            "model_forward_ms_per_sample": 99.0,
        },
    ],
)
def test_compare_rejects_incomplete_nonpositive_or_inconsistent_timing(
    tmp_path: Path, timing: dict[str, float]
) -> None:
    first = _report(
        tmp_path / "first" / "metrics.json",
        checkpoint="a.pt",
        ade=1.0,
        overrides={"timing": timing},
    )
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)

    with pytest.raises(ContractError, match="timing"):
        compare_reports((first, second))


@pytest.mark.parametrize(
    ("field", "invalid", "message"),
    [
        ("warmup_batches", 0, "positive integer"),
        ("warmup_batches", True, "positive integer"),
        ("warmup_samples", 0, "positive integer"),
        ("warmup_samples", False, "positive integer"),
        ("warmup_seconds", 0.0, "positive finite"),
        ("warmup_seconds", "0.01", "positive finite"),
    ],
)
def test_compare_rejects_invalid_warmup_values(
    tmp_path: Path, field: str, invalid: Any, message: str
) -> None:
    first = _report(tmp_path / "first" / "metrics.json", checkpoint="a.pt", ade=1.0)
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)
    report = json.loads(first.read_text(encoding="utf-8"))
    report["timing"][field] = invalid
    first.write_text(json.dumps(report), encoding="utf-8")

    with pytest.raises(ContractError, match=message):
        compare_reports((first, second))


@pytest.mark.parametrize(
    ("timing_update", "message"),
    [
        ({"warmup_batches": 6}, "exceeds evaluation batch count"),
        ({"warmup_samples": 21}, "exceeds sample_count"),
        ({"warmup_samples": 5}, "exceeds warmup batch capacity"),
    ],
)
def test_compare_rejects_impossible_warmup_counts(
    tmp_path: Path, timing_update: dict[str, int], message: str
) -> None:
    first = _report(tmp_path / "first" / "metrics.json", checkpoint="a.pt", ade=1.0)
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)
    report = json.loads(first.read_text(encoding="utf-8"))
    report["timing"].update(timing_update)
    first.write_text(json.dumps(report), encoding="utf-8")

    with pytest.raises(ContractError, match=message):
        compare_reports((first, second))


@pytest.mark.parametrize(
    ("field", "different"),
    [
        ("warmup_batches", 2),
        ("warmup_samples", 3),
    ],
)
def test_compare_requires_same_warmup_batch_and_sample_conditions(
    tmp_path: Path, field: str, different: int
) -> None:
    first = _report(tmp_path / "first" / "metrics.json", checkpoint="a.pt", ade=1.0)
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)
    report = json.loads(second.read_text(encoding="utf-8"))
    report["timing"][field] = different
    second.write_text(json.dumps(report), encoding="utf-8")

    with pytest.raises(ContractError, match=rf"different timing\.{field}"):
        compare_reports((first, second))


def test_compare_allows_different_warmup_elapsed_time(tmp_path: Path) -> None:
    first = _report(tmp_path / "first" / "metrics.json", checkpoint="a.pt", ade=1.0)
    second = _report(tmp_path / "second" / "metrics.json", checkpoint="b.pt", ade=0.8)
    report = json.loads(second.read_text(encoding="utf-8"))
    report["timing"]["warmup_seconds"] = 0.02
    second.write_text(json.dumps(report), encoding="utf-8")

    comparison = compare_reports((first, second))

    assert comparison["status"] == "FAIR_OPEN_LOOP_COMPARISON_READY"


def test_cli_catches_type_error_without_traceback(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr(
        comparison_module,
        "compare_reports",
        lambda _paths: (_ for _ in ()).throw(TypeError("malformed nested value")),
    )

    result = main(
        [
            str(tmp_path / "one.json"),
            str(tmp_path / "two.json"),
            "--output-json",
            str(tmp_path / "summary.json"),
            "--output-markdown",
            str(tmp_path / "summary.md"),
        ]
    )

    captured = capsys.readouterr()
    assert result == 2
    assert "COMPARISON_ERROR: malformed nested value" in captured.err
    assert "Traceback" not in captured.err
