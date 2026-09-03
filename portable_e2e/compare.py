"""Fail-closed comparison of open-loop reports produced on one fixed split."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Any, Mapping, Sequence

from .contract import ContractError, _loads_json


EVALUATION_ID = "portable_e2e.open_loop_evaluation.v0"
COMPARISON_ID = "portable_e2e.open_loop_comparison.v0"
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
CORE_METRICS = (
    "loss",
    "regression_loss",
    "candidate_score_loss",
    "oracle_ade_m",
    "selected_ade_m",
    "selected_fde_m",
    "selected_speed_mae_mps",
    "selected_yaw_mae_rad",
    "selected_kinematic_speed_mae_mps",
)
REQUIRED_HORIZON_METRICS = (
    "ade_1p0s_m",
    "fde_1p0s_m",
    "speed_mae_1p0s_mps",
    "ade_3p0s_m",
    "fde_3p0s_m",
    "speed_mae_3p0s_mps",
)
REQUIRED_METRICS = frozenset((*CORE_METRICS, *REQUIRED_HORIZON_METRICS))
REQUIRED_TIMING_FIELDS = frozenset(
    (
        "warmup_batches",
        "warmup_samples",
        "warmup_seconds",
        "evaluation_wall_seconds",
        "model_forward_seconds",
        "model_forward_ms_per_sample",
    )
)
SUMMARY_METRICS = (
    "selected_ade_m",
    "selected_fde_m",
    "selected_speed_mae_mps",
    "selected_yaw_mae_rad",
    "selected_kinematic_speed_mae_mps",
    "ade_1p0s_m",
    "fde_1p0s_m",
    "ade_3p0s_m",
    "fde_3p0s_m",
    "ade_6p4s_m",
    "fde_6p4s_m",
)


def _nonempty_string(report: Mapping[str, Any], name: str, path: Path) -> str:
    value = report.get(name)
    if not isinstance(value, str) or not value.strip():
        raise ContractError(f"{path} field {name!r} must be a nonempty string")
    return value


def _sha256(report: Mapping[str, Any], name: str, path: Path) -> str:
    value = _nonempty_string(report, name, path)
    if SHA256_PATTERN.fullmatch(value) is None:
        raise ContractError(f"{path} field {name!r} must be a lowercase SHA-256")
    return value


def _positive_integer(report: Mapping[str, Any], name: str, path: Path) -> int:
    value = report.get(name)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ContractError(f"{path} field {name!r} must be a positive integer")
    return value


def _nonempty_object(report: Mapping[str, Any], name: str, path: Path) -> Mapping[str, Any]:
    value = report.get(name)
    if not isinstance(value, dict) or not value:
        raise ContractError(f"{path} field {name!r} must be a nonempty object")
    return value


def _validate_runtime(report: Mapping[str, Any], path: Path) -> None:
    runtime = _nonempty_object(report, "runtime", path)
    for name in ("torch_num_threads", "torch_num_interop_threads"):
        _positive_integer(runtime, name, path)
    for name in ("omp_num_threads", "mkl_num_threads"):
        if name not in runtime:
            raise ContractError(f"{path} runtime field {name!r} is missing")
        value = runtime[name]
        if value is not None and not isinstance(value, str):
            raise ContractError(
                f"{path} runtime field {name!r} must be a string or null"
            )


def _finite_number(value: Any, context: str, *, positive: bool = False) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or (positive and float(value) <= 0.0)
    ):
        qualifier = "positive finite" if positive else "finite"
        raise ContractError(f"{context} must be a {qualifier} number")
    return float(value)


def _read_report(path: Path) -> Mapping[str, Any]:
    try:
        report = _loads_json(path.read_text(encoding="utf-8"), str(path))
    except OSError as error:
        raise ContractError(f"cannot read evaluation report {path}: {error}") from error
    if report.get("evaluation_id") != EVALUATION_ID:
        raise ContractError(f"{path} is not a supported evaluation report")
    if report.get("status") != "OPEN_LOOP_EVALUATION_COMPLETE":
        raise ContractError(f"{path} is not a completed evaluation")

    _sha256(report, "dataset_fingerprint_sha256", path)
    _sha256(report, "training_dataset_fingerprint_sha256", path)
    _sha256(report, "corpus_fingerprint_sha256", path)
    _sha256(report, "checkpoint_sha256", path)
    _sha256(report, "model_config_sha256", path)
    sample_count = _positive_integer(report, "sample_count", path)
    _positive_integer(report, "model_parameter_count", path)
    batch_size = _positive_integer(report, "batch_size", path)
    _nonempty_string(report, "device", path)
    _nonempty_string(report, "checkpoint", path)
    _nonempty_string(report, "evaluation_split", path)
    _validate_runtime(report, path)
    _nonempty_object(report, "hardware", path)

    timing = report.get("timing")
    if not isinstance(timing, dict):
        raise ContractError(f"{path} does not contain timing metrics")
    if set(timing) != REQUIRED_TIMING_FIELDS:
        missing = sorted(REQUIRED_TIMING_FIELDS - set(timing))
        extra = sorted(set(timing) - REQUIRED_TIMING_FIELDS)
        raise ContractError(
            f"{path} timing fields do not match the evaluation schema; "
            f"missing={missing}, extra={extra}"
        )
    parsed_timing = {
        name: _finite_number(timing[name], f"{path} timing field {name!r}", positive=True)
        for name in (
            "warmup_seconds",
            "evaluation_wall_seconds",
            "model_forward_seconds",
            "model_forward_ms_per_sample",
        )
    }
    warmup_batches = _positive_integer(timing, "warmup_batches", path)
    warmup_samples = _positive_integer(timing, "warmup_samples", path)
    evaluation_batches = math.ceil(sample_count / batch_size)
    if warmup_batches > evaluation_batches:
        raise ContractError(f"{path} warmup_batches exceeds evaluation batch count")
    if warmup_samples > sample_count:
        raise ContractError(f"{path} warmup_samples exceeds sample_count")
    if warmup_samples > warmup_batches * batch_size:
        raise ContractError(f"{path} warmup_samples exceeds warmup batch capacity")
    if parsed_timing["model_forward_seconds"] > parsed_timing["evaluation_wall_seconds"]:
        raise ContractError(f"{path} model forward time exceeds evaluation wall time")
    expected_ms = 1000.0 * parsed_timing["model_forward_seconds"] / sample_count
    if not math.isclose(
        parsed_timing["model_forward_ms_per_sample"],
        expected_ms,
        rel_tol=1.0e-9,
        abs_tol=1.0e-9,
    ):
        raise ContractError(f"{path} per-sample timing is internally inconsistent")

    metrics = report.get("metrics")
    if not isinstance(metrics, dict):
        raise ContractError(f"{path} does not contain metrics")
    missing_metrics = sorted(REQUIRED_METRICS - set(metrics))
    if missing_metrics:
        raise ContractError(f"{path} is missing required metrics: {missing_metrics}")
    for name, value in metrics.items():
        parsed = _finite_number(value, f"{path} metric {name!r}")
        if parsed < 0.0:
            raise ContractError(f"{path} metric {name!r} must be nonnegative")

    metric_counts = report.get("metric_counts")
    if not isinstance(metric_counts, dict):
        raise ContractError(f"{path} does not contain metric_counts")
    if set(metric_counts) != set(metrics):
        missing = sorted(set(metrics) - set(metric_counts))
        extra = sorted(set(metric_counts) - set(metrics))
        raise ContractError(
            f"{path} metric_counts keys do not match metrics; "
            f"missing={missing}, extra={extra}"
        )
    for name, value in metric_counts.items():
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ContractError(
                f"{path} metric count {name!r} must be a positive integer"
            )
        if value > sample_count:
            raise ContractError(f"{path} metric count {name!r} exceeds sample_count")
    for name in CORE_METRICS:
        if metric_counts[name] != sample_count:
            raise ContractError(
                f"{path} core metric count {name!r} must equal sample_count"
            )
    return report


def compare_reports(paths: Sequence[Path]) -> dict[str, Any]:
    if len(paths) < 2:
        raise ContractError("comparison needs at least two evaluation reports")
    reports = [_read_report(path.expanduser().resolve()) for path in paths]
    reference = reports[0]
    fixed_fields = (
        "dataset_fingerprint_sha256",
        "training_dataset_fingerprint_sha256",
        "corpus_fingerprint_sha256",
        "sample_count",
        "device",
        "evaluation_split",
        "batch_size",
        "runtime",
        "hardware",
    )
    for field in fixed_fields:
        expected = reference.get(field)
        for index, report in enumerate(reports[1:], start=1):
            if report.get(field) != expected:
                raise ContractError(
                    f"report {paths[index]} has a different {field}; comparison is not fair"
                )
    reference_timing = reference["timing"]
    for field in ("warmup_batches", "warmup_samples"):
        expected = reference_timing[field]
        for index, report in enumerate(reports[1:], start=1):
            if report["timing"][field] != expected:
                raise ContractError(
                    f"report {paths[index]} has a different timing.{field}; "
                    "comparison is not fair"
                )
    metric_names = set(reference["metrics"])
    reference_counts = reference["metric_counts"]
    for index, report in enumerate(reports[1:], start=1):
        if set(report["metrics"]) != metric_names:
            raise ContractError(
                f"report {paths[index]} has a different metric key set; comparison is not fair"
            )
        if report["metric_counts"] != reference_counts:
            raise ContractError(
                f"report {paths[index]} has different metric_counts; comparison is not fair"
            )
    rows = []
    for path, report in zip(paths, reports):
        metrics = report["metrics"]
        timing = report.get("timing")
        if not isinstance(timing, dict):
            raise ContractError(f"{path} does not contain timing metrics")
        row = {
            "label": path.parent.name or path.stem,
            "report": str(path.expanduser().resolve()),
            "checkpoint": report["checkpoint"],
            "checkpoint_sha256": report["checkpoint_sha256"],
            "model_config_sha256": report["model_config_sha256"],
            "model_parameter_count": report["model_parameter_count"],
            "model_forward_ms_per_sample": timing["model_forward_ms_per_sample"],
            "metrics": {
                name: metrics[name] for name in SUMMARY_METRICS if name in metrics
            },
        }
        rows.append(row)
    return {
        "comparison_id": COMPARISON_ID,
        "status": "FAIR_OPEN_LOOP_COMPARISON_READY",
        "dataset_fingerprint_sha256": reference["dataset_fingerprint_sha256"],
        "training_dataset_fingerprint_sha256": reference[
            "training_dataset_fingerprint_sha256"
        ],
        "corpus_fingerprint_sha256": reference["corpus_fingerprint_sha256"],
        "sample_count": reference["sample_count"],
        "device": reference["device"],
        "evaluation_split": reference["evaluation_split"],
        "batch_size": reference["batch_size"],
        "warmup_batches": reference_timing["warmup_batches"],
        "warmup_samples": reference_timing["warmup_samples"],
        "runtime": reference["runtime"],
        "hardware": reference["hardware"],
        "metric_counts": reference_counts,
        "reports": rows,
        "warning": (
            "OPEN-LOOP COMPARISON ONLY — LATENCY IS REFERENCE-ONLY BECAUSE "
            "NO SHARED-SYSTEM RESOURCE OR SCHEDULER RESERVATION WAS ENFORCED; "
            "CLOSED-LOOP AND SAFETY GATES STILL REQUIRED"
        ),
    }


def _markdown(comparison: Mapping[str, Any]) -> str:
    reports = comparison["reports"]
    present_metrics = [
        name
        for name in SUMMARY_METRICS
        if any(name in report["metrics"] for report in reports)
    ]
    headings = ["model", "parameters", "forward ms/sample", *present_metrics]
    lines = [
        "# Portable E2E open-loop comparison",
        "",
        f"- samples: {comparison['sample_count']}",
        f"- evaluation split: `{comparison['evaluation_split']}`",
        f"- batch size: {comparison['batch_size']}",
        (
            "- warmup: "
            f"{comparison['warmup_batches']} batch(es), "
            f"{comparison['warmup_samples']} sample(s)"
        ),
        f"- device: `{comparison['device']}`",
        f"- split fingerprint: `{comparison['dataset_fingerprint_sha256']}`",
        (
            "- training split fingerprint: "
            f"`{comparison['training_dataset_fingerprint_sha256']}`"
        ),
        (
            "- forward latency is reference-only: no shared-system resource or "
            "scheduler reservation was enforced"
        ),
        "- this table does not grant closed-loop or vehicle-control approval",
        "",
        "| " + " | ".join(headings) + " |",
        "| " + " | ".join("---" for _ in headings) + " |",
    ]
    for report in reports:
        values = [
            str(report["label"]),
            str(report["model_parameter_count"]),
            f"{float(report['model_forward_ms_per_sample']):.3f}",
        ]
        for name in present_metrics:
            value = report["metrics"].get(name)
            values.append("—" if value is None else f"{float(value):.5f}")
        lines.append("| " + " | ".join(values) + " |")
    lines.append("")
    return "\n".join(lines)


def _write_new(path: Path, payload: str) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise ContractError(f"refusing to overwrite comparison output: {path}")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.tmp.", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, path)
        except FileExistsError as error:
            raise ContractError(f"refusing to overwrite comparison output: {path}") from error
    finally:
        if temporary.exists():
            temporary.unlink()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare fixed-split open-loop reports.")
    parser.add_argument("reports", nargs="+", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        output_json = args.output_json.expanduser().resolve()
        output_markdown = args.output_markdown.expanduser().resolve()
        if output_json == output_markdown:
            raise ContractError("JSON and Markdown comparison outputs must be different files")
        for output in (output_json, output_markdown):
            if output.exists():
                raise ContractError(f"refusing to overwrite comparison output: {output}")
        comparison = compare_reports(args.reports)
        _write_new(
            output_json,
            json.dumps(comparison, indent=2, sort_keys=True, allow_nan=False) + "\n",
        )
        _write_new(output_markdown, _markdown(comparison))
    except (ContractError, OSError, TypeError, ValueError) as error:
        print(f"COMPARISON_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps(comparison, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
