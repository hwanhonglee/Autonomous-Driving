#!/usr/bin/env python3
"""Fit guarded CARLA accel/brake-map candidates from an actuation sweep CSV."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Sequence
import warnings

import matplotlib

matplotlib.use("Agg")
with warnings.catch_warnings():
    # Some mixed system/user matplotlib installs warn about the unused 3D
    # backend. This analyzer produces only 2D figures.
    warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")
    import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAP_DIR = (
    ROOT
    / "src/universe/autoware_universe/simulator/autoware_carla_interface"
    / "calibration_maps"
)
DEFAULT_CONVERTER_CONFIG = (
    ROOT
    / "src/universe/autoware_universe/simulator/autoware_carla_interface"
    / "config/raw_vehicle_cmd_converter.param.yaml"
)
DEFAULT_STEER_MAP = DEFAULT_MAP_DIR / "steer_map.csv"
CONVERTER_MAP_KEYS = {
    "accel": "csv_path_accel_map",
    "brake": "csv_path_brake_map",
    "steer": "csv_path_steer_map",
}
REQUIRED_COLUMNS = {
    "sim_time_s",
    "phase",
    "case_id",
    "case_kind",
    "repeat",
    "applied_accel",
    "applied_brake",
    "velocity_mps",
    "imu_accel_x_mps2",
    "safety_ok",
    "fit_eligible",
}
ENRICHED_COLUMNS = [
    "analysis_status",
    "analysis_invalid_reasons",
    "derived_accel_mps2",
    "fit_measured_accel_mps2",
    "fit_row_eligible",
    "fit_exclusion_reason",
    "map_kind",
    "map_pedal_index",
    "map_velocity_index",
    "map_pedal",
    "map_velocity_mps",
    "cell_sample_count",
    "cell_repeat_count",
    "cell_covered",
    "cell_original_accel_mps2",
    "cell_measured_accel_mps2",
    "cell_candidate_accel_mps2",
    "cell_modified",
    "cell_fit_clipped",
    "cell_rejected_reason",
]


@dataclass(frozen=True)
class MapTable:
    path: Path
    corner_label: str
    velocities: np.ndarray
    pedals: np.ndarray
    values: np.ndarray
    header_tokens: tuple[str, ...]
    row_label_tokens: tuple[str, ...]
    value_tokens: tuple[tuple[str, ...], ...]


@dataclass
class CellFit:
    map_kind: str
    pedal_index: int
    velocity_index: int
    pedal: float
    velocity_mps: float
    sample_count: int
    repeat_count: int
    measured_accel_mps2: float | None
    original_accel_mps2: float
    candidate_accel_mps2: float
    covered: bool
    modified: bool
    fit_clipped: bool
    rejected_reason: str

    def as_dict(self) -> dict[str, object]:
        return {
            "map_kind": self.map_kind,
            "pedal_index": self.pedal_index,
            "velocity_index": self.velocity_index,
            "pedal": self.pedal,
            "velocity_mps": self.velocity_mps,
            "sample_count": self.sample_count,
            "repeat_count": self.repeat_count,
            "measured_accel_mps2": self.measured_accel_mps2,
            "original_accel_mps2": self.original_accel_mps2,
            "candidate_accel_mps2": self.candidate_accel_mps2,
            "covered": self.covered,
            "modified": self.modified,
            "fit_clipped": self.fit_clipped,
            "rejected_reason": self.rejected_reason,
        }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--sweep-summary",
        type=Path,
        help="Sweep summary used to reject partial or failed runs.",
    )
    parser.add_argument(
        "--original-accel-map",
        type=Path,
        default=DEFAULT_MAP_DIR / "accel_map.csv",
    )
    parser.add_argument(
        "--original-brake-map",
        type=Path,
        default=DEFAULT_MAP_DIR / "brake_map.csv",
    )
    parser.add_argument(
        "--stock-converter-config",
        type=Path,
        default=DEFAULT_CONVERTER_CONFIG,
        help="Complete stock CARLA converter config used as the candidate base.",
    )
    parser.add_argument(
        "--stock-steer-map",
        type=Path,
        default=DEFAULT_STEER_MAP,
        help="Unmodified steer map referenced by the generated candidate config.",
    )
    parser.add_argument("--candidate-dir", type=Path)
    parser.add_argument("--min-samples", type=int, default=30)
    parser.add_argument("--min-repeats", type=int, default=3)
    parser.add_argument(
        "--fit-source",
        choices=("derived", "imu"),
        default="derived",
        help="Acceleration signal used for fitting (both signals remain in the CSV).",
    )
    parser.add_argument(
        "--max-cell-change",
        type=float,
        default=0.75,
        help="Maximum absolute acceleration change allowed in one covered cell.",
    )
    parser.add_argument(
        "--strict-monotonic-margin",
        type=float,
        default=1.0e-6,
        help="Minimum acceleration separation between adjacent pedal rows.",
    )
    return parser.parse_args(argv)


def _finite_float(value: object, field: str, row_number: int | None = None) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        location = f" at row {row_number}" if row_number is not None else ""
        raise ValueError(f"{field}{location} is not a number: {value!r}") from error
    if not math.isfinite(result):
        location = f" at row {row_number}" if row_number is not None else ""
        raise ValueError(f"{field}{location} must be finite")
    return result


def _is_true(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def read_map(path: Path) -> MapTable:
    path = path.expanduser().resolve()
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.reader(stream))
    if len(rows) < 2 or len(rows[0]) < 2:
        raise ValueError(f"invalid actuation map: {path}")
    width = len(rows[0])
    if any(len(row) != width for row in rows):
        raise ValueError(f"ragged actuation map: {path}")
    velocities = np.asarray(
        [_finite_float(token, "velocity") for token in rows[0][1:]], dtype=float
    )
    pedals = np.asarray(
        [_finite_float(row[0], "pedal") for row in rows[1:]], dtype=float
    )
    values = np.asarray(
        [[_finite_float(token, "acceleration") for token in row[1:]] for row in rows[1:]],
        dtype=float,
    )
    if np.any(np.diff(velocities) <= 0.0) or np.any(np.diff(pedals) <= 0.0):
        raise ValueError(f"map axes must be strictly increasing: {path}")
    if not math.isclose(float(pedals[0]), 0.0, abs_tol=1.0e-12):
        raise ValueError(f"first pedal row must be zero: {path}")
    return MapTable(
        path=path,
        corner_label=rows[0][0],
        velocities=velocities,
        pedals=pedals,
        values=values,
        header_tokens=tuple(rows[0][1:]),
        row_label_tokens=tuple(row[0] for row in rows[1:]),
        value_tokens=tuple(tuple(row[1:]) for row in rows[1:]),
    )


def validate_source_maps(accel: MapTable, brake: MapTable) -> None:
    if accel.velocities.shape != brake.velocities.shape or not np.array_equal(
        accel.velocities, brake.velocities
    ):
        raise ValueError("accel and brake map velocity axes must be identical")
    if not np.array_equal(accel.values[0], brake.values[0]):
        raise ValueError("accel and brake maps must share an identical zero-pedal row")
    if np.any(np.diff(accel.values, axis=0) <= 0.0):
        raise ValueError("original accel map is not strictly increasing with pedal")
    if np.any(np.diff(brake.values, axis=0) >= 0.0):
        raise ValueError("original brake map is not strictly decreasing with pedal")


def read_raw_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.expanduser().resolve().open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"CSV has no header: {path}")
        missing = sorted(REQUIRED_COLUMNS - set(reader.fieldnames))
        if missing:
            raise ValueError(f"raw sweep CSV is missing columns: {', '.join(missing)}")
        rows = [dict(row) for row in reader]
    if not rows:
        raise ValueError(f"raw sweep CSV has no rows: {path}")
    return list(reader.fieldnames), rows


def read_sweep_summary(path: Path) -> dict[str, object]:
    path = path.expanduser().resolve()
    with path.open(encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, dict):
        raise ValueError(f"sweep summary must contain a JSON object: {path}")
    return payload


def derive_acceleration(rows: list[dict[str, str]]) -> np.ndarray:
    """Differentiate velocity inside each independent command case."""
    result = np.full(len(rows), np.nan, dtype=float)
    groups: dict[tuple[str, str], list[tuple[int, float, float]]] = {}
    for index, row in enumerate(rows):
        case_id = row["case_id"].strip()
        repeat = row["repeat"].strip()
        if not case_id or not repeat:
            continue
        try:
            time_s = _finite_float(row["sim_time_s"], "sim_time_s", index + 2)
            velocity = _finite_float(row["velocity_mps"], "velocity_mps", index + 2)
        except ValueError:
            # Startup and watchdog rows intentionally leave telemetry blank. Keep
            # them in the enriched output, but never let them poison other rows.
            continue
        groups.setdefault((case_id, repeat), []).append((index, time_s, velocity))

    for samples in groups.values():
        ordered = sorted(samples, key=lambda sample: sample[1])
        indices = [sample[0] for sample in ordered]
        times = np.asarray([sample[1] for sample in ordered])
        velocities = np.asarray([sample[2] for sample in ordered])
        for position, row_index in enumerate(indices):
            before = position - 1
            while before >= 0 and times[before] >= times[position]:
                before -= 1
            after = position + 1
            while after < len(indices) and times[after] <= times[position]:
                after += 1
            if before >= 0 and after < len(indices):
                delta_t = times[after] - times[before]
                result[row_index] = (velocities[after] - velocities[before]) / delta_t
            elif after < len(indices):
                delta_t = times[after] - times[position]
                result[row_index] = (velocities[after] - velocities[position]) / delta_t
            elif before >= 0:
                delta_t = times[position] - times[before]
                result[row_index] = (velocities[position] - velocities[before]) / delta_t
    return result


def _nearest_index(axis: np.ndarray, value: float) -> int:
    return int(np.argmin(np.abs(axis - value)))


def classify_rows(
    rows: list[dict[str, str]],
    derived: np.ndarray,
    accel_map: MapTable,
    brake_map: MapTable,
    fit_source: str,
) -> tuple[list[dict[str, object]], dict[tuple[str, int, int], list[int]]]:
    enriched: list[dict[str, object]] = []
    assignments: dict[tuple[str, int, int], list[int]] = {}
    for index, (row, derived_accel) in enumerate(zip(rows, derived)):
        extra: dict[str, object] = {
            "analysis_status": "",
            "analysis_invalid_reasons": "",
            "derived_accel_mps2": (
                float(derived_accel) if math.isfinite(float(derived_accel)) else ""
            ),
            "fit_measured_accel_mps2": "",
            "fit_row_eligible": 0,
            "fit_exclusion_reason": "",
            "map_kind": "",
            "map_pedal_index": "",
            "map_velocity_index": "",
            "map_pedal": "",
            "map_velocity_mps": "",
            "cell_sample_count": "",
            "cell_repeat_count": "",
            "cell_covered": "",
            "cell_original_accel_mps2": "",
            "cell_measured_accel_mps2": "",
            "cell_candidate_accel_mps2": "",
            "cell_modified": "",
            "cell_fit_clipped": "",
            "cell_rejected_reason": "",
        }
        kind = row["case_kind"].strip().lower()
        reasons: list[str] = []
        if row["phase"].strip().lower() != "sample":
            reasons.append("phase")
        if not _is_true(row["fit_eligible"]):
            reasons.append("fit_eligible")
        if not _is_true(row["safety_ok"]):
            reasons.append("safety")
        if kind not in {"accel", "brake", "coast"}:
            reasons.append("case_kind")
        measured = derived_accel
        if fit_source == "imu":
            try:
                measured = _finite_float(
                    row["imu_accel_x_mps2"], "imu_accel_x_mps2", index + 2
                )
            except ValueError:
                measured = math.nan
        if not math.isfinite(float(measured)):
            reasons.append("acceleration")

        if not reasons:
            table = accel_map if kind in {"accel", "coast"} else brake_map
            pedal_field = "applied_accel" if kind == "accel" else "applied_brake"
            try:
                pedal = (
                    0.0
                    if kind == "coast"
                    else _finite_float(row[pedal_field], pedal_field, index + 2)
                )
            except ValueError:
                reasons.append("pedal")
                pedal = math.nan
            try:
                velocity = _finite_float(
                    row["velocity_mps"], "velocity_mps", index + 2
                )
            except ValueError:
                reasons.append("velocity")
                velocity = math.nan
            if not reasons:
                pedal_index = (
                    0 if kind == "coast" else _nearest_index(table.pedals, pedal)
                )
                velocity_index = _nearest_index(table.velocities, velocity)
                # Only explicit coast cases may update the shared zero-pedal row;
                # malformed zero-valued accel/brake cases cannot claim that coverage.
                if pedal_index == 0 and kind != "coast":
                    reasons.append("zero_pedal")
                else:
                    key = (kind, pedal_index, velocity_index)
                    assignments.setdefault(key, []).append(index)
                    extra.update(
                        {
                            "fit_measured_accel_mps2": float(measured),
                            "fit_row_eligible": 1,
                            "map_kind": kind,
                            "map_pedal_index": pedal_index,
                            "map_velocity_index": velocity_index,
                            "map_pedal": float(table.pedals[pedal_index]),
                            "map_velocity_mps": float(table.velocities[velocity_index]),
                        }
                    )
        extra["fit_exclusion_reason"] = ";".join(reasons)
        enriched.append(extra)
    return enriched, assignments


def _bounded_isotonic(
    target: np.ndarray, lower: np.ndarray, upper: np.ndarray
) -> np.ndarray:
    """Project targets to a nondecreasing sequence with per-value bounds."""
    if target.ndim != 1 or lower.shape != target.shape or upper.shape != target.shape:
        raise ValueError("isotonic inputs must be one-dimensional and equally sized")
    if np.any(lower > upper):
        raise ValueError("infeasible isotonic bounds")

    blocks: list[dict[str, object]] = []
    for index in range(len(target)):
        block = {
            "start": index,
            "end": index + 1,
            "sum": float(target[index]),
            "weight": 1,
            "lower": float(lower[index]),
            "upper": float(upper[index]),
        }
        block["value"] = float(
            np.clip(
                block["sum"] / block["weight"], block["lower"], block["upper"]
            )
        )
        blocks.append(block)
        while len(blocks) >= 2 and float(blocks[-2]["value"]) > float(
            blocks[-1]["value"]
        ):
            right = blocks.pop()
            left = blocks.pop()
            merged_lower = max(float(left["lower"]), float(right["lower"]))
            merged_upper = min(float(left["upper"]), float(right["upper"]))
            if merged_lower > merged_upper + 1.0e-12:
                raise ValueError("infeasible bounded monotonic fit")
            merged = {
                "start": left["start"],
                "end": right["end"],
                "sum": float(left["sum"]) + float(right["sum"]),
                "weight": int(left["weight"]) + int(right["weight"]),
                "lower": merged_lower,
                "upper": merged_upper,
            }
            merged["value"] = float(
                np.clip(
                    merged["sum"] / merged["weight"],
                    merged["lower"],
                    merged["upper"],
                )
            )
            blocks.append(merged)

    result = np.empty_like(target, dtype=float)
    for block in blocks:
        result[int(block["start"]) : int(block["end"])] = float(block["value"])
    return result


def _fit_monotonic_column(
    original: np.ndarray,
    estimates: np.ndarray,
    covered: np.ndarray,
    direction: int,
    max_cell_change: float,
    margin: float,
) -> np.ndarray:
    transformed_original = direction * original
    target = transformed_original.copy()
    target[covered] = direction * estimates[covered]
    lower = transformed_original.copy()
    upper = transformed_original.copy()
    lower[covered] -= max_cell_change
    upper[covered] += max_cell_change

    indices = np.arange(len(original), dtype=float)
    shifted_target = target - margin * indices
    shifted_lower = lower - margin * indices
    shifted_upper = upper - margin * indices
    fitted = _bounded_isotonic(shifted_target, shifted_lower, shifted_upper)
    candidate = direction * (fitted + margin * indices)
    candidate[~covered] = original[~covered]
    candidate[0] = original[0]
    if np.any(np.diff(direction * candidate) < margin * (1.0 - 1.0e-6)):
        raise ValueError("bounded fitting failed to preserve strict pedal monotonicity")
    return candidate


def fit_maps(
    rows: list[dict[str, str]],
    enriched: list[dict[str, object]],
    assignments: dict[tuple[str, int, int], list[int]],
    accel_map: MapTable,
    brake_map: MapTable,
    min_samples: int,
    min_repeats: int,
    max_cell_change: float,
    margin: float,
) -> tuple[dict[str, np.ndarray], dict[tuple[str, int, int], CellFit]]:
    tables = {"accel": accel_map, "brake": brake_map}
    candidates = {kind: table.values.copy() for kind, table in tables.items()}
    estimates: dict[str, np.ndarray] = {
        kind: np.full(table.values.shape, np.nan) for kind, table in tables.items()
    }
    covered_masks: dict[str, np.ndarray] = {
        kind: np.zeros(table.values.shape, dtype=bool) for kind, table in tables.items()
    }
    coast_estimates = np.full(len(accel_map.velocities), np.nan)
    coast_covered = np.zeros(len(accel_map.velocities), dtype=bool)
    coast_rejected_reasons = ["" for _ in accel_map.velocities]
    counts: dict[tuple[str, int, int], tuple[int, int]] = {}

    for key, row_indices in assignments.items():
        kind, pedal_index, velocity_index = key
        values = np.asarray(
            [float(enriched[index]["fit_measured_accel_mps2"]) for index in row_indices]
        )
        repeats = {rows[index]["repeat"].strip() for index in row_indices}
        sample_count = len(values)
        repeat_count = len(repeats)
        counts[key] = (sample_count, repeat_count)
        estimate = float(np.median(values))
        if kind == "coast":
            coast_estimates[velocity_index] = estimate
            if sample_count >= min_samples and repeat_count >= min_repeats:
                coast_covered[velocity_index] = True
        else:
            estimates[kind][pedal_index, velocity_index] = estimate
            if sample_count >= min_samples and repeat_count >= min_repeats:
                covered_masks[kind][pedal_index, velocity_index] = True

    # The zero-pedal response is one physical row shared by both maps. Its
    # feasible interval accounts for adjacent covered cells moving at the same
    # time; uncovered cells remain hard bounds at their original values.
    for velocity_index, is_covered in enumerate(coast_covered):
        if not is_covered:
            continue
        original = float(accel_map.values[0, velocity_index])
        brake_lower_bounds = brake_map.values[:, velocity_index].copy()
        brake_lower_bounds[covered_masks["brake"][:, velocity_index]] -= (
            max_cell_change
        )
        accel_upper_bounds = accel_map.values[:, velocity_index].copy()
        accel_upper_bounds[covered_masks["accel"][:, velocity_index]] += (
            max_cell_change
        )
        pedal_offsets = margin * np.arange(len(brake_map.pedals), dtype=float)
        lower = max(
            original - max_cell_change,
            float(np.max(brake_lower_bounds[1:] + pedal_offsets[1:])),
        )
        pedal_offsets = margin * np.arange(len(accel_map.pedals), dtype=float)
        upper = min(
            original + max_cell_change,
            float(np.min(accel_upper_bounds[1:] - pedal_offsets[1:])),
        )
        if lower > upper:
            coast_rejected_reasons[velocity_index] = "no_shared_monotonic_interval"
            continue
        shared_value = float(np.clip(coast_estimates[velocity_index], lower, upper))
        candidates["accel"][0, velocity_index] = shared_value
        candidates["brake"][0, velocity_index] = shared_value

    for kind, table in tables.items():
        direction = 1 if kind == "accel" else -1
        for velocity_index in range(len(table.velocities)):
            covered = covered_masks[kind][:, velocity_index].copy()
            covered[0] = False
            column_estimates = estimates[kind][:, velocity_index].copy()
            working_column = table.values[:, velocity_index].copy()
            working_column[0] = candidates[kind][0, velocity_index]
            column_estimates[~covered] = working_column[~covered]
            candidates[kind][:, velocity_index] = _fit_monotonic_column(
                working_column,
                column_estimates,
                covered,
                direction,
                max_cell_change,
                margin,
            )
    cells: dict[tuple[str, int, int], CellFit] = {}
    for kind, table in tables.items():
        for pedal_index, pedal in enumerate(table.pedals):
            for velocity_index, velocity in enumerate(table.velocities):
                key = (kind, pedal_index, velocity_index)
                coverage_key = (
                    ("coast", 0, velocity_index) if pedal_index == 0 else key
                )
                sample_count, repeat_count = counts.get(coverage_key, (0, 0))
                estimate = (
                    coast_estimates[velocity_index]
                    if pedal_index == 0
                    else estimates[kind][pedal_index, velocity_index]
                )
                estimate_value = float(estimate) if math.isfinite(float(estimate)) else None
                original = float(table.values[pedal_index, velocity_index])
                candidate = float(candidates[kind][pedal_index, velocity_index])
                is_covered = (
                    bool(coast_covered[velocity_index])
                    if pedal_index == 0
                    else bool(covered_masks[kind][pedal_index, velocity_index])
                )
                rejected_reason = (
                    coast_rejected_reasons[velocity_index] if pedal_index == 0 else ""
                )
                modified = is_covered and not math.isclose(
                    original, candidate, rel_tol=0.0, abs_tol=1.0e-12
                )
                clipped = (
                    is_covered
                    and estimate_value is not None
                    and not math.isclose(
                        estimate_value, candidate, rel_tol=0.0, abs_tol=1.0e-9
                    )
                )
                cells[key] = CellFit(
                    map_kind=kind,
                    pedal_index=pedal_index,
                    velocity_index=velocity_index,
                    pedal=float(pedal),
                    velocity_mps=float(velocity),
                    sample_count=sample_count,
                    repeat_count=repeat_count,
                    measured_accel_mps2=estimate_value,
                    original_accel_mps2=original,
                    candidate_accel_mps2=candidate,
                    covered=is_covered,
                    modified=modified,
                    fit_clipped=clipped,
                    rejected_reason=rejected_reason,
                )

    for key, row_indices in assignments.items():
        kind, _, velocity_index = key
        cell = cells[("accel", 0, velocity_index)] if kind == "coast" else cells[key]
        for index in row_indices:
            enriched[index].update(
                {
                    "cell_sample_count": cell.sample_count,
                    "cell_repeat_count": cell.repeat_count,
                    "cell_covered": int(cell.covered),
                    "cell_original_accel_mps2": cell.original_accel_mps2,
                    "cell_measured_accel_mps2": (
                        cell.measured_accel_mps2
                        if cell.measured_accel_mps2 is not None
                        else ""
                    ),
                    "cell_candidate_accel_mps2": cell.candidate_accel_mps2,
                    "cell_modified": int(cell.modified),
                    "cell_fit_clipped": int(cell.fit_clipped),
                    "cell_rejected_reason": cell.rejected_reason,
                }
            )
    return candidates, cells


def _atomic_write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        Path(temporary_name).unlink(missing_ok=True)
        raise


def write_enriched_csv(
    path: Path,
    raw_fieldnames: list[str],
    rows: list[dict[str, str]],
    enriched: list[dict[str, object]],
) -> None:
    fieldnames = raw_fieldnames + [name for name in ENRICHED_COLUMNS if name not in raw_fieldnames]
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        newline="",
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=str(path.parent),
        delete=False,
    ) as stream:
        temporary = Path(stream.name)
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row, extra in zip(rows, enriched):
            writer.writerow({**row, **extra})
        stream.flush()
        os.fsync(stream.fileno())
    try:
        os.replace(temporary, path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def _format_value(value: float) -> str:
    return f"{value:.9g}"


def write_candidate_map(path: Path, table: MapTable, candidate: np.ndarray) -> None:
    rows = [[table.corner_label, *table.header_tokens]]
    for pedal_index, pedal_token in enumerate(table.row_label_tokens):
        values: list[str] = []
        for velocity_index in range(len(table.velocities)):
            original = table.values[pedal_index, velocity_index]
            fitted = candidate[pedal_index, velocity_index]
            if fitted == original:
                values.append(table.value_tokens[pedal_index][velocity_index])
            else:
                values.append(_format_value(float(fitted)))
        rows.append([pedal_token, *values])
    with path.open("x", newline="", encoding="utf-8") as stream:
        csv.writer(stream, lineterminator="\n").writerows(rows)


def build_candidate_converter_config(
    stock_config: Path,
    stock_steer_map: Path,
    candidate_dir: Path,
) -> tuple[str, dict[str, object]]:
    stock_config = stock_config.expanduser().resolve()
    stock_steer_map = stock_steer_map.expanduser().resolve()
    candidate_dir = candidate_dir.expanduser().resolve()
    if not stock_config.is_file():
        raise ValueError(f"stock converter config is not a file: {stock_config}")
    if not stock_steer_map.is_file():
        raise ValueError(f"stock steer map is not a file: {stock_steer_map}")

    stock_bytes = stock_config.read_bytes()
    try:
        payload = yaml.safe_load(stock_bytes)
        parameters = payload["/**"]["ros__parameters"]
    except (KeyError, TypeError, yaml.YAMLError) as error:
        raise ValueError(
            f"invalid raw vehicle command converter config: {stock_config}"
        ) from error
    if not isinstance(parameters, dict):
        raise ValueError(
            f"converter ros__parameters must be a mapping: {stock_config}"
        )
    missing = sorted(set(CONVERTER_MAP_KEYS.values()) - set(parameters))
    if missing:
        raise ValueError(
            "stock converter config is missing map parameters: " + ", ".join(missing)
        )

    candidate_files = {
        "accel_map": (candidate_dir / "accel_map.csv").resolve(),
        "brake_map": (candidate_dir / "brake_map.csv").resolve(),
        "converter_config": (
            candidate_dir / "raw_vehicle_cmd_converter.param.yaml"
        ).resolve(),
        "stock_steer_map": stock_steer_map,
    }
    parameters[CONVERTER_MAP_KEYS["accel"]] = str(candidate_files["accel_map"])
    parameters[CONVERTER_MAP_KEYS["brake"]] = str(candidate_files["brake_map"])
    parameters[CONVERTER_MAP_KEYS["steer"]] = str(stock_steer_map)
    content = (
        "# Generated by scripts/e2e/fit_validate_carla_actuation_map.py.\n"
        "# Non-map parameters are preserved from the stock CARLA config.\n"
        + yaml.safe_dump(payload, sort_keys=False)
    )
    metadata: dict[str, object] = {
        "stock_converter_config": str(stock_config),
        "stock_converter_config_sha256": hashlib.sha256(stock_bytes).hexdigest(),
        "stock_steer_map": str(stock_steer_map),
        "stock_steer_map_sha256": _sha256(stock_steer_map),
        "files": {name: str(path) for name, path in candidate_files.items()},
    }
    return content, metadata


def write_candidate_maps(
    candidate_dir: Path,
    accel_map: MapTable,
    brake_map: MapTable,
    candidates: dict[str, np.ndarray],
    converter_config_content: str,
) -> None:
    candidate_dir = candidate_dir.expanduser().resolve()
    if candidate_dir in {accel_map.path.parent, brake_map.path.parent}:
        raise ValueError("candidate directory must differ from the original map directory")
    if candidate_dir.exists():
        if not candidate_dir.is_dir():
            raise ValueError(f"candidate path is not a directory: {candidate_dir}")
        if any(candidate_dir.iterdir()):
            raise FileExistsError(f"candidate directory is not empty: {candidate_dir}")
    else:
        candidate_dir.mkdir(parents=True)
    write_candidate_map(candidate_dir / "accel_map.csv", accel_map, candidates["accel"])
    write_candidate_map(candidate_dir / "brake_map.csv", brake_map, candidates["brake"])
    _atomic_write_text(
        candidate_dir / "raw_vehicle_cmd_converter.param.yaml",
        converter_config_content,
    )


def validate_candidate_destination(
    candidate_dir: Path,
    output_dir: Path,
    accel_map: MapTable,
    brake_map: MapTable,
) -> Path:
    candidate_dir = candidate_dir.expanduser().resolve()
    source_dirs = {accel_map.path.parent, brake_map.path.parent}
    if any(source == candidate_dir or source in candidate_dir.parents for source in source_dirs):
        raise ValueError("candidate directory must be outside the original map directory")
    if candidate_dir == output_dir or candidate_dir in output_dir.parents:
        raise ValueError("analysis output directory cannot be inside the candidate directory")
    if candidate_dir.exists():
        if not candidate_dir.is_dir():
            raise ValueError(f"candidate path is not a directory: {candidate_dir}")
        if any(candidate_dir.iterdir()):
            raise FileExistsError(f"candidate directory is not empty: {candidate_dir}")
    return candidate_dir


def _save_figure(fig: plt.Figure, path: Path) -> None:
    with tempfile.NamedTemporaryFile(
        prefix=f".{path.stem}.", suffix=path.suffix, dir=str(path.parent), delete=False
    ) as stream:
        temporary = Path(stream.name)
    try:
        fig.savefig(temporary, dpi=150, bbox_inches="tight")
        os.replace(temporary, path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    finally:
        plt.close(fig)


def write_response_plot(
    path: Path,
    tables: dict[str, MapTable],
    candidates: dict[str, np.ndarray],
    cells: dict[tuple[str, int, int], CellFit],
    analysis_status: str,
    invalid_reasons: Sequence[str],
) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    for axis, kind in zip(axes, ("accel", "brake")):
        table = tables[kind]
        image = axis.imshow(
            candidates[kind],
            origin="lower",
            aspect="auto",
            cmap="coolwarm",
            extent=(-0.5, len(table.velocities) - 0.5, -0.5, len(table.pedals) - 0.5),
        )
        for pedal_index in range(len(table.pedals)):
            for velocity_index in range(len(table.velocities)):
                cell = cells[(kind, pedal_index, velocity_index)]
                if cell.modified:
                    axis.scatter(
                        velocity_index,
                        pedal_index,
                        marker="s",
                        s=80,
                        facecolors="none",
                        edgecolors="black",
                        linewidths=1.3,
                    )
        axis.set_title(f"{kind.capitalize()} candidate response")
        axis.set_xlabel("Velocity [m/s]")
        axis.set_ylabel("Pedal")
        axis.set_xticks(range(len(table.velocities)))
        axis.set_xticklabels([f"{value:g}" for value in table.velocities], rotation=45)
        axis.set_yticks(range(len(table.pedals)))
        axis.set_yticklabels([f"{value:g}" for value in table.pedals])
        fig.colorbar(image, ax=axis, label="Acceleration [m/s^2]")
    title = "CARLA actuation response (outlined cells were modified)"
    if analysis_status == "invalid":
        title += f"\nINVALID ANALYSIS: {', '.join(invalid_reasons)}"
    fig.suptitle(title, color="crimson" if analysis_status == "invalid" else "black")
    _save_figure(fig, path)


def write_coverage_plot(
    path: Path,
    tables: dict[str, MapTable],
    cells: dict[tuple[str, int, int], CellFit],
    analysis_status: str,
    invalid_reasons: Sequence[str],
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    for column, kind in enumerate(("accel", "brake")):
        table = tables[kind]
        for row_index, metric in enumerate(("sample_count", "repeat_count")):
            matrix = np.asarray(
                [
                    [
                        getattr(cells[(kind, pedal_index, velocity_index)], metric)
                        for velocity_index in range(len(table.velocities))
                    ]
                    for pedal_index in range(len(table.pedals))
                ],
                dtype=float,
            )
            axis = axes[row_index, column]
            image = axis.imshow(matrix, origin="lower", aspect="auto", cmap="viridis")
            axis.set_title(f"{kind.capitalize()} {metric.replace('_', ' ')}")
            axis.set_xlabel("Velocity [m/s]")
            axis.set_ylabel("Pedal")
            axis.set_xticks(range(len(table.velocities)))
            axis.set_xticklabels(
                [f"{value:g}" for value in table.velocities], rotation=45
            )
            axis.set_yticks(range(len(table.pedals)))
            axis.set_yticklabels([f"{value:g}" for value in table.pedals])
            fig.colorbar(image, ax=axis)
    title = "CARLA actuation sweep coverage"
    if analysis_status == "invalid":
        title += f"\nINVALID ANALYSIS: {', '.join(invalid_reasons)}"
    fig.suptitle(title, color="crimson" if analysis_status == "invalid" else "black")
    _save_figure(fig, path)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(
    raw_csv: Path,
    output_dir: Path,
    original_accel_map: Path,
    original_brake_map: Path,
    sweep_summary: Path | None = None,
    candidate_dir: Path | None = None,
    min_samples: int = 30,
    min_repeats: int = 3,
    fit_source: str = "derived",
    max_cell_change: float = 0.75,
    strict_monotonic_margin: float = 1.0e-6,
    stock_converter_config: Path = DEFAULT_CONVERTER_CONFIG,
    stock_steer_map: Path = DEFAULT_STEER_MAP,
) -> dict[str, object]:
    if min_samples <= 0 or min_repeats <= 0:
        raise ValueError("minimum samples and repeats must be positive")
    if not math.isfinite(max_cell_change) or max_cell_change <= 0.0:
        raise ValueError("max cell change must be positive and finite")
    if not math.isfinite(strict_monotonic_margin) or strict_monotonic_margin <= 0.0:
        raise ValueError("strict monotonic margin must be positive and finite")
    if fit_source not in {"derived", "imu"}:
        raise ValueError("fit source must be 'derived' or 'imu'")

    raw_csv = raw_csv.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    sweep_summary_path = (
        sweep_summary.expanduser().resolve() if sweep_summary is not None else None
    )
    sweep_payload = (
        read_sweep_summary(sweep_summary_path)
        if sweep_summary_path is not None
        else None
    )
    accel_map = read_map(original_accel_map)
    brake_map = read_map(original_brake_map)
    validate_source_maps(accel_map, brake_map)
    source_dirs = {accel_map.path.parent, brake_map.path.parent}
    if any(source == output_dir or source in output_dir.parents for source in source_dirs):
        raise ValueError("analysis output directory must be outside the original map directory")
    if candidate_dir is not None:
        candidate_dir = validate_candidate_destination(
            candidate_dir, output_dir, accel_map, brake_map
        )
    original_hashes = {"accel": _sha256(accel_map.path), "brake": _sha256(brake_map.path)}
    raw_fields, rows = read_raw_csv(raw_csv)
    derived = derive_acceleration(rows)
    enriched, assignments = classify_rows(
        rows, derived, accel_map, brake_map, fit_source
    )
    candidates, cells = fit_maps(
        rows,
        enriched,
        assignments,
        accel_map,
        brake_map,
        min_samples,
        min_repeats,
        max_cell_change,
        strict_monotonic_margin,
    )
    if not np.array_equal(candidates["accel"][0], candidates["brake"][0]):
        raise AssertionError("candidate maps do not share an identical zero-pedal row")
    if np.any(np.diff(candidates["accel"], axis=0) <= 0.0) or np.any(
        np.diff(candidates["brake"], axis=0) >= 0.0
    ):
        raise AssertionError("candidate fitting violated pedal monotonicity")

    metrics = {
        "raw_rows": len(rows),
        "fit_eligible_rows": sum(
            int(extra["fit_row_eligible"]) for extra in enriched
        ),
        "covered_cells": sum(cell.covered for cell in cells.values()),
        "modified_cells": sum(cell.modified for cell in cells.values()),
        "clipped_cells": sum(cell.fit_clipped for cell in cells.values()),
        "shared_zero_covered_velocity_cells": sum(
            cells[("accel", 0, velocity_index)].covered
            for velocity_index in range(len(accel_map.velocities))
        ),
        "shared_zero_rejected_velocity_cells": sum(
            bool(cells[("accel", 0, velocity_index)].rejected_reason)
            for velocity_index in range(len(accel_map.velocities))
        ),
    }
    invalid_reasons: list[str] = []
    if sweep_payload is not None:
        sweep_status = str(sweep_payload.get("status", "")).strip().lower()
        if not sweep_status:
            invalid_reasons.append("sweep_status_missing")
        elif sweep_status != "completed":
            invalid_reasons.append("sweep_not_completed")
        rows_written = sweep_payload.get("rows_written")
        if isinstance(rows_written, int) and rows_written != len(rows):
            invalid_reasons.append("sweep_row_count_mismatch")
    if metrics["fit_eligible_rows"] == 0:
        invalid_reasons.append("no_fit_eligible_rows")
    if metrics["covered_cells"] == 0:
        invalid_reasons.append("no_covered_cells")
    analysis_status = "invalid" if invalid_reasons else "valid"
    invalid_reason_text = ";".join(invalid_reasons)
    for extra in enriched:
        extra["analysis_status"] = analysis_status
        extra["analysis_invalid_reasons"] = invalid_reason_text
    candidate_written = candidate_dir is not None and analysis_status == "valid"
    converter_config_content = None
    converter_config_metadata = None
    if candidate_written:
        assert candidate_dir is not None
        converter_config_content, converter_config_metadata = build_candidate_converter_config(
            stock_converter_config, stock_steer_map, candidate_dir
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    response_csv = output_dir / "actuation_response.csv"
    response_json = output_dir / "actuation_response.json"
    response_png = output_dir / "actuation_response.png"
    coverage_png = output_dir / "actuation_coverage.png"
    write_enriched_csv(response_csv, raw_fields, rows, enriched)

    cell_records = [
        cells[key].as_dict()
        for key in sorted(cells, key=lambda item: (item[0], item[1], item[2]))
    ]
    summary: dict[str, object] = {
        "schema_version": 1,
        "status": analysis_status,
        "invalid_reasons": invalid_reasons,
        "input": {
            "raw_csv": str(raw_csv),
            "raw_csv_sha256": _sha256(raw_csv),
            "sweep_summary": (
                str(sweep_summary_path) if sweep_summary_path is not None else None
            ),
            "sweep_summary_sha256": (
                _sha256(sweep_summary_path)
                if sweep_summary_path is not None
                else None
            ),
            "original_accel_map": str(accel_map.path),
            "original_accel_map_sha256": original_hashes["accel"],
            "original_brake_map": str(brake_map.path),
            "original_brake_map_sha256": original_hashes["brake"],
        },
        "settings": {
            "fit_source": fit_source,
            "min_samples": min_samples,
            "min_repeats": min_repeats,
            "max_cell_change_mps2": max_cell_change,
            "strict_monotonic_margin_mps2": strict_monotonic_margin,
        },
        "sweep": sweep_payload,
        "summary": metrics,
        "candidate_dir": str(candidate_dir) if candidate_written else None,
        "candidate_requested_dir": str(candidate_dir) if candidate_dir else None,
        "candidate_written": candidate_written,
        "candidate_converter_config": converter_config_metadata,
        "cells": cell_records,
        "artifacts": {
            "response_csv": str(response_csv),
            "response_json": str(response_json),
            "response_png": str(response_png),
            "coverage_png": str(coverage_png),
        },
    }
    _atomic_write_text(response_json, json.dumps(summary, indent=2, sort_keys=True) + "\n")
    tables = {"accel": accel_map, "brake": brake_map}
    write_response_plot(
        response_png,
        tables,
        candidates,
        cells,
        analysis_status,
        invalid_reasons,
    )
    write_coverage_plot(
        coverage_png,
        tables,
        cells,
        analysis_status,
        invalid_reasons,
    )
    if candidate_written:
        assert candidate_dir is not None
        assert converter_config_content is not None
        write_candidate_maps(
            candidate_dir,
            accel_map,
            brake_map,
            candidates,
            converter_config_content,
        )

    if _sha256(accel_map.path) != original_hashes["accel"] or _sha256(
        brake_map.path
    ) != original_hashes["brake"]:
        raise AssertionError("an original map changed while writing candidates")
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        summary = run(
            raw_csv=args.raw_csv,
            output_dir=args.output_dir,
            original_accel_map=args.original_accel_map,
            original_brake_map=args.original_brake_map,
            sweep_summary=args.sweep_summary,
            candidate_dir=args.candidate_dir,
            min_samples=args.min_samples,
            min_repeats=args.min_repeats,
            fit_source=args.fit_source,
            max_cell_change=args.max_cell_change,
            strict_monotonic_margin=args.strict_monotonic_margin,
            stock_converter_config=args.stock_converter_config,
            stock_steer_map=args.stock_steer_map,
        )
    except (OSError, ValueError, csv.Error) as error:
        print(f"Actuation analysis input error: {error}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {"status": summary["status"], **summary["summary"]},
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if summary["status"] == "valid" else 3


if __name__ == "__main__":
    raise SystemExit(main())
