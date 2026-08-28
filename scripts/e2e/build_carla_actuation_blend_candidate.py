#!/usr/bin/env python3
"""Build a guarded accel-only blend of two CARLA actuation maps."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
from typing import Any

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
FIT_HELPER_PATH = Path(__file__).with_name("fit_validate_carla_actuation_map.py")
DEFAULT_CARLA_DIR = (
    ROOT
    / "src/universe/autoware_universe/simulator/autoware_carla_interface"
)
DEFAULT_STOCK_MAP_DIR = DEFAULT_CARLA_DIR / "calibration_maps"
DEFAULT_FULL_CANDIDATE_DIR = (
    ROOT
    / "artifacts/calibration/carla_actuation/2026-08-25_phase1_baseline03/candidate"
)
MAP_KEYS = {
    "accel": "csv_path_accel_map",
    "brake": "csv_path_brake_map",
    "steer": "csv_path_steer_map",
}


def _load_fit_helper() -> Any:
    spec = importlib.util.spec_from_file_location("fit_validate_carla_actuation_map", FIT_HELPER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load fit helper: {FIT_HELPER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


FIT = _load_fit_helper()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _file_record(path: Path) -> dict[str, object]:
    resolved = path.resolve()
    return {
        "path": str(resolved),
        "sha256": _sha256(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _parameters(path: Path) -> dict[str, object]:
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
        parameters = payload["/**"]["ros__parameters"]
    except (OSError, UnicodeError, yaml.YAMLError, KeyError, TypeError) as error:
        raise ValueError(f"invalid converter config: {path}") from error
    if not isinstance(parameters, dict):
        raise ValueError(f"converter parameters must be a mapping: {path}")
    return parameters


def _non_map_parameters(path: Path) -> dict[str, object]:
    parameters = dict(_parameters(path))
    for key in MAP_KEYS.values():
        parameters.pop(key, None)
    return parameters


def _validate_compatible_axes(stock: Any, candidate: Any) -> None:
    if stock.values.shape != candidate.values.shape:
        raise ValueError("stock and candidate accel maps have different shapes")
    if not np.array_equal(stock.velocities, candidate.velocities):
        raise ValueError("stock and candidate accel maps have different velocity axes")
    if not np.array_equal(stock.pedals, candidate.pedals):
        raise ValueError("stock and candidate accel maps have different pedal axes")
    if not np.all(np.isfinite(candidate.values)):
        raise ValueError("candidate accel map contains a non-finite value")
    if np.any(np.diff(candidate.values, axis=0) <= 0.0):
        raise ValueError("candidate accel map is not strictly increasing by pedal")


def build_candidate(
    *,
    output_dir: Path,
    alpha: float,
    stock_config: Path,
    stock_accel: Path,
    stock_brake: Path,
    stock_steer: Path,
    full_candidate_accel: Path,
    source_analysis: Path | None = None,
) -> dict[str, object]:
    if not 0.0 < alpha < 1.0:
        raise ValueError("alpha must be strictly between zero and one")
    output_dir = output_dir.expanduser().resolve()
    inputs = [stock_config, stock_accel, stock_brake, stock_steer, full_candidate_accel]
    if source_analysis is not None:
        inputs.append(source_analysis)
    inputs = [path.expanduser().resolve() for path in inputs]
    if any(not path.is_file() for path in inputs):
        missing = [str(path) for path in inputs if not path.is_file()]
        raise FileNotFoundError("input files do not exist: " + ", ".join(missing))
    if output_dir.exists():
        raise FileExistsError(f"output directory already exists: {output_dir}")

    stock_accel_table = FIT.read_map(stock_accel)
    stock_brake_table = FIT.read_map(stock_brake)
    full_candidate_table = FIT.read_map(full_candidate_accel)
    FIT.validate_source_maps(stock_accel_table, stock_brake_table)
    _validate_compatible_axes(stock_accel_table, full_candidate_table)

    blended = stock_accel_table.values.copy()
    blended[1:, :] = stock_accel_table.values[1:, :] + alpha * (
        full_candidate_table.values[1:, :] - stock_accel_table.values[1:, :]
    )
    if not np.all(np.isfinite(blended)):
        raise ValueError("blended accel map contains a non-finite value")
    if np.any(np.diff(blended, axis=0) <= 0.0):
        raise ValueError("blended accel map is not strictly increasing by pedal")

    source_hashes = {path: _sha256(path) for path in inputs}
    output_dir.mkdir(parents=True)
    try:
        accel_output = output_dir / "accel_map.csv"
        brake_output = output_dir / "brake_map.csv"
        steer_output = output_dir / "steer_map.csv"
        config_output = output_dir / "raw_vehicle_cmd_converter.param.yaml"
        manifest_output = output_dir / "blend_manifest.json"

        FIT.write_candidate_map(accel_output, stock_accel_table, blended)
        shutil.copyfile(stock_brake, brake_output)
        shutil.copyfile(stock_steer, steer_output)
        config_content, _ = FIT.build_candidate_converter_config(
            stock_config, steer_output, output_dir
        )
        config_content = config_content.replace(
            "Generated by scripts/e2e/fit_validate_carla_actuation_map.py.",
            "Generated by scripts/e2e/build_carla_actuation_blend_candidate.py.",
            1,
        )
        FIT._atomic_write_text(config_output, config_content)

        output_accel_table = FIT.read_map(accel_output)
        output_brake_table = FIT.read_map(brake_output)
        FIT.validate_source_maps(output_accel_table, output_brake_table)
        expected = stock_accel_table.values.copy()
        expected[1:, :] = stock_accel_table.values[1:, :] + alpha * (
            full_candidate_table.values[1:, :] - stock_accel_table.values[1:, :]
        )
        formula_error = float(np.max(np.abs(output_accel_table.values - expected)))
        minimum_gap = float(np.min(np.diff(output_accel_table.values, axis=0)))
        config_parameters = _parameters(config_output)
        expected_references = {
            MAP_KEYS["accel"]: str(accel_output),
            MAP_KEYS["brake"]: str(brake_output),
            MAP_KEYS["steer"]: str(steer_output),
        }
        references_match = all(
            config_parameters.get(key) == value for key, value in expected_references.items()
        )
        non_map_parameters_match = _non_map_parameters(config_output) == _non_map_parameters(
            stock_config
        )
        validation = {
            "status": "valid",
            "shape": list(output_accel_table.values.shape),
            "axes_match_stock": bool(
                np.array_equal(output_accel_table.velocities, stock_accel_table.velocities)
                and np.array_equal(output_accel_table.pedals, stock_accel_table.pedals)
            ),
            "all_values_finite": bool(np.all(np.isfinite(output_accel_table.values))),
            "accel_strictly_increasing_by_pedal": bool(
                np.all(np.diff(output_accel_table.values, axis=0) > 0.0)
            ),
            "brake_strictly_decreasing_by_pedal": bool(
                np.all(np.diff(output_brake_table.values, axis=0) < 0.0)
            ),
            "shared_zero_pedal_row_matches": bool(
                np.array_equal(output_accel_table.values[0], output_brake_table.values[0])
            ),
            "zero_pedal_row_matches_stock": bool(
                np.array_equal(output_accel_table.values[0], stock_accel_table.values[0])
            ),
            "formula_max_abs_error_after_serialization_mps2": formula_error,
            "minimum_accel_pedal_gap_mps2": minimum_gap,
            "brake_byte_identical_to_stock": _sha256(brake_output) == _sha256(stock_brake),
            "steer_byte_identical_to_stock": _sha256(steer_output) == _sha256(stock_steer),
            "converter_non_map_parameters_match_stock": non_map_parameters_match,
            "converter_map_references_match_outputs": references_match,
        }
        if formula_error > 5.0e-9 or not all(
            value
            for key, value in validation.items()
            if key
            in {
                "axes_match_stock",
                "all_values_finite",
                "accel_strictly_increasing_by_pedal",
                "brake_strictly_decreasing_by_pedal",
                "shared_zero_pedal_row_matches",
                "zero_pedal_row_matches_stock",
                "brake_byte_identical_to_stock",
                "steer_byte_identical_to_stock",
                "converter_non_map_parameters_match_stock",
                "converter_map_references_match_outputs",
            }
        ):
            raise AssertionError(f"generated candidate failed validation: {validation}")

        manifest: dict[str, object] = {
            "schema_version": 1,
            "status": "valid",
            "kind": "linear_accel_blend",
            "blend": {
                "candidate_weight": alpha,
                "stock_weight": 1.0 - alpha,
                "formula": "stock + candidate_weight * (full_candidate - stock)",
                "scope": "nonzero_accel_pedal_rows",
                "shared_zero_pedal_policy": (
                    "stock; preserves the byte-identical stock brake-map boundary"
                ),
            },
            "generator": _file_record(Path(__file__)),
            "inputs": {
                "stock_converter_config": _file_record(stock_config),
                "stock_accel_map": _file_record(stock_accel),
                "stock_brake_map": _file_record(stock_brake),
                "stock_steer_map": _file_record(stock_steer),
                "full_candidate_accel_map": _file_record(full_candidate_accel),
                "source_calibration_analysis": (
                    _file_record(source_analysis) if source_analysis is not None else None
                ),
            },
            "outputs": {
                "converter_config": _file_record(config_output),
                "accel_map": _file_record(accel_output),
                "brake_map": _file_record(brake_output),
                "steer_map": _file_record(steer_output),
            },
            "validation": validation,
        }
        FIT._atomic_write_text(
            manifest_output, json.dumps(manifest, indent=2, sort_keys=True) + "\n"
        )
    except BaseException:
        shutil.rmtree(output_dir, ignore_errors=True)
        raise

    if any(_sha256(path) != source_hashes[path] for path in inputs):
        raise AssertionError("an input file changed while generating the blend candidate")
    return manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--alpha", type=float, default=0.4)
    parser.add_argument(
        "--stock-config",
        type=Path,
        default=DEFAULT_CARLA_DIR / "config/raw_vehicle_cmd_converter.param.yaml",
    )
    parser.add_argument(
        "--stock-accel", type=Path, default=DEFAULT_STOCK_MAP_DIR / "accel_map.csv"
    )
    parser.add_argument(
        "--stock-brake", type=Path, default=DEFAULT_STOCK_MAP_DIR / "brake_map.csv"
    )
    parser.add_argument(
        "--stock-steer", type=Path, default=DEFAULT_STOCK_MAP_DIR / "steer_map.csv"
    )
    parser.add_argument(
        "--full-candidate-accel",
        type=Path,
        default=DEFAULT_FULL_CANDIDATE_DIR / "accel_map.csv",
    )
    parser.add_argument(
        "--source-analysis",
        type=Path,
        default=DEFAULT_FULL_CANDIDATE_DIR.parent / "analysis/actuation_response.json",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        manifest = build_candidate(
            output_dir=args.output_dir,
            alpha=args.alpha,
            stock_config=args.stock_config,
            stock_accel=args.stock_accel,
            stock_brake=args.stock_brake,
            stock_steer=args.stock_steer,
            full_candidate_accel=args.full_candidate_accel,
            source_analysis=args.source_analysis,
        )
    except (AssertionError, FileExistsError, FileNotFoundError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
