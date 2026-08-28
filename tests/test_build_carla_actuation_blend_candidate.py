from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml


MODULE_PATH = (
    Path(__file__).parents[1]
    / "scripts/e2e/build_carla_actuation_blend_candidate.py"
)
SPEC = importlib.util.spec_from_file_location(
    "build_carla_actuation_blend_candidate", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
blend = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = blend
SPEC.loader.exec_module(blend)


def _write_map(path: Path, rows: list[str]) -> None:
    path.write_text("default,0,2\n" + "\n".join(rows) + "\n", encoding="utf-8")


def _inputs(tmp_path: Path) -> dict[str, Path]:
    stock_accel = tmp_path / "stock_accel.csv"
    stock_brake = tmp_path / "stock_brake.csv"
    stock_steer = tmp_path / "stock_steer.csv"
    full_accel = tmp_path / "full_accel.csv"
    config = tmp_path / "stock.param.yaml"
    analysis = tmp_path / "analysis.json"
    _write_map(stock_accel, ["0,-0.2,-0.4", "0.5,1.0,0.8", "1.0,2.0,1.8"])
    _write_map(stock_brake, ["0,-0.2,-0.4", "0.5,-1.0,-1.2", "1.0,-2.0,-2.2"])
    _write_map(full_accel, ["0,-0.1,-0.2", "0.5,0.6,0.4", "1.0,1.4,1.2"])
    stock_steer.write_bytes(b"stock steer bytes\n")
    config.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        f"    csv_path_accel_map: {stock_accel}\n"
        f"    csv_path_brake_map: {stock_brake}\n"
        f"    csv_path_steer_map: {stock_steer}\n"
        "    convert_accel_cmd: true\n"
        "    max_throttle: 0.4\n",
        encoding="utf-8",
    )
    analysis.write_text("{}\n", encoding="utf-8")
    return {
        "stock_config": config,
        "stock_accel": stock_accel,
        "stock_brake": stock_brake,
        "stock_steer": stock_steer,
        "full_candidate_accel": full_accel,
        "source_analysis": analysis,
    }


def test_builds_positive_pedal_blend_and_preserves_stock_boundaries(tmp_path):
    inputs = _inputs(tmp_path)
    output = tmp_path / "candidate"
    manifest = blend.build_candidate(output_dir=output, alpha=0.4, **inputs)

    stock = blend.FIT.read_map(inputs["stock_accel"])
    full = blend.FIT.read_map(inputs["full_candidate_accel"])
    actual = blend.FIT.read_map(output / "accel_map.csv")
    assert np.array_equal(actual.values[0], stock.values[0])
    assert actual.values[1:] == pytest.approx(
        stock.values[1:] + 0.4 * (full.values[1:] - stock.values[1:])
    )
    assert (output / "brake_map.csv").read_bytes() == inputs["stock_brake"].read_bytes()
    assert (output / "steer_map.csv").read_bytes() == inputs["stock_steer"].read_bytes()
    parameters = yaml.safe_load(
        (output / "raw_vehicle_cmd_converter.param.yaml").read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    assert parameters["csv_path_accel_map"] == str(output / "accel_map.csv")
    assert parameters["csv_path_brake_map"] == str(output / "brake_map.csv")
    assert parameters["csv_path_steer_map"] == str(output / "steer_map.csv")
    assert parameters["max_throttle"] == 0.4
    assert manifest["status"] == "valid"
    assert manifest["validation"]["shared_zero_pedal_row_matches"] is True


def test_rejects_existing_destination(tmp_path):
    inputs = _inputs(tmp_path)
    output = tmp_path / "candidate"
    output.mkdir()
    with pytest.raises(FileExistsError):
        blend.build_candidate(output_dir=output, alpha=0.4, **inputs)
