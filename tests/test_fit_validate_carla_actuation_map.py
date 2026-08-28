import csv
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml


MODULE_PATH = (
    Path(__file__).parents[1] / "scripts/e2e/fit_validate_carla_actuation_map.py"
)
SPEC = importlib.util.spec_from_file_location("fit_validate_carla_actuation_map", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


RAW_FIELDS = [
    "sim_time_s",
    "wall_time_s",
    "phase",
    "case_id",
    "case_kind",
    "repeat",
    "pedal_level",
    "requested_accel",
    "requested_brake",
    "applied_accel",
    "applied_brake",
    "velocity_mps",
    "steering_rad",
    "imu_accel_x_mps2",
    "pose_x_m",
    "pose_y_m",
    "status_age_wall_s",
    "watchdog_active",
    "safety_ok",
    "fit_eligible",
]


def write_sources(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    accel = source / "accel_map.csv"
    brake = source / "brake_map.csv"
    accel.write_text(
        "default,0,1\n"
        "0,0,-0.2\n"
        "0.1,0.5,0.3\n"
        "0.2,1,0.8\n",
        encoding="utf-8",
    )
    brake.write_text(
        "default,0,1\n"
        "0,0,-0.2\n"
        "0.1,-0.5,-0.7\n"
        "0.2,-1,-1.2\n",
        encoding="utf-8",
    )
    return accel, brake


def sample_rows():
    rows = []
    # Two independent repeats cover accel pedal 0.1 at the zero-speed knot.
    for repeat in (1, 2):
        for step in range(5):
            time_s = step * 0.1
            rows.append(
                {
                    "sim_time_s": time_s,
                    "wall_time_s": 100 + time_s,
                    "phase": "sample",
                    "case_id": f"accel-01-r{repeat}",
                    "case_kind": "accel",
                    "repeat": repeat,
                    "pedal_level": 0.1,
                    "requested_accel": 0.1,
                    "requested_brake": 0,
                    "applied_accel": 0.1,
                    "applied_brake": 0,
                    "velocity_mps": 0.08 * step,
                    "steering_rad": 0,
                    "imu_accel_x_mps2": 0.8,
                    "pose_x_m": 0,
                    "pose_y_m": 0,
                    "status_age_wall_s": 0.01,
                    "watchdog_active": 0,
                    "safety_ok": 1,
                    "fit_eligible": 1,
                }
            )
    # This adjacent cell has enough samples but only one repeat, so it must stay exact.
    for step in range(5):
        rows.append(
            {
                "sim_time_s": step * 0.1,
                "wall_time_s": 200 + step * 0.1,
                "phase": "sample",
                "case_id": "accel-02-r1",
                "case_kind": "accel",
                "repeat": 1,
                "pedal_level": 0.2,
                "requested_accel": 0.2,
                "requested_brake": 0,
                "applied_accel": 0.2,
                "applied_brake": 0,
                "velocity_mps": 0.2 * step,
                "steering_rad": 0,
                "imu_accel_x_mps2": 2.0,
                "pose_x_m": 0,
                "pose_y_m": 0,
                "status_age_wall_s": 0.01,
                "watchdog_active": 0,
                "safety_ok": 1,
                "fit_eligible": 1,
            }
        )
    return rows


def write_raw(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=RAW_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def startup_row():
    row = {field: "" for field in RAW_FIELDS}
    row.update(
        {
            "sim_time_s": 0.05,
            "wall_time_s": 1.0,
            "phase": "wait_status",
            "requested_accel": 0,
            "requested_brake": 1,
            "status_age_wall_s": "inf",
            "watchdog_active": 1,
            "safety_ok": 1,
            "fit_eligible": 0,
        }
    )
    return row


def write_sweep_summary(path, *, status, reason, rows_written):
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "status": status,
                "reason": reason,
                "rows_written": rows_written,
            }
        ),
        encoding="utf-8",
    )


def read_values(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.reader(stream))
    return np.asarray([[float(value) for value in row[1:]] for row in rows[1:]])


def test_fit_modifies_only_covered_cells_and_writes_artifacts(tmp_path):
    accel, brake = write_sources(tmp_path)
    raw = tmp_path / "raw.csv"
    write_raw(raw, sample_rows())
    original_accel = accel.read_bytes()
    original_brake = brake.read_bytes()
    output = tmp_path / "analysis"
    candidate = tmp_path / "candidate"

    summary = MODULE.run(
        raw,
        output,
        accel,
        brake,
        candidate_dir=candidate,
        min_samples=5,
        min_repeats=2,
        fit_source="derived",
        max_cell_change=0.75,
    )

    source_accel = read_values(accel)
    fitted_accel = read_values(candidate / "accel_map.csv")
    source_brake = read_values(brake)
    fitted_brake = read_values(candidate / "brake_map.csv")
    assert fitted_accel[1, 0] == pytest.approx(0.8)
    assert fitted_accel[2, 0] == source_accel[2, 0]
    assert np.array_equal(fitted_accel[0], source_accel[0])
    assert np.array_equal(fitted_brake, source_brake)
    assert np.all(np.diff(fitted_accel, axis=0) > 0)
    assert np.all(np.diff(fitted_brake, axis=0) < 0)
    assert accel.read_bytes() == original_accel
    assert brake.read_bytes() == original_brake

    for name in (
        "actuation_response.csv",
        "actuation_response.json",
        "actuation_response.png",
        "actuation_coverage.png",
    ):
        assert (output / name).stat().st_size > 0
    with (output / "actuation_response.csv").open(newline="", encoding="utf-8") as stream:
        enriched = list(csv.DictReader(stream))
    assert "derived_accel_mps2" in enriched[0]
    assert float(enriched[0]["derived_accel_mps2"]) == pytest.approx(0.8)
    assert enriched[0]["cell_covered"] == "1"
    payload = json.loads((output / "actuation_response.json").read_text())
    assert payload["summary"] == summary["summary"]
    assert payload["summary"]["covered_cells"] == 1
    assert payload["summary"]["modified_cells"] == 1
    candidate_config = candidate / "raw_vehicle_cmd_converter.param.yaml"
    assert candidate_config.is_file()
    candidate_params = yaml.safe_load(candidate_config.read_text())["/**"]["ros__parameters"]
    stock_params = yaml.safe_load(MODULE.DEFAULT_CONVERTER_CONFIG.read_text())["/**"][
        "ros__parameters"
    ]
    assert candidate_params["csv_path_accel_map"] == str(
        (candidate / "accel_map.csv").resolve()
    )
    assert candidate_params["csv_path_brake_map"] == str(
        (candidate / "brake_map.csv").resolve()
    )
    assert candidate_params["csv_path_steer_map"] == str(MODULE.DEFAULT_STEER_MAP.resolve())
    for key, value in stock_params.items():
        if key not in MODULE.CONVERTER_MAP_KEYS.values():
            assert candidate_params[key] == value
    assert payload["candidate_converter_config"]["stock_converter_config_sha256"]


def test_bounded_fit_clips_before_uncovered_neighbor_and_remains_strict(tmp_path):
    accel, brake = write_sources(tmp_path)
    raw = tmp_path / "raw.csv"
    rows = sample_rows()
    for row in rows[:10]:
        row["velocity_mps"] = 10.0 * float(row["sim_time_s"])
        row["imu_accel_x_mps2"] = 10.0
    write_raw(raw, rows[:10])

    MODULE.run(
        raw,
        tmp_path / "analysis",
        accel,
        brake,
        candidate_dir=tmp_path / "candidate",
        min_samples=5,
        min_repeats=2,
        fit_source="imu",
        max_cell_change=5.0,
    )

    fitted = read_values(tmp_path / "candidate/accel_map.csv")
    # The measured target is 10, but the untouched 0.2-pedal row is the hard bound.
    assert fitted[1, 0] < fitted[2, 0]
    assert fitted[2, 0] == 1.0
    assert np.all(np.diff(fitted, axis=0) > 0)


def test_covered_coast_cell_updates_both_shared_rows_identically(tmp_path):
    accel, brake = write_sources(tmp_path)
    raw = tmp_path / "raw.csv"
    rows = []
    for repeat in (1, 2):
        for step in range(5):
            rows.append(
                {
                    **sample_rows()[0],
                    "sim_time_s": step * 0.1,
                    "case_id": f"coast-r{repeat}",
                    "case_kind": "coast",
                    "repeat": repeat,
                    "pedal_level": 0,
                    "requested_accel": 0,
                    "applied_accel": 0,
                    "velocity_mps": 0.02 * step,
                    "imu_accel_x_mps2": 0.2,
                }
            )
    write_raw(raw, rows)

    MODULE.run(
        raw,
        tmp_path / "analysis",
        accel,
        brake,
        candidate_dir=tmp_path / "candidate",
        min_samples=5,
        min_repeats=2,
    )

    fitted_accel = read_values(tmp_path / "candidate/accel_map.csv")
    fitted_brake = read_values(tmp_path / "candidate/brake_map.csv")
    assert fitted_accel[0, 0] == pytest.approx(0.2)
    assert fitted_brake[0, 0] == pytest.approx(0.2)
    assert np.array_equal(fitted_accel[0], fitted_brake[0])
    assert np.all(np.diff(fitted_accel, axis=0) > 0)
    assert np.all(np.diff(fitted_brake, axis=0) < 0)


def test_joint_coast_and_brake_fit_can_cross_original_brake_neighbor(tmp_path):
    accel, brake = write_sources(tmp_path)
    raw = tmp_path / "raw.csv"
    rows = []
    template = sample_rows()[0]
    for repeat in (1, 2):
        for step in range(5):
            common = {
                **template,
                "sim_time_s": step * 0.1,
                "repeat": repeat,
                "velocity_mps": 0.2,
            }
            rows.append(
                {
                    **common,
                    "case_id": f"coast-r{repeat}",
                    "case_kind": "coast",
                    "pedal_level": 0,
                    "requested_accel": 0,
                    "applied_accel": 0,
                    "imu_accel_x_mps2": -0.2,
                }
            )
            rows.append(
                {
                    **common,
                    "case_id": f"brake-01-r{repeat}",
                    "case_kind": "brake",
                    "pedal_level": 0.1,
                    "requested_accel": 0,
                    "requested_brake": 0.1,
                    "applied_accel": 0,
                    "applied_brake": 0.1,
                    "imu_accel_x_mps2": -0.3,
                }
            )
    write_raw(raw, rows)

    MODULE.run(
        raw,
        tmp_path / "analysis",
        accel,
        brake,
        candidate_dir=tmp_path / "candidate",
        min_samples=5,
        min_repeats=2,
        fit_source="imu",
    )

    source_accel = read_values(accel)
    source_brake = read_values(brake)
    fitted_accel = read_values(tmp_path / "candidate/accel_map.csv")
    fitted_brake = read_values(tmp_path / "candidate/brake_map.csv")
    assert fitted_accel[0, 0] == pytest.approx(-0.2)
    assert fitted_brake[0, 0] == pytest.approx(-0.2)
    assert fitted_brake[1, 0] == pytest.approx(-0.3)
    assert np.array_equal(fitted_accel[1:], source_accel[1:])
    assert np.array_equal(fitted_brake[2:], source_brake[2:])
    assert np.all(np.diff(fitted_accel, axis=0) > 0)
    assert np.all(np.diff(fitted_brake, axis=0) < 0)


def test_existing_nonempty_candidate_directory_is_rejected(tmp_path):
    accel, brake = write_sources(tmp_path)
    raw = tmp_path / "raw.csv"
    write_raw(raw, sample_rows())
    candidate = tmp_path / "candidate"
    candidate.mkdir()
    marker = candidate / "keep.txt"
    marker.write_text("do not overwrite", encoding="utf-8")

    with pytest.raises(FileExistsError):
        MODULE.run(
            raw,
            tmp_path / "analysis",
            accel,
            brake,
            candidate_dir=candidate,
            min_samples=5,
            min_repeats=2,
        )
    assert marker.read_text(encoding="utf-8") == "do not overwrite"


def test_output_inside_original_map_directory_is_rejected(tmp_path):
    accel, brake = write_sources(tmp_path)
    raw = tmp_path / "raw.csv"
    write_raw(raw, sample_rows())

    with pytest.raises(ValueError, match="outside the original map directory"):
        MODULE.run(
            raw,
            accel.parent / "analysis",
            accel,
            brake,
            min_samples=5,
            min_repeats=2,
        )
    assert not (accel.parent / "analysis").exists()


def test_failed_partial_sweep_with_blank_status_row_writes_invalid_artifacts(
    tmp_path, capsys
):
    accel, brake = write_sources(tmp_path)
    raw = tmp_path / "raw.csv"
    reset_row = {
        **sample_rows()[0],
        "phase": "reset_settle",
        "fit_eligible": 0,
    }
    rows = [startup_row(), reset_row]
    write_raw(raw, rows)
    sweep_summary = tmp_path / "sweep_summary.json"
    write_sweep_summary(
        sweep_summary,
        status="failed",
        reason="unexpected_reverse_velocity",
        rows_written=len(rows),
    )
    output = tmp_path / "analysis"
    candidate = tmp_path / "candidate"
    original_accel = accel.read_bytes()
    original_brake = brake.read_bytes()

    exit_code = MODULE.main(
        [
            "--raw-csv",
            str(raw),
            "--sweep-summary",
            str(sweep_summary),
            "--output-dir",
            str(output),
            "--original-accel-map",
            str(accel),
            "--original-brake-map",
            str(brake),
            "--candidate-dir",
            str(candidate),
        ]
    )

    captured = capsys.readouterr()
    assert exit_code == 3
    assert "Traceback" not in captured.out
    assert "Traceback" not in captured.err
    assert not candidate.exists()
    assert accel.read_bytes() == original_accel
    assert brake.read_bytes() == original_brake
    for name in (
        "actuation_response.csv",
        "actuation_response.json",
        "actuation_response.png",
        "actuation_coverage.png",
    ):
        assert (output / name).stat().st_size > 0

    payload = json.loads((output / "actuation_response.json").read_text())
    assert payload["status"] == "invalid"
    assert payload["invalid_reasons"] == [
        "sweep_not_completed",
        "no_fit_eligible_rows",
        "no_covered_cells",
    ]
    assert payload["sweep"]["reason"] == "unexpected_reverse_velocity"
    assert payload["candidate_requested_dir"] == str(candidate.resolve())
    assert payload["candidate_dir"] is None
    assert payload["candidate_written"] is False
    assert payload["candidate_converter_config"] is None
    with (output / "actuation_response.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        enriched = list(csv.DictReader(stream))
    assert len(enriched) == len(rows)
    assert enriched[0]["analysis_status"] == "invalid"
    assert enriched[0]["derived_accel_mps2"] == ""
    assert "sweep_not_completed" in enriched[0]["analysis_invalid_reasons"]


def test_completed_sweep_tolerates_blank_startup_row_and_writes_candidate(tmp_path):
    accel, brake = write_sources(tmp_path)
    raw = tmp_path / "raw.csv"
    rows = [startup_row(), *sample_rows()]
    write_raw(raw, rows)
    sweep_summary = tmp_path / "sweep_summary.json"
    write_sweep_summary(
        sweep_summary,
        status="completed",
        reason="all_cases_completed",
        rows_written=len(rows),
    )

    summary = MODULE.run(
        raw,
        tmp_path / "analysis",
        accel,
        brake,
        sweep_summary=sweep_summary,
        candidate_dir=tmp_path / "candidate",
        min_samples=5,
        min_repeats=2,
    )

    assert summary["status"] == "valid"
    assert summary["invalid_reasons"] == []
    assert summary["candidate_written"] is True
    assert (tmp_path / "candidate/accel_map.csv").is_file()
    with (tmp_path / "analysis/actuation_response.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        enriched = list(csv.DictReader(stream))
    assert enriched[0]["analysis_status"] == "valid"
    assert enriched[0]["derived_accel_mps2"] == ""
