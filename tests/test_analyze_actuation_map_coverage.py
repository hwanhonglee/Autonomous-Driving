from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "scripts/e2e/analyze_actuation_map_coverage.py"
SPEC = importlib.util.spec_from_file_location("analyze_actuation_map_coverage", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
coverage = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(coverage)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fixture(tmp_path: Path) -> tuple[Path, Path]:
    provenance = tmp_path / "provenance"
    provenance.mkdir()
    source = tmp_path / "source"
    source.mkdir()
    for name in coverage.IMPLEMENTATION_FILES:
        (source / name).write_text(
            "const double clamped_vel = "
            "CSVLoader::clampValue(vel, vel_index_, \"test\");\n",
            encoding="utf-8",
        )
    (source / "node.cpp").write_text(
        "const double vel = current_odometry_->twist.twist.linear.x;\n"
        "calculateAccelMap(vel, acc, accel_cmd_is_zero);\n"
        "calculateBrakeMap(vel, acc);\n"
        "double RawVehicleCommandConverterNode::calculateAccelMap(\n"
        "  const double current_velocity, const double desired_acc,\n"
        "  bool & accel_cmd_is_zero)\n"
        "{\n"
        "  double desired_accel_cmd = 0.0;\n"
        "  accel_map_.getThrottle(\n"
        "    desired_acc, std::abs(current_velocity), desired_accel_cmd);\n"
        "  return desired_accel_cmd;\n"
        "}\n"
        "double RawVehicleCommandConverterNode::calculateBrakeMap(\n"
        "  const double current_velocity, const double desired_acc)\n"
        "{\n"
        "  double desired_brake_cmd = 0.0;\n"
        "  brake_map_.getBrake(\n"
        "    desired_acc, std::abs(current_velocity), desired_brake_cmd);\n"
        "  return desired_brake_cmd;\n"
        "}\n",
        encoding="utf-8",
    )
    csv_text = "default,0,5,10,13.89\n0,0,-0.1,-0.2,-0.3\n"
    files = {}
    for name in ("accel_map", "brake_map"):
        artifact = provenance / f"{name}.csv"
        artifact.write_text(csv_text, encoding="utf-8")
        files[name] = {"artifact": artifact.name, "sha256": _sha256(artifact)}
    (provenance / "manifest.json").write_text(
        json.dumps({"schema_version": 1, "files": files}), encoding="utf-8"
    )
    return provenance, source


def test_target_inside_map_axis_passes(tmp_path: Path) -> None:
    provenance, source = _fixture(tmp_path)

    payload = coverage.analyze_coverage(
        provenance,
        profile_id="speed_30",
        target_speed_mps=8.333333333333334,
        allow_clamped_target=False,
        converter_source_directory=source,
    )

    assert payload["status"] == "PASS"
    assert payload["target_within_map_velocity_axis"] is True
    assert payload["target_envelope_classification"] == (
        "TARGET_ENVELOPE_COVERED_BY_MAP_AXIS"
    )
    assert payload["runtime_lookup_observation"]["available"] is False
    lookups = payload["provenance"]["converter_node"][
        "verified_map_lookups"
    ]
    assert set(lookups) == {"accel_map", "brake_map"}
    assert all(
        record["absolute_current_velocity_verified"] is True
        for record in lookups.values()
    )


def test_repository_converter_source_uses_absolute_speed_for_both_maps() -> None:
    record = coverage._converter_node_record(
        coverage.CONVERTER_SOURCE_DIRECTORY
    )

    lookups = record["verified_map_lookups"]
    assert lookups["accel_map"]["velocity_arguments"] == [
        "std::abs(current_velocity)"
    ]
    assert lookups["brake_map"]["velocity_arguments"] == [
        "std::abs(current_velocity)"
    ]


def test_out_of_range_target_blocks_by_default(tmp_path: Path) -> None:
    provenance, source = _fixture(tmp_path)
    output = tmp_path / "coverage.json"
    args = argparse.Namespace(
        provenance_dir=provenance,
        profile_id="speed_60_pilot",
        target_speed_mps=16.666666666666668,
        output=output,
        allow_clamped_target=False,
        converter_source_directory=source,
    )

    assert coverage.run(args) == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "BLOCKED"
    assert payload["target_excess_mps"] == pytest.approx(2.7766666666666673)


def test_target_below_map_axis_does_not_pass(tmp_path: Path) -> None:
    provenance, source = _fixture(tmp_path)
    for name in ("accel_map", "brake_map"):
        artifact = provenance / f"{name}.csv"
        artifact.write_text(
            "default,1,5,10,13.89\n0,0,-0.1,-0.2,-0.3\n", encoding="utf-8"
        )
    manifest = json.loads((provenance / "manifest.json").read_text(encoding="utf-8"))
    for name in ("accel_map", "brake_map"):
        manifest["files"][name]["sha256"] = _sha256(provenance / f"{name}.csv")
    (provenance / "manifest.json").write_text(
        json.dumps(manifest), encoding="utf-8"
    )

    payload = coverage.analyze_coverage(
        provenance,
        profile_id="below_axis",
        target_speed_mps=0.5,
        allow_clamped_target=False,
        converter_source_directory=source,
    )

    assert payload["status"] == "BLOCKED"
    assert payload["target_within_map_velocity_axis"] is False
    assert payload["target_shortfall_mps"] == pytest.approx(0.5)


def test_explicit_exploratory_opt_in_records_clamped_lookup(tmp_path: Path) -> None:
    provenance, source = _fixture(tmp_path)

    payload = coverage.analyze_coverage(
        provenance,
        profile_id="speed_60_pilot",
        target_speed_mps=16.666666666666668,
        allow_clamped_target=True,
        converter_source_directory=source,
    )

    assert payload["status"] == "EXPLORATORY"
    assert payload["target_envelope_classification"] == (
        "TARGET_ENVELOPE_EXCEEDS_MAP_AXIS_CLAMP_IF_REACHED"
    )
    assert (
        payload[
            "target_envelope_extension_authorized_for_exploratory_simulation"
        ]
        is True
    )
    assert payload["validation_boundary"]["real_vehicle_ready"] is False
    assert payload["validation_boundary"]["target_speed_is_converter_lookup_velocity"] is False


def test_observed_speed_distinguishes_runtime_lookup_from_target_envelope(
    tmp_path: Path,
) -> None:
    provenance, source = _fixture(tmp_path)

    payload = coverage.analyze_coverage(
        provenance,
        profile_id="speed_60_pilot",
        target_speed_mps=16.666666666666668,
        allow_clamped_target=True,
        observed_maximum_speed_mps=9.776464619638844,
        converter_source_directory=source,
    )

    observation = payload["runtime_lookup_observation"]
    assert payload["status"] == "EXPLORATORY"
    assert observation["within_map_velocity_axis"] is True
    assert observation["velocity_axis_clamping_observed"] is False
    assert observation["classification"] == "OBSERVED_LOOKUPS_WITHIN_MAP_AXIS"


def test_tampered_map_artifact_is_rejected(tmp_path: Path) -> None:
    provenance, source = _fixture(tmp_path)
    (provenance / "accel_map.csv").write_text(
        "default,0,1\n0,0,0\n", encoding="utf-8"
    )

    with pytest.raises(coverage.CoverageError, match="SHA-256"):
        coverage.analyze_coverage(
            provenance,
            profile_id="speed_60_pilot",
            target_speed_mps=16.666666666666668,
            allow_clamped_target=True,
            converter_source_directory=source,
        )


@pytest.mark.parametrize(
    ("lookup", "unsafe_lookup"),
    (
        (
            "accel_map_.getThrottle(\n"
            "    desired_acc, std::abs(current_velocity), desired_accel_cmd)",
            "accel_map_.getThrottle(\n"
            "    desired_acc, current_velocity, desired_accel_cmd)",
        ),
        (
            "brake_map_.getBrake(\n"
            "    desired_acc, std::abs(current_velocity), desired_brake_cmd)",
            "brake_map_.getBrake(\n"
            "    desired_acc, current_velocity, desired_brake_cmd)",
        ),
    ),
)
def test_signed_current_velocity_lookup_is_rejected_even_with_comment_decoy(
    tmp_path: Path, lookup: str, unsafe_lookup: str
) -> None:
    provenance, source = _fixture(tmp_path)
    node = source / "node.cpp"
    text = node.read_text(encoding="utf-8")
    node.write_text(
        text.replace(lookup, unsafe_lookup) + f"// decoy: {lookup}\n",
        encoding="utf-8",
    )

    with pytest.raises(
        coverage.CoverageError, match="does not use absolute current velocity"
    ):
        coverage.analyze_coverage(
            provenance,
            profile_id="speed_30",
            target_speed_mps=8.333333333333334,
            allow_clamped_target=False,
            converter_source_directory=source,
        )


def test_equivalent_absolute_velocity_alias_is_accepted(tmp_path: Path) -> None:
    provenance, source = _fixture(tmp_path)
    node = source / "node.cpp"
    text = node.read_text(encoding="utf-8")
    node.write_text(
        text.replace(
            "double desired_brake_cmd = 0.0;\n"
            "  brake_map_.getBrake(\n"
            "    desired_acc, std::abs(current_velocity), desired_brake_cmd);",
            "double desired_brake_cmd = 0.0;\n"
            "  const auto absolute_velocity = std::fabs(current_velocity);\n"
            "  brake_map_.getBrake(\n"
            "    desired_acc, absolute_velocity, desired_brake_cmd);",
        ),
        encoding="utf-8",
    )

    payload = coverage.analyze_coverage(
        provenance,
        profile_id="speed_30",
        target_speed_mps=8.333333333333334,
        allow_clamped_target=False,
        converter_source_directory=source,
    )

    brake = payload["provenance"]["converter_node"][
        "verified_map_lookups"
    ]["brake_map"]
    assert brake["velocity_arguments"] == ["absolute_velocity"]
