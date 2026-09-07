"""HH_260906 - Check independent fixed-pedal evidence reconstruction without a simulator or GPU."""

import copy
import json
from pathlib import Path
import shutil

import pytest

from scripts.e2e import summarize_carla_low_speed_response as summary


@pytest.fixture
def case_data():
    bounds = {"physical_decoder": {"maximum_acceleration_mps2": 2.9, "maximum_deceleration_mps2": 2.9},
              "runtime_speed_rate_gate": {"maximum_acceleration_mps2": 3.0, "maximum_deceleration_mps2": 6.0}}
    case = summary.matrix()[3]
    rows = []
    for index in range(230):
        command = {"throttle": 0.0 if index < 70 else 0.2, "brake": 1.0 if index < 70 else 0.0,
                   "steer": 0.0, "reverse": False, "hand_brake": False, "manual_gear_shift": False}
        rows.append({"frame": index, "timestamp": index * 0.05, "phase": "settle" if index < 70 else "throttle_hold",
                     "vx": 0.4 if index == 70 else 0.0, "vy": 0.0, "x": 0.0, "y": 0.0,
                     "travel_m": 0.0, "route_cte_m": 0.0, "collision": [],
                     "requested_control": command.copy(), "applied_control": command.copy()})
    physics = {"schema": "carla.vehicle_physics_snapshot.v1", "values": {"mass": 1375.0, "max_rpm": 5000.0,
               "wheels": [{"radius": 37.0, "max_brake_torque": 800.0, "tire_friction": 3.5} for _ in range(4)]}}
    report = {"case": case.copy(), "status": "complete", "training_data": False, "actor_id": 1,
              "cleanup": {"completed": True, "errors": []}, "collision_events": [], "state_count": 230,
              "post_despawn_empty_world_frame": 230, "phase_counts": {"settle": 70, "prepare": 0, "throttle_hold": 160, "brake_hold": 0},
              "maximum_speed_mps": 0.4, "final_speed_mps": 0.0, "vehicle_physics": physics,
              "motion_analysis": {"measurements": summary.summarize_rates(rows, bounds)}}
    return case, report, rows, bounds


def test_fixed_matrix_retains_every_planned_level():
    cases = summary.matrix()
    assert len(cases) == 12
    assert [item["level"] for item in cases[:6]] == [0.05, 0.1, 0.15, 0.2, 0.3, 0.4]
    assert [item["level"] for item in cases[6:]] == [0.02, 0.04, 0.06, 0.08, 0.1, 0.12]
    assert cases[0]["case_id"] == "01_throttle_0.05"
    assert cases[-1]["case_id"] == "12_brake_0.12"


def test_startup_impulse_and_both_decimation_offsets_retained(case_data):
    result = summary.verify_case(*case_data)
    rates = result["measurements"]
    native = rates["native_20hz"]["phases"]["all"]
    assert native["interval_count"] == 229
    assert native["maximum_speed_rate_mps2"] == pytest.approx(8)
    assert native["minimum_speed_rate_mps2"] == pytest.approx(-8)
    assert native["physical_decoder"]["violation_count"] == 2
    assert rates["derived_10hz_offset_0"]["phases"]["all"]["physical_decoder"]["violation_count"] == 2
    assert rates["derived_10hz_offset_1"]["phases"]["all"]["physical_decoder"]["violation_count"] == 0
    assert native["phase_boundary_intervals"][0]["from_phase"] == "settle"


@pytest.mark.parametrize("mutation", [
    lambda report, rows: rows[70]["applied_control"].update(throttle=0.3),
    lambda report, rows: rows[70].update(timestamp=3.52),
    lambda report, rows: rows[70].update(frame=700),
    lambda report, rows: rows[70].update(phase="prepare"),
    lambda report, rows: rows[70].update(vx=float("nan")),
    lambda report, rows: report.update(state_count=229),
    lambda report, rows: report["cleanup"].update(completed=False),
    lambda report, rows: report["vehicle_physics"]["values"].update(mass=True),
    lambda report, rows: report.update(maximum_speed_mps=99),
    lambda report, rows: report["motion_analysis"]["measurements"]["native_20hz"]["phases"]["all"]["physical_decoder"].update(violation_count=0),
])
def test_case_rejects_corrupt_measurement_or_plan(case_data, mutation):
    case, report, rows, bounds = copy.deepcopy(case_data)
    mutation(report, rows)
    with pytest.raises(ValueError):
        summary.verify_case(case, report, rows, bounds)


@pytest.fixture
def actual_copy(tmp_path):
    # HH_260906 - Optional local integration evidence is copied, never downloaded or changed in place.
    source = Path(__file__).resolve().parents[1] / "artifacts/training/2026-09-08/low_speed_response_v1/run_001"
    if not (source / "owner_result.json").is_file():
        pytest.skip("local completed calibration evidence is not distributed with the source checkout")
    destination = tmp_path / "trial"
    shutil.copytree(source, destination)
    return destination


def rewrite_json(path, change):
    value = json.loads(path.read_text())
    change(value)
    path.write_text(json.dumps(value))


def test_actual_full_publication_and_no_overwrite(actual_copy, tmp_path):
    pytest.importorskip("matplotlib")
    output = tmp_path / "summary"
    result = summary.publish(actual_copy, output)
    assert result["planned_and_included_case_count"] == 12
    assert result["vehicle_physics_identical_across_all_cases"] is True
    assert len({case["actor_id"] for case in result["cases"]}) == 12
    assert result["scope"]["cameras_collected"] is False
    assert result["scope"]["automatic_promotion"] is False
    assert str(actual_copy) not in (output / "summary.json").read_text()
    assert len(list(output.glob("*.png"))) == 3
    for line in (output / "SHA256SUMS").read_text().splitlines():
        expected, name = line.split("  ")
        assert summary.base.sha(output / name) == expected
    with pytest.raises(ValueError, match="already exists"):
        summary.publish(actual_copy, output)


@pytest.mark.parametrize("filename,mutation,message", [
    ("owner_result.json", lambda value: value.update(source_bytes_unchanged_and_archived=False), "source freeze"),
    ("lifecycle/stopped.json", lambda value: value.update(owner_pid=999999), "lifecycle"),
    ("actuation/manifest.json", lambda value: value["matrix"].pop(), "twelve"),
])
def test_actual_rejects_owner_lifecycle_or_case_omission(actual_copy, filename, mutation, message):
    rewrite_json(actual_copy / filename, mutation)
    with pytest.raises(ValueError, match=message):
        summary.verify_trial(actual_copy)


def test_actual_raw_and_archived_source_hashes_are_required(actual_copy):
    path = actual_copy / "actuation/01_throttle_0.05/states.jsonl"
    original = path.read_bytes()
    path.write_bytes(original + b"\n")
    with pytest.raises(ValueError, match="raw state hash"):
        summary.verify_trial(actual_copy)
    path.write_bytes(original)
    source = actual_copy / "provenance/scripts/e2e/calibrate_carla_low_speed_response.py"
    source.write_bytes(source.read_bytes() + b"\n")
    with pytest.raises(ValueError, match="source hash"):
        summary.verify_trial(actual_copy)


def test_v2_nine_case_plan_is_explicit_and_unknown_plans_rejected():
    cases = summary.matrix("low_speed_v2")
    assert len(cases) == 9
    assert [case["kind"] for case in cases] == ["coast"] * 2 + ["throttle"] * 3 + ["throttle_ramp"] * 4
    assert [case["hold_seconds"] for case in cases] == [20.0, 20.0] + [8.0] * 7
    assert summary.identification_contract()["ramp_policy"]["final_commanded_throttles"] == [0.08, 0.2, 0.4, 0.4]
    with pytest.raises(ValueError, match="unknown"):
        summary.matrix("low_speed_v3")


def make_v2_case(case_data, index):
    # HH_260906 - Synthetic states exercise plan verification, not simulator quality or plausible dynamics.
    _, report, old_rows, bounds = copy.deepcopy(case_data)
    case = summary.matrix("low_speed_v2")[index]
    phases = ["settle"] * 70 + (["prepare"] + ["coast_hold"] * 400 if case["kind"] == "coast" else
             ["throttle_hold" if case["kind"] == "throttle" else "throttle_ramp"] * 160)
    rows = []
    for i, phase in enumerate(phases):
        row = copy.deepcopy(old_rows[0])
        throttle, brake = (0, 1) if phase == "settle" else (0.3, 0) if phase == "prepare" else (
            (0, 0) if phase == "coast_hold" else (0.15, 0) if phase == "throttle_hold" else
            (min(0.4, case["ramp_rate_per_second"] * (i - 69) / 20), 0))
        row.update(frame=i, timestamp=i * 0.05, phase=phase, vx=3.0 if phase == "prepare" else 0.0)
        for field in ("requested_control", "applied_control"):
            row[field].update(throttle=throttle, brake=brake)
        rows.append(row)
    report.update(case=case, matrix_id="low_speed_v2", state_count=len(rows),
                  post_despawn_empty_world_frame=len(rows), maximum_speed_mps=3.0 if case["kind"] == "coast" else 0,
                  phase_counts={phase: phases.count(phase) for phase in summary.phase_names(rows)},
                  coast_entry_frame=70, coast_entry_speed_mps=3.0)
    report["motion_analysis"]["measurements"] = summary.summarize_rates(rows, bounds)
    return case, report, rows, bounds


@pytest.mark.parametrize("index", range(9))
def test_v2_all_nine_case_phase_and_pedal_contracts(case_data, index):
    case, report, rows, bounds = make_v2_case(case_data, index)
    result = summary.verify_case(case, report, rows, bounds, "low_speed_v2")
    assert result["state_count"] == len(rows)
    if index >= 5:
        assert result["final_commanded_throttle"] == pytest.approx([0.08, 0.2, 0.4, 0.4][index - 5])
    if index < 2:
        assert result["phase_counts"]["coast_hold"] == 400
        assert result["coast_entry_speed_mps"] == 3.0
    with pytest.raises(ValueError, match="immutable matrix"):
        summary.verify_case(case, report, rows, bounds)


@pytest.mark.parametrize("index,row_index,field,value", [(0, 71, "brake", 0.02), (5, 70, "throttle", 0.0),
                                                        (5, 229, "throttle", 0.4), (6, 71, "throttle", 0.5)])
def test_v2_zero_pedal_and_each_ramp_interval_are_verified(case_data, index, row_index, field, value):
    case, report, rows, bounds = make_v2_case(case_data, index)
    rows[row_index]["requested_control"][field] = value
    with pytest.raises(ValueError, match="raw pedal"):
        summary.verify_case(case, report, rows, bounds, "low_speed_v2")


@pytest.fixture
def actual_v2_copy(tmp_path):
    source = Path(__file__).resolve().parents[1] / "artifacts/training/2026-09-08/low_speed_response_v2/run_001"
    if not (source / "owner_result.json").is_file():
        pytest.skip("local completed second calibration evidence is not distributed with the source checkout")
    destination = tmp_path / "trial-v2"
    shutil.copytree(source, destination)
    return destination


def test_actual_v2_complete_publication_all_phases_and_no_promotion(actual_v2_copy, tmp_path):
    pytest.importorskip("matplotlib")
    result = summary.publish(actual_v2_copy, tmp_path / "published-v2")
    assert result["status"] == "NINE_MEASUREMENTS_VERIFIED_NOT_PROMOTED"
    assert result["matrix_id"] == "low_speed_v2"
    assert result["planned_and_included_case_count"] == 9
    assert sum(case["state_count"] for case in result["cases"]) == 2742
    assert len(result["executed_sources"]["source_sha256"]) == 10
    assert len(list((tmp_path / "published-v2").glob("*.png"))) == 3
    assert result["cases"][0]["final_speed_mps"] > 0.1
    for case in result["cases"][:2]:
        native = case["measurements"]["native_20hz"]["phases"]
        assert native["prepare"]["physical_decoder"]["violation_count"] == 1
        assert native["coast_hold"]["physical_decoder"]["violation_count"] == 0
    assert result["scope"]["training_data"] is False
    assert result["scope"]["cameras_collected"] is False


@pytest.mark.parametrize("filename,mutation,message", [
    ("actuation/manifest.json", lambda value: value.update(matrix_id="low_speed_v99"), "unknown"),
    ("actuation/manifest.json", lambda value: value["matrix"].pop(), "planned cases"),
    ("actuation/manifest.json", lambda value: value["identification_matrix_contract"]["ramp_policy"].update(all_ramps_reach_cap=True), "frozen v2"),
    ("owner_plan.json", lambda value: value["source_sha256"].pop("scripts/e2e/carla_low_speed_response_matrix.py"), "source freeze"),
    ("owner_plan.json", lambda value: value["collector_argv"].remove("--matrix"), "explicitly select"),
    ("lifecycle/stopped.json", lambda value: value.update(port_released=False), "stopped proof"),
])
def test_actual_v2_rejects_plan_or_provenance_mutation(actual_v2_copy, filename, mutation, message):
    rewrite_json(actual_v2_copy / filename, mutation)
    with pytest.raises(ValueError, match=message):
        summary.verify_trial(actual_v2_copy)


def test_actual_v2_physics_changed_between_cases_is_rejected_after_rehash(actual_v2_copy):
    path = actual_v2_copy / "actuation/02_coast_repeat_02/report.json"
    rewrite_json(path, lambda value: value["vehicle_physics"]["values"].update(mass=1376.0))
    rewrite_json(actual_v2_copy / "actuation/manifest.json",
                 lambda value: value["completed_cases"][1].update(report_sha256=summary.base.sha(path)))
    with pytest.raises(ValueError, match="physics changed between"):
        summary.verify_trial(actual_v2_copy)


def test_actual_v1_summary_values_remain_identical_to_published_evidence(actual_copy):
    path = Path(__file__).resolve().parents[1] / "docs/assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/04_pedal_response_calibration/summary.json"
    if not path.is_file():
        pytest.skip("published first-study evidence is absent")
    original = json.loads(path.read_text())
    current, _ = summary.verify_trial(actual_copy)
    for field in ("created_at_utc", "summarizer_source_sha256", "supporting_reader_source_sha256"):
        original.pop(field)
        current.pop(field)
    # HH_260906 - The new source-byte proof must not alter any previously published numerical evidence.
    current.pop("bounds_source_proof")
    assert current == original
