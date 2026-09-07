"""HH_260906 - Verify raw response calibration contracts without a live simulator."""

import copy
import json
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/e2e"))
import calibrate_carla_low_speed_response as calibration


BOUNDS = {
    "physical_decoder": {"maximum_acceleration_mps2": 2.9, "maximum_deceleration_mps2": 2.9},
    "runtime_speed_rate_gate": {"maximum_acceleration_mps2": 3.0, "maximum_deceleration_mps2": 6.0},
}


def physics_fixture():
    values = {name: 1.0 for name in calibration.PHYSICS_REQUIRED}
    wheel = {name: 1.0 for name in calibration.WHEEL_REQUIRED}
    wheel["position"] = SimpleNamespace(x=1.0, y=2.0, z=3.0)
    values["wheels"] = [SimpleNamespace(**copy.deepcopy(wheel)) for _ in range(4)]
    values["use_gear_autobox"] = True
    values["use_sweep_wheel_collision"] = False
    values["torque_curve"] = [SimpleNamespace(x=0.0, y=500.0), SimpleNamespace(x=5000.0, y=400.0)]
    values["steering_curve"] = [SimpleNamespace(x=0.0, y=1.0)]
    values["forward_gears"] = [SimpleNamespace(ratio=3.0, down_ratio=0.5, up_ratio=0.8)]
    values["center_of_mass"] = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    return SimpleNamespace(**values)


def test_matrix_is_exact_ordered_twelve_fresh_cases():
    cases = calibration.case_matrix()
    assert len(cases) == 12
    assert [case.level for case in cases[:6]] == [0.05, 0.10, 0.15, 0.20, 0.30, 0.40]
    assert [case.level for case in cases[6:]] == [0.02, 0.04, 0.06, 0.08, 0.10, 0.12]
    assert len({case.case_id for case in cases}) == 12
    assert all(case.kind == "throttle" for case in cases[:6])
    assert all(case.kind == "brake" for case in cases[6:])


def test_cli_preserves_wrapper_contract_without_allowing_map_load():
    args = calibration.parse_args(["new-output", "route.json", "--host", "127.0.0.1", "--port", "2100"])
    assert args.host == "127.0.0.1" and args.port == 2100
    assert args.physics_hz == 20 and args.allow_map_load is False
    assert args.matrix == "low_speed_v1"


def test_explicit_v2_matrix_does_not_replace_the_v1_default():
    args = calibration.parse_args(["new-output", "route.json", "--matrix", "low_speed_v2"])
    assert len(calibration.case_matrix(args.matrix)) == 9
    assert len(calibration.case_matrix()) == 12
    assert calibration.case_matrix()[0].case_id == "01_throttle_0.05"
    assert calibration.case_matrix()[-1].case_id == "12_brake_0.12"
    with pytest.raises(calibration.CalibrationError, match="unknown"):
        calibration.case_matrix("not_declared")


@pytest.mark.parametrize("flags", [["--ho", "127.0.0.2"], ["--po", "2101"], ["--allow-m"],
                                    ["--allow-map-load"], ["--physics-hz", "10"], ["--port", "0"],
                                    ["--port", "65536"], ["--timeout", "nan"], ["--timeout", "31"],
                                    ["--matrix", "unknown"], ["--mat", "low_speed_v2"]])
def test_cli_rejects_abbreviations_mutations_and_invalid_bounds(flags):
    with pytest.raises(SystemExit):
        calibration.parse_args(["new-output", "route.json", *flags])


def test_complete_physics_serialization_includes_nested_wheels_gears_and_extra_fields():
    fixture = physics_fixture()
    fixture.additional_build_parameter = 7.0
    result = calibration.physics_snapshot(fixture)
    assert result["values"]["wheels"][0]["position"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert result["values"]["forward_gears"][0]["ratio"] == 3.0
    assert result["values"]["use_gear_autobox"] is True
    assert result["extra_vehicle_fields"] == ["additional_build_parameter"]


@pytest.mark.parametrize("case", ["missing_mass", "missing_wheel_field", "wrong_wheel_count", "nonfinite", "unsupported", "null_mass", "boolean_mass", "string_mass", "bad_bool", "bad_vector"])
def test_physics_serialization_fails_closed_instead_of_saving_repr(case):
    fixture = physics_fixture()
    if case == "missing_mass": del fixture.mass
    if case == "missing_wheel_field": del fixture.wheels[0].max_brake_torque
    if case == "wrong_wheel_count": fixture.wheels.pop()
    if case == "nonfinite": fixture.mass = math.nan
    if case == "unsupported": fixture.unhandled = object()
    if case == "null_mass": fixture.mass = None
    if case == "boolean_mass": fixture.mass = True
    if case == "string_mass": fixture.mass = "1500"
    if case == "bad_bool": fixture.use_gear_autobox = 1
    if case == "bad_vector": fixture.center_of_mass.x = "zero"
    with pytest.raises(calibration.CalibrationError):
        calibration.physics_snapshot(fixture)


@pytest.mark.parametrize("changes, collisions, reason", [
    ({"travel_m": 80.01}, 0, "maximum_travel_exceeded"),
    ({"route_cte_m": 3.01}, 0, "maximum_cross_track_error_exceeded"),
    ({"vx": -0.1001}, 0, "reverse_motion_exceeded"),
    ({"vx": 10.01}, 0, "maximum_speed_exceeded"),
    ({"vx": math.nan}, 0, "nonfinite_motion"), ({}, 1, "collision"),
])
def test_safety_guards_preserve_strict_bounds(changes, collisions, reason):
    state = dict(vx=1.0, vy=0.0, travel_m=5.0, route_cte_m=0.0)
    state.update(changes)
    assert calibration.safety_reason(state, collisions) == reason


def test_safe_motion_does_not_discard_large_acceleration_measurements():
    assert calibration.safety_reason(dict(vx=1.0, vy=0.0, travel_m=0.0, route_cte_m=0.0, ax=-22.0), 0) is None


def test_rates_retain_start_stop_and_boundary_at_both_10hz_offsets():
    records = [dict(frame=index + 100, timestamp=index * 0.05, vx=speed, vy=0.0,
                    phase="settle" if index < 2 else "throttle_hold" if index < 4 else "brake_hold")
               for index, speed in enumerate([0.0, 0.0, 0.2, 0.3, 0.0, 0.0])]
    result = calibration.analyze_rates(records, BOUNDS)
    assert result["training_data"] is False
    native = result["measurements"]["native_20hz"]["phases"]
    assert native["throttle_hold"]["maximum_speed_rate_mps2"] == pytest.approx(4.0)
    assert native["brake_hold"]["minimum_speed_rate_mps2"] == pytest.approx(-6.0)
    assert native["brake_hold"]["phase_boundary_intervals"][0]["from_phase"] == "throttle_hold"
    assert result["measurements"]["derived_10hz_offset_0"]["sample_count"] == 3
    assert result["measurements"]["derived_10hz_offset_1"]["sample_count"] == 3


def test_rate_analysis_preserves_new_coast_and_ramp_phases_separately():
    rows = [dict(frame=index, timestamp=index * 0.05, vx=speed, vy=0.0, phase=phase)
            for index, (speed, phase) in enumerate([(3.0, "prepare"), (2.0, "coast_hold"),
                                                    (1.0, "coast_hold"), (0.0, "throttle_ramp")])]
    phases = calibration.analyze_rates(rows, BOUNDS)["measurements"]["native_20hz"]["phases"]
    assert phases["coast_hold"]["interval_count"] == 2
    assert phases["coast_hold"]["minimum_speed_rate_mps2"] == pytest.approx(-20)
    assert phases["coast_hold"]["phase_boundary_intervals"][0]["from_phase"] == "prepare"
    assert phases["throttle_ramp"]["interval_count"] == 1


def test_destroy_owned_never_discovers_or_deletes_foreign_actors():
    calls = []
    def actor(identity):
        return SimpleNamespace(id=identity, type_id="vehicle.test", destroy=lambda: calls.append(identity) or True)
    owned = [actor(1), actor(2)]
    foreign = actor(3)
    assert calibration.destroy_owned(owned) == []
    assert calls == [2, 1]
    assert foreign.id == 3


def test_destroy_false_is_not_reported_as_success():
    actor = SimpleNamespace(id=9, type_id="vehicle.test", destroy=lambda: False)
    assert "did not confirm success" in calibration.destroy_owned([actor])[0]


def test_lock_is_required_and_wrong_inherited_descriptor_refused(tmp_path, monkeypatch):
    monkeypatch.delenv("AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD", raising=False)
    with pytest.raises(calibration.CalibrationError, match="missing"):
        calibration.require_inherited_workspace_lock(tmp_path / "expected.lock")
    actual = tmp_path / "actual.lock"
    expected = tmp_path / "expected.lock"
    expected.touch()
    with actual.open("w") as stream:
        monkeypatch.setenv("AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD", str(stream.fileno()))
        with pytest.raises(calibration.CalibrationError, match="does not belong"):
            calibration.require_inherited_workspace_lock(expected)
        assert calibration.require_inherited_workspace_lock(actual)["verified_exclusive"] is True
        os.fstat(stream.fileno())


@pytest.mark.parametrize("changes", [{"fixed_delta_seconds": 0.1}, {"synchronous_mode": False},
                                    {"substepping": False}, {"max_substeps": 1}, {"max_substep_delta_time": 0.0}])
def test_timing_rejects_unapplied_or_inadequate_physics_settings(changes):
    values = dict(synchronous_mode=True, fixed_delta_seconds=0.05, substepping=True,
                  max_substep_delta_time=0.01, max_substeps=10)
    values.update(changes)
    with pytest.raises(calibration.CalibrationError, match="timing"):
        calibration.validate_world_timing(SimpleNamespace(**values))


def test_rate_report_marks_frame_and_timestamp_gaps():
    rows = [dict(frame=1, timestamp=0.0, vx=0.0, vy=0.0, phase="settle"),
            dict(frame=3, timestamp=0.1, vx=0.0, vy=0.0, phase="settle")]
    report = calibration.analyze_rates(rows, BOUNDS)["measurements"]["native_20hz"]
    assert report["cadence_violation_count"] == 1
    assert report["frame_stride_violation_count"] == 1


@pytest.mark.parametrize("kind", ["output", "partial", "dangling_output", "dangling_partial"])
def test_existing_targets_are_refused_before_any_carla_import(tmp_path, kind):
    output = tmp_path / "response"
    target = Path(str(output) + ".partial") if "partial" in kind else output
    if "dangling" in kind:
        target.symlink_to(tmp_path / "does_not_exist")
    else:
        target.mkdir()
    with pytest.raises(calibration.CalibrationError, match="already exists"):
        calibration.run(calibration.parse_args([str(output), str(tmp_path / "missing_route.json")]))


def route_file(tmp_path):
    path = tmp_path / "route.json"
    path.write_text(json.dumps({"schema_version": 1, "coordinate_reference": "base_link", "town": "Town07", "scenario": "straight",
                               "start_carla_transform": {"x": 0, "y": 0, "z": 0}, "goal_carla_transform": {"x": 100, "y": 0, "z": 0},
                               "route": [{"x": 0, "y": 0, "distance_m": 0, "vad_command": 3},
                                         {"x": 100, "y": 0, "distance_m": 100, "vad_command": 3}]}))
    return path


def test_world_settings_and_weather_restore_after_case_failure(tmp_path, monkeypatch):
    settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None, no_rendering_mode=False,
                               substepping=True, max_substep_delta_time=0.01, max_substeps=10)
    original_weather = object()
    applied, weather = [], []
    current = {"settings": copy.copy(settings)}
    def apply_settings(value):
        applied.append(copy.copy(value))
        current["settings"] = copy.copy(value)
    world = SimpleNamespace(get_map=lambda: SimpleNamespace(name="/Game/Carla/Maps/Town07"),
                            get_settings=lambda: copy.copy(current["settings"]), get_weather=lambda: original_weather,
                            apply_settings=apply_settings, set_weather=lambda value: weather.append(value))
    client = SimpleNamespace(set_timeout=lambda _: None, get_world=lambda: world,
                             get_server_version=lambda: "0.9.15", get_client_version=lambda: "0.9.15")
    fake_carla = SimpleNamespace(Client=lambda *_: client, WeatherParameters=SimpleNamespace(ClearNoon="clear"))
    monkeypatch.setitem(sys.modules, "carla", fake_carla)
    monkeypatch.setattr(calibration, "exclusive_world", lambda _: None)
    monkeypatch.setattr(calibration, "source_motion_bounds", lambda: BOUNDS)
    monkeypatch.setattr(calibration, "require_inherited_workspace_lock", lambda: {"verified_exclusive": True})
    def fail_case(*_args):
        raise calibration.CalibrationError("case failure retained")
    monkeypatch.setattr(calibration, "collect_case", fail_case)
    output = tmp_path / "response"
    args = calibration.parse_args([str(output), str(route_file(tmp_path))])
    with pytest.raises(calibration.CalibrationError, match="case failure retained"):
        calibration.run(args)
    assert applied[0].synchronous_mode is True and applied[0].fixed_delta_seconds == 0.05
    assert applied[-1].synchronous_mode is False and applied[-1].fixed_delta_seconds is None
    assert weather == ["clear", original_weather]
    manifest = json.loads((Path(str(output) + ".partial") / "manifest.json").read_text())
    assert manifest["status"] == "failed" and manifest["cleanup"]["completed"] is True
    assert manifest["case_ledger"][0]["status"] == "failed"
    assert all(entry["status"] == "not_run_after_failure" for entry in manifest["case_ledger"][1:])
    assert not output.exists()


def test_preflight_rejects_walkers_without_destroying_them(monkeypatch):
    monkeypatch.setattr(calibration.capture, "_preflight_exclusive", lambda _: None)
    actors = SimpleNamespace(filter=lambda pattern: [object()] if pattern == "walker.pedestrian.*" else [])
    with pytest.raises(calibration.CalibrationError, match="foreign dynamic actors"):
        calibration.exclusive_world(SimpleNamespace(get_actors=lambda: actors))
