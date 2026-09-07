"""HH_260906 - Test independent goal-stop QA and failed-trial accounting without simulator access."""

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest

from scripts.e2e import summarize_carla_goal_stop_trials as summary


@pytest.fixture
def native():
    config = {"profile_id": "comfortable_v1", "goal_tolerance_m": 1.0, "stopped_speed_mps": 0.1,
        "hold_seconds": 2.0, "minimum_tail_seconds": 6.5, "maximum_projection_step_m": 1.0,
        "maximum_projection_error_m": 3.0, "bounds": summary.source_bounds()}
    route = {"town": "Town07", "route": [{"x": 0.0, "y": 0.0, "distance_m": 0.0},
                                           {"x": 1.5, "y": 0.0, "distance_m": 1.5}]}
    states, cameras = [], []
    for frame in range(241):
        phase = "stationary_warmup" if frame < 10 else "driving" if frame < 111 else "stationary_tail"
        state = {"frame": frame, "timestamp": frame * 0.05, "capture_phase": phase,
            "x": 0.75, "y": 0.0, "vx": 0.0, "vy": 0.0, "route_progress_m": 0.75,
            "collision": [], "lane_invasion": [],
            "current_control": {"throttle": 0.0, "brake": 1.0, "steer": 0.0},
            "next_control": {"throttle": 0.0, "brake": 1.0, "steer": 0.0},
            "goal_stop": {"target_speed_mps": 0.0, "control_source": "fixture_measured_stop",
                          "complete": False, "hold_ticks": 0, "hold_duration_sec": 0.0}}
        states.append(state)
        if frame % 2 == 0:
            cameras.append({"frame": frame, "timestamp": state["timestamp"], "capture_phase": phase,
                "camera_order": list(summary.CAMERAS), "images": {name: f"images/{name}/{frame}.jpg" for name in summary.CAMERAS},
                "source_timestamps": {name: state["timestamp"] for name in summary.CAMERAS}})
    return states, cameras, route, config


def analyze(native):
    return summary.analyze_native(*native, summary.source_bounds())[0]


def test_recomputes_goal_dwell_without_believing_false_stored_completion(native):
    result = analyze(native)
    assert result["raw_scalar_quality_clear"] is True
    assert result["goal_reached_independently"] is True
    assert result["goal_dwell_seconds"] == pytest.approx(5.0)
    assert result["phase_counts"]["stationary_tail"] == {"native_states": 130, "camera_anchors": 65}
    assert result["tail_elapsed_from_driving_end_seconds"] == pytest.approx(6.5)


def test_goal_success_does_not_hide_low_speed_rate_spikes_or_control_snippets(native):
    native[0][20]["vx"] = 1.5
    native[0][20]["current_control"]["brake"] = 0.15
    result = analyze(native)
    assert result["goal_reached_independently"] is True
    assert result["raw_scalar_quality_clear"] is False
    for cadence in ("native_20hz", "camera_10hz"):
        qa = result["speed_rate_qa"][cadence]["by_phase"]
        assert qa["stationary_tail"]["physical_decoder"]["violation_count"] == 0
        violations = qa["driving"]["physical_decoder"]["violation_intervals"]
        assert len(violations) == 2
        deceleration = next(row for row in violations if row["speed_rate_mps2"] < 0)
        assert deceleration["from_applied_control"]["brake"] == 0.15
        assert deceleration["from_phase"] == deceleration["to_phase"] == "driving"


def test_both_cadences_retain_the_driving_to_tail_prefix(native):
    native[0][111]["vx"] = native[0][112]["vx"] = 0.4
    result = analyze(native)
    assert not result["flags"]["full_stopped_goal_tail"]
    for cadence in ("native_20hz", "camera_10hz"):
        tail = result["speed_rate_qa"][cadence]["by_phase"]["stationary_tail"]
        prefix = tail["phase_boundary_intervals"][0]
        assert prefix["from_phase"] == "driving" and prefix["to_speed_mps"] == 0.4


def test_missing_camera_prefix_is_not_a_complete_10hz_capture(native):
    native[1][:] = native[1][-2:]
    result = analyze(native)
    assert result["speed_rate_qa"]["camera_10hz"]["cadence_violation_count"] == 0
    assert not result["flags"]["camera_exact_10hz_full_capture_coverage"]


def test_claimed_route_progress_cannot_override_independent_xy_goal_error(native):
    for state in native[0]:
        state["x"] = 0.2
        state["goal_stop"]["complete"] = True
    result = analyze(native)
    assert not result["goal_reached_independently"]
    assert not result["flags"]["route_projection_matches_recording"]
    assert result["final_driving"]["goal_error_m"] == pytest.approx(1.3)


def test_duplicate_route_point_is_allowed_only_at_identical_arc_and_position(native):
    native[2]["route"].insert(1, copy.deepcopy(native[2]["route"][0]))
    assert analyze(native)["raw_scalar_quality_clear"]
    native[2]["route"][1]["x"] = 0.1
    with pytest.raises(summary.EvidenceError, match="invalid route arc"):
        analyze(native)


def test_terminal_duplicate_catalog_points_preserve_raw_route_and_exact_numerical_output(native):
    original = summary.analyze_native(*native, summary.source_bounds())
    native[2]["route"].extend([copy.deepcopy(native[2]["route"][-1]) for _ in range(3)])
    preserved = copy.deepcopy(native)
    assert summary.analyze_native(*native, summary.source_bounds()) == original
    assert native == preserved


def test_all_coincident_route_has_no_terminal_tangent_and_cannot_pass(native):
    native[2]["route"][-1]["x"] = 0.
    with pytest.raises(summary.EvidenceError, match="missing terminal route tangent"):
        analyze(native)


@pytest.mark.parametrize("interior_duplicate", [False, True])
def test_preexisting_numerical_output_is_exactly_equal_to_pre_fix_source(native, tmp_path, interior_duplicate):
    # HH_260906 - Preserve old valid-route outputs against the actual historical implementation, not a rewritten expectation.
    source = subprocess.check_output(["git", "show", "3535d97:scripts/e2e/summarize_carla_goal_stop_trials.py"], cwd=Path(__file__).resolve().parents[1])
    path = tmp_path / "old_summary.py"
    path.write_bytes(source)
    spec = importlib.util.spec_from_file_location("hh_pre_duplicate_endpoint_summary", path)
    old = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    if interior_duplicate:
        native[2]["route"].insert(1, copy.deepcopy(native[2]["route"][0]))
    bounds = summary.source_bounds()
    assert summary.analyze_native(*native, bounds) == old.analyze_native(*native, bounds)


def _write(path, value, jsonl=False):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row) + "\n" for row in value) if jsonl else json.dumps(value) + "\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def trials(tmp_path, native):
    root = tmp_path / "trials"
    for index, successful in ((1, False), (2, True)):
        trial = root / f"run_{index:03d}"
        directory = trial / ("episode" if successful else "episode.partial")
        route_sha = _write(directory / "route.json", native[2])
        _write(directory / "states.jsonl", native[0], jsonl=True)
        _write(directory / "camera_frames.jsonl", native[1], jsonl=True)
        _write(directory / "manifest.json", {"status": "complete" if successful else "failed",
            "provenance": {"route_sha256": route_sha, "collector_sha256": "1" * 64, "goal_stop_helper_sha256": "2" * 64},
            "capture_contract": {"physics_hz": 20, "camera_hz": 10, "camera_interval_ticks": 2,
                "goal_stop_profile": native[3], "client_map_loading_allowed": False, "target_speed_kmh": 30.0},
            "result": {"state_count": len(native[0]), "camera_anchor_count": len(native[1]), "goal_reached": True,
                "collision_event_count": 0, "lane_invasion_event_count": 0}, "cleanup": {"completed": True}})
        _write(trial / "owner_result.json", {"exit_code": 0 if successful else 1,
            "learned_model_control": False, "vehicle_control_approved": False})
        _write(trial / "owner_started.json", {"server_pid": 123, "server_pgid": 123, "port": 2100,
            "map": "Town07", "quality": "Low", "route_sha256": route_sha})
        _write(trial / "lifecycle/stopped.json", {"status": "PASS", "mode": "stopped", "stage": "stopped",
            "read_only": True, "port_released": True, "owner_process_state": None, "owner_pid": 123,
            "owner_pgid": 123, "generation_id": "expert_123", "host": "127.0.0.1", "port": 2100, "expected_map": "Town07"})
    return root


def test_all_finalized_failures_stay_in_denominator_and_plots_are_real_data(trials, tmp_path):
    pytest.importorskip("matplotlib")
    output = tmp_path / "summary"
    result = summary.summarize_trials(trials, output)
    assert result["total_discovered_and_included_trials"] == 2
    assert result["failed_trial_count"] == result["raw_quality_candidate_count"] == 1
    assert result["trials"][0]["status"] == "FAILED_RAW_QA_OR_CAPTURE"
    assert result["trials"][1]["status"] == "RAW_QA_CANDIDATE_ONLY"
    assert result["trials"][1]["independent_qa"]["maximum_measured_speed_kmh"] == 0.0
    assert result["trials"][1]["nominal_target_speed_kmh"] == 30.0
    assert result["scope"]["camera_pixels_opened"] is False
    assert str(trials) not in json.dumps(result)
    for path in output.glob("*.png"):
        assert path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert len(list(output.glob("*.png"))) == 2
    for line in (output / "SHA256SUMS").read_text().splitlines():
        expected, name = line.split("  ")
        assert hashlib.sha256((output / name).read_bytes()).hexdigest() == expected
    with pytest.raises(summary.EvidenceError, match="already exists"):
        summary.summarize_trials(trials, output)


def test_active_trial_cannot_be_silently_omitted(trials, tmp_path):
    (trials / "run_003").mkdir()
    with pytest.raises(summary.EvidenceError, match="INCOMPLETE"):
        summary.summarize_trials(trials, tmp_path / "summary")
    assert not (tmp_path / "summary").exists()


@pytest.mark.parametrize("mutation,match", [
    (lambda p: p.update(port_released=False), "stopped proof"),
    (lambda p: p.update(owner_pid=321), "ownership mismatch"),
    (lambda p: p.update(expected_map="Town01"), "endpoint mismatch"),
])
def test_stopped_proof_must_match_owned_generation(trials, mutation, match):
    path = trials / "run_001/lifecycle/stopped.json"
    value = json.loads(path.read_text())
    mutation(value)
    _write(path, value)
    with pytest.raises(summary.EvidenceError, match=match):
        summary.summarize_trial(trials / "run_001", summary.source_bounds())


def test_route_hash_and_planned_source_mismatch_fail_closed(trials):
    trial = trials / "run_001"
    path = trial / "episode.partial/route.json"
    path.write_text(path.read_text() + " ")
    with pytest.raises(summary.EvidenceError, match="route evidence hash"):
        summary.summarize_trial(trial, summary.source_bounds())


def test_empty_finalized_failed_capture_is_retained_not_reported_as_success(trials):
    trial = trials / "run_001"
    for name in ("states.jsonl", "camera_frames.jsonl"):
        (trial / "episode.partial" / name).write_text("")
    result, timeline = summary.summarize_trial(trial, summary.source_bounds())
    assert timeline is None
    assert result["raw_quality_candidate"] is False
    assert "nonempty_native_states" in result["independent_qa"]["failed_flags"]


def test_no_torch_import_is_needed_to_read_literal_bound_sources():
    source = (summary.ROOT / "scripts/e2e/summarize_carla_goal_stop_trials.py").read_text()
    assert "import torch" not in source and "from portable_e2e.model import" not in source
    assert summary.source_bounds()["physical_decoder"]["maximum_acceleration_mps2"] == 2.9
