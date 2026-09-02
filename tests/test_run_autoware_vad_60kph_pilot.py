from __future__ import annotations

from pathlib import Path
import subprocess


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts/e2e/run_autoware_vad_60kph_pilot.sh"


def test_60kph_pilot_help_and_shell_syntax() -> None:
    subprocess.run(["bash", "-n", str(SCRIPT)], check=True)
    result = subprocess.run(
        [str(SCRIPT), "--help"], check=True, text=True, capture_output=True
    )

    assert "straight-only" in result.stderr
    assert "Best-Effort depth-1" in result.stderr
    assert "localhost-only" in result.stderr
    assert "real-vehicle-ready" in result.stderr


def test_60kph_pilot_pins_owned_lifecycle_route_and_evidence_contract() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert 'e2e_acquire_workspace_runtime_lock "60 kph exploratory pilot"' in source
    assert "setsid scripts/e2e/run_carla_map.sh Town06" in source
    assert "--min-distance 430.0 --max-distance 460.0" in source
    assert "--preferred-distance 445.0" in source
    assert "--straight-capacity-profile town06_60kph_straight_pilot_v1" in source
    assert "--physical-straight-profile speed_60kph_straight_pilot" in source
    assert 'capacity.get("profile_id") != "town06_60kph_straight_pilot_v1"' in source
    assert 'physical.get("profile_id") != "speed_60kph_straight_pilot"' in source
    assert "route.resolve(strict=True)" in source
    assert 'physical_preflight.get("status") != "PASS"' in source
    assert "--recommended --speed-60kph-pilot --camera-source-5hz" in source
    assert "--visualize --capture-desktop" in source
    assert "AUTOWARE_E2E_CARLA_OWNER_PGID" in source
    assert "--expect-stopped" in source
    recorded_source = (
        ROOT / "scripts/e2e/run_recorded_route_trial.sh"
    ).read_text(encoding="utf-8")
    assert "analyze_actuation_map_coverage.py" in recorded_source
    assert "--allow-target-envelope-beyond-axis" in recorded_source
    assert '"${attempt_root}/actuation_map_coverage.json"' in source
    assert 'read_object("actuation_map_runtime_coverage.json")' in source
    assert '"actuation_velocity_axis_clamping_observed"' in source
    assert '"velocity_axis_clamping_observed"' in source
    assert "real_vehicle_ready\": False" in source
    assert "camera_source_5hz_validation.json" in source
    assert "_camera_source_5hz_evidence" in source
    assert "CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_DEPTH1_CONTRACT" in source
    assert "analyze_pilot_runtime_load.py" in source
    assert 'read_object("longitudinal_response.json")' in source
    assert 'read_object("runtime_load_analysis.json")' in source
    assert '"runtime_load_analysis_exit_status"' in source
    assert '"runtime_pattern"' in source
    assert 'runtime_load.get("schema_version") != 3' in source
    assert 'runtime_load.get("status") != "complete"' in source
    assert "not isinstance(runtime_problems, list)" in source
    assert "or bool(runtime_problems)" in source
    assert 'runtime_support.get("classification_supported") is not True' in source
    assert 'runtime_support.get("classification") != runtime_classification' in source
    assert 'runtime_claims.get("camera_delivery_pattern") is True' in source
    assert 'runtime_claims.get("host_wide_saturation_excluded") is True' in source
    assert '"runtime_load_classification_supported"' in source
    assert 'view.get("vehicle_centered") is not True' in source
    assert '"status": "PASS" if not problems else "FAILED"' in source
    assert "telemetry_cleanup_status" in source
    assert "pidstat -urd -h -p ALL 1" in source
    assert "pilot_failure.json" in source
    assert "pilot_complete=true" in source
