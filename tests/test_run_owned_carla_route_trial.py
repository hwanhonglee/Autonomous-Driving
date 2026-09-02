from __future__ import annotations

from pathlib import Path
import subprocess

import yaml


ROOT = Path(__file__).parents[1]
RUNNER = ROOT / "scripts/e2e/run_owned_carla_route_trial.sh"


def test_owned_route_runner_has_fail_closed_health_retry_contract() -> None:
    source = RUNNER.read_text(encoding="utf-8")

    assert "e2e_acquire_workspace_runtime_lock" in source
    assert "refusing to overlap owned CARLA generations" in source
    assert "--runtime-health-gate --runtime-health-timeout 45" in source
    assert '[[ "${health_status}" != "FAIL" ]]' in source
    assert "Trial failed outside the runtime-health gate; no automatic retry" in source
    assert "RUNTIME_HEALTH_RETRY" in source
    assert "route_sha256" in source
    assert '"${lifecycle_dir}/owner_cleanup_health.json"' in source
    assert '"${active_attempt}/owner_preflight_health.json"' not in source
    assert "e2e_stop_owned_process_group" in source


def test_owned_route_runner_resolves_case_sensitive_and_custom_bundle_names_from_manifest() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    matrix_path = ROOT / "scripts/e2e/autoware_vad_town_matrix.yaml"
    matrix = yaml.safe_load(matrix_path.read_text(encoding="utf-8"))
    bundles = matrix["validated_full_map_bundles"]

    assert 'full_map_manifest="${root}/scripts/e2e/autoware_vad_town_matrix.yaml"' in source
    assert 'full_map_path="${root}/data/maps/${map_id}_full"' not in source
    expected = {
        "Town07": ROOT / "data/maps/Town07_full",
        "Town03": ROOT / "data/maps/Town03_full",
        "C_track_1_0_7": ROOT / "data/maps/C_track_1_0_7_xodr_full",
    }
    by_canonical_name = {
        spec["canonical_carla_map"].rsplit("/", 1)[-1]: (
            matrix_path.parent / spec["path"]
        ).resolve()
        for spec in bundles.values()
    }
    for map_name, expected_path in expected.items():
        assert by_canonical_name[map_name] == expected_path.resolve()
        assert (expected_path / "map_bundle.json").is_file()


def test_owned_route_runner_help_is_read_only() -> None:
    completed = subprocess.run(
        [str(RUNNER), "--help"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0
    assert "fresh owned CARLA generations" in completed.stderr
    assert "--max-health-retries" in completed.stderr


def test_owned_route_runner_rejects_invalid_owner_options_before_output() -> None:
    completed = subprocess.run(
        [str(RUNNER), "--quality", "Medium", "unused-output", "unused-route"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "quality must be Low or Epic" in completed.stderr
