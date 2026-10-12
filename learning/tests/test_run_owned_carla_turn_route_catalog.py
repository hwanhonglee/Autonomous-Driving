"""HH_260906 - Guard the catalog-only owner entrypoint without starting CARLA."""

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools/e2e/run_owned_carla_turn_route_catalog.sh"


def _run(*arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", str(SCRIPT), *arguments],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )


def test_shell_syntax_and_help_do_not_start_carla() -> None:
    syntax = subprocess.run(
        ["bash", "-n", str(SCRIPT)],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )
    assert syntax.returncode == 0, syntax.stderr
    help_result = _run("--help")
    assert help_result.returncode == 0
    assert "town01|town01_v5|town01_multi|town03|town05_opt" in help_result.stdout


def test_unlisted_map_and_extra_arguments_fail_before_launch() -> None:
    for arguments in ((), ("town04",), ("town01", "--allow-map-load")):
        result = _run(*arguments)
        assert result.returncode == 2
        assert "Usage:" in result.stderr


def test_catalog_only_policy_is_fixed_in_owner_source() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    assert "--scenarios left,right --seeds \"${seeds}\" --pairs-per-seed 8" in source
    assert '--min-distance 200 --max-distance 230 --preferred-distance "${preferred_distance}"' in source
    assert "--physical-turn-profile speed_30kph" in source
    assert "--active-server-profile packaged_0915" in source
    assert "port=2100" in source
    assert "quality=Epic" in source
    assert "drive_long_route_town01_train_catalog_v4" in source
    assert "drive_long_route_town01_train_catalog_v5" in source
    assert "preferred_distance=160" in source
    assert "seeds=4,5" in source
    assert 'output_root="${root}/storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v1"' not in source
    assert 'output_root="${root}/storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v2"' not in source
    assert 'output_root="${root}/storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v3"' not in source
    assert "drive_long_route_town03_dev_catalog_v1" in source
    assert "drive_long_route_town05_opt_dev_catalog_v1" in source
    assert "role=PROVISIONAL_DEV" in source
    assert "--map-load-settle-sec 0" in source
    assert "[[ -e \"${output_root}\" || -L \"${output_root}\" ]]" in source
    assert "e2e_acquire_workspace_runtime_lock" in source
    assert "probe after_catalog" in source
    assert "grep -a -q '^CARLA_READY '" in source
    assert "rg -q '^CARLA_READY '" not in source
    assert "probe stopped --expect-stopped" in source
    assert "collect_carla_vad_expert.py" not in source
    assert "--allow-map-load" not in source
    assert "dataset_admission_granted': False" in source


def test_multi_maneuver_argv_is_exact_and_inspection_cannot_launch_carla() -> None:
    result = subprocess.run(
        ["bash", str(SCRIPT), "town01_multi", "--inspect-catalog-argv"],
        cwd=ROOT,
        capture_output=True,
        timeout=10,
        check=False,
    )
    assert result.returncode == 0, result.stderr.decode(errors="replace")
    assert result.stdout.endswith(b"\0")
    argv = [argument.decode() for argument in result.stdout[:-1].split(b"\0")]
    output_root = (
        ROOT
        / "storage/experiments/training/2026-09-27/"
        "drive_long_route_town01_multi_train_catalog_v1"
    )
    assert argv == [
        "--map-id", "town01", "--active-server-profile", "packaged_0915",
        "--host", "127.0.0.1", "--port", "2100", "--map-load-settle-sec", "0",
        "--output-root", str(output_root),
        "--scenarios", "right", "--seeds", "6,7", "--pairs-per-seed", "8",
        "--min-distance", "240", "--max-distance", "300",
        "--preferred-distance", "160", "--sampling-resolution", "1",
        "--max-traces", "100000",
    ]
    source = SCRIPT.read_text(encoding="utf-8")
    assert source.index("if (( inspect_catalog_argv != 0 )); then") < source.index(
        "e2e_acquire_workspace_runtime_lock"
    )
    assert source.index("if (( inspect_catalog_argv != 0 )); then") < source.index(
        "setsid bash tools/e2e/run_carla_map.sh"
    )
    assert "--physical-turn-profile" not in argv
    assert "--allow-map-load" not in argv
    assert "--endpoint-waypoint-spacing-m" not in argv
    assert all(not argument.startswith("--minimum-turn-") for argument in argv)
    assert all(not argument.startswith("--maximum-turn-") for argument in argv)
    from scripts.e2e.prepare_carla_expert_route_catalog import parse_args

    parsed = parse_args(argv)
    assert not parsed.allow_map_load
    assert parsed.physical_turn_profile is None
    assert parsed.scenarios == ("right",)
    assert parsed.seeds == (6, 7)


def test_old_catalog_argv_keeps_whole_route_physical_turn_gate() -> None:
    for mode in ("town01", "town01_v5", "town03", "town05_opt"):
        result = subprocess.run(
            ["bash", str(SCRIPT), mode, "--inspect-catalog-argv"],
            cwd=ROOT,
            capture_output=True,
            timeout=10,
            check=False,
        )
        assert result.returncode == 0, result.stderr.decode(errors="replace")
        argv = [argument.decode() for argument in result.stdout[:-1].split(b"\0")]
        assert argv[argv.index("--scenarios") + 1] == "left,right"
        assert argv[argv.index("--physical-turn-profile") + 1] == "speed_30kph"
        assert argv[argv.index("--min-distance") + 1] == "200"
        assert argv[argv.index("--max-distance") + 1] == "230"
