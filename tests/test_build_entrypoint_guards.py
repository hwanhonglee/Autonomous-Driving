from pathlib import Path
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_SCRIPTS = (
    REPO_ROOT / "scripts/e2e/build.sh",
    REPO_ROOT / "scripts/e2e/build_full.sh",
)


@pytest.mark.parametrize("script", BUILD_SCRIPTS)
def test_build_help_exits_before_any_workspace_side_effect(script: Path) -> None:
    result = subprocess.run(
        ["bash", str(script), "--help"],
        text=True,
        capture_output=True,
        check=False,
        timeout=5,
    )

    assert result.returncode == 0
    assert "Usage:" in result.stdout
    assert "workspace runtime lock" not in result.stdout + result.stderr


@pytest.mark.parametrize("script", BUILD_SCRIPTS)
def test_build_unknown_argument_is_rejected_before_side_effects(script: Path) -> None:
    result = subprocess.run(
        ["bash", str(script), "--definitely-not-supported"],
        text=True,
        capture_output=True,
        check=False,
        timeout=5,
    )

    assert result.returncode == 2
    assert "accepts no arguments" in result.stderr


@pytest.mark.parametrize("script", BUILD_SCRIPTS)
def test_argument_guard_precedes_runtime_lock_and_setup(script: Path) -> None:
    source = script.read_text(encoding="utf-8")

    guard = source.index("if (( $# > 0 )); then")
    runtime_lock = source.index("e2e_acquire_workspace_runtime_lock")
    assert guard < runtime_lock
