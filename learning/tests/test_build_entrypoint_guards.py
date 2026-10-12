# HH_260906 - Locate the repository independently of the relocated test directory.
from pathlib import Path
import subprocess

import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


REPO_ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
BUILD_SCRIPTS = (
    _hh_source_path(REPO_ROOT, "scripts/e2e/build.sh"),
    _hh_source_path(REPO_ROOT, "scripts/e2e/build_full.sh"),
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
