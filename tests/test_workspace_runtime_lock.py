from __future__ import annotations

import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
LOCK_HELPER = ROOT / "scripts/e2e/workspace_runtime_lock.sh"


def clean_environment() -> dict[str, str]:
    environment = dict(os.environ)
    environment.pop("AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD", None)
    return environment


def test_workspace_lock_is_inherited_by_direct_children() -> None:
    script = f"""
source {LOCK_HELPER}
e2e_acquire_workspace_runtime_lock parent
bash -c 'source {LOCK_HELPER}; e2e_acquire_workspace_runtime_lock child; echo CHILD_LOCK_PASS'
"""
    completed = subprocess.run(
        ["bash", "-c", script],
        check=False,
        capture_output=True,
        text=True,
        env=clean_environment(),
    )
    assert completed.returncode == 0, completed.stderr
    assert "CHILD_LOCK_PASS" in completed.stdout


def test_workspace_lock_rejects_a_second_owner() -> None:
    holder_script = f"""
source {LOCK_HELPER}
e2e_acquire_workspace_runtime_lock holder
echo HOLDER_READY
read -r release
"""
    holder = subprocess.Popen(
        ["bash", "-c", holder_script],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=clean_environment(),
    )
    try:
        assert holder.stdout is not None
        assert holder.stdout.readline().strip() == "HOLDER_READY"
        contender = subprocess.run(
            [
                "bash",
                "-c",
                f"source {LOCK_HELPER}; e2e_acquire_workspace_runtime_lock contender",
            ],
            check=False,
            capture_output=True,
            text=True,
            env=clean_environment(),
        )
        assert contender.returncode != 0
        assert "Another build or matrix owns shared Autoware E2E state" in (
            contender.stderr
        )
    finally:
        assert holder.stdin is not None
        holder.stdin.write("release\n")
        holder.stdin.flush()
        holder.communicate(timeout=5)

    assert holder.returncode == 0
