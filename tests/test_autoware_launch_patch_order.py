from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_REPOSITORY = ROOT / "src" / "launcher" / "autoware_launch"


def _apply(checkout: Path, patch_name: str) -> None:
    subprocess.run(
        ["git", "apply", str(ROOT / "patches" / patch_name)],
        cwd=checkout,
        check=True,
        capture_output=True,
        text=True,
    )


def test_clean_launch_patch_sequence_supports_normal_and_smart_mpc(tmp_path: Path) -> None:
    if not (LAUNCH_REPOSITORY / ".git").is_dir():
        pytest.skip("Imported autoware_launch source is not available")

    archive = subprocess.check_output(
        ["git", "-C", str(LAUNCH_REPOSITORY), "archive", "HEAD"]
    )
    subprocess.run(
        ["tar", "-x", "-C", str(tmp_path)],
        input=archive,
        check=True,
    )

    _apply(tmp_path, "autoware_launch_lateral_param_override.patch")
    _apply(tmp_path, "autoware_launch_vehicle_cmd_gate_param_override.patch")
    _apply(tmp_path, "autoware_launch_smart_mpc_nominal_ilqr_preset.patch")
    _apply(tmp_path, "autoware_launch_smart_mpc_runtime_param.patch")
