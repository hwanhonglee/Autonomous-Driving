from pathlib import Path
import json
import os
import subprocess

import yaml

from vad_training_test_utils import calibration_document
from vad_training_test_utils import complete_profile


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/e2e/record_vad_training_data.sh"
PROFILE = ROOT / "autoware_e2e_vad_launch/config/vad_real_data_collection.yaml"


def test_recorder_uses_minimal_ros_environment_and_auto_validation() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert "source scripts/e2e/env.sh" not in source
    assert "source /opt/ros/humble/setup.bash" in source
    assert "--compression-format zstd" in source
    assert "vad_dataset_recorder_${ROS_DOMAIN_ID}.lock" in source
    assert '"${contract_tool}" preflight' in source
    assert "validate_vad_training_bag.py" in source
    assert "render_vad_calibration_preview.py" in source


def test_dry_run_lists_six_camera_topics_without_starting_rosbag(tmp_path: Path) -> None:
    profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    calibration_path = tmp_path / "calibration.yaml"
    complete_profile(profile, calibration_path)
    calibration_path.write_text(
        yaml.safe_dump(calibration_document(profile), sort_keys=False), encoding="utf-8"
    )
    test_profile = tmp_path / "profile.yaml"
    test_profile.write_text(yaml.safe_dump(profile, sort_keys=False), encoding="utf-8")

    result = subprocess.run(
        [
            str(SCRIPT),
            "--profile",
            str(test_profile),
            "--dry-run",
            str(tmp_path / "capture"),
        ],
        check=False,
        text=True,
        capture_output=True,
        cwd=ROOT,
    )

    assert result.returncode == 0, result.stderr
    assert "ros2 bag record" in result.stdout
    for camera in (
        "CAM_FRONT",
        "CAM_BACK",
        "CAM_FRONT_LEFT",
        "CAM_BACK_LEFT",
        "CAM_FRONT_RIGHT",
        "CAM_BACK_RIGHT",
    ):
        assert f"/sensing/camera/{camera}/image_rect" in result.stdout


def test_duration_waits_for_recorder_flush_and_reaps_child(tmp_path: Path) -> None:
    profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    calibration_path = tmp_path / "calibration.yaml"
    complete_profile(profile, calibration_path)
    calibration_path.write_text(
        yaml.safe_dump(calibration_document(profile), sort_keys=False), encoding="utf-8"
    )
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=False), encoding="utf-8")

    fake_ros2 = tmp_path / "fake_ros2"
    fake_ros2.write_text(
        """#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == node && "${2:-}" == list ]]; then
  exit 0
fi
if [[ "${1:-}" != bag || "${2:-}" != record ]]; then
  exit 2
fi
shift 2
output=""
while [[ $# -gt 0 ]]; do
  if [[ "$1" == --output ]]; then output="$2"; shift 2; else shift; fi
done
mkdir -p "${output}"
echo $$ > "${output}/child.pid"
trap 'sleep 0.35; touch "${output}/flushed"; exit 130' INT TERM
while true; do sleep 0.05; done
""",
        encoding="utf-8",
    )
    fake_ros2.chmod(0o755)
    output = tmp_path / "capture"
    environment = os.environ.copy()
    environment["AUTOWARE_E2E_ROS2_BIN"] = str(fake_ros2)

    result = subprocess.run(
        [
            str(SCRIPT),
            "--profile",
            str(profile_path),
            "--skip-preflight",
            "--duration",
            "0.2",
            str(output),
        ],
        check=False,
        text=True,
        capture_output=True,
        cwd=ROOT,
        env=environment,
        timeout=15,
    )

    assert result.returncode == 130
    assert (output / "bag/flushed").is_file()
    child_pid = int((output / "bag/child.pid").read_text(encoding="utf-8"))
    try:
        os.kill(child_pid, 0)
    except ProcessLookupError:
        pass
    else:
        raise AssertionError(f"fake recorder process {child_pid} was not reaped")


def test_early_signal_exit_is_not_accepted_as_duration_completion(tmp_path: Path) -> None:
    profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    calibration_path = tmp_path / "calibration.yaml"
    complete_profile(profile, calibration_path)
    calibration_path.write_text(
        yaml.safe_dump(calibration_document(profile), sort_keys=False), encoding="utf-8"
    )
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=False), encoding="utf-8")

    fake_ros2 = tmp_path / "fake_early_ros2"
    fake_ros2.write_text(
        """#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == node && "${2:-}" == list ]]; then exit 0; fi
shift 2
output=""
while [[ $# -gt 0 ]]; do
  if [[ "$1" == --output ]]; then output="$2"; shift 2; else shift; fi
done
mkdir -p "${output}"
printf '%s\n' 'rosbag2_bagfile_information:' '  relative_file_paths: []' > "${output}/metadata.yaml"
exit 143
""",
        encoding="utf-8",
    )
    fake_ros2.chmod(0o755)
    output = tmp_path / "capture"
    environment = os.environ.copy()
    environment["AUTOWARE_E2E_ROS2_BIN"] = str(fake_ros2)

    result = subprocess.run(
        [
            str(SCRIPT),
            "--profile",
            str(profile_path),
            "--skip-preflight",
            "--duration",
            "5",
            str(output),
        ],
        check=False,
        text=True,
        capture_output=True,
        cwd=ROOT,
        env=environment,
        timeout=15,
    )

    manifest = json.loads((output / "session_manifest.json").read_text(encoding="utf-8"))
    assert result.returncode == 143
    assert manifest["status"] == "failed"
    assert not (output / "training_data_report.json").exists()
