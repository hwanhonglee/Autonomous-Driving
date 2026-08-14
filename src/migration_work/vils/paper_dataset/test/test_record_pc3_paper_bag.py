#!/usr/bin/env python3
# cspell:ignore getuid vils
"""Hardware-free regression tests for the fail-closed PC3 paper bag recorder."""

# HH_260814 - Exercise recorder integrity gates with fake ros2 and no ROS graph, sensor, or vehicle.

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SOURCE_DIR = Path(__file__).resolve().parents[1]


FAKE_ROS2 = r'''#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys

arguments = sys.argv[1:]
log = Path(os.environ["FAKE_ROS2_LOG"])
with log.open("a", encoding="utf-8") as stream:
    stream.write(" ".join(arguments[:2]) + "\n")

if arguments[:2] == ["bag", "record"]:
    output = Path(arguments[arguments.index("--output") + 1])
    output.mkdir(parents=True, exist_ok=False)
    (output / "partial_0.mcap").write_bytes(b"partial-or-closed-test-data")
    mode = os.environ.get("FAKE_ROS2_MODE", "success")
    if mode == "metadata_missing":
        raise SystemExit(0)
    duration_ns = 1_000_000_000 if mode == "early" else 10_000_000_000
    localization_count = 0 if mode == "empty" else 10
    tf_count = 0 if mode in {"empty", "missing_continuous"} else 10
    information = {
        "version": 8,
        "storage_identifier": "mcap",
        "duration": {"nanoseconds": duration_ns},
        "message_count": localization_count + tf_count,
        "topics_with_message_count": [
            {
                "topic_metadata": {
                    "name": "/localization/kinematic_state",
                    "type": "test_msgs/msg/State",
                    "serialization_format": "cdr",
                },
                "message_count": localization_count,
            },
            {
                "topic_metadata": {
                    "name": "/tf",
                    "type": "tf2_msgs/msg/TFMessage",
                    "serialization_format": "cdr",
                },
                "message_count": tf_count,
            },
        ],
    }
    (output / "metadata.yaml").write_text(
        json.dumps({"rosbag2_bagfile_information": information}), encoding="utf-8"
    )
    raise SystemExit(0)

if arguments[:2] == ["bag", "info"]:
    print("fake ros2 bag info")
    raise SystemExit(0)

raise SystemExit(64)
'''


class RecordPc3PaperBagTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.vils_dir = self.root / "vils"
        self.paper_dir = self.vils_dir / "paper_dataset"
        self.config_dir = self.vils_dir / "config"
        self.paper_dir.mkdir(parents=True)
        self.config_dir.mkdir()

        for name in (
            "record_pc3_paper_bag.sh",
            "pc3_paper_core_topics.txt",
            "pc3_paper_core_qos.yaml",
        ):
            shutil.copy2(SOURCE_DIR / name, self.paper_dir / name)
        self.script = self.paper_dir / "record_pc3_paper_bag.sh"
        self.dds = self.config_dir / "cyclonedds_vehicle_domain.xml"
        self.dds.write_text("<CycloneDDS/>\n", encoding="utf-8")

        self.fake_bin = self.root / "fake-bin"
        self.fake_bin.mkdir()
        fake_ros2 = self.fake_bin / "ros2"
        fake_ros2.write_text(FAKE_ROS2, encoding="utf-8")
        fake_ros2.chmod(0o755)
        fake_df = self.fake_bin / "df"
        fake_df.write_text("#!/bin/sh\nprintf 'Avail\\n999999999999\\n'\n", encoding="utf-8")
        fake_df.chmod(0o755)
        self.ros2_log = self.root / "ros2.log"

        self.run_dir = self.root / "evidence" / "20260814T000000Z-TEST_RUN_001"
        manifest = self.run_dir / "manifest"
        (self.run_dir / "rosbag").mkdir(parents=True)
        (self.run_dir / "events" / "recording_locks").mkdir(parents=True)
        manifest.mkdir()
        self.run_dir.chmod(0o700)
        shutil.copy2(
            self.paper_dir / "pc3_paper_core_topics.txt",
            manifest / "pc3_paper_core_topics.txt",
        )
        shutil.copy2(
            self.paper_dir / "pc3_paper_core_qos.yaml",
            manifest / "pc3_paper_core_qos.yaml",
        )
        self.run_manifest = {
            "schema_version": 1,
            "run_id": "TEST_RUN_001",
            "run_directory": str(self.run_dir),
            "uid": os.getuid(),
            "host": "pc3-test",
        }
        self.trial_manifest = {
            "schema_version": 1,
            "status": "PREPARED_NOT_STARTED",
            "paper_sample_eligible": "pending",
            "run_id": "TEST_RUN_001",
            "scenario_id": "C_TRACK_STATIC_01",
            "condition_id": "B0_REAL_ONLY",
            "replicate_id": 1,
            "planned_duration_sec": 10,
            "host": "pc3-test",
            "host_role": "PC3_MAP_LOCALIZATION_PHYSICAL_LIDAR",
            "recorder_profiles": {
                "core": "manifest/pc3_paper_core_topics.txt",
                "lidar": "manifest/pc3_paper_lidar_topics.txt",
            },
        }
        self._write_manifests()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _write_manifests(self) -> None:
        manifest = self.run_dir / "manifest"
        (manifest / "run.json").write_text(
            json.dumps(self.run_manifest), encoding="utf-8"
        )
        (manifest / "paper_trial.json").write_text(
            json.dumps(self.trial_manifest), encoding="utf-8"
        )

    def _run(self, mode: str = "success") -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment.update(
            {
                "PATH": f"{self.fake_bin}{os.pathsep}{environment['PATH']}",
                "ROS_DOMAIN_ID": "10",
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "ROS_LOCALHOST_ONLY": "0",
                "CYCLONEDDS_URI": f"file://{self.dds}",
                "FAKE_ROS2_MODE": mode,
                "FAKE_ROS2_LOG": str(self.ros2_log),
            }
        )
        return subprocess.run(
            (str(self.script), str(self.run_dir), "core", "10"),
            check=False,
            capture_output=True,
            text=True,
            timeout=15,
            env=environment,
        )

    def _outputs(self) -> list[Path]:
        return sorted((self.run_dir / "rosbag").glob("pc3_core_*"))

    @staticmethod
    def _sha256(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def test_success_writes_validated_output_and_two_completion_markers(self) -> None:
        completed = self._run()
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertEqual(len(self._outputs()), 1)
        output = self._outputs()[0]
        validation = json.loads(
            (output / "recording_validation.json").read_text(encoding="utf-8")
        )
        self.assertEqual(validation["message_count"], 20)
        self.assertEqual(validation["actual_duration_sec"], 10.0)
        self.assertTrue((output / "recording_complete.json").is_file())
        self.assertTrue(
            (self.run_dir / "events" / "pc3_core_recording_complete.json").is_file()
        )

    def test_metadata_missing_is_nonzero_and_preserves_partial_output(self) -> None:
        completed = self._run("metadata_missing")
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("without a safe metadata.yaml", completed.stderr)
        self.assertEqual(len(self._outputs()), 1)
        self.assertTrue((self._outputs()[0] / "partial_0.mcap").is_file())
        self.assertFalse((self._outputs()[0] / "recording_complete.json").exists())
        retry = self._run()
        self.assertNotEqual(retry.returncode, 0)
        self.assertIn("already has a completion marker or output", retry.stderr)
        self.assertEqual(
            self.ros2_log.read_text(encoding="utf-8").splitlines().count("bag record"), 1
        )

    def test_empty_recording_is_rejected_and_preserved(self) -> None:
        completed = self._run("empty")
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("message_count is missing or zero", completed.stderr)
        self.assertEqual(len(self._outputs()), 1)
        self.assertTrue((self._outputs()[0] / "metadata.yaml").is_file())
        self.assertFalse((self._outputs()[0] / "recording_complete.json").exists())

    def test_early_recording_is_rejected_and_preserved(self) -> None:
        completed = self._run("early")
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("recorded duration 1.000s", completed.stderr)
        self.assertEqual(len(self._outputs()), 1)
        self.assertTrue((self._outputs()[0] / "metadata.yaml").is_file())
        self.assertFalse((self._outputs()[0] / "recording_complete.json").exists())

    def test_missing_required_continuous_topic_count_is_rejected(self) -> None:
        completed = self._run("missing_continuous")
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("continuous topic /tf has 0 messages", completed.stderr)
        self.assertEqual(len(self._outputs()), 1)
        self.assertFalse((self._outputs()[0] / "recording_complete.json").exists())

    def test_sequential_duplicate_profile_is_rejected_before_ros2(self) -> None:
        first = self._run()
        self.assertEqual(first.returncode, 0, first.stderr)
        second = self._run()
        self.assertNotEqual(second.returncode, 0)
        self.assertIn("already has a completion marker or output", second.stderr)
        log_lines = self.ros2_log.read_text(encoding="utf-8").splitlines()
        self.assertEqual(log_lines.count("bag record"), 1)

    def test_source_and_frozen_profile_mismatch_is_rejected(self) -> None:
        frozen = self.run_dir / "manifest" / "pc3_paper_core_topics.txt"
        frozen.write_text(frozen.read_text(encoding="utf-8") + "# tampered\n", encoding="utf-8")
        completed = self._run()
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("source and frozen run profile differ", completed.stderr)
        self.assertFalse(self.ros2_log.exists())

    def test_manifest_run_id_mismatch_is_rejected_before_ros2(self) -> None:
        self.trial_manifest["run_id"] = "DIFFERENT_RUN"
        self._write_manifests()
        completed = self._run()
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("run_id differs", completed.stderr)
        self.assertFalse(self.ros2_log.exists())

    def test_sealed_hash_detects_coordinated_frozen_profile_tampering(self) -> None:
        manifest = self.run_dir / "manifest"
        topic = manifest / "pc3_paper_core_topics.txt"
        qos = manifest / "pc3_paper_core_qos.yaml"
        self.trial_manifest["profile_hashes"] = {
            "core": {
                "topics": {
                    "path": "manifest/pc3_paper_core_topics.txt",
                    "sha256": self._sha256(topic),
                },
                "qos": {
                    "path": "manifest/pc3_paper_core_qos.yaml",
                    "sha256": self._sha256(qos),
                },
            }
        }
        self._write_manifests()
        for source, frozen in (
            (self.paper_dir / "pc3_paper_core_topics.txt", topic),
            (self.paper_dir / "pc3_paper_core_qos.yaml", qos),
        ):
            altered = source.read_text(encoding="utf-8") + "# coordinated-tamper\n"
            source.write_text(altered, encoding="utf-8")
            frozen.write_text(altered, encoding="utf-8")
        completed = self._run()
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("hash differs", completed.stderr)
        self.assertFalse(self.ros2_log.exists())


if __name__ == "__main__":
    unittest.main()
