#!/usr/bin/env python3
"""Hardware-free regression tests for PC3 VILS evidence collection."""

# HH_260814 - Verify non-overwrite, map drift, offset parsing, redaction, and finalization offline.

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "collect_pc3_vils_evidence.py"
SPEC = importlib.util.spec_from_file_location("collect_pc3_vils_evidence", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Pc3VilsEvidenceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.evidence_root = self.root / "evidence"
        self.map_dir = self.root / "map"
        self.autoware = self.root / "autoware"
        self.ros2_ws = self.root / "ros2_ws"
        self.contract = self.root / "contract.yaml"
        self.topics = self.root / "topics.txt"
        self.map_dir.mkdir()
        self._write_map_files()
        self._create_autoware_tree()
        self._create_git_repo(self.autoware)
        self._create_git_repo(self.ros2_ws)
        self.contract.write_text("schema_version: 1\n", encoding="utf-8")
        self.topics.write_text("# test\n/localization/kinematic_state\n/tf\n", encoding="utf-8")

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _write_map_files(self) -> None:
        for index, name in enumerate(MODULE.MAP_FILES):
            (self.map_dir / name).write_text(f"map-{index}\n", encoding="utf-8")

    def _create_autoware_tree(self) -> None:
        for relative in MODULE.RUNTIME_MAP_CONFIGS:
            target = self.autoware / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text("/**:\n  ros__parameters: {}\n", encoding="utf-8")
        gnss = self.autoware / MODULE.GNSS_OFFSET_CONFIG
        gnss.parent.mkdir(parents=True, exist_ok=True)
        gnss.write_text(
            "/**:\n"
            "  ros__parameters:\n"
            "    projected_position_offset_enabled: true\n"
            "    projected_position_offset_x_m: 1.25\n"
            "    projected_position_offset_y_m: 2.5\n"
            "    projected_position_offset_z_m: 3.75\n",
            encoding="utf-8",
        )

    @staticmethod
    def _create_git_repo(path: Path) -> None:
        path.mkdir(parents=True, exist_ok=True)
        subprocess.run(("git", "init", "-q", str(path)), check=True)
        subprocess.run(("git", "-C", str(path), "config", "user.name", "test"), check=True)
        subprocess.run(
            ("git", "-C", str(path), "config", "user.email", "test@example.invalid"), check=True
        )
        marker = path / ".test-marker"
        marker.write_text("test\n", encoding="utf-8")
        subprocess.run(("git", "-C", str(path), "add", "."), check=True)
        subprocess.run(("git", "-C", str(path), "commit", "-qm", "fixture"), check=True)

    def _init(self) -> Path:
        arguments = MODULE.build_parser().parse_args(
            (
                "init",
                "--run-id",
                "TEST_001",
                "--evidence-root",
                str(self.evidence_root),
                "--map-dir",
                str(self.map_dir),
                "--autoware-root",
                str(self.autoware),
                "--ros2-root",
                str(self.ros2_ws),
                "--contract",
                str(self.contract),
                "--topics",
                str(self.topics),
            )
        )
        return MODULE.init_run(arguments)

    def _snapshot(self, run_dir: Path, phase: str) -> Path:
        arguments = MODULE.build_parser().parse_args(
            (
                "snapshot",
                "--run-dir",
                str(run_dir),
                "--phase",
                phase,
                "--skip-runtime-commands",
            )
        )
        return MODULE.snapshot_run(arguments)

    def test_end_to_end_non_overwriting_run(self) -> None:
        run_dir = self._init()
        self.assertEqual(stat.S_IMODE(run_dir.stat().st_mode), 0o700)
        before = self._snapshot(run_dir, "before")
        during = self._snapshot(run_dir, "during")
        after = self._snapshot(run_dir, "after")
        self.assertEqual(before.name, "before.json")
        self.assertEqual(during.name, "during_001.json")
        self.assertEqual(after.name, "after.json")
        with self.assertRaises(FileExistsError):
            self._snapshot(run_dir, "before")
        finalize_args = MODULE.build_parser().parse_args(("finalize", "--run-dir", str(run_dir)))
        checksum = MODULE.finalize_run(finalize_args)
        self.assertTrue(checksum.is_file())
        self.assertIn("manifest/run.json", checksum.read_text(encoding="utf-8"))
        with self.assertRaises(MODULE.EvidenceError):
            self._snapshot(run_dir, "during")

    def test_map_drift_is_recorded(self) -> None:
        run_dir = self._init()
        self._snapshot(run_dir, "before")
        changed = self.map_dir / "lanelet2_map.osm"
        changed.write_text("changed-map\n", encoding="utf-8")
        during = self._snapshot(run_dir, "during")
        payload = json.loads(during.read_text(encoding="utf-8"))
        self.assertIn("lanelet2_map.osm", payload["map_hash_drift"])

    def test_gnss_offsets_are_typed(self) -> None:
        parsed = MODULE.parse_gnss_offsets(self.autoware / MODULE.GNSS_OFFSET_CONFIG)
        self.assertTrue(parsed["values"]["projected_position_offset_enabled"])
        self.assertEqual(parsed["values"]["projected_position_offset_x_m"], 1.25)
        self.assertEqual(parsed["values"]["projected_position_offset_z_m"], 3.75)

    def test_run_id_and_topic_list_fail_closed(self) -> None:
        with self.assertRaises(MODULE.EvidenceError):
            MODULE.validate_run_id("../unsafe")
        duplicate = self.root / "duplicate_topics.txt"
        duplicate.write_text("/tf\n/tf\n", encoding="utf-8")
        with self.assertRaises(MODULE.EvidenceError):
            MODULE.read_topic_list(duplicate)

    def test_sensitive_strings_are_redacted(self) -> None:
        token = "ghp_" + "A" * 36
        basic = "Authorization: " + "Basic " + "QUJDRA=="
        credential_url = "https://" + "user" + ":" + "password" + "@example.invalid"
        value = f"{basic} {token} {credential_url}"
        redacted = MODULE.redact_text(value)
        self.assertNotIn(token, redacted)
        self.assertNotIn("QUJDRA==", redacted)
        self.assertNotIn("user:password", redacted)

    def test_runtime_environment_fails_closed(self) -> None:
        keys = (
            "ROS_DOMAIN_ID",
            "RMW_IMPLEMENTATION",
            "ROS_LOCALHOST_ONLY",
            "CYCLONEDDS_URI",
        )
        original = {key: os.environ.get(key) for key in keys}
        try:
            for key in keys:
                os.environ.pop(key, None)
            with self.assertRaises(MODULE.EvidenceError):
                MODULE.validate_reviewed_environment()
            os.environ.update(
                {
                    "ROS_DOMAIN_ID": "10",
                    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                    "ROS_LOCALHOST_ONLY": "0",
                    "CYCLONEDDS_URI": f"file://{MODULE.REVIEWED_DDS_CONFIG}",
                }
            )
            MODULE.validate_reviewed_environment()
        finally:
            for key, value in original.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value


if __name__ == "__main__":
    unittest.main()
