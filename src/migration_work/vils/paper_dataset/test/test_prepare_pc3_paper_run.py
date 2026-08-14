#!/usr/bin/env python3
"""Hardware-free tests for PC3 paper-run identity and frozen acquisition inputs."""

# HH_260814 - Test validation and exclusive writes without launching ROS, hardware, or vehicle processes.

from __future__ import annotations

import importlib.util
import json
from argparse import Namespace
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / "prepare_pc3_paper_run.py"
SPEC = importlib.util.spec_from_file_location("prepare_pc3_paper_run", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class PreparePc3PaperRunTest(unittest.TestCase):
    def make_args(self, root: Path, **overrides: object) -> Namespace:
        values: dict[str, object] = {
            "run_id": "VILS_B0_001",
            "scenario_id": "C_TRACK_STATIC_01",
            "condition_id": "B0_REAL_ONLY",
            "replicate_id": 1,
            "stage": "1.5",
            "mode": "real_only",
            "random_seed": 1001,
            "planned_duration_sec": 60,
            "evidence_root": root,
            "map_dir": root / "map",
            "autoware_root": root / "autoware",
            "ros2_root": root / "ros2_ws",
        }
        values.update(overrides)
        return Namespace(**values)

    def collector_success(self, root: Path, *, manifest_run_id: str = "VILS_B0_001"):
        def run(command: tuple[str, ...], **_: object) -> subprocess.CompletedProcess[str]:
            run_dir = root / "20260814T000000Z-VILS_B0_001"
            manifest_dir = run_dir / "manifest"
            manifest_dir.mkdir(parents=True, mode=0o700)
            (run_dir / "events").mkdir(mode=0o700)
            contract = Path(command[command.index("--contract") + 1])
            topics = Path(command[command.index("--topics") + 1])
            (manifest_dir / "pc3_vils_contract.yaml").write_bytes(contract.read_bytes())
            (manifest_dir / "pc3_vils_bag_topics.txt").write_bytes(topics.read_bytes())
            (manifest_dir / "run.json").write_text(
                json.dumps(
                    {
                        "run_id": manifest_run_id,
                        "run_directory": str(run_dir),
                    }
                ),
                encoding="utf-8",
            )
            return subprocess.CompletedProcess(command, 0, f"{run_dir}\n", "")

        return run

    def test_identifier_validation(self) -> None:
        self.assertEqual(MODULE.validate_id("run_id", "VILS_B0.001"), "VILS_B0.001")
        for invalid in ("", "../escape", "contains space", "a" * 65):
            with self.assertRaises(MODULE.PreparationError):
                MODULE.validate_id("run_id", invalid)

    def test_exclusive_json_is_owner_only_and_non_overwriting(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "trial.json"
            MODULE.exclusive_json(target, {"run_id": "TEST_001"})
            self.assertEqual(target.stat().st_mode & 0o777, 0o600)
            self.assertEqual(json.loads(target.read_text(encoding="utf-8"))["run_id"], "TEST_001")
            with self.assertRaises(FileExistsError):
                MODULE.exclusive_json(target, {"run_id": "TEST_002"})

    def test_exclusive_copy_is_owner_only_and_non_overwriting(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.txt"
            target = root / "target.txt"
            source.write_text("frozen\n", encoding="utf-8")
            MODULE.exclusive_copy(source, target)
            self.assertEqual(target.stat().st_mode & 0o777, 0o600)
            self.assertEqual(target.read_text(encoding="utf-8"), "frozen\n")
            with self.assertRaises(FileExistsError):
                MODULE.exclusive_copy(source, target)

    def test_stage_mode_matrix(self) -> None:
        accepted = {
            "1": "real_only",
            "1.5": "real_only",
            "2": "shadow",
            "3": "validated_candidate",
            "4": "hybrid_optional",
            "5": "hybrid_optional",
            "sil": "sil",
        }
        for stage, mode in accepted.items():
            with self.subTest(stage=stage, mode=mode):
                self.assertEqual(MODULE.validate_stage_mode(stage, mode), (stage, mode))
        for stage, mode in (("1.5", "shadow"), ("4", "hybrid_required"), ("sil", "real_only")):
            with self.subTest(stage=stage, mode=mode):
                with self.assertRaises(MODULE.PreparationError):
                    MODULE.validate_stage_mode(stage, mode)

    def test_mocked_end_to_end_preserves_collector_inputs_and_freezes_hashes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            args = self.make_args(root)
            with mock.patch.object(
                MODULE.subprocess, "run", side_effect=self.collector_success(root)
            ) as run_mock:
                run_dir = MODULE.prepare(args)

            command = run_mock.call_args.args[0]
            vils_dir = SCRIPT.parents[1]
            self.assertEqual(
                Path(command[command.index("--contract") + 1]),
                vils_dir / "pc3_vils_contract.yaml",
            )
            self.assertEqual(
                Path(command[command.index("--topics") + 1]),
                vils_dir / "pc3_vils_bag_topics.txt",
            )
            manifest_dir = run_dir / "manifest"
            self.assertEqual(
                (manifest_dir / "pc3_vils_contract.yaml").read_bytes(),
                (vils_dir / "pc3_vils_contract.yaml").read_bytes(),
            )
            self.assertEqual(
                (manifest_dir / "pc3_vils_bag_topics.txt").read_bytes(),
                (vils_dir / "pc3_vils_bag_topics.txt").read_bytes(),
            )
            trial = json.loads((manifest_dir / "paper_trial.json").read_text(encoding="utf-8"))
            self.assertEqual(trial["expected_recorder_profiles"], ["core"])
            for section in ("paper_contract", "dds"):
                record = trial["profile_hashes"][section]
                self.assertEqual(record["sha256"], MODULE.sha256_file(run_dir / record["path"]))
            for profile in ("core", "lidar"):
                for kind in ("topics", "qos"):
                    record = trial["profile_hashes"][profile][kind]
                    self.assertEqual(
                        record["sha256"], MODULE.sha256_file(run_dir / record["path"])
                    )
            self.assertEqual(
                stat.S_IMODE((run_dir / "events" / "recording_locks").stat().st_mode), 0o700
            )

    def test_duplicate_run_id_is_rejected_before_collector(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            existing = root / "different-name"
            (existing / "manifest").mkdir(parents=True)
            (existing / "manifest" / "run.json").write_text(
                json.dumps({"run_id": "VILS_B0_001"}), encoding="utf-8"
            )
            with mock.patch.object(MODULE.subprocess, "run") as run_mock:
                with self.assertRaisesRegex(MODULE.PreparationError, "run_id already exists"):
                    MODULE.prepare(self.make_args(root))
            run_mock.assert_not_called()

    def test_manifest_mismatch_marks_preparation_fail(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with mock.patch.object(
                MODULE.subprocess,
                "run",
                side_effect=self.collector_success(root, manifest_run_id="WRONG_RUN"),
            ):
                with self.assertRaisesRegex(MODULE.PreparationError, "run_id does not match"):
                    MODULE.prepare(self.make_args(root))
            abort = root / "20260814T000000Z-VILS_B0_001" / "manifest" / "abort.json"
            payload = json.loads(abort.read_text(encoding="utf-8"))
            self.assertEqual(payload["status"], "PREPARATION_FAIL")
            self.assertFalse(payload["paper_sample_eligible"])
            self.assertEqual(stat.S_IMODE(abort.stat().st_mode), 0o600)

    def test_post_init_copy_failure_marks_preparation_fail(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            real_copy = MODULE.exclusive_copy
            calls = 0

            def fail_second_copy(source: Path, destination: Path) -> None:
                nonlocal calls
                calls += 1
                if calls == 2:
                    raise OSError("injected freeze failure")
                real_copy(source, destination)

            with mock.patch.object(
                MODULE.subprocess, "run", side_effect=self.collector_success(root)
            ), mock.patch.object(MODULE, "exclusive_copy", side_effect=fail_second_copy):
                with self.assertRaisesRegex(MODULE.PreparationError, "post-init preparation failed"):
                    MODULE.prepare(self.make_args(root))
            abort = root / "20260814T000000Z-VILS_B0_001" / "manifest" / "abort.json"
            self.assertEqual(
                json.loads(abort.read_text(encoding="utf-8"))["status"], "PREPARATION_FAIL"
            )

    def test_zero_exit_without_output_marks_discovered_run(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)

            def collector_without_output(
                command: tuple[str, ...], **_: object
            ) -> subprocess.CompletedProcess[str]:
                completed = self.collector_success(root)(command)
                return subprocess.CompletedProcess(command, 0, "", completed.stderr)

            with mock.patch.object(
                MODULE.subprocess, "run", side_effect=collector_without_output
            ):
                with self.assertRaisesRegex(
                    MODULE.PreparationError, "did not return a run directory"
                ):
                    MODULE.prepare(self.make_args(root))
            abort = root / "20260814T000000Z-VILS_B0_001" / "manifest" / "abort.json"
            self.assertEqual(
                json.loads(abort.read_text(encoding="utf-8"))["status"], "PREPARATION_FAIL"
            )


if __name__ == "__main__":
    unittest.main()
