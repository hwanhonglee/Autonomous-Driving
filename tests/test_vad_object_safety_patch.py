from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "patches/autoware_vad_object_safety.manifest.json"
APPLY_SCRIPT = ROOT / "scripts/e2e/apply_vad_object_safety_patches.sh"
PROVENANCE_SCRIPT = ROOT / "scripts/e2e/vad_object_safety_build_provenance.py"
UNIVERSE = ROOT / "src/universe/autoware_universe"


def run(*args: str, cwd: Path | None = None, env=None):
    return subprocess.run(
        list(args), cwd=cwd, env=env, check=False, capture_output=True, text=True
    )


def clone_with_prerequisites(tmp_path: Path) -> Path:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    checkout = tmp_path / "autoware_universe"
    subprocess.run(
        ["git", "clone", "--shared", "--no-checkout", "-q", str(UNIVERSE), str(checkout)],
        check=True,
    )
    subprocess.run(
        [
            "git",
            "-C",
            str(checkout),
            "checkout",
            "-q",
            "--detach",
            manifest["repository"]["git_commit"],
        ],
        check=True,
    )
    for item in manifest["prerequisite_patches"]:
        subprocess.run(
            ["git", "-C", str(checkout), "apply", str(ROOT / item["path"])],
            check=True,
        )
    return checkout


def patch_environment(checkout: Path) -> dict[str, str]:
    return {
        **os.environ,
        "AUTOWARE_E2E_UNIVERSE_REPOSITORY": str(checkout),
        "AUTOWARE_E2E_VAD_OBJECT_SAFETY_MANIFEST": str(MANIFEST),
    }


def test_manifest_pins_exact_source_and_patch_contracts() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == 1
    assert manifest["contract_id"] == "autoware-vad-object-safety-v4"
    assert manifest["repository"]["git_commit"] == (
        "6e477c645efec33f7909095eea684474e97f5e3d"
    )
    assert [item["name"] for item in manifest["patches"]] == [
        "vad_object_yaw",
        "aeb_named_timeouts_oriented_bbox_vertical_overlap_and_longitudinal_roi",
    ]
    assert manifest["yaw_contract"] == {
        "training_dataset": "Bench2Drive CARLA",
        "raw_yaw": "atan2(sin_theta, cos_theta)",
        "physical_base_yaw": "normalize(-raw_yaw-pi/2)",
        "map_yaw": "normalize(physical_base_yaw+base_to_map_yaw)",
        "runtime_marker": (
            "autoware-e2e:bench2drive-second-yaw-to-physical-v1:-yaw-pi/2"
        ),
        "primary_source": (
            "https://github.com/Thinklab-SJTU/Bench2DriveZoo/blob/uniad/vad/"
            "mmcv/datasets/B2D_vad_dataset.py"
        ),
    }
    assert manifest["aeb_vertical_overlap_contract"] == {
        "object_frame": "base_link",
        "bounding_box_interval": (
            "center_z+-0.5*(abs(R20)*dx+abs(R21)*dy+abs(R22)*dz)"
        ),
        "bounding_box_orientation_scope": (
            "valid_normalizable_full_quaternion"
        ),
        "non_box_interval": "center_z+-positive_shape_height/2",
        "non_box_orientation_scope": "valid_normalizable_yaw_only",
        "non_box_shapes": ["CYLINDER", "POLYGON"],
        "detection_interval": (
            "[detection_range_min_height,vehicle_height+"
            "detection_range_max_height_margin]"
        ),
        "boundary_policy": "inclusive",
        "invalid_or_unsupported_policy": "conservative_2d_candidate",
        "runtime_marker": (
            "autoware-e2e:aeb-predicted-object-safety-v3:"
            "oriented-bbox-vertical-overlap+forward-longitudinal-roi"
        ),
    }
    assert manifest["aeb_longitudinal_roi_contract"] == {
        "arc_reference": "ego_path.front",
        "accepted_interval": "[0,path_length+longitudinal_offset]",
        "driving_direction_policy": (
            "signed_arc_relative_to_forward_or_reverse_ego_path"
        ),
        "rear_intersection_policy": (
            "reject_before_edge_distance_absolute_value"
        ),
        "nonfinite_policy": "reject",
        "pointcloud_parity": "utils::getObjectOnPathData",
        "runtime_marker": (
            "autoware-e2e:aeb-predicted-object-safety-v3:"
            "oriented-bbox-vertical-overlap+forward-longitudinal-roi"
        ),
    }

    for prerequisite in manifest["prerequisite_patches"]:
        path = ROOT / prerequisite["path"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == prerequisite["sha256"]
    for patch_contract in manifest["patches"]:
        patch = ROOT / patch_contract["path"]
        assert hashlib.sha256(patch.read_bytes()).hexdigest() == patch_contract["sha256"]
        headers = re.findall(
            r"^diff --git a/(\S+) b/(\S+)$",
            patch.read_text(encoding="utf-8"),
            re.MULTILINE,
        )
        assert all(left == right for left, right in headers)
        assert [left for left, _ in headers] == [
            item["path"] for item in patch_contract["files"]
        ]
        for source in patch_contract["files"]:
            assert re.fullmatch(r"[0-9a-f]{64}", source["patched_sha256"])
            if source["base_state"] == "file":
                assert re.fullmatch(r"[0-9a-f]{64}", source["base_sha256"])
            else:
                assert source["base_state"] == "absent"
                assert "base_sha256" not in source

    pinned_sources = {
        str(UNIVERSE.relative_to(ROOT) / source["path"])
        for patch_contract in manifest["patches"]
        for source in patch_contract["files"]
    }
    runtime_paths = [item["build_path"] for item in manifest["runtime_libraries"]]
    assert runtime_paths == [
        "build/autoware_tensorrt_vad/libautoware_tensorrt_vad_lib.so",
        (
            "build/autoware_autonomous_emergency_braking/"
            "libautoware_autonomous_emergency_braking_node.so"
        ),
        (
            "build/autoware_autonomous_emergency_braking/"
            "libautoware_autonomous_emergency_braking_helpers.so"
        ),
    ]
    for runtime in manifest["runtime_libraries"]:
        assert runtime["source_paths"]
        assert len(runtime["source_paths"]) == len(set(runtime["source_paths"]))
        assert set(runtime["source_paths"]).issubset(pinned_sources)


def test_patch_set_replays_from_pinned_head_and_is_idempotent(tmp_path: Path) -> None:
    checkout = clone_with_prerequisites(tmp_path)
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    first = run("bash", str(APPLY_SCRIPT), env=patch_environment(checkout))
    assert first.returncode == 0, first.stderr
    assert "Applied vad_object_yaw patch." in first.stdout
    assert (
        "Applied aeb_named_timeouts_oriented_bbox_vertical_overlap_and_"
        "longitudinal_roi patch."
        in first.stdout
    )

    expected_paths = []
    for patch_contract in manifest["patches"]:
        patch = ROOT / patch_contract["path"]
        reverse = run(
            "git", "-C", str(checkout), "apply", "--reverse", "--check", str(patch)
        )
        assert reverse.returncode == 0, reverse.stderr
        for item in patch_contract["files"]:
            expected_paths.append(item["path"])
            assert hashlib.sha256((checkout / item["path"]).read_bytes()).hexdigest() == (
                item["patched_sha256"]
            )
    status = run(
        "git",
        "-C",
        str(checkout),
        "status",
        "--short",
        "--untracked-files=all",
    )
    assert status.returncode == 0
    changed_paths = {
        line[3:] for line in status.stdout.splitlines() if len(line) > 3
    }
    assert set(expected_paths).issubset(changed_paths)

    second = run("bash", str(APPLY_SCRIPT), env=patch_environment(checkout))
    assert second.returncode == 0, second.stderr
    assert second.stdout.count("already applied (APPLIED)") == 2


def test_apply_script_rejects_source_drift(tmp_path: Path) -> None:
    checkout = clone_with_prerequisites(tmp_path)
    source = checkout / "e2e/autoware_tensorrt_vad/lib/output_converter/objects_converter.cpp"
    source.write_text(source.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    result = run("bash", str(APPLY_SCRIPT), env=patch_environment(checkout))
    assert result.returncode != 0
    assert "source SHA256 drift" in result.stderr


def test_build_and_runtime_entrypoints_enforce_provenance() -> None:
    assert os.access(APPLY_SCRIPT, os.X_OK)
    assert os.access(PROVENANCE_SCRIPT, os.X_OK)
    for relative in ("scripts/e2e/build.sh", "scripts/e2e/build_full.sh"):
        text = (ROOT / relative).read_text(encoding="utf-8")
        assert "apply_vad_object_safety_patches.sh" in text
        assert "vad_object_safety_build_provenance.py capture" in text
    runner = (ROOT / "scripts/e2e/run_autoware_vad_town_matrix.sh").read_text(
        encoding="utf-8"
    )
    assert "apply_vad_object_safety_patches.sh" in runner
    assert "vad_object_safety_build_provenance.py verify" in runner
