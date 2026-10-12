#!/usr/bin/env python3
"""HH_260929 - Package a CPU-only Town02 p00 map TRAIN candidate with p02 held out."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np
from PIL import Image

from scripts.e2e import audit_map_town02_train_scout as p00_audit
from scripts.e2e import audit_map_town02_p02_train_scout as p02_audit
from scripts.e2e import prepare_map_expansion_ai as expansion
from scripts.e2e import site_split_guard as split_guard
from scripts.e2e import site_split_guard_v2 as split_guard_v2


ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / "docs/assets/validation/2026-09-29/map_town02_p00_route_holdout/PLAN.md"
QA = ROOT / "docs/assets/validation/2026-09-29/map_alternative_train_sources/visual_qa/result.json"
RESEARCH_SCOPE = ROOT / "learning/config/map_town02_p00_research_scope_20260929.json"
P00_AUDIT = ROOT / "docs/assets/validation/2026-09-27/map_town02_train_scout/summary.json"
P02_AUDIT = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_train_scout/summary.json"
BASELINE = ROOT / "storage/datasets/prepared/map_expanded_20260926_ai_v2"
D1 = ROOT / "storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2"
OUTPUT = ROOT / "storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1"
TEST_SOURCE = ROOT / "learning/tests/test_prepare_map_town02_p00_route_holdout_data.py"
SCHEMA = "portable_e2e.map_town02_p00_holdout_data.v1"
P00_AUDIT_SHA256 = "8467dcf470e5e4cef2f9aee884736127989c13b19f03254d43a6448824f2ac47"
P02_AUDIT_SHA256 = "d2dd43837eeef50c9ab71106b6e67a926802fada111d735de2b859c51d1a9d49"
QA_SHA256 = "62f354445943ab8931c0fbf3a14530e29d72ad22f19a5f4f214ab10579bb8396"
PLAN_SHA256 = "95e0cbb1e918e626cdfab15a58c201a0a863d4e93ee5d440da1f2f91a8010ad5"
RESEARCH_SCOPE_SHA256 = "c4870d042a074f559be7ed841ba0351be007d2d562d9a0b7567db2ac95b719b0"
D1_MANIFEST_SHA256 = "313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6"
V1_SPLIT_SHA256 = "da0245f3a863b3b03dc0e364eb067354a27d36fc123d2bb860e1a4bff7a9690c"
V2_SPLIT_SHA256 = "323b68c690724bdb63a9955c9573a249d5c49f5001160a421e797249dce7ec0b"
LABELS = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture", "traffic_sign", "pole")
COUNTS = {"train_a": 497, "train_b": 729, "holdout_town02_p02": 232, "dev_town01": 118}
FRAME_SHAPE = (360, 640)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular input required: {path}")
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(chunk)
    return sha.hexdigest()


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def validate_audit(report: dict, *, status: str, route_sha256: str,
                   sign_one_to_four: int) -> list[dict]:
    """HH_260929 - Preserve the original sign failure and exact 0.25m frame selection."""
    require(report.get("status") == status
            and report.get("provenance", {}).get("town") == "Town02_Opt"
            and report["provenance"].get("route_sha256") == route_sha256
            and report.get("native_frame_wh") == [640, 360]
            and report.get("raw_frames") == 240
            and report.get("retained_frames_0p25m") == 232
            and report.get("retained_gate_passed") is True
            and report.get("small_sign_gate_passed") is False
            and report.get("retained_sign_frame_bins", {}).get("1-4") == sign_one_to_four
            and report.get("dataset_admitted") is False
            and report.get("model_trained") is False
            and report.get("hd_map_approved") is False,
            "Town02 source scope or historical small-sign failure changed")
    frames, stems = report.get("frames"), report.get("retained_stems")
    require(isinstance(frames, list) and len(frames) == 240
            and isinstance(stems, list) and len(stems) == len(set(stems)) == 232,
            "Town02 source frame listing differs")
    require([row.get("stem") for row in frames] == [f"frames/{i:04d}" for i in range(240)],
            "Town02 raw frame order differs")
    kept, _ = p00_audit.common.retain_indices(frames, minimum_m=0.25)
    rows = [frames[i] for i in kept]
    require([row["stem"] for row in rows] == stems
            and all(len(row.get("class_pixels", [])) == len(LABELS)
                    and sum(row["class_pixels"]) == FRAME_SHAPE[0] * FRAME_SHAPE[1]
                    for row in frames),
            "Town02 retention or semantic histogram differs")
    return rows


def minimum_pose_distance(a: list[dict], b: list[dict]) -> float:
    """HH_260929 - Check physical separation only within the same named Town."""
    return min(math.hypot(x["ego_x"] - y["ego_x"], x["ego_y"] - y["ego_y"])
               for x in a for y in b)


def validate_qa(qa: dict) -> None:
    require(qa.get("status") == "QA_CONDITIONAL_FIXED_VIEWS_ONLY"
            and qa.get("dataset_admitted") is False
            and qa.get("train_eligible") is False
            and qa.get("independent_dev_approved") is False
            and qa.get("fixed_frames") == ["0000", "0120", "0239"]
            and qa.get("retained_frame_count") == 232
            and qa.get("totals", {}).get("road_pixels") == 21_432_799
            and qa["totals"].get("lane_pixels") == 145_835
            and qa["totals"].get("road_positive_frames") == 232
            and qa["totals"].get("lane_positive_frames") == 232,
            "p00 conditional visual QA differs")
    source_sha = qa.get("source_sha256", {})
    require(source_sha.get("route") == p00_audit.ROUTE_SHA256
            and source_sha.get(str(P00_AUDIT.relative_to(ROOT))) == P00_AUDIT_SHA256
            and source_sha.get(str(split_guard.REGISTRY.relative_to(ROOT))) == V1_SPLIT_SHA256,
            "p00 QA source identities differ")
    qa_script = ROOT / "tools/e2e/audit_map_town02_p00_road_lane.py"
    require(qa.get("script_sha256") == digest(qa_script), "p00 QA script changed")
    visuals = qa.get("visual_sha256")
    require(isinstance(visuals, dict) and len(visuals) == 5,
            "p00 visual receipt incomplete")
    for filename, expected in visuals.items():
        require(Path(filename).name == filename and digest(QA.parent / filename) == expected,
                f"p00 visual bytes changed: {filename}")


def validate_research_scope(policy: dict, hashes: dict) -> None:
    """HH_260929 - Permit only this exploratory A/B, never formal admission or promotion."""
    expected = {
        "schema": "portable_e2e.map_town02_p00_research_scope.v1",
        "site": "Town02_Opt", "model": "map",
        "map_operation": "exploratory_train_candidate",
        "scope": "road_lane_rgb_semantics_only",
        "source_episode": str((p00_audit.OWNER / "mapping").relative_to(ROOT)),
        "source_route_sha256": p00_audit.ROUTE_SHA256,
        "source_audit_sha256": hashes["p00_audit_sha256"],
        "visual_qa_result_sha256": hashes["qa_sha256"],
        "preregistered_plan_sha256": hashes["plan_sha256"],
        "parent_site_split_v2_sha256": hashes["v2_split_sha256"],
        "source_raw_frames": 240, "source_retained_frames": 232,
        "train_a_frames": 497, "train_b_frames": 729,
        "heldout_route": "Town02_Opt p02, historically exposed internal diagnostic only",
    }
    for key, value in expected.items():
        require(policy.get(key) == value and type(policy.get(key)) is type(value),
                f"research scope changed: {key}")
    for key, value in {
        "exploratory_ab_train_authorized": True,
        "dataset_admitted": False,
        "independent_dev_approved": False,
        "automatic_model_promotion": False,
        "test_use_authorized": False,
        "vehicle_control_approved": False,
    }.items():
        require(type(policy.get(key)) is bool and policy[key] is value,
                f"research scope changed: {key}")


def preflight(output: Path = OUTPUT) -> dict:
    """HH_260929 - Re-audit both owned routes and keep the output absent on failure."""
    require(not output.exists() and not output.is_symlink(), "fresh output required")
    require(output.resolve() != BASELINE.resolve() and output.resolve() != D1.resolve()
            and output.resolve() != p00_audit.OWNER.resolve()
            and output.resolve() != p02_audit.OWNER.resolve(),
            "output overlaps a frozen source")
    hashes = {"plan_sha256": digest(PLAN), "qa_sha256": digest(QA),
              "research_scope_sha256": digest(RESEARCH_SCOPE),
              "p00_audit_sha256": digest(P00_AUDIT),
              "p02_audit_sha256": digest(P02_AUDIT),
              "v1_split_sha256": digest(split_guard.REGISTRY),
              "v2_split_sha256": digest(split_guard_v2.REGISTRY),
              "d1_manifest_sha256": digest(D1 / "manifest.json"),
              "builder_sha256": digest(Path(__file__)),
              "test_source_sha256": digest(TEST_SOURCE)}
    require(hashes["plan_sha256"] == PLAN_SHA256
            and hashes["research_scope_sha256"] == RESEARCH_SCOPE_SHA256
            and hashes["qa_sha256"] == QA_SHA256
            and hashes["p00_audit_sha256"] == P00_AUDIT_SHA256
            and hashes["p02_audit_sha256"] == P02_AUDIT_SHA256
            and hashes["v1_split_sha256"] == V1_SPLIT_SHA256
            and hashes["v2_split_sha256"] == V2_SPLIT_SHA256
            and hashes["d1_manifest_sha256"] == D1_MANIFEST_SHA256,
            "pinned input SHA differs")
    validate_research_scope(read_json(RESEARCH_SCOPE), hashes)
    registry = split_guard.validate_registry(read_json(split_guard.REGISTRY))
    registry_v2 = split_guard_v2.validate_registry(read_json(split_guard_v2.REGISTRY))
    require(registry["independent_dev_sites"] == []
            and split_guard.check("Town01", "map", "reuse_existing_train", registry)
            and split_guard.check("Town02_Opt", "map", "reuse_existing_train", registry)
            and registry_v2["independent_dev_sites"] == []
            and registry_v2["sites"]["Town02_Opt"]["map"] == ["reuse_existing_train"],
            "V1/V2 map TRAIN reuse role differs")
    parent = expansion.frozen_ai_v2(BASELINE)
    require(tuple(parent.get("labels", [])) == LABELS
            and parent["splits"]["train"]["count"] == 497
            and parent["splits"]["dev"]["count"] == 118,
            "frozen Town01 parent differs")
    d1 = read_json(D1 / "manifest.json")
    require(d1.get("parent_manifest_sha256") == expansion.AI_V2_PARENT_SHA256
            and d1.get("files", {}).get("train_d0", {}).get("sha256")
                == expansion.AI_V2_SPLIT_SHA256["train"]
            and d1["files"].get("train_d1", {}).get("count") == 729
            and digest(D1 / d1["files"]["train_d1"]["file"])
                == d1["files"]["train_d1"]["sha256"],
            "frozen D1 package differs")
    qa = read_json(QA)
    validate_qa(qa)
    p00 = read_json(P00_AUDIT)
    p02 = read_json(P02_AUDIT)
    require(p00_audit.build_summary() == p00 and p02_audit.build_summary() == p02,
            "owned Town02 full-source audit differs")
    rows00 = validate_audit(p00, status="REJECTED_SIGN_RICH_TRAIN_CANDIDATE",
                            route_sha256=p00_audit.ROUTE_SHA256, sign_one_to_four=3)
    rows02 = validate_audit(p02, status="REJECTED_SMALL_SIGN_TRAIN_CANDIDATE",
                            route_sha256=p02_audit.ROUTE_SHA256, sign_one_to_four=0)
    gap = minimum_pose_distance(rows00, rows02)
    require(gap > 100.0 and abs(gap - 197.03993312893482) < 0.001,
            "p00 and p02 routes are not the pinned physically separate pair")
    old_hashes = set(parent["splits"]["train"]["source_hashes"].values())
    old_hashes.update(parent["splits"]["dev"]["source_hashes"].values())
    p02_hashes = {value for row in rows02 for value in
                  (row["rgb_sha256"], row["semantic_npz_sha256"])}
    p00_hashes = {value for row in rows00 for value in
                  (row["rgb_sha256"], row["semantic_npz_sha256"])}
    require(not (p00_hashes & old_hashes or p00_hashes & p02_hashes
                 or p02_hashes & old_hashes),
            "raw TRAIN, heldout, or Town01 DEV frame hash collision")
    return {"parent": parent, "d1": d1, "p00": p00, "p02": p02,
            "rows00": rows00, "rows02": rows02, "gap_m": gap,
            "hashes": hashes}


def target_from_semantic(semantic: np.ndarray) -> np.ndarray:
    """HH_260929 - Map unchanged CARLA tag IDs through the frozen seven-class table."""
    require(semantic.shape == (*FRAME_SHAPE, 4) and semantic.dtype == np.uint8,
            "native semantic BGRA required")
    lut = np.zeros(256, dtype=np.uint8)
    for tag, label in p00_audit.common.TAG_TO_LABEL.items():
        lut[tag] = LABELS.index(label)
    return lut[semantic[..., 2]]


def decode_retained(source: Path, row: dict) -> tuple[np.ndarray, np.ndarray]:
    stem = row["stem"]
    require(stem.startswith("frames/") and len(stem) == len("frames/0000")
            and stem[7:].isdigit(), "unsafe retained frame stem")
    rgb_path, sensor_path = source / (stem + ".png"), source / (stem + ".npz")
    require(digest(rgb_path) == row["rgb_sha256"]
            and digest(sensor_path) == row["semantic_npz_sha256"],
            f"raw pair hash changed: {stem}")
    with Image.open(rgb_path) as image:
        require(image.mode == "RGB" and image.size == (640, 360),
                f"native RGB format differs: {stem}")
        rgb = np.asarray(image, dtype=np.uint8).copy()
    with np.load(sensor_path, allow_pickle=False) as sensor:
        require(set(sensor.files) == {"depth", "semantic", "world_from_camera"}
                and sensor["depth"].shape == (*FRAME_SHAPE, 4)
                and sensor["depth"].dtype == np.uint8
                and sensor["world_from_camera"].shape == (4, 4)
                and np.isfinite(sensor["world_from_camera"]).all(),
                f"native sensor/pose differs: {stem}")
        target = target_from_semantic(sensor["semantic"])
    require(np.bincount(target.ravel(), minlength=len(LABELS)).tolist()
            == row["class_pixels"], f"raw semantic histogram differs: {stem}")
    require(digest(rgb_path) == row["rgb_sha256"]
            and digest(sensor_path) == row["semantic_npz_sha256"],
            f"raw pair changed during decode: {stem}")
    return rgb, target


def load_fixed_split(parent: dict, split: str) -> tuple[np.ndarray, np.ndarray]:
    ref = parent["splits"][split]
    path = BASELINE / ref["file"]
    require(digest(path) == ref["sha256"], f"frozen Town01 {split} bytes changed")
    with np.load(path, allow_pickle=False) as source:
        require(set(source.files) == {"rgb", "target"},
                f"frozen Town01 {split} keys differ")
        rgb, target = source["rgb"], source["target"]
    require(rgb.shape == (ref["count"], *FRAME_SHAPE, 3)
            and target.shape == (ref["count"], *FRAME_SHAPE)
            and rgb.dtype == target.dtype == np.uint8
            and np.bincount(target.ravel(), minlength=len(LABELS)).tolist()
                == ref["class_pixels"], f"frozen Town01 {split} array ABI differs")
    return rgb, target


def class_pixels(target: np.ndarray) -> list[int]:
    require(target.dtype == np.uint8 and int(target.max()) < len(LABELS),
            "target class range differs")
    return np.bincount(target.ravel(), minlength=len(LABELS)).tolist()


def save_npz_new(path: Path, rgb: np.ndarray, target: np.ndarray) -> str:
    with path.open("xb") as stream:
        np.savez_compressed(stream, rgb=rgb, target=target)
    return digest(path)


def copy_new(source: Path, destination: Path) -> str:
    with source.open("rb") as incoming, destination.open("xb") as outgoing:
        shutil.copyfileobj(incoming, outgoing, length=1024 * 1024)
    require(digest(destination) == digest(source), "frozen split copy differs")
    return digest(destination)


def verify_package(output: Path, ready: dict) -> dict:
    """HH_260929 - Reopen bytes and reconstruct B/p02 from audited raw, not the manifest."""
    manifest = read_json(output / "manifest.json")
    require(manifest.get("schema") == SCHEMA
            and manifest.get("research_only") is True
            and manifest.get("exploratory_ab_train_authorized") is True
            and manifest.get("dataset_admitted") is False
            and manifest.get("independent_dev_approved") is False
            and manifest.get("parent_manifest_sha256") == expansion.AI_V2_PARENT_SHA256
            and manifest.get("provenance", {}).get("research_scope_sha256")
                == RESEARCH_SCOPE_SHA256,
            "serialized package policy differs")
    refs = manifest.get("files", {})
    require(set(refs) == set(COUNTS), "serialized package file keys differ")
    arrays = {}
    for key, count in COUNTS.items():
        ref = refs[key]
        require(ref.get("file") == f"{key}.npz" and ref.get("count") == count
                and digest(output / ref["file"]) == ref.get("sha256"),
                f"serialized package file identity differs: {key}")
        with np.load(output / ref["file"], allow_pickle=False) as archive:
            require(set(archive.files) == {"rgb", "target"},
                    f"serialized package keys differ: {key}")
            rgb, target = archive["rgb"], archive["target"]
        require(rgb.shape == (count, *FRAME_SHAPE, 3)
                and target.shape == (count, *FRAME_SHAPE)
                and rgb.dtype == target.dtype == np.uint8
                and class_pixels(target) == ref.get("class_pixels"),
                f"serialized package ABI or class histogram differs: {key}")
        arrays[key] = (rgb, target)
    a_rgb, a_target = arrays["train_a"]
    b_rgb, b_target = arrays["train_b"]
    h_rgb, h_target = arrays["holdout_town02_p02"]
    d_rgb, d_target = arrays["dev_town01"]
    require(refs["train_a"]["sha256"] == expansion.AI_V2_SPLIT_SHA256["train"]
            and refs["dev_town01"]["sha256"] == expansion.AI_V2_SPLIT_SHA256["dev"]
            and np.array_equal(b_rgb[:497], a_rgb)
            and np.array_equal(b_target[:497], a_target),
            "serialized B prefix differs from frozen Town01 TRAIN")
    a_parent, t_parent = load_fixed_split(ready["parent"], "train")
    d_parent, dt_parent = load_fixed_split(ready["parent"], "dev")
    require(np.array_equal(a_rgb, a_parent)
            and np.array_equal(a_target, t_parent)
            and np.array_equal(d_rgb, d_parent)
            and np.array_equal(d_target, dt_parent),
            "serialized Town01 TRAIN or DEV differs from frozen parent")
    for index, row in enumerate(ready["rows00"]):
        rgb, target = decode_retained(p00_audit.OWNER / "mapping", row)
        require(np.array_equal(b_rgb[497 + index], rgb)
                and np.array_equal(b_target[497 + index], target),
                f"serialized B suffix differs from audited p00 raw: {row['stem']}")
    for index, row in enumerate(ready["rows02"]):
        rgb, target = decode_retained(p02_audit.OWNER / "mapping", row)
        require(np.array_equal(h_rgb[index], rgb)
                and np.array_equal(h_target[index], target),
                f"serialized holdout differs from audited p02 raw: {row['stem']}")
    d1_ref = ready["d1"]["files"]["train_d1"]
    require(digest(D1 / d1_ref["file"]) == d1_ref["sha256"],
            "frozen D1 changed during package verification")
    with np.load(D1 / d1_ref["file"], allow_pickle=False) as archive:
        require(np.array_equal(archive["rgb"][497:], h_rgb)
                and np.array_equal(archive["target"][497:], h_target),
                "serialized p02 holdout differs from historic D1 suffix")
    return {"status": "SOURCE_TO_PACKAGE_VERIFIED", "p00_source_rows": 232,
            "p02_source_rows": 232, "a_parent_prefix": 497, "town01_dev": 118,
            "manifest_sha256": digest(output / "manifest.json")}


def build(output: Path = OUTPUT) -> dict:
    """HH_260929 - Write only a fresh four-split candidate package after CPU audits."""
    ready = preflight(output)
    a_rgb, a_target = load_fixed_split(ready["parent"], "train")
    dev_rgb, dev_target = load_fixed_split(ready["parent"], "dev")
    b_rgb = np.empty((729, *FRAME_SHAPE, 3), dtype=np.uint8)
    b_target = np.empty((729, *FRAME_SHAPE), dtype=np.uint8)
    b_rgb[:497], b_target[:497] = a_rgb, a_target
    p00_source = p00_audit.OWNER / "mapping"
    for index, row in enumerate(ready["rows00"], start=497):
        b_rgb[index], b_target[index] = decode_retained(p00_source, row)
    require(np.array_equal(b_rgb[:497], a_rgb)
            and np.array_equal(b_target[:497], a_target),
            "B changed the frozen Town01 TRAIN prefix")
    hold_rgb = np.empty((232, *FRAME_SHAPE, 3), dtype=np.uint8)
    hold_target = np.empty((232, *FRAME_SHAPE), dtype=np.uint8)
    p02_source = p02_audit.OWNER / "mapping"
    for index, row in enumerate(ready["rows02"]):
        hold_rgb[index], hold_target[index] = decode_retained(p02_source, row)
    d1_ref = ready["d1"]["files"]["train_d1"]
    with np.load(D1 / d1_ref["file"], allow_pickle=False) as archive:
        require(set(archive.files) == {"rgb", "target"}, "D1 array keys differ")
        require(np.array_equal(archive["rgb"][:497], a_rgb)
                and np.array_equal(archive["target"][:497], a_target)
                and np.array_equal(archive["rgb"][497:], hold_rgb)
                and np.array_equal(archive["target"][497:], hold_target),
                "p02 holdout differs from the exact D1 suffix")
    expected_p00 = np.sum(np.asarray([r["class_pixels"] for r in ready["rows00"]],
                                          dtype=np.int64), axis=0).tolist()
    expected_p02 = np.sum(np.asarray([r["class_pixels"] for r in ready["rows02"]],
                                          dtype=np.int64), axis=0).tolist()
    require(class_pixels(b_target[497:]) == expected_p00
            and class_pixels(hold_target) == expected_p02
            and class_pixels(dev_target) == ready["parent"]["splits"]["dev"]["class_pixels"],
            "p00/p02/DEV semantic totals differ")
    require(digest(PLAN) == ready["hashes"]["plan_sha256"]
            and digest(RESEARCH_SCOPE) == RESEARCH_SCOPE_SHA256
            and digest(QA) == QA_SHA256
            and digest(P00_AUDIT) == P00_AUDIT_SHA256
            and digest(P02_AUDIT) == P02_AUDIT_SHA256
            and digest(Path(__file__)) == ready["hashes"]["builder_sha256"]
            and digest(TEST_SOURCE) == ready["hashes"]["test_source_sha256"]
            and not output.exists(),
            "input changed or output appeared during assembly")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir(exist_ok=False)
    files = {
        "train_a": {"file": "train_a.npz", "sha256": copy_new(BASELINE / "train.npz", output / "train_a.npz"),
                    "count": 497, "towns": ["Town01"], "class_pixels": class_pixels(a_target)},
        "train_b": {"file": "train_b.npz", "sha256": save_npz_new(output / "train_b.npz", b_rgb, b_target),
                    "count": 729, "towns": ["Town01", "Town02_Opt"], "class_pixels": class_pixels(b_target)},
        "holdout_town02_p02": {
            "file": "holdout_town02_p02.npz",
            "sha256": save_npz_new(output / "holdout_town02_p02.npz", hold_rgb, hold_target),
            "count": 232, "towns": ["Town02_Opt"], "class_pixels": class_pixels(hold_target)},
        "dev_town01": {"file": "dev_town01.npz",
                       "sha256": copy_new(BASELINE / "dev.npz", output / "dev_town01.npz"),
                       "count": 118, "towns": ["Town01"], "class_pixels": class_pixels(dev_target)},
    }
    require(files["train_a"]["sha256"] == expansion.AI_V2_SPLIT_SHA256["train"]
            and files["dev_town01"]["sha256"] == expansion.AI_V2_SPLIT_SHA256["dev"],
            "frozen Town01 split copy changed")
    for key, rgb, target in (("train_b", b_rgb, b_target),
                             ("holdout_town02_p02", hold_rgb, hold_target)):
        with np.load(output / files[key]["file"], allow_pickle=False) as archive:
            require(set(archive.files) == {"rgb", "target"}
                    and np.array_equal(archive["rgb"], rgb)
                    and np.array_equal(archive["target"], target),
                    f"serialized {key} differs from audited source arrays")
    manifest = {
        "schema": SCHEMA,
        "status": "CPU_RESEARCH_CANDIDATE_PACKAGE_ONLY_NO_TRAINING",
        "research_only": True,
        "exploratory_ab_train_authorized": True,
        "dataset_admitted": False,
        "independent_dev_approved": False,
        "model_trained": False,
        "parent_manifest_sha256": expansion.AI_V2_PARENT_SHA256,
        "labels": list(LABELS),
        "image_size_wh": [640, 360],
        "array_keys": ["rgb", "target"],
        "array_dtype": "uint8",
        "files": files,
        "provenance": {
            **ready["hashes"],
            "parent_train_npz_sha256": expansion.AI_V2_SPLIT_SHA256["train"],
            "parent_dev_npz_sha256": expansion.AI_V2_SPLIT_SHA256["dev"],
            "d1_train_npz_sha256": d1_ref["sha256"],
            "p00_owner_and_route": ready["p00"]["provenance"],
            "p02_owner_and_route": ready["p02"]["provenance"],
            "p00_p02_minimum_retained_pose_distance_m": ready["gap_m"],
            "p00_retained_raw_pairs": [{"stem": r["stem"], "rgb_sha256": r["rgb_sha256"],
                                        "semantic_npz_sha256": r["semantic_npz_sha256"]}
                                       for r in ready["rows00"]],
            "p02_holdout_raw_pairs": [{"stem": r["stem"], "rgb_sha256": r["rgb_sha256"],
                                       "semantic_npz_sha256": r["semantic_npz_sha256"]}
                                      for r in ready["rows02"]],
        },
        "split_contract": {
            "train_b_prefix": "pixel-identical train_a 497 RGB and target frames in original order",
            "train_b_append": "Town02_Opt p00 retained 232 stems in published 0.25m order",
            "holdout_town02_p02": "Town02_Opt p02 retained 232; exact previous D1 suffix, excluded from both TRAIN arms",
            "dev_town01": "byte-identical parent Town01 exploratory DEV118",
            "town02_small_sign_gate": "FAILED_PREVIOUS_OBJECTIVE_3_OF_232_P00_0_OF_232_P02",
            "dev_used_for_training": False,
            "test_used": False,
            "model_trained": False,
            "hd_map_approved": False,
        },
    }
    with (output / "manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    verification = verify_package(output, ready)
    with (output / "verification.json").open("x", encoding="utf-8") as stream:
        json.dump(verification, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--execute", action="store_true", help="write one fresh CPU data package")
    args = parser.parse_args(argv)
    if args.execute:
        manifest = build()
        print(json.dumps({"status": manifest["status"], "output": str(OUTPUT),
                          "manifest_sha256": digest(OUTPUT / "manifest.json"),
                          "counts": {name: item["count"] for name, item in manifest["files"].items()}},
                         sort_keys=True))
    else:
        ready = preflight()
        print(json.dumps({"status": "READ_ONLY_PREFLIGHT_PASS", "output_absent": True,
                          "p00_retained": len(ready["rows00"]),
                          "p02_retained": len(ready["rows02"]),
                          "minimum_same_town_gap_m": ready["gap_m"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
