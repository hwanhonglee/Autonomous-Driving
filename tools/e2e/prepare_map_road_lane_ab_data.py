"""HH_260929 - Package one frozen Town01/Town02 road-lane A/B data contract."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image

from scripts.e2e import audit_map_town02_p02_train_scout as town02_audit
from scripts.e2e import audit_map_town05_dev_capture as town05_audit
from scripts.e2e import prepare_map_expansion_ai as expansion


ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / "docs/assets/validation/2026-09-29/map_road_lane_ab/PLAN.md"
TEST_SOURCE = ROOT / "learning/tests/test_prepare_map_road_lane_ab_data.py"
BASELINE = ROOT / "storage/datasets/prepared/map_expanded_20260926_ai_v2"
TOWN02_AUDIT = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_train_scout/summary.json"
TOWN05_AUDIT = ROOT / "docs/assets/validation/2026-09-27/map_town05_dev_capture/epic_v2_audit.json"
OUTPUT = ROOT / "storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2"
TOWN02_AUDIT_SHA256 = "d2dd43837eeef50c9ab71106b6e67a926802fada111d735de2b859c51d1a9d49"
TOWN05_AUDIT_SHA256 = "d64d77bf4f1d02892f59ac9debd18489c5acbf38842ec81ac4398f7052b950d1"
LABELS = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture", "traffic_sign", "pole")
COUNTS = {"train_d0": 497, "train_d1": 729, "dev_town01": 118, "dev_town05": 232}
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


def raw_frame_manifest_sha256(rows: list[dict]) -> str:
    """HH_260929 - Commit all raw pair identities and order, including dropped positions."""
    sha = hashlib.sha256()
    for row in rows:
        sha.update((f"{row['stem']}\t{row['rgb_sha256']}\t"
                    f"{row['semantic_npz_sha256']}\n").encode("ascii"))
    return sha.hexdigest()


def validate_report(report: dict, *, town: str, status: str, route_sha256: str) -> list[dict]:
    """HH_260929 - Accept exactly the predeclared retained native frames and source town."""
    require(report.get("status") == status and report.get("provenance", {}).get("town") == town
            and report["provenance"].get("route_sha256") == route_sha256
            and report.get("native_frame_wh") == [640, 360]
            and report.get("raw_frames") == 240
            and report.get("retained_frames_0p25m") == 232
            and report.get("retained_gate_passed") is True
            and report.get("dataset_admitted") is False
            and report.get("model_trained") is False
            and report.get("hd_map_approved") is False,
            f"{town} published source scope differs")
    if town == "Town02_Opt":
        require(report.get("small_sign_gate_passed") is False
                and report.get("retained_sign_frame_bins", {}).get("1-4") == 0,
                "Town02 small-sign failure changed")
    else:
        require(tuple(report.get("labels", [])) == LABELS,
                "Town05 seven-class labels changed")
    frames = report.get("frames")
    stems = report.get("retained_stems")
    require(isinstance(frames, list) and len(frames) == 240
            and isinstance(stems, list) and len(stems) == len(set(stems)) == 232,
            f"{town} frame listing differs")
    expected = [f"frames/{index:04d}" for index in range(240)]
    require([row.get("stem") for row in frames] == expected,
            f"{town} raw frame order differs")
    kept, _ = town02_audit.base.common.retain_indices(frames, minimum_m=0.25)
    retained = [frames[index] for index in kept]
    require([row["stem"] for row in retained] == stems
            and all("retained_0p25m" not in row
                    or row["retained_0p25m"] is (index in kept)
                    for index, row in enumerate(frames))
            and all(len(row.get("class_pixels", [])) == len(LABELS)
                    and sum(row["class_pixels"]) == FRAME_SHAPE[0] * FRAME_SHAPE[1]
                    for row in frames),
            f"{town} retention or semantic pixel counts differ")
    return retained


def preflight(output: Path = OUTPUT) -> dict:
    """HH_260929 - Recompute both full source audits before creating any output."""
    require(not output.exists() and not output.is_symlink(), "fresh output required")
    require(output.resolve() == OUTPUT.resolve() or output.parent != OUTPUT.parent,
            "do not substitute another production output")
    require(PLAN.is_file() and not PLAN.is_symlink(), "fixed local PLAN missing")
    plan_sha256 = digest(PLAN)
    builder_sha256 = digest(Path(__file__))
    test_source_sha256 = digest(TEST_SOURCE)
    parent = expansion.frozen_ai_v2(BASELINE)
    require(tuple(parent.get("labels", [])) == LABELS
            and parent["splits"]["train"]["source_summary"]["route_sha256"]
            != parent["splits"]["dev"]["source_summary"]["route_sha256"],
            "Town01 fixed route or labels differ")
    old_train = set(parent["splits"]["train"]["source_hashes"])
    old_dev = set(parent["splits"]["dev"]["source_hashes"])
    require(not old_train.intersection(old_dev), "Town01 TRAIN and DEV source paths overlap")

    require(digest(TOWN02_AUDIT) == TOWN02_AUDIT_SHA256
            and digest(TOWN05_AUDIT) == TOWN05_AUDIT_SHA256,
            "published Town02/Town05 audit bytes changed")
    town02 = read_json(TOWN02_AUDIT)
    town05 = read_json(TOWN05_AUDIT)
    require(town02_audit.build_summary() == town02,
            "Town02 owner, route or all 240 sensor pairs changed")
    require(town05_audit.build_summary() == town05,
            "Town05 owner, route or all 240 sensor pairs changed")
    retained02 = validate_report(town02, town="Town02_Opt",
                                 status="REJECTED_SMALL_SIGN_TRAIN_CANDIDATE",
                                 route_sha256=town02_audit.ROUTE_SHA256)
    retained05 = validate_report(town05, town="Town05_Opt",
                                 status="CAPTURE_AUDIT_PASS_NOT_ADMITTED",
                                 route_sha256=town05_audit.ROUTE_SHA256)
    require(town02_audit.OWNER.resolve() != town05_audit.OWNER.resolve()
            and town02_audit.OWNER.resolve() != BASELINE.resolve()
            and town05_audit.OWNER.resolve() != BASELINE.resolve(),
            "TRAIN/DEV source roots overlap")
    return {"parent": parent, "town02": town02, "town05": town05,
            "retained02": retained02, "retained05": retained05,
            "plan_sha256": plan_sha256, "builder_sha256": builder_sha256,
            "test_source_sha256": test_source_sha256}


def target_from_semantic(semantic: np.ndarray) -> np.ndarray:
    """HH_260929 - Keep the frozen CARLA tag table, with unknown tags as other."""
    require(semantic.shape == (*FRAME_SHAPE, 4) and semantic.dtype == np.uint8,
            "native semantic BGRA required")
    lut = np.zeros(256, dtype=np.uint8)
    for tag, label in town02_audit.base.common.TAG_TO_LABEL.items():
        lut[tag] = LABELS.index(label)
    return lut[semantic[..., 2]]


def decode_retained(source: Path, row: dict) -> tuple[np.ndarray, np.ndarray]:
    """HH_260929 - Recheck audited raw bytes at use, then decode full-FOV RGB and tag."""
    stem = row["stem"]
    require(stem.startswith("frames/") and len(stem) == len("frames/0000")
            and stem[7:].isdigit(), "unsafe native frame stem")
    rgb_path = source / f"{stem}.png"
    sensor_path = source / f"{stem}.npz"
    require(digest(rgb_path) == row["rgb_sha256"]
            and digest(sensor_path) == row["semantic_npz_sha256"],
            f"raw source hash changed: {stem}")
    with Image.open(rgb_path) as image:
        require(image.mode == "RGB" and image.size == (640, 360),
                f"native RGB differs: {stem}")
        rgb = np.asarray(image, dtype=np.uint8).copy()
    with np.load(sensor_path, allow_pickle=False) as sensor:
        require(set(sensor.files) == {"depth", "semantic", "world_from_camera"},
                f"native sensor payload differs: {stem}")
        target = target_from_semantic(sensor["semantic"])
    require(np.bincount(target.ravel(), minlength=len(LABELS)).tolist()
            == row["class_pixels"], f"oracle class pixels changed: {stem}")
    require(digest(rgb_path) == row["rgb_sha256"]
            and digest(sensor_path) == row["semantic_npz_sha256"],
            f"raw source changed during decode: {stem}")
    return rgb, target


def _load_fixed_split(parent: dict, split: str) -> tuple[np.ndarray, np.ndarray]:
    ref = parent["splits"][split]
    path = BASELINE / ref["file"]
    require(digest(path) == ref["sha256"], f"frozen {split} bytes changed")
    with np.load(path, allow_pickle=False) as source:
        require(set(source.files) == {"rgb", "target"},
                f"frozen {split} keys changed")
        rgb, target = source["rgb"], source["target"]
    require(rgb.shape == (ref["count"], *FRAME_SHAPE, 3)
            and target.shape == (ref["count"], *FRAME_SHAPE)
            and rgb.dtype == target.dtype == np.uint8
            and int(target.max()) < len(LABELS)
            and np.bincount(target.ravel(), minlength=len(LABELS)).tolist()
            == ref["class_pixels"], f"frozen {split} array ABI changed")
    return rgb, target


def _copy_new(source: Path, destination: Path) -> str:
    with source.open("rb") as incoming, destination.open("xb") as outgoing:
        shutil.copyfileobj(incoming, outgoing, length=1024 * 1024)
    require(digest(destination) == digest(source), f"copied bytes changed: {destination.name}")
    return digest(destination)


def _save_npz_new(path: Path, rgb: np.ndarray, target: np.ndarray) -> str:
    with path.open("xb") as stream:
        np.savez_compressed(stream, rgb=rgb, target=target)
    return digest(path)


def _retained_proof(rows: list[dict]) -> list[dict]:
    return [{"stem": row["stem"], "rgb_sha256": row["rgb_sha256"],
             "semantic_npz_sha256": row["semantic_npz_sha256"]} for row in rows]


def build(output: Path = OUTPUT) -> dict:
    """HH_260929 - Write four exact arrays and a last-written manifest, never overwrite."""
    ready = preflight(output)
    parent = ready["parent"]
    d0_rgb, d0_target = _load_fixed_split(parent, "train")
    dev01_rgb, dev01_target = _load_fixed_split(parent, "dev")
    require(d0_rgb.shape[0] == COUNTS["train_d0"]
            and dev01_rgb.shape[0] == COUNTS["dev_town01"],
            "fixed Town01 counts differ")
    d1_rgb = np.empty((COUNTS["train_d1"], *FRAME_SHAPE, 3), dtype=np.uint8)
    d1_target = np.empty((COUNTS["train_d1"], *FRAME_SHAPE), dtype=np.uint8)
    d1_rgb[:497], d1_target[:497] = d0_rgb, d0_target
    source02 = town02_audit.OWNER / "mapping"
    for index, row in enumerate(ready["retained02"], start=497):
        d1_rgb[index], d1_target[index] = decode_retained(source02, row)
    require(np.array_equal(d1_rgb[:497], d0_rgb)
            and np.array_equal(d1_target[:497], d0_target),
            "D1 changed the frozen D0 prefix")
    del d0_rgb, d0_target, dev01_rgb, dev01_target

    dev05_rgb = np.empty((COUNTS["dev_town05"], *FRAME_SHAPE, 3), dtype=np.uint8)
    dev05_target = np.empty((COUNTS["dev_town05"], *FRAME_SHAPE), dtype=np.uint8)
    source05 = town05_audit.OWNER / "mapping"
    for index, row in enumerate(ready["retained05"]):
        dev05_rgb[index], dev05_target[index] = decode_retained(source05, row)
    require(np.bincount(dev05_target.ravel(), minlength=len(LABELS)).tolist()
            == ready["town05"]["retained_class_pixels"],
            "Town05 DEV class pixels differ")
    d1_class_pixels = np.bincount(d1_target.ravel(), minlength=len(LABELS)).tolist()
    dev05_class_pixels = np.bincount(dev05_target.ravel(), minlength=len(LABELS)).tolist()
    require(digest(PLAN) == ready["plan_sha256"]
            and digest(Path(__file__)) == ready["builder_sha256"]
            and digest(TEST_SOURCE) == ready["test_source_sha256"]
            and digest(BASELINE / "manifest.json") == expansion.AI_V2_PARENT_SHA256
            and digest(TOWN02_AUDIT) == TOWN02_AUDIT_SHA256
            and digest(TOWN05_AUDIT) == TOWN05_AUDIT_SHA256,
            "input contract changed during data assembly")
    require(not output.exists() and not output.is_symlink(), "fresh output lost during assembly")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir(exist_ok=False)
    files = {
        "train_d0": {"file": "train_d0.npz", "sha256": _copy_new(BASELINE / "train.npz", output / "train_d0.npz"),
                     "count": 497, "towns": ["Town01"],
                     "class_pixels": parent["splits"]["train"]["class_pixels"]},
        "train_d1": {"file": "train_d1.npz", "sha256": _save_npz_new(output / "train_d1.npz", d1_rgb, d1_target),
                     "count": 729, "towns": ["Town01", "Town02_Opt"], "class_pixels": d1_class_pixels},
        "dev_town01": {"file": "dev_town01.npz", "sha256": _copy_new(BASELINE / "dev.npz", output / "dev_town01.npz"),
                       "count": 118, "towns": ["Town01"],
                       "class_pixels": parent["splits"]["dev"]["class_pixels"]},
        "dev_town05": {"file": "dev_town05.npz", "sha256": _save_npz_new(output / "dev_town05.npz", dev05_rgb, dev05_target),
                       "count": 232, "towns": ["Town05_Opt"], "class_pixels": dev05_class_pixels},
    }
    require(files["train_d0"]["sha256"] == expansion.AI_V2_SPLIT_SHA256["train"]
            and files["dev_town01"]["sha256"] == expansion.AI_V2_SPLIT_SHA256["dev"],
            "frozen Town01 split copy changed")
    for key, rgb, target in (("train_d1", d1_rgb, d1_target),
                             ("dev_town05", dev05_rgb, dev05_target)):
        with np.load(output / files[key]["file"], allow_pickle=False) as archive:
            require(set(archive.files) == {"rgb", "target"}
                    and np.array_equal(archive["rgb"], rgb)
                    and np.array_equal(archive["target"], target),
                    f"serialized {key} differs from audited arrays")
    manifest = {
        "schema": "portable_e2e.map_road_lane_ab_data.v1",
        "status": "DATA_PACKAGE_ONLY_NO_MODEL_TRAINED",
        "parent_manifest_sha256": expansion.AI_V2_PARENT_SHA256,
        "town02_split": "TRAIN_EXPLORATORY_ROAD_LANE_ONLY",
        "town05_split": "DEV_ONLY",
        "labels": list(LABELS), "image_size_wh": [640, 360],
        "array_keys": ["rgb", "target"], "array_dtype": "uint8",
        "files": files,
        "provenance": {
            "plan_sha256": ready["plan_sha256"],
            "builder_sha256": ready["builder_sha256"],
            "test_source_sha256": ready["test_source_sha256"],
            "ai_v2_manifest_sha256": expansion.AI_V2_PARENT_SHA256,
            "ai_v2_train_npz_sha256": expansion.AI_V2_SPLIT_SHA256["train"],
            "ai_v2_dev_npz_sha256": expansion.AI_V2_SPLIT_SHA256["dev"],
            "town01_train_route_sha256": parent["splits"]["train"]["source_summary"]["route_sha256"],
            "town01_dev_route_sha256": parent["splits"]["dev"]["source_summary"]["route_sha256"],
            "town02_audit_sha256": TOWN02_AUDIT_SHA256,
            "town02_owner_and_route": ready["town02"]["provenance"],
            "town02_raw_pair_manifest_sha256": raw_frame_manifest_sha256(ready["town02"]["frames"]),
            "town02_retained_raw_pairs": _retained_proof(ready["retained02"]),
            "town05_audit_sha256": TOWN05_AUDIT_SHA256,
            "town05_owner_and_route": ready["town05"]["provenance"],
            "town05_raw_pair_manifest_sha256": raw_frame_manifest_sha256(ready["town05"]["frames"]),
            "town05_retained_raw_pairs": _retained_proof(ready["retained05"]),
        },
        "split_contract": {
            "train_d1_prefix": "train_d0 exact rgb/target arrays in original order",
            "train_d1_append": "Town02_Opt p02 retained 232 stems in published 0.25m order",
            "dev_town01": "byte-identical AI v2 DEV118",
            "dev_town05": "Town05_Opt Epic v2 retained 232 stems in published 0.25m order",
            "town01_train_dev_source_path_overlap": 0,
            "town02_small_sign_gate": "REJECTED_SMALL_SIGN_TRAIN_CANDIDATE",
            "dev_used_for_training": False,
            "test_used": False,
            "cross_town_xy_compared": False,
            "model_trained": False,
            "hd_map_approved": False,
        },
    }
    with (output / "manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--execute", action="store_true", help="write the one fresh package")
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
                          "town02_retained": len(ready["retained02"]),
                          "town05_retained": len(ready["retained05"])}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
