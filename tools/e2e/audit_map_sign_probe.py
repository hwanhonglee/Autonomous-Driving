"""HH_260906 - Audit two owned native-resolution map sign probes without training."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
CAPTURES = {
    "p02_right": ROOT / "storage/experiments/training/2026-09-27/map_sign_native_p02_probe",
    "p04_left": ROOT / "storage/experiments/training/2026-09-27/map_sign_native_p04_probe",
}
DATASET = ROOT / "storage/datasets/prepared/map_expanded_20260926_ai_v2"
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_sign_probe"
CATEGORIES = ("zero", "one_to_four", "five_to_64", "at_least_65")
COLORS = {
    "other": (31, 41, 55),
    "road": (122, 129, 138),
    "sidewalk": (159, 133, 173),
    "lane_marking": (248, 211, 70),
    "traffic_light_fixture": (237, 93, 79),
    "traffic_sign": (229, 65, 185),
    "pole": (81, 190, 205),
}


def sha256(path: Path) -> str:
    """HH_260906 - Bind every conclusion to the actual local source bytes."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def histogram(values: list[int] | np.ndarray) -> dict[str, int]:
    a = np.asarray(values)
    return dict(
        zero=int(np.count_nonzero(a == 0)),
        one_to_four=int(np.count_nonzero((a >= 1) & (a <= 4))),
        five_to_64=int(np.count_nonzero((a >= 5) & (a <= 64))),
        at_least_65=int(np.count_nonzero(a >= 65)),
    )


def minimum_gap(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.sum((a[:, None] - b[None]) ** 2, axis=2)).min())


def load_dataset_reference() -> tuple[dict, dict[str, np.ndarray], dict]:
    manifest_path = DATASET / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    poses = {}
    histograms = {}
    for split in ("train", "dev"):
        ref = manifest["splits"][split]
        path = DATASET / ref["file"]
        if sha256(path) != ref["sha256"]:
            raise ValueError(f"{split} array differs from its frozen manifest")
        poses[split] = np.asarray(
            [[row["ego_x"], row["ego_y"]] for row in ref["source_records"]],
            dtype=np.float64,
        )
        if len(poses[split]) != ref["count"]:
            raise ValueError(f"{split} source-record count differs")
        with np.load(path, allow_pickle=False) as arrays:
            if arrays["target"].shape[0] != ref["count"]:
                raise ValueError(f"{split} frame count differs")
            counts = np.count_nonzero(arrays["target"] == manifest["labels"].index("traffic_sign"), axis=(1, 2))
        histograms[split] = histogram(counts)
    return manifest, poses, {"manifest_sha256": sha256(manifest_path), "histograms": histograms}


def audit_capture(name: str, root: Path, poses: dict[str, np.ndarray]) -> tuple[dict, np.ndarray]:
    """HH_260906 - Recompute semantic scale from saved sensor bytes and physical poses."""
    plan_path, result_path = root / "owner_plan.json", root / "owner_result.json"
    source = root / "mapping"
    plan, result = json.loads(plan_path.read_text()), json.loads(result_path.read_text())
    report = json.loads((source / "summary.json").read_text())
    route = Path(plan["route_path"])
    if not route.is_file() or sha256(route) != plan["route_sha256"] or report["route_sha256"] != plan["route_sha256"]:
        raise ValueError(f"{name}: route-byte identity failed")
    if (plan["capture_mode"] != "mapping-probe" or plan["quality"] != "Epic"
            or plan.get("source_bytes_archived") is not True or result["exit_code"] != 0
            or result.get("source_bytes_unchanged_and_archived") is not True
            or not result["source_checks"] or not all(result["source_checks"].values())
            or report["status"] != "COMPLETE_ORACLE_PLUMBING_ONLY"
            or report["town"] != "Town01" or report["physics_driving"] is not True
            or report["motion_mode"] != "basic-agent" or report["collision_events"] != 0
            or report["live_replay_exact_match"] is not True or report["cleanup_errors"]
            or report["label_table"].get("8") != "traffic_sign"):
        raise ValueError(f"{name}: owned physical mapping proof failed")
    for rel, expected in plan["source_sha256"].items():
        archive = root / "provenance" / rel
        if not archive.is_file() or sha256(archive) != expected:
            raise ValueError(f"{name}: archived source changed: {rel}")
    rows = json.loads((source / "records.json").read_text())
    if len(rows) != report["frame_count"] or len(rows) != 240:
        raise ValueError(f"{name}: raw record count differs")
    if len({row["stem"] for row in rows}) != len(rows):
        raise ValueError(f"{name}: repeated frame stem")
    counts: list[int] = []
    kept_indices: list[int] = []
    previous = None
    for index, row in enumerate(rows):
        stem = row["stem"]
        if stem != f"frames/{index:04d}":
            raise ValueError(f"{name}: unexpected frame ordering")
        rgb_path = source / (stem + ".png")
        semantic_path = source / (stem + ".npz")
        if (not rgb_path.resolve().is_relative_to(source.resolve())
                or not semantic_path.resolve().is_relative_to(source.resolve())
                or rgb_path.is_symlink() or semantic_path.is_symlink()):
            raise ValueError(f"{name}: unsafe source frame path")
        with Image.open(rgb_path) as rgb:
            if rgb.size != (640, 360) or rgb.mode != "RGB":
                raise ValueError(f"{name}: RGB format differs")
            rgb.verify()
        with np.load(semantic_path, allow_pickle=False) as arrays:
            semantic = arrays["semantic"]
            if semantic.shape != (360, 640, 4) or semantic.dtype != np.uint8:
                raise ValueError(f"{name}: semantic format differs")
            counts.append(int(np.count_nonzero(semantic[..., 2] == 8)))
        xy = np.array([row["ego_x"], row["ego_y"]], dtype=np.float64)
        if previous is None or np.linalg.norm(xy - previous) >= 0.25:
            kept_indices.append(index)
            previous = xy
    kept_poses = np.asarray([[rows[i]["ego_x"], rows[i]["ego_y"]] for i in kept_indices])
    kept_counts = [counts[i] for i in kept_indices]
    if len(kept_indices) < 24:
        raise ValueError(f"{name}: fewer than 24 distinct positions")
    best = max(kept_indices, key=lambda i: counts[i])
    if counts[best] == 0:
        raise ValueError(f"{name}: no positive sign example to visualize")
    return dict(
        route_path=str(route.relative_to(ROOT)),
        route_sha256=plan["route_sha256"],
        owner_plan_sha256=sha256(plan_path),
        owner_result_sha256=sha256(result_path),
        mapping_summary_sha256=sha256(source / "summary.json"),
        source_checks_passed=len(result["source_checks"]),
        capture=dict(
            town=report["town"], quality=plan["quality"],
            motion_mode=report["motion_mode"], ignores_traffic_lights=report["ignores_traffic_lights"],
            collision_events=report["collision_events"], live_replay_exact_match=report["live_replay_exact_match"],
            travel_m=report["travel_m"], agent_goal_reached=report["agent_goal_reached"],
        ),
        raw_frame_count=len(rows), retained_frame_count=len(kept_indices),
        raw_sign_histogram=histogram(counts), retained_sign_histogram=histogram(kept_counts),
        raw_sign_pixels=sum(counts), retained_sign_pixels=sum(kept_counts),
        min_distance_to_frozen_train_m=minimum_gap(kept_poses, poses["train"]),
        min_distance_to_frozen_dev_m=minimum_gap(kept_poses, poses["dev"]),
        example=dict(stem=rows[best]["stem"], sign_pixels=counts[best],
                     ego_x=rows[best]["ego_x"], ego_y=rows[best]["ego_y"]),
    ), kept_poses


def font(size: int) -> ImageFont.ImageFont:
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def make_histogram_png(summary: dict, output: Path) -> None:
    """HH_260906 - Show per-frame native sign-pixel strata, including all zero bins."""
    rows = [
        ("AI v2 TRAIN", summary["frozen_dataset"]["histograms"]["train"]),
        ("AI v2 DEV", summary["frozen_dataset"]["histograms"]["dev"]),
    ]
    for name in CAPTURES:
        rows.append((f"{name} raw", summary["captures"][name]["raw_sign_histogram"]))
        rows.append((f"{name} 0.25m kept", summary["captures"][name]["retained_sign_histogram"]))
    image = Image.new("RGB", (1400, 600), "white")
    draw = ImageDraw.Draw(image)
    heading, body = font(27), font(19)
    draw.text((24, 12), "Native traffic-sign pixels per 640 x 360 frame", fill=(20, 30, 42), font=heading)
    labels = ("0 px", "1-4 px", "5-64 px", ">=65 px")
    for j, label in enumerate(labels):
        draw.text((300 + j * 270, 75), label, fill=(20, 30, 42), font=body)
    max_count = 500
    bar_colors = ((119, 134, 151), (225, 64, 146), (240, 167, 58), (70, 142, 211))
    for i, (name, values) in enumerate(rows):
        y = 119 + i * 71
        draw.text((24, y + 4), name, fill=(20, 30, 42), font=body)
        for j, key in enumerate(CATEGORIES):
            x = 300 + j * 270
            count = values[key]
            draw.rectangle((x, y, x + 180, y + 25), outline=(209, 216, 224), fill=(248, 250, 252))
            if count:
                draw.rectangle((x, y, x + max(2, round(count / max_count * 180)), y + 25), fill=bar_colors[j])
            draw.text((x + 190, y + 2), str(count), fill=(20, 30, 42), font=body)
    draw.text((24, 555), "Counted from saved semantic tag 8; frame counts are not independent sign objects.",
              fill=(90, 102, 115), font=font(16))
    image.save(output / "sign_pixel_histogram.png")


def make_examples_png(summary: dict, output: Path) -> None:
    """HH_260906 - Display source RGB, source semantics and magnified sign crops."""
    width, row_height = 1664, 408
    image = Image.new("RGB", (width, 72 + 2 * row_height + 72), (247, 249, 252))
    draw = ImageDraw.Draw(image)
    draw.text((18, 14), "Owned CARLA RGB and oracle semantic frames - native 640 x 360", fill=(22, 31, 43), font=font(24))
    semantic_label_ids = {0: "other", 1: "road", 2: "sidewalk", 24: "lane_marking", 7: "traffic_light_fixture", 8: "traffic_sign", 6: "pole"}
    for row_index, (name, root) in enumerate(CAPTURES.items()):
        example = summary["captures"][name]["example"]
        stem = example["stem"]
        source = root / "mapping"
        rgb = Image.open(source / (stem + ".png")).convert("RGB")
        with np.load(source / (stem + ".npz"), allow_pickle=False) as arrays:
            tags = arrays["semantic"][..., 2].copy()
        colored = np.empty((360, 640, 3), dtype=np.uint8)
        colored[:] = COLORS["other"]
        for label_id, label in semantic_label_ids.items():
            colored[tags == label_id] = COLORS[label]
        semantic = Image.fromarray(colored)
        yy, xx = np.where(tags == 8)
        center_x, center_y = int((xx.min() + xx.max()) // 2), int((yy.min() + yy.max()) // 2)
        x0 = min(max(center_x - 24, 0), 592)
        y0 = min(max(center_y - 24, 0), 312)
        crop_box = (x0, y0, x0 + 48, y0 + 48)
        rgb_crop = rgb.crop(crop_box).resize((192, 192), Image.NEAREST)
        semantic_crop = semantic.crop(crop_box).resize((192, 192), Image.NEAREST)
        y = 72 + row_index * row_height
        draw.text((18, y + 2), f"{name} | {stem} | sign={example['sign_pixels']} px | source RGB / semantic / 4x crops",
                  fill=(22, 31, 43), font=font(18))
        top = y + 40
        image.paste(rgb, (0, top))
        image.paste(semantic, (640, top))
        image.paste(rgb_crop, (1280, top))
        image.paste(semantic_crop, (1472, top))
        draw.rectangle((1280, top, 1471, top + 191), outline=(22, 31, 43), width=2)
        draw.rectangle((1472, top, 1663, top + 191), outline=(22, 31, 43), width=2)
        draw.text((1283, top + 202), "RGB crop", fill=(50, 59, 70), font=font(15))
        draw.text((1475, top + 202), "label crop", fill=(50, 59, 70), font=font(15))
    legend = "Semantic colors: road gray | lane yellow | sidewalk lavender | light red | sign magenta | pole cyan"
    draw.text((18, 72 + 2 * row_height + 18), legend, fill=(48, 58, 70), font=font(17))
    image.save(output / "rgb_semantic_examples.png")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    manifest, poses, frozen = load_dataset_reference()
    captures = {}
    route_poses = {}
    for name, root in CAPTURES.items():
        captures[name], route_poses[name] = audit_capture(name, root, poses)
    inter_route_gap = minimum_gap(route_poses["p02_right"], route_poses["p04_left"])
    if any(c["retained_sign_histogram"]["one_to_four"] for c in captures.values()):
        raise ValueError("unexpected 1-4 px positives; revise the predeclared conclusion")
    summary = {
        "schema": "portable_e2e.map_sign_probe_audit.v1",
        "scope": "read-only audit of saved CARLA oracle semantic frames; no training or map approval",
        "sign_class": "traffic_sign",
        "sign_semantic_tag": 8,
        "native_frame_wh": [640, 360],
        "retention_rule": "keep first frame and each subsequent frame >=0.25m from last retained ego XY",
        "frozen_dataset": dict(name=DATASET.name, **frozen),
        "captures": captures,
        "minimum_inter_capture_retained_pose_distance_m": inter_route_gap,
        "predeclared_gate": {
            "minimum_retained_native_1_to_4_pixel_frames": 24,
            "minimum_train_pose_distance_m": 20,
            "minimum_dev_pose_distance_m": 80,
        },
        "decision": "NO_TRAIN_EXPANSION: neither route has 1-4-pixel traffic-sign labels after retention",
        "training_performed": False,
        "train_or_dev_bytes_modified": False,
        "model_approved": False,
    }
    output = args.output.resolve()
    if output.exists() and output.is_symlink():
        raise ValueError("output must not be a symlink")
    output.mkdir(parents=True, exist_ok=True)
    make_histogram_png(summary, output)
    make_examples_png(summary, output)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"output": str(output), "decision": summary["decision"],
                      "inter_capture_min_m": inter_route_gap,
                      "retained_counts": {name: c["retained_frame_count"] for name, c in captures.items()}}))


if __name__ == "__main__":
    main()
