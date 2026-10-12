"""HH_260906 - Audit one owned Town06 sign scout without training or cross-town pose comparison."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
OWNER = ROOT / "storage/experiments/training/2026-09-27/map_sign_town06_p06_scout_v1"
SOURCE = OWNER / "mapping"
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_sign_town06_p06"
ROUTE = ROOT / ("storage/experiments/validation/2026-09-01/"
                "autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town06/catalog/"
                "straight/routes/town06/straight/town06_straight_s0000_p06.json")
ROUTE_SHA256 = "e27e95cb60d40657638183f7f78ce2e1f4f070067707ac4dd3f2f89dd1c52283"
REFERENCE = {
    "p02_right": ROOT / "storage/experiments/training/2026-09-27/map_sign_native_p02_probe/mapping",
    "p04_left": ROOT / "storage/experiments/training/2026-09-27/map_sign_native_p04_probe/mapping",
}
FROZEN_DATASET = ROOT / "storage/datasets/prepared/map_expanded_20260926_ai_v2"
LABEL_COLORS = {
    0: (30, 42, 57), 1: (122, 129, 138), 2: (159, 133, 173),
    24: (248, 211, 70), 7: (237, 93, 79), 8: (229, 65, 185), 6: (81, 190, 205),
}


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def require_owned_completion() -> dict:
    """HH_260906 - Verify source, exact route and world provenance before reading image statistics."""
    plan = json.loads((OWNER / "owner_plan.json").read_text())
    result = json.loads((OWNER / "owner_result.json").read_text())
    started = json.loads((OWNER / "owner_started.json").read_text())
    report = json.loads((SOURCE / "summary.json").read_text())
    route = json.loads(ROUTE.read_text())
    if (ROUTE.is_symlink() or digest(ROUTE) != ROUTE_SHA256 or route["town"] != "Town06"
            or Path(plan["route_path"]).resolve() != ROUTE.resolve()
            or plan["route_sha256"] != ROUTE_SHA256 or started["route_sha256"] != ROUTE_SHA256
            or report["route_sha256"] != ROUTE_SHA256):
        raise ValueError("Town06 p06 route identity differs")
    if (plan["map"] != "Town06" or started["map"] != "Town06" or report["town"] != "Town06"
            or plan["capture_mode"] != "mapping-probe" or result["capture_mode"] != "mapping-probe"
            or plan["quality"] != "Epic" or started["quality"] != "Epic"
            or plan.get("source_bytes_archived") is not True or result["exit_code"] != 0
            or result.get("source_bytes_unchanged_and_archived") is not True
            or report["status"] != "COMPLETE_ORACLE_PLUMBING_ONLY"
            or report["physics_driving"] is not True or report["motion_mode"] != "basic-agent"
            or report.get("learned_model_control") is not False
            or report["collision_events"] != 0 or report["live_replay_exact_match"] is not True
            or report["cleanup_errors"] or report["label_table"].get("8") != "traffic_sign"
            or report["frame_count"] != 240):
        raise ValueError("owned physical Town06 mapping proof failed")
    argv = plan["collector_argv"]
    if (set(("--motion-mode", "basic-agent", "--frames", "240", "--ignore-traffic-lights",
             "--town06-sign-scout")) - set(argv)
            or argv[0] != str(SOURCE) or Path(argv[1]).resolve() != ROUTE.resolve()):
        raise ValueError("collector opt-in differs from pinned Town06 plan")
    for stage in ("ready", "after_capture", "stopped"):
        lifecycle = json.loads((OWNER / "lifecycle" / f"{stage}.json").read_text())
        if (lifecycle["status"] != "PASS" or lifecycle["expected_map"] != "Town06"
                or lifecycle["stage"] != stage or lifecycle["owner_pid"] != started["server_pid"]):
            raise ValueError(f"owned CARLA lifecycle failed at {stage}")
        if stage == "stopped" and lifecycle.get("port_released") is not True:
            raise ValueError("owned CARLA port remained active")
    expected_sources = plan["source_sha256"]
    checks = result["source_checks"]
    if set(checks) != set(expected_sources) or not all(checks.values()):
        raise ValueError("owner source checks incomplete")
    for relative, expected in expected_sources.items():
        archived = OWNER / "provenance" / relative
        if not archived.is_file() or archived.is_symlink() or digest(archived) != expected:
            raise ValueError(f"archived source differs: {relative}")
    return {
        "owner_plan_sha256": digest(OWNER / "owner_plan.json"),
        "owner_result_sha256": digest(OWNER / "owner_result.json"),
        "owner_started_sha256": digest(OWNER / "owner_started.json"),
        "mapping_summary_sha256": digest(SOURCE / "summary.json"),
        "archived_source_count": len(expected_sources),
        "route_sha256": ROUTE_SHA256,
        "town": "Town06", "world_lifecycle": "ready/after_capture/stopped PASS",
        "collision_events": 0, "live_replay_exact_match": True,
        "cleanup_errors": report["cleanup_errors"], "port_released_after_stop": True,
        "travel_m": report["travel_m"], "agent_goal_reached": report["agent_goal_reached"],
        "ignores_traffic_lights": report["ignores_traffic_lights"],
        "carla_server_version": report["server_version"],
        "carla_client_version": report["client_version"],
    }


def frame_histogram(values: list[int]) -> dict[str, int]:
    a = np.asarray(values)
    return {"zero": int(np.count_nonzero(a == 0)),
            "one_to_four": int(np.count_nonzero((a >= 1) & (a <= 4))),
            "five_to_64": int(np.count_nonzero((a >= 5) & (a <= 64))),
            "at_least_65": int(np.count_nonzero(a >= 65))}


def component_histogram(sizes: np.ndarray) -> dict[str, int]:
    return {"one_to_four": int(np.count_nonzero((sizes >= 1) & (sizes <= 4))),
            "five_to_64": int(np.count_nonzero((sizes >= 5) & (sizes <= 64))),
            "at_least_65": int(np.count_nonzero(sizes >= 65))}


def component_sizes(mask: np.ndarray) -> np.ndarray:
    """HH_260906 - Count eight-connected label fragments, never inferred object instances."""
    components, _, stats, _ = cv2.connectedComponentsWithStats(
        np.ascontiguousarray(mask.astype(np.uint8)), connectivity=8)
    sizes = stats[1:components, cv2.CC_STAT_AREA].astype(np.int64)
    if int(sizes.sum()) != int(mask.sum()):
        raise ValueError("connected-component area does not match traffic-sign pixels")
    return sizes


def semantic_counts(path: Path) -> tuple[int, np.ndarray]:
    with np.load(path, allow_pickle=False) as arrays:
        semantic = arrays["semantic"]
        if semantic.shape != (360, 640, 4) or semantic.dtype != np.uint8:
            raise ValueError(f"semantic frame format differs: {path}")
        mask = np.ascontiguousarray((semantic[..., 2] == 8).astype(np.uint8))
    return int(mask.sum()), component_sizes(mask)


def audit_frozen_split(split: str) -> dict:
    """HH_260906 - Cross-check fixed AI v2 label scale without changing its splits."""
    manifest_path = FROZEN_DATASET / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    labels = manifest["labels"]
    if labels.count("traffic_sign") != 1 or split not in ("train", "dev"):
        raise ValueError("frozen AI v2 label contract differs")
    metadata = manifest["splits"][split]
    source_summary = metadata["source_summary"]
    if (source_summary["town"] != "Town01"
            or source_summary["route_sha256"] == ROUTE_SHA256
            or len(metadata["source_records"]) != metadata["count"]):
        raise ValueError(f"frozen {split} source is not a distinct Town01 split")
    npz_path = FROZEN_DATASET / metadata["file"]
    if npz_path.is_symlink() or digest(npz_path) != metadata["sha256"]:
        raise ValueError(f"frozen {split} split bytes differ")
    with np.load(npz_path, allow_pickle=False) as arrays:
        target = arrays["target"]
        if target.shape != (metadata["count"], 360, 640) or target.dtype != np.uint8:
            raise ValueError(f"frozen {split} target format differs")
        sizes_per_frame = [component_sizes(frame == labels.index("traffic_sign"))
                           for frame in target]
    sizes = np.concatenate(sizes_per_frame)
    return {
        "frames": metadata["count"],
        "source_town": source_summary["town"],
        "source_route_sha256": source_summary["route_sha256"],
        "manifest_sha256": digest(manifest_path),
        "split_sha256": metadata["sha256"],
        "frames_with_1_to_4_pixel_components": sum(
            int(np.any((part >= 1) & (part <= 4))) for part in sizes_per_frame),
        "8_connected_component_histogram": component_histogram(sizes),
    }


def audit_frames(source: Path, *, verify_rgb: bool) -> tuple[dict, list[dict]]:
    """HH_260906 - Separate whole-frame pixel totals from 8-connected label fragments."""
    rows = json.loads((source / "records.json").read_text())
    if len(rows) != 240 or len({r["stem"] for r in rows}) != 240:
        raise ValueError("expected 240 distinct frame records")
    kept: list[int] = []
    pixels: list[int] = []
    components_per_frame: list[np.ndarray] = []
    examples: list[dict] = []
    previous = None
    previous_frame = None
    for i, row in enumerate(rows):
        stem = f"frames/{i:04d}"
        if row["stem"] != stem or (previous_frame is not None and row["frame"] <= previous_frame):
            raise ValueError("frame stem or CARLA frame ordering differs")
        previous_frame = row["frame"]
        rgb_path, npz_path = source / (stem + ".png"), source / (stem + ".npz")
        if (rgb_path.is_symlink() or npz_path.is_symlink() or not rgb_path.is_file()
                or not npz_path.is_file()):
            raise ValueError("missing or unsafe sensor source")
        if verify_rgb:
            with Image.open(rgb_path) as rgb:
                if rgb.size != (640, 360) or rgb.mode != "RGB":
                    raise ValueError("source RGB format differs")
                rgb.verify()
        total, sizes = semantic_counts(npz_path)
        pixels.append(total)
        components_per_frame.append(sizes)
        xy = np.asarray([row["ego_x"], row["ego_y"]], dtype=np.float64)
        if previous is None or np.linalg.norm(xy - previous) >= 0.25:
            kept.append(i)
            previous = xy
        examples.append({"stem": stem, "total_sign_pixels": total,
                         "tiny_component_count": component_histogram(sizes)["one_to_four"]})
    kept_sizes = np.concatenate([components_per_frame[i] for i in kept])
    raw_sizes = np.concatenate(components_per_frame)
    raw_tiny_frames = sum(int(np.any((s >= 1) & (s <= 4))) for s in components_per_frame)
    kept_tiny_frames = sum(int(np.any((components_per_frame[i] >= 1) &
                                      (components_per_frame[i] <= 4))) for i in kept)
    return {
        "raw_frames": len(rows), "retained_frames_0p25m": len(kept),
        "raw_total_pixel_histogram": frame_histogram(pixels),
        "retained_total_pixel_histogram": frame_histogram([pixels[i] for i in kept]),
        "raw_8_connected_component_histogram": component_histogram(raw_sizes),
        "retained_8_connected_component_histogram": component_histogram(kept_sizes),
        "raw_frames_with_1_to_4_pixel_components": raw_tiny_frames,
        "retained_frames_with_1_to_4_pixel_components": kept_tiny_frames,
        "raw_tag8_pixel_sum": sum(pixels),
        "retained_tag8_pixel_sum": sum(pixels[i] for i in kept),
        "smallest_total_pixel_frame": min((pixels[i], i) for i in kept)[1],
        "largest_total_pixel_frame": max((pixels[i], i) for i in kept)[1],
        "most_tiny_components_frame": max((examples[i]["tiny_component_count"], i) for i in kept)[1],
    }, examples


def typeface(size: int) -> ImageFont.ImageFont:
    font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(font_path), size) if font_path.is_file() else ImageFont.load_default()


def render_examples(summary: dict) -> None:
    """HH_260906 - Show full native frames and crops centered on verified tiny tag fragments."""
    indices = [0, summary["town06"]["largest_total_pixel_frame"],
               summary["town06"]["most_tiny_components_frame"]]
    image = Image.new("RGB", (1760, 77 + 3 * 400 + 62), (247, 249, 252))
    draw = ImageDraw.Draw(image)
    draw.text((20, 16), "Town06 p06: native RGB / oracle semantics / 4x tag8 crops",
              fill=(22, 31, 43), font=typeface(25))
    for row_index, index in enumerate(indices):
        stem = f"frames/{index:04d}"
        with Image.open(SOURCE / (stem + ".png")) as original:
            rgb = original.convert("RGB")
        with np.load(SOURCE / (stem + ".npz"), allow_pickle=False) as arrays:
            tags = arrays["semantic"][..., 2].copy()
        mask = np.ascontiguousarray((tags == 8).astype(np.uint8))
        components, _, stats, centers = cv2.connectedComponentsWithStats(mask, connectivity=8)
        tiny = [index for index in range(1, components)
                if 1 <= stats[index, cv2.CC_STAT_AREA] <= 4]
        if not tiny:
            raise ValueError(f"chosen visual example lacks a tiny tag8 component: {stem}")
        chosen = min(tiny, key=lambda part: abs(centers[part, 0] - 320)
                     + 2 * abs(centers[part, 1] - 115))
        center_x, center_y = centers[chosen]
        left = min(520, max(0, int(round(center_x)) - 60))
        top = min(315, max(0, int(round(center_y)) - 22))
        crop_box = (left, top, left + 120, top + 45)
        if not np.any(mask[top:top + 45, left:left + 120]):
            raise ValueError("selected visual crop missed the sign mask")
        colored = np.empty((360, 640, 3), dtype=np.uint8)
        colored[:] = LABEL_COLORS[0]
        for tag, color in LABEL_COLORS.items():
            colored[tags == tag] = color
        semantic = Image.fromarray(colored)
        y = 77 + row_index * 400
        total = summary["example_frames"][index]["total_sign_pixels"]
        tiny_count = summary["example_frames"][index]["tiny_component_count"]
        draw.text((20, y + 3), f"{stem} | total tag8={total} px | 1-4 px components={tiny_count} | crop={crop_box}",
                  fill=(22, 31, 43), font=typeface(18))
        y += 40
        image.paste(rgb, (0, y))
        image.paste(semantic, (640, y))
        image.paste(rgb.crop(crop_box).resize((480, 180), Image.NEAREST), (1280, y))
        image.paste(semantic.crop(crop_box).resize((480, 180), Image.NEAREST), (1280, y + 180))
        draw.rectangle((left, y + top, left + 119, y + top + 44), outline=(255, 255, 0), width=2)
        draw.rectangle((640 + left, y + top, 640 + left + 119, y + top + 44),
                       outline=(255, 255, 0), width=2)
        draw.rectangle((1280, y, 1759, y + 359), outline=(39, 51, 64), width=2)
    draw.text((20, 77 + 3 * 400 + 17),
              "Magenta=CARLA tag8. Tiny crop fragments are not verified sign instances.",
              fill=(65, 76, 88), font=typeface(17))
    image.save(OUTPUT / "rgb_semantic_horizon_examples.png")


def render_histograms(summary: dict) -> None:
    """HH_260906 - Keep frame-total and connected-component denominators visually distinct."""
    image = Image.new("RGB", (1420, 850), "white")
    draw = ImageDraw.Draw(image)
    heading, body = typeface(25), typeface(18)
    draw.text((20, 14), "Town06 p06: traffic-sign pixels and 8-connected fragments", fill=(22, 31, 43), font=heading)
    draw.text((20, 65), "Whole-frame tag8 pixel total (frames)", fill=(22, 31, 43), font=typeface(21))
    bar_colors = ((116, 132, 150), (230, 67, 163), (237, 164, 56), (66, 143, 208))
    columns = (("zero", "0 px"), ("one_to_four", "1-4 px"),
               ("five_to_64", "5-64 px"), ("at_least_65", ">=65 px"))
    for j, (_, label) in enumerate(columns):
        draw.text((260 + j * 280, 104), label, fill=(22, 31, 43), font=body)
    for i, (label, values) in enumerate((
            ("raw 240", summary["town06"]["raw_total_pixel_histogram"]),
            ("0.25m kept", summary["town06"]["retained_total_pixel_histogram"]))):
        y = 147 + i * 65
        draw.text((20, y + 5), label, fill=(22, 31, 43), font=body)
        for j, (key, _) in enumerate(columns):
            x, count = 260 + j * 280, values[key]
            draw.rectangle((x, y, x + 180, y + 28), fill=(247, 249, 252), outline=(210, 217, 225))
            if count:
                draw.rectangle((x, y, x + max(2, int(count / 240 * 180)), y + 28), fill=bar_colors[j])
            draw.text((x + 189, y + 2), str(count), fill=(22, 31, 43), font=body)
    draw.text((20, 318), "8-connected tag8 component size (components, NOT object instances)",
              fill=(22, 31, 43), font=typeface(21))
    for j, (_, label) in enumerate(columns[1:]):
        draw.text((340 + j * 330, 359), label, fill=(22, 31, 43), font=body)
    comparison_rows = [
        ("Town06 raw", summary["town06"]["raw_8_connected_component_histogram"]),
        ("Town06 kept", summary["town06"]["retained_8_connected_component_histogram"]),
        ("Town01 p02 raw", summary["reference_components"]["p02_right"]["raw_8_connected_component_histogram"]),
        ("Town01 p04 raw", summary["reference_components"]["p04_left"]["raw_8_connected_component_histogram"]),
        ("AI v2 TRAIN", summary["frozen_reference"]["train"]["8_connected_component_histogram"]),
        ("AI v2 DEV", summary["frozen_reference"]["dev"]["8_connected_component_histogram"]),
    ]
    max_components = max(row["one_to_four"] for _, row in comparison_rows)
    for i, (label, values) in enumerate(comparison_rows):
        y = 402 + i * 65
        draw.text((20, y + 5), label, fill=(22, 31, 43), font=body)
        for j, (key, _) in enumerate(columns[1:]):
            x, count = 340 + j * 330, values[key]
            draw.rectangle((x, y, x + 205, y + 28), fill=(247, 249, 252), outline=(210, 217, 225))
            if count:
                draw.rectangle((x, y, x + max(2, int(count / max_components * 205)), y + 28),
                               fill=bar_colors[j + 1])
            draw.text((x + 214, y + 2), str(count), fill=(22, 31, 43), font=body)
    draw.text((20, 812), "A frame may contain many 1-4 px fragments while its total tag8 count exceeds 4 px.",
              fill=(65, 76, 88), font=typeface(17))
    image.save(OUTPUT / "sign_frame_and_component_histograms.png")


def main() -> None:
    owner = require_owned_completion()
    town06, examples = audit_frames(SOURCE, verify_rgb=True)
    references = {}
    for name, source in REFERENCE.items():
        reference, _ = audit_frames(source, verify_rgb=False)
        references[name] = {"raw_frames": reference["raw_frames"],
                            "raw_8_connected_component_histogram": reference["raw_8_connected_component_histogram"],
                            "raw_frames_with_1_to_4_pixel_components": reference["raw_frames_with_1_to_4_pixel_components"]}
    frozen_reference = {split: audit_frozen_split(split) for split in ("train", "dev")}
    threshold = 24
    eligible = town06["retained_total_pixel_histogram"]["one_to_four"] >= threshold
    summary = {
        "schema": "portable_e2e.map_sign_town06_p06_audit.v1",
        "source": str(SOURCE.relative_to(ROOT)),
        "scope": "saved native Town06 oracle RGB/semantic scout; no model training or dataset admission",
        "owner_provenance": owner,
        "native_frame_wh": [640, 360], "semantic_tag": 8,
        "retention_rule": "first frame, then >=0.25m from last retained ego XY",
        "town06": town06,
        "reference_components": references,
        "frozen_reference": frozen_reference,
        "example_frames": examples,
        "predeclared_total_pixel_gate": {
            "minimum_retained_frames_whose_whole_frame_tag8_total_is_1_to_4_pixels": threshold,
            "observed": town06["retained_total_pixel_histogram"]["one_to_four"],
            "passed": eligible,
        },
        "component_interpretation": "8-connected mask islands are fragments, not verified object instances; they do not replace the frame-total admission gate",
        "decision": "NO_TRAIN_EXPANSION" if not eligible else "PENDING_INDEPENDENT_REVIEW",
        "training_performed": False, "dataset_modified": False, "model_approved": False,
        "cross_town_xy_distance_compared": False,
    }
    if eligible:
        raise ValueError("unexpected gate pass requires independent manual review before publication")
    if OUTPUT.is_symlink():
        raise ValueError("output must not be a symlink")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    render_examples(summary)
    render_histograms(summary)
    (OUTPUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"output": str(OUTPUT), "gate": summary["predeclared_total_pixel_gate"],
                      "components": town06["retained_8_connected_component_histogram"]}))


if __name__ == "__main__":
    main()
