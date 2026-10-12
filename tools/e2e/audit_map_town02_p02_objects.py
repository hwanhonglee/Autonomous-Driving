"""HH_260906 - Inventory native Town02 p02 sign components without data admission."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "storage/experiments/training/2026-09-27/map_town02_p02_sign_train_scout_epic_v1/mapping"
SCOUT = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_train_scout/summary.json"
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_object_qa"
WIDTH, HEIGHT = 640, 360
TAG = 8
TARGET_FRACTION = 0.95
CARDS_PER_PAGE = 8


def require(condition: bool, message: str) -> None:
    """HH_260906 - Fail closed on broken source contracts."""
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    """HH_260906 - Verify owner bytes against the prior raw audit."""
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def components(mask: np.ndarray, stem: str) -> list[dict]:
    """HH_260906 - Keep 8-connected pixel blobs distinct from physical sign identities."""
    require(mask.shape == (HEIGHT, WIDTH) and mask.dtype == np.bool_,
            "native boolean tag-8 mask required")
    count, _, stats, centroids = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    out: list[dict] = []
    for label in range(1, count):
        x, y, width, height, pixels = (int(value) for value in stats[label])
        out.append({
            "id": f"{stem.rsplit('/', 1)[-1]}_c{label:02d}",
            "stem": stem,
            "label": label,
            "pixels": pixels,
            "bbox_xywh": [x, y, width, height],
            "centroid_xy": [round(float(value), 4) for value in centroids[label]],
            "touches_border": x == 0 or y == 0 or x + width == WIDTH or y + height == HEIGHT,
        })
    require(sum(row["pixels"] for row in out) == int(mask.sum()),
            f"component pixels lost for {stem}")
    return out


def review_prefix(rows: list[dict], total_pixels: int) -> list[dict]:
    """HH_260906 - Select a deterministic minimal queue before manual labels exist."""
    require(total_pixels > 0 and sum(row["pixels"] for row in rows) == total_pixels,
            "component denominator differs")
    ordered = sorted(rows, key=lambda row: (
        -row["pixels"], row["stem"], *row["bbox_xywh"], row["id"]
    ))
    selected: list[dict] = []
    covered = 0
    for row in ordered:
        selected.append(row)
        covered += row["pixels"]
        if covered / total_pixels >= TARGET_FRACTION:
            break
    require((covered - selected[-1]["pixels"]) / total_pixels < TARGET_FRACTION,
            "review queue is not minimal")
    return selected


def propose_tracks(rows: list[dict]) -> tuple[list[dict], dict[str, int]]:
    """HH_260906 - Offer proximity-only tracks; never certify physical identity automatically."""
    grouped: dict[str, list[dict]] = {}
    for row in rows:
        grouped.setdefault(row["stem"], []).append(row)
    tracks: list[dict] = []
    assignment: dict[str, int] = {}
    for frame_index, stem in enumerate(sorted(grouped)):
        used: set[int] = set()
        for row in sorted(grouped[stem], key=lambda item: (-item["pixels"], item["id"])):
            candidates = []
            for track_id, track in enumerate(tracks):
                if track_id in used or not 1 <= frame_index - track["last_frame_index"] <= 3:
                    continue
                old = track["last_component"]
                distance = math.dist(row["centroid_xy"], old["centroid_xy"])
                size = max(*row["bbox_xywh"][2:], *old["bbox_xywh"][2:])
                if distance > max(30.0, min(120.0, 2.0 * size)):
                    continue
                area_ratio = max(row["pixels"], old["pixels"]) / min(
                    row["pixels"], old["pixels"]
                )
                if area_ratio > 16.0:
                    continue
                score = distance / max(12.0, size) + 0.15 * math.log(area_ratio)
                candidates.append((score, track_id))
            candidates.sort()
            if candidates and candidates[0][0] <= 2.0:
                chosen = candidates[0][1]
                ambiguous = len(candidates) > 1 and candidates[1][0] - candidates[0][0] <= 0.2
                track = tracks[chosen]
                track["component_ids"].append(row["id"])
                track["stems"].append(stem)
                track["total_tag8_pixels"] += row["pixels"]
                track["ambiguous_association"] |= ambiguous
                track["last_component"] = row
                track["last_frame_index"] = frame_index
            else:
                chosen = len(tracks)
                tracks.append({
                    "provisional_track_id": chosen,
                    "component_ids": [row["id"]],
                    "stems": [stem],
                    "total_tag8_pixels": row["pixels"],
                    "ambiguous_association": False,
                    "last_component": row,
                    "last_frame_index": frame_index,
                })
            used.add(chosen)
            assignment[row["id"]] = chosen
    public = [{key: value for key, value in track.items()
               if key not in ("last_component", "last_frame_index")}
              for track in tracks]
    return public, assignment


def build_inventory() -> dict:
    """HH_260906 - Recheck every retained source pair and all tag-8 source pixels."""
    scout = json.loads(SCOUT.read_text())
    require(scout["status"] == "REJECTED_SMALL_SIGN_TRAIN_CANDIDATE"
            and scout["retained_frames_0p25m"] == 232
            and scout["retained_tag8_pixel_sum"] == 58074
            and scout["retained_sign_frame_bins"]["1-4"] == 0,
            "p02 pinned scout or rejected tiny-sign gate changed")
    stems = scout["retained_stems"]
    require(len(stems) == len(set(stems)) == 232, "retained stem list differs")
    frame_rows = {row["stem"]: row for row in scout["frames"]}
    all_components: list[dict] = []
    frame_counts = []
    for stem in stems:
        frame = frame_rows[stem]
        require(frame["retained_0p25m"] is True, "nonretained frame selected")
        rgb_path, sensor_path = SOURCE / (stem + ".png"), SOURCE / (stem + ".npz")
        require(rgb_path.is_file() and sensor_path.is_file()
                and not rgb_path.is_symlink() and not sensor_path.is_symlink(),
                f"unsafe or missing native pair: {stem}")
        require(digest(rgb_path) == frame["rgb_sha256"]
                and digest(sensor_path) == frame["semantic_npz_sha256"],
                f"source pair SHA mismatch: {stem}")
        with Image.open(rgb_path) as rgb:
            require(rgb.size == (WIDTH, HEIGHT) and rgb.mode == "RGB",
                    f"RGB full-FOV decode differs: {stem}")
        with np.load(sensor_path, allow_pickle=False) as sensor:
            semantic = sensor["semantic"]
            require(semantic.shape == (HEIGHT, WIDTH, 4)
                    and semantic.dtype == np.uint8, f"semantic decode differs: {stem}")
            mask = semantic[..., 2] == TAG
        count = int(mask.sum())
        require(count == frame["tag8_pixels"], f"frame tag-8 sum differs: {stem}")
        frame_counts.append(count)
        all_components.extend(components(mask, stem))
    total = sum(frame_counts)
    require(total == scout["retained_tag8_pixel_sum"], "retained tag-8 denominator differs")
    queue = review_prefix(all_components, total)
    tracks, assignment = propose_tracks(all_components)
    for row in all_components:
        row["provisional_track_id"] = assignment[row["id"]]
        row["in_95pct_review_queue"] = row["id"] in {item["id"] for item in queue}
    selected_pixels = sum(row["pixels"] for row in queue)
    selected_frames = sorted({row["stem"] for row in queue})
    return {
        "schema": "portable_e2e.town02_p02_object_inventory.v1",
        "source_scout_sha256": digest(SCOUT),
        "owner": str(SOURCE.relative_to(ROOT)),
        "source_route_sha256": scout["provenance"]["route_sha256"],
        "small_sign_gate_passed": False,
        "retained_frames": len(stems),
        "retained_tag8_pixels": total,
        "component_count": len(all_components),
        "component_95pct_review_queue_count": len(queue),
        "component_95pct_review_queue_pixels": selected_pixels,
        "component_95pct_review_queue_fraction": selected_pixels / total,
        "component_95pct_review_queue_frames": len(selected_frames),
        "component_95pct_review_queue_ids": [row["id"] for row in queue],
        "component_95pct_review_queue_gallery_pages": math.ceil(len(queue) / CARDS_PER_PAGE),
        "temporal_tracks_are_proposals_only": True,
        "physical_sign_objects_automatically_certified": 0,
        "inspected_pixels_by_human": 0,
        "human_inspection_fraction": 0.0,
        "rgb_confirmed_distinct_physical_signs": 0,
        "rgb_confirmed_object_positive_frames": 0,
        "invalid_or_uncertain_fraction_within_inspected": None,
        "object_quality_gate": "INCONCLUSIVE_PENDING_MANUAL_REVIEW",
        "dataset_admitted": False,
        "model_trained": False,
        "all_components": all_components,
        "provisional_tracks": tracks,
    }


def _font(size: int):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except OSError:
        return ImageFont.load_default()


def component_card(row: dict) -> Image.Image:
    """HH_260906 - Show native-FOV context and an exact 4x RGB/tag crop."""
    stem = row["stem"]
    with Image.open(SOURCE / (stem + ".png")) as source:
        rgb = source.convert("RGB")
    with np.load(SOURCE / (stem + ".npz"), allow_pickle=False) as sensor:
        mask = sensor["semantic"][..., 2] == TAG
    x, y, width, height = row["bbox_xywh"]
    region = (max(0, x - 8), max(0, y - 8), min(WIDTH, x + width + 8),
              min(HEIGHT, y + height + 8))
    crop = rgb.crop(region)
    tag = mask[region[1]:region[3], region[0]:region[2]]
    crop_arr = np.asarray(crop).copy()
    crop_arr[tag] = (0.7 * crop_arr[tag] + 0.3 * np.array([255, 0, 205])).astype(np.uint8)
    crop4 = Image.fromarray(crop_arr).resize((crop.width * 4, crop.height * 4),
                                            resample=Image.NEAREST)
    require(crop4.width <= 440 and crop4.height <= 440,
            "component crop exceeds fixed contact-sheet cell")
    card = Image.new("RGB", (820, 490), (246, 248, 251))
    draw = ImageDraw.Draw(card)
    draw.text((8, 7), f"{row['id']} | {row['pixels']} px | tentative track {row['provisional_track_id']}",
              fill=(17, 28, 40), font=_font(17))
    context = rgb.resize((320, 180), resample=Image.BILINEAR)
    card.paste(context, (8, 39))
    draw.rectangle((8 + x // 2, 39 + y // 2,
                    8 + (x + width) // 2, 39 + (y + height) // 2),
                   outline=(255, 210, 0), width=2)
    card.paste(crop4, (352, 39))
    draw.text((8, 235), f"Native full FOV 640x360 | bbox {x},{y},{width},{height}",
              fill=(48, 62, 78), font=_font(14))
    draw.text((8, 260), "Right: exact 4x nearest RGB crop; tag 8 tinted magenta.",
              fill=(48, 62, 78), font=_font(14))
    draw.text((8, 284), "CARLA oracle only; not a prediction or object confirmation.",
              fill=(48, 62, 78), font=_font(14))
    return card


def gallery_pages(report: dict):
    """HH_260906 - Preserve a frame-indexed review queue without subjective labels."""
    by_id = {row["id"]: row for row in report["all_components"]}
    ids = report["component_95pct_review_queue_ids"]
    for start in range(0, len(ids), CARDS_PER_PAGE):
        current = ids[start:start + CARDS_PER_PAGE]
        page = Image.new("RGB", (1640, 1960), (230, 234, 239))
        for index, component_id in enumerate(current):
            page.paste(component_card(by_id[component_id]),
                       ((index % 2) * 820, (index // 2) * 490))
        yield f"contact_sheet_{start // CARDS_PER_PAGE + 1:02d}.png", page


def _png_bytes(image: Image.Image) -> bytes:
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


def boundary_preview() -> Image.Image:
    """HH_260906 - Audit start/middle/end road and lane context at native full FOV."""
    source = ROOT / "tools/e2e/audit_map_town02_p02_train_scout.py"
    spec = importlib.util.spec_from_file_location("_p02_native_preview", source)
    require(spec is not None and spec.loader is not None,
            "native p02 preview module missing")
    audit = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(audit)
    scout = json.loads(SCOUT.read_text())
    return audit.base.preview(scout, audit.OWNER,
                              chosen=("frames/0000", "frames/0120", "frames/0239"),
                              title="Town02_Opt p02 boundary QA")


def write_or_check(report: dict, *, write: bool) -> None:
    """HH_260906 - Generate evidence only under docs, never inside the owner source."""
    require(OUTPUT.is_dir() and not OUTPUT.is_symlink(), "QA document folder missing")
    require(not OUTPUT.resolve().is_relative_to(SOURCE.resolve()),
            "QA output overlaps source")
    report_path = OUTPUT / "component_inventory.json"
    boundary_path = OUTPUT / "road_lane_native_fov.png"
    gallery = OUTPUT / "contact_sheets"
    payload = (json.dumps(report, indent=2, ensure_ascii=False) + "\n").encode()
    boundary_bytes = _png_bytes(boundary_preview())
    if write:
        # HH_260906 - Resume only identical generated evidence after an interrupted render.
        require(not report_path.exists() or report_path.read_bytes() == payload,
                "existing QA inventory differs; refusing overwrite")
        gallery.mkdir(exist_ok=True)
        if not report_path.exists():
            report_path.write_bytes(payload)
        require(not boundary_path.exists() or boundary_path.read_bytes() == boundary_bytes,
                "existing boundary preview differs")
        if not boundary_path.exists():
            boundary_path.write_bytes(boundary_bytes)
        for name, image in gallery_pages(report):
            target = gallery / name
            encoded = _png_bytes(image)
            require(not target.exists() or target.read_bytes() == encoded,
                    f"existing contact sheet differs: {name}")
            if not target.exists():
                target.write_bytes(encoded)
    else:
        require(report_path.read_bytes() == payload, "inventory differs from source")
        require(boundary_path.read_bytes() == boundary_bytes,
                "boundary preview differs from source")
        expected = set()
        for name, image in gallery_pages(report):
            expected.add(name)
            require((gallery / name).read_bytes() == _png_bytes(image),
                    f"contact sheet differs: {name}")
        require({path.name for path in gallery.glob("*.png")} == expected,
                "contact sheet set differs")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true",
                        help="create one immutable QA inventory and review gallery")
    args = parser.parse_args()
    report = build_inventory()
    write_or_check(report, write=args.write)
    print(json.dumps({key: report[key] for key in (
        "retained_tag8_pixels", "component_count", "component_95pct_review_queue_count",
        "component_95pct_review_queue_pixels", "component_95pct_review_queue_frames",
        "object_quality_gate")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
