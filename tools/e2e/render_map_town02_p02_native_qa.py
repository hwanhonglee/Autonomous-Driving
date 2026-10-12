"""HH_260906 - Publish native RGB, oracle, and tag-8 views for p02 object QA."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SCOUT = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_train_scout/summary.json"
INVENTORY = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_object_qa/component_inventory.json"
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_object_qa/native_fov_evidence"
SOURCE = ROOT / "tools/e2e/audit_map_town02_p02_train_scout.py"
SPEC = importlib.util.spec_from_file_location("_p02_native_scout", SOURCE)
assert SPEC is not None and SPEC.loader is not None
scout = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(scout)
require = scout.base.common.require

# HH_260906 - These fixed examples cover object appearance over time and every selected border touch.
FEATURED_PAGES = (
    ("0000", "0006", "0008"),
    ("0015", "0016", "0017"),
    ("0020", "0048", "0050"),
    ("0058", "0065", "0070"),
    ("0083", "0084", "0088"),
    ("0089", "0090", "0208"),
    ("0223", "0238", "0239"),
)


def sha256(path: Path) -> str:
    """HH_260906 - Hash the immutable source bytes or a published panel."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build() -> tuple[dict, dict[str, Image.Image]]:
    """HH_260906 - Recheck source hashes before rendering exact 640x360 tiles."""
    summary = json.loads(SCOUT.read_text())
    inventory = json.loads(INVENTORY.read_text())
    frames = {row["stem"]: row for row in summary["frames"]}
    retained = set(summary["retained_stems"])
    require(summary["native_frame_wh"] == [640, 360], "native source size differs")
    components = {row["id"]: row for row in inventory["all_components"]}
    queue_frames = {components[identifier]["stem"].split("/")[-1]
                    for identifier in inventory["component_95pct_review_queue_ids"]}
    featured = [identifier for page in FEATURED_PAGES for identifier in page]
    require(set(featured) <= queue_frames, "featured native frame is outside the review queue")
    remaining = sorted(queue_frames - set(featured))
    pages = FEATURED_PAGES + tuple(tuple(remaining[start:start + 3])
                                   for start in range(0, len(remaining), 3))
    identifiers = [item for page in pages for item in page]
    require(len(identifiers) == len(set(identifiers)), "duplicate native evidence frame")
    require(set(identifiers) == queue_frames
            and len(queue_frames) == inventory["component_95pct_review_queue_frames"],
            "native evidence frames differ from preregistered queue")
    records = []
    for identifier in identifiers:
        stem = f"frames/{identifier}"
        require(stem in retained, f"unretained evidence frame: {stem}")
        frame = frames[stem]
        rgb = scout.OWNER / "mapping" / f"{stem}.png"
        semantic = scout.OWNER / "mapping" / f"{stem}.npz"
        require(rgb.is_file() and semantic.is_file(), f"source pair missing: {stem}")
        require(not rgb.is_symlink() and not semantic.is_symlink(),
                f"source pair is symlinked: {stem}")
        require(sha256(rgb) == frame["rgb_sha256"], f"source RGB hash differs: {stem}")
        require(sha256(semantic) == frame["semantic_npz_sha256"],
                f"source semantic hash differs: {stem}")
        with np.load(semantic, allow_pickle=False) as sensor:
            tags = sensor["semantic"][..., 2]
            require(tags.shape == (360, 640), f"semantic shape differs: {stem}")
            require(int(np.count_nonzero(tags == 8)) == frame["tag8_pixels"],
                    f"native tag-8 total differs: {stem}")
        records.append({
            "stem": stem,
            "rgb_source_sha256": frame["rgb_sha256"],
            "semantic_npz_source_sha256": frame["semantic_npz_sha256"],
            "native_tag8_pixels": frame["tag8_pixels"],
        })
    pictures = {}
    page_records = []
    for index, page in enumerate(pages, start=1):
        name = f"native_fov_sheet_{index:02d}.png"
        image = scout.base.preview(
            summary, scout.OWNER,
            chosen=tuple(f"frames/{identifier}" for identifier in page),
            title=f"Town02 p02 object QA native evidence {index:02d}",
        )
        require(image.mode == "RGB" and image.size == (1940, 1306),
                "native evidence panel size differs")
        pictures[name] = image
        page_records.append({"file": name, "frames": [f"frames/{identifier}" for identifier in page]})
    manifest = {
        "schema": "portable_e2e.town02_p02_native_object_qa_evidence.v1",
        "source_scout_sha256": sha256(SCOUT),
        "source_inventory_sha256": sha256(INVENTORY),
        "native_tile_wh": [640, 360],
        "panel_wh": [1940, 1306],
        "meaning": "All preregistered queue frames at native full FOV; first seven pages highlight temporal and border cases. Original component sheets have 320x180 context panels.",
        "oracle_is_model_prediction": False,
        "dataset_admitted": False,
        "frames": records,
        "pages": page_records,
    }
    return manifest, pictures


def write_or_check(manifest: dict, pictures: dict[str, Image.Image], *, write: bool) -> None:
    """HH_260906 - Keep derived evidence separate from the source capture."""
    require(not OUTPUT.resolve().is_relative_to(scout.OWNER.resolve()),
            "native evidence output overlaps source")
    payload = (json.dumps(manifest, indent=2, ensure_ascii=False) + "\n").encode()
    if write:
        require(not OUTPUT.exists(), "native evidence output already exists")
        OUTPUT.mkdir()
        (OUTPUT / "manifest.json").write_bytes(payload)
        for name, picture in pictures.items():
            (OUTPUT / name).write_bytes(scout.base.picture_bytes(picture))
        return
    require(OUTPUT.is_dir() and not OUTPUT.is_symlink(), "native evidence folder missing")
    require((OUTPUT / "manifest.json").read_bytes() == payload,
            "native evidence manifest differs")
    for name, picture in pictures.items():
        path = OUTPUT / name
        require(path.is_file() and not path.is_symlink(), f"native evidence panel missing: {name}")
        with Image.open(path) as saved:
            require(saved.mode == "RGB" and saved.size == picture.size,
                    f"native evidence panel shape differs: {name}")
            require(np.array_equal(np.asarray(saved), np.asarray(picture)),
                    f"native evidence panel pixels differ: {name}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    manifest, pictures = build()
    write_or_check(manifest, pictures, write=args.write)
    print(json.dumps({"native_frames": len(manifest["frames"]),
                      "native_panels": len(manifest["pages"])}, ensure_ascii=False))


if __name__ == "__main__":
    main()
