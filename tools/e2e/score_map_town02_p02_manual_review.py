"""HH_260906 - Score only explicit human-reviewed Town02 p02 sign components."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
INVENTORY = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_object_qa/component_inventory.json"
GEOMETRY = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_object_qa/world_geometry.json"
DECISIONS = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_object_qa/manual_review_decisions.json"
ADDENDUM = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_object_qa/manual_review_addendum.json"
NATIVE_EVIDENCE = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_object_qa/native_fov_evidence/manifest.json"
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_object_qa/manual_review_score.json"
SOURCE_FILE = ROOT / "tools/e2e/audit_map_town02_p02_objects.py"
SPEC = importlib.util.spec_from_file_location("_p02_object_inventory", SOURCE_FILE)
assert SPEC is not None and SPEC.loader is not None
objects = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(objects)


def score() -> dict:
    """HH_260906 - Leave all unlisted tag-8 pixels outside the inspected numerator."""
    inventory = json.loads(INVENTORY.read_text())
    geometry = json.loads(GEOMETRY.read_text())
    decisions = json.loads(DECISIONS.read_text())
    addendum = json.loads(ADDENDUM.read_text())
    native_evidence = json.loads(NATIVE_EVIDENCE.read_text())
    objects.require(geometry["component_inventory_sha256"] == objects.digest(INVENTORY)
                    and inventory["retained_tag8_pixels"] == 58074
                    and inventory["component_95pct_review_queue_count"] == 241,
                    "source inventory or world geometry changed")
    objects.require(addendum["base_manual_decisions_sha256"] == objects.digest(DECISIONS)
                    and addendum["source_inventory_sha256"] == objects.digest(INVENTORY)
                    and native_evidence["source_inventory_sha256"] == objects.digest(INVENTORY),
                    "review addendum or native evidence source differs")
    components = {row["id"]: row for row in inventory["all_components"]}
    world = {row["id"]: row for row in geometry["all_component_world_centroids"]}
    queue = inventory["component_95pct_review_queue_ids"]
    page_count = math.ceil(len(queue) / objects.CARDS_PER_PAGE)

    # HH_260906 - Score the original decisions and the separately audited new sheets.
    def checked_ids(document: dict, name: str, sheets: list[int]) -> list[str]:
        objects.require(isinstance(sheets, list) and sheets
                        and sheets == sorted(set(sheets))
                        and all(isinstance(page, int) and 1 <= page <= page_count
                                for page in sheets), f"{name} contact-sheet list is invalid")
        expected = {
            identifier
            for page in sheets
            for identifier in queue[(page - 1) * objects.CARDS_PER_PAGE:
                                    page * objects.CARDS_PER_PAGE]
        }
        labels = document["labels"]
        actual = [identifier for group in labels["rgb_confirmed_sign"].values()
                  for identifier in group]
        actual += labels["invalid_non_sign"] + labels["uncertain"]
        objects.require(len(actual) == len(set(actual)) and set(actual) == expected,
                        f"{name} declared sheets and actual decisions differ")
        return actual

    # HH_260906 - The byte-preserved original named these four sheets in prose.
    original_sheets = [1, 2, 3, 19]
    added_sheets = addendum["reviewed_contact_sheets"]
    original_ids = checked_ids(decisions, "original", original_sheets)
    added_ids = checked_ids(addendum, "addendum", added_sheets)
    objects.require(not set(original_ids).intersection(added_ids),
                    "addendum overwrites original decision")
    objects.require(not set(original_sheets).intersection(added_sheets),
                    "review sheets overlap")
    objects.require(set(original_ids + added_ids) == set(queue),
                    "review decisions do not cover the frozen queue")
    native_pages = addendum["reviewed_native_fov_pages"]
    objects.require(native_pages == list(range(1, len(native_evidence["pages"]) + 1))
                    and native_evidence["native_tile_wh"] == [640, 360]
                    and len(native_evidence["frames"]) == inventory["component_95pct_review_queue_frames"]
                    and {row["stem"] for row in native_evidence["frames"]}
                    == {components[key]["stem"] for key in queue},
                    "native full-FOV frame ledger differs from frozen queue")

    confirmed_groups = {name: list(ids) for name, ids in
                        decisions["labels"]["rgb_confirmed_sign"].items()}
    for name, ids in addendum["labels"]["rgb_confirmed_sign"].items():
        confirmed_groups.setdefault(name, []).extend(ids)
    invalid = decisions["labels"]["invalid_non_sign"] + addendum["labels"]["invalid_non_sign"]
    uncertain = decisions["labels"]["uncertain"] + addendum["labels"]["uncertain"]
    valid = [identifier for group in confirmed_groups.values() for identifier in group]
    all_ids = valid + invalid + uncertain
    objects.require(len(all_ids) == len(set(all_ids)), "duplicate manual review decision")
    objects.require(set(all_ids) <= set(components), "review references unknown component")
    reviewed_pixels = sum(components[key]["pixels"] for key in all_ids)
    valid_pixels = sum(components[key]["pixels"] for key in valid)
    invalid_pixels = sum(components[key]["pixels"] for key in invalid)
    uncertain_pixels = sum(components[key]["pixels"] for key in uncertain)
    objects.require(reviewed_pixels == valid_pixels + invalid_pixels + uncertain_pixels,
                    "review pixel accounting differs")
    confirmed_frame_stems = sorted({components[key]["stem"] for key in valid
                                    if components[key]["pixels"] >= 5})
    group_evidence = {}
    group_centroids = {}
    for name, ids in confirmed_groups.items():
        objects.require(len({components[key]["stem"] for key in ids}) >= 2,
                        "distinct RGB sign requires repeated frames")
        xyz = np.array([world[key]["world_median_xyz"] for key in ids], dtype=float)
        objects.require(np.isfinite(xyz).all(), "confirmed object has invalid world geometry")
        center = np.median(xyz, axis=0)
        max_drift = float(np.max(np.linalg.norm(xyz[:, :2] - center[:2], axis=1)))
        objects.require(max_drift <= 0.5,
                        f"manual object {name} not spatially stationary: {max_drift}")
        group_centroids[name] = center
        group_evidence[name] = {
            "reviewed_components": len(ids),
            "reviewed_frames": len({components[key]["stem"] for key in ids}),
            "reviewed_pixels": sum(components[key]["pixels"] for key in ids),
            "median_world_xyz_m": [round(float(v), 6) for v in center],
            "max_component_center_drift_xy_m": round(max_drift, 6),
        }
    minimum_group_separation = min(
        float(np.linalg.norm(group_centroids[left][:2] - group_centroids[right][:2]))
        for left in group_centroids for right in group_centroids if left < right
    )
    objects.require(minimum_group_separation >= 3.0,
                    "RGB object groups too close for conservative physical separation")
    denominator = inventory["retained_tag8_pixels"]
    inspected_fraction = reviewed_pixels / denominator
    uncertain_invalid_fraction = (invalid_pixels + uncertain_pixels) / reviewed_pixels
    object_gate = len(confirmed_groups) >= 3
    frame_gate = len(confirmed_frame_stems) >= 24
    quality_gate = uncertain_invalid_fraction <= 0.05
    coverage_gate = inspected_fraction >= 0.95
    if not coverage_gate:
        verdict = "INCONCLUSIVE_INCOMPLETE_VISUAL_COVERAGE"
    elif not all((quality_gate, object_gate, frame_gate)):
        verdict = "REJECTED_FIXED_OBJECT_GATE"
    else:
        verdict = "PASS_NOT_ADMITTED"
    return {
        "schema": "portable_e2e.town02_p02_manual_object_score.v2",
        "source_inventory_sha256": objects.digest(INVENTORY),
        "source_geometry_sha256": objects.digest(GEOMETRY),
        "manual_decisions_sha256": objects.digest(DECISIONS),
        "manual_addendum_sha256": objects.digest(ADDENDUM),
        "native_evidence_manifest_sha256": objects.digest(NATIVE_EVIDENCE),
        "reviewed_contact_sheets": sorted(original_sheets + added_sheets),
        "reviewed_native_fov_frames": len(native_evidence["frames"]),
        "reviewed_component_count": len(all_ids),
        "reviewed_tag8_pixels": reviewed_pixels,
        "retained_tag8_pixels": denominator,
        "inspected_fraction": inspected_fraction,
        "unreviewed_tag8_pixels": denominator - reviewed_pixels,
        "rgb_confirmed_sign_pixels": valid_pixels,
        "invalid_non_sign_pixels": invalid_pixels,
        "uncertain_pixels": uncertain_pixels,
        "invalid_or_uncertain_fraction_within_inspected": uncertain_invalid_fraction,
        "rgb_confirmed_distinct_physical_signs": len(confirmed_groups),
        "rgb_confirmed_object_positive_retained_frames": len(confirmed_frame_stems),
        "minimum_group_separation_world_xy_m": round(minimum_group_separation, 6),
        "group_evidence": group_evidence,
        "gate_components": {
            "coverage_at_least_95pct": coverage_gate,
            "invalid_or_uncertain_at_most_5pct": quality_gate,
            "distinct_rgb_signs_at_least_3": object_gate,
            "confirmed_5px_frames_at_least_24": frame_gate,
        },
        "small_sign_gate_passed": False,
        "object_quality_gate": verdict,
        "dataset_admitted": False,
        "model_trained": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    report = score()
    payload = (json.dumps(report, indent=2, ensure_ascii=False) + "\n").encode()
    if args.write:
        objects.require(not OUTPUT.exists(), "review score already exists")
        OUTPUT.write_bytes(payload)
    else:
        objects.require(OUTPUT.read_bytes() == payload, "review score differs")
    print(json.dumps({key: report[key] for key in (
        "reviewed_tag8_pixels", "inspected_fraction",
        "invalid_or_uncertain_fraction_within_inspected",
        "rgb_confirmed_distinct_physical_signs",
        "rgb_confirmed_object_positive_retained_frames", "object_quality_gate")},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
