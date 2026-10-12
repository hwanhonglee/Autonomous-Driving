"""HH_260906 - Audit the byte-pinned Town02 p02 scout without training or admission."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OWNER = ROOT / "storage/experiments/training/2026-09-27/map_town02_p02_sign_train_scout_epic_v1"
ROUTE = ROOT / (
    "storage/experiments/validation/2026-09-01/"
    "autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/"
    "town02_opt/catalog/straight/routes/town02_opt/straight/"
    "town02_opt_straight_s0000_p02.json"
)
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_train_scout"
ROUTE_SHA256 = "cad81f4cdfbc9dbcad8c3cc53092e6fd80f6d84dc4766b6d73758fa88dec6810"
REPORT_NAME = "summary.json"
PICTURE_NAME = "town02_p02_rgb_oracle_full_fov.png"
RESEARCH_INTENT = "town02_p02_sign_train_scout_only"
COLLECTOR_SCOPE = "--town02-sign-train-scout-p02"


def _load_audit_module():
    """HH_260906 - Reuse the audited native-sensor decoder and unchanged p00 thresholds."""
    source = ROOT / "tools/e2e/audit_map_town02_train_scout.py"
    spec = importlib.util.spec_from_file_location("_town02_map_audit_helpers", source)
    if spec is None or spec.loader is None:
        raise RuntimeError("Town02 source-audit helpers unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


base = _load_audit_module()


def choose_visual_stems(summary: dict) -> tuple[str, str, str]:
    """HH_260906 - Show low and high sign-count scenes plus a later native full-FOV view."""
    retained = [frame for frame in summary["frames"] if frame["retained_0p25m"]]
    base.common.require(len(retained) >= 3, "three retained visual scenes required")
    candidates = [min(retained, key=lambda row: (row["tag8_pixels"], row["stem"])),
                  max(retained, key=lambda row: (row["tag8_pixels"], row["stem"])),
                  retained[-1], retained[len(retained) // 2]]
    stems = []
    for row in candidates:
        if row["stem"] not in stems:
            stems.append(row["stem"])
        if len(stems) == 3:
            break
    base.common.require(len(stems) == 3, "three distinct visual scenes required")
    return tuple(stems)


def build_summary() -> dict:
    """HH_260906 - Apply the preregistered p00-derived gate to this independent owner."""
    summary = base.build_summary(OWNER, ROUTE, route_sha256=ROUTE_SHA256,
                                 research_split_intent=RESEARCH_INTENT,
                                 collector_scope=COLLECTOR_SCOPE)
    if summary["status"] == "REJECTED_SIGN_RICH_TRAIN_CANDIDATE":
        # HH_260906 - This gate rejects only tiny whole-frame signs, not every sign scale.
        summary["status"] = "REJECTED_SMALL_SIGN_TRAIN_CANDIDATE"
    summary["schema"] = "portable_e2e.map_town02_p02_sign_train_scout_audit.v1"
    summary["scope"] = (
        "pinned Town02_Opt p02 Epic raw scout; preregistered small-sign objective only; "
        "no model inference, training or dataset admission"
    )
    summary["visual_stems"] = list(choose_visual_stems(summary))
    summary["object_level_rgb_label_correspondence_qualified"] = False
    summary["other_sign_scale_admission_not_decided"] = True
    return summary


def preview(summary: dict):
    """HH_260906 - Preserve all 640x360 RGB, oracle and tag-8 overlay panels."""
    return base.preview(summary, OWNER, chosen=tuple(summary["visual_stems"]),
                        title="Town02_Opt p02 Epic scout")


def write_or_check(summary: dict, picture, *, write: bool = False) -> None:
    """HH_260906 - Keep generated evidence apart from immutable owner source."""
    base.write_or_check(summary, picture, OUTPUT, write=write, source_owner=OWNER,
                        report_name=REPORT_NAME, picture_name=PICTURE_NAME)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true", help="write fresh report and preview once")
    args = parser.parse_args()
    summary = build_summary()
    write_or_check(summary, preview(summary), write=args.write)
    print(json.dumps({"status": summary["status"],
                      "retained_frames_0p25m": summary["retained_frames_0p25m"],
                      "retained_sign_frame_bins": summary["retained_sign_frame_bins"]},
                     ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
