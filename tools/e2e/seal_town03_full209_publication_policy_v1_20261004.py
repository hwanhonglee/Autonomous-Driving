"""HH_261004 - Seal outcome-independent whole-drive media selection before the fresh full209 attempt."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v1"
PLAN = RUN.with_name(RUN.name + "_plan.json")
OUTPUT = ROOT / "docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/11_town03_full_route209"
POLICY = ROOT / "storage/experiments/plans/2026-10-04/town03_known_dev_straight_full209_v1/ROOT_prospective_publication_policy.json"
SCHEMA = "portable_e2e.town03_known_dev_straight_full209_prospective_publication_policy.20261004.v1"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    # HH_261004 - Stream every source byte outside all actual camera/NN timing intervals.
    require(path.is_file() and not path.is_symlink() and path.resolve() == path, "canonical regular source required")
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 ** 2), b""):
            value.update(block)
    return value.hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--plan-sha256", required=True)
    args = parser.parse_args(argv)
    require(not RUN.exists() and not RUN.is_symlink() and not OUTPUT.exists() and not OUTPUT.is_symlink(),
            "whole-media selection must precede actual episode and rendering")
    require(digest(PLAN) == args.plan_sha256, "root prospective full209 declaration changed")
    plan = json.loads(PLAN.read_bytes())
    require(plan["output"] == str(RUN) and plan["total_capture_seconds"] == 110
            and plan["maximum_native_state_count"] == 2200 and plan["maximum_camera_bundle_count"] == 1100
            and plan["attempts"] == 1 and plan["automatic_retry"] is False, "fresh exact full209 bounds required")
    selection = plan["prospective_visual_publication"]
    require(selection["gallery"] == str(OUTPUT.relative_to(ROOT))
            and selection["complete_actual_interval_including_failures_required"] is True
            and selection["outcome_or_failure_dependent_frame_selection"] is False
            and selection["real_six_cameras_required"] is True and selection["vehicle_centered"] is True
            and selection["actual_Autoware_or_RViz_screenshot_claim"] is False
            and selection["illustrative_or_generated_image_allowed"] is False,
            "owner's complete recorded-only sampling contract required")
    for name, expected in plan["source_sha256"].items():
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts, "safe owner source member required")
        require(digest(ROOT / relative) == expected, "owner source changed before visual declaration: " + name)
    value = dict(schema=SCHEMA,
        provenance="HH_261004 - Fix whole actual interval and all failure-tail sampling before one full209 drive; final publisher bytes are independently pinned before rendering, not falsely claimed pre-capture.",
        created_UTC=datetime.now(timezone.utc).isoformat(), created_before_actual_episode=True,
        actual_episode_observed=False, run_root=str(RUN), output=str(OUTPUT),
        run_declaration=dict(path=str(PLAN), sha256=args.plan_sha256), attempts=1, automatic_retry=False,
        media_selection=selection, owner_source_sha256=plan["source_sha256"],
        root_sealer_sha256=digest(Path(__file__).resolve()),
        final_publisher_source_frozen_before_capture_claimed=False,
        final_publisher_and_fonts_must_be_pinned_before_rendering=True,
        known_DEV_site_reused=True, independent_validation=False,
        full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    require(POLICY.resolve() == POLICY and not POLICY.exists() and not POLICY.is_symlink(), "create-only canonical policy required")
    POLICY.parent.mkdir(parents=True, exist_ok=True)
    with POLICY.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps(dict(policy_path=str(POLICY), policy_sha256=digest(POLICY),
        owner_sources_checked=len(plan["source_sha256"]), simulator_started=False, rendered=False)))


if __name__ == "__main__":
    main()
