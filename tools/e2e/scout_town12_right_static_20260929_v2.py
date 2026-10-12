#!/usr/bin/env python3
"""HH_260929 - Resume the frozen Town12 search with typed unsupported seams."""

from __future__ import annotations

import json
from pathlib import Path

from scripts.e2e import scout_town12_right_static_20260929 as base


ROOT = Path(__file__).resolve().parents[2]
PREREG = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static/preregister_v2.json"
OUTPUT = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static/report_v2.json"
ORIGINAL_CLEARANCE = base.nominal.provisional_nominal_clearance


def guarded_clearance(*args: object, **kwargs: object) -> dict:
    """HH_260929 - A >5 mm lane seam has no supported body verdict."""
    try:
        return ORIGINAL_CLEARANCE(*args, **kwargs)
    except base.nominal.lane.Unsupported as error:
        return {"status": "UNSUPPORTED", "error_type": type(error).__name__,
                "reason": str(error)}


def preregistered_inputs() -> dict:
    manifest = base.preregistered_inputs()
    base.require(PREREG.is_file() and not PREREG.is_symlink(),
                 "v2 preregistration is missing")
    plan = json.loads(PREREG.read_text())
    base.require(plan.get("schema") == "portable_e2e.town12_right_static_preregister.v2" and
                 plan.get("output_path") == str(OUTPUT.relative_to(ROOT)) and
                 plan.get("base_preregister_sha256") == base.digest(base.PREREG) and
                 plan.get("base_search_script_sha256") == base.digest(Path(base.__file__)) and
                 plan.get("v2_script_sha256") == base.digest(Path(__file__)) and
                 plan.get("typed_exception") ==
                 "scripts.e2e.audit_future_actor_lane_containment.Unsupported" and
                 plan.get("classification") == "UNSUPPORTED",
                 "v2 preregistration changed")
    base.require(not OUTPUT.exists() and not OUTPUT.is_symlink() and
                 OUTPUT.parent.is_dir() and not OUTPUT.parent.is_symlink(),
                 "v2 output must be new in a regular directory")
    return manifest


def main() -> None:
    manifest = preregistered_inputs()
    import carla
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    base.nominal.provisional_nominal_clearance = guarded_clearance
    report = base.scan(carla, GlobalRoutePlanner, manifest)
    report["schema"] = "portable_e2e.town12_right_static_scout.v2"
    report["preregistration_sha256"] = base.digest(PREREG)
    report["search_source_sha256"] = base.digest(Path(__file__))
    report["base_search_source_sha256"] = base.digest(Path(base.__file__))
    report["first_attempt"] = {
        "status": "STOPPED_WITHOUT_REPORT",
        "reason": "authorized route lanes have a gap wider than the 5mm endpoint tolerance",
        "exception_type": "scripts.e2e.audit_future_actor_lane_containment.Unsupported",
        "v1_preregistration_sha256": base.digest(base.PREREG),
    }
    base.require(not OUTPUT.exists() and not OUTPUT.is_symlink(),
                 "registered v2 output appeared during scan")
    OUTPUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["counts"], indent=2))


if __name__ == "__main__":
    main()
