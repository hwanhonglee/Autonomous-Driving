#!/usr/bin/env python3
"""HH_261001 - Audit a new Town01 p02/p04 capture before conditional admission."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from . import audit_map_paired_rgb_20261001_v2 as base_audit
from . import collect_map_paired_town01_p02_p04_20261001 as capture
from . import run_owned_map_paired_town01_p02_p04_20261001 as owner


def audit(source: Path) -> dict:
    source = source.absolute()
    if source not in (owner.CAMPAIGN / "p02/mapping", owner.CAMPAIGN / "p04/mapping"):
        raise ValueError("source must be the canonical p02 or p04 raw capture")
    plan_path = source.parent / "owner_plan.json"
    plan = json.loads(plan_path.read_text())
    route, sha = capture.checked_route(Path(plan["route_path"]))
    expected_name = "p02" if route["scenario"] == "right" else "p04"
    if source.parent.name != expected_name or plan.get("route_sha256") != sha:
        raise ValueError("scoped p02/p04 route and owner differ")
    if (plan.get("schema") != "portable_e2e.map_paired_town01_p02_p04_owner.v1"
            or plan.get("worker_path") != "tools/e2e/collect_map_paired_town01_p02_p04_20261001.py"
            or plan.get("frames") != 240 or plan.get("quality") != "Epic"
            or plan.get("source_assets") != capture.base.package_assets("Town01")
            or plan.get("source_bytes_archived") is not True
            or plan.get("external_source_bytes_archived") is not True):
        raise ValueError("prospective owner scope or packaged source differs")
    required = set(owner.SOURCES)
    if not required.issubset(plan["source_sha256"]):
        raise ValueError("scoped owner source closure is incomplete")
    route_ref = plan["route_archive"]
    if (route_ref["sha256"] != sha or route_ref["path"] != str(capture.ROUTE_ROOT / route["scenario"] /
            capture.ROUTES[sha][1]) or route_ref["archived_path"] != "provenance/route.json"
            or capture.base.digest(source.parent / route_ref["archived_path"]) != sha):
        raise ValueError("independent original route archive differs")
    result = json.loads((source.parent / "owner_result.json").read_text())
    if (result.get("route_archive_unchanged") is not True
            or result.get("collector_cleanup_ok") is not True
            or result.get("server_cleanup_ok") is not True
            or result.get("after_capture_ok") is not True
            or result.get("stopped_ok") is not True
            or result.get("error") is not None
            or result.get("qa_status") != "PENDING_QA"):
        raise ValueError("owner lifecycle or pending QA receipt differs")
    summary = json.loads((source / "summary.json").read_text())
    if (summary.get("capture_study") != "town01_p02_p04_same_tick_20261001_v1"
            or summary.get("capture_site") != expected_name
            or summary.get("qa_status") != "PENDING_QA"
            or summary.get("dataset_admitted") is not False
            or summary.get("train_eligible") is not False):
        raise ValueError("new capture identity or initial admission status differs")
    if owner.tree_bytes(source.parent) > owner.MAX_OWNER_BYTES or owner.tree_bytes(owner.CAMPAIGN) > owner.MAX_CAMPAIGN_BYTES:
        raise ValueError("capture exceeded its evidence storage cap")
    with capture.scoped_parent():
        report = base_audit.audit(source)
    report.update(
        schema="portable_e2e.map_paired_town01_p02_p04_qa.v1",
        source_scope=expected_name,
        intended_role=capture.ROLE,
        route_archive_sha256=sha,
        dataset_admitted=False,
        train_eligible=False,
        ctrack_train_eligible=False,
        independent_dev_approved=False,
        manual_render_review_required=True,
        scope="new same-tick Town01 p02/p04 FALSE/TRUE appearance candidate; 240-frame prefix",
        full_route_completed=False,
        autonomous_driving_pass=False,
        hd_map_approved=False,
        base_auditor_sha256=report["auditor_sha256"],
        auditor_sha256=capture.base.digest(Path(__file__).resolve()),
    )
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    source = args.source.absolute()
    output = args.output.absolute()
    expected = owner.CAMPAIGN / "qa" / f"{source.parent.name}.json"
    if output != expected or output.exists() or output.is_symlink():
        parser.error("fresh QA output must be the canonical p02/p04 receipt")
    report = audit(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({key: report[key] for key in (
        "status", "source_scope", "frame_count", "travel_m", "road_pixels", "lane_pixels")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
