#!/usr/bin/env python3
"""HH_260929 - Read-only consistency audit for Town12 V4 junction progress."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DIR = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static"
LEDGER = DIR / "progress_v4.jsonl"
REPORT = DIR / "report_v4.json"
TOTAL_JUNCTIONS = 628
COUNT_KEYS = frozenset({
    "right_connectors", "endpoint_pairs", "unreachable_pairs", "length_at_least_146m",
    "traverses_connector", "single_right_no_other_maneuver",
    "right_block_on_selected_connector", "turn_inside_30_120m",
    "physical_turn_pass", "nominal_pass", "nominal_fail", "nominal_unsupported",
    "signal_record_free_nominal_pass", "no_other_junction_nominal_pass",
})


class LedgerError(ValueError):
    """HH_260929 - A partial or changed ledger cannot support a route conclusion."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise LedgerError(message)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def route_hash_digest(rows: list[dict]) -> str:
    hashes = sorted(row["route_sha256"] for row in rows)
    return hashlib.sha256(json.dumps(hashes, separators=(",", ":")).encode()).hexdigest()


def audit(ledger_path: Path, report_path: Path | None = None) -> dict:
    require(ledger_path.is_file() and not ledger_path.is_symlink(),
            "progress ledger must be a regular file")
    lines = ledger_path.read_text().splitlines()
    require(bool(lines), "progress ledger has no completed junction")
    previous_counts = {key: 0 for key in COUNT_KEYS}
    previous_id = -1
    last = None
    for index, line in enumerate(lines, start=1):
        try:
            row = json.loads(line)
        except json.JSONDecodeError as error:
            raise LedgerError(f"invalid progress line {index}: {error}") from error
        require(row.get("schema") == "portable_e2e.town12_right_static_progress.v4" and
                row.get("total_junctions") == TOTAL_JUNCTIONS and
                row.get("completed_junctions") == index and
                row.get("partial_not_qualifying_evidence") is True,
                f"progress header changed at line {index}")
        junction_id = row.get("last_completed_junction_id")
        require(type(junction_id) is int and junction_id > previous_id,
                f"junction IDs are not unique and increasing at line {index}")
        previous_id = junction_id
        counts = row.get("cumulative_counts")
        require(isinstance(counts, dict) and set(counts) == COUNT_KEYS and
                all(type(value) is int and value >= previous_counts[key]
                    for key, value in counts.items()),
                f"cumulative counts decreased or changed keys at line {index}")
        require(counts["physical_turn_pass"] == counts["nominal_pass"] +
                counts["nominal_fail"] + counts["nominal_unsupported"],
                f"physical/body count mismatch at line {index}")
        stage = ("endpoint_pairs", "length_at_least_146m", "traverses_connector",
                 "single_right_no_other_maneuver", "right_block_on_selected_connector",
                 "turn_inside_30_120m", "physical_turn_pass")
        require(all(counts[a] >= counts[b] for a, b in zip(stage, stage[1:])) and
                counts["endpoint_pairs"] >= counts["unreachable_pairs"] +
                counts["length_at_least_146m"],
                f"stage counts are inconsistent at line {index}")
        route_hash = row.get("cumulative_physical_route_hashes_sha256")
        require(isinstance(route_hash, str) and len(route_hash) == 64 and
                all(char in "0123456789abcdef" for char in route_hash),
                f"invalid route digest at line {index}")
        previous_counts = counts
        last = row
    require(len(lines) <= TOTAL_JUNCTIONS, "progress exceeds registered junction count")
    result = {
        "status": "COMPLETE_LEDGER" if len(lines) == TOTAL_JUNCTIONS else "PARTIAL_OPERATIONS_ONLY",
        "completed_junctions": len(lines),
        "total_junctions": TOTAL_JUNCTIONS,
        "last_completed_junction_id": previous_id,
        "counts": previous_counts,
        "ledger_sha256": digest(ledger_path),
        "route_digest_sha256": last["cumulative_physical_route_hashes_sha256"],
    }
    if report_path is not None:
        require(len(lines) == TOTAL_JUNCTIONS and
                report_path.is_file() and not report_path.is_symlink(),
                "a final report requires all registered junctions")
        report = json.loads(report_path.read_text())
        rows = report.get("physical_pass_routes")
        require(report.get("schema") == "portable_e2e.town12_right_static_scout.v4" and
                report.get("progress_completed_junctions") == TOTAL_JUNCTIONS and
                report.get("progress_ledger_sha256") == result["ledger_sha256"] and
                report.get("counts") == previous_counts and
                isinstance(rows, list) and len(rows) == counts["physical_turn_pass"] and
                route_hash_digest(rows) == result["route_digest_sha256"],
                "final report disagrees with completed progress ledger")
        result["status"] = "COMPLETE_REPORT_CONSISTENT"
        result["report_sha256"] = digest(report_path)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ledger", type=Path, default=LEDGER)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.ledger, args.report), indent=2))


if __name__ == "__main__":
    main()
