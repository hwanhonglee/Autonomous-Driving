#!/usr/bin/env python3
"""HH_260906 - Select an outcome-independent right-turn window from a multi-maneuver route."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from scripts.e2e import audit_new_long_turn_candidates as source
from scripts.e2e import physical_turn_geometry as physical


# HH_260906 - Keep the 90 m moving-anchor distance and 9 km/h future budget fixed.
APPROACH_M = 30.0
WINDOW_M = 90.0
POST_TURN_M = 30.0
MIN_ROUTE_LEAD_M = 30.0
FUTURE_M = 9.0 / 3.6 * 6.4
FUTURE_GUARD_M = 10.0
PREVIOUS_MANEUVER_GUARD_M = 15.0
MANEUVERS = frozenset({"LEFT", "RIGHT", "STRAIGHT", "CHANGELANELEFT", "CHANGELANERIGHT"})
ALLOWED = MANEUVERS | {"LANEFOLLOW"}


class WindowError(ValueError):
    """HH_260906 - Reject malformed geometry rather than guessing a window."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise WindowError(message)


def _blocks(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    blocks: list[dict[str, Any]] = []
    for index, point in enumerate(points):
        option = point["road_option"]
        if option not in MANEUVERS:
            continue
        if blocks and blocks[-1]["option"] == option and blocks[-1]["end_index"] == index - 1:
            blocks[-1]["end_index"] = index
            blocks[-1]["end_m"] = float(point["distance_m"])
        else:
            blocks.append({"option": option, "start_index": index, "end_index": index,
                           "start_m": float(point["distance_m"]), "end_m": float(point["distance_m"])})
    # HH_260906 - Use the same midpoint command boundaries as physical_turn_geometry.
    for block in blocks:
        first, last = block["start_index"], block["end_index"]
        if first > 0:
            block["start_m"] = 0.5 * (float(points[first - 1]["distance_m"])
                                        + float(points[first]["distance_m"]))
        if last + 1 < len(points):
            block["end_m"] = 0.5 * (float(points[last]["distance_m"])
                                      + float(points[last + 1]["distance_m"]))
    return blocks


def _validated_points(route: dict[str, Any]) -> list[dict[str, Any]]:
    require(route.get("scenario") == "right" and route.get("coordinate_reference") == "base_link",
            "serialized right/base_link route required")
    try:
        points = physical._serialized_points(route)
    except physical.PhysicalTurnGeometryError as error:
        raise WindowError(str(error)) from error
    original = route["route"]
    require(abs(float(points[0]["distance_m"])) <= 1e-6,
            "route progress must begin at zero")
    previous = None
    for index, (point, normalized) in enumerate(zip(original, points)):
        option = normalized["road_option"]
        require(option in ALLOWED, f"unsupported road option at {index}: {option}")
        identity = tuple(point.get(key) for key in ("road_id", "section_id", "lane_id"))
        require(all(type(value) is int for value in identity), f"missing lane identity at {index}")
        if previous is not None and normalized["distance_m"] == previous[0]:
            require(identity == previous[1],
                    f"production lane gate cannot resolve zero-arc lane-ID change at {index}")
        previous = (normalized["distance_m"], identity)
    return points


def _subroute(route: dict[str, Any], start_m: float, end_m: float) -> dict[str, Any]:
    """HH_260906 - Include boundary-straddling original samples without editing their geometry."""
    points = route["route"]
    starts = [index for index, point in enumerate(points) if float(point["distance_m"]) <= start_m + 1e-9]
    ends = [index for index, point in enumerate(points) if float(point["distance_m"]) >= end_m - 1e-9]
    require(bool(starts) and bool(ends), "window boundaries are outside the route")
    first, last = starts[-1], ends[0]
    require(last > first, "window contains fewer than two source samples")
    offset = float(points[first]["distance_m"])
    copied = [{**point, "distance_m": float(point["distance_m"]) - offset}
              for point in points[first:last + 1]]
    return {"scenario": "right", "route_length_m": copied[-1]["distance_m"], "route": copied}


def select_window(route: dict[str, Any]) -> dict[str, Any]:
    """HH_260906 - Fix the earliest statically valid window before any lane or driving outcome."""
    points = _validated_points(route)
    blocks = _blocks(points)
    require(all(block["option"] not in {"LEFT", "CHANGELANELEFT", "CHANGELANERIGHT"}
                for block in blocks), "right-route search contains left turn or lane change")
    report: dict[str, Any] = {
        "schema": "portable_e2e.turn_centered_subwindow.v1",
        "status": "NO_ELIGIBLE_WINDOW",
        "scope": "static serialized-route screen only",
        "window_policy": {
            "approach_before_right_command_m": APPROACH_M,
            "moving_anchor_window_m": WINDOW_M,
            "post_right_command_inside_window_m": POST_TURN_M,
            "route_lead_before_window_m": MIN_ROUTE_LEAD_M,
            "9kph_6p4s_future_distance_m": FUTURE_M,
            "future_clearance_guard_m": FUTURE_GUARD_M,
            "previous_maneuver_guard_m": PREVIOUS_MANEUVER_GUARD_M,
            "candidate_order": "earliest eligible RIGHT block, before nominal lane or episode outcomes",
        },
        "directional_blocks": blocks,
        "candidate_count": 0,
        "rejections": [],
        "selected": None,
        "capture_authorized": False,
        "dataset_admitted": False,
        "training_executed": False,
    }
    if len(blocks) < 2:
        report["rejections"].append("multi-maneuver route requires at least two directional blocks")
        return report

    length = float(route["route_length_m"])
    for block in blocks:
        if block["option"] != "RIGHT":
            continue
        turn_start, turn_end = block["start_m"], block["end_m"]
        start = turn_start - APPROACH_M
        end = start + WINDOW_M
        horizon_end = end + FUTURE_M
        guard_end = horizon_end + FUTURE_GUARD_M
        reasons: list[str] = []
        if start < MIN_ROUTE_LEAD_M - 1e-9:
            reasons.append("less than 30 m route lead before the fixed window")
        if turn_end > end - POST_TURN_M + 1e-9:
            reasons.append("less than 30 m post-turn context inside the 90 m window")
        if guard_end > length + 1e-9:
            reasons.append("full 6.4 s static future plus 10 m guard is unavailable")
        for other in blocks:
            if other is block:
                continue
            if other["end_m"] >= start - PREVIOUS_MANEUVER_GUARD_M - 1e-9 and \
                    other["start_m"] <= guard_end + 1e-9:
                reasons.append("another directional maneuver intersects the protected window/future")
                break
        preflight = None
        if not reasons:
            try:
                subroute = _subroute(route, start, end)
                preflight = physical.analyze_serialized_physical_turn(
                    subroute, physical.SPEED_30KPH_TURN_GEOMETRY_CONTRACT)
                if preflight["status"] != "PASS":
                    reasons.append("unchanged physical-turn profile failed on isolated subroute")
            except (WindowError, physical.PhysicalTurnGeometryError) as error:
                reasons.append(f"isolated physical-turn geometry invalid: {error}")
        if reasons:
            report["rejections"].append({"right_block_start_index": block["start_index"],
                                         "reasons": reasons})
            continue
        report["status"] = "STATIC_WINDOW_CANDIDATE"
        report["candidate_count"] = 1
        report["selected"] = {
            "right_block_start_index": block["start_index"],
            "right_block_end_index": block["end_index"],
            "right_block_start_m": turn_start,
            "right_block_end_m": turn_end,
            "window_start_m": start,
            "window_end_m": end,
            "future_end_m_at_9kph": horizon_end,
            "protected_maneuver_free_end_m": guard_end,
            "route_length_m": length,
            "physical_turn_preflight": preflight,
        }
        break
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True,
                        help="SHA256 from an independently verified owned catalog")
    args = parser.parse_args()
    try:
        route, digest = source.read_json(args.route, args.expected_sha256)
        report = select_window(route)
    except (source.AuditError, WindowError) as error:
        parser.exit(2, f"ERROR: {error}\n")
    report["route_sha256"] = digest
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "STATIC_WINDOW_CANDIDATE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
