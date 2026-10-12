#!/usr/bin/env python3
"""HH_260928 - Bound Town13 static sign clusters without starting a simulator."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from typing import Any

from scripts.e2e.rank_town13_static_sign_routes import signal_xy


ROOT = Path(__file__).resolve().parents[2]
XODR = Path("/home/a/carla/Unreal/CarlaUE4/Content/Carla/Maps/Town13/OpenDrive/Town13.xodr")
XODR_SHA256 = "5354326f2fe4d26f0dcb5c82acf462257e638278363ab7adecb97ec3aa1f426f"
OUTPUT = ROOT / "docs/assets/validation/2026-09-28/map_town13_sign_cluster_feasibility/report.json"
NAMES = frozenset(("Sign_Stop", "Sign_Yield"))
MAX_CENTERLINE_OFFSET_M = 8.0
MIN_APPROACH_M = 20.0
MIN_TAIL_M = 20.0
TARGET_MIN_ROUTE_M = 120.0
TARGET_MAX_ROUTE_M = 250.0
DISPLAY_CLUSTER_LIMIT = 30


class AuditError(ValueError):
    """HH_260928 - Refuse to infer a corridor from unverifiable source data."""


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise AuditError(reason)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def parse_signs(xodr: Path) -> tuple[list[dict[str, Any]], dict[str, int]]:
    """HH_260928 - Locate only XODR stop/yield records on complete line/arc roads."""
    signs: list[dict[str, Any]] = []
    seen_roads: set[int] = set()
    seen_records: set[tuple[int, str]] = set()
    road_count = 0
    try:
        for _, road in ET.iterparse(xodr, events=("end",)):
            if road.tag != "road":
                continue
            road_count += 1
            road_id = int(road.attrib["id"])
            require(road_id not in seen_roads, f"duplicate XODR road ID: {road_id}")
            seen_roads.add(road_id)
            selected = [signal for signal in road.findall("signals/signal")
                        if signal.attrib.get("name") in NAMES]
            if selected:
                geometries = []
                for part in road.findall("planView/geometry"):
                    primitive = list(part)
                    require(len(primitive) == 1 and primitive[0].tag in ("line", "arc"),
                            f"unsupported planView geometry on road {road_id}")
                    values = {key: float(part.attrib[key])
                              for key in ("s", "x", "y", "hdg", "length")}
                    values["curvature"] = (float(primitive[0].attrib["curvature"])
                                           if primitive[0].tag == "arc" else 0.0)
                    require(all(math.isfinite(value) for value in values.values())
                            and values["length"] > 0,
                            f"invalid planView geometry on road {road_id}")
                    geometries.append(values)
                require(geometries and geometries == sorted(geometries, key=lambda row: row["s"]),
                        f"unordered or absent planView geometry on road {road_id}")
                road_length = float(road.attrib["length"])
                require(math.isfinite(road_length) and road_length > 0,
                        f"invalid XODR road length: {road_id}")
                for signal in selected:
                    signal_id = signal.attrib.get("id", "")
                    identity = road_id, signal_id
                    require(signal_id and identity not in seen_records,
                            f"duplicate or absent XODR signal identity: {identity}")
                    seen_records.add(identity)
                    s = float(signal.attrib["s"])
                    t = float(signal.attrib["t"])
                    require(math.isfinite(s) and math.isfinite(t)
                            and -1e-6 <= s <= road_length + 1e-6,
                            f"invalid XODR sign s/t on road {road_id}")
                    x, y = signal_xy(geometries, s, t)
                    require(math.isfinite(x) and math.isfinite(y),
                            f"nonfinite XODR sign XY on road {road_id}")
                    signs.append({"record_id": f"{road_id}:{signal_id}",
                                  "road_id": road_id, "signal_id": signal_id,
                                  "name": signal.attrib["name"],
                                  "orientation": signal.attrib.get("orientation"),
                                  "s_m": s, "t_m": t, "x_m": x, "y_m": y})
            road.clear()
    except (ET.ParseError, OSError, KeyError, TypeError, ValueError) as error:
        if isinstance(error, AuditError):
            raise
        raise AuditError(f"Town13 XODR parse failed: {error}") from error
    require(road_count > 0 and signs, "Town13 has no parsed roads or stop/yield signals")
    signs.sort(key=lambda row: (row["road_id"], row["signal_id"]))
    return signs, {"road_count": road_count, "stop_count": sum(s["name"] == "Sign_Stop" for s in signs),
                   "yield_count": sum(s["name"] == "Sign_Yield" for s in signs)}


def three_sign_route_lower_bound(signs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """HH_260928 - The two nearest signs minimize each possible middle sign's XY lower bound."""
    clusters: dict[tuple[str, str, str], dict[str, Any]] = {}
    for center in signs:
        distances = sorted(
            ((math.hypot(center["x_m"] - other["x_m"],
                         center["y_m"] - other["y_m"]), other)
             for other in signs if other["record_id"] != center["record_id"]),
            key=lambda row: (row[0], row[1]["record_id"]))
        if len(distances) < 2:
            break
        (first_distance, first), (second_distance, second) = distances[:2]
        # HH_260928 - Each sign may project up to 8 m from the unknown route centerline.
        lower_bound = (MIN_APPROACH_M + MIN_TAIL_M
                       + max(0.0, first_distance - 2 * MAX_CENTERLINE_OFFSET_M)
                       + max(0.0, second_distance - 2 * MAX_CENTERLINE_OFFSET_M))
        key = tuple(sorted((first["record_id"], center["record_id"], second["record_id"])))
        row = {"record_ids": list(key), "middle_record_id": center["record_id"],
               "names": [record["name"] for record in (first, center, second)],
               "road_ids": [record["road_id"] for record in (first, center, second)],
               "neighbor_euclidean_m": [round(first_distance, 3), round(second_distance, 3)],
               "minimum_possible_route_m": round(lower_bound, 3),
               "all_record_ids_distinct": True,
               "same_road_id": len({record["road_id"] for record in (first, center, second)}) == 1,
               "graph_route_proven": False,
               "camera_visibility_proven": False}
        previous = clusters.get(key)
        if previous is None or (row["minimum_possible_route_m"], row["middle_record_id"]) < (
                previous["minimum_possible_route_m"], previous["middle_record_id"]):
            clusters[key] = row
    return sorted(clusters.values(), key=lambda row: (
        row["minimum_possible_route_m"], row["record_ids"]))


def same_road_s_clusters(signs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """HH_260928 - Rank consecutive same-road triples without assuming directed reachability."""
    grouped: dict[int, list[dict[str, Any]]] = {}
    for sign in signs:
        grouped.setdefault(sign["road_id"], []).append(sign)
    rows = []
    for road_id, records in grouped.items():
        records.sort(key=lambda row: (row["s_m"], row["record_id"]))
        for first, middle, last in zip(records, records[1:], records[2:]):
            span = last["s_m"] - first["s_m"]
            rows.append({"road_id": road_id,
                         "record_ids": [first["record_id"], middle["record_id"], last["record_id"]],
                         "names": [first["name"], middle["name"], last["name"]],
                         "orientations": [first["orientation"], middle["orientation"], last["orientation"]],
                         "s_span_m": round(span, 3),
                         "all_same_orientation": len({first["orientation"], middle["orientation"],
                                                      last["orientation"]}) == 1,
                         "graph_route_proven": False, "camera_visibility_proven": False})
    return sorted(rows, key=lambda row: (row["s_span_m"], row["road_id"], row["record_ids"]))


def build_report(xodr: Path = XODR, expected_sha256: str = XODR_SHA256) -> dict[str, Any]:
    observed = digest(xodr)
    require(observed == expected_sha256, "Town13 XODR SHA256 differs from pinned source")
    signs, counts = parse_signs(xodr)
    clusters = three_sign_route_lower_bound(signs)
    same_road = same_road_s_clusters(signs)
    qualifying_250 = [row for row in clusters if row["minimum_possible_route_m"] <= TARGET_MAX_ROUTE_M]
    return {
        "schema": "portable_e2e.town13_static_sign_cluster_feasibility.v1",
        "status": ("EUCLIDEAN_NECESSARY_CONDITION_PRESENT" if qualifying_250
                   else "STOP_NO_EUCLIDEAN_THREE_SIGN_CORRIDOR"),
        "source_xodr": str(xodr), "source_sha256": observed,
        "counts": {**counts, "distinct_record_count": len(signs),
                   "distinct_road_count_with_signs": len({row["road_id"] for row in signs})},
        "rules": {"signal_names": sorted(NAMES), "record_identity": "road_id:signal_id",
                  "maximum_centerline_offset_m": MAX_CENTERLINE_OFFSET_M,
                  "minimum_approach_m": MIN_APPROACH_M, "minimum_tail_m": MIN_TAIL_M,
                  "target_route_length_m": [TARGET_MIN_ROUTE_M, TARGET_MAX_ROUTE_M],
                  "cluster_rule": "For each possible middle sign, its two nearest other XODR records minimize the XY necessary bound: 20+20+max(0,d1-16)+max(0,d2-16).",
                  "same_road_rule": "Consecutive signs by OpenDRIVE s on one road; s span is not a route or a visibility proof."},
        "euclidean_cluster_counts": {str(bound): sum(
            row["minimum_possible_route_m"] <= bound for row in clusters)
            for bound in (60, 100, 120, 180, 250)},
        "euclidean_cluster_count_total": len(clusters),
        "best_euclidean_clusters": clusters[:DISPLAY_CLUSTER_LIMIT],
        "same_road_triple_count_total": len(same_road),
        "best_same_road_triples": same_road[:DISPLAY_CLUSTER_LIMIT],
        "nominal_240_frame_distance_m": {"9_kph": 60, "15_kph": 100},
        "capture_authorized": False, "catalog_authorized": False,
        "limitations": [
            "The XY bound is necessary but not sufficient for a CARLA-directed road-graph route.",
            "Distinct XODR records may not be distinct RGB-visible physical signs.",
            "Sign orientation, lane applicability, mesh presence, occlusion and semantic pixels are unverified.",
            "240 frames at 10 Hz span 24 s: nominal 9/15 kph distances are 60/100 m; stops reduce them.",
            "No CARLA, UE4Editor, camera, GPU, SSH, training, TEST map or dataset admission was used."],
    }


def main() -> int:
    try:
        report = build_report()
        require(not OUTPUT.exists() and not OUTPUT.is_symlink(),
                "refusing to overwrite an existing Town13 cluster report")
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        with OUTPUT.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
    except (AuditError, OSError) as error:
        print(f"STOP_TOWN13_SIGN_CLUSTER_AUDIT: {error}", file=sys.stderr)
        return 2
    print(f"Town13 cluster audit: {report['status']}; "
          f"best={report['best_euclidean_clusters'][0]['minimum_possible_route_m'] if report['best_euclidean_clusters'] else None} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
