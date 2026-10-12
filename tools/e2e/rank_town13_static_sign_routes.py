#!/usr/bin/env python3
"""HH_260906 - Rank completed Town13 routes by static OpenDRIVE sign opportunities only."""

from __future__ import annotations

import bisect
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
OWNER = ROOT / "storage/experiments/validation/2026-09-27/map_town13_route_catalog_v2"
XODR = Path("/home/a/carla/Unreal/CarlaUE4/Content/Carla/Maps/Town13/OpenDrive/Town13.xodr")
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_town13_static_sign_rank/rank_v2.json"
XODR_SHA256 = "5354326f2fe4d26f0dcb5c82acf462257e638278363ab7adecb97ec3aa1f426f"
MAP_LOAD_NAME = "/Game/Carla/Maps/Town13/Town13"
LIVE_MAP_NAMES = (MAP_LOAD_NAME, "Carla/Maps/Town13/Town13")
PROFILE = "source_editor_0915_4ws"
SIGN_NAMES = frozenset(("Sign_Stop", "Sign_Yield"))
MAX_CENTERLINE_OFFSET_M = 8.0
MIN_APPROACH_AND_TAIL_M = 20.0


class RankError(ValueError):
    """HH_260906 - Missing ownership or unverifiable geometry prevents a route choice."""


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise RankError(reason)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    block_hash = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            block_hash.update(block)
    return block_hash.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    require(path.is_file() and not path.is_symlink(), f"JSON file absent or symlinked: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as error:
        raise RankError(f"invalid JSON: {path}") from error
    require(isinstance(data, dict), f"JSON object required: {path}")
    return data


def completed_owner(owner: Path, xodr: Path) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    """HH_260906 - Consume no catalog until the final owner result and cleanup receipts pass."""
    result = read_json(owner / "owner_result.json")
    require(result.get("schema") == "portable_e2e.owned_town13_catalog_result.v1"
            and result.get("status") == "PASS"
            and result.get("source_bytes_unchanged_and_archived") is True
            and result.get("source_assets_unchanged") is True
            and result.get("owned_processes_stopped_and_ports_released") is True
            and result.get("camera_capture_executed") is False
            and result.get("training_executed") is False
            and result.get("dataset_admission_granted") is False,
            "Town13 catalog owner is absent, incomplete, or failed")
    sealed = result.get("catalog")
    require(isinstance(sealed, dict) and sealed.get("status") == "complete"
            and isinstance(sealed.get("route_count"), int) and sealed["route_count"] > 0,
            "Town13 owner has no sealed complete route catalog")
    plan = read_json(owner / "owner_plan.json")
    require(plan.get("schema") == "portable_e2e.owned_town13_catalog_plan.v1"
            and plan.get("map_id") == "town13" and plan.get("split") == "validation"
            and plan.get("source_editor_profile") == PROFILE
            and plan.get("map_load_name") == MAP_LOAD_NAME
            and plan.get("port_triplet") == [2100, 2101, 2102]
            and plan.get("allow_map_load") is False
            and plan.get("camera_capture_executed") is False
            and plan.get("training_executed") is False
            and plan.get("dataset_admission_granted") is False,
            "Town13 owner plan does not match the fixed catalog-only contract")
    asset = plan.get("source_assets", {}).get("opendrive_path", {})
    require(asset.get("path") == str(xodr)
            and asset.get("sha256") == XODR_SHA256
            and asset.get("expected_sha256") == XODR_SHA256
            and asset.get("ok") is True,
            "Town13 plan is not bound to the pinned OpenDRIVE")
    require(digest(xodr) == XODR_SHA256, "Town13 OpenDRIVE bytes changed")
    sources = plan.get("source_sha256")
    require(isinstance(sources, dict) and len(sources) >= 10,
            "Town13 source archive ledger missing")
    for name, expected in sources.items():
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts
                and isinstance(expected, str) and len(expected) == 64
                and digest(owner / "provenance" / relative) == expected,
                f"Town13 archived source differs: {name}")
    for stage in ("ready", "after_catalog", "stopped"):
        receipt = read_json(owner / "lifecycle" / f"{stage}.json")
        require(receipt.get("status") == "PASS" and receipt.get("stage") == stage
                and receipt.get("expected_map") == "Town13"
                and receipt.get("mode") == ("stopped" if stage == "stopped" else "running"),
                f"Town13 {stage} lifecycle receipt failed")
        if stage == "stopped":
            require(receipt.get("port_released") is True,
                    "Town13 owner did not prove port release")
        else:
            require(receipt.get("active_map_name") in LIVE_MAP_NAMES,
                    f"Town13 {stage} live map differs")
    catalog_path = owner / "route_catalog.json"
    require(digest(catalog_path) == sealed.get("sha256"),
            "Town13 catalog bytes differ from owner seal")
    catalog = read_json(catalog_path)
    server = catalog.get("server", {})
    require(catalog.get("status") == "complete" and catalog.get("map_id") == "town13"
            and catalog.get("canonical_name") == "Town13"
            and catalog.get("load_name") == MAP_LOAD_NAME
            and catalog.get("server_profile") == PROFILE
            and server.get("active_server_profile") == PROFILE
            and server.get("active_map_name") in LIVE_MAP_NAMES
            and server.get("map_load_performed") is False
            and len(catalog.get("routes", [])) == sealed["route_count"],
            "Town13 catalog map or route count differs from owner seal")
    return plan, result, catalog


def route_paths(owner: Path, catalog: dict[str, Any]) -> list[tuple[dict[str, Any], dict[str, Any]]]:
    """HH_260906 - Verify every candidate route before ranking any of them."""
    routes = []
    seen_ids: set[str] = set()
    for item in catalog["routes"]:
        require(isinstance(item, dict) and item.get("status") == "ready"
                and isinstance(item.get("id"), str) and item["id"] not in seen_ids,
                "Town13 catalog route record invalid or duplicated")
        seen_ids.add(item["id"])
        name = item.get("path")
        require(isinstance(name, str), "Town13 route path missing")
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts
                and relative.parts[:2] == ("routes", "town13")
                and relative.suffix == ".json",
                "Town13 route path escapes its catalog")
        path = owner / relative
        require(digest(path) == item.get("sha256"),
                f"Town13 route differs from catalog SHA: {item['id']}")
        data = read_json(path)
        require(data.get("town") == "Town13"
                and data.get("coordinate_reference") == "base_link"
                and data.get("scenario") == item.get("scenario")
                and data.get("start_spawn_index") == item.get("start_spawn_index")
                and data.get("goal_spawn_index") == item.get("goal_spawn_index"),
                f"Town13 route metadata differs: {item['id']}")
        points = data.get("route")
        require(isinstance(points, list) and len(points) >= 2,
                f"Town13 route geometry absent: {item['id']}")
        previous = -math.inf
        for point in points:
            require(isinstance(point, dict) and isinstance(point.get("road_id"), int),
                    f"Town13 road identity invalid: {item['id']}")
            for key in ("x", "y", "distance_m"):
                require(isinstance(point.get(key), (int, float))
                        and math.isfinite(float(point[key])),
                        f"Town13 route {key} invalid: {item['id']}")
            arc = float(point["distance_m"])
            require(arc >= previous, f"Town13 route arc reversed: {item['id']}")
            previous = arc
        require(abs(float(points[0]["distance_m"])) < 1e-6
                and abs(previous - float(data["route_length_m"])) < 0.1,
                f"Town13 route length disagrees: {item['id']}")
        routes.append((item, data))
    return routes


def signal_xy(geometries: list[dict[str, float]], s: float, t: float) -> tuple[float, float]:
    """HH_260906 - Evaluate OpenDRIVE line/arc road-reference coordinates and lateral t."""
    starts = [geometry["s"] for geometry in geometries]
    index = bisect.bisect_right(starts, s) - 1
    require(index >= 0, "XODR signal precedes its road geometry")
    geometry = geometries[index]
    delta = s - geometry["s"]
    require(-1e-6 <= delta <= geometry["length"] + 1e-6,
            "XODR signal lies outside its road geometry")
    heading = geometry["hdg"]
    curvature = geometry["curvature"]
    if abs(curvature) < 1e-12:
        x = geometry["x"] + delta * math.cos(heading)
        y = geometry["y"] + delta * math.sin(heading)
        tangent = heading
    else:
        tangent = heading + curvature * delta
        x = geometry["x"] + (math.sin(tangent) - math.sin(heading)) / curvature
        y = geometry["y"] + (math.cos(heading) - math.cos(tangent)) / curvature
    return x - t * math.sin(tangent), y + t * math.cos(tangent)


def mapped_signs(xodr: Path, road_ids: set[int]) -> list[dict[str, Any]]:
    """HH_260906 - Read only static stop/yield entries on roads used by sealed routes."""
    signs: list[dict[str, Any]] = []
    seen_roads: set[int] = set()
    seen_signs: set[tuple[int, str]] = set()
    try:
        for _, road in ET.iterparse(xodr, events=("end",)):
            if road.tag != "road":
                continue
            road_id = int(road.attrib["id"])
            if road_id in road_ids:
                require(road_id not in seen_roads, "duplicate XODR road ID")
                seen_roads.add(road_id)
                geometries = []
                for part in road.findall("planView/geometry"):
                    primitive = list(part)
                    require(len(primitive) == 1 and primitive[0].tag in ("line", "arc"),
                            f"unsupported XODR planView primitive on road {road_id}")
                    attrs = part.attrib
                    values = {key: float(attrs[key]) for key in ("s", "x", "y", "hdg", "length")}
                    values["curvature"] = (float(primitive[0].attrib["curvature"])
                                           if primitive[0].tag == "arc" else 0.0)
                    require(all(math.isfinite(value) for value in values.values())
                            and values["length"] > 0,
                            f"invalid XODR geometry on road {road_id}")
                    geometries.append(values)
                require(geometries and geometries == sorted(geometries, key=lambda row: row["s"]),
                        f"XODR road {road_id} lacks ordered geometry")
                for signal in road.findall("signals/signal"):
                    if signal.attrib.get("name") not in SIGN_NAMES:
                        continue
                    sign_id = signal.attrib.get("id", "")
                    key = road_id, sign_id
                    require(sign_id and key not in seen_signs,
                            f"duplicate or missing XODR sign ID on road {road_id}")
                    seen_signs.add(key)
                    s, t = float(signal.attrib["s"]), float(signal.attrib["t"])
                    require(math.isfinite(s) and math.isfinite(t),
                            f"nonfinite XODR signal on road {road_id}")
                    # HH_260906 - OpenDRIVE y already matches serialized ROS y; CARLA transform y is inverted.
                    x, y = signal_xy(geometries, s, t)
                    signs.append({"road_id": road_id, "signal_id": sign_id,
                                  "name": signal.attrib["name"], "orientation": signal.attrib.get("orientation"),
                                  "x": x, "y": y})
            road.clear()
    except (ET.ParseError, OSError, KeyError, TypeError, ValueError) as error:
        if isinstance(error, RankError):
            raise
        raise RankError(f"Town13 XODR could not be parsed: {error}") from error
    require(seen_roads == road_ids, "sealed route references XODR road IDs that are absent")
    return signs


def project_sign(points: list[dict[str, Any]], sign: dict[str, Any]) -> tuple[float, float] | None:
    """HH_260906 - Project a mapped sign only onto the route's matching road segments."""
    best: tuple[float, float] | None = None
    for first, second in zip(points, points[1:]):
        if first["road_id"] != sign["road_id"] or second["road_id"] != sign["road_id"]:
            continue
        dx = float(second["x"]) - float(first["x"])
        dy = float(second["y"]) - float(first["y"])
        length2 = dx * dx + dy * dy
        if length2 <= 1e-12:
            continue
        u = max(0.0, min(1.0, ((sign["x"] - first["x"]) * dx
                               + (sign["y"] - first["y"]) * dy) / length2))
        x, y = first["x"] + u * dx, first["y"] + u * dy
        distance = math.hypot(sign["x"] - x, sign["y"] - y)
        arc = float(first["distance_m"]) + u * (float(second["distance_m"]) - float(first["distance_m"]))
        if best is None or distance < best[0]:
            best = distance, arc
    return best


def rank_routes(routes: list[tuple[dict[str, Any], dict[str, Any]]],
                signs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """HH_260906 - Fix ranking before RGB: static sign count, type diversity, then route ID."""
    ranked = []
    for item, data in routes:
        points = data["route"]
        length = float(data["route_length_m"])
        opportunities = []
        for sign in signs:
            projected = project_sign(points, sign)
            if projected is None:
                continue
            lateral, arc = projected
            if lateral <= MAX_CENTERLINE_OFFSET_M and MIN_APPROACH_AND_TAIL_M <= arc <= length - MIN_APPROACH_AND_TAIL_M:
                opportunities.append({"road_id": sign["road_id"], "signal_id": sign["signal_id"],
                                      "name": sign["name"], "orientation": sign["orientation"],
                                      "route_arc_m": round(arc, 3),
                                      "centerline_offset_m": round(lateral, 3)})
        opportunities.sort(key=lambda row: (row["route_arc_m"], row["road_id"], row["signal_id"]))
        ranked.append({"route_id": item["id"], "scenario": item["scenario"],
                       "route_path": item["path"], "route_sha256": item["sha256"],
                       "route_length_m": round(length, 3),
                       "mapped_sign_count": len(opportunities),
                       "mapped_sign_type_count": len({row["name"] for row in opportunities}),
                       "mapped_sign_opportunities": opportunities,
                       "camera_visibility_proven": False,
                       "capture_authorized": False})
    ranked.sort(key=lambda row: (-row["mapped_sign_count"],
                                  -row["mapped_sign_type_count"], row["route_id"]))
    return ranked


def build_report(owner: Path = OWNER, xodr: Path = XODR) -> dict[str, Any]:
    plan, result, catalog = completed_owner(owner, xodr)
    routes = route_paths(owner, catalog)
    roads = {point["road_id"] for _, route in routes for point in route["route"]}
    signs = mapped_signs(xodr, roads)
    ranked = rank_routes(routes, signs)
    selected = next((entry["route_id"] for entry in ranked if entry["mapped_sign_count"] > 0), None)
    return {
        "schema": "portable_e2e.town13_static_sign_route_rank.v2",
        "status": "STATIC_PRIORITY_ONLY" if selected else "STOP_NO_STATIC_SIGN_OPPORTUNITY",
        "source_split": plan["split"],
        "owner_status": result["status"],
        "catalog_sha256": result["catalog"]["sha256"],
        "xodr_sha256": XODR_SHA256,
        "ranking_rule": {"sign_names": sorted(SIGN_NAMES),
                         "same_road_id_required": True,
                         "maximum_centerline_offset_m": MAX_CENTERLINE_OFFSET_M,
                         "minimum_route_approach_and_tail_m": MIN_APPROACH_AND_TAIL_M,
                         "sort": "mapped sign count desc, mapped sign type count desc, route ID asc"},
        "selected_static_priority_route_id": selected,
        "route_count": len(ranked),
        "routes": ranked,
        "limits": ["Static OpenDRIVE object records are not RGB-visible objects.",
                   "Sign orientation, occlusion, rendering, semantic pixels and legal applicability are unverified.",
                   "This ranking does not authorize camera capture, DEV admission, training or deployment."],
    }


def main() -> int:
    try:
        report = build_report()
        require(not OUTPUT.exists() and not OUTPUT.is_symlink(),
                "refusing to overwrite an existing static rank")
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        with OUTPUT.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
    except (RankError, OSError) as error:
        print(f"STOP_TOWN13_STATIC_RANK: {error}", file=sys.stderr)
        return 2
    print(f"Town13 static rank: {report['status']}; selected={report['selected_static_priority_route_id']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
