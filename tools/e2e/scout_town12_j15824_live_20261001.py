#!/usr/bin/env python3
"""HH_261001 - Bind one Town12 right route to live terrain and traffic-light actors."""

from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import time
from typing import Any
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
REPORT = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static/report_v4.json"
REPORT_SHA256 = "e4ebf3298c6a625126b98f78039dec3acc4cddbab7c2166883045c95d160c2c9"
ROUTE_SHA256 = "98ea2396ebb5e89382f218ea4d388816735002bc4b0f21c0344d8fbcf05a229b"
XODR = Path("/home/a/carla/Unreal/CarlaUE4/Content/Carla/Maps/Town12/OpenDrive/Town12.xodr")
XODR_SHA256 = "26fa53d4902ac3397d35f948969f0c77c1bf023755ede2c67edf00bef817151f"
SOURCE_EGG = Path("/home/a/carla/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg")
SOURCE_PLANNER = Path("/home/a/carla/PythonAPI/carla/agents/navigation/global_route_planner.py")
PLANNER_SHA256 = "9b164a2de723d91d55cd1203543747580c1c6c509ed0294a6de17b001f7db8d0"
STATIC_OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_static_route_v1"
LIVE_NAMES = frozenset(("/Game/Carla/Maps/Town12/Town12", "Carla/Maps/Town12/Town12"))
SIGNAL_ID = "21120"
HOLD_SECONDS = 10.0


class ScoutError(RuntimeError):
    """HH_261001 - Reject an uncertain source, route, actor or physical-site identity."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ScoutError(message)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular input required: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def route_digest(points: list[dict]) -> str:
    return hashlib.sha256(json.dumps(points, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def write_new(path: Path, value: dict) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def static_binding() -> dict:
    require(digest(REPORT) == REPORT_SHA256, "Town12 V4 report changed")
    report = json.loads(REPORT.read_text())
    matches = [row for row in report["physical_pass_routes"] if row["route_sha256"] == ROUTE_SHA256]
    require(len(matches) == 1, "fixed supported route is absent or duplicated")
    row = matches[0]
    require(row["junction_id"] == 15824 and row["connector_road_id"] == 15851
            and row["connector_lane_id"] == 1 and row["nominal"]["status"] == "PASS"
            and row["nominal"]["production_lane_gate_supported"] is True
            and row["other_junction_ids_in_30_146m"] == [], "static route binding changed")
    require(digest(XODR) == XODR_SHA256, "Town12 source XODR changed")
    require(digest(SOURCE_PLANNER) == PLANNER_SHA256, "source planner changed")
    return row


def source_api():
    import carla
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    require(str(carla.__file__).startswith(str(SOURCE_EGG) + "/"), "wrong CARLA Python API")
    require(Path(inspect.getsourcefile(GlobalRoutePlanner)).resolve() == SOURCE_PLANNER,
            "wrong CARLA planner import")
    return carla, GlobalRoutePlanner


def transform(carla, pose: dict):
    xyz = pose["xyz_carla_m"]
    return carla.Transform(carla.Location(*xyz), carla.Rotation(yaw=pose["yaw_carla_deg"]))


def reconstruct(carla_map, row: dict, carla, planner_class) -> tuple[dict, Any]:
    from scripts.e2e.prepare_carla_route import serialize_route, transform_dict
    planner = planner_class(carla_map, 1.0)
    start, goal = transform(carla, row["start"]), transform(carla, row["goal"])
    traced = planner.trace_route(start.location, goal.location)
    points = serialize_route(traced, goal)
    require(route_digest(points) == ROUTE_SHA256, "reconstructed route differs from V4 exact polyline")
    right = [p for p in points if p["road_option"] == "RIGHT"]
    require(right and any(p["road_id"] == 15851 and p["lane_id"] == 1 for p in right),
            "live RIGHT route does not use the selected connector")
    return {
        "schema_version": 1, "town": "Town12", "scenario": "right",
        "coordinate_reference": "base_link", "sampling_resolution_m": 1.0,
        "start_carla_transform": transform_dict(start), "goal_carla_transform": transform_dict(goal),
        "route": points, "route_length_m": points[-1]["distance_m"],
        "static_polyline_sha256": ROUTE_SHA256, "source_report_sha256": REPORT_SHA256,
        "source_xodr_sha256": XODR_SHA256, "dataset_admitted": False,
    }, planner


def project(points: list[dict], x: float, y: float) -> dict:
    """HH_261001 - Project CARLA XY onto the fixed ROS route without rounding progress."""
    best = None
    for a, b in zip(points, points[1:]):
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        squared = dx * dx + dy * dy
        if squared <= 1e-12:
            continue
        ratio = min(1.0, max(0.0, ((x - a["x"]) * dx + (-y - a["y"]) * dy) / squared))
        px, py = a["x"] + ratio * dx, a["y"] + ratio * dy
        error = math.hypot(x - px, -y - py)
        candidate = (error, a["distance_m"] + ratio * (b["distance_m"] - a["distance_m"]), a)
        if best is None or candidate[:2] < best[:2]:
            best = candidate
    require(best is not None, "route contains no nonzero segment")
    return {"cross_track_m": best[0], "progress_m": best[1],
            "road_id": best[2]["road_id"], "lane_id": best[2]["lane_id"]}


def waypoint_record(waypoint, points: list[dict]) -> dict:
    require(waypoint is not None, "traffic-light API returned an absent waypoint")
    location = waypoint.transform.location
    return {"road_id": int(waypoint.road_id), "section_id": int(waypoint.section_id),
            "lane_id": int(waypoint.lane_id), "s": float(waypoint.s),
            "xyz_carla_m": [float(location.x), float(location.y), float(location.z)],
            "yaw_carla_deg": float(waypoint.transform.rotation.yaw),
            "route_projection": project(points, location.x, location.y)}


def signal_xml() -> dict:
    root = ET.fromstring(XODR.read_bytes())
    definitions, references = [], []
    for road in root.findall("road"):
        for signal in road.findall("./signals/*"):
            if signal.get("id") != SIGNAL_ID:
                continue
            record = {"road_id": int(road.get("id")), **signal.attrib,
                      "validity": [dict(item.attrib) for item in signal.findall("validity")],
                      "user_data_xml": [ET.tostring(item, encoding="unicode")
                                        for item in signal.findall("userData")]}
            (definitions if signal.tag == "signal" else references).append(record)
    require(len(definitions) == 1 and definitions[0].get("dynamic") == "yes",
            "controlling dynamic signal definition is absent or ambiguous")
    require(any(r["road_id"] == 15851 and {"fromLane": "1", "toLane": "1"} in r["validity"]
                for r in references), "selected connector has no pinned lane-valid signal reference")
    return {"signal_id": SIGNAL_ID, "definitions": definitions, "references": references,
            "painted_stop_line_xml_proven": False,
            "stop_boundary_method": "TrafficLight.get_stop_waypoints trigger-volume lane center"}


def signal_actor_record(world, carla_map, points: list[dict], evidence_path: Path | None = None) -> dict:
    from agents.tools.misc import get_trafficlight_trigger_location
    actors = list(world.get_actors().filter("traffic.traffic_light*"))
    matches = [actor for actor in actors if str(actor.get_opendrive_id()) == SIGNAL_ID]
    if evidence_path is not None:
        with evidence_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"stage": "actor_identity", "signal_id": SIGNAL_ID,
                "matches": [{"id": int(actor.id), "is_dormant": bool(actor.is_dormant)} for actor in matches],
                "all_traffic_light_actor_count": len(actors)}, sort_keys=True) + "\n")
    require(len(matches) == 1, f"signal {SIGNAL_ID} matched {len(matches)} live actors")
    actor = matches[0]
    require(not actor.is_dormant, "controlling traffic-light actor is dormant")
    affected = [waypoint_record(wp, points) for wp in actor.get_affected_lane_waypoints()]
    require(any(wp["road_id"] == 15851 and wp["lane_id"] == 1 for wp in affected),
            "live signal affected lanes do not include selected connector")
    stops = [waypoint_record(wp, points) for wp in actor.get_stop_waypoints()]
    trigger = waypoint_record(carla_map.get_waypoint(get_trafficlight_trigger_location(actor)), points)
    if evidence_path is not None:
        with evidence_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"stage": "waypoint_alignment", "actor_id": int(actor.id),
                "affected": affected, "stops": stops, "trigger": trigger}, sort_keys=True) + "\n")
    on_route = [wp for wp in stops if wp["route_projection"]["cross_track_m"] <= 0.75
                and 45.0 <= wp["route_projection"]["progress_m"] <= 83.0
                and wp["lane_id"] == wp["route_projection"]["lane_id"]
                and wp["road_id"] == wp["route_projection"]["road_id"]]
    require(len(on_route) == 1, "live trigger stop boundary is absent or ambiguous on the route")
    require(trigger["road_id"] == on_route[0]["road_id"] and trigger["lane_id"] == on_route[0]["lane_id"],
            "BasicAgent trigger waypoint differs from controlled approach lane")
    require(trigger["route_projection"]["cross_track_m"] <= 0.75,
            "BasicAgent trigger waypoint does not align with the selected approach")
    return {"actor_id": int(actor.id), "opendrive_id": str(actor.get_opendrive_id()),
            "state": str(actor.get_state()), "is_dormant": bool(actor.is_dormant),
            "affected_lane_waypoints": affected, "stop_waypoints": stops,
            "selected_stop_waypoint": on_route[0], "basic_agent_trigger_waypoint": trigger,
            "all_traffic_light_actor_count": len(actors),
            "basic_agent_red_hazard_runtime_proven": False,
            "painted_stop_line_alignment_proven": False}


def ground_evidence(world, carla, points: list[dict]) -> list[dict]:
    result = []
    for progress in (0.0, 65.0, 90.0, 120.0, 215.0):
        point = min(points, key=lambda item: abs(item["distance_m"] - progress))
        hit = world.ground_projection(carla.Location(point["x"], -point["y"], point["z"] + 5.0), 12.0)
        require(hit is not None, f"no loaded physical ground near route progress {progress}")
        delta = float(hit.location.z) - float(point["z"])
        require(abs(delta) <= 1.5, f"physical ground differs from route elevation at {progress}")
        result.append({"progress_m": point["distance_m"], "label": str(hit.label),
                       "xyz_carla_m": [float(hit.location.x), float(hit.location.y), float(hit.location.z)],
                       "elevation_difference_m": delta})
    return result


def assert_owner(pid: int, pgid: int) -> None:
    require(pid > 1 and pid == pgid and os.getpgid(pid) == pgid, "source editor owner PID/PGID differs")
    argv = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
    require(argv[0] == b"/home/a/UnrealEngine_4.26/Engine/Binaries/Linux/UE4Editor"
            and b"/Game/Carla/Maps/Town12/Town12" in argv and b"-carla-port=2100" in argv,
            "source editor process does not match owned Town12 command")


def execute(output: Path, owner_pid: int, owner_pgid: int) -> dict:
    row = static_binding()
    assert_owner(owner_pid, owner_pgid)
    carla, planner_class = source_api()
    client = carla.Client("127.0.0.1", 2100)
    client.set_timeout(60.0)
    world = client.get_world()
    identity = int(world.id)
    carla_map = world.get_map()
    require(carla_map.name in LIVE_NAMES, "full live map name differs")
    require(hashlib.sha256(carla_map.to_opendrive().encode()).hexdigest() == XODR_SHA256,
            "live XODR differs before site relocation")
    require(not list(world.get_actors().filter("vehicle.*"))
            and not list(world.get_actors().filter("sensor.*")), "unexpected vehicle or sensor before scout")
    require(not world.get_settings().synchronous_mode, "scout requires owned asynchronous startup world")
    route, _ = reconstruct(carla_map, row, carla, planner_class)
    write_new(output / "route.json", route)
    xml = signal_xml()
    write_new(output / "signal_xml.json", xml)
    spectator = world.get_spectator()
    require(world.get_settings().spectator_as_ego, "spectator does not control streaming without hero")
    location = transform(carla, row["start"])
    location.location.z += 15.0
    location.rotation.pitch = -30.0
    spectator.set_transform(location)
    # HH_261001 - Move only the spectator before creating any cameras; measure loaded geometry.
    deadline = time.monotonic() + 120.0
    last_error = None
    while time.monotonic() < deadline:
        assert_owner(owner_pid, owner_pgid)
        world.wait_for_tick(10.0)
        try:
            ground = ground_evidence(world, carla, route["route"])
            light = signal_actor_record(world, carla_map, route["route"], output / "signal_observations.jsonl")
            break
        except (ScoutError, RuntimeError) as error:
            last_error = str(error)
            with (output / "site_attempts.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps({"error": last_error, "frame": int(world.get_snapshot().frame)}) + "\n")
            time.sleep(0.5)
    else:
        raise ScoutError(f"target physical site or signal not ready: {last_error}")
    write_new(output / "streamed_site.json", {"ground_queries": ground, "signal": light,
              "spectator_relocated_before_sensors": True, "tile_id_api_available": False,
              "method": "ground-projection collision queries plus active controlling signal actor",
              "render_pixels_validated": False})
    first = world.get_snapshot()
    started = time.monotonic()
    observations = []
    while time.monotonic() - started < HOLD_SECONDS:
        assert_owner(owner_pid, owner_pgid)
        snapshot = world.wait_for_tick(10.0)
        require(int(client.get_world().id) == identity, "CARLA episode changed during identity hold")
        live_actor = world.get_actor(light["actor_id"])
        require(live_actor is not None and not live_actor.is_dormant
                and str(live_actor.get_opendrive_id()) == SIGNAL_ID, "controlling light changed during hold")
        observations.append({"frame": int(snapshot.frame), "elapsed_seconds": float(snapshot.timestamp.elapsed_seconds),
                             "actor_id": int(live_actor.id), "state": str(live_actor.get_state())})
    require(observations and observations[-1]["frame"] > int(first.frame), "world did not advance during hold")
    require(world.get_map().name in LIVE_NAMES
            and hashlib.sha256(world.get_map().to_opendrive().encode()).hexdigest() == XODR_SHA256,
            "map identity changed after hold")
    return {"schema": "portable_e2e.town12_j15824_live_scout.v1", "status": "PASS",
            "map_name": carla_map.name, "world_id": identity, "xodr_sha256": XODR_SHA256,
            "route_polyline_sha256": ROUTE_SHA256, "route_file_sha256": digest(output / "route.json"),
            "signal_xml": xml, "controlling_signal": light, "ground_queries": ground,
            "hold_wall_seconds": time.monotonic() - started, "hold_observations": observations,
            "actor_spawned": False, "capture_performed": False, "dataset_admitted": False,
            "next_capture_target_speed_kmh": 20.0, "capture_ready_for_instrumented_trial": True,
            "signal_compliance_proven": False, "painted_stop_line_alignment_proven": False}


def prepare_offline() -> dict:
    row = static_binding()
    require(not STATIC_OUTPUT.exists(), "static route output already exists")
    carla, planner_class = source_api()
    carla_map = carla.Map("offline-Town12-j15824-20261001", XODR.read_text())
    route, _ = reconstruct(carla_map, row, carla, planner_class)
    STATIC_OUTPUT.mkdir(parents=True)
    write_new(STATIC_OUTPUT / "route.json", route)
    result = {"status": "EXACT_STATIC_ROUTE_RECONSTRUCTED", "source_report_sha256": REPORT_SHA256,
              "static_route_polyline_sha256": ROUTE_SHA256, "route_file_sha256": digest(STATIC_OUTPUT / "route.json"),
              "route_length_m": route["route_length_m"], "signal_xml": signal_xml(),
              "live_validated": False, "capture_performed": False, "dataset_admitted": False}
    write_new(STATIC_OUTPUT / "binding.json", result)
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--prepare-offline", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--owner-pid", type=int)
    parser.add_argument("--owner-pgid", type=int)
    args = parser.parse_args(argv)
    if args.execute:
        require(args.output is not None and args.owner_pid is not None and args.owner_pgid is not None,
                "live scout requires explicit owner identity and output")
        require(args.output.is_dir() and not args.output.is_symlink(), "owned scout output directory absent")
        result = {"status": "FAIL", "capture_performed": False, "dataset_admitted": False}
        try:
            result = execute(args.output, args.owner_pid, args.owner_pgid)
        except Exception as error:
            result["error"] = f"{type(error).__name__}: {error}"
        write_new(args.output / "scout_result.json", result)
        print(json.dumps(result, sort_keys=True))
        return 0 if result["status"] == "PASS" else 1
    result = prepare_offline() if args.prepare_offline else {
        "status": "STATIC_BINDING_READY", "route": static_binding(),
        "simulator_contacted": False, "capture_performed": False}
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
