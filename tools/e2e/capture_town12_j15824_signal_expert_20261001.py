#!/usr/bin/env python3
"""HH_261001 - Collect the fixed Town12 right turn with six cameras and signal witnesses."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
from typing import Iterator

from scripts.e2e import collect_carla_vad_expert as collector
from scripts.e2e import scout_town12_j15824_live_20261001 as scout


ROOT = Path(__file__).resolve().parents[2]
TARGET_SPEED_KMH = 20.0


def collector_arguments(output: Path, route: Path) -> list[str]:
    return [str(output / "episode"), str(route), "--host", "127.0.0.1", "--port", "2100",
        "--expected-live-xodr-sha256", scout.XODR_SHA256, "--target-speed-kmh", "20",
        "--max-duration-sec", "150", "--physics-hz", "20", "--capture-hz", "10",
        "--stationary-warmup-sec", "2", "--stationary-tail-sec", "6.5",
        "--goal-stop-profile", "comfortable_v1", "--goal-tolerance-m", "1",
        "--control-transport", "acknowledged_batch", "--agent-initialization", "after_bootstrap",
        "--observe-traffic-lights", "--vehicle-type", "vehicle.toyota.prius",
        "--role-name", "hero", "--weather", "ClearNoon", "--seed", "0"]


def camera_contract() -> dict:
    specs = collector.load_camera_specs(collector.DEFAULT_MAPPING, collector.DEFAULT_CALIBRATION)
    return {"source_mapping": str(collector.DEFAULT_MAPPING),
            "source_mapping_sha256": scout.digest(collector.DEFAULT_MAPPING),
            "source_calibration": str(collector.DEFAULT_CALIBRATION),
            "source_calibration_sha256": scout.digest(collector.DEFAULT_CALIBRATION),
            "source_specs": [asdict(spec) for spec in specs],
            "applied_specs": [asdict(replace(spec, enable_postprocess_effects=True)) for spec in specs],
            "changed_fields": ["enable_postprocess_effects"],
            "geometry_order_and_calibration_changed": False,
            "reason": "Explicit source-editor RGB rendering variant; visual domain requires QA."}


def lane_sequence(points: list[dict]) -> list[tuple[int, int, int]]:
    sequence = []
    for point in points:
        lane = tuple(int(point[key]) for key in ("road_id", "section_id", "lane_id"))
        if not sequence or lane != sequence[-1]:
            sequence.append(lane)
    return sequence


def validate_plan(points: list[dict], expected: list[dict]) -> None:
    scout.require(lane_sequence(points) == lane_sequence(expected),
                  "runtime center-shifted BasicAgent plan uses a different lane sequence")
    right = [point for point in points if point["road_option"] == "RIGHT"]
    scout.require(right and any(point["road_id"] == 15851 and point["lane_id"] == 1 for point in right),
                  "runtime BasicAgent plan does not contain fixed RIGHT connector")
    scout.require(not any(point["road_option"] in {"LEFT", "CHANGELANELEFT", "CHANGELANERIGHT"}
                          for point in points), "runtime plan introduced another maneuver")


def signal_verdict(current: dict, previous: dict | None, crossed: bool) -> tuple[bool, str | None]:
    """HH_261001 - Classify measured front-bumper crossings without changing signal states."""
    state, distance = current["signal_state"], current["front_bumper_stop_distance_m"]
    if state not in {"Red", "Yellow", "Green"}:
        return crossed, "controlling traffic-light state is unavailable"
    if not crossed and distance <= 0:
        if previous is None or previous["signal_state"] != "Green" or state != "Green":
            return crossed, "front bumper crossed trigger stop boundary without two observed green frames"
        crossed = True
    if not crossed and state == "Red" and distance < 0.5 and current["speed_mps"] > 0.2:
        return crossed, "moving front bumper reached the red stop boundary"
    return crossed, None


def validate_scout_receipt(directory: Path) -> dict:
    receipt = json.loads((directory / "scout_result.json").read_text())
    scout.require(receipt.get("status") == "PASS" and receipt.get("xodr_sha256") == scout.XODR_SHA256
                  and receipt.get("route_polyline_sha256") == scout.ROUTE_SHA256
                  and receipt.get("hold_wall_seconds", 0) >= scout.HOLD_SECONDS,
                  "complete same-run signal scout receipt is required before actor creation")
    scout.require(scout.digest(directory / "route.json") == receipt.get("route_file_sha256"),
                  "live scout route file changed")
    return receipt


@contextmanager
def instrumented_collector(output: Path, route: dict, receipt: dict) -> Iterator[None]:
    import carla
    import agents.navigation.basic_agent as basic
    import agents.navigation.global_route_planner as global_planner
    from scripts.e2e.prepare_carla_route import serialize_route

    original_agent, original_planner = basic.BasicAgent, global_planner.GlobalRoutePlanner
    original_specs, original_observe, original_write = (collector.load_camera_specs,
        collector.traffic_light_observation, collector._write_json)
    signal_id = int(receipt["controlling_signal"]["actor_id"])
    stop = receipt["controlling_signal"]["selected_stop_waypoint"]
    stop_x, stop_y, _ = stop["xyz_carla_m"]
    stop_yaw = math.radians(stop["yaw_carla_deg"])
    previous = None
    crossed = False
    plan_evidence = []
    camera = camera_contract()

    class CheckedPlanner(original_planner):
        def trace_route(self, origin, destination):
            plan = super().trace_route(origin, destination)
            goal = carla.Transform(destination)
            goal.rotation = plan[-1][0].transform.rotation
            points = serialize_route(plan, goal)
            validate_plan(points, route["route"])
            plan_evidence.append({"runtime_center_polyline_sha256": scout.route_digest(points),
                                  "lane_sequence": lane_sequence(points),
                                  "original_base_link_polyline_sha256": scout.ROUTE_SHA256,
                                  "center_shift_m": collector.WHEELBASE_M / 2})
            return plan

    class WitnessAgent(original_agent):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            scout.require(self._ignore_traffic_lights is False and self._ignore_vehicles is False,
                          "BasicAgent hazard handling was disabled")
            scout.require(any(int(light.id) == signal_id for light in self._lights_list),
                          "BasicAgent light cache omits the bound controlling light")
            self.town12_hazard = None

        def _affected_by_traffic_light(self, *args, **kwargs):
            detected, light = super()._affected_by_traffic_light(*args, **kwargs)
            self.town12_hazard = {"detected": bool(detected),
                "actor_id": None if light is None else int(light.id),
                "source": "single original BasicAgent._affected_by_traffic_light call"}
            return detected, light

    def specs(*args, **kwargs):
        return tuple(replace(spec, enable_postprocess_effects=True) for spec in original_specs(*args, **kwargs))

    def observe(world, ego, agent, frame, *, basic_agent_run_step_this_frame):
        nonlocal previous, crossed
        original = original_observe(world, ego, agent, frame,
                                    basic_agent_run_step_this_frame=basic_agent_run_step_this_frame)
        scout.require(int(world.id) == receipt["world_id"], "episode changed before signal observation")
        light = world.get_actor(signal_id)
        scout.require(light is not None and not light.is_dormant
                      and str(light.get_opendrive_id()) == scout.SIGNAL_ID,
                      "bound controlling light is absent, dormant or changed")
        transform = ego.get_transform()
        forward = transform.get_forward_vector()
        vertices = ego.bounding_box.get_world_vertices(transform)
        front = max(vertices, key=lambda point: point.x * forward.x + point.y * forward.y + point.z * forward.z)
        distance = (stop_x - front.x) * math.cos(stop_yaw) + (stop_y - front.y) * math.sin(stop_yaw)
        velocity = ego.get_velocity()
        current = {"frame": int(frame), "timestamp": float(world.get_snapshot().timestamp.elapsed_seconds),
            "signal_actor_id": signal_id, "signal_opendrive_id": scout.SIGNAL_ID,
            "signal_state": str(light.get_state()).rsplit(".", 1)[-1],
            "front_bumper_stop_distance_m": float(distance),
            "front_corner_carla_xyz_m": [float(front.x), float(front.y), float(front.z)],
            "speed_mps": math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2),
            "basic_agent_red_hazard": getattr(agent, "town12_hazard", None)
                if basic_agent_run_step_this_frame else None,
            "basic_agent_ignore_traffic_lights": bool(agent._ignore_traffic_lights),
            "stop_boundary_source": "bound TrafficLight.get_stop_waypoints trigger-volume lane center",
            "painted_stop_line_alignment_proven": False}
        scout.require(world.get_snapshot().frame == frame, "signal geometry read crossed physics frames")
        crossed, failure = signal_verdict(current, previous, crossed)
        current.update(crossed_bound_stop_boundary=crossed, failure=failure)
        with (output / "signal_frames.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(current, sort_keys=True, allow_nan=False) + "\n")
            stream.flush()
        previous = current
        if failure:
            raise collector.CollectionError(failure)
        return {**original, "town12_signal": current}

    def write(path, value):
        if path.name == "manifest.json":
            value = {**value, "town12_signal_trial": {
                "source_scout_world_id": receipt["world_id"], "signal_actor_id": signal_id,
                "signal_opendrive_id": scout.SIGNAL_ID, "target_speed_kmh": TARGET_SPEED_KMH,
                "traffic_lights_ignored": False, "runtime_plan_checks": plan_evidence,
                "camera_render_variant": camera, "dataset_admitted": False,
                "qualification_30_kph": "NOT_CLAIMED", "admission_status": "PENDING_QA"}}
        original_write(path, value)

    try:
        global_planner.GlobalRoutePlanner, basic.BasicAgent = CheckedPlanner, WitnessAgent
        collector.load_camera_specs, collector.traffic_light_observation, collector._write_json = specs, observe, write
        yield
    finally:
        global_planner.GlobalRoutePlanner, basic.BasicAgent = original_planner, original_agent
        collector.load_camera_specs, collector.traffic_light_observation, collector._write_json = original_specs, original_observe, original_write


def execute(output: Path, scout_directory: Path, owner_pid: int, owner_pgid: int) -> dict:
    scout.assert_owner(owner_pid, owner_pgid)
    receipt = validate_scout_receipt(scout_directory)
    carla, _ = scout.source_api()
    client = carla.Client("127.0.0.1", 2100)
    client.set_timeout(60.0)
    world = client.get_world()
    scout.require(int(world.id) == receipt["world_id"], "scout belongs to a different CARLA episode")
    scout.require(world.get_map().name in scout.LIVE_NAMES, "wrong full map name before capture")
    route_path = scout_directory / "route.json"
    route = collector.load_route(route_path)
    scout.signal_actor_record(world, world.get_map(), route["route"])
    scout.ground_evidence(world, carla, route["route"])
    args = collector.parse_args(collector_arguments(output, route_path))
    with instrumented_collector(output, route, receipt):
        episode = collector.run(args)
    manifest = json.loads((episode / "manifest.json").read_text())
    return {"status": "PASS", "admission_status": "PENDING_QA", "dataset_admitted": False,
            "capture_performed": True, "actor_spawned": True, "episode": str(episode),
            "manifest_sha256": scout.digest(episode / "manifest.json"),
            "target_speed_kmh": TARGET_SPEED_KMH, "qualification_30_kph": "NOT_CLAIMED",
            "collector_result": manifest.get("result"), "camera_contract": camera_contract(),
            "signal_journal_sha256": scout.digest(output / "signal_frames.jsonl"),
            "still_required": ["continuous moving 30s", "all chosen anchors have actual future64",
                               "nine motion bounds", "actual body/sweep lane containment",
                               "signal boundary compliance", "six-camera visual QA", "split admission"]}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--scout", type=Path)
    parser.add_argument("--owner-pid", type=int)
    parser.add_argument("--owner-pgid", type=int)
    args = parser.parse_args(argv)
    if not args.execute:
        cameras = camera_contract()
        print(json.dumps({"status": "STATIC_CAPTURE_CONTRACT", "camera_order": collector.MODEL_CAMERA_ORDER,
                          "camera_geometry_unchanged": True, "postprocess_effects": True,
                          "mapping_sha256": cameras["source_mapping_sha256"],
                          "calibration_sha256": cameras["source_calibration_sha256"],
                          "target_speed_kmh": TARGET_SPEED_KMH, "simulator_contacted": False,
                          "dataset_admitted": False}, indent=2, sort_keys=True))
        return 0
    scout.require(all(value is not None for value in (args.output, args.scout, args.owner_pid, args.owner_pgid)),
                  "capture requires same-run scout, output and explicit owner identity")
    scout.require(args.output.is_dir() and not args.output.is_symlink(), "owned capture directory absent")
    try:
        result = execute(args.output, args.scout, args.owner_pid, args.owner_pgid)
    except Exception as error:
        episodes = [args.output / "episode", args.output / "episode.partial"]
        result = {"status": "FAIL", "error": f"{type(error).__name__}: {error}",
                  "capture_performed": any(any(path.glob("images/*/*.jpg")) for path in episodes),
                  "actor_spawned": None, "dataset_admitted": False, "admission_status": "REJECTED_OR_PENDING_QA"}
    scout.write_new(args.output / "capture_result.json", result)
    print(json.dumps({key: result[key] for key in ("status", "error", "capture_performed", "episode", "admission_status")
                      if key in result}, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
