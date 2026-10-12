#!/usr/bin/env python3

import argparse
import json
import math
import os
import time
from pathlib import Path

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.local_planner import RoadOption


TURN_OPTIONS = {
    RoadOption.LEFT,
    RoadOption.RIGHT,
    RoadOption.STRAIGHT,
    RoadOption.CHANGELANELEFT,
    RoadOption.CHANGELANERIGHT,
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create a deterministic CARLA global route for the Autoware VAD runner."
    )
    parser.add_argument("--host", default=os.environ.get("CARLA_HOST", "localhost"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("CARLA_PORT", "2000")))
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--town", default="Town01")
    parser.add_argument("--weather", default="ClearNoon")
    parser.add_argument(
        "--scenario",
        choices=("lane_follow", "straight", "left", "right", "any"),
        default="lane_follow",
    )
    parser.add_argument("--start-index", type=int)
    parser.add_argument("--goal-index", type=int)
    parser.add_argument("--min-distance", type=float, default=6.0)
    parser.add_argument("--max-distance", type=float, default=60.0)
    parser.add_argument("--preferred-distance", type=float, default=25.0)
    parser.add_argument("--sampling-resolution", type=float, default=1.0)
    parser.add_argument("--max-traces", type=int, default=5000)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if (args.start_index is None) != (args.goal_index is None):
        parser.error("--start-index and --goal-index must be provided together")
    if args.port <= 0 or args.timeout <= 0 or args.sampling_resolution <= 0:
        parser.error("port, timeout, and sampling resolution must be positive")
    if not 0 < args.min_distance <= args.max_distance:
        parser.error("distance range is invalid")
    if args.max_traces <= 0:
        parser.error("--max-traces must be positive")
    return args


def route_length(route):
    return sum(
        route[index - 1][0].transform.location.distance(route[index][0].transform.location)
        for index in range(1, len(route))
    )


def route_matches(route, scenario):
    if scenario == "any":
        return True
    options = {option for _, option in route}
    if scenario == "lane_follow":
        return not options.intersection(TURN_OPTIONS)
    if scenario == "straight":
        return RoadOption.STRAIGHT in options and not options.intersection(
            {RoadOption.LEFT, RoadOption.RIGHT, RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT}
        )
    if scenario == "left":
        return RoadOption.LEFT in options and RoadOption.RIGHT not in options
    if scenario == "right":
        return RoadOption.RIGHT in options and RoadOption.LEFT not in options
    return False


def select_route(planner, spawn_points, args):
    if args.start_index is not None:
        count = len(spawn_points)
        if not 0 <= args.start_index < count or not 0 <= args.goal_index < count:
            raise RuntimeError(f"spawn indices must be in [0, {count - 1}]")
        pairs = [(0.0, args.start_index, args.goal_index)]
    else:
        pairs = []
        for start_index, start in enumerate(spawn_points):
            for goal_index, goal in enumerate(spawn_points):
                if start_index == goal_index:
                    continue
                direct_distance = start.location.distance(goal.location)
                if args.min_distance * 0.6 <= direct_distance <= args.max_distance * 1.2:
                    pairs.append(
                        (abs(direct_distance - args.preferred_distance), start_index, goal_index)
                    )
        pairs.sort(key=lambda item: (item[0], item[1], item[2]))

    best = None
    trace_count = 0
    for _, start_index, goal_index in pairs:
        if trace_count >= args.max_traces:
            break
        trace_count += 1
        # CARLA 0.9.15 keeps the previous junction decision on the planner instance.
        # Reset it so route catalog generation does not depend on pair iteration order.
        planner._intersection_end_node = -1
        planner._previous_decision = RoadOption.VOID
        try:
            route = planner.trace_route(
                spawn_points[start_index].location, spawn_points[goal_index].location
            )
        except (KeyError, RuntimeError):
            continue
        if len(route) < 2 or not route_matches(route, args.scenario):
            continue
        length = route_length(route)
        if not args.min_distance <= length <= args.max_distance:
            continue
        score = abs(length - args.preferred_distance)
        maneuver_count = sum(
            option not in (RoadOption.LANEFOLLOW, RoadOption.VOID) for _, option in route
        )
        candidate = (score, maneuver_count, start_index, goal_index, length, route)
        if best is None or candidate[:4] < best[:4]:
            best = candidate
            if score <= args.sampling_resolution:
                break

    if best is None:
        raise RuntimeError(
            f"no {args.scenario} route in {args.min_distance:.1f}-{args.max_distance:.1f} m "
            f"after {trace_count} traces"
        )
    _, _, start_index, goal_index, length, route = best
    return start_index, goal_index, length, route, trace_count


def transform_dict(transform):
    return {
        "x": float(transform.location.x),
        "y": float(transform.location.y),
        "z": float(transform.location.z),
        "roll": float(transform.rotation.roll),
        "pitch": float(transform.rotation.pitch),
        "yaw": float(transform.rotation.yaw),
    }


def ros_pose_dict(transform):
    yaw = -math.radians(transform.rotation.yaw)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return {
        "x": float(transform.location.x),
        "y": float(-transform.location.y),
        "z": float(transform.location.z),
        "yaw": yaw,
    }


def _pose_distance_3d(first, second):
    return math.sqrt(
        (float(first["x"]) - float(second["x"])) ** 2
        + (float(first["y"]) - float(second["y"])) ** 2
        + (float(first["z"]) - float(second["z"])) ** 2
    )


def serialize_route(route, goal_transform):
    if not route:
        raise RuntimeError("cannot serialize an empty CARLA route")
    cumulative = 0.0
    points = []
    previous = None
    for index, (waypoint, option) in enumerate(route):
        transform = waypoint.transform
        if previous is not None:
            cumulative += previous.location.distance(transform.location)
        vad_command = int(option) - 1 if option != RoadOption.VOID else 3
        pose = ros_pose_dict(transform)
        points.append(
            {
                "index": index,
                **pose,
                "distance_m": cumulative,
                "road_option": option.name,
                "road_option_value": int(option),
                "vad_command": vad_command,
                "road_id": int(waypoint.road_id),
                "section_id": int(waypoint.section_id),
                "lane_id": int(waypoint.lane_id),
                "is_junction": bool(waypoint.is_junction),
            }
        )
        previous = transform

    original_goal_pose = ros_pose_dict(goal_transform)
    terminal = points[-1]
    # CARLA spawn/recommended transforms conventionally carry an actor-clearance
    # Z (commonly +0.3/+0.5 m), while planner waypoints are on the road surface.
    # Preserve the exact endpoint X/Y/yaw but keep the runtime route on the last
    # road waypoint elevation.  Otherwise a coincident endpoint becomes a false
    # zero-distance elevation jump in the physical-grade preflight.
    goal_pose = {**original_goal_pose, "z": float(terminal["z"])}
    terminal_gap = _pose_distance_3d(goal_pose, terminal)
    if terminal_gap > 1.0e-6:
        cumulative += terminal_gap
        points.append(
            {
                **terminal,
                "index": len(points),
                **goal_pose,
                "distance_m": cumulative,
            }
        )
    else:
        terminal.update(goal_pose)

    total = points[-1]["distance_m"]
    for point in points:
        point["remaining_m"] = total - point["distance_m"]
    return points


def normalized_goal_metadata(
    goal_transform, route, route_points, *, endpoint_source, endpoint_index
):
    """Return road-Z runtime goals plus the untouched endpoint provenance."""
    if not route or not route_points:
        raise RuntimeError("goal metadata requires a non-empty road route")
    original_carla = transform_dict(goal_transform)
    original_ros = ros_pose_dict(goal_transform)
    road_z = float(route[-1][0].transform.location.z)
    serialized_z = float(route_points[-1]["z"])
    values = tuple(original_carla.values()) + tuple(original_ros.values()) + (
        road_z,
        serialized_z,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise RuntimeError("goal endpoint metadata must be finite")
    if not math.isclose(serialized_z, road_z, rel_tol=0.0, abs_tol=1.0e-9):
        raise RuntimeError(
            "serialized terminal Z does not match the last road waypoint Z"
        )
    normalized_carla = {**original_carla, "z": road_z}
    normalized_ros = {**original_ros, "z": road_z}
    return normalized_carla, normalized_ros, {
        "endpoint_source": endpoint_source,
        "endpoint_index": int(endpoint_index),
        "original_goal_carla_transform": original_carla,
        "original_goal_ros_pose": original_ros,
        "terminal_z_normalization": {
            "policy": "last_road_waypoint_z",
            "original_endpoint_z_m": float(original_carla["z"]),
            "last_road_waypoint_z_m": road_z,
            "runtime_goal_z_m": road_z,
            "serialized_terminal_z_m": serialized_z,
            "applied_offset_m": road_z - float(original_carla["z"]),
        },
    }


def set_async_mode(world):
    settings = world.get_settings()
    if settings.synchronous_mode:
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)


def main():
    args = parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    current_town = world.get_map().name.rsplit("/", 1)[-1]
    if current_town != args.town:
        world = client.load_world(args.town)
        time.sleep(2.0)
    set_async_mode(world)

    try:
        weather = getattr(carla.WeatherParameters, args.weather)
    except AttributeError as error:
        choices = sorted(
            name for name in dir(carla.WeatherParameters) if name[:1].isupper()
        )
        raise RuntimeError(
            f"unknown weather {args.weather!r}; choose one of: {', '.join(choices)}"
        ) from error
    world.set_weather(weather)

    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()
    planner = GlobalRoutePlanner(carla_map, args.sampling_resolution)
    start_index, goal_index, length, route, trace_count = select_route(
        planner, spawn_points, args
    )
    start_transform = spawn_points[start_index]
    goal_transform = spawn_points[goal_index]
    route_points = serialize_route(route, goal_transform)
    (
        goal_carla_transform,
        goal_ros_pose,
        goal_endpoint_provenance,
    ) = normalized_goal_metadata(
        goal_transform,
        route,
        route_points,
        endpoint_source="spawn_points",
        endpoint_index=goal_index,
    )
    option_counts = {}
    for point in route_points:
        option = point["road_option"]
        option_counts[option] = option_counts.get(option, 0) + 1

    payload = {
        "schema_version": 1,
        "coordinate_reference": "base_link",
        "spawn_point_reference": "base_link",
        "town": carla_map.name.rsplit("/", 1)[-1],
        "weather": args.weather,
        "scenario": args.scenario,
        "sampling_resolution_m": args.sampling_resolution,
        "route_length_m": route_points[-1]["distance_m"],
        "start_spawn_index": start_index,
        "goal_spawn_index": goal_index,
        "start_carla_transform": transform_dict(start_transform),
        "start_ros_pose": ros_pose_dict(start_transform),
        "goal_carla_transform": goal_carla_transform,
        "goal_ros_pose": goal_ros_pose,
        "goal_endpoint_provenance": goal_endpoint_provenance,
        "spawn_point": ",".join(
            f"{value:.6f}"
            for value in (
                start_transform.location.x,
                start_transform.location.y,
                start_transform.location.z,
                start_transform.rotation.roll,
                start_transform.rotation.pitch,
                start_transform.rotation.yaw,
            )
        ),
        "option_counts": option_counts,
        "route": route_points,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")
    temporary.replace(args.output)
    print(
        f"route={args.output} town={payload['town']} weather={args.weather} "
        f"scenario={args.scenario} start={start_index} goal={goal_index} "
        f"length={payload['route_length_m']:.2f}m points={len(route_points)} traces={trace_count} "
        f"options={option_counts}"
    )
    print(f"spawn_point={payload['spawn_point']}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as error:
        raise SystemExit(f"ERROR: {error}") from error
