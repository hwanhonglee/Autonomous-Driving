#!/usr/bin/env python3
"""HH_260929 - Owned C-track RGB/depth/semantic TRAIN-candidate capture only.

This versioned collector deliberately duplicates the frozen mapping-probe v1
sensor, BasicAgent, replay, and cleanup loop. Its scope cannot be added to v1
without changing historical route-binding semantics or patching module globals.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import sys
import time

from scripts.e2e import site_split_guard_v2 as split_guard


ROOT = Path(__file__).resolve().parents[2]
TOWN = "C_track_1_0_7"
LIVE_MAP_NAMES = frozenset({
    "Carla/Maps/C_track_1_0_7", "/Game/Carla/Maps/C_track_1_0_7",
})
ROUTES = {
    "straight": (
        ROOT / "storage/experiments/validation/2026-09-01/"
        "autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/"
        "c_track_1_0_7/catalog/straight/routes/c_track_1_0_7/straight/"
        "c_track_1_0_7_straight_s0000_p03.json",
        "cb8632fa48598389a948fe552826ab05a0d2c737126dd3f39e4dde12d5e3b70e",
    ),
    "left": (
        ROOT / "storage/experiments/training/2026-09-08/turn_launch_matrix_v1/"
        "c_track_left/turn_launch_013_v1/run_002/episode/route.json",
        "2299e5bf2bc86789650da72336fdcfe7c11c585df0b9e0a0fe29a44730bae79b",
    ),
}
PACKAGED_ASSETS = {
    "CarlaUE4/Content/Carla/Maps/C_track_1_0_7.umap":
        "d44c26a3e7fae4e05cd01cc12022e96daf7da5b4650797718442b143743e72a7",
    "CarlaUE4/Content/Carla/Maps/OpenDrive/C_track_1_0_7.xodr":
        "3f760b8ec8cb117a975c9631a74948b620c991aeaef5c1b07497ebb37ec7be27",
}
EXPECTED_LIVE_XODR_SHA256 = PACKAGED_ASSETS[
    "CarlaUE4/Content/Carla/Maps/OpenDrive/C_track_1_0_7.xodr"]
CAPTURE_MODE = "mapping-ctrack-v2"
WORKER_SOURCE = "scripts/e2e/collect_carla_mapping_ctrack_v2.py"
FROZEN_V1_COLLECTOR_SHA256 = (
    "2fdfa553329f0224f49da7fc32996ea2fe5b3210f70df8dc0f68caeb3d679e4a"
)
REQUIRED_SOURCES = (
    WORKER_SOURCE,
    "scripts/e2e/collect_carla_mapping_probe.py",
    "scripts/e2e/site_split_guard_v2.py",
    "config/site_split_guard_20260929_v2.json",
    "scripts/e2e/site_split_guard.py",
    "config/site_split_guard_20260929.json",
    "portable_e2e/mapping.py",
    "portable_e2e/mapping_projection.py",
)
FRAMES = 240
FIXED_DELTA_SECONDS = 0.1
SUCCESS_STATUS = "COMPLETE_ORACLE_PLUMBING_ONLY"


def _hash_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular_file(path: Path) -> bool:
    return path.is_file() and not path.is_symlink()


def _route_binding(route_file: Path, scenario: str) -> tuple[dict, str]:
    """HH_260929 - Bind a scenario to one absolute route path and exact bytes."""
    expected_path, expected_sha256 = ROUTES[scenario]
    if (route_file.absolute() != expected_path or not _regular_file(route_file)
            or route_file.resolve() != route_file.absolute()):
        raise ValueError(f"C-track {scenario} requires its reviewed regular route file")
    payload = route_file.read_bytes()
    observed_sha256 = hashlib.sha256(payload).hexdigest()
    if observed_sha256 != expected_sha256:
        raise ValueError(f"C-track {scenario} route bytes differ from the pinned route")
    route = json.loads(payload)
    if (not isinstance(route, dict) or route.get("town") != TOWN
            or route.get("scenario") != scenario
            or route.get("coordinate_reference") != "base_link"
            or not isinstance(route.get("start_carla_transform"), dict)
            or not isinstance(route.get("route"), list)
            or len(route["route"]) < 2
            or any("road_option_value" not in point for point in route["route"])):
        raise ValueError("C-track route structure or scenario differs from reviewed binding")
    return route, observed_sha256


def _policy_preflight() -> str:
    """HH_260929 - Query the exact capture operation, never a generic TRAIN role."""
    frozen_source = ROOT / "tools/e2e/collect_carla_mapping_probe.py"
    if (not _regular_file(frozen_source)
            or _hash_file(frozen_source) != FROZEN_V1_COLLECTOR_SHA256):
        raise ValueError("frozen v1 mapping collector provenance changed")
    if not _regular_file(split_guard.REGISTRY):
        raise ValueError("V2 site split registry is absent or symlinked")
    registry = json.loads(split_guard.REGISTRY.read_text(encoding="utf-8"))
    if not split_guard.check(TOWN, "map", split_guard.CAPTURE_OPERATION, registry):
        raise ValueError("V2 C-track map owned-capture permission is denied")
    contract = registry["capture_contract"]
    if (contract["owned_capture_only"] is not True
            or any(contract[key] is not False for key in (
                "dataset_admitted", "train_eligible", "independent_dev_approved",
                "test_use_authorized"))):
        raise ValueError("V2 capture contract would admit or promote data")
    return _hash_file(split_guard.REGISTRY)


def _packaged_asset_preflight() -> dict[str, dict[str, str]]:
    """HH_260929 - Bind the owner to the reviewed Epic package, not a map name."""
    carla_root = os.environ.get("CARLA_ROOT")
    if not carla_root or not Path(carla_root).is_absolute():
        raise ValueError("absolute CARLA_ROOT is required for pinned C-track assets")
    root = Path(carla_root)
    result = {}
    for relative, expected_sha256 in PACKAGED_ASSETS.items():
        source = root / relative
        if not _regular_file(source):
            raise ValueError(f"packaged C-track asset is absent or symlinked: {relative}")
        observed_sha256 = _hash_file(source)
        if observed_sha256 != expected_sha256:
            raise ValueError(f"packaged C-track asset bytes changed: {relative}")
        result[relative] = {"path": str(source), "sha256": observed_sha256}
    return result


def _canonical_argv(args) -> list[str]:
    """HH_260929 - Use one exact argv in prelaunch and owned-generation proof."""
    return [str(args.output), str(args.route_file), "--host", args.host,
            "--port", str(args.port), "--motion-mode", "basic-agent",
            "--frames", str(FRAMES), "--ignore-traffic-lights",
            "--ctrack-straight" if args.scenario == "straight" else "--ctrack-left"]


def parse_args(argv=None):
    """HH_260929 - Perform CPU-only policy, route and package checks before launch."""
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("output", type=Path)
    parser.add_argument("route_file", type=Path)
    parser.add_argument("--host", choices=["127.0.0.1"], default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2100)
    parser.add_argument("--motion-mode", choices=["basic-agent"], required=True)
    parser.add_argument("--frames", type=int, choices=[FRAMES], required=True)
    parser.add_argument("--ignore-traffic-lights", action="store_true", required=True)
    scenario = parser.add_mutually_exclusive_group(required=True)
    scenario.add_argument("--ctrack-straight", action="store_true")
    scenario.add_argument("--ctrack-left", action="store_true")
    args = parser.parse_args(raw_argv)
    args.scenario = "straight" if args.ctrack_straight else "left"
    args.allow_map_load = False
    if raw_argv != _canonical_argv(args):
        parser.error("C-track capture requires the single canonical owned collector argv")
    if args.output.exists() or args.output.is_symlink() or not 1024 <= args.port <= 65533:
        parser.error("fresh output and bounded local port required")
    try:
        _policy_preflight()
        _route_binding(args.route_file, args.scenario)
        _packaged_asset_preflight()
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.error(str(error))
    return args


def _read_owner_json(path: Path) -> dict:
    if not _regular_file(path):
        raise ValueError(f"owned C-track proof is absent or symlinked: {path.name}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"owned C-track proof is not an object: {path.name}")
    return value


def _owner_proof(args, route_sha256: str, live_name: str) -> dict:
    """HH_260929 - Require this fresh Epic owner generation before actor spawn."""
    owner_root = args.output.parent
    plan = _read_owner_json(owner_root / "owner_plan.json")
    started = _read_owner_json(owner_root / "owner_started.json")
    ready = _read_owner_json(owner_root / "lifecycle/ready.json")
    pid = started.get("server_pid")
    if (not isinstance(pid, int) or pid <= 1 or started.get("server_pgid") != pid
            or os.getpgid(pid) != pid):
        raise ValueError("owned C-track CARLA process group is absent or changed")
    if (plan.get("capture_mode") != CAPTURE_MODE
            or plan.get("worker_path") != WORKER_SOURCE
            or plan.get("quality") != "Epic"
            or plan.get("map") != TOWN
            or plan.get("route_path") != str(args.route_file)
            or plan.get("route_sha256") != route_sha256
            or plan.get("host") != args.host or plan.get("port") != args.port
            or plan.get("collector_argv") != _canonical_argv(args)):
        raise ValueError("owned C-track Epic plan differs from exact capture command")
    if (started.get("quality") != "Epic" or started.get("map") != TOWN
            or started.get("route_sha256") != route_sha256
            or started.get("port") != args.port):
        raise ValueError("owned C-track startup proof differs from the plan")
    if (ready.get("status") != "PASS" or ready.get("mode") != "running"
            or ready.get("stage") != "ready"
            or ready.get("generation_id") != f"expert_{pid}"
            or ready.get("owner_pid") != pid or ready.get("owner_pgid") != pid
            or ready.get("port") != args.port or ready.get("host") != args.host
            or ready.get("expected_map") != TOWN
            or ready.get("active_map_basename") != TOWN
            or ready.get("active_map_name") != live_name
            or ready.get("read_only") is not True):
        raise ValueError("owned C-track ready probe differs from the live world")
    from portable_e2e.workspace_paths import source_path
    source_hashes = plan.get("source_sha256", {})
    for logical_name in REQUIRED_SOURCES:
        expected_sha256 = source_hashes.get(logical_name)
        source = source_path(ROOT, logical_name)
        archived = owner_root / "provenance" / logical_name
        if (not isinstance(expected_sha256, str) or len(expected_sha256) != 64
                or not _regular_file(source) or not _regular_file(archived)
                or _hash_file(source) != expected_sha256
                or _hash_file(archived) != expected_sha256):
            raise ValueError(f"owned C-track source provenance changed: {logical_name}")
    return {"owner_pid": pid, "owner_generation_id": ready["generation_id"],
            "owner_plan": str(owner_root / "owner_plan.json")}


def validate_live_pre_actor(args, world, route_sha256: str) -> dict:
    """HH_260929 - Match live OpenDRIVE bytes to packaged XODR before actors."""
    carla_map = world.get_map()
    live_name = str(carla_map.name)
    if live_name not in LIVE_MAP_NAMES:
        raise ValueError("live C-track world is not the reviewed packaged map")
    owner = _owner_proof(args, route_sha256, live_name)
    export = getattr(carla_map, "to_opendrive", None)
    if not callable(export):
        raise ValueError("live C-track OpenDRIVE export is unavailable")
    live_xodr = export()
    if not isinstance(live_xodr, str):
        raise ValueError("live C-track OpenDRIVE export is not text")
    live_sha256 = hashlib.sha256(live_xodr.encode("utf-8")).hexdigest()
    if live_sha256 != EXPECTED_LIVE_XODR_SHA256:
        raise ValueError("live C-track OpenDRIVE bytes differ from reviewed packaged XODR")
    actors = world.get_actors()
    if any(list(actors.filter(pattern)) for pattern in (
            "vehicle.*", "sensor.*", "walker.*", "controller.ai.walker")):
        raise ValueError("owned C-track world has unexpected dynamic actors before capture")
    return {**owner, "live_map_name": live_name, "live_xodr_sha256": live_sha256,
            "live_xodr_source": "carla.Map.to_opendrive"}


def receive(sensor_queue, frame):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        item = sensor_queue.get(timeout=max(.01, deadline - time.monotonic()))
        if item.frame == frame:
            return item
        if item.frame > frame:
            raise ValueError("C-track sensor skipped required frame")
    raise TimeoutError("C-track sensor synchronization failed")


def qa_summary_contract() -> dict:
    """HH_260929 - Match the fail-closed C-track archive auditor's source fields."""
    return {
        "physics_driving": True,
        "motion_mode": "basic-agent",
        "controller": "CARLA BasicAgent",
        "learned_model_control": False,
        "ignores_traffic_lights": True,
        "oracle_pose": True,
        "oracle_depth": True,
        "oracle_semantics": True,
        "dataset_admitted": False,
        "train_eligible": False,
        "independent_dev_approved": False,
        "test_use_authorized": False,
    }


def run(args):
    """HH_260929 - Duplicate v1's bounded sensor loop under the new pre-actor gate."""
    import carla
    import numpy as np
    from PIL import Image
    from portable_e2e.mapping import SemanticEvidenceMap, replay
    from portable_e2e.mapping_projection import project_frame

    policy_sha256 = _policy_preflight()
    route, route_sha256 = _route_binding(args.route_file, args.scenario)
    packaged_assets = _packaged_asset_preflight()
    if args.output.exists() or args.output.is_symlink():
        raise ValueError("C-track output must still be fresh")
    client = carla.Client(args.host, args.port)
    client.set_timeout(10)
    world = client.get_world()
    live_provenance = validate_live_pre_actor(args, world, route_sha256)
    # HH_260929 - Repeat mutable source checks immediately before world settings or actors.
    if (_policy_preflight() != policy_sha256
            or _route_binding(args.route_file, args.scenario)[1] != route_sha256
            or _packaged_asset_preflight() != packaged_assets):
        raise ValueError("C-track policy, route or package changed after live preflight")
    labels = {int(getattr(carla.CityObjectLabel, name)): label for name, label in (
        ("Roads", "road"), ("RoadLines", "lane_marking"), ("Sidewalks", "sidewalk"),
        ("TrafficLight", "traffic_light_fixture"), ("TrafficSigns", "traffic_sign"),
        ("Poles", "pole"))}
    original = world.get_settings()
    original_weather = world.get_weather()
    actors, sensors, records = [], [], []
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "frames").mkdir()
    (args.output / "pre_actor_provenance.json").write_text(json.dumps({
        "scenario": args.scenario, "route_path": str(args.route_file),
        "route_sha256": route_sha256, "site_split_v2_sha256": policy_sha256,
        "frozen_v1_collector_sha256": FROZEN_V1_COLLECTOR_SHA256,
        "packaged_assets": packaged_assets, **live_provenance,
        "dataset_admitted": False, "train_eligible": False,
    }, indent=2))
    evidence = SemanticEvidenceMap(f"carla_ue:{TOWN}", "oracle_sensor", max_cells=200000)
    report = dict(status="RUNNING", town=TOWN, scenario=args.scenario,
                  route_sha256=route_sha256, split_operation=split_guard.CAPTURE_OPERATION,
                  site_split_v2_sha256=policy_sha256, packaged_assets=packaged_assets,
                  frozen_v1_collector_sha256=FROZEN_V1_COLLECTOR_SHA256,
                  **live_provenance, **qa_summary_contract(), label_table=labels,
                  server_version=client.get_server_version(),
                  client_version=client.get_client_version())
    previous = {}

    def interrupted(*_):
        raise KeyboardInterrupt("owned C-track scan interrupted")

    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, interrupted)
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
        settings.substepping = True
        settings.max_substep_delta_time = .01
        settings.max_substeps = 10
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        reference = [{"x": point.transform.location.x,
                      "y": point.transform.location.y,
                      "z": point.transform.location.z,
                      "yaw": point.transform.rotation.yaw,
                      "width": point.lane_width, "junction": point.is_junction}
                     for point in world.get_map().generate_waypoints(1.0)]
        (args.output / "logical_map_reference.json").write_text(json.dumps(reference))
        bp = world.get_blueprint_library().find("vehicle.toyota.prius")
        bp.set_attribute("role_name", "owned_ctrack_mapping_probe_v2")
        transform = route["start_carla_transform"]
        vehicle = world.spawn_actor(
            bp, carla.Transform(carla.Location(transform["x"], transform["y"],
                                             transform["z"] + .5),
                                carla.Rotation(yaw=transform["yaw"])))
        actors.append(vehicle)
        vehicle.set_simulate_physics(True)
        collisions = []
        collision_sensor = world.spawn_actor(
            world.get_blueprint_library().find("sensor.other.collision"),
            carla.Transform(), attach_to=vehicle)
        actors.append(collision_sensor)
        collision_sensor.listen(lambda event: collisions.append(event.frame))
        for kind in ("rgb", "depth", "semantic_segmentation"):
            camera_bp = world.get_blueprint_library().find("sensor.camera." + kind)
            for key, value in {"image_size_x": "640", "image_size_y": "360", "fov": "90",
                               "sensor_tick": "0", "lens_k": "0", "lens_kcube": "0"}.items():
                if camera_bp.has_attribute(key):
                    camera_bp.set_attribute(key, value)
            if camera_bp.has_attribute("enable_postprocess_effects"):
                camera_bp.set_attribute("enable_postprocess_effects", "false")
            sensor = world.spawn_actor(
                camera_bp, carla.Transform(carla.Location(x=1.5, z=2.0),
                                           carla.Rotation(pitch=-12)), attach_to=vehicle)
            actors.append(sensor)
            sensor_queue = queue.Queue()
            sensor.listen(sensor_queue.put)
            sensors.append((sensor, sensor_queue))
        from agents.navigation.basic_agent import BasicAgent
        from agents.navigation.local_planner import RoadOption
        vehicle.apply_control(carla.VehicleControl(brake=1.))
        for _ in range(30):
            frame = world.tick()
            for _, sensor_queue in sensors:
                receive(sensor_queue, frame)
        agent = BasicAgent(vehicle, target_speed=15,
                           opt_dict={"ignore_traffic_lights": True})
        plan = [(world.get_map().get_waypoint(
                     carla.Location(float(point["x"]), -float(point["y"]), 0)),
                 RoadOption(int(point["road_option_value"])))
                for point in route["route"]]
        if any(waypoint is None for waypoint, _ in plan):
            raise ValueError("C-track route has an unmatched live waypoint")
        agent.set_global_plan(plan)
        start_wall = time.monotonic()
        previous_location = None
        travel = 0.0
        for index in range(FRAMES):
            vehicle.apply_control(agent.run_step())
            frame = world.tick()
            images = [receive(sensor_queue, frame) for _, sensor_queue in sensors]
            pose = vehicle.get_transform()
            location, yaw = pose.location, pose.rotation.yaw
            if previous_location is not None:
                travel += math.hypot(location.x - previous_location[0],
                                     location.y - previous_location[1])
            previous_location = (location.x, location.y)
            if max(image.timestamp for image in images) - min(
                    image.timestamp for image in images) > 1e-6:
                raise ValueError("C-track camera timestamp mismatch")
            matrices = [np.asarray(image.transform.get_matrix()) for image in images]
            if not all(np.allclose(matrices[0], matrix, atol=1e-5)
                       for matrix in matrices):
                raise ValueError("C-track camera pose mismatch")
            arrays = [np.frombuffer(image.raw_data, np.uint8).reshape(
                image.height, image.width, 4).copy() for image in images]
            observation = project_frame(
                arrays[1], arrays[2], matrices[0], frame_id=evidence.frame_id,
                timestamp_s=images[0].timestamp, fov_deg=90, label_table=labels)
            evidence.update(observation)
            stem = f"frames/{index:04d}"
            Image.fromarray(arrays[0][..., [2, 1, 0]]).save(args.output / (stem + ".png"))
            np.savez_compressed(args.output / (stem + ".npz"), depth=arrays[1],
                                semantic=arrays[2], world_from_camera=matrices[0])
            record = dict(timestamp_s=observation.timestamp_s, frame=frame,
                          stem=stem, ego_x=location.x, ego_y=location.y,
                          ego_yaw_deg=yaw, points=len(observation.points),
                          speed_mps=vehicle.get_velocity().length(),
                          route_index=None)
            records.append(record)
            print(json.dumps(record), flush=True)
        report.update(capture_wall_seconds=time.monotonic() - start_wall,
                      travel_m=travel, collision_events=len(collisions),
                      agent_goal_reached=bool(agent.done()))
        if travel < 3 or collisions:
            raise ValueError("C-track collection failed minimum travel or collision check")

        def saved_observations():
            for record in records:
                with np.load(args.output / (record["stem"] + ".npz"),
                             allow_pickle=False) as data:
                    yield project_frame(
                        data["depth"], data["semantic"], data["world_from_camera"],
                        frame_id=evidence.frame_id, timestamp_s=record["timestamp_s"],
                        fov_deg=90, label_table=labels)

        rebuilt = replay(saved_observations(), frame_id=evidence.frame_id,
                         source="oracle_sensor", max_cells=200000)
        report["live_replay_exact_match"] = rebuilt.snapshot() == evidence.snapshot()
        if not report["live_replay_exact_match"]:
            raise ValueError("C-track live/replay evidence mismatch")
        (args.output / "map.json").write_text(json.dumps(evidence.snapshot()))
        report.update(status=SUCCESS_STATUS,
                      frame_count=len(records), observed_cells=len(evidence.cells),
                      hd_map_approved=False, qa_pending=True)
    except BaseException as error:
        report.update(status="FAILED", error=repr(error))
        raise
    finally:
        cleanup_errors = []
        for sensor, _ in sensors:
            try:
                sensor.stop()
            except Exception as error:
                cleanup_errors.append(str(error))
        for actor in reversed(actors):
            try:
                if not actor.destroy():
                    cleanup_errors.append(f"actor {actor.id} destroy returned false")
            except Exception as error:
                cleanup_errors.append(str(error))
        try:
            world.apply_settings(original)
            world.set_weather(original_weather)
        except Exception as error:
            cleanup_errors.append(str(error))
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        report["cleanup_errors"] = cleanup_errors
        if cleanup_errors:
            report["status"] = "FAILED_CLEANUP"
        (args.output / "records.json").write_text(json.dumps(records, indent=2))
        (args.output / "summary.json").write_text(json.dumps(report, indent=2))
    if report["status"] != SUCCESS_STATUS:
        raise RuntimeError("C-track mapping capture failed or cleanup was incomplete")


if __name__ == "__main__":
    run(parse_args())
