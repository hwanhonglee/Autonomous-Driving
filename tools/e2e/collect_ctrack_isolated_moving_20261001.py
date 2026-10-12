#!/usr/bin/env python3
"""HH_261001 - Isolated CTrack moving RGB/depth/semantic external diagnostic capture.

No training eligibility or independent-evaluation certification is granted.
Historical capture permission is recorded separately from this run's intended role.
"""
from __future__ import annotations
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import sys
import time
from types import SimpleNamespace

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from workspace_python import configure
configure(ROOT)
from scripts.e2e import collect_carla_mapping_ctrack_v2 as historical
from scripts.e2e import run_owned_ctrack_isolated_probe_20261001_v2 as stationary
from scripts.e2e import prepare_ctrack_isolated_probe_20261001 as stage

INTENDED_ROLE = "external_map_diagnostic_evaluation"
TOWN = historical.TOWN
FRAMES = historical.FRAMES
FIXED_DELTA_SECONDS = historical.FIXED_DELTA_SECONDS
SUCCESS_STATUS = historical.SUCCESS_STATUS
FROZEN_V1_COLLECTOR_SHA256 = historical.FROZEN_V1_COLLECTOR_SHA256
split_guard = historical.split_guard
_hash_file = historical._hash_file
_policy_preflight = historical._policy_preflight
_route_binding = historical._route_binding
qa_summary_contract = historical.qa_summary_contract
receive = historical.receive
ROUTES = historical.ROUTES

def asset_preflight() -> dict:
    paths = {str(p): digest for p, digest in stage.PINNED.items()}
    package = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
    paths.update({str(package / relative): digest for relative,digest in historical.PACKAGED_ASSETS.items()})
    paths[str(package / "CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping")] = (
        "34a97404e1dbf61d2ebfb6dae18970ec5684f65c08c934cc7f65a5bd2b7f5192")
    result = {}
    for name, digest in paths.items():
        actual = stage.sha256(Path(name))
        if actual != digest:
            raise ValueError("source or preserved shipping input changed: " + name)
        result[name] = actual
    return result

def preflight(scenario: str) -> dict:
    route, digest = _route_binding(ROUTES[scenario][0], scenario)
    permission = _policy_preflight()
    stationary.import_carla()
    sys.path.insert(0, "/home/a/carla/PythonAPI/carla")
    import agents.navigation.basic_agent as basic_agent
    if not str(basic_agent.__file__).startswith("/home/a/carla/PythonAPI/carla/agents/"):
        raise ValueError("BasicAgent loaded from another CARLA distribution")
    import numpy
    import PIL
    return {"route_path":str(ROUTES[scenario][0]),"route_sha256":digest,
            "historical_capture_policy_sha256":permission,"source_assets":asset_preflight(),
            "intended_role":INTENDED_ROLE,"train_eligible":False,"dataset_admitted":False}

def validate_live_pre_actor(args, world, route_sha256):
    plan = json.loads((args.output.parent / "owner_plan.json").read_text())
    ready = json.loads((args.output.parent / "ready.json").read_text())
    if (plan["scenario"] != args.scenario or plan["route_sha256"] != route_sha256
            or plan["intended_role"] != INTENDED_ROLE or plan["port_triplet"][0] != args.port):
        raise ValueError("owned scenario/route/role/port mismatch")
    engine_plan = json.loads((Path(plan["stage_root"]) / "plan.json").read_text())
    current = stationary.find_editor(ready["launcher_pid"], engine_plan["editor_argv"])
    expected = ready["editor"]
    if not current or any(current[k] != expected[k] for k in ("pid","pgid","start_ticks","argv")):
        raise ValueError("owned editor generation differs from ready receipt")
    if not stationary.listening_socket_owned(current["pid"], args.port):
        raise ValueError("RPC socket is not owned by the expected editor")
    for source, item in plan["source_provenance"].items():
        if stage.sha256(Path(source)) != item["sha256"] or stage.sha256(Path(item["archived_path"])) != item["sha256"]:
            raise ValueError("source provenance changed before actors")
    carla_map = world.get_map()
    name = str(carla_map.name)
    live_digest = hashlib.sha256(carla_map.to_opendrive().encode()).hexdigest()
    if name not in stationary.LIVE_NAMES or live_digest != stage.PINNED[stage.XODR]:
        raise ValueError("exact staging map or live OpenDRIVE differs")
    actors = world.get_actors()
    if any(list(actors.filter(pattern)) for pattern in ("vehicle.*","sensor.*","walker.*","controller.ai.walker")):
        raise ValueError("dynamic actors already exist before the owned capture")
    return {"owner_pid":current["pid"],"owner_pgid":current["pgid"],
            "owner_generation_start_ticks":current["start_ticks"],"live_map_name":name,
            "live_xodr_sha256":live_digest,"live_xodr_source":"carla.Map.to_opendrive",
            "owner_plan":str(args.output.parent / "owner_plan.json")}

def capture(port: int, output: Path, scenario: str) -> dict:
    preflight(scenario)
    return run(SimpleNamespace(output=output, scenario=scenario, route_file=ROUTES[scenario][0],
                               host="127.0.0.1",port=port))


def run(args):
    """HH_261001 - Preserve the reviewed drive/sensor loop with isolated source bindings."""
    carla = stationary.import_carla()
    import numpy as np
    from PIL import Image
    from portable_e2e.mapping import SemanticEvidenceMap, replay
    from portable_e2e.mapping_projection import project_frame

    policy_sha256 = _policy_preflight()
    route, route_sha256 = _route_binding(args.route_file, args.scenario)
    source_assets = asset_preflight()
    if args.output.is_symlink() or any(args.output.iterdir()):
        raise ValueError("owned capture output must be an empty regular directory")
    client = carla.Client(args.host, args.port)
    client.set_timeout(10)
    world = client.get_world()
    live_provenance = validate_live_pre_actor(args, world, route_sha256)
    # HH_260929 - Repeat mutable source checks immediately before world settings or actors.
    if (_policy_preflight() != policy_sha256
            or _route_binding(args.route_file, args.scenario)[1] != route_sha256
            or asset_preflight() != source_assets):
        raise ValueError("C-track policy, route or package changed after live preflight")
    labels = {int(getattr(carla.CityObjectLabel, name)): label for name, label in (
        ("Roads", "road"), ("RoadLines", "lane_marking"), ("Sidewalks", "sidewalk"),
        ("TrafficLight", "traffic_light_fixture"), ("TrafficSigns", "traffic_sign"),
        ("Poles", "pole"))}
    original = world.get_settings()
    original_weather = world.get_weather()
    actors, sensors, records = [], [], []
    # HH_261001 - The owner created this empty capture directory.
    (args.output / "frames").mkdir()
    (args.output / "pre_actor_provenance.json").write_text(json.dumps({
        "scenario": args.scenario, "route_path": str(args.route_file),
        "route_sha256": route_sha256, "site_split_v2_sha256": policy_sha256,
        "frozen_v1_collector_sha256": FROZEN_V1_COLLECTOR_SHA256,
        "source_assets": source_assets, **live_provenance,
        "dataset_admitted": False, "train_eligible": False,
        "intended_role": INTENDED_ROLE,
    }, indent=2))
    evidence = SemanticEvidenceMap(f"carla_ue:{TOWN}", "oracle_sensor", max_cells=200000)
    report = dict(status="RUNNING", town=TOWN, scenario=args.scenario,
                  route_sha256=route_sha256, split_operation=split_guard.CAPTURE_OPERATION,
                  site_split_v2_sha256=policy_sha256, source_assets=source_assets,
                  frozen_v1_collector_sha256=FROZEN_V1_COLLECTOR_SHA256,
                  **live_provenance, **qa_summary_contract(), label_table=labels,
                  server_version=client.get_server_version(),
                  client_version=client.get_client_version(),
                  intended_role=INTENDED_ROLE, rgb_enable_postprocess_effects=True,
                  sensor_intrinsics={"width":640,"height":360,"horizontal_fov_deg":90,
                                     "K":[[320.0,0.0,320.0],[0.0,320.0,180.0],[0.0,0.0,1.0]],
                                     "lens_k":0.0,"lens_kcube":0.0},
                  fixed_delta_seconds=FIXED_DELTA_SECONDS,
                  camera_mount={"x":1.5,"y":0.0,"z":2.0,"pitch":-12.0},
                  target_speed_kph=15.0)
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
        bp.set_attribute("role_name", "owned_ctrack_external_eval_20261001")
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
                camera_bp.set_attribute("enable_postprocess_effects", "true" if kind == "rgb" else "false")
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
            control = agent.run_step()
            vehicle.apply_control(control)
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
            quality = stationary.rgb_quality(arrays[0][..., [2, 1, 0]].tobytes())
            tags = arrays[2][..., 2]
            observation = project_frame(
                arrays[1], arrays[2], matrices[0], frame_id=evidence.frame_id,
                timestamp_s=images[0].timestamp, fov_deg=90, label_table=labels)
            evidence.update(observation)
            stem = f"frames/{index:04d}"
            Image.fromarray(arrays[0][..., [2, 1, 0]]).save(args.output / (stem + ".png"))
            np.savez_compressed(args.output / (stem + ".npz"), rgb=arrays[0], depth=arrays[1],
                                semantic=arrays[2], world_from_camera=matrices[0])
            record = dict(timestamp_s=observation.timestamp_s, frame=frame,
                          stem=stem, ego_x=location.x, ego_y=location.y,
                          ego_yaw_deg=yaw, points=len(observation.points),
                          speed_mps=vehicle.get_velocity().length(),
                          route_index=None, ego_z=location.z,
                          ego_pitch_deg=pose.rotation.pitch, ego_roll_deg=pose.rotation.roll,
                          world_from_ego=pose.get_matrix(),
                          control={"throttle":control.throttle,"brake":control.brake,"steer":control.steer},
                          rgb_quality=quality,
                          road_pixels=int(np.count_nonzero(tags == 1)),
                          lane_pixels=int(np.count_nonzero(tags == 24)),
                          sensors={kind:{"actor_id":sensor.id, "frame":int(image.frame),
                              "timestamp_s":float(image.timestamp),
                              "width":int(image.width),"height":int(image.height),
                              "world_from_camera":matrix.tolist(),
                              "raw_bgra_sha256":hashlib.sha256(image.raw_data).hexdigest()}
                              for kind,(sensor,_),image,matrix in zip(
                                  ("rgb","depth","semantic_segmentation"),sensors,images,matrices)},
                          artifact_sha256={suffix:_hash_file(args.output / (stem + suffix))
                                           for suffix in (".png",".npz")})
            records.append(record)
            print(json.dumps({"index":index,"frame":frame,"speed_mps":record["speed_mps"],
                              "road_pixels":record["road_pixels"],"lane_pixels":record["lane_pixels"]}),
                  file=sys.stderr, flush=True)
        report.update(road_pixels=sum(row["road_pixels"] for row in records),
                      lane_pixels=sum(row["lane_pixels"] for row in records),
                      lane_positive_frames=sum(row["lane_pixels"] > 0 for row in records),
                      rgb_warning_frames=sum(row["rgb_quality"]["warning_detected"] for row in records),
                      capture_wall_seconds=time.monotonic() - start_wall,
                      travel_m=travel, collision_events=len(collisions),
                      agent_goal_reached=bool(agent.done()))
        if report["rgb_warning_frames"]:
            raise ValueError("RGB sky warning detected in moving capture")
        if not report["road_pixels"] or report["lane_positive_frames"] < 24:
            raise ValueError("native road/lane coverage absent or too sparse")
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
    return {"status":"PASS","summary_path":str(args.output / "summary.json"),
            "raw_labels_present":True,"rgb_quality":{"warning_detected":False,"visual_review_required":True},
            "frame_count":len(records),"travel_m":report["travel_m"],
            "road_pixels":report["road_pixels"],"lane_pixels":report["lane_pixels"],
            "intended_role":INTENDED_ROLE,"dataset_admitted":False,"train_eligible":False}
