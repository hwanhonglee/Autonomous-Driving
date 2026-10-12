"""HH_260906 - Owned oracle sensor capture; optional BasicAgent, never learned control."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import queue
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# HH_260906 - Admit one byte-pinned Town06 research scout without broadening the mapping collector to arbitrary worlds.
TOWN06_SCOUT_ROUTE_SHA256 = "e27e95cb60d40657638183f7f78ce2e1f4f070067707ac4dd3f2f89dd1c52283"
# HH_260906 - Scout one TRAIN-site Town02 route without claiming unseen sign labels are qualified.
TOWN02_TRAIN_SCOUT_ROUTE_SHA256 = frozenset({
    "520fc0236e32d0664bc125a15f45601e4ef020cf1e3f1db888332af0e188f08b",  # straight p00
})
# HH_260906 - Keep the new Town02 sign scout tied to a second, separately planned corridor.
TOWN02_P02_TRAIN_SCOUT_ROUTE_SHA256 = frozenset({
    "cad81f4cdfbc9dbcad8c3cc53092e6fd80f6d84dc4766b6d73758fa88dec6810",  # straight p02
})
# HH_260906 - Bind the next mapping TRAIN and sealed evaluation candidates to predeclared route bytes.
TOWN03_TRAIN_ROUTE_SHA256 = frozenset({
    "aee6b0fd6e5a6dfa882cd172b4cb10c2e0865d2729d3233f5f121bb4be0674d4",  # straight p03
    "8aa4040617f733e5df11910d3ac8113307e05a53334f29bc3387eb1370e2bf3c",  # right p03
})
# HH_260906 - Bind the independent map DEV candidate away from the driving Town04 TEST site.
TOWN05_DEV_ROUTE_SHA256 = frozenset({
    "efb8ccca533f25cae54b2eb7e03274c15795211488128185334ff2bc55f4744c",  # straight p00
})
# HH_260906 - Keep a sign-capable Town05 DEV scout on one separate right-turn route, never an open Town scope.
TOWN05_SIGN_DEV_SCOUT_ROUTE_SHA256 = frozenset({
    "2f7a25e8ace753bfdd412ae7ef65710a001d1f4b1ecd5765f3ee7b021ced29e3",  # right p00
})
TOWN07_TEST_ROUTE_SHA256 = frozenset({
    "804dc69cec5ebbc2edba70946694f312c77328b5cb5d7260b096af0775f34848",  # straight p00
    "384d0aca89517471aa58947bb6256edf8e18180138f276b935cd8e849a5e8410",  # right p00
})
# HH_260928 - Bind a single Town13 source-editor render diagnostic, never a qualified sign DEV.
TOWN13_RENDER_SENSOR_SCOUT_ROUTE_SHA256 = frozenset({
    "6927c0de706e5ffb89afce25a75a892d56f7ee212b577002787605802839520b",  # right s0001_p00
})
TOWN13_LIVE_MAP_NAMES = frozenset({
    "/Game/Carla/Maps/Town13/Town13", "Carla/Maps/Town13/Town13",
})


def validate_route_binding(route, route_sha256, world_town, town06_sign_scout,
                           town03_generalization_train=False, town07_generalization_test=False,
                           town05_generalization_dev=False, town02_sign_train_scout=False,
                           town02_sign_train_scout_p02=False,
                           town05_sign_dev_scout=False,
                           town13_render_sensor_scout=False):
    """HH_260906 - Keep exact scoped route bytes, route town and observed CARLA world aligned."""
    if route.get("coordinate_reference") != "base_link" or len(route.get("route", [])) < 2:
        raise ValueError("explicit ROS route coordinates required")
    if sum((town06_sign_scout, town03_generalization_train,
            town07_generalization_test, town05_generalization_dev,
            town02_sign_train_scout, town02_sign_train_scout_p02,
            town05_sign_dev_scout, town13_render_sensor_scout)) > 1:
        raise ValueError("mapping research scopes are mutually exclusive")
    if town06_sign_scout:
        expected_town, allowed_hashes = "Town06", {TOWN06_SCOUT_ROUTE_SHA256}
    elif town02_sign_train_scout:
        expected_town, allowed_hashes = "Town02_Opt", TOWN02_TRAIN_SCOUT_ROUTE_SHA256
    elif town02_sign_train_scout_p02:
        expected_town, allowed_hashes = "Town02_Opt", TOWN02_P02_TRAIN_SCOUT_ROUTE_SHA256
    elif town03_generalization_train:
        expected_town, allowed_hashes = "Town03", TOWN03_TRAIN_ROUTE_SHA256
    elif town05_generalization_dev:
        expected_town, allowed_hashes = "Town05_Opt", TOWN05_DEV_ROUTE_SHA256
    elif town05_sign_dev_scout:
        expected_town, allowed_hashes = "Town05_Opt", TOWN05_SIGN_DEV_SCOUT_ROUTE_SHA256
    elif town07_generalization_test:
        expected_town, allowed_hashes = "Town07", TOWN07_TEST_ROUTE_SHA256
    elif town13_render_sensor_scout:
        expected_town, allowed_hashes = "Town13", TOWN13_RENDER_SENSOR_SCOUT_ROUTE_SHA256
    else:
        expected_town, allowed_hashes = "Town01", None
    if route.get("town") != expected_town:
        raise ValueError("mapping probe route town differs from its explicit scope")
    if allowed_hashes is not None and route_sha256 not in allowed_hashes:
        raise ValueError(f"{expected_town} mapping research scope requires pinned route bytes")
    if world_town is not None:
        if town13_render_sensor_scout:
            # HH_260928 - A Town13 basename cannot authenticate the source-editor asset.
            if world_town not in TOWN13_LIVE_MAP_NAMES:
                raise ValueError("owned CARLA world differs from the bound Town13 source asset")
        elif world_town != expected_town:
            raise ValueError("owned CARLA world differs from the bound route town")


def load_bound_route(args, world_town=None):
    """HH_260906 - Recheck source route bytes after owner startup, before attaching any CARLA actors."""
    route_file = args.route_file
    if route_file.is_symlink() or not route_file.is_file():
        raise ValueError("mapping probe route must be a regular file")
    payload = route_file.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    route = json.loads(payload)
    validate_route_binding(route, digest, world_town, args.town06_sign_scout,
                           args.town03_generalization_train, args.town07_generalization_test,
                           args.town05_generalization_dev, args.town02_sign_train_scout,
                           args.town02_sign_train_scout_p02, args.town05_sign_dev_scout,
                           args.town13_render_sensor_scout)
    return route, digest


def parse_args(argv=None):
    p = argparse.ArgumentParser(allow_abbrev=False)
    p.add_argument("output", type=Path)
    p.add_argument("route_file", type=Path)
    p.add_argument("--host", choices=["127.0.0.1"], default="127.0.0.1")
    p.add_argument("--port", type=int, default=2100)
    p.add_argument("--frames", type=int, default=36)
    p.add_argument("--motion-mode", choices=["kinematic", "basic-agent"], default="kinematic")
    p.add_argument("--ignore-traffic-lights",action="store_true")
    research_scope = p.add_mutually_exclusive_group()
    research_scope.add_argument("--town06-sign-scout", action="store_true")
    research_scope.add_argument("--town02-sign-train-scout", action="store_true")
    research_scope.add_argument("--town02-sign-train-scout-p02", action="store_true")
    research_scope.add_argument("--town03-generalization-train", action="store_true")
    research_scope.add_argument("--town05-generalization-dev", action="store_true")
    research_scope.add_argument("--town05-sign-dev-scout", action="store_true")
    research_scope.add_argument("--town07-generalization-test", action="store_true")
    research_scope.add_argument("--town13-render-sensor-scout", action="store_true")
    a = p.parse_args(argv)
    a.allow_map_load = False
    if a.ignore_traffic_lights and a.motion_mode!="basic-agent":
        p.error("traffic-light override is only for an explicitly declared empty-world mapping drive")
    limit = 240 if a.motion_mode == "basic-agent" else 60
    if a.output.exists() or not 1024 <= a.port <= 65533 or not 2 <= a.frames <= limit:
        p.error("fresh output, valid port and bounded frame count required")
    if (a.town06_sign_scout or a.town02_sign_train_scout or a.town02_sign_train_scout_p02
            or a.town03_generalization_train
            or a.town05_generalization_dev or a.town05_sign_dev_scout
            or a.town07_generalization_test or a.town13_render_sensor_scout):
        if a.motion_mode != "basic-agent" or a.frames != 240 or not a.ignore_traffic_lights:
            p.error("scoped map research requires 240 physical BasicAgent frames and explicit traffic-light override")
        try:
            load_bound_route(a)
        except (OSError, ValueError, TypeError) as error:
            p.error(str(error))
    return a


def receive(q, frame):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        item = q.get(timeout=max(.01, deadline - time.monotonic()))
        if item.frame == frame:
            return item
        if item.frame > frame:
            raise ValueError("sensor skipped required frame")
    raise TimeoutError("sensor synchronization failed")


def run(a):
    import carla
    import numpy as np
    from PIL import Image
    from portable_e2e.mapping import SemanticEvidenceMap, replay
    from portable_e2e.mapping_projection import project_frame

    route, route_sha256 = load_bound_route(a)
    client = carla.Client(a.host, a.port)
    client.set_timeout(10)
    world = client.get_world()
    live_map_name = world.get_map().name
    town = live_map_name.rsplit("/", 1)[-1]
    validate_route_binding(route, route_sha256,
                           live_map_name if a.town13_render_sensor_scout else town,
                           a.town06_sign_scout,
                           a.town03_generalization_train, a.town07_generalization_test,
                           a.town05_generalization_dev, a.town02_sign_train_scout,
                           a.town02_sign_train_scout_p02, a.town05_sign_dev_scout,
                           a.town13_render_sensor_scout)
    if list(world.get_actors().filter("vehicle.*")) or list(world.get_actors().filter("sensor.*")):
        raise ValueError("unexpected existing actors; owned empty world required")
    # HH_260906 - Use installed enums; legacy web semantic IDs differ in this binary.
    labels = {int(getattr(carla.CityObjectLabel, name)): label for name, label in (
        ("Roads", "road"), ("RoadLines", "lane_marking"), ("Sidewalks", "sidewalk"),
        ("TrafficLight", "traffic_light_fixture"), ("TrafficSigns", "traffic_sign"), ("Poles", "pole"))}
    original = world.get_settings()
    original_weather = world.get_weather()
    actors, sensors, records = [], [], []
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output / "frames").mkdir()
    evidence = SemanticEvidenceMap(f"carla_ue:{town}", "oracle_sensor", max_cells=200000)
    report = dict(status="RUNNING", learned_model_control=False, physics_driving=a.motion_mode=="basic-agent",
                  motion_mode=a.motion_mode, controller="CARLA BasicAgent" if a.motion_mode=="basic-agent" else "kinematic relocation",
                  ignores_traffic_lights=a.ignore_traffic_lights,
                  oracle_pose=True, oracle_depth=True, oracle_semantics=True,
                  label_table=labels, server_version=client.get_server_version(),
                  client_version=client.get_client_version(), town=town,
                  route_sha256=route_sha256)
    if a.town03_generalization_train:
        report["research_split_intent"] = "town03_additive_train_candidate"
    elif a.town02_sign_train_scout:
        report["research_split_intent"] = "town02_sign_train_scout_only"
    elif a.town02_sign_train_scout_p02:
        report["research_split_intent"] = "town02_p02_sign_train_scout_only"
    elif a.town05_generalization_dev:
        report["research_split_intent"] = "town05_independent_dev_candidate"
    elif a.town05_sign_dev_scout:
        report["research_split_intent"] = "town05_sign_dev_scout_only"
    elif a.town07_generalization_test:
        report["research_split_intent"] = "town07_sealed_evaluation_candidate"
    elif a.town13_render_sensor_scout:
        report["research_split_intent"] = "town13_render_sensor_diagnostic_only"
        report["sign_dev_approved"] = False
        report["training_data_admitted"] = False
    previous = {}
    def interrupted(*_):
        raise KeyboardInterrupt("owned scan interrupted")
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, interrupted)
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = .1
        settings.substepping = True
        settings.max_substep_delta_time = .01
        settings.max_substeps = 10
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        # HH_260906 - Record map reference separately; it never feeds projected observations.
        reference = [{"x": w.transform.location.x, "y": w.transform.location.y,
                      "z": w.transform.location.z, "yaw": w.transform.rotation.yaw,
                      "width": w.lane_width, "junction": w.is_junction}
                     for w in world.get_map().generate_waypoints(1.0)]
        (a.output / "logical_map_reference.json").write_text(json.dumps(reference))
        bp = world.get_blueprint_library().find("vehicle.toyota.prius")
        bp.set_attribute("role_name", "owned_mapping_probe")
        t = route["start_carla_transform"]
        vehicle = world.spawn_actor(bp, carla.Transform(carla.Location(t["x"], t["y"], t["z"] + .5),
                                      carla.Rotation(yaw=t["yaw"])))
        actors.append(vehicle)
        vehicle.set_simulate_physics(a.motion_mode == "basic-agent")
        collisions=[]
        if a.motion_mode == "basic-agent":
            collision_sensor = world.spawn_actor(world.get_blueprint_library().find("sensor.other.collision"),
                                                 carla.Transform(),attach_to=vehicle)
            actors.append(collision_sensor)
            collision_sensor.listen(lambda event: collisions.append(event.frame))
        for kind in ("rgb", "depth", "semantic_segmentation"):
            bp = world.get_blueprint_library().find("sensor.camera." + kind)
            for key, value in {"image_size_x": "640", "image_size_y": "360", "fov": "90",
                               "sensor_tick": "0", "lens_k": "0", "lens_kcube": "0"}.items():
                if bp.has_attribute(key):
                    bp.set_attribute(key, value)
            if bp.has_attribute("enable_postprocess_effects"):
                bp.set_attribute("enable_postprocess_effects", "false")
            sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.5, z=2.0),
                                       carla.Rotation(pitch=-12)), attach_to=vehicle)
            actors.append(sensor)
            q = queue.Queue()
            sensor.listen(q.put)
            sensors.append((sensor, q))
        start_wall = time.monotonic()
        agent=None
        if a.motion_mode == "basic-agent":
            from agents.navigation.basic_agent import BasicAgent
            from agents.navigation.local_planner import RoadOption
            vehicle.apply_control(carla.VehicleControl(brake=1.))
            for _ in range(30):
                frame=world.tick()
                for _,q in sensors: receive(q,frame)
            agent=BasicAgent(vehicle,target_speed=15,opt_dict={'ignore_traffic_lights':a.ignore_traffic_lights})
            plan=[(world.get_map().get_waypoint(carla.Location(float(p['x']),-float(p['y']),0)),
                   RoadOption(int(p['road_option_value']))) for p in route['route']]
            agent.set_global_plan(plan)
        previous_location=None
        travel=0.
        indices = np.linspace(0, len(route["route"]) - 1, a.frames, dtype=int)
        for index, route_index in enumerate(indices):
            p = route["route"][int(route_index)]
            # HH_260906 - Scripted relocation for sensor testing; no autonomous control claim.
            loc = carla.Location(float(p["x"]), -float(p["y"]), float(p.get("z", .3)))
            waypoint = world.get_map().get_waypoint(loc)
            loc.z = waypoint.transform.location.z + .3
            yaw = -math.degrees(float(p["yaw"]))
            if agent is None:
                vehicle.set_transform(carla.Transform(loc, carla.Rotation(yaw=yaw)))
            else:
                vehicle.apply_control(agent.run_step())
            # HH_260906 - Three ticks settle rendering; measurements must still match frame and pose.
            images = None
            for _ in range(3 if agent is None else 1):
                frame = world.tick()
                images = [receive(q, frame) for _, q in sensors]
            pose=vehicle.get_transform()
            loc,yaw=pose.location,pose.rotation.yaw
            if previous_location is not None:
                travel+=math.hypot(loc.x-previous_location[0],loc.y-previous_location[1])
            previous_location=(loc.x,loc.y)
            if max(x.timestamp for x in images) - min(x.timestamp for x in images) > 1e-6:
                raise ValueError("sensor timestamp mismatch")
            matrices = [np.asarray(x.transform.get_matrix()) for x in images]
            if not all(np.allclose(matrices[0], m, atol=1e-5) for m in matrices):
                raise ValueError("sensor pose mismatch")
            arrays = [np.frombuffer(x.raw_data, np.uint8).reshape(x.height, x.width, 4).copy()
                      for x in images]
            obs = project_frame(arrays[1], arrays[2], matrices[0], frame_id=evidence.frame_id,
                                timestamp_s=images[0].timestamp, fov_deg=90, label_table=labels)
            evidence.update(obs)
            stem = f"frames/{index:04d}"
            Image.fromarray(arrays[0][..., [2, 1, 0]]).save(a.output / (stem + ".png"))
            np.savez_compressed(a.output / (stem + ".npz"), depth=arrays[1], semantic=arrays[2],
                                world_from_camera=matrices[0])
            record = dict(timestamp_s=obs.timestamp_s, frame=frame, stem=stem, ego_x=loc.x,
                          ego_y=loc.y, ego_yaw_deg=yaw, points=len(obs.points),
                          speed_mps=vehicle.get_velocity().length(),
                          route_index=int(route_index) if agent is None else None)
            records.append(record)
            print(json.dumps(record), flush=True)
        report["capture_wall_seconds"] = time.monotonic() - start_wall
        report.update(travel_m=travel,collision_events=len(collisions),
                      agent_goal_reached=bool(agent.done()) if agent is not None else None)
        if agent is not None and (travel < 3 or collisions):
            raise ValueError("moving collection failed minimum travel or collision check")
        # HH_260906 - Rebuild from saved raw sensor bytes, not cached live observations.
        def saved_observations():
            for r in records:
                with np.load(a.output / (r["stem"] + ".npz"), allow_pickle=False) as data:
                    yield project_frame(data["depth"], data["semantic"], data["world_from_camera"],
                                        frame_id=evidence.frame_id, timestamp_s=r["timestamp_s"],
                                        fov_deg=90, label_table=labels)
        rebuilt = replay(saved_observations(), frame_id=evidence.frame_id, source="oracle_sensor",
                         max_cells=200000)
        report["live_replay_exact_match"] = rebuilt.snapshot() == evidence.snapshot()
        if not report["live_replay_exact_match"]:
            raise ValueError("replay mismatch")
        (a.output / "map.json").write_text(json.dumps(evidence.snapshot()))
        report.update(status="COMPLETE_ORACLE_PLUMBING_ONLY", frame_count=len(records),
                      observed_cells=len(evidence.cells), hd_map_approved=False)
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
        (a.output / "records.json").write_text(json.dumps(records, indent=2))
        (a.output / "summary.json").write_text(json.dumps(report, indent=2))
    if report["status"] != "COMPLETE_ORACLE_PLUMBING_ONLY":
        raise RuntimeError("mapping probe failed")


if __name__ == "__main__":
    run(parse_args())
