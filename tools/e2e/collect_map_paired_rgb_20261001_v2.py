#!/usr/bin/env python3
"""HH_261001 - V2 paired RGB; raw target postprocess attribute only when supported."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shutil
import signal
import sys
import time

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
PACKAGE = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
SHIPPING = PACKAGE/"CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
ROLE = "known_train_routes_paired_appearance_diagnostic"
ROUTES = {
    "df68ac6046064faefe5d6a19425e0b038e94d2171878741f5c713a35ec0ce8de":("Town01","right"),
    "520fc0236e32d0664bc125a15f45601e4ef020cf1e3f1db888332af0e188f08b":("Town02_Opt","straight")}
MAP_SHA = {"Town01":"e48e2a4a3322f614c3fd8d1c066ae239c0368254c5508592a6f4b1830862dd47",
           "Town02_Opt":"72162e08a4f033ee02bf720dbb4e93a457ad6aa097d693f549a4f5cb2a3beaed"}
XODR_SHA = {"Town01":"97a7f6ac67812567e5c8ee0599cd823b23f80f30f3f97c502212e38b72e2b709",
            "Town02_Opt":"953c05f17def231239ffcadba3307628a82d0d335b67b4e29c3098f7aed1dd7d"}
SENSOR_SPECS = (("rgb_false","rgb","false"),("rgb_true","rgb","true"),
                ("depth","depth","false"),("semantic","semantic_segmentation","false"))
ATTRIBUTES = {"image_size_x":"640","image_size_y":"360","fov":"90",
              "sensor_tick":"0","lens_k":"0","lens_kcube":"0"}


def configure_camera(bp, kind, postprocess):
    """HH_261001 - RGB switch is mandatory; raw-target support is recorded honestly."""
    for key,value in ATTRIBUTES.items():
        if not bp.has_attribute(key):
            raise ValueError("required camera attribute missing: "+key)
        bp.set_attribute(key,value)
    supported = bp.has_attribute("enable_postprocess_effects")
    if kind == "rgb" and not supported:
        raise ValueError("RGB must expose enable_postprocess_effects")
    if supported:
        bp.set_attribute("enable_postprocess_effects",postprocess)
    return {"supported":supported,"applied_value":postprocess if supported else None,
            "raw_target_historical_equivalence":kind != "rgb"}


def digest(path):
    if not path.is_file() or path.is_symlink():
        raise ValueError(f"regular source required: {path}")
    state = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda:stream.read(1024*1024),b""):
            state.update(block)
    return state.hexdigest()


def package_assets(town):
    content = PACKAGE/"CarlaUE4/Content/Carla/Maps"
    expected = {content/(town+".umap"):MAP_SHA[town],
        content/"OpenDrive"/(town+".xodr"):XODR_SHA[town],
        SHIPPING:"34a97404e1dbf61d2ebfb6dae18970ec5684f65c08c934cc7f65a5bd2b7f5192",
        PACKAGE/"CarlaUE4/Config/DefaultEngine.ini":"c948c7b8ada9fafa38485a8523fb331a9b76af29fc6675fe77fe2e26c560206c"}
    if any(digest(path) != sha for path,sha in expected.items()):
        raise ValueError("pinned packaged map/runtime/config differs")
    return {str(path):sha for path,sha in expected.items()}


def load_bound_route(args, world_town=None):
    sha = digest(args.route_file)
    route = json.loads(args.route_file.read_text())
    if (sha not in ROUTES or (route.get("town"),route.get("scenario")) != ROUTES[sha]
            or route.get("coordinate_reference") != "base_link" or len(route.get("route",[])) < 2):
        raise ValueError("only exact Town01 TRAIN rightp01 or Town02 TRAIN straightp00 allowed")
    if world_town is not None and world_town != route["town"]:
        raise ValueError("live map differs from bound TRAIN route")
    return route,sha


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("output",type=Path)
    parser.add_argument("route_file",type=Path)
    parser.add_argument("--host",choices=["127.0.0.1"],default="127.0.0.1")
    parser.add_argument("--port",type=int,default=2400)
    parser.add_argument("--frames",type=int,choices=[240],default=240)
    args = parser.parse_args(argv)
    args.output,args.route_file = args.output.absolute(),args.route_file.absolute()
    if args.output.exists() or args.output.is_symlink() or not 1024 <= args.port <= 65533:
        parser.error("fresh output and valid port required")
    route,_ = load_bound_route(args)
    package_assets(route["town"])
    if shutil.disk_usage(ROOT).free < 5*1024**3:
        raise ValueError("minimum 5 GiB disk reserve required")
    args.allow_map_load = False
    args.motion_mode,args.ignore_traffic_lights = "basic-agent",True
    for name in ("town06_sign_scout","town03_generalization_train","town07_generalization_test",
            "town05_generalization_dev","town02_sign_train_scout_p02","town05_sign_dev_scout",
            "town13_render_sensor_scout"):
        setattr(args,name,False)
    args.town02_sign_train_scout = route["town"] == "Town02_Opt"
    return args


def pair_owner_binding(args,world,route_sha):
    """HH_261001 - Bind exact package listener to the shell owner's live process group."""
    owner = args.output.parent
    plan = json.loads((owner/"owner_plan.json").read_text())
    started = json.loads((owner/"owner_started.json").read_text())
    ready = json.loads((owner/"lifecycle/ready.json").read_text())
    if (plan.get("capture_mode") != "mapping-paired-rgb" or plan["route_sha256"] != route_sha
            or plan["port"] != args.port or started["route_sha256"] != route_sha
            or ready.get("status") != "PASS" or ready["owner_pgid"] != started["server_pgid"]):
        raise ValueError("paired owner binding differs")
    candidates = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            raw = (proc/"stat").read_text(); fields = raw[raw.rfind(")")+2:].split()
            argv = (proc/"cmdline").read_bytes().split(b"\0")
            if int(fields[2]) == started["server_pgid"] and argv and argv[0] and Path(os.fsdecode(argv[0])).resolve() == SHIPPING:
                candidates.append({"pid":int(proc.name),"pgid":int(fields[2]),"start_ticks":int(fields[19])})
        except (FileNotFoundError,ProcessLookupError,PermissionError):
            continue
    if len(candidates) != 1:
        raise ValueError("one exact packaged simulator in owned process group required")
    process = candidates[0]
    inodes = set()
    for suffix in ("tcp","tcp6"):
        for line in Path(f"/proc/{process['pid']}/net/{suffix}").read_text().splitlines()[1:]:
            fields = line.split()
            if int(fields[1].rsplit(":",1)[1],16) == args.port and fields[3] == "0A":
                inodes.add(fields[9])
    sockets = set()
    for path in Path(f"/proc/{process['pid']}/fd").iterdir():
        try:
            sockets.add(os.readlink(path))
        except FileNotFoundError:
            continue
    if not any("socket:["+inode+"]" in sockets for inode in inodes):
        raise ValueError("RPC socket does not belong to the owned shipping process")
    live = world.get_map()
    live_sha = hashlib.sha256(live.to_opendrive().encode()).hexdigest()
    if (live.name not in ("Carla/Maps/"+plan["map"],"/Game/Carla/Maps/"+plan["map"])
            or live_sha != XODR_SHA[plan["map"]]):
        raise ValueError("full map identity or live OpenDRIVE differs")
    return {"shipping_process":process,"live_map_name":live.name,"live_xodr_sha256":live_sha,
            "owner_plan_sha256":digest(owner/"owner_plan.json"),"owner_started_sha256":digest(owner/"owner_started.json")}


def validate_pair_metadata(images, matrices, arrays):
    import numpy as np
    if len(images) != 4 or len({image.frame for image in images}) != 1:
        raise ValueError("four same-frame sensors required")
    if max(image.timestamp for image in images)-min(image.timestamp for image in images) > 1e-6:
        raise ValueError("paired camera timestamps differ")
    if any(array.shape != (360,640,4) or array.dtype != np.uint8 for array in arrays):
        raise ValueError("four native uint8 BGRA arrays required")
    if any(not np.allclose(matrices[0],matrix,atol=1e-5) for matrix in matrices[1:]):
        raise ValueError("paired camera matrices differ")
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
    load_bound_route(a,town)
    binding = pair_owner_binding(a,world,route_sha256)
    source_assets = package_assets(town)
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
    report.update(intended_role=ROLE,dataset_admitted=False,train_eligible=False,
        ctrack_train_eligible=False,independent_dev_approved=False,source_assets=source_assets,
        owner_binding=binding,rgb_conditions=["false","true"],frame_count_expected=240,
        sensor_intrinsics={"width":640,"height":360,"horizontal_fov_deg":90,
                           "K":[[320.0,0.0,320.0],[0.0,320.0,180.0],[0.0,0.0,1.0]],
                           "lens_k":0.0,"lens_kcube":0.0},
        camera_mount={"x":1.5,"y":0.0,"z":2.0,"pitch":-12.0},fixed_delta_seconds=.1,
        target_speed_kph=15.0,sensor_blueprint_attributes={},sensor_actor_attributes={},
        postprocess_attribute_contract={},collector_version="paired_rgb_v2_20261001",
        route_scope="existing TRAIN 240-frame prefix; no whole-route/junction completion claim")
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
        for name,kind,postprocess in SENSOR_SPECS:
            bp = world.get_blueprint_library().find("sensor.camera." + kind)
            report["postprocess_attribute_contract"][name] = configure_camera(bp,kind,postprocess)
            report["sensor_blueprint_attributes"][name] = {
                attribute.id:str(attribute) for attribute in bp}
            sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.5, z=2.0),
                                       carla.Rotation(pitch=-12)), attach_to=vehicle)
            actors.append(sensor)
            report["sensor_actor_attributes"][name] = dict(sensor.attributes)
            q = queue.Queue()
            sensor.listen(q.put)
            sensors.append((sensor, q))
        left_attributes = dict(report["sensor_actor_attributes"]["rgb_false"])
        right_attributes = dict(report["sensor_actor_attributes"]["rgb_true"])
        if (left_attributes.pop("enable_postprocess_effects") != "false"
                or right_attributes.pop("enable_postprocess_effects") != "true"
                or left_attributes != right_attributes):
            raise ValueError("paired RGB effective attributes differ beyond postprocess")
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
            if shutil.disk_usage(ROOT).free < 5*1024**3:
                raise ValueError("disk reserve dropped below 5 GiB")
            p = route["route"][int(route_index)]
            # HH_260906 - Scripted relocation for sensor testing; no autonomous control claim.
            loc = carla.Location(float(p["x"]), -float(p["y"]), float(p.get("z", .3)))
            waypoint = world.get_map().get_waypoint(loc)
            loc.z = waypoint.transform.location.z + .3
            yaw = -math.degrees(float(p["yaw"]))
            if agent is None:
                vehicle.set_transform(carla.Transform(loc, carla.Rotation(yaw=yaw)))
            else:
                control = agent.run_step()
                vehicle.apply_control(control)
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
            validate_pair_metadata(images,matrices,arrays)
            obs = project_frame(arrays[2], arrays[3], matrices[0], frame_id=evidence.frame_id,
                                timestamp_s=images[0].timestamp, fov_deg=90, label_table=labels)
            evidence.update(obs)
            stem = f"frames/{index:04d}"
            for arm,array in zip(("false","true"),arrays[:2]):
                Image.fromarray(array[..., [2,1,0]]).save(a.output/(stem+"."+arm+".png"))
            np.savez_compressed(a.output/(stem+".npz"),rgb_false=arrays[0],rgb_true=arrays[1],
                depth=arrays[2],semantic=arrays[3],world_from_camera=matrices[0])
            record = dict(timestamp_s=obs.timestamp_s, frame=frame, stem=stem, ego_x=loc.x,
                          ego_y=loc.y, ego_yaw_deg=yaw, points=len(obs.points),
                          speed_mps=vehicle.get_velocity().length(),
                          route_index=int(route_index) if agent is None else None)
            record.update(ego_z=loc.z,ego_pitch_deg=pose.rotation.pitch,ego_roll_deg=pose.rotation.roll,
                world_from_ego=pose.get_matrix(),control={"throttle":control.throttle,"brake":control.brake,"steer":control.steer},
                road_pixels=int(np.count_nonzero(arrays[3][...,2] == 1)),
                lane_pixels=int(np.count_nonzero(arrays[3][...,2] == 24)),
                sensors={name:{"actor_id":sensor.id,"frame":int(image.frame),"timestamp_s":float(image.timestamp),
                    "width":int(image.width),"height":int(image.height),"world_from_camera":matrix.tolist(),
                    "raw_bgra_sha256":hashlib.sha256(image.raw_data).hexdigest()}
                    for (name,_,_),(sensor,_),image,matrix in zip(SENSOR_SPECS,sensors,images,matrices)},
                artifact_sha256={suffix:digest(a.output/(stem+suffix)) for suffix in (".false.png",".true.png",".npz")})
            records.append(record)
            print(json.dumps(record), flush=True)
        report["capture_wall_seconds"] = time.monotonic() - start_wall
        report.update(travel_m=travel,collision_events=len(collisions),
                      agent_goal_reached=bool(agent.done()) if agent is not None else None)
        if agent is not None and (travel < 3 or collisions):
            raise ValueError("moving collection failed minimum travel or collision check")
        report.update(road_pixels=sum(row["road_pixels"] for row in records),
                      lane_pixels=sum(row["lane_pixels"] for row in records),
                      lane_positive_frames=sum(row["lane_pixels"] > 0 for row in records))
        if report["road_pixels"] < .01*240*640*360 or report["lane_positive_frames"] < 24:
            raise ValueError("road/lane semantic coverage absent")
        # HH_261001 - Rebuild from saved raw sensor bytes, not cached live observations.
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
        if package_assets(town) != source_assets:
            raise ValueError("packaged source bytes changed during capture")
        report["source_assets_unchanged"] = True
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

