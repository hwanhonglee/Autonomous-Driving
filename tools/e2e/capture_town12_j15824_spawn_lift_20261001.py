#!/usr/bin/env python3
"""HH_261001 - One source-backed 0.50 m spawn lift, with recorded physical checks."""

from __future__ import annotations

from contextlib import contextmanager
import json
import math
from pathlib import Path

from scripts.e2e import capture_town12_j15824_signal_expert_20261001 as base


ROOT, collector, scout = base.ROOT, base.collector, base.scout
SPAWN_Z_OFFSET_M = 0.5
NOMINAL_MANIFEST = ROOT / ("storage/experiments/training/2026-09-27/"
    "drive_long_route_town01_right_s0002_p06_9kph_steer045_purge15_trial_v1/episode/manifest.json")
NOMINAL_MANIFEST_SHA256 = "9258274bd2c426c186db2b811fca0e546b7423fcec75c11d41702c8bea14dbb8"
ORIGINAL_ARGUMENTS, ORIGINAL_CAMERA_CONTRACT = base.collector_arguments, base.camera_contract


def collector_arguments(output: Path, route: Path) -> list[str]:
    return [*ORIGINAL_ARGUMENTS(output, route), "--spawn-z-offset-m", "0.5"]


def camera_contract() -> dict:
    return {**ORIGINAL_CAMERA_CONTRACT(),
        "appearance_variant_id": "source_editor_postprocess_enabled_v1",
        "appearance_variant_explicitly_requested": True,
        "common10_legacy_adapter_compatible": False,
        "admission_requirement": "Versioned postprocess-enabled adapter, visual QA and explicit new-data appearance confound.",
        "ablation_caveat": "Old/new data differ in map and postprocess appearance; do not attribute an A/B gain to map alone."}


def nominal_body() -> dict:
    scout.require(scout.digest(NOMINAL_MANIFEST) == NOMINAL_MANIFEST_SHA256,
                  "pinned nominal Prius geometry manifest changed")
    body = json.loads(NOMINAL_MANIFEST.read_text())["capture_contract"]["actor_body_geometry"]
    scout.require(body["actor_type_id"] == "vehicle.toyota.prius", "nominal body is not Prius")
    return body


def spawn_geometry(route: dict, body: dict) -> dict:
    center = collector.shift_transform_local_x(route["start_carla_transform"], collector.WHEELBASE_M / 2)
    spawn = collector.apply_spawn_z_offset(center, SPAWN_Z_OFFSET_M)
    scout.require(all(center[key] == spawn[key] for key in ("x", "y", "yaw", "pitch", "roll")),
                  "spawn lift changed route XY or orientation")
    scout.require(abs(center["pitch"]) < 1e-9 and abs(center["roll"]) < 1e-9,
                  "flat-start nominal clearance checker does not support pitch or roll")
    box = body["bounding_box"]
    scout.require(all(abs(float(value)) < 1e-9 for value in box["rotation"].values()),
                  "rotated nominal bounding box is unsupported")
    location, extent = box["location"], box["extent"]
    bottom = spawn["z"] + location["z"] - extent["z"]
    top = spawn["z"] + location["z"] + extent["z"]
    return {"map_actor_center": center, "actor_spawn": spawn, "spawn_z_offset_m": SPAWN_Z_OFFSET_M,
            "nominal_body_bottom_z_m": bottom, "nominal_body_top_z_m": top,
            "nominal_manifest": str(NOMINAL_MANIFEST), "nominal_manifest_sha256": NOMINAL_MANIFEST_SHA256,
            "nominal_body": body, "xy_orientation_unchanged": True,
            "source_basis": "CarlaGameModeBase::GenerateSpawnPoints adds FVector(0,0,50) UE centimetres"}


def hit_record(hit) -> dict | None:
    return None if hit is None else {"label": str(hit.label),
        "xyz_carla_m": [float(hit.location.x), float(hit.location.y), float(hit.location.z)]}


def physical_spawn_checks(world, carla, route: dict, record: dict) -> None:
    """HH_261001 - Fail closed on sparse observed blockers; never claim volume overlap proof."""
    center, spawn = record["map_actor_center"], record["actor_spawn"]
    body = record["nominal_body"]["bounding_box"]
    loc, ext = body["location"], body["extent"]
    yaw = math.radians(spawn["yaw"])

    def xy(x, y):
        return spawn["x"] + x * math.cos(yaw) - y * math.sin(yaw), spawn["y"] + x * math.sin(yaw) + y * math.cos(yaw)

    map_ = world.get_map()
    waypoint = map_.get_waypoint(carla.Location(center["x"], center["y"], center["z"]), project_to_road=False)
    expected = route["route"][0]
    scout.require(waypoint is not None and (int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id))
                  == tuple(int(expected[key]) for key in ("road_id", "section_id", "lane_id")),
                  "unchanged XY actor center no longer lies on fixed start lane")
    record["start_lane"] = {key: int(getattr(waypoint, key)) for key in ("road_id", "section_id", "lane_id")}
    forbidden = [actor for actor in world.get_actors() if actor.type_id.startswith(("vehicle.", "walker.", "sensor."))]
    record["preexisting_dynamic_actors"] = [{"id": int(actor.id), "type_id": actor.type_id} for actor in forbidden]
    scout.require(not forbidden, "foreign vehicle/walker/sensor present before spawn; no actor will be destroyed")
    ground, rays = record["ground_samples"], record["clearance_rays"]
    xs = [loc["x"] + factor * ext["x"] for factor in (-1, 0, 1)]
    ys = [loc["y"] + factor * ext["y"] for factor in (-1, 0, 1)]
    for x in xs:
        for y in ys:
            px, py = xy(x, y)
            hit = world.ground_projection(carla.Location(px, py, center["z"] + 5.0), 12.0)
            row = {"local_xy_m": [x, y], "hit": hit_record(hit)}
            ground.append(row)
            scout.require(hit is not None, "nominal footprint physical ground is unavailable")
            scout.require(str(hit.label).rsplit(".", 1)[-1] in {"Roads", "RoadLines"},
                          "nominal footprint ground is not road surface")
            row["nominal_spawn_body_ground_gap_m"] = record["nominal_body_bottom_z_m"] - float(hit.location.z)
            scout.require(abs(float(hit.location.z) - center["z"]) <= 0.2,
                          "nominal footprint ground elevation differs from fixed road start")
            scout.require(row["nominal_spawn_body_ground_gap_m"] >= 0.25,
                          "source-backed lift lacks 0.25 m nominal bottom clearance")

    def ray(start, end):
        delta = [end[index] - start[index] for index in range(3)]
        length = math.sqrt(sum(value * value for value in delta))
        hit = world.project_point(carla.Location(*start), carla.Vector3D(*(value / length for value in delta)), length)
        rays.append({"start_xyz_carla_m": start, "end_xyz_carla_m": end, "hit": hit_record(hit)})
        scout.require(hit is None, "nominal spawn body sparse clearance ray hit a physical blocker")

    bottom, top = record["nominal_body_bottom_z_m"], record["nominal_body_top_z_m"]
    for z in (bottom, (bottom + top) / 2, top):
        for y in ys:
            ray([*xy(xs[0], y), z], [*xy(xs[-1], y), z])
        for x in xs:
            ray([*xy(x, ys[0]), z], [*xy(x, ys[-1]), z])
    for x in xs:
        for y in ys:
            ray([*xy(x, y), bottom], [*xy(x, y), top])
    record["minimum_nominal_body_ground_gap_m"] = min(row["nominal_spawn_body_ground_gap_m"] for row in ground)
    record["query_api"] = "ground_projection and project_point; large-map coordinate conversion verified in source"
    record["limitation"] = "9 ground samples and 27 rays on camera collision channel are sparse checks, not an exhaustive physics-volume overlap test. Nominal bounding box is cross-map evidence; actual spawned box still requires QA."


@contextmanager
def configured_worker(output: Path | None = None, scout_directory: Path | None = None):
    old_arguments, old_camera, old_run = base.collector_arguments, base.camera_contract, collector.run

    def checked_run(args):
        scout.require(output is not None and scout_directory is not None, "live spawn checks require owned output")
        receipt = base.validate_scout_receipt(scout_directory)
        carla, _ = scout.source_api()
        client = carla.Client("127.0.0.1", 2100)
        client.set_timeout(60.0)
        world = client.get_world()
        route = collector.load_route(scout_directory / "route.json")
        record = {**spawn_geometry(route, nominal_body()), "status": "CHECKING", "ground_samples": [], "clearance_rays": [],
                  "world_id": int(world.id), "spawn_attempt_budget": 1, "alternative_poses_allowed": False}
        try:
            scout.require(int(world.id) == receipt["world_id"], "episode changed before physical spawn checks")
            scout.require(args.spawn_z_offset_m == SPAWN_Z_OFFSET_M, "spawn offset does not match frozen trial")
            physical_spawn_checks(world, carla, route, record)
            record["status"] = "PASS_SPARSE_PRESPAWN_CHECKS"
        except Exception as error:
            record.update(status="FAIL", error=f"{type(error).__name__}: {error}")
            raise
        finally:
            scout.write_new(output / "spawn_clearance.json", record)
        old_write = collector._write_json

        def write(path, value):
            if path.name == "manifest.json":
                value = {**value, "town12_spawn_lift_trial": {**record,
                    "spawn_clearance_sha256": scout.digest(output / "spawn_clearance.json"),
                    "dataset_admitted": False}}
            old_write(path, value)

        try:
            collector._write_json = write
            return old_run(args)
        finally:
            collector._write_json = old_write

    try:
        base.collector_arguments, base.camera_contract, collector.run = collector_arguments, camera_contract, checked_run
        yield
    finally:
        base.collector_arguments, base.camera_contract, collector.run = old_arguments, old_camera, old_run


def main(argv=None) -> int:
    import sys
    arguments = list(sys.argv[1:] if argv is None else argv)
    def path_option(name):
        return Path(arguments[arguments.index(name) + 1]) if name in arguments else None
    with configured_worker(path_option("--output"), path_option("--scout")):
        return base.main(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
