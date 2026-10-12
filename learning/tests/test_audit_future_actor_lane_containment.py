"""HH_260906 - Exercise the future-only intended-lane gate without a CARLA server."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from shapely.geometry import box

SCRIPT = Path(__file__).resolve().parents[2] / "tools/e2e/audit_future_actor_lane_containment.py"
SPEC = importlib.util.spec_from_file_location("audit_future_actor_lane_containment", SCRIPT)
assert SPEC and SPEC.loader
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


COLLECTOR = Path(gate.__file__).with_name("collect_carla_vad_expert.py")


class FakeLocation:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class FakeRotation:
    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0):
        self.roll, self.pitch, self.yaw = roll, pitch, yaw


class FakeTransform:
    def __init__(self, location, rotation):
        self.location, self.rotation = location, rotation


class FakeBox:
    def __init__(self, location, extent):
        self.location, self.extent = location, extent
        self.rotation = FakeRotation()

    def get_world_vertices(self, transform):
        # HH_260906 - Model the same actor-local offset and box yaw used by CARLA's API.
        angle = math.radians(transform.rotation.yaw)
        local = math.radians(self.rotation.yaw)
        ca, sa = math.cos(angle), math.sin(angle)
        cb, sb = math.cos(local), math.sin(local)
        result = []
        for x in (-self.extent.x, self.extent.x):
            for y in (-self.extent.y, self.extent.y):
                for z in (-self.extent.z, self.extent.z):
                    bx, by = self.location.x + cb * x - sb * y, self.location.y + sb * x + cb * y
                    result.append(FakeLocation(transform.location.x + ca * bx - sa * by,
                                               transform.location.y + sa * bx + ca * by,
                                               transform.location.z + self.location.z + z))
        return result


class FakeMap:
    def __init__(self, name, xml):
        assert name == "archived-active-map"
        assert "<OpenDRIVE>" in xml

    def get_waypoint(self, location, project_to_road=False, lane_type=None):
        assert not project_to_road
        if not 0 <= location.x <= 20 or abs(location.y) >= 2:
            return None
        return self.get_waypoint_xodr(1, -1, location.x)

    def get_waypoint_xodr(self, road_id, lane_id, s):
        if road_id != 1 or lane_id != -1 or not 0 <= s <= 20:
            return None
        return SimpleNamespace(road_id=1, section_id=0, lane_id=-1, s=s, lane_width=4.0,
                               transform=FakeTransform(FakeLocation(x=s, y=0.0, z=0.0), FakeRotation()))


FakeCarla = SimpleNamespace(Location=FakeLocation, Vector3D=FakeLocation,
                            Rotation=FakeRotation, Transform=FakeTransform,
                            BoundingBox=FakeBox, Map=FakeMap,
                            LaneType=SimpleNamespace(Driving=1))


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, sort_keys=True) + "\n")


def episode(tmp_path: Path) -> Path:
    source = tmp_path / "episode"
    source.mkdir()
    route = {"coordinate_reference": "base_link", "route": [
        {"x": x, "y": 0.0, "z": 0.0, "distance_m": x,
         "road_id": 1, "section_id": 0, "lane_id": -1} for x in (0.0, 5.0, 10.0)]}
    _write_json(source / "route.json", route)
    xml = b'<OpenDRIVE><road id="1" length="20"><lanes><laneSection s="0"><right><lane id="-1" type="driving"/></right></laneSection></lanes></road></OpenDRIVE>'
    (source / "active_map_opendrive.xodr").write_bytes(xml)
    states = []
    for frame, x in enumerate((1.0, 1.1, 1.2), start=1):
        states.append({"frame": frame, "timestamp": frame * 0.05, "capture_phase": "driving",
                       "control_transport": {"status": "PASS"},
                       "actor_snapshot_transform_carla": {"x": x, "y": 0.0, "z": 0.0,
                                                          "roll": 0.0, "pitch": 0.0, "yaw": 0.0}})
    with (source / "states.jsonl").open("w") as stream:
        for state in states:
            stream.write(json.dumps(state) + "\n")
    manifest = {"status": "complete", "files": {
        "route": "route.json", "states": "states.jsonl",
        "active_map_opendrive": "active_map_opendrive.xodr"},
        "provenance": {"route_sha256": gate._sha256(source / "route.json"),
                       "collector_sha256": gate._sha256(COLLECTOR)},
        "capture_contract": {"physics_hz": 20.0,
                             "control_transport": {"mode": "acknowledged_batch"},
                             "agent_initialization": {"construction": {
                                 "status": "PASS", "before": {"actor_id": 7}, "after": {"actor_id": 7}}},
                             "actor_body_geometry": {
                                 "status": "RECORDED", "source": "carla.Vehicle.bounding_box",
                                 "reference_frame": "CARLA actor-local; not ROS rear-axle base_link",
                                 "actor_id": 7, "lane_body_containment_verified": False,
                                 "bounding_box": {"location": {"x": 0.0, "y": 0.0, "z": 0.0},
                                                  "extent": {"x": 0.5, "y": 0.5, "z": 0.5},
                                                  "rotation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}}},
                             "active_map_opendrive": {
                                 "status": "RECORDED", "source": "carla.Map.to_opendrive",
                                 "file": "active_map_opendrive.xodr", "sha256": hashlib.sha256(xml).hexdigest(),
                                 "size_bytes": len(xml), "lane_body_containment_verified": False}}}
    _write_json(source / "manifest.json", manifest)
    return source


def test_valid_sampled_actor_body_and_sweep_pass_without_training_admission(tmp_path: Path) -> None:
    source = episode(tmp_path)
    result = gate.audit_episode(source, carla_module=FakeCarla)
    assert result["status"] == "PASS"
    assert result["frames_checked"] == 3 and result["sweeps_checked"] == 2
    assert result["selected_intended_lanes"] == [[1, 0, -1]]
    assert result["training_data_approved"] is False
    assert result["physical_continuous_time_proven"] is False
    assert result["source_sha256"]["active_map_opendrive"] == gate._sha256(source / "active_map_opendrive.xodr")


def test_body_outside_intended_lane_fails(tmp_path: Path) -> None:
    source = episode(tmp_path)
    states = [json.loads(line) for line in (source / "states.jsonl").read_text().splitlines()]
    for state in states:
        state["actor_snapshot_transform_carla"]["y"] = 1.6
    (source / "states.jsonl").write_text("".join(json.dumps(row) + "\n" for row in states))
    result = gate.audit_episode(source, carla_module=FakeCarla)
    assert result["status"] == "FAIL"
    assert result["body_raw_sampled_lane_violation_count"] >= 1
    assert result["failure_detail"]["interpretation"] == "raw_sampled_lane_intrusion"


def test_interpolated_sweep_can_fail_when_sampled_bodies_fit(tmp_path: Path) -> None:
    source = episode(tmp_path)
    states = [json.loads(line) for line in (source / "states.jsonl").read_text().splitlines()]
    for row in states:
        row["actor_snapshot_transform_carla"]["y"] = 1.28
    states[1]["actor_snapshot_transform_carla"]["yaw"] = 5.0
    (source / "states.jsonl").write_text("".join(json.dumps(row) + "\n" for row in states))
    result = gate.audit_episode(source, carla_module=FakeCarla)
    assert result["status"] == "FAIL"
    assert result["body_raw_sampled_lane_violation_count"] == 0
    assert result["body_inset_10cm_violation_count"] == 0
    assert result["sweep_inset_10cm_violation_count"] >= 1
    assert result["failure_detail"]["kind"] == "bounded_snapshot_linear_sweep"


def test_only_route_listed_neighbor_can_bridge_a_sub_5mm_seam() -> None:
    first = (1, 0, -1)
    next_lane = (2, 0, -1)
    unrelated = (3, 0, -1)
    corridors = {first: box(0, -2, 5, 2), next_lane: box(5.002, -2, 10, 2),
                 unrelated: box(5.002, -2, 10, 2)}
    spanning_body = box(4.75, -0.5, 5.25, 0.5)
    assert gate.authorized_lane_region(corridors, {first, next_lane}, 0.1).covers(spanning_body)
    assert not gate.authorized_lane_region(corridors, {first}, 0.1).covers(spanning_body)
    assert gate.authorized_lane_region(corridors, {first, unrelated}, 0.1).covers(spanning_body)
    # HH_260906 - The caller, not geometric proximity, must authorize lane 2; lane 3 cannot be substituted.
    route = [{"road_id": 1, "section_id": 0, "lane_id": -1, "distance_m": 0.0},
             {"road_id": 2, "section_id": 0, "lane_id": -1, "distance_m": 5.0},
             {"road_id": 2, "section_id": 0, "lane_id": -1, "distance_m": 10.0}]
    assert gate._route_lanes_for_arc_interval(route, 4.8, 5.2) == {first, next_lane}
    assert unrelated not in gate._route_lanes_for_arc_interval(route, 4.8, 5.2)
    assert gate._route_lanes_for_arc_interval(route, 7.0, 8.0) == {next_lane}


def test_wide_gap_between_route_lanes_is_unsupported() -> None:
    first, second = (1, 0, -1), (2, 0, -1)
    with pytest.raises(gate.Unsupported, match="gap wider"):
        gate.authorized_lane_region({first: box(0, -2, 5, 2), second: box(5.02, -2, 10, 2)},
                                    {first, second}, 0.1)


def test_nonmonotone_raw_vs_inset_geometry_fails_closed(tmp_path: Path, monkeypatch) -> None:
    source = episode(tmp_path)
    def broken_region(_corridors, _lanes, inset_m):
        return box(-100, -100, 100, 100) if inset_m else box(0, -0.1, 10, 0.1)
    monkeypatch.setattr(gate, "authorized_lane_region", broken_region)
    with pytest.raises(gate.Unsupported, match="monotonicity"):
        gate.audit_episode(source, carla_module=FakeCarla)


@pytest.mark.parametrize("kind", ["route", "map", "collector", "actor", "absent_box", "frame", "lane"])
def test_missing_proof_never_passes(tmp_path: Path, kind: str) -> None:
    source = episode(tmp_path)
    manifest_path = source / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if kind == "route":
        (source / "route.json").write_text((source / "route.json").read_text() + " ")
    elif kind == "map":
        (source / "active_map_opendrive.xodr").write_bytes(
            (source / "active_map_opendrive.xodr").read_bytes() + b" ")
    elif kind == "collector":
        manifest["provenance"]["collector_sha256"] = "0" * 64
    elif kind == "actor":
        manifest["capture_contract"]["actor_body_geometry"]["actor_id"] = 8
    elif kind == "absent_box":
        del manifest["capture_contract"]["actor_body_geometry"]
    elif kind == "frame":
        rows = [json.loads(line) for line in (source / "states.jsonl").read_text().splitlines()]
        rows[1]["frame"] = 9
        (source / "states.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    elif kind == "lane":
        route = json.loads((source / "route.json").read_text())
        for point in route["route"]:
            point["lane_id"] = 1
        _write_json(source / "route.json", route)
        manifest["provenance"]["route_sha256"] = gate._sha256(source / "route.json")
    _write_json(manifest_path, manifest)
    with pytest.raises(gate.Unsupported):
        gate.audit_episode(source, carla_module=FakeCarla)


def test_self_crossing_ambiguous_route_projection_is_unsupported() -> None:
    route = [{"x": x, "y": -y, "distance_m": arc} for x, y, arc in
             ((0.0, 0.0, 0.0), (5.0, 0.0, 5.0), (0.0, 0.0, 10.0), (5.0, 0.0, 15.0))]
    with pytest.raises(gate.Unsupported, match="ambiguous"):
        gate._route_projection((2.5, 0.0), route)
