"""HH_261004 - CPU-only synthetic topology/geometry contracts; no real map, poses, model or server reads."""
from copy import deepcopy
import hashlib
import json
import sys
from types import SimpleNamespace

import pytest
from shapely.geometry import box

from scripts.e2e import audit_town03_straight_start_corridor_20261004 as audit


def digest(value):
    return hashlib.sha256(value).hexdigest()


@pytest.fixture
def fixture(monkeypatch):
    # HH_261004 - These explicitly synthetic pins never authenticate an actual capture.
    xml = b'''<OpenDRIVE><road id="1426" length="38.61" junction="1352"><link>
      <successor elementType="road" elementId="67" contactPoint="start"/></link><lanes><laneSection>
      <left><lane id="2" type="driving"><link><successor id="2"/></link></lane></left>
      </laneSection></lanes></road><junction id="1352"><connection id="7" incomingRoad="67"
      connectingRoad="1426" contactPoint="end"><laneLink from="2" to="2"/></connection></junction></OpenDRIVE>'''
    route = dict(coordinate_reference="base_link", route=[dict(road_id=1426, section_id=0, lane_id=2,
        x=0., y=float(i), z=0., distance_m=float(i)) for i in range(5)])
    canonical = digest(json.dumps(route, sort_keys=True, separators=(",", ":"), allow_nan=False).encode())
    monkeypatch.setattr(audit, "EXPECTED_XODR_SHA256", digest(xml))
    monkeypatch.setattr(audit, "EXPECTED_ROUTE_CANONICAL_SHA256", canonical)
    manifest = dict(provenance=dict(route_sha256=audit.EXPECTED_ROUTE_SHA256), capture_contract=dict(
        actor_body_geometry=dict(status="RECORDED", source="carla.Vehicle.bounding_box", actor_id=86,
            reference_frame="CARLA actor-local; not ROS rear-axle base_link"),
        active_map_opendrive=dict(status="RECORDED", source="carla.Map.to_opendrive",
            file="active_map_opendrive.xodr", map_name="Carla/Maps/Town03", sha256=digest(xml), size_bytes=len(xml))))
    faults = dict(previous=[(67, 0, 2)], yaw=0., width=0., gap=.002)

    class Waypoint:
        def __init__(self, road, lane, s):
            self.road_id, self.section_id, self.lane_id = road, 0, lane
            self.lane_width = 3.5 + (faults["width"] if road == 67 else 0)
            self.transform = SimpleNamespace(location=SimpleNamespace(x=0., y=s if road == 67 else s - 38.61, z=0.),
                rotation=SimpleNamespace(yaw=-90. + (faults["yaw"] if road == 67 else 0)))
        def previous(self, _):
            return [SimpleNamespace(road_id=r, section_id=s, lane_id=l) for r, s, l in faults["previous"]]

    fake = SimpleNamespace(Map=lambda *_: SimpleNamespace(get_waypoint_xodr=lambda r, l, s: Waypoint(r, l, s)))
    monkeypatch.setitem(sys.modules, "carla", fake)
    def corridors(_map, _carla, _points, _xml, requested, _indices):
        return {lane: box(-1.75, -20., 1.75, -.001) if lane == audit.INITIAL_ROUTE_LANE
                else box(-1.75, faults["gap"] - .001, 1.75, 6.051) for lane in requested}
    monkeypatch.setattr(audit.body, "lane_corridors", corridors)
    monkeypatch.setattr(audit.body, "_body_box", lambda *_: (None, 1.2))
    monkeypatch.setattr(audit.body, "body_footprint", lambda _box, p, _carla:
        box(p["x"] - .5, p["y"] - .9, p["x"] + .5, p["y"] + .9))
    states = [dict(frame=10 + i, timestamp=.05 * i, capture_phase="stationary_warmup" if i < 2 else "driving",
        research_control_source="expert_startup" if i < 2 else "learned_trajectory_tracker",
        actor_snapshot_transform_carla=dict(x=0., y=-arc, z=0., roll=0., pitch=0., yaw=-90.))
        for i, arc in enumerate((.4, .8, 1.2, 1.6, 2.))]
    return SimpleNamespace(xml=xml, route=route, manifest=manifest, states=states, faults=faults)


def call(f):
    return audit.body_census_with_provenance(f.manifest, f.route, f.states, f.xml, 86)


def test_connected_start_keeps_whole_denominator_original_failure_and_all_margins(fixture):
    rows, census, proof = call(fixture)
    assert len(rows) == 5 and proof["all_native_boxes"] == 5 and proof["all_snapshot_linear_sweeps"] == 4
    assert len(census) == 4 and all(v["status"] == "PASS" and v["violation_count"] == 0 for v in census.values())
    original = proof["recorded_route_only_census"]
    assert original["sampled_body_raw_lane"]["status"] == "FAIL"
    assert original["sampled_body_raw_lane"]["violation_frames"] == [10, 11]
    assert all(v["minimum_interior_clearance_m"] >= .1 for v in census.values())
    assert proof["prefix_body_frames"] == [10, 11] and proof["prefix_sweep_frames"] == [11, 12]
    assert proof["seam_gap_m"] == pytest.approx(.002)
    assert proof["lane_inset_m"] == .1 and proof["seam_close_limit_m"] == .005 and proof["sweep_position_guard_m"] == .02
    assert proof["original_fail_result_modified"] is proof["route_input_modified"] is False
    assert proof["feature_approved"] is proof["training_data_approved"] is False


def test_fixed_lo_arc_condition_not_selected_from_geometry_failure(fixture, monkeypatch):
    monkeypatch.setattr(audit.body, "body_footprint", lambda _box, p, _carla:
        box(p["x"] - .5, p["y"] - .2, p["x"] + .5, p["y"] + .2))
    rows, _, _ = call(fixture)
    assert rows[0]["recorded_route_only_body"]["raw_lane"]["area_gate_clear"] is True
    assert rows[0]["start_connected_prefix_added"] is True
    assert rows[2]["start_connected_prefix_added"] is False
    assert rows[2]["intended_lane_ids"] == [[1426, 0, 2]]
    assert rows[0]["intended_lane_ids"] == [[67, 0, 2], [1426, 0, 2]]
    assert rows[0]["original_recorded_route_lane_ids"] == [[1426, 0, 2]]


def test_two_result_owner_ABI_and_separate_static_proof(fixture):
    rows, census = audit.body_census(fixture.manifest, fixture.route, fixture.states, fixture.xml, 86)
    proof = audit.topology_provenance(fixture.manifest, fixture.route, fixture.xml, 86)
    assert len(rows) == 5 and len(census) == 4
    assert proof["authorized_start_prefix_lane"] == [67, 0, 2] and proof["unique_offline_previous_proven"] is True
    stored = rows[0]["authenticated_start_corridor_provenance"]
    assert stored["all_native_boxes"] == 5 and stored["all_snapshot_linear_sweeps"] == 4
    assert stored["recorded_route_only_census"]["sampled_body_raw_lane"]["status"] == "FAIL"
    assert stored["seam_gap_m"] == pytest.approx(.002) and stored["body_radius_m"] == 1.2
    assert "authenticated_start_corridor_provenance" not in rows[1]
    # HH_261004 - The caller can archive the actual proof without extra I/O or recursive JSON references.
    assert json.loads(json.dumps(rows, allow_nan=False))[0]["authenticated_start_corridor_provenance"] == stored


def test_inputs_and_shared_geometry_global_namespace_stay_unmodified(fixture):
    before = deepcopy((fixture.manifest, fixture.route, fixture.states))
    old = {name: getattr(audit.original, name) for name in ("body", "body_census", "region_measurement")}
    call(fixture)
    assert (fixture.manifest, fixture.route, fixture.states) == before
    assert all(getattr(audit.original, name) is value for name, value in old.items())


@pytest.mark.parametrize("previous", [[], [(67, 0, 3)], [(67, 1, 2)], [(68, 0, 2)], [(67, 0, 2), (68, 0, 2)]])
def test_unique_exact_physical_previous_required(fixture, previous):
    fixture.faults["previous"] = previous
    with pytest.raises(ValueError, match="unique declared upstream"):
        call(fixture)


@pytest.mark.parametrize("fault,value,message", [("gap", .006, "5mm"), ("yaw", .002, "yaw/width"), ("width", .000002, "yaw/width")])
def test_seam_yaw_width_must_meet_proof_not_weaken_safety(fixture, fault, value, message):
    fixture.faults[fault] = value
    with pytest.raises(ValueError, match=message): call(fixture)


@pytest.mark.parametrize("old,new", [
    (b'elementId="67"', b'elementId="68"'), (b'<successor id="2"', b'<successor id="3"'),
    (b'incomingRoad="67"', b'incomingRoad="68"'), (b'connectingRoad="1426"', b'connectingRoad="1425"'),
    (b'contactPoint="end"', b'contactPoint="start"'), (b'from="2" to="2"', b'from="2" to="3"'),
    (b'type="driving"', b'type="sidewalk"'), (b'junction="1352"', b'junction="1353"'),
])
def test_even_test_authenticated_xml_requires_explicit_declared_links(fixture, monkeypatch, old, new):
    fixture.xml = fixture.xml.replace(old, new)
    pin = digest(fixture.xml)
    monkeypatch.setattr(audit, "EXPECTED_XODR_SHA256", pin)
    fixture.manifest["capture_contract"]["active_map_opendrive"].update(sha256=pin, size_bytes=len(fixture.xml))
    with pytest.raises(ValueError, match="connection proof|junction road"):
        call(fixture)


@pytest.mark.parametrize("fault", ["xml", "route_content", "route_metadata", "map_size", "map_name", "map_sha", "actor", "reference"])
def test_exact_sealed_source_route_map_actor_required(fixture, fault):
    if fault == "xml": fixture.xml += b' '
    elif fault == "route_content": fixture.route["route"][0]["x"] = .01
    elif fault == "route_metadata": fixture.manifest["provenance"]["route_sha256"] = '0' * 64
    elif fault == "actor": fixture.manifest["capture_contract"]["actor_body_geometry"]["actor_id"] = 87
    elif fault == "reference": fixture.manifest["capture_contract"]["actor_body_geometry"]["reference_frame"] = 'rear axle'
    else:
        field = {"map_size": "size_bytes", "map_name": "map_name", "map_sha": "sha256"}[fault]
        fixture.manifest["capture_contract"]["active_map_opendrive"][field] = 'wrong'
    with pytest.raises(ValueError): call(fixture)


@pytest.mark.parametrize("constant", ["FROZEN_GEOMETRY_SHA256", "FROZEN_BODY_SHA256"])
def test_frozen_geometry_byte_guard_fails_before_consuming_map(fixture, monkeypatch, constant):
    monkeypatch.setattr(audit, constant, '0' * 64)
    with pytest.raises(ValueError, match="source changed"): call(fixture)


@pytest.mark.parametrize("fault", ["frame_gap", "time_gap", "too_few", "reverse", "translation", "angle"])
def test_existing_native_projection_and_sweep_guards_not_bypassed(fixture, fault):
    if fault == "frame_gap": fixture.states[1]["frame"] += 1
    elif fault == "time_gap": fixture.states[1]["timestamp"] += .001
    elif fault == "too_few": fixture.states = fixture.states[:1]
    elif fault == "reverse": fixture.states[1]["actor_snapshot_transform_carla"]["y"] = -.01
    elif fault == "translation": fixture.states[1]["actor_snapshot_transform_carla"]["x"] = 2.
    else: fixture.states[1]["actor_snapshot_transform_carla"]["yaw"] = -80.
    with pytest.raises((ValueError, audit.body.Unsupported)): call(fixture)


def test_lateral_intrusion_remains_failure_with_connected_prefix(fixture):
    for state in fixture.states: state["actor_snapshot_transform_carla"]["x"] = 1.5
    _, census, proof = call(fixture)
    assert all(v["status"] == "FAIL" for v in census.values())
    assert census["sampled_body_raw_lane"]["violation_count"] == 5
    assert proof["all_native_boxes"] == 5 and proof["all_snapshot_linear_sweeps"] == 4


def test_connected_prefix_is_not_authorized_after_arc_zero_support_finishes(fixture):
    rows, _, _ = call(fixture)
    assert all(r["intended_lane_ids"] == [[1426, 0, 2]] for r in rows[2:])
    assert all(r["previous_sweep"]["intended_lane_ids"] == [[1426, 0, 2]] for r in rows[3:])
