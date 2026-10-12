"""HH_261004 - Diagnose the exact Town03 straight start with its authenticated upstream lane.

This offline geometry kernel does not change a route, actor pose, old result,
inset, sweep bound, dataset admission or capability approval. Only the recorded
body support before route arc zero may use the explicitly connected (67,0,2).
Raw file-byte admission/source closure remains the owning caller's responsibility;
the sealed route's canonical content is checked independently here as well.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from scripts.e2e import audit_simulation_research_body_lane_20261001 as original

body = original.body
require = original.require
region_measurement = original.region_measurement
SCHEMA = "portable_e2e.town03_straight_start_connected_corridor.20261004.v1"
EXPECTED_ROUTE_SHA256 = "8fc442ed6cc6f6e40a4f346c2cfbda26105bd5a53c0992f7c6eb78964838cf06"
EXPECTED_ROUTE_CANONICAL_SHA256 = "05a693932d99a8cf9256e0819d7e2d17dddc730f53da7427d9b6c5603168eb42"
EXPECTED_XODR_SHA256 = "abf177bcd9b66cfefb7ac4919f6456815f1384f4229f1343c8a6a90bbb7713c9"
INITIAL_ROUTE_LANE = (1426, 0, 2)
PREFIX_LANE = (67, 0, 2)
FROZEN_GEOMETRY_SHA256 = "6e932dcb39fa40e664218d3d1dddf20dd524f54e99ca55d98793c76cce8c03ac"
FROZEN_BODY_SHA256 = "75d73449c8ffb17e27d1ddb419bd064a58ab052043b2a7b0ada69de8a65fa57d"
YAW_MATCH_DEG = .001
WIDTH_MATCH_M = 1e-6


def _source_guard():
    # HH_261004 - Reuse exact frozen geometry semantics without monkeypatching shared modules.
    for module, pin in ((original, FROZEN_GEOMETRY_SHA256), (body, FROZEN_BODY_SHA256)):
        require(hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest() == pin,
                "frozen geometry/body source changed")


def _prepare(manifest, route, xml, actor_id, carla):
    _source_guard()
    require(type(xml) is bytes and hashlib.sha256(xml).hexdigest() == EXPECTED_XODR_SHA256,
            "exact sealed Town03 OpenDRIVE required")
    require(manifest.get("provenance", {}).get("route_sha256") == EXPECTED_ROUTE_SHA256
            and hashlib.sha256(json.dumps(route, sort_keys=True, separators=(",", ":"),
                                         allow_nan=False).encode()).hexdigest() == EXPECTED_ROUTE_CANONICAL_SHA256,
            "exact sealed Town03 straight route required")
    capture = manifest["capture_contract"]
    record, map_record = capture["actor_body_geometry"], capture["active_map_opendrive"]
    require(record["status"] == "RECORDED" and record["source"] == "carla.Vehicle.bounding_box"
            and record["reference_frame"] == "CARLA actor-local; not ROS rear-axle base_link"
            and type(actor_id) is int and record["actor_id"] == actor_id,
            "recorded body is not bound to command actor")
    require(map_record["status"] == "RECORDED" and map_record["source"] == "carla.Map.to_opendrive"
            and map_record["file"] == "active_map_opendrive.xodr"
            and map_record["map_name"] == "Carla/Maps/Town03"
            and map_record["sha256"] == EXPECTED_XODR_SHA256 and map_record["size_bytes"] == len(xml),
            "archived active-map identity differs")
    require(route["coordinate_reference"] == "base_link", "route reference differs")
    points = []
    for point in route["route"]:
        body._triple(point)
        if not points or point["distance_m"] != points[-1]["distance_m"]:
            points.append(point)
        else:
            require(all(point[k] == points[-1][k] for k in ("x", "y", "z", "road_id", "section_id", "lane_id")),
                    "duplicate route distance changes geometry or lane")
    require(len(points) >= 2 and points[0]["distance_m"] == 0 and body._triple(points[0]) == INITIAL_ROUTE_LANE
            and PREFIX_LANE not in {body._triple(p) for p in points}, "start corridor scope differs")
    root = ET.fromstring(xml)
    require(root.tag == "OpenDRIVE", "OpenDRIVE root differs")
    road = root.find('./road[@id="1426"]')
    require(road is not None and road.attrib.get("junction") == "1352", "initial junction road differs")
    successor = road.find("./link/successor")
    lane = road.find('./lanes/laneSection/left/lane[@id="2"]')
    lane_successor = None if lane is None else lane.find("./link/successor")
    connection = root.find('./junction[@id="1352"]/connection[@id="7"]')
    require(successor is not None and successor.attrib == dict(elementType="road", elementId="67", contactPoint="start")
            and lane is not None and lane.attrib.get("type") == "driving"
            and lane_successor is not None and lane_successor.attrib == dict(id="2")
            and connection is not None and connection.attrib == dict(id="7", incomingRoad="67", connectingRoad="1426", contactPoint="end")
            and any(item.attrib == {"from": "2", "to": "2"} for item in connection.findall("laneLink")),
            "explicit road/lane/junction connection proof differs")
    length = float(road.attrib["length"])
    cmap = carla.Map("sealed-Town03-straight-start-offline", xml.decode())
    endpoint = cmap.get_waypoint_xodr(1426, 2, length - .001)
    upstream = cmap.get_waypoint_xodr(67, 2, .001)
    require(endpoint is not None and upstream is not None
            and (endpoint.road_id, endpoint.section_id, endpoint.lane_id) == INITIAL_ROUTE_LANE
            and (upstream.road_id, upstream.section_id, upstream.lane_id) == PREFIX_LANE,
            "exact lane endpoints unavailable")
    preceding = endpoint.previous(1.)
    require(len(preceding) == 1 and (preceding[0].road_id, preceding[0].section_id, preceding[0].lane_id) == PREFIX_LANE,
            "offline CARLA previous must prove a unique declared upstream lane")
    yaw_delta = body._angle_delta_degrees(endpoint.transform.rotation.yaw, upstream.transform.rotation.yaw)
    require(yaw_delta <= YAW_MATCH_DEG and abs(endpoint.lane_width - upstream.lane_width) <= WIDTH_MATCH_M
            and endpoint.lane_width > 2 * body.LANE_INSET_M, "connected lane yaw/width differs")
    lanes = {body._triple(p) for p in points}
    corridors = body.lane_corridors(cmap, carla, points, xml, lanes, set(range(len(points))))
    diagnostic_prefix = []
    for index, s in enumerate((.001, .051)):
        wp = cmap.get_waypoint_xodr(67, 2, s)
        require(wp is not None and (wp.road_id, wp.section_id, wp.lane_id) == PREFIX_LANE,
                "declared upstream sampling unavailable")
        center = wp.transform.location
        diagnostic_prefix.append(dict(road_id=67, section_id=0, lane_id=2, x=center.x, y=-center.y,
                                      z=center.z, distance_m=index * .05))
    # HH_261004 - These map samples authorize a small body-support corridor, never a new model route.
    corridors.update(body.lane_corridors(cmap, carla, diagnostic_prefix, xml, {PREFIX_LANE}, {0, 1}))
    gap = corridors[INITIAL_ROUTE_LANE].distance(corridors[PREFIX_LANE])
    require(gap <= body.SEAM_GAP_CLOSE_M, "connected prefix seam exceeds frozen 5mm tolerance")
    box, radius = body._body_box(manifest, carla)
    proof = dict(schema=SCHEMA, route_sha256=EXPECTED_ROUTE_SHA256, route_canonical_sha256=EXPECTED_ROUTE_CANONICAL_SHA256,
                 active_map_opendrive_sha256=EXPECTED_XODR_SHA256, initial_route_lane=list(INITIAL_ROUTE_LANE),
                 authorized_start_prefix_lane=list(PREFIX_LANE), junction_id=1352, connection_id=7,
                 positive_lane_travel_against_reference_s=True, unique_offline_previous_proven=True,
                 yaw_match_deg=yaw_delta, lane_width_m=endpoint.lane_width, seam_gap_m=gap,
                 seam_close_limit_m=body.SEAM_GAP_CLOSE_M, lane_inset_m=body.LANE_INSET_M,
                 sweep_position_guard_m=body.SWEEP_POSITION_GUARD_M, body_radius_m=radius,
                 prefix_condition="initial body-support lower route arc < 0; sweep uses previous actor arc",
                 route_input_modified=False, global_driving_lane_union_used=False, original_fail_result_modified=False,
                 physical_continuous_time_proven=False, training_data_approved=False, feature_approved=False)
    return points, corridors, box, radius, proof


def topology_provenance(manifest, route, xml, actor_id):
    """HH_261004 - Return authenticated static topology provenance without contacting a simulator."""
    import carla
    return _prepare(manifest, route, xml, actor_id, carla)[-1]


def _summary(rows, field="sampled_body", sweep_field="previous_sweep"):
    result = {}
    for region in ("raw_lane", "inset_10cm_lane"):
        for kind, key in (("sampled_body", field), ("previous_sweep", sweep_field)):
            values = [(r["frame"], r[key][region] if kind == "sampled_body" else r[key]["regions"][region])
                      for r in rows if r[key] is not None]
            violations = [frame for frame, v in values if not v["area_gate_clear"]]
            margins = [(frame, v["minimum_boundary_clearance_m"]) for frame, v in values
                       if v["minimum_boundary_clearance_m"] is not None]
            result[kind + "_" + region] = dict(assessed_count=len(values), violation_count=len(violations),
                violation_frames=violations, maximum_outside_m2=max(v["outside_m2"] for _, v in values),
                minimum_interior_clearance_m=min(v for _, v in margins) if margins else None,
                minimum_clearance_frame=min(margins, key=lambda item: item[1])[0] if margins else None,
                status="PASS" if values and not violations else "FAIL")
    return result


def body_census_with_provenance(manifest, route, states, xml, actor_id):
    """HH_261004 - Keep every native box/sweep and also report the unextended original corridor census."""
    import carla
    points, corridors, box, radius, proof = _prepare(manifest, route, xml, actor_id, carla)
    require(len(states) >= 2, "at least two complete contiguous native states required")
    cache = {}
    def regions(lo, hi):
        original_ids = frozenset(body._route_lanes_for_arc_interval(points, lo, hi))
        ids = original_ids | ({PREFIX_LANE} if lo < 0 else set())
        def cached(key):
            if key not in cache:
                cache[key] = {name: body.authorized_lane_region(corridors, key, inset)
                              for name, inset in (("raw_lane", 0.), ("inset_10cm_lane", body.LANE_INSET_M))}
            return cache[key]
        return cached(frozenset(ids)), cached(original_ids), sorted(ids), sorted(original_ids)
    rows, previous = [], None
    for state in states:
        pose = state["actor_snapshot_transform_carla"]
        arc, segment, distance = body._route_projection((pose["x"], pose["y"]), points)
        footprint = body.body_footprint(box, pose, carla)
        allowed, unextended, ids, old_ids = regions(arc - radius, arc + radius)
        row = dict(frame=state["frame"], timestamp=state["timestamp"], capture_phase=state["capture_phase"],
            research_control_source=state.get("research_control_source"), pose_carla=pose,
            independent_actor_arc_m=arc, independent_actor_route_distance_m=distance,
            intended_lane_ids=[list(i) for i in ids], original_recorded_route_lane_ids=[list(i) for i in old_ids],
            start_connected_prefix_added=arc - radius < 0, body_polygon_carla_xy=list(footprint.exterior.coords),
            sampled_body={name: region_measurement(footprint, r) for name, r in allowed.items()},
            recorded_route_only_body={name: region_measurement(footprint, r) for name, r in unextended.items()},
            previous_sweep=None, recorded_route_only_sweep=None)
        if previous:
            old_state, old_arc, old_segment, old_footprint = previous
            require(state["frame"] == old_state["frame"] + 1
                    and abs(state["timestamp"] - old_state["timestamp"] - .05) <= 1e-6,
                    "native sweep requires contiguous 20 Hz states")
            require(-.25 <= arc - old_arc <= body.MAX_FRAME_PROGRESS_M and 0 <= segment - old_segment <= 3,
                    "route projection jumps or reverses")
            envelope = body.swept_body(old_footprint, footprint, old_state["actor_snapshot_transform_carla"], pose, radius)
            allowed, unextended, ids, old_ids = regions(old_arc - radius, arc + radius)
            row["previous_sweep"] = dict(from_frame=old_state["frame"], to_frame=state["frame"],
                intended_lane_ids=[list(i) for i in ids], original_recorded_route_lane_ids=[list(i) for i in old_ids],
                start_connected_prefix_added=old_arc - radius < 0, envelope_carla_xy=list(envelope.exterior.coords),
                regions={name: region_measurement(envelope, r) for name, r in allowed.items()})
            row["recorded_route_only_sweep"] = dict(regions={name: region_measurement(envelope, r) for name, r in unextended.items()})
        rows.append(row)
        previous = state, arc, segment, footprint
    proof = dict(proof, all_native_boxes=len(rows), all_snapshot_linear_sweeps=len(rows) - 1,
                 recorded_route_only_census=_summary(rows, "recorded_route_only_body", "recorded_route_only_sweep"),
                 prefix_body_frames=[r["frame"] for r in rows if r["start_connected_prefix_added"]],
                 prefix_sweep_frames=[r["frame"] for r in rows if r["previous_sweep"] and r["previous_sweep"]["start_connected_prefix_added"]],
                 prefix_body_phase_counts=dict(Counter(r["capture_phase"] for r in rows if r["start_connected_prefix_added"])))
    # HH_261004 - Persist actual authenticated topology and original four gates through the owner's unchanged two-result ABI.
    rows[0]["authenticated_start_corridor_provenance"] = proof
    return rows, _summary(rows), proof


def body_census(manifest, route, states, xml, actor_id):
    """HH_261004 - Preserve the existing owner kernel's two-result ABI without any module-global overrides."""
    rows, census, _ = body_census_with_provenance(manifest, route, states, xml, actor_id)
    return rows, census
