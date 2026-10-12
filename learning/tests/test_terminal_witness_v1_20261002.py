"""HH_261002 - Synthetic causal endpoint proofs only; no simulator, model, future labels or deployment."""

from copy import deepcopy
from dataclasses import replace
import hashlib
import json
import math
import inspect

import pytest

from portable_e2e import terminal_witness_v1_20261002 as witness


def encoded(document):
    payload=json.dumps(document,sort_keys=True,separators=(",",":"),allow_nan=True).encode()
    return payload,hashlib.sha256(payload).hexdigest()


def fixture(x=190.,stamp=1_000_000_000,sequence=0):
    mission=dict(schema=witness.MISSION_SCHEMA,coordinates=dict(witness.COORDINATES),mission_id="mission-full200",
        map_id="synthetic-map",route_id="declared-route",source_route_artifact_sha256="a"*64,
        source_available_timestamp_ns=0,complete_mission_route=True,polyline_map_xy_m=[[0.,0.],[200.,0.]],
        goal_map_xy_m=[200.,0.],stop_reference_offset_m=.75)
    remaining=200.-x;length=min(remaining,120.)
    distances=[float(i) for i in range(int(math.floor(length))+1)]
    if length-distances[-1]>1.e-6:distances.append(length)
    points=[[s,0.] for s in distances]
    obs=dict(schema=witness.OBSERVATION_SCHEMA,coordinates=dict(witness.COORDINATES),mission_id=mission["mission_id"],
        map_id=mission["map_id"],route_id=mission["route_id"],source_route_artifact_sha256="a"*64,sequence_index=sequence,
        anchor_timestamp_ns=stamp,ego_timestamp_ns=stamp,pose_timestamp_ns=stamp,navigation_timestamp_ns=stamp,
        camera_timestamp_ns=[stamp]*6,camera_source_sha256=[("%x"%i)*64 for i in range(1,7)],
        ego_source_sha256="b"*64,pose_source_sha256="c"*64,position_map_m=[x,0.,.1],orientation_xyzw=[0.,0.,0.,1.],
        provided_route_xy_base_m=points,provided_route_support_m=length,route_anchor_arc_m=x,
        remaining_mission_arc_m=remaining,goal_base_m=[remaining,0.])
    return mission,obs


def evaluate(mission,obs,*,mode=witness.VERIFIED_TERMINAL,now=None,previous=None,**overrides):
    mp,ms=encoded(mission) if mission is not None else (None,None)
    op,os=encoded(obs)
    args=dict(mission_payload=mp,expected_mission_sha256=ms,expected_route_artifact_sha256="a"*64,
        observation_payload=op,expected_observation_sha256=os,now_ns=obs["anchor_timestamp_ns"] if now is None else now,
        requested_mode=mode,previous=previous)
    args.update(overrides)
    return witness.evaluate_terminal_witness(**args)


def test_verified_endpoint_uses_fullmission_currentpose_not_truncated_lastpoint():
    mission,obs=fixture();result=evaluate(mission,obs)
    assert result.status=="VERIFIED" and result.terminal_permission
    actual=result.witness
    assert actual.remaining_mission_arc_m==10. and actual.provided_route_arc_m==10.
    assert actual.goal_base_m==(10.,0.) and actual.signed_remaining_stop_reference_arc_m==9.25
    assert actual.stop_reference_base_m==(9.25,0.) and actual.original_GO18m_contract_unchanged
    assert not actual.vehicle_control_approved and not actual.path_support_is_free_space_or_body_safety


def test_known_long_mission_prefix_is_ordinary_or_terminal_rejected():
    mission,obs=fixture(0.)
    ordinary=evaluate(mission,obs,mode=witness.ORDINARY)
    assert ordinary.status=="VERIFIED" and not ordinary.terminal_permission
    assert ordinary.witness.provided_route_arc_m==120. and ordinary.witness.remaining_mission_arc_m==200.
    assert ordinary.witness.ordinary_causal_decoder_support_m==64. and ordinary.witness.goal_base_m==(200.,0.)
    assert not ordinary.witness.provided_prefix_reaches_mission_endpoint
    terminal=evaluate(mission,obs)
    assert terminal.status=="REJECTED" and terminal.reason_code=="TRUNCATED_NAVIGATION_NOT_ENDPOINT"
    assert terminal.witness is None and not terminal.terminal_permission


def test_unknown_short_prefix_never_relaxes_old_GO18m_or_authenticates_goal():
    _,obs=fixture()
    for key in ("route_anchor_arc_m","remaining_mission_arc_m","goal_base_m"):obs[key]=None
    ordinary=evaluate(None,obs,mode=witness.ORDINARY)
    assert ordinary.status=="VERIFIED" and ordinary.reason_code=="UNKNOWN_MISSION_ORDINARY_ONLY"
    assert ordinary.witness.original_GO18m_contract_unchanged and ordinary.witness.remaining_mission_arc_m is None
    assert ordinary.witness.ordinary_causal_decoder_support_m==10. and not ordinary.terminal_permission
    assert evaluate(None,obs).reason_code=="UNKNOWN_MISSION_TERMINAL_FORBIDDEN"


@pytest.mark.parametrize("x,remaining,signed",[(199.5,.5,-.25),(200.,0.,-.75)])
def test_stop_reference_distance_negative_and_true_endpoint_no_fake_padding(x,remaining,signed):
    mission,obs=fixture(x);result=evaluate(mission,obs)
    assert result.status=="VERIFIED" and result.witness.remaining_mission_arc_m==remaining
    assert result.witness.signed_remaining_stop_reference_arc_m==signed
    assert result.witness.goal_base_m==(remaining,0.)
    assert len(result.witness.provided_route_xy_base_m)==(1 if x==200. else 2)


@pytest.mark.parametrize("changed,code",[
    ("anchor_bool","TIMESTAMP_INVALID"),("sequence_bool","TIMESTAMP_INVALID"),
    ("pose_nan","NONFINITE_OR_BOOLEAN_NUMBER"),("goal_inf","NONFINITE_OR_BOOLEAN_NUMBER"),
    ("support_bool","NONFINITE_OR_BOOLEAN_NUMBER"),("pose_bool","NONFINITE_OR_BOOLEAN_NUMBER"),
    ("quaternion","QUATERNION_INVALID"),("camera_missing","CAMERA_ABI_INVALID"),
    ("camera_time","CAMERA_FRAME_SKEW"),("camera_sha","SOURCE_SHA_INVALID"),
    ("support","PROVIDED_SUPPORT_MISMATCH"),("prefix","NAVIGATION_PREFIX_MISMATCH"),
    ("anchor","MISSION_DISTANCE_MISMATCH"),("remaining","MISSION_DISTANCE_MISMATCH"),
    ("goal","GOAL_BASE_MISMATCH"),("coordinates","COORDINATE_REFERENCE_MISMATCH"),
    ("route_source","MISSION_SOURCE_MISMATCH"),("identity","MISSION_IDENTITY_MISMATCH"),
    ("pose_future","NAVIGATION_POSE_TIME_MISMATCH"),("future_field","DOCUMENT_SCHEMA_INVALID")])
def test_actual_current_observation_errors_fail_closed_without_permissive_fallback(changed,code):
    mission,obs=fixture()
    if changed=="anchor_bool":obs["anchor_timestamp_ns"]=True
    elif changed=="sequence_bool":obs["sequence_index"]=True
    elif changed=="pose_nan":obs["position_map_m"][0]=float("nan")
    elif changed=="goal_inf":obs["goal_base_m"][0]=float("inf")
    elif changed=="support_bool":obs["provided_route_support_m"]=True
    elif changed=="pose_bool":obs["position_map_m"][0]=True
    elif changed=="quaternion":obs["orientation_xyzw"]=[0.,0.,0.,2.]
    elif changed=="camera_missing":obs["camera_timestamp_ns"].pop()
    elif changed=="camera_time":obs["camera_timestamp_ns"][0]-=1
    elif changed=="camera_sha":obs["camera_source_sha256"][0]="bad"
    elif changed=="support":obs["provided_route_support_m"]+=.1
    elif changed=="prefix":obs["provided_route_xy_base_m"][5][1]=.001
    elif changed=="anchor":obs["route_anchor_arc_m"]-=.1
    elif changed=="remaining":obs["remaining_mission_arc_m"]+=.1
    elif changed=="goal":obs["goal_base_m"][0]-=.1
    elif changed=="coordinates":obs["coordinates"]["position_reference"]="actor_center"
    elif changed=="route_source":obs["source_route_artifact_sha256"]="d"*64
    elif changed=="identity":obs["mission_id"]="unknown"
    elif changed=="pose_future":obs["pose_timestamp_ns"]+=1
    else:obs["teacher_future_xy"]=[[0.,0.]]
    now=1_000_000_100 if changed=="pose_future" else 1_000_000_000
    actual=evaluate(mission,obs,now=now)
    # HH_261002 - A changed interior point may fail the exact arc check before the prefix-coordinate check.
    assert actual.status=="REJECTED" and actual.reason_code in ({code,"PROVIDED_SUPPORT_MISMATCH"} if changed=="prefix" else {code})
    assert actual.witness is None and not actual.terminal_permission


@pytest.mark.parametrize("change,code",[("incomplete","MISSION_COMPLETENESS_NOT_DECLARED"),
    ("bool_complete","MISSION_COMPLETENESS_NOT_DECLARED"),("endpoint","MISSION_ENDPOINT_MISMATCH"),
    ("available_future","MISSION_NOT_CAUSALLY_AVAILABLE"),("offset_bool","NONFINITE_OR_BOOLEAN_NUMBER"),
    ("offset_negative","STOP_REFERENCE_INVALID"),("duplicate","DUPLICATE_ROUTE_POINT"),
    ("source","MISSION_SOURCE_MISMATCH"),("future_field","DOCUMENT_SCHEMA_INVALID")])
def test_fullmission_assertion_and_original_source_binding_not_guessable_from_prefix(change,code):
    mission,obs=fixture()
    if change=="incomplete":mission["complete_mission_route"]=False
    elif change=="bool_complete":mission["complete_mission_route"]=1
    elif change=="endpoint":mission["goal_map_xy_m"][0]-=1
    elif change=="available_future":mission["source_available_timestamp_ns"]=1_000_000_001
    elif change=="offset_bool":mission["stop_reference_offset_m"]=True
    elif change=="offset_negative":mission["stop_reference_offset_m"]=-.1
    elif change=="duplicate":mission["polyline_map_xy_m"].insert(1,[0.,0.])
    elif change=="source":mission["source_route_artifact_sha256"]="d"*64
    else:mission["actual_stop_future_arc_m"]=10.
    result=evaluate(mission,obs)
    assert result.status=="REJECTED" and result.reason_code==code


@pytest.mark.parametrize("kind,code",[("stale","STALE_OR_FUTURE_INPUT"),("future","STALE_OR_FUTURE_INPUT"),
    ("skew","CURRENT_INPUT_SKEW"),("navpose","NAVIGATION_POSE_TIME_MISMATCH")])
def test_source_clocks_are_current_causal_and_aligned(kind,code):
    mission,obs=fixture();now=1_000_000_000
    if kind=="stale":now+=150_000_001
    elif kind=="future":now-=1
    elif kind=="skew":obs["ego_timestamp_ns"]-=50_000_001
    else:obs["navigation_timestamp_ns"]-=1
    assert evaluate(mission,obs,now=now).reason_code==code


def test_monotonic_observed_stream_and_mission_progress_preserved_not_clipped():
    mission,obs=fixture();first=evaluate(mission,obs).witness
    _,current=fixture(190.2,1_100_000_000,1)
    result=evaluate(mission,current,previous=first)
    assert result.status=="VERIFIED" and math.isclose(result.witness.route_projection_arc_m,190.2)
    _,backwards=fixture(189.9,1_100_000_000,1)
    assert evaluate(mission,backwards,previous=first).reason_code=="NONMONOTONIC_MISSION_PROGRESS"
    _,duplicate=fixture(190.2,1_000_000_000,1)
    assert evaluate(mission,duplicate,previous=first).reason_code=="NONMONOTONIC_OBSERVATION"
    assert evaluate(mission,current).reason_code=="SEQUENCE_UNPROVEN"
    changed=deepcopy(mission);changed["stop_reference_offset_m"]=.5
    assert evaluate(changed,current,previous=first).reason_code=="SOURCE_EPOCH_CHANGED"


@pytest.mark.parametrize("points,position",[
    ([[0.,0.],[10.,0.],[10.,10.],[0.,10.],[0.,0.],[20.,0.]],[5.,0.,0.]),
    ([[0.,0.],[10.,0.],[10.,10.],[0.,10.],[0.,0.]],[0.,0.,0.]),
    ([[0.,0.],[10.,0.],[10.,.03],[0.,.03],[20.,.03]],[5.,.01,0.])])
def test_revisited_endpoint_crossing_and_nearby_loop_arcs_reject_nearestpoint_authentication(points,position):
    mission,obs=fixture();mission["polyline_map_xy_m"]=points;mission["goal_map_xy_m"]=points[-1]
    obs["position_map_m"]=position
    assert evaluate(mission,obs).reason_code=="AMBIGUOUS_MISSION_PROJECTION"


@pytest.mark.parametrize("override",[{"expected_observation_sha256":"d"*64},{"expected_mission_sha256":"d"*64},
    {"expected_route_artifact_sha256":True},{"mission_payload":bytearray(b"{}")},{"now_ns":True},{"requested_mode":True}])
def test_immutable_source_bytes_exact_external_sha_and_boolean_boundaries(override):
    mission,obs=fixture();result=evaluate(mission,obs,**override)
    assert result.status=="REJECTED" and not result.terminal_permission


def test_no_teacher_future_keyword_or_model_runtime_dispatch_and_scope_is_prepared():
    mission,obs=fixture()
    with pytest.raises(TypeError):evaluate(mission,obs,teacher_future_xy=((1.,2.),))
    contract=witness.prepared_contract()
    assert contract["prepared_not_deployed"] and contract["terminal_dispatch_not_implemented"]
    assert not contract["existing_neural_inputs_or_weights_changed"] and not contract["new_learned_heads"]
    assert contract["main_functions_approved"]==contract["subfunctions_approved"]==0
    source=inspect.getsource(witness)
    assert "import torch" not in source and "from .runtime" not in source and "from .path_time_decoder" not in source


def test_curved_mission_rotated_actual_pose_and_rear_cross_track_offset_preserved():
    mission,obs=fixture()
    mission["polyline_map_xy_m"]=[[0.,0.],[10.,0.],[10.,20.]];mission["goal_map_xy_m"]=[10.,20.]
    obs.update(position_map_m=[10.2,5.,.1],orientation_xyzw=[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)],
        route_anchor_arc_m=15.,remaining_mission_arc_m=15.,goal_base_m=[15.,.2],
        provided_route_xy_base_m=[[float(i),.2] for i in range(16)],provided_route_support_m=15.)
    result=evaluate(mission,obs)
    assert result.status=="VERIFIED" and result.terminal_permission
    assert result.witness.position_map_m==(10.2,5.,.1) and result.witness.orientation_xyzw==tuple(obs["orientation_xyzw"])
    assert math.dist(result.witness.goal_base_m,(15.,.2))<1.e-6
    assert result.witness.remaining_mission_arc_m==15. and result.witness.provided_route_arc_m==15.
    assert math.dist(result.witness.stop_reference_base_m,(14.25,.2))<1.e-6


@pytest.mark.parametrize("change",["negative_support","negative_anchor","negative_remaining","huge_integer"])
def test_small_negative_distance_is_not_accepted_by_tolerance_or_overflow(change):
    mission,obs=fixture(200.)
    if change=="negative_support":obs["provided_route_support_m"]=-1.e-7
    elif change=="negative_anchor":obs["route_anchor_arc_m"]=-1.e-7
    elif change=="negative_remaining":obs["remaining_mission_arc_m"]=-1.e-7
    else:obs["position_map_m"][0]=10**1000
    result=evaluate(mission,obs)
    assert result.status=="REJECTED" and not result.terminal_permission


def test_duplicate_json_keys_cannot_replace_an_authenticated_field_silently():
    mission,obs=fixture();payload,sha=encoded(mission);op,_=encoded(obs)
    malicious=op[:-1]+b',"sequence_index":0}'
    result=witness.evaluate_terminal_witness(mission_payload=payload,expected_mission_sha256=sha,
        expected_route_artifact_sha256="a"*64,observation_payload=malicious,
        expected_observation_sha256=hashlib.sha256(malicious).hexdigest(),now_ns=1_000_000_000,requested_mode=witness.VERIFIED_TERMINAL)
    assert result.reason_code=="DUPLICATE_JSON_KEY" and result.witness is None


def test_unknown_mission_cannot_claim_a_goal_or_actual_remaining_even_in_ordinary_mode():
    _,obs=fixture()
    result=evaluate(None,obs,mode=witness.ORDINARY)
    assert result.reason_code=="UNKNOWN_MISSION_CLAIM" and not result.terminal_permission


@pytest.mark.parametrize("x",[10.001,10.025,10.05,10.075,10.10,10.125,10.15,10.199])
@pytest.mark.parametrize("cte",[-.2,-.04,.04,.2])
def test_dense0p1_connected_straight_clamped_vertices_are_not_false_branches(x,cte):
    mission,obs=fixture(x)
    mission["polyline_map_xy_m"]=[[i/10.,0.] for i in range(2001)]
    obs["position_map_m"][1]=cte
    obs["provided_route_xy_base_m"]=[[s,-cte] for s,_ in obs["provided_route_xy_base_m"]]
    obs["goal_base_m"][1]=-cte
    result=evaluate(mission,obs,mode=witness.ORDINARY)
    assert result.status=="VERIFIED" and not result.terminal_permission
    assert abs(result.witness.route_projection_arc_m-x)<1.e-6
    assert abs(result.witness.provided_route_arc_m-120.)<1.e-6


def test_normal_connected_corner_vertex_is_one_minimum_not_two_branches():
    mission,obs=fixture()
    mission["polyline_map_xy_m"]=[[0.,0.],[10.,0.],[10.,10.]];mission["goal_map_xy_m"]=[10.,10.]
    obs.update(position_map_m=[10.2,-.2,.1],orientation_xyzw=[0.,0.,0.,1.],route_anchor_arc_m=10.,
        remaining_mission_arc_m=10.,goal_base_m=[-.2,10.2],provided_route_support_m=10.,
        provided_route_xy_base_m=[[-.2,.2+float(i)] for i in range(11)])
    result=evaluate(mission,obs)
    assert result.status=="VERIFIED" and result.terminal_permission
    assert result.witness.route_projection_arc_m==10.
    points=((0.,0.),(10.,0.),(10.,10.))
    assert len(witness._distance_local_minima(points,witness._cumulative(points),obs["position_map_m"]))==1


def test_dense_terminal_navigation_matches_sparse_and_exact_old_canonical_projection():
    from portable_e2e.contract import _canonical_route_in_base
    mission,obs=fixture(190.125)
    points=tuple((i/10.,0.) for i in range(2001));mission["polyline_map_xy_m"]=points
    obs["position_map_m"][1]=.038
    route,goal,anchor=_canonical_route_in_base(points,position_m=obs["position_map_m"],orientation_xyzw=obs["orientation_xyzw"],
        contract=dict(navigation=dict(maximum_anchor_distance_m=5.,route_resample_spacing_m=1.,route_lookahead_m=120.)),context="synthetic")
    obs.update(provided_route_xy_base_m=route,provided_route_support_m=sum(math.dist(a,b) for a,b in zip(route,route[1:])),
        goal_base_m=goal,route_anchor_arc_m=anchor)
    result=evaluate(mission,obs)
    assert result.status=="VERIFIED" and result.witness.route_projection_arc_m==anchor
    assert result.witness.provided_route_xy_base_m==route


def test_zero_remaining_new_abi_does_not_change_legacy_Common10_reader():
    from portable_e2e.contract import _canonical_route_in_base,ContractError
    mission,obs=fixture(200.)
    assert evaluate(mission,obs).status=="VERIFIED"
    with pytest.raises(ContractError,match="no route remaining"):
        _canonical_route_in_base(((0.,0.),(200.,0.)),position_m=obs["position_map_m"],orientation_xyzw=obs["orientation_xyzw"],
            contract=dict(navigation=dict(maximum_anchor_distance_m=5.,route_resample_spacing_m=1.,route_lookahead_m=120.)),context="synthetic")
    assert witness.prepared_contract()["zero_remaining_one_point_is_new_terminal_ABI_not_legacy_Common10_acceptance"]


@pytest.mark.parametrize("field,value",[("route_projection_arc_m",True),("route_projection_arc_m",-.1),
    ("route_projection_arc_m",float("nan")),("route_projection_arc_m",float("inf")),("route_projection_arc_m",None),
    ("remaining_mission_arc_m",-.1),("remaining_mission_arc_m",True),("observation_sha256",True)])
def test_malformed_public_previous_witness_cannot_bypass_progress_or_source_checks(field,value):
    mission,obs=fixture();previous=evaluate(mission,obs).witness
    _,current=fixture(190.2,1_100_000_000,1)
    malformed=replace(previous,**{field:value})
    assert evaluate(mission,current,previous=malformed).status=="REJECTED"
