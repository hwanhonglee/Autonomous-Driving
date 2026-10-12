"""HH_261002 - Authenticate causal mission endpoints without changing any model or decoder dispatch.

This prepared, standard-library-only ABI distinguishes a supplied navigation
prefix from an externally declared COMPLETE mission.  Completeness is a trusted
declaration assertion, not proof inferred from geometry.  Route-artifact SHA is
an externally pinned provenance assertion: this module parses the new mission
declaration, not arbitrary original simulator artifacts.  No future labels,
teacher controls, learned intentions, or speed overrides are accepted.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math

from .contract import ContractError


SCHEMA = "portable_e2e.causal_terminal_witness.20261002.v1"
MISSION_SCHEMA = "portable_e2e.complete_mission_declaration.20261002.v1"
OBSERVATION_SCHEMA = "portable_e2e.current_terminal_navigation_observation.20261002.v1"
ORDINARY, VERIFIED_TERMINAL = "ORDINARY", "VERIFIED_TERMINAL"
COORDINATES = dict(map_frame="map",base_frame="base_link",base_axes="x_forward_y_left_z_up",
                   position_reference="rear_axle_base_link")
PREFIX_LIMIT_M, PREFIX_SPACING_M, DECODER_SUPPORT_LIMIT_M = 120., 1., 64.
GEOMETRY_TOLERANCE_M, PROGRESS_TOLERANCE_M, PROJECTION_AMBIGUITY_M = 1.e-6, 1.e-6, .05
MAXIMUM_INPUT_AGE_NS, MAXIMUM_INPUT_SKEW_NS = 150_000_000, 50_000_000


class TerminalWitnessError(ContractError):
    """HH_261002 - Keep a stable failure code distinct from explanatory text."""
    def __init__(self,code,message):
        super().__init__(message);self.code=code


def _require(condition,code,message):
    if not condition:raise TerminalWitnessError(code,message)


def _sha(value,name):
    _require(type(value) is str and len(value)==64 and all(c in "0123456789abcdef" for c in value),
             "SOURCE_SHA_INVALID",name+" must be an exact lowercase SHA256")
    return value


def _number(value,name):
    _require(type(value) in (int,float),"NONFINITE_OR_BOOLEAN_NUMBER",name+" must be a finite nonboolean number")
    try:parsed=float(value)
    except (ValueError,OverflowError) as error:
        raise TerminalWitnessError("NONFINITE_OR_BOOLEAN_NUMBER",name+" cannot be represented finitely") from error
    _require(math.isfinite(parsed),"NONFINITE_OR_BOOLEAN_NUMBER",name+" must be a finite nonboolean number")
    return parsed


def _stamp(value,name):
    _require(type(value) is int and value>=0,"TIMESTAMP_INVALID",name+" must be a nonnegative integer nanosecond stamp")
    return value


def _text(value,name):
    _require(type(value) is str and value and value.strip()==value,"IDENTITY_INVALID",name+" must be a nonempty exact identity")
    return value


def _vector(value,size,name):
    _require(type(value) in (list,tuple) and len(value)==size,"VECTOR_ABI_INVALID",name+" has the wrong shape")
    return tuple(_number(v,name) for v in value)


def _points(value,maximum,name):
    _require(type(value) in (list,tuple) and 1<=len(value)<=maximum,"POLYLINE_ABI_INVALID",name+" point count is invalid")
    points=tuple(_vector(v,2,name) for v in value)
    _require(all(math.dist(a,b)>1.e-9 for a,b in zip(points,points[1:])),"DUPLICATE_ROUTE_POINT",name+" contains a zero-length segment")
    return points


def _document(payload,expected,limit,schema,keys):
    _sha(expected,"expected document SHA")
    _require(type(payload) is bytes and 0<len(payload)<=limit,"DOCUMENT_ABI_INVALID","bounded immutable JSON bytes required")
    _require(hashlib.sha256(payload).hexdigest()==expected,"SOURCE_SHA_MISMATCH","document differs from the external caller pin")
    def unique(pairs):
        result={}
        for key,value in pairs:
            _require(key not in result,"DUPLICATE_JSON_KEY","duplicate JSON key: "+key);result[key]=value
        return result
    try:document=json.loads(payload,object_pairs_hook=unique)
    except TerminalWitnessError:raise
    except (ValueError,UnicodeError) as error:raise TerminalWitnessError("JSON_INVALID",str(error)) from error
    _require(type(document) is dict and set(document)==set(keys) and document.get("schema")==schema,
             "DOCUMENT_SCHEMA_INVALID","exact new schema/keys required; extra future or teacher fields are forbidden")
    _require(document["coordinates"]==COORDINATES,"COORDINATE_REFERENCE_MISMATCH","actual rear-base map coordinate convention required")
    return document


MISSION_KEYS = ("schema","coordinates","mission_id","map_id","route_id","source_route_artifact_sha256",
    "source_available_timestamp_ns","complete_mission_route","polyline_map_xy_m","goal_map_xy_m","stop_reference_offset_m")
OBSERVATION_KEYS = ("schema","coordinates","mission_id","map_id","route_id","source_route_artifact_sha256","sequence_index",
    "anchor_timestamp_ns","ego_timestamp_ns","pose_timestamp_ns","navigation_timestamp_ns","camera_timestamp_ns",
    "camera_source_sha256","ego_source_sha256","pose_source_sha256","position_map_m","orientation_xyzw",
    "provided_route_xy_base_m","provided_route_support_m","route_anchor_arc_m","remaining_mission_arc_m","goal_base_m")


@dataclass(frozen=True)
class TerminalWitness:
    """HH_261002 - Derived geometry/provenance only; a verified endpoint is not permission to actuate or stop."""
    schema: str
    mode: str
    reason_code: str
    mission_id: str
    map_id: str
    route_id: str
    mission_declaration_sha256: str | None
    route_artifact_sha256: str
    observation_sha256: str
    sequence_index: int
    anchor_timestamp_ns: int
    ego_timestamp_ns: int
    pose_timestamp_ns: int
    navigation_timestamp_ns: int
    camera_timestamp_ns: tuple[int,...]
    camera_source_sha256: tuple[str,...]
    ego_source_sha256: str
    pose_source_sha256: str
    position_map_m: tuple[float,float,float]
    orientation_xyzw: tuple[float,float,float,float]
    provided_route_xy_base_m: tuple[tuple[float,float],...]
    provided_route_arc_m: float
    ordinary_causal_decoder_support_m: float
    route_projection_arc_m: float | None
    remaining_mission_arc_m: float | None
    goal_base_m: tuple[float,float] | None
    stop_reference_base_m: tuple[float,float] | None
    signed_remaining_stop_reference_arc_m: float | None
    provided_prefix_reaches_mission_endpoint: bool
    terminal_permission: bool
    original_GO18m_contract_unchanged: bool = True
    path_support_is_free_space_or_body_safety: bool = False
    vehicle_control_approved: bool = False
    prepared_not_model_performance: bool = True


@dataclass(frozen=True)
class TerminalWitnessDecision:
    status: str
    reason_code: str
    reason: str
    witness: TerminalWitness | None

    @property
    def terminal_permission(self):
        return self.status=="VERIFIED" and self.witness is not None and self.witness.terminal_permission


def _cumulative(points):
    result=[0.]
    for a,b in zip(points,points[1:]):result.append(result[-1]+math.dist(a,b))
    return result


def _sample(points,arcs,distance):
    for i,(start,end) in enumerate(zip(arcs,arcs[1:])):
        if distance<=end:
            ratio=(distance-start)/(end-start);a,b=points[i],points[i+1]
            return (a[0]+ratio*(b[0]-a[0]),a[1]+ratio*(b[1]-a[1]))
    return points[-1]


def _distance_local_minima(points,arcs,position):
    """HH_261002 - Clamped endpoints on a continuing segment are not independent topology hypotheses."""
    minima=[]
    for i,(a,b) in enumerate(zip(points,points[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1];squared=dx*dx+dy*dy
        ratio=((position[0]-a[0])*dx+(position[1]-a[1])*dy)/squared
        if 0.<ratio<1.:
            point=(a[0]+ratio*dx,a[1]+ratio*dy)
            minima.append((arcs[i]+ratio*math.sqrt(squared),math.dist(position[:2],point)))
    for i,point in enumerate(points):
        rx,ry=point[0]-position[0],point[1]-position[1]
        incoming=None if i==0 else (rx*(point[0]-points[i-1][0])+ry*(point[1]-points[i-1][1]))/(arcs[i]-arcs[i-1])
        outgoing=None if i==len(points)-1 else (rx*(points[i+1][0]-point[0])+ry*(points[i+1][1]-point[1]))/(arcs[i+1]-arcs[i])
        # HH_261002 - A connected vertex is a minimum only when distance decreases into it and increases out of it.
        if (incoming is None or incoming<=0.) and (outgoing is None or outgoing>=0.):
            minima.append((arcs[i],math.dist(position[:2],point)))
    unique=[]
    for candidate in sorted(minima):
        if unique and abs(candidate[0]-unique[-1][0])<=GEOMETRY_TOLERANCE_M:
            unique[-1]=min(unique[-1],candidate,key=lambda value:value[1])
        else:unique.append(candidate)
    _require(bool(unique),"MISSION_PROJECTION_UNDEFINED","connected mission has no finite distance minimum")
    return unique


def _project(points,arcs,position,forward):
    candidates=[]
    for i,(a,b) in enumerate(zip(points,points[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        ratio=max(0.,min(1.,((position[0]-a[0])*dx+(position[1]-a[1])*dy)/(length*length)))
        point=(a[0]+ratio*dx,a[1]+ratio*dy)
        squared=math.dist(position[:2],point)**2;alignment=(forward[0]*dx+forward[1]*dy)/length
        candidates.append((round(squared,12),-alignment,i,arcs[i]+ratio*length,math.sqrt(squared)))
    best=min(candidates)
    # HH_261002 - Heading tie-breaking does not authenticate which occurrence of a looping path the vehicle occupies.
    minima=_distance_local_minima(points,arcs,position)
    _require(not any(abs(distance-best[4])<=PROJECTION_AMBIGUITY_M and abs(arc-best[3])>GEOMETRY_TOLERANCE_M
        for arc,distance in minima),"AMBIGUOUS_MISSION_PROJECTION","nearby genuine local distance minima have distinct mission arcs")
    _require(best[4]<=5.,"POSE_OUTSIDE_MISSION_ROUTE","current rear-base pose is outside original5m navigation bound")
    return best[3]


def _evaluate(*,mission_payload,expected_mission_sha256,expected_route_artifact_sha256,
              observation_payload,expected_observation_sha256,now_ns,requested_mode,previous):
    _require(requested_mode in (ORDINARY,VERIFIED_TERMINAL) and type(requested_mode) is str,"MODE_INVALID","explicit ordinary or verified-terminal mode required")
    now=_stamp(now_ns,"now");artifact=_sha(expected_route_artifact_sha256,"expected original route artifact SHA")
    obs=_document(observation_payload,expected_observation_sha256,1024**2,OBSERVATION_SCHEMA,OBSERVATION_KEYS)
    identities={name:_text(obs[name],name) for name in ("mission_id","map_id","route_id")}
    _require(_sha(obs["source_route_artifact_sha256"],"observed route artifact SHA")==artifact,"MISSION_SOURCE_MISMATCH","navigation artifact pin differs")
    sequence=_stamp(obs["sequence_index"],"sequence")
    stamps={name:_stamp(obs[name],name) for name in ("anchor_timestamp_ns","ego_timestamp_ns","pose_timestamp_ns","navigation_timestamp_ns")}
    _require(type(obs["camera_timestamp_ns"]) is list and len(obs["camera_timestamp_ns"])==6,"CAMERA_ABI_INVALID","six actual camera source stamps required")
    cameras=tuple(_stamp(v,"camera stamp") for v in obs["camera_timestamp_ns"])
    _require(all(v==stamps["anchor_timestamp_ns"] for v in cameras),"CAMERA_FRAME_SKEW","six cameras must share this exact current anchor")
    _require(type(obs["camera_source_sha256"]) is list and len(obs["camera_source_sha256"])==6,"CAMERA_ABI_INVALID","six actual camera source hashes required")
    camera_sha=tuple(_sha(v,"camera source SHA") for v in obs["camera_source_sha256"])
    ego_sha=_sha(obs["ego_source_sha256"],"ego source SHA");pose_sha=_sha(obs["pose_source_sha256"],"pose source SHA")
    all_stamps=tuple(stamps.values())+cameras
    _require(all(0<=now-s<=MAXIMUM_INPUT_AGE_NS for s in all_stamps),"STALE_OR_FUTURE_INPUT","current source is stale or future-dated")
    _require(max(all_stamps)-min(all_stamps)<=MAXIMUM_INPUT_SKEW_NS,"CURRENT_INPUT_SKEW","pose/ego/camera/navigation clocks disagree")
    _require(stamps["pose_timestamp_ns"]==stamps["navigation_timestamp_ns"]<=stamps["anchor_timestamp_ns"]
        and stamps["ego_timestamp_ns"]<=stamps["anchor_timestamp_ns"],"NAVIGATION_POSE_TIME_MISMATCH","navigation must derive from this causal measured pose")
    position=_vector(obs["position_map_m"],3,"position");quaternion=_vector(obs["orientation_xyzw"],4,"quaternion")
    _require(abs(sum(v*v for v in quaternion)-1.)<=1.e-6,"QUATERNION_INVALID","actual unit quaternion required; no silent renormalization")
    qx,qy,qz,qw=quaternion;fx,fy=1.-2.*(qy*qy+qz*qz),2.*(qx*qy+qw*qz);norm=math.hypot(fx,fy)
    _require(norm>=1.e-6,"HEADING_UNDEFINED","actual pose has no map-plane forward heading")
    forward=(fx/norm,fy/norm)
    def to_base(point):
        dx,dy=point[0]-position[0],point[1]-position[1]
        return (forward[0]*dx+forward[1]*dy,-forward[1]*dx+forward[0]*dy)
    provided=_points(obs["provided_route_xy_base_m"],128,"provided navigation")
    supplied_arc=_cumulative(provided)[-1]
    claimed_support=_number(obs["provided_route_support_m"],"provided support")
    _require(claimed_support>=0 and abs(claimed_support-supplied_arc)<=GEOMETRY_TOLERANCE_M,
             "PROVIDED_SUPPORT_MISMATCH","provided support must equal the real supplied polyline arc")
    _require(supplied_arc<=PREFIX_LIMIT_M+GEOMETRY_TOLERANCE_M,"PREFIX_HORIZON_EXCEEDED","current canonical navigation is bounded120m")
    if previous is None:
        _require(sequence==0,"SEQUENCE_UNPROVEN","initial observation must declare sequence0")
    else:
        _require(type(previous) is TerminalWitness and previous.schema==SCHEMA and previous.mode in (ORDINARY,VERIFIED_TERMINAL),
                 "PREVIOUS_WITNESS_INVALID","same ABI previous witness required")
        _stamp(previous.sequence_index,"previous sequence")
        for name in stamps:_stamp(getattr(previous,name),"previous "+name)
        _sha(previous.observation_sha256,"previous observation SHA")
        _require(previous.original_GO18m_contract_unchanged is True and previous.vehicle_control_approved is False
            and previous.terminal_permission is (previous.mode==VERIFIED_TERMINAL),
            "PREVIOUS_WITNESS_INVALID","previous witness cannot alter the prepared-only permission contract")
        if expected_mission_sha256 is None:
            _require(previous.mode==ORDINARY and previous.route_projection_arc_m is None and previous.remaining_mission_arc_m is None,
                     "PREVIOUS_WITNESS_INVALID","unknown prior mission must remain ordinary with no claimed mission arc")
        else:
            old_arc=_number(previous.route_projection_arc_m,"previous projection arc")
            old_remaining=_number(previous.remaining_mission_arc_m,"previous remaining arc")
            _require(0<=old_arc<=100_000. and 0<=old_remaining<=100_000.,
                     "PREVIOUS_WITNESS_INVALID","previous mission distances must remain finite nonnegative actual values")
        _require((previous.mission_id,previous.map_id,previous.route_id,previous.mission_declaration_sha256,previous.route_artifact_sha256)
            ==(identities["mission_id"],identities["map_id"],identities["route_id"],expected_mission_sha256,artifact),
            "SOURCE_EPOCH_CHANGED","mission/source change requires a new externally declared witness stream")
        _require(sequence==previous.sequence_index+1 and all(stamps[n]>getattr(previous,n) for n in stamps),
                 "NONMONOTONIC_OBSERVATION","observation sequence and each current source clock must strictly advance")
    anchor=remaining=goal=stop_base=signed_stop=None;reaches=False
    if mission_payload is None:
        _require(expected_mission_sha256 is None and requested_mode==ORDINARY,"UNKNOWN_MISSION_TERMINAL_FORBIDDEN","unknown mission cannot authorize terminal mode")
        _require(all(obs[name] is None for name in ("route_anchor_arc_m","remaining_mission_arc_m","goal_base_m")),
                 "UNKNOWN_MISSION_CLAIM","unknown prefix must not claim a mission projection, goal or remaining distance")
        code="UNKNOWN_MISSION_ORDINARY_ONLY"
    else:
        mission=_document(mission_payload,expected_mission_sha256,8*1024**2,MISSION_SCHEMA,MISSION_KEYS)
        _require(mission["complete_mission_route"] is True,"MISSION_COMPLETENESS_NOT_DECLARED","only externally authenticated full-mission assertion is accepted")
        _require(all(_text(mission[n],n)==identities[n] for n in identities),"MISSION_IDENTITY_MISMATCH","mission/map/route identities differ")
        _require(_sha(mission["source_route_artifact_sha256"],"mission original artifact SHA")==artifact,"MISSION_SOURCE_MISMATCH","full mission and navigation reference different original artifacts")
        _require(_stamp(mission["source_available_timestamp_ns"],"mission availability")<=min(all_stamps),"MISSION_NOT_CAUSALLY_AVAILABLE","full mission source was not available at the current observations")
        points=_points(mission["polyline_map_xy_m"],100_000,"full mission")
        _require(len(points)>=2,"MISSION_GEOMETRY_INVALID","full mission requires a nonzero route")
        arcs=_cumulative(points);total=arcs[-1]
        _require(total<=100_000.,"MISSION_GEOMETRY_INVALID","original100km mission bound exceeded")
        endpoint=_vector(mission["goal_map_xy_m"],2,"mission goal")
        _require(math.dist(endpoint,points[-1])<=GEOMETRY_TOLERANCE_M,"MISSION_ENDPOINT_MISMATCH","declared goal must match full mission endpoint")
        offset=_number(mission["stop_reference_offset_m"],"stop reference offset")
        _require(0<=offset<total,"STOP_REFERENCE_INVALID","declared stop reference must lie on the full mission")
        anchor=_project(points,arcs,position,forward);remaining=total-anchor
        if previous is not None and previous.route_projection_arc_m is not None:
            _require(anchor+PROGRESS_TOLERANCE_M>=previous.route_projection_arc_m,"NONMONOTONIC_MISSION_PROGRESS","measured projection regressed; it is not clipped")
        claimed_anchor=_number(obs["route_anchor_arc_m"],"claimed anchor")
        claimed_remaining=_number(obs["remaining_mission_arc_m"],"claimed remaining")
        _require(claimed_anchor>=0 and claimed_remaining>=0 and abs(claimed_anchor-anchor)<=GEOMETRY_TOLERANCE_M
            and abs(claimed_remaining-remaining)<=GEOMETRY_TOLERANCE_M,
            "MISSION_DISTANCE_MISMATCH","claimed distances differ from full-mission/current-pose projection")
        goal=to_base(endpoint)
        _require(math.dist(_vector(obs["goal_base_m"],2,"observed goal"),goal)<=GEOMETRY_TOLERANCE_M,
                 "GOAL_BASE_MISMATCH","goal base must derive from the full mission endpoint and actual pose")
        length=min(remaining,PREFIX_LIMIT_M);requested=[anchor+i*PREFIX_SPACING_M for i in range(int(math.floor(length))+1)]
        if anchor+length-requested[-1]>GEOMETRY_TOLERANCE_M:requested.append(anchor+length)
        canonical=tuple(to_base(_sample(points,arcs,s)) for s in requested)
        _require(len(provided)==len(canonical) and all(math.dist(a,b)<=GEOMETRY_TOLERANCE_M for a,b in zip(provided,canonical)),
                 "NAVIGATION_PREFIX_MISMATCH","supplied navigation is not the canonical causal120m full-mission prefix")
        reaches=remaining<=PREFIX_LIMIT_M and math.dist(provided[-1],goal)<=GEOMETRY_TOLERANCE_M
        _require(requested_mode!=VERIFIED_TERMINAL or reaches,"TRUNCATED_NAVIGATION_NOT_ENDPOINT","a cut prefix cannot authorize terminal mode")
        signed_stop=total-offset-anchor;stop_base=to_base(_sample(points,arcs,total-offset))
        code="VERIFIED_TERMINAL_ENDPOINT" if requested_mode==VERIFIED_TERMINAL else "VERIFIED_MISSION_ORDINARY_ONLY"
    witness=TerminalWitness(SCHEMA,requested_mode,code,**identities,mission_declaration_sha256=expected_mission_sha256,
        route_artifact_sha256=artifact,observation_sha256=expected_observation_sha256,sequence_index=sequence,**stamps,
        camera_timestamp_ns=cameras,camera_source_sha256=camera_sha,ego_source_sha256=ego_sha,pose_source_sha256=pose_sha,
        position_map_m=position,orientation_xyzw=quaternion,provided_route_xy_base_m=provided,provided_route_arc_m=supplied_arc,
        ordinary_causal_decoder_support_m=min(supplied_arc,DECODER_SUPPORT_LIMIT_M),route_projection_arc_m=anchor,
        remaining_mission_arc_m=remaining,goal_base_m=goal,stop_reference_base_m=stop_base,
        signed_remaining_stop_reference_arc_m=signed_stop,provided_prefix_reaches_mission_endpoint=reaches,
        terminal_permission=requested_mode==VERIFIED_TERMINAL)
    return TerminalWitnessDecision("VERIFIED",code,"Prepared causal geometry witness only; no decoder dispatch or actuation approval.",witness)


def evaluate_terminal_witness(*,mission_payload:bytes|None,expected_mission_sha256:str|None,expected_route_artifact_sha256:str,
    observation_payload:bytes,expected_observation_sha256:str,now_ns:int,requested_mode:str,previous:TerminalWitness|None=None):
    """HH_261002 - Return explicit rejection, never downgrade a failed terminal request into permissive ordinary mode."""
    try:
        return _evaluate(mission_payload=mission_payload,expected_mission_sha256=expected_mission_sha256,
            expected_route_artifact_sha256=expected_route_artifact_sha256,observation_payload=observation_payload,
            expected_observation_sha256=expected_observation_sha256,now_ns=now_ns,requested_mode=requested_mode,previous=previous)
    except TerminalWitnessError as error:
        return TerminalWitnessDecision("REJECTED",error.code,str(error),None)


def prepared_contract():
    """HH_261002 - Explicitly describe scope without changing the current GO-only101-tensor runtime."""
    return dict(schema=SCHEMA,mission_schema=MISSION_SCHEMA,observation_schema=OBSERVATION_SCHEMA,
        coordinate_contract=dict(COORDINATES),prepared_not_deployed=True,existing_neural_inputs_or_weights_changed=False,
        new_learned_heads=False,original_GO18m_contract_unchanged=True,ordinary_dispatch_requires_frozen_decoder=True,
        terminal_dispatch_not_implemented=True,intent_relabelled=False,teacher_future_arguments_accepted=False,
        externally_pinned_complete_mission_assertion_not_inferred_from_prefix=True,
        original_route_artifact_bytes_not_parsed_or_self_authenticated_here=True,
        navigation_prefix_limit_m=PREFIX_LIMIT_M,ordinary_decoder_support_limit_m=DECODER_SUPPORT_LIMIT_M,
        canonical_prefix_coordinate_tolerance_m=GEOMETRY_TOLERANCE_M,projection_topology_ambiguity_m=PROJECTION_AMBIGUITY_M,
        projection_hypotheses="genuine connected-polyline local distance minima; same-arc witnesses deduplicated",
        projection_arc_deduplication_tolerance_m=GEOMETRY_TOLERANCE_M,
        signed_stop_reference_distance_can_be_negative=True,support_is_not_free_space_or_body_safety=True,
        zero_remaining_terminal_may_have_one_real_route_point_not_synthetic_padding=True,
        zero_remaining_one_point_is_new_terminal_ABI_not_legacy_Common10_acceptance=True,
        legacy_Common10_reader_still_rejects_no_remaining_route=True,
        vehicle_control_approved=False,main_functions_approved=0,subfunctions_approved=0)
