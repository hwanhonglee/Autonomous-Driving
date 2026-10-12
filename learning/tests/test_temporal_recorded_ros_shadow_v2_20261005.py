"""HH_261005 - Exercise ONLY synthetic scalar/source admission; never import ROS, Torch or an actual model.

Synthetic bytes below are deliberately NOT JPEG/BGRA captures. Passing these
fixtures does not prove numerical replay, DDS latency, live ROS safety or P3.
"""
import ast
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "learning/portable_e2e/temporal_recorded_ros_bundle_v1_20261005.py"
NODE = ROOT / "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_v1_20261005.py"
spec = importlib.util.spec_from_file_location("_hh_recorded_bundle_scalar_fixture", SOURCE)
module = importlib.util.module_from_spec(spec); sys.modules[spec.name] = module
spec.loader.exec_module(module)


def scalar_archive():
    calibration = tuple((float(i),)+(0.,)*15 for i in range(6))
    context = dict(schema=module.SCHEMA+".context",split="val",episode_id="synthetic_known_DEV",
        stream_source_sha256="a"*64,recorded_JPEG_only=True,camera_order=list(module.ROLES),
        episode_root=str(ROOT/"storage/experiments/validation/SYNTHETIC_DO_NOT_READ"),
        calibration=calibration,rig_id=module.sha(json.dumps(calibration,allow_nan=False).encode()),
        optical_frames={r:r+"_optical" for r in module.ROLES},jpeg_sha256={},
        camera_transforms={r:dict(translation=[0.,0.,0.],quaternion=[0.,0.,0.,1.]) for r in module.ROLES},
        route_xy_map_m=[[0.,0.],[200.,0.]])
    states = [dict(frame=1000+i,timestamp=(1000+i)/20.,x=.1*i,y=0.,z=0.,yaw=.25,
        vx=2.5,vy=0.,ax=0.,ay=0.,yaw_rate=0.,steering_tire_angle_rad=.123456789,command=3)
        for i in range(module.EXPECTED_NATIVE)]
    cameras, bundles, rows = [], [], []
    for ordinal,state in enumerate(states[::2]):
        frame=state["frame"]; selected=[]; images={}
        for position,role in enumerate(module.ROLES):
            object_id=100000+ordinal*6+position; path="images/"+role+"/"+str(frame)+".jpg"
            raw=("SYNTHETIC_NOT_JPEG:"+path).encode();context["jpeg_sha256"][path]=module.sha(raw);images[role]=path
            stamp=int(round(state["timestamp"]*1.e9))
            rows.append(dict(role=role,native_frame=frame,image_object_id=object_id,receipt_sequence=len(rows)+1,
                actor_id=position+1,source_image_timestamp_simulation_ns=stamp,
                source_BGRA_encoding_prefixed_sha256=module.sha(("SYNTHETIC_NOT_BGRA:"+path).encode()),
                receipt_error=None,original_queue_put_error=None,original_queue_put_returned=True,
                original_queue_put_calls=1,source_byte_count=640*360*4,source_pixels_changed=False,source_timestamp_changed=False))
            selected.append(dict(role=role,native_frame=frame,image_object_id=object_id,
                source_image_timestamp_simulation_ns=stamp))
        cameras.append(dict(frame=frame,timestamp=state["timestamp"],camera_order=list(module.ROLES),images=images,
            source_timestamps={r:state["timestamp"] for r in module.ROLES}))
        bundles.append(dict(native_frame=frame,cameras=selected))
    callbacks=dict(errors=[],inflight_callbacks=0,all_callbacks_including_bootstrap_or_unconsumed_preserved=True,
        scalar_receipt_limit_exceeded=False,camera_bindings=[dict(role=r,actor_id=i+1) for i,r in enumerate(module.ROLES)],
        callbacks=rows,callback_count=len(rows),bundles=bundles)
    return dict(context=context,cameras=cameras,states=states,callbacks=callbacks)


@pytest.fixture(scope="module")
def archive():
    return scalar_archive()


@pytest.fixture(scope="module")
def index(archive):
    return module.RecordedIndex(**archive)


def payloads(index,ordinal):
    anchor=index.anchors[ordinal]
    return {row["role"]:dict(timestamp_ns=anchor.timestamp_ns,frame_id=index.context["optical_frames"][row["role"]],
        format="jpeg",data=("SYNTHETIC_NOT_JPEG:"+row["jpeg_path"]).encode()) for row in anchor.cameras}


def authority_values():
    return dict(schema=module.SCHEMA+".ROOT_authority",authorized=True,purpose="recorded_JPEG_shadow_replay_only",
        expected_camera_anchors=918,expected_native_states=1836,original_full_callback_postcheck_PASS=True,
        replay_id="synthetic-source-only",shared_monotonic_clock_localhost=True,external_owned_runtime_reviewed=True,
        TF_limits=module.TF_LIMITS,ROS_DOMAIN_ID=93,ROS_LOCALHOST_ONLY="1",RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
        source_sha256={},live_sensor_authorized=False,actuation_authorized=False,
        TEST_access_authorized=False,model_promoted=False,original_BGRA_parity_claimed=False,P3_approved=False)


def test_entire_interval_native_callback_identity_not_stamp_division(index):
    assert len(index.anchors)==918
    for ordinal,anchor in enumerate(index.anchors):
        assert anchor.ordinal==ordinal and anchor.native_frame==1000+2*ordinal
        assert anchor.provenance["recorded_JPEG_only"] and not anchor.provenance["original_BGRA_parity_claimed"]
        assert [r["receipt_sequence"] for r in anchor.cameras]==list(range(ordinal*6+1,ordinal*6+7))
        assert len({r["actor_id"] for r in anchor.cameras})==6


def test_complete918_cold_real_history_padding_is_invalid_not_observed(index):
    admission=module.BundleAdmission(index)
    for ordinal,anchor in enumerate(index.anchors):
        returned,history,mask=admission.admit(anchor.provenance,payloads(index,ordinal),deepcopy(anchor.state),
            index.calibration,tf_verified_rig_id=index.context["rig_id"])
        assert returned is anchor and len(history)==len(mask)==10
        valid=min(ordinal+1,10);assert mask==(False,)*(10-valid)+(True,)*valid
        assert history[:10-valid]==((0.,)*13,)*(10-valid)
        assert history[10-valid:]==tuple(module.features(r.state) for r in index.anchors[max(0,ordinal-9):ordinal+1])
    receipt=admission.denominator();assert receipt["admitted"]==918 and receipt["UNEXECUTED"]==0
    assert receipt["P3_approved"] is False and receipt["full_DDS_to_publication_timing_verified"] is False
    with pytest.raises(ValueError):admission.admit(index.anchors[-1].provenance,payloads(index,917),index.anchors[-1].state,
        index.calibration,tf_verified_rig_id=index.context["rig_id"])


@pytest.mark.parametrize("fault",["split","stream_sha","camera_order","duplicate_optical","calibration_nan","rig",
    "route_nan","route_short","TF_quaternion","jpeg_path","jpeg_sha","native_bool","native_gap","native_nan",
    "camera_frame","camera_order_row","camera_time","missing_role","selected_object","selected_time",
    "actor_bool","receipt_bool","receipt_duplicate","callback_count_bool","callback_error","source_bytes",
    "pixels_changed","stamp_changed","missing_bundle","inflight","preserve_false"])
def test_scalar_archive_fail_closed_all_failed_tail_counts_preserved(archive,fault):
    value=deepcopy(archive);c=value["context"];cb=value["callbacks"];row=cb["callbacks"][0]
    if fault=="split":c["split"]="train"
    elif fault=="stream_sha":c["stream_source_sha256"]="unsealed"
    elif fault=="camera_order":c["camera_order"].reverse()
    elif fault=="duplicate_optical":c["optical_frames"][module.ROLES[0]]=c["optical_frames"][module.ROLES[1]]
    elif fault=="calibration_nan":c["calibration"]=list(c["calibration"]);c["calibration"][0]=(float("nan"),)*16
    elif fault=="rig":c["rig_id"]="b"*64
    elif fault=="route_nan":c["route_xy_map_m"][0][0]=float("nan")
    elif fault=="route_short":c["route_xy_map_m"].pop()
    elif fault=="TF_quaternion":c["camera_transforms"][module.ROLES[0]]["quaternion"]=[0.,0.,0.,2.]
    elif fault=="jpeg_path":name=next(iter(c["jpeg_sha256"]));c["jpeg_sha256"]["../escape"]=c["jpeg_sha256"].pop(name)
    elif fault=="jpeg_sha":c["jpeg_sha256"][next(iter(c["jpeg_sha256"]))]="invalid"
    elif fault=="native_bool":value["states"][0]["frame"]=True
    elif fault=="native_gap":value["states"][1]["frame"]+=1
    elif fault=="native_nan":value["states"][1]["vx"]=float("nan")
    elif fault=="camera_frame":value["cameras"][0]["frame"]+=1
    elif fault=="camera_order_row":value["cameras"][0]["camera_order"].reverse()
    elif fault=="camera_time":value["cameras"][0]["timestamp"]+=.05
    elif fault=="missing_role":value["cameras"][0]["images"].pop(module.ROLES[0])
    elif fault=="selected_object":cb["bundles"][0]["cameras"][0]["image_object_id"]+=10000
    elif fault=="selected_time":cb["bundles"][0]["cameras"][0]["source_image_timestamp_simulation_ns"]+=1
    elif fault=="actor_bool":row["actor_id"]=True
    elif fault=="receipt_bool":row["receipt_sequence"]=True
    elif fault=="receipt_duplicate":cb["callbacks"][1]["receipt_sequence"]=1
    elif fault=="callback_count_bool":cb["callback_count"]=True
    elif fault=="callback_error":row["receipt_error"]="actual failure retained"
    elif fault=="source_bytes":row["source_byte_count"]-=1
    elif fault=="pixels_changed":row["source_pixels_changed"]=True
    elif fault=="stamp_changed":row["source_timestamp_changed"]=True
    elif fault=="missing_bundle":cb["bundles"].pop()
    elif fault=="inflight":cb["inflight_callbacks"]=1
    else:cb["all_callbacks_including_bootstrap_or_unconsumed_preserved"]=False
    with pytest.raises((ValueError,KeyError,TypeError)):module.RecordedIndex(**value)


@pytest.mark.parametrize("fault",["future","stale_stamp","wrong_frame","changed_pixels","raw_encoding","TF",
    "calibration","state_pose","state_nan","state_command","steering_one_ULP"])
def test_admission_failure_never_advances_real_history(index,fault):
    a=module.BundleAdmission(index);anchor=index.anchors[0];proof=deepcopy(anchor.provenance)
    images=payloads(index,0);state=deepcopy(anchor.state);cal=index.calibration;tf=index.context["rig_id"]
    role=module.ROLES[0]
    if fault=="future":proof=deepcopy(index.anchors[1].provenance)
    elif fault=="stale_stamp":images[role]["timestamp_ns"]-=1
    elif fault=="wrong_frame":images[role]["frame_id"]="another_optical_frame"
    elif fault=="changed_pixels":images[role]["data"]+=b"changed"
    elif fault=="raw_encoding":images[role]["format"]="bgra8"
    elif fault=="TF":tf="f"*64
    elif fault=="calibration":cal=tuple(reversed(cal))
    elif fault=="state_pose":state["x"]+=.00001
    elif fault=="state_nan":state["ax"]=float("nan")
    elif fault=="state_command":state["command"]=True
    else:
        bits=struct.unpack("<I",struct.pack("<f",state["steering_tire_angle_rad"]))[0]
        state["steering_tire_angle_rad"]=struct.unpack("<f",struct.pack("<I",bits+1))[0]
    with pytest.raises((ValueError,TypeError)):a.admit(proof,images,state,cal,tf_verified_rig_id=tf)
    assert a.next_ordinal==0 and len(a.history)==0 and a.denominator()["UNEXECUTED"]==918


def test_float32_steering_message_exact_bits_not_loose_tolerance(index):
    a=module.BundleAdmission(index);anchor=index.anchors[0];state=deepcopy(anchor.state)
    state["steering_tire_angle_rad"]=struct.unpack("<f",struct.pack("<f",state["steering_tire_angle_rad"]))[0]
    assert state["steering_tire_angle_rad"]!=anchor.state["steering_tire_angle_rad"]
    _,history,_=a.admit(anchor.provenance,payloads(index,0),state,index.calibration,tf_verified_rig_id=index.context["rig_id"])
    assert history[-1]==module.features(anchor.state)


@pytest.mark.parametrize("fault",[None,"authority","replay_id","source_tuple","future_clock","bool_clock"])
def test_transport_actual_monotonic_chronology_separate_from_native(index,fault):
    authority=dict(replay_id="synthetic",admitted_authority_sha256="a"*64)
    anchor=index.anchors[0];packet=module.transport_packet(anchor,authority,10)
    if fault=="authority":packet["authority_sha256"]="b"*64
    elif fault=="replay_id":packet["replay_id"]="other"
    elif fault=="source_tuple":packet["provenance"]["native_frame"]+=1
    elif fault=="future_clock":packet["publisher_bundle_started_ns"]=21
    elif fault=="bool_clock":packet["publisher_bundle_started_ns"]=True
    if fault is None:assert module.verify_transport(packet,anchor,authority,20,30)==10
    else:
        with pytest.raises(ValueError):module.verify_transport(packet,anchor,authority,20,30)


@pytest.mark.parametrize("fault",[None,"ordinal","frame","stamp","bool_command","wrong_command","route","authority","replay"])
def test_independently_received_navigation_matches_actual_command_causal_route_and_anchor(index,fault):
    anchor=index.anchors[0];authority=dict(replay_id="synthetic",admitted_authority_sha256="a"*64)
    packet=module.navigation_packet(anchor,index,authority)
    if fault in ("ordinal","frame","stamp"):
        key={"ordinal":"ordinal","frame":"native_frame","stamp":"timestamp_ns"}[fault];packet[key]+=1
    elif fault=="bool_command":packet["command"]=True
    elif fault=="wrong_command":packet["command"]=4
    elif fault=="route":packet["route_xy_map_m"][0][0]+=.01
    elif fault=="authority":packet["authority_sha256"]="b"*64
    elif fault=="replay":packet["replay_id"]="other"
    if fault is None:assert module.verify_navigation(packet,anchor,index,authority)==anchor.state["command"]
    else:
        with pytest.raises(ValueError):module.verify_navigation(packet,anchor,index,authority)


def test_default_denial_precedes_every_read(monkeypatch):
    monkeypatch.setattr(module,"read_sealed",lambda *args:pytest.fail("default DENY touched a file"))
    with pytest.raises(ValueError,match="default DENY"):module.authorize(None,None)


def test_source_only_entry_denial_even_with_enable_precedes_reads(monkeypatch):
    monkeypatch.setattr(module,"read_sealed",lambda *args:pytest.fail("source-only DENY touched a file"))
    with pytest.raises(ValueError,match="source-only V1 DENY"):module.authorize(None,"a"*64,enabled=True)


def isolated_environment(monkeypatch):
    for key in ("ROS_DISCOVERY_SERVER","ROS_STATIC_PEERS","CYCLONEDDS_URI","FASTRTPS_DEFAULT_PROFILES_FILE"):
        monkeypatch.delenv(key,raising=False)
    for key,value in dict(ROS_DOMAIN_ID="93",ROS_LOCALHOST_ONLY="1",RMW_IMPLEMENTATION="rmw_cyclonedds_cpp").items():
        monkeypatch.setenv(key,value)


@pytest.mark.parametrize("fault",[None,"TEST","actuator","live","BGRA","P3","owner","clock","TF_loosened",
    "source_changed","missing_vad","missing_bundle","missing_package_init"])
def test_authority_exact_scalar_grant_before_model_ROS(tmp_path,monkeypatch,fault):
    monkeypatch.setattr(module,"ROOT",tmp_path)
    # HH_261005 - Open ONLY this synthetic grant fixture; the real V1 entry remains hard-denied.
    monkeypatch.setattr(module,"ACTUAL_ENTRY_REVIEWED",True)
    isolated_environment(monkeypatch)
    old_canonical=module.canonical
    monkeypatch.setattr(module,"canonical",lambda path:old_canonical(path,boundary=tmp_path))
    value=authority_values()
    names={module.SELF,module.NODE,module.CORE,module.CLOSURE_HELPER,module.OLD_NODE,
        "autonomy/autoware_e2e_vad_launch/scripts/vad_route_logic.py",
        "learning/portable_e2e/runtime_weight_bundle.py","learning/portable_e2e/__init__.py"}
    for name in names:
        path=tmp_path/name;path.parent.mkdir(parents=True,exist_ok=True);raw=("SYNTHETIC_SOURCE:"+name).encode();path.write_bytes(raw)
        value["source_sha256"][name]=module.sha(raw)
    # HH_261005 - Admission uses a scalar synthetic table; a separate genuine-source AST test verifies the real walker.
    monkeypatch.setattr(module,"project_source_closure",lambda:set(names))
    monkeypatch.setattr(module,"CORE_SHA",value["source_sha256"][module.CORE])
    monkeypatch.setattr(module,"CLOSURE_SHA",value["source_sha256"][module.CLOSURE_HELPER])
    fields={"TEST":"TEST_access_authorized","actuator":"actuation_authorized","live":"live_sensor_authorized",
        "BGRA":"original_BGRA_parity_claimed","P3":"P3_approved","owner":"external_owned_runtime_reviewed",
        "clock":"shared_monotonic_clock_localhost"}
    if fault in fields:value[fields[fault]]=not value[fields[fault]]
    elif fault=="TF_loosened":value["TF_limits"]=dict(translation_m=.1,rotation_rad=.1)
    elif fault=="source_changed":value["source_sha256"][module.NODE]="c"*64
    elif isinstance(fault,str) and fault.startswith("missing_"):
        key={"missing_vad":"autonomy/autoware_e2e_vad_launch/scripts/vad_route_logic.py",
            "missing_bundle":"learning/portable_e2e/runtime_weight_bundle.py",
            "missing_package_init":"learning/portable_e2e/__init__.py"}[fault]
        del value["source_sha256"][key]
    path=tmp_path/"ROOT_synthetic_authority.json";raw=module.encode(value);path.write_bytes(raw)
    if fault is None:assert module.authorize(path,module.sha(raw),enabled=True)["admitted_authority_sha256"]==module.sha(raw)
    else:
        with pytest.raises(ValueError):module.authorize(path,module.sha(raw),enabled=True)


@pytest.mark.parametrize("raw",[b'{"a":1,"a":2}',b'{"a":NaN}',b'{"nested":[1e400]}',b'{}'])
def test_bounded_json_duplicate_nonfinite_or_oversize_denial(raw):
    with pytest.raises(ValueError):module.decode(raw,1 if raw==b'{}' else 100)


@pytest.mark.parametrize("fault",[None,"missing_domain","default_domain","bool_domain","oversize_domain",
    "domain_mismatch","localhost_missing","not_localhost","foreign_RMW","RMW_mismatch",
    "discovery_server","static_peers","cyclone_profile","fastdds_profile"])
def test_isolated_declared_DDS_environment_before_dependencies(fault):
    value=authority_values();environ=dict(ROS_DOMAIN_ID="93",ROS_LOCALHOST_ONLY="1",RMW_IMPLEMENTATION="rmw_cyclonedds_cpp")
    if fault=="missing_domain":environ.pop("ROS_DOMAIN_ID")
    elif fault=="default_domain":value["ROS_DOMAIN_ID"]=0;environ["ROS_DOMAIN_ID"]="0"
    elif fault=="bool_domain":value["ROS_DOMAIN_ID"]=True;environ["ROS_DOMAIN_ID"]="True"
    elif fault=="oversize_domain":value["ROS_DOMAIN_ID"]=233;environ["ROS_DOMAIN_ID"]="233"
    elif fault=="domain_mismatch":environ["ROS_DOMAIN_ID"]="94"
    elif fault=="localhost_missing":environ.pop("ROS_LOCALHOST_ONLY")
    elif fault=="not_localhost":value["ROS_LOCALHOST_ONLY"]=environ["ROS_LOCALHOST_ONLY"]="0"
    elif fault=="foreign_RMW":value["RMW_IMPLEMENTATION"]=environ["RMW_IMPLEMENTATION"]="rmw_unreviewed"
    elif fault=="RMW_mismatch":environ["RMW_IMPLEMENTATION"]="rmw_fastrtps_cpp"
    elif fault is not None:
        environ[{"discovery_server":"ROS_DISCOVERY_SERVER","static_peers":"ROS_STATIC_PEERS",
            "cyclone_profile":"CYCLONEDDS_URI","fastdds_profile":"FASTRTPS_DEFAULT_PROFILES_FILE"}[fault]]="foreign"
    if fault is None:module.verify_ros_environment(value,environ)
    else:
        with pytest.raises(ValueError):module.verify_ros_environment(value,environ)


@pytest.mark.parametrize("fault",[None,"missing_member","changed_source","foreign_cached_path","symlink",
    "missing_module_path","local_private_unsealed","foreign_scripts"])
def test_cached_imported_project_module_paths_current_bytes_exact(tmp_path,monkeypatch,fault):
    boundary=tmp_path/"owned";boundary.mkdir();path=boundary/"learning/portable_e2e/runtime.py"
    path.parent.mkdir(parents=True);path.write_bytes(b"SYNTHETIC_SOURCE_NO_IMPORT")
    monkeypatch.setattr(module,"ROOT",boundary)
    old_canonical=module.canonical
    monkeypatch.setattr(module,"canonical",lambda value:old_canonical(value,boundary=boundary))
    table={str(path.relative_to(boundary)):module.sha(path.read_bytes())}
    cached=SimpleNamespace(__name__="portable_e2e.runtime",__file__=str(path))
    if fault=="missing_member":table={}
    elif fault=="changed_source":path.write_bytes(b"CHANGED")
    elif fault in ("foreign_cached_path","foreign_scripts"):
        foreign=tmp_path/"foreign.py";foreign.write_bytes(b"FOREIGN");cached.__file__=str(foreign)
        if fault=="foreign_scripts":cached.__name__="scripts.e2e.foreign"
    elif fault=="symlink":link=boundary/"alias.py";link.symlink_to(path);cached.__file__=str(link)
    elif fault=="missing_module_path":del cached.__file__
    elif fault=="local_private_unsealed":cached.__name__="_private_local_project";table={}
    if fault is None:module.verify_imported_project_modules(dict(source_sha256=table),[cached])
    else:
        with pytest.raises((ValueError,FileNotFoundError)):module.verify_imported_project_modules(dict(source_sha256=table),[cached])


def test_genuine_pinned_stdlib_closure_explicit_autonomy_dynamic_and_package_seeds():
    # HH_261005 - AST/source bytes ONLY: the training module and all traced runtime modules are never imported.
    sources=module.project_source_closure()
    assert {module.SELF,module.NODE,module.CORE,module.OLD_NODE,module.CLOSURE_HELPER,
        "autonomy/autoware_e2e_vad_launch/scripts/vad_route_logic.py",
        "learning/portable_e2e/__init__.py","learning/portable_e2e/runtime_weight_bundle.py",
        "learning/portable_e2e/temporal_route_joint_export_research_20261002.py"} <= sources
    assert all(name.endswith(".py") and (ROOT/name).is_file() for name in sources)
    tree=ast.parse(SOURCE.read_bytes());function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="project_source_closure")
    text=ast.unparse(function)
    assert '"imported_source_closure"' in ast.get_source_segment(SOURCE.read_text(),function)
    assert "(NODE, OLD_NODE)" in text and "__init__.py" in text
    assert module.sha((ROOT/module.CLOSURE_HELPER).read_bytes())==module.CLOSURE_SHA


def scalar_node_function(name):
    node=next(n for n in ast.parse(NODE.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name==name)
    namespace=dict(math=math)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(NODE),"exec"),namespace)
    return namespace[name]


@pytest.mark.parametrize("seconds,nanoseconds",[(0,0),(50,123),(True,0),(0,False),(-1,0),(0,-1),(0,10**9),(0.,0),(0,1.)])
def test_ROS_stamp_strict_native_time_no_bool_or_coercion(seconds,nanoseconds):
    function=scalar_node_function("stamp_ns");stamp_value=SimpleNamespace(sec=seconds,nanosec=nanoseconds)
    if type(seconds) is int and type(nanoseconds) is int and seconds>=0 and 0<=nanoseconds<10**9:
        assert function(stamp_value)==seconds*10**9+nanoseconds
    else:
        with pytest.raises(ValueError):function(stamp_value)


def test_message_set_uses_real_stamp_and_supported_K_D_R_P_only(index):
    ns=SimpleNamespace
    def time_value():return ns(sec=0,nanosec=0)
    def header():return ns(stamp=time_value(),frame_id="")
    def vector():return ns(x=0.,y=0.,z=0.)
    def rotation():return ns(x=0.,y=0.,z=0.,w=0.)
    ros=ns(Odometry=lambda:ns(header=header(),child_frame_id="",pose=ns(pose=ns(position=vector(),orientation=rotation())),
        twist=ns(twist=ns(linear=vector(),angular=vector()))),
        AccelWithCovarianceStamped=lambda:ns(header=header(),accel=ns(accel=ns(linear=vector()))),
        SteeringReport=lambda:ns(stamp=time_value(),steering_tire_angle=0.),CameraInfo=lambda:ns(header=header()),
        TransformStamped=lambda:ns(header=header(),child_frame_id="",transform=ns(translation=vector(),rotation=rotation())),
        Clock=lambda:ns(clock=time_value()),TFMessage=lambda **values:ns(**values))
    function=scalar_node_function("message_set")
    stamp_function=next(n for n in ast.parse(NODE.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name=="stamp")
    exec(compile(ast.Module(body=[stamp_function],type_ignores=[]),str(NODE),"exec"),function.__globals__)
    k=(200.,0.,320.,0.,200.,180.,0.,0.,1.)
    # HH_261005 - This real rig-metadata-shaped fixture intentionally has NO live rectification_r/projection_p attributes.
    rig=ns(cameras=tuple(ns(name=r,optical_frame=index.context["optical_frames"][r],width_px=640,height_px=360,
        intrinsic_k=k,distortion_d=()) for r in module.ROLES))
    odom,acc,steer,infos,tf,clock=function(index.anchors[0],rig,index.context,ros)
    assert odom.header.frame_id=="map" and odom.child_frame_id=="base_link" and acc.header.frame_id=="base_link"
    assert clock.clock.sec*10**9+clock.clock.nanosec==index.anchors[0].timestamp_ns
    assert len(tf.transforms)==6 and steer.steering_tire_angle==index.anchors[0].state["steering_tire_angle_rad"]
    assert all(info.k==list(k) and info.d==[0.]*5 and info.r==[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        and info.p==[200.,0.,320.,0.,0.,200.,180.,0.,0.,0.,1.,0.] for info in infos.values())


def test_sources_default_Deny_shadow_only_and_frozen_policy_no_mutation():
    tree=ast.parse(NODE.read_bytes());top=[]
    for row in tree.body:
        if isinstance(row,(ast.Import,ast.ImportFrom)):top.append(ast.unparse(row))
    assert all("torch" not in row and "rclpy" not in row and "portable_e2e" not in row for row in top)
    funcs={n.name:ast.unparse(n) for n in tree.body if isinstance(n,ast.FunctionDef)}
    assert funcs["main"].index("pure.authorize(") < funcs["main"].index("dependencies(") < funcs["main"].index("ros.rclpy.init(")
    assert funcs["dependencies"].index("pure.verify_ros_environment(") < funcs["dependencies"].index("import rclpy")
    assert funcs["dependencies"].index("pure.verify_imported_project_modules(") < funcs["dependencies"].index("import rclpy")
    assert funcs["load_runtime"].index("pure.verify_ros_environment(") < funcs["load_runtime"].index("from portable_e2e")
    assert funcs["load_runtime"].index("pure.verify_imported_project_modules(") < funcs["load_runtime"].index("loader.load_route_joint_research_npz(")
    assert "pure.verify_navigation(" in funcs["recorded_node"] and "pure.navigation_packet(" in funcs["recorded_publisher"]
    assert "source_root=ROOT" in funcs["load_runtime"] and "configure_runtime_execution('cpu')" in funcs["load_runtime"]
    assert "ObservedTemporalFrameV2" in funcs["build_observed_frame"] and "_canonical_route_in_base" in funcs["build_observed_frame"]
    assert "apply_control" not in NODE.read_text() and "create_client" not in NODE.read_text()
    assert "fixed_subscription" in funcs["recorded_node"] and "resolved == topic" in funcs["recorded_node"]
    assert "clock_type=ros.ClockType.ROS_TIME" in funcs["recorded_node"]
    assert "adapter.accept(" in funcs["recorded_node"] and "result.status" in funcs["recorded_node"]
    assert "input_tensor_hash_parity_implemented=False" in funcs["recorded_node"]
    assert "direct_vs_ROS_numerical_parity_verified=False" in funcs["recorded_node"]
    assert funcs["recorded_node"].index("self.receipt.publish(ros.String(data=pure.encode(receipt).decode()))",funcs["recorded_node"].index("receipt_post_publish_completion_unavailable")) < funcs["recorded_node"].index("stages['receipt_publication_finished_ns']")
    assert "wall_10Hz_replay_claimed=False" in funcs["recorded_publisher"]
    assert module.sha((ROOT/module.CORE).read_bytes())==module.CORE_SHA
