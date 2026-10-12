"""HH_261008 - V3 preserves V2 failure, fixes exact AST membership and adds own-run wire/independent-closer fixtures.

Scalar/AST/fake-buffer only; no actual records, NumPy, Pillow, plotting or NN.

Genuine frozen scalar geometry is exercised on synthetic poses, not a new
model/output equivalence test. GIF fixtures prove caller byte/order/lifetime
rules, not the actual Pillow encoder ABI or visual correctness.
"""
import ast
from copy import deepcopy
import csv
import gzip
import hashlib
import importlib.util
from io import BytesIO, StringIO
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/"tools/e2e/report_recorded_vehicle_centered_gallery_20261008_v3.py"


@pytest.fixture
def gallery():
    spec=importlib.util.spec_from_file_location("_hh_test_record_gallery",SOURCE)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def selected():
    return dict(candidate_index=0,xy_base_m=[[.2*(i+1),.01] for i in range(64)],speed_mps=[2.]*64,
        heading_rad=[0.]*64,timestep_s=.1,logit_margin=None,planar_extent_m=12.8)


def index_and_rows():
    anchors=[SimpleNamespace(ordinal=i,native_frame=1000+2*i,timestamp_ns=10_000_000_000+100_000_000*i,provenance={"genuine_synthetic_ordinal":i}) for i in range(918)]
    topics=["synthetic_topic_"+str(i) for i in range(17)]
    direct=[dict(ordinal=a.ordinal,native_frame=a.native_frame,timestamp_ns=a.timestamp_ns,status="TRAJECTORY",selected=selected(),
        input_CDR_sha256={topic:"a"*64 for topic in topics},direct_outputs_published=False) for a in anchors]
    direct[0].update(status="PREFILL",selected=None);direct[2].update(status="REJECTED",selected=None)
    sub=[dict(deepcopy(row),source_selected_callback_provenance=deepcopy(anchors[i].provenance)) for i,row in enumerate(direct[:29])]
    sub[-1].update(status="FAILED_RETAINED",selected=None);sub[-1].pop("source_selected_callback_provenance")
    return SimpleNamespace(anchors=anchors),dict(direct_anchors=direct,subscriber_anchors=sub),topics


def test_import_is_stdlib_no_array_or_runtime(gallery):
    assert not any(name in sys.modules for name in ("numpy","matplotlib","PIL","torch","rclpy","carla"))
    tree=ast.parse(SOURCE.read_bytes())
    modules=[n.module.split(".")[0] for n in tree.body if isinstance(n,ast.ImportFrom) and n.module]
    modules += [a.name.split(".")[0] for n in tree.body if isinstance(n,ast.Import) for a in n.names]
    assert not set(modules)&{"numpy","matplotlib","PIL","torch","rclpy","carla"}


def test_whole2x918_failure_NULL_no_future_output(gallery):
    index,rows,topics=index_and_rows();result=gallery.source_records(index,rows,topics)
    assert list(result)==["DIRECT","ROS"] and all(len(values)==918 for values in result.values())
    assert result["DIRECT"][0]["stored_selection_sha256"] is None
    assert result["DIRECT"][2]["selected_points"]==0
    assert result["DIRECT"][3]["selected_points"]==64
    assert result["DIRECT"][3]["source_join"]=="ORIGINAL_DIRECT_PRODUCER_SOURCE_WITNESS_INDEX_AND_17CDR_RECEIPTS"
    assert result["ROS"][28]["status"]=="FAILED_RETAINED" and result["ROS"][28]["selected_points"]==0
    assert result["ROS"][28]["source_join"]=="UNKNOWN_FAILED_ORIGINAL_ADMISSION"
    tail=result["ROS"][29:]
    assert len(tail)==889 and all(r["status"]=="UNEXECUTED" and r["stored_selection_sha256"] is None and r["GIF_delay_ms"] is None for r in tail)
    assert all(r["captured_control_origin"]=="UNKNOWN" for values in result.values() for r in values)
    assert result["DIRECT"][-1]["GIF_delay_ms"]==result["ROS"][28]["GIF_delay_ms"]==0
    assert result["ROS"][0]["GIF_delay_ms"]==100


@pytest.mark.parametrize("fault",["direct_missing","ROS_missing","direct_order","ROS_order","frame","stamp","source","nonnull_missing_source","topic_missing","topic_extra","bad_sha","direct_published"])
def test_whole_original_source_denials(gallery,fault):
    index,rows,topics=index_and_rows()
    if fault=="direct_missing":rows["direct_anchors"].pop()
    elif fault=="ROS_missing":rows["subscriber_anchors"].pop()
    elif fault=="direct_order":rows["direct_anchors"][1]["ordinal"]=2
    elif fault=="ROS_order":rows["subscriber_anchors"][1]["ordinal"]=2
    elif fault=="frame":rows["direct_anchors"][0]["native_frame"]+=1
    elif fault=="stamp":rows["subscriber_anchors"][0]["timestamp_ns"]+=1
    elif fault=="source":rows["subscriber_anchors"][0]["source_selected_callback_provenance"]={}
    elif fault=="nonnull_missing_source":rows["subscriber_anchors"][0].pop("source_selected_callback_provenance")
    elif fault=="topic_missing":rows["direct_anchors"][0]["input_CDR_sha256"].pop(topics[0])
    elif fault=="topic_extra":rows["direct_anchors"][0]["input_CDR_sha256"]["foreign"]="a"*64
    elif fault=="bad_sha":rows["direct_anchors"][0]["input_CDR_sha256"][topics[0]]="z"*64
    else:rows["direct_anchors"][0]["direct_outputs_published"]=True
    with pytest.raises(ValueError):gallery.source_records(index,rows,topics)


@pytest.mark.parametrize("status",["PREFILL","REJECTED","REJECTED_ROS_END_TO_END_DEADLINE","FAILED_RETAINED","UNEXECUTED"])
def test_rejected_prefill_failure_never_draw_even_prior_selection_exists(gallery,status):
    assert gallery.stored_selection(dict(status=status,selected=selected())) is None


@pytest.mark.parametrize("fault",["missing","extra","point_count","XY_width","XY_bool","speed_count","speed_nan","heading_count","heading_inf","step","candidate_bool","margin_inf","extent_nan"])
def test_selected_complete_original_ABI_denials(gallery,fault):
    value=selected()
    if fault=="missing":value.pop("heading_rad")
    elif fault=="extra":value["unverified_world_xy"]=[]
    elif fault=="point_count":value["xy_base_m"].pop()
    elif fault=="XY_width":value["xy_base_m"][0].append(1.)
    elif fault=="XY_bool":value["xy_base_m"][0][0]=True
    elif fault=="speed_count":value["speed_mps"].pop()
    elif fault=="speed_nan":value["speed_mps"][0]=float("nan")
    elif fault=="heading_count":value["heading_rad"].pop()
    elif fault=="heading_inf":value["heading_rad"][0]=float("inf")
    elif fault=="step":value["timestep_s"]=.2
    elif fault=="candidate_bool":value["candidate_index"]=True
    elif fault=="margin_inf":value["logit_margin"]=float("inf")
    else:value["planar_extent_m"]=float("nan")
    with pytest.raises(ValueError):gallery.stored_selection(dict(status="TRAJECTORY",selected=value))


@pytest.fixture
def geometry(gallery):
    names=(gallery.COLLECTOR,gallery.CONTRACT,gallery.RUNTIME_CONTRACT,gallery.OLD_ROS)
    plan={"source_sha256":{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in names}}
    return gallery.scalar_geometry(plan)


def test_genuine_CARLA_to_ROS_rear_axle_then_no_second_y_flip(gallery,geometry):
    pose=dict(x=10.,y=20.,z=1.,roll=0.,pitch=0.,yaw=90.)
    state=geometry["base_link_state"](pose,[0.,2.,0.],[0.,0.,0.],0.,2.85)
    assert state["x"]==pytest.approx(10.) and state["y"]==pytest.approx(-18.575)
    assert state["yaw"]==pytest.approx(-gallery.math.pi/2)
    q=(0.,0.,gallery.math.sin(state["yaw"]/2),gallery.math.cos(state["yaw"]/2))
    inv=geometry["map_to_base"]((state["x"],state["y"],state["z"]),q,"scalar_fixture")
    forward=geometry["transform_base_trajectory_to_map"](SimpleNamespace(**selected()),ego_x_m=state["x"],ego_y_m=state["y"],ego_yaw_rad=geometry["quaternion_yaw"](SimpleNamespace(x=q[0],y=q[1],z=q[2],w=q[3])))
    for point,original in zip(forward,selected()["xy_base_m"]):assert inv(point[:2])==pytest.approx(original,abs=1e-12)
    assert not any(name in sys.modules for name in ("numpy","torch","rclpy","carla"))


def native_fixture(geometry):
    pose=dict(x=10.,y=20.,z=1.,roll=0.,pitch=0.,yaw=35.)
    converted=geometry["base_link_state"](pose,[1.,2.,0.],[.1,.2,0.],.1,2.85)
    states=[dict(converted,frame=1000+i,timestamp=10.+i*.05,actor_snapshot_transform_carla=deepcopy(pose),world_velocity_carla=[1.,2.,0.],
        world_acceleration_carla=[.1,.2,0.],world_angular_velocity_carla_deg_s=[0.,0.,.1]) for i in range(1836)]
    manifest=dict(coordinate_contract=dict(map_frame="ROS map: x forward/east, y left, z up",ego_frame="ROS rear-axle base_link: x forward, y left, z up",wheelbase_m=2.85),
        capture_contract=dict(learned_research=dict(expert_startup_until_vx_above_mps=.5,no_teacher_fallback_after_handoff=True)))
    return states,manifest


@pytest.mark.parametrize("fault",[None,"y_sign","body_center","wheelbase","missing","frame_gap","time_gap","raw_nonfinite","vector_shape","teacher_fallback"])
def test_all1836_native_agreement_and_no_control_frame_relabel(gallery,geometry,fault):
    states,manifest=native_fixture(geometry)
    if fault=="y_sign":states[0]["y"]=-states[0]["y"]
    elif fault=="body_center":states[0]["x"]+=1.425
    elif fault=="wheelbase":manifest["coordinate_contract"]["wheelbase_m"]=3.
    elif fault=="missing":states.pop()
    elif fault=="frame_gap":states[1]["frame"]+=1
    elif fault=="time_gap":states[1]["timestamp"]+=.01
    elif fault=="raw_nonfinite":states[0]["world_velocity_carla"][0]=float("nan")
    elif fault=="vector_shape":states[0]["world_acceleration_carla"].append(0.)
    elif fault=="teacher_fallback":manifest["capture_contract"]["learned_research"]["no_teacher_fallback_after_handoff"]=False
    if fault is None:
        result=gallery.verify_native_geometry(states,manifest,geometry)
        assert result["native_states"]==1836 and result["maximum_original_conversion_error"]==0.
        assert result["per_frame_control_origin"]=="UNKNOWN" and not result["physical_actuation_proof"]
    else:
        with pytest.raises(ValueError):gallery.verify_native_geometry(states,manifest,geometry)


@pytest.mark.parametrize("text",["", "VmRSS: 1 kB", "VmRSS: 1 kB\nVmRSS: 1 kB\nVmHWM: 2 kB", "VmRSS: -1 kB\nVmHWM: 2 kB", "VmRSS: 2 MB\nVmHWM: 3 kB", "VmRSS: 3 kB\nVmHWM: 2 kB"])
def test_current_process_memory_parser_fail_closed(gallery,monkeypatch,text):
    monkeypatch.setattr(gallery,"Path",lambda p:SimpleNamespace(read_text=lambda:text))
    with pytest.raises(ValueError):gallery.current_resources()


def test_memory_inherited_ru_is_recorded_not_gate_and_failure_sample_retained(gallery,monkeypatch):
    monkeypatch.setattr(gallery,"Path",lambda p:SimpleNamespace(read_text=lambda:"VmRSS: 10 kB\nVmHWM: 20 kB"))
    monkeypatch.setattr(gallery.resource,"getrusage",lambda _:SimpleNamespace(ru_maxrss=1024**2))
    monkeypatch.setattr(gallery,"budget",lambda **kwargs:{})
    trace=[];result=gallery.guard(gallery.time.monotonic(),trace=trace)
    assert result["VmHWM"]==20480 and result["inherited_ru_maxrss_bytes_not_gate"]==1024**3 and len(trace)==1
    monkeypatch.setattr(gallery,"current_resources",lambda:dict(pid=1,VmRSS=gallery.RSS,VmHWM=gallery.RSS+1,inherited_ru_maxrss_bytes_not_gate=1))
    with pytest.raises(ValueError):gallery.guard(gallery.time.monotonic(),trace=trace)
    assert len(trace)==2 and trace[-1][3]==gallery.RSS+1


def test_fake_streaming_two_frames_no_save_all_and_oversize_before_write(gallery,monkeypatch):
    calls=[]
    class Plugin:
        @staticmethod
        def getheader(image,**kwargs):calls.append(("header",image,kwargs));return [b"GIF89a"],None
        @staticmethod
        def getdata(image,**kwargs):calls.append(("frame",image,kwargs));return [b"encoded_owned_frame"]
    monkeypatch.setattr(gallery,"budget",lambda *args,**kwargs:{"gif":0});stream=BytesIO()
    gallery.append_gif_frame(stream,"owned1",Plugin,100,True);gallery.append_gif_frame(stream,"owned2",Plugin,0,False)
    assert [call[0] for call in calls]==["header","frame","frame"]
    assert [call[2]["duration"] for call in calls if call[0]=="frame"]==[100,0]
    frames=[call for call in calls if call[0]=="frame"]
    assert frames[0][2]["loop"]==0 and "loop" not in frames[1][2]
    before=stream.getvalue();monkeypatch.setattr(gallery,"budget",lambda *args,**kwargs:{"gif":gallery.GIFS})
    with pytest.raises(ValueError):gallery.append_gif_frame(stream,"not_written",Plugin,100,False)
    assert stream.getvalue()==before


def test_complete_CSV_NULLs_and_failed_tail_roundtrip(gallery):
    index,rows,topics=index_and_rows();records=gallery.source_records(index,rows,topics)
    records["ROS"][28].update(render_status="FAILED_RETAINED",render_error="synthetic diagnostic frame failure")
    values=list(csv.DictReader(StringIO(gzip.decompress(gallery.csv_bytes(records)).decode(),newline="")))
    assert len(values)==1836 and [int(r["ordinal"]) for r in values[:918]]==list(range(918))
    assert values[918+28]["render_error"]=="synthetic diagnostic frame failure"
    assert values[918+29]["status"]=="UNEXECUTED" and values[918+29]["stored_selection_sha256"]==values[918+29]["GIF_delay_ms"]==""


def test_fixed_protocol_and_actual_admission_precedes_framework_and_full_readers(gallery):
    fixed=gallery.fixed();assert fixed["expected_anchors_per_arm"]==918 and fixed["ROS_UNEXECUTED"]==889
    assert fixed["PNG_source_ordinals"]==[0,459,917] and fixed["per_frame_old_control_origin"]=="UNKNOWN"
    assert gallery.SOURCES+gallery.GIFS+3*gallery.PNG+gallery.CSV+gallery.CACHE+gallery.PLANS<=gallery.TOTAL-gallery.CLOSING
    assert gallery.TOTAL==32*1024**2 and gallery.CLOSING==4*1024**2 and gallery.RSS==512*1024**2 and gallery.WALL==900
    tree=ast.parse(SOURCE.read_bytes());functions={p.name:p for p in tree.body if isinstance(p,ast.FunctionDef)};body=ast.unparse(functions["execute"])
    assert body.index("grant == dict")<body.index("verify(plan)")<body.index("bundle.load_index")<body.index("import matplotlib")
    assert "codec.verify_ledger" in body and "clock.read_ledgers" in body and "completed_GIF_frames" in body
    assert "actual_RViz_screenshot=False" in body and "captured_public_coordinates_available=False" in body
    assert "save_all" not in body and "model.load" not in body and "forward(" not in body
    scalar=ast.unparse(functions["scalar_geometry"])
    assert 'route.body[first:last + 1]' in scalar and 'deepcopy(inner)' in scalar
    draw=ast.unparse(functions["draw"])
    assert 'r[\'frame\'] <= anchor.native_frame' in draw or 'r["frame"] <= anchor.native_frame' in draw
    assert "transform_base_trajectory_to_map" in draw and "stored_selection(row)" in draw
    assert "body_outline" not in draw and "future" not in draw


def test_parent_identity_sources_and_scalar_origin_are_not_V5_relabel(gallery):
    assert gallery.EPISODE.name=="episode" and "2026-10-04" in str(gallery.EPISODE) and "full209_v1" in str(gallery.EPISODE)
    assert list(gallery.PARENT_PINS.values())==[
        "022a86a082f800a0011b64b94a7847ef73df126fa5aa07132a4977e58daec371",
        "0ae924163bc2f6323a54766091aec731ecbf4e4d013557aea2062474ac995ff9",
        "a0bcdbcae72dc562ad3eafd086eb0b64301f122d678c5c046e7116372bb5cdcb"]
    assert "control_receipts" not in SOURCE.read_text() and "learned_control.jsonl" not in SOURCE.read_text()
    assert not any(name in sys.modules for name in ("numpy","matplotlib","PIL","torch","rclpy","carla"))


# HH_261008 - Fake axes exercise the genuine draw() geometry without Matplotlib, recorded arrays or render execution.
class FakeLine:
    def set_data(self,x,y):self.x,self.y=list(x),list(y)


class FakeAxis:
    def __init__(self,xlim=None,ylim=None):self.xlim,self.ylim=xlim,ylim
    def set_xlim(self,left,right):self.xlim=(left,right)
    def set_ylim(self,bottom,top):self.ylim=(bottom,top)


@pytest.mark.parametrize("ordinal,position",[(0,(-20.,7.)),(459,(40.,-30.)),(917,(97.,66.))])
@pytest.mark.parametrize("status",["TRAJECTORY","PREFILL","FAILED_RETAINED"])
def test_offcenter_start_middle_end_keep_both_views_centered_and_complete_route_contained(gallery,geometry,ordinal,position,status):
    route=[(-20.,7.),(40.,-30.),(97.,66.),(105.,12.)]
    x,y=position;state=dict(x=x,y=y,z=0.,yaw=.4)
    anchor=SimpleNamespace(ordinal=ordinal,native_frame=2000,timestamp_ns=10_000_000_000+ordinal*100_000_000,state=state)
    states=[dict(frame=1998,x=x-4.,y=y+2.),dict(frame=2000,x=x,y=y),dict(frame=2001,x=10**8,y=-10**8)]
    local=FakeAxis((-24.,24.),(-24.,24.));overview=FakeAxis();artists=[tuple(FakeLine() for _ in range(3)) for _ in range(2)]
    ego=FakeLine();captions=[];draws=[]
    fig=SimpleNamespace(canvas=SimpleNamespace(draw=lambda:draws.append(1)))
    value=(fig,local,overview,artists,ego,SimpleNamespace(set_text=captions.append))
    row=dict(status=status,selected=selected())
    assert gallery.draw(value,"DIRECT",anchor,row,states,route,geometry) is fig
    assert (sum(local.xlim)/2,sum(local.ylim)/2)==(0.,0.)
    assert (sum(overview.xlim)/2,sum(overview.ylim)/2)==pytest.approx((x,y),abs=1e-12)
    assert overview.xlim[1]-overview.xlim[0]==pytest.approx(overview.ylim[1]-overview.ylim[0])
    assert ego.x==[x] and ego.y==[y] and draws==[1]
    points=route+[(r["x"],r["y"]) for r in states if r["frame"]<=anchor.native_frame]
    points+=list(zip(artists[1][2].x,artists[1][2].y))
    assert all(overview.xlim[0]<px<overview.xlim[1] and overview.ylim[0]<py<overview.ylim[1] for px,py in points)
    extent=max(max(abs(px-x),abs(py-y)) for px,py in points)
    assert overview.xlim==(x-max(5.,extent)-5.,x+max(5.,extent)+5.)
    assert (10**8,-10**8) not in list(zip(artists[1][1].x,artists[1][1].y))
    assert len(artists[0][2].x)==len(artists[1][2].x)==(64 if status=="TRAJECTORY" else 0)


@pytest.mark.parametrize("fault",[None,"RSS","wall","parser"])
def test_closing_sample_FAIL_retains_full947_completed_and_both918_ledgers(gallery,monkeypatch,fault):
    ledger={arm:[[i,"PASS" if arm=="DIRECT" or i<29 else "UNEXECUTED",None] for i in range(918)] for arm in gallery.ARMS}
    result=dict(status="COMPLETE_RECORD_DIAGNOSTIC_NOT_PUBLIC_COORDINATE_OR_FEATURE_PROOF",closing_errors=[],
        completed_GIF_frames={"DIRECT":918,"ROS":29},whole_render_ledger=deepcopy(ledger))
    observed=dict(pid=12,VmRSS=100,VmHWM=gallery.RSS+1 if fault=="RSS" else gallery.RSS,inherited_ru_maxrss_bytes_not_gate=10**12)
    def sample():
        if fault=="parser":raise ValueError("SYNTHETIC_MISSING_CLOSING_MEMORY")
        return observed
    monkeypatch.setattr(gallery,"current_resources",sample)
    monkeypatch.setattr(gallery.time,"monotonic",lambda:gallery.WALL+1 if fault=="wall" else gallery.WALL)
    gallery.check_closing_resources(result,0.)
    assert result["whole_render_ledger"]==ledger and all(len(rows)==918 for rows in result["whole_render_ledger"].values())
    assert result["completed_GIF_frames"]=={"DIRECT":918,"ROS":29}
    if fault is None:
        assert result["status"].startswith("COMPLETE_") and result["closing_errors"]==[]
    else:
        assert result["status"]=="FAILED_RETAINED" and len(result["closing_errors"])==1
    assert result["sampled_resources_closing"]==(None if fault=="parser" else observed)
    assert result["sampled_closing_wall_seconds"]==(None if fault=="parser" else gallery.WALL+1 if fault=="wall" else gallery.WALL)


def test_v2_frozen_V1_and_ROOT_finding_preserved_as_source_and_metadata(gallery):
    assert len(gallery.STOPPED_V1_PINS)==3
    for name,pin in gallery.STOPPED_V1_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    finding=gallery.read_json(ROOT/next(name for name in gallery.STOPPED_V1_PINS if name.endswith(".json")))
    assert finding["status"]=="STOPPED_SOURCE_ONLY_NOT_RENDER_OR_MODEL_FAILURE"
    assert all(finding[key]==0 for key in ("actual_pure_tests","actual_prepare_attempts","actual_rendered_frames","model_inferences","training_steps"))
    functions={part.name:part for part in ast.parse(SOURCE.read_bytes()).body if isinstance(part,ast.FunctionDef)}
    table=ast.unparse(functions["tables"])
    assert "*STOPPED_V1_PINS" in table and "stopped V1 source/finding changed" in table
    expected=ast.parse("inputs.update({str(ROOT/name):pin for name,pin in STOPPED_V1_PINS.items()})").body[0]
    assert sum(ast.dump(part,include_attributes=False)==ast.dump(expected,include_attributes=False) for part in functions["tables"].body)==1
    fixed=gallery.fixed();assert fixed["all_scene_views_ego_centered"] and fixed["overview_contains_full_original_route_causal_past_and_stored_selection"]
    assert fixed["local_view_fixed_clip_m"]==24 and fixed["closing_result_encode_write_after_last_sample"]
    assert gallery.TOTAL==32*1024**2 and gallery.CLOSING==4*1024**2
    execute=ast.unparse(functions["execute"])
    assert execute.index("whole_render_ledger")<execute.index("check_closing_resources(result, started)")<execute.index("write_new(OUT / 'result.json'")


def test_v2_original_science_exact_and_only_overview_bound_seam_changes_draw(gallery):
    old_path=ROOT/"tools/e2e/report_recorded_vehicle_centered_gallery_20261008_v1.py"
    old={part.name:part for part in ast.parse(old_path.read_bytes()).body if isinstance(part,ast.FunctionDef)}
    new={part.name:part for part in ast.parse(SOURCE.read_bytes()).body if isinstance(part,ast.FunctionDef)}
    changed={"tables","fixed","scene","draw","execute"}
    assert new.keys()==old.keys()|{"check_closing_resources","own_run_transport_proof"}
    for name in old.keys()-changed:assert ast.dump(new[name],include_attributes=False)==ast.dump(old[name],include_attributes=False),name
    part=deepcopy(new["draw"])
    expected=ast.parse('''extent=max((max(abs(p[0]-state["x"]),abs(p[1]-state["y"])) for p in (*route,*past,*mapped)),default=0.)
radius=max(5.,extent)+5.
overview.set_xlim(state["x"]-radius,state["x"]+radius)
overview.set_ylim(state["y"]-radius,state["y"]+radius)
ego.set_data([state["x"]],[state["y"]])''').body
    starts=[i for i in range(len(part.body)-len(expected)+1) if all(ast.dump(a,include_attributes=False)==ast.dump(b,include_attributes=False) for a,b in zip(part.body[i:i+len(expected)],expected))]
    assert len(starts)==1
    original=ast.parse('''xs=[p[0] for p in route]
ys=[p[1] for p in route]
overview.set_xlim(min(xs)-5,max(xs)+5)
overview.set_ylim(min(ys)-5,max(ys)+5)
ego.set_data([state["x"]],[state["y"]])''').body
    part.body[starts[0]:starts[0]+len(expected)]=original
    assert ast.dump(part,include_attributes=False)==ast.dump(old["draw"],include_attributes=False)


# HH_261008 - Exercise the four genuine scalar wire-join ASTs on explicit synthetic receipts, never ROS or stored payloads.
def transport_fixture(gallery):
    index,rows,_=index_and_rows()
    tree=ast.parse((ROOT/gallery.BUNDLE).read_bytes())
    assignments={n.targets[0].id:n.value for n in tree.body if isinstance(n,ast.Assign) and len(n.targets)==1 and isinstance(n.targets[0],ast.Name)}
    prefix=ast.literal_eval(assignments["INPUT_PREFIX"]);roles=ast.literal_eval(assignments["ROLES"])
    neutral=[prefix+"/"+key for key in ("odometry","acceleration","steering","tf","clock")]
    neutral += [prefix+"/"+role+"/"+kind for role in roles for kind in ("compressed","camera_info")]
    whole=neutral+[prefix+"/provenance",prefix+"/navigation"]
    typed="autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_v1_20261005.py"
    pins={gallery.BUNDLE:gallery.FIXED_PINS[gallery.BUNDLE],typed:"e0e1b001c3684dfdf1ea41274c4c39b7f9a286d008d37d942aefbe35a3591e7e"}
    clock=SimpleNamespace(__file__=str(ROOT/gallery.CLOCK),FIXED_SOURCE_PINS=dict(pins),ARMS={"not_V19c":"unchanged fake globals"})
    plan={"source_sha256":{gallery.CLOCK:gallery.FIXED_PINS[gallery.CLOCK],**pins}};parent={"replay_id":"SYNTHETIC_V19c_receipt_scope"}
    for row in rows["direct_anchors"]:row["input_CDR_sha256"]={name:"a"*64 for name in neutral}
    rows["publisher_anchors"]=[]
    for row in rows["subscriber_anchors"]:
        row.update(replay_id=parent["replay_id"],authority_sha256=gallery.PARENT_PINS["authority"],
            actual_received_input_CDR_sha256={name:"a"*64 for name in whole},stages={"ROS_admission_finished_ns":1})
        published=deepcopy(row);published["published_input_CDR_sha256"]=published.pop("actual_received_input_CDR_sha256")
        rows["publisher_anchors"].append(published)
    return plan,parent,index,rows,clock,neutral,whole


def test_genuine_private_V19c_wire_join_all29_and_global_CLOCK_unchanged(gallery):
    plan,parent,index,rows,clock,neutral,whole=transport_fixture(gallery)
    old=deepcopy(vars(clock));rows["subscriber_anchors"][3]["status"]="REJECTED"
    proof=gallery.own_run_transport_proof(plan,parent,index,rows,clock)
    assert vars(clock)==old and proof["DIRECT_neutral17_rows"]==918 and proof["ROS_sent_received_whole19_rows"]==29
    assert [row["ordinal"] for row in proof["rows"]]==list(range(29))
    assert proof["rows"][0]["auxiliary_binding"]=="SOURCE_BOUND_ORIGINAL_ADMISSION_WITNESS"
    assert proof["rows"][-1]["auxiliary_binding"]=="UNKNOWN_NO_ORIGINAL_ADMISSION_WITNESS"
    assert proof["auxiliary_raw_String_bytes_freshly_revalidated"] is False
    assert all(row["auxiliary_raw_bytes_freshly_revalidated"] is False for row in proof["rows"])
    assert len(neutral)==17 and len(whole)==19 and not any(name in sys.modules for name in ("numpy","matplotlib","PIL","torch","rclpy","carla"))


@pytest.mark.parametrize("fault",["missing_pub","pub_duplicate","sub_replay","pub_authority","send_receive","direct_neutral","missing_topic","extra_topic","badSHA","pub_frame","sub_stamp","pub_status","source","missing_source","topic_source_pin","CLOCK_origin"])
def test_actual_own_run_receipt_mismatches_DENY_without_drawing(gallery,fault):
    plan,parent,index,rows,clock,neutral,whole=transport_fixture(gallery)
    sub,pub=rows["subscriber_anchors"],rows["publisher_anchors"]
    if fault=="missing_pub":pub.pop()
    elif fault=="pub_duplicate":pub[2]["ordinal"]=1
    elif fault=="sub_replay":sub[2]["replay_id"]="OTHER_SYNTHETIC_RUN"
    elif fault=="pub_authority":pub[2]["authority_sha256"]="b"*64
    elif fault=="send_receive":pub[2]["published_input_CDR_sha256"][whole[-1]]="b"*64
    elif fault=="direct_neutral":rows["direct_anchors"][2]["input_CDR_sha256"][neutral[0]]="b"*64
    elif fault=="missing_topic":sub[2]["actual_received_input_CDR_sha256"].pop(whole[-1])
    elif fault=="extra_topic":pub[2]["published_input_CDR_sha256"]["foreign"]="a"*64
    elif fault=="badSHA":sub[2]["actual_received_input_CDR_sha256"][neutral[0]]="z"*64
    elif fault=="pub_frame":pub[2]["native_frame"]+=1
    elif fault=="sub_stamp":sub[2]["timestamp_ns"]+=1
    elif fault=="pub_status":pub[2]["status"]="UNVERIFIED_STATUS"
    elif fault=="source":pub[2]["source_selected_callback_provenance"]={}
    elif fault=="missing_source":sub[2].pop("source_selected_callback_provenance")
    elif fault=="topic_source_pin":plan["source_sha256"][gallery.BUNDLE]="b"*64
    else:clock.__file__="/foreign/clock.py"
    with pytest.raises(ValueError):gallery.own_run_transport_proof(plan,parent,index,rows,clock)


def test_private_AST_wire_functions_exact_and_join_precedes_any_render_provider(gallery):
    functions={n.name:n for n in ast.parse(SOURCE.read_bytes()).body if isinstance(n,ast.FunctionDef)}
    selected=functions["own_run_transport_proof"];text=ast.unparse(selected)
    expected={"input_topic_contract","validate_CDR_map","own_run_input_receipts","receipt_map_sha"}
    declaration=next(n.value for n in selected.body if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=="wanted")
    assert ast.literal_eval(declaration)==expected
    original=[n for n in ast.parse((ROOT/gallery.CLOCK).read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name in expected]
    assert len(original)==4 and not any(n.decorator_list for n in original)
    assert "functions = [n for n in ast.parse" in text and "body=functions" in text
    assert "frozen = dict(clock.FIXED_SOURCE_PINS)" in text and "ARMS={'V19c': dict(PARENT_PINS)}" in text
    assert "parent_plan['replay_id']" in text and "namespace['own_run_input_receipts']('V19c'" in text
    actual=ast.unparse(functions["execute"])
    assert actual.index("own_run_transport_proof(plan, p, index, rows, clock)")<actual.index("import matplotlib")
    assert "parent_evidence(" not in text and "summarize_arm(" not in text and "clock.ARMS =" not in text


# HH_261008 - Execute only the genuine finalbody AST with fake independent closers; no actual grant, dataset, provider or output write.
@pytest.mark.parametrize("faults",[("close",),("seals",),("CSV",),("runtime",),("census",),("close","CSV"),("runtime","census")])
def test_actual_closing_double_fault_keeps_primary_error_and_full918x2_ledger(gallery,monkeypatch,faults):
    index,rows,topics=index_and_rows();records=gallery.source_records(index,rows,topics)
    for arm in gallery.ARMS:
        for row in records[arm][:918 if arm=="DIRECT" else 29]:row["render_status"]="PASS"
    ledger={arm:[[r["ordinal"],r["render_status"],r["render_error"]] for r in records[arm]] for arm in gallery.ARMS}
    result=dict(status="FAILED_RETAINED",error="SYNTHETIC_PRIOR_RENDER_FAILURE",closing_errors=[],CSV_sha256=None,
        completed_GIF_frames={"DIRECT":918,"ROS":29})
    def fail(kind):
        if kind in faults:raise ValueError("SYNTHETIC_"+kind+"_CLOSING_FAILURE")
    writes=[]
    def write(path,raw,*args,**kwargs):
        if path.name.endswith("csv.gz"):fail("CSV")
        writes.append((path.name,raw));return "a"*64
    def census(_):fail("census");return {"SYNTHETIC_ONLY":"a"*64}
    function=next(n for n in ast.parse(SOURCE.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name=="execute")
    choices=[n for n in ast.walk(function) if isinstance(n,ast.Try) and n.finalbody and any(isinstance(k,ast.Call) and isinstance(k.func,ast.Name) and k.func.id=="check_closing_resources" for statement in n.finalbody for k in ast.walk(statement))]
    assert len(choices)==1
    monkeypatch.setattr(gallery,"current_resources",lambda:dict(pid=12,VmRSS=100,VmHWM=200,inherited_ru_maxrss_bytes_not_gate=10**12))
    namespace=dict(vars(gallery),result=result,records=records,plt=SimpleNamespace(close=lambda _:fail("close")),
        verify=lambda _:fail("seals"),plan=dict(source_sha256={"SYNTHETIC_ONLY":"a"*64},input_sha256={}),
        write_new=write,census=census,budget=lambda **kwargs:{},resource_trace=[],started=gallery.time.monotonic(),
        sys=SimpleNamespace(modules={"torch":object()} if "runtime" in faults else {}))
    exec(compile(ast.Module(body=deepcopy(choices[0].finalbody),type_ignores=[]),str(SOURCE)+":fake_finalbody_only","exec"),namespace)
    assert result["error"]=="SYNTHETIC_PRIOR_RENDER_FAILURE" and result["status"]=="FAILED_RETAINED"
    assert result["whole_render_ledger"]==ledger and result["completed_GIF_frames"]=={"DIRECT":918,"ROS":29}
    assert len(result["closing_errors"])==len(faults) and all(len(part)==918 for part in result["whole_render_ledger"].values())
    assert result["output_sha256_before_result"]==(None if "census" in faults else {"SYNTHETIC_ONLY":"a"*64})
    assert writes[-1][0]=="result.json" and json_result(writes[-1][1])["whole_render_ledger"]==ledger


def json_result(raw):
    import json
    return json.loads(raw)


def test_preserved_V2_failure_and_source_finding_without_actual_render_claim(gallery):
    assert len(gallery.PRESERVED_V2_PINS)==4
    for name,pin in gallery.PRESERVED_V2_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    failure=gallery.read_json(ROOT/next(name for name in gallery.PRESERVED_V2_PINS if "source_test_failure" in name))
    assert failure["actual_pure_passed"]==68 and failure["actual_pure_failed"]==1 and failure["conditional_prepare_executed"] is False
    assert failure["actual_rendered_frames"]==failure["NN_inferences"]==failure["training_steps"]==0
    tables=next(n for n in ast.parse(SOURCE.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name=="tables")
    expected=ast.parse("inputs.update({str(ROOT/name):pin for name,pin in PRESERVED_V2_PINS.items()})").body[0]
    assert sum(ast.dump(n,include_attributes=False)==ast.dump(expected,include_attributes=False) for n in tables.body)==1


def test_all_V2_geometry_science_and_nonclosing_execution_exact_after_only_wire_join(gallery):
    old_path=ROOT/"tools/e2e/report_recorded_vehicle_centered_gallery_20261008_v2.py"
    old={n.name:n for n in ast.parse(old_path.read_bytes()).body if isinstance(n,ast.FunctionDef)}
    new={n.name:n for n in ast.parse(SOURCE.read_bytes()).body if isinstance(n,ast.FunctionDef)}
    assert new.keys()==old.keys()|{"own_run_transport_proof"}
    for name in old.keys()-{"tables","fixed","execute"}:assert ast.dump(new[name],include_attributes=False)==ast.dump(old[name],include_attributes=False),name
    current=deepcopy(new["execute"]);previous=old["execute"]
    wanted=ast.parse('result["whole_own_run_transport_proof"]=own_run_transport_proof(plan,p,index,rows,clock)').body[0]
    removed=[]
    class RemoveExactWire(ast.NodeTransformer):
        def visit_Assign(self,node):
            if ast.dump(node,include_attributes=False)==ast.dump(wanted,include_attributes=False):removed.append(1);return None
            return self.generic_visit(node)
    current=RemoveExactWire().visit(current);assert removed==[1]
    def main_try(function):
        matches=[n for n in ast.walk(function) if isinstance(n,ast.Try) and n.finalbody and any(isinstance(k,ast.Call) and isinstance(k.func,ast.Name) and k.func.id=="check_closing_resources" for statement in n.finalbody for k in ast.walk(statement))]
        assert len(matches)==1;return matches[0]
    # Closing hardening is separately exercised above; all original processing/timers/provider/drawing statements remain byte-structure equivalent.
    main_try(current).finalbody=deepcopy(main_try(previous).finalbody)
    assert ast.dump(current,include_attributes=False)==ast.dump(previous,include_attributes=False)
