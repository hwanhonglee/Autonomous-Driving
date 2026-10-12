"""HH_261008 - Scalar/AST/fake-buffer gallery tests only; no actual records, NumPy, Pillow, plotting or NN.

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
SOURCE=ROOT/"tools/e2e/report_recorded_vehicle_centered_gallery_20261008_v1.py"


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
