"""HH_261008 - Render stored V19c model selections around the recorded rear-axle origin, without inference.

These are two-dimensional RECORD DIAGNOSTICS, not RViz, a new drive, physical
vehicle body geometry or captured public Trajectory/Path coordinates. The old
wire evidence contains hashes, not decoded point values. Only genuine stored
TRAJECTORY selections are drawn; rejected/failed/unexecuted outputs stay absent.
Per-frame capture-control origin remains UNKNOWN. The original capture disclosed
expert startup followed by learned research control, not pure learned driving.
Preparation is not permission: actual metadata/provider work needs externally
SHA-pinned fixed ROOT files and the existing shared lease. Resource guards are
sampled, not OS containment or a hard preemptive deadline.
"""
from __future__ import annotations

import argparse
import ast
import collections
from copy import deepcopy
import csv
import fcntl
import gzip
import hashlib
import importlib.metadata
import importlib.util
from io import BytesIO, StringIO
import json
import math
import os
from pathlib import Path
import resource
import stat
import sys
import time
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve()
SCHEMA="portable_e2e.recorded_vehicle_centered_gallery.20261008.v1"
PLAN_DIR=ROOT/"storage/experiments/plans/2026-10-08/recorded_vehicle_centered_gallery_v1"
OUT=ROOT/"storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v1"
PLAN=PLAN_DIR/"ROOT_plan.json";AUTHORITY=PLAN_DIR/"ROOT_execution_authority.json"
LOCK=ROOT/"storage/assets/locks/autoware_e2e_runtime.lock"
PARENT_PLAN_DIR=ROOT/"storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v19c"
PARENT_OUT=ROOT/"storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v19c"
PARENT_PINS={"plan":"022a86a082f800a0011b64b94a7847ef73df126fa5aa07132a4977e58daec371",
    "authority":"0ae924163bc2f6323a54766091aec731ecbf4e4d013557aea2062474ac995ff9",
    "result":"a0bcdbcae72dc562ad3eafd086eb0b64301f122d678c5c046e7116372bb5cdcb"}
EPISODE=ROOT/"storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v1/episode"
CLOCK="tools/e2e/report_recorded_ros_clock_comparison_20261008_v4.py"
CODEC="tools/e2e/owned_lossless_jsonl_20261008_v2.py"
BUNDLE="learning/portable_e2e/temporal_recorded_ros_bundle_v1_20261005.py"
COLLECTOR="tools/e2e/collect_carla_vad_expert.py"
CONTRACT="learning/portable_e2e/contract.py"
RUNTIME_CONTRACT="learning/portable_e2e/runtime_contract.py"
OLD_ROS="autonomy/autoware_e2e_vad_launch/scripts/portable_e2e_shadow_node.py"
FIXED_PINS={CLOCK:"3cdb6dc2a14706dce6cf9be374ab7cfb2877b5488c7d185454139efc557fb152",
    CODEC:"f867fc18ff6f85038e3ea4dab8fd4b7b50bba84be8655c4db68bd6eab8eff479",
    BUNDLE:"6fa21ab9447bcf73bdd2fd79b0ab72ddd94b531d3a11459334f754fdfbad7d82"}
TOTAL,CLOSING,SOURCES,GIFS,PNG,CSV,CACHE,PLANS,META=32*1024**2,4*1024**2,8*1024**2,12*1024**2,1024**2,2*1024**2,512*1024,1536*1024,512*1024
FLOOR,SHARED,RSS,WALL=2*1024**3,64*1024**2,512*1024**2,900
PNG_ORDINALS=(0,459,917)
ARMS=("DIRECT","ROS")
STATUSES={"PREFILL","REJECTED","TRAJECTORY","REJECTED_ROS_END_TO_END_DEADLINE","FAILED_RETAINED"}
PIL_NAMES=("PIL","PIL._version","PIL.Image","PIL.GifImagePlugin","PIL.PngImagePlugin",
    "PIL.ImageFile","PIL.ImagePalette","PIL.ImageMode","PIL._binary","PIL._imaging")
PIL_FILES={name:"/usr/lib/python3/dist-packages/"+name.replace(".","/")+".py" for name in PIL_NAMES}
PIL_FILES.update({"PIL":"/usr/lib/python3/dist-packages/PIL/__init__.py",
    "PIL._imaging":"/usr/lib/python3/dist-packages/PIL/_imaging.cpython-310-x86_64-linux-gnu.so"})


def need(value,message):
    if not value:raise ValueError(message)


def canonical(path,exists=True):
    path=Path(path)
    need(path.is_absolute() and path==path.resolve(strict=exists) and not any(p.is_symlink() for p in (path,*path.parents)),"canonical nonsymlink required")
    return path


def sha(path):
    path=canonical(path);before=path.stat();need(stat.S_ISREG(before.st_mode) and before.st_nlink==1,"single-link regular input required")
    h=hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda:stream.read(1024**2),b""):h.update(block)
    after=path.stat();need(all(getattr(before,k)==getattr(after,k) for k in ("st_dev","st_ino","st_size","st_mtime_ns","st_ctime_ns")),"sealing input changed")
    return h.hexdigest()


def encode(value):return (json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False)+"\n").encode()


def read_json(path,pin=None,limit=4*1024**2):
    path=canonical(path);need(path.stat().st_size<=limit,"bounded JSON required");raw=path.read_bytes()
    if pin is not None:need(hashlib.sha256(raw).hexdigest()==pin,"RAW JSON differs")
    def unique(items):
        value={}
        for k,v in items:need(k not in value,"duplicate JSON key");value[k]=v
        return value
    value=json.loads(raw,object_pairs_hook=unique,parse_constant=lambda v:need(False,"nonfinite JSON denied"))
    need(type(value) is dict,"metadata object required");return value


def source_module(name,pin):
    path=ROOT/name;need(sha(path)==pin,"source module pin differs")
    spec=importlib.util.spec_from_file_location("_hh_gallery_"+path.stem,path);module=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=module;spec.loader.exec_module(module)
    need(not any(k in sys.modules for k in ("torch","rclpy","carla")),"metadata module loaded a model/ROS runtime")
    return module


def census(root):
    values={}
    for path in sorted(canonical(root).rglob("*")):
        mode=path.lstat().st_mode;need(stat.S_ISDIR(mode) or stat.S_ISREG(mode),"special/symlink artifact denied")
        if stat.S_ISREG(mode):need(path.stat().st_size<=32*1024**2,"bounded original artifact required");values[str(path)]=sha(path)
    return values


def budget(additional=0,closing=False,roots=None):
    need(type(additional) is int and additional>=0,"prospective exact bytes required")
    need(os.statvfs(ROOT).f_bavail*os.statvfs(ROOT).f_frsize>=FLOOR+SHARED+(0 if closing else CLOSING),"unchanged shared floor/reserve denied")
    totals=collections.Counter()
    for root in (PLAN_DIR,OUT) if roots is None else roots:
        root=canonical(root,exists=root.exists())
        if not root.exists():continue
        for path in root.rglob("*"):
            mode=path.lstat().st_mode;need(stat.S_ISDIR(mode) or stat.S_ISREG(mode),"special output denied")
            if not stat.S_ISREG(mode):continue
            need(path.stat().st_nlink==1,"hardlinked output denied");size=path.stat().st_size;totals["all"]+=size
            relative=path.relative_to(root)
            key="source" if root==PLAN_DIR and relative.parts[0]=="sources" else "plan" if root==PLAN_DIR else "gif" if path.suffix==".gif" else "cache" if relative.parts[0]=="matplotlib_cache" else "other"
            totals[key]+=size
    need(totals["source"]<=SOURCES and totals["plan"]<=PLANS and totals["gif"]<=GIFS and totals["cache"]<=CACHE,"component sampled quota exceeded")
    need(totals["all"]+additional<=TOTAL-(0 if closing else CLOSING),"combined32MiB quota/reserve exceeded")
    return dict(totals)


def write_new(path,raw,limit,closing=False):
    need(type(raw) is bytes and len(raw)<=limit,"complete bounded bytes required");canonical(path,exists=False);budget(len(raw),closing)
    with Path(path).open("xb") as stream:need(stream.write(raw)==len(raw),"partial output");stream.flush();os.fsync(stream.fileno())
    return hashlib.sha256(raw).hexdigest()


def parent():
    plan=read_json(PARENT_PLAN_DIR/"ROOT_plan.json",PARENT_PINS["plan"])
    result=read_json(PARENT_OUT/"result.json",PARENT_PINS["result"])
    grant=read_json(PARENT_PLAN_DIR/"ROOT_execution_authority.json",PARENT_PINS["authority"])
    need(plan["schema"]=="portable_e2e.owned_recorded_ROS_shadow.20261008.v19c" and result["source_sha256_after"]==plan["source_sha256"]
        and result["input_sha256_after"]==plan["input_sha256"] and result["cleanup_errors"]==[] and all(c["exit"] is not None for c in result["children"]),"genuine ended V19c required")
    need(grant["plan_path"]==str(PARENT_PLAN_DIR/"ROOT_plan.json") and grant["plan_sha256"]==PARENT_PINS["plan"] and grant["source_sha256"]==plan["source_sha256"],"parent RAW authority join differs")
    return plan


def provider_tables(clock):
    spec=importlib.util.find_spec("PIL")
    need(spec is not None and spec.origin==PIL_FILES["PIL"] and importlib.metadata.version("Pillow")=="9.0.1","selected Pillow origin/version differs")
    return dict(clock=clock.provider_hash(),Pillow_version="9.0.1",Pillow_selected_sha256={p:sha(p) for p in PIL_FILES.values()},
        scope="selected_loaded_entries_config_font_NOT_complete_transitive_environment")


def tables():
    clock=source_module(CLOCK,FIXED_PINS[CLOCK]);p=parent();sources=dict(p["source_sha256"])
    names=(*FIXED_PINS,COLLECTOR,CONTRACT,RUNTIME_CONTRACT,OLD_ROS,str(HERE.relative_to(ROOT)),
        "learning/tests/test_recorded_vehicle_centered_gallery_20261008_v1.py","tools/workspace_python.py","tools/canonical_workspace.py")
    for name in names:
        pin=sha(ROOT/name);need(name not in sources or sources[name]==pin,"parent source conflict");sources[name]=pin
    need(all(sources[name]==pin for name,pin in FIXED_PINS.items()),"fixed scientific reader changed")
    archived={str(PARENT_PLAN_DIR/"sources"/name):pin for name,pin in p["source_sha256"].items()}
    for path,pin in archived.items():need(sha(path)==pin,"original parent source archive differs")
    inputs=census(PARENT_OUT)
    inputs.update({str(PARENT_PLAN_DIR/name):PARENT_PINS[key] for key,name in (("plan","ROOT_plan.json"),("authority","ROOT_execution_authority.json"))})
    for item in p["scalar_inputs"].values():need(sha(item["path"])==item["sha256"],"original scalar input changed");inputs[item["path"]]=item["sha256"]
    for name in ("manifest.json","route.json"):
        path=str(EPISODE/name);pin=p["input_sha256"][path];need(sha(path)==pin,"original capture manifest/route changed");inputs[path]=pin
    return dict(source_sha256=dict(sorted(sources.items())),input_sha256=dict(sorted(inputs.items())),parent_archive_sha256=archived,selected_providers=provider_tables(clock))


def fixed():
    return dict(expected_anchors_per_arm=918,DIRECT_GIF_frames=918,ROS_GIF_actual_prefix_frames=29,ROS_UNEXECUTED=889,
        PNG_source_ordinals=list(PNG_ORDINALS),selected_draw_status="TRAJECTORY_ONLY",captured_public_output_coordinates_available=False,
        map_projection_from_stored_selection_is_source_reconstruction_not_observed_public_coordinates=True,
        native20Hz_past_only=True,source_timestamps_not_runtime_latency_for_animation=True,GIF_centisecond_quantization=True,last_display_delay_ms=0,recorded_interval_loop_not_new_native_frames=True,
        per_frame_old_control_origin="UNKNOWN",capture_origin="disclosed_expert_startup_then_learned_research_no_teacher_fallback_after_handoff",
        ego_marker="recorded_ROS_rear_axle_base_link_origin_no_body_outline",metadata_2D_not_HD_map=True,NN_inferences=0,training_steps=0,
        actual_RViz_screenshot=False,new_drive=False,formal_main_approved=0,formal_sub_approved=0,source_archive_limit=SOURCES,
        GIF_sum_limit=GIFS,PNG_limit_each=PNG,PNG_count=3,CSV_gz_limit=CSV,cache_limit=CACHE,plan_metadata_limit=PLANS,
        combined_limit=TOTAL,closing_reserve=CLOSING,result_limit=META,floor=FLOOR,shared=SHARED,CPU_affinity=[3],threads=1,
        sampled_VmHWM_limit=RSS,sampled_wall_seconds=WALL,OS_containment_or_hard_preemption_claimed=False,
        complete_transitive_provider_environment_claimed=False,original_100ms_threshold_unchanged=True)


def prepare():
    need(not PLAN_DIR.exists() and not OUT.exists(),"fresh candidate only; no retry/overwrite")
    evidence=tables();size=sum((ROOT/n).stat().st_size for n in evidence["source_sha256"])
    plan=dict(schema=SCHEMA,fixed=fixed(),**evidence)
    need(size<=SOURCES and len(encode(plan))*2+512*1024<=PLANS and SOURCES+GIFS+3*PNG+CSV+CACHE+PLANS<=TOTAL-CLOSING,"unchanged prospective32MiB reservations do not fit")
    PLAN_DIR.mkdir(parents=True)
    for name,pin in evidence["source_sha256"].items():
        path=PLAN_DIR/"sources"/name;path.parent.mkdir(parents=True,exist_ok=True);raw=(ROOT/name).read_bytes();need(hashlib.sha256(raw).hexdigest()==pin,"source archive changed")
        write_new(path,raw,SOURCES)
    pin=write_new(PLAN_DIR/"candidate_plan.json",encode(plan),PLANS)
    return dict(status="PREPARED_NOT_AUTHORIZED",candidate_plan_sha256=pin,ROOT_plan=str(PLAN),ROOT_authority=str(AUTHORITY),source_count=len(evidence["source_sha256"]),metadata_inputs=len(evidence["input_sha256"]))


def verify(plan):
    need(plan["schema"]==SCHEMA and plan["fixed"]==fixed(),"fixed prospective gallery protocol differs")
    for name,pin in plan["source_sha256"].items():need(sha(ROOT/name)==pin and sha(PLAN_DIR/"sources"/name)==pin,"current/gallery source archive differs")
    for category in ("input_sha256","parent_archive_sha256"):
        for path,pin in plan[category].items():need(sha(path)==pin,"complete original metadata/source archive differs")
    need(tables()=={k:plan[k] for k in ("source_sha256","input_sha256","parent_archive_sha256","selected_providers")},"complete metadata/provider census changed")


def scalar_geometry(plan):
    # HH_261008 - Extract only genuine scalar functions; never import collector, model, trainer or ROS module bodies.
    namespace=dict(math=math,ContractError=ValueError,WHEELBASE_M=2.85)
    def functions(name,wanted):
        need(sha(ROOT/name)==plan["source_sha256"][name],"scalar original source changed")
        nodes=[p for p in ast.parse((ROOT/name).read_bytes()).body if isinstance(p,ast.FunctionDef) and p.name in wanted]
        need({p.name for p in nodes}==set(wanted) and len(nodes)==len(wanted),"unique original scalar AST required");return nodes
    nodes=functions(COLLECTOR,{"wrap_angle","shift_transform_local_x","base_link_state"})+functions(RUNTIME_CONTRACT,{"_finite_number","transform_base_trajectory_to_map"})+functions(OLD_ROS,{"quaternion_yaw"})
    route=functions(CONTRACT,{"_canonical_route_in_base"})[0]
    first=next(i for i,p in enumerate(route.body) if isinstance(p,ast.Assign) and ast.unparse(p.targets[0])=="(qx, qy, qz, qw)")
    last=next(i for i,p in enumerate(route.body) if isinstance(p,ast.Assign) and ast.unparse(p.targets[0])=="(ego_x, ego_y)")
    inner=[p for p in route.body if isinstance(p,ast.FunctionDef) and p.name=="to_base"];need(len(inner)==1 and first<last,"original map-to-base scalar seam differs")
    factory=ast.parse("def map_to_base(position_m,orientation_xyzw,context): pass").body[0]
    factory.body=deepcopy(route.body[first:last+1])+deepcopy(inner)+[ast.Return(value=ast.Name(id="to_base",ctx=ast.Load()))]
    module=ast.Module(body=[ast.ImportFrom(module="__future__",names=[ast.alias(name="annotations")],level=0),*nodes,factory],type_ignores=[])
    exec(compile(ast.fix_missing_locations(module),str(HERE)+":genuine_scalar_AST_only","exec"),namespace)
    return namespace


def current_resources():
    values={}
    for line in Path("/proc/self/status").read_text().splitlines():
        fields=line.split()
        if fields and fields[0] in ("VmRSS:","VmHWM:"):
            key=fields[0][:-1];need(key not in values and len(fields)==3 and fields[1].isdigit() and fields[2]=="kB","strict current process memory units")
            values[key]=int(fields[1])*1024
    need(set(values)=={"VmRSS","VmHWM"} and 0<values["VmRSS"]<=values["VmHWM"],"current process memory observation missing")
    values.update(pid=os.getpid(),inherited_ru_maxrss_bytes_not_gate=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024)
    return values


def guard(started,closing=False,trace=None):
    observed=current_resources()
    if trace is not None:
        need(len(trace)<2048,"bounded sampled resource journal exceeded")
        trace.append([time.monotonic_ns(),observed["pid"],observed["VmRSS"],observed["VmHWM"],observed["inherited_ru_maxrss_bytes_not_gate"]])
    need(observed["VmHWM"]<=RSS,"sampled current process512MiB exceeded")
    need(time.monotonic()-started<=WALL,"sampled900s exceeded, not a hard preemption");budget(closing=closing);return observed


def stored_selection(row):
    if row["status"]!="TRAJECTORY":return None
    selected=row.get("selected");need(type(selected) is dict and set(selected)=={"candidate_index","xy_base_m","speed_mps","timestep_s","logit_margin","planar_extent_m","heading_rad"},"authentic complete stored selection required")
    need(type(selected["candidate_index"]) is int and selected["candidate_index"]>=0 and selected["timestep_s"]==.1,"stored selected ABI differs")
    for key,width in (("xy_base_m",2),("speed_mps",None),("heading_rad",None)):
        value=selected[key];need(type(value) is list and len(value)==64,"stored64 points/speeds/headings required")
        if width:need(all(type(point) is list and len(point)==width for point in value),"stored XY shape differs")
        numbers=[n for point in value for n in point] if width else value
        need(all(type(n) in (int,float) and math.isfinite(n) for n in numbers),"stored selected values must be finite")
    for key in ("logit_margin","planar_extent_m"):
        value=selected[key];need((key=="logit_margin" and value is None) or type(value) in (int,float) and math.isfinite(value),"stored selected scalar differs")
    return selected


def source_records(index,rows,neutral_topics):
    direct,sub=rows["direct_anchors"],rows["subscriber_anchors"]
    need(len(index.anchors)==len(direct)==918 and len(sub)==29,"complete original918/direct29ROS expected")
    result={}
    for arm,observed in (("DIRECT",direct),("ROS",sub)):
        need([r["ordinal"] for r in observed]==list(range(len(observed))),"ordered original prefix required")
        values=[]
        for anchor in index.anchors:
            record=dict(arm=arm,ordinal=anchor.ordinal,native_frame=anchor.native_frame,timestamp_ns=anchor.timestamp_ns,status="UNEXECUTED",
                stored_selection_sha256=None,selected_points=0,render_status="UNEXECUTED",render_error=None,captured_control_origin="UNKNOWN",
                source_provenance_sha256=hashlib.sha256(encode(anchor.provenance)).hexdigest(),source_join="ORIGINAL_INDEX_ONLY_UNEXECUTED",GIF_delay_ms=None)
            if anchor.ordinal<len(observed):
                row=observed[anchor.ordinal];need(row["native_frame"]==anchor.native_frame and row["timestamp_ns"]==anchor.timestamp_ns and row["status"] in STATUSES,"actual row source/status differs")
                provenance=row.get("source_selected_callback_provenance")
                if provenance is None and arm=="DIRECT":
                    pins=row.get("input_CDR_sha256")
                    need(type(pins) is dict and set(pins)==set(neutral_topics) and len(neutral_topics)==17 and row.get("direct_outputs_published") is False
                        and all(type(pin) is str and len(pin)==64 and all(c in "0123456789abcdef" for c in pin) for pin in pins.values()),"genuine source-bound direct17 CDR receipts required")
                    record["source_join"]="ORIGINAL_DIRECT_PRODUCER_SOURCE_WITNESS_INDEX_AND_17CDR_RECEIPTS"
                elif provenance is None:need(row["status"]=="FAILED_RETAINED","nonfailed row missing actual source");record["source_join"]="UNKNOWN_FAILED_ORIGINAL_ADMISSION"
                else:need(encode(provenance)==encode(anchor.provenance),"actual source provenance differs");record["source_join"]="ACTUAL_STORED_RECEIPT_INDEX_JOIN"
                selected=stored_selection(row);record["status"]=row["status"]
                if selected is not None:record.update(stored_selection_sha256=hashlib.sha256(encode(selected)).hexdigest(),selected_points=64)
                record["GIF_delay_ms"]=gif_delay(index.anchors,anchor.ordinal,len(observed))
            values.append(record)
        result[arm]=values
    return result


def gif_delay(anchors,ordinal,count):
    if ordinal==count-1:return 0
    delta=anchors[ordinal+1].timestamp_ns-anchors[ordinal].timestamp_ns
    need(type(delta) is int and 99_999_000<=delta<=100_001_000,"recorded10Hz timestamps required, no latency-based animation")
    return int(round(delta/10_000_000))*10


def verify_native_geometry(states,manifest,geometry):
    contract=manifest["coordinate_contract"]
    need(contract["map_frame"]=="ROS map: x forward/east, y left, z up" and contract["ego_frame"]=="ROS rear-axle base_link: x forward, y left, z up"
        and contract["wheelbase_m"]==2.85,"actual original ROS/rear-axle geometry required")
    capture=manifest["capture_contract"]["learned_research"]
    need(capture["expert_startup_until_vx_above_mps"]==.5 and capture["no_teacher_fallback_after_handoff"] is True,"genuine mixed capture disclosure required")
    need(len(states)==1836 and all(b["frame"]==a["frame"]+1 and abs(b["timestamp"]-a["timestamp"]-.05)<=1e-6 for a,b in zip(states,states[1:])),"entire original native20Hz chain required")
    maximum=0.
    for row in states:
        pose=row["actor_snapshot_transform_carla"];need(set(pose)=={"x","y","z","roll","pitch","yaw"},"actual raw pose fields required")
        vectors=[row[k] for k in ("world_velocity_carla","world_acceleration_carla","world_angular_velocity_carla_deg_s")]
        need(all(type(v) in (list,tuple) and len(v)==3 for v in vectors),"exact three-coordinate native vectors required")
        raw=[*pose.values(),*(n for vector in vectors for n in vector)]
        need(all(type(v) in (int,float) and math.isfinite(v) for v in raw),"actual raw native values required")
        rebuilt=geometry["base_link_state"](pose,row["world_velocity_carla"],row["world_acceleration_carla"],row["world_angular_velocity_carla_deg_s"][2],2.85)
        errors=[abs(row[k]-v) for k,v in rebuilt.items()];maximum=max(maximum,*errors)
        need(max(errors)<=1e-9,"unchanged original snapshot-to-ROS agreement screen failed")
    return dict(native_states=1836,maximum_original_conversion_error=maximum,agreement_tolerance=1e-9,additional_y_sign_flip=False,
        capture_origin=fixed()["capture_origin"],per_frame_control_origin="UNKNOWN",physical_actuation_proof=False)


def scene(plt):
    fig,(local,overview)=plt.subplots(1,2,figsize=(8,4),dpi=100)
    local.set(xlim=(-24,24),ylim=(-24,24),xlabel="Recorded rear-axle forward x (m)",ylabel="left y (m)")
    local.set_aspect("equal");local.grid(alpha=.2);local.plot([0],[0],"ko",markersize=5,label="Recorded origin (no body outline)");local.arrow(0,0,2,0,width=.05,color="black")
    artists=[]
    for ax in (local,overview):
        artists.append(tuple(ax.plot([],[],color=c,lw=w,label=label)[0] for c,w,label in (("#777777",1,"Original planned navigation"),("#2868a8",1.5,"Measured causal traveled prefix"),("#c93636",1.6,"Stored model selection (not public coords)"))))
        ax.set_aspect("equal",adjustable="box");ax.legend(fontsize=5,loc="lower left")
    ego,=overview.plot([],[],"ko",markersize=4);overview.set(xlabel="Original ROS map x (m)",ylabel="Original ROS map y (m)")
    caption=fig.suptitle("",fontsize=8);fig.text(.02,.01,"RECORD DIAGNOSTIC / not RViz, new drive, future GT or measured body. Capture-control per frame UNKNOWN.",fontsize=6)
    fig.subplots_adjust(top=.79,bottom=.17,wspace=.35)
    return fig,local,overview,artists,ego,caption


def draw(scene_value,arm,anchor,row,states,route,geometry):
    fig,local,overview,artists,ego,caption=scene_value;state=anchor.state
    q=(0.,0.,math.sin(state["yaw"]/2),math.cos(state["yaw"]/2));to_base=geometry["map_to_base"]((state["x"],state["y"],state["z"]),q,"recorded_gallery")
    past=[(r["x"],r["y"]) for r in states if r["frame"]<=anchor.native_frame]
    selected=stored_selection(row);base=[] if selected is None else selected["xy_base_m"]
    yaw=geometry["quaternion_yaw"](SimpleNamespace(x=q[0],y=q[1],z=q[2],w=q[3]))
    mapped=[] if selected is None else geometry["transform_base_trajectory_to_map"](SimpleNamespace(**selected),ego_x_m=state["x"],ego_y_m=state["y"],ego_yaw_rad=yaw)
    for ax,groups,lines in ((local,([to_base(p) for p in route],[to_base(p) for p in past],base),artists[0]),(overview,(route,past,mapped),artists[1])):
        for points,line in zip(groups,lines):line.set_data([p[0] for p in points],[p[1] for p in points])
    xs=[p[0] for p in route];ys=[p[1] for p in route];overview.set_xlim(min(xs)-5,max(xs)+5);overview.set_ylim(min(ys)-5,max(ys)+5);ego.set_data([state["x"]],[state["y"]])
    text="NONE" if selected is None else "64 stored points; speed "+format(min(selected["speed_mps"]),".2f")+".."+format(max(selected["speed_mps"]),".2f")+"m/s"
    caption.set_text(arm+" ordinal "+str(anchor.ordinal)+" /918 | native "+str(anchor.native_frame)+" | recorded ns "+str(anchor.timestamp_ns)+"\n"+row["status"]+" | model path "+text+" | public point values NOT archived")
    fig.canvas.draw();return fig


def append_gif_frame(stream,image,plugin,delay,first):
    # HH_261008 - Emit one owned rendered image; never Pillow save_all's accumulating im_frames or original image bytes.
    blocks=[]
    if first:blocks.extend(plugin.getheader(image,info={"loop":0,"optimize":False})[0])
    params=dict(duration=delay,disposal=2,include_color_table=True,optimize=False)
    if first:params["loop"]=0
    blocks.extend(plugin.getdata(image,**params))
    raw=b"".join(blocks);need(len(raw)<=GIFS,"single encoded diagnostic frame exceeds GIF sum bound")
    used=budget(len(raw));need(used.get("gif",0)+len(raw)<=GIFS,"prospective two-GIF sum bound exceeded before write")
    need(stream.write(raw)==len(raw),"partial GIF frame");stream.flush();need(stream.tell()<=GIFS,"GIF file exceeds sum bound");budget()
    return len(raw)


def csv_bytes(records):
    values=[r for arm in ARMS for r in records[arm]];need(len(values)==1836,"full2x918 CSV denominator required")
    out=StringIO(newline="");writer=csv.DictWriter(out,fieldnames=list(values[0]));writer.writeheader();writer.writerows(values)
    return gzip.compress(out.getvalue().encode(),mtime=0)


def execute(plan_sha,authority_sha):
    started=time.monotonic();plan=read_json(PLAN,plan_sha,PLANS);grant=read_json(AUTHORITY,authority_sha,PLANS)
    need(grant==dict(schema=SCHEMA+".ROOT_authority",plan_path=str(PLAN),plan_sha256=plan_sha,source_sha256=plan["source_sha256"],selected_providers=plan["selected_providers"],
        stored_scalar_gallery_only=True,NN_authorized=False,training_authorized=False,GPU_authorized=False,ROS_authorized=False,CARLA_authorized=False,TEST_authorized=False),"external exact ROOT permission required")
    for k,v in {"CUDA_VISIBLE_DEVICES":"","PYTHONNOUSERSITE":"1","PYTHONDONTWRITEBYTECODE":"1","OMP_NUM_THREADS":"1","MKL_NUM_THREADS":"1","OPENBLAS_NUM_THREADS":"1"}.items():need(os.environ.get(k)==v,"clean single-thread environment required")
    need(not any(k in sys.modules for k in ("numpy","matplotlib","PIL","torch","rclpy","carla")),"fresh pre-framework process required")
    for k in ("PYTHONPATH","PYTHONHOME","MPLCONFIGDIR","MATPLOTLIBRC","MATPLOTLIBDATA","MPLBACKEND"):need(not os.environ.get(k),"unsealed provider environment denied")
    need(not (Path.cwd()/"matplotlibrc").exists() and 3 in os.sched_getaffinity(0),"unsealed plot config/CPU3 unavailable")
    with LOCK.open("r") as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);verify(plan);guard(started);need(not OUT.exists(),"fresh output only");OUT.mkdir(parents=True);os.sched_setaffinity(0,{3})
        result=dict(schema=SCHEMA+".result",status="FAILED_RETAINED",error=None,closing_errors=[],expected_status_rows=1836,completed_GIF_frames={a:0 for a in ARMS},PNG_sha256={},GIF_sha256={},CSV_sha256=None,NN_inferences=0,training_steps=0,model_performance_improved=False)
        records=None;cursor=None;plt=None;resource_trace=[]
        try:
            clock=source_module(CLOCK,plan["source_sha256"][CLOCK]);codec=source_module(CODEC,plan["source_sha256"][CODEC]);bundle=source_module(BUNDLE,plan["source_sha256"][BUNDLE])
            p=parent();index=bundle.load_index(p);rows,proof=clock.read_ledgers(PARENT_OUT,codec);records=source_records(index,rows,clock.input_topic_contract()["neutral17"])
            for name,file in clock.LOG_NAMES.items():need(codec.verify_ledger(PARENT_OUT/(file+".lossless"),**codec.prospective_capacity()["components"][name])["original_stream_sha256"]==proof[name]["original_stream_sha256"],"whole reader verification differs")
            states=[bundle.decode(line,256*1024) for line in Path(p["scalar_inputs"]["states"]["path"]).read_bytes().splitlines()]
            geometry=scalar_geometry(plan);result["native_geometry"]=verify_native_geometry(states,read_json(EPISODE/"manifest.json"),geometry);result["whole8_codec_reader_proof"]=proof
            guard(started,trace=resource_trace);os.environ.update(MPLCONFIGDIR=str(OUT/"matplotlib_cache"),MATPLOTLIBRC=str(clock.PROVIDER_CONFIG),MPLBACKEND="Agg")
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from matplotlib import font_manager
            from matplotlib.backends import backend_agg
            from PIL import Image, GifImagePlugin, PngImagePlugin
            result["selected_clock_provider_receipt"]=clock.imported_provider_receipt(dict(provider_hash=plan["selected_providers"]["clock"]),plt,font_manager)
            for name,path in PIL_FILES.items():need(getattr(sys.modules.get(name),"__file__",None)==path,"loaded selected Pillow origin differs")
            need(sys.modules["PIL"].__version__==plan["selected_providers"]["Pillow_version"] and provider_tables(clock)==plan["selected_providers"],"actual selected Pillow/config/font seals differ")
            result["loaded_Pillow_selected_origins"]=dict(PIL_FILES)
            for arm,observed in (("DIRECT",rows["direct_anchors"]),("ROS",rows["subscriber_anchors"])):
                view=scene(plt);path=OUT/(arm+"_whole_actual_interval.gif")
                with path.open("xb") as stream:
                    for ordinal,row in enumerate(observed):
                        cursor=(arm,ordinal);record=records[arm][ordinal];record["resources_before"]=None
                        sample=guard(started,trace=resource_trace);fig=draw(view,arm,index.anchors[ordinal],row,states,index.context["route_xy_map_m"],geometry)
                        image=Image.frombytes("RGB",fig.canvas.get_width_height(),fig.canvas.tostring_rgb()).convert("P",palette=Image.ADAPTIVE,colors=64)
                        append_gif_frame(stream,image,GifImagePlugin,record["GIF_delay_ms"],ordinal==0);image.close();record.pop("resources_before")
                        record["render_status"]="PASS";result["completed_GIF_frames"][arm]+=1;result["last_resources"]=sample;guard(started,trace=resource_trace)
                    used=budget(1);need(used.get("gif",0)+1<=GIFS,"GIF sum trailer reserve denied before write")
                    need(stream.write(b";")==1,"GIF trailer partial");stream.flush();os.fsync(stream.fileno())
                plt.close(view[0]);result["GIF_sha256"][arm]=sha(path)
            for ordinal in PNG_ORDINALS:
                cursor=("PNG",ordinal)
                # HH_261008 - Fixed source positions are rendered independently, including ROS UNEXECUTED; never select successful best frames.
                view=scene(plt);draw(view,"DIRECT",index.anchors[ordinal],rows["direct_anchors"][ordinal],states,index.context["route_xy_map_m"],geometry)
                ROS=records["ROS"][ordinal];view[-1].set_text(view[-1].get_text()+"\nROS same source: "+ROS["status"]+"; captured control per frame UNKNOWN")
                buffer=BytesIO();view[0].savefig(buffer,format="png",dpi=120);plt.close(view[0]);result["PNG_sha256"][str(ordinal)]=write_new(OUT/("source_"+str(ordinal).zfill(4)+"_DIRECT_ROS_status.png"),buffer.getvalue(),PNG);guard(started,trace=resource_trace)
            need(result["completed_GIF_frames"]=={"DIRECT":918,"ROS":29},"whole918/29 actual GIF denominator required")
            result["status"]="COMPLETE_RECORD_DIAGNOSTIC_NOT_PUBLIC_COORDINATE_OR_FEATURE_PROOF"
        except BaseException as failure:
            result["error"]=repr(failure)[:2048]
            if records is not None and cursor is not None and cursor[0] in ARMS:
                records[cursor[0]][cursor[1]]["render_status"]="FAILED_RETAINED";records[cursor[0]][cursor[1]]["render_error"]=repr(failure)[:256]
        finally:
            if plt is not None:plt.close("all")
            try:verify(plan)
            except BaseException as failure:result["closing_errors"].append(repr(failure)[:2048]);result["status"]="FAILED_RETAINED"
            try:
                if records is not None:
                    for arm in ARMS:
                        for row in records[arm]:row.pop("resources_before",None)
                    result["CSV_sha256"]=write_new(OUT/"complete918_per_arm_status.csv.gz",csv_bytes(records),CSV,closing=True)
                    result["status_counts"]={a:dict(collections.Counter(r["status"] for r in records[a])) for a in ARMS}
                    result["whole_render_ledger"]={a:[[r["ordinal"],r["render_status"],r["render_error"]] for r in records[a]] for a in ARMS}
                else:result["whole_render_ledger"]={a:[[n,"UNEXECUTED",None] for n in range(918)] for a in ARMS}
            except BaseException as failure:result["closing_errors"].append("complete CSV/ledger: "+repr(failure)[:2048]);result["status"]="FAILED_RETAINED"
            need(not any(k in sys.modules for k in ("torch","rclpy","carla")),"no model/ROS/CARLA imports")
            result.update(fixed=fixed(),source_sha256=plan["source_sha256"],metadata_input_sha256=plan["input_sha256"],sampled_resources_closing=current_resources(),
                sampled_resources=resource_trace,sampled_resource_columns=["monotonic_ns","pid","VmRSS_bytes","VmHWM_bytes","inherited_ru_maxrss_bytes_not_gate"],
                output_sha256_before_result=census(OUT),
                wall_seconds=time.monotonic()-started,wall_scope="execute_entry_through_closing_metadata_before_result_write",formal_main_approved=0,formal_sub_approved=0,
                captured_public_coordinates_available=False,actual_RViz_screenshot=False,causal_or_sustained_gain_claimed=False)
            write_new(OUT/"result.json",encode(result),META,closing=True);budget(closing=True)
        return dict(status=result["status"],output=str(OUT),completed_GIF_frames=result["completed_GIF_frames"])


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);parser.add_argument("--mode",choices=("prepare","execute"),required=True)
    parser.add_argument("--plan-sha256");parser.add_argument("--authority-sha256");args=parser.parse_args(argv)
    if args.mode=="prepare":
        with LOCK.open("r") as lock:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);value=prepare()
    else:need(args.plan_sha256 and args.authority_sha256,"external ROOT RAW SHA arguments required");value=execute(args.plan_sha256,args.authority_sha256)
    print(json.dumps(value,sort_keys=True));return 2 if value["status"]=="FAILED_RETAINED" else 0


if __name__=="__main__":raise SystemExit(main())
