"""HH_261008 - Probe exactly five recorded-source GIF encodings, never the full interval or any NN.
A separate ROOT grant is mandatory; a positive sample is only a byte-size estimate.
No frame dropping, criteria change, real driving or inherited full-render authority.
"""
from __future__ import annotations
import argparse
import collections
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import stat
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve()
GALLERY=ROOT/"tools/e2e/report_recorded_vehicle_centered_gallery_20261008_v7.py"
SCHEMA="portable_e2e.recorded_gallery_encoding_probe.20261008.v1"
PD=ROOT/"storage/experiments/plans/2026-10-08/recorded_gallery_encoding_probe_v1"
OUT=ROOT/"storage/experiments/diagnostics/2026-10-08/recorded_gallery_encoding_probe_v1"
PLAN=PD/"ROOT_plan.json";AUTH=PD/"ROOT_execution_authority.json"
V6PD=ROOT/"storage/experiments/plans/2026-10-08/recorded_vehicle_centered_gallery_v6"
SLOTS=(("DIRECT",0),("DIRECT",459),("DIRECT",917),("ROS",0),("ROS",28))
TOTAL,CLOSING,SOURCES,PLANS,CACHE,IMAGE,META=2*1024**2,512*1024,512*1024,384*1024,256*1024,64*1024,128*1024
FLOOR,SHARED,RSS,WALL=2*1024**3,64*1024**2,512*1024**2,300
SOURCE_NAMES=(str(GALLERY.relative_to(ROOT)),str(HERE.relative_to(ROOT)),
    "learning/tests/test_recorded_vehicle_centered_gallery_20261008_v7.py",
    "learning/tests/test_recorded_gallery_gif_encoding_probe_20261008_v1.py")

def need(value,message):
    if not value:raise ValueError(message)

def encode(value):return (json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False)+"\n").encode()

def load_gallery(pin=None):
    need(HERE==ROOT/"tools/e2e/probe_recorded_gallery_gif_encoding_20261008_v1.py" and HERE==HERE.resolve(strict=True),"exact probe source required")
    raw=GALLERY.read_bytes();need(pin is None or hashlib.sha256(raw).hexdigest()==pin,"RAW V7 helper source differs")
    spec=importlib.util.spec_from_file_location("_hh_probe_v7_gallery",GALLERY);g=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=g;spec.loader.exec_module(g)
    need(not any(name in sys.modules for name in ("numpy","matplotlib","PIL","torch","rclpy","carla")),"fresh stdlib source process required")
    return g

def budget(g,additional=0,closing=False):
    need(type(additional) is int and additional>=0,"prospective byte count required")
    v=os.statvfs(ROOT);need(v.f_bavail*v.f_frsize>=FLOOR+SHARED+(0 if closing else CLOSING),"unchanged disk floor/reserve denied")
    totals=collections.Counter()
    for root in (PD,OUT):
        if not root.exists():continue
        g.canonical(root)
        for path in root.rglob("*"):
            mode=path.lstat().st_mode;need(stat.S_ISDIR(mode) or stat.S_ISREG(mode),"special probe output denied")
            if not stat.S_ISREG(mode):continue
            need(path.stat().st_nlink==1,"probe hardlink denied");size=path.stat().st_size;totals["all"]+=size
            relative=path.relative_to(root)
            kind="sources" if root==PD and relative.parts[0]=="sources" else "plans" if root==PD else "cache" if relative.parts[0]=="matplotlib_cache" else "images" if path.suffix==".gif" else "meta"
            totals[kind]+=size
    need(totals["sources"]<=SOURCES and totals["plans"]<=PLANS and totals["cache"]<=CACHE and totals["images"]<=5*IMAGE,"component probe quota exceeded")
    need(totals["all"]+additional<=TOTAL-(0 if closing else CLOSING),"whole2MiB probe quota/reserve exceeded")
    return dict(totals)

def write_new(g,path,raw,limit,closing=False):
    need(type(raw) is bytes and len(raw)<=limit,"complete bounded probe bytes required");g.canonical(path,exists=False);budget(g,len(raw),closing)
    with Path(path).open("xb") as stream:need(stream.write(raw)==len(raw),"partial probe bytes");stream.flush();os.fsync(stream.fileno())
    return hashlib.sha256(raw).hexdigest()

def fixed():
    return dict(slots=[list(slot) for slot in SLOTS],source_canvas_size_px=[800,400],GIF_size_px=[600,300],GIF_palette_colors=16,
        same_V7_draw_and_encoding_helpers=True,sample_only=True,full947_render_authorized=False,source_frame_dropping=False,
        encoded_sample_envelope_is_not_future_worstcase_proof=True,NN_inferences=0,training_steps=0,GPU=False,ROS=False,CARLA=False,
        actual_RViz_screenshot=False,new_drive=False,formal_main_approved=0,formal_sub_approved=0,
        combined_limit=TOTAL,closing_reserve=CLOSING,source_copy_limit=SOURCES,plan_metadata_limit=PLANS,
        selected_single_GIF_limit=IMAGE,cache_limit=CACHE,result_limit=META,CPU_affinity=[3],threads=1,
        sampled_VmHWM_limit=RSS,sampled_wall_seconds=WALL,OS_containment_or_hard_preemption_claimed=False)

def mapping(g,sources):
    # HH_261008 - Reuse all173 genuine V6 archived sources; copy only exact new/history names, never alias or invent a V7 full-render grant.
    old=g.read_json(V6PD/"ROOT_plan.json",g.PRESERVED_V6_PINS[str((V6PD/"ROOT_plan.json").relative_to(ROOT))])
    result=g.read_json(ROOT/"storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v6/result.json",
        g.PRESERVED_V6_PINS["storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v6/result.json"])
    need(len(old["source_sha256"])==173 and result["source_sha256"]==old["source_sha256"] and result["closing_errors"]==[],"genuine failed V6 final source closure required")
    previous=old["source_sha256"];need(set(previous)<=set(sources) and all(sources[n]==pin for n,pin in previous.items()),"complete V6 membership changed")
    extra=set(sources)-set(previous);need(set(SOURCE_NAMES)<=extra and len(extra)==10,"exact173 ancestors plus10 fresh/history sources required")
    result={name:str((PD if name in extra else V6PD)/"sources"/name) for name in sources}
    for name in previous:need(g.sha(result[name])==sources[name],"genuine reused V6 archive differs")
    return result,extra

def prepare(g):
    need(not PD.exists() and not OUT.exists(),"fresh probe candidate only")
    evidence=g.tables();archives,extra=mapping(g,evidence["source_sha256"])
    plan=dict(schema=SCHEMA,fixed=fixed(),source_archive_path=archives,**evidence)
    needed=sum((ROOT/n).stat().st_size for n in extra)
    need(needed<=SOURCES and 2*len(encode(plan))+64*1024<=PLANS,"prospective probe copies/plan do not fit")
    need(SOURCES+PLANS+CACHE+5*IMAGE<=TOTAL-CLOSING,"prospective fixed probe reservations do not fit")
    PD.mkdir(parents=True)
    for name in sorted(extra):
        target=Path(archives[name]);target.parent.mkdir(parents=True,exist_ok=True);raw=(ROOT/name).read_bytes()
        need(hashlib.sha256(raw).hexdigest()==evidence["source_sha256"][name],"fresh probe source changed")
        write_new(g,target,raw,SOURCES)
    pin=write_new(g,PD/"candidate_plan.json",encode(plan),PLANS)
    return dict(status="PREPARED_NOT_AUTHORIZED",candidate_plan_sha256=pin,full_render_permission=False)

def verify(g,plan):
    need(plan["schema"]==SCHEMA and plan["fixed"]==fixed(),"fixed five-row probe declaration differs")
    need(g.tables()=={k:plan[k] for k in ("source_sha256","input_sha256","parent_archive_sha256","selected_providers")},"whole metadata/provider table drift")
    archives,_=mapping(g,plan["source_sha256"]);need(archives==plan["source_archive_path"],"exact mixed source archive map differs")
    for name,pin in plan["source_sha256"].items():need(g.sha(ROOT/name)==pin and g.sha(archives[name])==pin,"whole current/archive source changed")

def guard(g,started,closing=False):
    observed=g.current_resources()
    need(observed["VmHWM"]<=RSS and time.monotonic()-started<=WALL,"sampled probe memory/wall denied")
    budget(g,closing=closing);return observed

def sample_estimate(records):
    passed=[r["encoded_file_bytes"] for r in records if r["status"]=="PASS"]
    return dict(observed_complete_samples=len(passed),minimum_bytes=min(passed) if passed else None,
        mean_bytes=sum(passed)/len(passed) if passed else None,maximum_bytes=max(passed) if passed else None,
        sample_max_times947_bytes=max(passed)*947 if passed else None,full_GIF_sum_cap_bytes=12*1024**2,
        estimate_only_NOT_guarantee_or_permission=True,full947_verified=False)

def execute(g,plan_sha,authority_sha):
    started=time.monotonic();plan=g.read_json(PLAN,plan_sha,PLANS);grant=g.read_json(AUTH,authority_sha,PLANS)
    need(grant==dict(schema=SCHEMA+".ROOT_authority",plan_path=str(PLAN),plan_sha256=plan_sha,source_sha256=plan["source_sha256"],
        selected_providers=plan["selected_providers"],five_source_rows_only=True,full_render_authorized=False,
        NN_authorized=False,training_authorized=False,GPU_authorized=False,ROS_authorized=False,CARLA_authorized=False,TEST_authorized=False),"exact separate ROOT probe permission required")
    for k,v in {"CUDA_VISIBLE_DEVICES":"","PYTHONNOUSERSITE":"1","PYTHONDONTWRITEBYTECODE":"1","OMP_NUM_THREADS":"1","MKL_NUM_THREADS":"1","OPENBLAS_NUM_THREADS":"1"}.items():need(os.environ.get(k)==v,"clean one-thread probe environment required")
    for k in ("PYTHONPATH","PYTHONHOME","MPLCONFIGDIR","MATPLOTLIBRC","MATPLOTLIBDATA","MPLBACKEND"):need(not os.environ.get(k),"unsealed probe provider environment denied")
    need(not (Path.cwd()/"matplotlibrc").exists() and 3 in os.sched_getaffinity(0),"CPU3/config denied")
    with g.LOCK.open("r") as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);verify(g,plan);guard(g,started);need(not OUT.exists(),"fresh actual probe only")
        OUT.mkdir(parents=True);os.sched_setaffinity(0,{3})
        records=[dict(arm=a,ordinal=i,status="UNEXECUTED",source_status=None,native_frame=None,timestamp_ns=None,
            GIF_delay_ms=None,encoded_file_bytes=None,encoded_sha256=None,error=None) for a,i in SLOTS]
        result=dict(schema=SCHEMA+".result",status="FAILED_RETAINED",error=None,closing_errors=[],expected_samples=5,samples=records,NN_inferences=0,training_steps=0)
        cursor=None;plt=None
        try:
            clock=g.source_module(g.CLOCK,plan["source_sha256"][g.CLOCK]);codec=g.source_module(g.CODEC,plan["source_sha256"][g.CODEC]);bundle=g.source_module(g.BUNDLE,plan["source_sha256"][g.BUNDLE])
            p=g.parent();index=bundle.load_index(p);rows,proof=clock.read_ledgers(g.PARENT_OUT,codec)
            result["whole8_codec_reader_proof"]=proof;result["whole_own_run_transport_proof"]=g.own_run_transport_proof(plan,p,index,rows,clock)
            source=g.source_records(index,rows,clock.input_topic_contract()["neutral17"])
            states=[bundle.decode(line,256*1024) for line in Path(p["scalar_inputs"]["states"]["path"]).read_bytes().splitlines()]
            geometry=g.scalar_geometry(plan);result["native_geometry"]=g.verify_native_geometry(states,g.read_json(g.EPISODE/"manifest.json"),geometry)
            guard(g,started);os.environ.update(MPLCONFIGDIR=str(OUT/"matplotlib_cache"),MATPLOTLIBRC=str(clock.PROVIDER_CONFIG),MPLBACKEND="Agg")
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from matplotlib import font_manager
            from matplotlib.backends import backend_agg
            from PIL import Image,GifImagePlugin,PngImagePlugin
            result["selected_clock_provider_receipt"]=clock.imported_provider_receipt(dict(provider_hash=plan["selected_providers"]["clock"]),plt,font_manager)
            for name,path in g.PIL_FILES.items():need(getattr(sys.modules.get(name),"__file__",None)==path,"actual selected Pillow origin differs")
            need(sys.modules["PIL"].__version__==plan["selected_providers"]["Pillow_version"] and g.provider_tables(clock)==plan["selected_providers"],"selected provider seals differ")
            for cursor,record in enumerate(records):
                arm,ordinal=record["arm"],record["ordinal"];original=source[arm][ordinal];anchor=index.anchors[ordinal]
                record.update(source_status=original["status"],native_frame=anchor.native_frame,timestamp_ns=anchor.timestamp_ns,GIF_delay_ms=original["GIF_delay_ms"])
                row=rows["direct_anchors" if arm=="DIRECT" else "subscriber_anchors"][ordinal]
                view=g.scene(plt);image=None
                try:
                    g.draw(view,arm,anchor,row,states,index.context["route_xy_map_m"],geometry)
                    image=g.gif_palette_frame(view[0],Image)
                    raw=g.encode_gif_frame_bytes(image,GifImagePlugin,record["GIF_delay_ms"],True)+b";"
                    record["encoded_file_bytes"]=len(raw)
                    record["encoded_sha256"]=write_new(g,OUT/(arm+"_"+str(ordinal).zfill(4)+".gif"),raw,IMAGE)
                    record["status"]="PASS"
                finally:
                    if image is not None:image.close()
                    plt.close(view[0])
                result["last_resources"]=guard(g,started)
            need(all(r["status"]=="PASS" for r in records),"all five complete samples required")
            result["status"]="COMPLETE_FIVE_RECORD_DIAGNOSTIC_ENCODINGS_NOT_FULL_INTERVAL_PROOF"
        except BaseException as failure:
            result["error"]=repr(failure)[:2048]
            if cursor is not None:records[cursor].update(status="FAILED_RETAINED",error=repr(failure)[:512])
        finally:
            for label,closer in (("plots",lambda:plt.close("all") if plt else None),("seals",lambda:verify(g,plan))):
                try:closer()
                except BaseException as failure:result["closing_errors"].append(label+": "+repr(failure)[:512]);result["status"]="FAILED_RETAINED"
            result.update(fixed=fixed(),source_sha256=plan["source_sha256"],metadata_input_sha256=plan["input_sha256"],estimate=sample_estimate(records),
                full_render_authorized=False,actual_RViz_screenshot=False,new_drive=False,formal_main_approved=0,formal_sub_approved=0,wall_seconds=time.monotonic()-started)
            try:result["sampled_resources_closing"]=guard(g,started,closing=True)
            except BaseException as failure:result["closing_errors"].append("resource: "+repr(failure)[:512]);result["status"]="FAILED_RETAINED"
            write_new(g,OUT/"result.json",encode(result),META,closing=True);budget(g,closing=True)
        return dict(status=result["status"],output=str(OUT),completed_samples=sum(r["status"]=="PASS" for r in records),full_render_authorized=False)

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);parser.add_argument("--mode",choices=("prepare","execute"),required=True)
    parser.add_argument("--plan-sha256");parser.add_argument("--authority-sha256");args=parser.parse_args(argv)
    if args.mode=="prepare":
        g=load_gallery()
        with g.LOCK.open("r") as lock:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);value=prepare(g)
    else:
        need(args.plan_sha256 and args.authority_sha256,"external ROOT RAW probe hashes required")
        need(PLAN==PLAN.resolve(strict=True) and not any(item.is_symlink() for item in (PLAN,*PLAN.parents)),"canonical probe plan required")
        before=PLAN.stat();need(stat.S_ISREG(before.st_mode) and before.st_nlink==1 and before.st_size<=PLANS,"bounded single-link ROOT probe plan required")
        with PLAN.open("rb") as stream:
            opened=os.fstat(stream.fileno());raw=stream.read(PLANS+1);closed=os.fstat(stream.fileno())
        after=PLAN.stat();fields=("st_mode","st_nlink","st_dev","st_ino","st_size","st_mtime_ns","st_ctime_ns")
        need(all(tuple(getattr(value,key) for key in fields)==tuple(getattr(before,key) for key in fields) for value in (opened,closed,after))
            and len(raw)==before.st_size and len(raw)<=PLANS,"ROOT probe plan generation changed")
        need(hashlib.sha256(raw).hexdigest()==args.plan_sha256,"RAW probe plan differs before helper import")
        p=json.loads(raw);g=load_gallery(p["source_sha256"][str(GALLERY.relative_to(ROOT))]);value=execute(g,args.plan_sha256,args.authority_sha256)
    print(json.dumps(value,sort_keys=True));return 2 if value["status"]=="FAILED_RETAINED" else 0

if __name__=="__main__":raise SystemExit(main())
