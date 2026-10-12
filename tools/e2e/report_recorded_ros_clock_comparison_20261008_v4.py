"""HH_261008 - Compare the ended V16b/V18c recorded clocks and all failed tails without another inference.

V4 keeps the prospective reservations from V3 and fixes only the declared cross-run input scope.
V3 prospectively reserves 8 MiB for the larger immutable source union and 1 MiB
per diagnostic PNG within the unchanged 16 MiB total / 2 MiB closing reserve.
All918 rows per arm remain present. Common observations and complete pairs are
derived from actual source/neutral17-CDR/status evidence, not a fixed25 validity label.
Full19 wire receipts are still checked within each run; run-specific String bytes
are not available for fresh decoding or revalidation by this metadata-only report.
Selected provider entries/font are sealed, not a complete transitive runtime.

Preparation is not approval. Actual metadata reading/rendering requires fresh
externally SHA-pinned ROOT plan/authority and the shared runtime lease. Charts
do not demonstrate a causal speedup, driving function or pure network latency.
Missing marks stay NULL; raw NN/audit durations are nested, not additive bars.
"""
from __future__ import annotations

import argparse
import ast
import csv
from copy import deepcopy
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
import stat
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve()
SCHEMA="portable_e2e.recorded_ROS_clock_comparison.20261008.v4"
PLAN_DIR=ROOT/"storage/experiments/plans/2026-10-08/recorded_ros_clock_comparison_v4"
PLAN=PLAN_DIR/"ROOT_plan.json";AUTHORITY=PLAN_DIR/"ROOT_execution_authority.json"
OUT=ROOT/"storage/experiments/diagnostics/2026-10-08/recorded_ros_clock_comparison_v4"
LOCK=ROOT/"storage/assets/locks/autoware_e2e_runtime.lock"
TOTAL,CLOSING,SOURCE_LIMIT,CSV_LIMIT,PNG_LIMIT,META_LIMIT=16*1024**2,2*1024**2,8*1024**2,2*1024**2,1024**2,1024**2
PLAN_LIMIT,AUTH_LIMIT,MPL_CACHE_LIMIT=256*1024,512*1024,512*1024
FLOOR,SHARED=2*1024**3,64*1024**2
ARMS={
    "V16b":dict(version="16b",result="c3e3c0dc7618c5e481faa75e3b627003503ed5f87600aa3ba4397da426b0f528",plan="cfe85b74d119c809cba97c1d58a473af520686b2d8ebcf685465764fdc4cece3",authority="3257f8551c4143d32300bb1c83eb97ad31175fafacfa93f048e0dd7a4fb9de11"),
    "V18c":dict(version="18c",result="951a27e7c95b65f0c9b031804d01b2917af78979fbd8a7d0342b7a52affbe355",plan="ca96993ef80d27bf068bd4c64aaf0ba9ff71fe608e4d879042350d5977d6905f",authority="8b656e0df71c10210a512e0300356e75bf0eb0462194cab3010ac99e560c4c90"),
}
# HH_261008 - Seal frozen V1 failure sources without editing or replacing them.
OLD_V1_PINS={
    "tools/e2e/report_recorded_ros_clock_comparison_20261008_v1.py":"761d3cd05afa716a06ce0496f513f42183dd9e9435d08267ea6174ba72aa037d",
    "learning/tests/test_recorded_ros_clock_comparison_20261008_v1.py":"666e03ef0b13dcf7c25b32b2e2296c76fe90e7a6985bebe58716907ce47847e4",
}
# HH_261008 - Preserve both earlier clock-report revisions; neither parent experiment nor old chart is rewritten.
OLD_V2_PINS={
    "tools/e2e/report_recorded_ros_clock_comparison_20261008_v2.py":"f3088fc4bacd4483186b319d4572b08ac459e24d1cac3aa0d241715cd66aca5e",
    "learning/tests/test_recorded_ros_clock_comparison_20261008_v2.py":"898676fc2747c5766e1453e9d0d81dd0b44c4ca4986c719810fb0e13fbf500b7",
}
# HH_261008 - Preserve V3's actual pure PASS/preparation and semantic source finding; no V3 rendering or runtime followed.
OLD_V3_PINS={
    "tools/e2e/report_recorded_ros_clock_comparison_20261008_v3.py":"30d24fd1b5b78d3d2fb887420144ce648b90d2d8b1df94a874a496c5ede16280",
    "learning/tests/test_recorded_ros_clock_comparison_20261008_v3.py":"c497fc34fe71195e6f312193acba5839dc6341b49b5d5535e3a7496a5fde2736",
}
OLD_V3_EVIDENCE={
    str(ROOT/"storage/experiments/diagnostics/2026-10-08/recorded_ros_clock_v3_source_finding_v1.json"):"a3fbc9687958be3fce0cfb9455ef9d395e9e19ef6454b31f2d15ec8efaa42364",
}
# HH_261008 - Only top-level find_spec is allowed before approval; dotted lookups could import their parent package.
PROVIDER_VERSIONS={"numpy":"1.21.5","matplotlib":"3.5.1"}
PROVIDER_ORIGINS={
    "numpy":"/usr/lib/python3/dist-packages/numpy/__init__.py",
    "matplotlib":"/usr/lib/python3/dist-packages/matplotlib/__init__.py",
}
PROVIDER_MODULE_FILES={
    **PROVIDER_ORIGINS,
    "numpy.core._multiarray_umath":"/usr/lib/python3/dist-packages/numpy/core/_multiarray_umath.cpython-310-x86_64-linux-gnu.so",
    "matplotlib.pyplot":"/usr/lib/python3/dist-packages/matplotlib/pyplot.py",
    "matplotlib.backends.backend_agg":"/usr/lib/python3/dist-packages/matplotlib/backends/backend_agg.py",
    "matplotlib.backends._backend_agg":"/usr/lib/python3/dist-packages/matplotlib/backends/_backend_agg.cpython-310-x86_64-linux-gnu.so",
    "matplotlib.font_manager":"/usr/lib/python3/dist-packages/matplotlib/font_manager.py",
    "matplotlib.ft2font":"/usr/lib/python3/dist-packages/matplotlib/ft2font.cpython-310-x86_64-linux-gnu.so",
    "matplotlib.rcsetup":"/usr/lib/python3/dist-packages/matplotlib/rcsetup.py",
}
PROVIDER_DATA=Path("/usr/share/matplotlib/mpl-data")
PROVIDER_CONFIG=PROVIDER_DATA/"matplotlibrc"
PROVIDER_FONT=PROVIDER_DATA/"fonts/ttf/DejaVuSans.ttf"
PROVIDER_FILES=tuple(sorted({*PROVIDER_MODULE_FILES.values(),str(PROVIDER_CONFIG),str(PROVIDER_FONT)}))
FIXED_SOURCE_PINS={
    **OLD_V1_PINS,
    **OLD_V2_PINS,
    **OLD_V3_PINS,
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_v1_20261005.py":"e0e1b001c3684dfdf1ea41274c4c39b7f9a286d008d37d942aefbe35a3591e7e",
    "tools/e2e/owned_lossless_jsonl_20261008_v1.py":"5e859fbffa4aaebe81f8e57426cb57600a9c9272190f1e4887428991c6cccbb5",
    "learning/tests/test_owned_lossless_jsonl_20261008_v1.py":"b4c82537b3efaf4547f2c33323ee1f88508ff10bbd1cb12dcc5783d91187648f",
    "tools/e2e/owned_lossless_jsonl_20261008_v2.py":"f867fc18ff6f85038e3ea4dab8fd4b7b50bba84be8655c4db68bd6eab8eff479",
    "learning/tests/test_owned_lossless_jsonl_20261008_v2.py":"27c2b6f60947f5b1a37c6453afbea3107afc645601d3a9459b746db6201a8ba0",
    "tools/e2e/report_recorded_ros_latency_20261007_v4.py":"3878437476f9f99636a4e6620a951b36b718651f8589b755189db2f15b96e2b4",
    "learning/tests/test_recorded_ros_latency_report_20261007_v4.py":"6f99f259320f660263be6333c1c8386c0089285e062df6b3459113787c2c0983",
    "learning/portable_e2e/temporal_recorded_ros_bundle_v1_20261005.py":"6fa21ab9447bcf73bdd2fd79b0ab72ddd94b531d3a11459334f754fdfbad7d82",
}
LOG_NAMES={"direct_anchors":"direct_anchors.jsonl","subscriber_anchors":"subscriber_anchors.jsonl",
    "publisher_anchors":"publisher_anchors.jsonl","subscriber_stdout":"subscriber.stdout",
    "publisher_stdout":"publisher.stdout","subscriber_output_calls":"subscriber_output_calls.jsonl",
    "publisher_received_outputs":"publisher_received_outputs.jsonl","comparison":"comparison.jsonl"}
MARKS=("publisher_bundle_started_ns","all_ROS_inputs_ready_ns","policy_started_ns","policy_finished_ns",
    "conversion_finished_ns","trajectory_publication_finished_ns","actual_ACK_publish_completed_ns","original_console_returned_ns")
PHASES=("begin_to_input_ready","input_ready_to_policy_start","policy","converter",
    "conversion_to_trajectory_calls_finish","trajectory_calls_finish_to_ACK","ACK_to_console_return")


def need(value,message):
    if not value:raise ValueError(message)


def canonical(path,exists=True):
    path=Path(path)
    need(path.is_absolute() and path==path.resolve(strict=exists) and not any(p.is_symlink() for p in (path,*path.parents)),"canonical non-symlink path required")
    return path


def sha(path):
    path=canonical(path);before=path.stat();need(stat.S_ISREG(before.st_mode),"regular source/metadata input required")
    h=hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda:stream.read(1024**2),b""):h.update(block)
    after=path.stat();need(all(getattr(before,k)==getattr(after,k) for k in ("st_dev","st_ino","st_size","st_mtime_ns","st_ctime_ns")),"file changed during sealing")
    return h.hexdigest()


def encode(value):return (json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False)+"\n").encode()


def decode(raw):
    def unique(items):
        value={}
        for k,v in items:need(k not in value,"duplicate JSON key denied");value[k]=v
        return value
    return json.loads(raw,object_pairs_hook=unique,parse_constant=lambda v:need(False,"nonfinite JSON denied"))


def read_json(path,pin=None,limit=4*1024**2):
    path=canonical(path);need(path.stat().st_size<=limit,"bounded scalar metadata required")
    raw=path.read_bytes()
    if pin is not None:need(hashlib.sha256(raw).hexdigest()==pin,"exact raw JSON seal differs")
    value=decode(raw);need(type(value) is dict,"metadata object required");return value


def census(root):
    root=canonical(root);values={}
    for path in sorted(root.rglob("*")):
        mode=path.lstat().st_mode;need(stat.S_ISDIR(mode) or stat.S_ISREG(mode),"special/symlink artifact denied")
        if stat.S_ISREG(mode):
            need(path.stat().st_nlink==1 and path.stat().st_size<=32*1024**2,"single-link bounded artifact required")
            values[str(path)]=sha(path)
    return values


def budget(additional=0,closing=False,roots=None):
    roots=(PLAN_DIR,OUT) if roots is None else roots
    need(type(additional) is int and additional>=0,"prospective byte count required")
    need(os.statvfs(ROOT).f_bavail*os.statvfs(ROOT).f_frsize>=FLOOR+SHARED+(0 if closing else CLOSING),"shared floor/reserve denied")
    total=cache=0
    for root in roots:
        canonical(root,exists=root.exists())
        if not root.exists():continue
        for path in root.rglob("*"):
            mode=path.lstat().st_mode;need(stat.S_ISDIR(mode) or stat.S_ISREG(mode),"special/symlink output denied")
            if stat.S_ISREG(mode):
                need(path.stat().st_nlink==1,"hardlinked output denied");size=path.stat().st_size;total+=size
                if root==OUT and path.relative_to(root).parts[0]=="matplotlib_cache":cache+=size
    need(cache<=MPL_CACHE_LIMIT,"sampled private Matplotlib cache quota exceeded")
    need(total+additional<=TOTAL-(0 if closing else CLOSING),"combined plan/output16MiB limit exceeded")
    return total


def write_bytes(path,raw,limit,closing=False):
    need(type(raw) is bytes and len(raw)<=limit,"bounded complete output bytes required")
    canonical(path,exists=False);budget(len(raw),closing)
    with Path(path).open("xb") as stream:need(stream.write(raw)==len(raw),"partial output write");stream.flush()
    return hashlib.sha256(raw).hexdigest()


def load_stdlib_source(name):
    path=ROOT/name;need(sha(path)==FIXED_SOURCE_PINS[name],"frozen metadata/codec source changed")
    spec=importlib.util.spec_from_file_location("_hh_clock_"+path.stem,path);value=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=value;spec.loader.exec_module(value)
    need(not any(name in sys.modules for name in ("torch","rclpy","carla")),"metadata reader imported a runtime")
    return value


def gallery_math():
    # HH_261008 - Compile only genuine source-pinned scalar functions, not the old renderer or a trainer namespace.
    name="tools/e2e/report_recorded_ros_latency_20261007_v4.py";path=ROOT/name
    need(sha(path)==FIXED_SOURCE_PINS[name],"frozen actual-gallery source differs")
    names={"delta","milliseconds","original_and_deferred_diagnostics"}
    functions=[p for p in ast.parse(path.read_bytes()).body if isinstance(p,ast.FunctionDef) and p.name in names]
    need({p.name for p in functions}==names,"three genuine scalar functions required")
    namespace=dict(need=need,math=math)
    exec(compile(ast.Module(body=functions,type_ignores=[]),str(path),"exec"),namespace)
    return namespace


def parent_paths(arm):
    version=ARMS[arm]["version"];folder="temporal_recorded_ros_shadow_v"+str(version)
    return ROOT/"storage/experiments/diagnostics/2026-10-08"/folder,ROOT/"storage/experiments/plans/2026-10-08"/folder


def parent_evidence(arm):
    run,plans=parent_paths(arm);pins=ARMS[arm]
    plan=read_json(plans/"ROOT_plan.json",pins["plan"]);result=read_json(run/"result.json",pins["result"])
    read_json(plans/"ROOT_execution_authority.json",pins["authority"])
    need(result["source_sha256_after"]==plan["source_sha256"] and result["input_sha256_after"]==plan["input_sha256"]
        and result["cleanup_errors"]==[] and all(child["exit"] is not None for child in result["children"]),"genuine ended parent closing seals required")
    return run,plans,plan,result


def provider_hash():
    # HH_261008 - Prepare selected source/native-entry/config/font hashes without importing NumPy or Matplotlib.
    origins={};versions={}
    for name,wanted in PROVIDER_ORIGINS.items():
        spec=importlib.util.find_spec(name)
        need(spec is not None and spec.origin==wanted and str(canonical(spec.origin))==wanted,"selected installed provider origin differs")
        origins[name]=spec.origin;versions[name]=importlib.metadata.version(name)
        need(versions[name]==PROVIDER_VERSIONS[name],"selected installed provider version differs")
    return dict(module_origins=origins,versions=versions,selected_sha256={path:sha(path) for path in PROVIDER_FILES},
        selected_config=str(PROVIDER_CONFIG),selected_font=str(PROVIDER_FONT),
        scope="selected_entry_native_source_config_and_font_NOT_complete_transitive_environment_equivalence")


def imported_provider_receipt(plan,plt,font_manager):
    # HH_261008 - The source-only seals become an actual loaded-origin/font receipt only after external ROOT permission.
    for name,wanted in PROVIDER_MODULE_FILES.items():
        module=sys.modules.get(name)
        need(module is not None and getattr(module,"__file__",None)==wanted,"loaded selected provider origin differs")
    for name,wanted in PROVIDER_VERSIONS.items():need(getattr(sys.modules[name],"__version__",None)==wanted,"loaded selected provider version differs")
    matplotlib=sys.modules["matplotlib"]
    need(canonical(matplotlib.get_data_path())==PROVIDER_DATA and canonical(matplotlib.matplotlib_fname())==PROVIDER_CONFIG,"loaded selected plotting config/data differs")
    plt.rcParams.update({"font.family":["DejaVu Sans"],"text.usetex":False})
    actual_font=canonical(font_manager.findfont(font_manager.FontProperties(family=["DejaVu Sans"]),fallback_to_default=False))
    need(actual_font==PROVIDER_FONT,"actual selected font differs")
    need(provider_hash()==plan["provider_hash"],"selected provider changed before render")
    return dict(selected_font=str(actual_font),module_origins=dict(PROVIDER_MODULE_FILES),versions=dict(PROVIDER_VERSIONS),
        provider_hash=plan["provider_hash"],transitive_environment_equivalence_claimed=False)


def tables():
    sources={name:sha(ROOT/name) for name in (*FIXED_SOURCE_PINS,str(HERE.relative_to(ROOT)),"learning/tests/test_recorded_ros_clock_comparison_20261008_v4.py","tools/workspace_python.py","tools/canonical_workspace.py")}
    need(all(sources[name]==pin for name,pin in FIXED_SOURCE_PINS.items()),"frozen report dependencies changed")
    inputs=dict(OLD_V3_EVIDENCE);parent_sources={}
    for path,pin in OLD_V3_EVIDENCE.items():need(sha(path)==pin,"preserved V3 source finding changed")
    for arm in ARMS:
        run,plans,plan,_=parent_evidence(arm);inputs.update(census(run))
        for key,file in (("plan","ROOT_plan.json"),("authority","ROOT_execution_authority.json")):inputs[str(plans/file)]=ARMS[arm][key]
        for item in plan["scalar_inputs"].values():
            need(sha(item["path"])==item["sha256"],"genuine original scalar index input changed");inputs[item["path"]]=item["sha256"]
        for name,pin in plan["source_sha256"].items():
            need(not Path(name).is_absolute() and (ROOT/name).resolve(strict=True)==ROOT/name,"canonical parent source path required")
            need(name not in sources or sources[name]==pin,"parent/current report source conflict")
            need(sha(ROOT/name)==pin and sha(plans/"sources"/name)==pin,"parent current/archive source changed")
            sources[name]=pin;parent_sources[str(plans/"sources"/name)]=pin
    return dict(source_sha256=dict(sorted(sources.items())),input_sha256=dict(sorted(inputs.items())),parent_archive_sha256=parent_sources,provider_hash=provider_hash())


def fixed():
    return dict(arms=list(ARMS),expected_anchors_per_arm=918,fixed_paired_ordinals=None,
        paired_scope="all_actual_common_source_neutral17_CDR_status_derived_subsets_not_fixed25_or_causal_experiment",
        native_Trajectory_Path_call_marks_retained=True,all_ROS_inputs_ready_clock_retained=True,
        cross_run_neutral_input_topics=input_topic_contract()["neutral17"],auxiliary_String_topics=input_topic_contract()["auxiliary2"],
        full19_sent_received_CDR_must_match_within_each_run=True,cross_run_full19_CDR_identity_expected=False,
        auxiliary_String_raw_bytes_available=False,auxiliary_String_fresh_payload_revalidation_claimed=False,
        auxiliary_validation_scope="existing_original_admission_source_frame_stamp_replay_authority_witness_only",
        original_ACK_deadline_ms=100,original_console_clock_separate=True,missing_values_are_NULL_not_zero=True,all_failure_statuses_retained=True,
        no_interpolation=True,local_stage_SUM_is_not_wall=True,raw_NN_and_audit_are_nested_not_additional_bar=True,whole16MiB_combined=True,
        phase_interval_units="milliseconds",integer_clock_and_frame_units_preserved=True,
        output_limit=TOTAL,closing_reserve=CLOSING,source_archive_limit=SOURCE_LIMIT,csv_gz_limit=CSV_LIMIT,png_limit=PNG_LIMIT,
        raw_candidate_and_ROOT_plan_limit=PLAN_LIMIT,ROOT_authority_limit=AUTH_LIMIT,result_limit=META_LIMIT,private_Matplotlib_cache_limit=MPL_CACHE_LIMIT,
        floor_bytes=FLOOR,shared_reserve_bytes=SHARED,CPU_affinity=[3],threads=1,sampled_wall_limit_seconds=600,
        selected_plotting_provider_entries_config_and_font_sealed=True,complete_transitive_environment_equivalence_claimed=False,
        hard_preemptive_deadline_or_OS_containment_claimed=False,NN_inferences=0,training_steps=0,actual_RViz_screenshot=False,formal_main_approved=0,formal_sub_approved=0)


def prepare():
    need(not PLAN_DIR.exists() and not OUT.exists(),"fresh candidate only; no overwrite/retry")
    evidence=tables();source_bytes=sum((ROOT/name).stat().st_size for name in evidence["source_sha256"])
    need(source_bytes<=SOURCE_LIMIT,"complete unique-path source archive budget exceeded")
    plan=dict(schema=SCHEMA,fixed=fixed(),**evidence)
    need(len(encode(plan))<=PLAN_LIMIT and source_bytes+2*PLAN_LIMIT+AUTH_LIMIT+2*PNG_LIMIT+CSV_LIMIT+MPL_CACHE_LIMIT<=TOTAL-CLOSING
        and META_LIMIT<=CLOSING,"prospective complete output and closing reservation does not fit")
    PLAN_DIR.mkdir(parents=True)
    for name,pin in evidence["source_sha256"].items():
        path=PLAN_DIR/"sources"/name;path.parent.mkdir(parents=True,exist_ok=True)
        raw=(ROOT/name).read_bytes();need(hashlib.sha256(raw).hexdigest()==pin,"source changed before archive")
        write_bytes(path,raw,SOURCE_LIMIT)
    result=write_bytes(PLAN_DIR/"candidate_plan.json",encode(plan),PLAN_LIMIT)
    return dict(status="PREPARED_NOT_AUTHORIZED",candidate_plan_sha256=result,source_count=len(evidence["source_sha256"]),metadata_input_count=len(evidence["input_sha256"]),ROOT_plan=str(PLAN),ROOT_authority=str(AUTHORITY))


def verify(plan):
    need(plan["schema"]==SCHEMA and plan["fixed"]==fixed(),"exact fixed clock protocol required")
    for name,pin in plan["source_sha256"].items():need(sha(ROOT/name)==pin and sha(PLAN_DIR/"sources"/name)==pin,"current/report source archive changed")
    for category in ("input_sha256","parent_archive_sha256"):
        for path,pin in plan[category].items():need(sha(path)==pin,"complete original metadata/source input changed")
    need(tables()=={key:plan[key] for key in ("source_sha256","input_sha256","parent_archive_sha256","provider_hash")},"whole original file census/provider table changed")


def read_ledgers(run,codec):
    capacity=codec.prospective_capacity();rows={};proof={}
    for name,file in LOG_NAMES.items():
        root=run/(file+".lossless");limits=capacity["components"][name]
        raw=list(codec.iter_rows(root,**limits)) # HH_261008 - Fully consume before accepting any row or complete prefix.
        rows[name]=[codec.decode_row(value) for value in raw]
        proof[name]=dict(rows=len(raw),original_stream_sha256=hashlib.sha256(b"".join(raw)).hexdigest(),original_bytes=sum(map(len,raw)))
        if name.endswith("_stdout"):rows[name]=list(zip(raw,rows[name]))
    for role in ("direct","subscriber","publisher"):
        closed=read_json(run/(role+"_codec_closing.json"),limit=META_LIMIT)
        need(closed["codec_failure"] is False and all(row["failure"] is None and row["oversized_fragment_receipt"] is None and not row["errors"] for row in closed["ledgers"]),"actual codec failure is retained, not accepted")
    return rows,proof


def console_map(witness,printed,anchors):
    need(witness["expected_anchors"]==918 and witness["original_ACK_clock_unchanged"] is True
        and witness["original_ACK_clock_includes_later_journal_or_console"] is False and len(witness["rows"])==len(printed)<=919,"genuine separate console denominator required")
    joined={}
    for number,((raw,value),row) in enumerate(zip(printed,witness["rows"])):
        need(row["console_call"]==number and hashlib.sha256(raw[:-1]).hexdigest()==row["receipt_arg_byte_sha256"],"actual original print argument bytes differ")
        if "ordinal" not in value:need(row["binding"]=="ORIGINAL_CLOSING_SUMMARY_NOT_ANCHOR","actual summary cannot be an anchor");continue
        ordinal=value["ordinal"];need(type(ordinal) is int and 0<=ordinal<918,"actual printed ordinal required")
        anchor=anchors[ordinal];need(value["native_frame"]==anchor.native_frame and value["timestamp_ns"]==anchor.timestamp_ns,"printed actual frame/stamp differs")
        need(ordinal not in joined,"duplicate printed anchor retained, not joined twice")
        if row["binding"]=="ACTUAL_RECEIPT_SOURCE_JOIN":
            provenance=value["source_selected_callback_provenance"]
            need(encode(provenance)==encode(anchor.provenance) and row["source_provenance_sha256"]==hashlib.sha256(json.dumps(provenance,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest(),"actual printed source join differs")
            need(row["ordinal"]==ordinal and row["native_frame"]==anchor.native_frame and row["timestamp_ns"]==anchor.timestamp_ns
                and row["publisher_bundle_started_ns"]==value["stages"]["publisher_bundle_started_ns"],"actual console witness identity/begin differs")
            need(type(row["original_console_returned_ns"]) is int and row["original_console_returned_ns"]>=row["original_receipt_stage_ns"],"actual console chronology differs")
            stage=value["stages"].get("receipt_publication_finished_ns",value["stages"].get("receipt_publication_entered_ns"))
            need(row["original_receipt_stage_ns"]==stage and row["actual_until_original_console_log_complete_ms"]
                ==(row["original_console_returned_ns"]-row["publisher_bundle_started_ns"])/1e6,"actual console endpoint/duration differs")
        else:need(row["binding"]=="UNKNOWN" and row["actual_until_original_console_log_complete_ms"] is None,"unknown console timing may not be invented")
        joined[ordinal]=row
    return joined


def input_topic_contract():
    # HH_261008 - Derive exact names from the pinned producer vocabulary, not a length17 filter or dynamically encountered keys.
    name="learning/portable_e2e/temporal_recorded_ros_bundle_v1_20261005.py"
    path=ROOT/name;need(sha(path)==FIXED_SOURCE_PINS[name],"exact frozen topic vocabulary required")
    assignments={item.targets[0].id:item.value for item in ast.parse(path.read_bytes()).body
        if isinstance(item,ast.Assign) and len(item.targets)==1 and isinstance(item.targets[0],ast.Name)}
    roles=ast.literal_eval(assignments["ROLES"]);prefix=ast.literal_eval(assignments["INPUT_PREFIX"])
    need(type(roles) is tuple and len(roles)==6 and len(set(roles))==6 and all(type(role) is str for role in roles)
        and type(prefix) is str and prefix.startswith("/"),"exact source-derived camera roles/prefix required")
    producer="autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_v1_20261005.py"
    need(sha(ROOT/producer)==FIXED_SOURCE_PINS[producer],"exact original typed-message producer required")
    neutral=[prefix+"/"+key for key in ("odometry","acceleration","steering","tf","clock")]
    neutral += [prefix+"/"+role+"/"+kind for role in roles for kind in ("compressed","camera_info")]
    auxiliary=[prefix+"/provenance",prefix+"/navigation"]
    need(len(set(neutral))==17 and not set(neutral)&set(auxiliary),"source topic sets must remain disjoint17 plus2")
    return dict(neutral17=sorted(neutral),auxiliary2=sorted(auxiliary),whole19=sorted(neutral+auxiliary))


def validate_CDR_map(value,topics):
    need(type(value) is dict and set(value)==set(topics),"exact declared CDR topic-name set required")
    need(all(type(pin) is str and len(pin)==64 and all(char in "0123456789abcdef" for char in pin)
        for pin in value.values()),"actual CDR receipt SHA values required")
    return value


def own_run_input_receipts(arm,value,published,direct,anchor,parent_identity,topics):
    # HH_261008 - Keep all actual wire hashes unchanged; auxiliary admission evidence comes from the frozen original validation, not raw String re-decoding.
    need(type(parent_identity) is dict and set(parent_identity)=={"replay_id","authority_sha256"}
        and type(parent_identity["replay_id"]) is str and bool(parent_identity["replay_id"])
        and parent_identity["authority_sha256"]==ARMS[arm]["authority"],"exact parent run replay/RAW authority join required")
    received=validate_CDR_map(value.get("actual_received_input_CDR_sha256"),topics["whole19"])
    sent=validate_CDR_map(published.get("published_input_CDR_sha256"),topics["whole19"])
    neutral=validate_CDR_map(direct.get("input_CDR_sha256"),topics["neutral17"])
    need(received==sent and all(received[name]==neutral[name] for name in topics["neutral17"]),
        "actual direct17/sent19/received19 CDR differs within own run")
    for row in (value,published):
        need(row.get("replay_id")==parent_identity["replay_id"] and row.get("authority_sha256")==parent_identity["authority_sha256"],
            "original own-run receipt replay/authority differs")
        need(all(type(row.get(name)) is int and row[name]==getattr(anchor,name)
            for name in ("ordinal","native_frame","timestamp_ns")),"original own-run receipt frame/stamp differs")
        provenance=row.get("source_selected_callback_provenance")
        if provenance is not None:need(encode(provenance)==encode(anchor.provenance),"original own-run admission source differs")
        else:need(row.get("status")=="FAILED_RETAINED","nonfailed original receipt missing admission source")
    admission=value.get("stages",{}).get("ROS_admission_finished_ns")
    bound=type(admission) is int and admission>0 and value.get("source_selected_callback_provenance") is not None \
        and published.get("source_selected_callback_provenance") is not None
    return dict(full19_sha256=receipt_map_sha(received),
        neutral17_sha256=receipt_map_sha({name:received[name] for name in topics["neutral17"]}),
        auxiliary2_sha256=receipt_map_sha({name:received[name] for name in topics["auxiliary2"]}),
        auxiliary_binding="SOURCE_BOUND_ORIGINAL_ADMISSION_WITNESS" if bound else "UNKNOWN_NO_ORIGINAL_ADMISSION_WITNESS",
        auxiliary_raw_bytes_freshly_revalidated=False)


def receipt_map_sha(value):
    # HH_261008 - These are hashes of existing scalar receipts, not newly computed tensor/image hashes.
    if value is None:return None
    need(type(value) is dict and bool(value),"nonempty actual scalar receipt map required")
    return hashlib.sha256(encode(value)).hexdigest()


def actual_output_marks(record,events,received,maths):
    # HH_261008 - Retain a published Trajectory before a denied Path; a missing completion remains NULL.
    suffixes={"shadow_trajectory":"trajectory_call","shadow_path":"path_call"}
    seen=set();published=set();received_CDR={}
    for event in events:
        topic=event["topic"];suffix=topic.rsplit("/",1)[-1]
        need(suffix in suffixes and suffix not in seen,"unique declared actual output call required");seen.add(suffix)
        need(event["ordinal"]==record["ordinal"],"actual output ordinal differs")
        prefix=suffixes[suffix];begin=event.get("publisher_bundle_started_ns");start=record[MARKS[0]]
        if begin is not None:need(type(begin) is int and begin==start,"actual output publisher clock differs")
        for name,field in (("entry","actual_publish_entered_ns"),("finish","actual_publish_completed_ns")):
            mark=event.get(field);record[prefix+"_"+name+"_ns"]=mark
            if mark is not None:
                need(type(mark) is int and start is not None,"actual call clock requires integer publisher start")
                record[prefix+"_"+name+"_ms"]=maths["delta"](start,mark)
                ACK=record.get("actual_ACK_publish_completed_ns")
                if ACK is not None:need(type(ACK) is int and mark<=ACK,"actual output call mark occurs after original ACK")
        entered=event.get("actual_publish_entered_ns");finished=event.get("actual_publish_completed_ns")
        if entered is not None and finished is not None:need(finished>=entered,"actual output call chronology differs")
        if event["actually_published"] is True:
            published.add(suffix)
            matches=[rx for rx in received if rx["topic"]==topic]
            need(len(matches)<=1,"duplicate received output retained, not merged")
            if matches:
                rx=matches[0];need(rx["ordinal"]==record["ordinal"] and rx["timestamp_ns"]==record["timestamp_ns"]
                    and rx["sha256"]==event["CDR_sha256"],"actual call/received source stamp CDR differs")
                received_CDR[topic]=rx["sha256"]
    need(all(rx["topic"].rsplit("/",1)[-1] in published for rx in received),"received output without actual published call denied")
    record["complete_output_pair"]={"shadow_trajectory","shadow_path"}<=published
    record["complete_received_output_pair"]=record["complete_output_pair"] and len(received_CDR)==2
    record["partial_trajectory"]="shadow_trajectory" in published and "shadow_path" not in published
    record["received_output_CDR_receipts_sha256"]=receipt_map_sha(received_CDR) if received_CDR else None


def summarize_arm(arm,rows,witness,anchors,maths,*,parent_identity):
    topics=input_topic_contract()
    need(len(anchors)==918 and [a.ordinal for a in anchors]==list(range(918)),"genuine whole918 source index required")
    direct,sub,pub=(rows[name] for name in ("direct_anchors","subscriber_anchors","publisher_anchors"))
    need(len(direct)==918 and [v["ordinal"] for v in direct]==list(range(918)),"whole918 direct denominator required")
    need(len(sub)<=918 and [v["ordinal"] for v in sub]==list(range(len(sub))),"whole ordered actual subscriber prefix required")
    need([v["ordinal"] for v in pub]==list(range(len(pub))) and [v["ordinal"] for v in rows["comparison"]]==list(range(918)),"whole publisher/comparison failure denominator required")
    need(len(sub)==len(pub),"complete observed subscriber/publisher prefixes differ; preserve unknown evidence")
    for value in pub:
        anchor=anchors[value["ordinal"]]
        need(value["native_frame"]==anchor.native_frame and value["timestamp_ns"]==anchor.timestamp_ns,"publisher original index join differs")
        if "source_selected_callback_provenance" in value:need(encode(value["source_selected_callback_provenance"])==encode(anchor.provenance),"actual publisher source differs")
    console=console_map(witness,rows["subscriber_stdout"],anchors)
    table=[];counts={};calls=rows["subscriber_output_calls"];received=rows["publisher_received_outputs"]
    for event in (*calls,*received):need(type(event["ordinal"]) is int and 0<=event["ordinal"]<len(sub),"actual output event has no observed source anchor")
    for ordinal,anchor in enumerate(anchors):
        need(direct[ordinal]["native_frame"]==anchor.native_frame and direct[ordinal]["timestamp_ns"]==anchor.timestamp_ns,"direct original index join differs")
        validate_CDR_map(direct[ordinal].get("input_CDR_sha256"),topics["neutral17"])
        record=dict(arm=arm,ordinal=ordinal,native_frame=anchor.native_frame,timestamp_ns=anchor.timestamp_ns,status="UNEXECUTED",
            direct_status=direct[ordinal]["status"],source_provenance_sha256=hashlib.sha256(encode(anchor.provenance)).hexdigest(),
            full_ACK_ms=None,console_return_ms=None,policy_ms=None,raw_NN_ms=None,audit_flush_ms=None,diagnostic_UNKNOWN=None,
            complete_output_pair=False,complete_received_output_pair=False,partial_trajectory=False,denied_output_entry_ms=None,denied_output_topic=None,
            input_CDR_receipts_sha256=None,NN_input_receipts_sha256=None,NN_head_receipts_sha256=None,received_output_CDR_receipts_sha256=None,
            recorded_direct_DDS_tensor_equal=None,recorded_direct_DDS_raw_equal=None,recorded_direct_DDS_status_equal=None,input_ready_ms=None,
            neutral17_input_CDR_receipts_sha256=None,auxiliary2_String_CDR_receipts_sha256=None,auxiliary_original_validation_binding="UNEXECUTED",
            auxiliary_raw_String_payload_revalidated=False)
        for prefix in ("trajectory_call","path_call"):
            for mark in ("entry","finish"):
                record[prefix+"_"+mark+"_ns"]=record[prefix+"_"+mark+"_ms"]=None
        for key in (*MARKS,*PHASES):record[key]=None
        if ordinal<len(sub):
            value=sub[ordinal];need(value["native_frame"]==anchor.native_frame and value["timestamp_ns"]==anchor.timestamp_ns,"subscriber original index join differs")
            record["status"]=value["status"];stages=value.get("stages",{})
            if "source_selected_callback_provenance" in value:need(encode(value["source_selected_callback_provenance"])==encode(anchor.provenance),"actual subscriber source differs")
            for key in MARKS[:-2]:record[key]=stages.get(key)
            incoming_proof=own_run_input_receipts(arm,value,pub[ordinal],direct[ordinal],anchor,parent_identity,topics)
            record["input_CDR_receipts_sha256"]=incoming_proof["full19_sha256"]
            record["neutral17_input_CDR_receipts_sha256"]=incoming_proof["neutral17_sha256"]
            record["auxiliary2_String_CDR_receipts_sha256"]=incoming_proof["auxiliary2_sha256"]
            record["auxiliary_original_validation_binding"]=incoming_proof["auxiliary_binding"]
            numerical=value.get("policy_diagnostics",{}).get("owned_recorded_numerical_event",{})
            if numerical.get("tensor_sha256"):
                need(len(numerical["tensor_sha256"])==9,"whole original nine-input receipt required")
                record["NN_input_receipts_sha256"]=receipt_map_sha(numerical["tensor_sha256"])
            if numerical.get("raw_tensor_sha256"):
                need(len(numerical["raw_tensor_sha256"])==5,"all five original head receipts required")
                record["NN_head_receipts_sha256"]=receipt_map_sha(numerical["raw_tensor_sha256"])
            comparison=rows["comparison"][ordinal]
            record["recorded_direct_DDS_tensor_equal"]=comparison.get("tensor_equal")
            record["recorded_direct_DDS_raw_equal"]=comparison.get("raw_equal")
            record["recorded_direct_DDS_status_equal"]=None if "direct_status" not in comparison or "DDS_status" not in comparison else comparison["direct_status"]==comparison["DDS_status"]
            record["actual_ACK_publish_completed_ns"]=value.get("actual_ACK_publish_completed_ns")
            found=console.get(ordinal)
            if found is not None and found["binding"]=="ACTUAL_RECEIPT_SOURCE_JOIN":record["original_console_returned_ns"]=found["original_console_returned_ns"]
            for key,left,right in zip(PHASES,MARKS,MARKS[1:]):
                if record[left] is not None and record[right] is not None:record[key]=maths["delta"](record[left],record[right])
            for key,end in (("full_ACK_ms","actual_ACK_publish_completed_ns"),("console_return_ms","original_console_returned_ns")):
                if record[MARKS[0]] is not None and record[end] is not None:record[key]=maths["delta"](record[MARKS[0]],record[end])
            if record[MARKS[0]] is not None and record[MARKS[1]] is not None:record["input_ready_ms"]=maths["delta"](record[MARKS[0]],record[MARKS[1]])
            try:
                diagnostics=maths["original_and_deferred_diagnostics"](value)
                for key,source in (("policy_ms","total_seconds"),("raw_NN_ms","raw_nn_seconds")):
                    if diagnostics.get(source) is not None:record[key]=maths["milliseconds"](diagnostics[source])
                record["audit_flush_ms"]=maths["milliseconds"](diagnostics["owned_recorded_deferred_audit_timing"]["audit_flush_seconds"])
            except (ValueError,KeyError,TypeError) as error:record["diagnostic_UNKNOWN"]=repr(error)[:256]
            if record["policy_ms"] is not None and record["policy"] is not None:need(record["policy_ms"]<=record["policy"],"original policy timer exceeds same-anchor wrapper interval")
            actual_output_marks(record,[v for v in calls if v["ordinal"]==ordinal],[v for v in received if v["ordinal"]==ordinal],maths)
            denied=[v for v in calls if v["ordinal"]==ordinal and v["actually_published"] is False]
            need(len(denied)<=1,"multiple denied calls for one actual anchor retained, not merged")
            if denied:
                event=denied[0];record["denied_output_topic"]=event["topic"]
                if event.get("publisher_bundle_started_ns") is not None and event.get("actual_publish_entered_ns") is not None:
                    record["denied_output_entry_ms"]=maths["delta"](event["publisher_bundle_started_ns"],event["actual_publish_entered_ns"])
        counts[record["status"]]=counts.get(record["status"],0)+1;table.append(record)
    need(sum(counts.values())==918,"full status denominator required")
    return table,dict(status_counts=counts,observed=len(sub),UNEXECUTED=918-len(sub),all_output_calls=len(calls),all_received_outputs=len(received),
        complete_pairs=sum(r["complete_output_pair"] for r in table),partial_trajectory=sum(r["partial_trajectory"] for r in table),
        complete_received_pairs=sum(r["complete_received_output_pair"] for r in table),
        full_ACK_timely=sum(r["full_ACK_ms"] is not None and r["full_ACK_ms"]<=100 for r in table),console_bound=sum(r["console_return_ms"] is not None for r in table))


def paired_summary(tables):
    names=list(ARMS);a,b=(tables[arm] for arm in names)
    need(len(a)==len(b)==918,"both complete918 status tables required")
    for left,right in zip(a,b):need(all(left[k]==right[k] for k in ("ordinal","native_frame","timestamp_ns","source_provenance_sha256")),"all918 cross-arm original source identity differs")
    common=[n for n in range(918) if a[n]["status"]!="UNEXECUTED" and b[n]["status"]!="UNEXECUTED"]
    need(bool(common),"no actual common observations; no invented paired timing")
    pairs=[];complete=[];same_context=[];same_output=[]
    for n in common:
        left,right=a[n],b[n]
        full19_equal=left["input_CDR_receipts_sha256"] is not None and left["input_CDR_receipts_sha256"]==right["input_CDR_receipts_sha256"]
        auxiliary_equal=left["auxiliary2_String_CDR_receipts_sha256"] is not None and left["auxiliary2_String_CDR_receipts_sha256"]==right["auxiliary2_String_CDR_receipts_sha256"]
        CDR_equal=left["neutral17_input_CDR_receipts_sha256"] is not None and left["neutral17_input_CDR_receipts_sha256"]==right["neutral17_input_CDR_receipts_sha256"]
        context_equal=left["NN_input_receipts_sha256"] is not None and left["NN_input_receipts_sha256"]==right["NN_input_receipts_sha256"]
        output_equal=left["received_output_CDR_receipts_sha256"] is not None and left["received_output_CDR_receipts_sha256"]==right["received_output_CDR_receipts_sha256"]
        same_complete=CDR_equal and left["status"]==right["status"]=="TRAJECTORY" and left["complete_received_output_pair"] and right["complete_received_output_pair"]
        if same_complete:
            complete.append(n)
            if context_equal:same_context.append(n)
            if output_equal:same_output.append(n)
        pairs.append(dict(ordinal=n,reference_status=left["status"],candidate_status=right["status"],same_neutral17_actual_input_CDR=CDR_equal,
            same_full19_wire_CDR=full19_equal,same_auxiliary2_String_CDR=auxiliary_equal,run_specific_auxiliary_difference_expected=True,
            same_nine_tensor_receipts=context_equal,same_received_output_CDR=output_equal,same_source_neutral17_CDR_status_complete_pair=bool(same_complete),
            full_ACK_difference_ms=None if left["full_ACK_ms"] is None or right["full_ACK_ms"] is None else right["full_ACK_ms"]-left["full_ACK_ms"],
            console_difference_ms=None if left["console_return_ms"] is None or right["console_return_ms"] is None else right["console_return_ms"]-left["console_return_ms"]))
    statistics_by_arm={}
    for arm in names:
        statistics_by_arm[arm]={}
        for key in ("full_ACK_ms","console_return_ms","input_ready_ms","policy_ms","raw_NN_ms","audit_flush_ms"):
            measured=[tables[arm][n][key] for n in complete if tables[arm][n][key] is not None]
            statistics_by_arm[arm][key]=dict(n=len(measured),mean=None if not measured else statistics.fmean(measured),
                median=None if not measured else statistics.median(measured),missing_UNKNOWN=len(complete)-len(measured))
    return dict(reference_arm=names[0],candidate_arm=names[1],all_common_observed=pairs,all_common_observed_ordinals=common,
        same_source_neutral17_CDR_status_complete_pair_ordinals=complete,same_nine_tensor_complete_pair_ordinals=same_context,
        same_received_output_CDR_complete_pair_ordinals=same_output,complete_pair_statistics=statistics_by_arm,
        complete_pair_statistics_scope="same_source_neutral17_CDR_both_TRAJECTORY_two_received_outputs; nine_tensor_context_and_output_CDR_equality_reported_separately",
        full19_cross_run_difference_expected_from_authority_and_begin_metadata=True,auxiliary_String_raw_payload_revalidated=False,
        all1836_status_rows_retained=True,paired_subset_not_full918_or_safety_PASS=True,causal_or_sustained_performance_claimed=False)


def chart(plt,arm,rows,counts):
    observed=[r for r in rows if r["status"]!="UNEXECUTED"];fig,axes=plt.subplots(2,1,figsize=(13,11),gridspec_kw=dict(height_ratios=[1,2]))
    ax=axes[0]
    for key,label,marker in (("full_ACK_ms","Original actual ACK completion","o"),("console_return_ms","Separate original console return","x")):
        measured=[r for r in observed if r[key] is not None];ax.scatter([r["ordinal"] for r in measured],[r[key] for r in measured],s=24,marker=marker,label=label)
    ax.axhline(100,color="red",linestyle="--",label="Unchanged100ms reference");ax.set_ylabel("Actual milliseconds");ax.legend(fontsize=8)
    ax.set_title(arm+" recorded clock diagnostic: "+str(len(observed))+" observed /918; "+str(counts["UNEXECUTED"])+" UNEXECUTED; not RViz or a causal speedup")
    colors=("#5878a4","#b9d2e8","#71b77c","#f1bd62","#db915c","#b688b6","#a6a6a6")
    for index,(phase,color) in enumerate(zip(PHASES,colors)):
        for row in observed:
            if row[phase] is None:continue
            start=row[MARKS[index]];begin=row[MARKS[0]]
            if begin is None:continue
            axes[1].barh(row["ordinal"],row[phase],left=(start-begin)/1e6,color=color,label=phase if row is next((r for r in observed if r[phase] is not None),None) else None)
    denied=[r for r in observed if r["denied_output_entry_ms"] is not None]
    if denied:axes[1].scatter([r["denied_output_entry_ms"] for r in denied],[r["ordinal"] for r in denied],marker="x",color="black",s=60,label="Actual denied publish entry")
    for key,label,marker in (("trajectory_call_finish_ms","Actual Trajectory call finish","."),("path_call_finish_ms","Actual Path call finish","+")):
        measured=[r for r in observed if r[key] is not None]
        axes[1].scatter([r[key] for r in measured],[r["ordinal"] for r in measured],marker=marker,s=12,label=label,color="black")
    # HH_261008 - Every measured interval remains drawn; only dense tick labels are thinned, never the918 CSV/status denominator.
    stride=max(1,math.ceil(len(observed)/28));labelled=[r for i,r in enumerate(observed) if i%stride==0 or r is observed[-1] or r["status"]=="FAILED_RETAINED"]
    axes[1].axvline(100,color="red",linestyle="--");axes[1].set_yticks([r["ordinal"] for r in labelled],[str(r["ordinal"])+": "+r["status"]+(" +partialTraj" if r["partial_trajectory"] else "") for r in labelled]);axes[1].invert_yaxis()
    axes[1].tick_params(axis="y",labelsize=7);axes[1].set_xlabel("Actual publisher-relative intervals; NULL gaps unfilled; NN/audit nested, not added; local19-message stage sums are not wall")
    axes[1].legend(fontsize=6,loc="upper right");fig.tight_layout();buffer=BytesIO();fig.savefig(buffer,format="png",dpi=120);plt.close(fig)
    raw=buffer.getvalue();need(len(raw)<=PNG_LIMIT,"complete diagnostic PNG exceeds bound");return raw


def execute(plan_sha,authority_sha):
    started=time.monotonic()
    plan=read_json(PLAN,plan_sha,PLAN_LIMIT);grant=read_json(AUTHORITY,authority_sha,AUTH_LIMIT)
    need(grant==dict(schema=SCHEMA+".ROOT_authority",plan_path=str(PLAN),plan_sha256=plan_sha,source_sha256=plan["source_sha256"],
        provider_hash=plan["provider_hash"],metadata_and_matplotlib_only=True,NN_authorized=False,training_authorized=False,GPU_authorized=False,ROS_authorized=False,CARLA_authorized=False,TEST_authorized=False),"external ROOT authority required")
    for key,value in {"CUDA_VISIBLE_DEVICES":"","PYTHONNOUSERSITE":"1","OMP_NUM_THREADS":"1","MKL_NUM_THREADS":"1","OPENBLAS_NUM_THREADS":"1","PYTHONDONTWRITEBYTECODE":"1"}.items():need(os.environ.get(key)==value,"clean single-thread metadata environment required")
    need(not any(name in sys.modules for name in ("numpy","matplotlib","torch","rclpy","carla")),"fresh source-only process required before rendering")
    for key in ("PYTHONPATH","PYTHONHOME","MPLBACKEND","MATPLOTLIBRC","MATPLOTLIBDATA","MPLCONFIGDIR"):need(not os.environ.get(key),"unsealed plotting/import environment denied")
    need(not (Path.cwd()/"matplotlibrc").exists(),"unsealed working-directory plotting config denied")
    need(3 in os.sched_getaffinity(0),"declared CPU3 unavailable")
    with LOCK.open("r") as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);verify(plan);budget();need(not OUT.exists(),"fresh actual output only")
        need(time.monotonic()-started<=600,"sampled admission deadline exceeded")
        OUT.mkdir(parents=True);os.sched_setaffinity(0,{3})
        result=dict(schema=SCHEMA+".result",status="FAILED_RETAINED",expected_rows=1836,completed_checks=0,error=None,closing_errors=[],
            NN_inferences=0,training_steps=0,model_performance_improved=False,actual_RViz_screenshot=False,formal_main_approved=0,formal_sub_approved=0)
        try:
            codec=load_stdlib_source("tools/e2e/owned_lossless_jsonl_20261008_v2.py");bundle=load_stdlib_source("learning/portable_e2e/temporal_recorded_ros_bundle_v1_20261005.py")
            maths=gallery_math();all_rows={};summaries={};proofs={}
            for arm in ARMS:
                run,_,parent,_=parent_evidence(arm);index=bundle.load_index(parent)
                rows,proofs[arm]=read_ledgers(run,codec)
                witness=read_json(run/"subscriber_original_console_completion.json",limit=META_LIMIT)
                all_rows[arm],summaries[arm]=summarize_arm(arm,rows,witness,index.anchors,maths,
                    parent_identity=dict(replay_id=parent["replay_id"],authority_sha256=ARMS[arm]["authority"]))
                need(time.monotonic()-started<=600,"sampled metadata/report deadline exceeded");budget()
            paired=paired_summary(all_rows);table=[r for arm in ARMS for r in all_rows[arm]]
            text=StringIO(newline="");writer=csv.DictWriter(text,fieldnames=list(table[0]));writer.writeheader();writer.writerows(table)
            csv_pin=write_bytes(OUT/"complete918_per_arm.csv.gz",gzip.compress(text.getvalue().encode(),mtime=0),CSV_LIMIT)
            os.environ["MPLCONFIGDIR"]=str(OUT/"matplotlib_cache")
            os.environ["MATPLOTLIBRC"]=str(PROVIDER_CONFIG);os.environ["MPLBACKEND"]="Agg"
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from matplotlib import font_manager
            from matplotlib.backends import backend_agg
            provider_receipt=imported_provider_receipt(plan,plt,font_manager)
            charts={}
            for arm in ARMS:
                charts[arm]=write_bytes(OUT/(arm+"_all_actual_prefix_clocks.png"),chart(plt,arm,all_rows[arm],summaries[arm]),PNG_LIMIT)
                need(time.monotonic()-started<=600,"sampled render deadline exceeded");budget()
            result.update(status="COMPLETE_NUMERIC_DIAGNOSTIC_NOT_FEATURE_APPROVAL",completed_checks=1836,summary=summaries,paired=paired,
                codec_whole_reader_proof=proofs,selected_plotting_provider_receipt=provider_receipt,CSV_sha256=csv_pin,PNG_sha256=charts,clock_marks=list(MARKS),phases=list(PHASES),
                original_ACK_and_safety_gates_unchanged=True,local_stage_SUM_not_wall=True,raw_NN_and_audit_nested_not_additive=True,
                console_binding_and_closing_serialization_excluded_from_anchor_clock=True)
        except BaseException as error:result["error"]=repr(error)[:4096]
        finally:
            try:verify(plan)
            except BaseException as error:result["closing_errors"].append(repr(error)[:4096]);result["status"]="FAILED_RETAINED"
            need(not any(name in sys.modules for name in ("torch","rclpy","carla")),"no NN/ROS/CARLA imports allowed")
            result.update(wall_seconds=time.monotonic()-started,wall_scope="execute_entry_through_postseal_before_final_result_serialization",
                source_sha256=plan["source_sha256"],metadata_input_sha256=plan["input_sha256"],selected_provider_hash=plan["provider_hash"],fixed=fixed())
            write_bytes(OUT/"result.json",encode(result),META_LIMIT,closing=True);budget(closing=True)
        return dict(status=result["status"],output=str(OUT),completed_checks=result["completed_checks"])


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);parser.add_argument("--mode",choices=("prepare","execute"),required=True)
    parser.add_argument("--plan-sha256");parser.add_argument("--authority-sha256");args=parser.parse_args(argv)
    if args.mode=="prepare":
        with LOCK.open("r") as lock:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);result=prepare()
    else:
        need(args.plan_sha256 and args.authority_sha256,"two external raw ROOT SHA arguments required")
        result=execute(args.plan_sha256,args.authority_sha256)
    print(json.dumps(result,sort_keys=True));return 0 if result["status"]!="FAILED_RETAINED" else 2


if __name__=="__main__":raise SystemExit(main())
