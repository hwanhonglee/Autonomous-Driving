"""HH_261005 - SOURCE ONLY independent complete-success V7 auditor. Read-only stdlib, no Torch/native/GPU query/rebuild or ROOT authority creation; unavailable successful engines must fail."""

import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import re
import stat

from . import prepare_good6000_cnn_tensorrt_build_v7_20261005 as producer
from . import run_owned_ctrack_isolated_probe_20261001 as processes

ROOT, HERE, OUTPUT = producer.ROOT, Path(__file__).resolve(), producer.OUTPUT
SCHEMA = "portable_e2e.good6000_cnn_tensorrt_v7_complete_success_audit.20261005.v1"
TEST = ROOT / "learning/tests/test_good6000_cnn_tensorrt_v7_complete_success_v1_20261005.py"
sha, need, read_json = producer.seals.file_sha, producer.need, producer.producer.read_json
POST = OUTPUT / "ROOT_complete_independent_postcheck.json"


def read_parent_json(path):
    need(Path(path).stat().st_size<=producer.PARENT_JSON_LIMIT,"original bounded parent metadata required")
    return read_json(path)


def original_evidence_unchanged(recorded,fresh):
    # HH_261005 - Free bytes and the contemporaneous /proc census count are dynamic observations, not immutable original model/source evidence.
    dynamic={"current_bavail_bytes","process_metadata_records_checked"}
    need({key:value for key,value in recorded.items() if key not in dynamic}
        =={key:value for key,value in fresh.items() if key not in dynamic}
        and recorded["current_bavail_bytes"]>=producer.FLOOR+producer.SHARED+producer.CLOSING
        and recorded["process_metadata_records_checked"]>0,"complete original101/7assets/selected106runtime/source/archive audit differs")


def successful_interface(plan,result,result_sha):
    # HH_261005 - This exact shape is consumed by frozen runtime V2 engine_proof. ROOT alone reviews and seals an actual emitted interface; this function never writes it.
    return dict(schema=producer.SCHEMA+".ROOT_complete_independent_postcheck",status="PASS_OF_COMPLETE_SUCCESS",
        result_sha256=result_sha,source_sha256=plan["source_sha256"],engine_sha256=result["engine_sha256"],
        all_original101_exact14_sources_assets_archives_headers_libraries_and_owned_cleanup_verified=True)


def raw_streams(raw):
    need(len(raw["commands"])==3 and "acquisition_error" not in raw,"complete recorded raw three-query observation required")
    outputs=[]
    for row,options in zip(raw["commands"],(
        ["--query-gpu=index,uuid,memory.free,memory.used,utilization.gpu","--format=csv,noheader,nounits"],
        ["--query-compute-apps=pid,gpu_uuid","--format=csv,noheader,nounits"],[])):
        need(row["argv"]==[str(producer.NVIDIA_SMI),"-i","0",*options] and row.get("returncode")==0 and "error" not in row,
            "exact successful recorded GPU query command required")
        need(type(row["started_monotonic_ns"]) is int and type(row["finished_monotonic_ns"]) is int
            and row["started_monotonic_ns"]<=row["finished_monotonic_ns"] and 0<row["timeout_seconds"]<=3,"bounded recorded query chronology required")
        streams={}
        for name in ("stdout","stderr"):
            data=base64.b64decode(row[name+"_base64"],validate=True)
            need(row[name+"_truncated"] is False and len(data)==row[name+"_bytes"]<=32768
                and hashlib.sha256(data).hexdigest()==row[name+"_sha256"],"raw full stream byte seal differs")
            streams[name]=data.decode("utf8")
        outputs.append(streams["stdout"])
    return outputs


def verify_observation(raw,value,plan,*,native_core,native_argv,baseline=None):
    # HH_261005 - Audit captured historical identities against raw PID/type lists, never fabricate current identities for the now-ended native process.
    scalar,app_text,full=raw_streams(raw)
    lines=[line for line in scalar.splitlines() if line.strip()];need(len(lines)==1,"one recorded physicalGPU0 row required")
    index,uuid,free,used,utilization=[part.strip() for part in lines[0].split(",")]
    need((int(index),uuid,int(free),int(used),int(utilization))==tuple(value[key] for key in ("index","uuid","free_MiB","used_MiB","utilization_percent"))
        and int(index)==0 and uuid==plan["GPU_uuid"],"recorded GPU scalar/UUID differs")
    apps=[dict(pid=int(line.split(",")[0]),uuid=line.split(",")[1].strip()) for line in app_text.splitlines() if line.strip()]
    entries=[]
    for line in full.splitlines():
        match=re.match(r"^\|\s*0\s+\S+\s+\S+\s+(\d+)\s+(\S+)\s",line)
        if match:entries.append((int(match[1]),match[2]))
    identities=value["full_process_identities"];app_identities=value["app_process_identities"]
    need(all(row["identity"] is not None and row["state"] not in (None,"Z") for row in identities+app_identities),"historically unknown/dead identity denied")
    need(value["full_NVIDIA_GPU0_inventory"]==full and apps==value["apps"]
        and entries==[(row["identity"]["pid"],row["kind"]) for row in identities]
        and len(apps)==len(app_identities) and len({pid for pid,kind in entries})==len(entries)
        and len({row["pid"] for row in apps})==len(apps),"raw inventory/captured identities/order/duplicates differ")
    graphics=[row["identity"] for row in identities if row["kind"]=="G"]
    need(sorted(graphics,key=lambda row:row["pid"])==sorted(plan["existing_desktop_graphics"],key=lambda row:row["pid"]),"exact four desktop generations/argv differ")
    compute=[row for row in identities if row["kind"]!="G"]
    need(all(row["kind"]=="C" for row in compute) and len(compute)<=1 and len(apps)<=1,"foreign/unknown compute type/count denied")
    for row in compute+app_identities:
        identity=row["identity"]
        need(all(identity.get(key)==native_core[key] for key in ("pid","ppid","pgid","start_ticks"))
            and identity.get("argv")==native_argv,"historical owned compute generation/argv differs")
    need(all(row==dict(pid=native_core["pid"],uuid=plan["GPU_uuid"]) for row in apps),"foreign compute/UUID denied")
    if baseline is None:
        need(not compute and not apps and value["free_MiB"]>=4096 and value["utilization_percent"]<=35,"original known-desktop initial availability denied")
    else:need(value["used_MiB"]<=baseline["used_MiB"]+2048,"recorded added GPU memory cap denied")
    return [row["identity"]["pid"] for row in compute]==[row["pid"] for row in apps]


def audit_journal(plan,result):
    journal=result["GPU_observation_journal"];need(journal,"all raw GPU observations required")
    expected={};baseline=None;sample=1;coherent_values=[]
    for index,row in enumerate(journal,1):
        prefix="gpu_observations/%04d"%index
        need(row["raw_path"]==prefix+".raw.json" and row["identity_path"]==prefix+".identity.json"
            and row["sample"]==sample and 1<=sample<=3,"recorded bounded observation ordering differs")
        for name,pin in ((row["raw_path"],row["raw_sha256"]),(row["identity_path"],row["identity_sha256"])):
            need(sha(OUTPUT/name)==pin,"raw/historical identity archive SHA differs");expected[name]=pin
        raw,value=read_parent_json(OUTPUT/row["raw_path"]),read_parent_json(OUTPUT/row["identity_path"])
        need(raw["started_monotonic_ns"]<=raw["finished_monotonic_ns"],"recorded complete observation chronology differs")
        coherent=verify_observation(raw,value,plan,native_core=result["native_child_creation_core"],native_argv=result["actual_native_argv"],baseline=baseline)
        decision=read_parent_json(OUTPUT/(prefix+".decision.json"));need(decision==row,"complete observation decision/result differs")
        expected[prefix+".decision.json"]=sha(OUTPUT/(prefix+".decision.json"))
        need(row["status"]==("COHERENT_STRICT_VERIFIED" if coherent else "OWNED_ONLY_LIST_DISAGREEMENT_NOT_ACCEPTED"),"failed/incorrectly accepted observation denied")
        if coherent:
            if baseline is None:baseline=value
            coherent_values.append(value);sample=1
        else:sample+=1
    need(sample==1 and len(coherent_values)>=2 and result["actual_before_native_known_desktop_inventory"]==coherent_values[0]
        and result["actual_after_native_known_desktop_inventory"]==coherent_values[-1]
        and not coherent_values[-1]["apps"]
        and all(row["kind"]=="G" for row in coherent_values[-1]["full_process_identities"]),"first/closing exact desktop-only coherent observations required")
    return expected


def owned_closure(result):
    compiler,native=result["compile_child_creation_core"],result["native_child_creation_core"]
    argv=result["actual_native_argv"]
    need(len(argv)==8 and argv[:5]==[str(OUTPUT/"builder"),result["actual_before_native_known_desktop_inventory"]["uuid"],
        str(OUTPUT/"input/cnn_fixed6.onnx"),str(OUTPUT/"input/cnn_fixed30.onnx"),str(OUTPUT/"native")]
        and int(argv[5])==native["ppid"]==compiler["ppid"] and int(argv[7])>=3,"exact actual owned builder/parent/lease CLI differs")
    for core in (compiler,native):
        need(core["pid"]==core["pgid"]==core["sid"] and type(core["start_ticks"]) is int and core["start_ticks"]>0,
            "captured owned new-session generation invalid")
        current=producer.seals.process_core(core["pid"])
        need(current is None or current["start_ticks"]!=core["start_ticks"],"original owned child generation still exists")
    parent=producer.seals.process_core(int(argv[5]))
    need(parent is None or parent["start_ticks"]!=int(argv[6]),"original V7 owner generation still exists")
    need(not any(row["pgid"] in {compiler["pgid"],native["pgid"]} for row in processes.all_processes()),"full compiler/native group survivor denied; no signals")


def audit(plan_sha,authority_sha,result_sha,checker_sha):
    need(sha(HERE)==checker_sha,"externally supplied exact independent checker source SHA required")
    plan=producer.verify(plan_sha,authority_sha)
    need(len(plan["source_sha256"])==73 and plan["fixed_batches"]==[6,30],"complete original V7 closure/fixed-batch count differs")
    need((OUTPUT/"result.json").stat().st_size<=producer.PARENT_JSON_LIMIT,"bounded original producer result required")
    result=producer.seals.read_pinned(OUTPUT/"result.json",result_sha)
    need(result["schema"]==producer.SCHEMA+".result" and result["status"]=="PARSED_BUILT_FP32_FEASIBILITY_ONLY_NOT_RUNTIME_PARITY"
        and result["compile_exit_code"]==result["native_exit_code"]==0 and result["compile_elapsed_seconds"]<=30
        and result["raw14_initializers_and_original101_verified"] is True
        and not any(key.endswith("error") or key.endswith("errors") for key in result),"complete guarded producer success required")
    original=producer.producer.audit()
    recorded_original=read_parent_json(OUTPUT/"original_readonly_postcheck.json")
    original_evidence_unchanged(recorded_original,original)
    manifest=read_json(producer.producer.OUTPUT/"export/feasibility.json")
    initializers=[producer.initializer.inspect((OUTPUT/"input"/("cnn_fixed%d.onnx"%batch)).read_bytes(),batch,manifest["exact14_CNN_state"]) for batch in (6,30)]
    need(read_parent_json(OUTPUT/"initializer14_audit.json")==initializers,"raw14 initializer independent evidence differs")
    expected={}
    for name,row in plan["immutable_copy_manifest"].items():
        current=Path(row["source_path"]);archive=OUTPUT/name
        need(current.stat().st_size==row["bytes"]==archive.stat().st_size and sha(current)==row["sha256"]==sha(archive),"whole current/archive source/input bytes differ")
        expected[name]=row["sha256"]
    for name,pin in (("ROOT_plan.json",plan_sha),("ROOT_execution_authority.json",authority_sha)):
        need(sha(OUTPUT/name)==pin,"exact archived ROOT grant differs");expected[name]=pin
    need(result["compile_argv"]==plan["compile_argv"] and sha(OUTPUT/"builder")==result["compiled_builder_sha256"],"actual pinned compiler/binary differs")
    captured=[json.loads(line,parse_constant=lambda value:need(False,"nonfinite native JSON"))
        for line in (OUTPUT/"native.stdout.log").read_text().splitlines()]
    need(captured==result["all_planned_batches"] and [row["batch"] for row in captured]==[6,30]
        and all(row["status"]=="PARSED_BUILT_FP32_FEASIBILITY_ONLY" and row["precision_flags_readback_verified"] is True
            and row["TF32_FP16_INT8_BF16_FP8_enabled"] is False and row["error"] is None
            and row["runtime_or_cached_output_parity_verified"] is False
            and type(row["seconds"]) in (int,float) and math.isfinite(row["seconds"]) and 0<=row["seconds"]<=120 for row in captured)
        and sum(row["seconds"] for row in captured)<=120,"both complete native cold/warm precision rows differ")
    for batch,row in zip((6,30),captured):
        path=OUTPUT/"native"/("cnn_fixed%d.engine"%batch)
        need(0<path.stat().st_size<=32*1024**2 and sha(path)==result["engine_sha256"][str(batch)],"complete bounded engine bytes/SHA differ")
        expected[str(path.relative_to(OUTPUT))]=sha(path)
    expected.update(audit_journal(plan,result));owned_closure(result)
    for name in ("original_readonly_postcheck.json","initializer14_audit.json","compile.stdout.log","compile.stderr.log","builder",
        "native.stdout.log","native.stderr.log","result.json"):expected[name]=sha(OUTPUT/name)
    interface=successful_interface(plan,result,result_sha)
    if POST.exists():need(read_parent_json(POST)==interface,"existing ROOT post interface differs");expected[POST.name]=sha(POST)
    actual={};total=0;private_extras={}
    for path in OUTPUT.rglob("*"):
        mode=path.lstat().st_mode;need(stat.S_ISDIR(mode) or stat.S_ISREG(mode),"special/symlink complete-output object denied")
        if stat.S_ISREG(mode):
            name=str(path.relative_to(OUTPUT));size=path.stat().st_size;need(size<=32*1024**2,"individual complete-output cap denied")
            actual[name]=sha(path);total+=size
            if name not in expected:
                need(path.is_relative_to(OUTPUT/"Scratch") or path.is_relative_to(OUTPUT/"UserCache"),"undeclared non-cache output artifact denied")
                private_extras[name]=actual[name]
    need(actual==expected|private_extras and total==result["closing_used_bytes"]+(OUTPUT/"result.json").stat().st_size+(POST.stat().st_size if POST.exists() else 0)
        and total<=plan["attempt_quota_bytes"]-plan["closing_reserve_bytes"],"complete tree/closing bytes/budget differ")
    need(producer.header_seals()==(plan["sdk_header_alias_to_canonical"],plan["sdk_header_sha256"]),"selected raw/canonical SDK header binding changed")
    original_evidence_unchanged(recorded_original,producer.producer.audit())
    producer.verify(plan_sha,authority_sha);need(sha(OUTPUT/"result.json")==result_sha and sha(HERE)==checker_sha,"post audit source/result seal changed")
    return dict(schema=SCHEMA,status="COMPLETE_SUCCESS_INDEPENDENT_SCALAR_AUDIT_NOT_RUNTIME_PARITY",candidate_ROOT_interface=interface,
        complete_output_file_count=len(actual),complete_output_bytes=total,private_cache_extra_file_count=len(private_extras),
        full_output_file_sha256=actual,checker_source_sha256=checker_sha,checker_test_sha256=sha(TEST),
        both_raw14_initializers_and_original101_and_all_declared_archives_verified=True,
        raw_GPU_observation_count=len(result["GPU_observation_journal"]),historical_native_SID_not_in_full_inventory_identity_rows=True,
        original_parent_compiler_native_generations_and_groups_gone=True,transitive_native_driver_ABI_complete_claimed=False,
        actual_runtime_inference_numerical_parity_latency_or_feature_approved=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("plan-sha256","authority-sha256","result-sha256","expected-checker-sha256"):parser.add_argument("--"+name,required=True)
    args=parser.parse_args(argv)
    report=audit(args.plan_sha256,args.authority_sha256,args.result_sha256,args.expected_checker_sha256)
    print(json.dumps(report,sort_keys=True,indent=2,allow_nan=False));return 0


if __name__=="__main__":raise SystemExit(main())
