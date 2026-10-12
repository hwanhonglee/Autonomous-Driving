"""HH_261005 - SOURCE ONLY owning V2: fixed ROOT admission, serial CPU6/GPU0 full888 screens and generation-owned lifecycle; no authority is created by this module."""

import argparse
from dataclasses import asdict
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import shutil
import signal
import stat
import subprocess
import sys
import time

from . import prepare_good6000_cnn_tensorrt_build_v7_20261005 as build
from . import benchmark_full209_thread468_v1_20261004 as screen
from . import benchmark_full209_thread468_v2_20261004 as screen_revision
from . import benchmark_full209_masked_copy_v1_20261004 as recorded
from . import run_owned_ctrack_isolated_probe_20261001 as processes

ROOT, HERE = build.ROOT, Path(__file__).resolve()
MODULE = "scripts.e2e.prepare_good6000_cnn_tensorrt_runtime_v2_20261005"
SCHEMA = "portable_e2e.good6000_cnn_tensorrt_runtime.20261005.v2"
PLAN = ROOT / "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_runtime_v2/ROOT_plan.json"
AUTHORITY = PLAN.with_name("ROOT_execution_authority.json")
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-05/good6000_cnn_tensorrt_runtime_v2"
CPP = HERE.with_name("good6000_cnn_tensorrt_runtime_v2_20261005.cpp")
BACKEND = ROOT / "learning/portable_e2e/temporal_cached_tensorrt_backend_v2_20261005.py"
TESTS = [ROOT / "learning/tests/test_good6000_cnn_tensorrt_runtime_v2_20261005.py",
    ROOT / "learning/tests/test_temporal_cached_tensorrt_backend_v2_20261005.py"]
PREFIX = ["/usr/bin/python3", "-B", str(ROOT / "tools/workspace_python.py"), "-m", MODULE, "--"]
FROZEN_V1 = {
    "tools/e2e/good6000_cnn_tensorrt_runtime_v1_20261005.cpp": "aaba02ebeb0c28a9e48bf100fe4899d2137d1d566275e9e5caaa6748a44283d4",
    "learning/portable_e2e/temporal_cached_tensorrt_backend_v1_20261005.py": "6c3011cc9d041df1c7db4ac7897192a128423e72175854513a4b02ae487d6bfa",
    "learning/tests/test_temporal_cached_tensorrt_backend_v1_20261005.py": "14c8c063e1987098137321147a003ae4e012665c8186e59ad9db46a73d4f81b8",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_runtime_v1/SOURCE_ONLY_parity_protocol.md": "77af8053e5d407197dbe9b0a8ad3514dcdf68cabc7245ea0b399dc882071058a",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_runtime_v1/ROOT_source_review_only.json": "e28671be710d243399f15c2a6f76e64db30350e85f5f0764ef36b347fe4ca201",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_runtime_v1/source_prepared_review.json": "1500ec2081c2a5cf74f9dca599ef2ee3ed8a8d685c7d60bd7186c81383ac85d6",
    "tools/e2e/prepare_good6000_cnn_tensorrt_build_v7_20261005.py": "212f35abedfaa68c6a36d98e5a575a92ed603503a68c6208a127bcbff6ec8e9a",
    "learning/tests/test_good6000_cnn_tensorrt_build_v7_20261005.py": "a12aa82671ec1d18be996001e48c84dbfeb802eee901e8f7fc51998475dadd08",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v7/source_prepared_review.json": "f459a551e5df0cbcc27e25d0fb8362ea6cf075d30dd45f7149db83dbf84736e0",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v7/source_closure_and_prospective_contract.json": "ae1160ac9812637ef3e2dbaefbccd1ccabcfcc7ec62775afbef2cbfe95813df7"
}
need, sha, read_json = build.need, build.seals.file_sha, build.producer.read_json
FLOOR, SHARED, ATTEMPT, CLOSING = build.FLOOR, build.SHARED, build.ATTEMPT, build.CLOSING
JSON_LIMIT, FILE_LIMIT, ARM_SECONDS = 512*1024, 32*1024**2, 600
STAT_FIELDS = ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")


def exact(first, second):
    return json.dumps(first, sort_keys=True, allow_nan=False) == json.dumps(second, sort_keys=True, allow_nan=False)


def new_json(path, value):
    raw = (json.dumps(value, sort_keys=True, indent=2, allow_nan=False)+"\n").encode()
    need(len(raw) <= JSON_LIMIT, "bounded metadata before any write required")
    path = recorded.safe_path(Path(path), missing_tail=True)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream: stream.write(raw)


def complete_json(path,value):
    # HH_261005 - A create-only empty commit marker is opened only after the data FD closes; readers never parse a still-being-written handshake JSON.
    new_json(path,value)
    with Path(str(path)+".complete").open("xb"):pass


def append_row(folder, name, value):
    raw = (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)+"\n").encode()
    need(len(raw) <= 32768, "bounded complete scalar row required")
    budget(); path = recorded.safe_path(folder/name, missing_tail=True)
    need(not path.exists() or path.stat().st_size + len(raw) <= FILE_LIMIT, "all failed rows retained within file cap")
    with path.open("ab" if path.exists() else "xb") as stream: stream.write(raw)


def budget(startup=False):
    need(shutil.disk_usage(ROOT).free >= FLOOR+SHARED+(ATTEMPT if startup else CLOSING), "unchanged floor/shared/reserve denied")
    total = 0
    if OUTPUT.exists():
        for path in OUTPUT.rglob("*"):
            mode = path.lstat().st_mode
            need(stat.S_ISREG(mode) or stat.S_ISDIR(mode), "special/symlink output denied")
            if stat.S_ISREG(mode):
                need(path.stat().st_size <= FILE_LIMIT, "individual file quota denied"); total += path.stat().st_size
    need(total <= ATTEMPT-CLOSING, "complete census closing reserve exhausted")
    return total


def source_pins():
    pins = build.source_pins() | recorded.source_snapshot()
    for name, pin in FROZEN_V1.items():
        need(sha(ROOT/name) == pin, "frozen build/prototype dependency changed"); pins[name] = pin
    for path in [HERE, CPP, BACKEND, *TESTS, Path(screen.__file__), Path(screen_revision.__file__), screen.TEST,
        ROOT/"learning/tests/test_full209_thread468_v2_20261004.py"]:
        name = str(path.relative_to(ROOT)); pin = sha(path)
        need(name not in pins or pins[name] == pin, "original dependency changed"); pins[name] = pin
    return pins


def engine_proof(build_plan_sha, build_authority_sha, result_sha, postcheck_sha):
    # HH_261005 - Successful future V7 proof is mandatory, currently unavailable. Reject before output/compile/Torch; old failed V5/V6 cannot supply engines.
    plan = build.verify(build_plan_sha, build_authority_sha)
    result_path, post_path = build.OUTPUT/"result.json", build.OUTPUT/"ROOT_complete_independent_postcheck.json"
    result = build.seals.read_pinned(result_path, result_sha); post = build.seals.read_pinned(post_path, postcheck_sha)
    need(result.get("status") == "PARSED_BUILT_FP32_FEASIBILITY_ONLY_NOT_RUNTIME_PARITY"
        and result.get("native_exit_code") == 0 and result.get("compile_exit_code") == 0
        and not any(key.endswith("error") or key.endswith("errors") for key in result), "successful guarded V7 producer required")
    rows = result["all_planned_batches"]
    need([row["batch"] for row in rows] == [6,30] and all(row["status"] == "PARSED_BUILT_FP32_FEASIBILITY_ONLY"
        and row["precision_flags_readback_verified"] is True and row["TF32_FP16_INT8_BF16_FP8_enabled"] is False for row in rows),
        "both cold/warm FP32 producer rows required")
    need(post == dict(schema=build.SCHEMA+".ROOT_complete_independent_postcheck", status="PASS_OF_COMPLETE_SUCCESS",
        result_sha256=result_sha, source_sha256=plan["source_sha256"], engine_sha256=result["engine_sha256"],
        all_original101_exact14_sources_assets_archives_headers_libraries_and_owned_cleanup_verified=True),
        "exact ROOT independent full successful producer postcheck required")
    for destination, row in plan["immutable_copy_manifest"].items():
        need(sha(Path(row["source_path"])) == row["sha256"] == sha(build.OUTPUT/destination), "V7 current/archive seal differs")
    need(sha(build.OUTPUT/"ROOT_plan.json") == build_plan_sha and sha(build.OUTPUT/"ROOT_execution_authority.json") == build_authority_sha,
        "archived successful V7 ROOT grants differ")
    engines = {}
    for batch in (6,30):
        path = build.OUTPUT/"native"/("cnn_fixed%d.engine"%batch); pin = result["engine_sha256"][str(batch)]
        need(0 < path.stat().st_size <= FILE_LIMIT and sha(path) == pin, "exact successful bounded engine required")
        engines[str(batch)] = dict(path=str(path), sha256=pin, bytes=path.stat().st_size)
    return dict(build_plan_sha256=build_plan_sha, build_authority_sha256=build_authority_sha,
        result_path=str(result_path), result_sha256=result_sha, postcheck_path=str(post_path), postcheck_sha256=postcheck_sha,
        engine=engines)


def libraries():
    paths = build.libraries(); paths.pop("libnvonnxparser.so")
    return paths | {"libcrypto.so.3": Path("/usr/lib/x86_64-linux-gnu/libcrypto.so.3").resolve(strict=True)}


def header_seals():
    aliases, pins = build.header_seals()
    for path in (Path("/usr/include/openssl/sha.h"), Path("/usr/include/openssl/opensslv.h")):
        need(path == path.resolve(strict=True), "canonical selected OpenSSL headers required"); pins[str(path)] = sha(path)
    return aliases, pins


def compile_argv(paths):
    return [str(build.COMPILER), "-std=c++17", "-O2", "-fno-fast-math", "-I"+str(build.SDK/"include/x86_64-linux-gnu"),
        "-I"+str(build.CUDA/"include"), str(CPP), *map(str, paths.values()),
        "-Wl,-rpath,"+":".join(sorted({str(path.parent) for path in paths.values()})), "-o", str(OUTPUT/"cnn_runtime")]


def contract(build_plan_sha, build_authority_sha, result_sha, postcheck_sha):
    proof = engine_proof(build_plan_sha, build_authority_sha, result_sha, postcheck_sha)
    desktop = build.desktop_proof(); paths = libraries(); aliases, headers = header_seals(); sources = source_pins()
    producer_plan = read_json(build.producer.PLAN)
    events = screen.original_decoder_provenance(recorded.event_catalog(recorded.recorded.camera_catalog(),
        recorded.jsonl(recorded.EPISODE/"temporal_prefill.jsonl"), recorded.jsonl(recorded.EPISODE/"learned_control.jsonl")),
        recorded.jsonl(recorded.EPISODE/"learned_control.jsonl"))
    report_path = build.producer.OUTPUT/"export/feasibility.json"
    entries = [(ROOT/name, "source/"+name, pin) for name,pin in sources.items()]
    entries += [(Path(row["path"]), "engines/cnn_fixed%s.engine"%batch, row["sha256"]) for batch,row in proof["engine"].items()]
    entries += [(build.DESKTOP_PROOF,"desktop_metadata_evidence.json",build.DESKTOP_PROOF_SHA),
        (build.PLAN,"producer_ROOT_plan.json",build_plan_sha), (build.AUTHORITY,"producer_ROOT_authority.json",build_authority_sha),
        (Path(proof["result_path"]),"producer_result.json",result_sha), (Path(proof["postcheck_path"]),"producer_ROOT_postcheck.json",postcheck_sha),
        (report_path,"original101_exact14_manifest.json",sha(report_path))]
    copies = {name:dict(source_path=str(path), sha256=pin, bytes=path.stat().st_size) for path,name,pin in entries}
    need(sum(row["bytes"] for row in copies.values())+8*JSON_LIMIT < ATTEMPT-CLOSING, "immutable archive and metadata reserve exceed quota")
    return dict(schema=SCHEMA+".ROOT_plan", output=str(OUTPUT), source_sha256=sources, successful_engine_producer=proof,
        original101_exact14_manifest_sha256=sha(report_path), model_asset_sha256=producer_plan["model_asset_sha256"],
        runtime_source_sha256=producer_plan["runtime_source_sha256"], all_original_files=recorded.recorded.input_snapshot(),
        all888_event_catalog_sha256=screen.canonical_sha(events), fixed_arms=["CPU6","GPU0"], contexts=888,
        original_prefills=23, original_fresh_NN=865, original_ordinary=802, original_terminal_unverified=63,
        timed_contexts=recorded.selected_indices(), timed_calls_per_arm=30, cold_first_context_included=True,
        preload_CPU_threads=4, final_CPU_threads=6, interop_threads=1, OMP_NUM_THREADS="6", MKL_NUM_THREADS="6", OPENBLAS_NUM_THREADS="1",
        no_model_or_native_warmup=True, lossy_JPEG_not_original_BGRA=True, raw_atol=1e-6, raw_rtol=1e-6,
        original_full_forward_cached_and_all101_input_cache_callback_hook_guards=True, masked_copy_trust_forged=False,
        native_warm6_chunks_and_cold30=True, native_protocol_version=2, native_idle_timeout_seconds=10,
        setup_timeout_seconds=120, request_timeout_seconds=.100, arm_lifetime_seconds=ARM_SECONDS, closing_timeout_seconds=2,
        closing_owner_inventory_handoff_timeout_seconds=9,
        timer_includes_byte_copy_SHA_finite_transport_state_and_lightweight_ownership_source_stat_guards=True,
        GPU_inventories_periodic_external_owner_not_hidden_inside_NN_timer=True, owner_GPU_supervision_interval_seconds=1,
        GPU_query_and_budget_poll_gaps_honestly_retained=True, no_hard_native_or_SDK_scratch_syscall_bound_claimed=True,
        GPU_physical_index=0, GPU_uuid=desktop["gpu_uuid"], GPU_availability_scope="KNOWN_DESKTOP_NOT_IDLE",
        existing_desktop_graphics=desktop["NVIDIA_GPU0_process_identities"], initial_free_MiB=4096, initial_GPU_utilization_percent_maximum=35,
        maximum_added_GPU_used_MiB=2048, maximum_owned_only_coherence_samples=3, TF32_FP16_INT8_BF16_FP8_enabled=False,
        sdk_header_alias_to_canonical=aliases, sdk_header_sha256=headers, sdk_library_sha256={str(path):sha(path) for path in paths.values()},
        compiler_path=str(build.COMPILER), compiler_sha256=sha(build.COMPILER), nvidia_smi_path=str(build.NVIDIA_SMI), nvidia_smi_sha256=sha(build.NVIDIA_SMI),
        compile_argv=compile_argv(paths), compiler_timeout_seconds=30, compiler_wait_interval_seconds=.05,
        runtime_lock=str(build.seals.LOCK), free_floor_bytes=FLOOR, shared_reserve_bytes=SHARED,
        attempt_quota_bytes=ATTEMPT, closing_reserve_bytes=CLOSING, individual_file_limit_bytes=FILE_LIMIT,
        metadata_JSON_byte_limit=JSON_LIMIT, immutable_copy_manifest=copies, immutable_copy_bytes_total=sum(row["bytes"] for row in copies.values()),
        native_or_OpenSSL_transitive_binary_ABI_complete_claimed=False, model_or_runtime_promoted=False,
        real_BGRA_control_or_closed_loop_or_formal_feature_parity_verified=False)


def verify(plan_sha, authority_sha):
    need(all(type(pin) is str and re.fullmatch("[0-9a-f]{64}",pin) for pin in (plan_sha,authority_sha)), "external exact ROOT SHA arguments required")
    need(all(path.stat().st_size <= JSON_LIMIT for path in (PLAN,AUTHORITY)), "bounded fixed ROOT metadata required")
    plan, authority = build.seals.read_pinned(PLAN,plan_sha), build.seals.read_pinned(AUTHORITY,authority_sha)
    proof = plan["successful_engine_producer"]
    for name,pin in plan["model_asset_sha256"].items():
        need(sha(Path(name))==pin==sha(build.producer.OUTPUT/"archive"/Path(name).relative_to(ROOT)),"all seven current/archive model assets differ")
    for name,pin in plan["runtime_source_sha256"].items():
        need(sha(Path(name))==pin==sha(build.producer.OUTPUT/"archive_external"/Path(name).relative_to("/")),"selected runtime Python source current/archive differs")
    need(exact(plan,contract(proof["build_plan_sha256"],proof["build_authority_sha256"],proof["result_sha256"],proof["postcheck_sha256"])),
        "exact fixed fresh sources/assets/engines/full888/budget contract required")
    need(exact(authority,dict(schema=SCHEMA+".ROOT_execution_authority", explicit_private_CPU_compile_GPU0_CNN_and_full888_counterfactual_approved=True,
        plan_path=str(PLAN), plan_sha256=plan_sha, source_sha256=plan["source_sha256"], model_asset_sha256=plan["model_asset_sha256"],
        runtime_source_sha256=plan["runtime_source_sha256"], successful_engine_producer=proof,
        sdk_header_alias_to_canonical=plan["sdk_header_alias_to_canonical"], sdk_header_sha256=plan["sdk_header_sha256"],
        sdk_library_sha256=plan["sdk_library_sha256"], GPU_uuid=plan["GPU_uuid"], all_original_files=plan["all_original_files"],
        actual_simulator_or_control_or_TRAIN_or_TEST_or_model_promotion_authorized=False)), "exact ROOT authority required; no caller bool bypass")
    return plan


def observe_process(pid):
    first = build.seals.process_core(pid)
    if first is None: return None
    try:
        argv = [value.decode() for value in Path('/proc/'+str(pid)+'/cmdline').read_bytes().split(b'\0') if value]
        state = Path('/proc/'+str(pid)+'/stat').read_text().rsplit(')',1)[1].split()[0]
    except FileNotFoundError: return None
    need(build.seals.process_core(pid)==first, "process generation changed during observation")
    return dict(first, argv=argv, state=state)


def verify_lease(parent, fd):
    build.seals.verify_parent_lease(parent,fd)


def archive_verify(plan, plan_sha, authority_sha):
    for name,row in plan["immutable_copy_manifest"].items():
        need(sha(Path(row["source_path"]))==row["sha256"]==sha(OUTPUT/name), "complete current/archive binding differs")
    need(sha(OUTPUT/"ROOT_plan.json")==plan_sha and sha(OUTPUT/"ROOT_execution_authority.json")==authority_sha, "raw ROOT archive differs")


def source_stat_seal(plan):
    paths = [ROOT/name for name in plan["source_sha256"]] + [Path(name) for name in plan["model_asset_sha256"]|plan["runtime_source_sha256"]|plan["sdk_header_sha256"]|plan["sdk_library_sha256"]]
    paths += [PLAN,AUTHORITY,OUTPUT/"cnn_runtime",OUTPUT/"engines/cnn_fixed6.engine",OUTPUT/"engines/cnn_fixed30.engine"]
    return {str(path):[getattr(path.stat(),key) for key in STAT_FIELDS] for path in paths}


class OwnedTree:
    """HH_261005 - Only individually pinned current descendants are signalled using pidfds; no killpg and no foreign/reused generation fallback."""
    def __init__(self, process, core):
        need(core and core["pid"]==process.pid and core["ppid"]==os.getpid() and core["pgid"]==core["sid"]==process.pid,
            "owned new-session creation core required")
        self.process,self.root,self.known = process,core,{core["pid"]:core}

    def track(self):
        rows = processes.all_processes(); changed = True
        while changed:
            changed = False
            for row in rows:
                if row["pid"] not in self.known and row["ppid"] in self.known:
                    parent = self.known[row["ppid"]]; core = build.seals.process_core(row["pid"])
                    need(core and build.seals.process_core(parent["pid"])==parent and core["start_ticks"]>=parent["start_ticks"], "live known ancestry required")
                    need(len(self.known)<64,"bounded owned descendant census exceeded"); self.known[core["pid"]]=core; changed=True
        return rows

    def _live_owned(self, core):
        current = build.seals.process_core(core["pid"])
        if current is None: return False
        need(current==core,"changed/reused owned generation denied before signal")
        parent = self.known.get(core["ppid"])
        need((core==self.root and core["ppid"]==os.getpid()) or (parent is not None and build.seals.process_core(parent["pid"])==parent),
            "current pinned ancestry required before individual signal")
        return True

    def stop(self):
        result = dict(status="FAIL_NOT_VERIFIED", signalled=[], errors=[]); deadline=time.monotonic()+4
        try:
            try:self.track()
            except BaseException as error:result["errors"].append("tracking: "+type(error).__name__+": "+str(error))
            pending = sorted(self.known.values(),key=lambda row:row["start_ticks"],reverse=True)
            for core in pending:
                fd=None
                try:
                    if not self._live_owned(core): continue
                    fd=os.pidfd_open(core["pid"],0)
                    for signum in (signal.SIGTERM,signal.SIGKILL):
                        if not self._live_owned(core): break
                        need(time.monotonic()<deadline,"bounded owned cleanup deadline exhausted")
                        signal.pidfd_send_signal(fd,signum); result["signalled"].append(dict(core=core,signal=int(signum)))
                        end=min(deadline,time.monotonic()+.2)
                        while time.monotonic()<end and build.seals.process_core(core["pid"])==core:
                            self.process.poll(); time.sleep(.01)
                except BaseException as error:result["errors"].append("individual cleanup: "+type(error).__name__+": "+str(error))
                finally:
                    if fd is not None:os.close(fd)
            self.process.wait(timeout=max(.001,deadline-time.monotonic()))
            survivors=[row for row in processes.all_processes() if row["pgid"] in {core["pgid"] for core in self.known.values()}]
            need(not survivors,"full group survivor census failed; unknown generations are not signalled")
            if not result["errors"]:result["status"]="OWNED_GENERATIONS_GONE"
        except BaseException as error:result["errors"].append(type(error).__name__+": "+str(error))
        return result


def gpu_observe(plan,journal,*,deadline,baseline=None,native_core=None,native_argv=None):
    # HH_261005 - Reuse frozen parsing/ownership math, but write exclusively to V2 output without altering V7 globals. Query gaps remain recorded outside NN calls.
    for sample in range(1,4):
        raw=build.gpu_snapshot(deadline);prefix="gpu_observations/%04d"%(len(journal)+1)
        row=dict(sample=sample,status="FAIL_RETAINED_NO_RETRY",raw_path=prefix+".raw.json")
        new_json(OUTPUT/row["raw_path"],raw);row["raw_sha256"]=sha(OUTPUT/row["raw_path"]);journal.append(row)
        try:
            need(time.monotonic()<=deadline,"fixed supervision deadline reached")
            value=build.parse_gpu_observation(raw);new_json(OUTPUT/(prefix+".identity.json"),value)
            coherent=build.owned_only_coherence(value,plan,baseline=baseline,native_core=native_core,native_argv=native_argv)
            if coherent:build.verify_gpu(value,plan,baseline=baseline,native_core=native_core,native_argv=native_argv)
            budget();need(time.monotonic()<=deadline,"fixed supervision budget deadline reached")
            row["status"]="COHERENT_STRICT_VERIFIED" if coherent else "OWNED_ONLY_LIST_DISAGREEMENT_NOT_ACCEPTED"
        except BaseException as error:
            row["error"]=type(error).__name__+": "+str(error);new_json(OUTPUT/(prefix+".decision.json"),row);raise
        new_json(OUTPUT/(prefix+".decision.json"),row)
        if coherent:return value
    raise RuntimeError("three retained owned-only samples exhausted; no silent retry")


def worker_command(arm,plan_sha,authority_sha,parent,fd):
    return PREFIX+["--mode","worker","--arm",arm,"--plan-sha256",plan_sha,"--authority-sha256",authority_sha,
        "--parent-pid",str(parent["pid"]),"--parent-start",str(parent["start_ticks"]),"--runtime-lease-fd",str(fd)]


def failure_denominator():
    return dict(schema=SCHEMA+".result",status="FAIL_RETAINED",planned_arms=["CPU6","GPU0"],
        all_arms=[dict(arm=arm,status="UNEXECUTED_NOT_VERIFIED",planned_contexts=888,planned_timers=30,
            complete_context_status=["UNEXECUTED_NOT_VERIFIED"]*888,
            timed_contexts=recorded.selected_indices(),complete_timing_status=["UNEXECUTED_NOT_VERIFIED"]*30) for arm in ("CPU6","GPU0")],
        GPU_observation_journal=[])


def retain_child_result(row,arm):
    # HH_261005 - A failed worker's authentic complete ledger is evidence too; import it before exit-code admission without promoting failure to PASS.
    path=OUTPUT/arm/"result.json"
    row.update(child_ledger_status="NOT_YET_VERIFIED",complete_context_status=["UNKNOWN_NOT_RECORDED"]*888,
        complete_timing_status=["UNKNOWN_NOT_RECORDED"]*30)
    if not path.exists():
        row.update(child_ledger_status="ABSENT_UNVERIFIED",complete_context_status=["UNKNOWN_NOT_RECORDED"]*888,
            complete_timing_status=["UNKNOWN_NOT_RECORDED"]*30)
        return
    need(path.stat().st_size<=JSON_LIMIT,"bounded child failure ledger required")
    pin=sha(path);child=read_json(path);need(sha(path)==pin,"child failure ledger changed while reading")
    contexts,timers=child["complete_context_status"],child["complete_timing_status"]
    need(child["schema"]==SCHEMA+".arm" and child["arm"]==arm and child["planned_contexts"]==888 and child["planned_timers"]==30
        and len(contexts)==888 and len(timers)==30 and all(value in ("UNEXECUTED_NOT_VERIFIED","FAIL_NOT_VERIFIED","COMPLETED") for value in contexts)
        and all(value in ("UNEXECUTED_NOT_VERIFIED","COMPLETED","REJECTED") for value in timers)
        and contexts.count("COMPLETED")==child["completed_contexts"]
        and sum(value!="UNEXECUTED_NOT_VERIFIED" for value in timers)==child["attempted_timers"]
        and 0<=child["reconstructed_camera_bundles"]<=901,"complete failed worker denominator malformed")
    row.update(child_ledger_status="AUTHENTIC_COMPLETE_PLANNED_DENOMINATOR_RETAINED",child_result_status=child["status"],
        result_sha256=pin,complete_context_status=contexts,complete_timing_status=timers,
        completed_contexts=child["completed_contexts"],attempted_timers=child["attempted_timers"],
        reconstructed_camera_bundles=child["reconstructed_camera_bundles"])
    for name in ("all888_context_rows.jsonl","all30_timing_rows.jsonl","context_attempts.jsonl"):
        retained=OUTPUT/arm/name
        if retained.exists():row[name+"_sha256"]=sha(retained)


def client_context(plan_sha,authority_sha,lease_fd):
    # Admission is heavyweight exactly once before Popen, then stat/PID/lease guards remain inside every measured request.
    plan=verify(plan_sha,authority_sha);archive_verify(plan,plan_sha,authority_sha);budget()
    owner=read_json(OUTPUT/"owner_creation.json");parent=owner["core"]
    need(owner["argv"]==PREFIX+["--mode","execute","--plan-sha256",plan_sha,"--authority-sha256",authority_sha]
        and observe_process(parent["pid"])["argv"]==owner["argv"],"exact ROOT-admitted parent CLI required")
    verify_lease(parent,lease_fd)
    actual=observe_process(os.getpid())
    need(actual["argv"]==worker_command("GPU0",plan_sha,authority_sha,parent,lease_fd),"exact fixed GPU worker CLI required")
    need(os.environ.get("CUDA_VISIBLE_DEVICES")==plan["GPU_uuid"] and os.environ.get("NVIDIA_TF32_OVERRIDE")=="0",
        "exact physicalGPU0 FP32 environment required")
    compiler=read_json(OUTPUT/"compile_result.json")
    need(compiler["exit_code"]==0 and compiler["cleanup"]["status"]=="OWNED_GENERATIONS_GONE"
        and sha(OUTPUT/"cnn_runtime")==compiler["binary_sha256"],"successful owned pinned compiler result required")
    return plan,parent,source_stat_seal(plan)


def client_guard(client):
    # HH_261005 - Small fixed ROOT metadata and observed CLIs bind the object on every entry; an object.__new__ caller dictionary cannot manufacture admission.
    plan=build.seals.read_pinned(PLAN,client.plan_sha);authority=build.seals.read_pinned(AUTHORITY,client.authority_sha)
    need(exact(plan,client.plan) and authority["schema"]==SCHEMA+".ROOT_execution_authority"
        and authority["plan_path"]==str(PLAN) and authority["plan_sha256"]==client.plan_sha
        and authority["explicit_private_CPU_compile_GPU0_CNN_and_full888_counterfactual_approved"] is True,"current fixed ROOT admission differs")
    need(observe_process(os.getpid())["argv"]==worker_command("GPU0",client.plan_sha,client.authority_sha,client.parent,client.lease_fd)
        and observe_process(client.parent["pid"])["argv"]==PREFIX+["--mode","execute","--plan-sha256",client.plan_sha,"--authority-sha256",client.authority_sha],
        "exact currently admitted owner/worker CLIs required")
    verify_lease(client.parent,client.lease_fd)
    need(build.seals.process_core(os.getpid())==client.worker_core and time.monotonic()<client.lifetime_deadline,
        "worker generation/lifetime changed")
    need(not (OUTPUT/"GPU0/owner_supervision_failed.json").exists(),"retained external GPU supervision failure")
    for name,seal in client.stat_seals.items():
        path=Path(name);need(path==path.resolve(strict=True) and [getattr(path.stat(),key) for key in STAT_FIELDS]==seal,
            "source/asset/binary/engine generation changed inside request")
    need({name:str(Path(name).resolve(strict=True)) for name in client.plan["sdk_header_alias_to_canonical"]}
        ==client.plan["sdk_header_alias_to_canonical"],"SDK header resolution changed")


def native_argv(plan,parent,lease_fd):
    return [str(OUTPUT/"cnn_runtime"),plan["GPU_uuid"],str(OUTPUT/"engines/cnn_fixed6.engine"),str(OUTPUT/"engines/cnn_fixed30.engine"),
        plan["successful_engine_producer"]["engine"]["6"]["sha256"],plan["successful_engine_producer"]["engine"]["30"]["sha256"],
        str(parent["pid"]),str(parent["start_ticks"]),str(lease_fd),"HH_CNN_FP32_BYTE_V2"]


def compile_owned(plan,env):
    started=time.monotonic();proc=None;owned=None;row=dict(status="FAIL_RETAINED",exit_code=None,cleanup=dict(status="NO_CHILD"))
    try:
        need(sha(build.COMPILER)==plan["compiler_sha256"],"current compiler binary seal differs")
        need(header_seals()==(plan["sdk_header_alias_to_canonical"],plan["sdk_header_sha256"]),"precompile alias/bytes differ")
        with (OUTPUT/"compile.stdout.log").open("xb") as out,(OUTPUT/"compile.stderr.log").open("xb") as err:
            proc=subprocess.Popen(plan["compile_argv"],cwd=ROOT,env=env,stdout=out,stderr=err,start_new_session=True)
            core=build.seals.process_core(proc.pid);row["creation_core"]=core;owned=OwnedTree(proc,core)
            while proc.poll() is None:
                owned.track();budget();need(build.seals.process_core(proc.pid)==core or proc.poll() is not None,"compiler generation changed")
                remaining=started+30-time.monotonic();need(remaining>0,"fixed compiler deadline exceeded")
                try:proc.wait(timeout=min(.05,remaining))
                except subprocess.TimeoutExpired:pass
            row["exit_code"]=proc.returncode;need(proc.returncode==0,"private compile failed; logs retained")
        need(header_seals()==(plan["sdk_header_alias_to_canonical"],plan["sdk_header_sha256"]),"postcompile alias/bytes differ")
        row.update(status="COMPILED_NOT_RUNTIME_PARITY",binary_sha256=sha(OUTPUT/"cnn_runtime"))
    except BaseException as error:row["error"]=type(error).__name__+": "+str(error)
    finally:
        if owned is not None:row["cleanup"]=owned.stop()
        if proc is not None:row["exit_code"]=proc.returncode
        row["seconds"]=time.monotonic()-started
        if row["cleanup"]["status"]!="OWNED_GENERATIONS_GONE":row["status"]="FAIL_RETAINED"
        new_json(OUTPUT/"compile_result.json",row)
    need(row["status"]=="COMPILED_NOT_RUNTIME_PARITY","owned compiler failure retained; no retry")
    return row


def worker(args):
    plan=verify(args.plan_sha256,args.authority_sha256);archive_verify(plan,args.plan_sha256,args.authority_sha256);budget()
    owner=read_json(OUTPUT/"owner_creation.json");parent=owner["core"]
    need(parent["pid"]==args.parent_pid and parent["start_ticks"]==args.parent_start,"exact declared parent generation required")
    verify_lease(parent,args.runtime_lease_fd)
    need(observe_process(os.getpid())["argv"]==worker_command(args.arm,args.plan_sha256,args.authority_sha256,parent,args.runtime_lease_fd),"exact worker argv required")
    before_inputs=recorded.recorded.input_snapshot();need(before_inputs==plan["all_original_files"],"complete original archive changed")
    cameras=recorded.recorded.camera_catalog();learned=recorded.jsonl(recorded.EPISODE/"learned_control.jsonl")
    events=screen.original_decoder_provenance(recorded.event_catalog(cameras,recorded.jsonl(recorded.EPISODE/"temporal_prefill.jsonl"),learned),learned)
    need(screen.canonical_sha(events)==plan["all888_event_catalog_sha256"],"original complete causal contexts differ")
    arm=OUTPUT/args.arm;arm.mkdir(exist_ok=False);timings=[];completed=0;reconstructed=0;client=None;model=None;baseline=None
    result=dict(schema=SCHEMA+".arm",arm=args.arm,status="FAIL_RETAINED",planned_contexts=888,planned_timers=30,
        complete_context_status=["UNEXECUTED_NOT_VERIFIED"]*888,complete_timing_status=["UNEXECUTED_NOT_VERIFIED"]*30)
    try:
        import torch
        screen.initialize_worker_runtime(torch)
        from portable_e2e.simulation_research import ControllerConfig
        from portable_e2e.temporal_single_copy_history_research_20261001 import SingleCopyOwnedHeadroomPolicyV2
        from portable_e2e import temporal_cached_tensorrt_backend_v2_20261005 as backend
        loaded=recorded.models();screen.set_threads_after_load(torch,6)
        original=loaded["A"];model=loaded["B"] if args.arm=="CPU6" else original
        controller=ControllerConfig(**read_json(recorded.EPISODE/"learned_summary.json")["controller"])
        catalog={row["camera_ordinal"]:(index,row) for index,row in enumerate(events)};selected={index:pair for pair,index in enumerate(recorded.selected_indices())}
        builder=None
        with torch.no_grad():
            for ordinal,frame,past,identity in recorded.reconstructed_frames(cameras,recorded.jsonl(recorded.EPISODE/"states.jsonl")):
                reconstructed+=1
                if ordinal not in catalog:continue
                index,event=catalog[ordinal];append_row(arm,"context_attempts.jsonl",dict(context_index=index,status="STARTED"));result["complete_context_status"][index]="FAIL_NOT_VERIFIED"
                if builder is None:builder=SingleCopyOwnedHeadroomPolicyV2(model,controller,longitudinal_pi=True,**identity)
                tensors,window=builder.build_inputs(frame,past_frames=past,now_ns=frame.timestamp_ns,ego_stamp_ns=frame.timestamp_ns,route_stamp_ns=frame.timestamp_ns)
                # Start only after strict model and first actual reconstructed input are ready; never idle through the CPU arm or reconstructor setup.
                if args.arm=="GPU0" and client is None:
                    client=backend.PreparedTensorRTClientV2(args.plan_sha256,args.authority_sha256,args.runtime_lease_fd)
                    model=backend.create_backend_model(original,client)
                if baseline is None:baseline=model._state_identity()
                need(torch.get_num_threads()==6 and torch.get_num_interop_threads()==1,"one-time runtime configuration changed")
                inputs=recorded.tensor_hashes(tensors)
                need(all(inputs[name]==pin for name,pin in event["original_input_sha256"].items() if name!="images") and bool(tensors["time_valid"].all().item()),"all actual nonimage/causal inputs differ")
                started=time.perf_counter_ns() if index in selected else None;error=None
                try:raw=model.forward_cached(tensors,window=window)
                except BaseException as caught:error=caught;raise
                finally:
                    if started is not None:
                        elapsed=time.perf_counter_ns()-started;pair=selected[index]
                        status="COMPLETED" if error is None else "REJECTED";result["complete_timing_status"][pair]=status
                        timing=dict(pair=pair,context_index=index,elapsed_ns=elapsed,elapsed_ms=elapsed/1e6,status=status,first_cold=index==0,
                            exception=None if error is None else type(error).__name__+": "+str(error),no_deadline_filter=True)
                        timings.append(timing);append_row(arm,"all30_timing_rows.jsonl",timing)
                values=screen.raw_values(raw)
                need(inputs==recorded.tensor_hashes(tensors) and model._state_identity()==baseline and not model._recovery_active
                    and model._recovery_features is None and model.image_encoder.request is None,"complete input101/callback/cache/recovery cleanup changed")
                decoded=screen.ordinary_decoder_screen(raw,tensors)
                if event["kind"]=="original_fresh_NN":need(decoded["causal_navigation_support_m"]==event["original_causal_support_m"],"actual decoder support differs")
                row=dict(context_index=index,camera_ordinal=ordinal,native_frame=event["native_frame"],kind=event["kind"],
                    anchor_timestamp_ns=event["anchor_timestamp_ns"],status="COMPLETED",input_sha256=inputs,cache_window_sha256=screen.canonical_sha(asdict(window)),
                    raw_output=values,raw_output_sha256=recorded.tensor_hashes(raw),cache=screen.compact_cache(model),ordinary_decoder=decoded,
                    original_decoder_role=event["original_decoder_role"],all101_inputs_callbacks_state_unchanged=True,original_BGRA_or_engine_parity_claimed=False)
                append_row(arm,"all888_context_rows.jsonl",row);completed+=1;result["complete_context_status"][index]="COMPLETED"
        need(reconstructed==901 and completed==888 and len(timings)==30,"complete cold/intermediate/tail denominator required")
        need(model._state_identity()==baseline,"poststate changed")
        result.update(status="COMPLETE_RAW_OUTPUTS_NOT_YET_COMPARATIVE_PARITY",timing=recorded.recorded.stats([row["elapsed_ms"] for row in timings]),
            all_context_rows_sha256=sha(arm/"all888_context_rows.jsonl"),all_timing_rows_sha256=sha(arm/"all30_timing_rows.jsonl"))
    except BaseException as error:result["error"]=type(error).__name__+": "+str(error)
    finally:
        if client is not None:
            result["native_close"]=client.close()
            if result["native_close"]["status"]!="NORMAL_ACK_EXIT_AND_OWNED_GENERATIONS_GONE":result["status"]="FAIL_RETAINED"
        try:
            need(model is None or baseline is None or model._state_identity()==baseline,"closing all101 state differs")
            need(before_inputs==recorded.recorded.input_snapshot(),"closing complete original archive changed")
            verify(args.plan_sha256,args.authority_sha256);archive_verify(plan,args.plan_sha256,args.authority_sha256);budget()
        except BaseException as error:result.update(status="FAIL_RETAINED",closing_error=type(error).__name__+": "+str(error))
        result.update(completed_contexts=completed,reconstructed_camera_bundles=reconstructed,attempted_timers=len(timings))
        new_json(arm/"result.json",result)
    return 0 if result["status"]=="COMPLETE_RAW_OUTPUTS_NOT_YET_COMPARATIVE_PARITY" else 2


def execute(plan_sha,authority_sha):
    with build.seals.runtime_lease() as fd:
        plan=verify(plan_sha,authority_sha);budget(startup=True)
        need(not OUTPUT.exists() and OUTPUT.parent.is_dir(),"fresh output required; no retry")
        # All actual producer/assets are independently verified only after fixed admission, never during SOURCEONLY preparation.
        build.producer.audit();report=read_json(build.producer.OUTPUT/"export/feasibility.json")
        for batch in (6,30):build.initializer.inspect((build.producer.OUTPUT/"export"/("cnn_fixed%d.onnx"%batch)).read_bytes(),batch,report["exact14_CNN_state"])
        OUTPUT.mkdir(mode=0o700,exist_ok=False);resource.setrlimit(resource.RLIMIT_CORE,(0,0));resource.setrlimit(resource.RLIMIT_FSIZE,(FILE_LIMIT,FILE_LIMIT))
        result=failure_denominator()
        active=None;owned=None;baseline=None
        try:
            for name,row in plan["immutable_copy_manifest"].items():
                source=Path(row["source_path"]);need(source.stat().st_size==row["bytes"] and sha(source)==row["sha256"],"immutable copy changed before write")
                dest=OUTPUT/name;dest.parent.mkdir(parents=True,exist_ok=True)
                with source.open("rb") as src,dest.open("xb") as out:shutil.copyfileobj(src,out,1024**2)
                need(sha(dest)==row["sha256"],"exact raw archive differs")
            for source,name,pin in ((PLAN,"ROOT_plan.json",plan_sha),(AUTHORITY,"ROOT_execution_authority.json",authority_sha)):
                need(source.stat().st_size<=JSON_LIMIT,"ROOT metadata oversize before copy")
                with source.open("rb") as src,(OUTPUT/name).open("xb") as out:shutil.copyfileobj(src,out)
                need(sha(OUTPUT/name)==pin,"raw ROOT bytes differ")
            archive_verify(plan,plan_sha,authority_sha)
            parent=build.seals.process_core(os.getpid());new_json(OUTPUT/"owner_creation.json",dict(core=parent,argv=observe_process(parent["pid"])["argv"]))
            env=build.seals.environment();env.update(CUDA_VISIBLE_DEVICES="",NVIDIA_TF32_OVERRIDE="0",CUDA_CACHE_DISABLE="1",
                TMPDIR=str(OUTPUT/"Scratch"),XDG_CACHE_HOME=str(OUTPUT/"UserCache"),CUDA_CACHE_PATH=str(OUTPUT/"UserCache/CUDA"),
                LD_LIBRARY_PATH=":".join(sorted({str(path.parent) for path in libraries().values()})))
            for folder in ("Scratch","UserCache","gpu_observations"):(OUTPUT/folder).mkdir()
            compile_owned(plan,env)
            for index,arm in enumerate(("CPU6","GPU0")):
                if arm=="GPU0":baseline=gpu_observe(plan,result["GPU_observation_journal"],deadline=time.monotonic()+9)
                command=worker_command(arm,plan_sha,authority_sha,parent,fd);armenv=dict(env,CUDA_VISIBLE_DEVICES=plan["GPU_uuid"] if arm=="GPU0" else "")
                start=time.monotonic();native_core=None;native_command=None
                with (OUTPUT/(arm+".stdout.log")).open("xb") as out,(OUTPUT/(arm+".stderr.log")).open("xb") as err:
                    active=subprocess.Popen(command,cwd=ROOT,env=armenv,stdout=out,stderr=err,start_new_session=True,pass_fds=(fd,))
                    core=build.seals.process_core(active.pid);owned=OwnedTree(active,core);new_json(OUTPUT/(arm+".creation.json"),dict(core=core,argv=command))
                    result["all_arms"][index].update(status="STARTED_FAIL_NOT_VERIFIED",creation_core=core,argv=command)
                    last=0
                    while active.poll() is None:
                        owned.track();budget();need(time.monotonic()<start+ARM_SECONDS,"fixed full arm lifetime exceeded")
                        need(build.seals.process_core(active.pid)==core or active.poll() is not None,"worker generation changed")
                        receipt=OUTPUT/arm/"native_creation.json"
                        marker=Path(str(receipt)+".complete")
                        closing=OUTPUT/arm/"native_close_started.json"
                        request=OUTPUT/arm/"native_close_requested.json"
                        if arm=="GPU0" and Path(str(request)+".complete").exists() and native_core is not None:
                            req=read_json(request);need(req["core"]==native_core and req["failed"] is False,"exact normal closing handoff required")
                            acknowledgement=OUTPUT/arm/"native_close_owner_ack.json"
                            if not acknowledgement.exists():
                                need(build.seals.process_core(native_core["pid"])==native_core,"live pinned native before inventory handoff required")
                                complete_json(acknowledgement,dict(core=native_core,sequence=req["sequence"],status="NO_GPU_OBSERVATION_IN_FLIGHT"))
                        if arm=="GPU0" and closing.exists() and native_core is not None and build.seals.process_core(native_core["pid"])==native_core:
                            close=read_json(closing)
                            need(time.monotonic_ns()<=close["started_monotonic_ns"]+2*10**9,"owned native normal-exit hard watchdog exceeded")
                        if arm=="GPU0" and marker.exists() and not Path(str(request)+".complete").exists() and not closing.exists() and time.monotonic()-last>=1:
                            launch=read_json(receipt);native_core=launch["core"];native_command=native_argv(plan,core,fd)
                            need(launch["argv"]==native_command,"exact descendant native argv differs")
                            from portable_e2e.temporal_cached_tensorrt_backend_v2_20261005 import verify_native_identity
                            verify_native_identity(observe_process(native_core["pid"]),native_core,native_command)
                            gpu_observe(plan,result["GPU_observation_journal"],deadline=min(start+ARM_SECONDS,time.monotonic()+9),baseline=baseline,native_core=native_core,native_argv=native_command)
                            admission=OUTPUT/arm/"native_supervision_admitted.json"
                            if not admission.exists():complete_json(admission,dict(core=native_core,argv=native_command,status="COHERENT_STRICT_VERIFIED"))
                            last=time.monotonic()
                        try:active.wait(timeout=.05)
                        except subprocess.TimeoutExpired:pass
                result["all_arms"][index].update(exit_code=active.returncode,seconds=time.monotonic()-start)
                cleanup=owned.stop();result["all_arms"][index]["cleanup"]=cleanup;owned=None
                retain_child_result(result["all_arms"][index],arm)
                need(active.returncode==0 and cleanup["status"]=="OWNED_GENERATIONS_GONE","arm failure retained; no restart")
                result["all_arms"][index].update(status="COMPLETE",result_sha256=sha(OUTPUT/arm/"result.json"));active=None
            first,second=[recorded.jsonl(OUTPUT/arm/"all888_context_rows.jsonl") for arm in ("CPU6","GPU0")]
            need(len(first)==len(second)==888,"whole888 outputs required")
            rejected=0;rawpass=True;available=True;count=0
            for a,b in zip(first,second):
                need(all(a[key]==b[key] for key in ("context_index","camera_ordinal","native_frame","kind","anchor_timestamp_ns","input_sha256","cache_window_sha256","original_decoder_role")),"identical causal inputs/denominator required")
                raw=screen.raw_comparison(a["raw_output"],b["raw_output"]);decoder=screen.decoder_comparison(a["ordinary_decoder"],b["ordinary_decoder"])
                rawpass=rawpass and all(row["allclose"] for row in raw.values())
                if decoder["status"]=="ORDINARY_GO_COUNTERFACTUAL_ONLY":count+=1;available=available and decoder["ordinary_numeric_screen_passed"]
                rejected+=any(row["status"]=="ORDINARY_DECODER_REJECTED" for row in (a["ordinary_decoder"],b["ordinary_decoder"]))
                append_row(OUTPUT,"all888_comparison_rows.jsonl",dict(context_index=a["context_index"],raw5head=raw,ordinary_decoder=decoder,
                    retained_cache_feature_bitwise_diagnostic_equal=a["cache"]==b["cache"],not_original_BGRA_or_full_engine_parity=True))
            result.update(status="PASS_COUNTERFACTUAL_ONLY" if rawpass and available and rejected==0 and count>0 else "FAIL_RETAINED",
                all888_raw5head_strict1e6_passed=rawpass,available_ordinary_geometry_passed=available,available_ordinary_count=count,
                retained_decoder_rejections=rejected,terminal_proof_and_actual_BGRA_unverified=True,actual_cycle_gain_or_feature_approval=False)
        except BaseException as error:
            result["error"]=type(error).__name__+": "+str(error)
            if (OUTPUT/"GPU0").exists():new_json(OUTPUT/"GPU0/owner_supervision_failed.json",dict(error=result["error"]))
        finally:
            if owned is not None:result["active_cleanup"]=owned.stop()
            if active is not None:
                try:retain_child_result(result["all_arms"][index],arm)
                except BaseException as error:result.update(status="FAIL_RETAINED",child_failure_ledger_guard_error=str(error))
            if baseline is not None:
                try:result["closing_GPU"]=gpu_observe(plan,result["GPU_observation_journal"],deadline=time.monotonic()+9,baseline=baseline)
                except BaseException as error:result.update(status="FAIL_RETAINED",closing_GPU_error=str(error))
            try:
                verify(plan_sha,authority_sha);archive_verify(plan,plan_sha,authority_sha);result["closing_used_bytes"]=budget()
            except BaseException as error:result.update(status="FAIL_RETAINED",closing_guard_error=str(error))
            new_json(OUTPUT/"result.json",result)
        return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("execute","worker"),required=True)
    parser.add_argument("--plan-sha256",required=True);parser.add_argument("--authority-sha256",required=True)
    parser.add_argument("--arm",choices=("CPU6","GPU0"));parser.add_argument("--runtime-lease-fd",type=int)
    parser.add_argument("--parent-pid",type=int);parser.add_argument("--parent-start",type=int)
    args=parser.parse_args(argv)
    if args.mode=="worker":
        need(args.arm and args.runtime_lease_fd is not None and args.parent_pid and args.parent_start,"exact internal worker arguments required")
        return worker(args)
    need(all(value is None for value in (args.arm,args.runtime_lease_fd,args.parent_pid,args.parent_start)),"worker arguments cannot alter parent")
    result=execute(args.plan_sha256,args.authority_sha256)
    print(json.dumps(dict(status=result["status"],all888_raw5head_strict1e6_passed=result.get("all888_raw5head_strict1e6_passed")),allow_nan=False))
    return 0 if result["status"]=="PASS_COUNTERFACTUAL_ONLY" else 2


if __name__=="__main__":raise SystemExit(main())
