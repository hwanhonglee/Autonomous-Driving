"""HH_261005 - Independent readonly whole V6 runtime inventory-guard failure postcheck; no compiler/native parser/Torch/GPU execution or original output mutation."""

import json
import os
from pathlib import Path
import stat

from . import audit_good6000_cnn_legacy_onnx_v3_postcheck_v1_20261005 as cpu
from . import audit_good6000_cnn_onnx_initializers_v1_20261005 as wire
from . import prepare_good6000_cnn_tensorrt_build_v6_20261005 as owner
from . import run_owned_ctrack_isolated_probe_20261001 as guards

PLAN_SHA = "29fba2a28595fd87f6ff95f75556df62012ee856701df2cfa95e2afcb63d242d"
AUTH_SHA = "fad221b5c32be5fd9c27057601bbf46080df84e12ac95a53f559dbf266c64e31"
RESULT_SHA = "01f3cd0e473e29856b699b146d969f777f911027fd92824e9dd40f55dbe60a5b"


def audit():
    plan = owner.verify(PLAN_SHA, AUTH_SHA)
    root = owner.OUTPUT
    cpu.need(cpu.sha(root / "result.json") == RESULT_SHA, "exact actual runtime inventory-guard failure result required")
    result = cpu.read_json(root / "result.json")
    cpu.need(result["schema"] == owner.SCHEMA + ".result" and result["status"] == "FAIL_RETAINED"
        and result["compile_exit_code"] == 0 and 0 <= result["compile_elapsed_seconds"] <= 30
        and result["all_planned_batches"] == [dict(batch=batch,status="FAIL_NOT_VERIFIED") for batch in (6,30)]
        and result["native_child_creation_core"] is not None and result["error"] == "RuntimeError: foreign compute/full inventory difference denied"
        and not any(key in result for key in ("cleanup_errors","final_guard_error","closing_GPU_guard_error","native_rows_guard_error")),
        "actual failure or full two-batch denominator differs")
    cpu.need(result["raw14_initializers_and_original101_verified"] is True and result["GPU_or_model_runtime_promoted"] is False
        and result["complete_framework_binary_ABI_or_cached_output_parity_or_latency_gain_verified"] is False,
        "failure scope incorrectly promoted")
    expected = {}
    for destination,row in plan["immutable_copy_manifest"].items():
        original,archive = Path(row["source_path"]),root / destination
        cpu.need(cpu.sha(original) == row["sha256"] == cpu.sha(archive)
            and original.stat().st_size == archive.stat().st_size == row["bytes"], "complete immutable source/archive copy differs")
        expected[destination] = row["sha256"]
    proofs = ((owner.PLAN,root/"ROOT_plan.json",PLAN_SHA),(owner.AUTHORITY,root/"ROOT_execution_authority.json",AUTH_SHA))
    for original,archive,pin in proofs:
        cpu.need(cpu.sha(original) == pin == cpu.sha(archive) and original.stat().st_size <= owner.PARENT_JSON_LIMIT,
            "raw ROOT proof seal or bound differs")
        expected[archive.name] = pin
    original_cpu = cpu.audit()
    archived_cpu = cpu.read_json(root/"original_readonly_postcheck.json")
    for key in ("status","source_pairs_verified","runtime_Python_source_pairs_verified","model_asset_pairs_verified",
        "full101_NPZ_shapes_dtypes_finite_values_raw_SHA_verified","exact14_shared_producer_storage_manifest_verified",
        "producer_all101_state_callbacks_cache_structure_unchanged_guards","complete_batch_denominator","ONNX_sha256",
        "complete_output_file_sha256","complete_output_file_count","complete_output_bytes"):
        cpu.need(archived_cpu[key] == original_cpu[key], "complete original CPU/model/archive evidence changed")
    state = cpu.read_json(cpu.OUTPUT/"export/feasibility.json")["exact14_CNN_state"]
    initializers = [wire.inspect((root/"input"/("cnn_fixed%d.onnx"%batch)).read_bytes(),batch,state) for batch in (6,30)]
    cpu.need(initializers == cpu.read_json(root/"initializer14_audit.json"), "actual original14/static IO audit differs")
    cpu.need(sum(row["bytes"] for row in plan["immutable_copy_manifest"].values()) == plan["immutable_copy_bytes_total"]
        and result["actual_initial_immutable_and_ROOT_proof_copy_bytes"] == plan["immutable_copy_bytes_total"]
            + sum(path.stat().st_size for path in (owner.PLAN,owner.AUTHORITY)), "complete admitted copy-byte sum differs")
    cpu.need(result["compile_argv"] == plan["compile_argv"], "actual compiler argv differs")
    builder_pin = "e79027ad21fc2b08a34c3f70c15495e3b97299c2e35db0b48fe4b526ff7e3258"
    cpu.need(cpu.sha(root/"builder") == builder_pin == result["compiled_builder_sha256"],
        "exact successful private compiler output differs")
    native_core = result["native_child_creation_core"]
    argv = result["actual_native_argv"]
    parent = result["compile_child_creation_core"]["ppid"]
    cpu.need(argv[:5] == [str(root/"builder"),plan["GPU_uuid"],str(root/"input/cnn_fixed6.onnx"),
        str(root/"input/cnn_fixed30.onnx"),str(root/"native")] and len(argv)==8 and argv[5]==str(parent)
        and argv[6].isdigit() and int(argv[6])>0 and argv[7].isdigit() and int(argv[7])>=0
        and native_core["ppid"]==parent, "exact owned native invocation/parent differs")
    before, after = (result[key] for key in ("actual_before_native_known_desktop_inventory","actual_after_native_known_desktop_inventory"))
    for snapshot in (before,after):
        owner.verify_gpu(snapshot,plan,baseline=None if snapshot is before else before)
        raw = []
        for line in snapshot["full_NVIDIA_GPU0_inventory"].splitlines():
            match = owner.re.match(r"^\\|\\s*0\\s+\\S+\\s+\\S+\\s+(\\d+)\\s+(\\S+)\\s",line)
            if match: raw.append((int(match[1]),match[2]))
        cpu.need(raw == [(row["identity"]["pid"],row["kind"]) for row in snapshot["full_process_identities"]]
            and len(raw)==4 and not snapshot["apps"],"retained before/closing raw GPU inventory/identity census differs")
    cpu.need(not any("observation" in key or "journal" in key for key in result),
        "historic rejected observation unexpectedly claimed available")
    cpu.need((root/"native.stdout.log").stat().st_size == (root/"native.stderr.log").stat().st_size == 0,
        "exact empty native logs required; no historical rejected snapshot can be recovered")
    for name in ("original_readonly_postcheck.json","initializer14_audit.json","compile.stdout.log","compile.stderr.log","native.stdout.log","native.stderr.log","builder","result.json"):
        expected[name] = cpu.sha(root/name)
        if name.endswith(".json"): cpu.need((root/name).stat().st_size <= owner.PARENT_JSON_LIMIT,"bounded parent JSON differs")
    actual,total = {},0
    for parent,directories,files in os.walk(root,followlinks=False):
        for name in directories+files:
            path=Path(parent)/name;mode=path.lstat().st_mode
            cpu.need(stat.S_ISDIR(mode) or stat.S_ISREG(mode),"special/symlink output object")
            if stat.S_ISREG(mode): actual[str(path.relative_to(root))]=cpu.sha(path);total+=path.stat().st_size
    cpu.need(actual == expected and len(actual)==77 and total == result["closing_used_bytes"]+(root/"result.json").stat().st_size,
        "complete failed output file census or closing bytes differ")
    cpu.need(total <= owner.ATTEMPT-owner.CLOSING and not any((root/name).exists() for name in
        ("native/cnn_fixed6.engine","native/cnn_fixed30.engine")),
        "quota exceeded or an unverified engine artifact exists")
    records=guards.all_processes()
    for core in (result["compile_child_creation_core"],native_core):
        cpu.need(core["pid"]==core["pgid"]==core["sid"] and core["ppid"]>0 and core["start_ticks"]>0,
            "owned compiler/native creation identity differs")
        cpu.need(not any(row["pgid"]==core["pgid"] for row in records),"owned compiler/native group survivor")
        cpu.need(not Path("/proc/"+str(core["pid"])).exists() and not Path("/proc/"+str(core["ppid"])).exists(),
            "original compiler/native/parent PID still exists; generation requires inspection")
    for identity in plan["existing_desktop_graphics"]:
        current=guards.process_record(identity["pid"])
        cpu.need(current and current["state"]!="Z" and {key:value for key,value in current.items() if key!="state"}==identity,
            "sealed original desktop generation/argv changed")
    current_used=owner.budget()
    cpu.need(current_used == total,"actual complete current quota walk differs")
    return dict(explanation="HH_261005 - Readonly source/assets/headers/tools/archive/process/census proof; original inventory-guard failure and both unverified batches remain FAIL; missing rejected midrun observation has UNKNOWN cause.",
        status="PASS_INDEPENDENT_POSTCHECK_OF_RETAINED_V6_INVENTORY_FAILURE_NOT_GPU_SUCCESS",plan_sha256=PLAN_SHA,
        authority_sha256=AUTH_SHA,result_sha256=RESULT_SHA,producer_status="FAIL_RETAINED",compiler_exit_code=0,
        compiler_elapsed_seconds=result["compile_elapsed_seconds"],all_planned_batches=result["all_planned_batches"],
        current_archived_source_pairs=len(plan["source_sha256"]),immutable_copy_files_verified=len(plan["immutable_copy_manifest"]),selected_header_alias_target_pairs_verified=15,
        direct_SDK_library_seals_verified=3,compiler_and_Nvidia_tool_current_binary_seals_verified=2,
        original_CPU_source_runtime_Python_model_asset_pairs=[34,106,7],original101_state_and_exact14_ONNX_bytes_verified=True,
        whole_output_file_count=len(actual),whole_output_bytes=total,whole_output_file_sha256=actual,
        owned_compiler_native_and_parent_PID_generations_groups_gone=True,original_four_desktop_generation_argv_current_verified=True,
        retained_before_and_closing_full_GPU_inventories_exact_four_graphics_no_compute_verified=True,
        historic_rejected_midrun_GPU_observation_missing=True, actual_rejection_cause="UNKNOWN_NOT_RECOVERABLE_FROM_RETAINED_EVIDENCE",
        non_atomic_inventory_hypothesis_PROVEN=False, fresh_NVIDIA_GPU_queries_executed_by_postchecker=False,
        engine_build_or_model_inference_parity_latency_feature_approval=False,complete_transitive_native_dependency_ABI_claimed=False,
        postchecker_executed_Torch_compiler_native_GPU_or_model=False)


if __name__=="__main__":
    try: print(json.dumps(audit(),indent=2,allow_nan=False))
    except BaseException as error:
        print(json.dumps(dict(status="FAIL_OR_UNVERIFIED_RETAINED",error=type(error).__name__+": "+str(error))))
        raise SystemExit(2)

