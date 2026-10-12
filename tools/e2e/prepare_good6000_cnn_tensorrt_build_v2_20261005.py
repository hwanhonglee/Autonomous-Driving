"""HH_261005 - Fresh V2 admits only the exactly sealed existing desktop graphics, not idle GPU; fixed6/30 FP32 build still requires fresh ROOT authority and an exclusive runtime lease."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import shutil
import subprocess
import time

from . import prepare_good6000_cnn_legacy_onnx_v3_20261005 as seals
from . import audit_good6000_cnn_legacy_onnx_v3_postcheck_v1_20261005 as producer
from . import audit_good6000_cnn_onnx_initializers_v1_20261005 as initializer

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
CPP = HERE.with_suffix(".cpp").with_name("good6000_cnn_tensorrt_build_v1_20261005.cpp")
TEST = ROOT / "learning/tests/test_good6000_cnn_tensorrt_build_v2_20261005.py"
SDK = ROOT / "storage/assets/vendor/tensorrt-10.8/root/usr"
CUDA = ROOT / "storage/assets/vendor/cuda-12.8/root/usr/local/cuda-12.8"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-05/good6000_cnn_tensorrt_build_v2"
PLAN = ROOT / "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v2/ROOT_plan.json"
AUTHORITY = PLAN.with_name("ROOT_execution_authority.json")
SCHEMA = "portable_e2e.good6000_cnn_tensorrt_build.20261005.v2"
DESKTOP_PROOF = ROOT / "storage/experiments/validation/2026-10-05/owned_town03_stationary_wheel_rpc_v4/preflight.json"
DESKTOP_PROOF_SHA = "9f8eaa8958adb12cc6691100b51af38e8b7c3b070ee0aa40a4739567f5d53146"
FROZEN_V1_PINS = {
    "tools/e2e/audit_good6000_cnn_legacy_onnx_v3_postcheck_v1_20261005.py": "2d60d6fbe6ff7e9dfac3d8a5178fbe17b092a09d3a359a319cc7d5debc347352",
    "tools/e2e/audit_good6000_cnn_onnx_initializers_v1_20261005.py": "bb0f075eb3e0190889adb733e37e96e439c047ee8c7c4473d0b58a82af300dff",
    "tools/e2e/good6000_cnn_tensorrt_build_v1_20261005.cpp": "51955dd5afc73aa95122533de25e989910b96109893c6853f659397a04c38921",
    "tools/e2e/prepare_good6000_cnn_tensorrt_build_v1_20261005.py": "f70d196f211d31786fbafaa96058d5cfb2369150a6679b948bdfffb7eb8d2b7e",
    "learning/tests/test_good6000_cnn_tensorrt_build_v1_20261005.py": "f711f1ed370ff9612e4baaf9516a1e05745c8447d3f6d90efdcb30684257fd11",
}
FLOOR, SHARED, ATTEMPT, CLOSING = 5 * 1024**3, 64 * 1024**2, 768 * 1024**2, 16 * 1024**2
GPU_MAX_MIB, GPU_FREE_MIB = 2048, 4096
COMPILER, NVIDIA_SMI = Path("/usr/bin/g++").resolve(), Path("/usr/bin/nvidia-smi").resolve()


def need(value, message):
    if not value: raise RuntimeError(message)


def source_pins():
    # HH_261005 - Explicitly inherit the reviewed original34 source closure and V3 source/test, not merely an indirect postcheck claim.
    need(seals.file_sha(producer.PLAN) == producer.PLAN_SHA, "exact original ROOT producer plan required for closure")
    result = dict(producer.read_json(producer.PLAN)["source_sha256"])
    for name, pin in FROZEN_V1_PINS.items():
        need(seals.file_sha(ROOT / name) == pin, "frozen V1 dependency changed")
        result[name] = pin
    for path in (HERE, CPP, TEST, Path(seals.__file__), ROOT / "learning/tests/test_good6000_cnn_legacy_onnx_v3_20261005.py",
        Path(producer.__file__), Path(initializer.__file__),
        ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py",
        ROOT / "tools/e2e/run_owned_ctrack_isolated_probe_20261001.py", ROOT / "tools/e2e/prepare_ctrack_isolated_probe_20261001.py"):
        name, pin = str(path.relative_to(ROOT)), seals.file_sha(path)
        need(name not in result or result[name] == pin, "frozen producer dependency changed")
        result[name] = pin
    return result


def libraries():
    # HH_261005 - Resolve explicit existing vendor paths, never choose system8.6 headers/libraries or install another runtime.
    return {name: (SDK / "lib/x86_64-linux-gnu" / name).resolve(strict=True)
        for name in ("libnvinfer.so", "libnvonnxparser.so")} | {"libcudart.so": (CUDA / "lib64/libcudart.so").resolve(strict=True)}


def compile_argv(paths):
    return [str(COMPILER), "-std=c++17", "-O2", "-fno-fast-math", "-I" + str(SDK / "include/x86_64-linux-gnu"),
        "-I" + str(CUDA / "include"), str(CPP), *map(str, paths.values()),
        "-Wl,-rpath," + ":".join(sorted({str(path.parent) for path in paths.values()})), "-o", str(OUTPUT / "builder")]


def contract(gpu_uuid):
    need(type(gpu_uuid) is str and re.fullmatch(r"GPU-[0-9a-fA-F-]{36}", gpu_uuid), "exact prospective physicalGPU0 UUID required")
    paths, desktop = libraries(), desktop_proof()
    need(gpu_uuid == desktop["gpu_uuid"], "same physical GPU0 as sealed desktop proof required")
    headers = sorted((SDK / "include/x86_64-linux-gnu").glob("Nv*.h")) + [CUDA / "include/cuda_runtime_api.h", CUDA / "include/driver_types.h"]
    return dict(schema=SCHEMA, output=str(OUTPUT), source_sha256=source_pins(), sdk_header_sha256={str(path): seals.file_sha(path) for path in headers},
        sdk_library_sha256={str(path): seals.file_sha(path) for path in paths.values()}, compiler_path=str(COMPILER),
        compiler_sha256=seals.file_sha(COMPILER), nvidia_smi_path=str(NVIDIA_SMI), nvidia_smi_sha256=seals.file_sha(NVIDIA_SMI),
        compile_argv=compile_argv(paths), compile_timeout_seconds=30, build_timeout_seconds=120,
        GPU_physical_index=0, GPU_uuid=gpu_uuid, initial_free_MiB=GPU_FREE_MIB, maximum_added_GPU_used_MiB=GPU_MAX_MIB,
        initial_GPU_utilization_percent_maximum=35, workspace_bytes=64 * 1024**2,
        GPU_availability_scope="KNOWN_DESKTOP_NOT_IDLE", existing_desktop_graphics=desktop["NVIDIA_GPU0_process_identities"],
        desktop_metadata_evidence_path=str(DESKTOP_PROOF), desktop_metadata_evidence_sha256=DESKTOP_PROOF_SHA,
        fixed_batches=[6, 30], precision="FP32_strongly_typed", TF32_FP16_INT8_BF16_FP8_enabled=False,
        original_producer_plan_sha256=producer.PLAN_SHA, original_producer_authority_sha256=producer.AUTH_SHA,
        runtime_lock=str(seals.LOCK), free_floor_bytes=FLOOR, shared_reserve_bytes=SHARED,
        attempt_quota_bytes=ATTEMPT, closing_reserve_bytes=CLOSING, individual_file_limit_bytes=32 * 1024**2,
        library_archival_or_complete_native_dependency_closure_claimed=False, execution_context_or_enqueue_or_control=False,
        dynamic_soname_dependencies_transitive_CUDA_headers_or_driver_ABI_verified=False,
        build_may_internally_profile_GPU_tactics=True, numerical_cached_parity_or_latency_gain_verified=False)


def verify(plan_sha, authority_sha):
    plan, authority = seals.read_pinned(PLAN, plan_sha), seals.read_pinned(AUTHORITY, authority_sha)
    exact = lambda left, right: json.dumps(left, sort_keys=True, allow_nan=False) == json.dumps(right, sort_keys=True, allow_nan=False)
    need(exact(plan, contract(plan.get("GPU_uuid"))), "exact fresh source/vendor/resource declaration required")
    need(exact(authority, dict(schema=SCHEMA + ".ROOT_execution_authority", explicit_private_CPU_compile_and_GPU0_parse_build_only_approved=True,
        plan_path=str(PLAN), plan_sha256=plan_sha, GPU_uuid=plan["GPU_uuid"], source_sha256=plan["source_sha256"],
        sdk_header_sha256=plan["sdk_header_sha256"], sdk_library_sha256=plan["sdk_library_sha256"],
        desktop_metadata_evidence_path=str(DESKTOP_PROOF), desktop_metadata_evidence_sha256=DESKTOP_PROOF_SHA,
        GPU_availability_scope="KNOWN_DESKTOP_NOT_IDLE", existing_desktop_graphics=plan["existing_desktop_graphics"])), "exact ROOT compile/GPU authority required")
    return plan


def budget(startup=False):
    need(shutil.disk_usage(ROOT).free >= FLOOR + SHARED + (ATTEMPT if startup else CLOSING), "unchanged floor/shared/reserve required")
    used = 0
    if OUTPUT.exists():
        for path in OUTPUT.rglob("*"):
            need(not path.is_symlink() and (path.is_file() or path.is_dir()), "special output object denied")
            if path.is_file(): used += path.stat().st_size
    need(used <= ATTEMPT - CLOSING, "all-file output census closing reserve reached")
    return used


def gpu_snapshot():
    # PhysicalGPU0 only; read inventory immediately before native launch and during the owned build, never inspect/operate GPU1.
    gpu = subprocess.run([str(NVIDIA_SMI), "-i", "0", "--query-gpu=index,uuid,memory.free,memory.used,utilization.gpu", "--format=csv,noheader,nounits"],
        capture_output=True, text=True, timeout=3, check=True)
    apps = subprocess.run([str(NVIDIA_SMI), "-i", "0", "--query-compute-apps=pid,gpu_uuid", "--format=csv,noheader,nounits"],
        capture_output=True, text=True, timeout=3, check=True)
    full = subprocess.run([str(NVIDIA_SMI), "-i", "0"], capture_output=True, text=True, timeout=3, check=True)
    need(all(len(value.encode()) <= 32768 for value in (gpu.stdout, apps.stdout, full.stdout)), "bounded physicalGPU0 inventories required")
    rows = [line for line in gpu.stdout.splitlines() if line.strip()]
    need(len(rows) == 1, "one physicalGPU0 record required")
    index, uuid, free, used, utilization = [value.strip() for value in rows[0].split(",")]
    from . import run_owned_ctrack_isolated_probe_20261001 as guards
    identities = []
    for line in full.stdout.splitlines():
        match = re.match(r"^\|\s*0\s+\S+\s+\S+\s+(\d+)\s+(\S+)\s", line)
        if match:
            pid, kind = int(match[1]), match[2]
            record = guards.process_record(pid)
            need(record is not None and record["state"] != "Z", "fresh live full-inventory PID generation required")
            identities.append(dict(kind=kind, identity={key: value for key, value in record.items() if key != "state"}))
    need(len({row["identity"]["pid"] for row in identities}) == len(identities), "duplicate full-inventory PID denied")
    return dict(index=int(index), uuid=uuid, free_MiB=int(free), used_MiB=int(used), utilization_percent=int(utilization),
        apps=[dict(pid=int(line.split(",")[0]), uuid=line.split(",")[1].strip()) for line in apps.stdout.splitlines() if line.strip()],
        full_NVIDIA_GPU0_inventory=full.stdout, full_process_identities=identities)


def desktop_proof():
    need(seals.file_sha(DESKTOP_PROOF) == DESKTOP_PROOF_SHA, "exact canonical ROOT desktop metadata evidence required")
    proof = producer.read_json(DESKTOP_PROOF)
    need(proof["status"] == "PREPARED_NO_CARLA_CONNECTION", "original desktop proof role differs")
    gpu = proof["GPU0"]; identities = gpu["NVIDIA_GPU0_process_identities"]
    need(gpu["gpu_index"] == 0 and gpu["stationary_under_desktop_load_only"] is True and not gpu["compute_app_inventory"]
        and [row["pid"] for row in identities] == [74276, 74426, 75186, 2021714], "only original four desktop identities admitted")
    return gpu


def verify_gpu(snapshot, plan, *, baseline=None, native_core=None):
    # HH_261005 - Exactly preserve known desktop generations/argv; permit no new graphics and only our current owned native compute generation.
    need(snapshot["index"] == 0 and snapshot["uuid"] == plan["GPU_uuid"], "declared physicalGPU0 differs")
    graphics = [row["identity"] for row in snapshot["full_process_identities"] if row["kind"] == "G"]
    need(sorted(graphics, key=lambda row: row["pid"]) == sorted(plan["existing_desktop_graphics"], key=lambda row: row["pid"]),
        "known desktop PID generation/argv or graphics inventory changed")
    compute = [row for row in snapshot["full_process_identities"] if row["kind"] != "G"]
    need(all(row["kind"] == "C" for row in compute), "unknown full GPU process type denied")
    if compute or snapshot["apps"]:
        need(native_core is not None and seals.process_core(native_core["pid"]) == native_core,
            "exact current owned native compute generation required")
        need(len(compute) == len(snapshot["apps"]) == 1 and compute[0]["identity"]["pid"] == native_core["pid"]
            and snapshot["apps"] == [dict(pid=native_core["pid"], uuid=plan["GPU_uuid"])], "foreign compute/full inventory difference denied")
    if baseline is None:
        need(not compute and not snapshot["apps"] and snapshot["free_MiB"] >= GPU_FREE_MIB
            and snapshot["utilization_percent"] <= 35, "initial known-desktop GPU0 availability denied; not idle")
    else: need(snapshot["used_MiB"] <= baseline["used_MiB"] + GPU_MAX_MIB, "GPU added-memory resource guard failed")


def limits():
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    resource.setrlimit(resource.RLIMIT_FSIZE, (32 * 1024**2, 32 * 1024**2))


def new_json(path, value):
    with path.open("x") as stream: json.dump(value, stream, indent=2, allow_nan=False)


def execute(plan_sha, authority_sha):
    # HH_261005 - No compilation, GPU query, artifact output or native import is reachable before fixed authority/resource/lease admission.
    with seals.runtime_lease() as lease_fd:
        plan = verify(plan_sha, authority_sha); budget(startup=True)
        need(not OUTPUT.exists() and OUTPUT.parent.is_dir(), "fresh output and existing parent required; no retry")
        original = producer.audit()
        report = producer.read_json(producer.OUTPUT / "export/feasibility.json")
        checks = [initializer.inspect((producer.OUTPUT / "export" / ("cnn_fixed%d.onnx" % batch)).read_bytes(), batch, report["exact14_CNN_state"])
            for batch in (6, 30)]
        OUTPUT.mkdir(exist_ok=False)
        result = dict(schema=SCHEMA + ".result", status="FAIL_RETAINED", all_planned_batches=[dict(batch=batch, status="FAIL_NOT_VERIFIED")
            for batch in (6, 30)], raw14_initializers_and_original101_verified=True, GPU_or_model_runtime_promoted=False,
            complete_framework_binary_ABI_or_cached_output_parity_or_latency_gain_verified=False)
        process, core, compile_process, compile_core, rows, compile_begin, baseline = None, None, None, None, [], None, None
        try:
            for folder in ("input", "native", "Scratch", "UserCache", "source"): (OUTPUT / folder).mkdir()
            rows = [(ROOT / name, OUTPUT / "source" / name, sha) for name, sha in plan["source_sha256"].items()]
            rows += [(PLAN, OUTPUT / "ROOT_plan.json", plan_sha), (AUTHORITY, OUTPUT / "ROOT_execution_authority.json", authority_sha)]
            rows += [(DESKTOP_PROOF, OUTPUT / "desktop_metadata_evidence.json", DESKTOP_PROOF_SHA)]
            rows += [(producer.OUTPUT / "export" / ("cnn_fixed%d.onnx" % batch), OUTPUT / "input" / ("cnn_fixed%d.onnx" % batch), check["ONNX_sha256"])
                for batch, check in zip((6, 30), checks)]
            for src, dest, pin in rows:
                dest.parent.mkdir(parents=True, exist_ok=True)
                with src.open("rb") as input, dest.open("xb") as output: shutil.copyfileobj(input, output, 1024**2)
                need(seals.file_sha(src) == pin == seals.file_sha(dest), "exact raw source/authority/ONNX archive differs")
            new_json(OUTPUT / "original_readonly_postcheck.json", original); new_json(OUTPUT / "initializer14_audit.json", checks)
            env = dict(os.environ, CUDA_VISIBLE_DEVICES=plan["GPU_uuid"], NVIDIA_TF32_OVERRIDE="0", CUDA_CACHE_DISABLE="1",
                CUDA_CACHE_PATH=str(OUTPUT / "UserCache"), XDG_CACHE_HOME=str(OUTPUT / "UserCache"), TMPDIR=str(OUTPUT / "Scratch"),
                LD_LIBRARY_PATH=":".join(sorted({str(path.parent) for path in libraries().values()})))
            for key in ("PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"): env.pop(key, None)
            with (OUTPUT / "compile.stdout.log").open("xb") as out, (OUTPUT / "compile.stderr.log").open("xb") as err:
                compile_begin = time.monotonic()
                compile_process = subprocess.Popen(plan["compile_argv"], cwd=ROOT, env=env, stdout=out, stderr=err,
                    start_new_session=True, preexec_fn=limits)
                compile_core = seals.process_core(compile_process.pid)
                result["compile_argv"] = plan["compile_argv"]
                result["compile_child_creation_core"] = compile_core
                need(compile_core and compile_core["ppid"] == os.getpid()
                    and compile_core["pgid"] == compile_core["sid"] == compile_process.pid, "owned compiler generation/session required")
                try: result["compile_exit_code"] = compile_process.wait(timeout=30)
                finally: result["compile_elapsed_seconds"] = time.monotonic() - compile_begin
            need(result["compile_exit_code"] == 0, "private compile failed; all logs/fixed rows retained")
            result["compiled_builder_sha256"] = seals.file_sha(OUTPUT / "builder")
            baseline = gpu_snapshot()
            verify_gpu(baseline, plan)
            result["actual_before_native_known_desktop_inventory"] = baseline
            parent = seals.process_core(os.getpid())
            argv = [str(OUTPUT / "builder"), plan["GPU_uuid"], str(OUTPUT / "input/cnn_fixed6.onnx"), str(OUTPUT / "input/cnn_fixed30.onnx"),
                str(OUTPUT / "native"), str(parent["pid"]), str(parent["start_ticks"]), str(lease_fd)]
            result["actual_native_argv"] = argv
            with (OUTPUT / "native.stdout.log").open("xb") as out, (OUTPUT / "native.stderr.log").open("xb") as err:
                deadline = time.monotonic() + 120
                process = subprocess.Popen(argv, cwd=ROOT, env=env, stdout=out, stderr=err, start_new_session=True,
                    pass_fds=(lease_fd,), preexec_fn=limits)
                core = seals.process_core(process.pid)
                need(core and core["ppid"] == os.getpid() and core["pgid"] == core["sid"] == process.pid, "exact owned native child core required")
                while process.poll() is None:
                    need(time.monotonic() <= deadline, "fixed parse/build deadline exceeded")
                    current = gpu_snapshot()
                    need(time.monotonic() <= deadline, "fixed parse/build deadline exceeded during GPU inventory")
                    verify_gpu(current, plan, baseline=baseline, native_core=core)
                    budget(); time.sleep(.2)
                result["native_exit_code"] = process.wait(timeout=2)
            raw = (OUTPUT / "native.stdout.log").read_text()
            need(len(raw) <= CLOSING, "bounded native rows required")
            captured = [json.loads(line) for line in raw.splitlines()]
            if [row.get("batch") for row in captured] == [6, 30]: result["all_planned_batches"] = captured
            need(result["native_exit_code"] == 0 and len(captured) == 2 and all(row["status"] == "PARSED_BUILT_FP32_FEASIBILITY_ONLY"
                and row["precision_flags_readback_verified"] is True and row["TF32_FP16_INT8_BF16_FP8_enabled"] is False for row in captured),
                "both FP32 parse/build rows required; not runtime parity")
            result["engine_sha256"] = {str(batch): seals.file_sha(OUTPUT / "native" / ("cnn_fixed%d.engine" % batch)) for batch in (6, 30)}
            result["status"] = "PARSED_BUILT_FP32_FEASIBILITY_ONLY_NOT_RUNTIME_PARITY"
        except BaseException as error: result["error"] = type(error).__name__ + ": " + str(error)
        finally:
            cleanup_errors = []
            for label, child, generation in (("compiler", compile_process, compile_core), ("native", process, core)):
                try: seals.stop_owned(child, generation)
                except BaseException as error: cleanup_errors.append(label + ": " + str(error))
            try:
                from . import run_owned_ctrack_isolated_probe_20261001 as guards
                records = guards.all_processes()
                # A missing/finished leader does not waive the complete PGID census; unknown generations are never signalled.
                for child in (compile_process, process):
                    need(child is None or not any(row["pgid"] == child.pid for row in records), "owned compile/native process-group survivor")
            except BaseException as error: cleanup_errors.append("complete process-group census: " + str(error))
            if cleanup_errors: result.update(status="FAIL_RETAINED", cleanup_errors=cleanup_errors)
            result["compile_child_creation_core"] = compile_core
            result["compile_exit_code"] = compile_process.returncode if compile_process is not None else None
            if "compile_elapsed_seconds" not in result:
                result["compile_elapsed_seconds"] = None if compile_begin is None else time.monotonic() - compile_begin
            result["native_child_creation_core"] = core
            # HH_261005 - Keep any completed native row even if resource admission, timeout or cleanup failed; never drop the other planned batch.
            try:
                if (OUTPUT / "native.stdout.log").exists():
                    raw = (OUTPUT / "native.stdout.log").read_text()
                    need(len(raw) <= CLOSING, "bounded native failure rows required")
                    captured = [json.loads(line) for line in raw.splitlines()]
                    need(len(captured) <= 2 and [row.get("batch") for row in captured] == [6, 30][:len(captured)], "native failure row order differs")
                    for index, row in enumerate(captured): result["all_planned_batches"][index] = row
            except BaseException as error: result.update(status="FAIL_RETAINED", native_rows_guard_error=str(error))
            if baseline is not None:
                try:
                    closing_gpu = gpu_snapshot()
                    verify_gpu(closing_gpu, plan, baseline=baseline)
                    result["actual_after_native_known_desktop_inventory"] = closing_gpu
                except BaseException as error: result.update(status="FAIL_RETAINED", closing_GPU_guard_error=str(error))
            try:
                verify(plan_sha, authority_sha)
                need(all(seals.file_sha(src) == pin == seals.file_sha(dest) for src, dest, pin in rows), "closing raw archive seal differs")
                if "compiled_builder_sha256" in result:
                    need(seals.file_sha(OUTPUT / "builder") == result["compiled_builder_sha256"], "compiled private builder changed")
                result["closing_used_bytes"] = budget()
            except BaseException as error: result.update(status="FAIL_RETAINED", final_guard_error=str(error))
            new_json(OUTPUT / "result.json", result)
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-sha256", required=True); parser.add_argument("--authority-sha256", required=True)
    args = parser.parse_args(argv)
    result = execute(args.plan_sha256, args.authority_sha256)
    print(json.dumps(dict(schema=result["schema"], status=result["status"])))
    return 0 if result["status"] == "PARSED_BUILT_FP32_FEASIBILITY_ONLY_NOT_RUNTIME_PARITY" else 2


if __name__ == "__main__": raise SystemExit(main())
