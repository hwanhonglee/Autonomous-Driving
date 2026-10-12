"""HH_261005 - Fresh V6 uses only the uniquely renamed hh_json_quote native source and pins the complete retained V5 compiler failure; all V5 ownership/header/quota/precision/cleanup guards remain unchanged."""

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
CPP = HERE.with_suffix(".cpp").with_name("good6000_cnn_tensorrt_build_v6_20261005.cpp")
TEST = ROOT / "learning/tests/test_good6000_cnn_tensorrt_build_v6_20261005.py"
SDK = ROOT / "storage/assets/vendor/tensorrt-10.8/root/usr"
CUDA = ROOT / "storage/assets/vendor/cuda-12.8/root/usr/local/cuda-12.8"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-05/good6000_cnn_tensorrt_build_v6"
PLAN = ROOT / "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v6/ROOT_plan.json"
AUTHORITY = PLAN.with_name("ROOT_execution_authority.json")
SCHEMA = "portable_e2e.good6000_cnn_tensorrt_build.20261005.v6"
DESKTOP_PROOF = ROOT / "storage/experiments/validation/2026-10-05/owned_town03_stationary_wheel_rpc_v4/preflight.json"
DESKTOP_PROOF_SHA = "9f8eaa8958adb12cc6691100b51af38e8b7c3b070ee0aa40a4739567f5d53146"
FROZEN_V1_PINS = {
    "tools/e2e/audit_good6000_cnn_legacy_onnx_v3_postcheck_v1_20261005.py": "2d60d6fbe6ff7e9dfac3d8a5178fbe17b092a09d3a359a319cc7d5debc347352",
    "tools/e2e/audit_good6000_cnn_onnx_initializers_v1_20261005.py": "bb0f075eb3e0190889adb733e37e96e439c047ee8c7c4473d0b58a82af300dff",
    "tools/e2e/good6000_cnn_tensorrt_build_v1_20261005.cpp": "51955dd5afc73aa95122533de25e989910b96109893c6853f659397a04c38921",
    "tools/e2e/prepare_good6000_cnn_tensorrt_build_v1_20261005.py": "f70d196f211d31786fbafaa96058d5cfb2369150a6679b948bdfffb7eb8d2b7e",
    "learning/tests/test_good6000_cnn_tensorrt_build_v1_20261005.py": "f711f1ed370ff9612e4baaf9516a1e05745c8447d3f6d90efdcb30684257fd11",
}
FROZEN_V2_PINS = {
    "tools/e2e/prepare_good6000_cnn_tensorrt_build_v2_20261005.py": "ae94992b65f84761a63c404d046bd6e7dde3a08e545cde32c84c6bbcb42a8239",
    "learning/tests/test_good6000_cnn_tensorrt_build_v2_20261005.py": "af2f2a4abf4ee907a9c589dbbd98c599055a6ba325d774d27b528f7cf9400083",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v2/source_prepared_review.json": "2f2d248688401885774ddcbb37bd266d3656d09f33d4a1c24c1203168da0d9aa",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v2/source_closure.json": "c4d5efc8333c0d716c03810267616568d14fac4326e5adf5ed01cb446ecdbbb4",
}
FROZEN_V3_PINS = {
    "tools/e2e/prepare_good6000_cnn_tensorrt_build_v3_20261005.py": "b39b840732a74d7e2f7b5bd9da2909e7292162c31e82bd94126698ae3facab2b",
    "learning/tests/test_good6000_cnn_tensorrt_build_v3_20261005.py": "e1c5cfbc3e7a514900d6231c199a51ad20299bcda348887148efefa72881898f",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v3/source_prepared_review.json": "4d8128f99e40a90ee00ebc91a7aba198578b5557cd0bfdc84bd3d124d21f153d",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v3/source_closure.json": "91224bdfd5494bf61fa9a3747f233da90fe2563f94b31cb10e9e0f949c4df22e",
}
FROZEN_V4_PINS = {
    "tools/e2e/prepare_good6000_cnn_tensorrt_build_v4_20261005.py": "1bd04a9ecd0c0911805b7d71a2fa4f133ffd43fea31fe3e94d1d677614fe7c79",
    "learning/tests/test_good6000_cnn_tensorrt_build_v4_20261005.py": "7ce9ed54c0531596e3d7d83947b559b02e238016c1357d84759a6fb5bb07c34d",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v4/source_prepared_review.json": "0494710b7d666863f3327f4a5b33c08438b033e3dee08d2c1e981bea51d4638a",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v4/source_closure_and_immutable_copy_bytes.json": "754cf33b10862a00bfb2b3d4f98c89b09cdff713924aa7e423469355f7e06e94",
}
FROZEN_V5_PINS = {
    "tools/e2e/prepare_good6000_cnn_tensorrt_build_v5_20261005.py": "b20c327f72ef0a6c08c902e558a1df9ef0a64da4aa02b28ef0e420c68b0c6dd7",
    "learning/tests/test_good6000_cnn_tensorrt_build_v5_20261005.py": "15052a2596e03acc4bfac15a54fecf98e1be8930bc54fa121fdbdb179e3eea70",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v5/source_prepared_review.json": "fcc4bee4b8dff924e20abf0139acc5a3f3be4e23b09343c558742206f0246fce",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v5/source_closure_and_selected_header_seals.json": "9e332dca116ed4544b2ad5e4a9516a0e2b82c089d250a6603a3616c0d9e37945",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v5/ROOT_plan.json": "70dc143afb04171c4f9692c56da92f6912ae9d2c4878bb7d8ad4e5245279454c",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v5/ROOT_execution_authority.json": "8a404e436a2ee43afbb9b4882afcfe02df7b8919827b853dac8278707565e3b4",
    "storage/experiments/diagnostics/2026-10-05/good6000_cnn_tensorrt_build_v5/result.json": "c199d7919ff1a0ed3fd42ecba2f51d87b0bd8e0ef1422573f03a22144cad24c0",
    "storage/experiments/diagnostics/2026-10-05/good6000_cnn_tensorrt_build_v5/compile.stderr.log": "442bc88b40db3bb26ca53c6bdc78b2393a1a5435a059c5a987a0ba0cd6fb115c",
    "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_build_v5/independent_whole_compile_failure_postcheck_v1_20261005.json": "e53770209374f0dd90432312fffa4a5db89796ba58c907bf0fa564e176773cf3",
    "tools/e2e/audit_good6000_cnn_tensorrt_v5_compile_failure_v1_20261005.py": "dca8cc5beaf88935072c783d80c99eee00fb3fa59b91f12acc62bc51efe62cdf",
}
FLOOR, SHARED, ATTEMPT, CLOSING = 5 * 1024**3, 64 * 1024**2, 512 * 1024**2, 16 * 1024**2
PARENT_JSON_LIMIT = 512 * 1024
GPU_MAX_MIB, GPU_FREE_MIB = 2048, 4096
COMPILER, NVIDIA_SMI = Path("/usr/bin/g++").resolve(), Path("/usr/bin/nvidia-smi").resolve()


def need(value, message):
    if not value: raise RuntimeError(message)


def source_pins():
    # HH_261005 - Explicitly inherit the reviewed original34 source closure and V3 source/test, not merely an indirect postcheck claim.
    need(seals.file_sha(producer.PLAN) == producer.PLAN_SHA, "exact original ROOT producer plan required for closure")
    result = dict(producer.read_json(producer.PLAN)["source_sha256"])
    for name, pin in (FROZEN_V1_PINS | FROZEN_V2_PINS | FROZEN_V3_PINS | FROZEN_V4_PINS | FROZEN_V5_PINS).items():
        need(seals.file_sha(ROOT / name) == pin, "frozen V1/V2/V3/V4/V5 dependency changed")
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


def header_seals():
    # HH_261005 - Keep the compiler's raw include ABI while binding each alias resolution and canonical bytes inside the approved SDK version root.
    rows = [(path, SDK) for path in sorted((SDK / "include/x86_64-linux-gnu").glob("Nv*.h"))]
    rows += [(CUDA / "include" / name, CUDA) for name in ("cuda_runtime_api.h", "driver_types.h")]
    aliases, pins = {}, {}
    for raw, approved in rows:
        need(approved == approved.resolve(strict=True), "approved SDK version root must itself be canonical")
        target = raw.resolve(strict=True)
        need(target.is_relative_to(approved) and target.is_file(), "SDK header alias escapes approved version root")
        aliases[str(raw)] = str(target)
        pins[str(target)] = seals.file_sha(target)
    return aliases, pins


def copy_manifest(pins):
    entries = [(ROOT / name, "source/" + name, pin) for name, pin in pins.items()]
    entries += [(DESKTOP_PROOF, "desktop_metadata_evidence.json", DESKTOP_PROOF_SHA)]
    entries += [(producer.OUTPUT / "export" / ("cnn_fixed%d.onnx" % batch), "input/cnn_fixed%d.onnx" % batch,
        seals.file_sha(producer.OUTPUT / "export" / ("cnn_fixed%d.onnx" % batch))) for batch in (6, 30)]
    result = {}
    for path, destination, pin in entries:
        need(seals.file_sha(path) == pin, "immutable input copy source changed")
        result[destination] = dict(source_path=str(path), sha256=pin, bytes=path.stat().st_size)
    return result


def contract(gpu_uuid):
    need(type(gpu_uuid) is str and re.fullmatch(r"GPU-[0-9a-fA-F-]{36}", gpu_uuid), "exact prospective physicalGPU0 UUID required")
    paths, desktop = libraries(), desktop_proof()
    need(gpu_uuid == desktop["gpu_uuid"], "same physical GPU0 as sealed desktop proof required")
    aliases, header_pins = header_seals()
    pins = source_pins(); copies = copy_manifest(pins)
    need(sum(row["bytes"] for row in copies.values()) + 5 * PARENT_JSON_LIMIT <= ATTEMPT - CLOSING, "fixed archive plus metadata bounds exceed tighter quota")
    return dict(schema=SCHEMA, output=str(OUTPUT), source_sha256=pins, sdk_header_alias_to_canonical=aliases, sdk_header_sha256=header_pins,
        sdk_library_sha256={str(path): seals.file_sha(path) for path in paths.values()}, compiler_path=str(COMPILER),
        compiler_sha256=seals.file_sha(COMPILER), nvidia_smi_path=str(NVIDIA_SMI), nvidia_smi_sha256=seals.file_sha(NVIDIA_SMI),
        compile_argv=compile_argv(paths), compile_timeout_seconds=30, build_timeout_seconds=120,
        compiler_wait_interval_seconds=.05, immutable_copy_manifest=copies,
        immutable_copy_bytes_total=sum(row["bytes"] for row in copies.values()), ROOT_plan_and_authority_each_byte_limit=PARENT_JSON_LIMIT,
        parent_audit_initializer_result_JSON_each_byte_limit=PARENT_JSON_LIMIT,
        unknown_Scratch_UserCache_or_transitive_SDK_total_hard_bound_claimed=False,
        native_budget_checks_are_periodic_after_up_to_three_3second_GPU_queries=True,
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
    need(all(path.stat().st_size <= PARENT_JSON_LIMIT for path in (PLAN, AUTHORITY)), "bounded ROOT plan/authority metadata required")
    plan, authority = seals.read_pinned(PLAN, plan_sha), seals.read_pinned(AUTHORITY, authority_sha)
    exact = lambda left, right: json.dumps(left, sort_keys=True, allow_nan=False) == json.dumps(right, sort_keys=True, allow_nan=False)
    need(exact(plan, contract(plan.get("GPU_uuid"))), "exact fresh source/vendor/resource declaration required")
    need(exact(authority, dict(schema=SCHEMA + ".ROOT_execution_authority", explicit_private_CPU_compile_and_GPU0_parse_build_only_approved=True,
        plan_path=str(PLAN), plan_sha256=plan_sha, GPU_uuid=plan["GPU_uuid"], source_sha256=plan["source_sha256"],
        sdk_header_alias_to_canonical=plan["sdk_header_alias_to_canonical"], sdk_header_sha256=plan["sdk_header_sha256"], sdk_library_sha256=plan["sdk_library_sha256"],
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


def verify_gpu(snapshot, plan, *, baseline=None, native_core=None, native_argv=None):
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
        identity = compute[0]["identity"]
        # HH_261005 - A live current PID alone cannot validate an earlier full inventory; bind its complete captured generation and actual argv too.
        need(all(identity.get(key) == native_core[key] for key in ("pid", "ppid", "pgid", "start_ticks"))
            and type(native_argv) is list and native_argv and identity.get("argv") == native_argv,
            "captured compute generation/ancestry/argv differs from exact owned native launch")
    if baseline is None:
        need(not compute and not snapshot["apps"] and snapshot["free_MiB"] >= GPU_FREE_MIB
            and snapshot["utilization_percent"] <= 35, "initial known-desktop GPU0 availability denied; not idle")
    else: need(snapshot["used_MiB"] <= baseline["used_MiB"] + GPU_MAX_MIB, "GPU added-memory resource guard failed")


def limits():
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    resource.setrlimit(resource.RLIMIT_FSIZE, (32 * 1024**2, 32 * 1024**2))


def new_json(path, value):
    data = json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False).encode("utf8")
    need(len(data) <= PARENT_JSON_LIMIT, "bounded parent JSON required before any write")
    with path.open("xb") as stream: stream.write(data)


def execute(plan_sha, authority_sha):
    # HH_261005 - No compilation, GPU query, artifact output or native import is reachable before fixed authority/resource/lease admission.
    with seals.runtime_lease() as lease_fd:
        plan = verify(plan_sha, authority_sha); budget(startup=True)
        need(not OUTPUT.exists() and OUTPUT.parent.is_dir(), "fresh output and existing parent required; no retry")
        observed_immutable_bytes = sum(Path(row["source_path"]).stat().st_size for row in plan["immutable_copy_manifest"].values())
        proof_bytes = {str(path): path.stat().st_size for path in (PLAN, AUTHORITY)}
        need(observed_immutable_bytes == plan["immutable_copy_bytes_total"] and all(size <= PARENT_JSON_LIMIT for size in proof_bytes.values()),
            "actual original copy bytes or metadata bounds changed before copying")
        original = producer.audit()
        report = producer.read_json(producer.OUTPUT / "export/feasibility.json")
        checks = [initializer.inspect((producer.OUTPUT / "export" / ("cnn_fixed%d.onnx" % batch)).read_bytes(), batch, report["exact14_CNN_state"])
            for batch in (6, 30)]
        OUTPUT.mkdir(exist_ok=False)
        result = dict(schema=SCHEMA + ".result", status="FAIL_RETAINED", all_planned_batches=[dict(batch=batch, status="FAIL_NOT_VERIFIED")
            for batch in (6, 30)], raw14_initializers_and_original101_verified=True, GPU_or_model_runtime_promoted=False,
            complete_framework_binary_ABI_or_cached_output_parity_or_latency_gain_verified=False)
        result["actual_initial_immutable_and_ROOT_proof_copy_bytes"] = observed_immutable_bytes + sum(proof_bytes.values())
        result["actual_ROOT_proof_input_bytes"] = proof_bytes
        process, core, compile_process, compile_core, rows, compile_begin, baseline = None, None, None, None, [], None, None
        try:
            for folder in ("input", "native", "Scratch", "UserCache", "source"): (OUTPUT / folder).mkdir()
            rows = [(Path(row["source_path"]), OUTPUT / destination, row["sha256"]) for destination, row in plan["immutable_copy_manifest"].items()]
            rows += [(PLAN, OUTPUT / "ROOT_plan.json", plan_sha), (AUTHORITY, OUTPUT / "ROOT_execution_authority.json", authority_sha)]
            for src, dest, pin in rows:
                dest.parent.mkdir(parents=True, exist_ok=True)
                with src.open("rb") as input, dest.open("xb") as output: shutil.copyfileobj(input, output, 1024**2)
                need(seals.file_sha(src) == pin == seals.file_sha(dest), "exact raw source/authority/ONNX archive differs")
            need(sum(dest.stat().st_size for src, dest, pin in rows) == result["actual_initial_immutable_and_ROOT_proof_copy_bytes"],
                "actual complete archive copy bytes differ from admission")
            new_json(OUTPUT / "original_readonly_postcheck.json", original); new_json(OUTPUT / "initializer14_audit.json", checks)
            env = dict(os.environ, CUDA_VISIBLE_DEVICES=plan["GPU_uuid"], NVIDIA_TF32_OVERRIDE="0", CUDA_CACHE_DISABLE="1",
                CUDA_CACHE_PATH=str(OUTPUT / "UserCache"), XDG_CACHE_HOME=str(OUTPUT / "UserCache"), TMPDIR=str(OUTPUT / "Scratch"),
                LD_LIBRARY_PATH=":".join(sorted({str(path.parent) for path in libraries().values()})))
            for key in ("PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"): env.pop(key, None)
            budget()
            need(header_seals() == (plan["sdk_header_alias_to_canonical"], plan["sdk_header_sha256"]), "SDK header aliases/targets changed before compile")
            with (OUTPUT / "compile.stdout.log").open("xb") as out, (OUTPUT / "compile.stderr.log").open("xb") as err:
                compile_begin = time.monotonic()
                compile_process = subprocess.Popen(plan["compile_argv"], cwd=ROOT, env=env, stdout=out, stderr=err,
                    start_new_session=True, preexec_fn=limits)
                compile_core = seals.process_core(compile_process.pid)
                result["compile_argv"] = plan["compile_argv"]
                result["compile_child_creation_core"] = compile_core
                need(compile_core and compile_core["ppid"] == os.getpid()
                    and compile_core["pgid"] == compile_core["sid"] == compile_process.pid, "owned compiler generation/session required")
                try:
                    while compile_process.poll() is None:
                        budget()
                        current_core = seals.process_core(compile_process.pid)
                        need(current_core == compile_core or (current_core is None and compile_process.poll() is not None),
                            "owned compiler generation changed while polling")
                        if compile_process.poll() is not None: break
                        remaining = compile_begin + 30 - time.monotonic()
                        need(remaining > 0, "fixed private compile deadline exceeded")
                        try: compile_process.wait(timeout=min(.05, remaining))
                        except subprocess.TimeoutExpired: pass
                    result["compile_exit_code"] = compile_process.returncode
                finally: result["compile_elapsed_seconds"] = time.monotonic() - compile_begin
            need(result["compile_exit_code"] == 0, "private compile failed; all logs/fixed rows retained")
            need(header_seals() == (plan["sdk_header_alias_to_canonical"], plan["sdk_header_sha256"]), "SDK header aliases/targets changed after compile")
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
                    verify_gpu(current, plan, baseline=baseline, native_core=core, native_argv=argv)
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
