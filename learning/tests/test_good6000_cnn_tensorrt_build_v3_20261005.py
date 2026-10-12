"""HH_261005 - Pure protobuf/source/resource fixtures only: no compiler, native parser, GPU query, Torch, actual ONNX/NPZ or model execution."""

import ast
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import struct
from types import SimpleNamespace

import pytest

from scripts.e2e import prepare_good6000_cnn_tensorrt_build_v3_20261005 as owner
from scripts.e2e import audit_good6000_cnn_onnx_initializers_v1_20261005 as wire


def vi(value):
    result = bytearray()
    while value > 127: result.append((value & 127) | 128); value >>= 7
    result.append(value)
    return bytes(result)


def integer(number, value): return vi(number << 3) + vi(value)
def blob(number, value): return vi((number << 3) | 2) + vi(len(value)) + value


def info(name, shape):
    dims = b"".join(blob(1, integer(1, size)) for size in shape)
    tensor = integer(1, 1) + blob(2, dims)
    return blob(1, name.encode()) + blob(2, blob(1, tensor))


def synthetic_model(batch=6, fault=None):
    raw = struct.pack("<f", 1.25)
    names = owner.seals.CNN_NAMES
    original = {name: dict(shape=[1], dtype="float32", raw_bytes=4, raw_sha256=hashlib.sha256(raw).hexdigest()) for name in names}
    graph = b""
    for index, name in enumerate(names):
        data = integer(1, 1) + integer(2, 1) + blob(8, name.encode()) + blob(9, raw)
        if fault == "external" and index == 0: data += blob(13, b"unsealed")
        if fault == "singular_duplicate" and index == 0: data += integer(2, 1)
        if fault == "name" and index == 0: data = integer(1, 1) + integer(2, 1) + blob(8, b"foreign.weight") + blob(9, raw)
        if fault == "dtype" and index == 0: data = integer(1, 1) + integer(2, 10) + blob(8, name.encode()) + blob(9, raw)
        if fault == "bytes" and index == 0: data = integer(1, 1) + integer(2, 1) + blob(8, name.encode()) + blob(9, b"1234")
        graph += blob(5, data)
    if fault == "duplicate": graph += blob(5, integer(1, 1) + integer(2, 1) + blob(8, names[0].encode()) + blob(9, raw))
    graph += blob(11, info("normalized_images", [batch, 3, 180, 320])) + blob(12, info("cnn_features", [batch, 60, 128]))
    if fault == "dynamic":
        dynamic = blob(1, b"cnn_features") + blob(2, blob(1, integer(1, 1) + blob(2, blob(1, blob(2, b"dynamic_batch")))))
        graph = graph.replace(blob(12, info("cnn_features", [batch, 60, 128])), blob(12, dynamic))
    graph += blob(1, blob(4, b"Conv"))
    if fault == "sparse": graph += blob(15, b"")
    model = integer(1, 7) + blob(7, graph) + blob(8, integer(2, 11 if fault == "opset" else 12))
    if fault == "training": model += blob(20, b"")
    return model, original


@pytest.mark.parametrize("batch", (6, 30))
def test_primary_proto_field_map_all14_FP32_bytes_and_static_IO(batch):
    model, original = synthetic_model(batch)
    report = wire.inspect(model, batch, original)
    assert len(report["initializers"]) == 14 and report["exact14_initializers_verified"]
    assert report["input"]["shape"] == [batch, 3, 180, 320] and report["output"]["shape"] == [batch, 60, 128]
    assert not report["independent_full_ONNX_semantics_or_TensorRT_support_or_numerical_parity_verified"]


@pytest.mark.parametrize("fault", ("external", "dtype", "bytes", "duplicate", "singular_duplicate", "name", "dynamic", "sparse", "opset", "training"))
def test_nonoriginal_or_mutable_protobuf_denied_before_any_native_parser(fault):
    model, original = synthetic_model(fault=fault)
    with pytest.raises(RuntimeError): wire.inspect(model, 6, original)


@pytest.mark.parametrize("data", (b"\x80", b"\x08" + b"\xff" * 10, b"\x0a\x03xx", b"\x0b", b"\x00"))
def test_truncated_group_invalid_or_overflow_wire_payload_denied(data):
    with pytest.raises(RuntimeError): wire.fields(data)


def test_scalar_ROOT_authority_denial_is_before_compiler_GPU_or_output(monkeypatch):
    monkeypatch.setattr(owner.seals, "read_pinned", lambda *args: (_ for _ in ()).throw(RuntimeError("no ROOT authority")))
    monkeypatch.setattr(owner, "contract", lambda *args: pytest.fail("unadmitted authority cannot inspect vendor assets"))
    monkeypatch.setattr(owner.subprocess, "Popen", lambda *args, **kwargs: pytest.fail("no compiler/native child"))
    monkeypatch.setattr(owner, "gpu_snapshot", lambda: pytest.fail("no GPU query"))
    with pytest.raises(RuntimeError): owner.verify("a" * 64, "b" * 64)


def test_explicit_vendor_compile_paths_no_system8_no_fast_math_or_install(monkeypatch):
    monkeypatch.setattr(owner, "COMPILER", Path("/fixture/g++"))
    paths = {"nvinfer": Path("/fixture/vendor10.8/libnvinfer.so"), "parser": Path("/fixture/vendor10.8/libnvonnxparser.so"),
        "cuda": Path("/fixture/cuda12.8/libcudart.so")}
    argv = owner.compile_argv(paths)
    assert "-fno-fast-math" in argv and not any(value == "-ffast-math" for value in argv)
    assert "-I" + str(owner.SDK / "include/x86_64-linux-gnu") in argv
    assert "-I" + str(owner.CUDA / "include") in argv
    assert all(str(path) in argv for path in paths.values()) and "-o" in argv


def test_allfile_budget_keeps_original_floor_shared_attempt_closing(monkeypatch, tmp_path):
    monkeypatch.setattr(owner, "OUTPUT", tmp_path)
    monkeypatch.setattr(owner.shutil, "disk_usage", lambda path: SimpleNamespace(free=owner.FLOOR + owner.SHARED + owner.ATTEMPT))
    (tmp_path / "one").write_bytes(b"123")
    assert owner.budget(startup=True) == 3
    (tmp_path / "symlink").symlink_to("/proc")
    with pytest.raises(RuntimeError): owner.budget()


def test_CPP_source_only_precision_parentdeath_static2denominator_no_inference_context():
    source = owner.CPP.read_text()
    for flag in ("kTF32", "kFP16", "kINT8", "kBF16", "kFP8"): assert flag in source
    assert "config->clearFlag(flag)" in source and "config->getFlag(flag)" in source and "kSTRONGLY_TYPED" in source
    assert "PR_SET_PDEATHSIG, SIGKILL" in source and "owner_guard(argv); // HH_261005" in source
    assert "for (int index = 0; index < 2; ++index)" in source and "int batch = index ? 30 : 6" in source
    assert "parse(data.data(), data.size(), nullptr)" in source and "buildSerializedNetwork" in source
    assert "createExecutionContext" not in source and "enqueue" not in source.replace("inference enqueue", "")
    assert "system(" not in source and "exec(" not in source and "cudaSetDevice(0)" in source


def test_owner_compile_native_lifecycle_and_original_V3_dependency_source_guards():
    source = owner.HERE.read_text()
    assert "compile_process = subprocess.Popen" in source and "compile_process.wait(timeout=30)" in source
    assert "start_new_session=True" in source and 'for child in (compile_process, process)' in source
    assert "for label, child, generation" in source and "seals.stop_owned(child, generation)" in source
    assert "test_good6000_cnn_legacy_onnx_v3_20261005.py" in source and 'producer.read_json(producer.PLAN)["source_sha256"]' in source
    tree = ast.parse(source)
    assert not any(isinstance(node, ast.Import) and any(alias.name.startswith(("torch", "onnx", "tensorrt")) for alias in node.names) for node in ast.walk(tree))


def fake_admitted_owner(monkeypatch, tmp_path, failure=None):
    # HH_261005 - Replace every authority/process/GPU/asset seam with private tiny fixtures; never run g++, Nvidia, native code or original weights.
    root = tmp_path / "root"; root.mkdir()
    output = root / "output"
    original = root / "original"; (original / "export").mkdir(parents=True)
    for batch in (6, 30): (original / "export" / ("cnn_fixed%d.onnx" % batch)).write_bytes(b"synthetic protobuf fixture")
    plan_path, auth_path = root / "ROOT_plan.json", root / "ROOT_authority.json"
    for path in (plan_path, auth_path): path.write_bytes(b"{}")
    desktop_path = root / "desktop_proof.json"; desktop_path.write_bytes(b"synthetic desktop evidence")
    monkeypatch.setattr(owner, "DESKTOP_PROOF", desktop_path)
    monkeypatch.setattr(owner, "DESKTOP_PROOF_SHA", hashlib.sha256(desktop_path.read_bytes()).hexdigest())
    monkeypatch.setattr(owner, "ROOT", root); monkeypatch.setattr(owner, "OUTPUT", output)
    monkeypatch.setattr(owner, "PLAN", plan_path); monkeypatch.setattr(owner, "AUTHORITY", auth_path)
    monkeypatch.setattr(owner.producer, "OUTPUT", original)
    plan = dict(source_sha256={}, GPU_uuid="GPU-" + "0" * 36, compile_argv=["/fixture/g++"], existing_desktop_graphics=desktop_identities())
    trace = []
    def verify(*args):
        trace.append("verify")
        if failure == "source_changed" and trace.count("verify") == 2: raise RuntimeError("source changed")
        return plan
    monkeypatch.setattr(owner, "verify", verify)
    monkeypatch.setattr(owner, "budget", lambda **kwargs: 0)
    monkeypatch.setattr(owner.producer, "audit", lambda: {"fixture": True})
    monkeypatch.setattr(owner.producer, "read_json", lambda path: {"exact14_CNN_state": {}})
    monkeypatch.setattr(owner.initializer, "inspect", lambda data, batch, state: {"ONNX_sha256": hashlib.sha256(data).hexdigest()})
    monkeypatch.setattr(owner.seals, "file_sha", lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest())
    monkeypatch.setattr(owner, "libraries", lambda: {"fixture": root / "libfixture.so"})
    @contextmanager
    def lease(): yield 321
    monkeypatch.setattr(owner.seals, "runtime_lease", lease)
    monkeypatch.setattr(owner.seals, "process_core", lambda pid: dict(pid=pid, ppid=owner.os.getpid(), pgid=pid, sid=pid, start_ticks=123))
    class Child:
        def __init__(self, pid, compiler): self.pid, self.compiler, self.returncode = pid, compiler, None
        def wait(self, timeout):
            trace.append(("wait", self.pid, timeout))
            if self.compiler and failure == "compile_timeout": raise owner.subprocess.TimeoutExpired(["fixture"], timeout)
            self.returncode = 1 if self.compiler and failure == "compile_failed" else 0
            return self.returncode
        def poll(self): return self.returncode
    children = []
    def popen(argv, **kwargs):
        compiler = not children
        trace.append("compile" if compiler else "native")
        assert kwargs["start_new_session"] is True and kwargs["env"]["CUDA_VISIBLE_DEVICES"] == plan["GPU_uuid"]
        child = Child(80001 + len(children), compiler); children.append(child)
        if compiler: (output / "builder").write_bytes(b"synthetic nonexecutable compiler output")
        else:
            rows = [dict(batch=batch, status="PARSED_BUILT_FP32_FEASIBILITY_ONLY", precision_flags_readback_verified=True,
                TF32_FP16_INT8_BF16_FP8_enabled=False) for batch in (6, 30)]
            if failure == "partial_native": rows = rows[:1]
            for row in rows:
                kwargs["stdout"].write((json.dumps(row) + "\n").encode())
                (output / "native" / ("cnn_fixed%d.engine" % row["batch"])).write_bytes(b"synthetic engine bytes")
            child.returncode = 2 if failure == "partial_native" else 0
        return child
    monkeypatch.setattr(owner.subprocess, "Popen", popen)
    monkeypatch.setattr(owner, "gpu_snapshot", lambda: dict(index=0, uuid=plan["GPU_uuid"], apps=[dict(pid=9,uuid=plan["GPU_uuid"])]
        if failure == "foreign_app" else [], free_MiB=1 if failure == "resource" else 4096, used_MiB=254, utilization_percent=23,
        full_process_identities=[dict(kind="G", identity=row) for row in plan["existing_desktop_graphics"]]))
    def stop(child, core):
        trace.append(("stop", None if child is None else child.pid))
        if child is not None: child.returncode = -9 if child.returncode is None else child.returncode
        if failure == "cleanup_error" and child is not None: raise RuntimeError("cleanup failed")
    monkeypatch.setattr(owner.seals, "stop_owned", stop)
    from scripts.e2e import run_owned_ctrack_isolated_probe_20261001 as guards
    monkeypatch.setattr(guards, "all_processes", lambda: [dict(pgid=80001)] if failure == "survivor" else [])
    return output, plan_path, auth_path, trace, children


@pytest.mark.parametrize("failure", ("compile_failed", "compile_timeout", "foreign_app", "resource", "cleanup_error", "survivor", "source_changed", "partial_native"))
def test_failure_retains_two_batches_logs_group_proof_and_no_retry(monkeypatch, tmp_path, failure):
    output, plan, authority, trace, children = fake_admitted_owner(monkeypatch, tmp_path, failure)
    result = owner.execute(hashlib.sha256(plan.read_bytes()).hexdigest(), hashlib.sha256(authority.read_bytes()).hexdigest())
    assert result["status"] == "FAIL_RETAINED" and [row["batch"] for row in result["all_planned_batches"]] == [6, 30]
    assert (output / "result.json").is_file() and (output / "compile.stdout.log").is_file() and (output / "compile.stderr.log").is_file()
    assert result["compile_child_creation_core"]["pid"] == 80001 and result["compile_exit_code"] is not None
    assert all(child.returncode is not None for child in children) and ("stop",80001) in trace
    if failure in ("compile_failed", "compile_timeout", "foreign_app", "resource"):
        assert "native" not in trace and all(row["status"] == "FAIL_NOT_VERIFIED" for row in result["all_planned_batches"])
    if failure in ("cleanup_error", "survivor"): assert result["cleanup_errors"]
    if failure == "source_changed": assert result["final_guard_error"] == "source changed"
    if failure == "partial_native":
        assert result["all_planned_batches"][0]["status"] == "PARSED_BUILT_FP32_FEASIBILITY_ONLY"
        assert result["all_planned_batches"][1]["status"] == "FAIL_NOT_VERIFIED"
    previous = len(children)
    with pytest.raises(RuntimeError, match="no retry"):
        owner.execute(hashlib.sha256(plan.read_bytes()).hexdigest(), hashlib.sha256(authority.read_bytes()).hexdigest())
    assert len(children) == previous


def desktop_identities():
    return [dict(pid=pid, ppid=pid-1, pgid=pid, start_ticks=123, argv=[name]) for pid, name in zip(
        (74276, 74426, 75186, 2021714), ("Xorg", "gnome-shell", "snapd-desktop-integration", "code"))]


def fixture_gpu():
    plan = dict(GPU_uuid="GPU-"+"0"*36, existing_desktop_graphics=desktop_identities())
    snapshot = dict(index=0, uuid=plan["GPU_uuid"], free_MiB=4096, used_MiB=254, utilization_percent=23, apps=[],
        full_process_identities=[dict(kind="G", identity=row.copy()) for row in desktop_identities()])
    return plan, snapshot


def test_explicit_known_desktop_23percent_is_not_idle_and_post_inventory_exact():
    plan, snapshot = fixture_gpu()
    owner.verify_gpu(snapshot, plan)
    owner.verify_gpu(snapshot, plan, baseline=snapshot)
    assert snapshot["utilization_percent"] > 5


@pytest.mark.parametrize("fault", ("generation", "argv", "extra_G", "missing_G", "foreign_C", "unknown_type", "utilization", "memory", "uuid"))
def test_changed_desktop_or_foreign_or_resource_inventory_denied(fault):
    plan, snapshot = fixture_gpu()
    if fault == "generation": snapshot["full_process_identities"][0]["identity"]["start_ticks"] += 1
    if fault == "argv": snapshot["full_process_identities"][0]["identity"]["argv"] = ["foreign"]
    if fault == "extra_G": snapshot["full_process_identities"].append(dict(kind="G", identity=dict(pid=999)))
    if fault == "missing_G": snapshot["full_process_identities"].pop()
    if fault == "foreign_C":
        snapshot["full_process_identities"].append(dict(kind="C", identity=dict(pid=999)))
        snapshot["apps"] = [dict(pid=999, uuid=plan["GPU_uuid"])]
    if fault == "unknown_type": snapshot["full_process_identities"].append(dict(kind="C+G", identity=dict(pid=999)))
    if fault == "utilization": snapshot["utilization_percent"] = 36
    if fault == "memory": snapshot["free_MiB"] = 4095
    if fault == "uuid": snapshot["uuid"] = "different"
    with pytest.raises(RuntimeError): owner.verify_gpu(snapshot, plan)


def test_only_exact_current_our_compute_generation_may_join_known_desktop(monkeypatch):
    plan, snapshot = fixture_gpu(); baseline = dict(snapshot)
    core = dict(pid=555, start_ticks=456, ppid=owner.os.getpid(), pgid=555, sid=555)
    argv = ["/fixture/native-builder", "fixed6", "fixed30"]
    captured = {key: core[key] for key in ("pid", "ppid", "pgid", "start_ticks")}
    captured["argv"] = argv
    snapshot["full_process_identities"] = snapshot["full_process_identities"] + [dict(kind="C",identity=captured)]
    snapshot["apps"] = [dict(pid=555,uuid=plan["GPU_uuid"])]
    monkeypatch.setattr(owner.seals,"process_core",lambda pid:core)
    owner.verify_gpu(snapshot,plan,baseline=baseline,native_core=core,native_argv=argv)
    monkeypatch.setattr(owner.seals,"process_core",lambda pid:dict(core,start_ticks=457))
    with pytest.raises(RuntimeError): owner.verify_gpu(snapshot,plan,baseline=baseline,native_core=core,native_argv=argv)
    with pytest.raises(RuntimeError): owner.verify_gpu(snapshot,plan,baseline=baseline)


def test_closing_extra_graphics_or_added_memory_is_retained_denial():
    plan, baseline = fixture_gpu(); _, closing = fixture_gpu()
    closing["used_MiB"] = baseline["used_MiB"] + owner.GPU_MAX_MIB + 1
    with pytest.raises(RuntimeError): owner.verify_gpu(closing,plan,baseline=baseline)
    closing["used_MiB"] = baseline["used_MiB"]
    closing["full_process_identities"].append(dict(kind="G",identity=dict(pid=888)))
    with pytest.raises(RuntimeError): owner.verify_gpu(closing,plan,baseline=baseline)


def test_unsealed_or_wrong_role_desktop_proof_denies_before_actual_GPU(monkeypatch):
    monkeypatch.setattr(owner.seals,"file_sha",lambda path:"wrong")
    monkeypatch.setattr(owner.producer,"read_json",lambda path:pytest.fail("changed proof cannot be used"))
    with pytest.raises(RuntimeError): owner.desktop_proof()
    monkeypatch.setattr(owner.seals,"file_sha",lambda path:owner.DESKTOP_PROOF_SHA)
    monkeypatch.setattr(owner.producer,"read_json",lambda path:dict(status="WRONG_ROLE"))
    with pytest.raises(RuntimeError): owner.desktop_proof()


def test_fresh_inventory_before_native_and_after_cleanup_not_only_callable_presence(monkeypatch,tmp_path):
    output,plan,authority,trace,children=fake_admitted_owner(monkeypatch,tmp_path)
    original_snapshot=owner.gpu_snapshot
    def fresh(): trace.append("fresh GPU inventory");return original_snapshot()
    monkeypatch.setattr(owner,"gpu_snapshot",fresh)
    result=owner.execute(hashlib.sha256(plan.read_bytes()).hexdigest(),hashlib.sha256(authority.read_bytes()).hexdigest())
    assert result["status"] == "PARSED_BUILT_FP32_FEASIBILITY_ONLY_NOT_RUNTIME_PARITY"
    assert trace.index("fresh GPU inventory") < trace.index("native")
    assert max(index for index,value in enumerate(trace) if value=="fresh GPU inventory") > trace.index(("stop",80002))
    assert "actual_before_native_known_desktop_inventory" in result and "actual_after_native_known_desktop_inventory" in result


def test_frozen_V1_and_original_CNN_CPP_dependency_source_delta_only():
    source=owner.HERE.read_text()
    assert "FROZEN_V1_PINS" in source and len(owner.FROZEN_V1_PINS)==5
    assert owner.CPP.name == "good6000_cnn_tensorrt_build_v1_20261005.cpp"
    assert 'GPU_availability_scope="KNOWN_DESKTOP_NOT_IDLE"' in source and "initial_GPU_utilization_percent_maximum=35" in source
    assert 'rows += [(DESKTOP_PROOF, OUTPUT / "desktop_metadata_evidence.json", DESKTOP_PROOF_SHA)]' in source


@pytest.mark.parametrize("field",("pid","ppid","pgid","start_ticks","argv"))
def test_full_compute_inventory_generation_or_argv_mismatch_denied_even_if_live_core_matches(monkeypatch,field):
    plan,snapshot=fixture_gpu();baseline=dict(snapshot)
    core=dict(pid=555,start_ticks=456,ppid=owner.os.getpid(),pgid=555,sid=555)
    argv=["/fixture/native-builder","fixed6","fixed30"]
    identity={key:core[key] for key in ("pid","ppid","pgid","start_ticks")};identity["argv"]=argv
    identity[field]=["/foreign/native"] if field=="argv" else identity[field]+1
    snapshot["full_process_identities"]=snapshot["full_process_identities"]+[dict(kind="C",identity=identity)]
    snapshot["apps"]=[dict(pid=555,uuid=plan["GPU_uuid"])]
    monkeypatch.setattr(owner.seals,"process_core",lambda pid:core)
    with pytest.raises(RuntimeError): owner.verify_gpu(snapshot,plan,baseline=baseline,native_core=core,native_argv=argv)


def test_inventory_then_current_generation_change_is_fail_closed(monkeypatch):
    plan,snapshot=fixture_gpu();baseline=dict(snapshot)
    core=dict(pid=555,start_ticks=456,ppid=owner.os.getpid(),pgid=555,sid=555)
    argv=["/fixture/native-builder","fixed6","fixed30"]
    identity={key:core[key] for key in ("pid","ppid","pgid","start_ticks")};identity["argv"]=argv
    snapshot["full_process_identities"]=snapshot["full_process_identities"]+[dict(kind="C",identity=identity)]
    snapshot["apps"]=[dict(pid=555,uuid=plan["GPU_uuid"])]
    monkeypatch.setattr(owner.seals,"process_core",lambda pid:dict(core,start_ticks=457))
    with pytest.raises(RuntimeError,match="exact current owned native compute generation"):
        owner.verify_gpu(snapshot,plan,baseline=baseline,native_core=core,native_argv=argv)


def test_V3_runtime_actual_argv_and_frozen_V2_receipt_closure_is_explicit():
    source=owner.HERE.read_text()
    assert "native_core=core, native_argv=argv" in source and len(owner.FROZEN_V2_PINS)==4
    assert "good6000_cnn_tensorrt_build_v2/source_prepared_review.json" in source
    assert "good6000_cnn_tensorrt_build_v2/source_closure.json" in source
