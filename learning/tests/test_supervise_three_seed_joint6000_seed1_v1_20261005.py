"""HH_261005 - Test the sealed seed1 supervisor with scalar, filesystem and process stand-ins.

No helper import, SSH, Torch, GPU query, remote file or real process is used.
The authentic transported try/except/finally and pure generation functions are
compiled privately. Synthetic endpoint success is NOT an actual training run.
"""

import ast
import base64
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import signal
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tools/maintenance/supervise_three_seed_joint6000_seed1_v1_20261005.py"


def require(value, message):
    if not value:
        raise RuntimeError(message)


def encode(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def sha(value):
    return hashlib.sha256(value).hexdigest()


def local_scope():
    tree = ast.parse(SOURCE.read_bytes())
    scope = dict(require=require, hashlib=hashlib, json=json, Path=Path, __file__=str(SOURCE))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id in ("REMOTE_PREFIX", "REMOTE_BODY") for target in node.targets):
            scope[node.targets[0].id] = ast.literal_eval(node.value)
        if isinstance(node, ast.FunctionDef) and node.name in ("same_generation", "admit_family", "remote_program", "sha", "encode"):
            exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"), scope)
    return scope


def remote_tree():
    return ast.parse(local_scope()["remote_program"]())


def remote_functions(scope, names):
    nodes = [node for node in remote_tree().body if isinstance(node, ast.FunctionDef) and node.name in names]
    require(len(nodes) == len(names), "all authentic scalar helper functions required")
    exec(compile(ast.Module(body=nodes, type_ignores=[]), "transported_scalar_only", "exec"), scope)


def generation(pid=100, ppid=77, *, pgid=100, session=100, start_ticks=1000):
    return dict(pid=pid, ppid=ppid, pgid=pgid, session=session, start_ticks=start_ticks)


@pytest.mark.parametrize("key", ("pid", "pgid", "session", "start_ticks"))
def test_same_generation_rejects_reused_identity(key):
    same = local_scope()["same_generation"]
    original = generation()
    assert same(original, dict(original, ppid=1))
    changed = dict(original)
    changed[key] += 1
    assert not same(original, changed)


def test_observed_parent_chain_and_known_orphan_are_owned_but_unseen_orphan_is_not():
    admit = local_scope()["admit_family"]
    leader = generation()
    child = generation(101, 100, start_ticks=1001)
    grandchild = generation(102, 101, start_ticks=1002)
    known, unknown = admit({102: grandchild, 101: child, 100: leader}, {100: leader}, leader)
    assert set(known) == {100, 101, 102} and not unknown
    orphan = dict(child, ppid=1)
    stranger = generation(103, 1, start_ticks=1003)
    known, unknown = admit({101: orphan, 103: stranger}, known, leader)
    assert 101 in known and 103 not in known and unknown == [stranger]
    empty_known, unknown = admit({103: stranger}, {100: leader}, leader)
    assert set(empty_known) == {100} and unknown == [stranger]


def test_changed_known_generation_and_predating_child_refused():
    admit = local_scope()["admit_family"]
    leader = generation()
    for current in ({100: dict(leader, start_ticks=9999)},
                    {100: leader, 101: generation(101, 100, start_ticks=999)}):
        with pytest.raises(RuntimeError):
            admit(current, {100: leader}, leader)


def test_cleanup_only_signals_previously_admitted_generations_never_unknown_or_group():
    leader, owned, unknown = generation(), generation(101, 1, start_ticks=1001), generation(102, 1, start_ticks=1002)
    rows = {101: owned, 102: unknown}
    sent = []
    clock = iter(range(100))
    scope = dict(local_scope(), leader=leader, known={100: leader, 101: owned}, unknown=[unknown],
        signal=signal, time=SimpleNamespace(monotonic=lambda: next(clock), sleep=lambda duration: None),
        child=SimpleNamespace(poll=lambda: 0), subprocess=SimpleNamespace(TimeoutExpired=TimeoutError))
    scope["group"] = lambda: dict(rows)
    scope["observe"] = lambda: dict(rows)
    scope["identity"] = lambda pid: dict(rows[pid])
    scope["os"] = SimpleNamespace(kill=lambda pid, sig: sent.append((pid, sig)))
    remote_functions(scope, {"cleanup"})
    receipt = scope["cleanup"]()
    assert sent == [(101, signal.SIGTERM), (101, signal.SIGKILL)]
    assert receipt["unknown_session_members"] == [unknown] and receipt["remaining_session"] == rows
    assert receipt["foreign_signals_sent"] is False
    scope["identity"] = lambda pid: dict(owned, start_ticks=9999)
    sent.clear()
    with pytest.raises(RuntimeError):
        scope["cleanup"]()
    assert sent == []


def test_signal_handler_and_parent_death_guard_are_actual_private_functions_not_a_claim():
    recorded = []
    scope = dict(require=require, result={}, signal=SimpleNamespace(SIGHUP=1, SIGTERM=15, SIGINT=2, SIG_IGN="IGNORE",
        signal=lambda sig, handler: recorded.append((sig, handler))),
        ctypes=SimpleNamespace(CDLL=lambda *args, **kwargs: SimpleNamespace(prctl=lambda *args: recorded.append(args) or 0)),
        os=SimpleNamespace(getppid=lambda: 77, getpid=lambda: 100, kill=lambda pid, sig: recorded.append((pid, sig))))
    remote_functions(scope, {"supervisor_signal", "parent_death_guard"})
    scope["parent_death_guard"](77)
    assert recorded == [(1, 15, 0, 0, 0)]
    scope["parent_death_guard"](88)
    assert recorded[-1] == (100, 15)
    with pytest.raises(RuntimeError, match="interrupted"):
        scope["supervisor_signal"](15, None)
    assert scope["result"]["supervisor_received_signal"] == 15
    assert recorded[-3:] == [(1, "IGNORE"), (15, "IGNORE"), (2, "IGNORE")]


def remote_fixture(tmp_path, *, fault=None, large_metadata=False):
    # HH_261005 - All paths are new pytest temporary scalar metadata; the real private workspace is never opened.
    workspace = tmp_path / "private_fixture"
    plans, source, campaign = (workspace / name for name in ("plans", "sources", "campaign"))
    for path in (plans, source, campaign):
        path.mkdir(parents=True)
    output, receipt = campaign / "seed_2026100501", plans / "seed_2026100501_fit_supervisor_v1"
    cpu_path, gpu_path = plans / "ROOT_CPU_TRAIN_only_authorization.json", plans / "ROOT_GPU0_TRAIN_only_authorization.json"
    payload = b"synthetic-source-no-model"
    (source / "scalar.py").write_bytes(payload)
    archive, cpu = b"synthetic-archive", b"synthetic-CPU-grant"
    schedules = {str(2026100501 + index): {"schedule_sha256": str(index + 1) * 64} for index in range(3)}
    admission = encode(dict(declaration=dict(all_seed_schedules=schedules)))
    contract = dict(sources={"scalar.py": sha(payload)})
    grant = encode(dict(authorized_modes=["fit-one"], physical_GPU_index=0, contract=contract,
        schedule_sha256={key: row["schedule_sha256"] for key, row in schedules.items()}))
    (plans / "sealed_sources.tar").write_bytes(archive)
    (plans / "CPU_admission.stdout").write_bytes(admission)
    cpu_path.write_bytes(cpu)
    package = dict(entries=[dict(path="scalar.py", bytes=len(payload), sha256=sha(payload))], archive_sha256=sha(archive),
        CPU_authority_sha256=sha(cpu), admission_sha256=sha(admission), GPU_authority=base64.b64encode(grant).decode(),
        GPU_authority_sha256=sha(grant), contract_sha256=sha(encode(contract)), protocol={"synthetic": True})
    scope = dict(local_scope(), base64=base64, datetime=datetime, timezone=timezone, workspace=workspace, plans=plans,
        source=source, campaign=campaign, output=output, receipt=receipt, cpu_path=cpu_path, gpu_path=gpu_path,
        package=package, created=False, child=None, leader=None, known={}, unknown=[], status="FAILED_NO_RETRY",
        result={"large_scalar_fixture": "x" * 100_000} if large_metadata else {})
    kernel = SimpleNamespace(environ={"UNCHANGED_CONTEXT": "scalar"}, getpid=lambda: 77, getppid=lambda: 77)
    child = SimpleNamespace(pid=100, returncode=None)
    child.poll = lambda: child.returncode
    child.wait = lambda **kwargs: child.returncode
    child.stdout, child.stderr = (SimpleNamespace(fileno=lambda fd=fd: fd) for fd in (10, 11))
    raw_streams = {10: [b"unfiltered\x00stdout-1\n", b"stdout-2\n", b""], 11: [b"whole-stderr\n", b""]}
    full_stdout = b"".join(raw_streams[10])
    full_stderr = b"".join(raw_streams[11])
    calls, printed, keys = [], [], {}
    clock = [0.]

    def now():
        clock[0] += .01
        return clock[0]

    def identity(pid):
        return generation(pid, 77 if pid == 100 else 1)

    def read(fd, count):
        assert count == 65536
        return raw_streams[fd].pop(0)

    class Selector:
        def register(self, stream, event, data):
            keys[stream.fileno()] = SimpleNamespace(fileobj=stream, data=data)

        def unregister(self, stream):
            del keys[stream.fileno()]
            if not keys:
                child.returncode = 1 if fault == "exit" else 0

        def get_map(self):
            return keys

        def select(self, timeout):
            assert timeout == .25
            return [(key, None) for key in tuple(keys.values())]

    def popen(argv, **kwargs):
        calls.append((argv, kwargs))
        if fault == "spawn":
            raise RuntimeError("synthetic spawn failure before raw drain")
        output.mkdir()
        summary = dict(status="FIXED6000_SEED_TRAIN_COMPLETE_NOT_PROMOTED", seed=2026100502 if fault == "seed2" else 2026100501,
            fit=dict(optimizer_updates=6000, examples_seen=24000), final_endpoint=dict(all101_numeric_finite=True, frozen47_exact=True))
        (output / "summary.json").write_bytes(encode(summary))
        if fault == "post_source":
            (source / "scalar.py").write_bytes(b"changed-after-admission")
        if fault == "artifact":
            (output / "alias").symlink_to(source / "scalar.py")
        return child

    scope.update(os=kernel, identity=identity, time=SimpleNamespace(monotonic=now, sleep=lambda delay: None),
        shutil=SimpleNamespace(disk_usage=lambda path: SimpleNamespace(free=6 * 1024**3)),
        selectors=SimpleNamespace(DefaultSelector=Selector, EVENT_READ=1),
        subprocess=SimpleNamespace(Popen=popen, PIPE="FAKE_PIPE", TimeoutExpired=TimeoutError),
        signal=signal, print=lambda value, **kwargs: printed.append(value))
    kernel.read = read
    kernel.kill = lambda *args: pytest.fail("completed fake job cannot send any process signal")
    remote_functions(scope, {"require", "sha", "encode", "canonical", "new", "seals", "observe", "cleanup"})
    scope["group"] = lambda: {}  # No real /proc access: synthetic process is represented by the fake Popen only.
    scope["parent_death_guard"] = lambda parent: None  # Tested separately with a fake prctl above; never execute a real preexec function.
    if fault == "source":
        (source / "scalar.py").write_bytes(b"changed-before-admission")
    if fault == "CPU":
        cpu_path.write_bytes(b"changed-CPU")
    if fault == "archive":
        (plans / "sealed_sources.tar").write_bytes(b"changed-archive")
    if fault == "existing_grant":
        gpu_path.write_bytes(b"never-overwrite")
    return SimpleNamespace(scope=scope, calls=calls, printed=printed, output=output, receipt=receipt,
        gpu_path=gpu_path, cpu_path=cpu_path, cpu=cpu, full_stdout=full_stdout, full_stderr=full_stderr)


def run_remote(fixture):
    body = next(node for node in remote_tree().body if isinstance(node, ast.Try))
    exec(compile(ast.Module(body=[body], type_ignores=[]), "authentic_remote_lifecycle_fake_providers", "exec"), fixture.scope)
    return json.loads(fixture.printed[-1])


def test_exact_seed1_environment_raw_complete_and_no_cpu_grant_mutation(tmp_path):
    f = remote_fixture(tmp_path)
    response = run_remote(f)
    assert response["status"] == "COMPLETE_SEED1_FIXED6000_NOT_PROMOTED" and len(f.calls) == 1
    argv, kwargs = f.calls[0]
    assert argv[argv.index("--seed") + 1] == "2026100501" and argv[argv.index("--mode") + 1] == "fit-one"
    assert kwargs["start_new_session"] is True and callable(kwargs["preexec_fn"])
    env = kwargs["env"]
    assert {key: env[key] for key in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS", "PYTHONUNBUFFERED")} == {
        "OMP_NUM_THREADS": "4", "MKL_NUM_THREADS": "4", "OPENBLAS_NUM_THREADS": "1", "PYTHONUNBUFFERED": "1"}
    assert env["CUDA_VISIBLE_DEVICES"] == "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
    assert f.cpu_path.read_bytes() == f.cpu
    assert (f.receipt / "fit.stdout").read_bytes() == f.full_stdout and (f.receipt / "fit.stderr").read_bytes() == f.full_stderr
    result = response["result"]
    assert result["complete_raw_stdout_stderr_retained"] is True
    assert result["seed2_or_seed3_launched"] is result["automatic_retry"] is False
    assert result["post_current_archive_CPU_GPU_seals_exact"] is True


@pytest.mark.parametrize("fault", ("source", "CPU", "archive", "existing_grant"))
def test_prewrite_readonly_seals_fail_without_any_grant_or_spawn(tmp_path, fault):
    f = remote_fixture(tmp_path, fault=fault)
    response = run_remote(f)
    assert response["status"] == "FAILED_NO_RETRY" and f.calls == [] and not f.receipt.exists()
    assert response["result"]["complete_raw_stdout_stderr_retained"] is False
    assert "UNKNOWN" in response["result"]["unreceived_pipe_tail"]
    if fault == "existing_grant":
        assert f.gpu_path.read_bytes() == b"never-overwrite"
    else:
        assert not f.gpu_path.exists()


@pytest.mark.parametrize("fault", ("spawn", "exit", "seed2", "post_source", "artifact"))
def test_every_spawn_endpoint_and_final_audit_failure_retained_no_retry(tmp_path, fault):
    f = remote_fixture(tmp_path, fault=fault)
    response = run_remote(f)
    assert response["status"] == "FAILED_NO_RETRY" and len(f.calls) == 1
    result = json.loads((f.receipt / "result.json").read_bytes())
    assert result["failed_attempt_retained"] is True and result["automatic_retry"] is result["seed2_or_seed3_launched"] is False
    if fault == "spawn":
        assert result["complete_raw_stdout_stderr_retained"] is False and "UNKNOWN" in result["unreceived_pipe_tail"]
    else:
        assert (f.receipt / "fit.stdout").read_bytes() == f.full_stdout and (f.receipt / "fit.stderr").read_bytes() == f.full_stderr
    if fault == "post_source":
        assert "post_source_seal_error" in result and "post_current_archive_CPU_GPU_seals_exact" not in result
    if fault == "artifact":
        assert "partial_artifact_inventory_error" in result and (f.output / "alias").is_symlink()


def test_bounded_metadata_response_preserves_full_remote_result_without_slices(tmp_path):
    f = remote_fixture(tmp_path, large_metadata=True)
    response = run_remote(f)
    assert response["status"] == "FAILED_METADATA_BOUND_FULL_REMOTE_RESULT_RETAINED"
    assert len(f.printed[-1].encode()) <= 96 * 1024
    raw = (f.receipt / "result.json").read_bytes()
    assert json.loads(raw)["large_scalar_fixture"] == "x" * 100_000
    failure = json.loads((f.receipt / "metadata_bound_failure.json").read_bytes())
    assert failure["full_result_sha256"] == sha(raw) and failure["automatic_retry"] is False


def test_source_contract_fixed_boundaries_and_ancestry_code_exactly_transportable():
    scope = local_scope()
    tree, program = ast.parse(SOURCE.read_bytes()), scope["remote_program"]()
    transported = ast.parse(program)
    for name in ("same_generation", "admit_family"):
        local = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
        remote = next(n for n in transported.body if isinstance(n, ast.FunctionDef) and n.name == name)
        assert ast.dump(local, include_attributes=False) == ast.dump(remote, include_attributes=False)
    text = SOURCE.read_text()
    assert "time.monotonic()-interrupted_at>20" in program and "UNKNOWN" in program
    assert "descendant closure UNKNOWN" in text and "unreceived_pipe_tail" in program
    assert "key.data.write(raw);key.data.flush()" in program
    assert not any(isinstance(n, ast.Subscript) and isinstance(n.value, ast.Name) and n.value.id == "raw" for n in ast.walk(transported))
    assert "killpg" not in text and "seed_2026100502" not in program and "seed_2026100503" not in program
    assert "seals(gpu_required=False)" in program and program.index("seals(gpu_required=False)") < program.index("receipt.mkdir()")
    assert program.index("seals(gpu_required=False)") < program.index("with gpu_path.open('xb')")


def admission_fixture(tmp_path, *, fault=None):
    # HH_261005 - Compile only local admission statements preceding remote_program / first local write.
    tree = ast.parse(SOURCE.read_bytes())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
    cut = next(index for index, node in enumerate(main.body) if isinstance(node, ast.Assign)
        and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name) and node.value.func.id == "remote_program")
    probe = ast.FunctionDef(name="probe", args=main.args, body=main.body[:cut] + [ast.Return(ast.Name("manifest", ast.Load()))], decorator_list=[])
    plan, cpu, out = tmp_path / "plan", tmp_path / "CPU", tmp_path / "fresh_output"
    plan.mkdir(); cpu.mkdir()
    rows = [dict(path="learning/source_%d.py" % index, sha256="a" * 64, bytes=1) for index in range(33)]
    sources = {row["path"]: row["sha256"] for row in rows[:31]}
    if fault == "duplicate": rows[-1] = dict(rows[0])
    if fault == "absolute": rows[-1]["path"] = "/outside/source.py"
    if fault == "traversal": rows[-1]["path"] = "../outside.py"
    if fault == "count": rows.pop()
    if fault == "closure": rows[0]["sha256"] = "b" * 64
    if fault == "runtime_count": sources.pop(next(iter(sources)))
    archive = "c" * 64
    manifest = encode(dict(entries=rows, archive_sha256="d" * 64 if fault == "archive" else archive))
    schedules = {str(index): {"schedule_sha256": str(index) * 64} for index in (1, 2, 3)}
    contract = {"sources": sources}
    grant = encode(dict(authorized_modes=["validate-only"] if fault == "grant_mode" else ["fit-one"], contract=contract,
        schedule_sha256={key: row["schedule_sha256"] for key, row in schedules.items()}))
    admission = encode(dict(declaration=dict(all_seed_schedules=schedules)))
    (plan / "ROOT_GPU0_TRAIN_only_authorization.json").write_bytes(grant)
    (cpu / "remote_CPU_admission.stdout").write_bytes(admission)
    (cpu / "transfer_manifest.json").write_bytes(manifest)
    scope = dict(Path=Path, json=json, require=require, sha=sha, encode=encode, PLAN=plan, CPU=cpu, OUT=out,
        canonical=lambda path: path.resolve(strict=True), shutil=SimpleNamespace(disk_usage=lambda path: SimpleNamespace(free=100*1024**2)),
        GPU_AUTH_SHA=sha(grant), ADMISSION_SHA=sha(admission), TRANSFER_MANIFEST_SHA="0"*64 if fault == "manifest_pin" else sha(manifest),
        CONTRACT_SHA=sha(encode(contract)), SOURCE_ARCHIVE_SHA=archive)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[probe], type_ignores=[])), "local_admission_scalar_only", "exec"), scope)
    return scope, out


def test_exact_manifest_and_signed_runtime_sources_checked_before_any_package_write(tmp_path):
    scope, out = admission_fixture(tmp_path)
    assert len(scope["probe"]()["entries"]) == 33 and not out.exists()


@pytest.mark.parametrize("fault", ("duplicate", "absolute", "traversal", "count", "closure", "runtime_count", "archive", "grant_mode", "manifest_pin"))
def test_bad_transfer_or_grant_fails_before_local_or_remote_write(tmp_path, fault):
    scope, out = admission_fixture(tmp_path, fault=fault)
    with pytest.raises(RuntimeError):
        scope["probe"]()
    assert not out.exists()
