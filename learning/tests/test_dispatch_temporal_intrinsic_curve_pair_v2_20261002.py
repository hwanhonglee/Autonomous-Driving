"""HH_261002 - Synthetic V2 transfer/foreground dispatch proofs; no actual SSH/CUDA/data/model/fit."""
from __future__ import annotations
import ast
import hashlib
import io
import json
from pathlib import Path
import shlex
import tarfile
from types import SimpleNamespace

import pytest
from scripts.e2e import dispatch_temporal_intrinsic_curve_pair_v2_20261002 as prepare
from scripts.e2e import run_prepared_temporal_intrinsic_curve_pair_v2_20261002 as actual


def sha(data):
    return hashlib.sha256(data).hexdigest()


@pytest.mark.parametrize("validate", [True, False])
def test_exact_new_command_matches_foreground_and_old_parent_assets(validate):
    values = shlex.split(prepare.command("a" * 64, validate=validate))
    assert prepare.MODULE in values and actual.MODULE == prepare.MODULE
    assert values[values.index("--output") + 1] == str(prepare.RUN) == str(actual.RUN)
    assert values[values.index("--parent-run") + 1] == str(prepare.previous.PARENT)
    assert values[values.index("--parent-history-sha256") + 1] == actual.previous.HISTORY_SHA
    if not validate:
        assert values == actual.expected_command("a" * 64)
    assert "--" in values and "pip" not in values and "sudo" not in values


def test_new_source29_tar_auth30_keeps_all_original28(tmp_path):
    sources = prepare.revision.source_snapshot()
    original = prepare.revision.pair.source_snapshot()
    assert len(sources) == 29 and len(original) == 28
    assert all(sources[name] == pin for name, pin in original.items())
    auth = tmp_path / "authority.json"; auth.write_bytes(b"root authority")
    payload, pins = prepare.package(sources, auth, sha(auth.read_bytes()))
    with tarfile.open(fileobj=io.BytesIO(payload)) as archive:
        members = archive.getmembers(); assert len(members) == 30
        for member in members:
            assert member.isfile() and member.mode == 0o600 and not Path(member.name).is_absolute()
            assert sha(archive.extractfile(member).read()) == pins[member.name]
    assert prepare.REVISION_SOURCE in pins


def test_remote_copy_and_gpu_predicates_only_change_exact_identity_counts():
    old = ast.parse(prepare.previous.REMOTE_COPY); new = ast.parse(prepare._remote_copy())
    old_constants = [n for n in ast.walk(old) if isinstance(n, ast.Constant)]
    new_constants = [n for n in ast.walk(new) if isinstance(n, ast.Constant)]
    differences = [(a.value, b.value) for a, b in zip(old_constants, new_constants) if a.value != b.value]
    assert len(differences) == 5 and (29, 30) in differences and (28, 29) in differences
    assert "EXACT29_SOURCE_PLUS_AUTH_CREATE_ONLY_COPIED" in prepare._remote_copy()
    old = ast.parse(actual.previous.REMOTE_PREFLIGHT); new = ast.parse(actual._remote_preflight())
    for name in ("gpu0_idle_read_only",):
        a = next(n for n in old.body if isinstance(n, ast.FunctionDef) and n.name == name)
        b = next(n for n in new.body if isinstance(n, ast.FunctionDef) and n.name == name)
        assert ast.dump(a, include_attributes=False) == ast.dump(b, include_attributes=False)
    assert "len(pins) == 30" in actual._remote_preflight()


def test_prepare_contract_scope_restored_on_failure():
    pair = prepare.revision.pair
    original = pair.contract, pair.source_snapshot, pair.raw_parent.gpu_preflight
    with pytest.raises(RuntimeError, match="sentinel"):
        with prepare.activated():
            assert pair.contract is prepare.revision.contract
            assert len(pair.contract()["sources"]) == 29
            with pytest.raises(ValueError, match="cannot nest"):
                with prepare.activated(): pass
            raise RuntimeError("sentinel")
    assert (pair.contract, pair.source_snapshot, pair.raw_parent.gpu_preflight) == original


def test_real_prepare_main_transfer30_cpu_validate_no_fit(tmp_path, monkeypatch):
    sources = prepare.revision.source_snapshot(); contract = prepare.revision.contract()
    auth = tmp_path / "authorization.json"; auth.write_bytes(b"synthetic external root authorization")
    authority = sha(auth.read_bytes()); files = dict(sources, **{"authorization.json": authority})
    receipt = tmp_path / "storage/experiments/training/2026-10-02/prepare_v2"
    archived_source = tmp_path / "tools/e2e/dispatch_temporal_intrinsic_curve_pair_v2_20261002.py"
    archived_source.parent.mkdir(parents=True); archived_source.write_bytes(Path(prepare.__file__).read_bytes())
    monkeypatch.setattr(prepare, "ROOT", tmp_path)
    monkeypatch.setattr(prepare.revision, "contract", lambda: contract)
    monkeypatch.setattr(prepare.revision, "source_snapshot", lambda: sources)
    monkeypatch.setattr(prepare.revision.pair, "validate_authorization", lambda *a: dict(actual_parent_history_sha256=prepare.previous.PARENT_HISTORY_SHA))
    monkeypatch.setattr(prepare, "package", lambda *a: (b"synthetic tar", files))
    validation = dict(status="VALIDATED_TRAIN_ONLY_CPU_NO_CUDA_NO_OUTPUT", protocol=contract, source_sha256=sources,
        parent_history_sha256=prepare.previous.PARENT_HISTORY_SHA, trainable_state_tensor_count=54,
        frozen_state_tensor_count=47, holdout_payload_loaded=False, model_promoted=False)
    calls = []
    def run(argv, **kwargs):
        calls.append(argv)
        result = dict(file_sha256=files) if len(calls) == 1 else validation
        return SimpleNamespace(returncode=0, stdout=json.dumps(result).encode(), stderr=b"original stderr")
    monkeypatch.setattr(prepare.previous.subprocess, "run", run)
    assert prepare.main(["--mode", "prepare", "--authorization", str(auth), "--authorization-sha256", authority,
        "--expected-runner-sha256", sources[prepare.REVISION_SOURCE], "--expected-loss-sha256",
        sources["learning/portable_e2e/intrinsic_spatial_path_loss_v1_20261002.py"], "--receipt", str(receipt)]) == 0
    assert len(calls) == 2 and "--mode validate-only" in calls[1][-1] and "--mode fit" not in calls[1][-1]
    declaration = json.loads((receipt / "declaration.json").read_text())
    summary = json.loads((receipt / "summary.json").read_text())
    assert len(declaration["source_sha256"]) == summary["source_file_count"] == 29
    assert str(prepare.SOURCE) == declaration["remote_source"] and str(prepare.RUN) == declaration["remote_run"]
    assert summary["optimizer_started"] is False and (receipt / "source" / archived_source.relative_to(tmp_path)).is_file()


@pytest.fixture
def ready(tmp_path, monkeypatch):
    monkeypatch.setattr(actual, "ROOT", tmp_path)
    directory = tmp_path / "storage/experiments/training/2026-10-02/temporal_intrinsic_curve_pair_dispatch_v2"
    directory.mkdir(parents=True); monkeypatch.setattr(actual, "PREPARED", directory)
    prepare_file = tmp_path / "tools/e2e/dispatch_temporal_intrinsic_curve_pair_v2_20261002.py"
    prepare_file.parent.mkdir(parents=True); prepare_file.write_bytes(b"# synthetic prepare source\n")
    monkeypatch.setattr(actual, "PREPARE_FILE", prepare_file)
    names = ["tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py",
        "learning/portable_e2e/intrinsic_spatial_path_loss_v1_20261002.py", actual.REVISION_SOURCE]
    names += ["learning/fixture_" + str(i) + ".py" for i in range(26)]
    pins = {}
    for i, name in enumerate(names):
        path = tmp_path / name; path.parent.mkdir(parents=True, exist_ok=True)
        data = ("# fixture " + str(i)).encode(); path.write_bytes(data); pins[name] = sha(data)
    monkeypatch.setattr(actual.previous, "RUNNER_SHA", pins[names[0]])
    monkeypatch.setattr(actual.previous, "LOSS_SHA", pins[names[1]])
    authority = "a" * 64; command = shlex.join(actual.expected_command(authority))
    document = dict(source_sha256=pins, authorization_sha256=authority, dispatch_source_sha256=sha(prepare_file.read_bytes()),
        parent_actual6000_history_sha256=actual.previous.HISTORY_SHA, parent_four_actual_sha256=actual.previous.PARENT_PINS,
        physical_GPU_allowed=actual.GPU, existing_private_venv=str(actual.previous.REMOTE / "venvs/py312/bin/python"),
        optimizer_started=False, packages_installed=False, remote_source=str(actual.SOURCE), remote_run=str(actual.RUN),
        proposed_fit_command_requires_separate_root_execution=command)
    validation = b'{"status":"VALIDATED_TRAIN_ONLY_CPU_NO_CUDA_NO_OUTPUT"}\n'
    (directory / "remote_validation_stdout.json").write_bytes(validation)
    summary = dict(status="PRIVATE_REMOTE_TRAIN_ONLY_CPU_PREFLIGHT_PASS_NO_FIT", optimizer_started=False,
        remote_validation_sha256=sha(validation), proposed_fit_command_requires_separate_root_execution=command)
    calls = []
    def execute(argv, **kwargs):
        calls.append(("preflight", argv)); return SimpleNamespace(returncode=0, stdout=b'{"status":"PASS"}\n', stderr=b"")
    class Process:
        def __init__(self, argv, stdout, stderr):
            self.pid = 123; calls.append(("fit", argv)); stdout.write(b"actual raw fit stdout\n"); stderr.write(b"actual raw diagnostic\n")
        def wait(self): return 0
    monkeypatch.setattr(actual.previous.subprocess, "run", execute)
    monkeypatch.setattr(actual.previous.subprocess, "Popen", Process)
    def seal():
        for name, value in (("declaration.json", document), ("summary.json", summary)):
            (directory / name).write_text(json.dumps(value, sort_keys=True))
        return ["--expected-prepare-source-sha256", sha(prepare_file.read_bytes()), "--expected-revision-sha256", pins[actual.REVISION_SOURCE],
            "--mode", "fit", "--expected-prepared-declaration-sha256", sha((directory / "declaration.json").read_bytes()),
            "--expected-prepared-summary-sha256", sha((directory / "summary.json").read_bytes()),
            "--expected-authority-sha256", authority, "--output", str(tmp_path / "storage/experiments/training/2026-10-02/actual_v2")]
    return SimpleNamespace(seal=seal, document=document, summary=summary, calls=calls, pins=pins)


def test_real_full_foreground_dispatch_checks29_then_preserves_raw(ready):
    argv = ready.seal(); assert actual.main(argv) == 0
    assert [call[0] for call in ready.calls] == ["preflight", "fit"]
    assert ready.calls[1][1] == actual.SSH + [ready.document["proposed_fit_command_requires_separate_root_execution"]]
    output = Path(argv[-1]); assert (output / "actual_fit_stdout.jsonl").read_bytes() == b"actual raw fit stdout\n"
    assert json.loads((output / "source_postcheck.json").read_text())["model_promoted"] is False
    assert json.loads((output / "declaration.json").read_text())["source_files"] == ready.pins
    with pytest.raises(ValueError, match="fresh local"):
        actual.main(argv)


def test_external_revision_mismatch_rejects_before_any_receipt_or_ssh(ready):
    argv = ready.seal(); argv[argv.index("--expected-revision-sha256") + 1] = "b" * 64
    with pytest.raises(ValueError, match="revision SHA differs"): actual.main(argv)
    assert ready.calls == [] and not Path(argv[-1]).exists()


@pytest.mark.parametrize("field, wrong", [("remote_source", "wrong"), ("remote_run", "wrong"),
    ("parent_four_actual_sha256", {}), ("optimizer_started", True), ("packages_installed", True)])
def test_wrong_preparation_never_dispatches(ready, field, wrong):
    ready.document[field] = wrong
    with pytest.raises(ValueError): actual.main(ready.seal())
    assert ready.calls == []


def test_same_v1_module_command_cannot_be_reused(ready):
    value = ready.document["proposed_fit_command_requires_separate_root_execution"].replace(actual.MODULE,
        "scripts.e2e.train_temporal_intrinsic_curve_pair_20261002")
    ready.document["proposed_fit_command_requires_separate_root_execution"] = value
    ready.summary["proposed_fit_command_requires_separate_root_execution"] = value
    with pytest.raises(ValueError, match="arbitrary command"): actual.main(ready.seal())
    assert ready.calls == []


def test_local_revision_mutation_after_gpu_query_never_launches(ready, monkeypatch):
    original = actual.previous.subprocess.run
    def mutate(*args, **kwargs):
        result = original(*args, **kwargs); (actual.ROOT / actual.REVISION_SOURCE).write_bytes(b"mutated"); return result
    monkeypatch.setattr(actual.previous.subprocess, "run", mutate)
    with pytest.raises(ValueError, match="changed"): actual.main(ready.seal())
    assert [call[0] for call in ready.calls] == ["preflight"]


@pytest.mark.parametrize("flag", ["--expected-prepare-source-sha256", "--expected-revision-sha256"])
def test_boolean_or_nonsha_external_identity_fails(ready, flag):
    argv = ready.seal(); argv[argv.index(flag) + 1] = "True"
    with pytest.raises(ValueError, match="external V2 SHA256"): actual.main(argv)
    assert ready.calls == []
