"""HH_261002 - Full synthetic20k archives; no actual weights, data, SSH or GPU."""
import ast
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest

from scripts.e2e import mirror_completed_temporal_left_joint_20261002 as tool


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True); path.write_text(json.dumps(value, allow_nan=False))


@pytest.fixture(scope="module")
def original(tmp_path_factory):
    base = tmp_path_factory.mktemp("hh_synthetic_LEFT20k_mirror")
    run, source = base / "run", base / "source"; run.mkdir(); source.mkdir(); sources = {}
    for i in range(29):
        name = "tools/synthetic_%02d.py" % i; path = source / name; path.parent.mkdir(exist_ok=True)
        path.write_text("# synthetic source %d\n" % i); sources[name] = tool.digest(path)
    path = source / tool.TRAINER; path.parent.mkdir(parents=True)
    path.write_bytes((tool.ROOT / tool.TRAINER).read_bytes()); sources[tool.TRAINER] = tool.digest(path)
    protocol = dict(schema=tool.TRAINING, sources=sources, actual_parent_final6000=tool.PARENT,
        actual_parent_history_sha256=tool.PARENT_HISTORY, new_LEFT_conversion_sha256=tool.LEFT_SHA,
        fixed_new_updates=20000, examples_seen=80000, training_episode_count=19, learning_rate=tool.learning_rate_config())
    put(source / "authorization.json", dict(schema=tool.TRAINING + ".prospective_authorization", authorized=True,
        contract=protocol, contract_sha256=tool.json_sha(protocol), model_promoted=False, vehicle_control_approved=False))
    auth = tool.digest(source / "authorization.json")
    order = [[i % 2718, 2718 + i % 327, 3045 + i % 2787, 5832 + i % 347] for i in range(20000)]
    tokens = [["sample_%d" % index for index in row] for row in order]
    cohorts = {name: dict(split="train") for name in ("recovery", "v4", "legacy")}
    cohorts["LEFT"] = dict(split="train", sample_count=347, episode_count=1, conversion_sha256=tool.LEFT_SHA)
    declaration = dict(schema=tool.TRAINING, protocol=protocol, source_sha256=sources, authorization_sha256=auth,
        training_episode_count=19, trainable_state_tensor_count=54, frozen_state_tensor_count=47,
        parent_history_sha256=tool.PARENT_HISTORY, cohorts=cohorts, fixed_sample_ids=tokens,
        fixed_sample_ids_sha256=tool.json_sha(tokens), fixed_schedule_sha256=tool.json_sha(order))
    put(run / "declaration.json", declaration); declsha = tool.digest(run / "declaration.json")
    for name in sources:
        target = run / "source" / name; target.parent.mkdir(parents=True, exist_ok=True); shutil.copyfile(source / name, target)
    namespace = tool.journal_namespace(tool.journal_code()); receipts = []; prefix = hashlib.sha256(); history = bytearray()
    for index, (ids, samples) in enumerate(zip(order, tokens), 1):
        row = dict(optimizer_updates=index, examples_seen=4 * index, indices=ids, sample_ids=samples,
            batch_sha256="a" * 64, loss=.5, preclip_gradient_norm=.1, elapsed_fit_seconds=index * .01,
            legacy_curved=index % 2 == 1, recovery_focus=index % 2 == 1,
            learning_rates={g: namespace["learning_rate"](index, g) for g in ("shared", "heads")})
        payload = (json.dumps(row) + "\n").encode(); history.extend(payload); prefix.update(payload)
        if index % 1000: continue
        export = run / ("exports/step_%06d" % index); export.mkdir(parents=True)
        checkpoint = run / ("checkpoints/step_%06d.pt" % index); checkpoint.parent.mkdir(exist_ok=True)
        checkpoint.write_bytes(b"hash only synthetic PT, deliberately NOT pickle")
        (export / "model_state.npz").write_bytes(b"hash only synthetic NPZ, deliberately NOT numpy")
        metadata = dict(schema=tool.EXPORT, training_schema=tool.TRAINING, optimizer_updates=index,
            additional_optimizer_updates=index, maximum_optimizer_updates=20000, examples_seen=4 * index,
            known_total_optimizer_updates=12200 + index, source_sha256=sources, protocol=protocol,
            declaration_sha256=declsha, history_sha256=prefix.hexdigest(), frozen_state_sha256="f" * 64,
            trainable_state_tensor_count=54, frozen_state_tensor_count=47, learning_rate=tool.learning_rate_config(),
            fixed_sample_ids_sha256=declaration["fixed_sample_ids_sha256"], fixed_schedule_sha256=declaration["fixed_schedule_sha256"],
            runtime_loader_implemented=False, model_promoted=False, vehicle_control_approved=False)
        put(export / "metadata.json", metadata)
        receipts.append(dict(optimizer_updates=index, checkpoint_sha256=tool.digest(checkpoint),
            npz_sha256=tool.digest(export / "model_state.npz"), metadata_sha256=tool.digest(export / "metadata.json")))
    (run / "history.jsonl").write_bytes(history)
    fit = dict(optimizer_updates=20000, examples_seen=80000, completed_fixed_endpoint=True,
        DEV_or_HOLDOUT_read=False, model_promoted=False, checkpoints=receipts); put(run / "fit_result.json", fit)
    screens = dict(diagnostic_screen_count=74, all_diagnostic_screens_passed=False,
        checks=[dict(name="screen_%02d" % i, passed=i % 2 == 0) for i in range(74)])
    put(run / "summary.json", dict(schema=tool.TRAINING, status="FIXED20000_ACTUAL_LEFT_JOINT_COMPLETE_NOT_PROMOTED",
        fit=fit, all71_original_screens_retained=True, full_LEFT347_retained=True,
        endpoint_diagnostic_screens=screens, model_promoted=False, vehicle_control_approved=False))
    put(run / "evaluation_opened_after_final.json", dict(checkpoint_selection=False,
        endpoint=dict(receipts[-1], all101_numeric_finite=True, frozen47_byte_identical=True,
            all20000_actual_journal_rows_verified=True)))
    for label in ("parent", "final"):
        for cohort in ("recovery_train", "recovery_holdout", "v4", "legacy", "known599", "LEFT"):
            put(run / (label + "_" + cohort + ".json"), dict(synthetic=True))
    return base


@pytest.fixture
def archive(original, tmp_path):
    shutil.copytree(original, tmp_path / "archive")
    return tmp_path / "archive/run", tmp_path / "archive/source"


def inspect_run(run, source):
    return tool.inspect_run(run, source, tool.digest(run / "summary.json"), tool.digest(run / "declaration.json"),
        tool.digest(source / "authorization.json"), tool.journal_code())


def test_complete_single20k_all_failures_and_hash_only_artifacts(archive):
    run, source = archive; evidence = inspect_run(run, source)
    assert len(evidence["sources"]) == 30 and evidence["diagnostic_pass_count"] == 37
    assert evidence["journal"]["actual_updates"] == 20000 and evidence["journal"]["actual_examples_seen"] == 80000
    assert min(evidence["journal"]["all347_fair_exposure_counts"]) == 57
    assert max(evidence["journal"]["all347_fair_exposure_counts"]) == 58
    assert sum(name.startswith("checkpoints/") for name in evidence["files"]) == 20
    assert evidence["endpoint"]["weights_sha256"] == tool.digest(run / "exports/step_020000/model_state.npz")
    assert not (run / "completed_run_authority.json").exists()


@pytest.mark.parametrize("change", ["incomplete", "fit_only", "failure_receipt", "missing_checkpoint", "missing_export", "missing_eval",
    "source_changed", "archive_changed", "auth_changed", "promoted", "19TRAIN", "parent", "LEFTreceipt", "checkpoint_hash",
    "journal_prefix", "eval_early", "screen_missing", "screen_duplicate", "screen_nonbool", "screen_false_claim"])
def test_completion_source_endpoint_and_all74_fail_closed(archive, change):
    run, source = archive
    if change in ("incomplete", "fit_only", "promoted", "screen_missing", "screen_duplicate", "screen_nonbool", "screen_false_claim"):
        path = run / "summary.json"; d = tool.read(path, tool.digest(path))
        if change == "incomplete": d["status"] = "INCOMPLETE_FIXED20K_NO_EVALUATION_NOT_PROMOTED"
        elif change == "fit_only": d["fit"]["completed_fixed_endpoint"] = False
        elif change == "promoted": d["model_promoted"] = True
        elif change == "screen_missing": d["endpoint_diagnostic_screens"]["checks"].pop()
        elif change == "screen_duplicate": d["endpoint_diagnostic_screens"]["checks"][-1]["name"] = "screen_00"
        elif change == "screen_nonbool": d["endpoint_diagnostic_screens"]["checks"][0]["passed"] = 1
        else: d["endpoint_diagnostic_screens"]["all_diagnostic_screens_passed"] = True
        put(path, d)
    elif change == "failure_receipt": put(run / "failure_receipt.json", dict(failed=True))
    elif change == "missing_checkpoint": (run / "checkpoints/step_001000.pt").unlink()
    elif change == "missing_export": (run / "exports/step_020000/model_state.npz").unlink()
    elif change == "missing_eval": (run / "parent_known599.json").unlink()
    elif change == "source_changed": (source / "tools/synthetic_00.py").write_text("changed")
    elif change == "archive_changed": (run / "source/tools/synthetic_00.py").write_text("changed")
    elif change in ("auth_changed", "19TRAIN", "parent", "LEFTreceipt"):
        path = source / "authorization.json"; d = tool.read(path, tool.digest(path))
        if change == "auth_changed": d["authorized"] = False
        elif change == "19TRAIN": d["contract"]["training_episode_count"] = 18
        elif change == "parent": d["contract"]["actual_parent_final6000"]["weights_sha256"] = "b" * 64
        else: d["contract"]["new_LEFT_conversion_sha256"] = "b" * 64
        put(path, d)
    elif change in ("checkpoint_hash", "journal_prefix"):
        path = run / "exports/step_001000/metadata.json"; d = tool.read(path, tool.digest(path))
        d["history_sha256" if change == "journal_prefix" else "source_sha256"] = "b" * 64; put(path, d)
    else:
        path = run / "evaluation_opened_after_final.json"; d = tool.read(path, tool.digest(path)); d["endpoint"]["all20000_actual_journal_rows_verified"] = False; put(path, d)
    with pytest.raises(Exception): inspect_run(run, source)


@pytest.mark.parametrize("change", ["count", "ids", "indices", "LR", "typed_index", "batch", "nonfinite", "duplicate", "LEFTfaircycle"])
def test_full_actual_history_rejects_any_invented_update(archive, change):
    run, source = archive; path = run / "history.jsonl"; rows = path.read_bytes().splitlines(); d = json.loads(rows[99])
    if change == "count": rows.pop()
    elif change == "duplicate": rows[99] = rows[99][:-1] + b',"optimizer_updates":100}'
    elif change == "nonfinite": rows[99] = rows[99].replace(b'"loss": 0.5', b'"loss": 1e400')
    else:
        if change == "ids": d["sample_ids"][3] = "unknown"
        elif change == "indices": d["indices"][0] = 2718
        elif change == "LR": d["learning_rates"]["heads"] *= 2
        elif change == "typed_index": d["optimizer_updates"] = 100.0
        elif change == "batch": d["batch_sha256"] = "bad"
        else: d["indices"][3] = 5832
        rows[99] = json.dumps(d).encode()
    path.write_bytes(b"\n".join(rows) + b"\n")
    with pytest.raises(Exception): inspect_run(run, source)


def test_frozen_pure_journal_code_ast_is_exact_and_no_torch_import():
    path = tool.ROOT / "learning/portable_e2e/temporal_left_joint_numeric_loader_20261002.py"
    selected = [n for n in ast.parse(path.read_text()).body if isinstance(n, ast.FunctionDef) and n.name in ("learning_rate", "verify_history")]
    assert ast.dump(ast.Module(body=selected, type_ignores=[])) == ast.dump(ast.parse(tool.journal_code()))
    assert tool.canonical is tool.frozen.canonical and tool.inventory is tool.frozen.inventory


def test_remote_program_uses_same_algorithm_and_private_cpu_only_python(archive, monkeypatch):
    run, source = archive; monkeypatch.setattr(tool, "RUN", run); monkeypatch.setattr(tool, "SOURCE", source)
    args = SimpleNamespace(expected_summary_sha256=tool.digest(run / "summary.json"), expected_declaration_sha256=tool.digest(run / "declaration.json"), authorization_sha256=tool.digest(source / "authorization.json"))
    command = tool.remote_command(args); program = shlex.split(command[-1])[-1]
    result = subprocess.run([sys.executable, "-I", "-B", "-c", program], capture_output=True, check=True)
    assert json.loads(result.stdout) == inspect_run(run, source)
    assert "CUDA_VISIBLE_DEVICES=" in command[-1] and "import torch" not in program and "nvidia-smi" not in program
    assert "python -I -B" in command[-1] and command[:len(tool.SSH)] == tool.SSH


def test_parser_rejects_duplicate_nonfinite_and_oversize(tmp_path):
    path = tmp_path / "bad.json"
    for text in ('{"x":1,"x":2}', '{"x":NaN}', '{"x":1e400}'):
        path.write_text(text)
        with pytest.raises(ValueError): tool.read(path, tool.digest(path))
    path.write_text('{"x":1}')
    with pytest.raises(ValueError, match="bounded"): tool.read(path, tool.digest(path), maximum=1)


def test_symlink_and_lexical_escape_rejected(archive):
    run, source = archive; (run / "linked").symlink_to(source, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"): inspect_run(run, source)
    with pytest.raises(ValueError): tool.canonical(tool.ROOT / "tools/../storage")


def test_first_read_sha_rechecked_before_parse(archive, monkeypatch):
    run, source = archive; original_read = tool.read
    def changed(path, expected, maximum=16 * 1024**2):
        if path == run / "summary.json": put(path, dict(original_read(path, expected), mutation=True))
        return original_read(path, expected, maximum)
    monkeypatch.setattr(tool, "read", changed)
    with pytest.raises(ValueError, match="changed before parsing"): inspect_run(run, source)


def test_fixed_scope_create_only_mirror_and_no_self_authorization(archive, tmp_path, monkeypatch):
    run, source = archive; before = inspect_run(run, source); local, receipt = tmp_path / "mirror", tmp_path / "receipt"
    code = tool.journal_code(); monkeypatch.setattr(tool, "journal_code", lambda: code)
    monkeypatch.setattr(tool, "ROOT", source); monkeypatch.setattr(tool, "LOCAL", local); monkeypatch.setattr(tool, "RECEIPT", receipt)
    commands = []
    def invoked(command, **kwargs):
        commands.append(command)
        if command[0] == "rsync": shutil.copytree(run, local, dirs_exist_ok=True); return SimpleNamespace(returncode=0, stdout=b"", stderr=b"")
        return SimpleNamespace(returncode=0, stdout=json.dumps(before).encode(), stderr=b"")
    monkeypatch.setattr(tool.subprocess, "run", invoked); monkeypatch.setattr(tool.shutil, "disk_usage", lambda p: SimpleNamespace(free=20 * 1024**3))
    argv = [v for k, pin in (("expected-summary-sha256", tool.digest(run / "summary.json")), ("expected-declaration-sha256", tool.digest(run / "declaration.json")),
        ("authorization-sha256", tool.digest(source / "authorization.json")), ("expected-tool-sha256", tool.digest(tool.__file__))) for v in ("--" + k, pin)]
    assert tool.main(argv) == 0
    evidence = tool.read(receipt / "completed_run_for_root_review.json", tool.digest(receipt / "completed_run_for_root_review.json"))
    assert evidence["numeric_state_validated"] is evidence["simulator_control_authorized"] is evidence["model_promoted"] is False
    assert evidence["journal"]["actual_examples_seen"] == 80000 and evidence["diagnostic_screen_count"] == 74
    assert "--ignore-existing" in next(c for c in commands if c[0] == "rsync") and tool.inventory(local) == before["files"]
    with pytest.raises(ValueError, match="fresh destinations"): tool.main(argv)


def test_fixed_public_paths_are_single20k_not_old_pair():
    assert str(tool.RUN).endswith("runs/training/temporal_left_joint_20261002_v1")
    assert str(tool.SOURCE).endswith("source_worktrees/temporal_left_joint_20261002_v1")
    assert tool.LOCAL.name == "temporal_left_joint_v1" and tool.RECEIPT.name == "temporal_left_joint_mirror_v1"


@pytest.mark.parametrize("change", ["disk_floor", "remote_changed", "timeout"])
def test_copy_failure_keeps_partial_logs_and_never_authorizes(archive, tmp_path, monkeypatch, change):
    run, source = archive; before = inspect_run(run, source); local, receipt = tmp_path / "mirror", tmp_path / "receipt"
    code = tool.journal_code(); monkeypatch.setattr(tool, "journal_code", lambda: code)
    monkeypatch.setattr(tool, "ROOT", source); monkeypatch.setattr(tool, "LOCAL", local); monkeypatch.setattr(tool, "RECEIPT", receipt)
    calls = []
    def invoked(command, **kwargs):
        calls.append(command)
        if change == "timeout": raise subprocess.TimeoutExpired(command, 1800, output=b"partial stdout", stderr=b"partial stderr")
        if command[0] == "rsync": shutil.copytree(run, local, dirs_exist_ok=True); return SimpleNamespace(returncode=0, stdout=b"", stderr=b"")
        value = before if len(calls) == 1 else dict(before, changed=True)
        return SimpleNamespace(returncode=0, stdout=json.dumps(value).encode(), stderr=b"")
    monkeypatch.setattr(tool.subprocess, "run", invoked)
    monkeypatch.setattr(tool.shutil, "disk_usage", lambda p: SimpleNamespace(free=1 if change == "disk_floor" else 20 * 1024**3))
    argv = [v for k, pin in (("expected-summary-sha256", tool.digest(run / "summary.json")), ("expected-declaration-sha256", tool.digest(run / "declaration.json")),
        ("authorization-sha256", tool.digest(source / "authorization.json")), ("expected-tool-sha256", tool.digest(tool.__file__))) for v in ("--" + k, pin)]
    with pytest.raises(ValueError): tool.main(argv)
    assert (receipt / "failure_receipt.json").is_file() and not (receipt / "completed_run_for_root_review.json").exists()
    if change == "disk_floor": assert len(calls) == 1 and not local.exists()
    elif change == "timeout":
        assert (receipt / "remote_before_stdout.txt").read_bytes() == b"partial stdout"
        assert (receipt / "remote_before_stderr.txt").read_bytes() == b"partial stderr"
    else: assert local.is_dir() and tool.inventory(local) == before["files"]
