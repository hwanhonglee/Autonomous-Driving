"""HH_261005 - Scalar/AST fixtures only; no actual evaluation, dataset, Torch, SSH or subprocess."""
from __future__ import annotations

import ast
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tools/maintenance/postcheck_three_seed_final_evaluation_v1_20261005.py"
spec = importlib.util.spec_from_file_location("hh_scalar_whole_eval_post", SOURCE)
post = importlib.util.module_from_spec(spec); spec.loader.exec_module(post)


@pytest.fixture
def small(monkeypatch):
    # HH_261005 - Explicit tiny scalar fixture, not authentic cohort membership or a model screen.
    endpoints = ("common3000", "GOOD6000"); counts = dict.fromkeys(post.COUNTS, 1)
    monkeypatch.setattr(post, "ENDPOINTS", endpoints); monkeypatch.setattr(post, "COUNTS", counts)
    expected = {c: [(i, "token-" + c, "episode-" + c)] for i, c in enumerate(counts)}
    metrics, journal = {}, []
    for endpoint in endpoints:
        metrics[endpoint] = {}
        for cohort in counts:
            index, token, episode = expected[cohort][0]
            row = dict(index=index, token=token, scalar=0.125)
            if cohort in ("legacy", "known599"): row["episode_id"] = episode
            journal.append(dict(endpoint=endpoint, cohort=cohort, ordinal=0, record=deepcopy(row)))
            if cohort in post.LATE_EPISODE_COHORTS: row["episode_id"] = episode
            metrics[endpoint][cohort] = dict(rows=[row])
    return journal, metrics, expected


def test_complete_ordered_rows_only_three_late_episode_fields(small):
    result = post.reconcile_records(*small)
    assert result["independently_verified_completed"] == result["full_denominator"] == 12
    assert result["journal_finalized_metric_row_byte_equality_claimed"] is False
    assert all("episode_id" not in r["record"] for r in small[0] if r["cohort"] in post.LATE_EPISODE_COHORTS)


@pytest.mark.parametrize("fault", ("missing", "extra", "order", "duplicate", "endpoint", "cohort", "ordinal_bool",
    "index_bool", "token", "nan", "extra_envelope", "late_missing", "late_changed_scalar", "late_wrong_episode",
    "journal_added_episode", "legacy_changed", "legacy_wrong_episode", "metric_missing", "metric_short", "expected_wrong", "expected_type"))
def test_all_faults_reject_without_selected_row_recovery(small, fault):
    rows, metrics, expected = deepcopy(small); first = rows[0]; late = metrics["common3000"]["recovery_train"]["rows"][0]
    if fault == "missing": rows.pop()
    elif fault == "extra": rows.append(deepcopy(rows[-1]))
    elif fault == "order": rows[0], rows[1] = rows[1], rows[0]
    elif fault == "duplicate": rows[1] = deepcopy(rows[0])
    elif fault == "endpoint": first["endpoint"] = "seed_2026100503"
    elif fault == "cohort": first["cohort"] = "known599"
    elif fault == "ordinal_bool": first["ordinal"] = False
    elif fault == "index_bool": first["record"]["index"] = False
    elif fault == "token": first["record"]["token"] = "another-authentic-looking-token"
    elif fault == "nan": first["record"]["scalar"] = float("nan")
    elif fault == "extra_envelope": first["replayed"] = True
    elif fault == "late_missing": late.pop("episode_id")
    elif fault == "late_changed_scalar": late["scalar"] += 0.001
    elif fault == "late_wrong_episode": late["episode_id"] = "other-episode"
    elif fault == "journal_added_episode": first["record"]["episode_id"] = late["episode_id"]
    elif fault in ("legacy_changed", "legacy_wrong_episode"):
        row = metrics["common3000"]["legacy"]["rows"][0]
        row["scalar" if fault == "legacy_changed" else "episode_id"] = "changed"
        if fault == "legacy_wrong_episode": rows[3]["record"]["episode_id"] = "changed"
    elif fault == "metric_missing": metrics["common3000"].pop("v4")
    elif fault == "metric_short": metrics["GOOD6000"]["intrinsic_legacy_TRAIN_only"]["rows"] = []
    elif fault == "expected_wrong": expected["recovery_train"] = [(0, "wrong-catalog-token", "episode-recovery_train")]
    elif fault == "expected_type": expected["recovery_train"] = [(False, "token-recovery_train", "episode-recovery_train")]
    with pytest.raises(RuntimeError): post.reconcile_records(rows, metrics, expected)


def test_full35315_planned_unknown_is_not_zero_or_reported_pass():
    summary = dict(complete_denominator=dict(full_denominator=35315, all_completed=True))
    result = post.planned_ledger(summary)
    assert result["full_denominator"] == 35315 and result["independently_verified_completed"] is None
    assert sum(r["expected"] for v in result["all30_planned_metric_files"].values() for r in v.values()) == 35315
    assert result["retained_original_reported_denominator"] == summary["complete_denominator"]
    assert all(r["verified_completed"] is None for v in result["all30_planned_metric_files"].values() for r in v.values())


def denominator_fixture():
    ledger = dict(full_denominator=35315, verified_counts={e: dict(post.COUNTS) for e in post.ENDPOINTS})
    summary = dict(complete_denominator=dict(full_denominator=35315, all_completed=True,
        failed_partial_rows_retained=True, last_unretained_actual_candidate=None, full_completed_scalar_journal_bytes=123,
        endpoints={e: {c: dict(expected=n, completed=n, UNEXECUTED=0) for c, n in post.COUNTS.items()} for e in post.ENDPOINTS}))
    return summary, ledger


@pytest.mark.parametrize("fault", (None, "partial", "bytes", "unretained", "missing_cell", "denominator"))
def test_original_all30_receipt_is_independently_checked(fault):
    summary, ledger = denominator_fixture(); row = summary["complete_denominator"]
    if fault == "partial": row["endpoints"]["GOOD6000"]["known599"]["UNEXECUTED"] = 1
    elif fault == "bytes": row["full_completed_scalar_journal_bytes"] = 122
    elif fault == "unretained": row["last_unretained_actual_candidate"] = dict(token="late")
    elif fault == "missing_cell": row["endpoints"]["seed_2026100503"].pop("v4")
    elif fault == "denominator": ledger["full_denominator"] = 35314
    if fault is None: post.verify_denominator(summary, ledger, 123)
    else:
        with pytest.raises(RuntimeError): post.verify_denominator(summary, ledger, 123)


def scalar_screen_metrics():
    # HH_261005 - Original comparator scalars only; row lists here are NOT full actual cohort records.
    metric = {}
    for c, episodes, n in (("recovery_train", 9, 2718), ("recovery_holdout", 2, 604), ("legacy", 8, 2487), ("known599", 2, 599)):
        values = {"e%d" % i: dict(ade_6p4s_m=.05, fde_6p4s_m=.1, instantaneous_speed_mae_mps=.3,
            raw_curvature=.02, raw_spatial_preferred_speed_mae_mps=.25) for i in range(episodes)}
        denominator = "full_fixed_denominator" if c.startswith("recovery_") else "full_fixed_eligible_denominator"
        metric[c] = dict(per_episode=values, rows=[dict(index=0, token=c)], groups=dict(all_eligible=deepcopy(values["e0"])), **{denominator: n})
    metric["v4"] = dict(full_fixed_denominator=327, rows=[dict(token="v4")],
        mean_ade_6p4s_m=.05, mean_fde_6p4s_m=.1, mean_instantaneous_speed_mae_mps=.3)
    metric["intrinsic_legacy_TRAIN_only"] = dict(rows=[])
    return {e: deepcopy(metric) for e in post.ENDPOINTS}


def test_only_exact_original_two_private_functions_recompute_all497_and_failures():
    function = post.pure_screens(ROOT); metrics = scalar_screen_metrics()
    summary = dict(common3000_screens={e: function(metrics["common3000"], metrics[e]) for e in post.ENDPOINTS[1:]},
        GOOD6000_reference_screens={e: function(metrics["GOOD6000"], metrics[e]) for e in post.ENDPOINTS[2:]})
    result = post.verify_screens(metrics, summary, function)
    assert result["screen_count"] == 497 and result["failed_screen_count"] == 77
    assert result["metric_aggregate_arithmetic_independently_recomputed"] is False
    assert set(function.__globals__) == {"math", "require", "previous", "__builtins__", "endpoint_screens"}
    assert set(function.__globals__["previous"].endpoint_screens.__globals__) == {"math", "require", "__builtins__", "endpoint_screens"}
    summary["common3000_screens"]["GOOD6000"]["checks"][0]["threshold"] += .001
    with pytest.raises(RuntimeError): post.verify_screens(metrics, summary, function)


def test_screen_source_change_rejects_before_scalar_compile(monkeypatch):
    monkeypatch.setattr(post, "RECOVERY_SHA", "0" * 64)
    with pytest.raises(RuntimeError, match="frozen screen source"): post.pure_screens(ROOT)


@pytest.mark.parametrize("raw", (b'{"x":1,"x":2}', b'{"x":NaN}', b'{"x":Infinity}'))
def test_raw_json_duplicates_nonfinite_denied(raw):
    with pytest.raises(RuntimeError): post.decode(raw)


def test_read_exact_raw_not_semantic_reencoded_hash_and_no_alias(tmp_path):
    path = tmp_path / "raw.json"; raw = b'{"world_ns":1834547769345423011,"x":5.0}\n'; path.write_bytes(raw)
    value = post.read(path, expected=post.sha(raw))
    assert value["world_ns"] == 1834547769345423011 and type(value["x"]) is float
    with pytest.raises(RuntimeError): post.read(path, expected=post.sha(post.encode(value)))
    alias = tmp_path / "alias.json"; alias.symlink_to(path)
    with pytest.raises(RuntimeError): post.read(alias)
    path.write_bytes(b'{"x":1e400}')
    with pytest.raises(RuntimeError): post.read(path)


def process_fixture():
    leader = dict(pid=101, ppid=100, pgid=101, session=101, start_ticks=200)
    child = dict(pid=102, ppid=101, pgid=101, session=101, start_ticks=201)
    parent = dict(pid=100, ppid=99, pgid=90, session=90, start_ticks=190)
    return (dict(leader=leader, known_generations=[deepcopy(leader), child], remaining_session={}, unknown_session_members=[]),
        dict(leader=deepcopy(leader), caller=parent), dict(caller=deepcopy(parent)), [])


@pytest.mark.parametrize("fault", (None, "leader_generation", "parent", "boolean_pid", "orphan", "duplicate", "foreign_group", "unknown", "survivor"))
def test_historical_owned_ancestry_and_actual_closed_session(fault):
    first, started, caller, current = process_fixture()
    if fault == "leader_generation": first["leader"] = dict(first["leader"], start_ticks=201)
    elif fault == "parent": caller["caller"]["pid"] = 1
    elif fault == "boolean_pid": started["leader"]["pid"] = True
    elif fault == "orphan": first["known_generations"][1]["ppid"] = 999
    elif fault == "duplicate": first["known_generations"].append(deepcopy(first["known_generations"][1]))
    elif fault == "foreign_group": first["known_generations"][1]["pgid"] = 999
    elif fault == "unknown": first["unknown_session_members"] = [dict(pid=999)]
    elif fault == "survivor": current = [deepcopy(started["leader"])]
    if fault is None: post.verify_owned_receipts(first, started, caller, current)
    else:
        with pytest.raises(RuntimeError): post.verify_owned_receipts(first, started, caller, current)


def endpoint_fixture():
    authority = dict(completed_seeds={}); immutable, documents = {}, {}
    for number in (1, 2, 3):
        seed = 2026100500 + number; root = "/owned/seed_%d" % seed
        entry = dict(endpoint_root=root)
        for name in ("whole_postcheck", "binding", "review"):
            path, pin = root + "/" + name + ".json", str(number)*64
            entry[name+"_path"], entry[name+"_sha256"] = path, pin; immutable[path] = pin
        checks = dict.fromkeys(("all_6000_schedule_rows_exact", "all_12_checkpoint_triplets_exact", "all_101_final_and_frozen47_exact",
            "source_current_and_archive_grants_exact", "complete_raw_streams_exact", "owned_session_closed", "all_failed_tails_preserved"), True)
        documents[entry["whole_postcheck_path"]] = dict(seed=seed, status="PASS_WHOLE_SEED%d_FIXED6000_NOT_PROMOTED" % number,
            checked_optimizer_updates=6000, checked_examples_seen=24000, checks=checks)
        files = {"file%02d.json" % i: str(number)*64 for i in range(42)}
        documents[entry["binding_path"]] = dict(seed=seed, endpoint_root=root, whole_training_postcheck_sha256=entry["whole_postcheck_sha256"], files=files)
        documents[entry["review_path"]] = dict(seed=seed, endpoint_root=root, whole_training_postcheck_sha256=entry["whole_postcheck_sha256"],
            endpoint_binding_sha256=entry["binding_sha256"])
        immutable.update({root+"/"+name: pin for name, pin in files.items()}); authority["completed_seeds"][str(seed)] = entry
    def reader(path, *, expected):
        assert immutable[str(path)] == expected
        return deepcopy(documents[str(path)])
    return authority, immutable, documents, reader


@pytest.mark.parametrize("fault", (None, "binding_seed", "binding_bool", "binding_root", "binding_post", "review_seed", "review_root",
    "review_post", "review_binding", "swapped_authentic_binding", "missing42", "unsealed_payload", "whole_training_fail", "whole_tail"))
def test_all9_authentic_documents_and42_payload_seals_cannot_cross_seed(fault):
    authority, immutable, documents, reader = endpoint_fixture(); entry = authority["completed_seeds"]["2026100501"]
    binding = documents[entry["binding_path"]]; review = documents[entry["review_path"]]; original = documents[entry["whole_postcheck_path"]]
    if fault == "binding_seed": binding["seed"] = 2026100502
    elif fault == "binding_bool": binding["seed"] = True
    elif fault == "binding_root": binding["endpoint_root"] = "/owned/seed_2026100502"
    elif fault == "binding_post": binding["whole_training_postcheck_sha256"] = "2"*64
    elif fault == "review_seed": review["seed"] = 2026100502
    elif fault == "review_root": review["endpoint_root"] = "/owned/seed_2026100502"
    elif fault == "review_post": review["whole_training_postcheck_sha256"] = "2"*64
    elif fault == "review_binding": review["endpoint_binding_sha256"] = "2"*64
    elif fault == "swapped_authentic_binding": documents[entry["binding_path"]] = deepcopy(documents[authority["completed_seeds"]["2026100502"]["binding_path"]])
    elif fault == "missing42": binding["files"].pop("file41.json")
    elif fault == "unsealed_payload": immutable.pop(entry["endpoint_root"] + "/file41.json")
    elif fault == "whole_training_fail": original["status"] = "FAIL_OR_UNKNOWN"
    elif fault == "whole_tail": original["checks"]["all_failed_tails_preserved"] = False
    if fault is None: post.verify_endpoint_documents(authority, immutable, reader)
    else:
        with pytest.raises(RuntimeError): post.verify_endpoint_documents(authority, immutable, reader)


class FakeSSH:
    def __init__(self, *, timeout=False): self.pid, self.returncode, self.actions, self.timeout = 4567, None, [], timeout
    def poll(self): return self.returncode
    def terminate(self): self.actions.append("TERM")
    def kill(self): self.actions.append("KILL"); self.returncode = -9
    def wait(self, timeout):
        self.actions.append(("WAIT", timeout))
        if self.timeout and self.returncode is None: raise subprocess.TimeoutExpired("fake-ssh", timeout)
        self.returncode = -15 if self.returncode is None else self.returncode; return self.returncode


@pytest.mark.parametrize("fault", (None, "wait_timeout", "generation", "survivor"))
def test_local_cleanup_signals_only_exact_direct_owned_generation(monkeypatch, fault):
    child = FakeSSH(timeout=fault == "wait_timeout"); owned = dict(pid=4567, ppid=os.getpid(), pgid=4567, session=4567, start_ticks=100)
    monkeypatch.setattr(post, "identity", lambda pid: dict(owned, start_ticks=101) if fault == "generation" else dict(owned))
    monkeypatch.setattr(post, "session_members", lambda core: [dict(pid=999)] if fault == "survivor" else [])
    result = post.cleanup_ssh(child, owned)
    if fault == "generation": assert "error" in result and child.actions == []
    elif fault == "survivor": assert "error" in result and "KILL" not in child.actions
    else: assert result["remaining_session"] == [] and ("KILL" in child.actions) == (fault == "wait_timeout")


def test_nonexistent_fixed_review_denies_before_any_ssh(monkeypatch, tmp_path):
    monkeypatch.setattr(post, "REVIEW", tmp_path / "absent_ROOT_review.json")
    monkeypatch.setattr(post.subprocess, "Popen", lambda *a, **k: pytest.fail("SSH forbidden before ROOT admission"))
    with pytest.raises(FileNotFoundError): post.main(["--review-sha256", "a" * 64])


def fake_admitted_caller(monkeypatch, tmp_path, fault):
    # HH_261005 - A complete mock admission isolates local receipt/failure mechanics; it is not an actual ROOT grant.
    plan = tmp_path / "plans"; out = plan / "supervisor_v2"; out.mkdir(parents=True)
    review_path = plan / "post_review.json"; authority_path = plan / "eval_authority_v2.json"; execution_path = plan / "eval_review_v2.json"
    monkeypatch.setattr(post, "ROOT", tmp_path); monkeypatch.setattr(post, "PLAN", plan); monkeypatch.setattr(post, "OUT", out)
    monkeypatch.setattr(post, "REVIEW", review_path); monkeypatch.setattr(post, "EVAL_AUTHORITY", authority_path)
    monkeypatch.setattr(post, "EVAL_REVIEW", execution_path); monkeypatch.setattr(post, "TEST", tmp_path / "test_source.py")
    program = "# source-only mock remote program\n"; monkeypatch.setattr(post, "remote_program", lambda: program)
    review = dict(schema=post.SCHEMA + ".ROOT_execution_review", status="APPROVED_READONLY_WHOLE_FINAL_EVALUATION_POSTCHECK",
        source_sha256="a"*64, test_sha256="a"*64, read_only_CPU_authorized=True, NN_forward_authorized=False,
        optimizer_authorized=False, CUDA_authorized=False, expected_local_supervisor_stdout_sha256="d"*64,
        evaluation_authority_sha256="b"*64, evaluation_execution_review_sha256="c"*64,
        source_archive_sha256="e"*64, remote_program_sha256=post.sha(program.encode()))
    local = dict(status="COMPLETE_FIXED_FIVE_EVALUATION_NOT_PROMOTED",
        remote_receipt="/home/pro6000/personal/hwanhong/portable_e2e/plans/evaluation/" + post.STUDY + "/supervisor_v2")
    authority = dict(contract=dict(source_sha256={"source%02d.py" % i: "d"*64 for i in range(35)}))
    execution = dict(archive_sha256="e"*64, immutable_input_sha256={})
    table = {review_path: review, out / "SSH.stdout": local, authority_path: authority, execution_path: execution}
    monkeypatch.setattr(post, "read", lambda p, *a, **k: deepcopy(table[Path(p)]))
    def digest(path):
        path = Path(path)
        if path == tmp_path/post.SUPERVISOR: return post.SUPERVISOR_SHA
        if path == tmp_path/post.SUPERVISOR_TEST: return post.SUPERVISOR_TEST_SHA
        if path == authority_path: return "b"*64
        if path == execution_path: return "c"*64
        if path in (review_path, Path(post.__file__), post.TEST): return "a"*64
        return "d"*64
    monkeypatch.setattr(post, "digest", digest); monkeypatch.setattr(post, "canonical", Path)
    monkeypatch.setattr(post.shutil, "disk_usage", lambda p: SimpleNamespace(free=128*1024**2))
    child = FakeSSH(); owned = dict(pid=child.pid, ppid=os.getpid(), pgid=child.pid, session=child.pid, start_ticks=100)
    changed = [False]
    monkeypatch.setattr(post, "identity", lambda pid: dict(owned, start_ticks=101 if changed[0] else 100))
    monkeypatch.setattr(post, "session_members", lambda core: [])
    raw = post.encode(dict(status="FAIL_OR_UNKNOWN_RETAIN_ORIGINAL", complete_denominator=post.planned_ledger())) + b"\n"
    def communicate(payload, timeout):
        assert timeout == post.POST_SECONDS and post.decode(payload)["read_only_CPU_authorized"] is True
        if fault in ("timeout", "timeout_changed_generation"):
            changed[0] = fault == "timeout_changed_generation"
            raise subprocess.TimeoutExpired("fake-ssh", timeout, output=b"partial-json", stderr=b"timeout-tail")
        child.returncode = 7 if fault == "transport" else 0
        return (raw if fault != "oversize" else b"x" * post.MAX_LOCAL_BYTES), b"full-fake-stderr\n"
    child.communicate = communicate
    def popen(argv, **kwargs):
        assert argv[0] == "ssh" and kwargs["start_new_session"] is True
        assert "CUDA_VISIBLE_DEVICES=" in argv[-1] and "ConnectTimeout=10" in argv
        return child
    monkeypatch.setattr(post.subprocess, "Popen", popen)
    return out, child, raw


@pytest.mark.parametrize("fault", (None, "transport", "timeout", "timeout_changed_generation", "oversize"))
def test_fake_caller_retains_whole_or_unknown_denominator_and_received_tails(monkeypatch, tmp_path, fault):
    out, child, raw = fake_admitted_caller(monkeypatch, tmp_path, fault)
    if fault is None:
        assert post.main(["--review-sha256", "a"*64]) == 2
        report = json.loads((out/"ROOT_whole_final_evaluation_postcheck_v1.json").read_bytes())
        assert report["status"] == "FAIL_OR_UNKNOWN_RETAIN_ORIGINAL" and report["SSH_session_closed"] is True
        assert (out/"whole_final_evaluation_postcheck_v1.SSH.stdout").read_bytes() == raw
    else:
        with pytest.raises((RuntimeError, subprocess.TimeoutExpired)): post.main(["--review-sha256", "a"*64])
        failure = json.loads((out/"whole_final_evaluation_postcheck_v1.failure.json").read_bytes())
        assert failure["complete_denominator"]["full_denominator"] == 35315
        assert failure["complete_denominator"]["independently_verified_completed"] is None
        assert failure["automatic_retry"] is False and "UNKNOWN" in failure["remote_outcome"]
        if fault == "transport": assert (out/"whole_final_evaluation_postcheck_v1.SSH.stdout").read_bytes() == raw
        if fault.startswith("timeout"):
            assert (out/"whole_final_evaluation_postcheck_v1.SSH_timeout.stdout").read_bytes() == b"partial-json"
            assert (out/"whole_final_evaluation_postcheck_v1.SSH_timeout.stderr").read_bytes() == b"timeout-tail"
        if fault == "timeout_changed_generation": assert child.actions == [] and "error" in failure["SSH_cleanup"]
    assert sum(p.stat().st_size for p in out.iterdir()) <= post.MAX_LOCAL_BYTES


def test_actual_remote_program_structure_no_model_loss_optimizer_or_writes():
    program = post.remote_program(); tree = ast.parse(program); compile(tree, "source_only_postcheck_AST", "exec")
    assert program == post.remote_program() and "supervisor_v2" in program
    assert [n.name for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "endpoint_screens"] == []
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
    denied = {"evaluate", "forward", "__getitem__", "_targets", "load_joint_parent", "load_route_joint_research_npz",
        "load_reviewed_seed_endpoint", "spatial_inventory", "optimizer", "fit", "kill", "unlink", "mkdir", "write_bytes", "write_text"}
    assert not [n for n in calls if isinstance(n.func, ast.Attribute) and n.func.attr in denied]
    assert not [n for n in calls if isinstance(n.func, ast.Name) and n.func.id in ("open", "print") and n.func.id != "print"]
    opens = [n for n in calls if isinstance(n.func, ast.Attribute) and n.func.attr == "open"]
    assert all((isinstance(n.func.value, ast.Name) and n.func.value.id == "tarfile"
        and any(k.arg == "mode" and isinstance(k.value, ast.Constant) and k.value.value == "r:" for k in n.keywords))
        or (n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == "rb") for n in opens)
    imports = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
    assert not any((isinstance(n, ast.Import) and any(a.name.split(".")[0] in ("torch", "numpy", "onnx", "carla") for a in n.names)) for n in imports)
    gate = program.index("original complete supervisor guards required")
    assert gate < program.index("from scripts.e2e import train_temporal_intrinsic_curve_pair_20261002 as pair")
    assert "pair.load_train(data)" in program and "pair.load_evaluation_only(data,training,low,legacy)" in program
    assert "datasets[c].examples[i].token" in program  # Expected catalog derives from metadata, not final metric rows.
    assert "metric_aggregate_arithmetic_independently_recomputed=False" in program


def test_source_imports_stdlib_only_and_original_files_unchanged():
    tree = ast.parse(SOURCE.read_bytes())
    roots = set()
    for node in tree.body:
        if isinstance(node, ast.Import): roots.update(n.name.split(".")[0] for n in node.names)
        if isinstance(node, ast.ImportFrom): roots.add(node.module.split(".")[0])
    assert not roots & {"torch", "numpy", "portable_e2e", "scripts", "onnx", "carla"}
    assert hashlib.sha256((ROOT/post.SUPERVISOR).read_bytes()).hexdigest() == post.SUPERVISOR_SHA
    assert hashlib.sha256((ROOT/post.SUPERVISOR_TEST).read_bytes()).hexdigest() == post.SUPERVISOR_TEST_SHA
