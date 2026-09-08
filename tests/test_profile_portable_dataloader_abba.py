"""HH_260906 - Exercise ABBA provenance, CPU spawning and exact-parity failure handling with synthetic fixtures only."""

import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import profile_portable_training as base
from scripts.e2e import profile_portable_dataloader_abba as study


def plan():
    return dict(study.plan_contract(base), declared_at_utc="2026-09-08T20:00:00Z",
        source_sha256={name: "a" * 64 for name in base.SOURCE_PATHS}, worker_source_sha256="b" * 64)


def test_exact_abba_protocol_one_epoch():
    p = study.validate_plan(plan(), base)
    assert [r["num_workers"] for r in p["arms"]] == [0, 2, 2, 0]
    assert p["expected_metric_rows"] == 1148 and p["expected_total_sample_exposures"] == 4588
    assert p["train_config"]["max_steps"] == p["train_config"]["checkpoint_interval"] == 287
    for workers in (0, 2):
        cfg = study.train_config(base, workers)
        assert {k: v for k, v in cfg.items() if k != "num_workers"} == {k: v for k, v in p["train_config"].items() if k != "num_workers"}


@pytest.mark.parametrize("key,value", [("arms", list(reversed(plan()["arms"]))), ("expected_train_samples", True),
    ("expected_steps_per_fit", 16), ("multiprocessing_start_method", "fork"), ("worker_cuda_visible_devices", "0"),
    ("checkpoint_ignored_fields", ["optimizer_state_dict"]), ("source_commit", "a" * 40),
    ("worker_source_sha256", "bad"), ("frozen_helper_sha256", "c" * 64), ("model_config", "other")])
def test_changed_protocol_rejected(key, value):
    p = plan(); p[key] = value
    with pytest.raises(ValueError): study.validate_plan(p, base)


@pytest.mark.parametrize("key,value", [("batch_size", 8), ("seed", 42), ("verify_image_sha256", False),
    ("num_workers", 2), ("learning_rate", .01), ("max_steps", 288), ("checkpoint_interval", 16)])
def test_only_arm_worker_override_is_allowed(key, value):
    p = plan(); p["train_config"][key] = value
    with pytest.raises(ValueError): study.validate_plan(p, base)


def test_extra_plan_key_and_missing_source_rejected():
    p = plan(); p["retry_until_pass"] = True
    with pytest.raises(ValueError): study.validate_plan(p, base)
    p = plan(); p["source_sha256"].pop("portable_e2e/train.py")
    with pytest.raises(ValueError): study.validate_plan(p, base)


@pytest.mark.parametrize("workers", [True, False, 1, 3, 2.0, "2"])
def test_worker_count_is_strict_integer(workers):
    with pytest.raises(ValueError): study.train_config(base, workers)


def test_deadline_reserves_whole_operation():
    study.check_budget(plan(), base, 1290, datetime(2026, 9, 9, 0, 38, tzinfo=timezone.utc))
    for now in (datetime(2026, 9, 9, 0, 39, tzinfo=timezone.utc), datetime(2026, 9, 8, 19, 59, tzinfo=timezone.utc)):
        with pytest.raises(ValueError): study.check_budget(plan(), base, 1290, now)


def metrics():
    return [dict(global_step=i, epoch=0, samples_seen=min(i * 4, 1147), loss=1. / i, gradient_norm=1.,
        regression_loss=.5, candidate_score_loss=.1, domain_samples_seen={"carla": min(i * 4, 1147)},
        batch_domain_sample_counts={"carla": 3 if i == 287 else 4}) for i in range(1, 288)]


def write_metrics(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))


def test_all287_rows_include_final_three_sample_batch(tmp_path):
    path = tmp_path / "metrics.jsonl"; write_metrics(path, metrics())
    rows = study.read_metrics(path, base)
    assert rows[-1]["samples_seen"] == 1147 and rows[-1]["batch_domain_sample_counts"] == {"carla": 3}


@pytest.mark.parametrize("fault", ["first_missing", "last_missing", "duplicate", "bool_step", "bool_samples", "last_four",
    "epoch", "nan", "missing_loss", "truncated", "wrong_domain"])
def test_metrics_failure_denominators_never_hidden(tmp_path, fault):
    rows = metrics()
    if fault == "first_missing": rows.pop(0)
    elif fault == "last_missing": rows.pop()
    elif fault == "duplicate": rows[5] = rows[4]
    elif fault == "bool_step": rows[0]["global_step"] = True
    elif fault == "bool_samples": rows[0]["samples_seen"] = True
    elif fault == "last_four": rows[-1]["samples_seen"] = 1148
    elif fault == "epoch": rows[-1]["epoch"] = 1
    elif fault == "nan": rows[9]["gradient_norm"] = float("nan")
    elif fault == "missing_loss": rows[3].pop("loss")
    elif fault == "wrong_domain": rows[-1]["domain_samples_seen"] = {"real": 1147}
    path = tmp_path / "metrics.jsonl"; write_metrics(path, rows)
    if fault == "truncated": path.write_bytes(path.read_bytes().rstrip())
    with pytest.raises(ValueError): study.read_metrics(path, base)


def checkpoint(workers):
    return dict(created_at_utc="a", train_config=study.train_config(base, workers),
        dataset_fingerprint_sha256=base.TRAIN_SHA, corpus_fingerprint_sha256=base.CORPUS_SHA, training_split="train",
        state=dict(global_step=287, samples_seen=1147, epoch=0, next_batch_index=287),
        model_state_dict={"weight": torch.tensor([1., 2.])},
        optimizer_state_dict={"step": torch.tensor(287.), "exp_avg": torch.tensor([.3, .4])},
        torch_rng_state=torch.tensor([1, 2], dtype=torch.uint8), cuda_rng_state=torch.tensor([3, 4], dtype=torch.uint8),
        runtime_abi={"torch": "same"}, device_abi={"device_uuid": base.GPU_UUID},
        loss_config=base.LOSS_CONFIG.copy(), nested={"created_at_utc": "not ignored", "num_workers": "not ignored"})


def test_only_top_creation_and_declared_worker_config_normalized():
    a = checkpoint(0); b = checkpoint(2); b["created_at_utc"] = "b"
    av, bv = study.checkpoint_view(a, 0, base), study.checkpoint_view(b, 2, base)
    assert base.exact_difference(av, bv, torch, ignore_created=True) is None
    assert b["train_config"]["num_workers"] == 2
    b["nested"]["num_workers"] = "different"
    assert base.exact_difference(av, study.checkpoint_view(b, 2, base), torch, ignore_created=True)


@pytest.mark.parametrize("field", ["optimizer_state_dict", "model_state_dict", "torch_rng_state", "cuda_rng_state", "device_abi", "runtime_abi"])
def test_all_other_checkpoint_fields_exact(field):
    a = checkpoint(0); b = checkpoint(2); b.pop(field)
    assert base.exact_difference(study.checkpoint_view(a, 0, base), study.checkpoint_view(b, 2, base), torch, ignore_created=True)


@pytest.mark.parametrize("field,value", [("dataset_fingerprint_sha256", "a" * 64),
    ("corpus_fingerprint_sha256", "a" * 64), ("training_split", "test"), ("loss_config", {})])
def test_even_mutually_identical_wrong_checkpoint_scope_is_rejected(field, value):
    payload = checkpoint(0); payload[field] = value
    with pytest.raises(ValueError, match="dataset/split/loss"): study.checkpoint_view(payload, 0, base)


@pytest.mark.parametrize("fault", ["wrong_workers", "bool_workers", "lr", "epoch_incremented", "partial_samples"])
def test_checkpoint_schedule_and_save_boundary_not_relaxed(fault):
    p = checkpoint(2)
    if fault == "wrong_workers": p["train_config"]["num_workers"] = 0
    elif fault == "bool_workers": p["train_config"]["num_workers"] = False
    elif fault == "lr": p["train_config"]["learning_rate"] = .01
    elif fault == "epoch_incremented": p["state"]["epoch"] = 1
    elif fault == "partial_samples": p["state"]["samples_seen"] = 1144
    with pytest.raises(ValueError): study.checkpoint_view(p, 2, base)


def test_compare_keeps_all_metric_and_checkpoint_results_on_parity_failure(tmp_path):
    for arm, _ in study.ARMS: write_metrics(tmp_path / arm / "fit/metrics.jsonl", metrics())
    def read(path, device):
        assert str(device) == "cpu"
        arm = path.parents[2].name; p = checkpoint(dict(study.ARMS)[arm])
        if arm == study.ARMS[2][0]: p["cuda_rng_state"][0] = 8
        return p, "c" * 64
    report = {}
    with pytest.raises(ValueError, match="parity"):
        study.compare_outputs(tmp_path, base, torch, SimpleNamespace(_read_checkpoint_file=read), report)
    assert len(report["metrics_parity"]) == 3 and len(report["checkpoint_parity"]) == 4
    assert not report["checkpoint_parity"][2]["exact"]


def test_frozen_helper_import_rejects_unreviewed_bytes_before_exec(tmp_path):
    path = tmp_path / "helper.py"; path.write_text("raise RuntimeError('must not execute')")
    with pytest.raises(ValueError, match="SHA"): study.load_helper(path)


def test_spawn_cannot_be_selected_after_torch_import():
    with pytest.raises(ValueError, match="precede torch"): study.configure_spawn()


def test_spawn_rejects_existing_fork_context(monkeypatch):
    monkeypatch.delitem(sys.modules, "torch")
    monkeypatch.setattr(study.mp, "get_start_method", lambda **_: "fork")
    with pytest.raises(ValueError, match="non-spawn"): study.configure_spawn()


def test_actual_two_worker_spawn_is_cpu_only_and_keeps_order(tmp_path):
    # HH_260906 - This subprocess handles nine synthetic CPU integers only; no dataset, CUDA call or model fit occurs.
    directory = tmp_path / "worker_proofs"; directory.mkdir()
    source = Path(study.__file__).resolve(); launcher = tmp_path / "synthetic_spawn.py"
    launcher.write_text("\n".join([
        "import multiprocessing as mp, os, runpy, json",
        "if __name__ == '__mp_main__':",
        "    runpy.run_path(" + repr(str(source)) + ", run_name='__mp_main__')",
        "elif __name__ == '__main__':",
        "    mp.set_start_method('spawn')",
        "    os.environ['" + study.FIT_PID_ENV + "'] = str(os.getpid())",
        "    import torch",
        "    loader = torch.utils.data.DataLoader(torch.arange(9), batch_size=4, num_workers=2)",
        "    values = [int(x) for batch in loader for x in batch]",
        "    print(json.dumps({'values': values, 'pid': os.getpid(), 'group': os.getpgrp()}))",
    ]) + "\n")
    env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1", **{study.WORKER_PROOF_ENV: str(directory)})
    result = subprocess.run([sys.executable, str(launcher)], env=env, capture_output=True, text=True, timeout=45, check=True)
    result = json.loads(result.stdout)
    assert result["values"] == list(range(9))
    rows = study.validate_worker_proofs(directory, 2, result["pid"], result["group"], hashlib.sha256(source.read_bytes()).hexdigest(), base)
    assert len(rows) == 2


def test_only_explicit_owned_group_is_terminated(monkeypatch):
    calls = []; existing = {777}
    def killpg(pid, sig):
        calls.append((pid, sig)); assert pid == 777
        if pid not in existing: raise ProcessLookupError
        if sig == signal.SIGTERM: existing.remove(pid)
    monkeypatch.setattr(study.os, "killpg", killpg)
    p = SimpleNamespace(pid=777, wait=lambda **_: 0)
    result = study.cleanup_session(p)
    assert result["group_absent"] and result["signals"] == ["SIGTERM"]
    assert all(pid == 777 for pid, _ in calls)


def test_successful_exited_group_needs_no_signal(monkeypatch):
    def absent(*_): raise ProcessLookupError
    monkeypatch.setattr(study.os, "killpg", absent)
    result = study.cleanup_session(SimpleNamespace(pid=777))
    assert result["group_absent"] and result["signals"] == []


def test_launch_failure_retains_arm_and_does_not_retry(tmp_path, monkeypatch):
    args = argparse.Namespace(plan=tmp_path / "plan.json", output_dir=tmp_path)
    class Failed:
        pid = 777
        def wait(self, **_): return 2
    calls = []
    def popen(command, **kwargs):
        calls.append((command, kwargs)); return Failed()
    monkeypatch.setattr(study.subprocess, "Popen", popen)
    monkeypatch.setattr(study, "cleanup_session", lambda p: {"group_absent": True})
    report = {"arms": []}
    with pytest.raises(ValueError, match="fit failed"): study.launch_fit(args, plan(), study.ARMS[0][0], 9, base, report)
    assert len(calls) == 1 and len(report["arms"]) == 1
    assert calls[0][1]["start_new_session"] is True and calls[0][1]["pass_fds"] == (9,)
    assert report["arms"][0]["exit_code"] == 2


def test_timeout_always_runs_owned_cleanup(tmp_path, monkeypatch):
    args = argparse.Namespace(plan=tmp_path / "plan.json", output_dir=tmp_path); cleaned = []
    class Hung:
        pid = 888
        def wait(self, **_): raise subprocess.TimeoutExpired("owned", 240)
    monkeypatch.setattr(study.subprocess, "Popen", lambda *_, **__: Hung())
    monkeypatch.setattr(study, "cleanup_session", lambda p: cleaned.append(p.pid) or {"group_absent": True})
    with pytest.raises(subprocess.TimeoutExpired): study.launch_fit(args, plan(), study.ARMS[1][0], 9, base, {"arms": []})
    assert cleaned == [888]


def test_intermediate_cleanup_preserves_supervisor_alarm_after_fit(tmp_path, monkeypatch):
    args = argparse.Namespace(plan=tmp_path / "plan.json", output_dir=tmp_path)
    class Failed:
        pid = 777
        def wait(self, **_): return 2
    monkeypatch.setattr(study.subprocess, "Popen", lambda *_, **__: Failed())
    monkeypatch.setattr(study, "cleanup_session", lambda p: {"group_absent": True})
    previous = signal.getsignal(signal.SIGALRM)
    with base.bounded_signals(30):
        with pytest.raises(ValueError): study.launch_fit(args, plan(), study.ARMS[0][0], 9, base, {"arms": []})
        remaining, _ = signal.getitimer(signal.ITIMER_REAL)
        assert 0 < remaining <= 30
    assert signal.getsignal(signal.SIGALRM) == previous


def test_expired_cleanup_budget_does_not_restart_full_alarm(monkeypatch):
    times = iter([100., 102.])
    monkeypatch.setattr(study.time, "monotonic", lambda: next(times))
    with base.bounded_signals(1):
        with pytest.raises(ValueError, match="budget expired"):
            with study.intermediate_cleanup_signals(): pass
        assert signal.getitimer(signal.ITIMER_REAL)[0] == 0


def test_failed_finalization_still_writes_report_and_nonself_manifest(tmp_path):
    (tmp_path / "partial.json").write_text("{}\n")
    report = dict(status="ABBA_COMPLETE_EXACT_PARITY_NOT_PROMOTED", arms=[])
    study.finalize(tmp_path, report, {"mutated_source": lambda: False}, base, plan())
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["status"] == "FAILED_DIAGNOSTIC_NOT_PROMOTED"
    assert len(report["unperformed_arms"]) == 4 and report["postcheck_errors"]
    for row in (tmp_path / "SHA256SUMS").read_text().splitlines():
        value, name = row.split("  "); assert name != "SHA256SUMS"
        assert hashlib.sha256((tmp_path / name).read_bytes()).hexdigest() == value


def test_output_symlink_is_retained_as_failed_inventory(tmp_path):
    (tmp_path / "unsafe").symlink_to("/definitely/not/read")
    report = dict(status="STARTED", arms=[])
    study.finalize(tmp_path, report, {}, base, plan())
    assert report["status"] == "FAILED_DIAGNOSTIC_NOT_PROMOTED" and report["postcheck_errors"]


def test_cli_abbreviations_never_load_helper(monkeypatch):
    monkeypatch.setattr(study, "load_helper", lambda: pytest.fail("no helper import"))
    with pytest.raises(SystemExit): study.main(["--pla", "bad"])
