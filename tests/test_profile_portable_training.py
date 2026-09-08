"""HH_260906 - Validate the bounded profiler protocol with synthetic CPU fixtures, never a GPU or real dataset."""

import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import profile_portable_training as study


def plan():
    return dict(schema=study.PLAN_SCHEMA, diagnostic_id="hh260909-training-profiler-16step-pair-v1",
        source_commit=study.SOURCE_COMMIT, gpu_uuid=study.GPU_UUID, dataset=study.DATASET,
        dataset_manifest_sha256=study.MANIFEST_SHA, corpus_fingerprint_sha256=study.CORPUS_SHA,
        train_fingerprint_sha256=study.TRAIN_SHA, model_config=study.MODEL_CONFIG, model_config_sha256=study.MODEL_SHA,
        expected_train_samples=1147, expected_train_episodes=3, train_config=copy.deepcopy(study.TRAIN_CONFIG),
        loss_config=copy.deepcopy(study.LOSS_CONFIG), arms=list(study.ARMS), profiler_options=study.PROFILE_OPTIONS.copy(),
        checkpoint_ignored_fields=["created_at_utc"], finish_before_utc="2026-09-09T01:00:00Z",
        external_wall_timeout_seconds=360, internal_wall_timeout_seconds=330, safety_reserve_seconds=60,
        approval=study.DENIALS.copy(), declared_at_utc="2026-09-08T19:00:00Z",
        source_sha256={name: "a" * 64 for name in study.SOURCE_PATHS}, profiler_source_sha256="b" * 64)


def test_reviewed_plan_and_deadline():
    assert study.validate_plan(plan()) == plan()
    study.verify_budget(plan(), datetime(2026, 9, 9, 0, 53, tzinfo=timezone.utc))


@pytest.mark.parametrize("field,value", [("source_commit", "a" * 40), ("expected_train_samples", True),
    ("dataset", "other"), ("arms", list(reversed(study.ARMS))), ("checkpoint_ignored_fields", ["created_at_utc", "optimizer_state_dict"]),
    ("external_wall_timeout_seconds", 600), ("gpu_uuid", "GPU-other"), ("profiler_source_sha256", "bad")])
def test_unreviewed_plan_rejected(field, value):
    data = plan(); data[field] = value
    with pytest.raises(ValueError): study.validate_plan(data)


@pytest.mark.parametrize("field,value", [("num_workers", 2), ("max_steps", 15), ("batch_size", 8),
    ("verify_image_sha256", False), ("checkpoint_interval", 8), ("seed", 20260904)])
def test_training_options_cannot_change(field, value):
    data = plan(); data["train_config"][field] = value
    with pytest.raises(ValueError): study.validate_plan(data)


@pytest.mark.parametrize("when", [datetime(2026, 9, 8, 18, 59, tzinfo=timezone.utc), datetime(2026, 9, 9, 0, 54, tzinfo=timezone.utc)])
def test_future_plan_and_insufficient_cleanup_budget_fail(when):
    with pytest.raises(ValueError): study.verify_budget(plan(), when)


def test_source_inventory_must_be_exact():
    data = plan(); data["source_sha256"].pop("portable_e2e/train.py")
    with pytest.raises(ValueError): study.validate_plan(data)


def payload():
    return dict(created_at_utc="2026-09-08T19:00:00Z", model_state_dict={"weights": torch.tensor([1., 2.])},
        optimizer_state_dict={"state": {0: {"exp_avg": torch.tensor([.2, .3]), "step": torch.tensor(16.)}}},
        torch_rng_state=torch.tensor([1, 2, 3], dtype=torch.uint8), cuda_rng_state=torch.tensor([4, 5], dtype=torch.uint8),
        device_abi={"device_uuid": study.GPU_UUID}, state={"global_step": 16, "samples_seen": 64})


def test_created_timestamp_only_can_differ():
    a = payload(); b = copy.deepcopy(a); b["created_at_utc"] = "2026-09-08T19:00:01Z"
    assert study.exact_difference(a, b, torch, ignore_created=True) is None
    assert study.exact_difference(a, b, torch) is not None


@pytest.mark.parametrize("key", ["model_state_dict", "optimizer_state_dict", "torch_rng_state", "cuda_rng_state", "device_abi", "state"])
def test_no_hidden_checkpoint_field_can_be_ignored(key):
    a = payload(); b = copy.deepcopy(a); b.pop(key)
    assert study.exact_difference(a, b, torch, ignore_created=True)


def test_tensor_dtype_signed_zero_and_nonfinite_are_not_equal():
    assert study.exact_difference(torch.tensor([0.]), torch.tensor([-0.]), torch)
    assert study.exact_difference(torch.tensor([1.]), torch.tensor([1.], dtype=torch.float64), torch)
    assert study.exact_difference(torch.tensor([float("nan")]), torch.tensor([float("nan")]), torch)
    assert study.exact_difference(0., -0., torch)
    assert study.exact_difference(True, 1, torch)


def test_nested_creation_field_is_not_ignored():
    a = dict(created_at_utc="a", nested=dict(created_at_utc="a"))
    b = dict(created_at_utc="b", nested=dict(created_at_utc="b"))
    assert study.exact_difference(a, b, torch, ignore_created=True) == "$/nested/created_at_utc: scalar differs"
    assert study.exact_difference({}, {}, torch, ignore_created=True)


def metrics():
    return [dict(global_step=i, samples_seen=i * 4, loss=1. / i, gradient_norm=1., regression_loss=.5,
        candidate_score_loss=.1) for i in range(1, 17)]


def write_metrics(path, rows):
    with path.open("x") as stream:
        stream.write("".join(json.dumps(row) + "\n" for row in rows))


@pytest.mark.parametrize("change", ["short", "bad_step", "wrong_exposure", "nonfinite", "bool_step", "truncated"])
def test_metrics_never_drop_first_or_failed_steps(tmp_path, change):
    rows = metrics()
    if change == "short": rows.pop(0)
    elif change == "bad_step": rows[0]["global_step"] = 0
    elif change == "wrong_exposure": rows[-1]["samples_seen"] = 63
    elif change == "nonfinite": rows[2]["loss"] = float("nan")
    elif change == "bool_step": rows[0]["global_step"] = True
    path = tmp_path / "metrics.jsonl"; write_metrics(path, rows)
    if change == "truncated": path.write_bytes(path.read_bytes().rstrip())
    with pytest.raises(ValueError): study.read_metrics(path)


def test_output_existing_and_symlink_alias_rejected(tmp_path):
    workspace = tmp_path / "workspace"; data = tmp_path / "dataset"; data.mkdir()
    diagnostics = workspace / "runs/diagnostics"; diagnostics.mkdir(parents=True)
    target = diagnostics / "fresh"
    assert study.checked_output(target, data, workspace) == target
    target.mkdir()
    with pytest.raises(ValueError): study.checked_output(target, data, workspace)
    (diagnostics / "alias").symlink_to(data, target_is_directory=True)
    with pytest.raises(ValueError): study.checked_output(diagnostics / "alias/new", data, workspace)
    with pytest.raises(ValueError): study.checked_output(data / "new", data, workspace)


def test_gpu_idle_queries_only_physical_zero_and_rejects_foreign_process(monkeypatch, tmp_path):
    calls = []
    def query(command, **kwargs):
        calls.append(command)
        return "0, " + study.GPU_UUID if "--query-gpu=index,uuid" in command else study.GPU_UUID + ", 1234"
    monkeypatch.setattr(study.subprocess, "check_output", query)
    with pytest.raises(ValueError, match="occupied"): study.gpu_idle(tmp_path)
    assert all(command[1:3] == ["-i", "0"] for command in calls)
    assert len(calls) == 2


def test_actual_torch_uuid_uses_exact_prefix_free_canonical_value():
    actual = study.GPU_UUID.removeprefix("GPU-")
    assert study.verify_torch_uuid(SimpleNamespace(uuid=actual)) == actual
    for invalid in (study.GPU_UUID, actual.upper(), "other-" + actual, None, actual + "extra"):
        with pytest.raises(ValueError, match="UUID"): study.verify_torch_uuid(SimpleNamespace(uuid=invalid))


def test_source_snapshot_is_offline_and_binds_every_exact_file(monkeypatch, tmp_path):
    data = plan(); original = b"same reviewed source\n"
    for name in study.SOURCE_PATHS:
        path = tmp_path / name; path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(original)
        data["source_sha256"][name] = study.sha(original)
    script = tmp_path / "profiler.py"; script.write_text("# same independent profiler\n")
    data["profiler_source_sha256"] = study.digest(script)
    monkeypatch.setattr(study, "MODEL_SHA", study.sha(original))
    calls = []
    def git(command, **kwargs):
        calls.append(command)
        assert command[:4] == ["git", "-c", "protocol.allow=never", "show"]
        assert kwargs["env"]["GIT_NO_LAZY_FETCH"] == "1" and kwargs["env"]["GIT_ALLOW_PROTOCOL"] == ""
        assert kwargs["env"]["GIT_TERMINAL_PROMPT"] == "0"
        return original
    monkeypatch.setattr(study.subprocess, "check_output", git)
    assert len(study.source_identity(data, tmp_path, script)) == len(study.SOURCE_PATHS) + 1
    assert len(calls) == len(study.SOURCE_PATHS)
    (tmp_path / "portable_e2e/train.py").write_text("changed source\n")
    with pytest.raises(ValueError, match="source mismatch"): study.source_identity(data, tmp_path, script)


def test_lease_rejects_busy_without_waiting_or_stopping(tmp_path):
    parent = tmp_path / "runs/campaigns"; parent.mkdir(parents=True)
    with study.gpu_lease(tmp_path):
        with pytest.raises(BlockingIOError):
            with study.gpu_lease(tmp_path): pytest.fail("must not acquire another lease")
    with study.gpu_lease(tmp_path): pass


def fixture_pair(tmp_path, *, mismatch=False, kernels=True, fail_at=None):
    calls = []; contexts = []
    class Profiler:
        def __enter__(self): contexts.append("enter"); return self
        def __exit__(self, *exc): contexts.append("exit"); return False
        def key_averages(self):
            return [SimpleNamespace(key="aten::test", count=16, device_type="CPU", cpu_time_total=12.,
                self_cpu_time_total=10., device_time_total=8., self_device_time_total=7.)]
        def export_chrome_trace(self, path):
            Path(path).write_text(json.dumps(dict(traceEvents=[dict(cat="kernel")] if kernels else [])))
    def profile(**kwargs):
        assert kwargs == dict(activities=["CPU", "CUDA"], **study.PROFILE_OPTIONS)
        return Profiler()
    facade = SimpleNamespace(Tensor=torch.Tensor, strided=torch.strided, uint8=torch.uint8, equal=torch.equal,
        isfinite=torch.isfinite, device=torch.device, cuda=SimpleNamespace(synchronize=lambda device: calls.append(("sync", device))),
        profiler=SimpleNamespace(ProfilerActivity=SimpleNamespace(CPU="CPU", CUDA="CUDA"), profile=profile,
            supported_activities=lambda: {"CPU", "CUDA"}))
    def train_model(**kwargs):
        arm = kwargs["run_dir"].name; calls.append(("train", arm, dict(kwargs)))
        kwargs["run_dir"].mkdir()
        rows = metrics()
        if mismatch and arm == study.ARMS[1]: rows[2]["loss"] += .01
        if fail_at == arm:
            write_metrics(kwargs["run_dir"] / "metrics.jsonl", rows[:3])
            raise RuntimeError("retained partial trainer failure")
        write_metrics(kwargs["run_dir"] / "metrics.jsonl", rows)
        return dict(status="TRAINING_TARGET_REACHED", state={"global_step": 16, "samples_seen": 64})
    def read_checkpoint(path, device):
        assert str(device) == "cpu"
        data = payload(); data["created_at_utc"] = path.parts[-3]
        return data, "a" * 64
    trainer = SimpleNamespace(train_model=train_model, _read_checkpoint_file=read_checkpoint)
    report = dict(arms=[])
    kwargs = dict(output=tmp_path, dataset=object(), model_config=object(), torch=facade,
        trainer=trainer, loss_config=object(), train_config=object(), report=report)
    return kwargs, calls, contexts


def test_two_real_trainer_calls_same_inputs_and_scope_with_complete_parity(tmp_path):
    kwargs, calls, contexts = fixture_pair(tmp_path)
    study.train_pair(**kwargs)
    train_calls = [row for row in calls if row[0] == "train"]
    assert [row[1] for row in train_calls] == list(study.ARMS)
    first, second = (row[2].copy() for row in train_calls)
    first.pop("run_dir"); second.pop("run_dir")
    assert first == second
    assert first["resume"] is False and first["training_split"] == "train"
    assert contexts == ["enter", "exit"]
    assert kwargs["report"]["metrics_parity"] == dict(rows_per_arm=16, total_retained_rows=32, exact=True, first_difference=None)
    assert kwargs["report"]["checkpoint_parity"]["exact"] is True
    assert kwargs["report"]["arms"][1]["cuda_kernel_event_count"] == 1


def test_profiler_metric_difference_retained_and_fails(tmp_path):
    kwargs, _, _ = fixture_pair(tmp_path, mismatch=True)
    with pytest.raises(ValueError, match="changed metrics"): study.train_pair(**kwargs)
    assert kwargs["report"]["metrics_parity"]["exact"] is False
    assert (tmp_path / "cpu_cuda_trace.json").exists()


def test_missing_cuda_events_is_not_claimed_full_profile(tmp_path):
    kwargs, _, _ = fixture_pair(tmp_path, kernels=False)
    with pytest.raises(ValueError, match="CUDA kernel events"): study.train_pair(**kwargs)
    assert len(kwargs["report"]["arms"]) == 2


@pytest.mark.parametrize("arm", study.ARMS)
def test_training_failure_keeps_partial_rows_and_no_extra_attempt(tmp_path, arm):
    kwargs, calls, _ = fixture_pair(tmp_path, fail_at=arm)
    with pytest.raises(RuntimeError, match="partial trainer"): study.train_pair(**kwargs)
    assert len((tmp_path / arm / "metrics.jsonl").read_text().splitlines()) == 3
    assert len([row for row in calls if row[0] == "train"]) == 1 + (arm == study.ARMS[1])


def test_cli_abbreviation_is_rejected():
    with pytest.raises(SystemExit): study.main(["--pl", "none", "--output-dir", "none"])


def test_json_duplicate_keys_and_nonfinite_rejected(tmp_path):
    path = tmp_path / "plan.json"; path.write_text('{"x":1,"x":2}')
    with pytest.raises(ValueError): study.json_read(path)
    path.write_text('{"x":NaN}')
    with pytest.raises(ValueError): study.json_read(path)


@pytest.mark.parametrize("failure", ["mutation", "removed", "exception", "unsafe_output"])
def test_postcheck_failure_preserves_report_and_no_manifest_selfhash(tmp_path, failure):
    partial = tmp_path / "partial.json"; partial.write_text('{"completed_steps":3}\n')
    report = dict(arms=[dict(arm=study.ARMS[0], status="STARTED")], status="FAILED_DIAGNOSTIC_NOT_PROMOTED")
    original = study.digest(partial)
    checks = {"input": lambda: study.digest(partial) == original}
    if failure == "mutation": partial.write_text('{"completed_steps":4}\n')
    elif failure == "removed": partial.unlink()
    elif failure == "exception": checks = {"input": lambda: (_ for _ in ()).throw(RuntimeError("input vanished"))}
    else: (tmp_path / "unsafe.json").symlink_to(partial)
    study.finalize_output(tmp_path, report, checks, "2099-01-01T00:00:00Z")
    saved = json.loads((tmp_path / "report.json").read_text())
    assert saved["status"] == "FAILED_DIAGNOSTIC_NOT_PROMOTED" and saved["postcheck_errors"]
    assert saved["unperformed_arms"] == [study.ARMS[1]]
    for line in (tmp_path / "SHA256SUMS").read_text().splitlines():
        expected, name = line.split("  ")
        assert name != "SHA256SUMS" and study.digest(tmp_path / name) == expected


def test_finalization_after_deadline_cannot_be_complete(tmp_path):
    report = dict(arms=[], status="PROFILE_COMPLETE_EXACT_PARITY_NOT_PROMOTED")
    study.finalize_output(tmp_path, report, {}, "2000-01-01T00:00:00Z")
    assert report["status"] == "FAILED_DIAGNOSTIC_NOT_PROMOTED" and report["deadline_met"] is False


def test_repeated_real_signals_cannot_interrupt_final_persistence(tmp_path):
    report = dict(arms=[], status="FAILED_DIAGNOSTIC_NOT_PROMOTED")
    def repeated_signals():
        for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGALRM):
            signal.raise_signal(sig)
        return True
    before = {sig: signal.getsignal(sig) for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGALRM)}
    with study.bounded_signals(10):
        with pytest.raises(TimeoutError): signal.raise_signal(signal.SIGTERM)
        with study.finalization_signals():
            study.finalize_output(tmp_path, report, {"repeated_signals": repeated_signals}, "2099-01-01T00:00:00Z")
        with pytest.raises(TimeoutError): signal.raise_signal(signal.SIGINT)
    assert (tmp_path / "report.json").is_file() and (tmp_path / "SHA256SUMS").is_file()
    assert all(signal.getsignal(sig) == handler for sig, handler in before.items())
