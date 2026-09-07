"""HH_260906 - Test bounded personal GPU0 ownership without invoking SSH, CUDA or a subprocess."""

from datetime import datetime, timezone
from types import SimpleNamespace
import signal

import pytest

from scripts.e2e import run_owned_portable_decoder_probe as owner


def plan():
    return dict(schema="portable_e2e.owned_decoder_probe_plan.v1", gpu_uuid=owner.shared.GPU_UUID,
        input_relative=owner.INPUT, output_relative=owner.OUTPUT, finish_before_utc=owner.DEADLINE,
        cooperative_wall_seconds=3600, outer_wall_seconds=3900, causal_model_training=False,
        training_data_approved=False, automatic_retry=False, source_commit="a" * 40,
        probe_sha256="b" * 64, input_manifest_sha256="c" * 64, runner_sha256="d" * 64,
        shared_runner_sha256="e" * 64)


def test_exact_plan():
    assert owner.validate_plan(plan()) == plan()


@pytest.mark.parametrize("key,value", [("gpu_uuid", "0"), ("input_relative", "../dataset"),
    ("output_relative", "/tmp/probe"), ("cooperative_wall_seconds", True), ("outer_wall_seconds", 10000),
    ("causal_model_training", True), ("automatic_retry", True), ("training_data_approved", 0),
    ("source_commit", "HEAD"), ("runner_sha256", "x" * 64)])
def test_unreviewed_plan_rejected(key, value):
    p = plan(); p[key] = value
    with pytest.raises(RuntimeError):
        owner.validate_plan(p)


def test_deadline_requires_full_outer_and_cleanup_margin():
    assert owner.budget(owner.DEADLINE, now=datetime(2026, 9, 7, 23, 50, tzinfo=timezone.utc)) == 4200
    with pytest.raises(RuntimeError):
        owner.budget(owner.DEADLINE, now=datetime(2026, 9, 7, 23, 50, 1, tzinfo=timezone.utc))
    with pytest.raises(RuntimeError):
        owner.budget("2026-09-09T01:00:00Z")


def test_environment_hides_other_gpu_and_user_python(monkeypatch):
    monkeypatch.setenv("PYTHONPATH", "/untrusted")
    monkeypatch.setenv("PYTHONHOME", "/untrusted")
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "1")
    env = owner.environment()
    assert env["CUDA_VISIBLE_DEVICES"] == owner.shared.GPU_UUID
    assert all(env[k] == "4" for k in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS", "NUMEXPR_NUM_THREADS"))
    assert "PYTHONPATH" not in env and "PYTHONHOME" not in env


def test_finished_child_is_never_signaled(monkeypatch):
    monkeypatch.setattr(owner.os, "killpg", lambda *args: pytest.fail("finished process signaled"))
    owner.terminate_owned(None)
    owner.terminate_owned(SimpleNamespace(poll=lambda: 0))


def test_only_owned_session_is_signaled(monkeypatch):
    calls = []
    monkeypatch.setattr(owner.os, "killpg", lambda pid, sig: calls.append((pid, sig)))
    owner.terminate_owned(SimpleNamespace(pid=123456, poll=lambda: None, wait=lambda timeout: calls.append(timeout)))
    assert calls == [(123456, signal.SIGTERM), 90]


def test_owned_escalation_uses_same_session(monkeypatch):
    calls = []
    monkeypatch.setattr(owner.os, "killpg", lambda pid, sig: calls.append((pid, sig)))
    def wait(timeout):
        calls.append(timeout)
        if timeout == 90:
            raise owner.subprocess.TimeoutExpired("owned", timeout)
    owner.terminate_owned(SimpleNamespace(pid=123456, poll=lambda: None, wait=wait))
    assert calls == [(123456, signal.SIGTERM), 90, (123456, signal.SIGKILL), 5]


def test_repeated_signal_cannot_interrupt_owned_cleanup_and_handlers_restore(monkeypatch):
    calls = []
    previous = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    monkeypatch.setattr(owner.os, "killpg", lambda pid, sig: calls.append((pid, sig)))
    def wait(timeout):
        signal.raise_signal(signal.SIGTERM)
        signal.raise_signal(signal.SIGINT)
        calls.append(timeout)
        if timeout == 90:
            raise owner.subprocess.TimeoutExpired("owned", timeout)
    with owner.cleanup_signal_guard():
        owner.terminate_owned(SimpleNamespace(pid=123456, poll=lambda: None, wait=wait))
        assert all(signal.getsignal(s) == signal.SIG_IGN for s in previous)
    assert calls == [(123456, signal.SIGTERM), 90, (123456, signal.SIGKILL), 5]
    assert {s: signal.getsignal(s) for s in previous} == previous


def test_cleanup_guard_restores_handlers_after_evidence_failure():
    previous = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    with pytest.raises(OSError):
        with owner.cleanup_signal_guard():
            raise OSError("simulated evidence write error")
    assert {s: signal.getsignal(s) for s in previous} == previous
