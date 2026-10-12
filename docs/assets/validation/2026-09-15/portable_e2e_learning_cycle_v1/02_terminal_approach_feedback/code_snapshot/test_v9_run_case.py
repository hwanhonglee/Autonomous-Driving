"""HH_260906 - CPU-only v9 one-case orchestration tests; distinct module name keeps the historical v7 suite in the same regression."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import ModuleType

import pytest


@pytest.fixture
def runner(tmp_path, monkeypatch):
    path = Path(__file__).with_name("run_case.py")
    spec = importlib.util.spec_from_file_location("v9_case_runner_under_test", path)
    module = importlib.util.module_from_spec(spec)
    old_path = sys.path[:]
    spec.loader.exec_module(module)
    sys.path[:] = old_path
    root = tmp_path / "repository"
    root.mkdir()
    base = root / "campaign"
    base.mkdir()
    monkeypatch.setattr(module, "ROOT", root)
    monkeypatch.setattr(module, "BASE", base)
    pins = {}
    for category in ("source_sha256", "input_sha256", "audit_math_sha256", "orchestration_sha256", "analysis_sha256"):
        source = root / (category + ".txt")
        source.write_text("frozen " + category)
        pins[category] = {source.name: module.sha(source)}
    plan = {**pins, "automatic_retry": False,
            "cases": [{"id": name, "arm": "coast" if index == 0 else "feedback",
                       "command": ["mock-owned-wrapper", "--case", name]}
                      for index, name in enumerate(module.CASES)]}
    plan_path = base / "plan.json"
    plan_path.write_text(json.dumps(plan))
    module.test_plan = plan
    module.test_plan_sha = module.sha(plan_path)
    module.test_calls = []

    def run_forbidden(*args, **kwargs):
        raise AssertionError("Every subprocess must be explicitly mocked")

    monkeypatch.setattr(module.subprocess, "run", run_forbidden)
    monkeypatch.setattr(sys, "argv", [str(path), "--case", module.CASES[0],
                                      "--plan-sha256", module.test_plan_sha])
    return module


def select_case(runner, monkeypatch, index):
    monkeypatch.setattr(sys, "argv", ["run_case.py", "--case", runner.CASES[index],
                                      "--plan-sha256", runner.test_plan_sha])


def install_children(runner, monkeypatch, *, readiness_code=0, wrapper_code=0,
                     after_readiness=None, wrapper_output=False):
    def mocked_run(command, **kwargs):
        runner.test_calls.append(list(command))
        assert kwargs["cwd"] == runner.ROOT
        if "scripts.e2e.probe_carla_port_readiness" in command:
            assert command[command.index("--port") + 1] == "2100"
            output = Path(command[command.index("--output") + 1])
            output.write_text(json.dumps({"status": "MOCK_READY" if readiness_code == 0 else "MOCK_BLOCKED"}))
            if after_readiness:
                after_readiness()
            return subprocess.CompletedProcess(command, readiness_code)
        assert command == runner.test_plan["cases"][0]["command"]
        kwargs["stdout"].write(b"mock wrapper log\n")
        if wrapper_output:
            case_root = runner.BASE / runner.CASES[0]
            case_root.mkdir()
            (case_root / "owner_result.json").write_text('{"mock":true}')
        return subprocess.CompletedProcess(command, wrapper_code)

    monkeypatch.setattr(runner.subprocess, "run", mocked_run)


def result(runner, index=0):
    return json.loads((runner.BASE / "execution" / runner.CASES[index] / "result.json").read_text())


def test_one_case_one_wrapper_and_no_retry(runner, monkeypatch):
    install_children(runner, monkeypatch, wrapper_output=True)
    assert runner.main() == 0
    assert len(runner.test_calls) == 2
    report = result(runner)
    assert report["status"] == "WRAPPER_COMPLETED"
    assert report["wrapper_attempt_count"] == 1
    assert report["automatic_retry"] is False
    assert report["training_data_approved"] is False
    assert report["owner_result_sha256"] == runner.sha(runner.BASE / runner.CASES[0] / "owner_result.json")
    preserved = (runner.BASE / "execution" / runner.CASES[0] / "result.json").read_bytes()
    with pytest.raises(ValueError, match="already exists"):
        runner.main()
    assert len(runner.test_calls) == 2
    assert (runner.BASE / "execution" / runner.CASES[0] / "result.json").read_bytes() == preserved
    assert not any((runner.BASE / name).exists() for name in runner.CASES[1:])


@pytest.mark.parametrize("category", ["source_sha256", "input_sha256", "audit_math_sha256", "orchestration_sha256", "analysis_sha256", "plan"])
def test_changed_pins_after_readiness_never_invoke_wrapper(runner, monkeypatch, category):
    target = runner.BASE / "plan.json" if category == "plan" else runner.ROOT / next(iter(runner.test_plan[category]))
    install_children(runner, monkeypatch, after_readiness=lambda: target.write_text("changed"))
    with pytest.raises(ValueError):
        runner.main()
    assert len(runner.test_calls) == 1
    assert result(runner)["status"] == "PIN_MISMATCH_NOT_ATTEMPTED"
    assert result(runner)["wrapper_attempt_count"] == 0
    assert not (runner.BASE / "execution" / runner.CASES[0] / "wrapper_started.json").exists()
    assert not (runner.BASE / runner.CASES[0]).exists()


def test_readiness_failure_retained_and_cannot_retry(runner, monkeypatch):
    install_children(runner, monkeypatch, readiness_code=7)
    assert runner.main() == 7
    assert result(runner)["status"] == "READINESS_BLOCKED_NOT_ATTEMPTED"
    assert result(runner)["wrapper_attempt_count"] == 0
    preserved = (runner.BASE / "execution" / runner.CASES[0] / "result.json").read_bytes()
    with pytest.raises(FileExistsError):
        runner.main()
    assert len(runner.test_calls) == 1
    assert (runner.BASE / "execution" / runner.CASES[0] / "result.json").read_bytes() == preserved


@pytest.mark.parametrize("prior_index", [0])
def test_any_prior_gate_failure_prevents_execution(runner, monkeypatch, prior_index):
    select_case(runner, monkeypatch, 1)
    audited = []
    fake = ModuleType("scripts.e2e.summarize_carla_terminal_approach")

    def summarize_one(path, case, plan):
        assert path == runner.BASE / case["id"] and plan == runner.test_plan
        audited.append(case["id"])
        return ({"execution_and_scalar_pixel_protocol_clear": case["id"] != runner.CASES[prior_index],
                 "status": "COMPLETE", "all_original_future_metrics_clear": False}, None, None, None)

    fake.summarize_one = summarize_one
    monkeypatch.setitem(sys.modules, fake.__name__, fake)
    with pytest.raises(ValueError, match="prior case failed"):
        runner.main()
    assert audited == list(runner.CASES[:prior_index + 1])
    assert not (runner.BASE / "execution").exists()
    assert not runner.test_calls


def test_later_case_output_prevents_out_of_order_execution(runner):
    (runner.BASE / runner.CASES[1]).mkdir()
    with pytest.raises(ValueError, match="out of order"):
        runner.main()
    assert not (runner.BASE / "execution").exists()


@pytest.mark.parametrize("location", ["input_parent", "execution_parent", "case_output"])
def test_symlink_redirection_rejected_before_child(runner, monkeypatch, tmp_path, location):
    outside = tmp_path / "outside"
    outside.mkdir()
    if location == "input_parent":
        original = runner.ROOT / next(iter(runner.test_plan["input_sha256"]))
        redirected = outside / original.name
        redirected.write_bytes(original.read_bytes())
        (runner.ROOT / "alias").symlink_to(outside, target_is_directory=True)
        runner.test_plan["input_sha256"] = {"alias/" + original.name: runner.sha(redirected)}
        (runner.BASE / "plan.json").write_text(json.dumps(runner.test_plan))
        runner.test_plan_sha = runner.sha(runner.BASE / "plan.json")
        select_case(runner, monkeypatch, 0)
    elif location == "execution_parent":
        (runner.BASE / "execution").symlink_to(outside, target_is_directory=True)
    else:
        (runner.BASE / runner.CASES[0]).symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"):
        runner.main()
    assert not runner.test_calls
    assert sorted(item.name for item in outside.iterdir()) == (["input_sha256.txt"] if location == "input_parent" else [])


@pytest.mark.parametrize("stage", ["readiness", "wrapper"])
@pytest.mark.parametrize("error", [OSError("mock invocation error"), KeyboardInterrupt("mock interrupted wait")])
def test_invocation_failure_receipt_never_claims_cleanup(runner, monkeypatch, stage, error):
    def mocked_run(command, **kwargs):
        runner.test_calls.append(command)
        if "scripts.e2e.probe_carla_port_readiness" in command and stage == "wrapper":
            Path(command[command.index("--output") + 1]).write_text('{"mock":true}')
            return subprocess.CompletedProcess(command, 0)
        raise error

    monkeypatch.setattr(runner.subprocess, "run", mocked_run)
    with pytest.raises(type(error)):
        runner.main()
    report = result(runner)
    assert report["status"] == ("READINESS_INVOCATION_FAILED_NOT_ATTEMPTED" if stage == "readiness"
                                 else "WRAPPER_INVOCATION_OR_WAIT_FAILED_STATUS_UNCONFIRMED")
    assert report["wrapper_attempt_count"] == (0 if stage == "readiness" else 1)
    assert not any("cleanup" in name or "released" in name for name in report)
    assert report["automatic_retry"] is False


@pytest.mark.parametrize("has_output,expected", [(False, "PRELAUNCH_BLOCKED_NOT_CAPTURED"), (True, "WRAPPER_FAILED_WITH_OUTPUT")])
def test_wrapper_failure_counts_attempt_and_preserves_output_scope(runner, monkeypatch, has_output, expected):
    install_children(runner, monkeypatch, wrapper_code=12, wrapper_output=has_output)
    assert runner.main() == 12
    assert result(runner)["status"] == expected
    assert result(runner)["wrapper_attempt_count"] == 1
    assert len(runner.test_calls) == 2
