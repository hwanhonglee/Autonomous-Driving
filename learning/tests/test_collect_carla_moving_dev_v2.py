"""HH_260906 - Check isolated DEV collector dispatch without any simulator access."""
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from scripts import e2e
from scripts.e2e import collect_carla_moving_dev_v2 as driver


@pytest.fixture
def helper(monkeypatch):
    # HH_260906 - Stub only the new contract; keep the real collector parser and its fail-closed checks.
    module = ModuleType("scripts.e2e.carla_moving_capture_dev_v2")
    calls = []

    def configure(args):
        calls.append(args)
        return {"schema": "fixture.dev.contract.v2"}

    module.configuration_from_args = configure
    monkeypatch.setitem(sys.modules, module.__name__, module)
    monkeypatch.setattr(e2e, "carla_moving_capture_dev_v2", module, raising=False)
    return module, calls


def argv():
    return ["unused_output", "unused_route.json", "--moving-task-plan", "unused_plan.json",
            "--moving-task-plan-sha256", "a" * 64]


@pytest.mark.parametrize("flags", [[], ["--moving-task-plan", "unused_plan.json"],
                                  ["--moving-task-plan-sha256", "a" * 64]])
def test_missing_declaration_rejected_and_original_hook_restored(helper, flags):
    _, calls = helper
    previous = driver.base.moving_capture_module
    with pytest.raises(SystemExit) as error:
        driver.parse_args(["unused_output", "unused_route.json", *flags])
    assert error.value.code == 2
    assert not calls
    assert driver.base.moving_capture_module is previous
    assert previous(SimpleNamespace()) is None


def test_parser_delegates_arguments_and_uses_new_contract(helper):
    module, calls = helper
    previous = driver.base.moving_capture_module
    args = driver.parse_args(argv() + ["--seed", "201", "--port", "2100"])
    assert calls == [args]
    assert args.seed == 201 and args.port == 2100
    assert args.moving_task_plan == Path("unused_plan.json")
    assert driver.base.moving_capture_module is previous
    assert driver.moving_capture_module(args) is module
    assert calls == [args, args]
    assert driver.base.moving_capture_module is previous


def test_main_binds_base_parser_and_collection_and_restores_hook(helper, monkeypatch):
    module, calls = helper
    previous = driver.base.moving_capture_module
    received = []

    def main(options):
        assert driver.base.moving_capture_module is driver.moving_capture_module
        parsed = driver.base.parse_args(options)
        assert calls == [parsed]
        assert driver.base.moving_capture_module(parsed) is module
        received.append(options)
        return 17

    monkeypatch.setattr(driver.base, "main", main)
    options = argv()
    assert driver.main(options) == 17
    assert received == [options]
    assert len(calls) == 2
    assert driver.base.moving_capture_module is previous


@pytest.mark.parametrize("exception", [RuntimeError("failed"), SystemExit(2), KeyboardInterrupt()])
def test_main_restores_hook_after_failures(helper, monkeypatch, exception):
    previous = driver.base.moving_capture_module

    def main(_options):
        assert driver.base.moving_capture_module is driver.moving_capture_module
        raise exception

    monkeypatch.setattr(driver.base, "main", main)
    with pytest.raises(type(exception)):
        driver.main(argv())
    assert driver.base.moving_capture_module is previous


def test_invalid_contract_stops_before_base_run(helper, monkeypatch):
    module, _ = helper
    previous = driver.base.moving_capture_module
    observed = []

    def reject(_args):
        raise ValueError("fixture source closure differs")

    def run(_args):
        observed.append("unexpected collection")
        raise AssertionError("strict preflight must precede collection")

    monkeypatch.setattr(module, "configuration_from_args", reject)
    monkeypatch.setattr(driver.base, "run", run)
    with pytest.raises(SystemExit) as error:
        driver.main(argv())
    assert error.value.code == 2
    assert observed == []
    assert driver.base.moving_capture_module is previous


def test_original_argument_parser_stays_strict_and_unchanged(helper):
    previous = driver.base.moving_capture_module
    with pytest.raises(SystemExit) as error:
        driver.parse_args(argv() + ["--moving-task-p", "other.json"])
    assert error.value.code == 2
    assert driver.base.moving_capture_module is previous
    assert driver.base.parse_args(["unused_output", "unused_route.json"]).moving_task_plan is None
