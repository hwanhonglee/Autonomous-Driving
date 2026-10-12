"""HH_261002 - Source identity, actual provider namespace and strict scope tests; no capture or data admission."""
import ast
import copy
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import town01_low_speed_left45_task_v2_20261002 as task
from scripts.e2e import collect_town01_low_speed_left45_v2_20261002 as worker
from scripts.e2e import run_owned_town01_low_speed_left45_v2_20261002 as owner


def test_all_three_frozen_sources_unmodified_and_inverse_identity_ast_verified():
    for provider in (task, worker, owner):
        assert task.sha(Path(provider.baseline.__file__)) == provider.FROZEN_V1_SHA
        assert isinstance(provider.identity_ast(), ast.Module)


def test_public_v2_exact_same_physical_membership_seed_tail_and_denial():
    before = task.baseline.fixed_contract("v1", "pilot_00")
    after = task.fixed_contract("v2", "pilot_00")
    assert after["schema"] == "carla.actual_town01_left45_teacher.20261002.v2"
    assert after["run_id"] == "v2"
    assert set(before) == set(after)
    for key in before:
        if key not in ("schema", "run_id", "output", "declaration_path"):
            assert before[key] == after[key], key
    assert after["fixed_arguments"]["seed"] == after["seed"] == 2026102500
    assert after["expected_fixed_anchor_count"] == 347 and after["true_future_point_count"] == 22208
    assert after["diagnostic_tail_policy"]["native_ticks"] == 130
    assert after["future_qualification_required"] is True
    assert "training_data_approved" not in task.pending_scope()


@pytest.mark.parametrize("run", ["v1", "v3", "../v2", ""])
def test_v1_or_unknown_paths_never_reopened_or_admitted(run):
    with pytest.raises(ValueError): task.fixed_contract(run, "pilot_00")
    with pytest.raises(ValueError): task.paths(run)


def test_private_current_collector_and_owner_providers_not_stale_v1():
    assert worker.task is task and owner.task is task and owner.worker_module is worker
    assert worker.parse_args.__globals__["task"] is task
    assert worker._teacher["scope"].__wrapped__.__globals__["task"] is task
    assert worker._teacher["scope"].__wrapped__.__globals__["v2"].CombinedControl is worker.Launch
    assert owner.capture_argv.__globals__["worker_module"] is worker
    assert owner._originals["diagnostic_report"].__globals__["task"] is task
    assert owner._originals["build_plan"].__globals__["execution_contract"] is owner._original_contract
    assert worker.Launch.apply.__globals__["task"] is task
    assert worker.baseline.task is task.baseline
    assert owner.baseline.worker_module is worker.baseline


def test_public_capture_argv_new_paths_only_no_control_parameter_changes():
    before = owner.baseline.capture_argv("v1", "pilot_00", "a" * 64)
    after = owner.capture_argv("v2", "pilot_00", "a" * 64)
    assert len(before) == len(after)
    changes = [(a, b) for a, b in zip(before, after) if a != b]
    assert changes and all((a, b) == ("v1", "v2") or a.replace("teacher_v1", "teacher_v2") == b for a, b in changes)
    assert after[after.index("--seed") + 1] == "2026102500"
    assert after[after.index("--max-duration-sec") + 1] == "60"


def test_original_capture_physics_ast_and_owner_algorithms_match_v1_exactly():
    assert ast.dump(worker.capture_ast()) == ast.dump(worker.baseline.capture_ast())
    for name in ("motion_quality", "turn_screen", "teacher_command_audit", "diagnostic_report", "disk_check"):
        old = next(n for n in ast.parse(Path(owner.baseline.__file__).read_bytes()).body if isinstance(n, ast.FunctionDef) and n.name == name)
        new = next(n for n in owner.identity_ast().body if isinstance(n, ast.FunctionDef) and n.name == name)
        assert ast.dump(old) == ast.dump(new), name


def test_actual_scoped_reader_bound_then_restored_on_exception_and_nesting():
    prior = owner.lifecycle.independent_audit
    provider = SimpleNamespace(analyze_transport=object(), __file__="strict-reader")
    with mock.patch.object(owner, "runtime_sources", return_value={}):
        with mock.patch.object(owner.closing, "provider", return_value=provider):
            with pytest.raises(RuntimeError, match="fixture"):
                with owner.activated():
                    assert owner.lifecycle.independent_audit is provider
                    assert owner.lifecycle.MODULE == owner.NEW_MODULE
                    with pytest.raises(RuntimeError, match="unknown or nested"):
                        with owner.activated(): pass
                    raise RuntimeError("fixture")
    assert owner.lifecycle.independent_audit is prior
    assert owner._active is False


def test_provider_failure_restores_original_scope_and_reader():
    prior = owner.lifecycle.independent_audit
    with mock.patch.object(owner, "runtime_sources", return_value={}):
        with mock.patch.object(owner.closing, "provider", side_effect=ValueError("reader changed")):
            with pytest.raises(ValueError, match="reader changed"):
                with owner.activated(): pass
    assert owner.lifecycle.independent_audit is prior and owner._active is False


def test_foreign_receipt_reader_cannot_be_silently_replaced():
    with mock.patch.object(owner.lifecycle, "independent_audit", SimpleNamespace()):
        with pytest.raises(RuntimeError, match="unknown or nested"):
            with owner.activated(): pass
    assert owner.lifecycle.independent_audit is owner.original_ack


def test_public_main_contract_declares_new_schema_reader_and_fresh_module(capsys):
    assert owner.main(["--mode", "contract"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["run_id"] == "v2"
    assert result["schema"] == task.SCHEMA
    assert result["closing_receipt_reader_schema"] == owner.closing.SCHEMA
    assert result["original_V1_failed_capture_not_requalified"] is True
    assert owner.NEW_MODULE in result["worker_command"]
    assert "town01_low_speed_left45_teacher_v2" in result["owner_output"]


def test_fixed_complete347_current_v2_observations_replay_without_quality_filter():
    value = task.MovingTask(task.fixed_contract("v2", "pilot_00"))
    for frame in range(1, 1056):
        phase = "stationary_warmup" if frame <= 24 else "driving" if frame <= 925 else "stationary_tail"
        stamp = frame * .05000000074505806
        value.observe(frame=frame, timestamp=stamp, phase=phase, progress=frame * .1, base_stop_reason=None,
            event_count=0, observed_free_disk_bytes=10 * 1024 ** 3, native_actor_speed_mps=2.4,
            rear_base_CTE_m=0., observed_output_bytes=0)
        if frame % 2:
            value.saved_camera(dict(frame=frame, timestamp=stamp, capture_phase=phase,
                camera_order=list(task.original.CAMERAS), source_timestamps={name: stamp for name in task.original.CAMERAS},
                images={name: "actual.jpg" for name in task.original.CAMERAS}))
    result = value.result(tail_states=130)
    assert result["task_complete"] and len(value.selected) == 347
    assert value.support["frame"] == 925 and value.first["frame"] == 105 and value.last["frame"] == 797
    assert result["schema"] == task.SCHEMA + ".result"
    assert result["training_admission_state"] == "PENDING_QA_NOT_AUTHORIZED"


def test_source_closure_contains_all_originals_new_identities_and_strict_reader():
    pins = owner.runtime_sources()
    for name in ("town01_low_speed_left45_task_20261002.py", "collect_town01_low_speed_left45_20261002.py",
        "run_owned_town01_low_speed_left45_20261002.py", "town01_low_speed_left45_task_v2_20261002.py",
        "collect_town01_low_speed_left45_v2_20261002.py", "run_owned_town01_low_speed_left45_v2_20261002.py",
        "audit_town01_left45_closing_control_20261002.py"):
        assert "tools/e2e/" + name in pins
