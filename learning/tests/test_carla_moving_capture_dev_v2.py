# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Keep new DEV declaration fail-closed and legacy moving protocol byte-independent."""
from copy import deepcopy
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import carla_moving_capture_dev_v2 as dev
from scripts.e2e import collect_carla_moving_dev_v2 as driver

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())


@pytest.fixture
def declared(tmp_path, monkeypatch):
    for name in dev.SOURCE_PATHS:
        target = tmp_path/name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text("fixture source: "+name)
    route = tmp_path/dev.ROUTE_PATH
    route.parent.mkdir(parents=True, exist_ok=True)
    route.write_bytes((_hh_source_path(ROOT, dev.ROUTE_PATH)).read_bytes())
    mapping, calibration = tmp_path/"mapping.yaml", tmp_path/"calibration.yaml"
    mapping.write_text("fixture mapping"); calibration.write_text("fixture calibration")
    evidence = tmp_path/dev.SELECTION_PATH
    evidence.parent.mkdir(parents=True, exist_ok=True)
    evidence.write_bytes(dev.encoded(dict(schema="carla.dev_route_overlap_resolution.v1",
        status="GEOMETRY_ROUTE_SELECTED_NOT_CAPTURE_AUTHORIZED", proposed_split="val",
        selected_candidate=dict(route_id="town03_straight_s0000_p07", route_path=str(route), route_sha256=dev.ROUTE_SHA),
        execution_allowed=False, dataset_admission_approved=False)))
    monkeypatch.setattr(dev, "ROOT", tmp_path)
    monkeypatch.setattr(dev, "SELECTION_SHA", dev.sha(evidence))
    plan = dev.build_plan(tmp_path/dev.CAPTURE_PATH, route, mapping, calibration, "2026-09-16T00:00:00Z")
    path = tmp_path/dev.PLAN_PATH
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(dev.encoded(plan))
    args = SimpleNamespace(**dev.FIXED_ARGS, output=tmp_path/dev.CAPTURE_PATH, route_file=route,
        mapping=mapping, calibration=calibration, moving_task_plan=path, moving_task_plan_sha256=dev.sha(path))
    return args, plan, path


def test_declared_dev_identity_and_unchanged_native_math(declared):
    args, plan, _ = declared
    assert dev.configuration_from_args(args) == plan
    assert plan["schema"] == "carla.moving_task_capture_plan.v2"
    assert plan["town"] == "Town03" and plan["dataset_split_intent"] == "val"
    assert plan["fixed_arguments"]["seed"] == 201 and plan["route_id"].endswith("p07")
    assert plan["admission_status"] == "PENDING_QA"
    assert "development_only" not in plan and "training_data_approved" not in plan
    assert dev.MovingTask is dev.legacy.MovingTask
    assert dev.measured_progress is dev.legacy.measured_progress
    assert dev.legacy.plan_contract()["town"] == "Town07"
    assert dev.legacy.FIXED_ARGS["seed"] == 0


@pytest.mark.parametrize("key,value", [("seed", 0), ("target_speed_kmh", 30), ("capture_hz", 5),
    ("allow_map_load", True), ("allow_map_load", 0), ("port", 2000), ("weather", "CloudyNoon"),
    ("stationary_tail_sec", 6.4), ("agent_initialization", "before_bootstrap")])
def test_setting_change_is_rejected(declared, key, value):
    args, _, _ = declared
    setattr(args, key, value)
    with pytest.raises(ValueError):
        dev.configuration_from_args(args)


@pytest.mark.parametrize("mutation", ["source", "selection", "mapping", "route", "output", "missingflag", "symlink"])
def test_mutations_fail_before_world_access(declared, mutation):
    args, _, path = declared
    if mutation == "source":
        (_hh_source_path(dev.ROOT, dev.SOURCE_PATHS[0])).write_text("modified")
    elif mutation == "selection":
        (_hh_source_path(dev.ROOT, dev.SELECTION_PATH)).write_text("{}")
    elif mutation == "mapping":
        args.mapping.write_text("modified")
    elif mutation == "route":
        args.route_file.write_text("{}")
    elif mutation == "output":
        args.output = args.output.parent/"other"
    elif mutation == "missingflag":
        args.moving_task_plan = None
    else:
        alias = path.parent/"alias.json"; alias.symlink_to(path); args.moving_task_plan = alias
    with pytest.raises(ValueError):
        dev.configuration_from_args(args)


@pytest.mark.parametrize("change", [dict(dataset_split_intent="train"), dict(route_id="town03_straight_s0000_p04"),
    dict(required_future_support_ns=6_300_000_000), dict(maximum_attempts=2),
    dict(declared_at_utc="2026-09-16T00:00:00"),
    dict(declared_at_utc=(datetime.now(timezone.utc)+timedelta(days=2)).isoformat())])
def test_rehashed_plan_cannot_change_fixed_contract(declared, change):
    args, plan, path = declared
    value = deepcopy(plan); value.update(change)
    path.write_bytes(dev.encoded(value)); args.moving_task_plan_sha256 = dev.sha(path)
    with pytest.raises(ValueError):
        dev.configuration_from_args(args)


def test_flagless_direct_helper_has_no_legacy_fallback():
    with pytest.raises(ValueError):
        dev.configuration_from_args(SimpleNamespace())


def test_actual_driver_parser_accepts_generated_options_and_restores_legacy(declared):
    args, plan, path = declared
    options = dev.collector_options(path)
    options += ["--mapping", str(args.mapping), "--calibration", str(args.calibration)]
    previous = driver.base.moving_capture_module
    parsed = driver.parse_args([str(args.output), str(args.route_file), "--host", "127.0.0.1", "--port", "2100", *options])
    assert dev.configuration_from_args(parsed) == plan
    assert driver.base.moving_capture_module is previous


def test_redeclaring_existing_plan_is_refused(declared):
    args, _, path = declared
    before = path.read_bytes()
    with pytest.raises(ValueError, match="fresh fixed"):
        dev.main(["--declare", str(path)])
    assert path.read_bytes() == before
