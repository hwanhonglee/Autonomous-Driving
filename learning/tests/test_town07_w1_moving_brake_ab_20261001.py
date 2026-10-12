"""HH_261001 - Exercise isolated ownership, source archives and bounded sampled inputs without CARLA."""

from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np
import pytest

from scripts.e2e import research_town07_w1_moving_brake_ab_20261001 as study


def parsed_capture(index=0):
    args = study.short.collector.parse_args(study.capture_argv(index))
    args.research_speed_cap_mps, args.research_max_cte_m = 2.5, 1.
    return args


@pytest.mark.parametrize("index", [0, 1])
def test_complete_fixed_inherited_capture_arguments(index):
    args = parsed_capture(index)
    study.validate_capture(args, dict(town="Town07", scenario="straight"), index)
    assert args.control_transport == "acknowledged_batch"
    assert args.allow_map_load is False and args.stationary_tail_sec == 0.
    assert args.physics_substep_profile == "inherited"


@pytest.mark.parametrize("field,value", [("max_duration_sec", 21), ("physics_hz", 10),
    ("capture_hz", 20), ("target_speed_kmh", 30), ("research_speed_cap_mps", 3.),
    ("research_max_cte_m", 2.), ("spawn_z_offset_m", 0.), ("allow_map_load", True),
    ("control_transport", "direct"), ("goal_stop_profile", "comfortable_v2")])
def test_worker_capture_rejects_scope_changes(field, value):
    args = parsed_capture()
    setattr(args, field, value)
    with pytest.raises(ValueError):
        study.validate_capture(args, dict(town="Town07", scenario="straight"), 0)


def test_runtime_disk_guard_precedes_bundle_and_restores(monkeypatch):
    calls = []
    original = lambda *_args, **_kwargs: calls.append("bundle")
    monkeypatch.setattr(study.short.collector, "exact_camera_bundle", original)
    monkeypatch.setattr(study.shutil, "disk_usage", lambda _: SimpleNamespace(free=study.protocol.FLOOR))
    with pytest.raises(study.short.collector.CollectionError, match="floor"):
        with study.disk_guard():
            study.short.collector.exact_camera_bundle(None)
    assert not calls and study.short.collector.exact_camera_bundle is original


@pytest.mark.parametrize("index", [0, 1])
def test_owner_preflight_restores_globals_and_never_launches(monkeypatch, tmp_path, index):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    monkeypatch.setattr(study.protocol, "RUN_ROOT", tmp_path)
    monkeypatch.setattr(study, "declaration", lambda _: dict(source_sha256={}, external_agent_sha256={},
        asset_sha256={str(study.protocol.WEIGHTS): study.protocol.WEIGHTS_SHA,
                      str(study.protocol.METADATA): study.protocol.METADATA_SHA}))
    monkeypatch.setattr(study.protocol, "disk_check", lambda arms: dict(reserved_remaining_arms=arms))
    checks = []
    monkeypatch.setattr(study, "prior_attempt_ready", lambda _: checks.append("predecessor"))
    monkeypatch.setattr(study, "load_w1", lambda: checks.append("W1 loaded"))
    monkeypatch.setattr(owner, "preflight", lambda: dict(collector_command=owner.collector_command()))
    def forbidden(*_args):
        raise AssertionError("preflight launched a simulator")
    monkeypatch.setattr(owner, "execute", forbidden)
    saved = (owner.OUTPUT, owner.WEIGHTS, owner.METADATA, owner.PINS, owner.collector_command,
             owner.write_json, owner.lifecycle._workspace_lock)
    result = study.trial(SimpleNamespace(mode="preflight", trial=index, plan_sha256="a"*64))
    assert result["source_checkpoint_sha256"] == study.protocol.CHECKPOINT
    assert result["model_state_sha256"] == study.protocol.MODEL_STATE
    assert result["disk_reservation"]["reserved_remaining_arms"] == 2-index
    command = result["collector_command"]
    assert study.MODULE in command
    assert command[command.index("--trial") + 1] == str(index)
    assert command[command.index("--max-duration-sec") + 1] == "20"
    assert command[command.index("--target-speed-kmh") + 1] == "9"
    assert result["exploratory"] and result["ABBA"] is False
    assert checks == (["predecessor", "W1 loaded"] if index else ["W1 loaded"])
    assert (owner.OUTPUT, owner.WEIGHTS, owner.METADATA, owner.PINS, owner.collector_command,
            owner.write_json, owner.lifecycle._workspace_lock) == saved
    assert not list(tmp_path.iterdir())


def test_reserve_failure_leaves_no_owner_output(monkeypatch, tmp_path):
    monkeypatch.setattr(study.protocol, "RUN_ROOT", tmp_path)
    monkeypatch.setattr(study, "declaration", lambda _: {})
    def failed_reserve(_):
        raise ValueError("projected headroom unavailable")
    monkeypatch.setattr(study.protocol, "disk_check", failed_reserve)
    with pytest.raises(ValueError, match="headroom"):
        study.trial(SimpleNamespace(mode="preflight", trial=0, plan_sha256="a"*64))
    assert not list(tmp_path.iterdir())


def test_failed_motion_does_not_block_clean_predecessor(monkeypatch, tmp_path):
    monkeypatch.setattr(study.protocol, "RUN_ROOT", tmp_path)
    folder = study.protocol.output(0)
    folder.mkdir()
    study.write_new(folder / "declared_plan.json", {"fixed": True, "external_agent_sha256": {}})
    pinned = study.digest(folder / "declared_plan.json")
    study.write_new(folder / "owner_plan.json", dict(schema=study.protocol.SCHEMA,
        trial=study.protocol.trial(0), declaration_sha256=pinned,
        source_checkpoint_sha256=study.protocol.CHECKPOINT, source_sha256={}))
    result = dict(status="CAPTURE_FAILED_RETAINED", trial=study.protocol.trial(0),
        source_checkpoint_sha256=study.protocol.CHECKPOINT, source_hashes_match=True,
        external_source_hashes_match=True, archived_declaration_sha256=pinned,
        port_triplet_released=True, owner_cleanup_errors=[], progress_m=0.)
    study.write_new(folder / "owner_diagnostic_result.json", result)
    assert study.prior_attempt_ready(pinned)["status"] == "CAPTURE_FAILED_RETAINED"
    result["owner_cleanup_errors"] = ["owned process remains"]
    (folder / "owner_diagnostic_result.json").unlink()
    study.write_new(folder / "owner_diagnostic_result.json", result)
    with pytest.raises(ValueError, match="cleanup"):
        study.prior_attempt_ready(pinned)


def test_sampler_uses_existing_cadence_and_budget_rejects_before_npz(monkeypatch, tmp_path):
    original_save = study.exact.save_input_snapshot
    @contextmanager
    def sampled(directory):
        directory.mkdir()
        (directory / "index.jsonl").touch()
        yield
    monkeypatch.setattr(study.w1, "sampled_inputs", sampled)
    monkeypatch.setattr(study.protocol, "SAMPLE_LIMIT", 32)
    tensors = {"images": np.zeros((2, 2), dtype=np.float32)}
    decision = dict(raw_output={"retained": True}, control={"brake": 0.}, source_checkpoint_sha256="W1")
    with pytest.raises(study.short.collector.CollectionError, match="allowance"):
        with study.bounded_sampled_inputs(tmp_path):
            study.exact.save_input_snapshot(tmp_path / "sampled_exact_runtime_inputs", 0,
                tensors, {"now_ns": 1}, decision, 0.)
    assert study.exact.save_input_snapshot is original_save
    assert not list((tmp_path / "sampled_exact_runtime_inputs").glob("*.npz"))
    abort = study.read_json(tmp_path / "sample_allowance_abort.json")
    assert abort["rejected_decision"] == decision and abort["control_sent"] is False


def test_exact_sample_data_hashes_and_allowance_accounting(monkeypatch, tmp_path):
    @contextmanager
    def sampled(directory):
        directory.mkdir()
        (directory / "index.jsonl").touch()
        yield
    monkeypatch.setattr(study.w1, "sampled_inputs", sampled)
    tensors = {"images": np.arange(16, dtype=np.float32).reshape(4, 4)}
    decision = dict(raw_output={"retained": True}, source_checkpoint_sha256="W1", total_seconds=.01)
    directory = tmp_path / "sampled_exact_runtime_inputs"
    with study.bounded_sampled_inputs(tmp_path):
        row = study.exact.save_input_snapshot(directory, 0, tensors, {"now_ns": 1}, decision, .001)
    assert study.digest(directory / row["tensor_file"]) == row["tensor_file_sha256"]
    with np.load(directory / row["tensor_file"], allow_pickle=False) as arrays:
        assert np.array_equal(arrays["images"], tensors["images"])
    assert row["sample_directory_allowance_bytes"] == 128*1024**2
    assert study.directory_bytes(directory) < study.protocol.SAMPLE_LIMIT


def test_create_only_source_archive_keeps_declared_bytes(monkeypatch, tmp_path):
    source_root = tmp_path / "root"
    source_root.mkdir()
    source = source_root / "pinned.py"
    source.write_text("pinned source\n")
    plan = source_root / "plan.json"
    study.write_new(plan, {"fixed": True})
    monkeypatch.setattr(study.protocol, "ROOT", source_root)
    monkeypatch.setattr(study.protocol, "PLAN", plan)
    run = tmp_path / "run"
    run.mkdir()
    value = dict(source_sha256={})
    study.archive_sources(run, value, dict(source_sha256={"pinned.py": study.digest(source)},
                         external_agent_sha256={}), study.digest(plan))
    assert study.digest(run / "source/pinned.py") == value["source_sha256"]["pinned.py"]
    assert study.digest(run / "declared_plan.json") == study.digest(plan)


def test_cli_static_contract_never_declares_or_executes(monkeypatch):
    def forbidden(*_args, **_kwargs):
        raise AssertionError("static contract mutated or launched")
    monkeypatch.setattr(study.protocol, "build_plan", forbidden)
    monkeypatch.setattr(study, "write_new", forbidden)
    monkeypatch.setattr(study, "trial", forbidden)
    assert study.main(["--mode", "contract"]) == 0
    with pytest.raises(ValueError, match="overrides"):
        study.main(["--mode", "contract", "--max-duration-sec", "150"])
    with pytest.raises(ValueError, match="requires"):
        study.main(["--mode", "execute"])
