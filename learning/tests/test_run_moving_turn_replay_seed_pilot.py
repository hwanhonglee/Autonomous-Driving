# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - CPU/mock-only tests for the sealed E turn-replay diagnostic."""
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
PATH = _hh_source_path(ROOT, "scripts/e2e/run_moving_turn_replay_seed_pilot.py")
SPEC = importlib.util.spec_from_file_location("moving_turn_replay_seed_pilot_test", PATH)
m = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(m)


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(m.encoded(value))


def fake_sampling_plan():
    base = [{"base_occurrence_index": index, "dataset_index": index,
             "sample_token": f"sample-{index:04d}", "domain": "carla"}
            for index in range(1475)]
    replay = [{"replay_occurrence_index": index, "dataset_index": index,
               "sample_token": f"sample-{index:04d}", "domain": "carla"}
              for index in range(327)]
    tokens = [row["sample_token"] for row in replay]
    index_tokens = [{"dataset_index": row["dataset_index"], "token": row["sample_token"]} for row in replay]
    cohort = {"schema": "portable_e2e.cohort_replay.v1", "criterion": m.COHORT_CRITERION,
        "occurrence_order_hash_encoding": "sha256_canonical_json_array_v1",
        "replay_copies_per_epoch": 1, "base_sample_count": 1475,
        "membership_sample_count": 327, "membership_sample_tokens_sha256": m.canonical_sha256(tokens),
        "membership_index_token_sha256": m.canonical_sha256(index_tokens), "replay_occurrence_count": 327,
        "base_occurrences_sha256": m.canonical_sha256(base), "base_occurrences": base,
        "replay_occurrences_sha256": m.canonical_sha256(replay), "replay_occurrences": replay}
    return {"policy": "cohort_replay", "order_algorithm": "smooth_proportional_cohort_replay_interleave_v1",
        "seed_derivation": "sha256_seed_epoch_named_stream_v1", "known_domains": ["carla", "real"],
        "domain_ratios": {}, "replacement": False, "cross_stream_sample_reuse": True,
        "dataset_domain_counts": {"carla": 1475}, "epoch_domain_sample_counts": {"carla": 1802},
        "epoch_replayed_domain_sample_counts": {"carla": 327}, "epoch_discarded_sample_counts": {"carla": 0},
        "epoch_sample_count": 1802, "batch_size": 4, "batches_per_epoch": 451,
        "cohort_replay": cohort}


def fake_schedule():
    rows = []
    for epoch in range(20):
        streams = [("base", index, index) for index in range(1475)] + \
                  [("replay", index, index) for index in range(327)]
        rows.extend({"epoch": epoch, "position": position, "stream": stream,
                     "occurrence_index": occurrence, "dataset_index": dataset}
                    for position, (stream, occurrence, dataset) in enumerate(streams))
    return {"schema": "portable_e2e.cohort_replay_execution_schedule.v1", "epoch_count": 20,
        "epoch_sample_count": 1802, "row_count": 36040, "hash_encoding": "sha256_canonical_json_v1",
        "sha256": m.canonical_sha256(rows), "rows": rows}


@pytest.fixture(autouse=True)
def fixture_membership_hashes(monkeypatch):
    plan = fake_sampling_plan()["cohort_replay"]
    monkeypatch.setattr(m, "COHORT_TOKEN_SHA256", plan["membership_sample_tokens_sha256"])
    monkeypatch.setattr(m, "COHORT_INDEX_TOKEN_SHA256", plan["membership_index_token_sha256"])


def fixture_plan():
    sampling = fake_sampling_plan(); schedule = fake_schedule()
    train_examples, val_examples = "a" * 64, "b" * 64
    split_examples = {"train": train_examples, "val": val_examples}
    tensor = {split: hashlib.sha256((value + "\0" + split).encode()).hexdigest()
              for split, value in split_examples.items()}
    data = {"path": "datasets/prepared/moving/E", "manifest_sha256": "1" * 64,
        "corpus_fingerprint_sha256": "2" * 64, "split_fingerprint_sha256": tensor,
        "split_loader_sha256": {"train": "3" * 64, "val": "4" * 64},
        "split_examples_sha256": split_examples,
        "split_tokens_sha256": {"train": "5" * 64, "val": "6" * 64},
        "counts": {"train": 1475, "val": 665},
        "episode_ids": {"train": ["train0", "train1", "train2", "train3"],
            "val": [row["episode_id"] for row in m.VALIDATION_EPISODES]},
        "tree": {"manifest_sha256": "1" * 64, "file_count": 1, "directory_count": 1,
            "total_size_bytes": 2, "files": [{"path": "dataset.json", "size": 2, "sha256": "1" * 64}]}}
    base = {name: "7" * 64 for name in m.BASE_SOURCE_PATHS}
    current = dict(base)
    for index, name in enumerate(m.OVERLAY_PATHS): current[name] = str(index + 8) * 64
    files = {name: {"path": path, "sha256": str(index + 1) * 64}
             for index, (name, path) in enumerate(m.REFERENCE_FILES.items())}
    return {"schema": m.PLAN_SCHEMA, "campaign_id": m.CAMPAIGN_ID,
        "declared_at_utc": "2026-09-21T00:00:00Z", "base_commit": m.BASE_COMMIT,
        "gpu": dict(m.GPU), "source": {"base_commit": m.BASE_COMMIT, "base_sha256": base,
            "current_sha256": current, "overlay_paths": list(m.OVERLAY_PATHS),
            "added_paths": list(m.ADDED_SOURCE_PATHS),
            "added_sha256": {name: "a" * 64 for name in m.ADDED_SOURCE_PATHS}},
        "runner_sha256": m.digest(PATH), "evaluator_sha256": "d" * 64,
        "owner": {"path": m.OWNER_PATH, "sha256": m.OWNER_SHA256},
        "model_config": m.MODEL_PATH, "model_config_sha256": m.MODEL_SHA256,
        "model_config_canonical_sha256": m.MODEL_CANONICAL_SHA256, "model_id": m.MODEL_ID,
        "view_report_sha256": "e" * 64, "output": m.OUTPUT_PATH,
        "arm": {"label": m.ARM, "dataset_role": m.VIEW_ARM, "dataset": data,
            "expected_train_state": deepcopy(m.EXPECTED_TRAIN_STATE),
            "expected_sampling_stream_samples_seen": deepcopy(m.EXPECTED_STREAM_SAMPLES),
            "train_config": deepcopy(m.TRAIN_CONFIG), "sampling_plan": sampling,
            "sampling_plan_sha256": m.canonical_sha256(sampling), "execution_schedule": schedule},
        "validation": {"episodes": list(m.VALIDATION_EPISODES), "count": 665,
            "fingerprint_sha256": val_examples, "tokens_sha256": "6" * 64},
        "reference_C": {"root": "runs/diagnostics/reference", "campaign_id":
            "hh260921-moving-data-high-budget-ab-seed20260906-v1", "arm": "A_baseline_train",
            "files": files, "metrics": deepcopy(m.REFERENCE_METRICS),
            "geometry_pass_counts": deepcopy(m.REFERENCE_GEOMETRY)},
        "train_config": deepcopy(m.TRAIN_CONFIG), "loss_config": deepcopy(m.LOSS_CONFIG),
        "stage_order": list(m.STAGES), "stage_timeout_seconds": m.TIMEOUTS,
        "whole_timeout_seconds": m.WHOLE_TIMEOUT_SECONDS, "decision": deepcopy(m.DECISION), **m.DENIALS}


def test_exact_plan_binds_replay_and_twenty_epoch_schedule():
    plan = fixture_plan()
    m.validate_plan(plan, runner_sha256=m.digest(PATH), observed=datetime(2026, 9, 21, 1, tzinfo=timezone.utc))
    cohort = plan["arm"]["sampling_plan"]["cohort_replay"]
    assert cohort["base_sample_count"] == 1475 and cohort["membership_sample_count"] == 327
    assert plan["arm"]["execution_schedule"]["row_count"] == 36040
    assert plan["arm"]["execution_schedule"]["sha256"] == \
        m.canonical_sha256(plan["arm"]["execution_schedule"]["rows"])
    assert m.EXPECTED_TRAIN_STATE == {"epoch": 20, "next_batch_index": 0, "global_step": 9020,
        "samples_seen": 36040, "domain_samples_seen": {"carla": 36040}}
    assert m.EXPECTED_STREAM_SAMPLES == {"base": 29500, "replay": 6540}


def test_real_membership_hashes_are_preregistered(monkeypatch):
    monkeypatch.undo()
    assert m.COHORT_TOKEN_SHA256 == "eb59110e78718fe6c1d79aa233b550d1bea487ba929264ad30d9eb929575be2f"
    assert m.COHORT_INDEX_TOKEN_SHA256 == "d8f80fa3d19cb46fb97b17d1480371996ef11071e0005824c6cd0512d3ea7bde"


def test_train_command_has_only_fixed_cohort_replay_interface(tmp_path):
    plan = fixture_plan(); evaluator = tmp_path / "evaluator.py"; evaluator.write_text("# fixture\n")
    command = m.stage_command(plan, tmp_path, ROOT, tmp_path / "out", tmp_path / "plan.json",
                              evaluator, PATH, "train")
    assert command[command.index("--sampling-policy") + 1] == "cohort_replay"
    assert command[command.index("--max-steps") + 1] == "9020"
    assert command[command.index("--checkpoint-interval") + 1] == "451"
    assert "--resume" not in command and not any("threshold" in value or "copies" in value for value in command)
    checkpoint = tmp_path / "out" / m.ARM / "training/checkpoints/latest.pt"
    checkpoint.parent.mkdir(parents=True); checkpoint.write_bytes(b"fixture")
    plan_path = tmp_path / "plan.json"; put(plan_path, plan)
    subgroup = m.stage_command(plan, tmp_path, ROOT, tmp_path / "out", plan_path,
                               evaluator, PATH, "subgroup")
    for flag in ("--repo-root", "--workspace", "--runner", "--runner-sha256", "--plan",
                 "--plan-sha256", "--checkpoint", "--checkpoint-sha256", "--output-dir"):
        assert subgroup.count(flag) == 1


@pytest.mark.parametrize("change", ["test", "promotion", "policy", "steps", "stream", "reference",
                                           "overlay", "added", "schedule_sha", "schedule_count", "criterion"])
def test_plan_rejects_scope_provenance_or_exposure_mutation(change):
    plan = fixture_plan()
    if change == "test": plan["test_read_or_use"] = True
    elif change == "promotion": plan["decision"]["adoption_allowed"] = True
    elif change == "policy": plan["arm"]["sampling_plan"]["policy"] = "uniform_without_replacement"
    elif change == "steps": plan["arm"]["train_config"]["max_steps"] = 9019
    elif change == "stream": plan["arm"]["expected_sampling_stream_samples_seen"]["replay"] = 6539
    elif change == "reference": plan["reference_C"]["metrics"]["town03_right_turn_dev"]["ade_6p4s_m"] += 0.01
    elif change == "overlay": plan["source"]["overlay_paths"] = ["portable_e2e/train.py"]
    elif change == "added": plan["source"]["added_paths"] = []
    elif change == "schedule_sha": plan["arm"]["execution_schedule"]["sha256"] = "0" * 64
    elif change == "schedule_count": plan["arm"]["execution_schedule"]["row_count"] -= 1
    else: plan["arm"]["sampling_plan"]["cohort_replay"]["criterion"]["threshold_rad"] = 0.21
    with pytest.raises(ValueError): m.validate_plan(plan, runner_sha256=m.digest(PATH))


def gate_inputs():
    episodes = {}
    for episode in m.VALIDATION_EPISODES:
        label, count = episode["label"], episode["count"]
        episodes[label] = {"episode_id": episode["episode_id"], "count": count,
            "metrics": dict(m.MAXIMUM_E[label]), "metric_counts": {key: count for key in m.MAXIMUM_E[label]}}
    subgroup = {"episodes": episodes}
    audit = {"geometry": {"sample_count": 665, "any_candidate_geometry_pass_count": 664,
        "all_candidates_geometry_pass_count": 664,
        "selected_result": {"sample_count": 665, "geometry_pass_count": 664}}}
    return subgroup, audit


def test_gate_boundary_passes_without_promotion():
    result = m.diagnostic_gate(*gate_inputs())
    assert result["status"] == "PASS_SINGLE_SEED_DIAGNOSTIC" and result["pass"] is True
    assert result["adoption_allowed"] is result["automatic_promotion"] is False
    assert result["next_action"].endswith("20260907_20260908_20260909")


@pytest.mark.parametrize("label,key", [(label, key) for label, values in m.MAXIMUM_E.items() for key in values])
def test_each_metric_can_fail_gate(label, key):
    subgroup, audit = gate_inputs(); subgroup["episodes"][label]["metrics"][key] += 1e-9
    assert m.diagnostic_gate(subgroup, audit)["pass"] is False


@pytest.mark.parametrize("field", ["selected", "any_candidate", "all_candidates"])
def test_each_geometry_count_can_fail_gate(field):
    subgroup, audit = gate_inputs()
    if field == "selected": audit["geometry"]["selected_result"]["geometry_pass_count"] = 663
    else: audit["geometry"][field + "_geometry_pass_count"] = 663
    assert m.diagnostic_gate(subgroup, audit)["pass"] is False


@contextmanager
def nothing(*_args):
    yield


@pytest.mark.parametrize("fail", [False, True])
def test_mock_owned_run_is_sequential_fail_closed_and_never_promotes(tmp_path, monkeypatch, fail):
    workspace = tmp_path / "portable_e2e"; repo = workspace / "overlay_repo"; repo.mkdir(parents=True)
    external = workspace / "benchmarks/E"; external.mkdir(parents=True)
    runner = external / PATH.name; runner.write_bytes(PATH.read_bytes())
    evaluator = external / "evaluator.py"; evaluator.write_text("# fixture evaluator\n")
    owner = workspace / m.OWNER_PATH; owner.parent.mkdir(parents=True)
    owner.write_bytes((_hh_source_path(ROOT, "scripts/e2e/profile_portable_training.py")).read_bytes())
    pipeline = repo / m.PIPELINE_PATH; pipeline.parent.mkdir(parents=True); pipeline.write_text("# fixture pipeline\n")
    view = external / "view.json"; put(view, {"fixture": True})
    reference = workspace / "runs/diagnostics/reference"; reference.mkdir(parents=True)
    plan = fixture_plan(); plan["runner_sha256"] = m.digest(runner); plan["evaluator_sha256"] = m.digest(evaluator)
    plan["view_report_sha256"] = m.digest(view)
    plan_path = external / "plan.json"; put(plan_path, plan)
    calls = []
    def stage(command, _repo, _env, _log, timeout):
        calls.append((command, timeout)); item = workspace / m.OUTPUT_PATH / m.ARM
        if len(calls) == 1 and fail: return 17
        if "portable_e2e.train" in command:
            target = item / "training/checkpoints/latest.pt"; target.parent.mkdir(parents=True); target.write_bytes(b"checkpoint")
        return 0
    owner_api = SimpleNamespace(check_environment=lambda *_: None, gpu_lease=nothing,
        gpu_idle=lambda *_: dict(m.GPU, pre_cuda_compute_processes=0), bounded_signals=nothing,
        finalization_signals=nothing)
    pipeline_api = SimpleNamespace(run_owned_stage=stage)
    monkeypatch.setattr(m, "verify_sources", lambda *_: None)
    monkeypatch.setattr(m, "verify_dataset", lambda *_: None)
    monkeypatch.setattr(m, "verify_reference", lambda *_: reference)
    monkeypatch.setattr(m, "_archive", lambda *_: None)
    monkeypatch.setattr(m, "load_module", lambda path, _sha, name:
        owner_api if "gpu_owner" in name else pipeline_api)
    def verify(_plan, output, stage_name):
        item = output / m.ARM; target = item / (stage_name + "_mock.json"); put(target, {"stage": stage_name})
        if stage_name == "audit": put(item / "gate_v8.json", {"fixture": "audit"})
        if stage_name == "subgroup": put(item / "subgroup/summary.json", {"fixture": "subgroup"})
        checkpoint = item / "training/checkpoints/latest.pt"
        return {"path": target.relative_to(output).as_posix(), "sha256": m.digest(target),
                "checkpoint_sha256": m.digest(checkpoint)}
    monkeypatch.setattr(m, "verify_stage", verify)
    monkeypatch.setattr(m, "diagnostic_gate", lambda *_: {"status": "FAIL_SINGLE_SEED_DIAGNOSTIC",
                                                           "pass": False, "adoption_allowed": False})
    kwargs = dict(plan_path=plan_path, repo_root=repo, workspace=workspace, view_report=view,
                  reference_root=reference, evaluator=evaluator, runner=runner)
    if fail:
        with pytest.raises(ValueError, match="exited 17"): m.run(**kwargs)
        result = m.read(workspace / m.OUTPUT_PATH / "failed.json")
        assert result["status"] == "FAIL" and len(calls) == 1
    else:
        m.run(**kwargs); result = m.read(workspace / m.OUTPUT_PATH / "summary.json")
        assert len(calls) == 4 and result["status"] == "COMPLETE_NOT_PROMOTED"
        assert result["sampling_execution_schedule"] == {"row_count": 36040,
            "sha256": plan["arm"]["execution_schedule"]["sha256"]}
        assert result["automatic_promotion"] is False and result["test_read_or_use"] is False


def test_evaluator_cli_has_no_split_or_test_escape():
    path = _hh_source_path(ROOT, "scripts/e2e/evaluate_portable_moving_turn_replay_subgroups.py")
    spec = importlib.util.spec_from_file_location("turn_replay_evaluator_cli_test", path)
    evaluator = importlib.util.module_from_spec(spec); sys.modules[spec.name] = evaluator; spec.loader.exec_module(evaluator)
    args = evaluator.parse_args(["--repo-root", "/repo", "--workspace", "/workspace",
        "--runner", "/runner", "--runner-sha256", "a" * 64, "--plan", "/plan",
        "--plan-sha256", "b" * 64, "--checkpoint", "/checkpoint",
        "--checkpoint-sha256", "c" * 64, "--output-dir", "/output"])
    assert not hasattr(args, "split") and evaluator.DENIALS["test_read_or_use"] is False
