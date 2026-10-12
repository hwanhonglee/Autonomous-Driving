# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - CPU/mock tests for the VAL-only E subgroup evaluator."""
from dataclasses import dataclass
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
PATH = _hh_source_path(ROOT, "scripts/e2e/evaluate_portable_moving_turn_replay_subgroups.py")
SPEC = importlib.util.spec_from_file_location("moving_turn_replay_subgroups_test", PATH)
e = importlib.util.module_from_spec(SPEC); sys.modules[SPEC.name] = e; SPEC.loader.exec_module(e)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@dataclass(frozen=True)
class Episode:
    label: str
    episode_id: str
    count: int


def test_mock_evaluation_reads_only_val_and_persists_two_subgroups(tmp_path, monkeypatch):
    workspace = tmp_path / "workspace"; repo = workspace / "repo"; repo.mkdir(parents=True)
    dataset_root = workspace / "datasets/prepared/E"; dataset_root.mkdir(parents=True)
    runner_path = workspace / "runner.py"; runner_path.write_text("# sealed runner fixture\n")
    checkpoint = workspace / "checkpoint.pt"; checkpoint.write_bytes(b"opaque checkpoint")
    plan_path = workspace / "plan.json"; plan_path.write_text("{}\n")
    output_parent = workspace / "output"; output_parent.mkdir(); output = output_parent / "subgroup"
    helper_path = repo / "scripts/e2e/legacy.py"; helper_path.parent.mkdir(parents=True); helper_path.write_text("# helper\n")
    val_ids = ("turn", "straight")
    data = {"path": "datasets/prepared/E", "manifest_sha256": "1" * 64,
        "corpus_fingerprint_sha256": "2" * 64,
        "split_fingerprint_sha256": {"train": "3" * 64, "val": "4" * 64},
        "split_examples_sha256": {"train": "5" * 64, "val": "6" * 64},
        "counts": {"train": 1475, "val": 665},
        "episode_ids": {"train": ["t0", "t1", "t2", "t3"], "val": list(val_ids)}}
    sampling = {"policy": "cohort_replay"}; sampling_sha = "7" * 64
    plan = {"runner_sha256": sha(runner_path), "evaluator_sha256": sha(PATH),
        "gpu": {"uuid": "GPU-fixture"},
        "source": {"added_sha256": {"scripts/e2e/legacy.py": sha(helper_path)}},
        "arm": {"dataset": data, "sampling_plan": sampling, "sampling_plan_sha256": sampling_sha,
            "train_config": {"sampling_policy": "cohort_replay", "domain_ratios": []},
            "expected_sampling_stream_samples_seen": {"base": 29500, "replay": 6540}},
        "validation": {"fingerprint_sha256": "6" * 64, "episodes": [
            {"label": "town03_right_turn_dev", "episode_id": "turn", "count": 337},
            {"label": "town03_straight_dev", "episode_id": "straight", "count": 328}]}}
    checkpoint_payload = {"sampling_plan": sampling, "sampling_plan_sha256": sampling_sha,
        "train_config": {"sampling_policy": "cohort_replay", "domain_ratios": ()},
        "sampling_metrics": {"sampling_stream_samples_seen": {"base": 29500, "replay": 6540}},
        "training_split": "train", "training_episode_ids": data["episode_ids"]["train"],
        "dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["train"],
        "corpus_fingerprint_sha256": data["corpus_fingerprint_sha256"],
        "model_config_sha256": "8" * 64}

    def read(path):
        if Path(path) == plan_path: return plan
        return json.loads(Path(path).read_text())
    fake_runner = SimpleNamespace(ARM="E_turn_replay_train", BASE_COMMIT="base",
        LEGACY_EVALUATOR_PATH="scripts/e2e/legacy.py", regular=lambda path: Path(path).absolute(),
        digest=sha, read=read, validate_plan=lambda value, runner_sha256: value,
        canonical=lambda path: Path(path).absolute(), verify_sources=lambda *_: None,
        sha256=lambda value: value, resolve_dataset=lambda *_: dataset_root)
    examples = [SimpleNamespace(episode_id="turn") for _ in range(337)] + \
               [SimpleNamespace(episode_id="straight") for _ in range(328)]
    calls = []
    class Tensor:
        def __init__(self, values, *_args, **_kwargs):
            self.examples = values
            self.fingerprint_sha256 = data["split_fingerprint_sha256"]["val"] if len(values) == 665 else "9" * 64
    def evaluate_model(subset, **kwargs):
        calls.append((tuple(value.episode_id for value in subset.examples), kwargs)); return {"fixture": True}
    core = SimpleNamespace(load_training_examples=lambda *_args, **kwargs: (
            calls.append(("load", kwargs)) or SimpleNamespace(examples=examples,
                validation_report={"dataset_manifest_sha256": data["manifest_sha256"],
                    "dataset_fingerprint_sha256": data["corpus_fingerprint_sha256"]})),
        fingerprint_examples=lambda _: plan["validation"]["fingerprint_sha256"],
        Common10TorchDataset=Tensor, evaluate_model=evaluate_model)
    class DatasetSpec(SimpleNamespace):
        def __init__(self, **kwargs): super().__init__(**kwargs)
    fake_helper = SimpleNamespace(DatasetSpec=DatasetSpec, EpisodeSpec=Episode,
        preflight_view_metadata=lambda *_: {}, load_core=lambda *_: core,
        _checkpoint_model_config=lambda *_: (object(), checkpoint_payload),
        _episode_order=lambda _: (val_ids, {"turn": 337, "straight": 328}),
        _verify_episode_output=lambda _child, _report, **kwargs: {
            "episode_id": kwargs["episode"].episode_id, "count": kwargs["episode"].count,
            "metrics": {}, "metric_counts": {}},
        _verify_metadata_unchanged=lambda *_: None)
    monkeypatch.setattr(e, "load_module", lambda _path, _sha, name, _digest:
        fake_runner if "runner" in name else fake_helper)
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "GPU-fixture")
    args = SimpleNamespace(repo_root=repo, workspace=workspace, runner=runner_path,
        runner_sha256=sha(runner_path), plan=plan_path, plan_sha256=sha(plan_path),
        checkpoint=checkpoint, checkpoint_sha256=sha(checkpoint), output_dir=output)
    summary = e.run(args)
    assert summary["status"] == "COMPLETE_NOT_PROMOTED" and summary["combined_val_count"] == 665
    assert set(summary["episodes"]) == {"town03_right_turn_dev", "town03_straight_dev"}
    assert summary["test_read_or_use"] is summary["automatic_promotion"] is False
    assert calls[0] == ("load", {"split": "val", "mode": "planning", "check_image_hashes": True})
    assert len([row for row in calls if row[0] != "load"]) == 2
    assert json.loads((output / "summary.json").read_text()) == summary


def test_gpu_binding_fails_before_output_creation(tmp_path, monkeypatch):
    runner = tmp_path / "runner.py"; runner.write_text("# fixture\n")
    plan = tmp_path / "plan.json"; plan.write_text("{}\n")
    evaluator_sha = sha(PATH)
    fake = SimpleNamespace(regular=lambda path: Path(path), digest=sha, read=lambda _: {},
        validate_plan=lambda *_args, **_kwargs: {"evaluator_sha256": evaluator_sha,
            "gpu": {"uuid": "GPU-required"}}, canonical=lambda path: Path(path),
        verify_sources=lambda *_: None)
    monkeypatch.setattr(e, "load_module", lambda *_: fake)
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "GPU-wrong")
    args = SimpleNamespace(runner=runner, runner_sha256=sha(runner), plan=plan,
        plan_sha256=sha(plan), repo_root=tmp_path, workspace=tmp_path,
        checkpoint=tmp_path / "missing", checkpoint_sha256="a" * 64, output_dir=tmp_path / "output")
    with pytest.raises(ValueError, match="CUDA_VISIBLE_DEVICES"):
        e.run(args)
    assert not args.output_dir.exists()


def test_cli_exposes_no_split_or_test_override():
    args = e.parse_args(["--repo-root", "/repo", "--workspace", "/workspace",
        "--runner", "/runner", "--runner-sha256", "a" * 64, "--plan", "/plan",
        "--plan-sha256", "b" * 64, "--checkpoint", "/checkpoint",
        "--checkpoint-sha256", "c" * 64, "--output-dir", "/output"])
    assert not hasattr(args, "split") and e.DENIALS["test_read_or_use"] is False
