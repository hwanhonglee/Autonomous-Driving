# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Mocked tests for exact post-training moving-data subgroup evaluation."""
from __future__ import annotations

import argparse
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
SPEC = importlib.util.spec_from_file_location(
    "moving_subgroups_test_module",
    _hh_source_path(ROOT, "scripts/e2e/evaluate_portable_moving_data_subgroups.py"),
)
module = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


TRAIN_FP = "1" * 64
VAL_FP = "2" * 64
VAL_EXAMPLES_FP = "3" * 64
CORPUS_FP = "4" * 64
MODEL_FP = "5" * 64
SUBSET_FP = {
    module.EPISODES[0][1]: "6" * 64,
    module.EPISODES[1][1]: "7" * 64,
}


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_dataset(root: Path, *, val_split: str = "val") -> tuple[list[str], str]:
    train_ids = ["train_a", "train_b", "train_c"]
    rows = [
        (train_ids[0], "train", 300),
        (train_ids[1], "train", 300),
        (train_ids[2], "train", 547),
        (module.EPISODES[0][1], val_split, module.EPISODES[0][2]),
        (module.EPISODES[1][1], "val", module.EPISODES[1][2]),
    ]
    references = []
    for episode_id, split, count in rows:
        path = root / "episodes" / episode_id / "episode.json"
        write_json(path, {"episode_id": episode_id, "split": split, "sample_count": count})
        references.append(
            {
                "episode_id": episode_id,
                "manifest": f"episodes/{episode_id}/episode.json",
                "sha256": sha(path),
            }
        )
    write_json(root / "dataset.json", {"episodes": references})
    return train_ids, sha(root / "dataset.json")


def build_plan(tmp_path: Path, dataset: Path, train_ids: list[str], manifest_sha: str):
    source_sha = {name: "a" * 64 for name in module.CORE_SOURCE_PATHS}
    val_ids = [item[1] for item in module.EPISODES]
    arms = {}
    for arm in module.ARMS:
        ids = train_ids if arm == module.ARMS[0] else [*train_ids, "train_town07_moving"]
        arms[arm] = {
            "dataset": {
                "path": str(dataset),
                "manifest_sha256": manifest_sha if arm == module.ARMS[0] else "b" * 64,
                "corpus_fingerprint_sha256": CORPUS_FP if arm == module.ARMS[0] else "c" * 64,
                "split_fingerprint_sha256": {
                    "train": TRAIN_FP if arm == module.ARMS[0] else "d" * 64,
                    "val": VAL_FP,
                },
                "split_examples_sha256": {
                    "train": "e" * 64 if arm == module.ARMS[0] else "f" * 64,
                    "val": VAL_EXAMPLES_FP,
                },
                "counts": module.ARM_COUNTS[arm],
                "episode_ids": {"train": ids, "val": val_ids},
            }
        }
    plan = {
        "schema": module.PLAN_SCHEMA,
        "source_commit": module.SOURCE_COMMIT,
        "gpu": {"uuid": module.GPU_UUID},
        "source_sha256": source_sha,
        "evaluator_sha256": module.digest(Path(module.__file__)),
        "arms": arms,
        "validation": {
            "count": 665,
            "fingerprint_sha256": VAL_EXAMPLES_FP,
            "episodes": [
                {"label": label, "episode_id": episode_id, "count": count}
                for label, episode_id, count in module.EPISODES
            ],
        },
        "output": "campaign",
        "test_read_or_use": False,
    }
    path = tmp_path / "campaign" / "plan.json"
    write_json(path, plan)
    return plan, path


class FakeConfig:
    def __init__(self, value):
        self.value = dict(value)

    def to_dict(self):
        return dict(self.value)


class FakeModelConfig:
    @classmethod
    def from_mapping(cls, value):
        return FakeConfig(value)


class FakeTorchDataset:
    def __init__(self, examples, config, *, verify_image_sha256, split):
        self.examples = tuple(examples)
        self.config = config
        self.verify_image_sha256 = verify_image_sha256
        self.split = split
        episode_ids = {item.episode_id for item in self.examples}
        self.fingerprint_sha256 = (
            VAL_FP if len(episode_ids) == 2 else SUBSET_FP[next(iter(episode_ids))]
        )


def make_core(examples, checkpoint_sha, train_ids, *, payload_change=None, reorder=False):
    calls = []
    loaded_examples = tuple(reversed(examples)) if reorder else tuple(examples)
    payload = {
        "checkpoint_id": "portable-checkpoint",
        "training_split": "train",
        "dataset_fingerprint_sha256": TRAIN_FP,
        "corpus_fingerprint_sha256": CORPUS_FP,
        "training_episode_ids": sorted(train_ids),
        "model_config": {"model": "fixture"},
        "model_config_sha256": MODEL_FP,
    }
    if payload_change is not None:
        payload_change(payload)

    def evaluate(dataset, **kwargs):
        calls.append((dataset, kwargs))
        output = kwargs["output_dir"]
        trajectories = output / "trajectories"
        trajectories.mkdir(parents=True)
        for index in range(6):
            (trajectories / f"{index:04d}.png").write_bytes(b"png" + bytes([index]))
        episode_id = dataset.examples[0].episode_id
        report = {
            "status": "OPEN_LOOP_EVALUATION_COMPLETE",
            "checkpoint_sha256": checkpoint_sha,
            "training_dataset_fingerprint_sha256": TRAIN_FP,
            "dataset_fingerprint_sha256": dataset.fingerprint_sha256,
            "corpus_fingerprint_sha256": CORPUS_FP,
            "evaluation_split": "val",
            "evaluation_episode_count": 1,
            "sample_count": len(dataset.examples),
            "rendered_trajectory_png": 6,
            "metrics": {"selected_ade_m": 1.0 if episode_id == module.EPISODES[0][1] else 2.0},
            "metric_counts": {"selected_ade_m": len(dataset.examples)},
            "vehicle_control_approved": False,
        }
        write_json(output / "metrics.json", report)
        return report

    core = SimpleNamespace(
        CHECKPOINT_ID="portable-checkpoint",
        ModelConfig=FakeModelConfig,
        Common10TorchDataset=FakeTorchDataset,
        select_device=lambda name: SimpleNamespace(type="cuda", name=name),
        read_checkpoint_file=lambda path, device: (payload, checkpoint_sha),
        canonical_sha256=lambda value: MODEL_FP,
        load_training_examples=lambda *args, **kwargs: SimpleNamespace(
            examples=loaded_examples,
            fingerprint_sha256="8" * 64,
            validation_report={
                "dataset_manifest_sha256": kwargs.pop("_manifest_sha", None),
                "dataset_fingerprint_sha256": CORPUS_FP,
            },
        ),
        fingerprint_examples=lambda values: VAL_EXAMPLES_FP,
        evaluate_model=evaluate,
    )
    return core, calls, payload


@pytest.fixture
def case(tmp_path, monkeypatch):
    repo = tmp_path / "repo"
    repo.mkdir()
    dataset = tmp_path / "dataset_a"
    train_ids, manifest_sha = build_dataset(dataset)
    plan, plan_path = build_plan(tmp_path, dataset, train_ids, manifest_sha)
    checkpoint = tmp_path / "latest.pt"
    checkpoint.write_bytes(b"owned checkpoint")
    checkpoint_sha = sha(checkpoint)
    examples = tuple(
        SimpleNamespace(episode_id=episode_id, token=f"{episode_id}:{index:06d}")
        for _, episode_id, count in module.EPISODES
        for index in range(count)
    )
    core, calls, payload = make_core(examples, checkpoint_sha, train_ids)
    # The fake loader still binds its report to the actual planned manifest.
    core.load_training_examples = lambda *args, **kwargs: SimpleNamespace(
        examples=examples,
        fingerprint_sha256="8" * 64,
        validation_report={
            "dataset_manifest_sha256": manifest_sha,
            "dataset_fingerprint_sha256": CORPUS_FP,
        },
    )
    monkeypatch.setattr(module, "verify_repo_sources", lambda plan, root: None)
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", module.GPU_UUID)
    args = argparse.Namespace(
        plan=plan_path,
        plan_sha256=sha(plan_path),
        arm=module.ARMS[0],
        checkpoint=checkpoint,
        checkpoint_sha256=checkpoint_sha,
        repo_root=repo,
        output_dir=tmp_path / "evaluation",
    )
    return SimpleNamespace(
        args=args,
        plan=plan,
        plan_path=plan_path,
        repo=repo,
        dataset=dataset,
        manifest_sha=manifest_sha,
        train_ids=train_ids,
        checkpoint=checkpoint,
        checkpoint_sha=checkpoint_sha,
        examples=examples,
        core=core,
        calls=calls,
        payload=payload,
    )


def test_exact_two_episode_gpu_evaluation_and_create_only_summary(case):
    result = module.run(case.args, core=case.core)
    assert result["schema"] == module.SUMMARY_SCHEMA
    assert result["status"] == "COMPLETE_NOT_PROMOTED"
    assert result["arm"] == module.ARMS[0] and result["combined_val_count"] == 665
    assert tuple(result["episodes"]) == tuple(item[0] for item in module.EPISODES)
    assert [result["episodes"][label]["count"] for label, _, _ in module.EPISODES] == [337, 328]
    assert result["checkpoint"]["sha256"] == case.checkpoint_sha
    assert result["dataset"]["combined_val_fingerprint_sha256"] == VAL_FP
    assert all(result[key] is False for key in module.DENIALS)
    assert len(case.calls) == 2
    for (dataset, kwargs), (_, episode_id, count) in zip(case.calls, module.EPISODES):
        assert {item.episode_id for item in dataset.examples} == {episode_id}
        assert len(dataset.examples) == count
        assert kwargs["dataset_fingerprint_sha256"] == SUBSET_FP[episode_id]
        assert kwargs["corpus_fingerprint_sha256"] == CORPUS_FP
        assert kwargs["device_name"] == "cuda:0"
        assert kwargs["batch_size"] == 4 and kwargs["num_workers"] == 0
        assert kwargs["render_count"] == 6 and kwargs["evaluation_split"] == "val"
    assert json.loads((case.args.output_dir / "summary.json").read_text()) == result
    with pytest.raises(module.EvaluationError, match="fresh output"):
        module.run(case.args, core=case.core)


@pytest.mark.parametrize(
    "field,value,match",
    [
        ("plan_sha256", "f" * 64, "plan SHA-256"),
        ("checkpoint_sha256", "f" * 64, "checkpoint SHA-256"),
    ],
)
def test_cli_hash_pins_fail_before_evaluation(case, field, value, match):
    setattr(case.args, field, value)
    with pytest.raises(module.EvaluationError, match=match):
        module.run(case.args, core=case.core)
    assert not case.args.output_dir.exists() and case.calls == []


def test_exact_gpu_uuid_required(case, monkeypatch):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "0")
    with pytest.raises(module.EvaluationError, match="exact pinned GPU UUID"):
        module.run(case.args, core=case.core)
    assert case.calls == []


@pytest.mark.parametrize(
    "change,match",
    [
        (lambda payload: payload.update(training_split="val"), "not trained on TRAIN"),
        (lambda payload: payload.update(dataset_fingerprint_sha256="f" * 64), "TRAIN fingerprint"),
        (lambda payload: payload.update(corpus_fingerprint_sha256="f" * 64), "corpus fingerprint"),
        (lambda payload: payload.update(training_episode_ids=["wrong"]), "episode provenance"),
    ],
)
def test_checkpoint_provenance_is_arm_bound(case, change, match):
    core, calls, _ = make_core(case.examples, case.checkpoint_sha, case.train_ids, payload_change=change)
    core.load_training_examples = case.core.load_training_examples
    with pytest.raises(module.EvaluationError, match=match):
        module.run(case.args, core=core)
    assert calls == []


def test_test_split_is_rejected_from_metadata_before_loader(tmp_path, monkeypatch):
    repo = tmp_path / "repo"
    repo.mkdir()
    dataset = tmp_path / "dataset_a"
    train_ids, manifest_sha = build_dataset(dataset, val_split="test")
    _, plan_path = build_plan(tmp_path, dataset, train_ids, manifest_sha)
    checkpoint = tmp_path / "latest.pt"
    checkpoint.write_bytes(b"checkpoint")
    called = False

    def forbidden_loader(*args, **kwargs):
        nonlocal called
        called = True
        raise AssertionError("payload loader must not run")

    core = SimpleNamespace(load_training_examples=forbidden_loader)
    monkeypatch.setattr(module, "verify_repo_sources", lambda plan, root: None)
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", module.GPU_UUID)
    args = argparse.Namespace(
        plan=plan_path,
        plan_sha256=sha(plan_path),
        arm=module.ARMS[0],
        checkpoint=checkpoint,
        checkpoint_sha256=sha(checkpoint),
        repo_root=repo,
        output_dir=tmp_path / "evaluation",
    )
    with pytest.raises(module.EvaluationError, match="TEST/unknown split forbidden"):
        module.run(args, core=core)
    assert called is False and not args.output_dir.exists()


def test_loaded_val_order_and_full_tensor_fingerprint_are_exact(case):
    reordered = (case.examples[-1], *case.examples[:-1])
    case.core.load_training_examples = lambda *args, **kwargs: SimpleNamespace(
        examples=reordered,
        fingerprint_sha256="8" * 64,
        validation_report={
            "dataset_manifest_sha256": case.manifest_sha,
            "dataset_fingerprint_sha256": CORPUS_FP,
        },
    )
    with pytest.raises(module.EvaluationError, match="interleaved|count/order"):
        module.run(case.args, core=case.core)
    assert case.calls == []


def test_plan_rejects_loader_fingerprint_masquerading_as_shared_tensor_identity(case):
    case.plan["arms"][module.ARMS[1]]["dataset"]["split_fingerprint_sha256"]["val"] = "f" * 64
    write_json(case.plan_path, case.plan)
    case.args.plan_sha256 = sha(case.plan_path)
    with pytest.raises(module.EvaluationError, match="share one full VAL tensor fingerprint"):
        module.run(case.args, core=case.core)
    assert case.calls == []
