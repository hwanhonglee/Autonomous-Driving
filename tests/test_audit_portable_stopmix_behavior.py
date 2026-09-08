"""HH_260906 - Test synthetic validation diagnostics without launching simulation, training or a GPU."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

torch = pytest.importorskip("torch")
from scripts.e2e import audit_portable_stopmix_behavior as module
from portable_e2e.contract import ContractError
from portable_e2e.dataset import FEATURE_NAMES
from portable_e2e.model import ModelConfig, PHYSICAL_MODEL_ID, PHYSICAL_STOPMIX_MODEL_ID


def example(index=0, current=1.0, speed=None, valid_count=64):
    features = [0.] * 13
    features[FEATURE_NAMES.index("velocity_x_mps")] = current
    target_speed = list(speed if speed is not None else [1.] * 64)
    return SimpleNamespace(token=f"synthetic_{index}", episode_id="synthetic_validation",
        sequence_index=index, anchor_timestamp_ns=1000000000 + index * 100000000,
        features=tuple(features), targets_xy=tuple((.1 * (i + 1), 0.) if i < valid_count else None for i in range(64)),
        target_speed_mps=tuple(v if i < valid_count else None for i, v in enumerate(target_speed)),
        camera_sha256=tuple("a" * 64 for _ in range(6)), source_manifest_sha256="b" * 64)


def sample_arguments(index=0, model_id=PHYSICAL_MODEL_ID, current=1., target_speed=None, valid_count=64):
    count = 12 if model_id == PHYSICAL_STOPMIX_MODEL_ID else 6
    path = [[.1 * (i + 1), 0.] for i in range(64)]
    return dict(index=index, example=example(index, current, target_speed, valid_count),
        xy=[copy.deepcopy(path) for _ in range(count)], speed=[[1.] * 64 for _ in range(count)],
        logits=list(range(count)), target_xy=path, target_speed=target_speed or [1.] * 64,
        valid=[i < valid_count for i in range(64)], model_id=model_id)


@pytest.mark.parametrize("current,speeds,valid,expected", [
    (0., [0.] * 64, [True] * 64, "stationary_hold"),
    (-.1, [.1] * 64, [True] * 64, "stationary_hold"),
    (.10000000001, [.1] * 64, [True] * 64, "moving_to_stop"),
    (1., [1.] * 54 + [.1] * 10, [True] * 64, "moving_to_stop"),
    (0., [.5] * 64, [True] * 64, "continuing_motion"),
    (-1., [0.] * 64, [True] * 64, "other_motion"),
    (0., [0.] * 63 + [.10000000001], [True] * 64, "other_motion"),
    (0., [0.] + [None] * 63, [True] + [False] * 63, "unavailable_masks"),
    (0., [.49] * 64, [True] * 64, "other_motion"),
])
def test_fixed_group_priority_and_raw_boundary(current, speeds, valid, expected):
    assert module.motion_group(current, speeds, valid) == expected


@pytest.mark.parametrize("field,value", [("current", True), ("current", float("nan")),
    ("speeds", [float("inf")] * 64), ("speeds", [-.1] * 64), ("speeds", [True] * 64),
    ("valid", [1] * 64), ("valid", [False] * 64), ("valid", [True, False, True] + [False] * 61),
    ("valid", [True] * 63)])
def test_malformed_group_inputs_reject(field, value):
    values = {"current": 0., "speeds": [0.] * 64, "valid": [True] * 64}
    values[field] = value
    with pytest.raises(ContractError):
        module.motion_group(values["current"], values["speeds"], values["valid"])


def test_future_zero_does_not_use_current_anchor_and_exact_zero_is_separate():
    launch = module.speed_behavior([.1 * (i + 1) for i in range(64)])
    assert launch["first_exact_zero_future_index"] is None
    assert not launch["reacceleration_after_exact_future_zero"]
    decel = module.speed_behavior([1.] * 32 + [0.] * 32)
    assert decel["terminal_exact_zero"] and decel["first_exact_zero_future_index"] == 32
    assert not decel["reacceleration_after_exact_future_zero"]
    assert module.speed_behavior([0., .01] + [0.] * 62)["reacceleration_after_exact_future_zero"]
    low = module.speed_behavior([.01] * 64)
    assert low["terminal_at_or_below_0p1_mps"] and not low["terminal_exact_zero"]


@pytest.mark.parametrize("model_id,count", [(PHYSICAL_MODEL_ID, 6), (PHYSICAL_STOPMIX_MODEL_ID, 12)])
def test_all_candidates_metrics_and_stop_family_applicability(model_id, count):
    row = module.analyze_sample(**sample_arguments(model_id=model_id))
    assert row["selected_candidate_index"] == count - 1
    assert len(row["candidates"]) == count
    assert row["selected_stop_candidate"] is (True if count == 12 else None)
    assert row["selected_ade_m"] == row["selected_fde_m"] == row["selected_speed_mae_mps"] == 0
    assert row["ade_selection_regret_m"] == 0
    assert row["runtime_geometry"]["selected_geometry_pass"] is True
    result = module.summarize_rows([row], count)
    assert result["all_samples"]["sample_count"] == 1
    assert result["target_motion_groups"]["stationary_hold"]["sample_count"] == 0
    assert result["target_motion_groups"]["stationary_hold"]["metrics"] is None
    assert result["all_samples"]["stop_selection_count"] == (1 if count == 12 else None)


def test_masked_prefix_metrics_do_not_use_invalid_tail():
    arguments = sample_arguments(valid_count=10)
    for path in arguments["xy"]:
        path[10:] = [[1e5, 1e5]] * 54
    row = module.analyze_sample(**arguments)
    assert row["valid_future_points"] == 10 and row["target_motion_group"] == "unavailable_masks"
    assert row["selected_ade_m"] == row["selected_fde_m"] == 0.
    assert row["runtime_geometry"]["selected_geometry_pass"] is False


def test_original_raw_current_speed_gate_failure_remains():
    row = module.analyze_sample(**sample_arguments(index=108, current=8.4, model_id=PHYSICAL_STOPMIX_MODEL_ID))
    assert row["raw_current_vx_mps"] == 8.4
    assert not row["runtime_geometry"]["selected_geometry_pass"]
    assert "speed" in row["runtime_geometry"]["selected_failure_codes"]


@pytest.mark.parametrize("change", ["count", "nan_logits", "nan_xy", "nan_speed", "mask", "id"])
def test_prediction_or_target_contract_rejects(change):
    arguments = sample_arguments()
    if change == "count": arguments["xy"].pop()
    elif change == "nan_logits": arguments["logits"][0] = float("nan")
    elif change == "nan_xy": arguments["xy"][0][0][0] = float("nan")
    elif change == "nan_speed": arguments["speed"][0][0] = float("inf")
    elif change == "mask": arguments["valid"][-1] = False
    else: arguments["model_id"] = "unsupported"
    with pytest.raises(ContractError): module.analyze_sample(**arguments)


def test_partial_or_duplicate_summary_rows_reject():
    row = module.analyze_sample(**sample_arguments())
    with pytest.raises(ContractError): module.summarize_rows([], 6)
    with pytest.raises(ContractError): module.summarize_rows([{**row, "index": 1}], 6)
    with pytest.raises(ContractError): module.summarize_rows([row, {**row, "index": 1}], 6)


def test_output_rejects_direct_dataset_alias_target_and_checkpoint_tree(tmp_path, monkeypatch):
    repository, actual = tmp_path / "repo", tmp_path / "data_actual"
    repository.mkdir(); actual.mkdir()
    (repository / "datasets").symlink_to(actual, target_is_directory=True)
    checkpoint = tmp_path / "checkpoints/last.pt"
    checkpoint.parent.mkdir(); checkpoint.write_bytes(b"fixture")
    monkeypatch.setattr(module, "ROOT", repository)
    for output in (actual / "outside-selected-corpus", repository / "datasets/new", checkpoint.parent / "new"):
        with pytest.raises(ContractError): module.checked_output(output, actual / "selected", checkpoint)
    fresh = tmp_path / "outputs/new"
    assert module.checked_output(fresh, actual / "selected", checkpoint) == fresh
    fresh.mkdir(parents=True)
    with pytest.raises(ContractError): module.checked_output(fresh, actual / "selected", checkpoint)


@pytest.fixture
def fake_execution(tmp_path, monkeypatch):
    # HH_260906 - A reduced synthetic five-row corpus exercises the real writer; production constants remain 337 and twelve fixed plots.
    corpus = tmp_path / "corpus"; corpus.mkdir()
    (corpus / "dataset.json").write_text("{}")
    checkpoint = tmp_path / "checkpoints/last.pt"; checkpoint.parent.mkdir(); checkpoint.write_bytes(b"synthetic-only")
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "")
    monkeypatch.setattr(module, "DATASET_SHA256", module.digest(corpus / "dataset.json"))
    monkeypatch.setattr(module, "EXPECTED_SAMPLES", 5)
    monkeypatch.setattr(module, "RENDER_INDICES", (0, 2, 4))
    sources = {name: "1" * 64 for name in module.SOURCE_PATHS}
    monkeypatch.setattr(module, "source_identity", lambda commit: sources.copy())
    samples = tuple(example(index) for index in range(5))
    loaded = SimpleNamespace(examples=samples, fingerprint_sha256="2" * 64,
        validation_report={"dataset_fingerprint_sha256": "3" * 64})
    calls = []
    def load(*args, **kwargs):
        calls.append(kwargs)
        return loaded
    monkeypatch.setattr(module, "load_training_examples", load)
    model_id = PHYSICAL_STOPMIX_MODEL_ID
    cfg = ModelConfig.from_mapping(json.loads((module.ROOT / module.CONFIGS[model_id]).read_text()))
    monkeypatch.setattr(module, "_read_checkpoint_for_audit", lambda **kwargs: ({}, cfg, ("train1", "train2", "train3"),
        {"checkpoint_sha256": kwargs["expected_checkpoint_sha256"]}))
    class Dataset:
        def __init__(self, examples, config, **kwargs):
            self.examples, self.config, self.fingerprint_sha256 = examples, config, "4" * 64
        def __len__(self): return len(self.examples)
        def __getitem__(self, index):
            item = self.examples[index]
            return {"sample_id": item.token, "images": torch.tensor([float(index)]), "calibration": torch.zeros(1),
                "ego_history": torch.tensor([item.features], dtype=torch.float32), "ego_history_mask": torch.ones(1, dtype=torch.bool),
                "route_xy": torch.tensor([[0., 0.], [10., 0.]]), "route_mask": torch.ones(2, dtype=torch.bool),
                "target_xy": torch.tensor(item.targets_xy), "target_speed_mps": torch.tensor(item.target_speed_mps),
                "target_valid": torch.ones(64, dtype=torch.bool)}
    monkeypatch.setattr(module, "Common10TorchDataset", Dataset)
    forward_calls = []
    class Model:
        def __call__(self, *inputs):
            assert len(inputs) == 6
            forward_calls.append(tuple(tuple(t.shape) for t in inputs))
            batch = inputs[0].shape[0]
            xy = torch.tensor([[[.1 * (i + 1), 0.] for i in range(64)]] * cfg.candidate_count).unsqueeze(0).repeat(batch, 1, 1, 1)
            return xy, torch.ones(batch, cfg.candidate_count, 64), torch.zeros(batch, cfg.candidate_count)
    monkeypatch.setattr(module, "_validate_checkpoint_and_model", lambda *args, **kwargs: (Model(), {"model_parameter_count": 1056362}))
    args = SimpleNamespace(dataset=corpus, checkpoint=checkpoint, checkpoint_sha256=module.digest(checkpoint),
        expected_source_commit="5" * 40, expected_model_id=model_id, output_dir=tmp_path / "output",
        device="cpu", batch_size=2)
    return args, calls, forward_calls, sources


def test_synthetic_end_to_end_exact_rows_all_fixed_plots_and_provenance(fake_execution):
    args, calls, forward_calls, sources = fake_execution
    report = module.audit(args)
    assert report["status"] == "COMPLETE_NOT_PROMOTED"
    assert report["all_samples"]["sample_count"] == 5
    assert report["all_samples"]["geometry"]["sample_count"] == 5
    assert [r["index"] for r in report["renders"]] == [0, 2, 4]
    assert len(forward_calls) == 3
    assert calls == [dict(split="val", mode="planning", check_image_hashes=True)] * 2
    assert report["source_sha256"] == sources
    assert all(path in report["source_sha256"] for path in (*module.CONFIGS.values(), "portable_e2e/stop_primitive_research.py"))
    rows = [json.loads(line) for line in (args.output_dir / "samples.jsonl").read_text().splitlines()]
    assert [r["index"] for r in rows] == list(range(5))
    assert "test files" in report["integrity_scope"]
    assert report["test_inference_or_optimization_or_selection"] is False
    assert report["vehicle_control_approved"] is False
    for line in (args.output_dir / "SHA256SUMS").read_text().splitlines():
        sha, name = line.split("  ")
        assert module.digest(args.output_dir / name) == sha
    assert str(args.dataset) not in (args.output_dir / "summary.json").read_text()


def test_render_failure_preserves_completed_row_and_fails_closed(fake_execution, monkeypatch):
    args, *_ = fake_execution
    def fail(*args, **kwargs): raise OSError("synthetic renderer failure")
    monkeypatch.setattr(module, "render_trajectory_png", fail)
    with pytest.raises(OSError): module.audit(args)
    failure = json.loads((args.output_dir / "failed.json").read_text())
    assert failure["status"] == "INCOMPLETE" and failure["completed_sample_count"] == 1
    assert len((args.output_dir / "samples.jsonl").read_text().splitlines()) == 1
    assert not (args.output_dir / "summary.json").exists()
    assert not (args.output_dir / "SHA256SUMS").exists()


@pytest.mark.parametrize("which", ["checkpoint", "source", "corpus"])
def test_postcheck_change_cannot_publish_complete(fake_execution, monkeypatch, which):
    args, calls, _, sources = fake_execution
    original_render = module.render_trajectory_png
    def render(*values, **kwargs):
        original_render(*values, **kwargs)
        if which == "checkpoint": args.checkpoint.write_bytes(b"changed fixture")
        elif which == "source": sources["portable_e2e/model.py"] = "a" * 64
        else: (args.dataset / "dataset.json").write_text("{\"changed\": true}")
    monkeypatch.setattr(module, "render_trajectory_png", render)
    with pytest.raises(ContractError): module.audit(args)
    assert not (args.output_dir / "summary.json").exists()
    assert json.loads((args.output_dir / "failed.json").read_text())["status"] == "INCOMPLETE"


def test_wrong_visibility_and_input_sha_fail_before_dataset_read(fake_execution, monkeypatch):
    args, calls, *_ = fake_execution
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "0")
    with pytest.raises(ContractError): module.audit(args)
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "")
    args.checkpoint_sha256 = "0" * 64
    with pytest.raises(ContractError): module.audit(args)
    assert calls == [] and not args.output_dir.exists()


def test_cli_is_validation_only_and_requires_explicit_identity():
    common = ["data", "--checkpoint", "ckpt", "--checkpoint-sha256", "a" * 64,
        "--expected-source-commit", "b" * 40, "--expected-model-id", PHYSICAL_MODEL_ID, "--output-dir", "out"]
    assert module.parse_args(common).device == "cpu"
    for extra in (["--split", "test"], ["--allow-unapproved"], ["--device", "cuda:1"], ["--render-count", "1"]):
        with pytest.raises(SystemExit): module.parse_args(common + extra)


def test_fixed_production_contract_is_not_the_synthetic_fixture():
    assert module.EXPECTED_SAMPLES == 337
    assert module.RENDER_INDICES == (0, 31, 61, 92, 122, 153, 183, 214, 244, 275, 305, 336)
    assert module.DATASET_SHA256 == "18262e5aa4abbb3e03e35e379b5da1e5ce7fd339a9a8942e02b58ca737f7242c"
    assert set(module.INPUT_KEYS) == {"images", "calibration", "ego_history", "ego_history_mask", "route_xy", "route_mask"}


@pytest.mark.parametrize("failure", [None, "head", "blob", "module_path", "symlink", "short_commit"])
def test_source_proof_requires_exact_commit_bytes_and_loaded_module_path(tmp_path, monkeypatch, failure):
    root = tmp_path / "repo"; (root / "portable_e2e").mkdir(parents=True)
    name = "portable_e2e/model.py"
    file = root / name; file.write_text("# synthetic source fixture\n")
    source_bytes = file.read_bytes()
    sha = "c" * 40
    calls = []
    def run(command, **kwargs):
        calls.append((command, kwargs))
        if "rev-parse" in command:
            return SimpleNamespace(stdout=(("d" * 40 if failure == "head" else sha) + "\n").encode())
        return SimpleNamespace(stdout=b"changed" if failure == "blob" else source_bytes)
    monkeypatch.setattr(module, "ROOT", root)
    monkeypatch.setattr(module, "SOURCE_PATHS", (name,))
    monkeypatch.setattr(module.subprocess, "run", run)
    monkeypatch.setitem(module.sys.modules, "portable_e2e.model", SimpleNamespace(__file__=str(root / "other.py" if failure == "module_path" else file)))
    if failure == "symlink":
        original = root / "original.py"; file.rename(original); file.symlink_to(original)
    if failure:
        with pytest.raises(ContractError): module.source_identity(sha[:10] if failure == "short_commit" else sha)
    else:
        assert module.source_identity(sha) == {name: hashlib.sha256(source_bytes).hexdigest()}
    for command, options in calls:
        assert command[:3] == ["git", "-c", "protocol.allow=never"]
        assert options["env"]["GIT_NO_LAZY_FETCH"] == "1" and options["env"]["GIT_ALLOW_PROTOCOL"] == ""
        assert options["timeout"] == 15


def test_changed_corpus_postscan_rejects_before_summary(fake_execution, monkeypatch):
    args, *_ = fake_execution
    original = module.load_training_examples
    count = 0
    def load(*values, **kwargs):
        nonlocal count
        value = original(*values, **kwargs)
        count += 1
        if count == 2:
            return SimpleNamespace(examples=value.examples, fingerprint_sha256="9" * 64,
                validation_report=value.validation_report)
        return value
    monkeypatch.setattr(module, "load_training_examples", load)
    with pytest.raises(ContractError): module.audit(args)
    assert not (args.output_dir / "summary.json").exists()
    assert json.loads((args.output_dir / "failed.json").read_text())["completed_sample_count"] == 5


def test_changed_source_during_final_corpus_scan_rejects(fake_execution, monkeypatch):
    args, _, _, sources = fake_execution
    original = module.load_training_examples
    count = 0
    def load(*values, **kwargs):
        nonlocal count
        result = original(*values, **kwargs)
        count += 1
        if count == 2: sources["portable_e2e/stop_primitive_research.py"] = "9" * 64
        return result
    monkeypatch.setattr(module, "load_training_examples", load)
    with pytest.raises(ContractError): module.audit(args)
    assert not (args.output_dir / "summary.json").exists()


def test_wrong_model_config_or_validation_count_rejects_before_forward(fake_execution, monkeypatch):
    args, _, calls, _ = fake_execution
    args.expected_model_id = PHYSICAL_MODEL_ID
    with pytest.raises(ContractError): module.audit(args)
    assert calls == [] and not args.output_dir.exists()
    args.expected_model_id = PHYSICAL_STOPMIX_MODEL_ID
    monkeypatch.setattr(module, "EXPECTED_SAMPLES", 6)
    with pytest.raises(ContractError): module.audit(args)
    assert calls == [] and not args.output_dir.exists()


def test_checkpoint_provenance_cannot_overwrite_declared_analysis_scope(fake_execution, monkeypatch):
    args, *_ = fake_execution
    original = module._validate_checkpoint_and_model
    def validated(*values, **kwargs):
        model, evidence = original(*values, **kwargs)
        return model, {**evidence, "source_commit": "d" * 40, "status": "PROMOTED", "candidate_count": 0}
    monkeypatch.setattr(module, "_validate_checkpoint_and_model", validated)
    report = module.audit(args)
    assert report["source_commit"] == args.expected_source_commit
    assert report["status"] == "COMPLETE_NOT_PROMOTED" and report["candidate_count"] == 12
    assert report["checkpoint_validation"]["source_commit"] == "d" * 40
    assert "external campaign receipt" in report["source_commit_scope"]
