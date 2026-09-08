"""HH_260906 - Reject explicit admission denial without changing legacy data, features or approval claims."""

import json
from pathlib import Path

import pytest

from portable_e2e import contract as common10
from portable_e2e import dataset as loading
from tests.test_portable_e2e_contract import _sha, _write_dataset, _write_json


# HH_260906 - Enumerate the reviewed paths independently of the production implementation.
NATIVE_PATHS = (
    (), ("native_capture_contract",), ("native_capture_contract", "goal_stop_profile"),
    ("native_capture_contract", "goal_stop_profile", "effective_control"),
    ("native_capture_contract", "agent_initialization"), ("native_result",),
    ("native_result", "goal_stop_quality"), ("native_result", "goal_stop_quality", "config"),
    ("native_result", "goal_stop_quality", "driving_final_stop"),
    ("native_result", "goal_stop_quality", "development_pilot"),
)
ORIGINAL_PATHS = tuple(tuple(part.removeprefix("native_") for part in path)
                       for path in NATIVE_PATHS if path)


def marker_at(path, key, value):
    result = {}
    current = result
    for part in path:
        current = current.setdefault(part, {})
    current[key] = value
    return result


def amend_metadata(root: Path, target: str, addition=None, *, raw_text=None):
    """HH_260906 - Keep every synthetic provenance digest consistent while changing the requested metadata only."""
    dataset_path = root / "dataset.json"
    dataset = json.loads(dataset_path.read_text())
    episode_path = root / dataset["episodes"][0]["manifest"]
    episode = json.loads(episode_path.read_text())
    if target == "dataset":
        dataset.update(addition)
    elif target == "episode":
        episode.update(addition)
    else:
        path = episode_path.parent / episode["source_provenance"][target + "_file"]
        if raw_text is None:
            value = json.loads(path.read_text())
            value.update(addition)
            _write_json(path, value)
        else:
            path.write_text(raw_text)
        episode["source_provenance"][target + "_sha256"] = _sha(path)
    _write_json(episode_path, episode)
    dataset["episodes"][0]["sha256"] = _sha(episode_path)
    _write_json(dataset_path, dataset)


@pytest.mark.parametrize("path", NATIVE_PATHS + ORIGINAL_PATHS)
@pytest.mark.parametrize("key,value", [("training_data_approved", False), ("development_only", True)])
def test_every_reviewed_collection_container_rejects_explicit_denial(path, key, value):
    with pytest.raises(common10.ContractError, match="explicitly prohibits"):
        common10._require_admission_not_denied(marker_at(path, key, value), "collection",
                                               include_native_metadata=True)


@pytest.mark.parametrize("key", ["training_data_approved", "development_only"])
@pytest.mark.parametrize("value", [0, 1, None, "false", "true", [], {}])
def test_admission_markers_must_be_actual_booleans(key, value):
    with pytest.raises(common10.ContractError, match="must be a Boolean"):
        common10._require_admission_not_denied(
            marker_at(("native_capture_contract", "agent_initialization"), key, value),
            "collection", include_native_metadata=True,
        )


@pytest.mark.parametrize("target", ["dataset", "episode", "source_manifest", "collection_config"])
@pytest.mark.parametrize("mode", ["planning", "runtime", "schema"])
@pytest.mark.parametrize("key,value", [("training_data_approved", False), ("development_only", True)])
def test_hash_bound_root_denial_is_rejected_in_every_mode_before_sample_reads(
    tmp_path, monkeypatch, target, mode, key, value,
):
    root = tmp_path / "corpus"
    _write_dataset(root)
    amend_metadata(root, target, {key: value})
    def unexpected_payload_read(*args, **kwargs):
        pytest.fail("denied metadata must fail before sample payload loading")
    monkeypatch.setattr(common10, "_iter_jsonl", unexpected_payload_read)
    with pytest.raises(common10.ContractError, match="explicitly prohibits"):
        common10.validate_dataset(root, mode=mode)


@pytest.mark.parametrize("path", NATIVE_PATHS[1:])
def test_correctly_hashed_converted_native_denial_is_not_dropped(tmp_path, path):
    root = tmp_path / "corpus"
    _write_dataset(root)
    amend_metadata(root, "collection_config", marker_at(path, "training_data_approved", False))
    with pytest.raises(common10.ContractError, match="explicitly prohibits"):
        loading.load_training_examples(root, split="train", mode="schema")


@pytest.mark.parametrize("payload,message", [
    ('{"training_data_approved":true,"training_data_approved":false}', "duplicate JSON key"),
    ('{"training_data_approved":NaN}', "non-standard JSON number"),
    ('{"training_data_approved":"false"}', "must be a Boolean"),
    ('{"development_only":null}', "must be a Boolean"),
    ('[]', "root must be a JSON object"), ('{', "invalid JSON"),
])
def test_hash_bound_collection_json_uses_existing_strict_parser(tmp_path, payload, message):
    root = tmp_path / "corpus"
    _write_dataset(root)
    amend_metadata(root, "collection_config", raw_text=payload)
    with pytest.raises(common10.ContractError, match=message):
        common10.validate_dataset(root, mode="schema")


def test_legacy_absence_preserves_reports_features_fingerprints_and_original_bytes(tmp_path, monkeypatch):
    root = tmp_path / "corpus"
    _write_dataset(root)
    before = {path.relative_to(root): _sha(path) for path in root.rglob("*") if path.is_file()}
    enforced = loading.load_training_examples(root, split="train", mode="schema")
    monkeypatch.setattr(common10, "_require_admission_not_denied", lambda *args, **kwargs: None)
    monkeypatch.setattr(loading, "_require_admission_not_denied", lambda *args, **kwargs: None)
    historical = loading.load_training_examples(root, split="train", mode="schema")
    assert enforced == historical
    assert "training_data_approved" not in enforced.validation_report
    assert before == {path.relative_to(root): _sha(path) for path in root.rglob("*") if path.is_file()}


def test_non_denial_and_opaque_diagnostic_flags_are_not_inferred_as_dataset_approval(tmp_path):
    root = tmp_path / "corpus"
    _write_dataset(root)
    amend_metadata(root, "collection_config", {
        "training_data_approved": True, "development_only": False,
        "dataset_admission": False,
        "agent_initialization": {"training_data_approved": False},
        "native_runtime": {"development_only": True},
        "native_result": {"wall_timing": {"dataset_admission": False, "training_data_approved": False}},
    })
    result = loading.load_training_examples(root, split="train", mode="schema")
    assert len(result.examples) == 11
    assert "training_data_approved" not in result.validation_report


@pytest.mark.parametrize("split", ["train", "val"])
def test_loader_rechecks_collection_hash_after_full_validation(tmp_path, monkeypatch, split):
    root = tmp_path / "corpus"
    _write_dataset(root, episodes=(("episode_selected", split),))
    validate = loading.validate_dataset
    def validate_then_change(*args, **kwargs):
        report = validate(*args, **kwargs)
        path = root / "episodes/episode_selected/collection_config.json"
        _write_json(path, {"training_data_approved": False})
        return report
    monkeypatch.setattr(loading, "validate_dataset", validate_then_change)
    with pytest.raises(common10.ContractError, match="changed after validation"):
        loading.load_training_examples(root, split=split, mode="schema")


@pytest.mark.parametrize("target", ["dataset", "episode", "source_manifest", "collection_config"])
def test_loader_independently_checks_its_reloaded_markers(tmp_path, monkeypatch, target):
    root = tmp_path / "corpus"
    _write_dataset(root)
    amend_metadata(root, target, {"training_data_approved": False})
    # HH_260906 - A deliberately bypassed validator in this test proves the loader's second guard is not dead code.
    monkeypatch.setattr(loading, "validate_dataset", lambda *args, **kwargs:
                        {"dataset_manifest_sha256": _sha(root / "dataset.json")})
    with pytest.raises(common10.ContractError, match="explicitly prohibits"):
        loading.load_training_examples(root, split="train", mode="schema")


def test_previously_reproduced_planning_false_pass_now_rejects_all_301_examples(tmp_path):
    root = tmp_path / "corpus"
    _write_dataset(root, sample_count=301)
    assert len(loading.load_training_examples(root, split="train", mode="planning").examples) == 301
    amend_metadata(root, "collection_config", {"native_result": {"training_data_approved": False}})
    with pytest.raises(common10.ContractError, match="explicitly prohibits"):
        loading.load_training_examples(root, split="train", mode="planning")
