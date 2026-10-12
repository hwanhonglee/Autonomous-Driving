"""HH_260906 - Synthetic byte-exact view tests; original datasets are never used or modified."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from portable_e2e import contract as c
from scripts.e2e import prepare_portable_trainval_research_view as view
# HH_260906 - Use the canonical fixture namespace after root test-alias retirement.
from learning.tests import test_portable_e2e_contract as fixture


def repin(root, monkeypatch):
    monkeypatch.setattr(view, "SOURCE_MANIFEST_SHA256", fixture._sha(root / "dataset.json"))


def rewrite_episode(root, index, change):
    manifest = json.loads((root / "dataset.json").read_text())
    ref = manifest["episodes"][index]
    path = root / ref["manifest"]
    episode = json.loads(path.read_text())
    change(episode, path.parent)
    fixture._write_json(path, episode)
    ref["sha256"] = fixture._sha(path)
    fixture._write_json(root / "dataset.json", manifest)


def dataset(tmp_path, monkeypatch, *, count=11):
    root = tmp_path / "source"
    original_sample = fixture._sample
    def distinct_sample(folder, eid, index, **kwargs):
        row = original_sample(folder, eid, index, **kwargs)
        # HH_260906 - Distinct valid JPEG comment bytes prevent synthetic cross-split image reuse.
        comment = eid.encode()
        marker = b"\xff\xfe" + (len(comment) + 2).to_bytes(2, "big") + comment
        for camera in row["camera_bundle"]:
            path = folder / camera["path"]
            payload = path.read_bytes()
            path.write_bytes(payload[:2] + marker + payload[2:])
            camera["sha256"] = fixture._sha(path)
        return row
    with monkeypatch.context() as local:
        local.setattr(fixture, "_sample", distinct_sample)
        fixture._write_dataset(root, episodes=(("train_a", "train"), ("train_b", "train"),
            ("train_c", "train"), ("val_a", "val"), ("test_a", "test")), sample_count=count)
    def independent(index):
        def change(episode, folder):
            episode["map_id"] = episode["site_id"] = f"Town{index:02d}"
            semantic = c._route_geometry_semantic_hash(folder / episode["route_geometry_file"],
                episode=episode, contract=c.load_contract(), expected_sha256=episode["route_geometry_sha256"])
            episode["split_group_id"] = f"carla:{episode['map_id']}:{semantic}"
            episode["scene_group_id"] = f"{episode['split_group_id']}:traffic=17:weather=ClearNoon"
        return change
    rewrite_episode(root, 3, independent(3))
    rewrite_episode(root, 4, independent(4))
    monkeypatch.setattr(view, "COUNTS", {"train": 3 * count, "val": count, "test": count})
    repin(root, monkeypatch)
    return root


def outputs(tmp_path):
    return tmp_path / "view", tmp_path / "receipts" / "view.json"


def test_full_native_view_byte_exact_no_test_reads_even_with_poisoned_missing_payloads(tmp_path, monkeypatch):
    root = dataset(tmp_path, monkeypatch, count=301)
    test_root = root / "episodes/test_a"
    def poison(episode, folder):
        episode["sample_jsonl"] = "/unopened-test-targets"
        episode["route_geometry_file"] = "../../unopened-test-route"
        episode["source_provenance"]["source_manifest_file"] = "/unopened-test-provenance"
        episode["source_provenance"]["collection_config_file"] = "/unopened-test-config"
    rewrite_episode(root, 4, poison)
    for name in ("samples.jsonl", "source_manifest.json", "collection_config.json", "route_geometry.json"):
        (test_root / name).unlink()
    repin(root, monkeypatch)
    original_open = c._open_regular_read_only
    original_safe_file = c._safe_file
    opened = []
    def guard(path, *args, **kwargs):
        path = Path(path)
        assert not path.is_relative_to(test_root) or path.name == "episode.json", str(path)
        opened.append(path)
        return original_open(path, *args, **kwargs)
    def guarded_resolution(parent, *args, **kwargs):
        assert not Path(parent).is_relative_to(test_root), "TEST child paths must not even be resolved"
        return original_safe_file(parent, *args, **kwargs)
    monkeypatch.setattr(c, "_open_regular_read_only", guard)
    monkeypatch.setattr(c, "_safe_file", guarded_resolution)
    output, receipt = outputs(tmp_path)
    result = view.prepare_view(root, output, receipt)
    assert result["status"] == "VALIDATED_LEGACY_RESEARCH_VIEW_NOT_NEW_ADMISSION"
    assert result["counts"] == {"train": 903, "val": 301}
    assert result["native_validation"]["mode"] == "planning"
    assert result["native_validation"]["qualification"]["common_10hz_planning"] == "PASS"
    assert result["native_validation"]["image_hashes_checked"] is True
    assert result["source_inputs_unchanged"] is result["copied_payloads_byte_exact"] is True
    assert result["fresh_fits_only"] is True and result["optimizer_resume_allowed"] is False
    for key in ("training_data_approved", "new_quality_approval", "vehicle_control_approved", "labels_modified",
                "split_reassignment", "test_payloads_opened", "test_targets_opened", "test_images_opened", "test_provenance_opened"):
        assert result[key] is False
    assert not (output / "episodes/test_a").exists()
    assert not output.with_name("view.partial").exists()
    assert all("test_a" not in name for name in result["output_file_sha256"])
    assert [name for name in result["source_input_sha256"] if "test_a" in name] == ["episodes/test_a/episode.json"]
    assert {p.relative_to(output).as_posix() for p in output.rglob("*") if p.is_file()} == set(result["output_file_sha256"])
    for name, expected in result["output_file_sha256"].items():
        assert fixture._sha(output / name) == expected
        if name != "dataset.json":
            assert (output / name).read_bytes() == (root / name).read_bytes()
            assert (output / name).stat().st_ino != (root / name).stat().st_ino
    original = json.loads((root / "dataset.json").read_text())
    written = json.loads((output / "dataset.json").read_text())
    assert written == dict(original, dataset_id=view.DATASET_ID, created_at_utc=result["created_at_utc"], episodes=original["episodes"][:4])
    assert "training_data_approved" not in written
    assert json.loads(receipt.read_text()) == result
    assert c.validate_dataset(output)["dataset_fingerprint_sha256"] == result["corpus_fingerprint_sha256"]
    for split in ("train", "val"):
        loaded = view.data.load_training_examples(output, split=split)
        example_sha = view.data.fingerprint_examples(loaded.examples)
        assert result["split_example_fingerprint_sha256"][split] == example_sha
        assert result["split_loader_fingerprint_sha256"][split] == loaded.fingerprint_sha256
        assert result["split_tensor_fingerprint_sha256"][split] == hashlib.sha256((example_sha + "\0" + split).encode()).hexdigest()


@pytest.mark.parametrize("scope", ["dataset", "episode", "source_manifest", "collection_root", "collection_native"])
def test_explicit_denial_not_erased_or_copied(tmp_path, monkeypatch, scope):
    root = dataset(tmp_path, monkeypatch)
    if scope == "dataset":
        path = root / "dataset.json"
        value = json.loads(path.read_text())
        value["training_data_approved"] = False
        fixture._write_json(path, value)
    else:
        def deny(episode, folder):
            if scope == "episode":
                episode["training_data_approved"] = False
                return
            key = "source_manifest" if scope == "source_manifest" else "collection_config"
            path = folder / episode["source_provenance"][key + "_file"]
            value = json.loads(path.read_text())
            if scope == "collection_native":
                value["native_result"] = {"training_data_approved": False}
            else:
                value["training_data_approved"] = False
            fixture._write_json(path, value)
            episode["source_provenance"][key + "_sha256"] = fixture._sha(path)
        rewrite_episode(root, 0, deny)
    repin(root, monkeypatch)
    output, receipt = outputs(tmp_path)
    with pytest.raises(c.ContractError, match="explicitly prohibits"):
        view.prepare_view(root, output, receipt)
    assert not output.exists() and not output.with_name("view.partial").exists() and not receipt.exists()


@pytest.mark.parametrize("change", ["hash", "split", "missing_hash", "sample_count", "source_group"])
def test_pins_split_counts_and_independence_fail_before_copy(tmp_path, monkeypatch, change):
    root = dataset(tmp_path, monkeypatch)
    if change == "hash":
        monkeypatch.setattr(view, "SOURCE_MANIFEST_SHA256", "0" * 64)
    else:
        def mutate(episode, folder):
            if change == "split":
                episode["split"] = "train"
            elif change == "missing_hash":
                episode["source_provenance"].pop("collection_config_sha256")
            elif change == "sample_count":
                episode["sample_count"] += 1
            else:
                episode["source_session_id"] = "session:train_a"
        rewrite_episode(root, 3, mutate)
        repin(root, monkeypatch)
    output, receipt = outputs(tmp_path)
    with pytest.raises(c.ContractError):
        view.prepare_view(root, output, receipt)
    assert not output.exists() and not output.with_name("view.partial").exists()


@pytest.mark.parametrize("kind", ["existing_output", "existing_partial", "existing_report", "report_in_source", "report_in_output", "output_in_source", "source_symlink", "image_symlink"])
def test_fresh_disjoint_nonsymlink_paths(tmp_path, monkeypatch, kind):
    root = dataset(tmp_path, monkeypatch)
    output, receipt = outputs(tmp_path)
    if kind == "existing_output":
        output.mkdir()
    elif kind == "existing_partial":
        output.with_name("view.partial").mkdir()
    elif kind == "existing_report":
        receipt.parent.mkdir()
        receipt.write_text("preserve me")
    elif kind == "report_in_source":
        receipt = root / "receipt.json"
    elif kind == "report_in_output":
        receipt = output / "receipt.json"
    elif kind == "output_in_source":
        output = root / "view"
    elif kind == "source_symlink":
        alias = tmp_path / "alias"
        alias.symlink_to(root, target_is_directory=True)
        root = alias
    else:
        path = next((root / "episodes/train_a/images").rglob("*.jpg"))
        original = tmp_path / "image.jpg"
        path.rename(original)
        path.symlink_to(original)
    with pytest.raises(c.ContractError):
        view.prepare_view(root, output, receipt)
    if kind == "existing_report":
        assert receipt.read_text() == "preserve me"


def test_native_validation_failure_retains_partial_and_no_success(tmp_path, monkeypatch):
    # HH_260906 - A short, hash-consistent fixture fails the unchanged native 30-second planning requirement.
    root = dataset(tmp_path, monkeypatch)
    output, receipt = outputs(tmp_path)
    with pytest.raises(c.ContractError, match="duration"):
        view.prepare_view(root, output, receipt)
    assert not output.exists() and output.with_name("view.partial").is_dir()
    result = json.loads(receipt.read_text())
    assert result["status"] == "FAIL" and "output_file_sha256" not in result
    assert result["training_data_approved"] is False


def test_copy_input_change_retains_failure(tmp_path, monkeypatch):
    root = dataset(tmp_path, monkeypatch)
    output, receipt = outputs(tmp_path)
    original = c._read_regular_file_bounded
    def changed(path, maximum, context):
        value = original(path, maximum, context)
        if context == "selected copy input" and str(path).endswith(".jpg"):
            return value + b"changed"
        return value
    monkeypatch.setattr(c, "_read_regular_file_bounded", changed)
    with pytest.raises(c.ContractError, match="copy input changed"):
        view.prepare_view(root, output, receipt)
    assert not output.exists()
    assert json.loads(receipt.read_text())["status"] == "FAIL"


def test_invalid_future_mask_is_not_reformatted_or_admitted(tmp_path, monkeypatch):
    root = dataset(tmp_path, monkeypatch)
    def corrupt(row):
        row["labels"]["planning"]["valid"][0] = False
    fixture._mutate_first_sample(root, corrupt)
    repin(root, monkeypatch)
    output, receipt = outputs(tmp_path)
    with pytest.raises(c.ContractError, match="invalid|mask|null"):
        view.prepare_view(root, output, receipt)
    assert not output.exists()
    assert json.loads(receipt.read_text())["status"] == "FAIL"


def test_cli_cannot_override_source_pin_or_resume():
    with pytest.raises(SystemExit):
        view.main(["--source-root", "/source", "--output-root", "/output", "--report", "/report", "--resume"])
