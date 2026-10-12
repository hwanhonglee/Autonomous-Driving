"""HH_260906 - Verify exact A/B copies and fail-closed boundaries using synthetic Common10 sources."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys

import pytest

from portable_e2e import contract as c
from scripts.e2e import prepare_portable_moving_data_ab_views as ab
# HH_260906 - Use the canonical fixture namespace after root test-alias retirement.
from learning.tests import test_portable_e2e_contract as fixture


def repin(roots, monkeypatch):
    pins = {role: fixture._sha(root / "dataset.json") for role, root in roots.items()}
    monkeypatch.setattr(ab, "MANIFEST_SHA256", pins)
    for role in ("moving_train", "moving_dev"):
        fixture._write_json(roots[role] / "moving_conversion_qualification.json", dict(
            status="QUALIFIED_COMMON10_PLANNING_CONVERSION", qualified_for_common10_planning_conversion=True,
            dataset_manifest_sha256=pins[role], train_examples=ab.SOURCE_COUNTS[role].get("train", 0),
            val_examples=ab.SOURCE_COUNTS[role].get("val", 0), test_examples=0))


def rewrite_episode(root, index, change):
    manifest = json.loads((root / "dataset.json").read_text())
    ref = manifest["episodes"][index]
    path = root / ref["manifest"]
    episode = json.loads(path.read_text())
    change(episode, path.parent)
    fixture._write_json(path, episode)
    ref["sha256"] = fixture._sha(path)
    fixture._write_json(root / "dataset.json", manifest)


def make_sources(tmp_path, monkeypatch, count=11):
    roots = {role: tmp_path / role for role in ab.MANIFEST_SHA256}
    old_sample, old_write = fixture._sample, fixture._write_json

    def sample(folder, eid, index, **kwargs):
        row = old_sample(folder, eid, index, **kwargs)
        # HH_260906 - Distinct JPEG comment bytes avoid accidental synthetic TRAIN/VAL payload collisions.
        comment = eid.encode()
        marker = b"\xff\xfe" + (len(comment) + 2).to_bytes(2, "big") + comment
        for camera in row["camera_bundle"]:
            path = folder / camera["path"]
            payload = path.read_bytes()
            path.write_bytes(payload[:2] + marker + payload[2:])
            camera["sha256"] = fixture._sha(path)
        if eid == "new_dev":
            row["ego"]["position_m"][1] = 40.0
        return row

    def write(path, document):
        # HH_260906 - Give the new DEV episode a distinct route while preserving causal local navigation.
        if path.name == "route_geometry.json" and "new_dev" in path.parts:
            document = dict(document, polyline_m=[[x, y + 40.0] for x, y in document["polyline_m"]])
        old_write(path, document)

    with monkeypatch.context() as local:
        local.setattr(fixture, "_sample", sample)
        local.setattr(fixture, "_write_json", write)
        fixture._write_dataset(roots["baseline"], episodes=(("train_a", "train"), ("train_b", "train"),
            ("train_c", "train"), ("old_dev", "val")), sample_count=count)
        fixture._write_dataset(roots["moving_train"], episodes=(("new_train", "train"),), sample_count=count)
        fixture._write_dataset(roots["moving_dev"], episodes=(("new_dev", "val"),), sample_count=count)

    def map_change(town):
        def change(episode, folder):
            episode["map_id"] = episode["site_id"] = town
            semantic = c._route_geometry_semantic_hash(folder / episode["route_geometry_file"],
                episode=episode, contract=c.load_contract(), expected_sha256=episode["route_geometry_sha256"])
            episode["split_group_id"] = f"carla:{town}:{semantic}"
            episode["scene_group_id"] = f"{episode['split_group_id']}:traffic=17:weather=ClearNoon"
        return change

    for index, town in enumerate(("Town07", "Town01", "C_track_1_0_7", "Town03")):
        rewrite_episode(roots["baseline"], index, map_change(town))
    rewrite_episode(roots["moving_dev"], 0, map_change("Town03"))
    monkeypatch.setattr(ab, "SOURCE_COUNTS", {"baseline": {"train": count * 3, "val": count},
        "moving_train": {"train": count}, "moving_dev": {"val": count}})
    repin(roots, monkeypatch)
    return roots


def prepare(roots, output):
    return ab.prepare_views(roots["baseline"], roots["moving_train"], roots["moving_dev"], output)


def test_real_planning_and_loaders_produce_independent_identical_dev_views(tmp_path, monkeypatch):
    roots = make_sources(tmp_path, monkeypatch, count=301)
    output = tmp_path / "views"
    result = prepare(roots, output)
    assert result["status"] == "VALIDATED_AB_RESEARCH_VIEWS_NOT_NEW_ADMISSION"
    assert result["arms"][ab.ARM_A]["counts"] == {"train": 903, "val": 602}
    assert result["arms"][ab.ARM_B]["counts"] == {"train": 1204, "val": 602}
    assert result["shared_val_content_identical"] is True
    assert result["retained_train_group_overlap"]
    all_inodes = set()
    for arm, roles in ab.ARM_ROLES.items():
        row = result["arms"][arm]
        assert row["validation"]["qualification"]["common_10hz_planning"] == "PASS"
        assert row["validation"]["image_hashes_checked"] is True
        assert row["episode_ids"]["val"] == ["old_dev", "new_dev"]
        assert not (output / (arm + ".partial")).exists()
        assert len(row["tree"]["files"]) == row["tree"]["file_count"]
        for entry in row["tree"]["files"]:
            path = output / arm / entry["path"]
            assert fixture._sha(path) == entry["sha256"]
            metadata = path.stat()
            assert metadata.st_nlink == 1 and not path.is_symlink()
            assert (metadata.st_dev, metadata.st_ino) not in all_inodes
            all_inodes.add((metadata.st_dev, metadata.st_ino))
            if entry["path"] != "dataset.json":
                origin = next(root / entry["path"] for role, root in roots.items()
                              if role in roles and (root / entry["path"]).is_file())
                assert path.read_bytes() == origin.read_bytes()
                assert path.stat().st_ino != origin.stat().st_ino
    assert result["arms"][ab.ARM_A]["fingerprints"]["val"]["examples_sha256"] == result["arms"][ab.ARM_B]["fingerprints"]["val"]["examples_sha256"]
    assert result["arms"][ab.ARM_A]["fingerprints"]["val"]["loader_sha256"] != result["arms"][ab.ARM_B]["fingerprints"]["val"]["loader_sha256"]
    assert all(result[field] is False for field in ab.DENIALS)
    assert result["overlap_audit"]["unique_sample_tokens"] == 1806
    assert result["source_inputs_unchanged"] is result["copied_payloads_byte_exact"] is True
    assert fixture._sha(output / "plan.json") == result["plan_sha256"]
    assert json.loads((output / "report.json").read_text())["status"] == result["status"]


@pytest.mark.parametrize("change", [dict(split="test"), dict(map_id="Town04"), dict(map_id="Town15"),
    dict(split="train"), dict(training_data_approved=False), dict(sample_count=12),
    dict(source_session_id="session:train_a")])
def test_invalid_metadata_fails_before_any_payload_read(tmp_path, monkeypatch, change):
    roots = make_sources(tmp_path, monkeypatch)
    rewrite_episode(roots["moving_dev"], 0, lambda episode, _: episode.update(change))
    repin(roots, monkeypatch)
    original = ab.read_json
    def guard(path, *args, **kwargs):
        assert Path(path).name in ("dataset.json", "episode.json")
        return original(path, *args, **kwargs)
    monkeypatch.setattr(ab, "read_json", guard)
    with pytest.raises(c.ContractError):
        prepare(roots, tmp_path / "views")
    assert not (tmp_path / "views").exists()


def test_protected_reference_refused_without_opening_episode(tmp_path, monkeypatch):
    roots = make_sources(tmp_path, monkeypatch)
    manifest = json.loads((roots["moving_dev"] / "dataset.json").read_text())
    manifest["episodes"][0]["episode_id"] = "carla_town04-secret"
    manifest["episodes"][0]["manifest"] = "episodes/carla_town04-secret/episode.json"
    fixture._write_json(roots["moving_dev"] / "dataset.json", manifest)
    repin(roots, monkeypatch)
    with pytest.raises(c.ContractError, match="protected episode"):
        prepare(roots, tmp_path / "views")


@pytest.mark.parametrize("field", ["sample_id", "source_sample_id"])
def test_duplicate_sample_identifiers_rejected_before_outputs(tmp_path, monkeypatch, field):
    roots = make_sources(tmp_path, monkeypatch)
    prefix = "" if field == "sample_id" else "source:"
    fixture._mutate_first_sample(roots["moving_train"], lambda row: row.update({field: prefix + "train_a:000000"}))
    repin(roots, monkeypatch)
    with pytest.raises(c.ContractError, match="duplicate " + field):
        prepare(roots, tmp_path / "views")
    assert not (tmp_path / "views").exists()


def test_cross_split_image_sha_rejected_before_outputs(tmp_path, monkeypatch):
    roots = make_sources(tmp_path, monkeypatch)
    sample = json.loads((roots["baseline"] / "episodes/train_a/samples.jsonl").read_text().splitlines()[0])
    fixture._mutate_first_sample(roots["moving_dev"], lambda row: row["camera_bundle"][0].update(sha256=sample["camera_bundle"][0]["sha256"]))
    repin(roots, monkeypatch)
    with pytest.raises(c.ContractError, match="image SHA256 crosses"):
        prepare(roots, tmp_path / "views")


@pytest.mark.parametrize("kind", ["existing", "inside_source", "symlink", "hardlink", "manifest_hash", "rig"])
def test_unsafe_paths_and_changed_inputs_fail_closed(tmp_path, monkeypatch, kind):
    roots = make_sources(tmp_path, monkeypatch)
    output = tmp_path / "views"
    if kind == "existing":
        output.mkdir()
    elif kind == "inside_source":
        output = roots["baseline"] / "views"
    elif kind == "symlink":
        alias = tmp_path / "alias"
        alias.symlink_to(roots["baseline"], target_is_directory=True)
        roots["baseline"] = alias
    elif kind == "hardlink":
        (tmp_path / "linked.json").hardlink_to(roots["baseline"] / "dataset.json")
    elif kind == "manifest_hash":
        with (roots["baseline"] / "dataset.json").open("a") as stream:
            stream.write(" ")
    else:
        path = roots["moving_dev"] / "dataset.json"
        document = json.loads(path.read_text())
        document["rigs"][0]["sha256"] = "0" * 64
        fixture._write_json(path, document)
        repin(roots, monkeypatch)
    with pytest.raises(c.ContractError):
        prepare(roots, output)


def test_planning_failure_retains_partial_and_failure_receipt(tmp_path, monkeypatch):
    roots = make_sources(tmp_path, monkeypatch)
    output = tmp_path / "views"
    with pytest.raises(c.ContractError, match="duration"):
        prepare(roots, output)
    assert (output / (ab.ARM_A + ".partial")).is_dir()
    assert not (output / ab.ARM_A).exists()
    report = json.loads((output / "report.json").read_text())
    assert report["status"] == "FAIL_PARTIALS_RETAINED"
    assert report["automatic_cleanup"] is False


def test_copy_change_retains_partial_without_success(tmp_path, monkeypatch):
    roots = make_sources(tmp_path, monkeypatch)
    original = c._read_regular_file_bounded
    def changed(path, maximum, context):
        payload = original(path, maximum, context)
        return payload + b"changed" if context == "A/B byte copy" and Path(path).suffix == ".jpg" else payload
    monkeypatch.setattr(c, "_read_regular_file_bounded", changed)
    output = tmp_path / "views"
    with pytest.raises(c.ContractError, match="copy input changed"):
        prepare(roots, output)
    assert json.loads((output / "report.json").read_text())["status"] == "FAIL_PARTIALS_RETAINED"


def test_archived_cli_imports_explicit_repo_without_pythonpath(tmp_path):
    archived = tmp_path / "builder.py"
    archived.write_bytes(Path(ab.__file__).read_bytes())
    result = subprocess.run([sys.executable, "-I", "-B", str(archived), "--repo-root", str(ab.ROOT), "--help"],
        cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "--baseline-root" in result.stdout and "--repo-root" in result.stdout
