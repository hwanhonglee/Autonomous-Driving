#!/usr/bin/env python3
"""HH_260906 - Copy the frozen legacy v3 TRAIN/VAL closure without opening TEST payloads."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from portable_e2e import contract as c
from portable_e2e import dataset as data

SOURCE_MANIFEST_SHA256 = "18262e5aa4abbb3e03e35e379b5da1e5ce7fd339a9a8942e02b58ca737f7242c"
DATASET_ID = "carla-common10-trainval-research-20260914-v1"
COUNTS = {"train": 1147, "val": 337, "test": 309}
EPISODE_COUNTS = {"train": 3, "val": 1, "test": 1}
SCHEMA = "portable_e2e.trainval_research_view.v1"
# HH_260906 - Resolve owned module locations only; dataset inputs must remain canonical on entry.
SOURCE_PATHS = (Path(__file__).resolve(), Path(c.__file__).resolve(), Path(data.__file__).resolve(), c.DEFAULT_CONTRACT_PATH.resolve())
DENIALS = dict(training_data_approved=False, new_quality_approval=False, vehicle_control_approved=False,
    labels_modified=False, split_reassignment=False, test_payloads_opened=False, test_targets_opened=False,
    test_images_opened=False, test_provenance_opened=False, fresh_fits_only=True, optimizer_resume_allowed=False)


def require(value, message):
    if not value:
        raise c.ContractError(message)


def safe_path(path):
    path = Path(path).absolute()
    require(".." not in path.parts and not any(p.is_symlink() for p in (path, *path.parents)),
            "canonical nonsymlink paths required")
    return path


def digest(path):
    return c.sha256_file(safe_path(path))


def source_identity():
    return {str(path.resolve()): digest(path) for path in SOURCE_PATHS}


def encode(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def write_new(path, payload):
    safe_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(payload)


def prepare_view(source_root, output_root, report_path):
    source, output, report_path = map(safe_path, (source_root, output_root, report_path))
    require(source.is_dir(), "source dataset directory required")
    source = source.resolve(strict=True)
    partial = output.with_name(output.name + ".partial")
    for destination in (output, partial, report_path):
        require(not destination.exists(), "fresh output, partial and report paths required")
        require(not destination.is_relative_to(source) and not source.is_relative_to(destination),
                "source and outputs must be disjoint")
    require(not report_path.is_relative_to(output) and not report_path.is_relative_to(partial)
            and not output.is_relative_to(report_path) and not partial.is_relative_to(report_path),
            "report must be outside the native view")
    sources = source_identity()
    inputs, copies = {}, {}

    def read_json(path, expected):
        path = safe_path(path)
        require(path.is_relative_to(source), "metadata escaped source dataset")
        value, actual = c._read_json_and_sha256(path)
        require(actual == c._sha256(expected, "metadata SHA256"), "source metadata SHA256 mismatch")
        name = path.relative_to(source).as_posix()
        require(name not in inputs or inputs[name] == actual, "source metadata changed between reads")
        inputs[name] = actual
        return value

    def add_copy(path, expected):
        path = safe_path(path)
        require(path.is_relative_to(source), "copy escaped source dataset")
        name = path.relative_to(source).as_posix()
        require(name not in copies or copies[name] == expected, "conflicting copy reference")
        copies[name] = c._sha256(expected, "copy SHA256")

    original = read_json(source / "dataset.json", SOURCE_MANIFEST_SHA256)
    c._require_admission_not_denied(original, "source dataset")
    require(set(original) == {"schema_id", "contract_id", "contract_sha256", "dataset_id", "created_at_utc",
            "camera_order", "split_policy", "rigs", "episodes"}, "only the frozen v3 manifest layout is supported")
    contract = c.load_contract()
    require(original["schema_id"] == c.DATASET_SCHEMA_ID and original["contract_id"] == c.CONTRACT_ID
            and original["contract_sha256"] == c.contract_fingerprint(contract)
            and original["camera_order"] == list(c.CAMERA_ORDER) and original["split_policy"] == contract["split_policy"],
            "frozen native Common10 contract required")
    selected, omitted, claims, counts, episode_counts, identities = [], [], {}, Counter(), Counter(), set()
    # HH_260906 - First inspect only the five pinned episode manifests; never resolve a TEST payload reference.
    for ref in c._sequence(original["episodes"], "episodes"):
        path = c._safe_file(source, ref.get("manifest"), "episode manifest")
        episode = read_json(path, ref.get("sha256"))
        eid = c._string(episode.get("episode_id"), "episode_id")
        require(ref.get("episode_id") == eid and eid not in identities
                and ref["manifest"] == f"episodes/{eid}/episode.json", "fixed contained episode identity required")
        identities.add(eid)
        c._require_admission_not_denied(episode, "episode metadata")
        require(episode.get("schema_id") == c.EPISODE_SCHEMA_ID and episode.get("domain") == "carla",
                "only frozen CARLA episodes are supported")
        split = episode.get("split")
        require(split in COUNTS, "unreviewed source split")
        count = c._integer(episode.get("sample_count"), "sample_count", 1)
        counts[split] += count
        episode_counts[split] += 1
        for field in ("episode_id", "source_session_id", "split_group_id", "scene_group_id"):
            c._claim_split(claims, field, c._string(episode.get(field), field), split)
        if split == "test":
            omitted.append(dict(episode_id=eid, episode_manifest_sha256=ref["sha256"], declared_sample_count=count,
                                payloads_opened=False))
            continue
        selected.append((ref, path, episode))
    require(dict(counts) == COUNTS and dict(episode_counts) == EPISODE_COUNTS, "frozen v3 split counts differ")
    require(len(original["rigs"]) == 1, "exactly the frozen CARLA rig is required")
    rig_ref = original["rigs"][0]
    rig_path = c._safe_file(source, rig_ref.get("manifest"), "rig manifest")
    require(rig_ref["manifest"] == f"rigs/{rig_ref.get('rig_id')}.json", "contained rig layout required")
    rig = read_json(rig_path, rig_ref.get("sha256"))
    require(rig.get("domain") == "carla" and rig.get("rig_id") == rig_ref["rig_id"], "frozen CARLA rig required")
    add_copy(rig_path, rig_ref["sha256"])
    for ref, path, episode in selected:
        add_copy(path, ref["sha256"])
        provenance = c._mapping(episode.get("source_provenance"), "source_provenance")
        for name, expected, native in ((episode.get("route_geometry_file"), episode.get("route_geometry_sha256"), False),
                (provenance.get("source_manifest_file"), provenance.get("source_manifest_sha256"), False),
                (provenance.get("collection_config_file"), provenance.get("collection_config_sha256"), True)):
            child = c._safe_file(path.parent, name, "selected episode metadata")
            value = read_json(child, expected)
            c._require_admission_not_denied(value, "selected metadata", include_native_metadata=native)
            add_copy(child, expected)
        sample_path = c._safe_file(path.parent, episode.get("sample_jsonl"), "selected samples")
        sample_pin = c._sha256(episode.get("sample_jsonl_sha256"), "samples SHA256")
        sample_count = 0
        for _, sample in c._iter_jsonl(sample_path, expected_sha256=sample_pin):
            require(sample.get("episode_id") == episode["episode_id"], "sample episode identity differs")
            cameras = c._sequence(sample.get("camera_bundle"), "camera_bundle")
            require(tuple(camera.get("name") for camera in cameras) == c.CAMERA_ORDER, "six fixed cameras required")
            for camera in cameras:
                image = c._safe_file(path.parent, camera.get("path"), "selected camera image")
                add_copy(image, camera.get("sha256"))
            sample_count += 1
        require(sample_count == episode["sample_count"], "sample count differs from frozen metadata")
        inputs[sample_path.relative_to(source).as_posix()] = sample_pin
        add_copy(sample_path, sample_pin)

    report = dict(schema=SCHEMA, status="COPYING", created_at_utc=datetime.now(timezone.utc).isoformat(),
        source_root=str(source), output_root=str(output), source_dataset_manifest_sha256=SOURCE_MANIFEST_SHA256,
        source_dataset_id=original["dataset_id"], counts={key: COUNTS[key] for key in ("train", "val")},
        selected_episode_references=[ref for ref, _, _ in selected], omitted_test_metadata=omitted,
        source_sha256=sources, source_input_sha256=inputs, **DENIALS)
    partial.mkdir(parents=True, exist_ok=False)
    try:
        for name, expected in copies.items():
            path = safe_path(source / name)
            maximum = c.MAX_JPEG_FILE_BYTES if name.lower().endswith((".jpg", ".jpeg")) else c.MAX_JSON_FILE_BYTES
            payload = c._read_regular_file_bounded(path, maximum, "selected copy input")
            require(hashlib.sha256(payload).hexdigest() == expected, "copy input changed after reference validation")
            write_new(partial / name, payload)
            require(digest(partial / name) == expected, "copied bytes differ")
            inputs[name] = expected
        # HH_260906 - Preserve all original admission-bearing metadata; only the root identity/time/episode list changes.
        view = dict(original, dataset_id=DATASET_ID, created_at_utc=report["created_at_utc"],
                    episodes=[ref for ref, _, _ in selected])
        write_new(partial / "dataset.json", encode(view))
        validation = c.validate_dataset(partial, mode="planning", check_image_hashes=True)
        require(validation["status"] == "PASS" and validation["split_sample_counts"] == report["counts"],
                "unchanged full native planning validation failed")
        report.update(native_validation=validation, corpus_fingerprint_sha256=validation["dataset_fingerprint_sha256"],
                      split_example_fingerprint_sha256={}, split_loader_fingerprint_sha256={}, split_tensor_fingerprint_sha256={})
        for split in ("train", "val"):
            loaded = data.load_training_examples(partial, split=split, mode="planning", check_image_hashes=True)
            require(len(loaded.examples) == COUNTS[split]
                    and loaded.validation_report["dataset_fingerprint_sha256"] == report["corpus_fingerprint_sha256"],
                    "validated split identity changed")
            example_sha = data.fingerprint_examples(loaded.examples)
            report["split_example_fingerprint_sha256"][split] = example_sha
            report["split_loader_fingerprint_sha256"][split] = loaded.fingerprint_sha256
            report["split_tensor_fingerprint_sha256"][split] = hashlib.sha256((example_sha + "\0" + split).encode()).hexdigest()
        output_files = {name: digest(partial / name) for name in copies}
        output_files["dataset.json"] = digest(partial / "dataset.json")
        require(all(output_files[name] == expected for name, expected in copies.items()), "view bytes changed during validation")
        require(all(digest(source / name) == expected for name, expected in inputs.items()), "source bytes changed during copy")
        require(source_identity() == sources, "builder or native contract source changed")
        report.update(status="VALIDATED_LEGACY_RESEARCH_VIEW_NOT_NEW_ADMISSION", output_file_sha256=output_files,
            output_dataset_manifest_sha256=output_files["dataset.json"], source_inputs_unchanged=True,
            copied_payloads_byte_exact=True, fingerprint_change_declared=True,
            interpretation="Legacy planning-contract compatibility is not new quality/admission approval. No TEST payload was opened. "
                "Only TRAIN/VAL references remain; no resplit, row filtering, label rewriting or hardlinks. Fresh fits only; no optimizer resume.")
        require(not output.exists() and not report_path.exists(), "output appeared while preparing view")
        partial.rename(output)
        write_new(report_path, encode(report))
        return report
    except BaseException as error:
        report.update(status="FAIL", error=f"{type(error).__name__}: {error}", retained_partial_root=str(partial))
        if not report_path.exists():
            write_new(report_path, encode(report))
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    result = prepare_view(args.source_root, args.output_root, args.report)
    print(json.dumps({key: result[key] for key in ("status", "counts", "output_dataset_manifest_sha256")}))


if __name__ == "__main__":
    main()
