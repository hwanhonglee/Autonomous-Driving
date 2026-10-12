#!/usr/bin/env python3
"""HH_260906 - Copy exact moving-data A/B research views; never train or resplit data."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import sys

# HH_260906 - An archived builder may run outside git; select and verify the explicit native repo before imports.
_bootstrap = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
_bootstrap.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[2])
ROOT = (_bootstrap.parse_known_args()[0].repo_root if __name__ == "__main__"
        else Path(__file__).resolve().parents[2]).absolute()
if ".." in ROOT.parts or ROOT != ROOT.resolve() or not ROOT.is_dir() or any(p.is_symlink() for p in (ROOT, *ROOT.parents)):
    raise ValueError("canonical nonsymlink native repository required")
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
# HH_260906 - Archived entrypoints explicitly bootstrap a canonical checkout when selected.
if (ROOT/'tools/workspace_python.py').is_file() and (ROOT/'learning/portable_e2e').is_dir():
    import runpy
    runpy.run_path(str(ROOT/'tools/workspace_python.py'))['configure'](ROOT)
from portable_e2e import contract as c
from portable_e2e import dataset as data
from scripts.e2e import verify_tree_manifest as tree
if any(not Path(module.__file__).resolve().is_relative_to(ROOT) for module in (c, data, tree)):
    raise ValueError("native imports escaped the explicit repository")

MANIFEST_SHA256 = {
    "baseline": "c73900cedce870a0f55663fde08b3a64b90fae139506fbe2099fdc8bf991c4a9",
    "moving_train": "0cd658a8406ca3785844ab5cd0b44ac28d24466d1f3bc428b24cc541ce32db4c",
    "moving_dev": "07e39fe513d4f90b815613e8dc45e5a27e10adf0f6266ccc2cf7f02bda6b47ff",
}
SOURCE_COUNTS = {"baseline": {"train": 1147, "val": 337},
                 "moving_train": {"train": 328}, "moving_dev": {"val": 328}}
SOURCE_EPISODES = {"baseline": 4, "moving_train": 1, "moving_dev": 1}
MAP_SPLITS = {"Town07": "train", "Town01": "train", "C_track_1_0_7": "train", "Town03": "val"}
ARM_A, ARM_B = "A_baseline_train", "B_town07_augmented_train"
ARM_ROLES = {ARM_A: ("baseline", "moving_dev"), ARM_B: ("baseline", "moving_dev", "moving_train")}
SCHEMA = "portable_e2e.moving_data_ab_views.v1"
# HH_260906 - Canonicalize owned source files while keeping input symlink rejection unchanged.
SOURCE_PATHS = (Path(__file__).resolve(), Path(c.__file__).resolve(), Path(data.__file__).resolve(), Path(tree.__file__).resolve(),
                Path(c.__file__).resolve().with_name('__init__.py'), c.DEFAULT_CONTRACT_PATH.resolve())
DENIALS = dict(training_executed=False, gpu_used=False, model_or_vehicle_release_approved=False,
    dataset_admission_approved=False, training_data_approved=False, new_quality_approval=False,
    labels_modified=False, split_reassignment=False, test_references_included=False,
    test_payloads_opened=False, checkpoint_payloads_opened=False, source_datasets_modified=False,
    hardlinks_created=False, symlinks_created=False, automatic_retry=False, automatic_cleanup=False,
    optimizer_resume_allowed=False, remote_writes_performed=False)


def require(value, message):
    if not value:
        raise c.ContractError(message)


def safe(path):
    path = Path(path).absolute()
    require(".." not in path.parts and path == path.resolve()
            and not any(p.is_symlink() for p in (path, *path.parents)), "canonical nonsymlink path required")
    return path


def regular(path):
    path = safe(path)
    require(path.is_file() and path.stat().st_nlink == 1, "regular single-link input/output file required")
    return path


def sha(path):
    return c.sha256_file(regular(path))


def encode(document):
    return (json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def write_new(path, payload):
    path = safe(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(payload)


def child(root, name):
    require(type(name) is str and name and not Path(name).is_absolute()
            and ".." not in Path(name).parts and Path(name).as_posix() == name,
            "contained canonical relative reference required")
    path = safe(root / name)
    require(path.is_relative_to(root), "reference escaped source root")
    return path


def read_json(path, expected, inputs):
    path = regular(path)
    value, actual = c._read_json_and_sha256(path)
    require(actual == c._sha256(expected, "input SHA256"), "input SHA256 mismatch: " + str(path))
    require(str(path) not in inputs or inputs[str(path)] == actual, "input changed between reads")
    inputs[str(path)] = actual
    return value


def source_identity():
    return {str(path): sha(path) for path in SOURCE_PATHS}


def metadata_preflight(roots, inputs):
    """HH_260906 - Check all six episode identities/splits before touching any episode payload."""
    manifests, episodes, identities, claims = {}, [], set(), {}
    group_members, sessions = defaultdict(list), set()
    contract = c.load_contract()
    for role, root in roots.items():
        manifest = read_json(root / "dataset.json", MANIFEST_SHA256[role], inputs)
        c._require_admission_not_denied(manifest, "source dataset")
        require(manifest.get("schema_id") == c.DATASET_SCHEMA_ID and manifest.get("contract_id") == c.CONTRACT_ID
                and manifest.get("contract_sha256") == c.contract_fingerprint(contract)
                and manifest.get("camera_order") == list(c.CAMERA_ORDER)
                and manifest.get("split_policy") == contract["split_policy"], "fixed Common10 manifest required")
        refs = manifest.get("episodes")
        require(type(refs) is list and len(refs) == SOURCE_EPISODES[role], "source episode count differs")
        counts = Counter()
        for ref in refs:
            eid = ref.get("episode_id")
            require(type(eid) is str and re.fullmatch(r"[A-Za-z0-9_.-]+", eid)
                    and eid not in identities, "unique safe episode ID required")
            require(not re.search(r"town(?:04|15)(?:[^a-z0-9]|$)", eid.lower()), "protected episode reference refused")
            require(ref.get("manifest") == f"episodes/{eid}/episode.json", "fixed episode reference required")
            path = child(root, ref["manifest"])
            episode = read_json(path, ref.get("sha256"), inputs)
            require(episode.get("split") in ("train", "val"), "TEST/unknown split refused before payload reads")
            require(episode.get("map_id") in MAP_SPLITS
                    and MAP_SPLITS[episode["map_id"]] == episode["split"]
                    and episode.get("site_id") == episode["map_id"], "active map split policy differs")
            require(episode.get("schema_id") == c.EPISODE_SCHEMA_ID and episode.get("domain") == "carla"
                    and episode.get("episode_id") == eid, "episode identity differs")
            c._require_admission_not_denied(episode, "source episode")
            count = c._integer(episode.get("sample_count"), "sample_count", 1)
            counts[episode["split"]] += count
            if role != "baseline":
                require(episode["map_id"] == ("Town07" if role == "moving_train" else "Town03"),
                        "moving source map differs")
            for field in ("episode_id", "source_session_id", "split_group_id", "scene_group_id"):
                c._claim_split(claims, field, c._string(episode.get(field), field), episode["split"])
            session = episode["source_session_id"]
            require(session not in sessions, "duplicate source session across episodes")
            sessions.add(session)
            members = group_members[episode["split_group_id"]]
            require(not members or episode["split"] == "train", "repeated route group allowed only within TRAIN")
            members.append(dict(episode_id=eid, split=episode["split"], source_role=role))
            identities.add(eid)
            episodes.append(dict(role=role, root=root, path=path, reference=ref, episode=episode))
        require(dict(counts) == SOURCE_COUNTS[role], "source split sample counts differ")
        require(type(manifest.get("rigs")) is list and len(manifest["rigs"]) == 1, "one shared rig required")
        manifests[role] = manifest
    rig_refs = [manifest["rigs"][0] for manifest in manifests.values()]
    require(all(ref == rig_refs[0] for ref in rig_refs), "source rig references differ")
    return manifests, episodes, {key: value for key, value in group_members.items() if len(value) > 1}


def gather_closure(roots, manifests, episodes, inputs):
    """HH_260906 - Enumerate only manifest-referenced files; unreferenced source trees are never traversed."""
    copies = {role: {} for role in roots}
    sample_ids, source_ids, images = {}, {}, defaultdict(list)
    source_frames = set()

    def add(role, path, expected):
        path = regular(path)
        relative = path.relative_to(roots[role]).as_posix()
        expected = c._sha256(expected, "copy SHA256")
        require(relative not in copies[role] or copies[role][relative] == expected, "conflicting copy reference")
        copies[role][relative] = expected

    for role, root in roots.items():
        ref = manifests[role]["rigs"][0]
        require(ref["manifest"] == f"rigs/{ref['rig_id']}.json", "fixed rig reference required")
        path = child(root, ref["manifest"])
        read_json(path, ref["sha256"], inputs)
        add(role, path, ref["sha256"])
        if role != "baseline":
            # HH_260906 - Retain conversion receipts in provenance, without relabelling them as new admission.
            receipt_path = root / "moving_conversion_qualification.json"
            receipt = read_json(receipt_path, sha(receipt_path), inputs)
            require(receipt.get("status") == "QUALIFIED_COMMON10_PLANNING_CONVERSION"
                    and receipt.get("qualified_for_common10_planning_conversion") is True
                    and receipt.get("dataset_manifest_sha256") == MANIFEST_SHA256[role]
                    and receipt.get("test_examples") == 0
                    and all(receipt.get(split + "_examples") == SOURCE_COUNTS[role].get(split, 0)
                            for split in ("train", "val")), "moving conversion receipt differs")
    for item in episodes:
        role, path, episode = item["role"], item["path"], item["episode"]
        add(role, path, item["reference"]["sha256"])
        provenance = episode["source_provenance"]
        metadata = ((episode["route_geometry_file"], episode["route_geometry_sha256"], False),
                    (provenance["source_manifest_file"], provenance["source_manifest_sha256"], False),
                    (provenance["collection_config_file"], provenance["collection_config_sha256"], True))
        for name, expected, native in metadata:
            payload_path = child(path.parent, name)
            document = read_json(payload_path, expected, inputs)
            c._require_admission_not_denied(document, "source metadata", include_native_metadata=native)
            add(role, payload_path, expected)
        samples_path = child(path.parent, episode["sample_jsonl"])
        regular(samples_path)
        expected = c._sha256(episode["sample_jsonl_sha256"], "sample SHA256")
        count = 0
        for _, sample in c._iter_jsonl(samples_path, expected_sha256=expected):
            require(sample.get("episode_id") == episode["episode_id"], "sample episode identity differs")
            owner = dict(split=episode["split"], episode_id=episode["episode_id"], source_role=role)
            for field, seen in (("sample_id", sample_ids), ("source_sample_id", source_ids)):
                identity = c._string(sample.get(field), field)
                require(identity not in seen, "duplicate " + field + " across selected samples")
                seen[identity] = owner
            cameras = sample.get("camera_bundle")
            require(type(cameras) is list and tuple(camera.get("name") for camera in cameras) == c.CAMERA_ORDER,
                    "six fixed camera identities required")
            for camera in cameras:
                image_sha = c._sha256(camera.get("sha256"), "image SHA256")
                previous = images[image_sha]
                require(not previous or previous[0]["split"] == episode["split"], "image SHA256 crosses TRAIN/VAL splits")
                previous.append(dict(owner, sample_id=sample["sample_id"], camera=camera["name"]))
                frame = (episode["source_session_id"], camera["name"], camera.get("source_frame_id"))
                require(frame not in source_frames, "duplicate native source frame")
                source_frames.add(frame)
                add(role, child(path.parent, camera["path"]), image_sha)
            count += 1
        require(count == episode["sample_count"], "sample JSONL count differs")
        inputs[str(samples_path)] = expected
        add(role, samples_path, expected)
    overlap = dict(status="PASS", unique_sample_tokens=len(sample_ids), unique_source_sample_ids=len(source_ids),
        unique_image_sha256=len(images), image_reference_count=sum(len(v) for v in images.values()),
        duplicate_sample_token_count=0, duplicate_source_sample_id_count=0, cross_split_image_sha256_count=0,
        repeated_image_sha256={k: v for k, v in images.items() if len(v) > 1},
        unique_native_source_frames=len(source_frames),
        scope="Union of source episodes used by B; A is a subset. Reused A/B evaluation samples are intentional.")
    return copies, overlap


def view_inventory(root, expected):
    report = tree.build_tree_manifest(root)
    require(report["status"] == "PASS" and report["valid"], "output tree inventory failed")
    require({row["path"]: row["sha256"] for row in report["files"]} == expected, "output closure or bytes differ")
    identities = set()
    for row in report["files"]:
        metadata = regular(root / row["path"]).stat()
        identity = (metadata.st_dev, metadata.st_ino)
        require(identity not in identities, "output files share an inode")
        identities.add(identity)
    return {key: report[key] for key in ("manifest_sha256", "file_count", "directory_count", "total_size_bytes", "files")}


def prepare_views(baseline_root, moving_train_root, moving_dev_root, output_parent):
    roots = dict(zip(MANIFEST_SHA256, map(safe, (baseline_root, moving_train_root, moving_dev_root))))
    output = safe(output_parent)
    require(not output.exists(), "fresh output parent required")
    for root in roots.values():
        require(root.is_dir() and not output.is_relative_to(root) and not root.is_relative_to(output),
                "source and output trees must be disjoint")
    require(len(set(roots.values())) == 3, "three distinct source roots required")
    sources, inputs = source_identity(), {}
    manifests, episodes, repeated_groups = metadata_preflight(roots, inputs)
    copies, overlap = gather_closure(roots, manifests, episodes, inputs)
    now = datetime.now(timezone.utc).isoformat()
    counts = {arm: dict(sum((Counter(SOURCE_COUNTS[role]) for role in roles), Counter()))
              for arm, roles in ARM_ROLES.items()}
    plan = dict(schema=SCHEMA, status="DECLARED_NOT_BUILT", declared_at_utc=now,
        source_roots={role: str(root) for role, root in roots.items()}, source_manifest_sha256=MANIFEST_SHA256,
        source_sha256=sources, output_parent=str(output), arms={arm: dict(source_roles=list(roles), counts=counts[arm])
            for arm, roles in ARM_ROLES.items()}, retained_train_group_overlap=repeated_groups,
        unchanged_episode_references={role: manifests[role]["episodes"] for role in roots}, **DENIALS)
    output.mkdir(parents=True, exist_ok=False)
    write_new(output / "plan.json", encode(plan))
    result = dict(plan, status="BUILDING", plan_sha256=sha(output / "plan.json"), overlap_audit=overlap, arms={})
    try:
        for arm, roles in ARM_ROLES.items():
            partial = output / (arm + ".partial")
            partial.mkdir(exist_ok=False)
            expected = {}
            for role in roles:
                for name, expected_sha in copies[role].items():
                    if name in expected:
                        require(expected[name] == expected_sha, "arm copy path collision")
                        continue
                    source = regular(roots[role] / name)
                    maximum = c.MAX_JPEG_FILE_BYTES if source.suffix.lower() in (".jpg", ".jpeg") else c.MAX_JSON_FILE_BYTES
                    payload = c._read_regular_file_bounded(source, maximum, "A/B byte copy")
                    require(hashlib.sha256(payload).hexdigest() == expected_sha, "copy input changed after declaration")
                    write_new(partial / name, payload)
                    source_stat, target_stat = source.stat(), (partial / name).stat()
                    require(sha(partial / name) == expected_sha
                            and (source_stat.st_dev, source_stat.st_ino) != (target_stat.st_dev, target_stat.st_ino),
                            "copy is not independent and byte exact")
                    inputs[str(source)] = expected_sha
                    expected[name] = expected_sha
            manifest = dict(manifests["baseline"], dataset_id=f"carla-moving-data-ab-{arm.lower()}-research-v1",
                created_at_utc=now, episodes=[item["reference"] for role in roles for item in episodes if item["role"] == role])
            write_new(partial / "dataset.json", encode(manifest))
            expected["dataset.json"] = sha(partial / "dataset.json")
            validation = c.validate_dataset(partial, mode="planning", check_image_hashes=True)
            require(validation["status"] == "PASS" and validation["split_sample_counts"] == counts[arm],
                    "full planning validation counts differ")
            fingerprints = {}
            for split in ("train", "val"):
                loaded = data.load_training_examples(partial, split=split, mode="planning", check_image_hashes=True)
                require(len(loaded.examples) == counts[arm][split]
                        and loaded.validation_report["dataset_fingerprint_sha256"] == validation["dataset_fingerprint_sha256"],
                        "real loader count/fingerprint differs")
                fingerprints[split] = dict(count=len(loaded.examples), loader_sha256=loaded.fingerprint_sha256,
                    examples_sha256=data.fingerprint_examples(loaded.examples),
                    tokens_sha256=hashlib.sha256(encode([example.token for example in loaded.examples])).hexdigest())
            result["arms"][arm] = dict(output_root=str(output / arm), validation_root=str(partial),
                validation=validation, counts=counts[arm], fingerprints=fingerprints,
                episode_ids={split: [item["episode"]["episode_id"] for role in roles for item in episodes
                                    if item["role"] == role and item["episode"]["split"] == split]
                             for split in ("train", "val")},
                dataset_manifest_sha256=expected["dataset.json"], tree=view_inventory(partial, expected))
        require(result["arms"][ARM_A]["fingerprints"]["val"]["examples_sha256"]
                == result["arms"][ARM_B]["fingerprints"]["val"]["examples_sha256"], "A/B DEV example content differs")
        require(result["arms"][ARM_A]["fingerprints"]["val"]["tokens_sha256"]
                == result["arms"][ARM_B]["fingerprints"]["val"]["tokens_sha256"], "A/B DEV membership/order differs")
        for path, expected_sha in inputs.items():
            require(sha(path) == expected_sha, "source changed during A/B preparation")
        require(source_identity() == sources, "builder/validator/loader source changed")
        source_inventory = {}
        for role, root in roots.items():
            entries = [dict(path=Path(path).relative_to(root).as_posix(), sha256=digest, size=regular(path).stat().st_size)
                       for path, digest in sorted(inputs.items()) if Path(path).is_relative_to(root)]
            source_inventory[role] = dict(files=entries, file_count=len(entries),
                total_size_bytes=sum(entry["size"] for entry in entries),
                closure_json_sha256=hashlib.sha256(encode(entries)).hexdigest())
        result.update(status="VALIDATED_AB_RESEARCH_VIEWS_NOT_NEW_ADMISSION", source_input_sha256=inputs,
            source_input_inventory=source_inventory,
            source_inputs_unchanged=True, copied_payloads_byte_exact=True, shared_val_content_identical=True,
            loader_fingerprint_scope="Loader hashes include each whole-dataset manifest; compare examples_sha256 for shared VAL content.",
            inventory_scope="Output trees contain exactly referenced Common10 files. Source inputs cover selected metadata/payloads and moving qualification receipts; no recursive source scan.")
        for arm in ARM_ROLES:
            require(not (output / arm).exists(), "final arm path appeared during preparation")
            (output / (arm + ".partial")).rename(output / arm)
        write_new(output / "report.json", encode(result))
        return result
    except BaseException as error:
        result.update(status="FAIL_PARTIALS_RETAINED", error=f"{type(error).__name__}: {error}",
            source_input_sha256=inputs, retained_paths=[str(path) for path in output.iterdir()])
        if not (output / "report.json").exists():
            write_new(output / "report.json", encode(result))
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--baseline-root", type=Path, required=True)
    parser.add_argument("--moving-train-root", type=Path, required=True)
    parser.add_argument("--moving-dev-root", type=Path, required=True)
    parser.add_argument("--output-parent", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    args = parser.parse_args(argv)
    require(safe(args.repo_root) == ROOT, "repository override must be selected before native imports")
    report = prepare_views(args.baseline_root, args.moving_train_root, args.moving_dev_root, args.output_parent)
    print(json.dumps(dict(status=report["status"], report=str(args.output_parent / "report.json"),
        report_sha256=sha(args.output_parent / "report.json"), counts={arm: row["counts"] for arm, row in report["arms"].items()})))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
