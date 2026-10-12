"""HH_261002 - Matched physical-slot OFF versus balanced OFF/ON research contract."""

from __future__ import annotations

import hashlib
import io
import json
from pathlib import Path

import numpy as np

try:
    from . import map_appearance_matched_contract_20261001 as frozen
except ImportError:
    import map_appearance_matched_contract_20261001 as frozen

NAME = "map_paired_balanced_20261002_v1"
DATA_SCHEMA = "portable_e2e.map_paired_balanced_data.20261002.v1"
REGISTRY_SCHEMA = "portable_e2e.map_paired_balanced_research_registration.20261002.v1"
REVIEW_SCHEMA = "portable_e2e.map_paired_balanced_review_authorization.20261002.v1"
SOURCE_SCHEMA = "portable_e2e.map_paired_balanced_source.20261002.v1"
LABELS = frozen.LABELS
ORIGINAL_COUNT, PAIRED_COUNT, TOTAL_COUNT = 729, 480, 1209
STEPS, BATCH, SEED = 10000, 8, 20261002
LR, WEIGHT_DECAY = frozen.LR, frozen.WEIGHT_DECAY
ORIGINAL_SHA, PARENT_SHA = frozen.ORIGINAL_SHA, frozen.PARENT_SHA
PARENT_CHECKPOINT_SHA = frozen.PARENT_CHECKPOINT_SHA
ARCHITECTURE_SHA = frozen.ARCHITECTURE_SHA
FROZEN_SOURCE_SHA = {
    "map_appearance_matched_contract_20261001.py": "9fb7b057a09d955a8952699f3a5868a8f6a6d04cdf78ea66ada7eea742c53b96",
    "train_map_appearance_matched_20261001.py": "22c1564b3c8e50e671f6ffb480fae23ca8107b6ef0c09ce3b6a4ba3aa2117968",
    **ARCHITECTURE_SHA,
    "train_map_photometric_pilot.py": "4de567323d3964ec877da831ec2acb08e445fe77c9da659f052edcf52ca53484",
}
ROUTES = {
    "p02": "788665af9e3b5449634f1c5392e94b2a6258ae3262d26e69f3cf81b96791489a",
    "p04": "f4d474eee01892e16b8b78a69de5a3b32ccc8e4f227e30a9a89dc023dbfc25a7",
}
QA_SHA = {
    "p02": "2763f42908822b19465a1978e0f840a143efa7658613a7e945f7b935a0fea2ef",
    "p04": "0a38d8fbafdadb224b71d81fd2f6645286f7f523d32a6dd25e4231c993cfe473",
}
COUNTS = {"original": 729, "physical_pairs": 480, "new_rgb_views": 960,
          "available_rgb_views": 1689, "logical_slots_per_arm": 1209}
ARMS = {"control": "original729+new480_always_FALSE",
        "balanced": "original729+same_new480_predeclared_balanced_FALSE_TRUE"}
SOURCE_FILES = {"train_map_paired_balanced_20261002.py", "map_paired_balanced_contract_20261002.py",
                *FROZEN_SOURCE_SHA, "plan.md"}
require, apply_crop, common_weights = frozen.require, frozen.apply_crop, frozen.common_weights


def canonical(path: Path) -> Path:
    """HH_261002 - Reject lexical escapes and every symlink ancestor, even for fresh outputs."""
    path = Path(path)
    require(path.is_absolute() and ".." not in path.parts, "absolute unescaped path required")
    require(not any(part.is_symlink() for part in (path, *path.parents)), "symlink path forbidden")
    require(path.resolve() == path, "canonical path required")
    return path


def payload(path: Path, expected_sha: str | None = None) -> tuple[bytes, str]:
    canonical(path)
    require(path.is_file(), "regular file required: " + str(path))
    value = path.read_bytes()
    sha = hashlib.sha256(value).hexdigest()
    require(expected_sha is None or sha == expected_sha, "pinned bytes differ: " + str(path))
    return value, sha


def digest(path: Path) -> str:
    canonical(path)
    require(path.is_file(), "regular file required: " + str(path))
    state = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            state.update(chunk)
    return state.hexdigest()


def read_json(path: Path, expected_sha: str | None = None):
    value, _ = payload(path, expected_sha)
    return json.loads(value)


def read_npz(path: Path, expected_sha: str) -> dict:
    value, _ = payload(path, expected_sha)
    with np.load(io.BytesIO(value), allow_pickle=False) as archive:
        require(len(archive.files) == len(set(archive.files)), "duplicate archive keys")
        return {name: archive[name].copy() for name in archive.files}


def write_new(path: Path, value) -> None:
    canonical(path)
    require(not path.exists(), "create-only output already exists: " + str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def schedule():
    """HH_261002 - Each physical pair has one slot; alternating ON/OFF visits never add slots."""
    indices, crops = frozen.schedule(SEED)
    rng = np.random.default_rng(SEED + 2)
    initial = np.concatenate([rng.permutation(np.tile(np.asarray([0, 1], dtype=np.uint8), 120))
                              for _ in ROUTES])
    visits = np.zeros(PAIRED_COUNT, dtype=np.int64)
    bits = np.zeros(indices.size, dtype=np.uint8)
    for position, index in enumerate(indices.ravel()):
        if index >= ORIGINAL_COUNT:
            pair = int(index) - ORIGINAL_COUNT
            bits[position] = initial[pair] ^ (int(visits[pair]) % 2)
            visits[pair] += 1
    return indices, crops, bits.reshape(STEPS, BATCH)


def schedule_proof(indices, crops, bits) -> dict:
    require(indices.shape == (STEPS, BATCH) and indices.dtype == np.int64, "indices ABI differs")
    require(crops.shape == (STEPS, BATCH, 4) and crops.dtype == np.int16, "crops ABI differs")
    require(bits.shape == (STEPS, BATCH) and bits.dtype == np.uint8, "view-bit ABI differs")
    generated = schedule()
    require(all(np.array_equal(actual, expected) for actual, expected in zip((indices, crops, bits), generated)),
            "predeclared indices/crops/balanced view schedule differs")
    flat, views = indices.ravel(), bits.ravel()
    require(bool((views[flat < ORIGINAL_COUNT] == 0).all()), "original replay exposure changed")
    off = np.bincount(flat[(flat >= ORIGINAL_COUNT) & (views == 0)] - ORIGINAL_COUNT,
                      minlength=PAIRED_COUNT)
    on = np.bincount(flat[(flat >= ORIGINAL_COUNT) & (views == 1)] - ORIGINAL_COUNT,
                     minlength=PAIRED_COUNT)
    require(bool((np.abs(off - on) <= 1).all()), "unbalanced physical-pair views")
    return {"indices_sha256": hashlib.sha256(indices.tobytes()).hexdigest(),
            "crops_sha256": hashlib.sha256(crops.tobytes()).hexdigest(),
            "balanced_view_bits_sha256": hashlib.sha256(bits.tobytes()).hexdigest(),
            "control_view_bits_sha256": hashlib.sha256(np.zeros_like(bits).tobytes()).hexdigest(),
            "sample_exposures": np.bincount(flat, minlength=TOTAL_COUNT).tolist(),
            "original_exposures": int((flat < ORIGINAL_COUNT).sum()),
            "paired_exposures": int((flat >= ORIGINAL_COUNT).sum()),
            "balanced_FALSE_exposures_per_pair": off.tolist(),
            "balanced_TRUE_exposures_per_pair": on.tolist(),
            "balanced_FALSE_exposures": int(off.sum()), "balanced_TRUE_exposures": int(on.sum()),
            "balanced_route_exposures": {name: {"FALSE": int(off[i*240:(i+1)*240].sum()),
                                                "TRUE": int(on[i*240:(i+1)*240].sum())}
                                           for i, name in enumerate(ROUTES)},
            "full_epochs": flat.size // TOTAL_COUNT, "last_partial_epoch": flat.size % TOTAL_COUNT,
            "total_exposures_per_arm": int(flat.size)}


def optimization() -> dict:
    return {"steps_per_arm": STEPS, "batch": BATCH, "seed": SEED, "learning_rate": LR,
            "weight_decay": WEIGHT_DECAY, "optimizer": "fresh AdamW in each arm",
            "checkpoint_selection": "fixed_final_10000_only", "gradient_norm_cap": 5.0}


def review_contract(review, render_sha, plan_sha, captures) -> None:
    require(review.get("schema") == REVIEW_SCHEMA and review.get("study_id") == NAME,
            "explicit study review/authorization schema differs")
    require(review.get("parent_weights_sha256") == PARENT_SHA
            and review.get("render_manifest_sha256") == render_sha
            and review.get("training_plan_sha256") == plan_sha, "review byte/parent/plan binding differs")
    require(review.get("research_fit_authorized") is True
            and review.get("native_diagnostic_denials_preserved") is True
            and review.get("vehicle_control_approved") is False
            and review.get("dev_or_test_used_for_fit") is False
            and review.get("ctrack_train_eligible") is False
            and type(review.get("independent_new_dev_sites")) is int
            and review["independent_new_dev_sites"] == 0, "research scope was not separately authorized")
    require(review.get("counts") == COUNTS and review.get("arms") == ARMS
            and review.get("optimization") == optimization(), "review fitting protocol differs")
    require(isinstance(review.get("reviewer"), str) and len(review["reviewer"].strip()) >= 3
            and isinstance(review.get("review_note"), str) and len(review["review_note"].strip()) >= 20,
            "actual reviewer and explanatory review note required")
    reviews = review.get("source_reviews", {})
    require(set(reviews) == set(ROUTES), "both full captures require manual review")
    for name in ROUTES:
        row = reviews[name]
        require(row.get("source_capture") == str(captures[name])
                and row.get("qa_sha256") == QA_SHA[name] and row.get("route_sha256") == ROUTES[name]
                and row.get("reviewed_indices") == [0, 119, 239]
                and row.get("rgb_conditions") == ["false", "true"]
                and row.get("semantic_target_overlay_reviewed") is True
                and row.get("all_240_retained") is True
                and isinstance(row.get("review_note"), str) and len(row["review_note"].strip()) >= 20,
                "fixed RGB/semantic review receipt differs: " + name)


def validate_manifest(manifest, registry) -> None:
    require(manifest.get("schema") == DATA_SCHEMA and manifest.get("study_id") == NAME
            and manifest.get("labels") == list(LABELS) and manifest.get("counts") == COUNTS
            and manifest.get("arms") == ARMS, "matched study schema/counts/factor differs")
    require(registry.get("schema") == REGISTRY_SCHEMA and registry.get("study_id") == NAME
            and registry.get("research_fit_authorized") is True
            and registry.get("optimization") == optimization(), "bounded research registration missing")
    for item in (manifest, registry):
        require(item.get("parent_weights_sha256") == PARENT_SHA and item.get("route_sha256") == ROUTES
                and item.get("native_diagnostic_denials_preserved") is True
                and item.get("ctrack_train_eligible") is False
                and item.get("dev_or_test_used_for_fit") is False
                and item.get("vehicle_control_approved") is False
                and item.get("independent_new_dev_sites") == 0, "protected source/fit boundary differs")
    require(manifest.get("raw_model_inputs") == ["RGB"]
            and manifest.get("depth_or_pose_model_input") is False
            and manifest.get("source_receipts_modified") is False, "non-RGB input or source mutation")
    require(registry.get("counts") == COUNTS and registry.get("arms") == ARMS
            and registry.get("broad_registry_modified") is False
            and registry.get("data_files") == manifest.get("files")
            and registry.get("common_schedule") == manifest.get("schedule_contract"), "registry linkage differs")
    expected = [f"{name}:frames/{index:04d}" for name in ROUTES for index in range(240)]
    require(manifest.get("paired_sample_ids") == expected, "physical pairs were reordered/filtered/duplicated")
    require(set(manifest.get("files", {})) == {"original_train729.npz", "paired_train480.npz",
                                              "training_schedule.npz", "parent_weights.npz"}, "package members differ")
    require(manifest["files"]["original_train729.npz"]["sha256"] == ORIGINAL_SHA
            and manifest["files"]["parent_weights.npz"]["sha256"] == PARENT_SHA,
            "unchanged original replay and parent bytes required")


def load_package(root, expected_manifest_sha, expected_registry_sha):
    canonical(root)
    manifest = read_json(root / "manifest.json", expected_manifest_sha)
    registry = read_json(root / "study_train_registry.json", expected_registry_sha)
    validate_manifest(manifest, registry)
    require(manifest["study_registry_sha256"] == expected_registry_sha, "registry pointer differs")
    rows = read_json(root / "paired_samples.json", manifest["paired_samples_sha256"])
    require([row["sample_id"] for row in rows] == manifest["paired_sample_ids"], "provenance identity differs")
    loaded = {name: read_npz(root / name, ref["sha256"]) for name, ref in manifest["files"].items()
              if name != "parent_weights.npz"}
    require(digest(root / "parent_weights.npz") == PARENT_SHA, "parent changed")
    original, paired, saved = (loaded[name] for name in
                              ("original_train729.npz", "paired_train480.npz", "training_schedule.npz"))
    require(set(original) == {"rgb", "target"} and set(paired) == {"rgb_false", "rgb_true", "target"}
            and set(saved) == {"indices", "crops", "balanced_view_bits"}, "archive tensor keys differ")
    arrays = original["rgb"], original["target"], paired["rgb_false"], paired["rgb_true"], paired["target"]
    for value, shape in zip(arrays, ((729, 360, 640, 3), (729, 360, 640),
                                   (480, 360, 640, 3), (480, 360, 640, 3), (480, 360, 640))):
        require(value.shape == shape and value.dtype == np.uint8, "native data ABI differs")
    require(int(arrays[1].max()) < 7 and int(arrays[4].max()) < 7, "class ID out of range")
    for index, row in enumerate(rows):
        for key, value in (("rgb_false_sha256", arrays[2][index]), ("rgb_true_sha256", arrays[3][index]),
                           ("shared_target_sha256", arrays[4][index])):
            require(hashlib.sha256(value.tobytes()).hexdigest() == row[key], "decoded paired bytes differ")
    pixels = np.bincount(arrays[1].ravel(), minlength=7) + np.bincount(arrays[4].ravel(), minlength=7)
    require(pixels.tolist() == manifest["common_class_pixels"]
            and common_weights(pixels).tolist() == manifest["common_class_weights"], "shared target weights differ")
    indices, crops, bits = saved["indices"], saved["crops"], saved["balanced_view_bits"]
    require(schedule_proof(indices, crops, bits) == manifest["schedule_contract"], "schedule receipt differs")
    return manifest, arrays, indices, crops, bits
