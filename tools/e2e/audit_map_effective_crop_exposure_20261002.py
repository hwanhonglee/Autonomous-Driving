#!/usr/bin/env python3
"""HH_261002 - Reconstruct actual TRAIN target exposure, never mine DEV or fit a model.

All four completed V1/V2 arms retain their 32000 recorded exposures. A positive
full frame does not imply a positive cropped training target. The unchanged fit
venv and an external runtime/source/input authorization are required for actual
reconstruction; a different local Pillow backend is not silently substituted.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
import time
from types import FunctionType

import numpy as np

try:
    from . import train_map_hardnegative_continuation_v2_20261002 as study
except ImportError:
    import train_map_hardnegative_continuation_v2_20261002 as study

contract, mining, previous, require = study.contract, study.mining, study.previous, study.require
SCHEMA = "portable_e2e.map_effective_crop_exposure.20261002.v1"
AUTHORIZATION_SCHEMA = SCHEMA + ".authorization"
FROZEN_V2_SHA = "0af1e282e0f117b7e1fa53a694fc4f62cf5f6650e9cb757f4b3d18aba53a10c8"
FROZEN_SOURCE_SHA = {**study.FROZEN_SOURCE_SHA, Path(study.__file__).name: FROZEN_V2_SHA}
SOURCE_NAMES = set(FROZEN_SOURCE_SHA) | {Path(__file__).name}
SUMMARY_SHA = dict(v1="ea3674e4c76c8e4bef6b97b013e19c3934e49ed934903891c39b3bd0b3a49c6c",
                  v2="f96eed213353c4673ef37e03e06f52c59983557f10592c7929440108b716a160")
STEPS, BATCH, TOTAL, PIXELS = 4000, 8, 1209, 360 * 640
MAX_SECONDS, MAX_PIN_BYTES, MAX_PIN_FILES = 900, 2 * 1024**3, 10000
GROUPS = tuple(dict.fromkeys(mining.source_group(i) for i in range(TOTAL)))
EXPECTED_VERSIONS = dict(pillow="12.3.0", numpy="2.5.2")


def prepared_contract():
    return dict(schema=SCHEMA, operation="CPU_ONLY_ACTUAL_TRAIN_CROP_TARGET_CENSUS_NO_MODEL_NO_FIT",
        completed_summary_sha256=SUMMARY_SHA, physical_TRAIN_slots=TOTAL,
        runs=["v1", "v2"], arms=["control", "candidate"], steps_per_arm=STEPS,
        batch=BATCH, exposures_per_arm=STEPS * BATCH, full_exposure_denominator=4 * STEPS * BATCH,
        source_groups=list(GROUPS), original_stratum_frame_counts=dict(small_sign=391, large_sign=193, hard_negative=145),
        data_manifest_sha256=mining.DATA_MANIFEST_SHA, data_registry_sha256=mining.DATA_REGISTRY_SHA,
        original_mining_sha256=previous.MINING_SHA, initial_model_weights_sha256=mining.INITIAL_MODEL_SHA,
        labels=list(mining.LABELS), sign_class_id=5, actual_input_appearance="FALSE_OR_ORIGINAL",
        original_frozen_apply_crop_sha256=FROZEN_SOURCE_SHA["map_appearance_matched_contract_20261001.py"],
        required_current_unchanged_fit_backend_versions=EXPECTED_VERSIONS,
        runtime_binary_and_python_fingerprints_externally_pinned=True,
        fit_time_binary_fingerprints_not_recorded_in_original_receipts=True,
        current_private_fit_environment_reconstruction_not_historical_binary_attestation=True,
        maximum_wall_seconds=MAX_SECONDS, cache_contains_only_scalar_histograms=True,
        frozen_native_denials_preserved=True, crop_reroll=False, target_relabeling=False,
        known_DEV_or_HOLDOUT_payload_access=False, model_or_optimizer_or_CUDA_initialized=False,
        efficacy_or_sampler_causality_proven=False, training_authorized=False,
        model_promoted=False, vehicle_control_approved=False, independent_new_sites=0)


def strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    def reject(value):
        raise ValueError("nonfinite JSON constant: " + value)
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=reject)


class Pins:
    """HH_261002 - Preserve first-read bytes; never replace an old pin with a later digest."""
    def __init__(self):
        self.values = {}

    def file(self, path, expected=None, read=False):
        path = contract.canonical(Path(path))
        require(path.is_file(), "regular pinned input required")
        raw = path.read_bytes() if read else None
        sha = hashlib.sha256(raw).hexdigest() if read else contract.digest(path)
        require(expected is None or sha == expected, "externally pinned input changed: " + str(path))
        row = dict(sha256=sha, bytes=len(raw) if read else path.stat().st_size)
        require(str(path) not in self.values or self.values[str(path)] == row, "first-read input pin changed")
        self.values[str(path)] = row
        return raw if read else sha

    def document(self, path, expected=None):
        return strict_json(self.file(path, expected, read=True))

    def tree(self, root):
        root = contract.canonical(Path(root)); require(root.is_dir(), "pinned tree required")
        files = sorted(root.rglob("*"))
        require(len(files) <= MAX_PIN_FILES, "bounded input tree required")
        size = 0
        for path in files:
            contract.canonical(path)
            require(path.is_file() or path.is_dir(), "unknown input member")
            if path.is_file():
                size += path.stat().st_size
                require(size <= MAX_PIN_BYTES, "bounded input bytes required")
                self.file(path)

    def recheck(self):
        for path, row in self.values.items():
            require(Path(path).stat().st_size == row["bytes"] and contract.digest(Path(path)) == row["sha256"],
                "first-read input changed before publication: " + path)


def runtime_identity():
    # HH_261002 - Pillow NEAREST is part of target reconstruction, not merely an optional display dependency.
    import PIL
    from PIL import Image, _imaging
    files = {"PIL.Image": Path(Image.__file__), "PIL._imaging": Path(_imaging.__file__), "numpy": Path(np.__file__)}
    return dict(python_version=platform.python_version(), python_implementation=platform.python_implementation(),
        python_prefix=str(Path(sys.prefix)), pillow=PIL.__version__, numpy=np.__version__,
        provider_files={key:dict(path=str(path), sha256=contract.digest(path)) for key, path in files.items()})


def environment(identity):
    require(os.uname().nodename == "pro6000" and Path(sys.prefix) == mining.WORKSPACE / "venvs/py312",
        "actual unchanged private fit venv required; no local backend substitution")
    require(all(identity.get(key) == value for key, value in EXPECTED_VERSIONS.items()),
        "Pillow/NumPy backend differs from the measured actual fit environment")
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == "" and os.environ.get("PYTHONNOUSERSITE") == "1"
        and os.environ.get("OMP_NUM_THREADS") == os.environ.get("MKL_NUM_THREADS") == "1"
        and not any(os.environ.get(key) for key in ("PYTHONPATH", "PYTHONHOME", "PYTHONUSERBASE", "PYTHONSTARTUP"))
        and "torch" not in sys.modules, "CPU-only isolated diagnostic environment required")


def validate_authorization(value, source, identity, paths):
    require(type(value) is dict and value.get("schema") == AUTHORIZATION_SCHEMA
        and value.get("source_sha256") == source and value.get("runtime_identity") == identity
        and value.get("paths") == {key:str(path) for key, path in paths.items()}
        and json.dumps(value.get("contract"), sort_keys=True, allow_nan=False)
        == json.dumps(prepared_contract(), sort_keys=True, allow_nan=False), "exact external diagnostic declaration required")
    for key, wanted in (("research_diagnostic_authorized", True), ("native_denials_preserved", True),
        ("known_DEV_or_HOLDOUT_payload_access", False), ("fit_authorized", False),
        ("automatic_model_promotion", False), ("vehicle_control_approved", False)):
        require(value.get(key) is wanted, "authorization scope differs: " + key)
    require(type(value.get("reviewer")) is str and len(value["reviewer"].strip()) >= 3, "actual reviewer required")


def checked_paths(value):
    names = {"data", "mining", "v1", "v2", "source", "output"}
    require(type(value) is dict and set(value) == names, "six exact diagnostic paths required")
    paths = {key:contract.canonical(Path(path)) for key, path in value.items()}
    require(all(paths[key].is_dir() for key in names - {"output"}), "all immutable input directories required")
    output = paths["output"]
    require(not output.exists(), "fresh create-only output required")
    require(all(output != paths[key] and paths[key] not in output.parents for key in names - {"output"}),
        "diagnostic output cannot be inside immutable inputs")
    require(all(paths["source"] != paths[key] for key in ("data", "mining", "v1", "v2")), "source/input separation required")
    return paths


def source_snapshot(root, pins):
    source = {name:pins.file(root / name) for name in sorted(SOURCE_NAMES)}
    require(all(source[name] == sha for name, sha in FROZEN_SOURCE_SHA.items()), "all nine frozen source providers required")
    require(Path(__file__).resolve() == root / Path(__file__).name, "auditor escaped declared source package")
    for module in (study, previous, mining, contract, contract.frozen, previous.previous):
        require(Path(module.__file__).resolve() == root / Path(module.__file__).name, "provider escaped pinned source closure")
    return source


def schedules(paths, pins):
    # HH_261002 - Reuse the entire frozen original admission path in a private function namespace, never patch globals.
    pins.tree(paths["mining"])
    namespace = dict(study._old_load.__globals__, MINING_OUTPUT=paths["mining"])
    inventory, original, old_proof = FunctionType(study._old_load.__code__, namespace, "immutable_mining_load")()
    require(inventory["stratum_frame_counts"] == dict(small_sign=391, large_sign=193, hard_negative=145), "actual strata differ")
    fresh = study.schedules(inventory, original)
    proof = study.schedule_proof(inventory, fresh, original)
    receipt = pins.document(paths["v2"] / "new_schedule_archive_receipt.json")
    saved_proof = pins.document(paths["v2"] / "new_declared_schedule_proof.json", receipt["new_schedule_proof_file_sha256"])
    pins.file(paths["v2"] / "new_declared_schedule.npz", receipt["new_schedule_npz_sha256"])
    saved = contract.read_npz(paths["v2"] / "new_declared_schedule.npz", receipt["new_schedule_npz_sha256"])
    study.verify_schedule_tensor_abi(saved, saved_proof)
    require(saved_proof == proof and receipt["new_sampler_proof_sha256"] == study.sampler_proof_sha256(proof)
        and receipt["original_schedule_sha256"] == previous.MINING_SHA["proposed_schedule.npz"]
        and receipt["original_mining_artifacts_unmodified"] is True
        and receipt["training_authorized_by_this_receipt"] is False, "separate V2 schedule commitment differs")
    require(all(np.array_equal(saved[key], fresh[key]) for key in fresh), "actual V2 schedule differs")
    return inventory, dict(v1=original, v2=saved), dict(v1=old_proof, v2=proof)


def completed_rows(root, revision, schedule, proof, source, pins):
    summary = pins.document(root / "summary.json", SUMMARY_SHA[revision])
    fit = pins.document(root / "fit_complete.json")
    require(summary.get("schema") == (previous.SCHEMA if revision == "v1" else study.SCHEMA)
        and summary.get("source_and_inputs_unchanged") is True
        and summary.get("sampler_proof") == proof
        and summary.get("initial_model_weights_sha256") == mining.INITIAL_MODEL_SHA,
        "completed same-parent frozen sampler run required")
    expected_sources = {key:source[key] for key in (previous.SOURCE_NAMES if revision == "v1" else study.SOURCE_NAMES)}
    require(summary.get("source_sha256") == expected_sources, "completed run source closure differs")
    for name, sha in expected_sources.items(): pins.file(root / "source" / name, sha)
    require(fit.get("status") == "BOTH_FIXED_FINAL4000_ENDPOINTS_SAVED_BEFORE_KNOWN_EVALUATION"
        and fit.get("arms") == summary.get("arms_result"), "both fixed final endpoints required")
    require(pins.file(root / "inputs/manifest.json") == mining.DATA_MANIFEST_SHA
        and pins.file(root / "inputs/study_train_registry.json") == mining.DATA_REGISTRY_SHA, "run/data identity differs")
    auth = pins.document(root / "authorization.json", summary["authorization_sha256"])
    require(auth.get("source_sha256") == expected_sources and auth.get("research_fit_authorized") is True
        and auth.get("DEV_or_HOLDOUT_used_for_fit") is False and auth.get("native_denials_preserved") is True,
        "original run authorization scope differs")
    if revision == "v2": require(auth.get("new_sampler_proof_sha256") == study.sampler_proof_sha256(proof), "external V2 proof differs")
    result = {}
    for arm in ("control", "candidate"):
        info = summary["arms_result"][arm]
        require(type(info.get("steps")) is int and info["steps"] == STEPS
            and type(info.get("examples_seen")) is int and info["examples_seen"] == STEPS * BATCH,
            "all fixed exposures required")
        require(pins.document(root / arm / "training.json") == info, "actual training endpoint differs")
        pins.file(root / arm / "weights.npz", info["weights_sha256"])
        raw = pins.file(root / arm / "exposures.jsonl", info["full_exposure_journal_sha256"], read=True)
        rows = [strict_json(line) for line in raw.splitlines()]
        verify_rows(rows, arm, schedule, info)
        result[arm] = rows
    return result


def verify_rows(rows, arm, schedule, info):
    require(arm in ("control", "candidate") and len(rows) == STEPS, "all4000 actual chronological rows required")
    hashes = [hashlib.sha256() for _ in range(3)]
    for step, row in enumerate(rows):
        ids, boxes, bits = schedule[arm + "_indices"][step], schedule["shared_crops"][step], schedule["shared_FALSE_view_bits"][step]
        require(type(row) is dict and set(row) == {"step", "physical_slot_indices", "shared_crop_boxes", "actual_paired_TRUE_bits"}
            and type(row["step"]) is int and row["step"] == step + 1, "actual exposure row ABI differs")
        require(all(type(value) is int for value in row["physical_slot_indices"] + row["actual_paired_TRUE_bits"]
                    + [v for box in row["shared_crop_boxes"] for v in box]), "no Boolean/float exposure integers")
        require(row["physical_slot_indices"] == ids.tolist() and row["shared_crop_boxes"] == boxes.tolist()
            and row["actual_paired_TRUE_bits"] == bits.tolist() and not bool(bits.any()), "actual index/crop/FALSE chronology differs")
        for digest, value in zip(hashes, (ids, boxes, bits)): digest.update(value.tobytes())
    for key, digest in zip(("sample_sequence_sha256", "crop_sequence_sha256", "view_sequence_sha256"), hashes):
        require(info.get(key) == digest.hexdigest(), "all actual sequence digest differs")
    require(info.get("physical_slot_exposures") == np.bincount(schedule[arm + "_indices"].ravel(), minlength=TOTAL).tolist(),
        "all1209 actual exposure census differs")


def count_crop(rgb, target, box):
    require(contract.apply_crop is contract.frozen.apply_crop, "exact frozen crop provider required")
    require(target.shape == (360, 640) and target.dtype == np.uint8 and int(target.max()) < 7, "actual target ABI differs")
    require(len(box) == 4 and all(type(v) is int for v in box), "four actual integer crop coordinates required")
    top, left, height, width = box
    _, resized = contract.apply_crop(rgb, target, box)
    source_sign = int(np.count_nonzero(target[top:top + height, left:left + width] == 5))
    histogram = tuple(int(v) for v in np.bincount(resized.ravel(), minlength=7))
    require(sum(histogram) == PIXELS, "full actual resized target denominator differs")
    return source_sign, histogram


def empty_counts():
    return dict(exposures=0, full_frame_sign_positive_exposures=0, crop_zero_sign_exposures_from_positive_frames=0,
        raw_crop_cut_all_sign_exposures=0, resize_zero_sign_despite_positive_source_crop=0,
        crop_cut_some_sign_exposures=0, full_frame_sign_pixels_sum=0, source_crop_sign_pixels_sum=0,
        actual_resized_sign_pixels_sum=0, actual_resized_class_pixels=[0] * 7)


def add_counts(row, full, source_sign, histogram):
    row["exposures"] += 1
    row["full_frame_sign_positive_exposures"] += int(full > 0)
    row["crop_zero_sign_exposures_from_positive_frames"] += int(full > 0 and histogram[5] == 0)
    row["raw_crop_cut_all_sign_exposures"] += int(full > 0 and source_sign == 0)
    row["resize_zero_sign_despite_positive_source_crop"] += int(source_sign > 0 and histogram[5] == 0)
    row["crop_cut_some_sign_exposures"] += int(0 < source_sign < full)
    for key, value in (("full_frame_sign_pixels_sum", full), ("source_crop_sign_pixels_sum", source_sign),
                       ("actual_resized_sign_pixels_sum", histogram[5])): row[key] += value
    row["actual_resized_class_pixels"] = [a + b for a, b in zip(row["actual_resized_class_pixels"], histogram)]


def count_arm(rows, arrays, inventory, cache, deadline):
    result = dict(overall=empty_counts(), source_groups={key:empty_counts() for key in GROUPS},
        overlapping_original_strata={key:empty_counts() for key in inventory["strata"]},
        source_group_x_overlapping_original_strata={group:{key:empty_counts() for key in inventory["strata"]} for group in GROUPS},
        physical_slot_exposures=[0] * TOTAL)
    strata = {key:set(ids) for key, ids in inventory["strata"].items()}
    original_rgb, original_target, pair_false, _, pair_target = arrays
    for row in rows:
        require(time.monotonic() < deadline, "bounded CPU diagnostic time exhausted")
        for index, box in zip(row["physical_slot_indices"], row["shared_crop_boxes"]):
            key = (index, *box)
            if key not in cache:
                rgb, target = (original_rgb[index], original_target[index]) if index < 729 else (pair_false[index - 729], pair_target[index - 729])
                cache[key] = count_crop(rgb, target, box)
            source_sign, histogram = cache[key]
            full = inventory["records"][index]["sign_target_pixels"]
            source_group = mining.source_group(index)
            result["physical_slot_exposures"][index] += 1
            selected = [name for name, ids in strata.items() if index in ids]
            for group in (result["overall"], result["source_groups"][source_group],
                          *(result["overlapping_original_strata"][name] for name in selected),
                          *(result["source_group_x_overlapping_original_strata"][source_group][name] for name in selected)):
                add_counts(group, full, source_sign, histogram)
    require(result["overall"]["exposures"] == STEPS * BATCH and min(result["physical_slot_exposures"]) > 0,
        "complete actual exposure denominator required")
    require(sum(result["overall"]["actual_resized_class_pixels"]) == STEPS * BATCH * PIXELS, "all seven target labels required")
    require(sum(row["exposures"] for row in result["source_groups"].values()) == STEPS * BATCH, "source groups do not partition exposures")
    return result


def run(args):
    pins = Pins(); value = pins.document(Path(args.authorization), args.authorization_sha256)
    paths = checked_paths(value.get("paths")); source = source_snapshot(paths["source"], pins)
    identity = runtime_identity(); environment(identity)
    validate_authorization(value, source, identity, paths)
    for row in identity["provider_files"].values(): pins.file(Path(row["path"]), row["sha256"])
    deadline = time.monotonic() + MAX_SECONDS
    inventory, plans, proofs = schedules(paths, pins)
    rows = {revision:completed_rows(paths[revision], revision, plans[revision], proofs[revision], source, pins) for revision in ("v1", "v2")}
    pins.tree(paths["data"])
    manifest, arrays, _, _, _ = contract.load_package(paths["data"], mining.DATA_MANIFEST_SHA, mining.DATA_REGISTRY_SHA)
    previous.verify_train_identity(inventory, arrays)
    require(manifest["common_class_pixels"] == mining.CLASS_PIXELS, "all unchanged source target labels required")
    for array in arrays: array.flags.writeable = False
    pins.recheck()
    if args.validate_only:
        return dict(schema=SCHEMA, status="CPU_TRAIN_INPUTS_VERIFIED_NO_COUNT_OR_OUTPUT", source_sha256=source,
            runtime_identity=identity, known_DEV_or_HOLDOUT_payload_access=False, input_pin_count=len(pins.values))
    cache = {}; results = {revision:{arm:count_arm(rows[revision][arm], arrays, inventory, cache, deadline)
                                    for arm in ("control", "candidate")} for revision in ("v1", "v2")}
    require(results["v1"]["control"] == results["v2"]["control"], "identical actual controls must reconstruct identical target exposure")
    pins.recheck(); mining.reserve(paths["output"])
    report = dict(**prepared_contract(), status="ACTUAL_TRAIN_CROP_CENSUS_DIAGNOSTIC_NOT_CAUSAL_PROOF",
        runtime_identity=identity, source_sha256=source, authorization_sha256=args.authorization_sha256,
        actual_exposure_counts=results, scalar_cache_entries=len(cache), control_reconstruction_exactly_equal=True,
        first_read_input_pins=pins.values, all_input_pins_unchanged=True,
        source_crop_pixels_and_resized_pixels_have_different_area_denominators=True,
        original_strata_overlap_not_an_exposure_partition=True,
        training_and_source_arrays_unchanged=True, parent_predictions_not_decoded_or_recomputed=True)
    require("torch" not in sys.modules, "no model/runtime import permitted")
    paths["output"].mkdir(parents=True, exist_ok=False)
    contract.write_new(paths["output"] / "summary.json", report)
    pins.recheck()
    return dict(schema=SCHEMA, status=report["status"], output=str(paths["output"]),
        summary_sha256=contract.digest(paths["output"] / "summary.json"), scalar_cache_entries=len(cache))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("contract", "runtime-contract", "audit"), default="contract")
    parser.add_argument("--authorization"); parser.add_argument("--authorization-sha256")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    if args.mode == "contract": result = prepared_contract()
    elif args.mode == "runtime-contract": result = runtime_identity()
    else:
        require(args.authorization is not None and args.authorization_sha256 is not None, "actual TRAIN reads require separate external diagnostic authorization")
        result = run(args)
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__": raise SystemExit(main())
