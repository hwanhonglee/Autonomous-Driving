"""HH_261005 - Qualify only named TEST metadata and asset existence; never load a model or simulator.

The new authorization supplements, but never edits or bypasses, the frozen site
guard. This tool cannot admit TEST training, model selection or actual captures.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import stat


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.rte01_test_only_metadata_qualification.20261005.v1"
PLAN = Path("storage/experiments/plans/2026-10-05/rte01_test_only_metadata_qualification_v1")
AUTHORIZATION = PLAN / "authorization.json"
REPORT = PLAN / "metadata_qualification.json"
SITES = ("Town04", "Town10HD_Opt", "Town15")
GOOD_SHA = "b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625"
GUARD = "learning/config/site_split_guard_20260929_v2.json"
PROPOSAL = "storage/experiments/plans/2026-10-04/rte01_independent_test_metadata_proposal_v1/README.md"
TRAIN = "storage/experiments/training/"
RAW = TRAIN + "2026-10-01/temporal_v2_raw_bootstrap_v1/declaration.json"
BALANCED = TRAIN + "2026-10-01/temporal_v2_balanced_replay_v1/declaration.json"
RECOVERY = TRAIN + "2026-10-01/temporal_route_recovery_v3_v1/declaration.json"
JOINT = TRAIN + "2026-10-02/temporal_route_joint_v1/declaration.json"
BALANCED_META = TRAIN + "2026-10-01/temporal_v2_balanced_replay_v1/exports/step_003000/metadata.json"
RECOVERY_META = TRAIN + "2026-10-01/temporal_route_recovery_v3_v1/exports/step_003000/metadata.json"
GOOD_META = TRAIN + "2026-10-02/temporal_route_joint_v1/exports/step_006000/metadata.json"
PINS = {
    GUARD: "323b68c690724bdb63a9955c9573a249d5c49f5001160a421e797249dce7ec0b",
    RAW: "461c494886658d0c7032aa24d3678f694b32b23645332b27b7c99660d66f0baf",
    BALANCED: "b6165e816ca20ae5dd37446afbed7b0e674efd76a77d99eb0cb3114b99d02e93",
    RECOVERY: "34c8a55028ea7b4cb2678a73701b16425d811fb10dd77afb5554183dd79ef6c5",
    JOINT: "0041d4d229d3a9ad777e2e0d4873793b65eeab608cbeb69b83fcda01b98cc3e5",
    BALANCED_META: "88ffb8dcf9d21d0f756cad2c4ff478a74deee3b8e817f72a039a8885484bb4ec",
    RECOVERY_META: "4ace621d8cf1b8d3ce269bb77aabe9e89598f3f750e18ea707a0817086b3dc2d",
    GOOD_META: "a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04",
}
STATUS = {site: f"storage/experiments/validation/2026-08-31/maps/{name}/status.json"
          for site, name in zip(SITES, ("town04", "town10hd_opt", "town15"))}
METADATA = tuple(PINS) + tuple(STATUS.values()) + (
    "tools/e2e/carla_expert_suite.yaml", PROPOSAL,
    "storage/datasets/prepared/carla-common10-30kph-five-episodes-20260907-v3/dataset.json",
)
PACKAGED = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
SOURCE = Path("/home/a/carla")
EDITOR = Path("/home/a/UnrealEngine_4.26/Engine/Binaries/Linux/UE4Editor")
ASSETS = {
    "Town04": (PACKAGED / "CarlaUE4.sh", PACKAGED / "CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping",
               PACKAGED / "CarlaUE4/Content/Carla/Maps/Town04.umap", PACKAGED / "CarlaUE4/Content/Carla/Maps/OpenDrive/Town04.xodr",
               PACKAGED / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"),
    "Town10HD_Opt": (PACKAGED / "CarlaUE4.sh", PACKAGED / "CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping",
                     PACKAGED / "CarlaUE4/Content/Carla/Maps/Town10HD_Opt.umap", PACKAGED / "CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD.xodr",
                     PACKAGED / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"),
    "Town15": (EDITOR, SOURCE / "Unreal/CarlaUE4/CarlaUE4.uproject",
               SOURCE / "Unreal/CarlaUE4/Content/Carla/Maps/Town15/Town15.umap",
               SOURCE / "Unreal/CarlaUE4/Content/Carla/Maps/Town15/OpenDrive/Town15.xodr",
               SOURCE / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"),
}
MAX_METADATA_BYTES = 1024 * 1024


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def is_sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def authorization_document():
    # HH_261005 - Explicit user authority permits TEST-only preparation, not silent reopening for TRAIN or DEV.
    return dict(schema=SCHEMA + ".authorization", provenance="HH_261005 - Prepare the explicitly authorized independent TEST-only metadata scope before any TEST inference.",
        authority="Explicit user approval relayed by ROOT on 2026-10-05; ROOT reviews new actual declarations separately",
        feature="RTE-01", candidate_sites=list(SITES), model_weights_sha256=GOOD_SHA,
        allowed_operations=["named_metadata_read_and_bounded_SHA", "named_asset_lstat_only"],
        metadata_whitelist=list(METADATA), asset_lstat_whitelist={site: [str(p) for p in paths] for site, paths in ASSETS.items()},
        old_guard_modified=False, train_or_dev_role_change=False, TEST_training=False,
        TEST_NN_or_loss_or_labels=False, TEST_pixel_or_state_payload=False,
        model_or_checkpoint_selection=False, CARLA_GPU_SSH_or_install=False,
        actual_capture_authorized_by_this_document=False, model_promotion=False,
        pristine_or_independent_TEST_certified=False, main_functions_approved=0, subfunctions_approved=0)


def real_path(path, *, missing_tail=False):
    path = Path(path)
    require(path.is_absolute() and path.resolve() == path, "canonical absolute path required")
    for part in (path, *path.parents):
        if missing_tail and part == path and not part.exists():
            continue
        require(not part.is_symlink(), "symlink forbidden in metadata or output path")
    return path


def bounded_bytes(path):
    path = real_path(path)
    flags = os.O_RDONLY | os.O_NOFOLLOW
    fd = os.open(path, flags)
    try:
        before = os.fstat(fd)
        require(stat.S_ISREG(before.st_mode) and 0 < before.st_size <= MAX_METADATA_BYTES, "bounded regular metadata required")
        chunks, length = [], 0
        while True:
            block = os.read(fd, min(65536, MAX_METADATA_BYTES + 1 - length))
            if not block:
                break
            chunks.append(block); length += len(block)
            require(length <= MAX_METADATA_BYTES, "metadata grew beyond its bound")
        after = os.fstat(fd)
        require((before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns)
                == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns)
                and length == before.st_size, "metadata changed during read")
        return b"".join(chunks)
    finally:
        os.close(fd)


def unique_json(payload):
    def unique(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate metadata member")
            result[key] = value
        return result
    return json.loads(payload, object_pairs_hook=unique,
        parse_constant=lambda value: require(False, "nonfinite metadata"))


def read_metadata(name):
    require(type(name) is str and name in METADATA, "metadata path is not authorized")
    data = bounded_bytes(ROOT / name)
    sha = hashlib.sha256(data).hexdigest()
    require(name not in PINS or sha == PINS[name], "frozen metadata seal changed: " + name)
    return data, sha


def declared_ancestry(documents):
    """HH_261005 - Verify the declared parent graph, not weight bytes or unobserved remote dataset ingestion."""
    raw, balanced, recovery, joint = [documents[name] for name in (RAW, BALANCED, RECOVERY, JOINT)]
    bmeta, rmeta, good = [documents[name] for name in (BALANCED_META, RECOVERY_META, GOOD_META)]
    require(raw["initialization"] == "fresh random weights, not adapted W1" and raw["steps"] == 200,
            "fresh raw bootstrap declaration required")
    require(balanced["parent_checkpoint_sha256"] == "6c8bfceee5b3a2a0e332666b225769af236499baac9414294c18ccc6e9833ece",
            "balanced parent is not the declared raw200")
    require(recovery["parent_npz_sha256"] == bmeta["npz_sha256"]
            and recovery["parent_metadata_sha256"] == PINS[BALANCED_META]
            and joint["parent_npz_sha256"] == rmeta["npz_sha256"]
            and joint["parent_metadata_sha256"] == PINS[RECOVERY_META]
            and joint["parent_declaration_sha256"] == PINS[RECOVERY]
            and good["parent_npz_sha256"] == rmeta["npz_sha256"]
            and good["parent_metadata_sha256"] == PINS[RECOVERY_META]
            and good["npz_sha256"] == GOOD_SHA, "declared GOOD6000 ancestry mismatch")
    strata = balanced["legacy_strata"]
    maps = {site: strata["curved_by_map"].get(site, 0) + strata["straight_by_map"].get(site, 0)
            for site in set(strata["curved_by_map"]) | set(strata["straight_by_map"])}
    require(maps == {"C_track_1_0_7": 922, "Town01": 638, "Town07": 927}
            and sum(maps.values()) == strata["eligible_count"] == 2487, "declared eligible legacy sites changed")
    require(raw["train"]["manifest_sha256"] == balanced["cohorts"]["legacy"]["manifest_sha256"]
            and raw["train"]["temporal_fingerprint_sha256"] == balanced["cohorts"]["legacy"]["temporal_fingerprint_sha256"],
            "raw and balanced legacy manifests differ")
    return dict(model_weights_sha256=GOOD_SHA, declared_eligible_legacy_by_site=maps,
        new_cruise_and_signed_recovery_TRAIN_site="Town07 (pinned declarations; no remote payload opened)",
        reserved_sites_in_declared_TRAIN=False, exact_weight_bytes_or_remote_dataset_reverified=False,
        all_prior_current_model_inference_and_candidate_exposure_absence_verified=False,
        known_repeated_DEV_site="Town03", known_TRAIN_and_runtime_site="Town07",
        historical_VAD_or_teacher_exposure_is_not_current_weight_ingestion_proof=True)


def asset_stat(path):
    # HH_261005 - Existence is not renderer, dependency, map hash, calibration or sensor quality qualification.
    try:
        path = real_path(path)
        value = path.lstat()
        require(stat.S_ISREG(value.st_mode), "regular asset required")
        return dict(path=str(path), exists=True, regular=True, size_bytes=value.st_size,
                    executable=bool(value.st_mode & 0o111), asset_payload_read_or_hashed=False)
    except (OSError, RuntimeError) as error:
        return dict(path=str(path), exists=False, error_type=type(error).__name__, error=str(error), asset_payload_read_or_hashed=False)


def qualify(expected_authorization_sha256):
    require(is_sha(expected_authorization_sha256), "exact authorization SHA required")
    auth_bytes = bounded_bytes(ROOT / AUTHORIZATION)
    require(hashlib.sha256(auth_bytes).hexdigest() == expected_authorization_sha256, "authorization SHA changed")
    require(json.dumps(unique_json(auth_bytes), sort_keys=True, separators=(",", ":"), allow_nan=False)
            == json.dumps(authorization_document(), sort_keys=True, separators=(",", ":"), allow_nan=False),
            "authorization widened or changed")
    pins, docs = {}, {}
    for name in METADATA:
        payload, pins[name] = read_metadata(name)
        if name.endswith(".json"):
            docs[name] = unique_json(payload)
    guard = docs[GUARD]
    require(all(guard["sites"][site]["map"] == [] and guard["sites"][site]["drive"] == [] for site in SITES),
            "frozen reservation was reopened")
    ancestry = declared_ancestry(docs)
    candidates = {}
    for site in SITES:
        historical = docs[STATUS[site]]
        require(historical["canonical_name"] == site, "historical map identity differs")
        assets = [asset_stat(path) for path in ASSETS[site]]
        candidates[site] = dict(historical_status=historical["status"], historical_suite_split=historical["manifest_split"],
            historical_server_profile=historical["static_inventory"]["server_profile"], named_assets=assets,
            all_named_assets_exist=all(row["exists"] for row in assets),
            current_owned_renderer_or_six_camera_quality_verified=False, current_model_exposure_complete=False,
            qualified_independent_TEST=False, TEST_qualification_blockers=["complete_current_model_candidate_exposure_ledger",
                "fresh_owned_asset_source_generation_and_renderer_binding", "actual_six_camera_rig_and_quality_qualification",
                "new_root_reviewed_exact_route_and_TEST_only_execution_declaration"])
    return dict(schema=SCHEMA, provenance="HH_261005 - Retain authorization, frozen reservations, declared ancestry and unresolved TEST qualification separately.",
        created_UTC=datetime.now(timezone.utc).isoformat(), authorization_sha256=expected_authorization_sha256,
        metadata_sha256=pins, metadata_only=True, old_guard_modified=False,
        ancestry=ancestry, candidates=candidates, status="METADATA_PREPARED_NOT_INDEPENDENT_TEST_APPROVED",
        prospective_30_episode_matrix_declared=False, minimum_closed_loop_episodes_after_qualification=30,
        prospective_matrix_requires_fixed_model_routes_sites_seeds_and_at_least_three_weather_conditions=True,
        failures_retries_and_denominator_must_be_retained=True, new_model_NN_or_training_executed=False,
        CARLA_GPU_SSH_or_install_executed=False, TEST_payload_opened=False,
        main_functions_approved=0, subfunctions_approved=0)


def save_report(document):
    target = real_path(ROOT / REPORT, missing_tail=True)
    # HH_261005 - No frozen report is replaced, including a raced symlink or an earlier failed attempt.
    fd = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--expected-authorization-sha256", required=True)
    parser.add_argument("--save-create-only", action="store_true")
    args = parser.parse_args(argv)
    value = qualify(args.expected_authorization_sha256)
    if args.save_create_only:
        save_report(value)
    print(json.dumps(value, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
