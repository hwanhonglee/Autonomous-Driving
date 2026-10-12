"""HH_261004 - Seal prospective media decisions, then inventory and independently postcheck one new actual callback45v3 run."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from PIL import Image

from . import publish_temporal_route_joint_town03_straight_callback45_v3_20261004 as p


PLAN_SHA = "b2867e9c542f86d7da6cf8f530695e3c79aab6bc0546ebb0b72d2ea0c6b2bd24"
OLD_POLICY = p.ROOT / "storage/experiments/plans/2026-10-04/town03_known_dev_straight_callback45_v2/ROOT_prospective_publication_policy.json"
OLD_POLICY_SHA = "e40a02d32fd86f2a746e3a991479a1731dc066409c5f230b4595b87f1d5c4819"
TEST = p.ROOT / "learning/tests/test_publish_temporal_route_joint_town03_straight_callback45_v3_20261004.py"


def save_new(path, value):
    # HH_261004 - Refuse aliases and existing output; never rewrite a prior declaration, attempt or result.
    p.require(path.resolve() == path and not path.exists() and not path.is_symlink(), "new canonical evidence required")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write("\n")
    return p.prior.digest(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("policy", "caller", "postcheck"), required=True)
    parser.add_argument("--publisher-sha256", required=True)
    parser.add_argument("--test-sha256", required=True)
    args = parser.parse_args(argv)
    p.require(p.prior.digest(Path(p.__file__).resolve()) == args.publisher_sha256
              and p.prior.digest(TEST) == args.test_sha256, "root-reviewed new source or CPU test changed")
    plan_path = p.RUN.with_name(p.RUN.name + "_plan.json")
    p.require(plan_path.resolve() == plan_path and p.prior.digest(plan_path) == PLAN_SHA,
              "root prospective actual run declaration changed")
    sources = {str(path.relative_to(p.ROOT)): p.prior.digest(path) for path in p.publication_sources()}
    if args.mode == "policy":
        p.require(not p.RUN.exists() and not p.OUTPUT.exists(), "prospective policy must precede actual episode")
        p.require(p.prior.digest(OLD_POLICY) == OLD_POLICY_SHA, "original v2 policy changed")
        policy = json.loads(OLD_POLICY.read_text())
        policy.update(provenance="HH_261004 - Fix full45 media selection and exact-source/native7200 publisher before one genuine v3 attempt; all prior failures retained.",
            schema=p.POLICY_SCHEMA, run_root=str(p.RUN), output=str(p.OUTPUT),
            created_UTC=datetime.now(timezone.utc).isoformat(),
            run_declaration=dict(path=str(plan_path), sha256=PLAN_SHA),
            publisher=dict(path=str(Path(p.__file__).resolve()), sha256=args.publisher_sha256),
            publication_source_sha256=sources)
        sha = save_new(p.POLICY_PATH, policy)
        seal = dict(provenance="HH_261004 - Root reviewed the exact publisher and CPU proof before execution.",
            publisher_sha256=args.publisher_sha256, CPU_test_sha256=args.test_sha256,
            root_sealer_sha256=p.prior.digest(Path(__file__).resolve()),
            full_publication_source_sha256=sources, prospective_policy_sha256=sha,
            simulator_started=False, model_promoted=False)
        save_new(p.POLICY_PATH.parent / "ROOT_publication_source_seal.json", seal)
        print(json.dumps(dict(mode=args.mode, policy_sha256=sha, publication_source_count=len(sources))))
        return
    policy = json.loads(p.POLICY_PATH.read_text())
    seal = json.loads((p.POLICY_PATH.parent / "ROOT_publication_source_seal.json").read_text())
    p.require(p.prior.digest(p.POLICY_PATH) == seal["prospective_policy_sha256"]
        and sources == policy["publication_source_sha256"] == seal["full_publication_source_sha256"]
        and seal["publisher_sha256"] == args.publisher_sha256 and seal["CPU_test_sha256"] == args.test_sha256,
        "prospective media/source seal changed")
    caller_path = p.POLICY_PATH.parent / "ROOT_actual_publication_caller.json"
    if args.mode == "caller":
        p.require(p.RUN.is_dir() and not p.OUTPUT.exists(), "actual run required before new media caller")
        files = {name: p.prior.digest(p.RUN / name) for name in sorted(p.previous.inventory(p.RUN))}
        p.require(files["declaration.json"] == PLAN_SHA, "actual run declaration changed")
        value = dict(schema=p.CALLER_SCHEMA,
            provenance="HH_261004 - Inventory every actual v3 source and failed tail before publishing; no favorable selection.",
            run_root=str(p.RUN), output=str(p.OUTPUT), attempts=1,
            PNG_index_policy=p.PNG_INDEX_POLICY, PNG_percent_numerators=p.PNG_PERCENT_NUMERATORS,
            expected_full_PNG_camera_indices=p.FULL450_PNG_CAMERA_INDICES,
            publication_source_sha256=sources, font_sha256=policy["font_sha256"],
            prospective_publication_policy=dict(path=str(p.POLICY_PATH), sha256=seal["prospective_policy_sha256"]),
            run_files_sha256=files)
        sha = save_new(caller_path, value)
        print(json.dumps(dict(mode=args.mode, caller_sha256=sha, raw_file_count=len(files))))
        return
    caller_sha = p.prior.digest(caller_path)
    caller, bundle, pins = p.load_archive(caller_path, caller_sha, decode=True)
    provenance_path = p.OUTPUT / "publication_provenance.json"
    value = json.loads(provenance_path.read_text())
    p.require(value["caller_sha256"] == caller_sha, "published actual caller changed")
    media = value["media_sha256"]
    p.require(len(media) == 6, "one whole45 GIF and five predeclared PNG required")
    for name, sha in media.items():
        path = p.OUTPUT / name
        p.require(path.is_file() and not path.is_symlink() and p.prior.digest(path) == sha, "actual media changed")
        with Image.open(path) as saved:
            if path.suffix == ".gif":
                p.require(saved.n_frames == 91, "whole45 GIF frame denominator changed")
                durations = []
                for i in range(saved.n_frames):
                    saved.seek(i)
                    saved.load()
                    durations.append(saved.info["duration"])
                p.require(durations == value["GIF_duration_ms"] and sum(durations) == 45000,
                          "complete declared GIF interval changed")
            else:
                saved.load()
    pins.recheck()
    p.require(p.previous.inventory(p.RUN) == set(caller["run_files_sha256"]), "actual raw membership changed")
    receipt = dict(schema="root_callback45_v3_whole_record_publication_postcheck_v1",
        provenance="HH_261004 - Verify every actual raw/source JPEG and full-interval media; keep prior v2 FAIL unchanged.",
        raw_file_count=len(caller["run_files_sha256"]), original_JPEG_count=len(bundle["cameras"]) * 6,
        native_state_count=len(bundle["states"]), camera_bundle_count=len(bundle["cameras"]),
        media_count=len(media), media_sha256=media, source_and_input_sha256=dict(pins.values),
        all_raw_and_declared_source_sha256_unchanged=True, all_original_JPEG_decode_passed=True,
        original_status=bundle["diagnostic"]["status"], full_fixed45=bundle["full_fixed45"],
        frozen_v2_failure_preserved=True, failed_tail_cropped=False,
        native_stage_closure_passed=bundle["diagnostic"].get("native_pre_delivery_stage_closure_passed") is True,
        publisher_provenance_sha256=p.prior.digest(provenance_path),
        diagnostic_rendering_not_genuine_Autoware_RViz=True, no_training_or_feature_approval=True)
    destination = p.ROOT / "storage/experiments/diagnostics/2026-10-04/callback45_v3_publication_postcheck/ROOT_actual_whole_run_and_media.json"
    sha = save_new(destination, receipt)
    pins.recheck()
    print(json.dumps({key:receipt[key] for key in ("raw_file_count", "original_JPEG_count", "native_state_count",
        "camera_bundle_count", "media_count", "original_status", "native_stage_closure_passed")}, sort_keys=True))


if __name__ == "__main__":
    main()
