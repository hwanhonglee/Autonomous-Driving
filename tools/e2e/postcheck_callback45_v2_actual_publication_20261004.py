"""HH_261004 - Verify the entire frozen failed archive and genuine recorded media after publication."""
from __future__ import annotations

import json
from pathlib import Path

from PIL import Image

from . import publish_temporal_route_joint_town03_straight_callback45_v2_20261004 as publication


def main():
    caller_path = publication.POLICY_PATH.parent / "ROOT_actual_publication_caller.json"
    caller_sha = "88fa193dd88bf215e942da3e1ec417dd1bec81a1c14b2ea83682230cfaa76967"
    caller, bundle, pins = publication.load_archive(caller_path, caller_sha, decode=True)
    provenance_path = publication.OUTPUT / "publication_provenance.json"
    provenance = json.loads(provenance_path.read_text())
    publication.require(provenance["caller_sha256"] == caller_sha, "published caller changed")
    media = provenance["media_sha256"]
    publication.require(len(media) == 6, "one whole-interval GIF and five fixed PNGs required")
    for name, expected in media.items():
        path = publication.OUTPUT / name
        publication.require(not path.is_symlink() and path.is_file(), "regular published media required")
        publication.require(publication.prior.digest(path) == expected, "published media SHA changed")
        with Image.open(path) as saved:
            if path.suffix == ".gif":
                publication.require(saved.n_frames == 91, "whole actual GIF census changed")
                durations = []
                for frame in range(saved.n_frames):
                    saved.seek(frame)
                    saved.load()
                    durations.append(saved.info["duration"])
                publication.require(durations == provenance["GIF_duration_ms"], "actual playback changed")
            else:
                saved.load()
    pins.recheck()
    publication.require(publication.previous.inventory(publication.RUN)
                        == set(caller["run_files_sha256"]), "raw archive membership changed")
    receipt = dict(
        schema="root_callback45_v2_whole_record_publication_postcheck_v1",
        provenance="HH_261004 - Actual full failure and all native JPEGs retained; no approval or regrading.",
        raw_file_count=len(caller["run_files_sha256"]),
        all_raw_and_declared_source_sha256_unchanged=True,
        all_original_JPEG_decode_passed=True,
        original_JPEG_count=len(bundle["cameras"]) * 6,
        native_state_count=len(bundle["states"]), camera_bundle_count=len(bundle["cameras"]),
        media_count=len(media), media_sha256=media,
        original_status=bundle["diagnostic"]["status"],
        full_fixed45=bundle["full_fixed45"], failed_tail_cropped=False,
        original_frozen_callback_failure_preserved=True,
        diagnostic_rendering_not_genuine_Autoware_RViz=True,
        source_and_input_sha256=dict(pins.values),
        publisher_provenance_sha256=publication.prior.digest(provenance_path),
        no_training_or_feature_approval=True,
    )
    output = publication.ROOT / "storage/experiments/diagnostics/2026-10-04/callback45_v2_publication_postcheck"
    publication.require(not output.exists() and not output.is_symlink(), "fresh postcheck directory required")
    output.mkdir(parents=True, exist_ok=False)
    path = output / "ROOT_actual_whole_run_and_media.json"
    with path.open("x", encoding="utf-8") as stream:
        json.dump(receipt, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write("\n")
    pins.recheck()
    print(json.dumps({key: receipt[key] for key in (
        "raw_file_count", "original_JPEG_count", "native_state_count", "camera_bundle_count",
        "media_count", "original_status", "all_raw_and_declared_source_sha256_unchanged",
    )}, sort_keys=True))


if __name__ == "__main__":
    main()
