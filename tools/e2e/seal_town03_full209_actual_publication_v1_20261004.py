"""HH_261004 - Inventory and independently verify a completed whole-drive gallery without altering the actual attempt."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image

from . import publish_temporal_route_joint_town03_straight_full209_v1_20261004 as p


PLAN_SHA = "726ecb5e3ceabae34e098f9d5e4c694b1215518a4b19dec117bc896ebe8c9955"
POLICY_SHA = "9bd5a785a4a55cc4c2cc5d69ea8fa041a006a5f71a865a0f052444abdd3845c0"
PUBLISHER_SHA = "306c7a498def2972628a7aac89084162f53e3e74db51d06e2418f2e8a19920ca"
TEST = p.ROOT / "learning/tests/test_publish_town03_straight_full209_v1_20261004.py"
TEST_SHA = "9a4fe0a349ed51a87fa00530fe5112b51a95e7d02400e7c7ac0383e3f926ddf4"
CALLER = p.POLICY_PATH.parent / "ROOT_actual_publication_caller.json"
POSTCHECK = p.ROOT / "storage/experiments/diagnostics/2026-10-04/town03_full209_publication_postcheck/ROOT_actual_whole_run_and_media.json"


def save_new(path, value):
    # HH_261004 - Do not overwrite a caller, sealed source, gallery, failed attempt or verification receipt.
    p.require(path.resolve() == path and not path.exists() and not path.is_symlink(), "fresh canonical evidence required")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return p.prior.digest(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("caller", "postcheck"), required=True)
    args = parser.parse_args(argv)
    p.require(p.prior.digest(Path(p.__file__).resolve()) == PUBLISHER_SHA
        and p.prior.digest(TEST) == TEST_SHA, "root-reviewed frozen publisher/test changed")
    plan_path = p.RUN.with_name(p.RUN.name + "_plan.json")
    p.require(p.prior.digest(plan_path) == PLAN_SHA and p.prior.digest(p.POLICY_PATH) == POLICY_SHA,
        "pre-capture actual source and media declarations changed")
    p.require((p.RUN / "owner_result.json").is_file() and (p.RUN / "diagnostic_report.json").is_file(),
        "actual owner and complete diagnostic must end before publication inventory")
    sources = {str(path.relative_to(p.ROOT)): p.prior.digest(path) for path in p.publication_sources()}
    if args.mode == "caller":
        p.require(not p.OUTPUT.exists(), "actual run precedes this fresh gallery")
        files = {name: p.prior.digest(p.RUN / name) for name in sorted(p.previous.inventory(p.RUN))}
        p.require(files["declaration.json"] == PLAN_SHA, "actual executed declaration differs")
        fonts = {str(Path("/usr/share/fonts/truetype/dejavu") / name):
            p.prior.digest(Path("/usr/share/fonts/truetype/dejavu") / name)
            for name in ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf")}
        value = dict(schema=p.CALLER_SCHEMA,
            provenance="HH_261004 - Pin every completed actual raw/source/failure file and the frozen pure publisher before rendering; selection was fixed before capture.",
            attempts=1, run_root=str(p.RUN), output=str(p.OUTPUT), run_files_sha256=files,
            publication_source_sha256=sources, font_sha256=fonts,
            prospective_publication_policy=dict(path=str(p.POLICY_PATH), sha256=POLICY_SHA),
            root_sealer_sha256=p.prior.digest(Path(__file__).resolve()),
            publisher_source_sha256=PUBLISHER_SHA, publisher_CPU_test_sha256=TEST_SHA,
            publisher_sources_sealed_before_capture_claimed=False,
            independent_validation=False, formal_function_approval=False)
        sha = save_new(CALLER, value)
        print(json.dumps(dict(caller_sha256=sha, raw_file_count=len(files), publication_source_count=len(sources))))
        return
    caller_sha = p.prior.digest(CALLER)
    caller, bundle, pins = p.load_archive(CALLER, caller_sha, decode=True)
    provenance_path = p.OUTPUT / "publication_provenance.json"
    value = json.loads(provenance_path.read_bytes())
    p.require(value["caller_sha256"] == caller_sha and value["failed_tail_cropped"] is False,
        "gallery caller or whole failed-tail rule changed")
    indices = p.camera_indices(len(bundle["cameras"]))
    durations = p.playback(bundle, indices)
    pngs = p.png_indices(len(bundle["cameras"]))
    p.require(value["GIF_camera_indices"] == indices and value["GIF_duration_ms"] == durations
        and value["PNG_camera_indices"] == pngs and value["GIF_total_duration_ms"] == sum(durations),
        "full actual denominator or predeclared sampling changed")
    media = value["media_sha256"]
    p.require(set(media) == {"full_actual_six_cameras_2hz.gif", *pngs}, "whole GIF and all five fixed PNG required")
    for name, expected in media.items():
        path = p.OUTPUT / name
        p.require(path.is_file() and not path.is_symlink() and p.prior.digest(path) == expected, "published media changed")
        with Image.open(path) as saved:
            if path.suffix == ".gif":
                p.require(saved.n_frames == len(indices), "all actual selected GIF frames required")
                measured = []
                for index in range(saved.n_frames):
                    saved.seek(index)
                    saved.load()
                    measured.append(saved.info["duration"])
                p.require(measured == durations, "whole real timestamp display durations differ")
            else:
                saved.load()
    pins.recheck()
    p.require(p.previous.inventory(p.RUN) == set(caller["run_files_sha256"]), "whole original raw/source membership changed")
    receipt = dict(schema="root_town03_full209_whole_record_publication_postcheck.20261004.v1",
        provenance="HH_261004 - Recheck every original file and JPEG plus every media frame after publication; no research result becomes approval.",
        original_status=bundle["diagnostic"]["status"], raw_file_count=len(caller["run_files_sha256"]),
        native_state_count=len(bundle["states"]), camera_bundle_count=len(bundle["cameras"]),
        original_JPEG_count=6 * len(bundle["cameras"]), media_count=len(media),
        GIF_frame_count=len(indices), GIF_duration_ms=sum(durations), nominal_last_camera_display_hold_s=.1,
        final_display_hold_is_new_observation=False, media_sha256=media,
        source_and_input_sha256=dict(pins.values), all_original_JPEG_decode_passed=True,
        all_raw_and_declared_source_sha256_unchanged=True, original_raw_membership_unchanged=True,
        failed_tail_cropped=False, prospective_policy_sha256=POLICY_SHA,
        publisher_provenance_sha256=p.prior.digest(provenance_path),
        recorded_diagnostic_not_genuine_Autoware_RViz=True, independent_validation=False,
        full_RTE01_approved=False, learned_STOP_claim=False, formal_function_approval=False)
    sha = save_new(POSTCHECK, receipt)
    pins.recheck()
    print(json.dumps(dict(postcheck_sha256=sha, **{name: receipt[name] for name in
        ("original_status", "raw_file_count", "native_state_count", "camera_bundle_count",
         "original_JPEG_count", "media_count", "GIF_frame_count", "GIF_duration_ms")})))


if __name__ == "__main__":
    main()
