"""HH_261001 - Publish all three predeclared real 45-second PI repetitions without replacing raw outcomes."""

from __future__ import annotations

import json
from pathlib import Path

from PIL import Image

from . import publish_town07_w1_pi_gallery_20261001 as visual
from . import town07_w1_pi_cruise_repetition_contract_v3_20261001 as contract


OUTPUT = contract.ROOT / "docs/assets/validation/2026-10-01/town07_pi_cruise_repetition_results_v3"
CHECKS = {
    "actor_speed_at_most_27_mps", "actual_learned_progress_at_least_90_m",
    "all_body_and_sweeps_have_10_cm_intended_lane_clearance",
    "all_native_transport_snapshot_and_held_ack_binding", "camera_native_alignment",
    "cte_max_at_most_075_m", "cte_p95_at_most_035_m",
    "every_twentieth_exact_inputs_retained_within_128_MiB", "exact_fixed_45_second_capture",
    "no_collision_or_lane_events", "no_explicit_or_unexpected_model_stop",
    "no_rejected_or_emergency_commands", "original_all_inference_pixels_tensors_and_raw_outputs_bound",
    "unchanged_point_one_second_inference_control_deadline", "unintended_stationary_dwell_below_one_second",
}


def verify_summary(summary):
    # HH_261001 - Publish failures alongside passes; never drop or relabel a predeclared attempt.
    visual.require(summary.get("schema") == contract.SCHEMA + ".repetition_summary"
        and summary.get("total_predeclared_attempts") == 3
        and type(summary.get("passed_attempts")) is int and 0 <= summary["passed_attempts"] <= 3
        and type(summary.get("failed_attempts")) is int
        and summary["passed_attempts"] + summary["failed_attempts"] == 3
        and summary.get("failed_attempts_excluded") == 0,
        "all three original declared outcomes including failures required")
    expected_status = "THREE_RESEARCH_CRUISE_SCREENS_PASS" if summary["passed_attempts"] == 3 else "REPETITION_SCREEN_FAILED_RETAINED"
    visual.require(summary.get("status") == expected_status
        and len(summary.get("attempts", [])) == 3
        and sum(row["status"] == "PASS_LOW_SPEED_CRUISE_RESEARCH_SCREEN_ONLY" for row in summary["attempts"])
            == summary["passed_attempts"], "summary count and actual outcomes differ")
    visual.require(summary.get("full_feature_approval") is False
        and summary.get("full_RTE_01_approved") is False
        and summary.get("generalization_approved") is False
        and summary.get("outcome_selected_retry") is False,
        "limited same-TRAIN-route interpretation must remain explicit")
    visual.require([row["trial"] for row in summary["attempts"]] == list(contract.trial_specs()),
        "fixed three-attempt order required")


def verify_screen(screen, attempt, expected):
    visual.require(screen.get("schema") == contract.SCHEMA + ".motion_screen"
        and screen.get("trial") == attempt["trial"]
        and screen.get("declaration_sha256") == expected
        and screen.get("status") == attempt["status"]
        and screen.get("status") in ("PASS_LOW_SPEED_CRUISE_RESEARCH_SCREEN_ONLY", "FAIL_RETAINED")
        and screen.get("retained_in_three_attempt_denominator") is True,
        "original screen identity and outcome differ")
    visual.require(set(screen.get("checks", {})) == CHECKS
        and all(type(value) is bool for value in screen["checks"].values())
        and screen.get("failures") == [name for name, passed in screen["checks"].items() if not passed],
        "all fifteen original checks and every failure must remain visible")
    visual.require((screen["status"] == "PASS_LOW_SPEED_CRUISE_RESEARCH_SCREEN_ONLY") == all(screen["checks"].values()),
        "original failed screen cannot be relabelled")
    visual.require(screen.get("full_feature_approval") is False
        and screen.get("full_RTE_01_approved") is False
        and screen.get("original_goal_reached") is False
        and screen.get("original_goal_and_training_flags_not_rewritten") is True,
        "route noncompletion and original admission flags must not change")


def load_trial(attempt, expected):
    run = Path(attempt["trial"]["output"])
    screen_path = run / "cruise_motion_screen.json"
    screen = contract.read_json(screen_path)
    visual.require(contract.digest(screen_path) == attempt["motion_screen_sha256"]
        and contract.digest(run / "owner_result.json") == attempt["owner_result_sha256"],
        "sealed screen or owner outcome changed")
    verify_screen(screen, attempt, expected)
    episode = run / "episode.partial"
    states, cameras, commands = [visual.rows(episode / name) for name in
        ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl")]
    visual.require(len(states) == 900 and len(cameras) == 450, "complete forty-five-second records required")
    by_frame = {row["frame"]: row for row in states}
    geometry = {row["frame"]: row for row in screen["geometry"]}
    visual.require(len(by_frame) == len(geometry) == 900 and set(geometry) == set(by_frame),
        "actual measured body geometry must cover every native frame")
    pins = {str(screen_path.relative_to(contract.ROOT)): contract.digest(screen_path),
        str((run / "owner_result.json").relative_to(contract.ROOT)): attempt["owner_result_sha256"]}
    for name, sha in screen["raw_evidence_sha256"].items():
        path = episode / name
        visual.require(contract.digest(path) == sha, "original native evidence changed: " + name)
        pins[str(path.relative_to(contract.ROOT))] = sha
    for camera in cameras:
        state = by_frame[camera["frame"]]
        visual.require(camera["timestamp"] == state["timestamp"] and set(camera["images"]) == set(visual.CAMERAS),
            "exact camera/native observation binding required")
        for name in visual.CAMERAS:
            relative = Path(camera["images"][name])
            visual.require(not relative.is_absolute() and ".." not in relative.parts, "safe original image path required")
            path = episode / relative
            sha = contract.digest(path)
            visual.require(sha == screen["all_original_JPEG_sha256"][str(relative)],
                "original acknowledged episode image changed")
            with Image.open(path) as image:
                visual.require(image.size == (640, 360) and image.format == "JPEG", "original image ABI differs")
                image.load()
            pins[str(path.relative_to(contract.ROOT))] = sha
    return dict(arm=attempt["trial"]["trial_id"], run=run, episode=episode, states=states,
        cameras=cameras, commands=commands, by_frame=by_frame, geometry=geometry,
        route=contract.read_json(episode / "route.json")["route"], input_sha256=pins,
        screen=screen)


def main():
    visual.require(not OUTPUT.exists(), "create-only publication directory required")
    summary_path = contract.RUN_ROOT / "repetition_summary.json"
    summary_sha, summary = contract.digest(summary_path), contract.read_json(summary_path)
    verify_summary(summary)
    contract.verify_plan(contract.read_json(contract.PLAN), summary["declaration_sha256"])
    # HH_261001 - Verify the complete retained cohort before allocating any media output.
    bundles = [load_trial(attempt, summary["declaration_sha256"]) for attempt in summary["attempts"]]
    OUTPUT.mkdir(parents=True)
    artifacts = []
    for bundle in bundles:
        frames = [visual.render(bundle, index) for index in range(0, 450, 5)]
        name = bundle["arm"]
        png, gif = OUTPUT / (name + "_ego_centered.png"), OUTPUT / (name + "_actual_cameras_2hz.gif")
        frames[-1].save(png)
        frames[0].save(gif, save_all=True, append_images=frames[1:], duration=500, loop=0, optimize=False)
        visual.require(all(contract.digest(contract.ROOT / path) == sha
            for path, sha in bundle["input_sha256"].items()), "native source changed during publication")
        artifacts.append(dict(trial=name, native_states=900, original_camera_bundles=450,
            checked_original_jpegs=2700, gif_frames=90, playback_hz=2,
            png_camera_index=445, whole_capture_startup_and_original_termination_included=True,
            original_screen_status=bundle["screen"]["status"], original_failures=bundle["screen"]["failures"],
            metrics=bundle["screen"]["metrics"], files={path.name: contract.digest(path) for path in (png, gif)},
            input_sha256=bundle["input_sha256"]))
    visual.require(contract.digest(summary_path) == summary_sha, "original repetition summary changed")
    contract.write_new(OUTPUT / "visual_provenance.json", dict(
        comment="HH_261001 - Publish all actual low-speed cruise repetitions without implying feature completion.",
        source_sha256={str(Path(__file__).relative_to(contract.ROOT)): contract.digest(__file__),
            str(Path(visual.__file__).relative_to(contract.ROOT)): contract.digest(visual.__file__)},
        repetition_summary_sha256=summary_sha, declaration_sha256=summary["declaration_sha256"],
        repetition_status=summary["status"], passed_attempts=summary["passed_attempts"],
        failed_attempts=summary["failed_attempts"], failed_attempts_excluded=0,
        trials=artifacts, measured_vehicle_center_fixed_in_all_scenes=True,
        autoware_ui_captured=False, synthetic_driving_image_generated=False,
        full_feature_approval=False, full_RTE_01_approved=False, independent_generalization_approved=False))
    print(json.dumps(dict(status="ALL_THREE_ACTUAL_CAMERA_GALLERIES_CREATED", output=str(OUTPUT))))


if __name__ == "__main__":
    main()
