"""HH_261004 - Publish one measured callback-to-plan fixed45 attempt with immutable whole-interval evidence."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shutil
import sys

from PIL import Image, ImageDraw

from . import publish_temporal_route_joint_town03_right_20261002 as previous


ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_callback45_v1"
OUTPUT = ROOT / "docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/05_town03_callback_to_plan"
POLICY_PATH = ROOT / ("storage/experiments/plans/2026-10-04/town03_known_dev_straight_callback45_v1/"
    "ROOT_prospective_publication_policy.json")
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
SCHEMA = "portable_e2e.actual_town03_known_dev_straight_fixed45_publication.20261004.callback45_v1"
CALLER_SCHEMA = SCHEMA + ".caller_pins"
POLICY_SCHEMA = "portable_e2e.town03_known_dev_straight_prospective_publication_policy.20261004.callback45_v1"
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_straight_fixed45.20261004.callback45_v1"
CALLBACK_SCHEMA = "portable_e2e.actual_route_joint_preview05_town03_known_dev_straight_fixed45.20261004.callback45_v1.callback_receipts"
ROUTE_SHA = "8fc442ed6cc6f6e40a4f346c2cfbda26105bd5a53c0992f7c6eb78964838cf06"
PREVIOUS_SHA = "ab179034c9bd3e73f0776245695e0edcaacb9e316d2c580102786862a73bf7d8"
PNG_INDEX_POLICY = "predeclared_full450_[0,112,224,337,449]_start_25_50_75_100"
PNG_PERCENT_NUMERATORS = {
    "01_start.png": 0,
    "02_quarter.png": 1,
    "03_half.png": 2,
    "04_three_quarters.png": 3,
    "05_last.png": 4,
}
FULL450_PNG_CAMERA_INDICES = {
    "01_start.png": 0,
    "02_quarter.png": 112,
    "03_half.png": 224,
    "04_three_quarters.png": 337,
    "05_last.png": 449,
}
EXPECTED_BUDGET_ERROR = (
    "CollectionError: BasicAgent did not finish within the driving budget reserved by "
    "the maximum total duration and stationary tail"
)
MODEL_CAMERA_ORDER = previous.MODEL_CAMERA_ORDER
require, prior, visual = previous.require, previous.prior, previous.visual


def publication_sources():
    """HH_261004 - Seal this publisher and its complete imported diagnostic renderer closure."""
    require(prior.digest(Path(previous.__file__)) == PREVIOUS_SHA, "reviewed Town03 renderer changed")
    return previous.core.audit.source_closure(
        Path(__file__), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py")


def output_for(run):
    run = previous.canonical(run)
    require(run == RUN, "only the prospectively declared Oct4 straight run is publishable")
    return previous.canonical(OUTPUT)


def camera_indices(count):
    """HH_261004 - Keep fixed half-second bins and the exact final real camera, including failed tails."""
    require(type(count) is int and 1 <= count <= 450, "actual fixed45 camera denominator required")
    return sorted(set(range(0, count, 5)) | {count - 1})


def png_indices(count):
    """HH_261004 - Resolve only the prospectively fixed 0/25/50/75/100 percent sampling rule."""
    require(type(count) is int and 1 <= count <= 450, "actual PNG camera denominator required")
    if count == 450:
        return dict(FULL450_PNG_CAMERA_INDICES)
    return {name: (count - 1) * numerator // 4 for name, numerator in PNG_PERCENT_NUMERATORS.items()}


def playback(bundle, indices):
    return previous.playback(bundle, indices)


def verify_records(states, cameras, commands, receipts, geometry, *, unlogged_preNN=False):
    require(1 < len(states) <= 900 and 1 <= len(cameras) <= 450,
            "only the complete or retained-short fixed45 denominator is publishable")
    return previous.verify_records(
        states, cameras, commands, receipts, geometry, unlogged_preNN=unlogged_preNN)


def _pin_declared_sources(pins, run, plan):
    # HH_261004 - Verify live and archived bytes for every owner-declared source, external, and asset pin.
    groups = ((plan["source_sha256"], ROOT), (plan["external_source_sha256"], None),
              (plan["asset_sha256"], None))
    for table, relative_root in groups:
        for name, expected in table.items():
            source = previous.canonical(relative_root / name if relative_root is not None else name)
            if source.is_relative_to(ROOT):
                archived = run / "source" / source.relative_to(ROOT)
            else:
                require(source.is_relative_to(CARLA_ROOT), "declared source outside owned roots")
                archived = run / "source_external" / source.relative_to(CARLA_ROOT)
            pins.file(source, expected)
            pins.file(archived, expected)


def _verify_prospective_policy(caller, pins, sources):
    """HH_261004 - Make the post-episode caller bind the pre-episode media and source decision."""
    reference = caller.get("prospective_publication_policy", {})
    require(set(reference) == {"path", "sha256"}, "exact prospective publication policy reference required")
    path = previous.canonical(reference["path"])
    require(path == POLICY_PATH, "unexpected prospective publication policy path")
    pins.file(path, reference["sha256"])
    policy = pins.json(path)
    expected_sources = {str(source.relative_to(ROOT)): prior.digest(source) for source in sources}
    require(policy.get("schema") == POLICY_SCHEMA and policy.get("created_before_actual_episode") is True
        and policy.get("actual_episode_observed") is False
        and policy.get("run_root") == str(RUN) and policy.get("output") == str(OUTPUT)
        and policy.get("attempts") == 1 and policy.get("automatic_retry") is False
        and policy.get("PNG_selection", {}).get("policy") == PNG_INDEX_POLICY
        and policy.get("PNG_selection", {}).get("camera_indices_if_full450") == FULL450_PNG_CAMERA_INDICES
        and policy.get("PNG_selection", {}).get("selection_depends_on_outcome_or_failure_location") is False,
        "pre-episode run/output/PNG policy differs")
    gif = policy.get("GIF_selection", {})
    require(gif.get("regular_bin_seconds") == .5
        and gif.get("regular_camera_indices") == list(range(0, 450, 5))
        and gif.get("exact_last_camera_index") == 449
        and gif.get("resolved_full450_camera_indices_count") == 91
        and gif.get("resolved_full450_playback_duration_ms") == 45_000
        and gif.get("failed_tail_cropped") is False
        and gif.get("selection_depends_on_outcome_or_failure_location") is False,
        "pre-episode whole-GIF policy differs")
    require(policy.get("publication_source_sha256") == expected_sources
        and caller.get("publication_source_sha256") == expected_sources,
        "caller and pre-episode publication source closure must be byte identical")
    publisher = policy.get("publisher", {})
    require(publisher.get("path") == str(Path(__file__).resolve())
        and publisher.get("sha256") == prior.digest(Path(__file__).resolve()),
        "frozen final publisher differs from prospective policy")
    require(policy.get("font_sha256") == caller.get("font_sha256"),
            "caller fonts differ from prospective policy")
    declaration = policy.get("run_declaration", {})
    declaration_path = previous.canonical(declaration.get("path", ""))
    require(declaration_path == RUN.with_name(RUN.name + "_plan.json"),
            "unexpected prospectively declared owner plan path")
    pins.file(declaration_path, declaration.get("sha256"))
    require(caller["run_files_sha256"].get("declaration.json") == declaration.get("sha256"),
            "executed declaration differs from the pre-episode plan")
    for key in ("known_DEV_site_reused",):
        require(policy.get(key) is True, "prospective known-DEV disclosure differs")
    for key in ("independent_validation", "full_RTE01_approved", "full_MAIN9_completion_claimed",
                "training_data_approved", "vehicle_control_approved", "model_promoted"):
        require(policy.get(key) is False, "prospective denial differs: " + key)
    return policy


def verify_callback_contract(plan):
    """HH_261004 - Refuse a gallery whose prospective measured-clock contract became an unmeasured V3 claim."""
    timing = plan.get("actual_callback_timing", {})
    require(timing.get("schema") == CALLBACK_SCHEMA and timing.get("attempts") == 1
        and timing.get("latest_camera_bundles_required") == 450
        and timing.get("latest_camera_callback_receipts_required") == 2700
        and timing.get("first_callback_to_plan_maximum_seconds") == .1
        and timing.get("first_callback_to_ACK_maximum_seconds") == .1
        and timing.get("actual_base_policy_observation_statement_count") == 3
        and all(timing.get(key) is True for key in (
            "all_extra_bootstrap_or_unconsumed_callbacks_retained", "source_math_inverse_AST_required",
            "original_queue_put_argument_and_return_and_exception_preserved",
            "callback_to_policy_source_SHA_required", "every_causal_history_SHA_required",
            "fresh_and_held_and_teacher_and_prefill_separate"))
        and all(timing.get(key) is False for key in (
            "additional_queue_or_source_image_buffer", "input_control_or_inference_order_changed",
            "hardware_exposure_wall_clock_observed", "ten_hz_wall_rate_proven",
            "callback_receipt_file_IO_during_capture", "model_structure_changed", "model_weights_changed",
            "controller_math_changed", "decoder_gate_or_tracker_math_changed", "original_deadline_relaxed")),
        "exact prospective callback-to-plan contract required")


def verify_callback_outcome(diagnostic, pins, episode, cameras, commands):
    """HH_261004 - Recompute displayed measured maxima while retaining failed or absent callback evidence as FAIL."""
    require(diagnostic.get("hardware_exposure_to_ACK_proven") is False
        and diagnostic.get("ten_hz_wall_rate_proven") is False,
        "simulation callbacks cannot certify hardware exposure or real-wall ten Hz")
    passed = diagnostic.get("callback_straight_motion_body_policy_wall100ms_passed") is True
    if not passed:
        require(diagnostic.get("status") == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR",
                "failed callback or driving gates require an explicit failed outcome")
        return
    require(all(diagnostic.get(key) is True for key in (
        "archived_v3_evidence_preserved", "callback_receipt_identity_and_SHA_passed",
        "callback_plan_ACK_chain_passed", "fresh_callback_to_plan_wall100ms_passed",
        "fresh_callback_to_ACK_wall100ms_passed", "actual_six_camera_callback_observed")),
        "measured callback PASS cannot omit its actual six-camera and plan/ACK proof")
    path = episode / "callback_timing_receipts.json"
    require(str(path) in pins.values, "actual callback receipts require whole-archive pins")
    actual = pins.json(path)
    audit = diagnostic.get("actual_callback_timing", {})
    require(actual.get("schema") == CALLBACK_SCHEMA and actual.get("errors") == []
        and actual.get("callback_count") == len(actual.get("callbacks", [])) >= 2700
        and len(actual.get("bundles", [])) == len(cameras) == 450
        and len(actual.get("camera_bindings", [])) == 6
        and actual.get("inflight_callbacks") == 0 and actual.get("scalar_receipt_limit_exceeded") is False
        and actual.get("hardware_exposure_to_ACK_proven") is False
        and actual.get("ten_hz_wall_rate_proven") is False
        and audit.get("hardware_exposure_to_ACK_proven") is False
        and audit.get("ten_hz_wall_rate_proven") is False,
        "complete genuine measured callback receipts required for a PASS")
    fresh = [row for row in commands if row.get("learned_control_proposed") is True
             and row.get("held_between_camera_frames") is False]
    measured = audit.get("rows", [])
    require(len(measured) == len(fresh) == audit.get("fresh_NN_plan_count") > 0
        and audit.get("selected_callback_count") == 2700
        and audit.get("callback_count") == actual["callback_count"]
        and audit.get("extra_unselected_callback_count") == actual["callback_count"] - 2700,
        "all actual fresh plans and extra callbacks must remain in the displayed denominator")
    for row, proposal in zip(measured, fresh):
        require(row["inference_frame"] == proposal["inference_frame"]
            and row["ACK_sequence"] == proposal["ack_receipt_sequence"],
            "displayed callback plan row differs from its actual original proposal")
        for field, finish in (("callback_first_to_plan_seconds", "decoded_plan_ready_monotonic_ns"),
                              ("callback_first_to_ACK_seconds", "ACK_finished_monotonic_ns")):
            expected = (row[finish] - row["first_callback_monotonic_ns"]) / 1.e9
            require(type(expected) in (int, float) and math.isfinite(expected)
                and 0 <= expected <= .1 and row[field] == expected,
                "actual first callback to plan or ACK exceeded its unchanged 100ms gate")
    for field in ("callback_first_to_plan_seconds", "callback_first_to_ACK_seconds"):
        statistics = audit.get("metrics", {}).get(field, {})
        require(statistics.get("count") == len(measured)
            and statistics.get("maximum") == max(row[field] for row in measured),
            "displayed measured callback maximum or denominator differs")


def load_archive(caller_path, caller_sha, *, decode=False):
    """HH_261004 - Read one immutable actual archive; a short capture remains a visible FAIL denominator."""
    require("torch" not in sys.modules and "carla" not in sys.modules,
            "publication cannot run inference or a simulator")
    require(prior.digest(Path(previous.__file__)) == PREVIOUS_SHA, "reviewed Town03 renderer changed")
    pins = previous.Pins()
    pins.file(caller_path, caller_sha)
    caller = pins.json(caller_path)
    require(caller.get("schema") == CALLER_SCHEMA and caller.get("attempts") == 1
        and caller.get("PNG_index_policy") == PNG_INDEX_POLICY
        and caller.get("PNG_percent_numerators") == PNG_PERCENT_NUMERATORS
        and caller.get("expected_full_PNG_camera_indices") == FULL450_PNG_CAMERA_INDICES,
        "prospective single-attempt fixed PNG declaration required")
    run = previous.canonical(caller["run_root"])
    output = output_for(run)
    require(caller.get("output") == str(output), "unknown or crossed publication output")
    require(set(caller["run_files_sha256"]) == previous.inventory(run),
            "entire externally pinned run archive required")
    for name, expected in caller["run_files_sha256"].items():
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts and relative.as_posix() == name,
                "safe relative run member required")
        pins.file(run / relative, expected)
    sources = publication_sources()
    require(set(caller["publication_source_sha256"]) == {str(path.relative_to(ROOT)) for path in sources},
            "complete publication source closure required")
    policy = _verify_prospective_policy(caller, pins, sources)
    for path in sources:
        pins.file(path, caller["publication_source_sha256"][str(path.relative_to(ROOT))])
    for name in ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf"):
        path = Path("/usr/share/fonts/truetype/dejavu") / name
        pins.file(path, caller["font_sha256"][str(path)])

    plan, owner, diagnostic = [pins.json(run / name)
        for name in ("declaration.json", "owner_result.json", "diagnostic_report.json")]
    require(plan.get("schema") == OWNER_SCHEMA and plan.get("output") == str(run)
        and plan.get("town") == "Town03" and plan.get("scenario") == "straight"
        and plan.get("dataset_split") == "val" and plan.get("known_DEV_site_reused") is True
        and plan.get("independent_validation") is False and plan.get("attempts") == 1
        and plan.get("automatic_retry") is False and plan.get("total_capture_seconds") == 45
        and plan.get("maximum_native_state_count") == 900 and plan.get("maximum_camera_bundle_count") == 450
        and plan.get("full_original_navigation_points") == 211
        and plan.get("original_navigation_sha256") == ROUTE_SHA
        and plan.get("optimizer_updates") == 6000 and plan.get("speed_preview_s") == .5
        and plan.get("controller_target_cap_mps") == 2.4
        and plan.get("full_RTE01_approved") is False and plan.get("training_data_approved") is False
        and plan.get("vehicle_control_approved") is False and plan.get("model_promoted") is False,
        "exact fixed GOOD6000 known-DEV straight contract required")
    verify_callback_contract(plan)
    green = plan.get("owned_green_signal_fixture", {})
    require(green.get("original_own_world_traffic_actors") == "ALL" and green.get("state") == "Green"
        and green.get("frozen") is True and green.get("ignore_traffic_lights") is False
        and green.get("all_original_hazard_detectors_and_emergency_returns_unchanged") is True
        and green.get("closed_course_basic_straight_only_not_traffic_rule_performance") is True
        and plan.get("actual_scoped_camera_and_ack_transport_late_bound_before_capture") is True,
        "owned signal and actual camera/ACK disclosures required")
    # HH_261004 - Bind this separate gallery to the prospectively declared V3 recording and topology correction.
    start = plan.get("authenticated_start_corridor", {})
    require(plan.get("actual_deferred_JPEG_after_ACK_required") is True
        and start.get("upstream_lane") == [67, 0, 2]
        and start.get("enabled_only_when_body_arc_lower_bound_negative") is True
        and start.get("original_navigation_bytes_unchanged") is True,
        "exact V3 after-ACK and start-corridor declaration required")
    # HH_261004 - Require the narrow owner-schema repair and reject unrelated model or gate interventions.
    admission = plan.get("fixture_owner_schema_admission", {})
    require(admission.get("declared_owner_schema") == OWNER_SCHEMA
        and all(admission.get(key) is True for key in (
            "actual_fixture_module_global_required", "inherited_v2_deferred_recording_unchanged",
            "inherited_authenticated_start_corridor_unchanged"))
        and all(admission.get(key) is False for key in (
            "model_weights_changed", "controller_changed", "policy_changed", "route_or_map_changed",
            "camera_or_ACK_changed", "speed_or_acceleration_gate_changed", "latency_gate_changed",
            "body_area_or_inset_gate_changed")),
        "exact V3 fixture-only owner admission required")
    require(owner.get("declaration_sha256") == caller["run_files_sha256"]["declaration.json"]
        and diagnostic.get("schema") == OWNER_SCHEMA + ".diagnostic_report"
        and diagnostic.get("original_owner") == owner, "owner/diagnostic/declaration identity differs")
    for key in ("full_MAIN9_completion_claimed", "full_RTE01_approved", "training_data_approved",
                "vehicle_control_approved", "model_promoted"):
        require(diagnostic.get(key) is False, "original denial cannot be promoted: " + key)
    require(diagnostic.get("all_available_raw_files_retained_and_hashed") is True
        and diagnostic.get("raw_before_after_unchanged") is True,
        "original raw archive must remain complete and byte unchanged")
    _pin_declared_sources(pins, run, plan)

    episode = run / ("episode" if (run / "episode").is_dir() else "episode.partial")
    require(episode.is_dir(), "retained actual episode required")
    raw_names = {str(episode / name) for name in previous.inventory(episode)}
    require(set(diagnostic.get("raw_source_sha256", {})) == raw_names,
            "all actual and failed-tail raw files must remain in the diagnostic")
    for name, expected in diagnostic["raw_source_sha256"].items():
        require(pins.values.get(name) == expected, "raw snapshot differs from caller archive")
    manifest, route = pins.json(episode / "manifest.json"), pins.json(episode / "route.json")
    require(pins.values[str(episode / "route.json")] == ROUTE_SHA
        and route.get("town") == "Town03" and route.get("scenario") == "straight"
        and len(route.get("route", ())) == 211
        and manifest.get("status") == diagnostic.get("original_capture_status")
        and manifest.get("training_data_approved") is False
        and manifest.get("vehicle_control_approved") is False,
        "exact unchanged Town03 straight route and original outcome required")
    states, cameras, receipts = [pins.rows(episode / name)
        for name in ("states.jsonl", "camera_frames.jsonl", "control_receipts.jsonl")]
    summary = pins.json(episode / "learned_summary.json")
    unlogged_preNN = not (episode / "learned_control.jsonl").exists()
    if unlogged_preNN:
        counts = summary.get("counts", {})
        require(summary.get("handoff_occurred") is False
            and all(type(counts.get(key)) is int and counts[key] == 0 for key in
                ("inference_count", "prefill_count", "learned_commands_acknowledged", "emergency_commands"))
            and all(state.get("research_control_source") in (None, "expert_startup")
                    and state.get("research_inference_frame") is None for state in states),
            "missing proposal journal contradicts zero-NN evidence")
        commands = []
    else:
        commands = pins.rows(episode / "learned_control.jsonl")
    body = pins.json(run / "derived_body/summary.json")
    geometry = pins.rows(run / "derived_body/all_native_boxes_and_sweeps.jsonl.gz")
    require(body == diagnostic.get("independent_body") and body.get("all_native_boxes") == len(states)
        and body.get("all_snapshot_linear_sweeps") == len(states) - 1 and body.get("lane_inset_m") == .1,
        "all actual body and 10cm inset rows required")
    by_frame = verify_records(states, cameras, commands, receipts, geometry, unlogged_preNN=unlogged_preNN)
    require(diagnostic.get("actual_native_state_count") == len(states)
        and diagnostic.get("actual_camera_bundle_count") == len(cameras)
        and diagnostic.get("actual_ACK_receipt_count") == len(receipts),
        "actual state/camera/ACK denominators differ")
    full = len(states) == 900 and len(cameras) == 450
    phases = [state.get("capture_phase") for state in states]
    first_driving = next((index for index, phase in enumerate(phases) if phase == "driving"), len(phases))
    require(all(phase == "stationary_warmup" for phase in phases[:first_driving])
        and all(phase == "driving" for phase in phases[first_driving:]),
        "actual warmup/driving phase order required")
    if full:
        result = manifest.get("result", {})
        require(manifest.get("status") == "failed" and manifest.get("error") == EXPECTED_BUDGET_ERROR
            and first_driving == 24 and len(phases) - first_driving == 876
            and manifest.get("capture_contract", {}).get("capture_phase_schedule", {})
                .get("stationary_tail", {}).get("scheduled_ticks") == 0
            and "goal_reached" not in result and result.get("state_count") == 900
            and result.get("camera_anchor_count") == 450
            and diagnostic.get("bounded_capture_complete") is True
            and diagnostic.get("fixed45_full_denominator_passed") is True,
            "full fixed45 denominator must retain the exact expected budget tail")
        require(diagnostic.get("all_failed_tail_retained") is True,
                "complete failed-budget tail must remain explicitly retained")
    else:
        require(manifest.get("status") == "failed" and type(manifest.get("error")) is str
            and bool(manifest["error"]) and diagnostic.get("original_capture_error") == manifest["error"]
            and diagnostic.get("status") == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
            and diagnostic.get("bounded_capture_complete") is False
            and diagnostic.get("fixed45_full_denominator_passed") is False
            and diagnostic.get("learned_driving_screen_passed") is False
            and diagnostic.get("straight_motion_body_policy_wall100ms_passed") is False,
            "a retained short capture cannot become a straight PASS")
    images = []
    for camera in cameras:
        for name in visual.CAMERAS:
            path = previous.canonical(prior.safe_image(episode, camera["images"][name]))
            require(str(path) in pins.values, "real camera image lacks the caller pin")
            if decode:
                with Image.open(path) as image:
                    require(image.format == "JPEG" and image.size == (640, 360), "real JPEG ABI differs")
                    image.load()
            images.append(path)
    require(len(images) == len(set(images)) == 6 * len(cameras), "six distinct real cameras per bundle required")
    verify_callback_outcome(diagnostic, pins, episode, cameras, commands)
    trial = dict(trial_index=0,
        original_driving_screen_passed=diagnostic.get("learned_driving_screen_passed") is True,
        fresh_observed_delivery_to_ACK100ms_passed=diagnostic.get("observed_bundle_to_ACK_wall100ms_passed") is True)
    bundle = dict(episode=episode, states=states, cameras=cameras, commands=commands,
        receipts=receipts, by_frame=by_frame, geometry={row["frame"]: row for row in geometry},
        route=route["route"], trial=trial, diagnostic=diagnostic, output=output,
        unlogged_preNN=unlogged_preNN, original_learned_summary=summary, full_fixed45=full,
        prospective_publication_policy=policy,
        prospective_publication_policy_sha256=caller["prospective_publication_policy"]["sha256"])
    bundle["control_census"] = previous.control_census(bundle)
    require(bundle["control_census"]["observed_prior_learned_native_effect_count"] > 0
        or diagnostic.get("straight_motion_body_policy_wall100ms_passed") is False,
        "zero observed NN effect cannot be a driving PASS")
    pins.recheck()
    require(previous.inventory(run) == set(caller["run_files_sha256"]), "run membership changed during publication")
    return caller, bundle, pins


def outcome_text(bundle):
    census, diagnostic = bundle["control_census"], bundle["diagnostic"]
    if not census["observed_prior_learned_native_effect_count"]:
        return "NO observed NN effect / setup failure", "NN 제어 효과 0개 — 모델 직진 성능 판정 불가"
    if diagnostic.get("callback_straight_motion_body_policy_wall100ms_passed") is True:
        return "PASS known-DEV precursor / NOT RTE01 approval", "known-DEV 예비 기준 PASS(기능 승인 아님)"
    return "FAIL / all available failed tail retained", "FAIL(실패 구간 전부 보존)"


def render(bundle, index):
    """HH_261004 - Reuse six real cameras and the vehicle-centered route/travel/prior-NN-plan panel."""
    canvas = previous.renderer_call(previous.core.render, bundle, index)
    draw = ImageDraw.Draw(canvas)
    verdict, _ = outcome_text(bundle)
    draw.rectangle((0, 0, 1699, 91), fill=visual.BG)
    draw.text((24, 18), "Actual Town03 known-DEV STRAIGHT | " + verdict,
              fill=visual.WHITE, font=visual.font(23, True))
    draw.text((24, 63), "GOOD6000 / CPU4+1 / PI2.40 / preview0.5 / actual callback-to-plan clocks / fixed45",
              fill=visual.MUTED, font=visual.font(18))
    draw.rectangle((0, 801, 1699, 859), fill=visual.BG)
    draw.text((24, 807),
        f"Whole actual interval: {len(bundle['states'])}native / {len(bundle['cameras'])}six-camera / "
        f"{bundle['control_census']['observed_prior_learned_native_effect_count']} observed NN effects | MAIN0/9 SUB0/30",
        fill=visual.WHITE, font=visual.font(16))
    draw.text((24, 835),
        "Actual callback timing is NOT hardware exposure or wall 10Hz proof | recorded CARLA diagnostic, NOT Autoware/RViz",
        fill=visual.MUTED, font=visual.font(16))
    return canvas


def markdown(bundle, pngs):
    diagnostic, census = bundle["diagnostic"], bundle["control_census"]
    _, verdict = outcome_text(bundle)
    span = bundle["states"][-1]["timestamp"] - bundle["states"][0]["timestamp"]
    text = "# 실제 Town03 직진과 카메라 수신 시간 측정\n\n"
    text += verdict + "입니다. 독립 평가나 정식 RTE-01 승인이 아니며 현재 메인 0/9·하위 0/30은 변하지 않습니다.\n\n"
    audit = diagnostic.get("actual_callback_timing", {})
    metrics = audit.get("metrics", {})
    def milliseconds(field):
        value = metrics.get(field, {}).get("maximum")
        return "미확인" if value is None else f"{1000 * value:.3f}ms"
    text += ("이번에는 실제 카메라 callback의 첫 문장부터 계획 생성·명령 ACK까지 같은 단조 시계로 기록했습니다. "
        f"최대 수신→계획 {milliseconds('callback_first_to_plan_seconds')}, "
        f"수신→ACK {milliseconds('callback_first_to_ACK_seconds')}입니다. "
        "센서 하드웨어 노출 시각이나 실제 벽시계 10Hz 보장을 측정한 것은 아닙니다.\n\n")
    if not census["observed_prior_learned_native_effect_count"]:
        text += "실제 NN 제어 효과가 관측되기 전에 끝났으므로 모델 직진 성공·실패로 세지 않습니다.\n\n"
    text += (f"실제 상태 {len(bundle['states'])}개, 6카메라 묶음 {len(bundle['cameras'])}개를 전부 확인했습니다. "
        f"최초→마지막 상태 간격은 {span:.2f}초입니다. GIF는 고정 0.5초 표본과 정확한 마지막 실제 카메라를 포함하고, "
        "PNG는 실행 전 선언한 최초/25%/50%/75%/마지막 규칙만 사용합니다. 짧게 끝났다면 전체 가용 실패 꼬리를 유지하며 PASS로 바꾸지 않습니다.\n\n")
    errors = {key: diagnostic[key] for key in ("original_capture_error", "diagnostic_error", "callback_timing_diagnostic_error") if diagnostic.get(key)}
    if errors:
        text += "원본 오류(그대로 보존): `" + json.dumps(errors, ensure_ascii=False).replace("`", "'") + "`\n\n"
    text += "![실제 전체 기록](full_actual_six_cameras_2hz.gif)\n\n"
    for name in pngs:
        text += f"![{name}]({name})\n\n"
    text += ("지도 패널은 모든 장면에서 측정 차량을 중심에 두고 파랑 원 경로, 초록 실제 주행 궤적, "
        "보라색 실제 이전 ACK가 적용된 NN 계획만 표시합니다. 여섯 영상은 기록된 CARLA JPEG입니다. "
        "이는 기록 기반 진단 렌더링이며 실제 Autoware 또는 RViz 화면이 아닙니다. "
        "원본 판정·SHA·표본 인덱스는 [publication_provenance.json](publication_provenance.json)에 보존합니다.\n")
    return text


def publish(args):
    caller, bundle, pins = load_archive(
        previous.canonical(args.caller_pins), args.caller_pins_sha256, decode=args.render_all_media)
    output = bundle["output"]
    require(not output.exists() and not output.is_symlink(), "fresh create-only gallery output required")
    gif_indices = camera_indices(len(bundle["cameras"]))
    durations = playback(bundle, gif_indices)
    pngs = png_indices(len(bundle["cameras"]))
    if args.validate_only:
        return dict(validated=True, native_count=len(bundle["states"]), camera_count=len(bundle["cameras"]),
            full_fixed45=bundle["full_fixed45"], GIF_frames=len(gif_indices), PNG_camera_indices=pngs,
            rendered=False, output_created=False)
    ancestor = next(path for path in output.parents if path.exists())
    require(shutil.disk_usage(ancestor).free >= 5 * 1024**3 + 256 * 1024**2,
            "5GiB plus 256MiB publication reserve required")
    pins.recheck()
    output.mkdir(parents=True, exist_ok=False)
    frames = [render(bundle, index) for index in gif_indices]
    gif = output / "full_actual_six_cameras_2hz.gif"
    with gif.open("xb") as stream:
        frames[0].save(stream, format="GIF", save_all=True, append_images=frames[1:],
                       duration=durations, loop=0, optimize=False)
    with Image.open(gif) as saved:
        require(saved.n_frames == len(gif_indices), "whole actual GIF interval required")
        actual = []
        for index in range(saved.n_frames):
            saved.seek(index)
            actual.append(saved.info["duration"])
        require(actual == durations, "whole actual GIF timing changed")
    media = [gif]
    for name, index in pngs.items():
        path = output / name
        with path.open("xb") as stream:
            render(bundle, index).save(stream, format="PNG")
        media.append(path)
    displayed_gif = [previous.renderer_call(previous.core.displayed_receipt, bundle, index)
                     for index in gif_indices]
    displayed_png = {name: previous.renderer_call(previous.core.displayed_receipt, bundle, index)
                     for name, index in pngs.items()}
    displayed_plan_count = sum(row["prior_learned_plan_drawn"] for row in displayed_gif)
    provenance = dict(schema=SCHEMA, caller_sha256=args.caller_pins_sha256,
        prospective_publication_policy_path=str(POLICY_PATH),
        prospective_publication_policy_sha256=bundle["prospective_publication_policy_sha256"],
        source_and_input_sha256=dict(pins.values), actual_output=str(output),
        original_diagnostic=bundle["diagnostic"], original_learned_summary=bundle["original_learned_summary"],
        PNG_index_policy=PNG_INDEX_POLICY, PNG_percent_numerators=PNG_PERCENT_NUMERATORS,
        expected_full_PNG_camera_indices=FULL450_PNG_CAMERA_INDICES,
        resolved_actual_PNG_camera_indices=pngs, GIF_camera_indices=gif_indices,
        GIF_duration_ms=durations, GIF_total_duration_ms=sum(durations),
        displayed_GIF_frames=displayed_gif, displayed_PNG_frames=displayed_png,
        full_fixed45=bundle["full_fixed45"], failed_tail_cropped=False,
        exact_last_original_camera_included=True, last_native_frame=bundle["states"][-1]["frame"],
        last_camera_frame=bundle["cameras"][-1]["frame"], native_terminal_frame_without_camera_not_fabricated=True,
        six_real_recorded_cameras=True, measured_vehicle_center_fixed_in_all_scenes=True,
        route_and_traveled_trajectory_displayed=True, prior_applied_NN_plan_display_rule_enabled=True,
        actual_prior_applied_NN_plan_displayed=displayed_plan_count > 0,
        actual_prior_applied_NN_plan_GIF_frame_count=displayed_plan_count, prior_ACK_any_source=True,
        emergency_clears_purple_plan=True, teacher_startup_not_counted_as_model_control=True,
        recorded_episode_model_effect_count=bundle["control_census"]["observed_prior_learned_native_effect_count"],
        publication_executed_model_inference=False, publication_started_simulator=False,
        diagnostic_rendering_not_genuine_Autoware_RViz=True, autoware_UI_captured=False,
        actual_callback_timing= bundle["diagnostic"].get("actual_callback_timing"),
        hardware_exposure_to_ACK_proven=False, ten_hz_wall_rate_proven=False,
        synthetic_driving_image_generated=False, known_DEV_site_reused=True, independent_validation=False,
        full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
        model_promoted=False, vehicle_control_approved=False, training_data_approved=False,
        media_sha256={path.name: prior.digest(path) for path in media})
    pins.recheck()
    require(previous.inventory(Path(caller["run_root"])) == set(caller["run_files_sha256"]),
            "run membership changed during rendering")
    with (output / "publication_provenance.json").open("x") as stream:
        json.dump(provenance, stream, ensure_ascii=False, indent=2, allow_nan=False)
    with (output / "README.md").open("x") as stream:
        stream.write(markdown(bundle, pngs))
    pins.recheck()
    return dict(output=str(output), GIF_count=1, PNG_count=5, GIF_frames=len(gif_indices),
        original_raw_rewritten=False, outcome=bundle["diagnostic"]["status"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--caller-pins", required=True)
    parser.add_argument("--caller-pins-sha256", required=True)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--render-all-media", action="store_true")
    args = parser.parse_args(argv)
    require(args.validate_only != args.render_all_media,
            "choose validation-only or explicit root-authorized media rendering")
    print(json.dumps(publish(args), ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
