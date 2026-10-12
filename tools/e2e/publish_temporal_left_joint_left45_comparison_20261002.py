"""HH_261002 - Prepared full-record GOOD6000/LEFT20k publication; no inference, actuation or promotion.

ROOT must externally pin BOTH complete attempt archives before this reader runs.
Recorded startup, learned control, failures and nonlearned coast stay distinct.
"""
from __future__ import annotations

import argparse
import ast
import copy
import json
import math
from pathlib import Path
import shutil
import sys

from PIL import Image, ImageDraw

from . import publish_town07_natural_goal_stop_teacher_v6_20261002 as recorded
from . import publish_temporal_route_joint_town03_right_20261002 as archive

ROOT, visual, require = recorded.ROOT, recorded.visual, recorded.require
SCHEMA = "portable_e2e.actual_left_joint_left45_comparison_publication.20261002.v1"
CALLER_SCHEMA = SCHEMA + ".caller_pins"
CAPTURE_SCHEMA = "portable_e2e.actual_left_joint_left45.20261002.v1"
OWNER_SCHEMA = CAPTURE_SCHEMA + ".owned"
AUTHORITY_SCHEMA = "portable_e2e.temporal_left_joint_completed_run_authority.20261002.v1"
ARMS = ("parent6000", "LEFT20000")
PAIR = ROOT / "storage/experiments/validation/2026-10-02/temporal_left_joint_left45_v1"
OUTPUT = ROOT / "docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/23_actual_left_joint_left45_comparison"
ROUTE_SHA = "fb6741ff24da5ced4a14dd6e97370a710e4a0e426c0ae3b82e8b011d55bf7560"
COUNTS = dict(native=1055, camera=528, ACK=1060, JPEG=3168)
FROZEN = {
    "tools/e2e/publish_town07_natural_goal_stop_teacher_v6_20261002.py": "29ccf539330aa04ee89c3042431b44ea35f26ea145e8d99eaa8b4ced82d8eb55",
    "tools/e2e/publish_temporal_route_joint_town03_right_20261002.py": "ab179034c9bd3e73f0776245695e0edcaacb9e316d2c580102786862a73bf7d8",
    "tools/e2e/publish_temporal_v2_actual_failure_20261001.py": "667a646e7f2dc34e3b02b34ecda3993ac27409434b727ac7eacaa700bbb14abe",
    "tools/e2e/publish_town07_w1_pi_gallery_20261001.py": "ecf3b011e96e9942376b3a2e5502bff86e727a54e9c128726291a05b865fd78e",
    "tools/e2e/collect_temporal_left_joint_left45_20261002.py": "884f146c5ad06dfe01587b68d2afb1c77e742989b1a6ebb5856398adac4689e5",
    "tools/e2e/research_temporal_left_joint_left45_20261002.py": "7810e2285f6d90e4e66ab85d62b2a1eca87f55777859e6756d81643b8f21f7fa",
}
PROVIDERS = ("_blank_brake_control", "checked_agent_initialization_observation", "apply_spawn_z_offset",
             "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg")
ORDER = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")


def verify_frozen():
    for name, sha in FROZEN.items():
        require(recorded.digest(ROOT / name) == sha, "frozen publication/capture provider changed: " + name)


def public_contract():
    # HH_261002 - Static source queries do not import the neural owner or read a model/capture payload.
    verify_frozen()
    sources = archive.core.audit.source_closure(Path(__file__), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py")
    sources.update(ROOT / name for name in FROZEN)
    return dict(schema=CALLER_SCHEMA, arm_order=list(ARMS), pair=str(PAIR), output=str(OUTPUT),
        expected_complete_counts=dict(COUNTS), expected_complete_phases=dict(stationary_warmup=24, driving=901, stationary_tail=130),
        source_sha256={str(p.relative_to(ROOT)): recorded.digest(p) for p in sorted(sources)},
        font_sha256={str(recorded.FONT_ROOT / name): recorded.digest(recorded.FONT_ROOT / name)
            for name in ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf")},
        GIF_rule="all_saved_cameras_every5th_plus_exact_last_actual_clock_gaps_final100ms_display_hold",
        PNG_rule="fixed_saved_camera_indices0_132_264_395_527_when_complete; same_fraction_positions_when_partial",
        both_outcomes_required=True, no_success_frame_selection=True, TRAIN_seen_not_independent=True,
        sensor_callback_entry_observed=False, exposure_to_ACK_wall_latency_proven=False,
        simulator_started=False, model_inference_executed=False, model_promoted=False, vehicle_control_approved=False)


def records_validator():
    # HH_261002 - Same frozen clocks/ACK/native/pixels; only LEFT's N bodies replace V6's N+1 bootstrap membership.
    verify_frozen()
    original = next(n for n in ast.parse(Path(recorded.__file__).read_bytes()).body
                    if isinstance(n, ast.FunctionDef) and n.name == "validate_records")
    changed = copy.deepcopy(original)
    pairs = (("len(geometry) == len(states) + 1 and set(by_frame) < set(geometry)",
              "len(geometry) == len(states) and set(by_frame) == set(geometry)"),
             ('set(geometry)-set(by_frame) == {states[0]["frame"]-1}', "set(geometry)-set(by_frame) == set()"))
    for old, new in pairs:
        old_node, new_node = (ast.parse(value, mode="eval").body for value in (old, new))
        hits = [n for n in ast.walk(changed) if isinstance(n, ast.Call) and n.args and ast.dump(n.args[0]) == ast.dump(old_node)]
        require(len(hits) == 1 and isinstance(hits[0].func, ast.Name) and hits[0].func.id == "require", "exact original body-membership guard required")
        hits[0].args[0] = new_node
    inverse = copy.deepcopy(changed)
    for old, new in pairs:
        old_node, new_node = (ast.parse(value, mode="eval").body for value in (old, new))
        hits = [n for n in ast.walk(inverse) if isinstance(n, ast.Call) and n.args and ast.dump(n.args[0]) == ast.dump(new_node)]
        require(len(hits) == 1, "one inverse membership guard required"); hits[0].args[0] = old_node
    require(ast.dump(inverse) == ast.dump(original), "another frozen record guard changed")
    namespace = dict(vars(recorded))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])), __file__, "exec"), namespace)
    return namespace["validate_records"]


def prior_ack(state, receipts, commands, by_frame):
    receipt = recorded.observed_prior_ack(state, receipts)
    choices = [d for d in commands if d.get("ack_receipt_sequence") == receipt["sequence"]]
    require(len(choices) <= 1, "one actual proposal per prior ACK required")
    decision = choices[0] if choices else None
    if decision and decision.get("diagnostic_control_source") == "diagnostic_coast_context":
        require(decision.get("control_source") == decision.get("application", {}).get("applied_source") == "none"
            and decision.get("learned_control_proposed") is False and decision.get("diagnostic_context_only") is True
            and decision.get("learned_STOP_claimed") is False and decision.get("application", {}).get("learned_control_applied") is False
            and decision.get("command_frame") == receipt["before_frame"]
            and receipt["reason"] == "next_driving_control"
            and all(abs(receipt["requested_control"][k]) == 0. for k in ("throttle", "brake", "steer"))
            and receipt["requested_control"].get("manual_gear_shift") is True and receipt["requested_control"].get("gear") == 1
            and by_frame[receipt["before_frame"]].get("moving_task", {}).get("future_support_complete") is True,
            "only actual final support ACK can be labelled nonlearned diagnostic context")
        return receipt, decision, [], "Nonlearned support boundary / NO NN STOP"
    if decision is None and receipt.get("reason") in ("driving_end_brake", "next_stationary_tail_control"):
        require(state["capture_phase"] == "stationary_tail" and all(abs(receipt["requested_control"][k]) == 0.
            for k in ("throttle", "brake", "steer")) and receipt["requested_control"].get("manual_gear_shift") is True
            and receipt["requested_control"].get("gear") == 1,
            "declared coast is prior actual zero-pedal manual1, not learned STOP")
        return receipt, None, [], "Nonlearned zero-pedal coast context"
    return archive.prior.observed_prior_ack(state, receipts, commands, by_frame)


def validate_records(states, cameras, receipts, commands, geometry, complete):
    actual = dict(native=len(states), camera=len(cameras), ACK=len(receipts))
    require(type(complete) is bool and 1 < len(states) <= COUNTS["native"] and len(cameras) <= COUNTS["camera"]
        and len(receipts) <= COUNTS["ACK"], "whole bounded available original record required")
    if complete:
        require(actual == {k: COUNTS[k] for k in actual}
            and {p: sum(s["capture_phase"] == p for s in states) for p in ("stationary_warmup", "driving", "stationary_tail")}
                == dict(stationary_warmup=24, driving=901, stationary_tail=130), "full1055/528/1060 and24/901/130 phases required")
    by_frame = records_validator()(states, cameras, receipts, geometry, actual)
    require([c["frame"] for c in cameras] == [s["frame"] for s in states][::2]
        and all(tuple(c.get("camera_order", ())) == ORDER for c in cameras), "every original camera in exact writer order required")
    require(all(r.get("status") == "ACKNOWLEDGED" and r.get("server_accepted") is True
        and type(r.get("before_frame")) is int and r["before_frame"] == r.get("after_ack_frame") for r in receipts), "full actual accepted ACK chain required")
    sequences = [d["ack_receipt_sequence"] for d in commands]
    require(len(sequences) == len(set(sequences)) and set(sequences) <= {r["sequence"] for r in receipts}
        and all(d.get("application", {}).get("acknowledged") is True for d in commands), "all unique acknowledged proposals required")
    for state in states:
        receipt, _, _, _ = prior_ack(state, receipts, commands, by_frame)
        require(state["control_transport"].get("status") == "PASS" and receipt["before_frame"] == state["frame"] - 1,
            "actual prior native ACK binding required")
        requested, observed = receipt["requested_control"], state["current_control"]
        require(all(abs(requested[k] - observed[k]) <= 1e-6 for k in ("throttle", "brake", "steer"))
            and all(requested[k] == observed[k] for k in ("hand_brake", "reverse", "manual_gear_shift"))
            and (not requested["manual_gear_shift"] or requested["gear"] == observed["gear"]), "recorded observed pedals are not an outgoing proposal")
    return by_frame


def validate_recording(bundle, records, proof):
    cameras, receipts = bundle["cameras"], {r["sequence"]: r for r in bundle["receipts"]}
    require(proof.get("schema") == CAPTURE_SCHEMA + ".actual_recording_binding"
        and tuple(proof.get("copied_provider_names", ())) == PROVIDERS
        and proof.get("all_copied_mutable_providers_current") is True and proof.get("enqueue_and_ACK_share_exact_recorder") is True
        and proof.get("sensor_callback_entry_observed") is False and proof.get("exposure_to_ACK_wall_latency_proven") is False
        and [r["frame"] for r in records] == [c["frame"] for c in cameras], "all late-bound recorder sources and entire camera denominator required")
    acknowledged = 0
    for row, camera in zip(records, cameras):
        expected = [Path(camera["images"][name]) for name in ORDER]
        written = row.get("written_files")
        require(type(written) is list and len(written) == 6 and len(set(written)) == 6
            and row.get("schema") == "portable_e2e.exact_camera_recording_after_ACK.20261001.v1"
            and any(written == [str(bundle["episode"].parent / alias / relative) for relative in expected]
                    for alias in ("episode", "episode.partial"))
            and row.get("enqueued_camera_count") == row.get("written_camera_count") == 6
            and row.get("all_queued_images_written") is True and row.get("failure") is None
            and row.get("no_source_image_dropped") is True and row.get("source_timestamp_changed") is False
            and row.get("source_pixels_changed") is False, "six exact writer-order JPEG literals under one original episode alias required")
        if row.get("acknowledged_before_recording") is True:
            receipt = receipts.get(row.get("ACK_receipt_sequence"), {})
            require(receipt.get("server_accepted") is True and receipt.get("before_frame") == receipt.get("after_ack_frame") == row["frame"], "actual sameframe recording ACK required")
            acknowledged += 1
        else:
            require(bundle["manifest"]["status"] == "failed" and row is records[-1]
                and row.get("acknowledged_before_recording") is False and row.get("ACK_receipt_sequence") is None,
                "only final failed pending source may be retained without ACK; never fabricate acceptance")
    pending = proof.get("pending_source_camera_count")
    count = proof.get("actual_recording_bundle_count")
    require(type(pending) is type(count) is type(proof.get("actual_after_ACK_receipt_count")) is int
        and proof["actual_after_ACK_receipt_count"] == acknowledged
        and ((pending == 0 and count == len(records)) or (pending == 6 and count == len(records)-1 and acknowledged == len(records)-1)),
        "truthful complete or final-pending-failure recorder census required")
    return dict(actual_camera_count=len(cameras), actual_after_ACK_count=acknowledged,
        pending_source_camera_count=pending, all_recorded_after_ACK=bool(records) and acknowledged == len(records),
        sensor_exposure_or_callback_to_ACK_latency_proven=False)


def load_pair(caller_path, caller_sha, *, decode=False):
    require("torch" not in sys.modules and "carla" not in sys.modules, "publisher cannot run a model or simulator")
    pins = archive.Pins(); pins.file(caller_path, caller_sha); caller = pins.json(caller_path)
    wanted = public_contract()
    require(type(caller) is dict and set(caller) == {"contract", "batch_plan_sha256", "case_plan_sha256", "completed_run_authority", "completed_run_authority_sha256", "runs", "publication_authorized"}
        and caller["contract"] == wanted and caller["publication_authorized"] is True
        and set(caller["runs"]) == set(ARMS) and set(caller["case_plan_sha256"]) == set(ARMS), "externally sealed BOTH-arm publication contract required")
    for name, sha in wanted["source_sha256"].items(): pins.file(ROOT / name, sha)
    for name, sha in wanted["font_sha256"].items(): pins.file(name, sha)
    pins.file(PAIR / "batch_plan.json", caller["batch_plan_sha256"]); batch = pins.json(PAIR / "batch_plan.json")
    require(batch.get("schema") == OWNER_SCHEMA + ".batch" and batch.get("arm_order") == list(ARMS)
        and batch.get("physical_case_count") == 2 and batch.get("case_sha256") == caller["case_plan_sha256"]
        and batch.get("completed_run_authority_sha256") == caller["completed_run_authority_sha256"]
        and batch.get("automatic_retry") is False and batch.get("outcome_selected_retry") is False, "same prospectively fixed BOTH model plans required")
    authority_path = archive.canonical(caller["completed_run_authority"])
    require(authority_path.is_relative_to(ROOT / "storage/experiments/plans/2026-10-02"), "separate original ROOT completion authority required")
    pins.file(authority_path, caller["completed_run_authority_sha256"]); authority = pins.json(authority_path)
    require(authority.get("schema") == AUTHORITY_SCHEMA and authority.get("mode") == "RESEARCH_ONLY"
        and authority.get("fixed_endpoint_completed") is True and authority.get("model_promoted") is False
        and authority.get("vehicle_control_approved") is False and len(authority.get("source_sha256", {})) == 30, "actual final20k completion, not model promotion, required")
    bundles = []
    for arm in ARMS:
        run = PAIR / arm; table = caller["runs"][arm]
        require(type(table) is dict and set(table) == archive.inventory(run), "entire attempt inventory including failure/source archives required")
        for name, sha in table.items():
            relative = Path(name); require(not relative.is_absolute() and ".." not in relative.parts and relative.as_posix() == name, "canonical whole-archive member required")
            pins.file(run / relative, sha)
        plan_path = PAIR / (arm + "_plan.json"); pins.file(plan_path, caller["case_plan_sha256"][arm])
        plan = pins.json(plan_path); declaration = pins.json(run / "declaration.json")
        owner, diagnostic = (pins.json(run / name) for name in ("owner_result.json", "diagnostic_report.json"))
        endpoint = authority["parent"] if arm == ARMS[0] else authority["endpoint"]
        require(plan == declaration and pins.values[str(run / "declaration.json")] == caller["case_plan_sha256"][arm]
            and plan.get("schema") == CAPTURE_SCHEMA + ".observation_task" and plan.get("arm") == arm
            and plan.get("arm_order") == list(ARMS) and plan.get("owner_output") == str(run)
            and plan.get("selected_four_endpoint_sha256") == endpoint
            and plan.get("fixed_endpoint_updates") == (6000 if arm == ARMS[0] else 20000)
            and plan.get("completed_run_authority_sha256") == caller["completed_run_authority_sha256"]
            and plan.get("speed_preview_s") == .5 and plan.get("PI_target_cap_mps") == 2.4
            and plan.get("CPU_threads") == 4 and plan.get("CPU_interop") == 1
            and plan.get("training_data_approved") is False and plan.get("model_promoted") is False
            and plan.get("vehicle_control_approved") is False, "exact original GOOD6000/LEFT20k runtime/declaration/denials required")
        require(owner.get("declaration_sha256") == caller["case_plan_sha256"][arm]
            and diagnostic.get("schema") == OWNER_SCHEMA + ".diagnostic_report" and diagnostic.get("arm") == arm
            and diagnostic.get("original_owner") == owner and diagnostic.get("model_promoted") is False
            and diagnostic.get("vehicle_control_approved") is False and diagnostic.get("independent_validation") is False
            and diagnostic.get("TRAIN_seen_route") is True, "original outcome must remain unchanged and diagnostic-only")
        for section in ("source_sha256", "external_source_sha256", "asset_sha256"):
            for name, sha in plan[section].items():
                source = archive.canonical(ROOT / name if section == "source_sha256" else name)
                origin = ROOT if source.is_relative_to(ROOT) else recorded.CARLA_ROOT
                require(source.is_relative_to(origin), "declared source escaped original owned roots")
                saved = run / ("source" if origin == ROOT else "source_external") / source.relative_to(origin)
                pins.file(source, sha); pins.file(saved, sha)
        require(all(plan["source_sha256"].get(name) == sha for name, sha in authority["source_sha256"].items()), "all30 immutable training sources required in both declared closures")
        episode = run / ("episode" if (run / "episode").is_dir() else "episode.partial")
        raw = {str(episode / name) for name in archive.inventory(episode)}
        require(set(diagnostic["raw_source_sha256"]) == raw
            and all(pins.values.get(name) == sha for name, sha in diagnostic["raw_source_sha256"].items()), "all original raw failure bytes required")
        manifest, route = (pins.json(episode / name) for name in ("manifest.json", "route.json"))
        require(pins.values[str(episode / "route.json")] == ROUTE_SHA and route["town"] == "Town01" and route["scenario"] == "left"
            and all(plan.get(k) == value for k, value in manifest["capture_contract"]["learned_LEFT45_protocol"].items())
            and manifest.get("training_data_approved") is False and manifest.get("vehicle_control_approved") is False, "unaltered original TRAIN-seen LEFT navigation/native denials required")
        states, cameras, receipts = (pins.rows(episode / name) for name in ("states.jsonl", "camera_frames.jsonl", "control_receipts.jsonl"))
        commands = pins.rows(episode / "learned_control.jsonl") if (episode / "learned_control.jsonl").exists() else []
        geometry_rows = pins.rows(run / "derived_body/all_native_bodies_and_sweeps.jsonl.gz")
        geometry = {r["frame"]: r for r in geometry_rows}; body = pins.json(run / "derived_body/summary.json")
        require(len(geometry) == len(geometry_rows) == len(states) and body == diagnostic["full_capture_body_clearance"]
            and body["full_body_denominator"] == len(states) and body["full_sweep_denominator"] == len(states)-1
            and body["ten_centimeter_inset_preserved"] is True, "entire measured body/sweep outcomes including failure required")
        complete = diagnostic.get("bounded_capture_complete")
        by_frame = validate_records(states, cameras, receipts, commands, geometry, complete)
        recorded.validate_pixels(cameras, pins.rows(episode / "actual_camera_pixels.jsonl"))
        bundle = dict(arm=arm, episode=episode, states=states, cameras=cameras, receipts=receipts, commands=commands,
            geometry=geometry, by_frame=by_frame, route=route["route"], diagnostic=diagnostic, manifest=manifest, endpoint=endpoint)
        bundle["recording_census"] = validate_recording(bundle, pins.rows(episode / "recording_after_ack.jsonl"), pins.json(episode / "actual_recording_binding.json"))
        observed = [prior_ack(s, receipts, commands, by_frame)[1] for s in states]
        learned = [s["frame"] for s, d in zip(states, observed) if d is not None
            and d.get("application", {}).get("learned_control_applied") is True]
        require(learned or diagnostic.get("joint_motion_body_policy_wall100ms_passed") is False, "zero observed NN effects cannot be a driving PASS")
        bundle["observed_prior_learned_effect_count"] = len(learned)
        image_paths = []
        for camera in cameras:
            for name in visual.CAMERAS:
                path = archive.prior.safe_image(episode, camera["images"][name]); image_paths.append(path)
                require(pins.values.get(str(path)) == camera["saved_JPEG_sha256"][name], "external full saved JPEG pin differs")
                if decode:
                    with Image.open(path) as image: require(image.format == "JPEG" and image.size == (640, 360), "whole original camera FOV required"); image.load()
        require(len(image_paths) == len(set(image_paths)) == 6*len(cameras), "all distinct original six-camera JPEGs required even in validate-only")
        bundles.append(bundle)
    pins.recheck()
    return caller, bundles, pins


def screenshot_indices(count):
    require(type(count) is int and count > 0, "nonempty original camera interval required")
    positions = (0, 132, 264, 395, 527) if count == 528 else tuple((count-1)*n//4 for n in range(5))
    return dict(zip(("01_first.png", "02_quarter.png", "03_middle.png", "04_three_quarter.png", "05_last.png"), positions))


def displayed_receipt(bundle, index):
    state = bundle["by_frame"][bundle["cameras"][index]["frame"]]
    receipt, command, plan, label = prior_ack(state, bundle["receipts"], bundle["commands"], bundle["by_frame"])
    return dict(native_frame=state["frame"], prior_ACK_sequence=receipt["sequence"], source_label=label,
        purple_plan_displayed=bool(plan), inference_frame=None if command is None else command.get("inference_frame"),
        observed_current_control=state["current_control"], capture_phase=state["capture_phase"])


def render(bundle, index):
    # HH_261002 - Keep full six-camera FOV and reviewed ego-centred math; bounded short lines cannot repeat the old clipped title.
    camera = bundle["cameras"][index]; state = bundle["by_frame"][camera["frame"]]
    receipt, _, plan, label = prior_ack(state, bundle["receipts"], bundle["commands"], bundle["by_frame"])
    out = Image.new("RGB", (1700, 860), visual.BG); draw = ImageDraw.Draw(out)
    verdict = "PASS bounded precursor" if bundle["diagnostic"]["joint_motion_body_policy_wall100ms_passed"] else "FAIL retained"
    lines = (("Actual Town01 LEFT | " + bundle["arm"] + " | " + verdict, 23, visual.WHITE),
        ("TRAIN-seen route diagnostic | CPU4 / PI2.4 / preview0.5 | NOT full route, STOP, Autoware or RViz", 18, visual.MUTED))
    for y, (text, size, color) in zip((17, 58), lines):
        require(draw.textbbox((0, 0), text, font=visual.font(size, True))[2] <= 1652, "short readable header must fit the complete canvas")
        draw.text((24, y), text, fill=color, font=visual.font(size, True))
    elapsed = (recorded.stamp(state)-recorded.stamp(bundle["states"][0]))/1e9
    draw.text((24, 100), f"t={elapsed:.2f}s | frame={state['frame']} | observed rear speed={math.hypot(state['vx'],state['vy']):.4f}m/s | {state['capture_phase']}", fill=visual.WHITE, font=visual.font(19))
    for i, name in enumerate(visual.CAMERAS):
        x, y = 24+i%3*416, 150+i//3*277
        draw.text((x, y), name, fill=visual.WHITE, font=visual.font(18, True))
        with Image.open(archive.prior.safe_image(bundle["episode"], camera["images"][name])) as picture:
            out.paste(picture.convert("RGB").resize((400, 225), getattr(Image, "Resampling", Image).LANCZOS), (x, y+30))
    route = [(p["x"], p["y"]) for p in bundle["route"]]
    traveled = [(s["x"], s["y"]) for s in bundle["states"] if s["frame"] <= state["frame"]]
    out.paste(visual.map_panel(state, bundle["geometry"][state["frame"]], route, traveled, plan), (1270, 145))
    ctrl = state["current_control"]
    draw.text((24, 731), f"OBSERVED pedals: throttle={ctrl['throttle']:.3f} brake={ctrl['brake']:.3f} steer={ctrl['steer']:.3f} | prior ACK #{receipt['sequence']}", fill=visual.WHITE, font=visual.font(18, True))
    draw.text((24, 773), label + " | purple only from an earlier accepted learned ACK", fill=visual.AMBER, font=visual.font(17))
    draw.text((24, 813), "Warmup/startup and zero-pedal coast are NOT NN driving or STOP | all failures retained | MAIN0/9 SUB0/30", fill=visual.MUTED, font=visual.font(17))
    return out


def publish(output, caller_path, caller_sha, *, validate_only=False):
    output = archive.canonical(output); require(output == OUTPUT and not output.exists(), "fresh fixed BOTH-arm category required")
    caller, bundles, pins = load_pair(caller_path, caller_sha, decode=not validate_only)
    if validate_only:
        return dict(status="BOTH_PINNED_ARCHIVES_VALIDATED_NO_RENDER_NO_OUTPUT", arm_order=list(ARMS), output_created=False,
            counts={b["arm"]: dict(native=len(b["states"]), camera=len(b["cameras"]), ACK=len(b["receipts"])) for b in bundles})
    require(shutil.disk_usage(ROOT).free >= 5*1024**3 + 256*1024**2, "unchanged5GiB floor plus256MiB BOTH-gallery reserve required")
    pins.recheck(); output.mkdir(parents=True, exist_ok=False); reports = []
    for bundle in bundles:
        target = output / bundle["arm"]; target.mkdir(); cameras = bundle["cameras"]
        indices = recorded.sampled_indices(len(cameras)); durations = recorded.playback_durations(cameras, indices)
        frames = [render(bundle, i).quantize(colors=192) for i in indices]
        gif = target / "full_recorded_interval_six_camera_ego_centered_2hz.gif"
        with gif.open("xb") as stream: frames[0].save(stream, format="GIF", save_all=True, append_images=frames[1:], duration=durations, loop=0, disposal=2, optimize=False)
        with Image.open(gif) as saved:
            encoded = sum(saved.seek(i) or saved.info["duration"] for i in range(saved.n_frames))
            require(encoded == sum(durations), "encoded whole actual camera interval changed")
            encoded_count = saved.n_frames
        media = [gif]; pngs = screenshot_indices(len(cameras))
        for name, i in pngs.items():
            path = target / name
            with path.open("xb") as stream: render(bundle, i).save(stream, format="PNG")
            media.append(path)
        pins.recheck()
        report = dict(arm=bundle["arm"], original_diagnostic=bundle["diagnostic"], selected_four_endpoint_sha256=bundle["endpoint"],
            actual_counts=dict(native=len(bundle["states"]), camera=len(cameras), ACK=len(bundle["receipts"]), JPEG=len(cameras)*6),
            declared_complete_counts=dict(COUNTS), original_bounded_capture_complete=bundle["diagnostic"].get("bounded_capture_complete"),
            full_available_record_retained=True, failed_tail_cropped=False, GIF_camera_indices=indices, GIF_duration_ms=durations,
            GIF_encoded_frame_count=encoded_count, GIF_encoded_total_duration_ms=encoded, final_added_display_hold_ms=100,
            actual_camera_span_ns=recorded.stamp(cameras[-1])-recorded.stamp(cameras[0]), PNG_camera_indices=pngs,
            displayed_GIF_frames=[displayed_receipt(bundle, i) for i in indices],
            observed_prior_learned_effect_count=bundle["observed_prior_learned_effect_count"], recording_census=bundle["recording_census"],
            media_sha256={p.name: recorded.digest(p) for p in media})
        reports.append(report)
        with (target / "publication_provenance.json").open("x") as stream: json.dump(report, stream, indent=2, allow_nan=False)
    pins.recheck()
    for arm in ARMS: require(archive.inventory(PAIR/arm) == set(caller["runs"][arm]), "whole original inventory changed during rendering")
    provenance = dict(schema=SCHEMA, caller_sha256=caller_sha, arm_order=list(ARMS), attempts=2, results=reports,
        source_and_input_sha256=dict(pins.values), actual_CARLA_recorded_images=True, Autoware_RViz_UI=False,
        model_inference_executed=False, simulator_started=False, groundtruth_future_used_as_model_plan=False,
        sensor_exposure_or_callback_to_ACK100ms_proven=False, TRAIN_seen_not_independent=True,
        model_promoted=False, vehicle_control_approved=False, main_approved=0, subfunctions_approved=0)
    with (output / "publication_provenance.json").open("x") as stream: json.dump(provenance, stream, indent=2, allow_nan=False)
    text = "# 실제 NN 좌회전 비교 — 두 시도 전체 기록\n\n같은 TRAIN 사용 경로·CPU4·PI2.4·0.5초 조건의 GOOD6000/LEFT20k 비교입니다. 독립 평가·목적지 정차·정식 기능 승인이 아니며 메인0/9·하위0/30입니다.\n\n"
    for report in reports:
        arm, diag = report["arm"], report["original_diagnostic"]
        text += f"## {arm} — {diag['status']}\n\n실제 분모 `{report['actual_counts']}`; 원래 전체 완료 `{report['original_bounded_capture_complete']}`. 실패와 원본 판정을 그대로 보존합니다.\n\n![전체 실제 기록]({arm}/{gif.name})\n\n"
        for name in report["PNG_camera_indices"]: text += f"[{name}]({arm}/{name}) · "
        text += "\n\n"
    text += "파랑은 원 경로, 초록은 현재까지 측정 궤적, 보라는 현재 상태보다 앞서 ACK된 모델 계획입니다. 긴급제동·진단 경계·coast에서는 오래된 보라를 지웁니다. 페달은 실제 관측값이며 교사 출발/무페달 coast를 NN 제어나 STOP 성공으로 세지 않습니다. 실제 CARLA 기록 기반 화면이지 Autoware/RViz 촬영이 아닙니다. 전체 SHA·실패·시계는 [provenance](publication_provenance.json)에 남겼습니다.\n"
    with (output / "README.md").open("x") as stream: stream.write(text)
    pins.recheck()
    return dict(status="BOTH_ACTUAL_RECORDED_GALLERIES_CREATED_NO_APPROVAL", output=str(output), GIF=2, PNG=10,
        provenance_sha256=recorded.digest(output / "publication_provenance.json"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--print-contract", action="store_true"); parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--caller-pins", type=Path); parser.add_argument("--caller-pins-sha256")
    parser.add_argument("--output", type=Path, default=OUTPUT); args = parser.parse_args(argv)
    if args.print_contract:
        require(not args.validate_only and args.caller_pins is None and args.caller_pins_sha256 is None, "static source contract only")
        print(json.dumps(public_contract(), indent=2, allow_nan=False)); return 0
    require(args.caller_pins is not None and args.caller_pins_sha256 is not None, "separate pinned ROOT publication caller required")
    print(json.dumps(publish(args.output, args.caller_pins, args.caller_pins_sha256, validate_only=args.validate_only), allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
