"""HH_261001 - Independently audit every caller-pinned V3 trial without inference, actuation, or outcome selection."""

from __future__ import annotations

import argparse
import ast
from collections import Counter
from contextlib import contextmanager
import json
import math
from pathlib import Path
import re

from . import audit_temporal_v2_headroom_fast45_batch_20261001 as frozen
from . import audit_carla_acknowledged_control as transport

ROOT, CARLA_ROOT = frozen.ROOT, frozen.CARLA_ROOT
SCHEMA = "portable_e2e.actual_route_recovery_fast45_all_attempt_archive_audit.20261001.v1"
RECEIPT_SCHEMA = SCHEMA + ".caller_pins"
OWNER_SCHEMA = "portable_e2e.temporal_route_recovery_fast45_three_trials.20261001.v1"
EXPORT_SCHEMA = "portable_e2e.temporal_route_recovery_export_research.20261001.v3"
MODEL_ID = "portable_e2e.dense_nav.learned_route_recovery.v3.research"
FROZEN_HELPERS = {
    "tools/e2e/audit_temporal_v2_headroom_fast45_batch_20261001.py": "c5778336d2d3306c57254da123f07e2d6c34fd859affc122bf1429ab3719b0ff",
    "tools/e2e/research_temporal_route_recovery_fast45_town07_20261001.py": "078cec8e8b5694e32a93f000c15a3bf9756b5296598e9dfa7cc6b7abf76a3e2b",
    "tools/e2e/collect_temporal_route_recovery_fast45_20261001.py": "4b8e3d5ffbab373fa714c3e854858b9262343ad5a3d28c7eae7717eb5bc38447",
    "tools/e2e/audit_carla_acknowledged_control.py": "9d6fccb6874a5cdfc85e8bf5d9229435e4ba6fe6850eb63a9e83488f39bcc5bd",
    "tools/e2e/audit_carla_comfortable_v3_trial.py": "ba0c81cb0faf9d1c7f178cdea25f31861c73f416578b935a8240dcf25d53efed",
    "tools/e2e/summarize_carla_goal_stop_trials.py": "890984ed44df9dacc5112a2c2d099652b8fd0490e41c6856aea9c216a4029032",
}
require, digest, read_json, rows = frozen.require, frozen.digest, frozen.read_json, frozen.rows
percentile, stats, prior_ack, audit_body = frozen.percentile, frozen.stats, frozen.prior_ack, frozen.audit_body
_timing_active = False


def sha(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def add_pin(pins, path, expected=None):
    path = Path(path)
    value = digest(path) if expected is None else expected
    require(sha(value) and digest(path) == value, "caller/declared artifact SHA differs: " + str(path))
    require(str(path) not in pins or pins[str(path)] == value, "conflicting source pins")
    pins[str(path)] = value


def source_closure(*seeds):
    """HH_261001 - Deterministically pin imported local dependencies without importing model or simulator owners."""
    visited, pending = set(), [Path(p).resolve() for p in seeds]

    def path_for(name):
        parts = name.split(".")
        base = ROOT / ("tools" if parts[0] == "scripts" else "learning" if parts[0] == "portable_e2e" else "")
        if parts[0] not in ("scripts", "portable_e2e"):
            return None
        path = base.joinpath(*(parts[1:] if parts[0] == "scripts" else parts))
        for candidate in (path.with_suffix(".py"), path / "__init__.py"):
            if candidate.is_file(): return candidate
        return None

    while pending:
        path = pending.pop()
        if path in visited: continue
        require(path.is_relative_to(ROOT) and not path.is_symlink(), "publication dependency escaped canonical source")
        visited.add(path)
        relative = path.relative_to(ROOT)
        module = ".".join(("scripts", *relative.parts[1:]))[:-3] if relative.parts[0] == "tools" else ".".join(relative.parts[1:])[:-3]
        package = module.rsplit(".", 1)[0]
        for node in ast.walk(ast.parse(path.read_bytes())):
            targets = []
            if isinstance(node, ast.Import): targets = [item.name for item in node.names]
            elif isinstance(node, ast.ImportFrom):
                if node.level:
                    prefix = package.split(".")[:len(package.split("."))-node.level+1]
                    target = ".".join(prefix + ([node.module] if node.module else []))
                else: target = node.module or ""
                targets = [target] + [target + "." + item.name for item in node.names if item.name != "*"]
            for target in targets:
                parts = target.split(".")
                for length in range(1, len(parts)+1):
                    candidate = path_for(".".join(parts[:length]))
                    if candidate is not None and candidate not in visited: pending.append(candidate)
    return visited


@contextmanager
def declared_timing_checkpoint(checkpoint):
    # HH_261001 - Reuse frozen arithmetic under one explicit restored offline source binding, never edit old code or old outcomes.
    global _timing_active
    require(sha(checkpoint) and not _timing_active, "non-nested explicit timing checkpoint required")
    original = frozen.CHECKPOINT_SHA
    _timing_active = True; frozen.CHECKPOINT_SHA = checkpoint
    try: yield
    finally: frozen.CHECKPOINT_SHA = original; _timing_active = False


def timing_audit(commands, timing, receipts, states, cameras, declaration_sha, checkpoint):
    expected=[r for r in commands if r.get("learned_control_proposed") is True and r.get("held_between_camera_frames") is False]
    if not expected:
        require(not any(r.get("learned_control_proposed") is True and r.get("held_between_camera_frames") is False for r in timing),
            "unexpected accepted fresh timing without proposal")
        # HH_261001 - No accepted first proposal is an explicitly unproven/failed denominator, not all([])==True.
        return []
    with declared_timing_checkpoint(checkpoint):
        return frozen.audit_timing(commands, timing, receipts, states, cameras, declaration_sha)


def verify_export(metadata, plan):
    # HH_261001 - Numeric tensor equality is enforced by the pinned runtime loader; this reader verifies metadata lineage and all source identities.
    require(metadata.get("schema") == EXPORT_SCHEMA and metadata.get("model_id") == plan["model_architecture_id"] == MODEL_ID, "NEW101 tensor model/export identity differs")
    counts = dict(optimizer_updates=3000, known_total_optimizer_updates=6200, training_episode_count=18,
                  recovery_training_episode_count=9, replay_training_episode_count=9, holdout_episode_count=2)
    require(all(type(metadata.get(k)) is int and metadata[k] == v for k,v in counts.items()), "fixed3000/18TRAIN/2HOLDOUT lineage differs")
    require(metadata.get("backbone_frozen") is True and metadata.get("model_promoted") is False
            and metadata.get("vehicle_control_approved") is False and metadata.get("inference_loader_authorized") is False
            and metadata.get("stop_intent_trained") is False, "denied export scope changed")
    require(metadata.get("recovery_data_revision") == "prospective_actual_signed_recovery_moving_v4"
            and metadata.get("new_signed_recovery_cohort_moving_only") is True
            and metadata.get("startup_recovery_fitted") is False
            and metadata.get("raw_startup_retained_outside_new_signed_recovery_training") is True
            and metadata.get("fixed_recovery_native_frame_offsets") == [80,682]
            and metadata.get("fixed_recovery_driving_relative_window_seconds") == [4.0,34.1], "actual NEW rolling V4 training scope required; no startup claim")
    shapes, dtypes = metadata.get("state_tensor_shapes"), metadata.get("state_tensor_dtypes")
    require(isinstance(shapes,dict) and isinstance(dtypes,dict) and len(shapes) == len(dtypes) == 101
            and set(shapes) == set(dtypes) and all(v == "float32" for v in dtypes.values())
            and len(metadata.get("trainable_state_names",[])) == 4, "frozen97/learned4/101 float32 export ABI differs")
    require(metadata["checkpoint_sha256"] == plan["source_checkpoint_sha256"], "prospectively fixed final3000 checkpoint differs")
    sources = metadata["source_sha256"]
    require(len(sources) == 24 and "tools/e2e/train_temporal_route_recovery_moving_v4_20261001.py" in sources
            and all(plan["source_sha256"].get(name) == value for name,value in sources.items()), "all24 learning sources must be in runtime declaration/archive closure")


def verify_contract(plan, trial):
    require(plan.get("schema") == OWNER_SCHEMA and plan.get("trial_index") == trial
            and plan.get("total_capture_seconds") == 45 and plan.get("candidate_count") == 1
            and plan.get("controller_target_cap_mps") == 2.4 and plan.get("model_speed_cap_mps") == 2.5
            and plan.get("native_actor_planar_speed_maximum_mps") == 2.7
            and plan.get("policy_deadline_seconds") == .1 and plan.get("policy_deadline_relaxed") is False
            and plan.get("runtime_route_as_output_replacement") is False and plan.get("oracle_future_inputs") is False,
            "unchanged physical V3 trial domain required")
    require(plan.get("model_architecture_id")==MODEL_ID and plan.get("no_teacher_fallback_after_handoff") is True
        and plan["sensor_and_ACK_screen"]["native_state_count"]==900 and plan["sensor_and_ACK_screen"]["camera_bundle_count"]==450
        and plan["body_clearance_screen"]["all_native_boxes"]==900 and plan["body_clearance_screen"]["all_snapshot_linear_sweeps"]==899
        and plan["body_clearance_screen"]["lane_inset_m"]==.1, "original full-camera/body/safety declaration differs")
    motion = plan["precursor_motion_screen"]
    limits = dict(net_learned_progress_minimum_m=90., unintended_stationary_dwell_strictly_below_s=1.,
                  stationary_threshold_mps=.05, cte_p95_maximum_m=.35, cte_absolute_maximum_m=.75,
                  collisions_maximum=0, lane_invasions_maximum=0, rejected_learned_proposals_maximum=0)
    require(all(motion.get(k) == value for k,value in limits.items()), "no audit gate weakening allowed")


def independent_motion(states, geometry, commands):
    # HH_261001 - Once the first applied learned ACK has an observed effect, retain EVERY later native state, including failure/stop tails.
    applied = [r for r in commands if r.get("application",{}).get("learned_control_applied") is True]
    sequences = {r["ack_receipt_sequence"] for r in applied}
    first = next((i for i,s in enumerate(states) if s["control_transport"]["expected_receipt_sequence"] in sequences), None)
    post, geoms = (states[first:], geometry[first:]) if first is not None else ([],[])
    complete = bool(post) and all(s.get("capture_phase") == "driving" and s["control_transport"]["status"] == "PASS"
        and s["control_transport"]["expected_receipt_sequence"] in sequences for s in post)
    cte = [abs(g["independent_actor_route_distance_m"]) for g in geoms]
    progress = geoms[-1]["independent_actor_arc_m"]-geoms[0]["independent_actor_arc_m"] if len(geoms)>1 else None
    longest = dwell = 0.
    for state in post:
        dwell = dwell+.05 if math.hypot(*state["world_velocity_carla"][:2]) <= .05 else 0.
        longest = max(longest,dwell)
    base_v = [math.hypot(s["vx"],s["vy"]) for s in post]
    world_v = [math.hypot(*s["world_velocity_carla"][:2]) for s in post]
    fresh = [r for r in commands if r.get("learned_control_proposed") is True and r.get("held_between_camera_frames") is False]
    zero_proposals = [r for r in fresh if r.get("control",{}).get("desired_speed_mps",math.inf) <= .05]
    zero_applied = [r for r in applied if r.get("control",{}).get("desired_speed_mps",math.inf) <= .05]
    emergency = [r for r in commands if r.get("control_source") == "emergency_brake"]
    collision, invasion = sum(bool(s.get("collision")) for s in states), sum(bool(s.get("lane_invasion")) for s in states)
    speed_ok = bool(post) and all(math.isfinite(v) and v<=2.5 for v in base_v) and all(math.isfinite(v) and v<=2.7 for v in world_v)
    motion = complete and progress is not None and progress>=90. and longest<1. and percentile(cte,.95)<=.35 and max(cte)<=.75 \
        and collision==invasion==len(emergency)==len(zero_proposals)==len(zero_applied)==0 and speed_ok
    failure = None
    if emergency:
        command = emergency[0]
        require(all(r.get("control_source") == "emergency_brake" for r in commands[commands.index(command):]), "latched failure resumed or tail omitted")
        effect = next((s for s in states if s["control_transport"]["expected_receipt_sequence"] == command["ack_receipt_sequence"]),None)
        failure = dict(reason=command.get("emergency_reason"),outgoing_command_frame=command["command_frame"],
            ACK_sequence=command["ack_receipt_sequence"],first_observed_emergency_frame=effect["frame"] if effect else None,
            terminal_ACK_without_next_observation=effect is None,all_emergency_tail_command_count=len(emergency))
    return dict(motion_screen_passed=bool(motion), post_handoff_native_count=len(post),complete_post_handoff_interval=complete,
        native_actor_progress_m=progress,post_handoff_actor_CTE_p95_m=percentile(cte,.95) if cte else None,
        post_handoff_actor_CTE_maximum_m=max(cte) if cte else None,longest_stationary_dwell_s=longest,
        post_handoff_base_norm_maximum_mps=max(base_v) if base_v else None,
        post_handoff_actor_world_XY_norm_maximum_mps=max(world_v) if world_v else None,
        speed_envelopes_passed=speed_ok,original_zero_target_fresh_proposals=len(zero_proposals),
        original_zero_target_applied_fresh=sum(r.get("held_between_camera_frames") is False for r in zero_applied),
        original_zero_target_applied_held=sum(r.get("held_between_camera_frames") is True for r in zero_applied),
        collision_native_observations=collision,lane_invasion_native_observations=invasion,first_latched_failure=failure,
        outgoing_source_counts=dict(Counter(r["control_source"] for r in commands)),all_failed_tail_retained=True)


def audit_post_ACK_recording(folder, episode, cameras, timing, pins):
    # HH_261001 - Original partial/final path strings remain literal; resolve only their exact declared lexical aliases.
    path = episode/"recording_after_ack.jsonl"; add_pin(pins,path)
    records = rows(path); require(len(records) == len(cameras) == 450, "all450 actual post-ACK recording receipts required")
    by_frame, by_ack = {r["frame"]:r for r in records},{r["ACK_sequence"]:r for r in timing}
    require(len(by_frame)==450 and len(by_ack)==len(timing), "duplicate recording/frame/ACK identity")
    pictures = []
    for camera in cameras:
        record = by_frame[camera["frame"]]
        require(record["enqueued_camera_count"] == record["written_camera_count"] == 6
            and record["all_queued_images_written"] is True and record["failure"] is None
            and record["acknowledged_before_recording"] is True, "complete post-ACK recording proof differs")
        receipt = by_ack[record["ACK_receipt_sequence"]]
        require(receipt["ACK_server_accepted"] is True and receipt["command_frame"] == camera["frame"]
            and receipt["ACK_finished_monotonic_ns"]<=record["recording_started_monotonic_ns"]<=record["recording_finished_monotonic_ns"], "actual ACK did not precede recording")
        relatives = [Path(camera["images"][name]) for name in frozen.CAMERAS]
        require(all(not p.is_absolute() and ".." not in p.parts and len(p.parts)==3 and p.parts[:2]==("images",name) and p.suffix==".jpg"
            for name,p in zip(frozen.CAMERAS,relatives)), "unsafe six-camera lexical path")
        require(record["written_files"] in ([str(folder/alias/p) for p in relatives] for alias in ("episode.partial","episode")), "post-ACK image receipt literal layout differs")
        for relative in relatives:
            path=episode/relative
            require(path.absolute()==path.resolve() and not any(p.is_symlink() for p in (path,*path.parents)), "JPEG escaped owned canonical episode")
            add_pin(pins,path); pictures.append(str(path))
    require(len(pictures)==len(set(pictures))==2700, "all2700 distinct actual JPEGs required")
    return dict(status="PASS",bundle_count=450,image_count=2700,actual_ACK_precedes_all_recording=True,
        JPEG_bytes_hashed=True,JPEG_pixels_decoded=False,source_receipt_strings_rewritten=False)


def load_trial(batch, trial, expected_plan, caller, pins):
    folder = batch/("trial_%02d"%trial)
    episode = folder/"episode.partial"
    if not episode.is_dir(): episode=folder/"episode"
    pinned_names = dict(owner_result="owner_result.json",diagnostic_report="diagnostic_report.json",
                        body_summary="derived_body/summary.json",body_rows="derived_body/all900_native_boxes_and899_sweeps.jsonl.gz")
    for key,name in pinned_names.items(): add_pin(pins,folder/name,caller[key+"_sha256"])
    add_pin(pins,folder/"declaration.json",expected_plan["declaration_sha256"])
    plan,owner,diagnostic,body=[read_json(folder/name) for name in ("declaration.json",*list(pinned_names.values())[:3])]
    verify_contract(plan,trial)
    require(owner["declaration_sha256"]==expected_plan["declaration_sha256"] and owner["source_checkpoint_sha256"]==plan["source_checkpoint_sha256"], "owner/source declaration differs")
    for name,value in plan["source_sha256"].items():
        require(not Path(name).is_absolute() and ".." not in Path(name).parts, "unsafe declared source")
        add_pin(pins,ROOT/name,value); add_pin(pins,folder/"source"/name,value)
    metadata = None
    endpoint = plan["new_fixed_endpoint_export_sha256"]
    for section in ("asset_sha256","external_source_sha256"):
        for name,value in plan[section].items():
            source=Path(name)
            if source.is_relative_to(ROOT): archive=folder/"source"/source.relative_to(ROOT)
            else:
                require(source.is_relative_to(CARLA_ROOT), "unknown external asset/source root")
                archive=folder/"source_external"/source.relative_to(CARLA_ROOT)
            add_pin(pins,source,value); add_pin(pins,archive,value)
            if section=="asset_sha256" and source.name=="metadata.json" and value==endpoint["metadata"]:
                require(metadata is None, "ambiguous new final3000 metadata")
                metadata=read_json(archive)
    require(metadata is not None and metadata["npz_sha256"]==endpoint["weights"], "exact declared101 tensor export assets required")
    verify_export(metadata,plan)
    for name in ("manifest.json","route.json","states.jsonl","camera_frames.jsonl","learned_control.jsonl","control_receipts.jsonl","wall_timing.jsonl","active_map_opendrive.xodr"):
        add_pin(pins,episode/name)
    for name,value in body["input_sha256"].items(): add_pin(pins,name,value)
    manifest=read_json(episode/"manifest.json")
    states,cameras,commands,receipts,timing=[rows(episode/name) for name in ("states.jsonl","camera_frames.jsonl","learned_control.jsonl","control_receipts.jsonl","wall_timing.jsonl")]
    geometry=rows(folder/pinned_names["body_rows"])
    require(len(states)==len(geometry)==900 and len(cameras)==450 and len(receipts)==904, "whole900/450/904 raw trial denominator required")
    frames=[s["frame"] for s in states]
    require(frames==list(range(frames[0],frames[0]+900)) and [g["frame"] for g in geometry]==frames
        and [c["frame"] for c in cameras]==frames[::2], "complete native/geometry/camera frame chain required")
    require([r["sequence"] for r in receipts]==list(range(1,905)), "full ordered904ACK journal required")
    for state in states: prior_ack(state,receipts)
    wheelbase=manifest["coordinate_contract"]["wheelbase_m"]
    require(wheelbase==plan["recorded_base_link_wheelbase_m"], "actual wheelbase differs")
    ack=transport.analyze_transport(states,receipts,manifest["capture_contract"]["control_transport"]["bootstrap_observation"],wheelbase=wheelbase)
    camera=transport.analyze_camera_alignment(states,cameras)
    recording=audit_post_ACK_recording(folder,episode,cameras,timing,pins)
    decomposed=timing_audit(commands,timing,receipts,states,cameras,expected_plan["declaration_sha256"],plan["source_checkpoint_sha256"])
    motion=independent_motion(states,geometry,commands)
    census=audit_body(geometry,body["body_geometry"])
    body_pass=all(r["status"]=="PASS" for r in census.values())
    require(body_pass==body["body_and_swept_clearance_passed"], "body result census differs")
    owned=not owner["cleanup_errors"] and owner["source_hashes_match"] is True and owner["external_source_hashes_match"] is True
    independent=ack["status"]==camera["status"]=="PASS"
    controller=bool(decomposed)
    for row in commands:
        if not (row.get("learned_control_proposed") is True and row.get("held_between_camera_frames") is False): continue
        audit=row.get("longitudinal_pi_research",{}); target=row.get("control",{}).get("desired_speed_mps")
        controller &= isinstance(target,(int,float)) and math.isfinite(target) and audit.get("controller_target_cap_mps")==2.4 \
            and audit.get("original_model_target_mps")==target and audit.get("governed_controller_target_mps")==min(target,2.4) \
            and audit.get("observed_speed_clipped") is False and audit.get("observed_speed_mps")==row.get("measured_current_speed_mps") \
            and audit.get("one_fresh_camera_update_one_PI_integration") is True and audit.get("explicit_controller_calibration_intervention") is True \
            and audit.get("proposed_normal_tracker_control")==row.get("control")
    policy=bool(decomposed) and all(r["policy_100ms_passed"] for r in decomposed)
    driving=owned and independent and motion["motion_screen_passed"] and body_pass and policy and controller
    wall=owned and independent and bool(decomposed) and all(r["observed_delivery_to_ACK_100ms_passed"] for r in decomposed)
    require(driving==diagnostic["learned_driving_screen_passed"] and wall==diagnostic["predeclared_observed_bundle_to_ACK_wall_latency_screen_passed"], "independent driving/wall result differs from caller-pinned report")
    for item in (plan,owner,diagnostic,manifest):
        require(item.get("vehicle_control_approved") is False and item.get("model_promoted",False) is False, "raw denials changed")
    by_ACK={r["ACK_sequence"]:r for r in timing}
    rejected_policy=[dict(command_frame=r["command_frame"],inference_frame=r.get("inference_frame"),
        ACK_sequence=r["ack_receipt_sequence"],reason=r.get("emergency_reason"),policy_total_seconds=r.get("total_seconds"),
        raw_NN_seconds=r.get("raw_nn_seconds"),original_wall_timing=by_ACK.get(r["ack_receipt_sequence"]))
        for r in commands if r.get("learned_control_proposed") is not True and isinstance(r.get("total_seconds"),(int,float))]
    return dict(motion,trial_index=trial,audit_status="PASS_ARCHIVE_VERIFICATION",original_owner_status=owner["status"],
        original_capture_status=manifest["status"],source_checkpoint_sha256=plan["source_checkpoint_sha256"],
        fixed_export_sha256=endpoint,training_data_revision=metadata["recovery_data_revision"],startup_recovery_fitted=False,
        actual_native_state_count=900,actual_camera_bundle_count=450,actual_ACK_count=904,actual_JPEG_count=2700,
        independent_ACK=ack,independent_camera_alignment=camera,post_ACK_recording=recording,archived_body_census=census,
        original_driving_screen_passed=driving,original_body_screen_passed=body_pass,fresh_policy100ms_passed=policy,
        fresh_observed_delivery_to_ACK100ms_passed=wall,joint_motion_body_policy_wall100ms_passed=driving and wall,
        fresh_timing_denominator=len(decomposed),accepted_fresh_learned_timing_denominator=len(decomposed),
        accepted_fresh_timing_status="PROVEN_COMPLETE_ACCEPTED_DENOMINATOR" if decomposed else "NOT_PROVEN_NO_ACCEPTED_FRESH",
        timing_denominator_interpretation="accepted fresh learned proposals only; rejected/native-guard/latched commands are not claimed as accepted policy measurements",
        all_rejected_policy_timing_rows=rejected_policy,rejected_policy_total_row_count=len(rejected_policy),
        full_raw_commands_and_wall_receipts_pinned=True,delivery_to_ACK_ms=stats([r["delivery_to_ACK_ms"] for r in decomposed]),
        all_fresh_timing_decomposition=decomposed,above100ms_fresh_rows=[r for r in decomposed if not r["observed_delivery_to_ACK_100ms_passed"]])


def build_report(batch, batch_sha, caller_path, caller_sha):
    batch,caller_path=Path(batch),Path(caller_path)
    require(batch.absolute()==batch.resolve() and batch.is_relative_to(ROOT/"storage/experiments/validation"), "canonical owned validation batch required")
    pins={}
    plan_path=batch.with_name(batch.name+"_plan.json")
    add_pin(pins,plan_path,batch_sha); add_pin(pins,caller_path,caller_sha)
    batch_plan,caller=read_json(plan_path),read_json(caller_path)
    require(batch_plan["schema"]==OWNER_SCHEMA+".batch" and batch_plan["output"]==str(batch)
        and batch_plan["prospective_trial_count"]==3 and batch_plan["all_three_trials_required"] is True
        and batch_plan["attempts_per_trial"]==1 and batch_plan["automatic_retry"] is False
        and batch_plan["outcome_selected_retry"] is False and batch_plan["capture_seconds_per_trial"]==45
        and batch_plan["net_progress_minimum_m"]==90., "all3 prospective fixed45 trials required")
    require(caller.get("schema")==RECEIPT_SCHEMA and caller.get("batch_declaration_sha256")==batch_sha
        and isinstance(caller.get("trials"),list) and len(caller["trials"])==3
        and [r.get("trial_index") for r in caller["trials"]]==[0,1,2], "external caller must pin every original owner/report/body receipt")
    for name,value in FROZEN_HELPERS.items(): add_pin(pins,ROOT/name,value)
    for path in source_closure(Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py"): add_pin(pins,path)
    require([r["trial_index"] for r in batch_plan["trials"]]==[0,1,2], "no chosen2-of3 trials allowed")
    trials=[load_trial(batch,i,batch_plan["trials"][i],caller["trials"][i],pins) for i in range(3)]
    require(len({r["source_checkpoint_sha256"] for r in trials})==1 and all(r["fixed_export_sha256"]==trials[0]["fixed_export_sha256"] for r in trials), "same fixed final3000 endpoint across3 trials required")
    frozen.assert_pins(pins)
    joint=sum(r["joint_motion_body_policy_wall100ms_passed"] for r in trials)
    return dict(schema=SCHEMA,status="PASS_ALL_THREE_BASIC_DRIVING_AND_OBSERVED_WALL" if joint==3 else "FAIL_ALL_THREE_REQUIREMENT_ATTEMPTS_RETAINED",
        batch=str(batch),batch_declaration_sha256=batch_sha,caller_receipts_sha256=caller_sha,prospective_trials=3,trials=trials,
        driving_pass_count=sum(r["original_driving_screen_passed"] for r in trials),body_pass_count=sum(r["original_body_screen_passed"] for r in trials),
        observed_delivery_to_ACK100ms_trial_pass_count=sum(r["fresh_observed_delivery_to_ACK100ms_passed"] for r in trials),joint_motion_body_policy_wall_trial_pass_count=joint,
        all_trials_motion_body_policy_AND_wall100ms_passed=joint==3,all_actual_native_states=2700,all_actual_camera_bundles=1350,
        actual_JPEG_assets=8100,all_actual_ACKs=2712,all_failed_tail_retained=True,source_sha256=pins,
        official_function_total=9,official_subfunction_total=30,official_function_approved_count=0,official_subfunction_approved_count=0,
        startup_recovery_fitted=False,independent_map_validation=False,training_data_approved=False,model_promoted=False,vehicle_control_approved=False,
        full_RTE01_approved=False,sensor_callback_entry_unmeasured=True,exposure_to_ACK_wall_latency_proven=False,ten_hz_end_to_end_proven=False,
        exact_tensor_values_independently_reloaded=False,original_frozen97_check_enforced_by_pinned_numeric_loader=True,JPEG_pixels_decoded_by_this_audit=False)


def markdown(report):
    def number(value): return "미측정" if value is None else "%.3f"%value
    lines=["# Town07: 새 회복 모델 — 실제 45초 × 3회", "", "<!-- HH_261001 - Keep all prospective trials and distinguish basic driving from official feature approval. -->", "",
        "기본 주행 통과 %d/3 · 차체/10 cm 여유 %d/3 · 관측 bundle→ACK 100 ms %d/3 · 모두 동시 통과 %d/3." % (report["driving_pass_count"],report["body_pass_count"],report["observed_delivery_to_ACK100ms_trial_pass_count"],report["joint_motion_body_policy_wall_trial_pass_count"]),"",
        "| 회차 | 실제 전진 | CTE 95% / 최대 | 최대 수신→ACK | 기본 주행 | 벽시계 |", "|---|---:|---:|---:|---|---|"]
    for trial in report["trials"]:
        lines.append("| %d | %s m | %s / %s m | %s ms | %s | %s |" % (trial["trial_index"]+1,number(trial["native_actor_progress_m"]),number(trial["post_handoff_actor_CTE_p95_m"]),number(trial["post_handoff_actor_CTE_maximum_m"]),number(trial["delivery_to_ACK_ms"].get("maximum")),"PASS" if trial["original_driving_screen_passed"] else "FAIL","PASS" if trial["fresh_observed_delivery_to_ACK100ms_passed"] else "FAIL"))
    lines += ["", "학습한 것은 새 실제 복귀 데이터의 주행 4.0–34.1초 구간입니다. 초기 출발 회복을 학습했다고 주장하지 않습니다. 원래 97개 backbone tensor는 고정하고 4개 회복 branch tensor만 3,000 update 학습했습니다. 선택된 내비게이션 경로를 모델 출력으로 대체하지 않았습니다.", "",
        "전체 2,700 native 관측, 1,350개 실제 6카메라 묶음/8,100 JPEG, 2,712 ACK를 확인합니다. 실패 후 정지 구간도 포함하고 세 회차 중 좋은 결과만 골라 성공으로 판단하지 않습니다.", "",
        "100 ms 정책 처리와 관측 bundle delivery→ACK 벽시계 처리는 별도 기준입니다. 센서 노출/callback 진입부터의 완전한 10 Hz는 미측정입니다. 2.40 m/s 제어 headroom 보정도 모델 자체 성능과 구분합니다.", "",
        "지연 분해는 실제 수락된 fresh 모델 제안 전체를 분모로 합니다. 거절된 첫 추론과 native 안전 거절/latched 비상제동을 수락된 추론처럼 넣지 않습니다. 원래 거절 명령·벽시계 기록은 모두 별도로 보존하며, 수락된 추론이 0개인 경우 정책·벽시계 기준은 미증명/FAIL입니다.", "",
        "정식 기능 승인: 9개 중 0개 / 하위 30개 중 0개. 이 결과는 제한된 직진 기본 주행 예비 시험이며 모든 맵·회전·실차·정상 정지 기능의 승인이 아닙니다.", "",
        "[전체 독립 확인 JSON](batch_outcome.json) · [원자료·소스 SHA256](source_evidence.json)", ""]
    return "\n".join(lines)


def publish(output, *args):
    output=Path(output); require(not output.exists() and not output.is_symlink(), "fresh create-only audit output required")
    report=build_report(*args); frozen.assert_pins(report["source_sha256"])
    output.mkdir(parents=True,exist_ok=False)
    for name,value in (("batch_outcome.json",report),("source_evidence.json",report["source_sha256"])):
        with (output/name).open("x") as stream: json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
    with (output/"README.md").open("x") as stream: stream.write(markdown(report))
    frozen.assert_pins(report["source_sha256"])
    return dict(output=str(output),result_sha256=digest(output/"batch_outcome.json"),all_trials_joint_passed=report["all_trials_motion_body_policy_AND_wall100ms_passed"],simulator_or_GPU_used=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("batch","caller-receipts","output"): parser.add_argument("--"+name,type=Path,required=True)
    for name in ("batch-sha256","caller-receipts-sha256"): parser.add_argument("--"+name,required=True)
    parser.add_argument("--validate-only",action="store_true")
    args=parser.parse_args(argv)
    inputs=(args.batch,args.batch_sha256,args.caller_receipts,args.caller_receipts_sha256)
    result=build_report(*inputs) if args.validate_only else publish(args.output,*inputs)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))
    return 0


if __name__=="__main__": raise SystemExit(main())
