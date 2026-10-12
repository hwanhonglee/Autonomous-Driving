"""HH_261002 - Create-only three-arm actual learned recovery study, not full-route or MAIN9 approval."""

from __future__ import annotations

from contextlib import contextmanager
import math
from pathlib import Path
import re

from . import research_temporal_route_recovery_fast45_town07_20261001 as previous
from . import collect_temporal_signed_recovery45_20261002 as live


ROOT, base, parent = previous.ROOT, previous.previous.base, previous.previous
MODULE = "scripts.e2e.research_temporal_signed_recovery45_town07_20261002"
SCHEMA = "portable_e2e.actual_signed_recovery45_three_arms.20261002.v1"
FROZEN_OWNER_SHA = "078cec8e8b5694e32a93f000c15a3bf9756b5296598e9dfa7cc6b7abf76a3e2b"
GEOMETRY = ROOT/"storage/experiments/validation/2026-10-01/town07_signed_teacher_recovery_v4/predicted_initial_geometry.json"
GEOMETRY_SHA = "8c3fd6b87c692dbc983030dfd0173dc5e66ade7f74279a203eec4553239269d3"
require, digest, read_json = previous.require, previous.digest, previous.read_json
_fixed, _runtime, _diagnostic = previous.fixed_contract, previous.runtime_sources, previous.diagnostic_report
_capture, _summary = parent.capture_argv, parent.batch_summary
_active = False


def batch_paths(run_id):
    require(isinstance(run_id, str) and re.fullmatch(r"v[1-9][0-9]*", run_id), "fresh explicit signed-recovery version required")
    output = ROOT/("storage/experiments/validation/2026-10-02/temporal_signed_recovery45_town07_"+run_id)
    return output, output.with_name(output.name+"_plan.json")


def trial_profile():
    require(type(parent._trial) is int and 0 <= parent._trial < 3, "explicit three-arm trial required")
    return dict(live.PROFILES[parent._trial])


def capture_argv(run_id, longitudinal_pi, declaration_sha=base.PLAN_TOKEN):
    argv = _capture(run_id, longitudinal_pi, declaration_sha)
    index = argv.index("--seed")+1
    require(argv[index] == "0", "frozen baseline seed changed")
    argv[index] = str(trial_profile()["seed"])
    require("--agent-initialization" not in argv, "frozen baseline initialization argv changed")
    argv += ["--agent-initialization","after_bootstrap"]
    return argv


def initial_geometry_proof(spec):
    require(digest(GEOMETRY) == GEOMETRY_SHA, "declared earlier offline geometry proof changed")
    proof = read_json(GEOMETRY)
    matches = [row for row in proof["cases"] if row["signed_lateral_offset_ros_m"] == spec["signed_lateral_offset_ros_m"]
        and row["signed_yaw_offset_ros_deg"] == spec["signed_yaw_offset_ros_deg"]]
    require(proof["status"] == "PASS" and len(matches) == 1 and matches[0]["status"] == "PASS",
        "exact signed initial static footprint absent or below original10cm inset")
    return dict(receipt_path=str(GEOMETRY), receipt_sha256=GEOMETRY_SHA,
        exact_geometry_case=matches[0]["case_id"], seeds_not_geometry_equivalence_evidence=True,
        method="Earlier frozen OFFLINE static geometry only; actual all900/899 must still pass.",
        actual_new_capture_pass_claimed=False)


def fixed_contract(run_id, longitudinal_pi):
    spec = trial_profile()
    result = _fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA, seed=spec["seed"], agent_initialization="after_bootstrap", signed_recovery_profile=spec, capture_argv=capture_argv(run_id, True),
        experiment="Fixed V3 baseline actual signed recovery, both outward signs plus centered startup comparator",
        current_fixed_final3000_V3_weights_unchanged=True, no_new_training_or_checkpoint_selection=True,
        geometry_equivalence_proof=initial_geometry_proof(spec),
        actual_spawn_before_actor_and_sensors=True, captured_pixel_pose_transform=False,
        teacher_startup_revision="Qualified manual1/.11 launch/.025 native-time normal cap; zero normal steer<=.75m/s",
        learned_selected_decoded_path_actual_v0_PI_and_handoff_math_unchanged=True,
        normal_teacher_hooks_are_before_unchanged_BasicAgent_hazard_override=True,
        no_teacher_recovery_counted_as_learned=True, initialization_controls_or_snapshots_proxied=False,
        recovery_screen=dict(first_effect_reference="First native rear-base observation matched to a prior actual learned ACK",
            required_noncentered_case=parent._trial != 2, minimum_initial_abs_rear_CTE_m=.10,
            minimum_initial_abs_yaw_error_deg=.5, convergence_abs_rear_CTE_m=.10,
            convergence_abs_yaw_error_deg=.5, convergence_deadline_seconds=10.,
            convergence_deadline_interpretation="Enter both bands within10s of first learned effect, then remain continuously inside for2s; confirmation may occur by12s.",
            minimum_continuous_converged_seconds=2., required_initial_error_signs_match_declared_spawn=True,
            all_after_first_learned_effect_rows_retained=True,
            independent_map_or_initialization_geometry_claim=False),
        full_MAIN9_completion_claimed=False, full_route_goal_stop_approved=False,
        initialization_gear_contract=dict(actual_manual_gear_shift=True,actual_gear=1,
            parent_function_source_sha256=live.manual.PARENT_CHECKER_SOURCE_SHA,
            checker_revision=live.manual.TASK_SCHEMA, original_frame_actor_finite_PID_full_brake_guards_preserved=True),
        startup_recovery_was_not_training_scope=True,
        known_same_TRAIN_corridor_and_signed_geometry=True, unseen_seed_is_not_independent_test=True)
    return result


def runtime_sources():
    live.verify_frozen()
    require(digest(Path(previous.__file__)) == FROZEN_OWNER_SHA, "frozen V3 owner changed")
    pins = _runtime()
    pins.update({str(path.relative_to(ROOT)):digest(path)
        for path in sorted(base.imported_source_closure((Path(__file__).resolve(),)))})
    return pins


def rear_route_error(row, projector):
    """HH_261002 - Offline measured rear-base projection; this evaluator never feeds a learned control input."""
    projected = projector.project(float(row["x"]), float(row["y"]), float(row["route_progress_m"]))
    first, second = projector.points[projected.segment_index:projected.segment_index+2]
    dx, dy = float(second["x"])-float(first["x"]), float(second["y"])-float(first["y"])
    length = math.hypot(dx,dy)
    require(length > 1.e-6, "nondegenerate archived route segment required")
    cte = (dx*(float(row["y"])-float(first["y"]))-dy*(float(row["x"])-float(first["x"])))/length
    yaw = math.atan2(math.sin(float(row["yaw"])-math.atan2(dy,dx)), math.cos(float(row["yaw"])-math.atan2(dy,dx)))
    require(all(math.isfinite(value) for value in (cte,yaw,float(row["timestamp"]))), "finite native recovery metrics required")
    return cte, math.degrees(yaw)


def recovery_metrics(states, commands, route, spec):
    learned = {row["ack_receipt_sequence"] for row in commands
        if row.get("application",{}).get("learned_control_applied") is True}
    start = next((i for i,row in enumerate(states) if row.get("control_transport",{}).get("expected_receipt_sequence") in learned), None)
    if start is None:
        return dict(status="NOT_PROVEN_NO_LEARNED_EFFECT", recovery_screen_passed=False, case_screen_passed=False,
                    first_learned_effect=None, convergence=None, post_effect_native_count=0)
    projector = live.collector.RouteProjector(route)
    effects = states[start:]
    first_cte, first_yaw = rear_route_error(effects[0], projector)
    is_signed = spec["signed_lateral_offset_ros_m"] != 0.
    present = abs(first_cte) >= .10 and abs(first_yaw) >= .5 if is_signed else True
    correct_sign = first_cte*spec["signed_lateral_offset_ros_m"] > 0 and first_yaw*spec["signed_yaw_offset_ros_deg"] > 0 if is_signed else True
    first_time = float(effects[0]["timestamp"])
    inside_since, convergence = None, None
    rows = []
    complete = all(row.get("capture_phase") == "driving" and row.get("control_transport",{}).get("status") == "PASS"
        and row["control_transport"].get("expected_receipt_sequence") in learned for row in effects)
    for row in effects:
        cte,yaw = rear_route_error(row,projector)
        elapsed = float(row["timestamp"])-first_time
        rows.append(dict(frame=row["frame"], elapsed_simulation_seconds=elapsed, signed_rear_CTE_m=cte,
            signed_rear_yaw_error_deg=yaw, expected_prior_ACK_sequence=row.get("control_transport",{}).get("expected_receipt_sequence")))
        if abs(cte) <= .10 and abs(yaw) <= .5:
            if inside_since is None: inside_since=elapsed
            if convergence is None and inside_since <= 10. and elapsed-inside_since >= 2.-1.e-6:
                convergence=dict(first_converged_simulation_seconds=inside_since,
                    continuous_converged_seconds=elapsed-inside_since, confirmation_native_frame=row["frame"])
        else: inside_since=None
    status = "PASS_BOUNDED_SIGNED_RECOVERY" if present and correct_sign and complete and convergence else "FAIL_BOUNDED_SIGNED_RECOVERY"
    if not present or not correct_sign: status="NOT_PROVEN_INITIAL_PERTURBATION_REMOVED_OR_WRONG_SIGN"
    passed = bool(present and correct_sign and complete and convergence)
    if not is_signed: status="PASS_CENTER_SETUP_COMPARATOR" if passed else "FAIL_CENTER_SETUP_COMPARATOR"
    return dict(status=status, noncentered_recovery_case=is_signed,
        case_screen_passed=passed, recovery_screen_passed=passed and is_signed,
        centered_setup_screen_passed=passed and not is_signed,
        first_learned_effect=dict(native_frame=effects[0]["frame"],timestamp_simulation_seconds=first_time,
            expected_prior_ACK_sequence=effects[0]["control_transport"]["expected_receipt_sequence"],
            signed_rear_CTE_m=first_cte,signed_rear_yaw_error_deg=first_yaw),
        initial_actual_perturbation_still_present=present, initial_signed_errors_match_declared_signs=correct_sign,
        startup_teacher_recovery_is_not_learned_recovery=True, convergence=convergence,
        complete_post_effect_learned_interval=complete, post_effect_native_count=len(effects),
        rear_projection_is_offline_evaluation_not_runtime_input=True, full_post_effect_metric_rows=rows)


def diagnostic_report(output, plan, owner):
    result = _diagnostic(output, plan, owner)
    episode = output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
    try:
        states,commands = (parent.jsonl(episode/name) for name in ("states.jsonl","learned_control.jsonl"))
        manifest=read_json(episode/"manifest.json")
        spec=plan["signed_recovery_profile"]
        spawn=manifest["capture_contract"]["actual_signed_spawn"]
        changed,rear=live.task.perturb_actor_center(spawn["nominal_actor_center_carla"],wheelbase_m=2.85,
            signed_lateral_offset_ros_m=spec["signed_lateral_offset_ros_m"],signed_yaw_offset_ros_deg=spec["signed_yaw_offset_ros_deg"])
        spawn_ok=all(spawn.get(k)==v for k,v in spec.items()) and spawn["perturbed_actor_center_carla"]==changed \
            and spawn["perturbed_rear_reference_carla"]==rear and spawn["actual_requested_spawn_carla"]==dict(changed,z=changed["z"]+.5) \
            and spawn["applied_once_before_sensors_and_ticks"] is True and spawn["pixels_or_observed_poses_post_transformed"] is False \
            and manifest["result"]["actual_spawn_perturbation_call_count"]==1
        recovery = recovery_metrics(states,commands,read_json(episode/"route.json"),spec)
        parent.verify_derived_source_closure(output,plan)
    except Exception as error:
        spawn_ok=False
        recovery=dict(status="FAIL_RECOVERY_DIAGNOSTIC",recovery_screen_passed=False,case_screen_passed=False,error=type(error).__name__+": "+str(error))
    result.update(schema=SCHEMA+".diagnostic_report",actual_signed_spawn_identity_passed=spawn_ok,
        bounded_signed_recovery=recovery, full_MAIN9_completion_claimed=False)
    result["learned_driving_screen_passed"] = bool(result.get("learned_driving_screen_passed")) and spawn_ok and recovery["case_screen_passed"]
    result["status"]="PASS_BASIC_SIGNED_RECOVERY_PRECURSOR" if result["learned_driving_screen_passed"] else "FAIL_BASIC_SIGNED_RECOVERY_PRECURSOR"
    return result


def batch_summary(run_id, expected):
    """HH_261002 - Separate the two signed recovery denominators from the centered setup comparator."""
    result = _summary(run_id, expected)
    root,_ = batch_paths(run_id)
    cases=[]
    for index,spec in enumerate(live.PROFILES):
        path=root/("trial_%02d"%index)/"diagnostic_report.json"
        report=read_json(path) if path.is_file() else {}
        recovery=report.get("bounded_signed_recovery",{})
        cases.append(dict(trial_index=index,**spec,
            case_screen_passed=bool(recovery.get("case_screen_passed",False)),
            recovery_screen_passed=bool(recovery.get("recovery_screen_passed",False)),
            centered_setup_screen_passed=bool(recovery.get("centered_setup_screen_passed",False)),
            diagnostic_sha256=digest(path) if report else None))
    result.update(signed_recovery_case_count=2,centered_setup_comparator_count=1,signed_recovery_cases=cases[:2],
        centered_setup_comparator=cases[2],signed_recovery_pass_count=sum(row["recovery_screen_passed"] for row in cases[:2]),
        all_two_signed_recovery_cases_passed=all(row["recovery_screen_passed"] for row in cases[:2]),
        centered_setup_comparator_passed=cases[2]["centered_setup_screen_passed"],
        all_three_basic_screens_is_not_three_noncentered_recoveries=True,full_MAIN9_completion_claimed=False)
    return result


@contextmanager
def activated():
    """HH_261002 - Private fully restored version bindings retain frozen worker ownership and source closure."""
    global _active
    require(not _active and not previous._active and not parent._active, "signed recovery owner cannot nest")
    runtime_sources()
    names=dict(batch_paths=batch_paths,fixed_contract=fixed_contract,runtime_sources=runtime_sources,
        live=live,MODULE=MODULE,SCHEMA=SCHEMA,diagnostic_report=diagnostic_report)
    saved={name:getattr(previous,name) for name in names}
    saved_capture=parent.capture_argv
    saved_summary=parent.batch_summary
    saved_assets=dict(base.ASSETS)
    _active=True
    try:
        for name,value in names.items(): setattr(previous,name,value)
        parent.capture_argv=capture_argv
        parent.batch_summary=batch_summary
        # HH_261002 - The frozen V3 binding retains these non-weight assets in its create-only source archive.
        base.ASSETS=dict(base.ASSETS)|{GEOMETRY:GEOMETRY_SHA,
            live.manual.v2.CENTER_MANIFEST:live.manual.v2.CENTER_MANIFEST_SHA}
        with previous.bound_fixed_recovery_exports():
            yield
    finally:
        base.ASSETS=saved_assets
        parent.capture_argv=saved_capture
        parent.batch_summary=saved_summary
        for name,value in saved.items(): setattr(previous,name,value)
        _active=False


def main(argv=None):
    with activated(): return parent.main(argv)


if __name__=="__main__": raise SystemExit(main())
