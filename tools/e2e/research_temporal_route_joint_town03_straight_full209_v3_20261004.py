"""HH_261004 - Prepare a fresh synchronous recording/two-phase cleanup experiment; frozen V1/V2 failures remain immutable."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from pathlib import Path

from . import research_temporal_route_joint_town03_straight_full209_v2_20261004 as previous
from . import collect_temporal_route_joint_town03_straight_full209_v3_20261004 as live


base, ROOT = previous.base, previous.ROOT
require, digest = previous.require, previous.digest
MODULE = "scripts.e2e.research_temporal_route_joint_town03_straight_full209_v3_20261004"
SCHEMA = live.OWNER_SCHEMA
PARENT_SHA256 = "fc40b314ed3f7aa33d302699204a02ed7e6a4d85ad5ac04c0f64153a016150ca"
# HH_261004 - Capture authentic functions before scoped activation so no diagnostic can recurse through its changed alias.
_fixed, _sources, _diagnostic = previous.fixed_contract, previous.runtime_sources, previous.diagnostic_report
_active = False


def paths(run_id):
    require(run_id == "v3", "fresh full209 V3 requires exact v3 identity")
    output = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v3"
    return output, output.with_name(output.name + "_plan.json")


def capture_argv(run_id, longitudinal_pi, declaration_sha=base.PLAN_TOKEN):
    # HH_261004 - Change no capture option beyond the same declared110s seam inherited by full209 V1/V2.
    require(_active and run_id == "v3", "fresh capture requires its exclusive V3 owner scope")
    result = previous.previous._original_capture(run_id, longitudinal_pi, declaration_sha)
    index = result.index("--max-duration-sec") + 1
    require(result[index] == "45", "original fullroute duration seam changed")
    result[index] = "110"
    return result


def fixed_contract(run_id, longitudinal_pi):
    """HH_261004 - Seal recording/cleanup interventions before capture without asserting a measured gain or capability."""
    result = _fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA,
        experiment="Fresh known-DEV full209 V3 synchronous three-worker JPEG and two-phase signal restoration research",
        frozen_V1_preserved=True, frozen_V2_preserved=True, capture_raw_ABI_schema=live.SCHEMA,
        synchronous_parallel_JPEG_after_ACK=dict(
            schema=live.jpeg.SCHEMA, source="tools/e2e/parallel_exact_jpeg_after_ack_v1_20261004.py",
            worker_count=3, camera_role_order=list(live.jpeg.CAMERA_ORDER), every_six_camera_encode_submitted=True,
            all_submitted_encodes_joined_before_original_role_ordered_writes=True,
            no_background_work_or_next_tick_before_recording_finish=True, exact_original_OpenCV_JPEG_bytes_required=True,
            original_quality_and_create_only_descriptors_unchanged=True, source_BGRA_and_selected_callback_SHA_required=True,
            source_pixel_or_timestamp_retag_forbidden=True, controller_NN_weights_math_and_CPU_threads_unchanged=True,
            full_failed_interval_and_recording_errors_retained=True, actual_closed_loop_latency_gain_not_yet_proven=True),
        two_phase_signal_restoration=dict(
            schema=live.signals.SCHEMA, source="tools/e2e/two_phase_owned_signal_restoration_v1_20261004.py",
            all_original_43_actor_state_and_duration_readbacks_required=True,
            exact_original_state_proved_during_frozen_20Hz_tick_before_unfreeze=True,
            original_freeze_and_duration_proved_during_second_20Hz_tick=True,
            natural_unfrozen_phase_changes_censused_not_reset=True, elapsed_phase_rewind_forbidden=True,
            original_PID_actor_no_ego_no_sensor_and_settings_guards_unchanged=True,
            native_restoration_ticks_outside_capture_and_NN_denominator=2,
            independently_verified_complete_record_required=True, original_failed_cleanup_not_reinterpreted=True),
        official_completed_main_domains=0, official_completed_subfunctions=0,
        independent_validation=False, full_RTE01_approved=False, model_promoted=False)
    fixture=result["owned_green_signal_fixture"]
    fixture.update(post_vehicle_restoration_native_ticks=2,
        freeze_API_scope="world-global; one install call, frozen restore proof call, original-freeze restoration call",
        original_state_component_verified_before_unfreeze=True, unfrozen_phase_changes_after_state_proof_retained=True)
    result["prospective_visual_publication"]["gallery"] = "docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/13_town03_full_route209_v3"
    require(result["policy_deadline_seconds"] == .1 and result["model_speed_cap_mps"] == 2.5
        and result["controller_target_cap_mps"] == 2.4 and result["speed_preview_s"] == .5
        and result["native_actor_planar_speed_maximum_mps"] == 2.7
        and result["body_clearance_screen"]["lane_inset_m"] == .1
        and result["total_capture_seconds"] == 110 and result["original_catalog_goal_tolerance_m"] == 2.5
        and result["worker_command"][result["worker_command"].index("-m")+1] == MODULE,
        "fresh V3 altered inherited physical/deadline/goal or actual worker identity")
    return result


def runtime_sources():
    """HH_261004 - Include new helpers and tests in the complete unchanged V2 closure before declaration."""
    require(digest(Path(previous.__file__)) == PARENT_SHA256, "frozen full209 V2 owner changed")
    result=_sources();live.verify_frozen()
    seeds=[Path(__file__).resolve(),Path(live.__file__).resolve(),Path(live.jpeg.__file__).resolve(),
        Path(live.signals.__file__).resolve(),
        ROOT/"learning/tests/test_parallel_exact_jpeg_after_ack_v1_20261004.py",
        ROOT/"learning/tests/test_two_phase_owned_signal_restoration_v1_20261004.py",
        ROOT/"learning/tests/test_town03_full209_collector_v3_20261004.py",
        ROOT/"learning/tests/test_town03_full209_owner_v3_20261004.py"]
    require(all(path.resolve()==path and path.is_file() and not path.is_symlink() for path in seeds),
        "fresh V3 canonical source/test closure is not prepared")
    for path in base.imported_source_closure(seeds):
        name,sha=str(path.relative_to(ROOT)),digest(path)
        require(name not in result or result[name]==sha,"fresh V3 source closure conflict")
        result[name]=sha
    return result


def diagnostic_report(output,plan,owner):
    """HH_261004 - Add independent complete source/worker/cleanup proofs while retaining every authentic inherited FAIL."""
    result=_diagnostic(output,plan,owner)
    inherited=result.get("learned_driving_screen_passed") is True
    result.update(schema=SCHEMA+".diagnostic_report",fresh_owner_revision="v3",original_V1_preserved=True,
        original_V2_preserved=True,capture_raw_ABI_schema=live.SCHEMA,
        synchronous_parallel_after_ACK_recording_passed=False,two_phase_signal_restoration_passed=False,
        fresh_intervention_full_raw_and_source_closure_passed=False,
        inherited_full209_research_screen_passed_before_new_conjunction=inherited)
    try:
        original=previous.previous
        episode=output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
        raw=result.get("raw_source_sha256",{})
        require(bool(raw) and all(digest(path)==sha for path,sha in raw.items()),"original complete raw inputs changed before new audits")
        try:
            result["independent_two_phase_signal_restoration"]=live.signals.verify_record(
                original.read_json(episode/"owned_green_signal_fixture.json"))
            require(result["independent_two_phase_signal_restoration"].get("passed") is True,
                "complete actual two-phase signal record must independently pass")
            result["two_phase_signal_restoration_passed"]=True
        except Exception as error:
            result["two_phase_signal_restoration_diagnostic_error"]=type(error).__name__+": "+str(error)
        try:
            recordings,cameras,owned_sources,timings,states=[original.right.jsonl(episode/name) for name in (
                "recording_after_ack.jsonl","camera_frames.jsonl","owned_camera_source.jsonl","wall_timing.jsonl","states.jsonl")]
            callback_path=episode/"callback_timing_receipts.json"
            callback=original.read_callback_receipts(callback_path,raw.get(str(callback_path)))
            result["independent_synchronous_parallel_after_ACK_recording"]=live.audit_parallel_recordings(
                recordings,cameras,owned_sources,callback,timings,states,episode=episode,raw_pins=raw,digest=digest)
            require(result["independent_synchronous_parallel_after_ACK_recording"].get("passed") is True,
                "complete actual parallel recording/source chain must independently pass")
            result["synchronous_parallel_after_ACK_recording_passed"]=True
        except Exception as error:
            result["parallel_after_ACK_recording_diagnostic_error"]=type(error).__name__+": "+str(error)
        original.right.verify_closure(output,plan)
        require(all(digest(path)==sha for path,sha in raw.items()),"all actual raw bytes changed during new audits")
        result["fresh_intervention_full_raw_and_source_closure_passed"]=True
    except Exception as error:
        result["fresh_intervention_diagnostic_error"]=type(error).__name__+": "+str(error)
    passed=inherited and all(result.get(key) is True for key in (
        "synchronous_parallel_after_ACK_recording_passed","two_phase_signal_restoration_passed",
        "fresh_intervention_full_raw_and_source_closure_passed"))
    result.update(learned_driving_screen_passed=passed,full209_research_motion_body_goal_callback100ms_passed=passed,
        status="PASS_KNOWN_DEV_FULL209_RESEARCH" if passed else "FAIL_KNOWN_DEV_FULL209_RESEARCH",
        official_completed_main_domains=0,official_completed_subfunctions=0,independent_validation=False,
        full_RTE01_approved=False,model_promoted=False,learned_STOP_claim=False,
        actual_closed_loop_gain_claimed=False,vehicle_control_approved=False)
    return result


@contextmanager
def activated(pins):
    """HH_261004 - Scope only fresh delegates inside the entire frozen V2 ownership lifecycle and restore every alias."""
    global _active
    require(not _active and digest(Path(previous.__file__))==PARENT_SHA256,"fresh V3 owner cannot nest or use changed V2")
    changes=dict(paths=paths,capture_argv=capture_argv,fixed_contract=fixed_contract,
        runtime_sources=runtime_sources,diagnostic_report=diagnostic_report,live=live,MODULE=MODULE,SCHEMA=SCHEMA)
    saved={name:getattr(previous,name) for name in changes};_active=True
    try:
        for name,value in changes.items():setattr(previous,name,value)
        with previous.activated(pins):yield
    finally:
        for name,value in reversed(tuple(saved.items())):setattr(previous,name,value)
        _active=False


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("weights","metadata","checkpoint","declaration"):parser.add_argument("--expected-"+name+"-sha256",required=True)
    args,remaining=parser.parse_known_args(argv)
    pins={name+"_sha256":getattr(args,"expected_"+name+"_sha256") for name in ("weights","metadata","checkpoint","declaration")}
    with activated(pins):return base.main(remaining)


if __name__=="__main__":raise SystemExit(main())
