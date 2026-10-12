"""HH_261001 - Prospectively select actual first gear before teacher throttle; never alter failed v1/v2 data."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import hashlib
import math
from pathlib import Path
import sys

from scripts.e2e import run_owned_town07_signed_recovery_v2_20261001 as v2

previous, task, ROOT = v2.previous, v2.task, v2.ROOT
MODULE = "scripts.e2e.run_owned_town07_signed_recovery_v3_20261001"
TASK_SCHEMA = "carla.actual_signed_teacher_recovery.20261001.v3"
SCHEMA = TASK_SCHEMA+".owned_batch"
V2_SOURCE = ROOT/"tools/e2e/run_owned_town07_signed_recovery_v2_20261001.py"
V2_SOURCE_SHA = "435c36354f6cd878d962638ea5cca12519d57652c339f5493a7a9bedba7b19d7"
CASES = tuple(dict(case_id=f"train_{i:02d}", split="train", seed=2026100510+i,
    signed_lateral_offset_ros_m=lateral, signed_yaw_offset_ros_deg=yaw)
    for i, (lateral, yaw) in enumerate((lateral, yaw) for lateral in (-.20, 0., .20) for yaw in (-1.5, 0., 1.5))) + (
    dict(case_id="holdout_00", split="dev", seed=2026100610, signed_lateral_offset_ros_m=-.15, signed_yaw_offset_ros_deg=1.),
    dict(case_id="holdout_01", split="dev", seed=2026100611, signed_lateral_offset_ros_m=.15, signed_yaw_offset_ros_deg=-1.))
_runtime_sources, _annotate_manifest = v2.runtime_sources, v2.annotate_manifest
_active = False
PARENT_CHECKER_FILE = ROOT/"tools/e2e/collect_carla_vad_expert.py"
PARENT_CHECKER_SOURCE_SHA = "815ef4c5806d1914eadfdd12eba1a42c392ceee58521b6f786d3c83349372695"


def verify_frozen_dependencies():
    v2.verify_frozen_v1()
    previous.require(previous.digest(V2_SOURCE) == V2_SOURCE_SHA, "frozen failed v2 teacher revision changed")
    source = PARENT_CHECKER_FILE.read_text()
    functions = [row for row in ast.parse(source).body if isinstance(row,ast.FunctionDef)
        and row.name == "checked_agent_initialization_observation"]
    previous.require(len(functions) == 1 and hashlib.sha256(ast.get_source_segment(source,functions[0]).encode()).hexdigest()
        == PARENT_CHECKER_SOURCE_SHA, "frozen initialization checker function changed")


def checked_first_gear_initialization_observation(world, ego, expected_frame, record, *, controller=None):
    """HH_261001 - Keep every original initialization witness; require actual declared manual first gear under full brake."""
    base = previous.worker_module.base
    record["status"] = "PENDING"
    try:
        base.checked_agent_initialization_frame(expected_frame)
        snapshot = world.get_snapshot()
        frame = base.checked_agent_initialization_frame(snapshot.frame)
        if frame != expected_frame:
            raise base.CollectionError("BasicAgent initialization observation frame differs from the verified brake frame")
        actor = snapshot.find(ego.id)
        if actor is None or getattr(actor, "id", ego.id) != ego.id:
            raise base.CollectionError("BasicAgent initialization snapshot is missing the owned actor")
        transform = base._transform_dict(actor.get_transform())
        if any(not math.isfinite(value) for value in transform.values()):
            raise base.CollectionError("BasicAgent initialization snapshot pose is nonfinite")
        observed = base.AcknowledgedControlTransport.checked_control(ego.get_control())
        after_frame = base.checked_agent_initialization_frame(world.get_snapshot().frame)
        timestamp = float(snapshot.timestamp.elapsed_seconds)
        if not math.isfinite(timestamp):
            raise base.CollectionError("BasicAgent initialization snapshot timestamp is nonfinite")
        record.update(expected_frame=expected_frame, frame_before=frame, frame_after=after_frame,
            timestamp=timestamp, actor_id=ego.id, actor_snapshot_transform_carla=transform,
            current_control=observed, raw_state_reference="CARLA actor snapshot API reference point; physical COM identity is unverified.")
        if after_frame != frame:
            raise base.CollectionError("world advanced while observing BasicAgent initialization")
        if (observed["steer"] != 0.0 or observed["throttle"] != 0.0 or observed["brake"] != 1.0
                or any(observed[key] for key in ("hand_brake", "reverse"))
                or observed["manual_gear_shift"] is not True or type(observed["gear"]) is not int or observed["gear"] != 1):
            raise base.CollectionError("BasicAgent initialization requires the verified zero-steering full-service-brake state")
        if controller is not None:
            past = getattr(controller, "past_steering", None)
            if isinstance(past, bool) or not isinstance(past, (int, float)) or not math.isfinite(past):
                raise base.CollectionError("BasicAgent PID past_steering is missing or nonfinite")
            record["pid_past_steering"] = float(past)
            if past != observed["steer"]:
                raise base.CollectionError("BasicAgent PID history differs from frame-bound zero steering")
        record["status"] = "PASS"
        return record
    except Exception as error:
        record.update(status="FAIL", error_type=type(error).__name__)
        raise


def annotate_manifest(manifest):
    _annotate_manifest(manifest)
    if manifest is None: return
    manifest["control_source"] = "Disclosed_BasicAgent_signed_recovery_teacher_preselected_manual_first_gear_v3"
    manifest["capture_contract"]["teacher_controller"].update(
        normal_launch_controller_revision=TASK_SCHEMA+".preselected_first_gear_normal_control",
        gear=1, manual_gear_shift=True, bootstrap_and_stationary_blank_full_brake_first_gear=True,
        normal_only_gear_intervention_before_unchanged_BasicAgent_hazard_override=True,
        final_hazard_return_modified_after_override=False,
        initialization_gear_contract=dict(actual_manual_gear_shift=True, actual_gear=1,
            parent_function_source_sha256=PARENT_CHECKER_SOURCE_SHA,
            all_original_frame_actor_finite_PID_and_full_brake_guards_preserved=True,
            observed_control_or_snapshot_proxy_used=False),
        automatic_neutral_spool_up_cause_hypothesis_not_outcome_proof=True,
        inherited_v2_launch_calibration=True, actual_measured_states_unchanged=True)


class FirstGearRecoveryLaunchControl(v2.SmoothRecoveryLaunchControl):
    """HH_261001 - Use the frozen .11/.025 normal pedal law but actually keep first gear under normal control."""
    def apply(self, control, **kwargs):
        result = super().apply(control, **kwargs)
        # HH_261001 - This hook is inside LocalPlanner; no transport-time rewrite after BasicAgent hazard returns.
        result.gear, result.manual_gear_shift = 1, True
        self.last.update(schema=TASK_SCHEMA+".preselected_first_gear_normal_control",
            manual_gear_shift=True, gear=1, final_hazard_return_modified_after_override=False,
            normal_gear_intervention_before_BasicAgent_hazard_override=True,
            original_planner_gear_and_flags_preserved_in_raw_proposal=True,
            profile_normal_control=previous.worker_module.base.control_dict(result))
        return result


def fixed_contract(run_id, case_id):
    previous.require(run_id == "v3", "this new prospective first-gear cohort owns run-id v3 only")
    result = v2._fixed_contract(run_id, case_id)
    result["controller"] = dict(result["controller"], launch_throttle=.11,
        post_handoff_throttle_ramp_per_second=.025, post_handoff_cap=.20,
        normal_launch_controls_inherited_v2=False, inherited_v2_pedal_calibration=True,
        gear=1, manual_gear_shift=True, native_acceleration_qa_limit_mps2=3.,
        gear_preselected_under_full_brake_before_first_throttle=True,
        final_BasicAgent_hazard_throttle_brake_steer_untouched=True)
    result.update(recovery_cohort_revision="v3_preselected_actual_first_gear_full_factorial_not_previous_success_subset",
        teacher="BasicAgent/original hazards; disclosed manual first gear under bootstrap/full brake and normal-only .11 launch/.025 native-time cap slew; no measured-state or label clipping.",
        initial_body_preflight=dict(method="Pinned actual Prius box / archived Town07 route+XODR; two identical predicted static footprints and bounded sweep.",
            manifest_path=str(v2.CENTER_MANIFEST), manifest_sha256=v2.CENTER_MANIFEST_SHA,
            all_eleven_initial_boxes_raw_and_10cm_inset_required=True, simulated_or_observed_data=False,
            actual_full_capture_body_and_sweeps_still_required=True),
        previous_failed_grids_preserved=True, previous_successful_subset_not_used_as_v3_training=True,
        source_v2_wrapper_sha256=V2_SOURCE_SHA)
    result["initialization_gear_contract"] = dict(actual_manual_gear_shift=True, actual_gear=1,
        parent_function_source_sha256=PARENT_CHECKER_SOURCE_SHA,
        all_original_frame_actor_finite_PID_and_full_brake_guards_preserved=True,
        observed_control_or_snapshot_proxy_used=False, original_automatic_gear_predicate_unchanged=False)
    return result


def runtime_sources():
    verify_frozen_dependencies()
    pins = _runtime_sources()
    for path in previous.lifecycle.imported_source_closure((Path(__file__).resolve(),)):
        pins[str(path.relative_to(ROOT))] = previous.digest(path)
    previous.require(pins.get(str(V2_SOURCE.relative_to(ROOT))) == V2_SOURCE_SHA, "v2 private dependency changed")
    return dict(sorted(pins.items()))


def declare_batch(run_id):
    previous.require(run_id == "v3", "only fresh first-gear v3 may be declared")
    geometry = v2.initial_body_preflight()
    result = v2._declare_batch(run_id)
    output, _ = task.paths(run_id)
    previous.write_new(output/"predicted_initial_geometry.json", geometry)
    return dict(result, predicted_initial_geometry_sha256=previous.digest(output/"predicted_initial_geometry.json"))


@contextmanager
def activated():
    """HH_261001 - Reuse immutable v2 machinery through restored private overrides; no source or old artifact edits."""
    global _active
    previous.require(not _active, "first-gear recovery activation cannot nest")
    verify_frozen_dependencies()
    with v2.activated():
        task_changes = dict(CASES=CASES, SCHEMA=TASK_SCHEMA, fixed_contract=fixed_contract)
        owner_changes = dict(MODULE=MODULE, SCHEMA=SCHEMA, CASE_IDS=tuple(row["case_id"] for row in CASES),
            runtime_sources=runtime_sources, declare_batch=declare_batch)
        v2_changes = dict(CASES=CASES, SCHEMA=SCHEMA, annotate_manifest=annotate_manifest)
        old_task = {key:getattr(task,key) for key in task_changes}
        old_owner = {key:getattr(previous,key) for key in owner_changes}
        old_v2 = {key:getattr(v2,key) for key in v2_changes}
        base, control_module = previous.worker_module.base, previous.worker_module.v2
        old_blank, old_control = base._blank_brake_control, control_module.CombinedControl
        old_initialization = base.checked_agent_initialization_observation
        def full_brake(carla):
            control = old_blank(carla)
            previous.require(control.throttle == 0. and control.brake == 1. and control.steer == 0.
                and not control.hand_brake and not control.reverse, "original pure full-brake command required")
            control.gear, control.manual_gear_shift = 1, True
            return control
        _active = True
        try:
            for key,value in task_changes.items(): setattr(task,key,value)
            for key,value in owner_changes.items(): setattr(previous,key,value)
            for key,value in v2_changes.items(): setattr(v2,key,value)
            base._blank_brake_control, control_module.CombinedControl = full_brake, FirstGearRecoveryLaunchControl
            base.checked_agent_initialization_observation = checked_first_gear_initialization_observation
            yield
        finally:
            base._blank_brake_control, control_module.CombinedControl = old_blank, old_control
            base.checked_agent_initialization_observation = old_initialization
            for key,value in old_v2.items(): setattr(v2,key,value)
            for key,value in old_owner.items(): setattr(previous,key,value)
            for key,value in old_task.items(): setattr(task,key,value)
            _active = False


def main(argv=None):
    values = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--run-id", default="v3")
    ours, _ = parser.parse_known_args(values)
    previous.require(ours.run_id == "v3", "failed older cohorts/arbitrary paths cannot be reused")
    if "--run-id" not in values and not any(value.startswith("--run-id=") for value in values): values += ["--run-id", "v3"]
    with activated(): return previous.main(values)


if __name__ == "__main__": raise SystemExit(main())
