"""HH_261001 - New prospective moving-recovery DATA scope, retaining full actual startup and failed older cohorts."""

from __future__ import annotations

import argparse
import ast
import copy
from contextlib import contextmanager
import math
from pathlib import Path
import sys

from scripts.e2e import run_owned_town07_signed_recovery_v3_20261001 as v3

previous, task, ROOT = v3.previous, v3.task, v3.ROOT
MODULE = "scripts.e2e.run_owned_town07_signed_recovery_v4_20261001"
TASK_SCHEMA = "carla.actual_signed_teacher_recovery.20261001.v4"
SCHEMA = TASK_SCHEMA+".owned_batch"
V3_SOURCE = ROOT/"tools/e2e/run_owned_town07_signed_recovery_v3_20261001.py"
V3_SOURCE_SHA = "01f6157bd61e32fe525166b4698fa70f8b82156843ecc1bb93edb538fd706a9a"
FIRST_FRAME, LAST_FRAME = 80, 682
FIRST_TIME_NS, LAST_TIME_NS = 4_000_000_000, 34_100_000_000
NATIVE_COUNT, CAMERA_COUNT = 965, 483
CASES = tuple(dict(case_id=f"train_{i:02d}", split="train", seed=2026100710+i,
    signed_lateral_offset_ros_m=lateral, signed_yaw_offset_ros_deg=yaw)
    for i, (lateral, yaw) in enumerate((lateral, yaw) for lateral in (-.20, 0., .20) for yaw in (-1.5, 0., 1.5))) + (
    dict(case_id="holdout_00", split="dev", seed=2026100810, signed_lateral_offset_ros_m=-.15, signed_yaw_offset_ros_deg=1.),
    dict(case_id="holdout_01", split="dev", seed=2026100811, signed_lateral_offset_ros_m=.15, signed_yaw_offset_ros_deg=-1.))
_runtime_sources, _annotate_manifest = v3.runtime_sources, v3.annotate_manifest
_active = False


def verify_frozen_dependencies():
    v3.verify_frozen_dependencies()
    previous.require(previous.digest(V3_SOURCE) == V3_SOURCE_SHA, "frozen v3 first-gear source changed")


def rolling_task_ast():
    """HH_261001 - Copy the pinned original class with only seven explicit membership/history literals changed."""
    verify_frozen_dependencies()
    source = Path(task.__file__).read_text()
    nodes = [row for row in ast.parse(source).body if isinstance(row,ast.ClassDef) and row.name == "MovingTask"]
    previous.require(len(nodes) == 1, "exact frozen membership class required")
    node = copy.deepcopy(nodes[0]); node.name = "RollingRecoveryTaskV4"
    counts = dict(close_end=0, selection_first=0, selection_last=0, preceding_phase=0, preceding_flag=0,
        result_first=0, result_last=0)
    for method in node.body:
        if not isinstance(method,ast.FunctionDef): continue
        for item in ast.walk(method):
            if method.name == "observe" and isinstance(item,ast.Compare) and isinstance(item.left,ast.Name) \
                    and item.left.id == "relative_frame" and len(item.ops)==1 and isinstance(item.ops[0],ast.Gt) \
                    and len(item.comparators)==1 and isinstance(item.comparators[0],ast.Constant) and item.comparators[0].value==620:
                item.comparators[0].value=LAST_FRAME; counts["close_end"]+=1
            if method.name == "observe" and isinstance(item,ast.keyword) and item.arg == "camera_selection_eligible":
                for part in ast.walk(item.value):
                    if isinstance(part,ast.Constant) and type(part.value) is int and part.value in (18,620):
                        before=part.value; part.value=FIRST_FRAME if before==18 else LAST_FRAME
                        counts["selection_first" if before==18 else "selection_last"]+=1
            if method.name == "saved_camera" and isinstance(item,ast.Compare) and isinstance(item.left,ast.Subscript) \
                    and isinstance(item.left.value,ast.Name) and item.left.value.id == "guard" \
                    and isinstance(item.left.slice,ast.Constant) and item.left.slice.value == "phase":
                for part in item.comparators:
                    if isinstance(part,ast.Constant) and part.value == "stationary_warmup":
                        part.value="driving"; counts["preceding_phase"]+=1
            if method.name == "saved_camera" and isinstance(item,ast.keyword) and item.arg == "preceding_guard_may_be_stationary_warmup":
                previous.require(isinstance(item.value,ast.Constant) and item.value.value is True, "original history flag changed")
                item.value.value=False; counts["preceding_flag"]+=1
            if method.name == "result" and isinstance(item,ast.keyword) and item.arg == "selected_driving_relative_time_ns":
                for part in ast.walk(item.value):
                    if isinstance(part,ast.Constant) and type(part.value) is int and part.value in (900_000_000,31_000_000_000):
                        before=part.value; part.value=FIRST_TIME_NS if before==900_000_000 else LAST_TIME_NS
                        counts["result_first" if before==900_000_000 else "result_last"]+=1
    previous.require(all(value==1 for value in counts.values()), "only seven exact rolling membership substitutions allowed")
    return node, counts


# HH_261001 - The known, source-pinned class is compiled under this NEW source identity; no original bytecode/source mutation.
_task_node, MEMBERSHIP_AST_SUBSTITUTIONS = rolling_task_ast()
_task_namespace = dict(task.__dict__, SCHEMA=TASK_SCHEMA, __name__=__name__)
exec(compile(ast.fix_missing_locations(ast.Module(body=[_task_node],type_ignores=[])),str(Path(__file__).resolve()),"exec"),_task_namespace)
RollingRecoveryTaskV4 = _task_namespace["RollingRecoveryTaskV4"]


def adapted_diagnostic():
    """HH_261001 - Reuse every frozen capture/ACK/pixel/body/source/spawn audit, changing only exact new denominators."""
    verify_frozen_dependencies()
    source = Path(previous.__file__).read_text()
    node = copy.deepcopy(next(row for row in ast.parse(source).body
        if isinstance(row,ast.FunctionDef) and row.name == "diagnostic_report"))
    counts={903:0,452:0}
    for item in ast.walk(node):
        if isinstance(item,ast.Constant) and type(item.value) is int and item.value in counts:
            before=item.value; item.value=NATIVE_COUNT if before==903 else CAMERA_COUNT; counts[before]+=1
    previous.require(counts=={903:2,452:2}, "only two native/two camera denominator literals may change")
    namespace=dict(previous.__dict__,SCHEMA=SCHEMA)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(Path(__file__).resolve()),"exec"),namespace)
    return namespace["diagnostic_report"]


_diagnostic = adapted_diagnostic()


def diagnostic_report(output,plan,owner):
    result = _diagnostic(output,plan,owner)
    episode=output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
    faults=[]
    try:
        states=previous.jsonl(episode/"states.jsonl")
        for row in states:
            if row["capture_phase"] != "driving": continue
            scalar=math.hypot(row["ax"],row["ay"])
            if not math.isfinite(scalar) or scalar>3.:
                faults.append(dict(frame=row["frame"],timestamp=row["timestamp"],scalar_acceleration_mps2=scalar,
                    driving_relative_native_frames=row["moving_task"]["driving_relative_native_frames"],
                    original_current_control=row["current_control"],raw_native_retained=True))
    except Exception as error:
        result.update(status="FAIL_RETAINED",actual_teacher_capture_passed=False,
            full_raw_startup_fault_report_error=type(error).__name__+": "+str(error))
    result.update(dataset_training_scope="PROSPECTIVE_ROLLING_RECOVERY_ONLY_NOT_STARTUP_TRAIN",
        all_raw_startup_and_tail_states_retained=True,full_raw_driving_acceleration_over3_faults=faults,
        outside_protected_window_faults_not_erased_or_relabelled=True,
        protected_history_and_future_bounds_unrelaxed=True,per_case_adaptive_or_quality_crop=False,
        expected_native_states=NATIVE_COUNT,expected_camera_bundles=CAMERA_COUNT,expected_JPEG_count=CAMERA_COUNT*6,
        expected_full_raw_sweeps=NATIVE_COUNT-1,startup_training_or_startup_success_claimed=False)
    return result


def fixed_contract(run_id,case_id):
    previous.require(run_id=="v4", "moving-recovery DATA revision4 owns a fresh v4 cohort only")
    # HH_261001 - Reuse the unchanged v3 controller/initialization contract, then prospectively declare the NEW data interval.
    source = Path(V3_SOURCE).read_text()
    node=copy.deepcopy(next(row for row in ast.parse(source).body if isinstance(row,ast.FunctionDef) and row.name=="fixed_contract"))
    changed=0
    for item in ast.walk(node):
        if isinstance(item,ast.Compare) and isinstance(item.left,ast.Name) and item.left.id=="run_id":
            for part in item.comparators:
                if isinstance(part,ast.Constant) and part.value=="v3": part.value="v4"; changed+=1
    previous.require(changed==1,"exact inherited v3 fixed-contract version guard required")
    namespace=dict(v3.__dict__)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(Path(__file__).resolve()),"exec"),namespace)
    result=namespace["fixed_contract"](run_id,case_id)
    result.update(schema=TASK_SCHEMA,recovery_cohort_revision="DATA_V4_PROSPECTIVE_ROLLING_RECOVERY_NOT_STARTUP_TRAIN",
        selected_driving_relative_time_ns=[FIRST_TIME_NS,LAST_TIME_NS],
        selection_clock="Exact native frame displacement80..682; camera every2frames; measured timestamps NEVER rewritten.",
        selection_rule="ALL302 six-camera anchors in fixed inclusive4.0..34.1s, identical for every predeclared case; not quality-conditioned.",
        closure_rule="Close after34.1s; unchanged teacher through actual last anchor+6.4s, then130 real full-brake tail ticks.",
        first_history_driving_relative_time_ns=[3_100_000_000,FIRST_TIME_NS],
        preceding_history_native_guard_phase="driving",
        dataset_training_scope="PROSPECTIVE_ROLLING_RECOVERY_ONLY_NOT_STARTUP_TRAIN",
        all_raw_startup_and_tail_states_retained=True,full_raw_startup_faults_reported=True,
        outside_protected_window_faults_not_erased_or_relabelled=True,
        per_case_adaptive_or_quality_crop=False,startup_training_or_startup_success_claimed=False,
        expected_native_states=NATIVE_COUNT,expected_camera_bundles=CAMERA_COUNT,expected_JPEG_count=CAMERA_COUNT*6,
        expected_full_raw_sweeps=NATIVE_COUNT-1,previous_failed_cohorts_all_preserved=True,
        source_v3_wrapper_sha256=V3_SOURCE_SHA, membership_AST_substitutions=dict(MEMBERSHIP_AST_SUBSTITUTIONS))
    return result


def annotate_manifest(manifest):
    _annotate_manifest(manifest)
    if manifest is None: return
    manifest["control_source"]="Disclosed_BasicAgent_DATA_V4_moving_recovery_with_unchanged_manual_first_gear_V3_controller"
    manifest["capture_contract"]["moving_recovery_training_scope"]=dict(schema=TASK_SCHEMA,
        selected_driving_native_frames=[FIRST_FRAME,LAST_FRAME],selected_driving_relative_time_ns=[FIRST_TIME_NS,LAST_TIME_NS],
        all_raw_startup_retained=True,startup_training_claimed=False,per_case_adaptive_or_quality_crop=False,
        controller_and_initialization_revision_unchanged_from_manual_first_gear_V3=True,
        expected_native_states=NATIVE_COUNT,expected_camera_bundles=CAMERA_COUNT)


def runtime_sources():
    verify_frozen_dependencies()
    pins=_runtime_sources()
    for path in previous.lifecycle.imported_source_closure((Path(__file__).resolve(),)):
        pins[str(path.relative_to(ROOT))]=previous.digest(path)
    previous.require(pins.get(str(V3_SOURCE.relative_to(ROOT)))==V3_SOURCE_SHA,"frozen v3 source closure differs")
    return dict(sorted(pins.items()))


def declare_batch(run_id):
    previous.require(run_id=="v4","only the new prospective rolling cohort may be declared")
    geometry=v3.v2.initial_body_preflight()
    result=v3.v2._declare_batch(run_id)
    output,_=task.paths(run_id)
    previous.write_new(output/"predicted_initial_geometry.json",geometry)
    return dict(result,predicted_initial_geometry_sha256=previous.digest(output/"predicted_initial_geometry.json"))


@contextmanager
def activated():
    global _active
    previous.require(not _active,"moving-recovery activation cannot nest")
    verify_frozen_dependencies()
    with v3.activated():
        task_changes=dict(CASES=CASES,SCHEMA=TASK_SCHEMA,fixed_contract=fixed_contract,MovingTask=RollingRecoveryTaskV4)
        owner_changes=dict(MODULE=MODULE,SCHEMA=SCHEMA,CASE_IDS=tuple(row["case_id"] for row in CASES),
            runtime_sources=runtime_sources,declare_batch=declare_batch,diagnostic_report=diagnostic_report)
        v2_changes=dict(CASES=CASES,SCHEMA=SCHEMA,annotate_manifest=annotate_manifest)
        saved_task={k:getattr(task,k) for k in task_changes}; saved_owner={k:getattr(previous,k) for k in owner_changes}
        saved_v2={k:getattr(v3.v2,k) for k in v2_changes}
        _active=True
        try:
            for k,value in task_changes.items(): setattr(task,k,value)
            for k,value in owner_changes.items(): setattr(previous,k,value)
            for k,value in v2_changes.items(): setattr(v3.v2,k,value)
            yield
        finally:
            for k,value in saved_v2.items(): setattr(v3.v2,k,value)
            for k,value in saved_owner.items(): setattr(previous,k,value)
            for k,value in saved_task.items(): setattr(task,k,value)
            _active=False


def main(argv=None):
    values=list(sys.argv[1:] if argv is None else argv)
    parser=argparse.ArgumentParser(add_help=False,allow_abbrev=False);parser.add_argument("--run-id",default="v4")
    ours,_=parser.parse_known_args(values)
    previous.require(ours.run_id=="v4","old failed or arbitrary cohorts cannot be reused")
    if "--run-id" not in values and not any(value.startswith("--run-id=") for value in values): values += ["--run-id","v4"]
    with activated(): return previous.main(values)


if __name__=="__main__": raise SystemExit(main())
