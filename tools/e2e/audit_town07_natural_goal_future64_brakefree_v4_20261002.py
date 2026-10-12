"""HH_261002 - Preserve frozen true64 diagnostics; add authenticated brake-free phases and speed-independent defined-geometry witnesses. Never TRAIN admission."""

from __future__ import annotations

import argparse
import ast
from collections import Counter
from contextlib import contextmanager
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace

from scripts.e2e import audit_town07_natural_goal_future64_v2_20261002 as support
from scripts.e2e import town07_natural_goal_stop_brakefree_v4_20261002 as task

previous, ROOT = support.previous, support.ROOT
SCHEMA = "carla.actual_natural_goal_future64_brakefree_diagnostic.20261002.v4"
TASK_SCHEMA, PROFILE = task.SCHEMA, task.PROFILE
require, canonical = previous.require, previous.canonical
FROZEN_AUDIT_PATH, FROZEN_AUDIT_SHA = support.FROZEN_AUDIT_PATH, support.FROZEN_AUDIT_SHA
V4_SOURCE_PINS = {
    "tools/e2e/town07_natural_goal_stop_brakefree_v4_20261002.py": "a6a61e203c522427d79270c8c24979820e25e616eb25c18844046282eb590a01",
    "tools/e2e/collect_town07_natural_goal_stop_brakefree_v4_20261002.py": "2290977d11ad806560c31a762ce6757fc06ce49b4e08fd4996f528ad36dd76bd",
    "tools/e2e/run_owned_town07_natural_goal_stop_brakefree_v4_20261002.py": "3b337e3934cd1fbfd72e12366262261934d969b4868f55e57cdbf65df5c3aecb",
}
SOURCE_PINS = dict(support.SOURCE_PINS, **task.FROZEN, **V4_SOURCE_PINS)
SOURCE_PINS["tools/e2e/audit_town07_natural_goal_future64_v2_20261002.py"]="ebd1f584f9cb41b8bbd912ee4026d26dd5ffe6c24c401b89f2ef5c4b62653487"
_active = False


def verify_frozen():
    pins = {}
    for name, expected in SOURCE_PINS.items():previous.pinned_bytes(ROOT/name,pins,expected)
    return pins


def definition(name):
    """HH_261002 - Inverse-AST prove only the prospective run-id, seed or explicit new governor marker differs."""
    tree = ast.parse((ROOT/FROZEN_AUDIT_PATH).read_bytes())
    original = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
    node = copy.deepcopy(original); changes=[]
    for value in ast.walk(node):
        if name=="inspect" and isinstance(value,ast.Call) and isinstance(value.func,ast.Attribute) and value.func.attr=="fixed_contract":
            require(ast.dump(value,include_attributes=False)==ast.dump(ast.parse("natural_task.fixed_contract('v1','pilot_00')",mode="eval").body,include_attributes=False),"unknown frozen inspector provider")
            changes.append((value.args[0],"value","v1"));value.args[0].value="v4"
        elif name=="audit_core" and isinstance(value,ast.Constant) and type(value.value) is int and value.value==2026102200:
            changes.append((value,"value",2026102200));value.value=task.SEED
        elif name=="governor_receipt_binding" and isinstance(value,ast.Compare) and ast.dump(value.left,include_attributes=False)==ast.dump(ast.parse("row.get('governor_math_changed')",mode="eval").body,include_attributes=False):
            require(len(value.comparators)==len(value.ops)==1 and isinstance(value.ops[0],ast.Is) and value.comparators[0].value is False,"unknown governor provenance predicate")
            changes.append((value.comparators[0],"value",False));value.comparators[0].value=True
    require(len(changes)==1,"one exact version-only mechanical binding required")
    revised = copy.deepcopy(node)
    for value,field,old in changes:setattr(value,field,old)
    require(ast.dump(node,include_attributes=False)==ast.dump(original,include_attributes=False),"any original source/future/denial/bound statement changed")
    namespace = dict(vars(previous))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[revised],type_ignores=[])),str(__file__)+":pinned_"+name,"exec"),namespace)
    return namespace[name]


def intent_witness(row):
    # HH_261002 - Mission phases are actual teacher labels, not oracle inputs or positive-brake-pedal labels.
    record = row["goal_stop"]; require(record.get("profile_id")==PROFILE,"exact v4 goal profile required")
    speed=math.hypot(previous.number(row["vx"],"vx"),previous.number(row["vy"],"vy"))
    phase=record.get("terminal_mission_phase");coast=record.get("coast_source_measurements");hold=record.get("low_speed_service_hold_source")
    current_ns=previous.stamp(row)
    def prior(source):
        return isinstance(source,dict) and type(source.get("frame")) is int and source["frame"]<=row["frame"] \
            and round(previous.number(source["timestamp"],"mission source timestamp")*1e9)<=current_ns
    target=previous.number(record["target_speed_mps"],"actual mission target")
    valid=record.get("pilot_failure_reason") is None
    if phase=="GO":valid=valid and coast is None and hold is None
    elif phase=="BRAKE_TO_STOP":valid=valid and prior(coast) and hold is None and target==0.
    elif phase=="HOLD":
        valid=valid and prior(coast) and prior(hold) and target==0. and speed<=previous.STATIONARY_SPEED \
            and record.get("goal_window") is True and record.get("upstream_of_endpoint") is True \
            and previous.number(record["terminal_overshoot_m"],"overshoot")<=0.
    else:valid=False
    return dict(intent=phase if phase in ("GO","BRAKE_TO_STOP","HOLD") else None,valid=bool(valid),
        teacher_phase=record.get("phase"),actual_terminal_mission_phase=phase,source_frame=row["frame"],
        source_timestamp_ns=current_ns,measured_current_speed_mps=speed,annotations_are_labels_not_model_inputs=True)


def governor_receipt_binding(states,updates):
    report=definition("governor_receipt_binding")(states,updates)
    fields=("terminal_mission_phase","coast_source_measurements","low_speed_service_hold_source","coast_elapsed_s",
        "pilot_failure_reason","source_frame","source_timestamp","training_admission_state")
    for state in states:
        candidates=[r["original_governor_update"] for r in updates if r["measured_state_frame"]==state["frame"]]
        require(any(all(k in state["goal_stop"] and r.get(k)==state["goal_stop"][k] for k in fields) for r in candidates),"actual native mission differs from same-frame update receipt")
        require(state["goal_stop"]["source_frame"]==state["frame"] and state["goal_stop"]["source_timestamp"]==state["timestamp"],"future/current mission source cannot be renamed")
    report.update(actual_new_mission_fields_bind_sameframe_receipts=True,original_governor_math_unchanged=False,
        disclosed_v4_coast_latch_and_pending_marker=True,full_causal_owner_mission_replay_is_prior_pinned_evidence=True)
    return report


def defined_geometry(rows,*,initial_speed_mps):
    """HH_261002 - Two genuine non-tiny XY chords define curvature regardless of low speed; keep all64 raw labels and old flags."""
    require(len(rows)==64,"all64 rows including missing true futures required")
    prior_speed=previous.number(initial_speed_mps,"actual anchor v0");require(prior_speed>=0.,"actual nonnegative anchor v0 required")
    prior_xy=(0.,0.);prior_heading=None;prior_chord_speed=0.;prior_yaw=0.;result=[];missing=False
    for index,row in enumerate(rows):
        require(row["index"]==index and type(row["valid"]) is bool,"unmodified ordered true64 membership required")
        metrics={name:[None,False] for name in ("xy_curvature","xy_lateral_acceleration","yaw_label_heading_envelope")}
        length=None
        if row["valid"]:
            require(not missing,"true future cannot resume after unavailable physical support")
            require(isinstance(row["xy"],list) and len(row["xy"])==2,"true XY pair required")
            xy=tuple(previous.number(v,"true XY") for v in row["xy"])
            yaw=previous.number(row["yaw_rad"],"true yaw");speed=previous.number(row["speed_mps"],"true endpoint speed")
            delta=(xy[0]-prior_xy[0],xy[1]-prior_xy[1]);length=math.hypot(*delta);chord_speed=length/.1
            heading=math.atan2(delta[1],delta[0]) if length>previous.HEADING_DISTANCE else None
            if heading is not None and prior_heading is not None:
                curvature=abs(previous.wrap(heading-prior_heading))/length
                lateral=curvature*max(prior_chord_speed,chord_speed)**2
                metrics["xy_curvature"]=[curvature,curvature>previous.LEGACY_CURVATURE+1e-6]
                metrics["xy_lateral_acceleration"]=[lateral,lateral>previous.LEGACY_LATERAL+1e-6]
            if heading is not None:
                yaw_delta=abs(previous.wrap(yaw-prior_yaw))
                bound=min(previous.LEGACY_CURVATURE,previous.LEGACY_LATERAL/max(prior_speed,speed,1e-6)**2)*speed*.1
                metrics["yaw_label_heading_envelope"]=[yaw_delta,yaw_delta>bound+1e-6]
            prior_xy,prior_heading,prior_chord_speed,prior_yaw,prior_speed=xy,heading,chord_speed,yaw,speed
        else:prior_heading=None;missing=True
        result.append(dict(index=index,actual_XY_chord_length_m=length,metrics=metrics,
            curvature_defined_by_two_actual_non_tiny_chords=metrics["xy_curvature"][0] is not None,
            positive_or_low_speed_never_used_as_curvature_mask=True,tiny_chord_distance_threshold_m=previous.HEADING_DISTANCE,
            null_is_unassessed_not_zero_label=True))
    return result


def audit_core(*args):
    summary,windows=definition("audit_core")(*args)
    counts={n:Counter() for n in ("xy_curvature","xy_lateral_acceleration","yaw_label_heading_envelope")}
    extra=0
    for window in windows:
        values=defined_geometry(window["true_future_rows"],initial_speed_mps=window["current_semantic_witness"]["measured_current_speed_mps"])
        for row,value in zip(window["true_future_rows"],values):
            row["strict_defined_geometry_witness"]=value
            for name,(metric,failed) in value["metrics"].items():counts[name]["unassessed" if metric is None else "violations" if failed else "assessed_nonviolations"]+=1
        if any(failed for value in values for _,failed in value["metrics"].values()):
            extra+=1;window["failure_reasons"].append("defined_geometry_exceeded_even_at_low_speed")
            window["necessary_defined_domain_checks_clear"]=False
    summary.update(failed_fixed_window_count=sum(not w["necessary_defined_domain_checks_clear"] for w in windows),
        strict_defined_geometry_failed_window_count=extra,strict_defined_geometry_metric_counts={k:dict(v) for k,v in counts.items()},
        fixed_window_failure_counts=dict(Counter(e for w in windows for e in w["failure_reasons"])),
        all_frozen_legacy_and_speed_conditioned_domain_metrics_retained=True,low_speed_alone_never_masks_defined_geometry=True,
        strict_curvature_requires_two_actual_chords_above1e_minus4_m=True,strict_geometry_null_is_unassessed_not_zero=True,
        actual_terminal_mission_labels_not_old_envelope_phase_used=True)
    return summary,windows


@contextmanager
def activated():
    global _active
    require(not _active and not support._active and not task._active,"new diagnostic cannot attach to active capture or another adapter")
    verify_frozen()
    values=dict(SCHEMA=SCHEMA,TASK_SCHEMA=TASK_SCHEMA,PROFILE=PROFILE,SOURCE_PINS=SOURCE_PINS,natural_task=task,
        audit_core=audit_core,intent_witness=intent_witness,governor_receipt_binding=governor_receipt_binding)
    saved={k:getattr(previous,k) for k in values};_active=True
    try:
        for k,v in values.items():setattr(previous,k,v)
        yield
    finally:
        for k,v in saved.items():setattr(previous,k,v)
        _active=False


def declared_closure(plan,episode,pins):
    # HH_261002 - Execute the pinned v2 full-closure reader unchanged under an explicit v4 source-table namespace.
    tree=ast.parse(Path(support.__file__).read_bytes());node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="declared_closure")
    namespace=dict(vars(support),task=SimpleNamespace(FROZEN_V1=task.FROZEN),V2_SOURCE_PINS=V4_SOURCE_PINS)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(__file__)+":unchanged_all_declared_closure","exec"),namespace)
    return namespace["declared_closure"](plan,episode,pins)


def inspect(episode,plan_path,plan_sha):
    first={};previous.pinned_bytes(Path(__file__),first)
    with activated():
        summary,windows,pins=definition("inspect")(episode,plan_path,plan_sha)
        for path,expected in first.items():previous.pinned_bytes(path,pins,expected)
        plan=previous.document(plan_path,pins,plan_sha);manifest=previous.document(canonical(episode)/"manifest.json",pins)
        closure=declared_closure(plan,canonical(episode),pins)
        metadata=manifest["capture_contract"]["teacher_controller"]
        require(manifest["control_source"]=="Disclosed_BasicAgent_manual1_1p50_latched_brakefree_coast_low_speed_hold_v4"
            and metadata["normal_brake_cap"]==0. and metadata["normal_throttle_cap"]==.20
            and metadata["causal_brakefree_terminal_mission"]==task.mission_contract(),"actual declared v4 zero-brake/service-HOLD protocol differs")
        for name in ("control_receipts.jsonl","actual_teacher_control.jsonl","actual_hazard_overrides.jsonl"):previous.pinned_bytes(canonical(episode)/name,pins)
        states=previous.journal(canonical(episode)/"states.jsonl",pins)
        full=task.full_native_quality(manifest,states)
        summary.update(full_bootstrap_warmup_driving_HOLD_tail_scalar_bounds=full,
            full_declared_live_archive_source_API_asset_closure=closure,teacher_task_schema=TASK_SCHEMA,
            actual_coast_hold_controller_config_matches=True,source_capture_failed=summary["source_capture_failed"]
                or full["status"]!="PASS_SCALAR_ONLY" or summary["original_owner_capture_passed"] is not True or summary["original_body_report_passed"] is not True,
            original_owner_full_mission_and_control_replay_is_prior_pinned_evidence=True,
            adapter_never_grants_training_or_vehicle_approval=True)
        previous.recheck(pins);return summary,windows,pins


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("episode","case-plan","case-plan-sha256"):parser.add_argument("--"+name,required=True)
    parser.add_argument("--output");args=parser.parse_args(argv)
    summary,windows,pins=inspect(args.episode,args.case_plan,args.case_plan_sha256)
    if args.output:
        output=canonical(args.output)
        require(output.is_relative_to(ROOT) and not output.exists() and not output.is_relative_to(canonical(args.episode).parent),"fresh derived output outside immutable pilot required")
        previous.recheck(pins);output.mkdir(parents=True,exist_ok=False)
        with (output/"summary.json").open("x",encoding="utf-8") as stream:json.dump(summary,stream,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)
        with (output/"all_fixed_anchor_true_future64.jsonl").open("x",encoding="utf-8") as stream:
            for row in windows:stream.write(json.dumps(row,sort_keys=True,allow_nan=False)+"\n")
        previous.recheck(pins)
    print(json.dumps({k:v for k,v in summary.items() if k!="source_artifact_sha256"},indent=2,allow_nan=False))
    return 0 if summary["failed_fixed_window_count"]==0 and summary["full_raw_event_count"]==0 and not summary["source_capture_failed"] else 2


if __name__=="__main__":raise SystemExit(main())
