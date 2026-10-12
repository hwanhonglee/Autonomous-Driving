"""HH_261004 - Seal and explain every authentic V2 terminal/ordinary/GC timing without regrading or rerunning the failed drive."""

from __future__ import annotations

import argparse
import ast
from collections import Counter, defaultdict
import copy
import hashlib
import inspect
import json
from pathlib import Path
import textwrap

from . import analyze_full209_actual_callback_critical_path_v1_20261004 as frozen


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
TEST = ROOT / "learning/tests/test_full209_v2_terminal_stage_critical_path_v1_20261004.py"
RUN = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v2"
EPISODE = RUN / "episode.partial"
PLAN = ROOT / "storage/experiments/plans/2026-10-04/full209_v2_terminal_stage_critical_path_readonly_v1/declaration.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-04/full209_v2_terminal_stage_critical_path_readonly_v1"
SCHEMA = "portable_e2e.full209_v2_terminal_stage_critical_path_readonly.20261004.v1"
TIMING_SCHEMA = "portable_e2e.actual_temporal_navigation_terminal_stage_timing.20261004.v3"
STAGES = ("entry", "source_binding", "provenancehash", "canonicalgeometry", "missionconstruction",
    "observationserialization", "witnessevaluation", "receiptasdict", "terminaldecoder", "outputattachment")
ORDINARY = ("entry", "ordinary_exact_delegate")
COUNTS = dict(native=1801, camera=901, ACK=1805, fresh=865)
FIXED_SOURCES = {
    "tools/e2e/analyze_full209_actual_callback_critical_path_v1_20261004.py": "08fb1bde42d495d4a4cdc8270c72c23002a89bb1de5401234ac1bcdfb3d81124",
    "learning/tests/test_full209_actual_callback_critical_path_v1_20261004.py": "7fc2cb06a63526232f4d50aafc7520de8c2021d14b8128ef5aa865a0e949382e",
    "learning/portable_e2e/temporal_navigation_terminal_policy_v3_timing_20261004.py": "1c81b1b39f5e47b0a707f6b527e0ee95680f577ab0051f54d04a4cdf694ea785",
    "tools/e2e/collect_temporal_route_joint_town03_straight_full209_v2_20261004.py": "d35ee2942e63d9a24a6a19340b59ea998a85c11f4d5a21d50a011366922b1dac",
    "tools/e2e/research_temporal_route_joint_town03_straight_full209_v2_20261004.py": "fc40b314ed3f7aa33d302699204a02ed7e6a4d85ad5ac04c0f64153a016150ca",
}
require, digest = frozen.require, frozen.digest


def sources():
    result={name:digest(ROOT/name) for name in FIXED_SOURCES}
    require(result==FIXED_SOURCES,"frozen diagnosis or actual timing source changed")
    result.update({str(path.relative_to(ROOT)):digest(path) for path in (HERE,TEST)})
    return result


def raw_seal():
    """HH_261004 - Hash the whole ended run before declaration; no root publication manifest exists yet and none is invented."""
    inventory=frozen.tree_members(RUN)
    hashes={name:digest(RUN/name) for name in inventory["files"]}
    require(frozen.tree_members(RUN)==inventory,"original V2 tree changed while sealing")
    archived={name[7:]:sha for name,sha in hashes.items() if name.startswith("source/")}
    require(archived and all(digest(ROOT/name)==sha for name,sha in archived.items()),"every archived actual source must remain frozen")
    for name in tuple(FIXED_SOURCES)[2:]:require(archived[name]==FIXED_SOURCES[name],"actual timing source was not archived")
    return dict(run_root=str(RUN),all_original_files_sha256=hashes,metadata_inventory_sha256=
        hashlib.sha256(json.dumps(inventory,sort_keys=True).encode()).hexdigest(),all_archived_source_sha256=archived,
        sources=sources(),source_pixels_analyzed=False,model_or_simulator_started=False)


def snapshot():
    return raw_seal()


def whole_critical_path():
    """HH_261004 - Reuse the frozen complete join math with four explicit actual-denominator substitutions and no global mutation."""
    original=ast.parse(textwrap.dedent(inspect.getsource(frozen.analyze))).body[0]
    changed=copy.deepcopy(original)
    replacement={1836:COUNTS["native"],918:COUNTS["camera"],1840:COUNTS["ACK"],883:COUNTS["fresh"]}
    counts=Counter()
    for node in ast.walk(changed):
        if isinstance(node,ast.Constant) and type(node.value) is int and node.value in replacement:
            counts[node.value]+=1;node.value=replacement[node.value]
    require(counts==Counter({name:1 for name in replacement}),"exact four readonly denominator seams required")
    restored=copy.deepcopy(changed);inverse={value:key for key,value in replacement.items()}
    for node in ast.walk(restored):
        if isinstance(node,ast.Constant) and type(node.value) is int and node.value in inverse:node.value=inverse[node.value]
    require(ast.dump(restored)==ast.dump(original),"readonly critical-path inverse changed another statement")
    namespace={**vars(frozen),"RUN":RUN,"EPISODE":EPISODE,"SCHEMA":SCHEMA+".whole_path","snapshot":snapshot}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),
        str(HERE)+":readonly_denominator_inverse_verified","exec"),namespace)
    result=namespace[original.name]()
    for previous,new in (("all883_fresh_critical_path_rows","all_fresh_critical_path_rows"),
        ("all1836_native_rows","all_native_rows"),("all918_quota_rows","all_quota_rows")):
        result[new]=result.pop(previous)
    result["actual_denominators"]=COUNTS.copy()
    return result


def union_overlap_ns(intervals,start,finish):
    """HH_261004 - Count GC interval union, not a double-counted nested-generation sum or exclusive CPU estimate."""
    clipped=sorted((max(a,start),min(b,finish)) for a,b in intervals if min(b,finish)>max(a,start))
    result=0;left=right=None
    for a,b in clipped:
        if right is None:left,right=a,b
        elif a<=right:right=max(right,b)
        else:result+=right-left;left,right=a,b
    return result+(0 if right is None else right-left)


def validate_timing(value):
    require(value["schema"]==TIMING_SCHEMA and value["frozen_bridge_source_sha256"]=="e4c0118de9d9e420890d1c0f11df4fe469cf9cdb7e969f1cd96d2448798c40ab",
        "actual original bridge timing identity required")
    require(value["original_bridge_AST_inverse_verified"] is True and value["original_policy_deadline_seconds"]==.1
        and all(value[name] is True for name in ("original_policy_deadline_unchanged","stage_work_inside_original_policy_deadline",
            "scalar_receipt_finalization_also_inside_policy_deadline","stage_sum_excludes_final_scalar_receipt_attachment_overhead",
            "original_inputs_math_checks_order_exceptions_unchanged","timing_only_research"))
        and all(value[name] is False for name in ("source_rehashed_during_call","numerical_outputs_changed","weights_changed","learned_STOP_claim","vehicle_control_approved")),
        "original math/deadline and limited timing claims required")
    stages=value["stages"];names=tuple(row["stage"] for row in stages);status=value["status"]
    require(status in ("COMPLETED","FAILED") and stages and len(names)==len(set(names)),"finite complete-or-failed stage receipt required")
    if status=="COMPLETED":require(names in (STAGES,ORDINARY) and value["exception_type"] is None,"complete ordinary or terminal stage sequence required")
    else:require((names==ORDINARY or names==STAGES[:len(names)]) and type(value["exception_type"]) is str and value["exception_type"],"failed original prefix and exception must be retained")
    start=value["call_started_monotonic_ns"];finish=value["call_finished_monotonic_ns"]
    require(type(start) is type(finish) is int and 0<start<=finish and value["total_observed_ns"]==finish-start,"whole actual bridge clock identity required")
    previous=start
    for row in stages:
        a,b=row["start_monotonic_ns"],row["finish_monotonic_ns"]
        require(type(a) is type(b) is int and a==previous<=b and row["elapsed_ns"]==b-a,"contiguous exact bridge stages required")
        previous=b
    require(previous==finish and sum(row["elapsed_ns"] for row in stages)==finish-start,"no missing bridge stage tail permitted")
    gc=value["gc_observation"]
    require(gc["same_thread_only"] is True and gc["maximum_events"]==32 and gc["bounded_event_overflow_disclosed"] is True
        and all(gc[name] is False for name in ("collection_behavior_changed","collection_forced","collection_disabled","thresholds_changed"))
        and len(gc["events"])<=32,"unchanged bounded same-thread GC observation required")
    for name in ("actual_start_count","actual_stop_count","overflow_start_count","unmatched_start_count"):
        require(type(gc[name]) is int and gc[name]>=0,"nonnegative explicit GC census required")
    closed=[]
    for event in gc["events"]:
        a,b=event["start_monotonic_ns"],event["finish_monotonic_ns"]
        require(type(a) is int and start<=a<=finish,"actual GC start must be inside this call")
        if b is not None:
            require(type(b) is int and a<=b<=finish,"actual GC stop must be inside this call");closed.append((a,b))
    full_gc=(gc["overflow_start_count"]==gc["unmatched_start_count"]==0 and gc["actual_start_count"]==gc["actual_stop_count"]==len(closed))
    return dict(branch="ordinary" if names==ORDINARY else "terminal" if len(names)>1 else "failed_unresolved",
        complete_gc_observation=full_gc,closed_GC_interval_union_ms=union_overlap_ns(closed,start,finish)/1e6,
        stage_GC_overlap_ms={row["stage"]:union_overlap_ns(closed,row["start_monotonic_ns"],row["finish_monotonic_ns"])/1e6 for row in stages})


def timing_rows(commands,paths,cameras):
    by_frame={row["frame"]:row for row in paths};by_stamp={int(round(row["timestamp"]*1e9)):row["frame"] for row in cameras}
    rows=[];seen={};aliases=[];uncalled=[]
    for ordinal,command in enumerate(commands):
        value=command.get("navigation_terminal_timing_research")
        if value is None:
            if command.get("navigation_terminal_timing_attachment_outside_original_policy_timer") is True:
                uncalled.append(dict(command_ordinal=ordinal,frame=command.get("inference_frame"),emergency_reason=command.get("emergency_reason")))
            continue
        require(type(value) is dict,"actual timing receipt cannot be a fabricated scalar")
        key=(value["frame_timestamp_ns"],value["call_started_monotonic_ns"],value["call_finished_monotonic_ns"])
        encoded=json.dumps(value,sort_keys=True,allow_nan=False)
        if key in seen:require(encoded==seen[key],"held timing copy changed its original receipt")
        else:seen[key]=encoded
        if command.get("held_between_camera_frames") is True:
            aliases.append(dict(command_ordinal=ordinal,source_stamp_ns=key[0],call_started_ns=key[1],call_finished_ns=key[2]));continue
        require(command.get("held_between_camera_frames") is False and command.get("navigation_terminal_timing_attachment_outside_original_policy_timer") is True,
            "fresh timing must come from actual policy, not teacher or held control")
        validated=validate_timing(value);frame=by_stamp[key[0]]
        require(frame==command["inference_frame"],"actual timing source/frame identity differs")
        path=by_frame.get(frame)
        row=dict(command_ordinal=ordinal,frame=frame,status=value["status"],exception_type=value["exception_type"],
            emergency_reason=command.get("emergency_reason"),original_timing_receipt=value,**validated)
        row["stage_ms"]={stage["stage"]:stage["elapsed_ns"]/1e6 for stage in value["stages"]}
        if path is not None:
            require(value["status"]=="COMPLETED" and path["raw_NN_finished_ns"]<=key[1]<=key[2]<=path["decoded_plan_ready_ns"],"actual NN/bridge/decoded chronology differs")
            nav=command.get("navigation_terminal_research") or {}
            expected="ordinary_exact_delegate" if validated["branch"]=="ordinary" else "authenticated_navigation_terminal_V2"
            require(nav.get("branch")==expected and nav.get("frame_timestamp_ns")==key[0],"timing branch differs from original actual navigation receipt")
            row.update(first_callback_plan_ms=path["first_callback_plan_ms"],first_callback_ACK_ms=path["first_callback_ACK_ms"],
                plan100ms_failed=path["plan100ms_failed"],ACK100ms_failed=path["ACK100ms_failed"],
                NN_to_bridge_started_ms=(key[1]-path["raw_NN_finished_ns"])/1e6,
                bridge_finished_to_plan_ready_ms=(path["decoded_plan_ready_ns"]-key[2])/1e6)
        else:require(value["status"]=="FAILED","completed actual bridge cannot lose its fresh source-to-plan row")
        rows.append(row)
    require(len({row["frame"] for row in rows})==len(rows),"one fresh bridge receipt per actual source required")
    require(set(by_frame)=={row["frame"] for row in rows if row["status"]=="COMPLETED"},"every actual accepted fresh plan must retain its timing")
    fresh_keys={(row["original_timing_receipt"]["frame_timestamp_ns"],row["original_timing_receipt"]["call_started_monotonic_ns"],
        row["original_timing_receipt"]["call_finished_monotonic_ns"]) for row in rows}
    require(all((row["source_stamp_ns"],row["call_started_ns"],row["call_finished_ns"]) in fresh_keys for row in aliases),
        "held timing must alias a retained actual fresh call")
    return rows,aliases,uncalled


def analyze(expected):
    require(digest(PLAN)==expected,"exact prospective readonly analysis declaration required")
    plan=frozen.json_file(PLAN,limit=4*1024**2);before=raw_seal()
    require(plan["schema"]==SCHEMA+".prospective" and plan["input_source_seal"]==before and plan["actual_denominators"]==COUNTS,"readonly declared input/source closure changed")
    whole=whole_critical_path();commands=frozen.json_lines(EPISODE/"learned_control.jsonl");cameras=frozen.json_lines(EPISODE/"camera_frames.jsonl")
    rows,aliases,uncalled=timing_rows(commands,whole["all_fresh_critical_path_rows"],cameras)
    require(len(rows)==COUNTS["fresh"] and not uncalled,"all actual865 bridge calls required for this sealed V2 run")
    groups={}
    for branch in sorted({row["branch"] for row in rows}):
        selected=[row for row in rows if row["branch"]==branch]
        stages={name:frozen.stats([row["stage_ms"][name] for row in selected if name in row["stage_ms"]]) for name in sorted({name for row in selected for name in row["stage_ms"]})}
        groups[branch]=dict(count=len(selected),statuses=dict(Counter(row["status"] for row in selected)),stage_ms=stages,
            GC_union_ms=frozen.stats([row["closed_GC_interval_union_ms"] for row in selected]),
            plan100ms_failed=sum(row.get("plan100ms_failed",False) for row in selected),ACK100ms_failed=sum(row.get("ACK100ms_failed",False) for row in selected))
    after=raw_seal();require(before==after,"original V2 bytes/sources changed during readonly diagnosis")
    return dict(schema=SCHEMA,provenance="HH_261004 - Retain complete failed actual interval and GC overflow/exception census; diagnose only.",
        declaration_sha256=expected,input_source_seal_before=before,input_source_seal_after=after,input_source_unchanged=True,
        original_FAIL_not_regraded=True,whole_actual_critical_path=whole,all_fresh_bridge_timing_rows=rows,
        held_timing_aliases=aliases,policy_steps_without_bridge_call=uncalled,branch_stage_metrics=groups,
        worst_callback_ACK_rows=sorted(rows,key=lambda row:row.get("first_callback_ACK_ms",-1),reverse=True)[:10],
        worst_observed_GC_rows=sorted(rows,key=lambda row:row["closed_GC_interval_union_ms"],reverse=True)[:10],
        GC_or_scheduling_exclusive_cause_proven=False,stage_time_is_inclusive_wall_time=True,
        scalar_final_receipt_attachment_excluded_from_stage_sum_but_inside_policy_and_callback_deadline=True,
        model_forward_or_CARLA_started=False,normalization_activated=False,formal_function_approved=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("declare","execute"),required=True);parser.add_argument("--expected-declaration-sha256")
    args=parser.parse_args(argv);require((args.mode=="execute")==bool(args.expected_declaration_sha256),"execution requires exact declaration SHA")
    require(not OUTPUT.exists(),"fresh create-only readonly output required")
    if args.mode=="declare":
        require(not PLAN.exists(),"new readonly declaration cannot overwrite")
        frozen.write_new_json(PLAN,dict(schema=SCHEMA+".prospective",input_source_seal=raw_seal(),actual_denominators=COUNTS,output=str(OUTPUT),runtime_activation=False))
        print(json.dumps(dict(status="DECLARED_NOT_EXECUTED",declaration_sha256=digest(PLAN)),sort_keys=True));return 0
    report=analyze(args.expected_declaration_sha256);frozen.write_new_json(OUTPUT/"report.json",report)
    summary=dict(schema=SCHEMA+".summary",original_FAIL_not_regraded=True,input_source_unchanged=True,
        fresh_bridge_count=len(report["all_fresh_bridge_timing_rows"]),branch_stage_metrics=report["branch_stage_metrics"],
        plan100ms_failed_frames=report["whole_actual_critical_path"]["plan100ms_failed_frames"],
        ACK100ms_failed_frames=report["whole_actual_critical_path"]["ACK100ms_failed_frames"],report_sha256=digest(OUTPUT/"report.json"))
    frozen.write_new_json(OUTPUT/"summary.json",summary);print(json.dumps(summary,indent=2,sort_keys=True));return 0


if __name__=="__main__":raise SystemExit(main())
