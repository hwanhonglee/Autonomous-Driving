#!/usr/bin/env python3
"""HH_260906 - Replay frozen v7 curvature and native reference motion without changing targets, gates, or admission."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.e2e import audit_carla_curvature_conditioning as conditioning

PLAN_SHA256 = "a7708081681efb7dddf9fc52e547a0950b500dcb2c36f5be9f1deaa2402d0a48"
SEAL_SHA256 = "e8ee637d098f96f801d19d1a33694713db0a95f0540fa4e206af7bd2465ca640"
SUMMARY_SHA256 = "5760bb9292161e962a2d289428d6000430a1082f6ed9b77862d90cbdbc96d91b"
FUTURE_SHA256 = "4267fef536a08539b147701069902e6f8ceafaa0bd00465da8d36f3dd1e8e6e0"
CONDITIONING_SHA256 = "165b1545ee476349bdb58cd4e8fc610a85242ff5d44a1c0edd8024a954ad1385"
CASES = (("01_immediate_A1", "immediate"), ("02_ramp_B1", "ramp"), ("03_ramp_B2", "ramp"), ("04_immediate_A2", "immediate"))
RAW_NAMES = ("manifest.json", "states.jsonl", "camera_frames.jsonl")
PUBLISHED_SOURCE_PATHS = {
    "portable_e2e/config/common_10hz_v1.contract.json", "portable_e2e/contract.py", "portable_e2e/model.py", "portable_e2e/runtime_contract.py",
    *{"scripts/e2e/"+name for name in ("audit_carla_acknowledged_control.py", "audit_carla_brake_free_goal_stop.py",
        "audit_carla_comfortable_v3_trial.py", "audit_carla_raw_pre_admission.py", "audit_portable_target_feasibility.py",
        "prepare_carla_common10_dataset.py", "summarize_carla_goal_stop_trials.py", "summarize_carla_hold_brake_ramp_abba.py",
        "summarize_carla_low_speed_steering_abba.py", "carla_goal_stop_profile.py")}}
require, number = conditioning.require, conditioning.number
safe, loads, json_lines, sha = conditioning.safe_path, conditioning.loads, conditioning.json_lines, conditioning.sha


def read(path, pin, ledger):
    path = safe(path)
    require(str(path) not in ledger or ledger[str(path)] == pin, "evidence fingerprint changed between reads")
    return conditioning.read(path, pin, ledger)


def vector_sub(first, second):
    return [a-b for a, b in zip(first, second)]


def base_velocity_world_ros(row):
    c, s = math.cos(number(row["yaw"])), math.sin(number(row["yaw"]))
    vx, vy = number(row["vx"]), number(row["vy"])
    return [c*vx-s*vy, s*vx+c*vy]


def planar_velocity_replay(row, wheelbase=2.85):
    """HH_260906 - This is the already-recorded yaw-only omega-cross-offset correction, not a proposed missing correction."""
    pose = row["actor_snapshot_transform_carla"]
    yaw = math.radians(number(pose["yaw"]))
    c, s = math.cos(yaw), math.sin(yaw)
    world = row["world_velocity_carla"]
    vx = c*number(world[0])+s*number(world[1])
    center_vy = -(-s*number(world[0])+c*number(world[1]))
    yaw_rate = -math.radians(number(row["world_angular_velocity_carla_deg_s"][2]))
    vy = center_vy-wheelbase/2.*yaw_rate
    require(row["vx"] == vx and row["vy"] == vy and row["yaw_rate"] == yaw_rate, "existing planar velocity/omega-cross-offset formula mismatch")
    actor_world = [number(world[0]), -number(world[1])]
    base_world = base_velocity_world_ros(row)
    return dict(existing_planar_formula_exact_match=True, raw_center_body_forward_left_mps=[vx, center_vy],
        recorded_base_body_forward_left_mps=[vx, vy], existing_planar_offset_correction_body_mps=[0., -wheelbase/2.*yaw_rate],
        actor_world_velocity_ros_xy_mps=actor_world, base_world_velocity_ros_xy_mps=base_world,
        base_minus_actor_world_velocity_ros_xy_mps=vector_sub(base_world, actor_world))


def pose_offset(yaw_carla, pitch_carla, wheelbase=2.85):
    h = wheelbase/2.
    return [-h*math.cos(pitch_carla)*math.cos(yaw_carla), h*math.cos(pitch_carla)*math.sin(yaw_carla)]


def integrate_interval(delta, left_velocity, right_velocity, dt, source_yaw):
    c, s = math.cos(source_yaw), math.sin(source_yaw)
    result = {}
    for name, velocity in (("left_endpoint", left_velocity), ("right_endpoint", right_velocity),
            ("trapezoid", [(a+b)*.5 for a, b in zip(left_velocity, right_velocity)])):
        integral = [v*dt for v in velocity]
        residual = vector_sub(delta, integral)
        result[name] = dict(velocity_integral_world_ros_xy_m=integral, signed_pose_minus_integral_world_ros_xy_m=residual,
            signed_pose_minus_integral_source_forward_left_m=[c*residual[0]+s*residual[1], -s*residual[0]+c*residual[1]],
            residual_norm_m=math.hypot(*residual))
    return result


def native_interval(first, second, earlier=None, wheelbase=2.85):
    dt = number(second["timestamp"])-number(first["timestamp"])
    require(second["frame"] == first["frame"]+1 and abs(dt-.05) <= 5e-7, "native interval cannot bridge a frame/time gap")
    native = conditioning.native_segment(first, second, earlier)
    left, right = planar_velocity_replay(first, wheelbase), planar_velocity_replay(second, wheelbase)
    actor_delta = vector_sub(conditioning.actor_xy(second), conditioning.actor_xy(first))
    base_delta = [second[key]-first[key] for key in ("x", "y")]
    native["measured_float_timestamp_duration_seconds"] = dt
    native["actor_signed_position_velocity"] = integrate_interval(actor_delta, left["actor_world_velocity_ros_xy_mps"],
        right["actor_world_velocity_ros_xy_mps"], dt, first["yaw"])
    native["virtual_base_signed_position_velocity"] = integrate_interval(base_delta, left["base_world_velocity_ros_xy_mps"],
        right["base_world_velocity_ros_xy_mps"], dt, first["yaw"])
    native["existing_planar_velocity_replay"] = {"source": left, "destination": right}
    poses = [row["actor_snapshot_transform_carla"] for row in (first, second)]
    yaw0, yaw1 = (math.radians(number(pose["yaw"])) for pose in poses)
    pitch0, pitch1 = (math.radians(number(pose["pitch"])) for pose in poses)
    origin, yaw_only, end = pose_offset(yaw0,pitch0,wheelbase), pose_offset(yaw1,pitch0,wheelbase), pose_offset(yaw1,pitch1,wheelbase)
    yaw_change, pitch_change = vector_sub(yaw_only,origin), vector_sub(end,yaw_only)
    observed = vector_sub(base_delta,actor_delta)
    closure = vector_sub(observed,[a+b for a,b in zip(yaw_change,pitch_change)])
    require(max(map(abs,closure)) <= 1e-12, "finite-difference pose offset decomposition mismatch")
    native["pose_offset_decomposition"] = dict(order="yaw first at source pitch, then pitch at destination yaw",
        observed_virtual_base_minus_actor_delta_ros_xy_m=observed,
        yaw_first_offset_change_ros_xy_m=yaw_change, pitch_second_offset_change_ros_xy_m=pitch_change,
        arithmetic_closure_residual_ros_xy_m=closure,
        existing_planar_velocity_correction_trapezoid_ros_xy_m=[dt*.5*(a+b) for a,b in zip(
            left["base_minus_actor_world_velocity_ros_xy_mps"],right["base_minus_actor_world_velocity_ros_xy_mps"])],
        angular_velocity_axis_assumption_added=False,
        interpretation="Finite difference of the recorded reference-offset formula; ordering is declared and is not causal attribution. Position-derived offset change need not equal endpoint angular-rate integration.")
    return native


def exact_window_points(states, anchor_frame, point_index):
    require(type(point_index) is int and 0 <= point_index < 64,"invalid future point index")
    stamps = [conditioning.stamp(row) for row in states]
    anchor = next(row for row in states if row["frame"] == anchor_frame)
    origin = dict(point_index=-1, timestamp_ns=conditioning.stamp(anchor), base_world_ros_xy_m=[anchor["x"],anchor["y"]],
        actor_world_ros_xy_m=conditioning.actor_xy(anchor), base_anchor_xy_m=[0.,0.], actor_anchor_xy_m=[0.,0.],
        binding=dict(mode="exact_native_state",frames=[anchor_frame],timestamps_ns=[conditioning.stamp(anchor)]))
    points, heading, reference_index, previous, unassessed = [origin], 0., None, [0.,0.], 0
    last_geometry = None
    for index in range(point_index+1):
        target = conditioning.stamp(anchor)+(index+1)*100_000_000
        future,binding = conditioning.interpolate(states,stamps,target)
        xy = conditioning.relative([future["x"],future["y"]],[anchor["x"],anchor["y"]],anchor["yaw"])
        before_reference = reference_index
        geometry,heading,reference_index = conditioning.geometry(xy,previous,heading,reference_index,index)
        point = dict(point_index=index,timestamp_ns=target,base_world_ros_xy_m=[future["x"],future["y"]],
            actor_world_ros_xy_m=future["actor_xy"],base_anchor_xy_m=xy,
            actor_anchor_xy_m=conditioning.relative(future["actor_xy"],conditioning.actor_xy(anchor),anchor["yaw"]),binding=binding)
        points.append(point)
        if index < point_index:
            unassessed += geometry["curvature_rad_per_m"] is None
        previous,last_geometry = xy,geometry
    heading_segment = None if before_reference is None else points[before_reference:before_reference+2]
    return dict(anchor_frame=anchor_frame,point_index=point_index,anchor_body_yaw_ros_rad=anchor["yaw"],
        anchor=origin,last_up_to_three_points=points[-3:],previous_assessed_heading_segment=heading_segment,
        unassessed_steps_before_target=unassessed,original_geometry=last_geometry,
        first_point_uses_body_heading_not_a_third_position=point_index == 0,
        interpretation="Three-position data are provided only when present. A first future point uses anchor body heading; no fictitious prior position is manufactured.")


def composite_100ms(first,middle,last,left_interval,right_interval):
    result=dict(start_frame=first["frame"],midpoint_frame=middle["frame"],end_frame=last["frame"],
        duration_seconds=last["timestamp"]-first["timestamp"],midpoint_retained=True)
    for key,get in (("actor_signed_position_velocity",conditioning.actor_xy),("virtual_base_signed_position_velocity",lambda row:[row["x"],row["y"]])):
        delta=vector_sub(get(last),get(first))
        estimates={}
        for name in ("left_endpoint","right_endpoint","trapezoid"):
            integral=[a+b for a,b in zip(left_interval[key][name]["velocity_integral_world_ros_xy_m"],right_interval[key][name]["velocity_integral_world_ros_xy_m"])]
            residual=vector_sub(delta,integral)
            estimates[name]=dict(composite_velocity_integral_world_ros_xy_m=integral,
                signed_pose_minus_integral_world_ros_xy_m=residual,residual_norm_m=math.hypot(*residual))
        result[key]=dict(world_pose_delta_ros_xy_m=delta,estimates=estimates)
    return result


def analyze_case(states,cameras,futures,published,wheelbase=2.85):
    replay,boundaries = conditioning.analyze(states,cameras,futures,published,wheelbase)
    intervals = [dict(native_source_index=i-1,native_destination_index=i,**native_interval(states[i-1],states[i],states[i-2] if i>=2 else None,wheelbase))
        for i in range(1,len(states))]
    for boundary in boundaries:
        endpoint = boundary["episode_relative_100ms_tick"]*2
        boundary["native_100ms_composite_signed_integration"]=composite_100ms(states[endpoint-2],states[endpoint-1],states[endpoint],intervals[endpoint-2],intervals[endpoint-1])
        boundary["surrounding_native_interval_indices"] = [i for i in range(max(1,endpoint-10),min(len(states),endpoint+11))]
        boundary["surrounding_native_intervals"] = [intervals[i-1] for i in boundary["surrounding_native_interval_indices"]]
        boundary["neighborhood_definition"] = "Intervals indexed by destination observation: central interval plus ten preceding and ten following where available; overlapping neighborhoods are not independent events."
        worst = max(boundary["window_occurrences"],key=lambda row:row["original_curvature_rad_per_m"])
        first = next((row for row in boundary["window_occurrences"] if row["point_index"]==0),None)
        boundary["worst_original_window_points"] = exact_window_points(states,worst["anchor_frame"],worst["point_index"])
        boundary["first_point_window_points"] = None if first is None else exact_window_points(states,first["anchor_frame"],0)
        require(boundary["worst_original_window_points"]["original_geometry"]["curvature_rad_per_m"] == worst["original_curvature_rad_per_m"], "worst window geometry mismatch")
    replay.update(native_interval_count=len(intervals),existing_planar_velocity_formula_exact_match=True,
        maximum_pose_offset_arithmetic_closure_residual_m=max(abs(v) for interval in intervals for v in interval["pose_offset_decomposition"]["arithmetic_closure_residual_ros_xy_m"]))
    return replay,boundaries,intervals


def parse_seal(payload):
    pins={}
    for line in payload.decode().splitlines():
        require(re.fullmatch(r"[0-9a-f]{64}  [^\r\n]+",line),"invalid seal line")
        pin,name=line.split("  ",1)
        require(name not in pins and not Path(name).is_absolute() and ".." not in Path(name).parts,"unsafe or duplicate seal path")
        pins[name]=pin
    require(set(pins)=={"summary.json","paired_comparisons.json","future_windows.jsonl","image_inventory.jsonl","control_policy_witnesses.jsonl"},"unexpected sealed v7 payload set")
    require(pins["summary.json"]==SUMMARY_SHA256 and pins["future_windows.jsonl"]==FUTURE_SHA256,"sealed v7 summary/future identity differs")
    return pins


def run(campaign_root,evidence_dir,plan_path,plan_sha256,seal_sha256,script_sha256,*,output_dir=None):
    campaign,evidence,plan_path=safe(campaign_root),safe(evidence_dir),safe(plan_path)
    ledger={}
    read(Path(__file__),script_sha256,ledger)
    read(ROOT/"scripts/e2e/audit_carla_curvature_conditioning.py",CONDITIONING_SHA256,ledger)
    require(plan_sha256==PLAN_SHA256 and seal_sha256==SEAL_SHA256,"only the frozen v7 plan and private analysis seal are supported")
    plan=loads(read(plan_path,plan_sha256,ledger))
    require(plan["schema"]=="carla.hold_brake_ramp_abba_plan.v1" and [(c["id"],c["arm"]) for c in plan["cases"]]==list(CASES) and
        plan["training_data_approved"] is False and plan["gate_changes"] is False,"v7 plan scope differs")
    output=None if output_dir is None else safe(output_dir)
    if output is not None:
        require(not output.exists() and output.parent.is_dir(),"fresh output with existing parent required")
        require(all(output!=root and root not in output.parents and output not in root.parents for root in (campaign,evidence)),"output must be disjoint from frozen evidence")
    pins=parse_seal(read(evidence/"SHA256SUMS",seal_sha256,ledger))
    published=loads(read(evidence/"summary.json",pins["summary.json"],ledger))
    require(published["schema"]=="carla.hold_brake_ramp_abba_summary.v1" and published["status"]=="DIAGNOSED_NOT_ADMITTED" and
        published["plan_sha256"]==plan_sha256 and published["training_data_approved"] is False and published["dataset_export"] is False and
        published["target_retiming"] is False and published["gate_changes"] is False and published["planned_case_count"]==4,"sealed v7 diagnostic scope differs")
    source_pins=published["source_sha256"]["files"]
    require(set(source_pins)==PUBLISHED_SOURCE_PATHS and all(source_pins[name]==pin for name,pin in conditioning.MATH_SOURCE_PINS.items()),"original source closure/math identity differs")
    for name,pin in source_pins.items():
        read(ROOT/name,pin,ledger)
    for category in ("source_sha256","input_sha256","audit_math_sha256","orchestration_sha256"):
        for name,pin in plan[category].items():
            require(not Path(name).is_absolute() and ".." not in Path(name).parts,"unsafe prospective source/input path")
            read(ROOT/name,pin,ledger)
    future_payload=read(evidence/"future_windows.jsonl",pins["future_windows.jsonl"],ledger)
    future_rows=json_lines(future_payload)
    require(len(future_rows)==published["future_anchor_record_count"]==published["audited_camera_anchor_count"],"complete anchor denominator differs")
    require([(c["case_id"],c["arm"]) for c in published["cases"]]==list(CASES),"sealed case order differs")
    require(all((r["case_id"],r["arm"]) in CASES for r in future_rows),"foreign future case/arm")
    summaries,boundaries,intervals=[],[],[]
    for case,case_summary in zip(plan["cases"],published["cases"]):
        raw_root=campaign/case["id"]
        require(safe(ROOT/case["command"][2])==raw_root and case_summary["status"]=="COMPLETE_DIAGNOSTIC_NOT_ADMITTED","case root/retained diagnostic status differs")
        payloads={name:read(raw_root/"episode"/name,case_summary["metadata_sha256"][str(raw_root/"episode"/name)],ledger) for name in RAW_NAMES}
        manifest=loads(payloads["manifest.json"])
        profile=manifest["capture_contract"]["goal_stop_profile"]
        require(profile["profile_id"]=="comfortable_v7_hold_brake_ramp" and profile["hold_brake_mode"]==case["arm"] and
            profile["low_speed_steering_mode"]=="zero" and manifest["result"]["training_data_approved"] is False and
            manifest["coordinate_contract"]["wheelbase_m"]==2.85,"raw profile/reference/denial differs")
        states,cameras=json_lines(payloads["states.jsonl"]),json_lines(payloads["camera_frames.jsonl"])
        windows=[row for row in future_rows if row["case_id"]==case["id"]]
        require(len(states)==case_summary["native_state_count"] and len(cameras)==case_summary["camera_anchor_count"],"raw count differs from sealed case")
        result,groups,steps=analyze_case(states,cameras,windows,case_summary)
        summaries.append(dict(case_id=case["id"],arm=case["arm"],**result))
        boundaries.extend(dict(case_id=case["id"],arm=case["arm"],**row) for row in groups)
        intervals.extend(dict(case_id=case["id"],arm=case["arm"],**row) for row in steps)
    summary=dict(schema="carla.native_boundary_diagnostic.v1",status="ORIGINAL_FAILURES_RETAINED_NOT_ADMITTED",
        generated_at_utc=datetime.now(timezone.utc).isoformat(),cases=summaries,case_count=4,
        native_state_count=sum(row["counts"]["native_state_count"] for row in summaries),native_interval_count=len(intervals),
        original_anchor_record_count=len(future_rows),unique_case_endpoint_failure_count=len(boundaries),
        original_overlapping_failure_count=sum(row["duplicate_window_failure_count"] for row in boundaries),
        scope=dict(training_data_approved=False,new_data_admission=False,targets_changed=False,thresholds_changed=False,failures_removed=False,
            images_opened=False,model_executed=False,simulator_contacted=False,other_metric_values_recomputed=False,prior_pixel_or_ACK_QA_rerun=False),
        original_future_windows=dict(path="future_windows.jsonl",sha256=sha(future_payload),bytes=len(future_payload),byte_identical_to_frozen_input=True),
        source_sha256=dict(script=script_sha256,conditioning=CONDITIONING_SHA256,original_report_sources=source_pins),
        input_sha256=ledger,plan_sha256=plan_sha256,original_seal_sha256=seal_sha256,
        interval_integration_contract="Signed left endpoint, right endpoint, and trapezoid separately on each measured 50 ms interval; composite 100 ms sums retain the midpoint. Residual means sampled pose delta minus sampled velocity integral, not a physical error verdict.",
        curvature_contract=dict(minimum_step_m=1e-4,comparison="strict >",maximum_rad_per_m=.2,rounding_tolerance=1e-6,
            initial_heading="anchor body-forward, held until an assessed displacement",original_flags_unchanged=True),
        limitations=["Curvature is exactly replayed; other original metric flags/counts are retained, not all recomputed.",
            "Raw position, endpoint velocity, and finite-interval averages are different observations. Their disagreement does not identify a simulator cause.",
            "The existing planar omega-cross-offset correction is already present; no speculative 3D angular-axis correction is applied.",
            "Yaw-then-pitch decomposition is a declared finite-difference coordinate calculation, not physical causal attribution.",
            "Float32 spacing is a representation diagnostic, not an uncertainty bound or justification for removing a failure.",
            "Camera timestamps/frames and prior QA hashes are bound; image pixels and control ACKs are not rerun.",
            "All original anchor bytes, including tail context, are preserved; overlapping windows and neighborhoods are not independent events."])
    for name,pin in tuple(ledger.items()):
        read(name,pin,{})
    if output is not None:
        output.mkdir()
        with (output/"summary.json").open("xb") as stream:
            stream.write((json.dumps(summary,indent=2,allow_nan=False)+"\n").encode())
        for name,rows in (("native_intervals.jsonl",intervals),("boundary_witnesses.jsonl",boundaries)):
            with (output/name).open("xb") as stream:
                for row in rows:stream.write((json.dumps(row,sort_keys=True,allow_nan=False)+"\n").encode())
        with (output/"future_windows.jsonl").open("xb") as stream:stream.write(future_payload)
        with (output/"SHA256SUMS").open("x") as stream:
            stream.write("".join(f"{sha(path.read_bytes())}  {path.name}\n" for path in sorted(output.iterdir()) if path.name!="SHA256SUMS"))
    return summary


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("campaign-root","evidence-dir","plan"):
        parser.add_argument("--"+name,required=True,type=Path)
    for name in ("plan-sha256","seal-sha256","script-sha256"):
        parser.add_argument("--"+name,required=True)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--output-dir",type=Path)
    group.add_argument("--verify-only",action="store_true")
    args=parser.parse_args(argv)
    try:
        result=run(args.campaign_root,args.evidence_dir,args.plan,args.plan_sha256,args.seal_sha256,args.script_sha256,output_dir=args.output_dir)
    except (ValueError,KeyError,TypeError,OSError) as error:
        parser.exit(1,f"native boundary audit failed: {error}\n")
    print(json.dumps({key:result[key] for key in ("status","native_state_count","native_interval_count","original_anchor_record_count","unique_case_endpoint_failure_count")},sort_keys=True))
    return 0


if __name__=="__main__":
    raise SystemExit(main())
