"""HH_261002 - Probe necessary terminal boundaries on retained denied V6 data, never run a model or authorize TRAIN."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import shutil

from portable_e2e import terminal_witness_v1_20261002 as witness

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "carla.actual_terminal_boundary_feasibility_probe.20261002.v1"
AUTH_SCHEMA = SCHEMA + ".external_authorization"
QA_SCHEMA = "carla.actual_natural_goal_future64_brakefree_diagnostic.20261002.v6"
CASE_SHA = "aa52a2edb0a3afcd6bb4ac02fb29e0fdeef6f57ba351e9171dc0159b0e54b244"
ROUTE_SHA = "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9"
QA_SHA = "feade474c19d9cb90ddf35d742e4fdf1a5639ff5841ea9ded295f883d8729dde"
WINDOWS_SHA = "e0e2481275ac023c1e7cb56bb9f14b7ba144e3c5a7eb09398e3768fb002dd41f"
FROZEN = {
    "learning/portable_e2e/__init__.py": "c5c53d6d0650bd67d4aeb3a828d55fae978987b61bbe6842234739b07431e8ef",
    "learning/portable_e2e/terminal_witness_v1_20261002.py": "52704ec0214053f7942a2657a76bf18fed2cbbbf81d729c3d512d66484672ca1",
    "learning/portable_e2e/contract.py": "e2e3a2669fa94aa98cb7e50f20ffa7db433b99eead9a12c654392e4b293a5c09",
    "learning/portable_e2e/path_time_decoder_terminal_v1_20261002.py": "df9beb1d7f71c6757907f7b6e7792ba3d9b7e2679d75c165b952cf6f6895943a",
    "tools/workspace_python.py": "2221cdf71e4ec840566fb19c180329887e53035ef1f319b02221c547141a5b21",
    "tools/canonical_workspace.py": "5d5ee618dba8dfdd84cc6da2877fb2dce8b5ba0017e4a503eb3c55d28401d0ed",
}
RAW_NAMES = ("manifest.json", "route.json", "states.jsonl", "camera_frames.jsonl", "actual_camera_pixels.jsonl",
    "actual_goal_governor.jsonl", "actual_goal_target_assignments.jsonl", "actual_teacher_control.jsonl",
    "control_receipts.jsonl", "actual_hazard_overrides.jsonl")
INPUT_KEYS = {"qa_summary", "true64_windows", "case_plan", "mission", "owner_report", "body_summary", *RAW_NAMES}
CAMERAS = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")


def require(value, message):
    if not value: raise ValueError(message)


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), "finite nonboolean measured number required")
    return float(value)


def stamp(row):
    return round(number(row["timestamp"]) * 1_000_000_000)


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def canonical(value):
    path = Path(value)
    require(path.is_absolute() and ".." not in path.parts, "absolute nonescaping path required")
    require(not any(p.is_symlink() for p in (path, *path.parents)), "symlink ancestors forbidden")
    require(path == path.resolve(), "canonical path required")
    return path


def unique(pairs):
    value = {}
    for key, child in pairs:
        require(key not in value, "duplicate JSON key")
        value[key] = child
    return value


def decode(payload):
    return json.loads(payload, object_pairs_hook=unique,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError("nonfinite JSON constant")))


def digest_file(path):
    require(path.is_file(), "regular input file required")
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for part in iter(lambda: stream.read(4 * 1024**2), b""): result.update(part)
    return result.hexdigest()


def read(path, expected, pins, maximum=8 * 1024**2):
    path = canonical(path)
    require(path.stat().st_size <= maximum, "bounded input required")
    payload = path.read_bytes()
    require(sha(payload) == expected, "external input SHA mismatch")
    pins[str(path)] = expected
    return payload


def rows(path, expected, pins):
    path = canonical(path)
    require(path.stat().st_size <= 512 * 1024**2, "bounded complete journal required")
    result, digest = [], hashlib.sha256()
    with path.open("rb") as stream:
        for line in stream:
            require(0 < len(line) <= 2 * 1024**2 and line.endswith(b"\n"), "bounded complete JSONL row required")
            digest.update(line); result.append(decode(line))
    require(digest.hexdigest() == expected, "external complete journal SHA mismatch")
    pins[str(path)] = expected
    return result


def recheck(pins):
    for path, expected in pins.items():
        require(digest_file(canonical(path)) == expected, "source/input changed during probe")


def mission_proposal(case, route, *, case_sha, route_sha):
    """HH_261002 - Print an assertion for separate root sealing; read no observations or future labels."""
    require(case_sha == CASE_SHA and route_sha == ROUTE_SHA, "exact retained V6 case and original route required")
    require(case["schema"] == "carla.actual_natural_goal_stop_teacher.20261002.v6"
        and case["route_sha256"] == route_sha and case["route_length_m"] == 210.5975914062836
        and case["natural_goal_configuration"]["stop_buffer_m"] == .75, "full declared mission/offset differs")
    points = [[number(p["x"]), number(p["y"])] for p in route["route"]]
    require(route["town"] == case["town"] == "Town07" and route["coordinate_reference"] == "base_link"
        and route["route_length_m"] == case["route_length_m"], "route reference differs")
    require(abs(witness._cumulative(points)[-1] - case["route_length_m"]) <= 1e-6, "actual full route arc differs")
    return dict(schema=witness.MISSION_SCHEMA, coordinates=dict(witness.COORDINATES),
        mission_id="retained-v6-pilot-00-complete-mission", map_id="Town07", route_id="pinned-straight-210p597591m",
        source_route_artifact_sha256=route_sha, source_available_timestamp_ns=0,
        complete_mission_route=True, polyline_map_xy_m=points, goal_map_xy_m=points[-1], stop_reference_offset_m=.75)


def exact_zero_reference(states):
    """HH_261002 - Exact zero is an observed event, not the .05 entry threshold or a stationary pose assumption."""
    hold = [i for i, s in enumerate(states) if s["capture_phase"] == "driving"
        and s["goal_stop"]["terminal_mission_phase"] == "HOLD"]
    tail = [i for i, s in enumerate(states) if s["capture_phase"] == "stationary_tail"]
    require(hold and len(tail) == 130, "full HOLD and all130 native tail states required")
    speed = [math.hypot(number(s["vx"]), number(s["vy"])) for s in states]
    candidates = [i for i in hold if speed[i] == 0. and all(speed[j] == 0. for j in hold if j >= i)
        and all(speed[j] == 0. for j in tail)]
    if not candidates:
        return dict(status="NO_SUSTAINED_EXACT_ZERO_REFERENCE", first_HOLD_frame=states[hold[0]]["frame"],
            first_HOLD_speed_mps=speed[hold[0]], event_does_not_use_threshold_zero=True), None
    index = candidates[0]; origin = states[index]
    later = states[index:]
    displacement = [math.hypot(number(s["x"])-origin["x"], number(s["y"])-origin["y"]) for s in later]
    drift_arc = sum(math.hypot(number(b["x"])-number(a["x"]), number(b["y"])-number(a["y"])) for a,b in zip(later,later[1:]))
    return dict(status="OBSERVED_EXACT_ZERO_THROUGH_REMAINING_HOLD_AND_ALL130_TAIL", frame=origin["frame"],
        timestamp_ns=stamp(origin), position_map_xy_m=[origin["x"], origin["y"]],
        first_HOLD_frame=states[hold[0]]["frame"], first_HOLD_speed_mps=speed[hold[0]],
        all_remaining_HOLD_and_tail_exact_speed_zero=True, remaining_HOLD_count=sum(i >= index for i in hold),
        stationary_tail_count=130, maximum_later_pose_displacement_m=max(displacement),
        later_actual_pose_drift_arc_m=drift_arc, event_does_not_assert_pose_stationary=True), index


def causal_observation(mission_payload, mission_sha, route_sha, state, camera):
    """HH_261002 - No future/teacher-phase/stop-label argument exists in this causal reconstruction API."""
    mission = decode(mission_payload); points = mission["polyline_map_xy_m"]; arcs = witness._cumulative(points)
    yaw = number(state["yaw"]); forward = (math.cos(yaw), math.sin(yaw))
    position = [number(state[n]) for n in ("x", "y", "z")]; timestamp = stamp(state)
    require(camera["frame"] == state["frame"] and stamp(camera) == timestamp
        and camera["camera_order"] == list(CAMERAS), "actual same-frame six-camera/current-pose binding required")
    times = [round(number(camera["source_timestamps"][n])*1e9) for n in CAMERAS]
    require(times == [timestamp]*6, "six current camera clocks differ")
    projection = witness._project(points, arcs, position, forward); remaining = arcs[-1] - projection
    def local(point):
        dx,dy = point[0]-position[0], point[1]-position[1]
        return [forward[0]*dx+forward[1]*dy, -forward[1]*dx+forward[0]*dy]
    length = min(remaining, witness.PREFIX_LIMIT_M)
    requested = [projection+i for i in range(int(math.floor(length))+1)]
    if projection+length-requested[-1] > witness.GEOMETRY_TOLERANCE_M: requested.append(projection+length)
    prefix = [local(witness._sample(points, arcs, s)) for s in requested]
    document = dict(schema=witness.OBSERVATION_SCHEMA, coordinates=dict(witness.COORDINATES),
        **{k:mission[k] for k in ("mission_id", "map_id", "route_id")}, source_route_artifact_sha256=route_sha,
        sequence_index=0, anchor_timestamp_ns=timestamp, ego_timestamp_ns=timestamp, pose_timestamp_ns=timestamp,
        navigation_timestamp_ns=timestamp, camera_timestamp_ns=times,
        camera_source_sha256=[camera["original_BGRA_source_sha256"][n] for n in CAMERAS],
        ego_source_sha256=sha(encoded(state)), pose_source_sha256=sha(encoded({k:state[k] for k in ("frame","timestamp","x","y","z","yaw")})),
        position_map_m=position, orientation_xyzw=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)],
        provided_route_xy_base_m=prefix, provided_route_support_m=witness._cumulative(prefix)[-1],
        route_anchor_arc_m=projection, remaining_mission_arc_m=remaining, goal_base_m=local(points[-1]))
    payload = encoded(document)
    mode = witness.VERIFIED_TERMINAL if remaining <= witness.PREFIX_LIMIT_M else witness.ORDINARY
    decision = witness.evaluate_terminal_witness(mission_payload=mission_payload, expected_mission_sha256=mission_sha,
        expected_route_artifact_sha256=route_sha, observation_payload=payload, expected_observation_sha256=sha(payload),
        now_ns=timestamp, requested_mode=mode)
    require(decision.status == "VERIFIED", "causal reconstructed Witness rejected: "+decision.reason_code)
    return decision, document


def boundary_checks(v0, v1, distance, phase, *, support, terminal):
    """HH_261002 - Necessary scalar/prefix tests only; no claim that33knots fit64points or that a path is safe."""
    v0,v1,distance,support = map(number,(v0,v1,distance,support))
    require(v0 >= 0. and v1 >= 0. and distance >= 0. and support >= 0., "measured nonnegative bounds required")
    a0=(v1-v0)/.1; s1=.05*(v0+v1); minimum=v0*v0/4.; residual=s1+v1*v1/4.
    early=v0>0. and v1==0. and minimum<=distance<=.05*v0
    stationary=phase=="HOLD" and v0==0. and distance==0. and a0==0.
    checks=dict(measured_v0_within2p5=v0<=2.5, observed_v1_within2p5=v1<=2.5,
        first100ms_mean_a0_within_minus2_plus1p5=-2.<=a0<=1.5, D_ge_v0_squared_over4=distance>=minimum,
        # HH_261002 - Zero GO prefix may depart later; positive preferred-speed reachability is not tested here.
        exact_first100_prefix_and_remaining_braking=stationary or early
            or (not (v0>0. and v1==0.) and distance>s1 and distance>=residual),
        actual_terminal_support_contains_D=distance<=support+1e-6,
        HOLD_actual_entry_and_nonaccelerating=phase!="HOLD" or (v0<=.05 and a0<=0.),
        zero_HOLD_has_zero_D_a0=not(phase=="HOLD" and v0==0.) or stationary,
        BRAKE_positive_current_speed_and_D=phase!="BRAKE_TO_STOP" or (v0>0. and distance>0.),
        zero_D_only_exact_stationary_HOLD=distance>0. or stationary)
    return dict(v0_mps=v0, v1_true_100ms_mps=v1, a0_mean_100ms_mps2=a0, observed_stop_distance_label_m=distance,
        braking_minimum_distance_m=minimum, first100ms_trapezoid_arc_m=s1, prefix_plus_braking_distance_m=residual,
        early_sub100ms_stop_case=early, exact_stationary_HOLD_case=stationary, causal_support_m=support,
        causal_terminal_dispatch_available=terminal, checks=checks, all_necessary_boundaries_pass=all(checks.values()) and terminal,
        necessary_not_sufficient_for_decoder_representability=True, zero_preferred_positive_arc_unreachable_not_tested=True)


def true_heading_diagnostics(state, true, by_frame):
    """HH_261002 - Independently bind true labels to their recorded brackets, retaining signed XY and distinct heading definitions."""
    output=[];prior=(0.,0.);angle=number(state["yaw"]);c,s=math.cos(angle),math.sin(angle)
    for row in true:
        if not row["valid"]:
            output.append(dict(index=row["index"],valid=False));continue
        left,right=(by_frame[f][1] for f in row["source_native_bracket_frames"])
        times=[stamp(left),stamp(right)]
        require(times==row["source_native_bracket_timestamps_ns"] and times[0]<=row["target_timestamp_ns"]<=times[1],"true bracket times differ")
        ratio=0. if times[0]==times[1] else (row["target_timestamp_ns"]-times[0])/(times[1]-times[0])
        fields={k:number(left[k])+ratio*(number(right[k])-number(left[k])) for k in ("x","y","vx","vy")}
        wrap=lambda value:math.atan2(math.sin(value),math.cos(value))
        yaw=wrap(number(left["yaw"])+ratio*wrap(number(right["yaw"])-number(left["yaw"])))
        dx,dy=fields["x"]-state["x"],fields["y"]-state["y"]
        xy=[c*dx+s*dy,-s*dx+c*dy];speed=math.hypot(fields["vx"],fields["vy"])
        require(all(abs(a-b)<=1e-12 for a,b in zip(xy,row["xy"])) and abs(speed-row["speed_mps"])<=1e-12
            and abs(wrap(yaw-angle)-row["yaw_rad"])<=1e-12,"full true64 XY/yaw/endpointnorm differ from actual native bracket interpolation")
        chord=[xy[0]-prior[0],xy[1]-prior[1]];length=math.hypot(*chord)
        velocity=math.atan2(fields["vy"],fields["vx"]) if speed>0. else None
        output.append(dict(index=row["index"],valid=True,actual_native_bracket_span_ns=times[1]-times[0],
            actual_interpolation_ratio=ratio,measured_body_yaw_relative_to_anchor_rad=row["yaw_rad"],
            measured_base_velocity_heading_relative_to_body_rad=velocity,
            measured_velocity_heading_relative_to_anchor_rad=wrap(yaw+velocity-angle) if velocity is not None else None,
            actual_XY_chord_vector_m=chord,actual_XY_chord_length_m=length,
            XY_chord_heading_relative_to_anchor_rad=math.atan2(chord[1],chord[0]) if length>1e-4 else None,
            undefined_heading_is_NULL_not_zero=True,pose_or_heading_noise_not_proven=True))
        prior=xy
    return output


def native_projection_census(mission_payload,states):
    # HH_261002 - Include every20Hz native pose, so a between-camera backward settling step is never hidden.
    mission=decode(mission_payload);points=mission["polyline_map_xy_m"];arcs=witness._cumulative(points)
    previous=None;regressions=[]
    for row in states:
        yaw=number(row["yaw"])
        projected=witness._project(points,arcs,[number(row["x"]),number(row["y"]),number(row["z"])],(math.cos(yaw),math.sin(yaw)))
        if previous is not None and projected<previous:
            regressions.append(dict(frame=row["frame"],capture_phase=row["capture_phase"],
                signed_projection_delta_m=projected-previous,exceeds_frozen_1e_minus6_progress_tolerance=projected+1e-6<previous))
        previous=projected
    return dict(recorded_native_count=len(states),current_XY_not_recorded_monotone_route_progress_used=True,
        regression_count=len(regressions),maximum_regression_m=max((-r["signed_projection_delta_m"] for r in regressions),default=0.),
        regressions=regressions,bootstrap_separate_manifest_row_not_relabelled_as_recorded_native=True)


def analyze(mission_payload, mission_sha, route_sha, states, cameras, windows):
    """HH_261002 - Retain every fixed anchor and failure; future labels never enter the Witness/support call."""
    require(states and windows, "all actual source rows required")
    by_frame={s["frame"]:(i,s) for i,s in enumerate(states)}; camera_by_frame={c["frame"]:c for c in cameras}
    require(len(by_frame)==len(states) and len(camera_by_frame)==len(cameras), "unique actual frames required")
    require(all(b["source_frame"]>a["source_frame"] for a,b in zip(windows,windows[1:])),"fixed anchor order/uniqueness required")
    require(all(b["frame"]==a["frame"]+1 and 0<stamp(b)-stamp(a)<=50_000_001 for a,b in zip(states,states[1:])), "native20Hz census/gap differs")
    arcs=[0.]
    for a,b in zip(states,states[1:]): arcs.append(arcs[-1]+math.hypot(number(b["x"])-number(a["x"]),number(b["y"])-number(a["y"])))
    reference, reference_index=exact_zero_reference(states)
    output=[]; counts=Counter(); phases=Counter(); failures=Counter(); original_failures=Counter();phase_failures={}
    diagnostic_violations=Counter()
    discrepancies={};stream_previous=None;stream_rejected=None;previous_projection=None;regressions=[]
    for w in windows:
        i,state=by_frame[w["source_frame"]]; require(stamp(state)==w["source_timestamp_ns"], "exact anchor source timestamp differs")
        phase=w["current_semantic_witness"]["intent"]; require(phase==state["goal_stop"]["terminal_mission_phase"], "actual current semantic phase differs")
        phases[phase]+=1; original_failures.update(w["failure_reasons"])
        decision,observation=causal_observation(mission_payload,mission_sha,route_sha,state,camera_by_frame[state["frame"]])
        v=decision.witness
        if previous_projection is not None and v.route_projection_arc_m<previous_projection:
            regressions.append(dict(frame=state["frame"],signed_projection_delta_m=v.route_projection_arc_m-previous_projection,
                exceeds_frozen_1e_minus6_progress_tolerance=v.route_projection_arc_m+1e-6<previous_projection))
        previous_projection=v.route_projection_arc_m
        if stream_rejected is None:
            streamed=dict(observation,sequence_index=len(output));stream_payload=encoded(streamed)
            stream_decision=witness.evaluate_terminal_witness(mission_payload=mission_payload,expected_mission_sha256=mission_sha,
                expected_route_artifact_sha256=route_sha,observation_payload=stream_payload,expected_observation_sha256=sha(stream_payload),
                now_ns=stamp(state),requested_mode=v.mode,previous=stream_previous)
            stream_result=dict(status=stream_decision.status,reason_code=stream_decision.reason_code)
            counts["stream_witness_assessed_anchors"]+=1
            if stream_decision.status=="VERIFIED":stream_previous=stream_decision.witness
            else:
                stream_rejected=dict(frame=state["frame"],source_timestamp_ns=stamp(state),**stream_result)
                counts["stream_witness_rejected_anchors"]+=1
        else:
            stream_result=dict(status="UNASSESSED_AFTER_FIRST_STREAM_REJECTION",reason_code=stream_rejected["reason_code"])
            counts["stream_witness_unassessed_after_rejection"]+=1
        v0=math.hypot(number(state["vx"]),number(state["vy"])); require(v0==w["current_semantic_witness"]["measured_current_speed_mps"], "true measured v0 differs")
        true=w["true_future_rows"]
        require(len(true)==64 and all(r["index"]==j and type(r["valid"]) is bool and r["target_timestamp_ns"]==stamp(state)+(j+1)*100_000_000 for j,r in enumerate(true)), "all fixed64 labels required")
        counts["original_failed_anchors"]+=not w["necessary_defined_domain_checks_clear"]
        counts["true_future_rows"]+=len(true);counts["invalid_true_future_rows"]+=sum(not r["valid"] for r in true)
        heading=true_heading_diagnostics(state,true,by_frame)
        support=min(v.provided_route_arc_m,v.remaining_mission_arc_m,64.)
        counts["GO_old18m_support_incompatible"]+=phase=="GO" and support<18.
        row=dict(source_frame=state["frame"],source_timestamp_ns=stamp(state),current_phase=phase,
            source_true64_window_sha256=sha(encoded(w)), source_failure_reasons=list(w["failure_reasons"]),
            source_necessary_defined_domain_checks_clear=w["necessary_defined_domain_checks_clear"],
            source_true_future_rows=true, true_heading_diagnostics=heading, causal_navigation_reconstruction_not_saved_NN_input=True,
            independent_anchor_sequence0_is_not_stream_validation=True,stream_Witness=stream_result,
            causal_observation=observation, causal_Witness=dict(mode=v.mode,reason_code=v.reason_code,
                original_GO18m_contract_unchanged=v.original_GO18m_contract_unchanged,provided_arc_m=v.provided_route_arc_m,
                remaining_mission_arc_m=v.remaining_mission_arc_m,signed_stop_reference_arc_m=v.signed_remaining_stop_reference_arc_m,
                support_m=support,terminal_permission_is_prepared_not_live=v.terminal_permission))
        if reference_index is None or not true[0]["valid"]:
            row.update(boundary=None,unassessed_reason="no_sustained_exact_zero_reference" if reference_index is None else "first_true_future_unavailable")
            counts["boundary_unassessed"]+=1
        else:
            signed=arcs[reference_index]-arcs[i]
            distance=signed if i<=reference_index else 0.
            boundary=boundary_checks(v0,true[0]["speed_mps"],distance,phase,support=support,terminal=v.terminal_permission)
            chord=math.hypot(*true[0]["xy"])
            require(abs(boundary["a0_mean_100ms_mps2"]-true[0]["initial_measured_planar_acceleration_mps2"])<=1e-12,"source first100ms acceleration differs")
            body=number(state["yaw"]); velocity_body=math.atan2(number(state["vy"]),number(state["vx"])) if v0>0. else None
            boundary.update(signed_actual_native_arc_to_exact_zero_reference_m=signed,
                reference_event_already_occurred=i>reference_index,D_zero_after_observed_reference_is_event_not_pose_clipping=True,
                first100ms_true_XY_chord_m=chord,true_chord_minus_trapezoid_m=chord-boundary["first100ms_trapezoid_arc_m"],
                actual_source_bracket_frames=true[0]["source_native_bracket_frames"],actual_source_bracket_stamps_ns=true[0]["source_native_bracket_timestamps_ns"],
                anchor_measured_body_yaw_rad=body,anchor_measured_velocity_heading_relative_to_body_rad=velocity_body,
                zero_velocity_heading_is_NULL_not_zero=velocity_body is None)
            left=by_frame[true[0]["source_native_bracket_frames"][0]][0]
            prefix_arc=arcs[left]-arcs[i]+math.hypot(true[0]["xy"][0]-sum_part(state,states[left])[0],true[0]["xy"][1]-sum_part(state,states[left])[1])
            boundary.update(first100ms_actual_native_piecewise_XY_arc_m=prefix_arc,
                actual_native_prefix_arc_minus_trapezoid_m=prefix_arc-boundary["first100ms_trapezoid_arc_m"])
            in_horizon=reference["timestamp_ns"]<=stamp(state)+6_400_000_000
            applicable=v.terminal_permission and distance<=support+1e-6
            assessment=("ORDINARY_BRANCH_TERMINAL_LABEL_TESTS_INAPPLICABLE" if not v.terminal_permission
                else "TRUE_STOP_LABEL_OUTSIDE_ACTIVE_SUPPORT_NOT_PHYSICAL_INFEASIBILITY" if not applicable
                else "TERMINAL_BOUNDARY_ASSESSED_EVENT_IN_6p4" if in_horizon else "TERMINAL_BOUNDARY_ASSESSED_EVENT_AFTER_6p4")
            boundary.update(terminal_label_assessment=assessment,terminal_label_checks_applicable=applicable,
                actual_reference_event_within_6p4=in_horizon,ordinary_branch_not_rejected_for_future_stop_D=True)
            row["boundary"]=boundary
            diagnostic_violations.update(k for k,passed in boundary["checks"].items() if not passed)
            if applicable:
                failures.update(k for k,passed in boundary["checks"].items() if not passed)
                phase_failures.setdefault(phase,Counter()).update(k for k,passed in boundary["checks"].items() if not passed)
            discrepancies.setdefault(phase,[]).append(boundary["true_chord_minus_trapezoid_m"])
            counts["boundary_assessed"]+=1;counts["all_necessary_boundaries_and_dispatch_pass"]+=boundary["all_necessary_boundaries_pass"]
            counts["causal_terminal_dispatch_unavailable"]+=not v.terminal_permission
            counts[assessment]+=1;counts["reference_event_after_6p4"]+=not in_horizon
        output.append(row)
    summary=dict(schema=SCHEMA,status="NECESSARY_BOUNDARY_DIAGNOSTIC_NOT_DECODER_FIT_OR_TRAIN",selected_anchor_count=len(output),
        phase_counts=dict(phases),counts=dict(counts),boundary_failure_counts=dict(failures),source_failure_counts=dict(original_failures),
        boundary_failure_counts_by_phase={k:dict(v) for k,v in phase_failures.items()},
        all_label_diagnostic_inequality_violations_not_ordinary_failures=dict(diagnostic_violations),
        first100ms_chord_minus_trapezoid_by_phase={k:dict(count=len(v),minimum_m=min(v),maximum_m=max(v),
            mean_m=sum(v)/len(v),maximum_absolute_m=max(map(abs,v))) for k,v in discrepancies.items()},
        exact_zero_reference=reference,original_true64_and_NULL_geometry_preserved=True,observed_future_labels_not_Witness_inputs=True,
        independent_anchor_floor_does_not_assert_stream_witness_valid=True,first_stream_witness_rejection=stream_rejected,
        actual_current_pose_projection_regression_count=len(regressions),actual_current_pose_projection_regressions=regressions,
        projection_regression_field_scope="all selected camera anchors; full20Hz native census separately below",
        all_recorded_native_current_pose_projection_census=native_projection_census(mission_payload,states),
        maximum_actual_current_pose_projection_regression_m=max((-r["signed_projection_delta_m"] for r in regressions),default=0.),
        recorded_monotone_route_progress_not_used=True,stream_epoch_never_reset_after_rejection=True,
        physical_stop_arc_not_catalog_remaining_or_stop_buffer=True,not_a_full33knot_or64point_optimization=True,
        offline_reconstruction_not_saved_NN_navigation_or_live_permission=True,denied_source_not_promoted=True,
        training_authorized=False,vehicle_control_approved=False,model_promoted=False,main_functions_approved=0,subfunctions_approved=0)
    return summary,output


def sum_part(anchor,state):
    # HH_261002 - Rotate signed actual displacement only; no monotone projection or clipping of physical XY.
    dx,dy=state["x"]-anchor["x"],state["y"]-anchor["y"];c,s=math.cos(anchor["yaw"]),math.sin(anchor["yaw"])
    return c*dx+s*dy,-s*dx+c*dy


def run(args):
    pins={}; auth_payload=read(args.authorization,args.authorization_sha256,pins); auth=decode(auth_payload)
    require(auth["schema"]==AUTH_SCHEMA and auth["read_only_analysis_authorized"] is True
        and auth["offline_reconstruction_not_saved_NN_navigation"] is True and auth["training_authorized"] is False
        and auth["vehicle_control_approved"] is False and set(auth["inputs"])==INPUT_KEYS, "explicit offline-only root authorization required")
    for relative,expected in FROZEN.items():
        path=canonical(ROOT/relative);require(digest_file(path)==expected,"frozen prepared provider changed");pins[str(path)]=expected
    own=canonical(__file__);require(digest_file(own)==auth["probe_source_sha256"],"probe source external pin differs");pins[str(own)]=auth["probe_source_sha256"]
    sources={k:canonical(v["path"]) for k,v in auth["inputs"].items()}
    def doc(k):return decode(read(sources[k],auth["inputs"][k]["sha256"],pins))
    qa,case,route,manifest,owner,body=(doc(k) for k in ("qa_summary","case_plan","route.json","manifest.json","owner_report","body_summary"))
    require(auth["inputs"]["qa_summary"]["sha256"]==QA_SHA and auth["inputs"]["true64_windows"]["sha256"]==WINDOWS_SHA
        and auth["inputs"]["case_plan"]["sha256"]==CASE_SHA and auth["inputs"]["route.json"]["sha256"]==ROUTE_SHA,"exact retained V6 cohort pins required")
    require(qa["schema"]==QA_SCHEMA and qa["selected_anchor_count"]==1665 and qa["true_future_point_count"]==106560
        and qa["failed_fixed_window_count"]==149 and qa["source_capture_failed"] is True
        and qa["research_training_authorized"] is False and owner["actual_teacher_capture_passed"] is False
        and body["status"]=="PASS", "retained V6 failure/denominator/proofs must not be promoted")
    for k in RAW_NAMES:
        require(qa["source_artifact_sha256"].get(str(sources[k]))==auth["inputs"][k]["sha256"],"caller raw pin differs from full QA source proof")
    mission_payload=read(sources["mission"],auth["inputs"]["mission"]["sha256"],pins)
    require(decode(mission_payload)==mission_proposal(case,route,case_sha=CASE_SHA,route_sha=ROUTE_SHA),"external COMPLETE mission differs from original case/route only assertion")
    journals={k:rows(sources[k],auth["inputs"][k]["sha256"],pins) for k in RAW_NAMES if k.endswith(".jsonl")}
    windows=rows(sources["true64_windows"],WINDOWS_SHA,pins)
    states,cameras=journals["states.jsonl"],journals["camera_frames.jsonl"]
    require(len(states)==qa["raw_native_states"] and len(cameras)==qa["raw_camera_bundles"]
        and len(windows)==1665 and sum(len(w["true_future_rows"]) for w in windows)==106560,"whole retained source census differs")
    pixels={(r["frame"],r["camera"]):r for r in journals["actual_camera_pixels.jsonl"]}
    require(len(pixels)==len(cameras)*6,"all six camera pixel receipts required")
    for c in cameras:
        for n in CAMERAS:
            p=pixels[c["frame"],n]
            require(p["timestamp"]==c["source_timestamps"][n] and p["original_BGRA_sha256"]==c["original_BGRA_source_sha256"][n]
                and p["saved_JPEG_sha256"]==c["saved_JPEG_sha256"][n],"actual six-camera timestamp/pixel identity differs")
    summary,output=analyze(mission_payload,auth["inputs"]["mission"]["sha256"],ROUTE_SHA,states,cameras,windows)
    require(summary["counts"]["original_failed_anchors"]==149
        and sum(w["existing_fixed_GO_18m_support_incompatible"] for w in windows)==45,
        "all original failure and short-GO denominators must remain exact")
    summary.update(source_capture_failed=True,source_QA_failed_fixed_window_count=149,source_QA_sha256=QA_SHA,
        source_true64_journal_sha256=WINDOWS_SHA,source_manifest_status=manifest["status"],source_owner_passed=False,
        source_QA_original_GO_short18m_count=45,
        reconstructed_current_XY_GO_short18m_count=summary["counts"]["GO_old18m_support_incompatible"],
        source_artifact_sha256=dict(pins),prior_full_QA_source_closure_preserved_not_fresh_pixel_decode=True)
    destination=canonical(args.output)
    require(not destination.exists() and destination.is_relative_to(ROOT) and destination==canonical(auth["output"]),"fresh externally authorized output required")
    require(not any(destination.is_relative_to(p.parent) for p in sources.values()),"output cannot mutate any immutable source directory")
    ancestor=destination.parent
    while not ancestor.exists():ancestor=ancestor.parent
    require(shutil.disk_usage(ancestor).free>=5*1024**3+512*1024**2,"5GiB floor plus512MiB output reserve required")
    recheck(pins);destination.mkdir(parents=True,exist_ok=False)
    with (destination/"all_fixed_anchor_boundaries.jsonl").open("xb") as stream:
        for row in output:stream.write(encoded(row)+b"\n")
    with (destination/"summary.json").open("xb") as stream:stream.write(encoded(summary)+b"\n")
    recheck(pins)
    print(json.dumps({k:v for k,v in summary.items() if k!="source_artifact_sha256"},indent=2,allow_nan=False))
    return 0


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("propose-mission","probe"),required=True)
    for name in ("case-plan","case-plan-sha256","route","route-sha256","authorization","authorization-sha256","output"):
        parser.add_argument("--"+name)
    args=parser.parse_args(argv)
    if args.mode=="propose-mission":
        require(all((args.case_plan,args.case_plan_sha256,args.route,args.route_sha256)),"original case/route external pins required")
        pins={};case=decode(read(args.case_plan,args.case_plan_sha256,pins));route=decode(read(args.route,args.route_sha256,pins))
        result=mission_proposal(case,route,case_sha=args.case_plan_sha256,route_sha=args.route_sha256);recheck(pins)
        print(json.dumps(result,sort_keys=True,indent=2,allow_nan=False));return 0
    require(all((args.authorization,args.authorization_sha256,args.output)),"all root authorization/output pins required")
    return run(args)


if __name__=="__main__":raise SystemExit(main())
