"""HH_261002 - Diagnose true natural-goal futures without granting data or vehicle approval.

This standard-library-only audit keeps every prospectively selected anchor and
all 64 original-time labels. Stationary curvature is undefined, not a zero
curvature target. Separate legacy-formula failures are retained unchanged.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left, bisect_right
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from portable_e2e import contract
from scripts.e2e import prepare_carla_common10_dataset as original_math
from scripts.e2e import town07_natural_goal_stop_task_20261002 as natural_task


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "carla.actual_natural_goal_future64_diagnostic.20261002.v1"
TASK_SCHEMA = "carla.actual_natural_goal_stop_teacher.20261002.v1"
PROFILE = "natural_goal_stop_manual1_2p40_hold0p05.20261002.v1"
SOURCE_PINS = {
    "learning/portable_e2e/contract.py": "e2e3a2669fa94aa98cb7e50f20ffa7db433b99eead9a12c654392e4b293a5c09",
    "tools/e2e/prepare_carla_common10_dataset.py": "eab12ef9d83f7ca72d8396ac46d77082cbafbc9600b44c59d733cb76e0de7e65",
    "tools/e2e/audit_portable_target_feasibility.py": "8b56a4990f51639ec5ff9bca12f7453051772d1984e4db768613384b25f00894",
    "learning/portable_e2e/model.py": "fbbedef10b97add830e0d887400107b2bcd6dfb5f0c5c4901e399d2e4717fd6e",
    "tools/e2e/town07_natural_goal_stop_task_20261002.py": "a636058d890c6c28d47bec9de8c134ccaff3594c724ed67860ed8fd16324f35b",
    "tools/e2e/collect_town07_natural_goal_stop_20261002.py": "8eee92c1c83535316ff19189823faa898b10a3344eb6e43eb5844a94f184ce72",
    "tools/e2e/run_owned_town07_natural_goal_stop_20261002.py": "f05fdf610dbd8ec9ec410beb858af1c588b120f5a13b5cc39943d47d0a7eb1ce",
}
DT_NS, NATIVE_NS, FUTURE_POINTS = 100_000_000, 50_000_000, 64
STATIONARY_SPEED, HEADING_DISTANCE = .05, .0001
LEGACY_SPEED, LEGACY_ACCEL, LEGACY_CURVATURE, LEGACY_LATERAL = 30./3.6, 2.9, .2, 2.8
METRICS = ("speed_limit", "speed_acceleration", "speed_deceleration", "xy_speed_limit",
    "xy_acceleration", "xy_deceleration", "xy_curvature", "xy_lateral_acceleration", "yaw_label_heading_envelope")
DENIAL_PATHS = ((), ("result",), ("capture_contract",), ("capture_contract", "goal_stop_profile"),
    ("capture_contract", "goal_stop_profile", "effective_control"), ("capture_contract", "agent_initialization"),
    ("result", "goal_stop_quality"), ("result", "goal_stop_quality", "config"),
    ("result", "goal_stop_quality", "driving_final_stop"), ("result", "goal_stop_quality", "development_pilot"))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value, name):
    require(type(value) in (int, float) and math.isfinite(value), name+" must be finite numeric")
    return float(value)


def stamp(row):
    return round(number(row["timestamp"], "native timestamp")*1e9)


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def canonical(path):
    value = Path(path).absolute()
    require(".." not in value.parts and value == value.resolve()
        and not any(p.is_symlink() for p in (value, *value.parents)), "canonical nonsymlink path required")
    return value


def parse_json(payload):
    def pairs(values):
        result = {}
        for key, value in values:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    return json.loads(payload, object_pairs_hook=pairs,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError("nonfinite JSON "+value)))


def pinned_bytes(path, pins, expected=None):
    path = canonical(path)
    require(path.is_file(), "regular input required: "+str(path))
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    require(expected is None or digest == expected, "input SHA differs: "+str(path))
    require(str(path) not in pins or pins[str(path)] == digest, "input changed between reads")
    pins[str(path)] = digest
    return payload


def document(path, pins, expected=None):
    value = parse_json(pinned_bytes(path, pins, expected))
    require(isinstance(value, dict), "JSON object required")
    return value


def journal(path, pins):
    return [parse_json(line) for line in pinned_bytes(path, pins).splitlines() if line.strip()]


def recheck(pins):
    for path, expected in pins.items():
        require(hashlib.sha256(canonical(path).read_bytes()).hexdigest() == expected, "pinned input changed: "+path)


def denial_witness(manifest):
    """HH_261002 - Invoke the untouched denial guard; a diagnostic never converts denied raw input."""
    denied = []
    for path in DENIAL_PATHS:
        value = manifest
        for key in path:
            value = value.get(key) if isinstance(value, dict) else None
        if isinstance(value, dict):
            for key, prohibited in (("training_data_approved", False), ("development_only", True)):
                if key in value:
                    require(type(value[key]) is bool, "native admission marker must be Boolean")
                    if value[key] is prohibited:
                        denied.append(".".join((*path, key)))
    try:
        contract._require_admission_not_denied(manifest, "native goal pilot", include_native_metadata=True)
    except contract.ContractError:
        require(bool(denied), "original denial guard rejected an unrecorded restriction")
    else:
        require(not denied, "native denial guard was bypassed")
    return dict(native_explicit_denial_present=bool(denied), native_denial_fields=denied,
        raw_native_denials_overridden=False, original_denial_guard_executed=True)


def intent_witness(row):
    """HH_261002 - Current measured teacher phase is a label, never a runtime model input."""
    record = row["goal_stop"]
    require(record.get("profile_id") == PROFILE, "goal profile differs")
    speed = math.hypot(number(row["vx"], "vx"), number(row["vy"], "vy"))
    phase = record.get("phase")
    if phase == "cruise":
        intent, valid = "GO", True
    elif phase == "approach":
        intent, valid = "BRAKE_TO_STOP", speed > 0.
    elif phase in ("holding", "complete"):
        intent = "HOLD"
        valid = speed <= STATIONARY_SPEED and record.get("goal_window") is True \
            and record.get("upstream_of_endpoint") is True and number(record["terminal_overshoot_m"], "overshoot") <= 0.
    else:
        intent, valid = None, False
    return dict(intent=intent, valid=bool(valid), teacher_phase=phase, source_frame=row["frame"],
        source_timestamp_ns=stamp(row), measured_current_speed_mps=speed,
        annotations_are_labels_not_model_inputs=True)


def native_timeline(states):
    require(bool(states), "native states required")
    timeline, phase_index = [], -1
    phases = ("stationary_warmup", "driving", "stationary_tail")
    for index, row in enumerate(states):
        require(type(row.get("frame")) is int and row["frame"] >= 0, "integer native frame required")
        for name in ("x", "y", "z", "yaw", "vx", "vy", "ax", "ay", "yaw_rate"):
            number(row[name], name)
        now = stamp(row)
        if index:
            require(row["frame"] == states[index-1]["frame"]+1
                and abs(now-timeline[-1][0]-NATIVE_NS) <= 1, "native20Hz frame/timestamp gap")
        phase = row.get("capture_phase")
        require(phase in phases and phases.index(phase) >= phase_index, "native phase order differs")
        phase_index = phases.index(phase)
        for key in ("collision", "lane_invasion"):
            require(isinstance(row[key], list), "native events must remain lists")
        timeline.append((now, row))
    require(all(any(r["capture_phase"] == p for r in states) for p in phases), "all raw phases required")
    return timeline


def future_window(timeline, anchor):
    """HH_261002 - Reuse frozen true-time interpolation, never snapped native timestamps or route labels."""
    states, times = [r for _, r in timeline], [t for t, _ in timeline]
    anchor_ns, v0 = stamp(anchor), math.hypot(anchor["vx"], anchor["vy"])
    prior_xy, prior_speed, prior_geo, legacy_heading, prior_yaw = (0., 0.), v0, v0, 0., 0.
    legacy_prior_speed = legacy_prior_geo = max(0., min(anchor["vx"], LEGACY_SPEED))
    domain_heading, prior_domain_valid = 0., v0 > STATIONARY_SPEED
    rows = []
    for index in range(FUTURE_POINTS):
        target_ns = anchor_ns+(index+1)*DT_NS
        future = original_math._interpolate_state(timeline, target_ns)
        if future is None:
            rows.append(dict(index=index, target_timestamp_ns=target_ns, valid=False,
                invalid_reason="episode_end", xy=None, yaw_rad=None, speed_mps=None))
            continue
        upper = bisect_right(times, target_ns)
        lo, hi = upper-1, upper-1 if times[upper-1] == target_ns else upper
        xy, yaw = original_math._relative_xy_yaw(anchor, future)
        speed = math.hypot(future["vx"], future["vy"])
        vector = (xy[0]-prior_xy[0], xy[1]-prior_xy[1])
        distance = math.hypot(*vector); geo = distance/.1
        acceleration, geo_acceleration = (speed-prior_speed)/.1, (geo-prior_geo)/.1
        curvature = lateral = None
        if distance > HEADING_DISTANCE:
            heading = math.atan2(vector[1], vector[0])
            curvature = abs(wrap(heading-legacy_heading))/distance
            lateral = curvature*max(legacy_prior_geo, geo)**2
            legacy_heading = heading
        heading_defined = distance > HEADING_DISTANCE and min(prior_speed, speed) > STATIONARY_SPEED
        domain_curvature = None
        if heading_defined and prior_domain_valid:
            domain_curvature = abs(wrap(math.atan2(vector[1], vector[0])-domain_heading))/distance
        if heading_defined:
            domain_heading = math.atan2(vector[1], vector[0])
        prior_domain_valid = heading_defined
        yaw_delta = abs(wrap(yaw-prior_yaw))
        yaw_assessed = speed*.1 > HEADING_DISTANCE
        yaw_bound = min(LEGACY_CURVATURE, LEGACY_LATERAL/max(legacy_prior_speed, speed, 1e-6)**2)*speed*.1
        domain_yaw_bound = min(LEGACY_CURVATURE, LEGACY_LATERAL/max(prior_speed, speed, 1e-6)**2)*speed*.1
        metrics = {
            "speed_limit": [speed, speed > LEGACY_SPEED+1e-6],
            "speed_acceleration": [(speed-legacy_prior_speed)/.1, (speed-legacy_prior_speed)/.1 > LEGACY_ACCEL+1e-6],
            "speed_deceleration": [(speed-legacy_prior_speed)/.1, (speed-legacy_prior_speed)/.1 < -LEGACY_ACCEL-1e-6],
            "xy_speed_limit": [geo, geo > LEGACY_SPEED+1e-6],
            "xy_acceleration": [(geo-legacy_prior_geo)/.1, (geo-legacy_prior_geo)/.1 > LEGACY_ACCEL+1e-6],
            "xy_deceleration": [(geo-legacy_prior_geo)/.1, (geo-legacy_prior_geo)/.1 < -LEGACY_ACCEL-1e-6],
            "xy_curvature": [curvature, curvature is not None and curvature > LEGACY_CURVATURE+1e-6],
            "xy_lateral_acceleration": [lateral, lateral is not None and lateral > LEGACY_LATERAL+1e-6],
            "yaw_label_heading_envelope": [yaw_delta if yaw_assessed else None, yaw_assessed and yaw_delta > yaw_bound+1e-6],
        }
        # HH_261002 - Only mathematically undefined heading labels are masked; raw XY/velocity and old flags remain.
        domain_metrics = dict(metrics)
        domain_metrics["speed_acceleration"] = [acceleration, acceleration > LEGACY_ACCEL+1e-6]
        domain_metrics["speed_deceleration"] = [acceleration, acceleration < -LEGACY_ACCEL-1e-6]
        domain_metrics["xy_acceleration"] = [geo_acceleration, geo_acceleration > LEGACY_ACCEL+1e-6]
        domain_metrics["xy_deceleration"] = [geo_acceleration, geo_acceleration < -LEGACY_ACCEL-1e-6]
        domain_metrics["xy_curvature"] = [domain_curvature, domain_curvature is not None and domain_curvature > LEGACY_CURVATURE+1e-6]
        domain_metrics["xy_lateral_acceleration"] = [None if domain_curvature is None else domain_curvature*max(prior_geo, geo)**2,
            domain_curvature is not None and domain_curvature*max(prior_geo, geo)**2 > LEGACY_LATERAL+1e-6]
        yaw_defined = min(prior_speed, speed) > STATIONARY_SPEED and yaw_assessed
        domain_metrics["yaw_label_heading_envelope"] = [yaw_delta if yaw_defined else None, yaw_defined and yaw_delta > domain_yaw_bound+1e-6]
        semantic = intent_witness(states[lo])
        rows.append(dict(index=index, target_timestamp_ns=target_ns, valid=True, invalid_reason=None,
            xy=xy, yaw_rad=yaw, speed_mps=speed, instantaneous_speed_not_XY_chord_speed=True,
            source_native_bracket_frames=[states[lo]["frame"], states[hi]["frame"]],
            source_native_bracket_timestamps_ns=[times[lo], times[hi]],
            future_capture_phases=[states[lo]["capture_phase"], states[hi]["capture_phase"]],
            true_future_semantic_witness=semantic, legacy_formula_metrics=metrics,
            stationary_domain_metrics=domain_metrics, curvature_label_valid=domain_curvature is not None,
            undefined_curvature_is_not_zero_label=domain_curvature is None,
            initial_measured_planar_acceleration_mps2=acceleration if index == 0 else None,
            initial_XY_interval_acceleration_mps2=geo_acceleration if index == 0 else None))
        prior_xy, prior_speed, prior_geo, prior_yaw = xy, speed, geo, yaw
        legacy_prior_speed, legacy_prior_geo = speed, geo
    return rows


def native_intervals(timeline, driving_origin_ns):
    """HH_261002 - Actual20Hz scalar dv/dt must not disappear inside100ms future averages or curvature masks."""
    result = []
    for index in range(1,len(timeline)):
        first_ns, first = timeline[index-1]; second_ns, second = timeline[index]
        speed = math.hypot(second["vx"],second["vy"])
        acceleration = (speed-math.hypot(first["vx"],first["vy"]))/((second_ns-first_ns)/1e9)
        events = len(second["collision"])+len(second["lane_invasion"])
        reasons = []
        if abs(acceleration) > 3.+1e-6:reasons.append("actual_native_scalar_dvdt_exceeds3")
        # HH_261002 - The original physical2.9 bound is distinct from the retained3.0 native-support diagnostic.
        if abs(acceleration) > LEGACY_ACCEL+1e-6:reasons.append("actual_native_scalar_dvdt_exceeds_physical2p9")
        if speed > 2.5:reasons.append("actual_native_base_speed_exceeds2p5")
        actor_speed = None
        if "world_velocity_carla" in second:
            vector = second["world_velocity_carla"]
            require(isinstance(vector,list) and len(vector) == 3, "actual actor velocity vector required")
            actor_speed = math.hypot(number(vector[0],"actor vx"),number(vector[1],"actor vy"))
            if actor_speed > 2.7:reasons.append("actual_native_actor_speed_exceeds2p7")
        if events:reasons.append("actual_native_collision_or_invasion")
        result.append(dict(native_index=index,before_frame=first["frame"],after_frame=second["frame"],
            before_timestamp_ns=first_ns,after_timestamp_ns=second_ns,
            after_driving_relative_seconds=(second_ns-driving_origin_ns)/1e9,
            before_measured_planar_speed_mps=math.hypot(first["vx"],first["vy"]),
            after_measured_planar_speed_mps=speed,actual_scalar_dvdt_mps2=acceleration,
            native_scalar_dvdt_exceeds3=abs(acceleration) > 3.+1e-6,
            native_scalar_dvdt_exceeds_original_physical2p9=abs(acceleration) > LEGACY_ACCEL+1e-6,
            after_actual_actor_API_planar_speed_mps=actor_speed,
            after_API_base_acceleration_vector_norm_mps2=math.hypot(second["ax"],second["ay"]),
            failure_reasons=reasons))
    return result


def governor_receipt_binding(states, updates):
    """HH_261002 - Bind phase labels to actual same-frame update receipts, not a renamed control or future oracle."""
    by_frame = {}
    for row in updates:
        require(row.get("schema") == TASK_SCHEMA+".actual_governor_update"
            and row.get("governor_math_changed") is False and row.get("measured_speed_clipped") is False,
            "original actual governor update receipt required")
        require(type(row.get("measured_state_frame")) is int, "governor measured frame required")
        by_frame.setdefault(row["measured_state_frame"],[]).append(row["original_governor_update"])
    require(set(by_frame) == {r["frame"] for r in states}, "governor/native frame denominator differs")
    fields = ("profile_id","phase","remaining_route_arc_m","planar_goal_distance_m","terminal_overshoot_m",
        "measured_speed_mps","distance_speed_envelope_mps","goal_window","upstream_of_endpoint",
        "hold_ticks","required_hold_ticks","hold_duration_sec","complete")
    for state in states:
        original = state["goal_stop"]
        # HH_261002 - Frozen setup/tail writes target0 after update; only driving targets are learning witnesses.
        keys = fields+("target_speed_mps",) if state["capture_phase"] == "driving" else fields
        require(any(all(key in original and value.get(key) == original[key] for key in keys)
            for value in by_frame[state["frame"]]), "native goal phase differs from actual same-frame governor receipt")
        require(math.isclose(number(original["measured_speed_mps"],"governor measured speed"),
            math.hypot(state["vx"],state["vy"]),rel_tol=0.,abs_tol=1e-9), "governor measured v0 differs from original state")
    return dict(all_native_phase_labels_bind_actual_same_frame_governor_receipts=True,
        native_frame_count=len(states),actual_governor_update_receipt_count=len(updates),
        setup_tail_target_zero_post_update_is_not_a_training_intent_override=True,
        independent_full_governor_math_or_mission_projection_replay_performed=False)


def audit_core(plan, manifest, states, cameras):
    require(plan.get("schema") == TASK_SCHEMA and plan.get("case_id") == "pilot_00"
        and plan.get("seed") == 2026102200 and plan.get("selected_first_driving_relative_native_frame") == 80
        and plan.get("stationary_tail_native_ticks") == 130
        and plan.get("stationary_tail_can_be_true_future_context") is True
        and plan.get("stationary_tail_is_training_anchor") is False, "exact prospective natural-goal pilot required")
    denial = denial_witness(manifest)
    require(manifest["capture_contract"]["natural_goal_stop_teacher"] == plan, "native declaration differs")
    timeline = native_timeline(states); by_frame = {r["frame"]:i for i, r in enumerate(states)}
    driving = [r for r in states if r["capture_phase"] == "driving"]
    complete = manifest["result"]["moving_task"]["actual_goal_completion"]
    require(isinstance(complete, dict) and complete["frame"] in by_frame, "actual2sHOLD completion required")
    closing = states[by_frame[complete["frame"]]]
    require(stamp(closing) == complete["timestamp_ns"] and closing["capture_phase"] == "driving"
        and closing["goal_stop"].get("complete") is True and closing["goal_stop"].get("hold_duration_sec", 0.) >= 2.,
        "completion must bind actual driving native HOLD witness")
    require(sum(r["capture_phase"] == "stationary_tail" for r in states) == 130, "all130 raw tail states required")
    require(bool(cameras), "actual six-camera bundles required")
    first_frame, windows, selected = driving[0]["frame"], [], []
    intervals = native_intervals(timeline,stamp(driving[0]))
    for index, camera in enumerate(cameras):
        frame = camera["frame"]
        require(frame in by_frame and type(frame) is int, "camera has no native frame")
        state = states[by_frame[frame]]
        require(tuple(camera["camera_order"]) == contract.CAMERA_ORDER and tuple(camera["images"]) == contract.CAMERA_ORDER
            and tuple(camera["source_timestamps"]) == contract.CAMERA_ORDER, "exact producer camera order required")
        require(camera["capture_phase"] == state["capture_phase"] and stamp(camera) == stamp(state)
            and all(round(number(v, "camera stamp")*1e9) == stamp(state) for v in camera["source_timestamps"].values()),
            "actual same-frame six-camera native binding required")
        if index:
            require(frame == cameras[index-1]["frame"]+2 and abs(stamp(camera)-stamp(cameras[index-1])-DT_NS) <= 2,
                "six-camera10Hz frame/timestamp gap")
        eligible = state["capture_phase"] == "driving" and frame >= first_frame+80 and frame <= closing["frame"]
        require(type(camera.get("moving_task_selected")) is bool and camera["moving_task_selected"] is eligible,
            "event-defined anchor membership changed; never crop failed windows")
        if not eligible:
            continue
        selected.append(camera)
        failures = []
        history = cameras[max(0,index-9):index+1]
        history_valid = index >= 9 and len(history) == 10 and all(r["capture_phase"] == "driving" for r in history)
        if not history_valid or abs(stamp(camera)-stamp(history[0])-900_000_000) > 18:
            failures.append("actual10_bundle_driving_history")
        if by_frame[history[0]["frame"]] <= 0 or states[by_frame[history[0]["frame"]]-1]["capture_phase"] != "driving":
            failures.append("preceding_native_history_guard")
        ego_indices = [by_frame[frame]-18+2*i for i in range(10)]
        ego_masks = [i >= 0 and states[i]["capture_phase"] == "driving" for i in ego_indices]
        if not all(ego_masks):failures.append("actual_causal_ten_ego_history")
        rows = future_window(timeline, state)
        protected_lo = max(1,by_frame[history[0]["frame"]]-1)
        protected_hi = min(len(states)-1,bisect_left([t for t,_ in timeline],stamp(state)+6_400_000_000))
        protected_failures = [r for r in intervals if protected_lo <= r["native_index"] <= protected_hi and r["failure_reasons"]]
        if protected_failures:failures.append("protected_native20Hz_dynamics_or_events")
        semantic = intent_witness(state)
        if not semantic["valid"]:
            failures.append("undefined_or_contradictory_current_teacher_intent")
        if not all(r["valid"] for r in rows):
            failures.append("true64future_unavailable")
        if any(r["valid"] and r["speed_mps"] > 2.5 for r in rows) or math.hypot(state["vx"],state["vy"]) > 2.5:
            failures.append("actual_low_speed_envelope")
        if any(r["valid"] and any(flag for _,flag in r["stationary_domain_metrics"].values()) for r in rows):
            failures.append("defined_domain_discrete_bounds")
        if any(r["valid"] and not r["true_future_semantic_witness"]["valid"] for r in rows):
            failures.append("future_teacher_semantics_contradict_actual_state")
        support = number(state["goal_stop"]["remaining_route_arc_m"], "actual remaining route arc")
        windows.append(dict(source_frame=frame, source_timestamp_ns=stamp(state),
            driving_relative_native_frame=frame-first_frame, current_semantic_witness=semantic,
            causal_remaining_route_arc_m=support, causal_route_is_not_free_space_or_future_truth=True,
            existing_fixed_GO_18m_support_incompatible=semantic["intent"] == "GO" and support < 18.,
            actual_history_camera_frames=[r["frame"] for r in history], real_ten_bundle_history_valid=history_valid,
            actual_model_five_camera_history_frames=[r["frame"] for r in history[1::2]],
            actual_model_five_camera_history_mask=[True]*5 if len(history) == 10 else [False]*(5-len(history[1::2]))+[True]*len(history[1::2]),
            actual_ten_ego_history_native_frames=[states[i]["frame"] if valid else None for i,valid in zip(ego_indices,ego_masks)],
            actual_ten_ego_history_valid_mask=ego_masks,
            protected_native_indices=[protected_lo,protected_hi],protected_native_interval_failures=protected_failures,
            true_future_rows=rows, necessary_defined_domain_checks_clear=not failures, failure_reasons=failures))
    require(bool(selected) and selected[0]["frame"]-first_frame == 80
        and stamp(selected[-1])-stamp(selected[0]) >= 30_000_000_000, "all prospective anchors from4s/min30s span required")
    moving = manifest["result"]["moving_task"]
    require(len(selected) == moving["selected_camera_anchor_count"]
        and selected[0]["frame"] == moving["first_selected_camera"]["frame"]
        and selected[-1]["frame"] == moving["last_selected_camera"]["frame"], "native selected denominator differs")
    counters = {kind:{name:Counter() for name in METRICS} for kind in ("legacy_formula_metrics", "stationary_domain_metrics")}
    for window in windows:
        for row in window["true_future_rows"]:
            if row["valid"]:
                for kind in counters:
                    for name,(value,failed) in row[kind].items():
                        counters[kind][name]["unassessed" if value is None else "violations" if failed else "assessed_nonviolations"] += 1
    scalar_peaks = []
    for (first_ns,first),(second_ns,second) in zip(timeline,timeline[1:]):
        scalar_peaks.append((math.hypot(second["vx"],second["vy"])-math.hypot(first["vx"],first["vy"]))/((second_ns-first_ns)/1e9))
    protected = set(i for w in windows for i in range(w["protected_native_indices"][0],w["protected_native_indices"][1]+1))
    summary = dict(schema=SCHEMA, status="TRUE_FUTURE_DIAGNOSTIC_ONLY_NOT_ADMITTED",
        raw_native_states=len(states), raw_camera_bundles=len(cameras), raw_six_camera_image_count=len(cameras)*6,
        selected_anchor_count=len(windows), selected_span_ns=stamp(selected[-1])-stamp(selected[0]),
        true_future_point_count=sum(r["valid"] for w in windows for r in w["true_future_rows"]),
        required_true_future_point_count=len(windows)*64,
        anchor_teacher_intent_counts=dict(Counter(w["current_semantic_witness"]["intent"] for w in windows)),
        GO_anchors_incompatible_with_existing_18m_support=sum(w["existing_fixed_GO_18m_support_incompatible"] for w in windows),
        failed_fixed_window_count=sum(not w["necessary_defined_domain_checks_clear"] for w in windows),
        protected_native_failure_anchor_count=sum(bool(w["protected_native_interval_failures"]) for w in windows),
        protected_unique_native_interval_failure_count=sum(r["native_index"] in protected and bool(r["failure_reasons"]) for r in intervals),
        outside_protected_unique_native_interval_failure_count=sum(r["native_index"] not in protected and bool(r["failure_reasons"]) for r in intervals),
        all_raw_native_interval_failures=[r for r in intervals if r["failure_reasons"]],
        full_raw_native_scalar_dvdt_exceeds3_count=sum(r["native_scalar_dvdt_exceeds3"] for r in intervals),
        full_raw_native_scalar_dvdt_exceeds_original_physical2p9_count=sum(r["native_scalar_dvdt_exceeds_original_physical2p9"] for r in intervals),
        source_capture_status=manifest.get("status"),source_capture_failed=manifest.get("status") != "complete"
            or (manifest["result"].get("goal_stop_quality") or {}).get("status") != "PASS"
            or manifest["result"].get("moving_task",{}).get("task_complete") is not True,
        fixed_window_failure_counts=dict(Counter(e for w in windows for e in w["failure_reasons"])),
        full_raw_native_scalar_dvdt_acceleration_max_mps2=max(scalar_peaks),
        full_raw_native_scalar_dvdt_deceleration_min_mps2=min(scalar_peaks),
        full_raw_API_base_acceleration_vector_norm_max_mps2=max(math.hypot(r["ax"],r["ay"]) for r in states),
        full_raw_event_count=sum(len(r["collision"])+len(r["lane_invasion"]) for r in states),
        future_discrete_metric_counts={k:{n:dict(c) for n,c in v.items()} for k,v in counters.items()},
        legacy_formula_initial_speed_definition="Original max(0,min(vx,30/3.6)); retained diagnostic only, never current model v0.",
        stationary_domain_initial_speed_definition="Exact measured hypot(vx,vy); zero and near-zero are preserved.",
        stationary_curvature_null_is_unassessed_not_zero_label=True,
        all64_actual_XY_yaw_endpoint_speed_labels_preserved=True, actual_v0_is_measured_planar_norm_not_clipped=True,
        current_and_future_semantic_annotations_are_labels_only=True, original_constant_deceleration_decoder_fit_not_proven=True,
        initial_acceleration_is_first_100ms_endpoint_speed_difference_not_API_vector_norm=True,
        startup_anchors_selected=False, raw_startup_retained=True, stationary_tail_is_anchor=False,
        stationary_tail_allowed_as_true_future_context=True, no_failed_anchor_or_pixel_removed=True,
        training_data_approved=False, research_training_authorized=False, research_holdout_evaluation_authorized=False,
        dataset_conversion_performed=False, vehicle_control_approved=False, full_RTE04_approved=False,
        main_functions_approved=0, subfunctions_approved=0, **denial)
    return summary, windows


def inspect(episode, plan_path, plan_sha):
    episode, plan_path = canonical(episode), canonical(plan_path)
    require(episode.is_relative_to(ROOT) and plan_path.is_relative_to(ROOT), "workspace raw inputs required")
    require(episode.name in ("episode","episode.partial") and episode.is_dir(), "retained raw episode required")
    require(not (episode.parent/("episode.partial" if episode.name == "episode" else "episode")).exists(),
        "ambiguous finalized/partial raw episode")
    pins = {}
    for name, expected in SOURCE_PINS.items():
        pinned_bytes(ROOT/name, pins, expected)
    pinned_bytes(Path(__file__).resolve(), pins)
    plan = document(plan_path, pins, plan_sha)
    fixed = natural_task.fixed_contract("v1","pilot_00")
    require(all(key in plan and type(plan[key]) is type(value) and plan[key] == value for key,value in fixed.items()),
        "full prospectively fixed natural-goal contract differs")
    pinned_bytes(episode/"moving_task_plan.json", pins, plan_sha)
    manifest = document(episode/"manifest.json", pins)
    states, cameras = journal(episode/"states.jsonl",pins), journal(episode/"camera_frames.jsonl",pins)
    summary, windows = audit_core(plan, manifest, states, cameras)
    # HH_261002 - Bind owner/body claims to raw bytes, while explicitly not rerunning or approving those audits.
    owner = document(episode.parent/"owner_result.json", pins)
    diagnostic = document(episode.parent/"diagnostic_report.json", pins)
    body_path = episode.parent/"derived_body/summary.json"
    body = document(body_path,pins) if body_path.is_file() else dict(status="NOT_PROVEN",
        all_native_body_and_swept_clearance_passed=False,input_sha256={},
        reason="No retained body receipt; original failed owner may have stopped before geometry. Not recomputed by this future-only diagnostic.")
    require(owner["declaration_sha256"] == plan_sha, "actual owner plan differs")
    for name, expected in body["input_sha256"].items():
        pinned_bytes(name,pins,expected)
    for camera in cameras:
        require(set(camera["saved_JPEG_sha256"]) == set(contract.CAMERA_ORDER)
            and set(camera["original_BGRA_source_sha256"]) == set(contract.CAMERA_ORDER), "all six original pixel receipts required")
        for name in contract.CAMERA_ORDER:
            path = canonical(episode/camera["images"][name])
            require(path.is_relative_to(episode), "camera image escaped raw episode")
            pinned_bytes(path,pins,camera["saved_JPEG_sha256"][name])
    updates = journal(episode/"actual_goal_governor.jsonl",pins)
    governor_binding = governor_receipt_binding(states,updates)
    pinned_bytes(episode/"actual_goal_target_assignments.jsonl",pins)
    summary.update(case_plan_sha256=plan_sha, native_manifest_sha256=pins[str(episode/"manifest.json")],
        original_owner_capture_passed=diagnostic.get("actual_teacher_capture_passed") is True,
        original_body_report_passed=body.get("all_native_body_and_swept_clearance_passed") is True,
        actual_governor_receipt_binding=governor_binding,
        owner_and_body_are_prior_pinned_evidence_not_independently_rerun=True,
        source_artifact_sha256=pins)
    recheck(pins)
    return summary, windows, pins


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--episode",required=True);parser.add_argument("--case-plan",required=True)
    parser.add_argument("--case-plan-sha256",required=True);parser.add_argument("--output")
    args = parser.parse_args(argv)
    summary, windows, pins = inspect(args.episode,args.case_plan,args.case_plan_sha256)
    if args.output:
        output = canonical(args.output)
        require(output.is_relative_to(ROOT) and not output.exists()
            and not output.is_relative_to(canonical(args.episode).parent), "fresh derived output outside immutable pilot required")
        recheck(pins)
        output.mkdir(parents=True,exist_ok=False)
        with (output/"summary.json").open("x",encoding="utf-8") as stream:
            json.dump(summary,stream,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False);stream.write("\n")
        with (output/"all_fixed_anchor_true_future64.jsonl").open("x",encoding="utf-8") as stream:
            for row in windows:stream.write(json.dumps(row,sort_keys=True,allow_nan=False)+"\n")
        recheck(pins)
    print(json.dumps({k:v for k,v in summary.items() if k != "source_artifact_sha256"},indent=2,allow_nan=False))
    return 0 if summary["failed_fixed_window_count"] == 0 and summary["full_raw_event_count"] == 0 \
        and not summary["source_capture_failed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
