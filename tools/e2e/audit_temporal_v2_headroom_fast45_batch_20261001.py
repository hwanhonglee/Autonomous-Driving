"""HH_261001 - Independently retain every actual fast45 attempt and decompose all fresh delivery-to-ACK receipts.

This standard-library-only, create-only archive reader never runs inference,
geometry, a simulator or an actuator. Existing geometric rows remain original
evidence; camera files are counted but pixels are not inspected by this audit.
"""

from __future__ import annotations

import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
BATCH = ROOT / "storage/experiments/validation/2026-10-01/temporal_v2_headroom_fast45_town07_v1"
OUTPUT = BATCH.with_name("temporal_v2_headroom_fast45_batch_audit_v1")
BATCH_SHA = "677cd47bd9de5003ba1e661ff418585e3d3ddbc653f8de1a862ce1eb3426fac5"
CHECKPOINT_SHA = "7902216449dd8e7a601d63128a38773a5f3bf59c1660a7684b7f2d79791659b6"
SCHEMA = "portable_e2e.actual_headroom_fast45_all_attempt_archive_audit.20261001.v1"
CAMERAS = ("CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT")
GEOMETRY = {"sampled_body_raw_lane": ("sampled_body", "raw_lane", 900),
            "sampled_body_inset_10cm_lane": ("sampled_body", "inset_10cm_lane", 900),
            "previous_sweep_raw_lane": ("previous_sweep", "raw_lane", 899),
            "previous_sweep_inset_10cm_lane": ("previous_sweep", "inset_10cm_lane", 899)}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "regular evidence file required: " + str(path))
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            result.update(block)
    return result.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_bytes())


def rows(path):
    with (gzip.open if str(path).endswith(".gz") else open)(path, "rt") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def assert_pins(pins):
    for name, sha in pins.items():
        require(digest(name) == sha, "original evidence/source changed: " + str(name))


def percentile(values, fraction):
    # HH_261001 - Reproduce the original linear percentile without importing NumPy or changing the gate.
    require(values and 0 <= fraction <= 1, "nonempty percentile denominator required")
    ordered = sorted(values)
    position = (len(ordered)-1)*fraction
    lo, hi = math.floor(position), math.ceil(position)
    return ordered[lo] + (ordered[hi]-ordered[lo])*(position-lo)


def stats(values):
    return dict(count=len(values), mean=math.fsum(values)/len(values),
                p95=percentile(values, .95), maximum=max(values)) if values else dict(count=0)


def prior_ack(state, receipts):
    # HH_261001 - The latest prior ACK of ANY source controls the observation; outgoing same-frame commands do not.
    accepted = [row for row in receipts if row.get("status") == "ACKNOWLEDGED"
                and row.get("server_accepted") is True and row["before_frame"] < state["frame"]]
    require(accepted, "native observation lacks an accepted prior ACK")
    receipt = max(accepted, key=lambda row: row["sequence"])
    require(state["control_transport"]["status"] == "PASS"
            and state["control_transport"]["expected_receipt_sequence"] == receipt["sequence"],
            "actual native control is not bound to the latest prior ACK")
    return receipt


def decompose(timing, command):
    """HH_261001 - Separate nonoverlapping observed intervals; retain unknown residuals as unknown residuals."""
    observed = timing["observed_bundle"]
    delivery = observed["exact_bundle_delivery_monotonic_ns"]
    built = observed["observed_frame_build_finished_monotonic_ns"]
    start, finish = timing["proposal_started_monotonic_ns"], timing["proposal_finished_monotonic_ns"]
    ack_start, ack_finish = timing["ACK_started_monotonic_ns"], timing["ACK_finished_monotonic_ns"]
    require(delivery <= built <= start <= finish <= ack_start <= ack_finish, "monotonic interval order changed")
    policy = float(command["total_seconds"])*1000
    inputs = float(command["owned_camera_history"]["build_and_source_tensor_hash_seconds"])*1000
    nn = float(command["raw_nn_seconds"])*1000
    values = dict(observed_bundle_build_ms=(built-delivery)/1e6,
                  postobservation_preproposal_uninstrumented_ms=(start-built)/1e6,
                  session_wrapper_derived_residual_ms=(finish-start)/1e6-policy,
                  policy_input_build_and_source_tensor_hash_ms=inputs, raw_NN_pipeline_ms=nn,
                  policy_decoder_gate_PI_other_derived_residual_ms=policy-inputs-nn,
                  proposal_to_ACK_send_gap_ms=(ack_start-finish)/1e6,
                  ACK_send_ms=(ack_finish-ack_start)/1e6)
    require(all(math.isfinite(v) and v >= -1e-6 for v in values.values()), "negative/nonfinite timing component")
    wall = (ack_finish-delivery)/1e6
    require(math.isclose(math.fsum(values.values()), wall, abs_tol=1e-6)
            and math.isclose(timing["exact_bundle_delivery_to_ACK_seconds"]*1000, wall, abs_tol=1e-6)
            and math.isclose((finish-start)/1e9, timing["propose_seconds"], abs_tol=1e-12)
            and math.isclose((ack_finish-ack_start)/1e9, timing["ACK_send_seconds"], abs_tol=1e-12),
            "timing receipt arithmetic differs")
    return dict(command_frame=command["command_frame"], inference_frame=command["inference_frame"],
                ACK_sequence=timing["ACK_sequence"], delivery_to_ACK_ms=wall,
                policy_total_ms=policy, policy_100ms_passed=policy <= 100,
                observed_delivery_to_ACK_100ms_passed=wall <= 100,
                exact_bundle_wait_before_delivery_ms=observed["exact_bundle_wait_seconds"]*1000,
                components_ms=values, residuals_are_not_separately_instrumented=True,
                sensor_callback_entry_unmeasured=True, exposure_wall_clock_unproven=True)


def audit_timing(commands, timings, receipts, states, cameras, declaration_sha):
    expected = [row for row in commands if row.get("learned_control_proposed") is True
                and row.get("held_between_camera_frames") is False]
    fresh = [row for row in timings if row.get("learned_control_proposed") is True
             and row.get("held_between_camera_frames") is False]
    require(expected and len(expected) == len(fresh), "fresh timing denominator incomplete")
    by_ack = {row["ACK_sequence"]: row for row in fresh}
    require(len(by_ack) == len(fresh), "duplicate fresh timing ACK")
    ack_by_id, state_by_frame = {r["sequence"]: r for r in receipts}, {r["frame"]: r for r in states}
    camera_by_frame = {r["frame"]: r for r in cameras}
    result = []
    for command in expected:
        timing = by_ack[command["ack_receipt_sequence"]]
        receipt = ack_by_id[timing["ACK_sequence"]]
        require(timing["command_frame"] == command["command_frame"] == receipt["before_frame"]
                and timing["inference_frame"] == command["inference_frame"]
                and timing["ACK_server_accepted"] is True and receipt["server_accepted"] is True
                and timing["source_checkpoint_sha256"] == CHECKPOINT_SHA
                and timing["stream_source_sha256"] == declaration_sha, "fresh proposal/ACK/source mismatch")
        frame = command["inference_frame"]
        require(timing["observed_bundle"]["native_frame"] == frame
                and frame in state_by_frame and frame in camera_by_frame
                and timing["observed_bundle"]["source_image_timestamp_simulation_ns"]
                    == round(camera_by_frame[frame]["timestamp"]*1e9), "fresh observed native/camera frame mismatch")
        require(timing["clock"] == "time.perf_counter_ns_monotonic"
                and timing["policy_deadline_seconds"] == .1 and timing["policy_deadline_relaxed"] is False
                and timing["sensor_callback_entry_observed"] is False
                and timing["sensor_callback_entry_to_ACK_seconds"] is None
                and timing["exposure_to_ACK_wall_latency_proven"] is False
                and timing["ten_hz_end_to_end_proven"] is False, "timing scope or unchanged deadline differs")
        result.append(decompose(timing, command))
    return result


def audit_body(geometry, reported):
    # HH_261001 - Recount the original archived geometry; do not rerun or relax its frozen geometry algorithm.
    result = {}
    for name, (section, lane, count) in GEOMETRY.items():
        assessed = [(r["frame"], (r[section]["regions"] if section == "previous_sweep" else r[section])[lane])
                    for r in geometry if r[section] is not None]
        require(len(assessed) == count, "whole archived body/sweep denominator missing")
        bad = [frame for frame, row in assessed if row["area_gate_clear"] is not True]
        metric = reported[name]
        require(metric["assessed_count"] == count and metric["violation_frames"] == bad
                and metric["violation_count"] == len(bad)
                and metric["status"] == ("FAIL" if bad else "PASS"), "archived geometry census mismatch")
        maximum = max(row["outside_m2"] for _, row in assessed)
        require(math.isclose(metric["maximum_outside_m2"], maximum, abs_tol=1e-12), "outside-area maximum mismatch")
        result[name] = dict(status=metric["status"], assessed_count=count, violation_count=len(bad),
                            first_violation_frame=bad[0] if bad else None, maximum_outside_m2=maximum)
    return result


def load_trial(trial, expected_plan, pins):
    folder = BATCH / ("trial_%02d" % trial)
    episode = folder / "episode.partial"
    names = ["declaration.json", "owner_result.json", "diagnostic_report.json", "derived_body/summary.json",
             "derived_body/all900_native_boxes_and899_sweeps.jsonl.gz"]
    names += ["episode.partial/"+n for n in ("manifest.json", "route.json", "states.jsonl", "camera_frames.jsonl",
              "learned_control.jsonl", "control_receipts.jsonl", "wall_timing.jsonl", "active_map_opendrive.xodr")]
    pins.update({str(folder/n): digest(folder/n) for n in names})
    declaration, owner, diagnostic, body = [read_json(folder/n) for n in names[:4]]
    declaration_sha = digest(folder/"declaration.json")
    require(declaration_sha == expected_plan["declaration_sha256"] == owner["declaration_sha256"], "prospective trial identity differs")
    require(declaration["trial_index"] == trial and declaration["total_capture_seconds"] == 45
            and declaration["precursor_motion_screen"]["net_learned_progress_minimum_m"] == 90
            and declaration["controller_target_cap_mps"] == 2.4 and declaration["policy_deadline_seconds"] == .1,
            "predeclared trial bounds changed")
    for name, sha in declaration["source_sha256"].items():
        require(Path(name).is_relative_to(".") and ".." not in Path(name).parts, "unsafe local source pin")
        pins[str(ROOT/name)] = sha
        pins[str(folder/"source"/name)] = sha
    for section in ("external_source_sha256", "asset_sha256"):
        for name, sha in declaration[section].items():
            source = Path(name)
            if source.is_relative_to(ROOT):
                archive = folder/"source"/source.relative_to(ROOT)
            else:
                require(source.is_relative_to(CARLA_ROOT), "unknown external source root")
                archive = folder/"source_external"/source.relative_to(CARLA_ROOT)
            pins[str(source)] = pins[str(archive)] = sha
    pins.update(body["input_sha256"])
    assert_pins(pins)
    states, cameras, commands, receipts, timing = [rows(episode/n) for n in
        ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl", "control_receipts.jsonl", "wall_timing.jsonl")]
    geometry = rows(folder/names[4])
    require(len(states) == len(geometry) == 900 and len(cameras) == 450 and len(receipts) == 904,
            "all900 native/450 six-camera/904ACK rows required")
    frames = [r["frame"] for r in states]
    require(frames == list(range(frames[0], frames[0]+900))
            and [r["frame"] for r in geometry] == frames
            and [r["frame"] for r in cameras] == frames[::2], "complete native/camera/body chain differs")
    by_frame = {r["frame"]: r for r in states}
    paths = []
    for camera in cameras:
        require(set(camera["images"]) == set(CAMERAS) == set(camera["source_timestamps"])
                and abs(camera["timestamp"]-by_frame[camera["frame"]]["timestamp"]) < 1e-6
                and all(abs(t-camera["timestamp"]) < 1e-6 for t in camera["source_timestamps"].values()), "camera timestamp ABI changed")
        for name in camera["images"].values():
            relative = Path(name)
            require(not relative.is_absolute() and ".." not in relative.parts and relative.parts[0] == "images", "unsafe actual camera path")
            path = episode/relative
            require(path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(episode.resolve()), "missing actual camera asset")
            paths.append(str(path))
    require(len(paths) == len(set(paths)) == 2700, "all six distinct actual cameras per bundle required")
    require([r["sequence"] for r in receipts] == list(range(1,905)) and all(r["status"] == "ACKNOWLEDGED"
            and r["server_accepted"] is True for r in receipts), "full actual ACK chain differs")
    for state in states:
        prior_ack(state, receipts)
    applied = {r["ack_receipt_sequence"] for r in commands if r.get("application", {}).get("learned_control_applied") is True}
    first_effect = next(i for i, s in enumerate(states) if s["control_transport"]["expected_receipt_sequence"] in applied)
    post, post_geometry = states[first_effect:], geometry[first_effect:]
    require(len(post) == diagnostic["post_handoff_native_effect_interval_count"] == 865, "post-handoff tail omitted")
    progress = post_geometry[-1]["independent_actor_arc_m"]-post_geometry[0]["independent_actor_arc_m"]
    cte = [abs(r["independent_actor_route_distance_m"]) for r in post_geometry]
    for value, key in ((progress, "learned_effect_progress_m"), (percentile(cte,.95), "learned_effect_CTE_p95_m"),
                       (max(cte), "learned_effect_CTE_absolute_maximum_m")):
        require(math.isclose(value, diagnostic[key], abs_tol=1e-9), "independent whole-interval motion differs: "+key)
    emergencies = [r for r in commands if r["control_source"] == "emergency_brake"]
    require(emergencies, "actual failure-tail denominator missing")
    first = emergencies[0]
    require(all(r["control_source"] == "emergency_brake" for r in commands[commands.index(first):]), "latched failure unexpectedly resumed")
    failure_state = by_frame[first["command_frame"]]
    first_observed_emergency = next(s for s in states if s["control_transport"]["expected_receipt_sequence"] == first["ack_receipt_sequence"])
    decomposed = audit_timing(commands, timing, receipts, states, cameras, declaration_sha)
    census = audit_body(geometry, body["body_geometry"])
    require(owner["status"] == "CAPTURE_RETURNED_NONZERO_RETAINED" and owner["cleanup_errors"] == []
            and owner["source_hashes_match"] is True and owner["external_source_hashes_match"] is True,
            "original failed capture/source/ownership evidence differs")
    for item in (declaration, owner, diagnostic, read_json(episode/"manifest.json")):
        require(item["vehicle_control_approved"] is False and item.get("model_promoted", False) is False,
                "unapproved original scope changed")
    require(all(d["policy_100ms_passed"] for d in decomposed) == diagnostic["predeclared_fresh_policy100ms_screen_passed"], "policy result mismatch")
    wall_pass = all(d["observed_delivery_to_ACK_100ms_passed"] for d in decomposed)
    require(wall_pass == diagnostic["predeclared_observed_bundle_to_ACK_wall_latency_screen_passed"], "separate wall result mismatch")
    return dict(trial_index=trial, original_owner_status=owner["status"], original_manifest_failed=True,
                native_state_count=900, camera_bundle_count=450, actual_JPEG_count=2700, actual_ACK_count=904,
                post_handoff_native_count=len(post), outgoing_source_counts=dict(Counter(r["control_source"] for r in commands)),
                native_actor_progress_m=progress, post_handoff_actor_CTE_p95_m=percentile(cte,.95),
                post_handoff_actor_CTE_maximum_m=max(cte), longest_stationary_dwell_s=diagnostic["learned_effect_maximum_stationary_dwell_s"],
                post_handoff_base_norm_maximum_mps=max(math.hypot(s["vx"],s["vy"]) for s in post),
                post_handoff_actor_world_XY_norm_maximum_mps=max(math.hypot(*s["world_velocity_carla"][:2]) for s in post),
                first_latched_failure=dict(reason=first["emergency_reason"], outgoing_command_frame=first["command_frame"],
                    ACK_sequence=first["ack_receipt_sequence"], first_observed_emergency_frame=first_observed_emergency["frame"],
                    rear_base_CTE_m=failure_state["route_cte_m"], actual_unclipped_base_norm_mps=math.hypot(failure_state["vx"],failure_state["vy"]),
                    policy_total_seconds=first.get("total_seconds"), raw_NN_seconds=first.get("raw_nn_seconds"),
                    native_guard_before_model=True, all_emergency_tail_command_count=len(emergencies)),
                archived_body_census=census, original_driving_screen_passed=diagnostic["learned_driving_screen_passed"],
                original_body_screen_passed=body["body_and_swept_clearance_passed"],
                fresh_policy100ms_passed=all(d["policy_100ms_passed"] for d in decomposed),
                fresh_observed_delivery_to_ACK100ms_passed=wall_pass,
                fresh_timing_denominator=len(decomposed), delivery_to_ACK_ms=stats([d["delivery_to_ACK_ms"] for d in decomposed]),
                component_ms={k:stats([d["components_ms"][k] for d in decomposed]) for k in decomposed[0]["components_ms"]},
                all_fresh_timing_decomposition=decomposed,
                above100ms_fresh_rows=[d for d in decomposed if not d["observed_delivery_to_ACK_100ms_passed"]])


def build_report():
    batch_path = BATCH.with_name(BATCH.name+"_plan.json")
    require(digest(batch_path) == BATCH_SHA, "fixed prospective all-three declaration changed")
    batch = read_json(batch_path)
    require(batch["prospective_trial_count"] == 3 and batch["attempts_per_trial"] == 1
            and batch["automatic_retry"] is False and batch["outcome_selected_retry"] is False
            and batch["net_progress_minimum_m"] == 90 and batch["capture_seconds_per_trial"] == 45,
            "whole prospective repetition contract changed")
    pins = {str(batch_path):BATCH_SHA, str(Path(__file__)):digest(__file__)}
    for item in batch["trials"]:
        pins[item["declaration_path"]] = item["declaration_sha256"]
    trials = [load_trial(i, batch["trials"][i], pins) for i in range(3)]
    assert_pins(pins)
    fresh = [row for trial in trials for row in trial["all_fresh_timing_decomposition"]]
    return dict(schema=SCHEMA, status="FAIL_DRIVING_ALL_THREE_ATTEMPTS_RETAINED", prospective_trials=3,
                driving_pass_count=sum(t["original_driving_screen_passed"] for t in trials),
                body_pass_count=sum(t["original_body_screen_passed"] for t in trials),
                observed_delivery_to_ACK100ms_trial_pass_count=sum(t["fresh_observed_delivery_to_ACK100ms_passed"] for t in trials),
                joint_motion_body_policy_wall_trial_pass_count=sum(t["original_driving_screen_passed"] and t["fresh_observed_delivery_to_ACK100ms_passed"] for t in trials),
                all_fresh_timing_denominator=len(fresh), above100ms_fresh_count=sum(not r["observed_delivery_to_ACK_100ms_passed"] for r in fresh),
                all_actual_native_states=2700, all_actual_camera_bundles=1350, actual_JPEG_assets=8100,
                JPEG_pixels_decoded_or_hashed_by_this_audit=False, all_failed_tail_retained=True,
                source_sha256=pins, trials=trials, official_function_approved_count=0, official_subfunction_approved_count=0,
                official_function_total=9, official_subfunction_total=30, training_data_approved=False,
                vehicle_control_approved=False, model_promoted=False, full_RTE01_approved=False,
                timing_interpretation="policy/input+NN dominate measured intervals; ACK send sub-ms in actual slow rows; preproposal gap uninstrumented here, no physics or callback attribution",
                sensor_callback_entry_unmeasured=True, exposure_to_ACK_wall_latency_proven=False,
                ten_hz_end_to_end_proven=False, full_tick_wall_rate_proven=False,
                next_priority="real signed lateral/yaw teacher recovery collection, training and untouched held-out evaluation; no threshold relaxation")


def markdown(report):
    lines = ["# Town07: TemporalV2 실제 45초 × 3회 — 주행 실패 원인 확인", "",
             "HH_261001 - Retain every declared trial and every failure-tail observation.", "",
             "결론: 세 번 모두 약 21 m에서 옆으로 밀려 차선 경계 검사에 실패했습니다. **주행 0/3, 차체·10 cm 여유 검사 0/3**입니다. 속도 초과나 모델 추론 마감시간이 첫 정지 원인은 아닙니다.", "",
             "| 회차 | 실제 전진 | CTE 95% / 최대 | 첫 거부 CTE | 수신→ACK 최대 | 100 ms 초과 / fresh |", "|---|---:|---:|---:|---:|---:|"]
    for trial in report["trials"]:
        lines.append("| %d | %.3f m | %.3f / %.3f m | %.3f m | %.3f ms | %d / %d |" %
                     (trial["trial_index"]+1, trial["native_actor_progress_m"], trial["post_handoff_actor_CTE_p95_m"],
                      trial["post_handoff_actor_CTE_maximum_m"], trial["first_latched_failure"]["rear_base_CTE_m"],
                      trial["delivery_to_ACK_ms"]["maximum"], len(trial["above100ms_fresh_rows"]), trial["fresh_timing_denominator"]))
    lines += ["", "세 회차의 native 2,700개, 6카메라 묶음 1,350개(실제 JPEG 8,100개), 실패 후 비상제동 구간도 전부 남겼습니다. 충돌·lane-invasion 이벤트가 없더라도 차체 검사 실패를 지우지 않습니다.", "",
              "수신→ACK는 fresh 추론 284회 중 6회가 100 ms를 넘었습니다. 해당 6회에서 입력 구축·해시 약 25–26 ms, NN 파이프라인 약 61–65 ms가 대부분이고, 모델 밖 관측→proposal 준비 약 8 ms도 포함됩니다. ACK 송신 자체는 0.35–0.74 ms였습니다. 준비 구간은 이 원자료에서 세부 계측하지 않았으므로 JPEG/물리/네트워크 각 원인으로 단정하지 않습니다. 센서 노출→ACK와 완전한 10 Hz 벽시계 성능도 아직 증명하지 않았습니다.", "",
              "다음 작업은 실제로 차를 좌우·yaw 방향으로 벗어나게 둔 teacher 회복 주행을 수집하고, 그 데이터로 주행 모델을 보강한 뒤 미사용 회복 조건에서 재검증하는 것입니다. 모델/차체 안전 기준을 완화하지 않습니다.", "",
              "공식 기능 승인: **9개 중 0개 / 하위 30개 중 0개**. 연구용 예비 시험 결과이며 실차 승인이나 학습 데이터 승인이 아닙니다.", "",
              "[전체 배치·지연 구간 분해 JSON](batch_outcome.json) · [원자료·소스 SHA256](source_evidence.json)", ""]
    return "\n".join(lines)


def publish(output=OUTPUT):
    output = Path(output)
    require(not output.exists() and not output.is_symlink(), "fresh create-only audit output required")
    report = build_report()
    assert_pins(report["source_sha256"])
    output.mkdir(parents=True, exist_ok=False)
    for name, value in (("batch_outcome.json",report), ("source_evidence.json",report["source_sha256"])):
        with (output/name).open("x") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    with (output/"README.md").open("x") as stream:
        stream.write(markdown(report))
    assert_pins(report["source_sha256"])
    return dict(output=str(output), result_sha256=digest(output/"batch_outcome.json"),
                driving_pass_count=report["driving_pass_count"], fresh_count=report["all_fresh_timing_denominator"],
                above100ms_count=report["above100ms_fresh_count"], simulator_or_GPU_used=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=OUTPUT)
    parser.add_argument("--validate-only",action="store_true")
    args=parser.parse_args()
    if args.validate_only:
        result=build_report()
        print(json.dumps(dict(validated=True, fresh_count=result["all_fresh_timing_denominator"], above100ms_count=result["above100ms_fresh_count"])))
    else:
        print(json.dumps(publish(args.output)))


if __name__ == "__main__":
    main()
