"""HH_261004 - Reproduce a create-only CPU diagnosis of frozen v2 without regrading its FAIL or inventing absent native clocks."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from scripts.e2e import collect_temporal_route_joint_town03_straight_callback45_v3_20261004 as live
from scripts.e2e import research_temporal_route_joint_town03_straight_callback45_v3_20261004 as owner


OUTPUT = owner.ROOT / "storage/experiments/diagnostics/2026-10-04/callback45_v2_exact_object_readonly_diagnosis_v3"
V3_SHA = {
    "tools/e2e/collect_temporal_route_joint_town03_straight_callback45_v3_20261004.py": "da3a9733a6670152d2dd814bcc4921c91d18a303063ff20b9fb29e89b5d1f163",
    "tools/e2e/research_temporal_route_joint_town03_straight_callback45_v3_20261004.py": "771eb289627ea8b060d6a4b6f832f79dc7c5f3a7e8c9f53e6623531d4b709169",
    "learning/tests/test_temporal_route_joint_town03_straight_callback45_v3_20261004.py": "3a13f6e15effc845add18f505736dfb45ddbcab891d8e3ed47cadd9becbe0625",
}
RAW_NAMES = ("camera_frames.jsonl", "learned_control.jsonl", "wall_timing.jsonl",
             "learned_summary.json", "recording_after_ack.jsonl", "cpu_thread_readback.jsonl")


def _hashes(paths):
    """HH_261004 - Check canonical regular inputs before and after reading; never modify the frozen evidence."""
    result = {}
    for path in paths:
        owner.require(path.resolve() == path and path.is_file(), "canonical regular audit input required")
        result[str(path.relative_to(owner.ROOT))] = owner.digest(path)
    return dict(sorted(result.items()))


def diagnosis():
    """HH_261004 - Adapt only an in-memory schema label; every wall latency still uses its exact consumed source object."""
    owner.require(all(owner.digest(owner.ROOT / name) == sha for name, sha in V3_SHA.items()),
                  "reviewed fresh v3 source differs")
    archived_plan = owner.read_json(owner.CALLBACK_V2_OUTPUT / "declaration.json")
    pins = dict(weights_sha256=archived_plan["new_fixed_endpoint_export_sha256"]["weights"],
                metadata_sha256=archived_plan["new_fixed_endpoint_export_sha256"]["metadata"],
                checkpoint_sha256=archived_plan["source_checkpoint_sha256"],
                declaration_sha256=archived_plan["actual_training_declaration_sha256"])
    # HH_261004 - The real source closure requires the sealed model pin scope; this context does not launch a worker or simulator.
    with owner.activated(pins):
        closure = owner.runtime_sources()
    episode = owner.CALLBACK_V2_OUTPUT / "episode.partial"
    paths = ([owner.ROOT / name for name in closure]
             + [owner.CALLBACK_V2_OUTPUT / name for name in owner.CALLBACK_V2_EVIDENCE_SHA]
             + [episode / name for name in RAW_NAMES] + [Path(__file__).resolve()])
    before = _hashes(paths)
    archived = owner._verify_callback_v2_failure()
    value = owner.read_callback_receipts(episode / "callback_timing_receipts.json",
        owner.CALLBACK_V2_EVIDENCE_SHA["episode.partial/callback_timing_receipts.json"],
        schema=live.SCHEMA.replace("callback45_v3", "callback45_v2") + ".callback_receipts")
    owner.require("native_stage_marks" not in value,
                  "this audit requires the actual v2 archive without later native capture marks")
    value["schema"] = live.SCHEMA + ".callback_receipts"
    cameras, commands, timing = (owner._jsonl(episode / name) for name in RAW_NAMES[:3])
    counts = owner.read_json(episode / "learned_summary.json")["counts"]
    result = live.audit_receipts(value, cameras, commands, timing, counts)
    owner.require((result["callback_count"], result["selected_callback_count"],
                   result["extra_unselected_callback_count"], len(result["rows"])) == (5405, 2700, 2705, 415),
                  "complete frozen v2 callback/fresh-plan denominator differs")
    by_ack = {row["ACK_sequence"]: row for row in timing}
    by_command = {row["ack_receipt_sequence"]: row for row in commands
                  if row.get("learned_control_proposed") is True and row.get("held_between_camera_frames") is False}
    stages = []
    for row in result["rows"]:
        ack = by_ack[row["ACK_sequence"]]
        bundle = ack["observed_bundle"]
        clocks = (row["first_callback_monotonic_ns"], row["last_callback_monotonic_ns"],
                  bundle["exact_bundle_delivery_monotonic_ns"], bundle["observed_frame_build_finished_monotonic_ns"],
                  ack["proposal_started_monotonic_ns"], row["input_ready_monotonic_ns"],
                  row["raw_nn_finished_monotonic_ns"], row["decoded_plan_ready_monotonic_ns"],
                  ack["proposal_finished_monotonic_ns"], ack["ACK_started_monotonic_ns"], ack["ACK_finished_monotonic_ns"])
        names = ("six_camera_arrival_seconds", "last_callback_to_bundle_delivery_seconds",
                 "bundle_delivery_to_owned_frame_ready_seconds", "owned_frame_ready_to_proposal_seconds",
                 "proposal_to_input_ready_seconds", "input_ready_to_raw_NN_seconds", "raw_NN_to_decoded_plan_seconds",
                 "decoded_plan_to_proposal_finished_seconds", "proposal_finished_to_ACK_started_seconds", "ACK_send_seconds")
        owner.require(all(a <= b for a, b in zip(clocks, clocks[1:])), "authenticated critical-path clocks reversed")
        segments = {name: (b - a) / 1.e9 for name, a, b in zip(names, clocks, clocks[1:])}
        owner.require(sum(b - a for a, b in zip(clocks, clocks[1:])) == clocks[-1] - clocks[0],
                      "complete untrimmed original source-to-ACK critical path required")
        stages.append(dict(inference_frame=row["inference_frame"], ACK_sequence=row["ACK_sequence"], **segments,
            original_policy_reported_seconds=by_command[row["ACK_sequence"]]["total_seconds"],
            first_callback_to_plan_seconds=row["callback_first_to_plan_seconds"],
            first_callback_to_ACK_seconds=row["callback_first_to_ACK_seconds"]))
    stage_metrics = {name: live._stats([row[name] for row in stages]) for name in stages[0] if name.endswith("_seconds")}
    recording = owner._jsonl(episode / "recording_after_ack.jsonl")
    owner.require(len(recording) == 450 and all(row["enqueued_camera_count"] == row["written_camera_count"] == 6
        and row["failure"] is None and row["all_queued_images_written"] is True
        and row["recording_started_monotonic_ns"] >= by_ack[row["ACK_receipt_sequence"]]["ACK_finished_monotonic_ns"]
        for row in recording), "all450 original post-ACK six-camera recording receipts required")
    fresh_ack = {row["ACK_sequence"] for row in result["rows"]}
    cpu = owner._jsonl(episode / "cpu_thread_readback.jsonl")
    owner.require(len(cpu) == 877 and all(row["intra_op_threads"] == 4 and row["inter_op_threads"] == 1 for row in cpu),
                  "original numerical thread readbacks differ")
    plan_misses = [row for row in result["rows"] if row["callback_first_to_plan_seconds"] > .1]
    ack_misses = [row for row in result["rows"] if row["callback_first_to_ACK_seconds"] > .1]
    owner.require(len(plan_misses) == 5 and len(ack_misses) == 7
        and result["fresh_callback_to_plan_wall100ms_passed"] is False
        and result["fresh_callback_to_ACK_wall100ms_passed"] is False,
        "original full-source100ms failed tails must remain failures")
    after = _hashes(paths)
    owner.require(before == after, "input/source changed during readonly CPU diagnosis")
    return dict(schema="portable_e2e.callback45_v2_exact_object_readonly_diagnosis.20261004.v3",
        provenance="HH_261004 - CPU-only derived analysis of unchanged genuine v2 callback evidence; not a new drive or approval.",
        created_UTC=datetime.now(timezone.utc).isoformat(), original_experiment=str(owner.CALLBACK_V2_OUTPUT),
        original_status=archived["status"], original_failure_preserved=True, original_experiment_regraded=False,
        in_memory_schema_adapter_only=True, actual_CARLA_or_GPU_job_executed=False,
        fresh_v3_source_sha256=V3_SHA, full_v3_runtime_source_closure_sha256=closure,
        callback_v2_expected_evidence_sha256=owner.CALLBACK_V2_EVIDENCE_SHA,
        callback_receipt_input_bytes=(episode / "callback_timing_receipts.json").stat().st_size,
        input_and_source_sha256_before=before, input_and_source_sha256_after=after,
        input_and_source_unchanged=True, original_callback_native_stage_marks=None,
        native_pre_delivery_closure_passed=None, native_markers_not_recorded_in_v2=True,
        exact_source_join=result, critical_path=dict(full415rows=stages, metrics=stage_metrics,
            worst_plan_row=max(stages, key=lambda row: row["first_callback_to_plan_seconds"])),
        all_plan_100ms_failed_rows=plan_misses, all_ACK_100ms_failed_rows=ack_misses,
        plan_100ms_failure_count=len(plan_misses), ACK_100ms_failure_count=len(ack_misses),
        post_ACK_recording=dict(all450_six_camera_flush_seconds=live._stats([row["recording_seconds"] for row in recording]),
            fresh415_six_camera_flush_seconds=live._stats([row["recording_seconds"] for row in recording
                if row["ACK_receipt_sequence"] in fresh_ack]), outside_callback_to_current_plan_or_ACK=True,
            occurs_before_next_synchronous_tick=True, disk_vs_encode_vs_scheduling_separately_measured=False),
        callback_post_put_SHA_elapsed_seconds=live._stats([(row["source_SHA_finished_monotonic_ns"]
            - row["queue_put_finished_monotonic_ns"]) / 1.e9 for row in value["callbacks"]]),
        original_CPU_thread_readbacks=dict(count=len(cpu), all_intra4_inter1=True,
            original_external_CPU_load_recorded=False, historical_interference_absent_proven=False),
        v3_CPU_test_history=[dict(session=51868, passed=95, failed=1, duration_seconds=30.18,
            cause="camera-only native aggregate field was incorrectly requested from non-camera ticks; fixed only in unfrozen v3"),
            dict(session=86397, passed=96, failed=0, duration_seconds=30.29),
            dict(session=92841, passed=97, failed=0, duration_seconds=30.14)],
        derived_audit_CPU_failure_history=[dict(exit_code=1, duration_seconds=.529,
            cause="Initial check-only lacked the real activated GOOD6000 source pin scope; no output was created")],
        deployed_runtime_optimization=False, hardware_exposure_to_ACK_proven=False, ten_hz_wall_rate_proven=False,
        training_data_approved=False, full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
        model_promoted=False, vehicle_control_approved=False)


def _readme(report):
    """HH_261004 - Explain source-identity success separately from the still-failing100ms wall latency."""
    result, metrics = report["exact_source_join"], report["critical_path"]["metrics"]
    table = "\n".join(f"| {name} | {value['count']} | {value['p95'] * 1000:.3f} | {value['maximum'] * 1000:.3f} |"
                      for name, value in metrics.items())
    return f"""<!-- HH_261004 - Preserve failed full-source timing and all415 actual source joins in a separate CPU-only diagnostic. -->
# 실제 v2 입력 연결·지연 재분석 (원래 FAIL 유지)

차량을 다시 주행시킨 결과가 아닙니다. 동결된 v2 실험을 로컬 CPU로 읽기만 했습니다.
정확한 원본 영상 연결은 성공했지만, 첫 카메라 수신부터 계산·명령 적용까지의 100ms 기준은 아직 미달입니다.
정식 기능 승인을 추가하지 않으며, 학습 승인·실차 적용·모델 승격도 하지 않았습니다.

## 원본을 정확히 연결했는가?

수신 5,405개 중 실제 사용 2,700개(450묶음 × 6카메라), 미사용 2,705개를 모두 보존했습니다.
역할·원래 프레임·실제 영상 객체ID로 먼저 연결한 뒤 시뮬레이션 시각·소유 센서·BGRA SHA를 확인했습니다.
같은 프레임이 반복 수신된 2,231개 그룹은 모두 서로 다른 영상 객체와 픽셀이었습니다.
반복 프레임에서 빠른 시각이나 늦은 시각을 유리하게 골라 쓰지 않았습니다.
과거 5묶음의 실제 신경망 입력까지 12,450회 SHA 검증했습니다.

## 남은 지연 문제

실제 신경망 계산 415회를 모두 포함합니다. 첫 카메라 수신→계획 최대 {result['metrics']['callback_first_to_plan_seconds']['maximum'] * 1000:.3f}ms,
첫 카메라 수신→수락된 제어 ACK 최대 {result['metrics']['callback_first_to_ACK_seconds']['maximum'] * 1000:.3f}ms입니다.
100ms 초과는 계획 5회, ACK 7회이며 실패 구간을 삭제하지 않았습니다.
관측 계획 생성률 {result['observed_decoded_plan_wall_rate_hz']:.3f}Hz는 참고 계측이지 10Hz 보장이나 하드웨어 노출 지연 증명이 아닙니다.

| 실제 단계 (seconds 필드, 수치는 ms) | 전체 표본 수 | p95(ms) | 최대(ms) |
| --- | ---: | ---: | ---: |
{table}

최악 프레임에서 가장 큰 개별 구간은 마지막 카메라 수신→기존 영상 묶음 전달 47.888ms였습니다.
v2에는 물리 tick·상태 처리·위험 확인 내부 경계 시계가 없으므로 해당 native 단계 결과는 JSON에 명시적으로 null입니다.
후속 v3는 그 경계를 원래 순서 그대로 계측하도록 준비했으며, 여기서는 존재하지 않는 기록을 만들지 않았습니다.
JPEG 저장은 현재 ACK 후이며 다음 동기 tick 전입니다. 저장 지연은 측정했지만 디스크·인코딩·스케줄링 원인을 분리 입증한 것은 아닙니다.
CPU 4/1 설정 877회는 확인했으나 당시 다른 프로세스 간섭이 없었다고 입증할 자료는 없습니다.

## 자료와 재현

- [전체 415회 연결·단계·실패 꼬리·전후 SHA](report.json)
- [자료별 SHA](artifact_manifest.json)

저장 결과를 덮어쓰지 않고 읽기 재계산하는 명령입니다.

```bash
cd /home/a/autoware_e2e
CUDA_VISIBLE_DEVICES='' OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 python3 -B tools/workspace_python.py -m scripts.e2e.audit_callback45_v2_exact_object_readonly_20261004 -- --check-only
```

원래 `callback45_v2`의 FAIL 결과와 원본 파일은 변경되지 않았습니다.
"""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-only", action="store_true", help="Recompute without creating or changing any output")
    args = parser.parse_args(argv)
    report = diagnosis()
    if not args.check_only:
        # HH_261004 - Keep the derived destination create-only and refuse a legacy alias or existing experiment.
        owner.require(OUTPUT.resolve() == OUTPUT and not OUTPUT.exists(), "fresh canonical diagnostic destination required")
        OUTPUT.mkdir(parents=True, exist_ok=False)
        for name, content in (("report.json", json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"),
                              ("README.md", _readme(report))):
            with (OUTPUT / name).open("x", encoding="utf-8") as stream:
                stream.write(content)
        manifest = dict(schema=report["schema"] + ".artifact_manifest", create_only=True,
            original_experiment_regraded=False,
            artifact_sha256={name: owner.digest(OUTPUT / name) for name in ("report.json", "README.md")},
            reproducer_sha256=owner.digest(Path(__file__).resolve()))
        with (OUTPUT / "artifact_manifest.json").open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps(dict(output=None if args.check_only else str(OUTPUT),
        original_status=report["original_status"], sources_unchanged=report["input_and_source_unchanged"],
        callback_count=5405, selected_callback_count=2700, extra_callback_count=2705,
        fresh_plan_count=415, plan100ms_misses=5, ACK100ms_misses=7,
        native_capture_marks=None, formal_function_approval=False), sort_keys=True))


if __name__ == "__main__":
    main()
