#!/usr/bin/env python3
"""HH_260906 - Publish all eight reviewed expert trials, never selecting a winning launch profile or admitting training data."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

from scripts.e2e import audit_carla_turn_launch_matrix as audit
from scripts.e2e import render_carla_raw_trial as visual

ROOT = Path(__file__).resolve().parents[2]
AUDIT_SHA = "19a2511ce32bc06ca5cc5cd027d5ed0c0fcb6d3aed01195c8d0182250857f1ca"
AUDITOR_SHA = "2f1013cdaa8b2d94329d2dce6ca5a903c772dcd5d4145e35b88c2d3422859eab"
require = audit.base.require
sha = audit.base.sha
encode = lambda value: (json.dumps(audit.v4.sanitize(value), indent=2, allow_nan=False) + "\n").encode()


def safe_file(root, relative):
    relative = Path(relative)
    require(not relative.is_absolute() and ".." not in relative.parts, "unsafe source path")
    path = root / relative
    require(path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(root.resolve()), "missing or escaping source file")
    return path


def verify_entries(root, entries):
    require(len({e["path"] for e in entries}) == len(entries), "duplicate source ledger path")
    for entry in entries:
        path = safe_file(root, entry["path"])
        require(path.stat().st_size == entry["size_bytes"] and sha(path) == entry["sha256"], "source/image ledger differs")


def source_pins(report):
    pins = dict(report["audit_source_sha256"])
    require(pins.get("scripts/e2e/audit_carla_turn_launch_matrix.py") == AUDITOR_SHA, "unreviewed matrix auditor")
    for module in (visual, visual.view, audit.v4):
        path = Path(module.__file__).resolve()
        name, current = str(path.relative_to(ROOT)), sha(path)
        require(name not in pins or pins[name] == current, "reviewed overlapping source changed")
        pins[name] = current
    pins[str(Path(__file__).resolve().relative_to(ROOT))] = sha(Path(__file__))
    return pins


def verify_sources(pins):
    for name, expected in pins.items():
        require(sha(safe_file(ROOT, name)) == expected, "publication/audit source changed")


def bind_inputs(campaign, audit_root):
    # HH_260906 - The reviewed report is pinned byte-for-byte; all underlying metadata and JPEG bytes are checked again.
    path = safe_file(audit_root, "audit.json")
    require(sha(path) == AUDIT_SHA, "reviewed all-eight audit SHA differs")
    report = audit.base._loads(path.read_bytes())
    require(report["schema"] == "portable_e2e.turn_launch_matrix_audit.v1" and report["status"] == "AUDITED_NOT_ADMITTED"
        and report["planned_cases"] == report["discovered_cases"] == report["finalized_cases"] == 8
        and report["not_run_cases"] == 0 and report["all_planned_cases_retained"] is True, "all eight final cases required")
    require(report["reviewed_execution_commit"] == audit.COMMIT and report["prospective_plan_sha256"] == audit.ORIGINAL_PLAN_SHA
        and len(report["continuation_reviews"]) == 7 and report["resume_authorization"] is not None, "prospective or continuation binding missing")
    for key in ("training_data_approved", "dataset_admission", "automatic_winner_selection", "full_future_xy_admission",
                "physical_actuation_proven", "repetitions_are_independent_routes"):
        require(report[key] is False, "research scope was promoted")
    pins = source_pins(report); verify_sources(pins)
    verify_entries(campaign, report["source_manifest"])
    verify_entries(ROOT, report["historical_source_manifest"])
    contexts = []
    require(len(report["cases"]) == 8, "case denominator differs")
    for case, expected in zip(report["cases"], audit.expected_cases()):
        require(all(case[key] == value for key, value in expected.items()) and case["status"] == "FINALIZED", "fixed case order or identity differs")
        trial = case["audit"]; root = campaign / case["output"]
        name = f"{case['profile']}_run_{case['replicate']:03d}_image_hashes.json"
        ledger_path = safe_file(audit_root, name)
        images = audit.base._loads(ledger_path.read_bytes())
        image_proof = trial["image_byte_integrity"]
        require(image_proof["all_images_decoded"] is True and image_proof["content_or_visual_quality_approved"] is False
            and len(images) == image_proof["file_count"] and sum(e["size_bytes"] for e in images) == image_proof["total_size_bytes"]
            and hashlib.sha256(json.dumps(images, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
                == image_proof["canonical_sorted_file_ledger_sha256"], "decoded image ledger binding differs")
        verify_entries(root, trial["source_manifest"]); verify_entries(root, images)
        fresh, timeline = audit.base.summarize_trial(root, None)
        require(timeline is not None and audit.v4.sanitize(fresh["independent_qa"]) == trial["independent_qa"], "fresh scalar recomputation differs")
        require(audit.v4.sanitize(audit.handoff_diagnostic(trial, timeline)) == trial["handoff_diagnostic"], "fresh handoff diagnostic differs")
        require(case["independent_scalar_quality_clear"] is trial["independent_qa"]["raw_scalar_quality_clear"], "scalar case status differs")
        contexts.append({"case": case, "root": root, "images": images, "ledger_name": name,
            "ledger_sha256": sha(ledger_path), "timeline": timeline})
    return report, contexts, pins


def recheck(campaign, audit_root, report, contexts, pins):
    require(sha(safe_file(audit_root, "audit.json")) == AUDIT_SHA, "audit changed during publication")
    verify_sources(pins); verify_entries(campaign, report["source_manifest"])
    verify_entries(ROOT, report["historical_source_manifest"])
    for item in contexts:
        require(sha(safe_file(audit_root, item["ledger_name"])) == item["ledger_sha256"], "image ledger changed")
        verify_entries(item["root"], item["case"]["audit"]["source_manifest"])
        verify_entries(item["root"], item["images"])


def select_views(data):
    # HH_260906 - Selection is outcome-independent: first driving camera, catalog LEFT midpoint, and first measured goal-complete camera.
    cameras, states = data["cameras"], data["states"]
    rows = [states[data["frames"][c["frame"]]] for c in cameras]
    driving = [i for i, row in enumerate(rows) if row["capture_phase"] == "driving"]
    require(driving, "no driving camera for declared complete trial")
    first = driving[0]
    goals = [i for i, row in enumerate(rows) if row.get("goal_stop", {}).get("complete") is True]
    left = [(i, point) for i, point in enumerate(data["route"]["route"]) if point["road_option"] == "LEFT"]
    require(left and [i for i, _ in left] == list(range(left[0][0], left[-1][0] + 1)), "catalog must contain one contiguous LEFT segment")
    midpoint = (left[0][1]["distance_m"] + left[-1][1]["distance_m"]) / 2
    reached = min(r["route_progress_m"] for r in rows) <= midpoint <= max(r["route_progress_m"] for r in rows)
    turn = min(range(len(rows)), key=lambda i: abs(rows[i]["route_progress_m"] - midpoint)) if reached else None
    start_time = rows[first]["timestamp"]
    window = [i for i in range(first, len(rows), 2) if 0 <= rows[i]["timestamp"] - start_time <= 8.000001]
    require(window, "empty launch preview")
    return {"snapshots": {"01_first_driving": first, "02_catalog_left_midpoint": turn, "03_first_goal_complete": goals[0] if goals else None},
        "catalog_left_midpoint_m": midpoint, "catalog_left_reached": reached, "goal_complete_camera_available": bool(goals),
        "launch_camera_indices": window, "launch_start_timestamp": start_time, "launch_window_seconds": 8.0,
        "camera_stride": 2, "playback_fps": 10, "nominal_preview_speedup": 2.0}


def render_case(item, destination):
    data = visual.load_trial(item["root"])
    selection = select_views(data); destination.mkdir()
    records = {}
    for name, index in selection["snapshots"].items():
        if index is None:
            records[name] = {"status": "NOT_REACHED_OR_NOT_RECORDED", "image_created": False}
            continue
        visual.render_frame(data, index, 2, 10).save(destination / f"{name}.png")
        row = data["states"][data["frames"][data["cameras"][index]["frame"]]]
        records[name] = {"status": "ACTUAL_RECORDED_CAMERA", "camera_index": index, "frame": row["frame"],
            "timestamp": row["timestamp"], "capture_phase": row["capture_phase"], "route_progress_m": row["route_progress_m"],
            "png_sha256": sha(destination / f"{name}.png")}
    with tempfile.TemporaryDirectory(prefix="hh260906_launch_preview_") as temporary:
        work = Path(temporary)
        for index, camera_index in enumerate(selection["launch_camera_indices"]):
            canvas = visual.render_frame(data, camera_index, 2, 10)
            draw = visual.ImageDraw.Draw(canvas)
            draw.rectangle((0, 872, 1600, 900), fill="#18344c")
            draw.text((20, 876), "LAUNCH WINDOW ONLY: first driving camera + 0-8 sim seconds | 2x preview | NOT a full-route movie",
                fill="white", font=visual.view._font(17, True))
            canvas.save(work / f"{index:04d}.png")
        movie = destination / "04_launch_only_0-8simsec_2x.gif"
        subprocess.run(["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error", "-n", "-framerate", "10", "-i", str(work / "%04d.png"),
            "-filter_complex", "split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            "-loop", "0", str(movie)], check=True, timeout=180)
    with visual.Image.open(movie) as gif:
        require(gif.size == (1600, 900) and gif.n_frames == len(selection["launch_camera_indices"]), "launch GIF frame count or size differs")
        durations = []
        for index in range(gif.n_frames):
            gif.seek(index); durations.append(gif.info.get("duration"))
        require(all(value == 100 for value in durations), "launch GIF is not the declared 10 fps")
    verify_displayed_images(data, item)
    provenance = {"schema": "carla.turn_launch_matrix_visual.v1", "case_id": item["case"]["case_id"],
        "selection": selection, "snapshots": records, "displayed_image_sha256": data["displayed_image_sha256"],
        "source_metadata_sha256": data["source_metadata_sha256"], "episode_directory": data["episode"].name,
        "launch_gif_sha256": sha(movie), "gif_frame_count": len(durations), "gif_total_duration_ms": sum(durations),
        "camera_source": "Six unmodified JPEG sources, full-FOV letterbox; measured ego-centered route.",
        "future_policy": "Only available observed future positions, not predictions or padded training targets.",
        "learned_model_control": False, "autoware_closed_loop": False, "training_data_approved": False}
    (destination / "visual_provenance.json").write_bytes(encode(provenance))
    return provenance


def verify_displayed_images(data, item):
    # HH_260906 - The existing renderer resolves absolute paths even when the publication CLI receives relative roots.
    ledger = {e["path"]: e["sha256"] for e in item["images"]}
    root = item["root"].resolve()
    for name, digest in data["displayed_image_sha256"].items():
        relative = str((data["episode"] / name).resolve().relative_to(root))
        require(ledger.get(relative) == digest, "displayed camera differs from independently audited JPEG")


def render_plots(contexts, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for zoom, name in ((False, "01_whole_record_speed_acceleration.png"), (True, "02_launch_window_acceleration_throttle.png")):
        fig, axes = plt.subplots(2, 4, figsize=(19.2, 10.8), dpi=100, squeeze=False)
        for column, profile in enumerate(sorted(audit.PEDALS)):
            items = [i for i in contexts if i["case"]["profile"] == profile]
            for item in items:
                timeline = item["timeline"]; rows = timeline["native_states"]; intervals = timeline["native_intervals"]
                origin = next(r["elapsed_s"] for r in rows if r["phase"] == "driving") if zoom else 0.
                label = f"r{item['case']['replicate']} scalar {'PASS' if item['case']['independent_scalar_quality_clear'] else 'FAIL'}"
                style = "-" if item["case"]["replicate"] == 1 else "--"
                axes[0, column].plot([r["time_from_capture_start_s"] - origin for r in intervals] if zoom else [r["elapsed_s"] for r in rows],
                    [r["speed_rate_mps2"] for r in intervals] if zoom else [r["speed_mps"] * 3.6 for r in rows], style, label=label, linewidth=1.1)
                axes[1, column].plot([r["elapsed_s"] - origin for r in rows] if zoom else [r["time_from_capture_start_s"] for r in intervals],
                    [r["current_control"]["throttle"] for r in rows] if zoom else [r["speed_rate_mps2"] for r in intervals], style, label=label, linewidth=1.1)
            accel_axis = axes[0 if zoom else 1, column]
            for bound in (-2.9, 2.9): accel_axis.axhline(bound, color="#b32924", linestyle=":", linewidth=1)
            axes[0, column].set_title(f"Fixed launch throttle {audit.PEDALS[profile]:.2f}")
            for row in range(2):
                axis = axes[row, column]; axis.grid(alpha=.22); axis.legend(fontsize=9, loc="upper right")
                axis.set_xlabel("Seconds after first driving observation" if zoom else "Seconds from first captured state")
                axis.set_ylabel(("Native 20 Hz speed rate (m/s²)" if row == 0 else "Reported throttle") if zoom
                    else ("Measured speed (km/h)" if row == 0 else "Native 20 Hz speed rate (m/s²)"))
                if zoom: axis.set_xlim(0, 8)
        fig.suptitle("Launch window (0-8 s), same raw data; not the full route" if zoom else "All eight complete raw records: warmup + driving + stopped tail", fontsize=18)
        fig.text(.5, .018, "BasicAgent expert, NOT learned/Autoware control | 14.4 km/h target, NOT 30 km/h qualification | Same-route/seed repeats, no winner or admission", ha="center", fontsize=11)
        fig.tight_layout(rect=(.005, .042, .995, .95)); fig.savefig(output / name); plt.close(fig)


def summary_rows(contexts):
    rows = []
    for item in contexts:
        case = item["case"]; qa = case["audit"]["independent_qa"]
        native = qa["speed_rate_qa"]["native_20hz"]["by_phase"]["all"]
        camera = qa["speed_rate_qa"]["camera_10hz"]["by_phase"]["all"]
        rows.append({"case_id": case["case_id"], "profile": case["profile"], "replicate": case["replicate"],
            "scalar_status": "PASS" if case["independent_scalar_quality_clear"] else "FAIL", "launch_throttle": audit.PEDALS[case["profile"]],
            "native_states": sum(p["native_states"] for p in qa["phase_counts"].values()),
            "camera_anchors": sum(p["camera_anchors"] for p in qa["phase_counts"].values()), "jpeg_count": len(item["images"]),
            "native_min_mps2": native["minimum_mps2"], "native_max_mps2": native["maximum_mps2"],
            "camera_min_mps2": camera["minimum_mps2"], "camera_max_mps2": camera["maximum_mps2"],
            "camera_decoder_violation_count": camera["physical_decoder"]["violation_count"],
            "native_decoder_violation_count": native["physical_decoder"]["violation_count"],
            "goal_error_m": qa["final_driving"]["goal_error_m"], "goal_dwell_s": qa["goal_dwell_seconds"],
            "pilot_protocol_pass": case["audit"]["pilot_protocol"]["all_checks_pass"],
            "dataset_admission": False})
    return rows


def readme(summary):
    lines = ["# C-track 좌회전: 출발 페달 4종 × 같은 조건 2회", "",
        "CARLA의 **BasicAgent 전문가 주행 수집**입니다. 사람이 수동 운전한 영상도, 학습 모델 또는 Autoware 자율주행 폐루프 시험도 아닙니다.", "",
        "총 8개를 모두 남겼습니다. 스칼라 품질만 2/8 PASS, 6/8 FAIL이며 **학습 데이터 승인·자동 최적 설정 선택은 0건**입니다. 목표는 14.4 km/h로, 30 km/h 검증 결과가 아닙니다.", "",
        "설정: 같은 C-track 경로/시드, Epic, 물리 20 Hz, 카메라 10 Hz, 640×360 카메라 6개. 매번 새 소유 CARLA를 실행했지만 반복별 요약값이 같으며 다양한 경로나 무작위 실험 8개를 의미하지 않습니다.", "",
        "## 전체 결과", "", "| 순서 | 출발 throttle / 반복 | 스칼라 | 20 Hz native 최대 / 10 Hz camera 최대 (m/s²) | 목표 오차(m) | 화면 폴더 |",
        "|---|---|---|---|---|---|"]
    for i, row in enumerate(summary["cases"], 1):
        lines.append(f"| {i} | {row['launch_throttle']:.2f} / {row['replicate']} | {row['scalar_status']} | {row['native_max_mps2']:.6f} / {row['camera_max_mps2']:.6f} | {row['goal_error_m']:.6f} | [{row['case_id']}](./{row['case_id']}/) |")
    lines += ["", "모두 목표 도달·2초 정지 및 governor/ACK/초기화 검사 통과. 출발 페달 .13 두 회만 native 가속도 스칼라 검사 통과했습니다. .12/.14/.15의 실패는 더 이른 출발 구간에 기록되었고, 인계 주변 최대값은 모두 2.9 m/s² 아래입니다. 이후 throttle ramp 시작값은 모든 설정에서 .15로 동일하며, 이 인계가 실패 원인이라고 단정하지 않습니다.", "",
        "[전체 주행 속도·가속도](01_whole_record_speed_acceleration.png) · [출발 0–8초 계측](02_launch_window_acceleration_throttle.png) · [8개 독립 원자료 감사](audit.json)", "",
        "실제 카메라 10 Hz 간격의 속도 차분은 8회 모두 스칼라 기준 이내지만 native 20 Hz에서는 6회가 실패합니다. 관측 간격이 순간 변화의 크기를 다르게 보이게 할 수 있습니다. 화면이 부드러워 보여도 물리 검사 통과가 아니며, 이 결과만으로 모든 GUI 끊김의 원인을 카메라 Hz라고 결론내리지 않습니다.", "",
        "## 화면 보는 법", "",
        "각 폴더의 01은 첫 driving 카메라, 02는 원래 경로의 LEFT 구간 거리 중간점에 가장 가까운 실제 카메라, 03은 goal-complete가 실제 기록된 첫 카메라입니다. 차량은 지도 중앙, 6개 원본 카메라는 화각을 자르지 않은 letterbox입니다. 궤적은 관측된 과거/미래이지 모델 예측이 아닙니다. 정지 끝부분의 없는 미래를 채우지 않았습니다.", "",
        "04 GIF는 **첫 driving 카메라부터 0–8 시뮬레이션 초만** 담은 **2배속 출발 미리보기**입니다. 전체 경로 영상이나 실제 20 Hz 화면 재생 증명이 아닙니다. 원본 10 Hz 카메라를 stride 2로 골라 10 fps로 재생합니다. 전체 구간 수치·실패는 위 전체 그래프와 audit.json에 별도 보존했습니다.", "",
        f"전체 분모: native {summary['native_states']:,}개, 카메라 시점 {summary['camera_anchors']:,}개, JPEG {summary['jpeg_count']:,}개. 모든 JPEG의 SHA와 640×360 디코딩 검사를 독립 감사에서 확인했습니다. 디코딩은 장면/재질 품질 승인이나 전체 미래 XY 품질 승인이 아닙니다. 제어값은 reported/ACK 관측이며 물리 토크 적용 시점 증명이 아닙니다.", "",
        "## 출처와 재생성", "",
        "audit.json은 검토된 원본 SHA와 동일한 경로 비식별 감사입니다. 각 image_hashes.json은 원본 JPEG 전체 SHA 목록이며 원본 JPEG 자체는 배포하지 않습니다. publication_manifest.json에는 감사·렌더러 소스 SHA, 시점 선택과 출력 SHA가 들어 있습니다. 원본 실패, 이전 7개 진행 검토서의 당시 제한 사항, 별도 재개 승인을 덮어쓰지 않았습니다.", "",
        "git clone만으로 원본 수집 데이터를 얻거나 시험이 자동 실행되지는 않습니다. 아래 재생성에는 개인 보관 원본 8회, 연결된 이전 감사 입력, 해당 기록에 필요한 Git 소스 객체가 있어야 합니다. 설치·학습·CARLA 재실행 없이 읽기 전용으로 새 폴더에 렌더링하며, 기존 출력 경로는 거부합니다. 현재 Python 환경의 Pillow/Matplotlib와 ffmpeg가 필요합니다.", "", "```bash",
        "python3 -m scripts.e2e.curate_carla_turn_launch_matrix \\",
        "  --campaign-root artifacts/training/2026-09-08/turn_launch_matrix_v1 \\",
        "  --audit-root artifacts/training/2026-09-08/turn_launch_matrix_audit_v1 \\",
        "  --output-dir artifacts/training/2026-09-08/turn_launch_matrix_publication_replay_v1", "```", "",
        "정식 학습 편입·모델 개선·실차 운용 승인은 별도입니다. 이번 자료를 통과한 학습 데이터나 새 모델 성능으로 사용하지 않습니다.", ""]
    return "\n".join(lines).encode()


def publish(campaign, audit_root, output):
    output = audit.v4.new_output(output, [campaign, audit_root])
    report, contexts, pins = bind_inputs(campaign, audit_root)
    require(shutil.which("ffmpeg") is not None, "ffmpeg must already be installed")
    rows = summary_rows(contexts)
    summary = {"schema": "portable_e2e.turn_launch_matrix_publication.v1", "status": "ALL_EIGHT_RETAINED_NOT_ADMITTED",
        "raw_audit_sha256": AUDIT_SHA, "cases": rows, "scalar_pass_count": sum(r["scalar_status"] == "PASS" for r in rows),
        **{key: sum(r[key] for r in rows) for key in ("native_states", "camera_anchors", "jpeg_count")},
        "training_data_approved": False, "automatic_winner_selection": False, "learned_model_control": False}
    recheck(campaign, audit_root, report, contexts, pins)
    output.mkdir(parents=True, exist_ok=False)
    visuals = {}
    for item in contexts:
        case_id = item["case"]["case_id"]
        visuals[case_id] = render_case(item, output / case_id)
        (output / case_id / "image_hashes.json").write_bytes(safe_file(audit_root, item["ledger_name"]).read_bytes())
    render_plots(contexts, output)
    recheck(campaign, audit_root, report, contexts, pins)
    (output / "audit.json").write_bytes(safe_file(audit_root, "audit.json").read_bytes())
    (output / "summary.json").write_bytes(encode(summary))
    (output / "README.md").write_bytes(readme(summary))
    files = [{"path": str(p.relative_to(output)), "sha256": sha(p), "size_bytes": p.stat().st_size} for p in sorted(output.rglob("*")) if p.is_file()]
    require(all(b"/home/" not in p.read_bytes() for p in output.rglob("*") if p.suffix in (".json", ".md")), "private account path in public metadata")
    (output / "publication_manifest.json").write_bytes(encode({"schema": "carla.turn_launch_matrix_publication_manifest.v1",
        "raw_audit_sha256": AUDIT_SHA, "source_sha256": pins, "source_execution_status": "Current source bytes executed; source hashes retained, no invented commit claim.",
        "input_postcheck_pass": True, "images_independently_decoded_count": summary["jpeg_count"], "visuals": visuals, "files": files,
        "training_data_approved": False, "automatic_winner_selection": False}))
    (output / "SHA256SUMS").write_text("".join(f"{sha(p)}  {p.relative_to(output)}\n" for p in sorted(output.rglob("*")) if p.is_file()))
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--campaign-root", type=Path, required=True)
    parser.add_argument("--audit-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    result = publish(args.campaign_root, args.audit_root, args.output_dir)
    print(json.dumps({k: result[k] for k in ("status", "scalar_pass_count", "native_states", "camera_anchors", "jpeg_count")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
