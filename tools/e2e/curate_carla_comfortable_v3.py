#!/usr/bin/env python3
"""HH_260906 - Publish both v3 attempts with independently rebound metadata and unchanged diagnostic image bytes."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

from scripts.e2e import audit_carla_comfortable_v3_trial as audit
from scripts.e2e.curate_independent_common10_capture_20260907 import sanitize

PNG_NAMES = ("01_start", "02_measured_cruise", "03_coast_entry", "04_goal_dwell", "05_final_observation", "06_maximum_observed_deceleration")
NOTICE = "Metadata view: private machine paths are redacted; original SHA256 values remain recorded. Historical '*applied_control' keys mean reported controls, not independently proven physical application timing. PNG/GIF bytes are copied unchanged. This is not a standalone raw-data replay or data admission."


def sha(data):
    return hashlib.sha256(data).hexdigest()


def encode(value):
    return (json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode()


def bind_visuals(trial, visual_root, expected_trial):
    # HH_260906 - Recheck displayed source images as well as state metadata before copying derivative visuals.
    ledger = {}
    proof = audit.read_json(visual_root, "visual_provenance.json", ledger)
    audit.require(proof.get("schema") == "carla_expert.raw_trial_visual_diagnostic.v1", "visual schema mismatch")
    episode = proof.get("source_episode_name")
    audit.require(episode in ("episode", "episode.partial"), "unsafe source episode")
    audit.require(proof.get("original_owner_exit_code") == expected_trial["owner_exit_code"]
        and proof.get("training_data_approved") is False and proof.get("learned_model_control") is False
        and proof.get("live_autoware_screenshot") is False, "visual scope mismatch")
    audit.require(proof.get("source_renderer_sha256") == audit.base.sha(audit.REPOSITORY / "scripts/e2e/render_carla_raw_trial.py")
        and proof.get("source_layout_sha256") == audit.base.sha(audit.REPOSITORY / "scripts/e2e/render_carla_vad_expert.py"), "visual renderer revision mismatch")
    audit.require(set(proof["source_metadata_sha256"]) == {"manifest.json", "route.json", "states.jsonl", "camera_frames.jsonl"}, "visual metadata coverage mismatch")
    for name, expected in proof["source_metadata_sha256"].items():
        audit.require(sha(audit.checked_bytes(trial, f"{episode}/{name}", {})) == expected, "visual source metadata changed")
    cameras = audit.read_json(trial, f"{episode}/camera_frames.jsonl", {}, lines=True)
    count = len(cameras)
    audit.require(proof.get("camera_anchor_count") == count and proof.get("camera_stride") == 5
        and proof.get("playback_fps") == 10.0, "visual cadence differs from reviewed playback")
    indices = sorted(set([*range(0, count, 5), count - 1]))
    audit.require(proof.get("rendered_indices") == indices and set(proof["png_indices"]) == set(PNG_NAMES), "visual frame selection mismatch")
    selected = set(indices) | set(proof["png_indices"].values())
    audit.require(all(isinstance(i, int) and not isinstance(i, bool) and 0 <= i < count for i in selected), "visual index outside capture")
    expected_images = {relative for i in selected for relative in cameras[i]["images"].values()}
    audit.require(set(proof["displayed_image_sha256"]) == expected_images, "displayed source image coverage mismatch")
    for relative, expected in proof["displayed_image_sha256"].items():
        audit.require(sha(audit.checked_bytes(trial, f"{episode}/{relative}", {})) == expected, "displayed raw image changed")
    result = {}
    for name in [*(f"{name}.png" for name in PNG_NAMES), "whole_recording_accelerated.gif"]:
        payload = audit.checked_bytes(visual_root, name, ledger)
        audit.require(payload.startswith(b"\x89PNG\r\n\x1a\n") if name.endswith("png") else payload.startswith((b"GIF87a", b"GIF89a")), "visual file type mismatch")
        result[name] = payload
    result["visual_provenance.json"] = encode({**sanitize(proof), "publication_notice": NOTICE,
        "raw_source_sha256": ledger["visual_provenance.json"]["sha256"]})
    return result, ledger


def readme(report):
    lines = ["# Town07 직진 comfortable_v3: 두 번 모두 미승인", "",
        "<!-- HH_260906 - Preserve both same-condition attempts and separate scalar motion checks from control-record alignment. -->", "",
        "같은 소스·조건으로 선언한 2회 실행 전체입니다. **속도·정지 스칼라 검사: 1회 실패 / 1회 충족. 제어 기록 연속성: 2회 모두 실패. 학습 데이터 승인: 0회.**", "",
        "Town07 / Prius / ClearNoon / Low / 경로 210.598 m. 명령 순항은 28.8 km/h이고 실제 속도 상한은 30 km/h입니다. BasicAgent 전문가 수집이며 학습 모델 또는 Autoware 실시간 구동 화면이 아닙니다.", "",
        "| 실행 | 20 Hz 상태 / 10 Hz 카메라 앵커 | 최고 속도 km/h | 연속 순항 s | 최종 목표거리 m | native 최소 가속도 m/s² | camera 최소 가속도 m/s² | 스칼라 검사 | 직전 제어 요청 불일치 |",
        "|---|---:|---:|---:|---:|---:|---:|---|---:|"]
    for trial in report["trials"]:
        q, p = trial["independent_qa"], trial["pilot_protocol"]
        counts = q["phase_counts"]
        n, c = (sum(v[k] for v in counts.values()) for k in ("native_states", "camera_anchors"))
        rates = q["speed_rate_qa"]
        control = p["control_observation_alignment"]
        lines.append(f"| {trial['trial_id']} | {n} / {c} | {q['maximum_measured_speed_kmh']:.5f} | {p['longest_continuous_cruise_seconds']:.2f} | {q['final_driving']['goal_error_m']:.6f} | {rates['native_20hz']['by_phase']['all']['minimum_mps2']:.6f} | {rates['camera_10hz']['by_phase']['all']['minimum_mps2']:.6f} | {'충족' if trial['raw_quality_candidate'] else 'FAIL'} | {control['one_row_mismatch_count']} / {control['adjacent_transition_count']} |")
    lines.extend(["", "두 실행 모두 목표점 상류 1 m 안에서 0.1 m/s 이하로 2초 정지한 뒤 6.5초 tail을 기록했고 충돌·차선침범·비상개입은 0입니다. 연속 순항 조건은 7.8–8.2 m/s에서 5초 이상입니다. 1차 감속 위반은 native frame 3391→3392이며 decoder ±2.9 및 runtime +3/−6 m/s² 한계를 넘었습니다. 카메라 간격 감속도 decoder 한계를 넘었습니다. 한계나 라벨을 완화하지 않았습니다.", "",
        "## 제어 기록의 별도 실패", "",
        "1차 79개, 2차 85개 프레임에서 `current_control`이 직전 행의 `next_control`과 일치하지 않았습니다(페달·조향 절대오차 1e−6 기준). 모두 두 행 전 요청과는 일치하지만, 이것만으로 실제 액추에이터 지연인지 관측 캐시 지연인지 증명할 수 없습니다. 프레임을 재정렬하거나 적용 시점을 추정해 PASS로 바꾸지 않았습니다. 원래 JSON의 `*applied_control` 키와 화면의 Reported control은 물리 적용 시점 보증이 아닙니다.", "",
        "기록된 실행 소스 10개는 시작 SHA·보관 바이트·종료 SHA·기록 Git 커밋과 모두 일치합니다. 2차에는 실행하지 않는 분석 파일의 수정 상태가 있었으므로 전체 작업트리가 깨끗했다고 주장하지 않습니다.", "",
        "## 실제 카메라·경로 화면", "",
        "6개 카메라의 전체 화각을 여백으로 맞추고 지도 패널의 차량을 중앙에 고정했습니다. 경로·과거 위치·관측된 미래 위치를 함께 보여줍니다. GIF는 카메라 5프레임마다 하나를 10 fps로 재생하는 약 5배속이며, 끝 프레임 포함으로 정확한 재생시간은 provenance에 기록했습니다. 10 Hz 모델 추론 성능 증명이 아닙니다.", "",
        "화면에 보이는 체크무늬 도로 재질은 원래 Low 설정 카메라 영상에 있습니다. 보정·가림 처리하지 않았으며 Epic 설정을 포함한 시각 자산 품질 검사가 별도로 필요합니다. 전체 원본 JPEG 픽셀·전체 미래 XY 적합성 검사는 아직 완료되지 않았고, 표시한 원본 JPEG들의 SHA만 추가 검증했습니다.", ""])
    for name in ("run_001", "run_002"):
        lines.extend([f"### {name}", "", f"[전체 기록 GIF](visuals/{name}/whole_recording_accelerated.gif) · [시각화 provenance](visuals/{name}/visual_provenance.json)", "",
            " · ".join(f"[{label}](visuals/{name}/{file}.png)" for label, file in zip(("시작", "순항", "코스트 진입", "목표 정지", "마지막 관측", "최대 감속"), PNG_NAMES)), ""])
    lines.extend(["## 재검증 범위", "", "[독립 계측 보고서](summary.json) · [원본/공개본 SHA 연결](publication_manifest.json) · [전체 공개 파일 SHA256](SHA256SUMS)", "",
        "공개 JSON은 개인정보 경로를 치환한 메타데이터 뷰이며 원본 SHA를 보존합니다. PNG/GIF는 원본 렌더 바이트 그대로입니다. 원본 주행·실패 기록은 비공개 artifacts에 보존되며 이 폴더만으로 원본을 재실행할 수는 없습니다. 테스트셋·GPU 학습·모델 제어·자동 배포는 수행하지 않았습니다.", ""])
    return "\n".join(lines).encode()


def curate(pilot_root, report_path, visual_root, output, expected_sha):
    pilot_root, report_path, visual_root, output = map(Path, (pilot_root, report_path, visual_root, output))
    audit.require(not output.exists() and not output.is_symlink()
        and not any(output.resolve().is_relative_to(p.resolve()) for p in (pilot_root, report_path.parent, visual_root)), "new publication must be outside input trees")
    raw = audit.checked_bytes(report_path.parent, report_path.name, {})
    audit.require(sha(raw) == expected_sha, "independent audit SHA mismatch")
    report = audit.base._loads(raw.decode())
    audit.require(report.get("schema") == "portable_e2e.comfortable_v3_independent_audit.v1" and report["trial_count"] == 2
        and report["auditor_source_sha256"] == audit.base.sha(Path(audit.__file__))
        and report["base_auditor_source_sha256"] == audit.base.sha(Path(audit.base.__file__)), "independent auditor revision mismatch")
    audit.require(report["pilot_plan_source"]["sha256"] == audit.base.sha(pilot_root / "pilot_plan.json"), "pilot plan SHA mismatch")
    names = ["run_001", "run_002"]
    parent = pilot_root / "town07_straight_calibration"
    audit.require(sorted(p.name for p in parent.iterdir() if p.name.startswith("run_")) == names, "attempt denominator changed")
    recomputed = [audit.audit_trial(parent / name) for name in names]
    audit.require(report["trials"] == recomputed, "stale or modified supplied audit differs from actual source recomputation")
    audit.require(report["development_screen_clear_count"] == sum(t["development_screen_clear"] for t in recomputed)
        and report["development_screen_fail_count"] == sum(not t["development_screen_clear"] for t in recomputed)
        and report["base_raw_quality_candidate_count"] == sum(t["raw_quality_candidate"] for t in recomputed), "audit aggregate count mismatch")
    outputs = {"summary.json": encode({**sanitize(report), "publication_notice": NOTICE, "raw_source_sha256": sha(raw)}), "README.md": readme(report)}
    source_visuals = {}
    for trial in recomputed:
        name = trial["trial_id"]
        payloads, ledger = bind_visuals(parent / name, visual_root / name, trial)
        outputs.update({f"visuals/{name}/{file}": payload for file, payload in payloads.items()})
        source_visuals[name] = ledger
    audit.require(audit.base.sha(report_path) == expected_sha, "audit changed during curation")
    for trial in recomputed:
        for entry in trial["source_manifest"]:
            audit.require(audit.base.sha(parent / trial["trial_id"] / entry["path"]) == entry["sha256"], "raw input changed during publication")
    publication = {"schema": "portable_e2e.comfortable_v3_publication.v1", "raw_audit_sha256": sha(raw),
        "curator_sha256": audit.base.sha(Path(__file__)), "source_visuals": source_visuals,
        "input_layout": "pilot/town07_straight_calibration/run_NNN and visuals/run_NNN; originals remain private",
        "published_files": {name: {"sha256": sha(payload), "size_bytes": len(payload),
            "byte_preserving_visual": name.endswith((".png", ".gif"))} for name, payload in outputs.items()},
        "training_data_approved": False, "automatic_promotion": False, "notice": NOTICE}
    outputs["publication_manifest.json"] = encode(publication)
    outputs["SHA256SUMS"] = "".join(f"{sha(data)}  {name}\n" for name, data in sorted(outputs.items())).encode()
    output.mkdir(parents=True, exist_ok=False)
    for name, data in outputs.items():
        destination = output / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
    return publication


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("pilot_root", type=Path)
    parser.add_argument("report_path", type=Path)
    parser.add_argument("visual_root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-audit-sha256", required=True)
    args = parser.parse_args(argv)
    curate(args.pilot_root, args.report_path, args.visual_root, args.output_dir, args.expected_audit_sha256)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
