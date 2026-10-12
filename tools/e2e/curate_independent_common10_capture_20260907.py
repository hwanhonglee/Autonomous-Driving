#!/usr/bin/env python3
"""HH_260906 - Publish immutable expert-capture metadata and exact observed media."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import shutil


REPO = Path(__file__).resolve().parents[2]
ROOT = REPO / "artifacts/training/2026-09-07/independent_common10_v1"
DATASET_ID = "carla-common10-30kph-five-episodes-20260907-v3"
DATASET = REPO / "datasets/prepared" / DATASET_ID
OUTPUT = REPO / "docs/assets/validation/2026-09-07/portable_e2e_learning_cycle_v1"


def sha256(path: Path) -> str:
    # HH_260906 - Stream large captured GIFs without loading their full payload into RAM.
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def sanitize(value):
    # HH_260906 - Retain source digests while replacing machine-specific root paths.
    if isinstance(value, str):
        account = r"[^/\s\"']+"
        boundary = r"(?=/|$|[\s\"'])"
        replacements = (
            (rf"/home/{account}/autoware_e2e{boundary}", "${REPO_ROOT}"),
            (rf"/home/{account}/carla-autoware-universe/CARLA_0\.9\.15{boundary}", "${CARLA_ROOT}"),
            (rf"/home/{account}/personal/{account}/dataset{boundary}", "${PERSONAL_DATASET_ROOT}"),
            (rf"/home/{account}/personal/{account}/portable_e2e{boundary}", "${PORTABLE_E2E_ROOT}"),
            (rf"/home/{account}{boundary}", "${USER_HOME}"),
        )
        for pattern, placeholder in replacements:
            value = re.sub(pattern, lambda _match: placeholder, value)
        return value
    if isinstance(value, list):
        return [sanitize(item) for item in value]
    if isinstance(value, dict):
        return {sanitize(key): sanitize(item) for key, item in value.items()}
    return value


def source_view(path: Path) -> dict:
    return {
        "publication_notice": (
            "Metadata view with machine-specific root placeholders. The original "
            "source digest is retained. This is not a standalone training dataset."
        ),
        "raw_source_path": path.relative_to(REPO).as_posix(),
        "raw_source_sha256": sha256(path),
        "record": sanitize(read_json(path)),
    }


def write_json(path: Path, payload) -> None:
    with path.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, indent=2, allow_nan=False) + "\n")


def write_text(path: Path, text: str) -> None:
    with path.open("x", encoding="utf-8") as stream:
        stream.write(text)


def copy_exact(source: Path, destination: Path) -> None:
    with source.open("rb") as incoming, destination.open("xb") as outgoing:
        shutil.copyfileobj(incoming, outgoing, 1024 * 1024)
    if sha256(source) != sha256(destination):
        raise RuntimeError(f"Copy digest mismatch: {destination}")


def finish_category(path: Path) -> None:
    # HH_260906 - Bind every published category file except the checksum file itself.
    lines = [
        f"{sha256(item)}  {item.relative_to(path).as_posix()}\n"
        for item in sorted(path.rglob("*"))
        if item.is_file()
    ]
    write_text(path / "SHA256SUMS", "".join(lines))


def main() -> None:
    summary = read_json(ROOT / "campaign_summary.json")
    categories = (
        ("02_new_training_town01_right", "town01_right_train_retry_02", "Town01"),
        ("03_held_out_test_town04_straight", "town04_straight_test", "Town04"),
    )
    targets = [OUTPUT / name for name, _, _ in categories] + [OUTPUT / "05_dataset_transfer"]
    if any(path.exists() or path.is_symlink() for path in targets):
        raise RuntimeError("Owned output categories must be entirely new")
    for name, scene, town in categories:
        destination = OUTPUT / name
        destination.mkdir(parents=True)
        episode = next(item for item in summary["new_episodes"] if item["map"] == town)
        run = ROOT / scene / "run_001"
        media = (
            ("centered_overview.png", "01_centered_expert_overview.png"),
            ("centered_drive.gif", "02_centered_expert_drive_5fps.gif"),
        )
        for source_name, output_name in media:
            copy_exact(run / "visual" / source_name, destination / output_name)
        for source, output_name in (
            (run / "episode/route.json", "03_aligned_route.json"),
            (run / "episode/manifest.json", "04_native_collection_metadata.json"),
            (run / "analysis/vad_export_64/manifest.json", "05_export_quality_metadata.json"),
            (ROOT / scene / "route_alignment.json", "07_route_alignment.json"),
        ):
            write_json(destination / output_name, source_view(source))
        write_json(destination / "06_native_camera_and_split_summary.json", sanitize(episode))
        result = episode["raw_result"]
        exporter = read_json(run / "analysis/vad_export_64/manifest.json")
        split_note = (
            "새 TRAIN episode입니다. 534개 중 현재 navigation command가 RIGHT인 것은 18개, "
            "LANEFOLLOW는 516개입니다. 534개 전부를 우회전 순간으로 세지 않습니다."
            if town == "Town01"
            else "사전에 지정한 map-held-out TEST episode입니다. 이 지표로 학습 설정·threshold·"
            "checkpoint를 고르지 않습니다. Town03 전체 episode는 기존 VAL로 유지합니다."
        )
        failure_note = ""
        if town == "Town01":
            copy_exact(
                ROOT / "town01_right_train/catalog.log",
                destination / "08_failed_catalog_preflight.log",
            )
            failure_note = (
                "\n첫 catalog preflight는 physical-turn gate가 `left,right` 동시 탐색을 요구해 "
                "주행 전에 종료됐습니다. [실패 원본 로그](08_failed_catalog_preflight.log)를 "
                "보존했습니다. 재시도에서는 두 방향을 탐색하고 사전 지정한 RIGHT만 수집했으며 "
                "geometry 기준은 유지했습니다. 실제 주행은 첫 시도에 통과했습니다.\n"
            )
        text = (
            "<!-- HH_260906 - Bind observed expert evidence to its declared dataset split. -->\n"
            f"# {town} / {'새 우회전 학습 데이터' if town == 'Town01' else '독립 직진 테스트 데이터'}\n\n"
            "이 화면은 CARLA **BasicAgent expert**가 주행하면서 수집한 실제 camera/state 자료입니다. "
            "학습한 Portable E2E 모델이나 Autoware가 이 경로를 제어한 화면은 아닙니다.\n\n"
            f"{split_note}\n\n"
            "[전체화면 PNG](01_centered_expert_overview.png) · "
            "[차량 중심 GIF](02_centered_expert_drive_5fps.gif)\n\n"
            "PNG와 GIF는 원본 bytes를 그대로 복사했습니다. 둘 다 1920×1080이고 GIF는 "
            "200 frames / 5 fps입니다. 여섯 camera와 중앙 ego, 기준 경로, 실제 궤적, "
            "0.1–6.4초 future label을 함께 표시합니다. GIF는 검토용 sampling이며 "
            "wall-clock 실행 속도나 camera 입력 Hz를 재는 자료가 아닙니다.\n\n"
            "| 항목 | 결과 |\n|---|---|\n"
            f"| 경로 / 원본 수집 길이 | {result['route_length_m']:.3f} m / {result['duration_sec']:.3f} s |\n"
            f"| 목표 / 최고 속도 | 30 km/h / {episode['maximum_speed_kph']:.3f} km/h |\n"
            f"| 원본 bundle / state | {result['camera_anchor_count']} / {result['state_count']} |\n"
            f"| native camera / physics | {episode['native_camera_rate_hz']:.9f} Hz / 20 Hz |\n"
            f"| 최대 camera skew | {episode['maximum_bundle_skew_s'] * 1000:.3f} ms |\n"
            f"| goal / 충돌 / 차선침범 | PASS / {result['collision_event_count']} / {result['lane_invasion_event_count']} |\n"
            f"| 최대 절대 CTE | {episode['maximum_absolute_cte_m']:.3f} m |\n"
            f"| 64-horizon export / Common10 선택 | {exporter['sample_count']} / {episode['prepared_sample_count']} samples |\n\n"
            "두 sample 수 차이 1개는 Common10이 stationary-tail anchor를 제외하기 때문입니다. "
            "tail state는 앞선 sample의 future context로 보존하며 프레임을 복제하거나 시간을 바꾸지 않았습니다.\n\n"
            "수집 조건은 640×360 six-camera, JPEG95, ClearNoon, seed1, 3.5초 brake warmup, "
            "6.5초 brake tail, Prius wheelbase2.850m입니다. Expert waypoint purge는 "
            "2.0m + 0.2s × speed(m/s), lateral PID 1.95/0.05/0.20, steering max0.8, "
            "lane offset0m를 사용했습니다. 외부 traffic actor는 추가하지 않았습니다.\n"
            f"{failure_note}\n"
            "원본 경로·manifest·수집/전송 SHA는 [metadata](06_native_camera_and_split_summary.json)에 "
            "있습니다. JSON 표시본은 machine root를 placeholder로 바꿨으며 각 wrapper에 "
            "원본 SHA를 남겼습니다. 발행본 무결성은 `sha256sum -c SHA256SUMS`로 확인합니다.\n\n"
            f"원본 episode: `artifacts/training/2026-09-07/independent_common10_v1/{scene}/run_001/episode`\n\n"
            "새 장애물·ACC·차선변경 label, 학습 모델 closed-loop, 실차 운용 승인을 입증하지 않습니다.\n"
        )
        write_text(destination / "README.md", text)
        finish_category(destination)
    destination = OUTPUT / "05_dataset_transfer"
    destination.mkdir(parents=True)
    proofs = []
    for filename in (
        "local_tree_manifest.json", "remote_staging_tree_manifest.json", "remote_prepared_tree_manifest.json"
    ):
        path = ROOT / "transfer" / filename
        report = read_json(path)
        source = report["source"]
        proofs.append({
            "source_path": path.relative_to(REPO).as_posix(),
            "source_file_sha256": sha256(path),
            "read_only": report["read_only"], "error_count": report["error_count"],
            "source_read_only": source["read_only"], "source_error_count": source["error_count"],
            **{key: source[key] for key in ("manifest_sha256", "file_count", "directory_count", "total_size_bytes")},
        })
    dry_run = ROOT / "transfer/rsync_checksum_dry_run.txt"
    if dry_run.stat().st_size != 0:
        raise RuntimeError("Checksum dry-run was not empty")
    write_json(destination / "01_transfer_integrity.json", {
        "status": "PASS", "dataset_id": DATASET_ID,
        "dataset_fingerprint_sha256": summary["dataset_fingerprint_sha256"],
        "dataset_manifest_sha256": summary["dataset_manifest_sha256"],
        "split_sample_counts": summary["split_sample_counts"],
        "remote_dataset_root": "${PERSONAL_DATASET_ROOT}/prepared/" + DATASET_ID,
        "tree_proofs": proofs,
        "checksum_dry_run": {"source_path": dry_run.relative_to(REPO).as_posix(), "sha256": sha256(dry_run), "size_bytes": 0, "change_count": 0},
        "promotion": "Same-filesystem no-overwrite staging rename, followed by final-tree and planning revalidation.",
        "network_transfer_stats": (ROOT / "transfer/rsync_transfer.log").read_text(),
        "remote_package_installations": 0, "remote_gpu_used_by_transfer": False,
        "vehicle_control_approved": False,
    })
    for source, name in (
        (ROOT / "transfer/remote_planning_validation.json", "02_dataset_planning_validation.json"),
        (ROOT / "conversion_source_proof.json", "03_reviewed_conversion_source.json"),
        (DATASET / "dataset.json", "05_dataset_manifest.json"),
    ):
        write_json(destination / name, source_view(source))
    write_json(destination / "04_preserved_previous_splits.json", {
        "status": "PASS", "previous_dataset_id": "carla-common10-30kph-town07-ctrack-town03-20260905-v2",
        "comparison_fields": ["split", "sample_count", "sample_jsonl_sha256", "route_geometry_sha256"],
        "episodes": summary["preserved_v2_episodes"], "test_policy": summary["test_selection_policy"],
    })
    write_text(destination / "README.md", (
        "<!-- HH_260906 - Publish compact verifiable transfer evidence without raw image datasets. -->\n"
        "# Common10 v3 / 개인 학습 서버 전송 검증\n\n"
        f"`{DATASET_ID}`는 로컬과 원격 personal venv의 planning 검증을 모두 통과했습니다. "
        "전체 dataset 원본은 Git에 넣지 않습니다. 이 폴더는 검토용 metadata와 무결성 증거입니다.\n\n"
        "| Split | Episode | Samples |\n|---|---|---:|\n"
        "| TRAIN | Town07 straight + CTrack left + 새 Town01 right | 1147 |\n"
        "| VAL | 기존 Town03 right 전체 | 337 |\n"
        "| TEST | 새 Town04 straight 전체 | 309 |\n"
        "| 합계 | 5 episodes | 1793 |\n\n"
        "기존 세 episode의 split·sample 수·sample JSONL SHA·route geometry SHA는 v2와 "
        "동일합니다. Town04 test는 모델·threshold 선택에 쓰지 않습니다.\n\n"
        f"Dataset fingerprint: `{summary['dataset_fingerprint_sha256']}`\n\n"
        f"Tree manifest: `{summary['local_remote_tree_manifest_sha256']}`\n\n"
        f"파일 {summary['file_count']:,}개, 디렉터리 {summary['directory_count']}개, "
        f"{summary['total_size_bytes']:,} bytes가 로컬·원격 staging·원격 prepared에서 일치했습니다. "
        "`rsync -rcn --delete`의 변경 목록은 0 byte였습니다. 새 staging만 사용했고 "
        "동일 filesystem에서 덮어쓰기 없이 승격한 후 다시 tree/contract를 검증했습니다.\n\n"
        "[전송 무결성](01_transfer_integrity.json) · "
        "[원격 planning 검증](02_dataset_planning_validation.json) · "
        "[기존 split 보존](04_preserved_previous_splits.json)\n\n"
        "변환 code와 전체 프로젝트 import closure, model config 등 10개 파일을 reviewed commit "
        "`081a71f7fc014d2864b790b0b0cae7378ae18e4c`의 git blob과 byte 단위로 대조해 일치를 확인한 뒤 "
        "converter가 지원하는 명시적 commit 옵션을 사용했습니다. 동시에 수정되던 문서·비교기 등은 "
        "변환 import 대상이 아니며 [source proof](03_reviewed_conversion_source.json)에 남겼습니다.\n\n"
        f"로컬 원본: `datasets/prepared/{DATASET_ID}`\n\n"
        f"원격 원본: `${{PERSONAL_DATASET_ROOT}}/prepared/{DATASET_ID}`\n\n"
        "개인 경로는 발행 JSON에서 placeholder로 바꿨습니다. 각 표시본은 원본 report SHA를 "
        "포함합니다. 원본 file-by-file manifest는 ignored artifacts의 `transfer/`에 보존했습니다. "
        "발행본은 `sha256sum -c SHA256SUMS`로 확인합니다.\n\n"
        "이 전송 작업은 원격 package/Git/GPU를 변경하지 않았습니다. Planning data PASS는 "
        "학습 모델 품질이나 closed-loop·실차 승인 PASS를 뜻하지 않습니다.\n"
    ))
    finish_category(destination)
    print("PASS: published exact expert media and metadata in categories 02, 03, and 05")


if __name__ == "__main__":
    main()
