# Town03 직진 DEV — 수집 전 경로 중복 검사와 재개 지점

<!-- HH_260906 - Record a bounded, non-executing DEV preflight and preserve the overlap finding before capture. -->

상태: **사전검사 도구 구현·실제 파일 검사 완료. 신규 수집·변환·학습은 미실행.**
사용자가 잔여 사용량 8%를 알린 뒤 작업을 작은 단위로 제한했다. 실제 계정 잔여량이나
이번 턴의 과금 토큰은 측정하지 않았으며, 8%를 특정 실행 시간으로 환산하지 않았다.

## 이번에 확인한 결과

| 항목 | 결과 |
| --- | --- |
| 후보 | `V03-S01`, `Town03`, `town03_straight_s0000_p04` |
| 제안 역할·조건 | 개발 VAL 전용, ClearNoon, seed 201, 15 km/h, 카메라 10 Hz·상태 20 Hz |
| 경로 SHA256 | `fe40145556a1e946b87f6c6c846df4c1e8cc08cc2fc75047b6b426bd12408812` |
| 평면 경로 길이 | 209.350010 m |
| 사전 선택 구간 | arc 30–165 m; 정속 가정 32.4초, 실제 시간 검증 전 |
| 6.4초 미래를 위한 명목 거리 이후 여유 | 17.683344 m; 실제 제동·시간 여유 보증 아님 |
| 비교 자료 | 기존 TRAIN 3 episode + DEV 1 episode + 신규 Town07 TRAIN pilot 1 episode |
| 기존 Town03 우회전 DEV 경로와 최소 평면 거리 | **3.499997 m** |
| 기존 경로의 5 m 이내에 있는 후보 waypoint | **53/213개**; 주행 시간이나 경로 길이 비율이 아님 |
| 판정 | **기존 DEV와의 공간적 근접 검토 필요. 수집 실행 허용 아님.** |

다른 맵의 수치상 XY 좌표는 같은 공간으로 비교하지 않았다. Town03끼리만 준비된 경로
polyline을 비교했다. 원본 Town03 경로 SHA
`47ea507b9a4228fdce79ef58eff1e1d2cf149c02dd2df413ebadf53756f771ee`에 기록된
`carla_map → map` identity 변환을 확인하고, 원본 XY와 준비된 polyline이 일치하는지도 검사했다.
선분 내부 교차도 거리 계산에 포함한다.

**이것은 TRAIN–DEV 유출을 발견했다는 뜻이 아니다.** 가까운 두 경로는 모두 DEV이며,
5 m는 보수적인 검토 신호이지 기존 데이터 품질 기준이나 새 자동 승인 기준이 아니다.
반대로 다른 SHA/seed라는 이유만으로 독립 경로라고 세어서도 안 된다. 이번 검사는 전체 경로
기하 비교이며 실제 주행 궤적·선택 30–165 m 구간의 중복 길이·모든 과거 사용 이력까지
검증하지 않았다. 기존 맵 manifest의 TRAIN 표기는 수정하지 않고, 새 제안의 DEV 역할과 구별했다.

신규 Town07 328표본을 비교 목록에 포함한 것은 공간·분할 점검을 위한 것이다.
기존 20-epoch 학습에 그 표본을 사용했다는 의미가 아니다.

## 코드·결과·검사

- [새 CPU 사전검사 도구](../../../../../../tools/e2e/audit_carla_moving_dev_preflight.py)
- [새 도구 테스트](../../../../../../learning/tests/test_audit_carla_moving_dev_preflight.py)
- [최종 실제 검사 JSON](../../../../../../storage/experiments/training/2026-09-16/moving_dev_preflight_v1/preflight_final.json)
- [최종 소스 보관본](../../../../../../storage/experiments/training/2026-09-16/moving_dev_preflight_v1/final_source)

최종 JSON SHA256: `339f946dce3cc491ecf4dd09c231cefaf05309fb617f589574c24dc5bf677e7c`.
관련 6개 테스트 파일 회귀 **245 passed / 66.41초**. 마지막 보고 필드 정리 후 새 도구와
cohort planner **60 passed / 0.09초**를 재확인했다. `git diff --check`도 통과했다.
이 수치는 코드 검사이며 수집 성공이나 모델 성능을 뜻하지 않는다.

초기 `preflight.json`과 `preflight_frame_verified.json`은 덮어쓰지 않았다.
초기판에는 좌표계 근거 결합이 없었고, 후속판에는 수집 전/후 남은 작업 목록을 분리하기 전의
표기가 있다. 각각의 도구 소스도 `initial_source`, `frame_verified_source`에 남겼다.
최종 판정·재개 기준은 **`preflight_final.json`**이다.

기존 `carla_moving_capture.py`, `qualify_carla_moving_common10.py`, 수집기, 실행 wrapper,
데이터·모델·기존 결과를 수정하지 않았다. 이번에는 로컬 CPU만 사용했으며 CARLA RPC,
SSH 접속, GPU 학습, 설치, TEST payload 열람, commit/push를 하지 않았다.
실제 수집을 하지 않았으므로 신규 주행 PNG/GIF도 만들지 않았다.

## 재현 명령

저장소 루트에서 실행한다. 출력 파일이 이미 있으면 거부하므로 아래 `recheck.json`도
이미 존재할 경우 새로운 이름을 지정한다. 기존 결과는 삭제하거나 덮어쓰지 않는다.

```bash
cd /home/a/autoware_e2e
PYTHONDONTWRITEBYTECODE=1 python3 -B -m scripts.e2e.audit_carla_moving_dev_preflight \
  --catalog-maps-root artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps \
  --reference-dataset datasets/prepared/carla-common10-trainval-research-20260914-v1 \
  --reference-dataset artifacts/training/2026-09-15/moving_cohort_v1/prepared_moving_train_v1 \
  --output-json artifacts/training/2026-09-16/moving_dev_preflight_v1/recheck.json

PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -q -p no:cacheprovider \
  tests/test_audit_carla_moving_dev_preflight.py \
  tests/test_plan_portable_moving_cohort.py \
  tests/test_plan_portable_map_coverage.py \
  tests/test_carla_moving_capture.py \
  tests/test_qualify_carla_moving_common10.py \
  tests/test_run_owned_carla_expert_trial.py
```

## 다음 재개 순서

1. 후보와 기존 DEV의 근접 구간이 같은 도로·인접 차선인지 확인하고, 선택 구간/미래 지원
   구간에서 겹치는 범위를 계산한다. 같은 그룹으로 관리할지 더 독립적인 후보를 사용할지
   근거를 기록한다. 새 경로를 고르면 선언·SHA를 수집 전에 새로 고정한다.
2. 확정한 DEV 사례용 버전 계약을 구현한다. 기존 Town07 전용 검증의 pin을 지우거나
   바꾸지 말고, owner 소스 보관·replay·raw 검사·DEV 변환까지 일관되게 확장한다.
   **이번 도구는 이 실행 기능을 구현하지 않았으며 일반 수집기 완성이라고 표시하지 않는다.**
3. CPU 회귀·소유권·포트 준비 확인 후 로컬 CARLA 수집 **1회**를 실행한다.
   BasicAgent 전문가 주행임을 명시하고, 실패 시 자동 반복하지 않는다.
4. 수집 후 실측 30초 이상 선택 구간·6.4초 미래 지원·영상 동기화·원시 품질을 검사한다.
   통과한 경우에만 DEV 변환 결과와 차량 중심 PNG/GIF를 발행한다.

이번 작업 묶음에는 원격 학습을 포함하지 않는다. 학습·모델 구조 A/B는 별도 후속 단계다.
