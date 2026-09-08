# 물리 적분 상한 A/B — C-track 저속 출발·정지 원인 계측

<!-- HH_260906 - Separate an explicitly prospective numerical experiment from data admission and learned vehicle control. -->

2026-09-09 새 수집 전 고정한 연구입니다. 기존 CARLA expert 제어로 주행하며,
학습 모델이 운전하거나 Autoware를 새로 제어하는 시험이 아닙니다.

| 조건 | A | B |
|---|---|---|
| 최대 physics substep 시간 | 10 ms | 5 ms |
| 최대 substep 수 | 10 | 10 |
| World / 카메라 | 20 Hz / 10 Hz | 동일 |
| 맵·날씨·seed | C-track / ClearNoon / 0 | 동일 |
| 목표 속도·제어 | 14.4 kph / turn_launch_013_v1 | 동일 |
| 화면 품질 | Epic | 동일 |

CARLA 0.9.15의 [설정 검증 코드](https://github.com/carla-simulator/carla/blob/0.9.15/LibCarla/source/carla/client/detail/Simulator.cpp)는
최대 substep 수의 지원 범위와 world 시간 간격의 관계를 검사합니다. 이 실험은 지원 범위 안에서
최대 시간 간격만 바꿉니다. 실제 내부 적분 횟수를 측정한 결과라고 해석하지 않습니다.

순서는 A1 → B1 → 독립 중간 감사와 진행 판단 기록 → A2 → B2입니다.
동일 환경에서 각 두 번의 반복이며, 서로 다른 맵·날씨에 대한 일반화 검증은 아닙니다.
모든 JPEG·native 20 Hz 시계열·10 Hz 두 위상·64점 미래 경로를 보존하고 검사합니다.
좋지 않은 저속 구간을 잘라내거나 라벨·곡률 한계를 수정하지 않습니다.

고정 계획은 저장소 기준 `artifacts/training/2026-09-09/substep_ab_v1/plan.json`입니다.
SHA-256: `28e6c891c74b3e33de5714a11510c51e2230874bd1335cfcfb44caf6b6d5a83e`.
실행 소스 기준은 `2e1db50f6ffd9c8994078fc7bc2f88f762a07782`의 명시된 12개 파일입니다.
각 실행은 같은 바이트를 전후 확인하고 별도로 보관합니다.

## 네 번의 실제 실행 결과

<!-- HH_260906 - Report repeated scalar and geometric failures with their original, non-independent denominators. -->

02:33:31 KST에 마지막 B 실행까지 종료됐습니다. **네 번 모두 데이터 미승인**입니다.
A는 출발·정차 가감속 검사는 통과했지만 저속 XY 곡률 초과가 남았고, B는 곡률 초과 창이
줄어든 대신 native 가감속·횡가속·목표 정차가 악화했습니다. 설정 변경을 채택하지 않습니다.

| 시도 | native 상태 / 카메라 묶음 | 완전한 64점 미래 창 | XY 곡률 초과 창 | native 가속도 최소 / 최대 (m/s²) | 목표 거리 / 정차 판정 |
|---|---:|---:|---:|---:|---|
| A1 | 1,939 / 970 | 905 | 161 | −1.6264 / +2.3993 | 0.9497 m / scalar PASS |
| B1 | 2,025 / 1,013 | 949 | 71 | −3.4820 / +3.4027 | 2.1514 m / coast timeout FAIL |
| A2 | 1,939 / 970 | 905 | 161 | −1.6264 / +2.3993 | 0.9497 m / scalar PASS |
| B2 | 2,025 / 1,013 | 949 | 71 | −3.4820 / +3.4027 | 2.1514 m / coast timeout FAIL |

총 **7,928개 native 상태·3,966개 카메라 묶음·23,796장 원본 JPEG**를 검사했습니다.
완전한 미래 창 3,708개와 B의 미완료 미래 문맥 128개, A의 tail 문맥 130개를 구분해 보존합니다.
A의 곡률 위반은 각 3,545개 점/창 조합·64개 고유 100 ms tick, B는 각 93개 조합·6개 tick입니다.
B에서는 횡가속 위반이 각 7개 창·7개 점/창 조합·1개 고유 tick에 추가로 나타났습니다.
겹치는 미래 창과 동일 초기 조건의 반복을 독립 주행 사건 수로 세지 않습니다.

12개 실행 소스 보관본, 센서·제어 수신·초기화·설정 readback, owner 종료·포트 반환은
모두 확인했습니다. B의 실패 절차가 의도대로 실행됐다는 프로토콜 PASS는 주행 품질 PASS가 아닙니다.
이 비교는 **14.4 kph 목표의 expert 시험**이며 30 kph 통과, learned closed-loop 또는 실차 검증이 아닙니다.

## 차량 중심 화면과 원본 기반 경로

아래 화면은 실제 6개 카메라 기록과 차량 중심 경로도입니다. 화면 전체를 1600×900으로 구성하고
원본 화각을 자르지 않습니다. 붉은 선은 기록된 미래 이동이며 모델 예측이 아닙니다.
Autoware 라이브 화면으로 표시하지 않습니다.

![B1 기록에서 경로 절반을 지난 모습](evidence/milestones/B_fine_5ms/run_001/02_first_half_route_crossing.png)

| 시도 | 전체 기록의 가속 미리보기 | 실제 조향이 가장 컸던 카메라 묶음 |
|---|---|---|
| A1 | [GIF](evidence/visuals/A_reference_10ms/run_001/whole_recording_accelerated.gif) | [PNG](evidence/milestones/A_reference_10ms/run_001/03_maximum_absolute_reported_steering.png) |
| B1 | [GIF](evidence/visuals/B_fine_5ms/run_001/whole_recording_accelerated.gif) | [PNG](evidence/milestones/B_fine_5ms/run_001/03_maximum_absolute_reported_steering.png) |
| A2 | [GIF](evidence/visuals/A_reference_10ms/run_002/whole_recording_accelerated.gif) | [PNG](evidence/milestones/A_reference_10ms/run_002/03_maximum_absolute_reported_steering.png) |
| B2 | [GIF](evidence/visuals/B_fine_5ms/run_002/whole_recording_accelerated.gif) | [PNG](evidence/milestones/B_fine_5ms/run_002/03_maximum_absolute_reported_steering.png) |

GIF는 원래 카메라 묶음 5개 간격과 마지막 관측을 선택해 10 fps로 재생한 미리보기입니다.
A는 195장/19.5초, B는 204장/20.4초이며 실제 GUI FPS나 주행 속도 계측을 대신하지 않습니다.
추가 PNG의 첫 13.68 kph 도달·경로 절반 통과·최대 조향 선택은 수집 후 정한 시각화 규칙이며,
좋은 구간만 품질 평가에 넣는 규칙이 아닙니다. 각 폴더의 provenance에 정확한 원본 frame/index가 있습니다.

## 검증 자료와 정정 이력

[전체 판정](evidence/audit.json) · [게시 파일·원본 해시](evidence/publication_manifest.json) ·
[게시 SHA-256](evidence/SHA256SUMS) · [첫 A/B 뒤의 반복 진행 검토](evidence/provenance/first_pair_review.json).
큰 전수 JSONL은 private `artifacts/training/2026-09-09/substep_ab_audit_complete_v2`에 보존하고,
게시 manifest에 정확한 경로·크기·SHA를 남겼습니다. 원본 checkpoint나 학습 데이터로 변환하지 않았습니다.

첫 두 감사 출력의 `SHA256SUMS`가 자기 자신의 빈 해시를 포함한 오류를 발견했습니다.
**두 감사의 audit.json을 포함한 8개 결과 파일의 해시는 모두 정상**이었으며, 기존 결과·진행 검토를
덮어쓰지 않았습니다. [사후 정정 기록](evidence/provenance/checksum_correction.json)에 발견 시각,
보존한 최초 실행 코드와 원본 체크섬 검증을 남겼습니다. 수정 감사기의 최종 네 payload는
`sha256sum -c`를 모두 통과했습니다. 사후 수정본이 A2 이전에 존재했다고 표시하지 않습니다.

## 실제 처리 시간 — 화면 FPS와 구분

<!-- HH_260906 - Reconstruct every wall-timing attempt independently while retaining failed captures and unchanged simulation timestamps. -->

네 실행의 7,928개 native 기록·3,966개 카메라 묶음을 전부 대조했습니다.
시뮬레이션 20 Hz / 10 Hz 주기는 유지됐고 시간 기록 누락·저장 오류는 없었습니다.
B1/B2는 시간 기록 자체가 정상이어도 원래 정지 실패 및 `PARTIAL_OR_FAILED_DIAGNOSTIC`을 유지합니다.

| 실행 | 실제 시간 기준 카메라 파일 기록 Hz | 기록 간격 p99 | 시뮬레이션 / 실제 시간 |
|---|---:|---:|---:|
| A1 | 35.03 | 37.80 ms | 3.48배 |
| B1 | 35.95 | 36.18 ms | 3.58배 |
| A2 | 35.61 | 36.67 ms | 3.54배 |
| B2 | 35.16 | 36.46 ms | 3.50배 |

약 35 Hz는 약 3.5배속 오프라인 수집의 **파일 기록 속도**이며 GUI FPS·학습 모델 추론 Hz가 아닙니다.
JPEG 변환·쓰기는 합산 in-tick 시간의 약 35%, 관측·제어 계산은 48–50%였습니다.
분모에 tick 사이 간격·계측 기록 저장·초기화/종료는 포함하지 않습니다.
`world_tick_snapshot`의 5–6%도 순수 물리 계산 시간이나 GPU 부하로 해석하지 않습니다.

![전체 native 속도·가감속·남은 경로](timing/01_all_native_speed_acceleration_goal.png)

[실제 기록 간격·처리 구간 그래프](timing/02_actual_wall_gaps_and_in_tick_shares.png) ·
[원본과 동일한 계측 결과 및 입력·소스 해시](timing/summary.json) · [게시 검증 및 SHA-256](timing/SHA256SUMS).

[별도 화면 검증](timing/visual_verification.json)에서 기존 게시 48개 파일의 체크섬을 확인했습니다.
PNG 30장과 GIF 4개는 원본 바이트와 같고 모두 1600×900이며, GIF의 총 798프레임과 각 100 ms 표시 시간을 검사했습니다.
PNG 안의 6개 카메라 영역 총 180개는 원본 JPEG 전체 화각을 같은 크기로 조정한 픽셀과 정확히 일치합니다.
8개 선택·출처 기록 및 실제 원본 이미지 해시도 대조했습니다. 기존 화면은 수정하지 않았으며,
이 검증은 영상 출처 확인이지 데이터 승인·주행 품질 통과를 의미하지 않습니다.
