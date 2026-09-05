# CARLA Common10 30 km/h 학습·검증 증거 — 2026-09-05

이 폴더는 현재 단계의 결과를 실행 계층별로 나눈 단일 검토 묶음이다. 최신
CARLA expert 수집, Common10 데이터 검증, 원격 1-epoch 학습과 Town03 open-loop
평가를 포함한다. `autoware_runtime_historical/`은 비교 편의를 위해 함께 둔
2026-09-02 Autoware selected-regression 사본이며, 이번 체크포인트를 Autoware에서
폐루프로 실행한 화면이 아니다.

## 현재 결론

| 범위 | 결과 | 의미 |
|---|---:|---|
| Town07 직진 30 km/h | PASS | train 309 sample |
| C-track 좌회전 30 km/h | PASS | B compact-lookahead, train 304 sample |
| Town03 우회전 30 km/h | PASS | 신호 준수, val 337 sample |
| 통합 Common10 planning | PASS | 3 episode, 950 sample, 정확한 약 10 Hz |
| 원격 CPU smoke | PASS | 이미지 decode부터 checkpoint까지 1 step |
| 원격 GPU 0 학습 | PASS | 613 train sample을 중복 없이 1 epoch/154 step |
| Town03 전체 open-loop val | COMPLETE | 337 sample, 6.4초 ADE/FDE 8.92/19.35 m |
| 이번 체크포인트의 Autoware/CARLA 폐루프 | NOT RUN | 다음 단계 |
| 실제 차량 제어 | NOT APPROVED | test split·real data·shadow·폐쇄시험 필요 |
| 모든 지원 Town의 Common10 직진+회전 | PENDING | 이번 PASS 범위를 확대 해석하면 안 됨 |

## 최신 CARLA expert 화면

모든 화면은 1600×900이며 오른쪽 패널의 차량은 중앙에 고정된다. 회전해도
heading-up으로 경로, 지나온 expert history, 앞으로 6.4초의 label trajectory와
전체 경로 inset을 동시에 볼 수 있다.

| 시나리오 | 전체 화면 PNG | 주행 GIF |
|---|---|---|
| Town07 직진 | [PNG](expert_data/town07_straight/centered_overview.png) | [GIF](expert_data/town07_straight/centered_drive.gif) |
| C-track 좌회전 | [PNG](expert_data/c_track_left_turn/centered_overview.png) | [GIF](expert_data/c_track_left_turn/centered_drive.gif) |
| Town03 우회전 | [PNG](expert_data/town03_right_turn/centered_overview.png) | [GIF](expert_data/town03_right_turn/centered_drive.gif) |

각 GIF는 60 frame, 검토용 5 fps, 12초다. 이것은 문서 재생 속도이며 센서
원본과 학습 bundle의 검증된 cadence는 약 10 Hz다.

## 수집·A/B 결과

C-track의 기존 A/default 제어는 목표 도달과 충돌 0이었지만 차선 침범 4회로
Common10 gate에서 제외했다. B는 waypoint purge lookahead를
`3.0 + 0.5 × speed(m/s)`에서 `2.0 + 0.2 × speed(m/s)`로 줄였고, lateral PID
`1.95/0.05/0.2`, 최대 steering `0.8`, lane offset `0.0 m`는 유지했다. 선택한
B 실행은 목표 도달, 충돌 0, 차선 침범 0이고 CTE mean/p95/max는
0.082/0.291/0.342 m다.

네 실행의 정확한 비교와 미선정 사유는
[reports/expert_control_ab_summary.json](reports/expert_control_ab_summary.json)에 있다.
미채택 실행을 manifest로 보존한 뒤 수행한 정리는
[reports/cleanup_summary.json](reports/cleanup_summary.json)에 기록했다.

Town03 첫 실행이 멈춘 것처럼 보인 원인은 카메라 10 Hz나 PC 부하가 아니다.
50초 driving budget 중 약 47.5초가 출발 직후 신호 대기로 소비됐다. 신호 무시나
PID 변경 없이 total budget만 120초로 늘린 새 실행은 30.2초 주행 후 목표에
도달했고 충돌·차선 침범은 모두 0이었다.

## Common10 데이터

통합 데이터셋은 다음과 같이 route/site 단위로 분리했다.

- train: Town07 직진 309 + C-track 좌회전 304 = 613 sample
- val: Town03 우회전 337 sample
- test: 없음

6개 카메라, 20 Hz physics/10 Hz camera, 3.5초 정지 warm-up, BasicAgent 주행,
6.5초 정지 tail, 0.1–6.4초 64-point 미래 궤적 계약을 사용했다. 세 episode 모두
목표 도달, 충돌 0, 차선 침범 0이다. planning validator에서 image SHA-256,
native frame identity, source-manifest binding, timestamp, causal route,
future coverage가 PASS했고 bundle skew/state delta/command age는 0 ms다.

기계 판독 요약은 [validation_summary.json](validation_summary.json)에 있다.

## 학습과 평가

서버에서는 개인 프로젝트 venv만 사용했고 시스템 Python·Conda에는 설치하지
않았다. 물리 GPU 0 한 장만 allowlist로 노출했으며 다른 GPU의 실행에는 접근하지
않았다. CPU 1-step smoke 후 perspective trajectory v0 모델(1,053,278 parameters)을
batch 4, seed 20260903, learning rate 1e-4, 154 step으로 학습했다. 613개 train
sample은 replacement 없이 정확히 한 번 사용됐고 버려진 sample은 0개다.

Town03 val 337개 전체 평가 결과는 다음과 같다.

| horizon | ADE | FDE |
|---:|---:|---:|
| 1.0 s | 1.432 m | 2.235 m |
| 3.0 s | 3.779 m | 7.706 m |
| 6.4 s | 8.919 m | 19.349 m |

모델 forward 계측은 0.791 ms/sample이었지만 공유 서버의 단일 실행 참고값이다.
ADE/FDE가 아직 크므로 빠른 추론과 좋은 주행 품질을 같은 의미로 보면 안 된다.
상세 수치와 artifact hash는
[reports/training_evaluation_summary.json](reports/training_evaluation_summary.json)에
있고, 154 step 원시 학습 계측은
[reports/training_metrics.jsonl](reports/training_metrics.jsonl)에 있다. 12개 예측 예시는
[model_open_loop/town03_val](model_open_loop/town03_val/)에
있으며 gray=route, green=expert, 색상 궤적=선택 후보다.

154 step 모두 clipping 전 gradient norm이 threshold 5.0을 넘었다. trainer가 실제로
clip을 적용했으므로 실행 실패는 아니지만, 다음 학습에서는 learning rate, loss scale과
gradient 분포를 별도 A/B로 확인해야 한다.

## 별도 Autoware 실행 화면

아래 자료는 2026-09-02의 별도 Autoware VAD 30 km/h selected-regression이다.
동일한 세 시나리오를 빠르게 비교하도록 사본을 한 폴더에 모았지만, 이번
Common10 expert나 새 체크포인트 결과와 provenance가 다르다.

| 시나리오 | 차량 중심 화면 | 주행 | 경로·제어 | steering | 속도 |
|---|---|---|---|---|---|
| Town07 직진 | [PNG](autoware_runtime_historical/town07_straight/01_autoware_vehicle_centered_fullscreen.png) | [GIF](autoware_runtime_historical/town07_straight/02_autoware_drive.gif) | [path](autoware_runtime_historical/town07_straight/03_path_vs_control.png) | [steer](autoware_runtime_historical/town07_straight/04_steering_tracking.png) | [speed](autoware_runtime_historical/town07_straight/05_speed_profile.png) |
| C-track 회전 | [PNG](autoware_runtime_historical/c_track_turn/01_autoware_vehicle_centered_fullscreen.png) | [GIF](autoware_runtime_historical/c_track_turn/02_autoware_drive.gif) | [path](autoware_runtime_historical/c_track_turn/03_path_vs_control.png) | [steer](autoware_runtime_historical/c_track_turn/04_steering_tracking.png) | [speed](autoware_runtime_historical/c_track_turn/05_speed_profile.png) |
| Town03 회전 | [PNG](autoware_runtime_historical/town03_turn/01_autoware_vehicle_centered_fullscreen.png) | [GIF](autoware_runtime_historical/town03_turn/02_autoware_drive.gif) | [path](autoware_runtime_historical/town03_turn/03_path_vs_control.png) | [steer](autoware_runtime_historical/town03_turn/04_steering_tracking.png) | [speed](autoware_runtime_historical/town03_turn/05_speed_profile.png) |

## 해석 제한과 다음 gate

- val은 학습 route와 분리된 Town03이지만 독립 test split은 아직 없다.
- 현재 모델 평가는 open-loop이며 Autoware planner/control adapter나 CARLA
  closed-loop를 통과하지 않았다.
- 장애물 회피, ACC, 정지선 일반화, 차선 변경은 이 데이터와 평가에서 검증하지
  않았다.
- 실측 데이터가 없고 sim-to-real 검증도 수행하지 않았다.
- 따라서 이 체크포인트는 연구 baseline이며 실제 차량 제어에 사용하면 안 된다.
- 다음 순서는 데이터/모델 A/B 고도화, 학습되지 않은 test episode 확보,
  CARLA closed-loop, 실제 데이터 replay/shadow, 폐쇄시험장 안전 gate다.

폴더 전체 파일의 상대 경로와 SHA-256은 `SHA256SUMS`로 고정한다.
