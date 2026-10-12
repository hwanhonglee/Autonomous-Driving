# 저장된 표지판 마스크의 크기·경계 진단 — 준비 V1

<!-- HH_261007 - Prospectively fix descriptive full-denominator bins and resources without changing old model criteria. -->

상태는 **소스 준비 중, 실제 재생 미실행**이다. 새 학습·추론·모델 변경 없이 최신 RGB7 native-fusion V3의 저장된 parent/control/candidate 결과를 비교한다. 메인 0/9·하위 0/30이며 지도 기능 승인과는 별개다.

## 고정 범위와 질문

Town01 118프레임과 Town02 232프레임을 세 모델 모두에서 확인한다. 전체 분모는 **3×350=1,050 arm-frame**이고 첫 프레임·실패·미실행 꼬리를 모두 남긴다. `target.npy`와 `prediction.npy`만 해제한다. RGB·TRAIN·TEST·가중치·depth·pose는 해제하거나 모델에 넣지 않는다. 전체 NPZ SHA256 바이트 봉인은 압축된 RGB 바이트를 불투명하게 읽지만 RGB NPY 항목을 열거나 해석하지 않는다.

정확한 7개 라벨은 기존 manifest의 other/road/sidewalk/lane_marking/traffic_light_fixture/**traffic_sign(index5)**/pole이다. 크기 bin은 결과 열람 전에 고정한 **1 / 2–4 / 5–16 / 17–64 / 65–256 / 257이상 픽셀**이다. 8-connected 라벨 성분을 모두 합산하며 영상의 첫·마지막 행/열에 닿는 성분을 border로 분류한다. 각 bin에 성분수·픽셀수·TP·FN 또는 FP·TP0 성분수·border 성분수/픽셀수 및 그 성분의 TP/error 픽셀을 남긴다. 프레임마다 49-cell confusion, 전체 성분 순서/수치 해시, sign FP/FN 및 other→sign FP를 남긴다. 모든 성분의 scan-order `[area,TP,FN_or_FP,border]` int64 해시를 남기며 거대한 개별 성분 목록은 쓰지 않는다.

목적은 작은 성분/영상 외곽과 오차가 **연관**되는지 기술하는 것이다. 연결성분은 실제 표지판 물체의 신원·개수·검출 성공을 보장하지 않는다. 보관된 두 마스크의 native 360×640 정렬은 확인하지만, crop 학습·잘못된 label·모델 구조를 원인으로 확정하지 않는다. best-offset 탐색·기준/라벨 변경·프레임 제외·작은 성분 무시를 하지 않는다.

## 기존 기준과 원본 보존

전체 350프레임의 7개 클래스 confusion/IoU/precision/recall/TP/FP/FN을 기존 정수·null 규칙 그대로 재계산하여 [최신 원본 결과](../../2026-10-02/portable_e2e_learning_cycle_v1/25_map_native_fusion_v3_results/results.json)와 정확히 비교한다. 기존 두 `diagnostic_screen` 함수의 AST를 소스 봉인 후 최소 private namespace에서 실행해 **27개 기준 전체**를 재계산한다. 원래 23통과·4실패와 알려진 EVAL 노출/new DEV 0을 유지한다. 진단 완결 여부는 모델 성능 PASS나 향상이 아니다. 현재 가중치·원본·V2 좌표 실패/수정 자료는 수정하지 않는다.

## 실행 승인과 자원

새 검증기: [diagnose_map_sign_components_20261007_v1.py](/home/a/autoware_e2e/tools/e2e/diagnose_map_sign_components_20261007_v1.py). fake/stdlib 시험: [test_diagnose_map_sign_components_20261007_v1.py](/home/a/autoware_e2e/learning/tests/test_diagnose_map_sign_components_20261007_v1.py).

ROOT 전체 소스·시험 검토 및 별도 순수시험 후 새 고정 authority가 필요하다. authority는 `storage/experiments/plans/2026-10-07/map_sign_components_v1/ROOT_authority.json`의 원본 파일 SHA를 외부 CLI로 전달하고, 소스8개·예측6개·target2개·manifest·results 원본 SHA와 고정 contract·단일 CPU를 결합한다. 호출자 bool이나 기본 contract 출력은 실제 실행 권한이 아니다. 실제 admission 후 전체 입력/소스를 before/after 봉인하고 authority 원본 바이트 및 소스 전체를 새 출력에 보관한다.

기존 `storage/assets/locks/autoware_e2e_runtime.lock`의 읽기 전용 FD로 비차단 exclusive flock을 직접 잡아 DDS/다른 공유 작업과 겹치지 않는다. 외부에서 같은 잠금을 중복 flock으로 잡아 자기 차단시키지 않는다. `PYTHONNOUSERSITE=1`, `PYTHONDONTWRITEBYTECODE=1`, CUDA 빈값, OMP/MKL/OpenBLAS 각각1, 기존 system NumPy1.21.5/SciPy1.8.0, ROOT가 선언한 현재 가능한 단일 CPU affinity를 import 전에 설정한다. 설치·GPU·ROS·CARLA·네트워크 작업은 없다. NumPy/SciPy 전체 바이너리/OS sandbox 봉인을 주장하지 않는다.

전체 분석과 종료 source/input 봉인에 SIGALRM **900초**를 적용한다. 작은 실패 영수증 작성은 deadline 뒤일 수 있으며 hard subprocess watchdog이라고 주장하지 않는다. `/proc` 대신 `ru_maxrss`로 매 load/frame/봉인 시 최고 RSS를 확인해 **512MiB 초과는 실패** 처리한다. 이 sampled peak guard는 할당 전 OS RSS 강제 장벽이 아니다. 최대 target1개+prediction1개씩 처리하며 float RGB 텐서를 만들지 않는다. 공유 가용 바이트 **2GiB floor+64MiB reserve**를 유지하고 전체 신규 출력 **2MiB 이하**를 prewrite 검증한다.

출력은 `storage/experiments/diagnostics/2026-10-07/map_sign_components_v1`만 create-only로 작성한다. 기존 폴더가 있으면 재시도/이어 쓰기를 거부한다. 실패해도 전체1,050개의 PASS/FAIL/UNEXECUTED 상태와 오류·현재까지 실제 숫자를 보존한다. 모델 inference0/train0/GPU0이고 oracle depth/pose도 이 진단에서 읽지 않는다. 데이터가 기존 학습/선별 이력에 노출된 사실은 지우지 않는다.

ROOT 승인 후의 재현 명령(아직 실행 금지):

```bash
CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -B tools/workspace_python.py tools/e2e/diagnose_map_sign_components_20261007_v1.py \
  --mode run --authority-sha ROOT_RAW_AUTHORITY_SHA256
```

실제 벽시계 예상 시간은 아직 계측하지 않아 제시하지 않는다. 900초는 성공 예상 시간이 아니라 고정 실행 한도다. crop/물체/HD-AMG의 원인·기능 주장이나 자동 승격은 하지 않는다.
