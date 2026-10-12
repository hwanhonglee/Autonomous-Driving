# 다음 지도 자료 경로: 통합 분할을 지키는 DEV·TRAIN 정찰

<!-- HH_260906 - Keep map data selection aligned with driving site splits and distinguish oracle labels from scouting. -->

**결정 제안:** 신규 지도 DEV는 **Town05_Opt 직진 p00**으로 사전 고정한다. 표지판용 신규 TRAIN 자료는 주행 TRAIN 계열인 **Town02_Opt 직진 p00**에서 먼저 정찰한다. Town02_Opt의 640×360 전방 `traffic_sign` 라벨은 아직 관측되지 않아 **적격 sign-rich TRAIN 경로가 확인됐다는 뜻은 아니다.** Town04·Town06·Town10HD_Opt는 기존 주행 TEST 또는 예약 TEST 사이트와 겹치므로 이 지도 DEV/TRAIN 계획에서 제외한다. 새 CARLA 수집·학습·평가·데이터셋 편입은 하지 않았다.

## 왜 자료를 바꿔야 하나

[후보 C의 고정 Town01 DEV 진단](../map_town_rehearsal_c/diagnosis/README.md)에서 AI v2→C의 차선 TP는 45,308→45,211로 거의 같지만 FP는 **3,745→5,051**로 늘었다. 표지판 TP도 1,262→1,232에 비해 FP가 **37,061→51,386**으로 크게 늘었다. Town03 신규 TRAIN 416장의 차선 라벨은 2,546,611픽셀, 표지판은 140픽셀뿐이다. 라벨 노출과 오검출의 관련성은 가설이며 원인 입증은 아니다. 같은 Town01 DEV의 재사용을 줄이고, 주행과 지도 모델의 **site 분할도 일치**시켜야 한다.

| 용도 | 경로 | 관측 근거 | 아직 모르는 것 |
|---|---|---|---|
| 새 지도 DEV 후보 | `storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town05_opt/catalog/straight/routes/town05_opt/straight/town05_opt_straight_s0000_p00.json` (SHA256 `efb8ccca533f25cae54b2eb7e03274c15795211488128185334ff2bc55f4744c`) | Town05_Opt ClearNoon 209.77m 직진. [과거 경로 검증](../../2026-09-02-all-towns-camera-source-5hz/town05_opt/autoware_vad/straight/matrix_validation.json)은 같은 SHA로 완주 PASS; [다중 카메라 화면](../../2026-09-02-all-towns-camera-source-5hz/town05_opt/expert_overview_1920x1080.png)에 도시 도로와 차선 표시가 보인다. [주행 자료 계획](../../../../../storage/experiments/training/2026-09-08/next_dataset_plan_v1/plan.json)의 Town05_Opt **우회전 p00**(SHA256 `2f7a25e8ace753bfdd412ae7ef65710a001d1f4b1ecd5765f3ee7b021ced29e3`)은 development_val이며, 이 지도 후보는 **직진 p00**으로 다른 경로다. 두 카탈로그 경로 중심선의 최소 XY 거리는 48.24m이나 같은 사이트로 묶는다. | 새 단일 전방 640×360 RGB/semantic 프레임·라벨 분포·모델 성능. 이전 RGB 화면을 보고 선택했으므로 맹검 TEST가 아닌 개발 자료다. |
| 신규 TRAIN *정찰* 후보 | `storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town02_opt/catalog/straight/routes/town02_opt/straight/town02_opt_straight_s0000_p00.json` (SHA256 `520fc0236e32d0664bc125a15f45601e4ef020cf1e3f1db888332af0e188f08b`) | Town02_Opt ClearNoon 177.13m 직진. [과거 경로 검증](../../2026-09-02-all-towns-camera-source-5hz/town02_opt/autoware_vad/straight/matrix_validation.json)은 완주 PASS; [다중 카메라 화면](../../2026-09-02-all-towns-camera-source-5hz/town02_opt/expert_overview_1920x1080.png)에 도시 도로가 보인다. Town02_Opt는 [주행 계획](../../../../../storage/experiments/training/2026-09-08/next_dataset_plan_v1/plan.json)의 P2 TRAIN 사이트이며 9월 15일 이동 코호트에도 같은 경로 ID가 있다. | 전방 영상에 실제 표지판이 있는지, tag 8 픽셀 수·개체 크기·라벨 정합성, 0.25m 보존 프레임 수. 현 시점에는 sign-rich 자격이 없다. |

## 제외한 경로와 분할 근거

- **Town04 직진 p00 제외.** 처음 검토한 경로 SHA256 `ee5335a664392136c7738b96ec639a9cffb116db6d5171e8d7af600a83cc1b6e`은 [주행 held-out TEST 수집기](../../../../../tools/e2e/run_independent_common10_capture_20260907.sh)의 **정확히 같은 원본 경로**다. [주행 TEST 메타데이터](../../2026-09-07/portable_e2e_learning_cycle_v1/03_held_out_test_town04_straight/07_route_alignment.json)에 기록되어 있다. 지도 DEV로 써서 통합 시스템의 site/route 독립성을 훼손하지 않는다. 주행 TEST 영상·정답·모델 출력은 열지 않았다.
- **Town06 직진 p06 TRAIN 제외.** [기존 원본 감사](../map_sign_town06_p06/README.md)에서 0.25m 보존 232장 모두 CARLA tag 8 양성, 합계 **11,630픽셀**로 표지판 의미 라벨 양성량은 확인됐다. 그러나 [기존 주행 자료 계획](../../../../../storage/experiments/training/2026-09-08/next_dataset_plan_v1/plan.json)은 Town06 **사이트**를 P3 예약 TEST에 둔다. 이 경로는 [지도 진단 평가](../map_town06_cross_town_eval/README.md)에도 이미 사용했다. 따라서 이번 통합 분할 아래에서는 TRAIN에 넣지 않는다. 이 원본의 프레임 전체 1–4픽셀 표지판은 **0장**이므로 작은 표지판 목표도 충족하지 않는다.
- **Town10HD_Opt TRAIN/DEV 제외.** 같은 주행 계획에서 이 사이트도 P3 예약 TEST다. 과거 다중 카메라 RGB에서 길가 표지 모양 물체가 보였어도 지도 수집기의 단일 전방 카메라 tag 8 라벨은 미확인이다. 새로운 표지판 TRAIN 경로라는 근거로 사용하지 않는다.
- **기존 Town01 p02/p04는 소형 표지판 TRAIN 대안이 아니다.** [원본 감사](../map_sign_probe/README.md)에서 두 경로 모두 0.25m 보존 후 프레임 전체 tag 8이 1–4픽셀인 장면은 **0장**이고, 두 경로끼리 차량 위치 최소 거리는 **0.017m**다. 서로 독립된 표지판 경로 둘로 세지 않는다.

이는 계획된 site 분할을 지키는 판단이다. 이미 반복 사용한 Town01 DEV는 새 독립 DEV나 TEST가 아니며, 봉인된 별도 지도 평가 원본·정답·예측은 열지 않았다.

## 수집·편입 전 조건

1. Town05_Opt 직진 p00을 **지도 DEV**로 고정하고, Town05_Opt 전체를 지도 TRAIN에서 배제한다. 소유 실행 종료, 원본 센서 쌍·해시, 충돌, 재생 일치와 0.25m 보존 수를 확인한다. 현재 이 경로의 차선·표지판 정답량이나 모델 점수는 없다.
2. Town02_Opt 직진 p00은 **TRAIN 자격 전 원본 정찰**로만 수집한다. 640×360 전체 프레임 tag 8 크기 히스토그램 `0 / 1–4 / 5–64 / ≥65`, 물체별 RGB/라벨 대응, 0.25m 보존, 경로·월드·센서 해시를 검사한다. 실제 양성 표지판이 충분히 관측되지 않으면 sign-rich TRAIN이라는 이름을 붙이지 않는다.
3. 적격 원본이 생겨도 기존 Town01 TRAIN/DEV 바이트와 위치·site 분리를 보존하고, 새 Town의 차선 노출과 표지판 노출을 한 번에 원인으로 단정하지 않는다. 단일 변경 요인·보호 기준·평가 절차를 수치 확인 전에 등록한다.

이 계획 작성 시점에는 `tools/e2e/collect_carla_mapping_probe.py`가 Town05_Opt·Town02_Opt 경로를 허용하지 않았다. 후속 수집 전에 두 경로 SHA와 월드가 정확히 일치하는 좁은 가드·검사가 필요했다. 이 문서는 수집 전 경로 계획이며 독립 TEST·자동 정밀지도·주행 모델 승인을 주장하지 않는다.

**후속 상태:** 위 문단은 수집 전 계획의 원문이다. 이후 Town05_Opt 직진 p00은 별도 소유 Epic v2로 수집하고 [원본 감사](../map_town05_dev_capture/README.md)를 완료했다. 240쌍 중 232장을 0.25m 간격으로 보존했지만 실제 이동 구간은 94.31m이며, 표지판 라벨은 보존 프레임마다 1–2픽셀이다. 지도 모델 예측·학습·데이터셋 편입은 아직 하지 않았다.

**Town02_Opt 후속 상태:** 직진 p00의 별도 [Epic 원본 정찰 감사](../map_town02_train_scout/README.md)는 저장된 240쌍과 0.25m 보존 232장을 확인했다. BasicAgent는 177.13m 경로 중 **94.30m만 이동**했고 목적지에는 도달하지 않았다. 사전 고정한 전체 프레임 태그 8 **1–4픽셀 ≥24장** 기준은 보존 **3장**으로 실패해 표지판 보강용 TRAIN 후보를 **거부**했다. 위 수집 전 계획은 변경하지 않았고, 데이터 편입도 승인하지 않았다.
