# Town05_Opt 지도 모델 교차-Town DEV 평가 — 실행 전 고정 계획

<!-- HH_260906 - Freeze source, denominators, models and non-promotion rules before opening Town05 predictions. -->

**상태: 사전 등록만 완료. 이 문서는 Town05 모델 추론·점수·학습 결과가 아니다.** 목적은 이미 보존된 Town05_Opt Epic 수집의 *관측된 첫 약 94.3 m*에서 기존 지도 AI v2와 미채택 후보 C의 도로·차선 의미 분할을 같은 입력으로 비교하는 것이다. 새 체크포인트 학습, CARLA 재실행, 원격 GPU 사용, 정밀지도 승인, learned-control 주행 평가는 이 단계에 없다.

## 평가 자료와 동결 모델

| 역할 | 정확한 대상과 SHA-256 | 제한 |
|---|---|---|
| Town05 DEV 입력 | `storage/experiments/training/2026-09-27/map_town05_independent_dev_capture_epic_v2/mapping/`의 원본 640×360 RGB PNG 240장과 CARLA semantic NPZ 240장; [수집 감사](../map_town05_dev_capture/epic_v2_audit.json) SHA-256 `d64d77bf4f1d02892f59ac9debd18489c5acbf38842ec81ac4398f7052b950d1` | 동일 Town05_Opt 직진 p00 경로의 첫 94.305 m. 209.773 m 경로 완주, 여러 날씨·경로, 맹검 TEST를 뜻하지 않는다. |
| 경로·수집 영수증 | route SHA-256 `efb8ccca533f25cae54b2eb7e03274c15795211488128185334ff2bc55f4744c`; owner plan `aed353013b25394be8f6594283e2fb8dfccfece10dcb4a06ab75991a82dabf8a`; mapping summary `99dcd8a68b36f0f24611c512f2471139f9f155ea222b697ce899090239376c89` | Town05_Opt, `quality=Epic`, BasicAgent 수집, 충돌 0, live/replay 정확 일치, 소유 실행 종료 0. 먼저 [Epic 수집 계획](../map_town05_dev_capture/PLAN_V2.md)과 감사의 원본 자격을 다시 확인한다. 첫 Low 회차는 평가에서 제외한다. |
| A: 기존 AI v2 | `storage/experiments/training/2026-09-26/dual_data_ai_v1/mapping_ai_v2_checkpoint/checkpoint.pt`, SHA-256 `ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380` | 동결된 기준 모델. |
| C: Town01:Town03 7:1 복습 | `docs/assets/validation/2026-09-27/map_town_rehearsal_c/remote_run_v2/checkpoint.pt`, SHA-256 `864de725655ef7219218f2b4a9da059630e6e3c30a18a010f74238938aa4a7f6` | [고정 Town01 DEV 보호 관문](../map_town_rehearsal_c/README.md)을 이미 통과하지 못한 미채택 후보. 이 평가로 과거 실패를 덮어쓰지 않는다. |
| 모델 구현 | `learning/portable_e2e/map_perception.py`, SHA-256 `1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b` | 두 체크포인트 모두 같은 7클래스 구조·130,503 파라미터여야 한다. |

이 자료의 분할은 **개발 DEV 후보**다. Town01·Town03은 기존 지도 TRAIN, Town05_Opt은 모델 비교·개발에만 사용하고 TRAIN이나 최종 TEST에 넣지 않는다. 같은 Town05_Opt site의 주행 우회전 자료도 development VAL에 있으므로 향후 통합 모델은 site 전체를 개발 쪽으로 취급한다. 지도 Town07과 주행 TEST 예약 Town04·Town06·Town10HD_Opt의 RGB·라벨·예측을 이번 평가에서 열지 않는다.

## 추론 전에 실패 종료할 검증

1. 수집 감사의 `CAPTURE_AUDIT_PASS_NOT_ADMITTED`, 소유 영수증·route·world·Epic 품질·동기 240쌍·프레임 순서·충돌 0·정리 PASS·live/replay 일치 및 아카이브 수집 소스 13개의 해시를 원본과 대조한다. PNG가 RGB 640×360, semantic가 `uint8 (360, 640, 4)`인지 전수 검사한다. 감사의 프레임별 PNG/NPZ SHA-256도 대조한다. 하나라도 불일치하면 점수를 만들지 않는다.
2. 두 체크포인트와 모델 구현의 SHA-256, 체크포인트의 7클래스 순서·입력 shape `[3, 360, 640]`·2,000 step·`deployable=false`, 엄격한 `state_dict` 적재와 파라미터 수를 검사한다. 신뢰할 수 있는 위 소유 파일만 CPU로 읽는다.
3. 모델 입력은 원본 RGB를 `float32 / 255`로 만든 것뿐이다. CARLA semantic tag는 정답 생성에만 사용한다. 기존 [Town06 교차-Town 평가기](../../../../../tools/e2e/evaluate_map_town06_cross_town.py)의 고정 매핑 `{1:road, 24:lane_marking, 2:sidewalk, 7:traffic_light_fixture, 8:traffic_sign, 6:pole}`과 7클래스 순서를 그대로 사용한다. 이미지 crop·resize·증강·confidence threshold·후처리·모델 선택은 하지 않는다. CPU 2 thread, `CUDA_VISIBLE_DEVICES=''`로 고정한다.

## 분모·지표·그림 선택

- **주 평가 분모**는 [수집 감사](../map_town05_dev_capture/epic_v2_audit.json)에 동결한 `retained_stems` 232장이다. 첫 프레임부터 마지막 보존 차량 XY 기준 0.25 m 이상 떨어진 프레임을 보존한다. 원본 240장 전부의 결과도 별도 표기하며 이를 주 분모와 섞지 않는다. 두 모델은 같은 프레임과 같은 정답으로 평가한다.
- 정답 행·예측 열의 7×7 픽셀 혼동행렬을 원본 240장과 보존 232장 각각 합산한다. **주 지표:** 도로와 차선 표시 각각의 IoU, precision, recall, TP/FP/FN 및 C−A IoU의 퍼센트포인트 차이. 도로↔`other`/보도, 차선→`other`/도로와 차선의 표지판 오검출도 진단한다. 7클래스 평균 IoU와 전체 픽셀 정확도는 부지표로만 보고한다. 동일 경로의 연속 프레임을 독립 표본처럼 취급하여 통계적 유의성을 주장하지 않는다.
- 표지판 정답은 보존 232장 합계 **256픽셀**이며 원본 240장 각각 **1 또는 2픽셀**이다. 따라서 표지판 IoU·precision·recall·오검출 수는 **진단용 수치만** 남기고 C의 합격·순위·학습 데이터 채택 근거로 쓰지 않는다. 표지판 개체 검출률이라고 부르지 않는다.
- 전체 화각 화면은 모델 예측을 보기 전에 정한 보존 프레임 순번 **0, 115, 231**인 `frames/0000`, `frames/0123`, `frames/0239`를 사용한다. 각 화면에서 입력 RGB / CARLA oracle / A / C의 원본 640×360 전체 화각을 표시한다. 결과가 잘 나온 프레임만 골라 교체하지 않는다. 이미지와 예측 배열의 입력 SHA·체크포인트 SHA·평가기 SHA·실행 환경을 결과에 기록한다.

## 판정 경계 — 자동 채택 없음

- Town05 주 지표의 *진단상 비퇴행 표식*은 **C의 도로 IoU ≥ A−1.0 pp 그리고 차선 IoU ≥ A−2.0 pp**일 때만 참이다. 기준을 관측 후 바꾸지 않는다. 이는 외부-Town 경향을 해석하기 위한 표식이며 배포·체크포인트 승격 조건이 아니다.
- C는 기존 동일 Town01 DEV에서 차선 **−2.528 pp**, 표지판 IoU **−0.888 pp**, 표지판 precision **−0.952 pp**로 사전 보호 관문에 실패했다. 따라서 Town05에서 주 지표가 좋아져도 **C의 `automatic_promotion=false`, `deployment_approved=false`는 유지**한다. Town05 실패 시에도 역으로 A가 실차 적합 또는 정밀지도 품질을 갖췄다고 주장하지 않는다.
- 향후 새 후보를 채택하려면 독립적으로 사전 등록한 Town01 DEV 보호 조건, 추가 교차-Town 개발/분할 검증, 봉인된 최종 TEST의 일회성 평가가 모두 필요하다. 이 문서만으로 Town07 봉인을 풀거나 학습을 시작하지 않는다.

## 실행 가능성 및 예상 소요

기존 Town06 CPU 평가기는 원본 해시 확인, 7클래스 혼동행렬, 전체 화각 패널 생성 코드를 갖지만 **Town06 경로·감사·후보 A/B가 하드코딩**되어 있어 그대로 실행하면 안 된다. 별도 Town05 전용 평가기를 만들고 이 계획의 입력/체크포인트를 잠근 뒤 단위검사·드라이런을 거쳐야 한다. Town06의 240장×2모델 CPU 추론은 약 60초였으므로 같은 크기의 이 비교는 추론 자체 약 **1–3분**, 구현·테스트·감사·시각 점검 포함 **30–60분 정도**로 예상한다(보장 시간이 아니다). 현재 단계에서 PRO6000, GPU1, CARLA 포트, 패키지 설치가 필요하지 않다.
