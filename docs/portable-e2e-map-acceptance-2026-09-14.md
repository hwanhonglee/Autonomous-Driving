# Portable E2E — 전체 맵·직진/회전 승인표와 다음 데이터 계획

<!-- HH_260906 - Separate local map availability, historical controller evidence, and new learned-model acceptance without opening held-out payloads. -->

## 결론: 전체 모델 6회 학습은 전체 맵 검증이 아니다

9월 14일의 6회는 **같은 데이터에서 손실 A/B × 초기화 seed 3개**를 비교한 fresh fit이다.
TRAIN은 Town07 직진 309 + C-track 좌회전 304 + Town01 우회전 534 = **1,147표본·3 episode**,
개발 VAL은 Town03 우회전 **337표본·1 episode**다. 겹치는 미래 구간이 있는 표본 수를
독립 주행 횟수로 세지 않는다. 새 모델은 세 seed 모두 장기 경로 오차 기준에 미달했고,
B의 seed04 ADE도 A보다 악화했다. 운용 체크포인트를 바꾸지 않았다.

[실제 6회 학습·평가·72개 경로 PNG](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/05_stopmix_regret_learning/README.md)는
학습 수행 증거이지 모든 Town·직진/회전·정지 기능을 충분히 검증했다는 증거가 아니다.
상위 **9개 영역·하위 30개 요구사항**, 부분 근거 **10/30**, 학습 모델 직접 주행의
**learned closed-loop 통과 0/30**은 그대로다. 아래 모든 맵에서도 새 Portable 모델의
직접 제어 통과는 **0**이다.

## 무엇을 확인했나

이번 문서는 로컬 파일만 읽어 확인한 현황과 앞으로의 제안이다. CARLA 접속·월드 변경·실행,
설치·학습·TEST payload 열람을 하지 않았다. 기존 결과나 split을 수정하지 않았다.

- [정규 맵 목록](../tools/e2e/carla_expert_suite.yaml): Town 변형과 커스텀 맵을 포함한 **19개 등록 항목**.
  이것을 CARLA의 모든 배포판·모든 사용자 보유 맵을 검색한 목록이라고 확대하지 않는다.
- [정적 자산 캐시](../storage/experiments/validation/2026-08-31/maps/carla_map_inventory.json)의 선언 경로를
  현재 파일시스템에서 다시 확인했다. `ready` 10개 중 Woraksan을 제외한 9개는 level/XODR가 있고,
  소스 에디터용 4개도 선언된 소스 level/XODR가 있다. 파일 존재는 오늘의 부팅 성공과 다르다.
- [Autoware 맵·경로 매트릭스](../tools/e2e/autoware_vad_town_matrix.yaml)는 9개의 full-map 묶음만
  바인딩한다. [Town 묶음](../tools/e2e/packaged_town_full_maps.yaml)과
  [커스텀 묶음](../tools/e2e/custom_map_bundles.yaml)의 용도·좌표계 제한도 별도다.
- [9월 1일 v16 경로 카탈로그](../storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps)의
  18개 직진/회전 카탈로그에 연결된 **182개 경로 파일**을 재해시했고 SHA·scenario 불일치가 없었다.
  직진 72·좌회전 55·우회전 55개는 기존 기하 후보이며, 독립 수집 episode나 새 모델 PASS 수가 아니다.

## 등록된 19개 맵을 빠짐없이 추적

<!-- HH_260906 - Preserve blocked entries and stale-manifest disagreements rather than shrinking the all-map denominator. -->

`준비`는 지정 packaged 자산·Autoware 맵 바인딩·기존 직진/회전 경로가 있다는 뜻이다.
아래 9개 준비 맵의 과거 VAD 성공을 새 Portable 모델의 성공으로 승계하지 않는다.
`차단`도 전체 목록에서 삭제하지 않으며 해결 근거가 생기기 전에는 PASS로 세지 않는다.

| 정규 항목 | 현재 파일·실행 준비 근거 | 기존 선택 경로: 직진 / 회전 |
| --- | --- | --- |
| Town01 | packaged 준비 | 직진 / 좌회전 |
| Town02 | 표준판 렌더러 실패 기록; Opt를 명시적으로 사용 | 표준판 차단 |
| Town02_Opt | packaged 준비 | 직진 / 좌회전 |
| Town03 | packaged 준비; 현재 개발 VAL 맵 | 직진 / 좌회전 |
| Town04 | packaged 준비; **현재 v3 TEST 잠금** | 과거 VAD 직진 / 좌회전만 별도 근거 |
| Town05 | 표준판 SceneCapture 렌더러 실패 기록; Opt 사용 | 표준판 차단 |
| Town05_Opt | packaged 준비 | 직진 / 좌회전 |
| Town06 | packaged 준비 | 직진 / 우회전 |
| Town07 | packaged 준비 | 직진 / 좌회전 |
| Town08 | 정규 목록에 로컬 배포 자산 없음으로 기록; 실행 바인딩 없음 | 차단 |
| Town09 | 정규 목록에 로컬 배포 자산 없음으로 기록; 실행 바인딩 없음 | 차단 |
| Town10HD | 표준판 미검증; packaged 실행은 Opt를 사용 | 표준판 차단 |
| Town10HD_Opt | packaged 준비 | 직진 / 우회전 |
| Town11 | 소스 level/XODR 존재, source editor 필요; 미장식 POC로 시각 학습 제외, admitted full-map 없음 | 차단 |
| Town12 | 소스 level/XODR 존재, source editor 필요; admitted Autoware full-map 없음 | 차단 |
| Town13 | 소스 level/XODR 존재, source editor 필요; admitted Autoware full-map 없음 | 차단 |
| Town15 | 소스 level/XODR 존재, source editor 필요; 정규 목록의 held-out, admitted full-map 없음 | 차단 |
| C_track_1_0_7 | packaged Epic·simulation XODR full-map 준비 | 직진 / 좌회전 |
| Woraksan_v1_0_3_parking_lot_hegiht_fit (`woraksan_1_0_3`) | **목록 `ready`와 현 자산 불일치**: 지정 packaged level/XODR 둘 다 없음; 매트릭스가 명시적으로 차단 | 차단 |

Woraksan의 `ready` 설명에는 과거 전문가 수집 성공이 남아 있지만, 이것만으로 현재 실행을
허용하면 안 된다. 반대로 지정 packaged 경로가 없다는 이유로 다른 위치의 소스·RoadRunner
자료까지 없다고 단정하지 않는다. Town11/12/13/15도 소스 자산 부재가 아니라 **실행 환경과
Autoware 지도 승인 부족**이다. 자산 복구·변환·새 설치가 필요하면 별도 범위와 권한을 정한다.

## 실행 준비 9개: 경로의 실제 방향과 후보 수

선택 경로 원문은 각 맵의 `docs/assets/validation/2026-09-01/<map>/autoware_vad/`
아래 `straight/autoware_vad_route.json`, `turn/autoware_vad_route.json`이다.
경로 길이는 직진과 회전을 따로 표시했다. 회전 폴더 이름만으로 좌·우 양쪽을 통과했다고
읽지 않는다. 후보 수는 위 v16 기하 카탈로그 기준이다.

| 맵 | 선택 직진 길이 (m) | 선택 회전·길이 (m) | 기하 후보: 직진 / 좌 / 우 |
| --- | ---: | --- | ---: |
| [Town01](assets/validation/2026-09-01/town01/autoware_vad/straight/autoware_vad_route.json) | 210.180 | [좌 81.272](assets/validation/2026-09-01/town01/autoware_vad/turn/autoware_vad_route.json) | 8 / 8 / 8 |
| [Town02_Opt](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/autoware_vad_route.json) | 177.132 | [좌 83.606](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/autoware_vad_route.json) | 8 / 8 / 3 |
| [Town03](assets/validation/2026-09-01/town03/autoware_vad/straight/autoware_vad_route.json) | 209.523 | [좌 73.122](assets/validation/2026-09-01/town03/autoware_vad/turn/autoware_vad_route.json) | 8 / 7 / 8 |
| [Town04](assets/validation/2026-09-01/town04/autoware_vad/straight/autoware_vad_route.json) | 210.010 | [좌 70.395](assets/validation/2026-09-01/town04/autoware_vad/turn/autoware_vad_route.json) | 8 / 8 / 5 |
| [Town05_Opt](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/autoware_vad_route.json) | 209.773 | [좌 75.023](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/autoware_vad_route.json) | 8 / 8 / 8 |
| [Town06](assets/validation/2026-09-01/town06/autoware_vad/straight/autoware_vad_route.json) | 210.485 | [우 88.108](assets/validation/2026-09-01/town06/autoware_vad/turn/autoware_vad_route.json) | 8 / 0 / 1 |
| [Town07](assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json) | 210.598 | [좌 82.614](assets/validation/2026-09-01/town07/autoware_vad/turn/autoware_vad_route.json) | 8 / 8 / 6 |
| [Town10HD_Opt](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/autoware_vad_route.json) | 171.220 | [우 79.756](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/autoware_vad_route.json) | 8 / 0 / 8 |
| [C-track](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/autoware_vad_route.json) | 209.470 | [좌 73.858](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/autoware_vad_route.json) | 8 / 8 / 8 |

즉 직진/대표 회전은 **18개 경로 칸**이 준비되어 있지만, 좌·우를 분리하면 27칸 중
Town06 좌회전과 Town10HD_Opt 좌회전은 이 카탈로그에서 후보가 없다.
검색 결과 0은 해당 맵 어디에도 좌회전이 없다는 증명이 아니다. 새 경로 탐색·기하 검증이
필요하며 기존 기준을 낮춰 칸을 채우지 않는다. 짧은 Town02_Opt·Town10HD_Opt 직진도
속도 노출 기준을 자동 통과하지 않는다.

## 같은 화면이라도 증거 종류가 다르다

| 증거 | 실제 제어 주체·측정 대상 | 새 Portable 모델에 인정할 수 있는 것 |
| --- | --- | --- |
| CARLA BasicAgent 전문가 수집 | BasicAgent/선언된 전문가 제어; 실제 카메라·상태·제어 ACK | 데이터 후보와 수집 QA만; 학습 모델 주행 PASS 아님 |
| 과거 Autoware VAD | VAD 경로와 Autoware 제어; 일부 속도 실험은 명시적 nominal 속도 프로파일 | 기존 스택의 해당 조건 실행; 새 체크포인트나 VAD 속도 예측 성능으로 승계 불가 |
| Portable live shadow | 실제 6카메라 입력·예측·10 Hz 지연; 차량 제어는 Autoware VAD | **그때 고정된** physical-v1의 shadow 계측만; 새 STOPMIX 6개 모델의 runtime/폐루프 근거 아님 |
| 새 Portable offline 평가 | 저장된 TRAIN/VAL 입력에서 경로·속도·후보 점수 계산 | ADE/FDE·속도·subgroup 결과; 화면은 예측 그림이지 Autoware 직접 주행 영상이 아님 |
| 앞으로의 Portable learned closed-loop | 고정된 학습 경로가 실제 tested planning output을 소유하고 ego를 제어 | 사전 기능 gate·전체 실패 분모·안전/10 Hz/원시 증거까지 통과한 칸만 승격 |

현재 shadow 근거는 [Town07 직진·C-track 좌·Town03 좌 3장면](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/README.md)과
[후속 제어 A/B](validation-2026-09-07-control-ab.md)다. Town03 **우회전 VAL**과는 다른 episode다.
[최신 Town07 v5 파일럿](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/06_brake_hold_pilot/README.md)은
실제 BasicAgent 수집으로 정지 유지까지 관측했지만 미래 곡률 **134/666** 구간이 실패해 미편입이다.
어느 카메라 화면에도 “새 모델 자율주행 성공”이라는 이름을 붙이지 않는다.

## 앞으로 사용할 승인 칸과 유지할 요구사항

<!-- HH_260906 - Keep the proposed collection and acceptance ledger prospective; no existing split, gate, or model is promoted by this document. -->

승인표의 한 칸은 **맵·경로 SHA·직진/좌/우·속도 구간·행동/기능 ID·split·날씨/seed·모델 SHA**로
식별한다. 각 칸에서 `MAP_READY → DATA_QUALIFIED → OPEN_LOOP → EXPORT_PARITY → SHADOW →
LEARNED_CLOSED_LOOP`를 분리하고 `NOT_RUN / BLOCKED / FAIL / PASS`와 원문 경로를 기록한다.
수집기·실제 제어 주체·입력 cadence·fallback 개입·실패 포함 횟수도 필수다.
앞 단계 PASS나 빠른 GPU forward를 다음 단계 PASS로 복사하지 않는다.

- **환경:** 원격 학습은 실행 직전 확인한 PRO6000 physical GPU 0과 지정 개인 venv만 사용한다.
  다른 GPU·Conda·시스템 설치·타 작업 종료·원격 Autoware 빌드를 하지 않는다.
  로컬은 CARLA·Autoware·원시 데이터·실행 증거 담당이다.
- **10 Hz:** 실제 native 6카메라와 bundle coverage를 검증한다. world 20 Hz, 저장 카메라 10 Hz,
  모델 forward, sensor-to-output/actuator 지연을 구별한다. 과거 5 Hz 카메라 실험이나
  20 Hz 상태를 줄여 그린 10 Hz 그래프로 새 모델의 10 Hz 입력 성능을 대신하지 않는다.
- **전체 맵·별도 회전:** 19개 등록 항목을 모두 추적하고 준비된 9개부터 직진/좌/우를 분리한다.
  막힌 맵은 이유를 남긴다. Town04의 과거 VAD 경로를 열람하는 것과 현재 v3 TEST 정답·영상에
  접근하는 것은 다르다. **Town04 TEST payload·모델 평가·선택 잠금은 그대로**이며,
  이번 문서는 Town04 새 개발 수집이나 학습 편입을 허가하지 않는다.
- **30 → 60 km/h:** 먼저 30 km/h 이하 Portable 기준선을 검증한다. 30은 모든 회전에서 강제로
  유지할 속도가 아니다. 곡률·횡가속·정지거리·fallback을 만족하는 속도로 회전하고,
  60 km/h는 새로운 경로 길이·제동거리·센서/지연·안전 조건을 고정한 후 별도 승인한다.
  과거 VAD 60 km/h 실적을 새 Portable 60 km/h 학습/주행 실적으로 세지 않는다.
- **실제 화면:** 새 장면마다 실제 CARLA/Autoware 캡처임을 명시하고 원본 카메라·시간·제어 주체,
  ego 중심 기준 경로와 기록 궤적을 표시한다. 생성 영상·수정된 주행 장면을 증거로 사용하지 않는다.
- **실차+시뮬레이션:** 향후 실측 데이터는 별도 보정·시간·출처·유효 label mask를 검증하고
  domain별 샘플링/혼합 비율 A/B와 별도 평가를 한다. 실측 replay → actuator-off shadow →
  폐쇄 시험장 순서이며 현재 실차 제어·공도 승인이나 real+sim 혼합 학습 완료 주장은 없다.
- **30개 기능:** [기능 매트릭스](../learning/config/portable_e2e_feature_matrix.yaml)와
  [단계별 gate](portable-e2e-feature-roadmap.md)를 유지한다. 일반 기능 최소 20 episode,
  안전 중요 기능 최소 30 episode 및 적용 가능한 seed/날씨·조명 각 3종 등 기존 요구를
  한 번의 맵 smoke로 대신하지 않는다. 기능별 분모와 맵별 coverage를 함께 보고한다.

## 다음 데이터·학습 계획 — 아직 실행·분할 선언이 아닌 제안

1. **현재 실패를 먼저 해결한다.** v5의 준비/출발/저속 곡률과 원시 위치·속도 일관성을
   검토한다. 구간 삭제·pose reset·label retiming·gate 완화 없이 새 수집이 모든 QA를 통과해야
   한다. 이미 거부된 02–06 진단·파일럿 원문을 새 학습 데이터로 포장하지 않는다.
2. **수집 전 새 TRAIN / 개발 VAL / release TEST 계획을 고정한다.** 현재 split을 바꾸지 않고,
   episode·route group·site/day 단위로 분리할 새 목록·경로 SHA·행동·날씨·seed·예산·중단 조건을
   먼저 선언한다. 아래 제안만으로 어느 맵도 새 split에 자동 배정하지 않는다. 반복해서 본
   Town03 개발 VAL은 untouched release TEST가 아니며, 기존 Town04 TEST도 계속 잠근다.
3. **기초 행동의 실제 빈칸을 채운다.** 독립 직진·좌·우, 자연 감속→정지 유지→재출발을
   TRAIN과 개발 VAL 각각에서 확보한다. [데이터 감사](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/01_data_readiness/README.md)의
   VAL 정지 유지 0개, TRAIN 118개가 Town01 한 episode에 몰린 문제, VAL 직진/좌회전 부재,
   ClearNoon 단일 조건을 우선 보강한다. 같은 경로 seed 반복을 새 site로 세지 않는다.
   처음부터 release용 독립 route group도 계획하되 모델/threshold 선택에는 열지 않는다.
4. **정답이 없는 기능을 만들었다고 하지 않는다.** lane/drivable·depth·신호·객체/점유공간과
   정지/재출발 원인을 나타내는 입력·정답은 기록·valid mask 계약을 먼저 정한다. 미기록 label을
   `0`이나 “없음” 정답으로 채우지 않는다. 현재 objects/occupancy 비활성 및 lane/depth/TL
   미기록 상태는 그대로다.
5. **새 자료를 검증한 뒤 전체 모델 비교를 한다.** immutable 데이터 버전·split·SHA → 소수
   TRAIN 표본 학습 가능성 확인 → 같은 조건 3-seed A/B → 절대 경로/속도 기준과 정지·회전·
   지속 주행 subgroup 퇴행 검사 순서다. 6회라는 횟수보다 독립 데이터 범위와 미달 원인이
   중요하며, 같은 작은 자료에서 epoch만 늘리는 것을 다음 목표로 삼지 않는다.
6. **조건을 통과한 단일 후보부터 실제 제어 범위를 늘린다.** export parity·10 Hz shadow·
   reject/fallback·제어 소유권을 확인한 후 제한된 CARLA learned closed-loop를 시작한다.
   첫 행동은 직진·좌·우·종점 정지·속도 추종이며 준비된 맵별 칸을 채우고, 차단 맵은 별도
   복구 계획으로 추적한다. 이후 교통규칙 → ACC → 장애물/AEB → 차선변경/합류 → 저속/주차와
   실차 전환을 [30개 기능 순서](portable-e2e-feature-roadmap.md)에 따라 확장한다.

[9월 14일 전체 상태](portable-e2e-status-2026-09-14.md) ·
[학습·검증 반복 가이드](portable-e2e-learning-loop.md)

<!-- HH_260906 - Link the implemented non-executable planner without changing the historical map/split manifests. -->
[07번 실제 후속 검사](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/07_coverage_and_curvature/README.md)에
새 맵별 planner와 학습 범위 검사를 구현했다. planner는 Town04/Town15 카탈로그도 열지 않고
16개 나머지 카탈로그 metadata의 161후보만 집계한다. 위 과거 VAD 182후보 조사와 검사 범위가 다르다.
95개 장면 유형 칸은 실행·데이터 승인 전 상태이며, 옛 suite의 TRAIN 선언을 자동 상속하지 않는다.
