# 지도 소형 표지판 TRAIN 정찰: 다음 경로 사전 계획

<!-- HH_260906 - Keep prospective TRAIN scouting separate from qualified labels, development scores, and held-out tests. -->

**상태: 경로 조사만 완료, 수집·학습 미실행.** [Town02_Opt 직진 p00 원본 감사](../map_town02_train_scout/README.md)는 정상 수집됐으나, 사전에 정한 `traffic_sign` 태그 8 **프레임 전체 1–4픽셀** 조건이 0.25m 보존 프레임 **3/232장**으로 최소 24장에 미달했다. 새 경로를 본다고 표지판 학습 문제가 해결된 것은 아니다. 다음 후보의 전방 640×360 RGB와 시맨틱 정답은 아직 관측하지 않았다.

## 순차 정찰 후보

| 순서 | 고정 원본 경로와 SHA256 | 기존 수집과의 기하 | 판단 |
| --- | --- | --- | --- |
| 1 | `Town02_Opt` 직진 `town02_opt_straight_s0000_p02.json`, `cad81f4cdfbc9dbcad8c3cc53092e6fd80f6d84dc4766b6d73758fa88dec6810` | ClearNoon, 173.46m, 시작 `(193.7, -294.6)` → 끝 `(193.7, -121.1)` (ROS XY). 이미 관측한 p00 첫 약 94m 구간과 최소 XY 거리 **197.0m**. | 같은 TRAIN site의 **다른 도로축**이라 p00과 중복 장면을 줄일 수 있다. 도시 도로의 표지판이 전방 화각에 들어오는지, tag 8이 1–4픽셀인지는 **확인되지 않았다**. 1차 원본 정찰만 권고. |
| 2, 조건부 | `Town02_Opt` 직진 `town02_opt_straight_s0000_p03.json`, `1e92b105aceb7f2e100ff603c92ac847d6f6d2cfdbba6321eb3aa25ba7469ffc` | ClearNoon, 173.46m, 시작 `(189.7, -121.1)` → 끝 `(189.7, -294.6)`. p02와 최소 XY 거리 **4.0m**, p00 첫 구간과 **193.0m**. | p02의 **반대 방향·인접 차로**를 보므로 가시 표지판의 면과 거리가 달라질 수 있는 보조 정찰이다. 같은 도로축이므로 독립 site/경로 두 개로 세지 않으며, p02 원본 결과와 중복 구간·물체를 감사한 뒤에만 고려한다. |

원본: [p02 경로](../../../../../storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town02_opt/catalog/straight/routes/town02_opt/straight/town02_opt_straight_s0000_p02.json), [p03 경로](../../../../../storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town02_opt/catalog/straight/routes/town02_opt/straight/town02_opt_straight_s0000_p03.json). 시작·끝·길이·XY 거리는 이 고정 카탈로그의 ROS 경로 좌표만으로 계산했다. **도로 기하에는 표지판의 존재, 방향, 카메라 가시성이 기록돼 있지 않다.** 과거 [Town02_Opt 다중 카메라 화면](../../2026-09-02-all-towns-camera-source-5hz/town02_opt/expert_overview_1920x1080.png)은 다른 주행 시점과 리그의 화면이라 이 두 경로의 표지판 정답 증거가 아니다.

## 실행한다면 바꾸지 않을 기준

1. 별도 실행 전 문서에 경로 SHA, 월드 `Town02_Opt`, ClearNoon, CARLA Epic, 전방 RGB/semantic 640×360·90°·10Hz, BasicAgent 물리 주행, 240프레임, 고유 출력 폴더를 고정한다. 현재 수집기의 좁은 Town02 p00 허용 목록에 p02/p03은 **없다**. 필요한 경우 각 원본 SHA·월드만 허용하는 가드와 단위 테스트를 먼저 추가한다. 신호등 무시를 사용했다면 교통규칙 준수 증거로 쓰지 않는다.
2. 소유 실행 종료 코드 0, 소스·경로 해시 일치, CARLA 수명주기, 충돌 0, 저장 원본/라이브 일치, 240 RGB/semantic 쌍 디코드, 0.25m 분리 보존 **120장 이상**을 원본 무결성으로 확인한다. 약 173m의 **전체 목적지 완주를 240프레임 수집으로 가정하지 않는다.** 미관측 구간을 결과에 포함하지 않는다.
3. 보존 프레임의 태그 8 **프레임 전체 픽셀 수**를 `0 / 1–4 / 5–64 / ≥65`로 세고, **1–4픽셀 프레임 최소 24장**을 충족하지 못하면 해당 정찰을 소형 표지판용 TRAIN에서 거부한다. 충족해도 RGB에서 해당 위치가 물리적으로 독립된 표지판인지, 의미 라벨·색 영상이 정합하는지, 같은 물체의 시간 연속 중복이 얼마인지 별도 확인한다. 프레임 전체 합계와 1–4픽셀 연결 성분·실제 표지판 개체 수는 서로 다른 지표다. 사후에 기존 판정 기준을 바꾸지 않는다.
4. Town02_Opt는 [통합 자료 분할 제안](../../../../../storage/experiments/training/2026-09-08/next_dataset_plan_v1/README.md)의 TRAIN site로만 취급한다. Town05_Opt는 새 지도 DEV, Town03은 주행 DEV, Town04·Town07·Town15는 held-out, Town06·Town10HD_Opt는 예약 TEST로 보호한다. 이들 RGB/라벨/예측을 후보 선정에 사용하지 않는다. 과거 지도 후보 C가 Town03 자료를 이미 TRAIN에 사용한 사실은 남아 있으며, 새 통합 site 분할이 그 누출을 소급해서 없애지는 못한다. Town02 p02/p03끼리도 같은 corridor의 반복 노출을 독립 검증으로 세지 않는다.

**비용·시간의 기준:** CARLA 240프레임은 10Hz에서 **시뮬레이션 24초**다. 바로 앞 Town02 p00 Epic 소유 실행은 기록상 약 **36초 wall time**(09:09:18→09:09:53 UTC), 보관 원본 약 **128MiB**였다. 다음 실행의 시작·시스템 부하에 따라 wall time과 저장량이 달라지므로 보장이 아니다. 경로당 로컬 CARLA 한 번과 전수 감사 시간이 추가되고, 이 *계획만으로는* SSH·GPU·모델 학습이나 별도 외부 서비스 호출이 없다. Codex 대화 크레딧 잔량은 이 저장소에서 읽을 수 없다. p02가 통과하기 전 p03을 자동 실행하지 않는다.

**다음 학습 결정은 별개다.** p02/p03의 정찰 통과는 데이터셋 자동 편입이나 모델 품질 개선을 뜻하지 않는다. 새 Town05 DEV 점수와 기존 Town01 DEV를 같은 고정 모델·측정 코드로 보고, 라벨 오검출 원인을 분리하는 단일 요인 A/B와 보호 기준을 사전 등록한 뒤에만 새 지도 학습을 제안한다. 봉인된 TEST는 선택에 쓰지 않는다.
