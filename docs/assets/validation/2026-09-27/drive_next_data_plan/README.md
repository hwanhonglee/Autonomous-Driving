# 다음 이동 우회전 자료: 경로 예산 감사와 진입 조건

<!-- HH_260906 - Keep prospective route geometry separate from captured training evidence and preserve sealed TEST. -->

**현재 판정: `BLOCKED_STATIC_ROUTE_BUDGET`.** 기존 카탈로그의 우회전 파일만으로는 이번에 검토한 9 km/h 또는 기존 cohort의 15 km/h 기준에서 **첫 anchor 전 30 m + 이동 anchor 30초 + 모든 anchor의 6.4초 미래**를 확보할 수 없다. 아래 TRAIN/DEV 경로는 새 경로를 설계할 때 보존할 **회전 구간 후보**이지, 현재 파일 그대로 수집하거나 학습에 편입할 수 있는 경로가 아니다. 이 감사에서 CARLA·SSH·GPU·모델·기준은 실행하거나 변경하지 않았고, Town04/Town15 TEST payload는 열지 않았다.

## 왜 다음 단계가 자료인지

- [K1 학습 기록](../../../../portable-e2e-k-recovery-execution-2026-09-22.md)과 [K0/K1 결과](../../2026-09-22/portable_e2e_learning_cycle_v1/01_full_model_recovery_k/README.md): K1은 기존 TRAIN 1,475개에 **직진 4 episode 1,312개**를 더한 2,787개로 학습했다. 같은 Town03 DEV 665개에서 직진 ADE는 2.6964→0.5113 m로 좋아졌지만 우회전 ADE는 3.2701→3.8325 m로 악화했다. 우회전 TRAIN의 독립 물리 episode는 여전히 하나였다. K1의 우회전 최선 후보 ADE 1.2991 m와 실제 선택 ADE 3.8325 m 사이의 2.5334 m는 분석용 선택 gap이며, 최선 후보는 미래 정답을 본 oracle이다.
- [AH 결과](../../2026-09-26/dual_model_recovery_ah/README.md): 같은 TRAIN 2,787개, 같은 Town03 DEV 665개에서 직진 ADE 0.3641 m, 우회전 ADE 3.5230 m로 개선됐지만, 우회전 ADE/FDE/속도 오차는 각각 **3.5230/10.1950 m/1.4859 m/s**로 원래 기준 **2.9353/7.7939 m/1.1076 m/s**를 모두 놓쳤다. 절대 기준은 6/9이고 한 seed·반복 DEV 결과다.
- [기존 M 수집](../../2026-09-22/portable_e2e_learning_cycle_v1/04_right_diversity_m/README.md)의 Town01 p02 487개 중 물리 목표 검사 통과 266개는 모두 현재 속도 ≤0.5 m/s였다. 이를 이동 우회전 보강으로 셀 수 없다.
- [AN](../drive_control_an/README.md)은 Town01 p03 BasicAgent 회차의 전체 6.4초 미래 XY 곡률 초과가 고유 100 ms 시점 6개 남았고, [AO](../drive_control_ao/README.md)는 10개로 늘었다. AO에는 2 m/s 이상 회전 초과도 1개 있다. 두 원본 모두 TRAIN 불가이며, 학습 모델 주행 결과가 아니다.

## TEST를 제외한 우회전 카탈로그 검사

2026-09-01 소유 경로 카탈로그에서 Town04/Town15 내부를 제외한 **17개 map 폴더의 우회전 JSON 50개**를 읽었다. 최장은 Town10HD_Opt p05 **116.655 m**, 다음은 Town07 p04 **115.493 m**였다. 정속 기준의 보수적 경로 예산 `30 m + 속도 × (30 + 6.4)초`는 15 km/h에 **181.667 m**, AN/AO의 9 km/h에 **121.000 m**다. 두 기준을 만족하는 기존 우회전 카탈로그 파일은 각각 **0개**다. 더 낮은 속도는 정적 거리 계산을 바꿀 수 있지만, 독립 episode·충분한 실제 이동·제어 품질·미래 라벨 품질을 입증하지 않으며 사후 속도 선택으로 기존 실패를 승인할 수 없다. [기존 비실행 cohort 계획](../../../../../storage/experiments/training/2026-09-15/moving_cohort_v1/cohort.json)도 모든 우회전 slot을 `ROUTE_EXTENSION_OR_SPEED_PLAN_NEEDED`, 독립성을 `NOT_CERTIFIED`, 실행을 `false`로 둔다.

기존 [연구용 TRAIN/DEV dataset manifest](../../../../../storage/datasets/prepared/carla-common10-trainval-research-20260914-v1/dataset.json)의 실제 우회전은 **TRAIN Town01 `right_s0001_p00` 534개**와 **DEV Town03 `right_s0000_p00` 337개**다. 각 준비 데이터의 경로 polyline은 약 **228.25 m / 208.97 m**로 위의 *짧은 신규 카탈로그 후보*와 다른 경로다. 이미 사용한 원본을 새 독립 우회전으로 다시 셀 수 없다.

| 역할과 회전 구간 원본 | 정적 근거 | 현재 차단점 |
| --- | --- | --- |
| TRAIN `T01-R01` [Town01 right p06](../../../../../storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town01/catalog/turn/routes/town01/right/town01_right_s0000_p06.json), SHA256 `6cc48c29b67a69f2be1469ed155d542b1da1bc06f4fa33c95d36ae903fbbbf29` | 85.346 m; RIGHT 약 32.63–48.39 m; 정적 회전 preflight PASS. **실제 기존 TRAIN** `right_s0001_p00` 준비 데이터 polyline과 waypoint 기준 전체 최단 약 73.66 m, p06 회전 부분 최단 약 78.31 m. | 15 km/h 계획의 anchor span 약 6.88초뿐이다. 현재 파일의 실제 주행과 route group 독립성은 인증되지 않았다. |
| DEV `V03-R01` [Town03 right p07](../../../../../storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town03/catalog/turn/routes/town03/right/town03_right_s0000_p07.json), SHA256 `d0b060733377d2414ebe99ea3b3943581c48930fa5b491b0e49ac9b10af39ead` | 86.516 m; RIGHT 약 44.54–65.84 m; 정적 회전 preflight PASS. **실제 기존 DEV** p00 준비 데이터 polyline과 waypoint 기준 전체 최단 약 74.18 m, p07 회전 부분 최단 약 94.83 m. | 15 km/h anchor span 약 7.16초이고 우회전 뒤 남은 약 20.67 m가 6.4초 정속 미래 26.67 m보다 짧다. 기존 suite의 이 경로 `train` 표기와 연구용 DEV 배정이 충돌해 split 잠금이 먼저 필요하다. |

위 거리는 직렬화 waypoint의 탐색 값으로, 차선 다각형·연결도로·확장 경로의 비중복 증명이 아니다. p06과 기존 K1 Town01 직진 p02의 경로 waypoint 최단은 약 18.3 m여서 **확장 설계 때** 기존 TRAIN과 같은 연속 주행이나 교차 site가 되는지도 재검사해야 한다. p07은 Town03에서 여러 번 관측한 개발 맵의 새 site 후보이므로 독립 release TEST가 아니다. 로컬 prepared dataset의 `dataset.json`/`episode.json`에서 이 두 정확한 route ID가 검색되지 않았다는 사실도 전체 수집 이력 부재의 증명은 아니다.

## 수집 전 고정할 계약

1. **경로와 split:** 위 원본 회전 구간을 기준으로 실제 연결 가능한 선행·후행 도로를 연장하고, 새 route ID/SHA·회전 위치·속도 계획·선택 anchor 범위·source session을 사전 고정한다. 확장본의 정적 turn preflight, 경로 길이, 차선 연결, 신호/정지 이유, 기존 TRAIN·DEV의 연속 주행/route/site 중복을 확인한다. Town01은 TRAIN, Town03은 DEV로 잠그며 기존 Town03 연구 DEV와 섞어 독립 Town TEST라고 부르지 않는다. Town04/Town15 TEST는 계속 미사용이다.
2. **원본 수집:** 소유 CARLA에서 BasicAgent와 단일 사전 선언 조건으로 수집하고 원본 6카메라, 20 Hz 차량 상태/명령, 10 Hz 동기 영상, 보정·시간·명령 ACK·warmup/tail을 모두 보존한다. 30 km/h는 상한이고 실제 회전 속도는 곡률·횡가속도에 맞춰 미리 계획한다. 새 경로의 물리 경계와 주행 결과를 보기 전에는 편입 수를 정하지 않는다.
3. **episode 전수 QA:** 6카메라 JPEG 디코딩, 상태/영상/명령 정렬, 선택 anchor의 연속 ≥30초와 매 anchor의 **64개 미래 점/6.4초**, 진입·회전·이탈에서 실제 **>0.5 m/s 이동** 표본 수, 충돌/차선 침범과 별도의 **차체 전체 의도 차로 포함**, 단조 route progress, 속도·가감속·yaw·XY 곡률을 원래 한계로 검사한다. 모든 미래 창의 위반이 0이어야 한다. 저속 실패 창 삭제, target clipping, 다른 기준점·시간축으로의 소급 변환, 한계 완화는 하지 않는다.
4. **편입 뒤 평가:** 합격한 독립 episode만 고정 TRAIN/DEV manifest에 추가하고 route/site/source-session 단위 중복을 다시 검사한다. 기존 K1/AH 직진 보존과 우회전 ADE/FDE/속도 및 후보 선택 gap을 같은 사전 기준으로 비교한다. 한 쌍의 기술 pilot만으로 채택하지 않으며, 반복 seed와 미사용 TEST, 10 Hz shadow/제한 CARLA 폐루프는 이후 별도 gate다.

**이번 회차 결론:** 기존 경로 그대로의 수집·변환·TRAIN 편입은 **차단**이다. 다음 작업은 p06 TRAIN과 p07 DEV의 연결 가능한 **새 연장 경로 설계 및 분리 검증**이며, 실제 수집 가능 여부는 그 새 경로와 실행 전 계약을 통과한 뒤에만 결정할 수 있다.
