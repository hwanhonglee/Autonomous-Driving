# Town01 다음 우회전 경로: 오프라인 차체 여유 선별

<!-- HH_260906 - Keep nominal route geometry separate from a measured BasicAgent or learned closed-loop result. -->

**결론: 현재 V4의 16개 우회전 중 새 물리 회전 수집을 허가할 후보는 0개다.** 이 결과는 [고정 30–120m 창](../drive_long_route_p06_capture/QA_v3/QA.md), [V4 정적 감사](../drive_long_route_prep/v4_candidate_audit.json), 이미 수집한 p06의 [실제 Town01 XODR·Prius 차체 기록](../drive_waypoint_purge_ab/QA_v1/C_lane_gate_30_120.json)을 결합한 **오프라인 선별**이다. CARLA 서버나 GPU를 실행하지 않았고, BasicAgent 새 촬영·데이터 편입·모델 학습·learned closed-loop는 하지 않았다. 기계 판정과 각 경로 SHA는 [rank.json](rank.json)에 남겼다.

| 묶음 | V4 경로 | 30–120m 창 | 기존 자료와 관계 | 차체 명목 결과 |
| --- | --- | --- | --- | --- |
| 기존 TRAIN 재사용 | s0002 p00/p04/p05, s0003 p02/p03 | 회전이 창 밖 | 기존 TRAIN과 동일 회전 또는 근접 | 검사 대상 아님 |
| 이미 수집한 p06 회전 | s0002 p06, s0003 p06 | 회전 약 40–56m | 서로 같은 물리 회전·기존 p06 원본 수집 | 신규 독립 장소 아님 |
| 다른 회전이지만 너무 늦음 | s0002 p01/p03/p07, s0003 p05 | 회전 시작 164–193m | 일부는 서로 같은 회전 | 고정 창의 학습 대상 아님 |
| 기존 TRAIN 교차로 근접 | s0002 p02, s0003 p01/p04 | 일부는 회전이 창 안 | 중심 30m 안; p02/p01/p04 같은 물리 회전 | 독립성 불충분 |
| **새 물리 회전 후보** | **s0003 p00/p07** | **회전 39–54m / 55–70m** | **같은 256→12 road/lane 회전, 중심 거리 0m; 독립 장소는 1개** | **보수적 10cm 안쪽 차로의 sweep 실패** |

두 새 후보에서 `30–120m` 이후의 직렬화 경로는 9km/h × 6.4초 = 16m보다 충분히 길다. 그러나 동일한 회전을 route 상에서 16m 달리 시작할 뿐이므로 두 개의 독립 훈련 장소로 세면 안 된다. 이 교차로의 명목 `base_link` 중심선에 기존 Prius 차체를 올려서 0.10m 간격으로 901개 몸체와 900개 짧은 회전-포함 sweep을 검사했다. rear-axle `base_link`에서 actor 중심으로 1.425m 앞을 적용했고, 동일 `active_map_opendrive.xodr`와 sampled raw 차로 및 10cm erosion을 사용했다.

| 새 회전 경로 | raw 몸체 이탈 | 10cm 안쪽 몸체 이탈 | raw sweep 이탈 | 10cm 안쪽 sweep 이탈 | 최소 10cm 몸체 여유 | 최대 10cm sweep 외부 면적 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| s0003 p00 | 0/901 | 0/901 | 0/900 | **70/900** | 0.0238m | 0.00934m² |
| s0003 p07 | 0/901 | 0/901 | 0/900 | **70/900** | 0.0238m | 0.00933m² |

**해석:** 명목 정지 snapshot의 차체는 raw 차로와 10cm 안쪽에 들어가지만 여유가 2.4cm에 불과하다. 짧은 snapshot 간 이동·회전의 보수적 바운드까지 넣으면 10cm 안쪽 차로를 벗어난다. 이는 실제 주행 이탈이나 충돌의 증거는 아니며, 반대로 원본 수집에 안전한 충분한 여유의 증거도 아니다. 수치가 작다고 이탈 70개를 버리거나 10cm 기준을 사후 변경하지 않는다. 0.25m sampling에서는 raw sweep도 극히 소량 이탈했고, 0.10m로 경로를 더 촘촘히 평가하자 raw는 통과했지만 10cm 안쪽은 계속 실패했다. **작은 오차에 민감한 좁은 기하**라는 뜻이다.

다음 조치는 이 V4 경로를 곧바로 촬영하는 것이 아니라, 새 출력 루트에서 **다른 물리 교차로와 넓은 유효 차로를 갖는 Town01 우회전**을 다시 검색하고 동일 검사를 통과시키는 것이다. 선택 창 안에서 회전 전 컨텍스트도 p00은 9m, p07은 25m뿐이다. 학습용 접근 장면을 늘리고 싶다면 회전 시작이 선택 창의 60m 이후이면서 120m 전에 회전이 끝나는 경로를 우선 검색한다. 그 후에야 단일 소유 CARLA BasicAgent 9km/h 원본 수집 → 모든 실제 64-step 미래·9개 motion bound·20Hz 차체 전체 검사 → 독립 DEV → GPU0 학습 여부 판단 순서다. 지도 모델의 추가 학습이나 기능 승인은 이 선별 결과와 별개다.

재현 명령(읽기 전용 원본, 보고서만 새로 생성):

```bash
source tools/e2e/env.sh
python3 tools/workspace_python.py tools/e2e/rank_town01_wide_turns.py \
  --output docs/assets/validation/2026-09-27/drive_next_wide_turn_rank/rank.json
```

[검사 코드](../../../../../tools/e2e/rank_town01_wide_turns.py)는 기존 차체/차로 오프라인 프로브의 geometry helper를 재사용한다. 이 계산도 실제 연속시간 안전성·차량 동역학·BasicAgent 경로추종·E2E 모델 주행을 증명하지 못한다. `capture_authorized=false`, `dataset_admitted=false`, `training_executed=false`는 유지한다.
