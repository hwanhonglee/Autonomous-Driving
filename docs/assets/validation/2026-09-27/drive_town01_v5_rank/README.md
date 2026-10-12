# Town01 V5 긴 우회전: 새 회전은 있으나 수집 허가 후보 없음

<!-- HH_260906 - Separate an owned route catalog, nominal geometry, expert capture and learned control. -->

**결론: V5의 우회전 16개는 물리적으로 7개 회전이며, V4·기존 TRAIN·수집된 C/p06에 없던 회전은 2곳이다. 그러나 고정 30–120m 창과 10cm 차체·sweep 기준을 모두 충족하고 현재 원본 QA로 검증 가능한 경로는 0개다.** [기계 판독 전체 결과](rank.json)에 route SHA, owner/lifecycle/source 확인, 각 후보 제외 사유를 기록했다. V5 카탈로그 검색에는 소유 CARLA 서버를 사용했고 `exit_code=0`, source 13/13, ready/after_catalog/stopped PASS·포트 해제를 확인했다. 그 뒤 오프라인 선별에서는 CARLA 서버·GPU·SSH를 실행하지 않았고 새 주행 수집·학습도 하지 않았다.

| 새 물리 회전 | 관련 V5 경로 | 고정 창 내 회전 | 판정 |
| --- | --- | --- | --- |
| 108→24, 중심 약 (95.6, -324.4)m | s0004 p04, s0005 p01 | p04는 130–145m, p01은 111–126m | 두 경로 모두 120m 이전에 회전이 끝나지 않음 |
| 169→6, 중심 약 (332.9, -324.5)m | s0004 p05, s0005 p02, s0005 p05 | p05: 91.3–106.6m; p02: 68.3–83.6m; 나머지는 121.6m까지 | p05/p02의 명목 10cm 안쪽 sweep 실패·현재 production lane gate의 zero-arc 차로-ID 전이 미지원 |

169→6 후보 두 개는 시작점이 달라도 **같은 물리 회전 한 곳**이다. 120m 뒤에도 9km/h × 6.4초에 필요한 16m 이상의 경로가 있고, 선택 창의 회전 전 컨텍스트는 s0004 p05가 61.3m, s0005 p02가 38.3m다. 원본 직렬화 경로에서 road `150→169`가 같은 거리·x/y/z/yaw인 연속 노드로 나타난다. 오프라인 예비 검사에서는 그 노드를 위치 보간용으로만 묶고 **두 road/lane ID를 모두** 보존해 같은 [실제 수집 Town01 XODR](../drive_waypoint_purge_ab/QA_v1/C_lane_gate_30_120.json)의 내측 차로를 조회했다. 기존 production [차체 검사기](../../../../../tools/e2e/audit_future_actor_lane_containment.py)는 이런 동일 거리의 차로-ID 변경을 의도적으로 거부한다. 이번에 그 안전 장치를 수정하거나 통과 처리하지 않았다.

| 명목 Prius 검사(0.10m 간격) | s0004 p05 | s0005 p02 |
| --- | ---: | ---: |
| raw 차로 밖 몸체 / 10cm 안쪽 밖 몸체 | 0/901 · 0/901 | 0/901 · 0/901 |
| raw 차로 밖 sweep / 10cm 안쪽 밖 sweep | 0/900 · **38/900** | 0/900 · **38/900** |
| 최소 10cm 안쪽 몸체 여유 | 0.02210m | 0.02210m |
| 최대 10cm 안쪽 sweep 외부 면적 | 0.00968m² | 0.00968m² |

이 수치는 **직렬화된 route 중심선에 Prius 차체를 올린 명목 형상**이며 실제 BasicAgent 궤적, 충돌이나 learned E2E 성능의 측정값이 아니다. sweep은 회전 중 차체를 보수적으로 둘러싸므로 작은 외부 면적도 0으로 취급하지 않는다. 사전 10cm 규칙을 낮추거나 38개 창을 삭제하지 않는다. 따라서 seam-aware production 검사만 추가해도 이 경로가 자동 합격하는 것은 아니다. 물리적으로 더 여유 있는 새 경로를 검색하거나, 기하·동역학에 맞는 독립 후보를 사전 등록해야 한다.

재현:

```bash
source tools/e2e/env.sh
python3 tools/workspace_python.py tools/e2e/rank_town01_v5_long_turns.py \
  --output docs/assets/validation/2026-09-27/drive_town01_v5_rank/rank.json
```

[코드](../../../../../tools/e2e/rank_town01_v5_long_turns.py)와 [회귀 테스트](../../../../../learning/tests/test_rank_town01_v5_long_turns.py)는 V5 소유 자료와 C/p06의 actor bbox·XODR 해시를 고정하고, seam의 포즈 변화는 거부하며, 양쪽 차로-ID를 유지한다. 테스트 5개가 통과했다. `capture_authorized=false`, `dataset_admitted=false`, `training_executed=false`다. 다음 단계도 새 사이트 탐색과 이 게이트 통과가 먼저이며, 그 후 단일 BasicAgent 원본 수집 → 실제 64-step 미래/9개 motion bound/차체 원본 QA → 모델 학습 순서다.
