# Town01 다중 기동 경로: 회전 중심 90m 창 2개, 수집 허가 0개

<!-- HH_260906 - Distinguish static route screening, nominal vehicle geometry, raw driving evidence and learned control. -->

**결론:** 소유 CARLA 카탈로그의 우회전 경로 16개 전체를 해시 검증 후 검사했다. 고정 정책의 회전 중심 90m 창이 가능한 파일은 2개지만 **같은 물리 회전 한 곳**이다. 두 파일 모두 기존 10cm 안쪽 차로 기준에서 명목 Prius 몸체 25/901, sweep 42/900이 밖으로 나가므로 새 원본 주행 수집은 허가하지 않는다. [전체 결과](rank.json)의 `capture_authorized=false`, `dataset_admitted=false`, `training_executed=false`가 현재 상태다.

| 단계 | 결과 | 의미 |
| --- | ---: | --- |
| 소유 CARLA 카탈로그 | 16개 경로, 완료 | [owner 결과](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_multi_train_catalog_v1/owner_result.json)의 exit 0·소스 13/13·ready/after_catalog/stopped·포트 해제 확인 |
| 정적 창 제외 | 9개 | 동일 거리에서 차로 ID가 바뀌어 기존 production 차로 게이트가 거부함. ID를 합치거나 규칙을 낮추지 않음 |
| 정적 창 제외 | 5개 | 30m 접근·90m 창·6.4초 미래/10m 여유 안에서 다른 기동이 겹치거나 길이가 부족함. 1개는 두 번째 기동 자체가 없음 |
| 정적 창 후보 | 2개/물리 회전 1곳 | 새 회전 279→4, 중심 약 (333.0, -127.4)m(ROS). 기존 TRAIN, V4 사이트, C/p06 수집 회전과 다름 |
| 명목 Prius 10cm 안쪽 차로 통과 | **0개** | 두 후보 모두 몸체와 sweep 기준 실패. raw 차로 밖은 0이지만 사전 등록된 안쪽 10cm 기준을 완화하지 않음 |

두 후보는 `s0006_p03`(회전 144.87–160.08m, 창 114.87–204.87m)과 `s0007_p02`(회전 94.10–109.31m, 창 64.10–154.10m)다. 둘의 차로 전이와 회전 위치가 같아서 학습 다양성 2개로 세지 않는다. 두 창 모두 회전 전 30m, 회전 뒤 30m, 9km/h 기준 6.4초 미래 16m와 추가 10m 여유를 정적으로 확보하고 고립 회전의 기존 물리 형상 검사는 통과했다. 그 다음 단계의 명목 형상 검사 결과가 아래처럼 실패했다.

| 명목 검사, 0.1m 간격 | s0006 p03 | s0007 p02 |
| --- | ---: | ---: |
| raw 차로 밖 몸체 / 10cm 안쪽 밖 몸체 | 0/901 · **25/901** | 0/901 · **25/901** |
| raw 차로 밖 sweep / 10cm 안쪽 밖 sweep | 0/900 · **42/900** | 0/900 · **42/900** |
| 최대 10cm 안쪽 외부 면적 | 0.02421m² | 0.02421m² |
| 현재 production zero-arc 차로-ID 게이트 | 지원 | 지원 |

검사에는 C/p06 실제 수집 때 보관한 **live Town01 OpenDRIVE**(SHA256 `97a7f6ac67812567e5c8ee0599cd823b23f80f30f3f97c502212e38b72e2b709`)와 그때 기록한 Toyota Prius 바운딩 박스를 사용했다. 기존 검사기처럼 ROS 경로의 `y`를 CARLA에서 반전하고, 각 관련 경로 점을 같은 XODR 차로 중심선에 최대 0.08m 오차로 대응시켰다. 두 후보에서 XODR 대응은 성공했고 raw 차로 밖 몸체·sweep은 0이므로, 이번 실패를 지도 좌표계가 뒤집힌 탓으로 해석하지 않는다. 그래도 이것은 **직렬화 경로 중심선을 따라 놓은 명목 차량** 검사일 뿐이며, 실제 BasicAgent 주행·연속시간 안전·learned E2E 성능을 입증하지 않는다.

[선별 코드](../../../../../tools/e2e/rank_town01_multi_turn_windows.py)는 [고정 90m 창 정책](../../../../../tools/e2e/select_turn_centered_subwindow.py)을 그대로 사용한다. [기존 V5 명목 차체 검사](../../../../../tools/e2e/rank_town01_v5_long_turns.py)는 시작/끝 거리를 인자로 받도록만 확장했고 기본 30–120m 결과가 바뀌지 않음을 회귀 시험했다. [테스트](../../../../../learning/tests/test_rank_town01_multi_turn_windows.py)를 포함한 관련 17개가 통과했다. 오프라인 선별은 CARLA 서버·GPU·SSH를 실행하거나 파일 수집/학습을 하지 않았다.

```bash
python3 tools/workspace_python.py tools/e2e/rank_town01_multi_turn_windows.py
python3 tools/workspace_python.py -m pytest -- -q \
  learning/tests/test_rank_town01_multi_turn_windows.py \
  learning/tests/test_rank_town01_v5_long_turns.py \
  learning/tests/test_select_turn_centered_subwindow.py
```

다음은 안쪽 10cm에서 몸체와 보수적 sweep이 모두 0인 **다른 물리 회전**을 찾는 것이다. [원본 QA의 고정 창 선택 부분](../drive_multi_maneuver_window/QA_WINDOW.md)은 독립 경로·선택 결과 SHA 바인딩을 받도록 준비했지만, 전체 QA/CLI는 여전히 기존 p06 원본·출처·출력에 고정돼 있다. 새 후보가 나오면 먼저 전체 auditor의 소유 trial/route/map/sensor/출력 바인딩과 차체·sweep 검사를 별도로 구현·시험해야 한다. 그 후 소유 BasicAgent 단일 원본 수집 → 20Hz 궤적·64-step 미래·9개 motion bound·차로 침범/충돌 QA → 데이터 승인 → 학습/별도 closed-loop 검증 순서다. 지금 단계에서 데이터나 모델 성능이 추가된 것은 아니다.
