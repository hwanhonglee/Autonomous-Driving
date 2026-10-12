# 긴 복합 경로의 우회전 부분 구간: 정적 선택 계약

<!-- HH_260906 - Pin one turn-centered window before inspecting nominal containment or any driven result. -->

**현재 결과: 선택기 구현·5개 테스트 PASS, 실제 신규 후보 0개, 수집 0회, 학습 0회.** [선택기](../../../../../tools/e2e/select_turn_centered_subwindow.py)는 이미 소유 검증과 SHA 검증을 거친 `right`/`base_link` 직렬화 경로를 읽고, 여러 방향 명령 중 첫 번째 정적 적격 우회전의 평가 구간을 결정한다. CARLA 서버, SSH, GPU를 실행하지 않고 원본 경로도 수정하지 않는다. 기존 V4 [p06 경로](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/routes/town01/right/town01_right_s0002_p06.json)는 실제 `RIGHT` 블록이 하나뿐이라 선택 결과 `NO_ELIGIBLE_WINDOW`이다. 테스트는 그 경로의 실제 기하 앞에 메모리상 `STRAIGHT` 명령을 추가해 복합 경로를 만들고, 창 선택·미래 구간 침범·짧은 tail·차로-ID seam 거부를 검사한다. 이 합성 테스트는 새 수집 경로를 발견했다는 뜻이 아니다.

## 평가 창을 먼저 고정하는 방법

`RIGHT` 명령 블록의 양 끝은 기존 물리 회전 검사와 같은 인접 waypoint 거리의 중간점이다. 각 블록마다 시작점 `T`에서 **정확히 30m 앞**을 창 시작 `S=T−30m`로, 창 끝을 `E=S+90m`로 계산한다. 이 창 안에는 회전 전에 30m, 회전 뒤에 최소 30m가 있어야 한다. 또 출발점에서 `S`까지 30m 이상, `E` 뒤에 9km/h × 6.4초인 16m의 정적 미래와 10m 여유가 있어야 한다. 다른 방향 명령은 `S−15m`부터 `E+26m` 사이에 없어야 한다. 차로 변경, 좌회전, 같은 거리에서 차로 ID가 달라지는 seam은 거부한다. 창 주변의 원본 waypoint로 만든 임시 부분 경로는 **기존** `speed_30kph` 물리 회전 기하 검사를 통과해야 한다. 이 이름은 형상 프로필이며 30km/h 주행 승인이 아니다.

적격 우회전이 둘이면 가장 먼저 나오는 블록을 선택한다. 기하/명령 조건은 원본 경로만으로 계산하며 차체 명목 결과나 실제 주행 결과를 본 뒤 위치를 옮기지 않는다. 하나도 적격하지 않으면 `candidate_count=0`이다. 정적 90m는 9km/h 기준 30초 이동거리 75m보다 길지만 **실제** 30초 연속 `>0.5m/s` 이동, 모든 anchor의 64개/6.4초 미래, 교차로 정지 여부를 입증하지 않는다. 선택 결과는 언제나 `capture_authorized=false`와 `dataset_admitted=false`이다.

읽기 전용 재현(기존 p06은 `candidate_count=0`이므로 선택기 종료 코드 2가 정상):

```bash
python3 tools/workspace_python.py tools/e2e/select_turn_centered_subwindow.py \
  --route storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/routes/town01/right/town01_right_s0002_p06.json \
  --expected-sha256 49d0c147e83ced265b9e954aacb7bad340970b87a42a060cbfcf1f7625387142
python3 tools/workspace_python.py -m pytest -- -q learning/tests/test_select_turn_centered_subwindow.py
```

## 다음 CARLA 후보 검색: 아직 실행하지 않음

기존 [소유 카탈로그 wrapper](../../../../../tools/e2e/run_owned_carla_turn_route_catalog.sh)는 `--physical-turn-profile speed_30kph`와 `left,right`를 강제한다. 그 프로필은 경로 **전체**에 정확히 하나의 방향 블록만 허용하므로 복합 경로를 만들지 못한다. 다음 검색에는 새 **catalog-only 소유 모드**가 필요하다. 기존 wrapper를 우회해 단독 생성기를 실행하거나 이미 존재하는 V4/V5 출력에 덮어쓰면 안 된다. 아래는 그 새 소유 모드가 고정하고 source archive에 기록해야 할 생성기 플래그다.

```text
--map-id town01 --active-server-profile packaged_0915
--host 127.0.0.1 --port 2100 --map-load-settle-sec 0
--output-root storage/experiments/training/2026-09-27/drive_long_route_town01_multi_train_catalog_v1
--scenarios right --seeds 6,7 --pairs-per-seed 8
--min-distance 240 --max-distance 300 --preferred-distance 160
--sampling-resolution 1 --max-traces 100000
```

`--physical-turn-profile`, 개별 회전 기하 옵션, `--allow-map-load`는 넣지 않는다. 생성기의 `right` 시나리오는 `RIGHT`가 포함되고 `LEFT`가 없는 경로를 찾지만, `STRAIGHT`나 여러 `RIGHT`는 허용한다. 따라서 소유 실행 뒤 이 선택기가 차로 변경과 창 안의 추가 명령을 거부하고, 선택된 부분 경로에 기존 물리 기하 프로필을 다시 적용한다. 240–300m는 **탐색 범위**일 뿐 적격 후보나 30초 실제 주행을 보장하지 않는다. 검색 seed는 독립 주행 장소 수를 뜻하지 않는다. 결과는 물리 교차점/road-lane 전이와 기존 Town01 TRAIN·p06 C 원본을 대조해 같은 장소를 묶어야 한다.

## 새 경로가 나와도 남는 관문

1. 새 소유 wrapper를 구현해 Epic Town01 cold start, 작업공간 잠금, 포트 2100–2102 독점, 새 출력 부재, source SHA archive, ready/after_catalog/stopped 영수증과 종료 시 포트 해제를 검증한다. 카탈로그가 실패하거나 조건에 맞는 route가 0개면 수집하지 않는다.
2. 소유 결과의 route SHA를 고정해 선택기를 실행한다. 첫 적격 창에서 archived live OpenDRIVE·Prius body로 **전체 차체와 10cm inset swept envelope**를 정적으로 검사한다. 명목 통과는 수집 후보 선별일 뿐이다. 다른 창으로 사후 이동하지 않는다.
3. 수집 전 원본 QA의 하드코딩된 `30–120m`와 단일 `physical_turn_preflight` 전제를 수정해야 한다. [현행 raw auditor](../../../../../tools/e2e/audit_long_turn_9kph_raw.py)는 진행거리 30–120m를 직접 사용한다. 새 감사는 route SHA와 창 `S,E`를 사전 계획에서만 읽고, 원본 전체를 보존하며 첫 `E` 초과 시 선택을 닫아야 한다. [차체 검사기](../../../../../tools/e2e/audit_future_actor_lane_containment.py)는 `--start-progress-m`/`--end-progress-m`을 이미 받지만, 복합 경로의 이전 회전까지 포함한 전체 episode 차체 검사도 따로 요구한다.
4. 단일 소유 BasicAgent 진단은 C와 같은 9km/h, steering cap 0.45, waypoint purge 1.5m, 동일 센서·맵·기상을 고정한다. 10Hz 여섯 카메라·20Hz 상태/ACK, **실제** 30초 연속 이동, 모든 선택 anchor의 실제 64점 미래, 기존 9개 운동 한계, 충돌0, 목표 도달, raw와 10cm inset 차체/sweep 위반0을 전수 확인한다. 실패 표본 삭제나 한계 완화는 금지한다. 통과해도 expert 1회로 학습 모델 성능은 승인되지 않는다.
5. 독립 DEV site가 아직 없다. Town03 새 검색은 기존 DEV와 같은 기하였고 Town05_Opt 200–230m 소유 검색은 0/199,536 trace였다. Town05_Opt는 통합 DEV와 과거 전역 TRAIN 표기가 충돌하므로 학습/체크포인트 노출을 잠그기 전에는 독립 검증이라고 부를 수 없다. 미개봉 TEST 지도 내부는 열지 않는다.

현재 Town01 V5의 새로운 물리 회전들은 [정적 순위](../drive_town01_v5_rank/README.md)에서 10cm sweep 또는 production lane seam 때문에 0개 수집 허가였다. 이번 선택기는 그 실패를 통과로 바꾸지 않는다.
