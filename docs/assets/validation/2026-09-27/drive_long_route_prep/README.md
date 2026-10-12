# 이동 우회전용 긴 경로: 소유 검색과 원본 QA

<!-- HH_260906 - Keep route generation a non-admission preflight; never promote a catalog or expert capture to learned driving evidence. -->

**상태: `TOWN01_RAW_QA_FAILED_TOWN03_NOVEL_DEV_UNAVAILABLE`.** 기존 카탈로그에서 9 km/h 및 15 km/h 이동 우회전 episode의 길이 예산을 충족하는 파일이 0개라는 [감사](../drive_next_data_plan/README.md)를 새 경로 검색으로 좁혔다. Town01 V4 소유 검색은 우회전 16개를 만들었고, 한 경로를 [단회 BasicAgent 수집·원본 QA](../drive_long_route_p06_capture/QA_v3/QA.md)했지만 곡률과 차체 차로 게이트 때문에 TRAIN 편입을 거부했다. [Town03 신규 DEV 검색](RESULT_TOWN03_V1.md)은 우회전 1개가 기존 213점 경로와 같아 새 DEV 자료를 만들지 못했다. 이번 단계에서 SSH/GPU 학습이나 learned closed-loop는 하지 않았다.

## 첫 시도 보존과 import 수정

`storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v1/`은 **실패 원본 그대로 보존한다.** [catalog.log](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v1/catalog.log)의 정확한 오류는 `ERROR: CARLA Python API and agents are required; source scripts/e2e/env.sh or prepend the selected profile's Python API to PYTHONPATH`였다. [owner_result.json](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v1/owner_result.json)의 `exit_code=1`, `source_bytes_unchanged_and_archived=true`, 오른쪽 후보 0, `route_catalog.json` 없음이다. [ready](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v1/lifecycle/ready.json), [after_catalog](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v1/lifecycle/after_catalog.json), [stopped](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v1/lifecycle/stopped.json) 영수증은 모두 `PASS`다. 이는 CARLA 경로 탐색 성공이 아니다.

V2는 약 150초의 외부 신호로 중단되어 좌회전 부분 파일 8개만 남았고, V3 사용자 서비스는 `rg`가 없는 기본 PATH 때문에 준비 로그를 읽지 못했다. 두 시도는 [V2 사전 계획](PLAN_V2.md), [V3 사전 계획](PLAN_V3.md) 및 각 원본 `owner_result.json`에 실패로 보존했다. V4는 로그 검사에 시스템 `grep -a`를 사용했고 [사전 계획](PLAN_V4.md)대로 사용자 서비스에서 실행했다. [V4 소유 결과](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/owner_result.json)는 `exit_code=0`, source 13/13 보존·동일, `route_catalog.json` complete, 좌/우 각 16개, ready/after_catalog/stopped `PASS`, 포트 2100–2102 해제다. `capture_executed=false`, `training_executed=false`, `dataset_admission_granted=false`를 유지한다.

[V4 우회전 정적 감사](v4_candidate_audit.json)는 [읽기 전용 검사기](../../../../../tools/e2e/audit_new_long_turn_candidates.py)로 소유 결과·카탈로그·각 경로 SHA를 확인한 뒤 기존 Town01 TRAIN s0001_p00과 회전 road/lane 전이·중심 거리·2m 이내 경로 겹침, 9/15 km/h의 길이 예산을 비교했다. 우회전 16개 중 `s0002_p06`, `s0003_p00`, `s0003_p05`, `s0003_p06`, `s0003_p07`은 이 **제한된** 감사의 플래그가 없었다. 다만 `s0002_p06`과 `s0003_p06`, `s0003_p00`과 `s0003_p07`은 각각 서로 같은 물리 회전 전이를 공유한다. 따라서 5개를 5개 독립 장소로 세지 않으며, 정적 감사의 `capture_authorized=false`는 그대로다. 자체 루프·접합부·신호·실제 30초 이동 및 전체 6.4초 미래는 별도 검사해야 한다.

실제 sourced `tools/e2e/env.sh`에서 `carla` 모듈은 import됐으나 최상위 `prepare_carla_route`만 없고 `scripts.e2e.prepare_carla_route`는 있었다. [생성기](../../../../../tools/e2e/prepare_carla_expert_route_catalog.py)는 옛 최상위 경로가 없는 경우에만 canonical helper로 fallback하도록 수정했다. sourced 환경에서 두 모듈의 import를 CARLA 서버 없이 재검증했으며, [회귀 테스트](../../../../../learning/tests/test_prepare_carla_expert_route_catalog.py)에도 같은 조건을 추가했다. v1을 덮거나 결과로 승인하지 않는다.

## 지금 가능한 것과 아직 없는 것

기존 [경로 생성기](../../../../../tools/e2e/prepare_carla_expert_route_catalog.py)는 `--min-distance`, `--max-distance`, `--preferred-distance`로 **직렬화된 3D 경로 길이**를 필터링한다. `--physical-turn-profile speed_30kph`는 좌·우회전 동시 검색(`--scenarios left,right`)만 허용하며, 최종 경로에서 하나의 연속된 회전 블록, 25 m 이상 선행, 10 m 이상 후행, 회전 곡률·heading·경사 등을 [물리 회전 계약](../../../../../tools/e2e/physical_turn_geometry.py)으로 검사한다. 예전 [독립 수집 스크립트](../../../../../tools/e2e/run_independent_common10_capture_20260907.sh)는 실제로 Town01에 `--min-distance 200 --max-distance 230 --preferred-distance 210 --scenarios left,right --physical-turn-profile speed_30kph`를 사용했고, 이미 연구 TRAIN에 사용한 [Town01 right s0001 p00](../../../../../storage/experiments/training/2026-09-07/independent_common10_v1/town01_right_train_retry_02/route_catalog/routes/town01/right/town01_right_s0001_p00.json) 228.251 m를 만들었다. 따라서 **길이 검색 자체는 코드 수정 없이 가능**하지만, 이 예전 경로는 새 독립 episode가 아니다.

| 정적 예산 | 경로 길이 하한 | 새 검색 200–230 m의 최소 여유 |
| --- | ---: | ---: |
| 시작 선행 30 m + 9 km/h × (이동 anchor 30초 + 미래 6.4초) | 121.000 m | 79.000 m |
| 시작 선행 30 m + 15 km/h × (이동 anchor 30초 + 미래 6.4초) | 181.667 m | 18.333 m |

여유는 **정속의 거리 산술**일 뿐이다. 실제 가속·감속·교차로 정지·우회전 시 속도와 마지막 유효 anchor 시점에 따라 30초 연속 이동 및 모든 anchor의 64개/6.4초 미래가 확보되지 않을 수 있다. 30 km/h 연속 주행을 이 200–230 m 검색으로 주장하지 않는다. 30 km/h로 같은 36.4초를 이동하면 선행 30 m 포함 약 333.33 m가 필요하다. 회전 속도는 횡가속도·곡률과 신호 상황으로 미리 정해야 한다.

## 분할과 탐색 공간

- **TRAIN: Town01, DEV: Town03**으로 고정한다. 기존 연구 데이터는 [manifest](../../../../../storage/datasets/prepared/carla-common10-trainval-research-20260914-v1/dataset.json)의 Town01 right `s0001_p00`와 Town03 right `s0000_p00`를 이미 사용했다. 새로운 seed·파일명·다른 시작점만으로 물리적으로 독립적인 site 또는 세션이 되지 않는다.
- [짧은 Town01 p06 / Town03 p07 조사](../drive_next_data_plan/README.md)는 회전 부분의 시작 후보이다. 기본 생성기는 그 부분을 고정해 전후 도로를 늘리는 기능은 없다. 새로 검색된 200–230 m 경로가 그 동일 회전 블록을 포함하는지는 경로의 좌표·road/lane·회전 위치로 사후 대조한다. 없으면 그 경로를 `p06/p07 extension`이라 부르지 않는다. 해당 블록의 정확한 확장이 필수이면 새 생성기 제약/편집 기능과 테스트를 **별도** 설계해야 한다.
- 우선 CARLA **spawn-point endpoints**와 기본 `all_pairs`를 사용한다. `--endpoint-waypoint-spacing-m`은 생성 waypoint endpoint를 지원하지만 도로·접합부 위치와 spawn 높이 검사를 새로 요구한다. 현재 `--endpoint-junction-policy exclude`는 `--straight-capacity-profile`에만 허용되어 물리 회전 프로필과 같이 쓸 수 없다. `directed_topology_straight_v1` 역시 직진 전용이다. spawn 탐색 실패 뒤 waypoint endpoint를 선택하면 이 제한을 숨기지 말고 접합부 출발/도착 여부와 충돌 가능성을 별도로 검사한다.
- **기존 출력은 불변:** Town01 V1/V2/V3 실패와 V4 완료, Town03 V1 완료 루트가 각각 존재한다. 생성기는 같은 이름의 JSON을 `os.replace`할 수 있으므로 향후 별도 시도는 반드시 새 출력 루트와 사전 부재 확인을 사용한다. 옛 카탈로그/episode/TEST를 덮지 않는다.

## V4와 Town03 V1의 기록된 검색 플래그

아래 Town01/Town03 플래그는 **완료된 소유 실행의 기록**이며 단독 명령으로 재실행하지 않는다. 경로 생성기는 이미 실행 중인 서버를 요구한다. `run_owned_carla_expert_trial.sh`는 **기존 경로 JSON을 받아 수집하는 wrapper**이지 경로 생성용 wrapper가 아니고, 예전 독립 수집 스크립트를 재실행하면 카탈로그뿐 아니라 새 촬영까지 이어져 이 단계의 범위를 넘는다. 별도 [catalog-only 소유 wrapper](../../../../../tools/e2e/run_owned_carla_turn_route_catalog.sh)가 고정 플래그·단일 CARLA 소유를 맡았다. 직접 CARLA를 띄우거나 생성기 명령만 단독 실행하지 않는다. `--allow-map-load`는 사용하지 않는다.

```bash
# HH_260906 - Execute only inside a verified, exclusive, owned Town01 CARLA session.
python3 tools/workspace_python.py tools/e2e/prepare_carla_expert_route_catalog.py \
  --map-id town01 --active-server-profile packaged_0915 \
  --host 127.0.0.1 --port 2100 --map-load-settle-sec 0 \
  --output-root storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4 \
  --scenarios left,right --seeds 2,3 --pairs-per-seed 8 \
  --min-distance 200 --max-distance 230 --preferred-distance 210 \
  --sampling-resolution 1 --physical-turn-profile speed_30kph \
  --max-traces 100000
```

```bash
# HH_260906 - Execute only inside a separate verified, exclusive, owned Town03 CARLA session.
python3 tools/workspace_python.py tools/e2e/prepare_carla_expert_route_catalog.py \
  --map-id town03 --active-server-profile packaged_0915 \
  --host 127.0.0.1 --port 2100 --map-load-settle-sec 0 \
  --output-root storage/experiments/validation/2026-09-27/drive_long_route_town03_dev_catalog_v1 \
  --scenarios left,right --seeds 1,2 --pairs-per-seed 8 \
  --min-distance 200 --max-distance 230 --preferred-distance 210 \
  --sampling-resolution 1 --physical-turn-profile speed_30kph \
  --max-traces 100000
```

실행 진입점은 아래와 같았다. **두 출력 모두 존재하므로 같은 명령을 다시 실행하지 않는다.** wrapper는 기존 루트가 있으면 실패하고, CARLA cold start 전에 Python runtime import를 확인하며, `Epic`·고정 맵/flag·source archive/재검사·단일 catalog timeout·CARLA 종료 영수증을 남기고 수집/학습을 호출하지 않는다. 기록된 wrapper SHA256은 `d6233d9881ebda58d7331f779519ea4bcdb2664c5d852bd3b79d5e00e79455d7`이다.

```bash
bash tools/e2e/run_owned_carla_turn_route_catalog.sh town01
bash tools/e2e/run_owned_carla_turn_route_catalog.sh town03
```

wrapper의 정적/무시작 [테스트](../../../../../learning/tests/test_run_owned_carla_turn_route_catalog.py), import 회귀 테스트, 정적 감사 [테스트](../../../../../learning/tests/test_audit_new_long_turn_candidates.py)는 통과했다. V4와 Town03 V1의 `owner_result.exit_code=0`은 catalog 탐색·기본 출처/lifecycle 검사가 끝났다는 뜻뿐이다. 특히 [Town03 결과](RESULT_TOWN03_V1.md)의 유일한 우회전은 기존 DEV와 같아 새 검증 자료로 인정하지 않는다.

`--pairs-per-seed 8`는 독립 episode 수 약속이 아니라 원본 후보의 탐색 quota다. Town01 seed 1 p00는 사용 완료라 seed 2,3에서 찾되 동일 경로 재등장 여부를 기하학으로 검사한다. Town03 DEV seed 0 p00도 사용 완료다. 한 map에서 지정 quota가 다 차지 않더라도 capacity profile 없이 카탈로그가 부분 성공할 수 있으므로, 파일 개수·rejection coverage·`max-traces` 도달 여부를 전수 읽는다. 다수 seed의 같은 좌표/road/lane 경로는 하나의 site로 묶어야 한다.

## 경로 후보 → 수집 허가의 필수 게이트

1. **출처:** manifest 맵·weather·server profile, 사용된 seed/flag, 생성기/회전 계약 SHA256, route JSON SHA256, CARLA owner 종료 상태를 보존한다. 수정된 생성기 `7954d90f27e750a9de183aacca5136fef660ed756ee477f4304f792494e43857`, 물리 회전 계약 `94359e82390ab8c115c514bae6cb874f233bc46b548a6cab239f2e49551bcdfd`이며 실행 전 재확인한다. 실패한 v1의 이전 생성기 해시는 별도 owner archive에 남긴다.
2. **정적 형상:** 실제 route `200–230 m`, `physical_turn_preflight.status=PASS`, 우회전 블록 정확히 하나, 무관한 LEFT/STRAIGHT/차선 변경 명령 없음, 진입 전 최소 30 m와 회전 후 적어도 **미래 6.4초를 넣을 수 있는** 거리, endpoint 연결/고도/충돌·신호 문제를 확인한다. 물리 프로필의 25 m 선행·10 m 후행 통과만으로 30 m 선행·6.4초 tail은 보증되지 않는다. 루프·자기교차·U-turn·중복 lane traversal은 route progress와 XY/road/lane 연속성으로 별도 거부한다. 거리 수치만 보고 회전 전후 길이가 충분하다고 판단하지 않는다.
3. **중복·분할:** 기존 TRAIN/DEV의 route geometry, 수집 source session, site ID 및 road/lane 연결, 교차점과 장거리 공용 segment를 대조한다. 우연히 가까운 교차로와 같은 도로 축을 지나는 것은 기계적인 waypoint 최단 거리만으로 독립이라고 판정하지 않는다. Town01 후보는 기존 Town01 TRAIN과 novel site/trajectory인지, Town03 DEV 후보는 기존 Town03 DEV와 다른 episode이더라도 DEV 안의 독립성 기준에 부합하는지 판정한다. TEST 지도·route 원본과의 중복 검사는 **기존 공개된 split manifest의 ID/해시만으로** 하고 밀봉된 TEST payload는 열지 않는다. 애매하면 신규 수집은 차단한다.
4. **단일 원본 주행:** 위 조건을 통과한 경로만 사전 선언한 속도·BasicAgent 설정으로 소유 CARLA에서 촬영한다. 기존 `run_owned_carla_expert_trial.sh`는 출력 디렉터리가 존재하면 거부하고 workspace lock/포트 소유권 및 종료를 점검하므로 생성된 **새 경로 파일**을 넘길 때 사용한다. 주행 전 `--quality Epic`, 기상/rig/제어 설정, wall-timeout을 명시하며, wrapper 기본값 `Low`에 기대지 않는다. BasicAgent 성공은 학습 모델의 자율주행 성공이 아니다.
5. **raw episode QA 후에만 편입:** 10 Hz 영상/20 Hz 상태·명령 시간 동기, 30초 연속 이동 anchor와 각 anchor의 6.4초/64 미래 점, 실제 이동 >0.5 m/s, 충돌·차선 경계와 차체 전체 의도 차로 포함, progress/속도/yaw/XY 곡률을 전체 원본으로 검사한다. 위반 창을 지우거나 라벨을 사후 재생성해 승인하지 않는다. 합격 수집만 TRAIN 또는 DEV에 편입하고 K1/AH와 같은 고정 지표/반복 seed로 비교한다. 최초 catalog PASS는 이 어느 것도 증명하지 않는다.

**중단 기준:** (a) 새 200–230 m 우회전이 없거나 `max-traces`에 막힘, (b) 동일 site/loop/접합부 위험, (c) 회전 전 30 m 또는 뒤 6.4초 미래 부족, (d) 원본 motion QA 실패 중 하나라도 있으면 해당 후보의 수집·학습은 진행하지 않는다. waypoint endpoint나 더 넓은 길이 대역의 재검색은 새 루트와 수정된 사전 계획으로만 한다.
