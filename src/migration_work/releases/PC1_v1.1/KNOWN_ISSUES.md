# PC1 v1.1 known issues

상태 값:

- `BLOCKER`: 실차 v1.1 주행 release 전에 해결 또는 명시적 안전 승인 필요
- `OPEN`: 기능/품질 문제, 담당과 검증 계획 필요
- `EXTERNAL`: PC1 commit이 아닌 다른 PC/시스템 소유 문제
- `INCLUDED_UNVERIFIED`: 전체 snapshot에는 포함되지만 기능 검증/안전 승인이 없는 기존 로컬 상태

## 문제 목록

총 22개를 추적한다. PC1 자체 release blocker 3개, 다른 PC가 소유하지만 통합을 막는 external blocker 2개, open 개선/검증 항목 15개, 전체 snapshot에 들어가지만 안전 승인이 없는 기존 상태 2개다.

| ID | 상태 | 문제 | 영향 | 소유/조치 |
|---|---|---|---|---|
| PC1-001 | BLOCKER | 전역 `/to_can_bus`와 sender allowlist 부재 | Domain 10의 다른 publisher가 PC1 물리 CAN에 frame을 쓸 수 있음 | ros2_socketcan 설계: PC별 namespace, publisher ownership, CAN ID allowlist |
| PC1-002 | BLOCKER | converter command freshness watchdog 부재 | control_cmd 단절 후 마지막 accel/steer 반복 가능 | converter timeout과 safe fallback, unit/integration test |
| PC1-003 | INCLUDED_UNVERIFIED | planning max velocity 36.0 m/s | 시험차/맵 요구와 불일치 시 과속 trajectory 가능 | snapshot에는 포함; v1.0 4.17과 비교하고 차량 책임자 승인 전 주행에 사용 금지 |
| PC1-004 | BLOCKER | `sample_vehicle` geometry 사용 | wheelbase/overhang/steering limit가 실제 IONIQ EV와 다를 수 있음 | 실제 차량 calibration package로 교체 |
| PC1-005 | EXTERNAL | PC3 MRM nodes/heartbeat 소실 | PC1 vehicle_cmd_gate가 follower 양의 가속을 `-2.4 m/s²` emergency stop으로 override | PC3 system launch와 MRM operators/handler 영구 복구 |
| PC1-006 | OPEN | start_planner safe path 미발견 | 2-point zero-velocity stop path, module WAITING 지속, 출발 불가 | route/pose/lane geometry와 planner debug 분석 |
| PC1-007 | EXTERNAL | PC2 pointcloud input topic mismatch | occupancy grid와 obstacle pointcloud 0 Hz, planning trajectory 차단 | PC2 input을 PC3 concatenated pointcloud에 영구 remap |
| PC1-008 | OPEN | `socket_can_bridge.launch.xml`의 `interface1` 오타 | non-can0 interface 설정이 무시되고 child default에 의존 | `interface` arg로 수정 후 parse/runtime test |
| PC1-009 | OPEN | cruise 0x4F1 exact-byte toggle | 추가 bit/frame drop 시 ROS control mode와 차량 mode 불일치 가능 | bit mask/state machine 및 recorded trace test |
| PC1-010 | OPEN | ACC edge state 변수 의심 오류 | 버튼 edge 인식 오류 가능 | `previous_acc_pressed`/`previous_cancel_pressed` 로직 unit test |
| PC1-011 | OPEN | 0x372 dual gear mapping 근거 미첨부 | 잘못된 P/R/N/D report 가능 | 실제 raw frame과 gear lever 상태 표 작성 |
| PC1-012 | OPEN | 0x386 velocity 단위 불명확 | 주석 m/s와 publish 전 `/3.6`가 충돌 | DBC factor/unit 확인 및 속도 계측 비교 |
| PC1-013 | OPEN | 0x394 DBC mapping 불일치 주석 | lateral/longitudinal accel과 heading rate가 부정확할 수 있음 | 올바른 DBC와 signal test vector 적용 |
| PC1-014 | OPEN | ADAPI web server가 Conda Python 3.13을 선택 | Humble rclpy Python 3.10 ABI import 실패, web server 종료 | ROS launch environment에서 conda PATH 분리 |
| PC1-015 | OPEN | 모든 PC의 `/rviz2` 동일 FQN | duplicate node, UI 자원 낭비와 진단 혼선 | PC1 한 대만 자동 RViz 또는 host별 unique name 정책 |
| PC1-016 | OPEN | legacy `chmod 777` | tty/video/docker socket을 모든 사용자에게 개방 | udev group, docker group, 최소 권한으로 대체 |
| PC1-017 | OPEN | package 내부 backup/pycache | build/install 시 과거 unsafe launch도 share에 노출 가능 | 전체 snapshot에는 포함; active launch와 구분하고 후속 정리 release에서 archive 정책 적용 |
| PC1-018 | OPEN | 전체 ros2_socketcan test suite 실패 | release 품질 gate 불완전 | merge-conflict config와 legacy lint 정리 후 재실행 |
| PC1-019 | OPEN | PC4 bridge/source 설정 미검증 | 논문 data direction, duplicate publisher, timestamp 품질 불명 | PC4 allowlist/bridge/logger/runtime audit |
| PC1-020 | INCLUDED_UNVERIFIED | local RViz config와 missing YOLOX launch | v1.0과 다른 UI/기능 tree가 snapshot에 보존됨 | 포함 사실을 명시하고 파일별 owner 검증; 자동 안전 승인 금지 |
| PC1-021 | OPEN | Autoware full snapshot 약 546 MiB/8,546 files | 첫 push/clone이 느리고 backup·test data로 repository가 지속 비대해질 수 있음 | v1.1은 요청에 따라 포함; 100 MB/file gate 유지, 차기 release에서 source/history 분리 검토 |
| PC1-022 | OPEN | nested `autoware_tools` current worktree 평탄화 | upstream HEAD의 tracked file 238개가 이미 삭제 상태여서 완전한 upstream checkout으로 재현되지 않음 | HEAD `a6b16571...`, present 625/deleted 238 기록; 필요 시 별도 검증된 vendor import 수행 |

## 전체 snapshot 위험의 해석

`_a` branch는 현재 작업공간 보존을 우선하므로 기존의 선택적 quarantine 정책을 사용하지 않는다. backup, pycache, report, 로컬 RViz/parameter 차이도 모두 들어간다. 이 정책은 자료 유실을 막지만 다음을 보장하지 않는다.

- 포함 파일이 모두 active source라는 보장
- 모든 launch/config가 서로 호환된다는 보장
- 미검증 파라미터가 실제 IONIQ EV 308에 안전하다는 보장
- 공개 repository 크기가 장기적으로 적절하다는 보장

`_a`와 `_r` branch 양쪽에 ros2_socketcan source가 존재하므로 실제 overlay에서는 한 사본만 package owner로 source해야 한다. 두 workspace를 동시에 overlay하여 같은 package가 중복 발견되는 상태는 release 검증 대상이 아니다.

## MRM 문제 상세

실측 시 trajectory follower는 양의 acceleration을 생성했지만 final `/control/command/control_cmd`가 velocity 0, acceleration -2.4로 바뀌었다. PC1 `vehicle_cmd_gate` log는 `system_emergency heartbeat is timeout`을 반복했고 `/system/fail_safe/mrm_state` publisher가 0이었다.

이 현상은 Auto/engage 버튼 실패나 start_planner 단독 문제가 아니다. PC3의 `mrm_handler`, emergency stop operator, comfortable stop operator가 graph에서 사라지거나 잘못된 component manager에 load되어 heartbeat가 끊긴 것이 직접 원인이었다. PC1 gate의 fail-safe를 끄거나 timeout을 늘려 숨기면 안 된다.

복구 gate:

- MRM 3 nodes 각각 1개
- `/system/fail_safe/mrm_state` 약 10 Hz
- emergency/comfortable status 정상 rate
- mrm state `NORMAL/NONE`
- `/system/operation_mode/availability.autonomous=true`
- final cmd가 follower cmd를 정상 gate한 결과이며 emergency override가 아님

## start_planner 문제 상세

start_planner는 갓길/도로 가장자리에서 본선으로 합류하는 pull-out module이다. UI에 module 이름이 계속 보이는 것은 plugin이 상시 load되기 때문에 정상일 수 있으나, 당시 live 상태는 실제 candidate `WAITING_APPROVAL`, `safe=false`였다.

소스 흐름:

```text
BehaviorPathPlannerNode
  -> PlannerManager / SubPlannerManager
  -> StartPlannerModule::run
  -> planWaitingApproval
  -> Shift / Geometric / Freespace pull-out search
  -> all candidates rejected
  -> two-point velocity=0 stop path
```

모듈은 safe pull-out end를 지나야 success가 되고 failure transition은 구현상 false다. 한 번 candidate pool에 들어간 WAITING module은 매 cycle 보존될 수 있으므로, 실제 차가 출발하지 않으면 계속 보이는 현상은 결과이기도 하다.

현재 `print_debug_info=false`라 개별 후보의 lane departure/collision/crop/no-path 이유가 숨겨져 있다. 먼저 이 debug만 일시적으로 켜고 한 search cycle을 캡처해야 한다. safety margin을 임의로 완화하거나 module을 disable해서 통과시키면 안 된다.

추가 소스 결함 후보로 launcher YAML의 여러 freespace/A* 값이 manager 구조체에 모두 매핑되지 않는 문제가 확인됐다. 이는 직접 원인으로 확정되지 않았으며 별도 code review와 unit test가 필요하다.

## release gate

다음이 모두 해결되기 전 태그를 “실차 주행 승인” 의미로 사용하지 않는다.

- PC1-001~006 처리
- 실제 vehicle geometry 적용
- 3-PC 동일 window 10분 이상 안정성
- MRM heartbeat와 autonomous availability 연속성
- start_planner 또는 정상 lane-start 조건에서 non-zero trajectory 검증
- control_cmd drop/watchdog test
- CAN payload/gear/cruise recorded evidence
- clean build/test 및 orphan-free stop 5회
