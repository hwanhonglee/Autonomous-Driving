# PC1 v1.1 validation record

## 판정 규칙

- `PASS`: 해당 범위의 실제 결과와 evidence가 있음
- `PARTIAL`: 일부 기능만 확인했거나 legacy failure가 남음
- `NOT_RUN`: 실행하지 않음
- `BLOCKED`: 선행 조건 때문에 acceptance를 수행할 수 없음

## 정적 및 빌드 결과

| 항목 | 결과 | 근거/비고 |
|---|---|---|
| PC1 v1.0 baseline fetch | PASS | remote commit `5b06fff6...`를 fresh sparse clone으로 비교 |
| `autoware.launch.xml` vs v1.0 | PASS | SHA256 `b9129744...`, byte-identical |
| restored `planning_simulator.launch.xml` vs v1.0 | PASS | SHA256 `a82e5f26...`, byte-identical |
| restored CAN launch XML vs v1.0 | PASS | top `c577b7ef...`; child `b52011c7...` |
| `MANIFEST.sha256` | PASS | `/home/a/autoware/src` 기준 11개 code/config/baseline entry를 `sha256sum -c`로 검증 |
| full Autoware snapshot inventory | PASS | 최초 공개 snapshot `2523b662...`: 8,553 tracked files, content 537,279,158 bytes; 100 MB 초과 파일 0개 |
| nested Git/submodule audit | PASS/PARTIAL | `.gitmodules`/formal gitlink 없음; nested `autoware_tools/.git` 1개 확인, 현재 파일 평탄화 예정 |
| nested `autoware_tools` completeness | PARTIAL | HEAD `a6b16571...`; 625개 파일 존재, upstream tracked deletion 238개는 현재 absence 그대로 보존 |
| portable ros2_ws payload | PASS | two shell files `bash -n`, DDS XML `xmllint`, modes 0755 |
| ros2_socketcan Release build | PASS | 2026-08-10 21:02, 2026-08-11 14:42, 15:03, 최종 19:05 build 성공 |
| enabled C++ gtests | PASS | final 19:06 `ctest -R '^ros2_socketcan_test$'`: 1 target, enabled assertions 3, 0 failure |
| disabled CAN tests | NOT_RUN | 7 tests disabled by package |
| full colcon test | PARTIAL/FAIL | 9 test groups 중 6 failure, legacy lint/config 포함 |
| XML lint | PASS | ros2_socketcan XML 8 files pass |
| final source/comment rebuild | PASS | `/home/a/autoware/log/build_2026-08-11_19-05-42`; 1 package 성공 |
| 2026-08-12 evidence-comment rebuild | PASS | `/home/a/autoware/log/build_2026-08-12_14-51-08`; Release, ros2_socketcan 1 package 성공 |
| 2026-08-12 enabled functional gtest | PASS | `ctest -R '^ros2_socketcan_test$'`: 1/1 target, 0 failure |

## 전체 test failure 분류

- copyright: custom converter source 3 files에 copyright notice 없음
- cpplint: workspace `CPPLINT.cfg`에 merge marker/invalid option, custom legacy formatting 다수
- flake8: backup launch Python과 active receiver launch whitespace
- lint_cmake: CMakeLists trailing whitespace
- pep257: workspace `setup.cfg` duplicate option으로 tool error
- uncrustify: legacy custom receiver/sender와 backup files 8개

이 결과를 “모든 test PASS”로 표현하면 안 된다. functional gtest 3개는 통과했지만 actual CAN receiver/sender tests 7개는 disabled다.

최종 build는 성공했지만 symlink install 과정에서 package 내부 `(copy_org)`, `(copy_ioniq_phev)`, `__pycache__`도 install share에 노출되는 것이 다시 확인됐다. 사용자는 현재 작업공간 전체 snapshot을 요청했으므로 이 파일들도 `_a` branch에는 포함한다. 그러나 active launch와 혼동될 위험은 그대로이며, 포함을 실행 승인으로 해석하면 안 된다.

## 전체 snapshot 감사

`/home/a/autoware`에서 generated `build/`, `install/`, `log/`와 모든 `.git` metadata를 제외해 집계한 준비 전 결과:

```text
regular files: 8,546
content bytes: 537,254,945
disk usage: approximately 546 MiB
files larger than 100,000,000 bytes: 0
```

가장 큰 non-Git 파일 두 개는 각각 48,000,176-byte PCD test map이다.

```text
src/universe/autoware.universe/common/autoware_test_utils/test_map/road_shoulder/pointcloud_map.pcd
src/universe/autoware.universe/common/autoware_test_utils/test_map/intersection/pointcloud_map.pcd
```

다음 현재 자료도 의도적으로 snapshot에 포함한다.

- package backup/copy 파일
- `migration_work` backup/report/inventory/test log
- `__pycache__`, `*.pyc`
- test bag/PCD/media와 로컬 IDE 설정

이 집계는 준비 스크립트와 최종 commit이 추가되기 전 값이므로 최종 tracked file count와 몇 개 차이날 수 있다. 최종 release evidence에는 `git ls-files` 기반 count와 tree size를 다시 기록해야 한다.

formal `.gitmodules` 또는 mode `160000` submodule은 기준 source에서 확인되지 않았다. `autoware_tools`는 자체 `.git`을 가진 unregistered nested repository였다. 평탄화 후 검증 기준은 다음과 같다.

```bash
git -C /home/a/autoware ls-files -s | awk '$1 == 160000 { print }'
find /home/a/autoware \
  -path /home/a/autoware/build -prune -o \
  -path /home/a/autoware/install -prune -o \
  -path /home/a/autoware/log -prune -o \
  -name .git -not -path /home/a/autoware/.git -print
```

두 명령 모두 출력이 없어야 한다. 준비 과정은 nested metadata를 `/home/a` 아래 timestamp backup으로 이동하고 실제 source 파일을 삭제하지 않아야 한다.

## SocketCAN shutdown evidence

수정 전 대표 로그:

`/home/a/.ros/log/2026-08-10-20-55-25-165240-a-32479/launch.log`

증상:

- receiver thread가 joinable 상태로 파괴됨
- process exit code `-6`
- 과거 launch log에서 같은 receiver abort가 반복 발견됨

수정 후 집계:

- receiver launch 26회 중 최초 died 1
- clean finish 23
- SIGTERM escalation 뒤 종료 1
- 종료 기록 없음 1

따라서 개선은 확인됐지만 100% clean shutdown acceptance로 과장하지 않는다. final release tree에서 5회 Ctrl+C, cleanup transition, no-CAN idle 각각 다시 검증한다.

## DDS/network runtime evidence

기본 CycloneDDS 자동 선택:

- selected NIC: 192.168.1.11 network
- remote node: 0

`enp0s31f6` 강제 후:

- PC3 map/sensing/system nodes 발견
- `/map/vector_map` publisher 1, transient-local, actual sample 수신
- `/map/pointcloud_map` publisher 1, transient-local, actual sample 수신
- 후속 실행에서 localization odometry/acceleration, GNSS, IMU, LiDAR, TF 수신 확인
- PC2 predicted objects 약 9 Hz 확인

이는 NIC 고정 효과를 검증하지만 PC2/PC3 process flap과 application readiness까지 보장하지 않는다.

## 3-PC integration evidence

관찰된 문제와 복구 흐름:

1. PC2 raw perception이 `/sensing/lidar/top/pointcloud_before_sync`를 구독했지만 publisher 0.
2. PC3 실제 source는 `/sensing/lidar/concatenated/pointcloud` 약 9~11 Hz.
3. 임시 compatibility relay가 생긴 뒤 before_sync, crop, single-frame, occupancy grid, obstacle pointcloud가 연쇄 복구.
4. final planning trajectory 약 10 Hz, trajectory follower control 약 33 Hz, gated control 약 10~33 Hz 확인.
5. relay는 PC2 영구 설정 변경을 대체하지 않으며 PC1 commit에 포함하지 않는다.

## 실차 출발 미성공 원인

시점에 따라 두 gate가 확인됐다.

- PC3 MRM heartbeat 부재: follower 양의 가속이 final -2.4 emergency stop으로 override.
- start_planner safe=false: 최종 trajectory 자체가 전점 velocity 0인 stop path.

따라서 “Auto 전환 성공”만으로 주행 readiness를 판정할 수 없다. 현재 v1.1에 대해 완전한 실차 route-to-motion PASS는 없다.

## 2026-08-12 실차 3-PC 관찰 기록

이 절은 합격 시험이 아니라 실제 문제를 재현한 incident evidence다. 관찰은 read-only ROS/CAN graph와 topic/log 측정으로 수행했으며 Codex가 engage, route, command 또는 CAN frame을 송신하지 않았다.

| 시간대(KST) | 관찰 | 판정 |
|---|---|---|
| 약 11:50 | ADAPI web server가 Conda Python 3.13을 선택하여 Humble Python 3.10 `rclpy` ABI import 실패; Domain 10 `/rviz2` 3개 | FAIL, PC1 환경 오염/duplicate FQN |
| 약 11:54~12:59 | raw GNSS/IMU/LiDAR/vehicle status는 수신되지만 localization은 UNINITIALIZED; lateral velocity 약 0.00111 m/s가 0.001 m/s stop gate 초과 | FAIL, CAN/정차 신호 검증 필요 |
| 약 13:00 | localization INITIALIZED, map→base_link와 kinematic state 약 40~50 Hz 복구 | 일시 PASS |
| 약 13:01:52 | localization이 약 12~13초 INITIALIZING으로 회귀한 뒤 재복구 | FAIL, continuity flap |
| 약 13:02:23 | route/trajectory/control 복구, hazard clear, AUTO/DRIVE와 physical CAN control request 활성 | 기능 체인 활성 확인, 안전 합격 아님 |
| 약 13:04:49 | PC2 perception stack/objects stream 소실 후 복구 | FAIL, remote-stack flap |
| 약 13:05:32 | NDT/trajectory deviation fault, MRM EMERGENCY_STOP; 차량 속도 약 2.47 m/s까지 관찰 | CRITICAL FAIL |
| 약 13:06:32 | 차량 속도 0/PARK 복귀 | 정지 확인; release gate는 여전히 FAIL |

같은 실행의 Chrony 확인에서는 public source `211.108.117.211`이 selected였고 PC3 `192.168.9.7`은 후보 source였다. 따라서 이 실행은 다중 PC timing/latency 논문 데이터 acceptance로 사용할 수 없다.

추가 CAN 증거:

- command callback이 끊긴 상태에서도 converter는 마지막 `-2.4 m/s²`, steering 0 명령을 CAN ID `0x630`으로 약 33.33 Hz 계속 송신했다.
- 당시 control request byte는 cruise=false일 때 0이었지만, cruise=true가 되면 같은 cached command에 request bit가 붙을 수 있다.
- 실제 AUTO 구간에는 fresh control command와 steering command가 CAN에 반영됐다. 즉 현재 bridge는 관찰 전용이 아니라 실제 actuator-capable path다.

이 결과로 PC1-001, PC1-002, PC1-023~026을 release blocker로 유지한다. 다음 주행 전에는 물리 차량을 확보한 상태에서 PC2/PC3 연속성, localization jump, MRM, command timeout과 공통 clock source를 각각 독립 시험해야 한다.

## direct in-place 준비 후 필수 재검증

```bash
cd /home/a/autoware
bash src/migration_work/releases/PC1_v1.1/PREPARE_IN_PLACE.sh

git status -sb
git branch --show-current
test "$(git rev-list --max-parents=0 HEAD)" = "$(git rev-parse HEAD)"
git tag --points-at HEAD
git ls-files -s | awk '$1 == 160000 { print }'

git -C /home/a/ros2_ws status -sb
git -C /home/a/ros2_ws branch --show-current
git -C /home/a/ros2_ws tag --points-at HEAD
```

예상 branch/tag와 clean 상태, parent 없는 독립 snapshot, gitlink 0개를 모두 확인한다. `PREPARE_IN_PLACE.sh`는 local commit/tag만 만들고 push/PAT는 사용하지 않는다.

## push 전 기능 재검증

```bash
# 준비된 각 worktree에서
colcon build --symlink-install --packages-select ros2_socketcan --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select ros2_socketcan --event-handlers console_direct+
colcon test-result --test-result-base build/ros2_socketcan --verbose

# static
xmllint --noout path/to/*.launch.xml
bash -n scripts/start.sh

# runtime, 안전이 확보된 실차 환경
ros2 node list -a | sort | uniq -cd
ros2 topic info -v /to_can_bus
ros2 topic info -v /control/command/control_cmd
ros2 topic hz /system/fail_safe/mrm_state
ros2 topic hz /planning/scenario_planning/trajectory
ros2 topic hz /control/trajectory_follower/control_cmd
```

반드시 전체 launch log, CAN statistics, exact commit SHA, tag, 시간 동기 상태를 함께 보존한다.
