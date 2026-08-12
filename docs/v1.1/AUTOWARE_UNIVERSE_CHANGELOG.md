# Autoware Universe v1.1 scope for PC1

## 기준과 게시 단위

- 작업 디렉터리: `/home/a/autoware`
- 기준 branch: `h2_i/IONIQ_EV_307/PC1_rocket/autoware_universe/v1.0`
- 기준 commit: `5b06fff6a713001f5bbb23aac2b34adb03e126f6`
- 새 branch: `h2_i/IONIQ_EV_308/PC1_rocket/autoware_universe/v1.1`
- 새 tag: `IONIQ_EV_308_PC1_a`

기준 commit은 과거 상태와의 차이를 설명할 때만 사용한다. 새 branch는 기존 history를 가져오지 않는 독립 snapshot으로 만들고 v1.0 branch/tag는 수정하지 않는다. 별도 clone에 선택 파일만 옮기는 방식도 사용하지 않는다. `/home/a/autoware`에서 [PREPARE_IN_PLACE.sh](PREPARE_IN_PLACE.sh)를 실행하여 현재 작업공간 전체를 한 snapshot으로 기록한다.

## 전체 snapshot 포함 정책

다음은 현재 존재하는 그대로 포함한다.

- `/home/a/autoware` root의 meta-repository 설정, 문서와 도구
- `/home/a/autoware/src` 전체 source tree
- package 내부 backup launch/source와 `(copy_org)`, `(copy_ioniq_phev)`
- `migration_work/backups`, reports, inventory, test logs 및 과거 실험 기록
- `__pycache__`, `*.pyc`, IDE 설정과 테스트 map/media
- 로컬에서 이미 존재했던 `max_vel: 36.0`, RViz layout, gear raw mapping 등의 차이

제외하는 것은 다음뿐이다.

- 재생성 가능한 `build/`, `install/`, `log/`
- root 및 nested `.git`/`.gitmodules` 같은 기존 VCS metadata
- 실제 credential은 발견 즉시 준비를 중단하는 release blocker

backup/pycache까지 포함하는 것은 일반적인 source release 권장안이 아니라 사용자가 현재 작업공간을 통째로 보존하기로 결정했기 때문이다. 이 파일은 안전하지 않은 과거 launch를 노출하거나 저장소를 불필요하게 키울 수 있으며, 포함 사실이 사용 승인이나 기능 검증을 의미하지 않는다.

준비 전 감사 기준 대상은 약 546 MiB, 일반 파일 8,546개였다. 단일 파일 100 MB 초과는 없었지만 48,000,176-byte PCD 두 개가 포함된다.

## nested repository 평탄화

formal submodule과 `.gitmodules`는 발견되지 않았다. 다음 경로만 별도 Git checkout이었다.

```text
src/universe/autoware.universe/autoware_tools/.git
origin: https://github.com/autowarefoundation/autoware_tools.git
HEAD: a6b16571dbff1c825a8f829ab3c7433e06b2b4c7
```

준비 과정은 nested `.git`/`.gitmodules`를 `/home/a` 아래 timestamp backup으로 이동하고 worktree 파일은 건드리지 않는다. 그 결과 현재 보이는 625개 파일은 `_a` branch의 일반 파일이 된다. nested repository에서 이미 tracked deletion이었던 238개 파일은 복원하지 않으므로, 이 snapshot은 upstream `autoware_tools` commit 전체가 아니라 당시 PC1 worktree 상태다.

## 확인된 기능 변경

### ros2_socketcan receiver lifecycle

대상 파일:

- `src/sensor_component/ros2_socketcan/ros2_socketcan/include/ros2_socketcan/socket_can_receiver_node.hpp`
- `src/sensor_component/ros2_socketcan/ros2_socketcan/src/socket_can_receiver_node.cpp`

변경 내용:

- atomic receive-worker 상태
- 명시적 destructor
- 공통 `stop_receiver_thread()`
- configure/start, cleanup/shutdown/destruction 연동
- CAN/CAN-FD loop stop condition
- 정상 idle `SocketCanTimeout` 처리
- 변경 이유를 설명하는 `HH_260810` 주석
- C++ source mode를 실행 파일 `100755`에서 일반 source `100644`로 정규화

기존 차량 CAN decoder/converter 전체를 재작성한 것이 아니다. gear raw mapping 차이는 이 작업 전부터 존재했으며 실차 DBC/candump 검증이 남아 있다.

## v1.0과 동일하게 복원한 파일

다음 파일은 중간 실험을 제거한 뒤 PC1 v1.0과 byte-identical이다. 전체 snapshot에는 들어가지만 v1.1 기능 변경으로 주장하지 않는다.

- `src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml`
- `src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml`
- `src/sensor_component/ros2_socketcan/ros2_socketcan/launch/can_brdige.launch.xml`
- `src/sensor_component/ros2_socketcan/ros2_socketcan/launch/socket_can_bridge.launch.xml`
- start_planner 관련 launcher/universe 파일
- pointcloud container launch 파일
- vehicle, param, sensor-kit tree

## 포함되지만 미검증인 기존 상태

| 파일/영역 | snapshot 상태 | 해석/후속 조치 |
|---|---|---|
| `common.param.yaml` `max_vel: 36.0` | 포함 | v1.0은 4.17; 차량/시험장 속도 승인 전 주행 release 근거로 사용 금지 |
| `autoware.rviz` | 포함 | 2025-05-23 저장 UI/automatic goal 관련 로컬 상태; 운영자 확인 필요 |
| receiver gear raw mapping | 포함 | 0x20/0x10 등 dual raw 허용; 실제 raw frame과 DBC 근거 필요 |
| missing YOLOX launch | 현재 absence 그대로 기록 | v1.0에 있던 파일을 snapshot 준비가 자동 복원하지 않음 |
| backup/pycache | 포함 | package install share에 unsafe alternate launch가 노출될 수 있음 |
| nested `autoware_tools` | 일반 파일로 평탄화 | upstream 완전 checkout 아님; 위 HEAD와 deletion count를 provenance로 사용 |

## commit과 검증 해석

전체 snapshot commit에는 확인된 receiver fix, 운영 문서, 과거 로컬 변경, backup/test 자료가 함께 들어간다. 따라서 commit 전체를 “모두 v1.1에서 새로 수정했고 검증 완료”로 설명하면 안 된다. 기능별 판단은 [CHANGELOG.md](CHANGELOG.md), [VALIDATION.md](VALIDATION.md), [KNOWN_ISSUES.md](KNOWN_ISSUES.md)를 기준으로 한다.

준비 및 게시:

```bash
cd /home/a/autoware
bash src/migration_work/releases/PC1_v1.1/PREPARE_IN_PLACE.sh
./push_v1_1.sh
```

준비 script는 local commit/tag까지만 만들며 push나 PAT 입력은 수행하지 않는다. 최초 snapshot 이후의 검증 문서는 review branch에서 기존 v1.1을 base로 PR한다. 병합 후 `_a` tag만 예상 old SHA를 확인한 조건부 갱신 대상으로 하며, 307 ref와 다른 tag는 force하지 않는다.
