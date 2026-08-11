# PC1 v1.1 changelog

## 변경 요약

| 영역 | v1.0/이전 상태 | 문제 | v1.1 변경 | 개선 효과 | 검증 상태 |
|---|---|---|---|---|---|
| SocketCAN receiver 종료 | receive thread가 lifecycle cleanup과 소멸 시 항상 안전하게 종료되지 않음 | Ctrl+C 때 joinable thread 소멸로 exit `-6`; cleanup 중 join 대기 가능 | atomic running flag, destructor, 공통 stop/join helper, cleanup/shutdown 연동 | bridge 종료 안정성 향상, publisher reset race 축소 | Release build 성공; 대다수 후속 run clean finish; 전체 test suite는 legacy lint로 실패 |
| SocketCAN idle timeout | 무프레임 poll timeout도 예외 경고 | CAN이 조용할 때 1초 주기 경고 스팸 | `SocketCanTimeout`을 정상 idle 상태로 별도 catch | 실제 CAN 오류와 정상 idle을 구분 | 소스·빌드 확인; 장기 회귀 테스트 필요 |
| Source file metadata | receiver cpp가 executable mode로 추적됨 | source tree 정리와 검토 잡음 | receiver hpp/cpp를 0644로 정규화 | 실행 대상과 source metadata 구분 | local mode 확인 |
| CAN 초기화 스크립트 | 이미 UP인 CAN에 bitrate를 바로 설정하면 실패할 수 있음 | 재실행 순서에 따라 수동 복구 필요 | can0/can1 down → bitrate 500000 → up, strict shell 및 존재 검사 | 동일 명령 반복 실행 가능, 장치 glob 누락으로 중단되지 않음 | `bash -n` 통과, can0/can1 500 kbit/s ERROR-ACTIVE 확인 |
| DDS NIC 선택 | 다중 NIC에서 CycloneDDS가 192.168.1.11 NIC를 임의 선택 | Domain 10에서 PC2/PC3 node/topic이 0개로 보임 | `cyclonedds_pc1.xml`에서 `enp0s31f6` 고정, shell export와 NM NIC binding | 192.168.9.0/24 차량망에서 원격 map/localization/perception discovery 복구 | 실제 원격 map 샘플 및 후속 3-PC graph 수신 확인 |
| 릴리스 관리 | 임시 fail-closed 문서와 현재 full bridge 상태가 혼재 | 잘못된 설정을 v1.1로 push할 위험 | final/reverted/pre-existing/unresolved 분리 문서화 | 코드 소유권과 검증 범위를 추적 가능 | 본 감사 문서와 SHA manifest로 확인 |
| 게시 단위 | 선택 파일 이식/fresh clone 절차를 검토함 | 사용자가 현재 작업공간 전체와 direct push를 요구 | `/home/a/autoware` 전체 snapshot `_a`, ros2_socketcan+ops+docs `_r`, nested Git 평탄화, in-place helper 제공 | 두 작업 directory에서 branch/tag 게시 가능 | 준비 전 546 MiB/8,546 files, 100 MB 초과 0; 최종 local commit/tree 재확인 필요 |

## SocketCAN receiver 상세

대상 파일:

- `sensor_component/ros2_socketcan/ros2_socketcan/include/ros2_socketcan/socket_can_receiver_node.hpp`
- `sensor_component/ros2_socketcan/ros2_socketcan/src/socket_can_receiver_node.cpp`

변경 논리는 다음과 같다.

1. receiver thread는 `on_configure()`에서 시작하지만 기존 Ctrl+C 경로에는 확실한 join이 없었다.
2. `std::thread` 객체가 joinable인 상태로 소멸하면 C++ 런타임이 `std::terminate()`를 호출한다.
3. 기존 `on_cleanup()`의 단순 join은 ROS context가 살아 있는 동안 worker가 inactive sleep loop를 계속 돌면 끝나지 않을 수 있었다.
4. 따라서 ROS 전체 상태와 별개인 `receiver_running_`을 추가하고, destructor/cleanup/shutdown이 같은 stop/join helper를 사용하도록 했다.
5. publisher를 reset하기 전에 worker를 먼저 join하여 receive thread가 해제된 publisher를 사용하는 race를 줄였다.
6. classic CAN과 CAN-FD 양쪽 loop에 같은 종료 조건과 timeout 처리를 적용했다.

## 운영 스크립트 상세

대상 파일:

- 현재 호스트: `/home/a/scripts/start.sh`
- v1.1 저장소 권장 위치: PC1 `ros2_ws/scripts/start.sh` 또는 별도 `pc1_ops/scripts/start.sh`

변경 내용:

- POSIX 호환성을 가장한 무명 shell 대신 Bash shebang을 명시했다.
- `set -eo pipefail`, ROS setup, `set -u` 순서로 명령 오류를 조기에 확인한다.
- can0/can1을 먼저 down한 뒤 bitrate를 설정하고 다시 up한다.
- `nullglob`와 배열을 사용하여 `/dev/tty*`, `/dev/video*`가 없을 때 literal glob으로 실패하지 않게 했다.
- Docker socket은 실제 socket일 때만 처리한다.
- 완료 메시지로 bitrate 적용 단계를 운영자에게 알린다.

`chmod 777` 자체는 기존 정책을 보존한 것이며 개선으로 간주하지 않는다. [KNOWN_ISSUES.md](KNOWN_ISSUES.md)에 보안 부채로 기록한다.

## 네트워크 상세

대상 파일:

- `migration_work/config/cyclonedds_pc1.xml`
- 현재 호스트의 `.bashrc` 내 `CYCLONEDDS_URI` export
- NetworkManager의 NIC별 connection binding

CycloneDDS가 두 유선 NIC 중 카메라/NPU망을 선택하면 차량 PC망의 multicast discovery를 볼 수 없었다. 차량망 NIC를 명시적으로 지정한 후 PC3 map loader, localization 및 PC2 perception endpoint를 발견했고 실제 map 메시지를 수신했다.

## 호환성

- 정상 `run_autoware`, `run_bridge`, `run_planning_universe` alias 이름은 유지한다.
- 정상 `autoware.launch.xml` 파일명은 유지한다.
- PC1 역사 모듈 구성인 Planning, Control, API, RViz는 유지한다.
- 물리 CAN 제어 프레임 형식과 ID `0x630`은 변경하지 않았다.
- cruise decode case `0x4F1`은 이번 변경에서 수정하지 않았다.
- gear raw-value 확장은 이번 변경 이전에 존재했으므로 별도 차량 CAN 검증이 필요하다.

## 릴리스 판정

현재 상태는 `RELEASE_CANDIDATE_WITH_BLOCKERS`이다. receiver build와 로컬 기능은 확인했지만, CAN watchdog/전역 TX topic, `max_vel=36.0`, sample vehicle geometry, MRM heartbeat 및 start_planner safe path 문제를 해결하거나 명시적으로 승인하기 전에는 실차 주행 release PASS로 표시하면 안 된다.

전체 snapshot에는 이번에 새로 수정하지 않은 backup, report, pycache, local RViz/parameter 차이도 포함된다. 따라서 `_a` commit의 모든 diff가 검증된 v1.1 기능 변경이라는 의미는 아니다.
