# PC1 v1.1 release audit

이 디렉터리는 IONIQ EV 308 PC1의 `v1.1` 소스 스냅샷, 변경 이유, 검증 결과와 공개 절차를 기록하는 정본 문서다. 기존 IONIQ EV 307 PC1 `v1.0` branch와 tag는 보존하며 force-push하지 않는다.

## 확정된 저장소 구성

두 작업 디렉터리에서 바로 branch를 준비하고 push한다. 별도 staging clone은 만들지 않는다.

| 작업 디렉터리 | 포함 범위 | branch | tag |
|---|---|---|---|
| `/home/a/autoware` | 현재 Autoware 작업공간 전체 스냅샷 | `h2_i/IONIQ_EV_308/PC1_rocket/autoware_universe/v1.1` | `IONIQ_EV_308_PC1_a` |
| `/home/a/ros2_ws` | PC1 `ros2_socketcan` + 운영 스크립트/config + v1.1 문서 | `h2_i/IONIQ_EV_308/PC1_rocket/ros2_ws/v1.1` | `IONIQ_EV_308_PC1_r` |

사용자 결정에 따라 두 branch 모두 현재 PC1 파일만 담는 독립 snapshot history로 만든다. 307 PC1 v1.0 commit은 변경 내용 설명을 위한 비교 기준으로만 남기며, 새 308 branch의 parent로 연결하거나 기존 branch를 checkout하지 않는다.

## 가장 짧은 실행 순서

처음 한 번만 준비 스크립트를 실행한다.

```bash
cd /home/a/autoware
bash src/migration_work/releases/PC1_v1.1/PREPARE_IN_PLACE.sh
```

그 뒤 각 디렉터리에서 해당 branch만 push한다.

```bash
cd /home/a/autoware
./push_v1_1.sh

cd /home/a/ros2_ws
./push_v1_1.sh
```

GitHub가 인증을 요구할 때 username과 PAT를 직접 입력한다. PAT는 스크립트, remote URL, 문서, shell history 또는 채팅에 저장하지 않는다. 자세한 사전 확인과 실패 시 조치는 [PUBLISH_WITH_TOKEN.md](PUBLISH_WITH_TOKEN.md)에 기록했다.

## 전체 스냅샷 정책

사용자 요청에 따라 `_a` branch는 선택된 수정 파일만 담는 release가 아니라 `/home/a/autoware`의 현재 내용을 그대로 보존하는 스냅샷이다.

- root 설정과 `src/` 전체를 포함한다.
- backup, `migration_work` reports/inventory/test logs, `__pycache__`, `*.pyc`, `(copy_org)`, `(copy_ioniq_phev)`, IDE 설정도 현재 존재하면 포함한다.
- `build/`, `install/`, `log/` 같은 재생성 가능한 산출물은 제외한다.
- root와 하위 directory의 `.git` metadata는 제외하고, 하위 repository의 실제 파일은 일반 파일로 평탄화한다.
- credential, PAT, private key가 발견되면 제외한 채 조용히 진행하지 않고 push를 중단한다.

2026-08-11 준비 전 감사에서 `/home/a/autoware`의 위 정책 대상은 약 546 MiB, 8,546개 일반 파일이었다. 단일 파일 100 MB 초과는 없었지만 48 MB PCD 두 개를 포함하므로 clone/push 시간이 길고 저장소가 큰 상태임을 의도적으로 받아들인 스냅샷이다.

`autoware_tools`는 정식 `.gitmodules` submodule이 아니라 독립 `.git`을 가진 중첩 checkout이었다. 준비 과정은 그 `.git` metadata만 제거하고 현재 보이는 625개 파일을 일반 파일로 포함한다. 중첩 repository 기준으로 이미 삭제 상태였던 238개 파일은 복원하지 않는다.

## 최종 운영 상태

- `run_autoware`는 기존 직접 alias이며 PC1 역사 구성인 Planning, Control, Autoware API, RViz를 실행한다.
- `autoware.launch.xml`은 PC1 v1.0과 byte-identical이다. 이 파일은 v1.1의 기능 변경이 아니다.
- `run_bridge`는 기존 직접 alias이며 CAN receiver, CAN sender, `twistController2VCU2EPS2ACC_node`를 모두 실행한다.
- `run_bridge`는 receive-only가 아니다. `/control/command/control_cmd`를 CAN ID `0x630`으로 변환하여 물리 `can0`에 송신할 수 있다.
- `start.sh`는 `can0`과 `can1`을 500 kbit/s로 설정하여 UP 상태로 만든다.
- ROS 2는 Humble, Domain ID 10, `rmw_cyclonedds_cpp`를 사용한다.
- PC1 CycloneDDS는 차량 PC망 NIC `enp0s31f6`에 고정한다.

## v1.1에 유지할 변경

1. SocketCAN receiver worker의 종료 경로를 명시적으로 stop/join하여 Ctrl+C 및 lifecycle cleanup 시 `std::terminate`/SIGABRT와 cleanup 교착을 줄였다.
2. 정상 idle receive timeout은 오류가 아니므로 경고 로그를 발생시키지 않도록 분리 처리했다.
3. `start.sh`를 반복 실행 가능하게 만들고, 존재하지 않는 장치 glob 때문에 중단되지 않도록 보강했다.
4. CycloneDDS NIC를 차량 PC망에 고정하여 다중 NIC 환경에서 PC2/PC3 discovery가 사라지는 문제를 해결했다.
5. 코드와 운영 상태, 원복된 실험, 검증 한계 및 미해결 결함을 v1.1 문서로 분리했다.

## 포함되지만 v1.1 기능 변경으로 주장하지 않는 것

- `autoware.launch.xml`, `planning_simulator.launch.xml`, `can_brdige.launch.xml`, `socket_can_bridge.launch.xml`: 중간 실험을 제거하고 v1.0과 동일하게 복원했다.
- start_planner 소스와 파라미터: 수정하지 않았고 읽기 전용 원인 분석만 수행했다.
- `common.param.yaml`의 `max_vel: 36.0`, 로컬 RViz 설정, dual gear raw mapping: 이번 작업 전에 존재했던 미검증 로컬 상태다.
- backup/pycache/report/test 자료: 기능 변경이 아니라 “현재 작업공간 전체 보존” 요구 때문에 포함한다.
- PC2 LiDAR relay와 PC3 MRM 복구: 다른 PC의 런타임/구성 문제이며 PC1 코드 변경이 아니다.

## 문서 구성

- [CHANGELOG.md](CHANGELOG.md): 최종 v1.1 기능 변경
- [AUTOWARE_UNIVERSE_CHANGELOG.md](AUTOWARE_UNIVERSE_CHANGELOG.md): `_a` 전체 스냅샷 범위
- [ROS2_WS_CHANGELOG.md](ROS2_WS_CHANGELOG.md): `_r` 운영/드라이버 범위
- [EXPERIMENT_AND_REVERT_HISTORY.md](EXPERIMENT_AND_REVERT_HISTORY.md): 적용했다가 되돌린 실험과 부작용
- [CAN_BRIDGE.md](CAN_BRIDGE.md): CAN 데이터 및 제어 경로
- [DDS_NETWORK.md](DDS_NETWORK.md): Domain 10과 네트워크 환경
- [KNOWN_ISSUES.md](KNOWN_ISSUES.md): 현존 결함과 릴리스 게이트
- [VALIDATION.md](VALIDATION.md): 빌드·테스트·런타임 및 스냅샷 증거
- [COMMIT_MANIFEST.md](COMMIT_MANIFEST.md): 포함/제외 정책과 branch/tag
- [ROS2_WS_MANIFEST.sha256](ROS2_WS_MANIFEST.sha256): 독립 `_r` tree의 핵심 파일 checksum
- [PUBLISH_WITH_TOKEN.md](PUBLISH_WITH_TOKEN.md): direct in-place PAT push 절차
- [ROLLBACK.md](ROLLBACK.md): v1.0 보존 및 v1.1 롤백 절차

## 공개 저장소 주의

비밀번호, SSH 키, 토큰, 실제 차량 CAN 로그 원문은 공개하지 않는다. NetworkManager UUID와 NIC MAC 주소도 공개 문서에서는 제외했다. 사설 IP는 분산 ROS 구성 재현에 필요한 범위만 기록했다. 전체 스냅샷 요구는 credential 공개 권한까지 의미하지 않는다.

## 최신 실차 판정

2026-08-12 3-PC 실차 연동에서 localization과 planning/control 체인이 실제로 활성화되고 차량이 AUTO/DRIVE로 움직이는 것까지 확인했다. 그러나 같은 실행에서 PC2 perception stack dropout, NDT/trajectory deviation, MRM EMERGENCY_STOP이 발생했으므로 이는 주행 합격이 아니라 재현된 통합 장애 기록이다. 상세 시간선과 차단 조건은 [VALIDATION.md](VALIDATION.md)와 [KNOWN_ISSUES.md](KNOWN_ISSUES.md)를 정본으로 사용한다.
