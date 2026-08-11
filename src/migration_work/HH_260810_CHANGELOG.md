# HH_260810 / PC1 v1.1 change log

이 파일은 현재 PC1 v1.1의 정본 changelog entry point다.

과거 이 파일에는 fail-closed/receive-only 실험 상태가 active configuration처럼 기록되어 있었으나, 해당 구성은 사용자 요구에 따라 모두 원복되었다. 원복된 실험과 그 부작용은 [EXPERIMENT_AND_REVERT_HISTORY.md](releases/PC1_v1.1/EXPERIMENT_AND_REVERT_HISTORY.md)에 보존한다.

## 현재 active 상태

- `run_autoware`: v1.0 direct alias
- `autoware.launch.xml`: v1.0과 byte-identical; Planning, Control, API, RViz 포함
- `run_bridge`: v1.0 direct full RX/TX alias
- `can_brdige.launch.xml`: receiver + sender + control-to-CAN converter
- `start.sh`: can0/can1 500 kbit/s UP
- ROS Domain ID 10, CycloneDDS, vehicle NIC 고정
- start_planner source/param 변경 없음

## 최종 유지 변경

| 영역 | 변경 | 이유 | 상태 |
|---|---|---|---|
| ros2_socketcan receiver | atomic running flag, destructor, shared stop/join, cleanup/shutdown 연동 | Ctrl+C exit -6 및 lifecycle cleanup 교착 방지 | build PASS, full test PARTIAL |
| receive timeout | idle `SocketCanTimeout` 별도 처리 | 정상 idle warning spam 제거 | source/build verified |
| `start.sh` | down→bitrate→up, strict shell, nullglob/존재 검사 | 반복 실행 안정화 | syntax/CAN state verified |
| DDS config | `enp0s31f6` 고정 | 다중 NIC 임의 선택으로 PC2/PC3 graph 소실 | runtime verified |
| release docs | final/reverted/pre-existing/unresolved 분리 | 잘못된 v1.1 commit 방지 | complete |

## v1.1 release 문서

- [README](releases/PC1_v1.1/README.md)
- [Final changelog](releases/PC1_v1.1/CHANGELOG.md)
- [Autoware Universe scope](releases/PC1_v1.1/AUTOWARE_UNIVERSE_CHANGELOG.md)
- [ros2_ws scope](releases/PC1_v1.1/ROS2_WS_CHANGELOG.md)
- [CAN bridge](releases/PC1_v1.1/CAN_BRIDGE.md)
- [DDS/network](releases/PC1_v1.1/DDS_NETWORK.md)
- [Known issues](releases/PC1_v1.1/KNOWN_ISSUES.md)
- [Validation](releases/PC1_v1.1/VALIDATION.md)
- [Commit manifest and `IONIQ_EV_308_PC1_a`/`IONIQ_EV_308_PC1_r` tag plan](releases/PC1_v1.1/COMMIT_MANIFEST.md)
- [Rollback](releases/PC1_v1.1/ROLLBACK.md)

## release status

`RELEASE_CANDIDATE_WITH_BLOCKERS`

v1.0은 보존하며 v1.1은 새 branch로 만든다. 현재 로컬 workspace는 usable Git metadata가 없으므로 fresh clone에 승인된 diff만 이식한 뒤 commit/push/tag한다.
