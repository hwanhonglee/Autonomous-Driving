# PC1 v1.1 rollback

## 원칙

- v1.0 branch와 `IONIQ_EV_307_PC1_a` tag를 변경하지 않는다.
- v1.1은 새 Autoware/ros2_ws branch와 `IONIQ_EV_308_PC1_a`, `IONIQ_EV_308_PC1_r` tag로 배포한다.
- rollback은 `(copy_org)` 파일을 active source에 덮어쓰는 방식이 아니라 Git commit/tag로 수행한다.
- 실차 process가 실행 중인 상태에서 source/install을 교체하지 않는다.

## 전체 rollback

v1.1 배포 전 실행 중인 launch를 각 terminal에서 Ctrl+C로 정리하고 orphan process가 없는지 확인한다. 이후 v1.0 tag 또는 commit으로 새 worktree를 checkout한다.

```bash
git switch --detach IONIQ_EV_307_PC1_a
```

운영 branch가 필요하면 tag에서 별도 rollback branch를 만든다.

```bash
git switch --create rollback/IONIQ_EV_307_PC1_a IONIQ_EV_307_PC1_a
```

build/install은 별도 디렉터리에서 다시 만든다. v1.1 build 산출물을 v1.0 source와 섞지 않는다.

## 부분 rollback

### receiver lifecycle patch

해당 fix commit만 revert한다.

```bash
git revert <receiver-lifecycle-commit>
```

이 경우 Ctrl+C exit -6와 cleanup hang이 다시 발생할 수 있으므로, rollback 사유와 대체 종료 전략을 기록해야 한다.

### DDS NIC binding

CycloneDDS config를 제거하기 전에 PC1이 어떤 NIC를 선택할지 확인한다. 자동 선택으로 되돌리면 PC2/PC3 graph가 다시 0개가 될 수 있다.

NetworkManager backup profile은 삭제하지 않는다. 현재 active profile을 내리고 검증된 backup profile을 명시적으로 올린다. UUID는 공개 문서가 아닌 현장 rollback 기록에서 확인한다.

### start.sh

v1.0 script를 복원하면 already-UP CAN에서 bitrate 설정이 실패할 수 있다. rollback 후에는 CAN down→bitrate→up 순서를 수동으로 수행해야 한다.

## tag rollback

공개된 `IONIQ_EV_308_PC1_a` 또는 `IONIQ_EV_308_PC1_r` tag를 다른 commit으로 강제 이동하지 않는다. 문제가 발견되면:

1. v1.1 branch에 fix commit 추가
2. 검증
3. 새 tag 이름 협의

이미 공유된 tag를 `git tag -f`와 force-push로 바꾸면 실험 재현성이 깨진다.

## rollback 후 검증

- exact commit SHA와 tag 기록
- `run_autoware`, `run_bridge` alias trace
- Domain/RMW/NIC/Chrony 상태
- can0/can1 state와 bitrate
- receiver/sender/converter process count
- map/localization/perception freshness
- MRM heartbeat와 final command gating
- Ctrl+C orphan 0

rollback은 코드 복구이지 차량 안전 승인 자체가 아니다. v1.0으로 돌아가도 기존 CAN watchdog, sample vehicle geometry, MRM 및 start_planner 문제는 남을 수 있다.
