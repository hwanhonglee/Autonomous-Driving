# PC1 v1.1 commit manifest

## 확정 branch와 tag

```text
Autoware branch: h2_i/IONIQ_EV_308/PC1_rocket/autoware_universe/v1.1
Autoware tag:    IONIQ_EV_308_PC1_a
ros2_ws branch:  h2_i/IONIQ_EV_308/PC1_rocket/ros2_ws/v1.1
ros2_ws tag:     IONIQ_EV_308_PC1_r
```

- 기존 PC1 307 `v1.0` branch와 `IONIQ_EV_307_PC1_a` tag는 보존한다.
- Autoware v1.1은 기존 307 history를 가져오지 않고 현재 `/home/a/autoware` 파일만 담는 독립 snapshot history로 만든다.
- 공개 저장소에 PC1 307 ros2_ws 기준 branch가 없으므로 ros2_ws v1.1은 독립 history로 만든다.
- 기존 307 branch/tag는 절대 이동하지 않는다. 308 `_a` tag는 최초 snapshot `2523b662...`에 이미 공개됐으나, 사용자가 2026-08-12 검증 기록을 포함한 v1.1 최종 커밋으로 한 번 갱신하도록 명시 승인했다. 갱신 전 원격 old SHA를 확인하고, PR 병합 뒤 정확한 새 HEAD로만 조건부 이동한 후 다시 고정한다.
- `_r` tag는 아직 원격에 없으므로 최초 검증 commit에 생성하며 force가 필요 없다.

## 사용자가 확정한 snapshot 범위

이 release는 선택 파일 manifest가 아니다. `_a`에는 `/home/a/autoware`의 현재 작업공간을 다음 정책으로 그대로 담는다.

| 상태 | 범위 | 이유 |
|---|---|---|
| INCLUDE | `/home/a/autoware` root의 source/config/docs | 현재 실행 환경 재현 |
| INCLUDE | `/home/a/autoware/src/**`의 현재 보이는 모든 내용 | 사용자 요청에 따른 전체 source snapshot |
| INCLUDE | backup, `(copy_org)`, `(copy_ioniq_phev)` | 과거 상태와 복구 흔적 보존 |
| INCLUDE | `migration_work/backups`, reports, inventory, test logs | 수정/실험/진단 이력 보존 |
| INCLUDE | `__pycache__`, `*.pyc`, IDE 설정 | “현재 작업공간 전체” 원칙; 권장 source 품질과는 별개 |
| FLATTEN | 모든 하위 repository의 worktree 파일 | submodule/nested Git 없이 일반 파일로 추적 |
| EXCLUDE | root와 하위 경로의 `.git` directory/file | 기존 VCS metadata 중첩 및 gitlink 방지 |
| EXCLUDE | generated `build/`, `install/`, `log/` | 재생성 가능하고 크며 호스트별 결과인 산출물 |
| BLOCK | PAT, password, private key, credential-bearing URL | credential 발견 시 commit/push 자체를 중단 |

따라서 이전 문서의 `DO_NOT_COMMIT`, `QUARANTINE`, 선택적 `INCLUDE` 정책은 파일 포함 범위에는 더 이상 적용하지 않는다. 예를 들어 `max_vel: 36.0`, 로컬 RViz 설정, gear mapping, backup launch도 현재 snapshot에 존재하면 들어간다. 다만 이 사실이 해당 설정의 기능 검증이나 안전 승인을 뜻하지는 않는다.

## nested Git 평탄화

감사에서 formal submodule 또는 `.gitmodules`는 발견되지 않았다. 대신 다음 중첩 checkout이 있었다.

```text
/home/a/autoware/src/universe/autoware.universe/autoware_tools/.git
origin: https://github.com/autowarefoundation/autoware_tools.git
HEAD: a6b16571dbff1c825a8f829ab3c7433e06b2b4c7
```

준비 스크립트는 donor source 파일을 삭제하지 않고 중첩 `.git` metadata만 repository 대상에서 제거한다. 현재 보이는 625개 파일은 일반 파일로 추적하며, nested repository에서 이미 삭제 상태였던 238개 파일은 자동 복구하지 않는다. 최종 index에는 mode `160000` gitlink와 `.gitmodules`가 없어야 한다.

## `_r` snapshot 범위

`/home/a/ros2_ws`는 [PREPARE_IN_PLACE.sh](PREPARE_IN_PLACE.sh)가 다음 배포 묶음으로 준비한다.

```text
src/ros2_socketcan/             # PC1 SocketCAN driver/package tree
scripts/start.sh                # can0/can1 500 kbit/s 초기화
config/cyclonedds_pc1.xml       # PC1 DDS NIC 고정
config/pc1_ros_env.sh           # Domain/RMW/DDS 및 alias 환경
docs/v1.1/                      # 본 release 감사 문서
push_v1_1.sh                    # ros2_ws branch/tag 게시 helper
```

`_a` 전체 snapshot에도 동일 ros2_socketcan source가 존재한다. 두 branch는 archive/배포 단위를 분리하기 위한 것이며, 실기동 overlay에서 같은 package 두 사본을 동시에 source하지 않는다.

## critical-file checksum

`MANIFEST.sha256`은 전체 8천여 파일 manifest가 아니라 다음 핵심 파일 11개의 변조 확인용이다.

- receiver lifecycle/timeout patch 2개와 control-to-CAN converter 1개
- DDS/ros2_ws payload 4개
- v1.0과 동일함을 확인한 launch 4개

전체 snapshot 포함 여부는 준비 후 `git status`, tracked file count, nested `.git`/gitlink 검사로 확인한다. 문서 파일은 `migration_work/releases/PC1_v1.1/` 전체를 포함한다.

## direct in-place 준비와 commit

별도 clone과 기존 branch checkout을 만들지 않는다. 수동으로 `git init`, nested `.git` 이동, 대규모 `git add`를 각각 실행하지 말고, 정확한 대상 경로를 고정한 준비 스크립트를 한 번 실행한다.

```bash
cd /home/a/autoware
bash src/migration_work/releases/PC1_v1.1/PREPARE_IN_PLACE.sh
```

준비 후 반드시 확인한다.

```bash
cd /home/a/autoware
git status -sb
git branch --show-current
git log --oneline --decorate -3
git ls-files -s | awk '$1 == 160000 { print }'
find . -path './build' -prune -o -path './install' -prune -o -path './log' -prune \
  -o -name .git -not -path './.git' -print

cd /home/a/ros2_ws
git status -sb
git branch --show-current
git log --oneline --decorate -3
```

Autoware는 전체 snapshot 하나가 추적 가능하도록 commit하고, ros2_ws는 별도 commit으로 유지한다. 기능 변경의 의미와 미검증 상태는 [CHANGELOG.md](CHANGELOG.md), [AUTOWARE_UNIVERSE_CHANGELOG.md](AUTOWARE_UNIVERSE_CHANGELOG.md), [KNOWN_ISSUES.md](KNOWN_ISSUES.md)에 남긴다.

## push

검증 뒤 각 repository의 helper만 실행한다.

```bash
cd /home/a/autoware
./push_v1_1.sh

cd /home/a/ros2_ws
./push_v1_1.sh
```

helper는 현재 branch, clean/expected state와 원격 ref를 확인해야 한다. `_a`의 기존 308 tag 이동은 예상 old SHA가 정확히 일치할 때만 허용하며, `_r`은 정상 최초 push만 허용한다. GitHub CLI 인증은 credential helper를 통해 사용하고 PAT는 어떤 tracked 파일이나 remote URL에도 기록하지 않는다.
