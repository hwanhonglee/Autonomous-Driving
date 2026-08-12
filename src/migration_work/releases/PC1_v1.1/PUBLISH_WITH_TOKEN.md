# Publish PC1 v1.1 with a GitHub PAT

## 결론

별도 clone을 만들지 않는다. `/home/a/autoware`와 `/home/a/ros2_ws`에서 각각 준비된 `push_v1_1.sh`를 실행한다.

```text
Autoware branch: h2_i/IONIQ_EV_308/PC1_rocket/autoware_universe/v1.1
Autoware tag:    IONIQ_EV_308_PC1_a
ros2_ws branch:  h2_i/IONIQ_EV_308/PC1_rocket/ros2_ws/v1.1
ros2_ws tag:     IONIQ_EV_308_PC1_r
History policy:  current-files-only independent snapshot
```

기존 307 v1.0 branch와 `IONIQ_EV_307_PC1_a` tag는 수정하거나 force-push하지 않는다.

## 1. 처음 한 번: 두 작업 디렉터리 준비

```bash
cd /home/a/autoware
bash src/migration_work/releases/PC1_v1.1/PREPARE_IN_PLACE.sh
```

이 helper는 별도 staging clone 대신 현재 작업 디렉터리에서 다음을 수행하도록 만든 것이다.

- 현재 Autoware 파일만 독립 snapshot commit으로 기록
- 현재 `/home/a/autoware` snapshot을 308 PC1 v1.1 branch에 기록
- `build/`, `install/`, `log/`와 모든 nested `.git` metadata 제외
- nested checkout의 실제 파일은 일반 tracked 파일로 평탄화
- `/home/a/ros2_ws`에 ros2_socketcan, ops payload, v1.1 docs 준비
- 각 디렉터리에 경로/branch/tag를 검증하는 `push_v1_1.sh` 설치
- 두 최종 commit과 lightweight `_a`/`_r` tag를 로컬에 생성하되 push/PAT 사용은 하지 않음

준비 스크립트가 오류를 내면 push 단계로 넘어가지 않는다. 특히 secret 후보, 100 MB 초과 파일 또는 예상하지 않은 repository 상태를 임의로 우회하지 않는다. 이미 공개된 308 `_a` ref는 본 문서의 예상 old SHA와 일치할 때만 검토 PR/최종 tag 절차로 갱신한다.

## 2. 준비 결과 확인

Autoware:

```bash
cd /home/a/autoware
git status -sb
git branch --show-current
git log --oneline --decorate -3
git remote -v
git ls-files -s | awk '$1 == 160000 { print }'
```

확인할 값:

- 현재 branch가 `h2_i/IONIQ_EV_308/PC1_rocket/autoware_universe/v1.1`
- commit이 parent 없는 독립 snapshot이고 현재 파일 전체를 추적
- gitlink 출력이 없음
- `build/`, `install/`, `log/`가 tracked 상태가 아님
- nested `.git` metadata가 tracked 상태가 아님

ros2_ws:

```bash
cd /home/a/ros2_ws
git status -sb
git branch --show-current
git log --oneline --decorate -3
git remote -v
```

현재 branch가 `h2_i/IONIQ_EV_308/PC1_rocket/ros2_ws/v1.1`인지 확인한다.

## 3. PAT 원칙

- 가능하면 `hwanhonglee/Autonomous-Driving` repository contents write 권한만 가진 fine-grained PAT를 사용한다.
- PAT를 source, script, `.env`, remote URL, commit message, shell history 또는 채팅에 붙여 넣지 않는다.
- HTTPS push가 묻는 `Username`에는 GitHub username, `Password`에는 PAT를 입력한다.
- terminal password prompt에 입력한 값은 화면과 command history에 표시되지 않는다.
- 평문 영구 저장인 `credential.helper store`는 사용하지 않는다.

원하면 게시 중에만 메모리 cache를 사용한다.

```bash
git config --global credential.helper 'cache --timeout=1800'
```

## 4. Autoware push

```bash
cd /home/a/autoware
./push_v1_1.sh
```

인증 prompt 예시:

```text
Username for 'https://github.com': <GitHub username>
Password for 'https://<username>@github.com': <personal access token>
```

Autoware의 최신 변경은 먼저 review branch에서 기존 308 PC1 Autoware v1.1 branch를 base로 Draft PR을 만든다. PR 병합 전 `IONIQ_EV_308_PC1_a`는 최초 snapshot `2523b662...`에 유지한다. 병합 후 원격 tag가 여전히 그 old SHA인지 확인하고, 사용자가 승인한 한 번의 조건부 tag 갱신으로 최종 v1.1 HEAD를 가리키게 한다. 307 ref는 변경하지 않는다.

## 5. ros2_ws push

Autoware push가 성공한 뒤 별도로 실행한다.

```bash
cd /home/a/ros2_ws
./push_v1_1.sh
```

helper는 준비 단계에서 최종 commit에 붙인 `IONIQ_EV_308_PC1_r` tag와 ros2_ws branch를 함께 push하고 확인한다. `_a`와 `_r`은 서로 다른 commit이므로 한 repository의 tag를 다른 작업 디렉터리에서 만들지 않는다.

## 6. remote 결과 확인

각 helper가 정상 종료했더라도 다음 네 ref가 모두 존재하는지 확인한다.

```bash
git -C /home/a/autoware ls-remote --heads origin \
  h2_i/IONIQ_EV_308/PC1_rocket/autoware_universe/v1.1
git -C /home/a/autoware ls-remote --tags origin IONIQ_EV_308_PC1_a
git -C /home/a/ros2_ws ls-remote --heads origin \
  h2_i/IONIQ_EV_308/PC1_rocket/ros2_ws/v1.1
git -C /home/a/ros2_ws ls-remote --tags origin IONIQ_EV_308_PC1_r
```

작업 후 메모리 credential cache를 비운다.

```bash
git credential-cache exit
```

## 전체 snapshot 주의

준비 전 감사 기준 Autoware 대상은 약 546 MiB, 8,546개 일반 파일이며 48 MB PCD 파일 두 개가 포함된다. 단일 파일 100 MB 초과는 없었지만 최초 push에 시간이 걸릴 수 있다. backup, report, pycache와 테스트 자료도 사용자가 요청한 현재 snapshot의 일부로 포함되므로, 공개 전 `git status`와 staged/committed tree를 직접 검토한다.

이 snapshot은 미검증 설정도 보존한다. `max_vel: 36.0`, local RViz 변경, gear mapping 및 backup launch의 포함은 안전 승인이나 실차 주행 PASS를 뜻하지 않는다.

## token 또는 credential 노출 시

push 전 검사:

```bash
rg -n --hidden --glob '!.git/**' \
  'github_pat_|ghp_|https://[^/@]+:[^/@]+@github\.com|BEGIN (RSA |OPENSSH |EC )?PRIVATE KEY' \
  /home/a/autoware /home/a/ros2_ws
```

문서의 검사 예시 문자열 자체가 결과에 잡힐 수 있으므로 match 경로와 문맥을 확인한다. 실제 PAT, password, private key가 보이면 push하지 않는다. 이미 노출했다면 GitHub에서 즉시 revoke하고 새 token을 만든 뒤 history에서도 제거한다.
