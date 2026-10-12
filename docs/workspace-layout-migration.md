# 실제 구조 이전과 레거시 제거 원장

<!-- HH_260906 - Temporary links are migration debt, never the desired final layout. -->

## 완료한 것 / 완료하지 않은 것

**최신 상태: 루트 호환 링크 16개 모두 제거, 남은 0개.**
2026-09-25에 마지막 `scripts`, `portable_e2e`, `datasets`, `artifacts` 링크를 제거했습니다.
제거 상태에서 연구 검사 8,372개 통과·기존 실패 10개·기존 건너뜀 8개(새 실패 없음), launch 373개 통과,
설치 패키지 282개·라이브러리 12개·실제 지도 준비 대상 9개의 조회를 확인했습니다. 실제 주행은 아닙니다.
2026-09-23에는 `docs/workflows`의 중복 바로가기 12개를 별도로 제거했습니다(루트 링크 집계와 별개).
전체 연구 검사에서 확인된 실패와 수정 내역은 [최종 제거 기록](workflows/05_maintenance/link_cleanup_20260925.md)을 확인하세요.
Python import/모듈 경로 호환 처리는 남아 있으며, 파일시스템 링크 제거와 구분합니다.
아래 내용은 이전 단계의 이력입니다. 2026-09-24에는 `data` 링크를 제거했습니다.
`build/install/log`까지 제거한 상태에서 관련 검사 347개와 launch 검사 373개(총 720개)가 통과했습니다.
기존 설치 282개의 검색, 주요 라이브러리 12개의 의존성, Python/CARLA import,
launch 인자 파싱 2개와 launch 패키지 재빌드를 확인했습니다.
[이전 runtime 검증·되돌린 변경·남은 원인](workflows/05_maintenance/runtime_alias_retirement.md).

### 이전 단계 기록

실제 폴더 16개를 동일 파일시스템에서 rename했습니다. 원래 디렉터리 inode가 유지됐고 데이터 삭제는 없습니다.
18,046개 내부 symlink를 검사해 경로가 달라진 34개를 재연결했습니다.
실행 중인 원격 학습이나 로컬 결과 감시는 종료하지 않았습니다.

**2026-09-22 후속 검사: 총 8개 링크 제거 완료, 8개 남음. 최종 완료 조건은 0개입니다.**
처음 제거한 `config/tests/workflows`에 이어 `src/autoware_e2e_vad_launch/cmake/include/patches`를
제거했습니다. 두 번째 묶음은 323개 검사와 canonical 설치 로딩 검사를 통과했습니다.
후속 확대 검사는 864개 통과·1개 건너뜀이며 앞선 323개와 중복됩니다.
내부 심볼릭 링크 3,233개는 원본과 같은 대상으로 새 경로를 연결했습니다. 데이터 파일 삭제는 없습니다.
현재 최우선 과제는 레거시 제거이며 새 데이터 수집·학습은 이후로 미룹니다.
제거 상태의 관련 검사 513개 통과. 실패한 앞선 두 시도에서는 링크를 복구했고,
실제 canonical 파일을 검사하도록 수정한 뒤 최종 재검사했습니다. 출처 SHA·symlink 거부 조건을 약화하지 않았습니다.
`autonomy/canonical_rebuild/`에서 최소 런타임 86개(16분 3초)와 launch 1개(2.04초) 빌드 완료.
격리된 shell에서 canonical 패키지 검색 6개, 공유 라이브러리 의존성 12개, Python import,
launch 2개 `--show-args` 파싱을 확인했습니다. 루트 Python 호환 링크는 import 검색에서 배제했습니다.
새 설치의 기본 환경 활성화·실제 노드 실행·전체 링크 제거로 간주하지 않습니다.
VS Code에서 숨겼다는 것을 삭제 완료로 계산하지 않습니다.
최소 런타임 외 전체 소스 재빌드·실제 CARLA 주행은 아직 수행하지 않았습니다.

| 옛 루트 경로 | 실제 새 경로 | 제거 전 필요한 확인 |
|---|---|---|
| src, autoware_e2e_vad_launch | autonomy/ 아래 같은 이름 | 실행 스크립트·패키지 검색을 새 경로로 전환 |
| build, install, log | autonomy/ 아래 같은 이름 | 새 위치에서 깨끗한 빌드·환경 로딩·실행 확인 |
| cmake, include, patches | autonomy/ 아래 같은 이름 | 빌드 옵션·패치 적용 경로 전환 |
| portable_e2e, config, tests | learning/ 아래 같은 이름 | Python 경로·설정·검사 도구의 옛 경로 참조 제거 |
| scripts | tools | `scripts.e2e` import·쉘 명령·루트 계산 전환 |
| data | storage/assets | 2026-09-24 제거 완료, 원본 자산 보존 |
| datasets, artifacts | storage/datasets, storage/experiments | 데이터 보호 경계·원본 기록 조회의 옛 경로 해소 필요 |
| workflows | docs/workflows | 안내·단축 도구·문서 링크 전환 |

## 레거시를 최종 없애는 순서

1. 새 코드·명령·계획은 canonical 경로를 사용합니다. 새 호환 경로를 만들지 않습니다.
2. `tools` 내부 import, 데이터 루트 옵션, 설치 진입점 경로를 전환하고 개별 회귀 검사를 통과시킵니다.
3. 기존 빌드를 보존한 별도 폴더에서 canonical 경로로 빌드합니다. 대규모 빌드를 현 작업에 몰래 포함하지 않습니다.
4. 학습/회수 종료 후 새 경로만 있는 별도 복제 환경에서 import·ROS launch·학습 dry-run·데이터 검사를 수행합니다.
5. 목록의 링크를 **하나씩** 제거하고 같은 검사를 다시 수행합니다. 링크 대상 디렉터리는 삭제하지 않습니다.
6. 루트 legacy 링크 0개와 canonical-only 검사 통과를 함께 확인해야 구조 개편 완료입니다.

과거 실행 원본의 경로 문자열·SHA를 일괄 바꾸면 재현 근거가 훼손됩니다. 원본은 이력으로 보존하고
새 실행용 manifest를 새 경로로 생성합니다. 원격의 frozen source worktree도 별도 이력으로 보존합니다.

## 복구 기록

로컬 `.workspace_archive/layout_v1_20260922/migration.json`에 이동·inode·링크 수정을 기록했습니다.
`symlinks_before.json`에는 이동 전 링크, `reference_edits/`에는 기계적 경로 수정 전 파일이 있습니다.
이전의 dirty Git 상태도 `git_status_before.bin`으로 보관했습니다. git reset/checkout이나 변경 폐기는 하지 않았습니다.

복구는 관련 로컬 작업이 없는 상태에서, 각 루트 항목이 기록된 symlink인지 확인하고 **링크만 해제한 뒤**
실제 폴더를 역순으로 되돌리는 방식입니다. 수정한 내부 링크와 문서·검사 경로도 기록에 맞게 되돌려야 합니다.
이후 추가 수정이 있을 수 있으므로 백업으로 현재 파일을 무조건 덮어쓰지 마세요.

원장 검사: `python3 tools/audit_legacy_layout.py`. 이 검사는 경로 계약 확인이며 완전한 의존성 감사는 아닙니다.
