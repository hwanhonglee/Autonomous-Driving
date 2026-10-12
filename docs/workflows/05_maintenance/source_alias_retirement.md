# 레거시 제거 우선 진행 — 소스 경로 5개 추가 제거

후속 상태: [build/install/log까지 제거해 총 11개 제거·5개 남음](runtime_alias_retirement.md).
아래의 8개 남음은 이 단계 완료 당시 기록입니다.

<!-- HH_260906 - Prioritize alias retirement over new collection/training and preserve every real target. -->

2026-09-22 사용자 요청에 따라 **남은 레거시 경로 정리를 데이터 보강보다 먼저** 진행합니다.
실행 중이던 원격 AG 학습과 결과 수집기는 유지했으며, 새 학습·데이터 수집·패키지 설치는 시작하지 않았습니다.

## 실제 제거한 것

| 제거한 루트 링크 | 보존된 실제 폴더 |
|---|---|
| `src` | `autonomy/src` |
| `autoware_e2e_vad_launch` | `autonomy/autoware_e2e_vad_launch` |
| `cmake` | `autonomy/cmake` |
| `include` | `autonomy/include` |
| `patches` | `autonomy/patches` |

이전의 `config/tests/workflows` 3개와 합쳐 **16개 중 8개 제거, 8개 남음**입니다.
위 표의 링크만 해제했습니다. 링크 대상 소스·설정·패치·데이터 파일은 삭제하지 않았습니다.
복구하려면 해당 옛 이름이 비어 있는지 확인한 뒤 표의 상대 경로로 링크를 다시 만들 수 있습니다.

## 어떤 검증을 했나요?

- 실행·빌드·검사 코드의 명시적 루트 소스 경로를 canonical 위치로 전환.
- 빌드/설치 내부 심볼릭 링크 **3,233개**를 동일한 실제 대상으로 재연결.
  후속 검사에서 누락·변경된 대상 **0개**.
- 링크 5개가 없는 상태에서 관련 검사 **323개 통과**.
- 주요 ROS 패키지 6개 검색, 공유 라이브러리 12개 의존성, 설치된 Python 패키지 import,
  launch 2개 `--show-args` 파싱 통과. 노드·CARLA·차량 제어는 시작하지 않음.
- 추가로 경로를 변경한 테스트 파일 전체와 bootstrap/경로 도구를 포함한 48개 파일의 확대 검사:
  **864개 통과, 1개 건너뜀**, 123.72초. 앞선 323개와 중복되므로 합산하지 않습니다.
- 새 소스 경로용 bootstrap·build 입력과 초보자 가이드 수정. 기존 build/install 캐시 전체 이전 완료는 아님.

처음 두 제거 검사는 실패해 링크를 복구했습니다. 누락된 문자열 경로·옛 구조의 테스트 fixture·
검사 자식 프로세스의 잠금 상속을 수정한 뒤 세 번째 검사에서 통과했습니다.
확대 검사에서는 package-local `config`를 workspace `learning/config`로 잘못 읽던 기존 경로와
문서의 옛 명령어를 기대하던 검사를 발견해 수정했습니다. 검사 삭제나 합격 기준 완화는 하지 않았습니다.

과거 manifest의 식별 문자열과 원본 SHA는 그대로 보존합니다. 파일을 읽는 지점에서만 새 소스 위치로
해석하도록 했으며, 옛 이름의 폴더·새 호환 링크를 만들지 않습니다. 기록된 파일 경로와 현재 위치를
비교할 때도 같은 실제 파일인지 확인하고 SHA·기록 내용은 동일해야 합니다.

## 남은 8개와 다음 순서

| 대상 | 필요한 후속 작업 |
|---|---|
| `build/install/log` | 기본 빌드·설치 진입점을 새 prefix로 일치시키고 기존 cache/hook 의존성을 해소. 기존 결과는 보존 |
| `tools/portable_e2e` | 실행 명령·Python import·설치 검색 경로 전환. CARLA의 `tools` 패키지와 이름 충돌 검증 |
| `data/datasets/artifacts` | 데이터 manifest·자산·결과 조회의 경로 전환. 원본 데이터와 과거 기록의 해시 유지 |

이 항목들이 남았으므로 **레거시 정리 완료 또는 모든 과거 실행 파일 호환을 주장하지 않습니다**.
새 prefix의 로딩 성공만으로 기존 캐시를 지우지 않습니다. 데이터 보강으로 작업을 전환하지 않고,
남은 경로 전환 → 링크 없는 검사 → 제거를 우선합니다.

작업 중 기존 AG 학습은 종료됐고 결과 수집도 끝났습니다. AF 대비 회전·직진 오차가 악화돼
채택하지 않았으며, 추가 학습은 실행하지 않았습니다. 이 결과 때문에 정리 우선순위를 바꾸지 않습니다.

## 기록과 재현

- [실제 제거 대상 JSON](autonomy_source_retirement_20260922.json)
- 코드 변경 전 보관: `.workspace_archive/retire_autonomy_sources_20260922/`,
  `.workspace_archive/retire_autonomy_sources_followup_20260922/`.
- 원본 심볼릭 링크 문자열과 변경 기록: 위 첫 폴더의 `changes.json`.
- 검사 로그: `storage/experiments/validation/2026-09-22/legacy_retirement/source_*`.
- 현재 루트 상태: `python3 tools/audit_legacy_layout.py`.

이전 변경과 새 변경을 Git에 아직 커밋하거나 push하지 않았습니다.
