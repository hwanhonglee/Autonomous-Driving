# 링크 정리 — 문서 바로가기 제거, 실행 호환 검증

<!-- HH_260906 - Report filesystem deletion, source-path fixes and unresolved replay gates separately. -->

## 현재 결론

이 문서는 9월 23일 기록입니다. 이후 `data` 링크를 제거한 [9월 24일 최신 상태](link_cleanup_20260924.md)를 먼저 확인하세요.

**작업별 안내의 중복 바로가기 12개는 제거했습니다. 루트 레거시 5개는 아직 남아 있습니다.**
둘은 별도 집계입니다. 루트 16개 중 제거된 수는 기존과 같은 11개입니다.

| 구분 | 상태 |
|---|---|
| `docs/workflows` 안 중복 파일·폴더 바로가기 | 12개 제거, 남은 바로가기 0개 |
| 루트 호환 링크 | `scripts`, `portable_e2e`, `data`, `datasets`, `artifacts` 5개 남음 |
| 실제 모델·데이터·원시 결과 | 이동·삭제하지 않음 |
| 새 학습·패키지 설치·SSH·GPU 작업 | 실행하지 않음 |
| Git commit/push | 하지 않음 |

바로가기를 없애기 전 안내서의 참조를 확인했고, 제거 후 원본 대상의 inode가 같은지 검사했습니다.
[제거한 정확한 경로·복구용 상대 대상](workflow_shortcut_retirement_20260923.json)을 보존했습니다.
복구가 필요하면 기록의 각 `path`가 비어 있는지 확인한 뒤 `target`을 가리키는 symlink만 다시 만들 수 있습니다.
원본 파일은 삭제하지 않았습니다.

## 함께 수정한 실제 문제

1. 테스트들이 삭제된 `tests` 패키지를 import하던 것을 `learning.tests`로 전환했습니다.
2. 실제 소스 파일을 열 때 루트 symlink를 넘기던 테스트를 canonical 위치로 전환했습니다.
   임시 테스트 저장소·원격의 고정 레이아웃은 별도 유지합니다. 논리적 파일 ID와 기대 SHA는 바꾸지 않습니다.
3. 현재 설정·주행 데이터 준비·전송 검증·수집 계획에서 확인된 실제 위치 참조를 수정했습니다.
4. `workspace_python.py`는 변경하면 안 되는 연구 코드의 바이트를 보존하면서
   실제 폴더에서 import하도록 돕습니다. **Python 이름 호환은 남아 있으며, 파일 I/O 전체를 대신 변환하지 않습니다.**
5. `check_workspace_links.py`로 전체 문서의 로컬 목적지를 반복 검사할 수 있게 했습니다.

사용 예:

```bash
python3 tools/audit_legacy_layout.py
python3 tools/check_workspace_links.py
source tools/e2e/env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' \
  python3 tools/workspace_python.py -m pytest -- -q -p no:cacheprovider learning/tests
```

`--fix`를 명시하지 않은 문서 검사는 읽기 전용입니다. 외부 URL·문서 앵커·HTML 링크는 검사 범위 밖입니다.
ROS launch 검사 폴더는 같은 이름의 테스트 파일이 있어 별도 실행합니다.

## 검증 기록

이번에 남은 링크를 지우기 **전** 전체 연구 검사를 실행했을 때:
**8,035 통과 / 206 실패 / 105 오류 / 8 건너뜀**(435.64초)이었습니다.
기존 일부 검사 통과가 전체 연구 도구의 정상 작동을 뜻하지 않는다는 점을 확인했습니다.

수정 중 같은 실패 파일 40개의 재검사에서는:
**1,548 통과 / 30 실패 / 오류 0**까지 확인했습니다. 이는 중간 결과이며 전체 최종 결과와 별개입니다.
남은 고정 소스 SHA를 현재 파일에 맞춰 바꾸거나 symlink 거부 검사를 끄지 않았습니다.

전체 재검사 결과는 **8,327 통과 / 29 실패 / 오류 0 / 8 건너뜀**(508.89초)입니다.
이후 확인된 경로 처리 예외를 수정하고 실패했던 29개를 모두 다시 실행해
**9개 해소 / 20개 미해결**을 확인했습니다. 마지막 수정 후 전체 8천여 개를 또 실행한 것은 아닙니다.

| 추가 확인 | 결과 |
|---|---|
| 수정된 학습 명령·소스 출처·지도 참조 등 6개 테스트 파일 | 305개 통과 |
| 데이터 준비·frozen-selector 관련 6개 테스트 파일 | 173개 통과 |
| 경로·문서·Python 진입점 경계 | 19개 통과 |
| 과거 deadline을 포함한 안전 조건 | 70개 통과 — 테스트 시계만 고정, 실제 마감 시각은 유지 |
| ROS launch 패키지 | 373개 통과 |
| 설치 환경 | 282개 패키지 검색, 12개 라이브러리 의존성, Python/CARLA import 정상 |
| launch·빌드 출처 | 인자 파싱 2개, launch 패키지만 재빌드 0.66초, 출처 검증 2개 정상 |
| 문서 | 315개, 로컬 목적지 5,363개, 끊김·루트 legacy 목적지 0개 |
| 보호된 과거 분석 소스 | 4개 SHA가 변경 전과 동일 |

위 테스트 묶음은 서로 겹치므로 통과 수를 합산하지 않습니다. CARLA/ROS 노드를 켜거나 차량을 주행한 검사가 아닙니다.
경로 수정 중 새로 발견된 5개 실패(원격 명령 문자열 4개, 외부 지도 자산 1개)도 해당 테스트를 복구·재검사했습니다.
모델 품질·주행 기능의 통과 수가 증가한 것으로 해석하면 안 됩니다.

남은 20개는 다음과 같습니다. 검사 삭제·skip·기대 SHA 변경으로 통과 처리하지 않았습니다.

| 실패 분류 | 개수 | 다음 해결 경계 |
|---|---:|---|
| 과거 조향·정지 분석의 `ROOT/옛_경로` 읽기 | 8 | 고정 소스의 실행 환경과 현재 실제 파일 조회를 분리 |
| 기존 DEV/회복 데이터의 고정 경로·기록 비교 | 6 | 원본 JSON을 유지하고 소스/증거 위치 해석을 명시 |
| 과거 곡률 계산 소스 closure 불일치 | 2 | 당시 소스 묶음 확인, 현재 코드로 해시 재고정 금지 |
| 과거 stopmix 요약기 SHA 불일치 | 3 | 원본 요약기 복원·격리 재현, 현재 SHA로 대체 금지 |
| regret 요약의 로컬 순수 함수 소스 읽기 | 1 | 논리적 소스 ID와 실제 파일 위치 분리 |

로그 위치: `storage/experiments/validation/2026-09-23/legacy_retirement/`.

- `baseline_full.log`: 변경 전 전체 검사.
- `canonical_test_paths2.log`, `canonical_imports.log`, `canonical_consumers.log`: 원인별 중간 재검사.
- `full_canonical_final.log`: 최종 연구 검사.
- `launch_final.log`: 별도 ROS launch 패키지 검사.
- `document_links_before.json`: 문서 314개·로컬 링크 5,355개, 깨진 목적지 0개였던 초기 확인.
- `document_links_after.json`: 갱신된 문서 전체 검사.
- `remaining_cases_final.log`: 마지막 29개 재검사, 20개 미해결의 정확한 항목·오류.
- `final_regression_fixes.log`, `final_consumers_targeted.log`, `clock_guard_final.log`: 수정 파일별 후속 검사.
- `runtime_final.log`: 기존 설치·launch·출처 재확인.
- `protected_sources.json`: 보호된 4개 소스의 변경 전/후 SHA 대조.

테스트 경로 일괄 전환 전 원본은 `.workspace_archive/test_source_paths_20260923/`에 보존했습니다.
일회성 변환 스크립트는 그 안 `migration_tools/`로 옮겨 현재 실행 도구 목록에서 제외했습니다.

## 아직 루트 5개를 제거하지 않은 이유

과거 실험에는 옛 경로를 포함한 원본 기록과 실행 코드의 고정 SHA가 같이 있습니다.
경로만 통째로 치환하면 재현 코드의 SHA가 달라지고, 링크만 삭제하면 원본 조회가 끊깁니다.
**다음 작업은 남은 실패를 현재 실행용 경로와 고정 실험 재현용 경로로 분리하는 것**입니다.
원본 기록·고정 소스는 보존하고 읽기 경계에서 위치만 해석한 뒤, 링크 없이 검증되는 항목부터 제거해야 합니다.
새 데이터 수집과 주행·지도 학습보다 이 호환 검증을 먼저 진행한다는 순서는 유지합니다.
