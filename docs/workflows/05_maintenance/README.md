# 05 — 환경·검사·정리

<!-- HH_260906 - Restrict cleanup to verified generated caches, never raw evidence or active training files. -->

상태 확인: `python3 tools/workspace.py status --remote`

전체 실행 파일 목록 갱신: `python3 tools/workspace.py catalog`

최신 구조 이전: [레거시 제거 원장](../../workspace-layout-migration.md).
현재 상태: [루트 링크 16개 모두 제거 — 남은 0개·최종 검증](link_cleanup_20260925.md).
이전 단계: [data 링크 제거·전체 회귀 및 설치 검증](link_cleanup_20260924.md).
이전 단계: [문서 바로가기 제거·전체 회귀 재검사](link_cleanup_20260923.md).
이전 단계: [runtime 링크 3개 추가 제거·남은 5개](runtime_alias_retirement.md).
이전 단계: [소스 경로 5개 제거](source_alias_retirement.md).
원장 검사: `python3 tools/audit_legacy_layout.py`.
문서 링크 검사: `python3 tools/check_workspace_links.py`.

회귀 검사 예:

```bash
source tools/e2e/env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CUDA_VISIBLE_DEVICES='' \
  python3 tools/workspace_python.py -m pytest -- -q -p no:cacheprovider \
  learning/tests/test_mapping.py learning/tests/test_mapping_projection.py
```

정리 내역: [읽기 쉬운 요약](cleanup_summary.md) · [원본 해시와 상세 기록](cleanup_record.json).
루트 구조의 실제 이동: [이동·복구 설명](root_reorganization.md).
이번 정리는 캐시를 압축 보관 후 제거했고, 확인된 빈 실패 폴더만 제거했습니다.
학습 데이터·checkpoint·원시 실패 기록·ROS 빌드 산출물·원격 환경·시스템 /tmp는 정리 대상이 아닙니다.

복구할 때는 기록의 ZIP을 **새 빈 복구 폴더**에 풀어 확인한 뒤 필요한 항목만 옮기세요.
재생성된 캐시나 다른 파일 위에 통째로 덮어쓰지 마세요.
