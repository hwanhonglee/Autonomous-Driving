# 실행 도구 — 이 폴더에서 무엇을 찾나요?

<!-- HH_260906 - Explain the original executable directory, not only its workflow shortcuts. -->

| 위치 | 역할 | 시작점 |
|---|---|---|
| `workspace.py` | 학습 상태 조회·파일 분류 | `python3 tools/workspace.py status --remote` |
| `e2e/` | 주행/지도 학습, CARLA 수집, 평가, 자료 생성 | [용도별 전체 파일 목록](../docs/workflows/EXECUTABLES.md) |
| `portable_e2e_runtime/` | 자체 모델을 실행 환경에 연결하는 도구 | 파일별 `--help`와 [모델 안내](../learning/portable_e2e/README.md) |
| `analysis/` | 분석 지원 도구 | 기존 결과를 읽는지 실행 환경을 바꾸는지 파일별 확인 |
| `archive_workspace_caches.py` | 검증한 생성 캐시 압축·정리 | 기본은 dry-run, `--apply`만 실제 정리 |
| `organize_workspace_root.py` | 과거 루트 matrix 결과의 일회성 이동 | 이미 완료했으므로 재실행하지 않음 |
| `audit_legacy_layout.py` | 임시 레거시 개수·경로 검사 | 최종 목표 0개, 런타임 검증과 별개 |
| `check_workspace_links.py` | 문서의 로컬 파일 목적지 검사 | 기본은 읽기 전용, 외부 URL·앵커·실행 경로 검사는 아님 |
| `workspace_python.py` | 실제 위치에서 Python 모듈·도구 실행 | 아래 사용법, 설치·학습을 자동으로 시작하지 않음 |
| `probe_remaining_aliases.py` | 루트 링크를 임시 해제하고 검사 후 복구 | 읽기 전용 명령이 아님. 실행 잠금 아래 유지보수할 때만 사용 |
| `retire_asset_alias.py` | data 링크의 전체 검증·제거 기록 | 2026-09-24 완료한 일회성 작업, 재실행하지 않음 |
| `retire_remaining_aliases.py` | 남은 네 링크의 전체 검증·제거 기록 | 2026-09-25 완료한 일회성 작업, 재실행하지 않음 |
| `canonical_workspace.py` | 원본 소스 바이트를 보존하는 모듈 경로 호환 | `workspace_python.py`가 명시적으로 사용 |

`tools/`는 실제 실행 파일 위치입니다. CARLA에도 `tools`라는 Python 패키지가 있으므로
이 폴더를 무조건 `import tools`로 가져오는 새 코드는 만들지 않습니다.

<!-- HH_260906 - Make the remaining import compatibility explicit rather than calling it legacy-free. -->
Python 도구가 옛 `scripts` import 이름을 쓰는 경우 다음처럼 실행합니다.

```bash
python3 tools/workspace_python.py tools/e2e/plan_unique_right_turn_collection_g.py --help
python3 tools/workspace_python.py -m pytest -- -q learning/tests/test_workspace_python.py
```

이 진입점은 `scripts`라는 **Python import 이름**을 실제 `tools` 폴더에 연결하고,
`portable_e2e`는 실제 `learning`에서 읽습니다. 또한 `canonical_workspace.py`가
이 저장소의 실행 모듈에 한해 `ROOT` 등 대문자 Path 상수와 그 루트에서 조합하는
옛 경로를 실제 위치로 해석합니다. 일반 `Path`/`open`을 전역 수정하지 않으며,
파일시스템 링크·기록의 문자열·원본 소스 바이트·해시 기준은 변경하지 않습니다.
심볼릭 링크·상위 경로 이동·원본 해시 검사는 그대로 적용됩니다.

이는 **파일시스템 링크 제거**이지 호환 코드가 모두 없어진 상태는 아닙니다.
과거 절대 경로 문자열이나 임의의 직접 Python 실행까지 자동 복구하지 않습니다.
재현 시에는 원본 소스 해시뿐 아니라 실행 진입점과 어댑터의 해시도 함께 기록해야 합니다.

`e2e/`의 파일명을 지금 일괄 변경하지 않는 이유는 기존 실행 명령·테스트 import·학습 소스 해시에
사용되기 때문입니다. 사용자가 찾는 분류는 전체 목록에 제공하고 실제 경로는 보존합니다.
모든 `.py`가 독립 실행용은 아닙니다. 원격 전용 학습 스크립트를 로컬 Python에서 실행하지 마세요.

[현재 주행·지도 학습](../docs/assets/validation/2026-09-22/portable_e2e_learning_cycle_v1/24_root_cleanup_and_controlled_learning/README.md)
