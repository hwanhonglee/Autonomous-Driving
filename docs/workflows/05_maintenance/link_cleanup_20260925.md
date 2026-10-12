# 루트 호환 링크 최종 제거 — 2026-09-25

<!-- HH_260906 - Separate physical alias retirement, execution compatibility, and scientific qualification. -->

현재 상태: **제거 완료. 루트 호환 링크 16개 모두 제거했고, 남은 링크는 0개입니다.**
마지막 네 링크를 제거한 상태에서 전체 검사와 설치·지도 조회를 완료했습니다.
데이터 파일 삭제는 0개이며, 실제 대상 디렉터리의 device/inode도 유지됐습니다.

## 최종 결과

| 확인 항목 | 결과 |
|---|---|
| 연구 검사 | 8,372개 통과 · 기존 미해결 10개 · 기존 건너뜀 8개 · 새 실패/수집 오류 0개 |
| Autoware launch 검사 | 373개 통과 |
| 기존 설치 조회 | 패키지 282개 · 주요 라이브러리 의존성 12개 · Portable/CARLA import 확인 |
| launch 실행 진입점 | 2개 `--show-args` 확인 · launch/config 패키지만 재빌드 |
| 실제 지도 자료 조회 | 기존과 같은 9개 준비 가능 · 전체 19개 중 기존 차단 10개 유지 |
| 문서 목적지 검사 | 문서 317개 · 로컬 링크 5,380개 · 옛 목적지/깨진 목적지 0개 |
| 원본 보호 | 데이터·모델·원시 기록 보존 · 고정 해시 기준 완화 없음 |

[최종 제거 기록](final_alias_retirement_20260925.json)과
[최종 검증 로그](../../../storage/experiments/validation/2026-09-25/legacy_retirement/final_verification2)를 보관했습니다.
코드·설정 해시 목록은 최종 검사 전후 동일함을 확인했습니다.

전날 기존 실패 20개 중 10개가 경로 전환으로 정상화됐습니다. 이는 모델 주행 성능 향상이 아니라
검사 코드가 실제 파일을 올바르게 읽게 된 결과입니다. **전체 연구 검사가 모두 통과한 것은 아닙니다.**
남은 10개는 기존의 곡률 진단 소스 고정 2개, 과거 moving 자료 경로·원본 결합 검증 5개,
STOPMIX 요약 도구 고정 해시 3개입니다. 정확한 실패 ID는 최종 제거 기록에 있습니다.
이번에는 새 학습·주행·원격 변경을 하지 않았습니다.

## 이번 대상

| 제거 대상 | 실제 파일이 보존되는 위치 |
|---|---|
| 루트 `scripts` 링크 | [tools](../../../tools/README.md) |
| 루트 `portable_e2e` 링크 | [learning/portable_e2e](../../../learning/portable_e2e/README.md) |
| 루트 `datasets` 링크 | `storage/datasets` |
| 루트 `artifacts` 링크 | `storage/experiments` |

앞서 제거한 루트 링크 12개에 이번 네 개를 더해 원래 16개를 모두 제거했습니다.
`docs/workflows`의 중복 바로가기 12개는 별도 집계이며 이미 제거했습니다.
ROS 설치·가상환경·외부 자산에 필요한 정상적인 내부 심볼릭 링크는 삭제 대상이 아닙니다.

## 무엇을 바꿨나

1. Shell 실행·빌드·수집 진입점은 `tools`의 실제 파일을 사용합니다.
2. 현재 지도 설정과 과거 실험 자료를 읽는 경로를 새 저장소 위치에 맞췄습니다.
   원본 JSON, 경로 좌표, 모델 파일, 데이터셋 내용은 수정하지 않았습니다.
3. Python은 명시적인 [실행 진입점](../../../tools/workspace_python.py)과
   [모듈 경로 어댑터](../../../tools/canonical_workspace.py)를 사용합니다.
   `scripts` import 이름과 저장소 모듈의 `ROOT` 기반 경로를 실제 위치로 해석합니다.
   일반 `Path`/`open`을 전역 수정하거나 파일시스템 링크를 새로 만들지 않습니다.
4. 해시가 고정된 핵심 분석 코드 4개와 과거 학습 캠페인 실행 코드의 원본 바이트를 보존했습니다.
   해시 검사·데이터 보호·기존 성능 합격 기준을 낮추지 않았습니다.
5. 테스트용 가상 저장소도 새 구조를 검사하도록 전환했습니다.
   과거 원격 구조를 검증하는 테스트는 정확한 원본 설정 바이트를 임시 폴더에 복사해 검사합니다.

**파일시스템 링크 제거와 Python 호환 코드 제거는 다릅니다.**
이번 작업은 전자입니다. 과거 소스 식별자·문자열·`scripts` import 이름은 일부 남아 있습니다.
과거 기록을 임의로 다시 쓰지 않았으며, 임의의 옛 절대 경로나 모든 직접 Python 실행을 자동 복구하지 않습니다.
재현 시에는 원본 코드뿐 아니라 실행 진입점·어댑터 해시도 함께 확인해야 합니다.

## 검증 범위

- 실행 잠금 아래 네 링크를 제거한 상태에서 전체 연구 검사, launch 검사, 설치 조회와 지도 준비를 수행합니다.
- 최종 검사 전후의 코드·설정 해시 목록과 실제 대상 디렉터리의 device/inode가 같아야 합니다.
- 기존 실패 목록 밖의 새 실패, 추가 건너뜀, 수집 오류, 불완전한 실행은 제거 확정을 막습니다.
- Autoware 노드·CARLA 주행·GPU 학습·원격 SSH 작업·패키지 설치는 시작하지 않습니다.
  기존 의존성을 이용한 launch/config 패키지 재빌드만 실행 경로 검사에 포함됩니다.
- 모델 성능 향상, 모든 Town 주행 성공, 실차 사용 가능성을 주장하는 검사가 아닙니다.

기준: [전날 검증 기록](link_cleanup_20260924.md).
초기 진단과 최종 로그는 [이번 검사 폴더](../../../storage/experiments/validation/2026-09-25/legacy_retirement)에 분리해 보관합니다.

## 새 실행 방법

저장소 루트에서 실행합니다.

```bash
cd /home/a/autoware_e2e
python3 tools/audit_legacy_layout.py
python3 tools/check_workspace_links.py
python3 tools/workspace_python.py tools/e2e/plan_unique_right_turn_collection_g.py --help
```

연구 검사와 launch 검사는 같은 이름의 테스트 파일이 있으므로 별도로 실행합니다.

```bash
source tools/e2e/env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CUDA_VISIBLE_DEVICES='' \
  python3 tools/workspace_python.py -m pytest -- -ra -q -p no:cacheprovider learning/tests
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CUDA_VISIBLE_DEVICES='' \
  python3 tools/workspace_python.py -m pytest -- -q -p no:cacheprovider autonomy/autoware_e2e_vad_launch/test
```

`audit_legacy_layout.py`는 폴더 구조만 검사합니다. 출력의
`canonical_only_runtime_verified: false`는 그 명령 자체가 런타임 검사를 실행하지 않는다는 뜻입니다.
설치 조회와 전체 검사 결과는 위 최종 제거 기록의 `canonical_runtime_checked: true`를 기준으로 확인합니다.

## 복구 범위

원본 데이터·모델·원시 결과는 그대로 있으므로 복구할 데이터 삭제는 없습니다.
필요할 때만 작업을 멈춘 후 위 표의 정확한 네 경로에 링크를 다시 만드는 것으로
이전 위치 접근을 되돌릴 수 있습니다. 새 코드와 과거 실행 환경이 동일하다는 의미는 아닙니다.
Shell 경로 전환 전 사본은 `.workspace_archive/shell_entry_paths_20260925`에 보관했습니다.
앞선 정리 기록과 백업도 삭제하지 않았습니다.
