# 레거시 제거 검증 — 3개 제거, 런타임 재빌드 완료·미활성화

<!-- HH_260906 - Report partial retirement honestly; compilation does not authorize automatic alias removal. -->

## 실제 제거 결과

| 제거한 루트 링크 | 보존된 실제 폴더 |
|---|---|
| `config` | `learning/config` |
| `tests` | `learning/tests` |
| `workflows` | `docs/workflows` |

대상 데이터·소스 파일을 삭제하지 않았습니다. 루트 링크 16개 중 **3개 제거, 13개 남음**입니다.
최종 목표는 여전히 0개이며 전체 제거 완료가 아닙니다.
복구 정보: [제거 기록](../../../../../workflows/05_maintenance/legacy_retirement_20260922.json).

## 검사와 수정

- 세 링크가 없는 상태에서 관련 회귀 검사 **513개 통과**.
- 제거 실패 시 링크를 복구하고, 실제 폴더이면 거부하는 검사와 레거시 원장 검사 **7개 통과**.
- canonical 환경 경로·빌드 격리 검사 **11개 통과**.
- 처음 두 제거 시도는 출처 검사 실패로 링크를 자동 복구했습니다. 원인은 옛 `scripts/` 링크를
  거쳐 원본을 읽는 경로였습니다. 실제 `tools/` 파일을 검사하도록 고쳤으며 symlink 거부와
  고정된 원본 SHA 검증을 그대로 유지했습니다. 실패한 검사를 삭제하거나 기준을 낮추지 않았습니다.
- launch의 Python package 경로를 `../../learning/portable_e2e`로 수정하고 별도 클린 빌드 **1개 패키지 성공**.

## 남은 검증은 왜 필요한가

기존 CMake cache와 설치 hook에 `/home/a/autoware_e2e/install/...` 같은 옛 경로가 남아 있습니다.
예를 들어 기존 launch의 `package.sh`에도 해당 경로가 있습니다. Python 도구의 `scripts.e2e` import와
역사적 데이터 계약 경로도 별도 전환이 필요합니다. 새 빌드 성공만으로 이 모든 의존성이 없어진 것은 아닙니다.

환경 스크립트는 모델 자산·CUDA/TensorRT·호환 헤더·CMake·설치 결과의 기본 경로를 canonical 위치로 바꿨습니다.
기존 빌드·설치는 보존하고 **로컬** `autonomy/canonical_rebuild/{build,install,log}`에 최소 런타임을 재빌드합니다.
명령은 `tools/e2e/check_canonical_runtime_build.sh`이며 workspace lock을 사용합니다.
실행 중에는 같은 스크립트를 중복 시작하지 마세요. 기존 출력이 있으면 거부합니다.

2026-09-22 20:24 이후 로그 확인: **86개 패키지, 16분 3초에 재빌드 완료**했으며
`CANONICAL_RUNTIME_BUILD_COMPLETE_NOT_ACTIVATED` 표시를 확인했습니다.
컴파일 stderr(경고 등)는 있었지만 colcon 실패 패키지는 보고되지 않았습니다.
활성 설치 전환·런타임 검증 완료라는 뜻은 아닙니다.
이 작업은 시스템/venv 패키지를 설치하거나 다운로드하지 않으며, 원격 GPU 서버에서 빌드하지 않습니다.
활성 설치를 바꾸거나 남은 링크를 자동 삭제하지도 않습니다.

## 로그·다음 확인

로그: `storage/experiments/validation/2026-09-22/legacy_retirement/`.

- `navigation_attempt1.log`, `navigation_attempt2.log`: 앞선 실패와 복구 근거.
- `navigation_pass.log`: 513개 통과 및 세 링크 제거 결과.
- `launch_build.log`: canonical launch 클린 빌드 성공.
- `runtime_build.log`: 완료된 최소 런타임 재빌드 전체 출력.

새 설치의 로딩/launch/공유 라이브러리·데이터 경로를 검사해야 합니다. 나머지 Python import와 소스·데이터 참조를
전환하고 링크 없이 검사를 통과한 항목만 제거합니다. 현재는 새 최소 런타임 빌드만 완주했으며,
활성화·폐루프 검증은 미완료입니다.

## 추가 로딩 검사

<!-- HH_260906 - Keep no-node loading evidence separate from active-runtime and alias-free operation. -->

후속으로 같은 canonical prefix에 launch 패키지도 **2.04초에 빌드 완료**했습니다.
`tools/e2e/check_canonical_runtime_load.sh`를 빈 초기 환경 shell에서 실행해 다음을 확인했습니다.

- 주요 패키지 6개가 모두 `autonomy/canonical_rebuild/install`에서 검색됨.
- 예전 install prefix가 주요 환경변수에 없음.
- 루트 호환 링크를 Python 검색에서 제외해도 설치된 `portable_e2e` import 성공.
- 주요 공유 라이브러리 **12개**의 `ldd` 결과에 `not found` 없음.
- shadow/calibration launch **2개**의 `--show-args` 파싱 성공.

ROS 노드·CARLA·차량 제어는 시작하지 않았으며 기존 기본 설치를 전환하지 않았습니다.
이 검사는 모든 외부 데이터 참조, 모든 launch, 13개 링크가 없는 상태를 검증하지 않습니다.
첫 점검 shell의 `set -u`는 ROS 생성 hook의 미정의 변수에 걸렸고, 일반 ROS sourcing 방식으로
수정한 후 위 검사를 수행했습니다. 빌드 실패나 원본 손상은 아니었습니다.
