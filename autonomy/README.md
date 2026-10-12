# Autoware 실행 작업공간

<!-- HH_260906 - Preserve existing build ABI while separating canonical source and generated outputs. -->

- `src/`: 가져온 Autoware 의존 소스.
- `autoware_e2e_vad_launch/`: 이 저장소의 launch·노드·ROS 설정.
- `cmake/`, `include/`, `patches/`: 빌드 호환 코드와 패치.
- `build/`, `install/`, `log/`: 기존 빌드·설치·로그 산출물. Git 대상 아님.

루트 `build/install/log` 링크는 제거했습니다. 기존 설치 282개는 보존하고 생성된 경로·내부 링크를 정리했습니다.
환경은 `source tools/e2e/env.sh`로 불러오며, 기본 빌드 도구도 이 폴더의 build/install/log를 사용합니다.
720개 검사와 launch 패키지 재빌드를 통과했습니다. 기존 전체 캐시를 새로 빌드했다는 뜻은 아닙니다.
`canonical_rebuild/`는 앞서 만든 별도 87개 패키지 검사 환경이며, 기본 282개 설치를 대체하지 않습니다.
[검증 범위·남은 작업](../docs/workflows/05_maintenance/runtime_alias_retirement.md).
소스 검색은 `colcon list --base-paths autonomy/src autonomy/autoware_e2e_vad_launch`로 명시합니다.

[전체 안내](../WORKSPACE.md) · [레거시 제거 기준](../docs/workspace-layout-migration.md)
