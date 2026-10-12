# 작업공간 안내 — 새 실제 구조

<!-- HH_260906 - Record verified physical alias removal separately from Python compatibility. -->

현재 실제 폴더는 **autonomy / learning / tools / storage / docs** 5개입니다.

| 폴더 | 내용 |
|---|---|
| [autonomy](autonomy/README.md) | Autoware 소스·launch·빌드·설치·로그·빌드 호환 코드 |
| [learning](learning/README.md) | 자체 모델·연구 설정·회귀 검사 |
| [tools](tools/README.md) | 실행·학습·수집·분석·관리 도구 |
| [storage](storage/README.md) | 데이터·모델 자산·원시 실험 결과 |
| [docs](docs/workflows/README.md) | 사용법·보고서·PNG/GIF·작업별 안내 |

<!-- HH_261007 - Add a categorized thesis entry without moving frozen four-PC evidence or restoring legacy aliases. -->

졸업논문 자료는 홈의 [thesis_4pc](/home/a/thesis_4pc/README.md)에서 찾아보세요. Documents·Downloads의26개 파일·폴더, 내부154개 파일 약2.59GB를 실제 분류 이동하고 전체 해시를 확인했습니다. [기존 PC1–4 분류 안내](docs/thesis_4pc/README.md)는 시스템 이력·시험 준비·고정 원본·분석·재현·용량 후보를 연결합니다. 실행 경로가 고정된4-PC 원본·저장소·Unreal 프로젝트는 그대로 보존했고 영구 삭제는 하지 않았습니다.

<!-- HH_261007 - Add the observed current thesis archive location without claiming frozen PC4 consumer migration or changing historical records. -->

2025 시험 자료와 분석의 현재 위치는 [홈의 VILS_Test](/home/a/thesis_4pc/VILS_Test)입니다. 이 폴더는 위 Documents·Downloads154개 이동 기록의 대상이 아닙니다. 과거 PC4 스크립트의 이전 절대경로는 그대로이므로, [새 재현 전 확인 사항](docs/thesis_4pc/05_reproduction/README.md)을 먼저 읽으세요.

실제 파일을 이동했고 루트 호환 링크 **16개 모두 검사 후 제거했습니다. 남은 링크는 0개**입니다.
마지막 `scripts`, `portable_e2e`, `datasets`, `artifacts` 링크도 제거했습니다. 원본 데이터는 보존했습니다.
2026-09-23에는 `docs/workflows`의 중복 바로가기 12개를 별도로 제거했습니다.
[최신 링크 정리·전체 검사 현황](docs/workflows/05_maintenance/link_cleanup_20260925.md).
전체 연구 검사 8,372개 통과·기존 실패 10개·건너뜀 8개이며 새 실패는 없습니다. launch 검사 373개도 통과했습니다.
파일시스템 링크와 달리 Python의 기존 import 이름·원본 경로 식별자 호환 처리는 명시적 실행 진입점에 남아 있습니다.
VS Code에서 숨기기만 한 상태가 아니며, 더 이상 필요 없는 루트 별칭 숨김 설정도 제거했습니다.
[레거시 제거 원장·완료 조건](docs/workspace-layout-migration.md)을 확인하세요.

<!-- HH_261008 - Qualify the preserved historical status command rather than reporting fixed September jobs as current work. -->
`python3 tools/workspace.py status` 및 `status --remote`는 **9월의 고정 실험 결과·로그 조회**입니다. 현재 실행 중인 학습을 확인하는 명령이 아닙니다. 지금 상태는 [최신 근거 색인](docs/assets/validation/2026-10-02/README.md)을 먼저 확인하세요. [새 읽기 전용 상태 도구와 실제 확인 결과](docs/assets/validation/2026-10-08/workspace_status/README.md)는103개 사전 검사·실제 명령·독립 지정 메타데이터 검산을 완료했습니다. 수동으로 고정한 목록의 종료 자료를 확인하며 서버 전체 자동 감시나 모든 입력 재검증은 아닙니다. 기존 파일은 보존합니다. 레거시 배치 검사는 `python3 tools/audit_legacy_layout.py`입니다.
설치 환경 `.venv*`, 라이선스·Git/pytest 설정과 기존 설치 진입점은 루트에 유지했습니다.
원격 학습 환경은 변경하지 않았습니다. 기존 282개 패키지의 설치를 `autonomy/install`에 보존했고,
패키지 검색·12개 공유 라이브러리 의존성·Python import·launch 2개 인자 파싱을 확인했습니다.
launch 패키지는 새 경로에서 재빌드했습니다. 전체 소스 재빌드·실제 주행 검증은 아닙니다.
[최근 정리 결과와 남은 기존 검사 실패](docs/workflows/05_maintenance/link_cleanup_20260925.md).
<!-- HH_261007 - Use the current status index instead of labeling September history as currently running learning. -->
[현재 주행과 지도 모델 상태 및 다음 계획](docs/assets/validation/2026-10-02/README.md).

---

<details>
<summary>이전 구조 정리 당시 기록 — 현재 위치·명령은 위 안내를 사용하세요</summary>

<!-- HH_260906 - Add stable navigation without relocating active jobs, imported modules or provenance paths. -->

**이 폴더는 Autoware의 ROS/colcon 작업공간이면서 자체 모델 연구 저장소입니다.**
`workflows/`는 별도 프로젝트가 아니라 이 안의 실제 파일을 목적별로 찾아가는 안내·바로가기입니다.
실제 코드는 `portable_e2e/`, `tools/`, `autoware_e2e_vad_launch/`에 있습니다.

루트에 잘못 생성됐던 `SUMMARY.md`, `aggregate.json`, `matrix_plan.json`, `maps/`는
`artifacts/validation/2026-08-31/legacy_root_matrix/`로 **실제 이동**했습니다. 30개 파일 해시는 동일합니다.
빈 `files/imagery/`와 `files/`도 제거했습니다. [이동·복구 설명](docs/workflows/05_maintenance/root_reorganization.md).

| 하고 싶은 일 | 열 곳 |
|---|---|
| 주행 모델 구조·학습 | [01_driving](docs/workflows/01_driving/README.md) |
| 자동 지도·도로 인식 | [02_mapping](docs/workflows/02_mapping/README.md) |
| CARLA 수집·Autoware 실행 | [03_simulation](docs/workflows/03_simulation/README.md) |
| 학습 결과·PNG·GIF | [04_results](docs/workflows/04_results/README.md) |
| 환경 점검·검사·정리 기록 | [05_maintenance](docs/workflows/05_maintenance/README.md) |

```bash
# HH_260906 - These status commands never start or stop training.
python3 tools/workspace.py status          # 로컬에 회수된 완료 결과
python3 tools/workspace.py status --remote # SSH로 최신 로그·완료 결과 조회
python3 tools/workspace.py guide          # 작업별 안내 파일 위치
```

모든 실행·분석 파일: [분류된 전체 목록](docs/workflows/EXECUTABLES.md).
과거 코드도 필요할 수 있으므로 목록에 있다는 이유만으로 실행하거나 삭제하지 않습니다.

## 원본 폴더의 역할

| 폴더 | 역할 | 정리 원칙 |
|---|---|---|
| portable_e2e/ | 자체 모델·손실·데이터·추론 코드 | import 경로 유지 |
| tools/ | 실행·수집·학습·분석 도구 | 새 목록으로 분류, 기존 실행 경로 유지 |
| config/, autoware_e2e_vad_launch/ | 설정·ROS launch | 기존 launch 경로 유지 |
| src/, include/, cmake/, patches/ | Autoware 의존 소스·빌드 지원 | 이동하지 않음 |
| data/, datasets/ | 모델·데이터·의존 자산·잠금 | 학습·실행 참조 보존 |
| artifacts/ | 원시 센서·로그·원본 실험 결과 | 출처/실패 근거 포함, 무작정 삭제하지 않음 |
| docs/assets/ | 읽기 쉬운 보고서·PNG·GIF | 보고서 링크 유지 |
| tests/ | 코드 회귀 검사 | 주행 기능 통과 수와 별개 |
| build/, install/, log/ | 빌드 산출물·ROS 환경·로그 | 현재 사용 가능성이 있어 삭제하지 않음 |
| .venv*, .venv-map/ | 로컬 도구 실행 환경 | 이동하면 환경 내 절대 경로가 깨질 수 있어 유지 |
| .workspace_archive/ | 검증 후 정리한 캐시의 복구 보관 | 실제 정리 기록 참조 |

## 루트에 남겨야 하는 파일

- `README.md`, `WORKSPACE.md`: 시작 안내. `E2E_SETUP.md`: 기존 상세 사용법과 외부 링크 호환 유지.
- `LICENSE`, `NOTICE`, `DISCLAIMER.md`, `CODE_OF_CONDUCT.md`, `CONTRIBUTING.md`: 라이선스·기여 정책.
- `setup-dev-env.sh`, `amd64.env`, `arm64.env`, `ansible-galaxy-requirements.yaml`: 설치 스크립트가 같은 위치의 파일을 직접 참조.
- `autoware.repos`, `simulator.repos`, `tools.repos`: 의존 저장소 목록; 설치 문서·도구의 기존 경로 유지.
- `setup.cfg`, `pytest.ini`, `CPPLINT.cfg`, `.git*`, `requirements-map.txt`: 검사·개발 설정과 설치 의존 목록.

루트의 ROS/colcon 배치나 실행 중 학습의 경로를 바꾸지 않았습니다. `workflows/`의 바로가기는
같은 원본을 가리키므로 데이터 복제가 아닙니다. 바로가기에서 원본을 수정하면 원본에도 반영됩니다.
바로가기 `.py` 파일을 직접 실행하지 말고 각 README의 환경·명령을 따르세요.

원격 학습 환경: `/home/pro6000/personal/hwanhong/portable_e2e`, 데이터: `../dataset/`.
물리 GPU 0만 사용, 개인 venv 외 설치 금지. 로컬과 원격을 혼동하지 않습니다.
</details>
