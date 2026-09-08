# 최종 코드 회귀검사: 4,518 PASS / 6 SKIP

<!-- HH_260906 - Preserve final successful commands and earlier failures separately; code tests do not qualify learned driving or real vehicles. -->

두 테스트 폴더를 **별도 명령으로** 실행한 최종 합계입니다. `tests/` 4,145개와 런치 패키지 373개를 더했습니다. 이전 재시도의 통과 수를 다시 더하지 않았습니다. 실제 CARLA·Autoware 주행, 새 학습 모델 성능, 30 km/h 검증 또는 실차 안전 승인 결과가 아닙니다.

## 마지막 검증 결과

| 범위 | PASS | FAIL | SKIP | pytest 표시 시간 | 종료 코드 | 원본 출력 |
|---|---:|---:|---:|---:|---:|---|
| `tests/` | 4,145 | 0 | 6 | 204.16초 | 0 | [main_tests_final.log](main_tests_final.log) |
| `autoware_e2e_vad_launch/test/` | 373 | 0 | 0 | 13.47초 | 0 | [launch_tests.log](launch_tests.log) |
| 두 폴더의 최종 합계 | **4,518** | **0** | **6** | 별도 실행 | 모두 0 | 아래 명령 참고 |

`pytest.ini`에 선언된 두 테스트 경로를 각각 지정했습니다. 런치 검사는 메시지·기하·설정 함수, mock ROS 노드, 임시 fake `ros2`로 명령을 확인하는 검사입니다. 런치 테스트를 실행했다고 실제 차량이나 시뮬레이터를 새로 켠 것은 아닙니다. 일부 코드 검사는 작은 합성 텐서 계산을 포함하지만 실제 데이터셋 재학습 캠페인이나 모델 승인으로 해석하지 않습니다.

6개 SKIP은 통과가 아닙니다.

- 5개: 로컬 Torch에 `torch.load(weights_only=...)`가 없어 안전한 체크포인트 재개·평가 검사를 실행할 수 없음. 안전 제한을 우회하지 않았습니다.
- 1개: 고정된 Woraksan 경로/지도 번들이 없어 해당 선택적 검사를 실행할 수 없음.

## 실패와 수정 경위도 보존

| 순서 | 결과 | 해석 / 원본 |
|---|---|---|
| 1. 환경을 source하지 않은 시도 | exit 5, 1 SKIP, 1.48초 | Autoware 메시지 import 불가. 전체 검사 통과가 아님. [원본 로그](attempt_01_unsourced_environment.log) |
| 2. 환경을 source한 첫 전체 검사 | 4,141 PASS / 1 FAIL / 6 SKIP, 204.71초 | 후보 순서를 바꾼 부동소수점 계산에 비트 단위 일치를 요구한 테스트 실패. [원본 로그](attempt_02_sourced_regression_failure.log) |
| 3. 산술 원인 진단 후 테스트만 수정 | 최종 4,145 PASS / 6 SKIP | 한 기존 테스트를 FP32·FP64 × 순서 2종의 4개로 확장. 실패 기록은 그대로 두고 전체 검사를 다시 실행함. |

테스트 전용 수정 커밋은 `2c71adf86eea0c94aab8a1733ad293b149d7887f`이며 변경 파일은 `tests/test_portable_e2e_frozen_selector.py` 하나입니다. 생산 scorer/decoder 구현, 가중치, 학습 데이터·라벨, 물리 제한 및 런타임 허용 기준은 변경하지 않았습니다.

작은 합성 fixture의 [독립 산술 진단 JSON](selector_roundoff_diagnostic.json)과 [실행된 진단 코드](selector_roundoff_diagnostic.py)를 원본 그대로 포함했습니다. FP32/FP64 각각 두 후보 순서를 확인했습니다.

이 진단 코드는 **당시 실행 소스의 검토용 원본**입니다. 코드가 출력하는 JSON의 `command` 필드는 원래 개인 보관 `artifacts/...` 상대경로 명령을 적은 고정 문자열입니다. 공개 경로의 복사본을 다시 실행해도 실제 재실행 명령으로 자동 갱신되지 않습니다. 이를 새 실행의 명령 증명으로 사용하지 마세요. 아래 재실행 안내는 최종 두 pytest 검사만 대상으로 합니다.

- 입력 정규화와 같은 입력의 계산은 정확히 일치합니다. 후보를 하나씩 계산한 기준값도 순서 변경에 대해 비트 단위로 일치합니다.
- 배치 후보 순서만 바꾸면 최종 점수 최대 절대 차이는 FP32 `8.940696716308594e-8`, FP64 `1.6653345369377348e-16`입니다. 이 fixture의 선택 후보 매핑은 모두 동일했습니다. BLAS의 정확한 분기·명령 원인은 확정하지 않았습니다.
- 순서 변경 비교에만 `atol = dtype epsilon`, `rtol = 4 × epsilon`을 사용하도록 테스트를 수정했습니다. 같은 입력의 정확한 일치 검사는 유지하고, `1e-3`로 오염시킨 점수는 반드시 거부하는 검사도 추가했습니다.
- JSON의 **FP64 첫 Linear 중간층 비교 2개는 제안된 허용오차 밖(`false`)**이라는 기록을 그대로 보존했습니다. 이를 숨기거나 모든 중간 계산이 통과했다고 주장하지 않습니다. 변경한 테스트는 최종 점수의 순서 대응을 검사하며 중간층 값을 승인하는 물리·안전 검사가 아닙니다.

## 실행 시점과 소스 고정

최종 `tests/` 실행 시작 기록은 **2026-09-08 14:47:40.534798 KST**, 완료를 확인한 시각은 **14:54:52 KST**입니다. 후자는 실제 프로세스가 종료한 정확한 시각이 아니며, 두 시각을 빼서 시험 소요 시간으로 사용하지 않습니다. pytest가 보고한 수행 시간은 **204.16초**입니다.

런치 테스트 관측 시각은 **14:40:47–14:41:02 KST**, pytest 시간은 **13.47초**입니다. 런치 실행 당시 HEAD는 `92bd7de47aa7c1cedca69f173b6fa4c2b37caa1e`, 최종 main 실행 HEAD는 위 `2c71adf…`입니다. 사이의 수정은 런치 패키지가 아닌 테스트 파일 하나이며, 런치 Python 24개를 현재 소스와 다시 비교해 동일함을 확인했습니다.

- [main_source_snapshot.json](main_source_snapshot.json): main 실행 전 Python 327개와 `env.sh`·`pytest.ini`의 SHA, 실행 후 독립 재확인 시각과 결과. **327개는 테스트 케이스 수가 아닌 소스 파일 수**입니다.
- [launch_source_snapshot.json](launch_source_snapshot.json): 런치 테스트 14개 파일 및 패키지 스크립트 10개 파일의 실행 전후 SHA. 모든 파일이 동일합니다.
- [provenance.json](provenance.json): 각 원본 파일 SHA·크기·실행 범위. [SHA256SUMS](SHA256SUMS): 공개 파일 전체 체크섬.

이는 선언된 소스의 무결성 검사입니다. 모든 YAML/XML, 외부 바이너리, 모델 가중치, 대용량 데이터셋이나 실제 주행 결과를 이 스냅샷만으로 검증했다는 뜻은 아닙니다. 원본 로그는 줄바꿈까지 동일하며, 환경 오류와 과거 FAIL을 최종 PASS로 덮어쓰지 않았습니다.

## 같은 두 코드 검사 실행하기

이미 이 저장소의 ROS/Autoware 의존성이 준비된 **로컬 저장소 최상위 폴더**에서 실행합니다. 새 `git clone`만 했거나 메시지 패키지가 준비되지 않았다면 우선 기존 환경 설정 가이드가 필요합니다. 아래 명령은 설치·빌드·실제 주행을 수행하는 명령이 아닙니다.

```bash
source scripts/e2e/env.sh
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  python3 -m pytest -q -rs tests
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  python3 -m pytest -q -rs autoware_e2e_vad_launch/test
```

원래 실행 명령은 각 source snapshot에 기록했습니다. 코드·Torch·환경이 달라지면 케이스 수와 결과도 달라질 수 있습니다. 이 공개 폴더는 기록 보관용이므로 여기에 시험 결과를 덮어쓰지 마세요.
