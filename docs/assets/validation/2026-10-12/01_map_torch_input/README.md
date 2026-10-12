# 지도 학습 입력의 CPU Tensor 연결 검증

<!-- HH_261012 - Separate completed CPU input parity and saved audit from trainer learning and capability approval. -->

기존 이미지·정답을 Torch Tensor로 바꿔도 원래 학습 코드의 입력과 **모든 값이 일치함을 실제 데이터에서 확인했습니다.** 원본 crop과 표지 보존 crop 각각 16/16배치가 저장·재검사·수신 확인까지 통과했습니다. 실제 시험은 종료됐습니다.

**이번에는 모델을 학습하거나 주행하지 않았습니다.** 로컬 CPU에서 입력 연결만 검증했으며 PRO6000·GPU·CARLA·ROS·패키지 설치는 사용하지 않았습니다. 정식 주행 완료는 메인 **0/9개**, 서브 **0/30개**입니다. [전체 목표와 합격 기준](../../../../portable-e2e-feature-roadmap.md)을 유지합니다.

## 무엇을 확인한 건가요

배치 하나는 이미지·정답 8쌍입니다. 앞선 작업이 NumPy 배열까지 연결한 것이었다면, 이번에는 그 배열을 실제 학습에 쓰는 Torch의 CPU Tensor로 복사했습니다.

```text
허용된 기존 RGB·정답 → 같은 영역 crop
  → float32 RGB·int64 정답 배열 → 독립 저장 공간의 CPU Tensor
  → 원래 학습 코드의 CPU 변환 결과와 모든 값·바이트 비교
  → 원본·Tensor 기록 저장 → 버퍼 재검사 → 실제 수신 확인(ACK)
```

RGB는 `[8,3,360,640]` float32의 0~1 값입니다. 이미 정규화된 배열을 다시 255로 나누지 않았습니다. 정답은 `[8,360,640]` int64이며 분류 번호 0~6을 유지했습니다. RGB는 BILINEAR, 정답은 NEAREST로 같은 선택 영역을 사용합니다.

비교 기준은 고정된 원래 trainer 소스의 `tensors` 함수입니다. **전체 trainer를 실행한 것이 아니라 해당 함수만 선택해 CPU 입력 변환을 계산했습니다.** NN 추론·손실 계산·역전파·optimizer 갱신은 모두 0회입니다. GPU나 서버의 다른 Torch 버전에서 같은 결과가 나온다고 확대하지 않습니다.

## 실제 결과

| 항목 | 원본 crop | 표지 보존 crop |
| --- | ---: | ---: |
| 선언 / 통과 / 실패 / 미실행 배치 | 16 / 16 / 0 / 0 | 16 / 16 / 0 / 0 |
| 생성·Tensor 값 비교·두 기록 저장·실제 ACK | 각각 16 | 각각 16 |
| 이미지·정답 입력 노출 | 128회 | 같은 128회 |
| 표지 정답 복구 | 0회 | 2회 |
| 미확인 pending / 종료 정리 오류 | 0 / 0 | 0 / 0 |
| 원래 20,000배치 중 이번 범위 밖 | 19,984 | 19,984 |

두 군이 **같은 128회 입력을 반복**한 비교입니다. 256개 독립 장면이나 전체 20,000배치를 검증한 것이 아닙니다. RGB float32의 실제 바이트와 정답 int64 값을 전량 비교했고, 허용 오차를 늘려 통과시키지 않았습니다.

10월 8일에 사용한 고정 첫 16배치를 이번 새 Torch 경계에서 다시 확인했습니다. 새 장면 수집이나 학습 데이터 확장이 아닙니다. 앞선 NumPy 단계의 통과만으로 미확인 Tensor 단계도 통과했다고 가정하지 않고 실제 변환을 검사한 것입니다.

후보의 복구는 6번 배치·위치 2·slot 1036의 **표지 정답 56→0픽셀**, 10번 배치·위치 4·slot 344의 **37→0픽셀**에 해당합니다. 원래 이미지 전체 영역을 선택해 정답을 보존했고 Tensor·ACK까지 연결됐습니다. **모델이 표지판을 2개 더 인식했다는 결과는 아닙니다.**

전체 실행은 **95.29초**, 관측 최대 VmHWM은 **386,965,504바이트, 약 369MiB**입니다. 종료 소스·보존 소스·입력·선택 provider·계획/권한 복사본의 seal **62개가 일치**했고 기록 4개도 종료 바이트 검사를 통과했습니다. 계획+출력 합계는 **952,021바이트**로 기존 2MiB 한도 안입니다. 단위시험 임시 자료와 별도 ROOT·독립 영수증은 이 합계 밖입니다.

512MiB·900초·2MiB·종료 여유 512KiB·쓰기 가능 공간 2GiB+64MiB의 기준을 낮추지 않았습니다. 메모리·시간은 관측 guard이며 OS 강제 차단 보장은 아닙니다. 선택한 provider 12개를 확인했지만 전체 라이브러리 의존성을 봉인했다는 뜻도 아닙니다. 사용한 기존 환경은 Python 3.10.12·NumPy 1.21.5·Torch 1.8.0a0이며 CUDA는 초기화하지 않았습니다.

[실제 전체 결과](../../../../../storage/experiments/diagnostics/2026-10-12/map_torch_input_boundary_v1/result.json) · [실제 명령·전체 출력·종료 상태](../../../../../storage/experiments/diagnostics/2026-10-12/map_torch_input_boundary_ROOT_run_v1.json) · [ROOT 종료 확인·파일 크기](../../../../../storage/experiments/diagnostics/2026-10-12/map_torch_input_boundary_ROOT_postcheck_v1.json).

## 코드 검토와 별도 검산

새 입력 변환기·실행기·시험 소스는 ROOT와 별도 검토자가 전체 검토했습니다. 합성 B8 배열 시험은 **48개 통과·11.58초**, 실행기의 스칼라·작은 가상 파일 시험은 **35개 통과·0.09초**입니다. 실제 데이터를 학습한 횟수나 승인 기능 수가 아닙니다.

값·dtype·shape·저장 공간 교체, 원본과 영수증 동시 변경, 다른 소유 스레드/프로세스, 중복 호출, 기록 전 실패·write 후 flush 실패·ACK 거부·종료 실패를 확인했습니다. 생성·저장·ACK 횟수를 따로 보존하고, 마지막 ACK 뒤 종료 실패를 가상의 17번째 실패로 더하지 않습니다.

**별도 코드의 저장 검산도 9.78초에 완료했습니다.** 지정 파일 69개의 앞뒤 RAW 바이트·세대, 원래 20,000행 메타데이터의 첫 16행, 두 군의 전체 원본/Tensor 기록과 실제 ACK를 연결했습니다. 원본 serial 영수증과 전체 bridge 검사 영수증을 저장 필드에서 복원해 내용 해시도 확인했습니다. 실제 결과 원문의 ROOT 고정 해시는 검산 전후 동일했습니다.

독립 검사기의 스칼라 시험은 **47개 통과·0.06초**입니다. 이 검사기는 배열을 다시 해독하거나 Torch 기준 변환을 두 번째 계산하지 않았습니다. 실제 값 비교는 첫 실행의 근거이고, 독립 검산은 저장 바이트·세대·메타데이터 연결의 근거입니다. 외부 소비자의 실제 실행이나 GPU 수치 등가성을 독립 인증했다고 주장하지 않습니다. 실제 시험과 독립 검산 모두 종료됐습니다.

[변환기 합성 시험 원문](../../../../../storage/experiments/diagnostics/2026-10-12/map_torch_bridge_synthetic_ROOT_suite_v1.json) · [실행기 시험 원문](../../../../../storage/experiments/diagnostics/2026-10-12/map_torch_owner_scalar_ROOT_suite_v1.json).

[독립 저장 검산 원문](../../../../../storage/experiments/diagnostics/2026-10-12/map_torch_input_boundary_v1_independent_v1.json) · [독립 검사기 시험 원문](../../../../../storage/experiments/diagnostics/2026-10-12/map_torch_checker_scalar_ROOT_suite_v1.json) · [독립 검산 명령·종료 상태](../../../../../storage/experiments/diagnostics/2026-10-12/map_torch_saved_checker_ROOT_run_v1.json).

## 완료·준비·아직 남은 작업

| 상태 | 내용 |
| --- | --- |
| 완료 | CPU Tensor 변환·원본/host/Tensor 재검사·원래 trainer 함수의 CPU 전량 비교·실제 16배치/군·저장·ACK·종료 처리·독립 저장 검산 |
| 준비 | 같은 모델 구조에서 crop만 바꾸는 후속 학습 설계. 실제 trainer나 학습 실행 권한은 아님 |
| 아직 미구현·미검증 | 새 trainer 연결, NN·손실·역전파, 서버 환경/GPU 수치 검증, 전체 20,000배치의 자원·기록 계획, 새 학습·인식 개선·독립 TEST |

앞서 작성한 학습 준비 JSON의 `NOT_YET_EXECUTED`는 **작성 시점의 상태**입니다. 이를 덮어쓰지 않고 [입력 시험 종료 후의 새 준비 현황](../../../../../storage/experiments/plans/2026-10-12/map_learning_readiness_after_CPU_boundary_v1.json)을 추가했습니다.

## 다음 학습은 어떻게 비교하나요

이전 학습은 서로 다른 구조의 모델 비교였습니다. crop 정책을 평가하려면 **두 군 모두 같은 MapContextRecovery·같은 시작 가중치·같은 손실·같은 optimizer·같은 입력 순서**를 사용해야 합니다. 바꾸는 것은 선택 crop 영역뿐입니다. 새 head나 새 손실을 함께 넣지 않습니다.

현재 제안은 기존 1,209개 슬롯(원본 729+FALSE paired 480), 배치 8, 군별 고정 20,000회입니다. 같은 weighted pixel CE와 fresh AdamW를 쓰고 조기 최고 모델 선택이나 숨긴 재시도를 하지 않습니다. 20,000회가 성능을 보장한다는 뜻은 아닙니다. 시작 가중치 파일·수치 상태·실제 모델 parameter 수는 아직 검증하지 않았습니다.

1. **로컬 비학습:** 새 trainer 소스를 입력 어댑터에 연결하고, 전체 loader·기록 묶음·메모리·용량·종료점·실패 처리 조건을 구현합니다. 기존 frozen trainer와 제한 실행 권한은 재사용하지 않습니다.
2. **PRO6000의 학습 전 확인:** 접속·개인 hwanhong venv·provider·실행 직전 물리 GPU 0의 가용성과 소유권을 확인합니다. GPU 1이나 다른 사용자의 프로세스는 사용/종료하지 않습니다. 같은 시작 수치 상태·빈 optimizer·RNG·parameter 순서와 CPU→CUDA/NN/손실 검증은 새 범위로 진행합니다.
3. **학습과 평가:** 허용된 TRAIN과 고정 종료점을 새로 선언한 두 군을 학습합니다. 기존 알려진 350장 진단 DEV의 23/27 통과·4개 실패·후보 미채택을 유지하고 표지 누락·오탐·다른 분류의 퇴행을 비교합니다. 이미 노출된 DEV를 새 TEST로 부르지 않습니다.
4. **주행은 별도:** 첫 RTE-01 직진과 SAF-01 100ms 지연부터 확인합니다. 기본 직진·좌우회전·속도 추종·종점 정지가 안정되기 전 새 기능 확대를 앞세우지 않습니다. 자동 지도는 보조이며 지도 없이 주행하도록 유지합니다.

[작성 시점의 crop-only 설계](../../../../../storage/experiments/plans/2026-10-12/map_crop_only_learning_preparation_v1.json) · [현재 남은 구체적 준비 조건](../../../../../storage/experiments/plans/2026-10-12/map_learning_readiness_after_CPU_boundary_v1.json).

이번 95.29초는 제한된 CPU 입력 시험 시간입니다. 전체 학습·개발·폐루프 시간이나 크레딧 잔액·미래 토큰 사용량으로 환산하지 않습니다. 새로운 인식/주행 능력, 다른 공개 모델과 동급 성능, 실차 운용 가능성을 입증한 결과는 없습니다.

## 재현과 원문 위치

[Tensor 변환기](../../../../../learning/portable_e2e/map_torch_cpu_input_bridge_20261012_v1.py) · [변환기 시험](../../../../../learning/tests/test_map_torch_cpu_input_bridge_20261012_v1.py) · [실제 실행기](../../../../../tools/e2e/verify_map_torch_input_boundary_20261012_v1.py) · [실행기 시험](../../../../../learning/tests/test_verify_map_torch_input_boundary_20261012_v1.py) · [별도 저장 검사기](../../../../../tools/e2e/check_map_torch_input_boundary_saved_20261012_v1.py).

[이번 ROOT 계획](../../../../../storage/experiments/plans/2026-10-12/map_torch_input_boundary_v1/ROOT_plan.json) · [FIRST16 CPU 입력 한정 권한](../../../../../storage/experiments/plans/2026-10-12/map_torch_input_boundary_v1/ROOT_authority.json) · [소스·시험·범위 검토](../../../../../storage/experiments/plans/2026-10-12/map_torch_input_boundary_v1/ROOT_execution_review_v1.json).

원본 방식과 표지 보존 방식 폴더에 각각 `exposures.jsonl`, `tensor_receipts.jsonl`이 있습니다. 각 파일은 1~16번 전체 배치를 보존했습니다. 실제 명령은 위 실행 원문에서 확인할 수 있습니다. 이미 종료된 명령은 고정 출력 때문에 재실행을 거부합니다. **새 revision·새 출력·새 원문 해시·자원 확인·한정 권한**으로 재현하며 기존 결과를 지우고 재시도하지 않습니다.

이번에는 PNG/GIF를 새로 만들지 않았습니다. [10월 8일의 실제 원본·crop·복구 PNG](../../2026-10-08/nontraining_batch_capture/02_actual_rescue_and_reader_ab/README.md)는 당시 입력 진단이며 새 모델 인식 결과나 Autoware/RViz 화면이 아닙니다.

`storage/experiments/`는 Git 제외 대상입니다. clone만으로 데이터·가중치·결과나 실행 환경이 모두 생기지는 않습니다. 레거시 경로/링크를 복원하지 않았고 기존 데이터·가중치·실패 기록을 보존했습니다. 삭제·commit·push는 하지 않았습니다.

<!-- HH_261012 - Retain the known frozen missing document while checking new navigation without automatic rewrites. -->

[읽기 전용 문서 링크 검사](../../../../../storage/experiments/diagnostics/2026-10-12/navigation_after_map_torch_input_v1.json)는 검사 시점 686개 문서·8,061개 로컬 링크에서 레거시 0·기존 누락 1·범위 밖 18개를 확인했습니다. 이번 새 문서의 잘못된 링크는 0개입니다. 누락은 기존 10월 1일 고정 소스의 `WEIGHT_PROVENANCE_CORRECTION.md`이며 추정 원본을 만들거나 frozen 기록을 덮어쓰지 않았습니다. 종료값 1은 보존했고 문서 링크 검사를 모델 성능 합격으로 세지 않습니다.
