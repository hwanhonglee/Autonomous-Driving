# 지도 학습 입력 형식과 수신 확인 연결 검증

<!-- HH_261008 - Separate actual serial host-input completion from model learning, independent pixel replay and driving approval. -->

**실제 이미지·정답을 학습에 사용할 수 있는 배열 형식으로 바꾸고, 값 검사·기록 저장·수신 확인까지 연결했습니다.** 원본 crop 방식과 표지 보존 방식 각각 첫 16배치가 모두 통과했습니다. 실행과 독립 저장 검산은 종료됐으며 **학습 중인 작업은 없습니다.**

이번에는 로컬 CPU만 사용했습니다. PRO6000·GPU·CARLA·ROS·실차 구동·패키지 설치는 사용하지 않았습니다. 다른 테스트의 시뮬레이터와 기존 frozen trainer도 건드리지 않았습니다.

주행 목표는 여전히 **메인 9개 영역·서브 30개 기능, 정식 완료 0/9·0/30**입니다. 첫 기능은 RTE-01 직진 차로 유지이며 SAF-01의 100ms 처리 지연 미달은 그대로입니다. 이번 지도 입력 검사를 주행 기능 완료로 세지 않습니다. [전체 목표와 합격 기준](../../../../../portable-e2e-feature-roadmap.md) · [두 모델의 실제 범위](../../model_scope/README.md).

## 무엇이 달라졌나요

앞선 작업에서는 원래 이미지와 정답을 읽고 같은 영역을 자르는 것까지 확인했습니다. 이번에는 그 뒤에 필요한 **입력 형식 변환과 실제 수신 확인**을 붙였습니다. 배치 하나는 이미지·정답 8개입니다.

```text
허용된 원본 이미지·정답 읽기
  → 원래 RGB·정답 해시 둘 다 재현
  → 원본 crop 또는 표지 보존 crop
  → float32 RGB·int64 정답으로 변환
  → 비학습 NumPy 검사기가 모든 실제 값 비교
  → 원본·변환 기록 저장 → 원본/변환 버퍼 재검사 → ACK
```

RGB는 0~255 정수에서 0~1 float32로 바꾸고, `[8,360,640,3]`을 모델에 흔히 쓰는 `[8,3,360,640]` 배열로 바꿉니다. 정답은 `[8,360,640]` int64이며 원래 분류 번호 0~6을 유지합니다. 배열 모양·타입·소유 여부·유한값·범위·실제 모든 값·바이트 해시를 검사했습니다. RGB는 원래 BILINEAR, 정답은 NEAREST이고 양쪽에 같은 선택 영역을 적용합니다.

**ACK는 이번 검사기가 해당 배치를 확인했다는 수신 기록입니다.** 생성만 했거나 저장만 했다고 소비 완료로 세지 않습니다. 실제 실행기는 독립 NumPy 기준 배열과 모든 값을 비교한 뒤 기록을 저장하고 ACK를 호출했습니다. 어댑터 라이브러리 자체는 외부 소비자의 실행이나 TRAIN 허가를 인증하지 않으며, 실제 학습기가 연결됐다는 뜻도 아닙니다.

## 실제 결과

| 확인 항목 | 원본 crop 방식 | 표지 보존 방식 |
| --- | ---: | ---: |
| 선언한 배치 / 통과 / 실패 / 미실행 | 16 / 16 / 0 / 0 | 16 / 16 / 0 / 0 |
| 생성 / 실제 값 검사 / 원본 기록 / 변환 기록 / ACK | 각 16 | 각 16 |
| 원본 이미지·정답 읽기 | 128회 | 같은 128회 |
| 기록 묶음 | 1~8, 9~16 | 1~8, 9~16 |
| 미확인 pending / 종료 정리 오류 | 0 / 0 | 0 / 0 |
| 원래 20,000배치 중 이번 범위 밖 | 19,984 | 19,984 |

두 군은 **같은 입력 128회 노출을 반복한 비교**입니다. 256개 독립 장면을 새로 검증한 것이 아닙니다. 원본군의 기록은 원래 6필드를 그대로 유지했고, 후보군의 worker·정책·journal은 앞선 고정 소스를 유지했습니다. 두 군 모두 같은 V3 reader의 제한 캐시를 사용합니다.

후보 입력에서는 실제 표지판 정답 복구가 2회 발생했습니다. 6번 배치·위치 2·slot 1036은 원본 56픽셀이 crop 뒤 0픽셀이 되어 전체 영역을 선택했고, 10번 배치·위치 4·slot 344는 37→0픽셀이 되어 복구했습니다. 변경된 이미지와 정답은 변환·수신 확인까지 통과했습니다. **모델이 표지판을 2개 더 인식했다는 결과는 아닙니다.**

전체 실행은 **75.26초**, 관측 최대 VmHWM은 **136,130,560바이트, 약 129.82MiB**였습니다. 종료 소스·보존 소스·입력·선택 provider의 41개 seal은 모두 일치했습니다. 기록 묶음 8개도 종료 바이트 검사에 통과했습니다. 계획과 실제 출력 합계는 **748,811바이트**로 2MiB 한도 안이며, 단위시험 임시 자료와 별도 ROOT·독립 검산 영수증은 이 합계 밖입니다.

메모리 512MiB·마감 900초·출력 2MiB·종료 여유 512KiB·쓰기 가능 공간 2GiB+64MiB의 기준은 낮추지 않았습니다. 관측 메모리·시간 검사는 OS 강제 차단 보장이 아닙니다. reader의 캐시 상태 예약 합계와 실제 native 메모리 할당량도 구분합니다. 종료 후 캐시 상태·예약 집계는 0입니다.

[실제 전체 결과](../../../../../../storage/experiments/diagnostics/2026-10-08/map_serial_training_input_v1/result.json) · [실제 명령·전체 stdout·종료 상태](../../../../../../storage/experiments/diagnostics/2026-10-08/map_serial_training_input_ROOT_run_v1.json) · [ROOT 종료 확인과 용량](../../../../../../storage/experiments/diagnostics/2026-10-08/map_serial_training_input_ROOT_postcheck_v1.json).

## 검사는 어디까지 믿을 수 있나요

어댑터의 합성 배열 시험은 **59개 통과·0개 실패·4.75초**, 실행기의 스칼라·작은 가상 파일 시험은 **35개 통과·0개 실패·0.05초**, 독립 검사기의 스칼라 시험은 **42개 통과·0개 실패·0.05초**였습니다. 서로 다른 범위의 검사이고 학습 횟수나 기능 수가 아닙니다.

버퍼와 영수증을 함께 바꾸는 경우, 다른 스레드·프로세스, 잘못된 ACK, 중복 ACK, 소비 중단, 부분 읽기, 기록 전·후 실패, 같은 inode·같은 길이의 기록 변조, 종료 정리 실패를 검사했습니다. 실패 시점 이후를 통과로 숨기지 않으며, 마지막 ACK 뒤 종료 실패도 완료 배치에 중복 실패로 더하지 않습니다. 소스 검토 중 발견한 초안 문제와 실제 시험 실패는 구분해 보존했습니다. [이번 단위시험 원문](../../../../../../storage/experiments/diagnostics/2026-10-08/map_serial_training_input_unit_suites_v1.json) · [어댑터 시험 원문](../../../../../../storage/experiments/diagnostics/2026-10-08/map_serial_adapter_synthetic_tests_v1.json) · [초안 소스 검토 기록](../../../../../../storage/experiments/diagnostics/2026-10-08/map_serial_training_input_SOURCE_review_v1.json).

**별도 코드의 독립 저장 검산도 3.11초에 완료했습니다.** 지정 파일 57개의 앞뒤 바이트 해시·세대를 확인하고, 원래 20,000행 메타데이터의 첫 16행과 두 군의 전체 원본·변환 기록·실제 ACK·종료 집계를 연결했습니다. 소스 14개·보존 복사 14개·입력 7개·선택 provider 6개와 새 원문 계획·권한·결과·기록을 포함합니다.

독립 검사기는 **저장 바이트와 메타데이터를 검산**했습니다. 이미지 배열을 다시 해독하거나 float32 변환을 두 번째로 계산하지 않았습니다. 실제 첫 실행의 모든 값 비교와 별도 코드의 저장 검산은 서로 다른 근거입니다. Torch/GPU 수치 등가성, 외부 학습기 소비, 전체 native 메모리 할당량을 독립 인증했다고 주장하지 않습니다. [독립 저장 검산 전체 결과](../../../../../../storage/experiments/diagnostics/2026-10-08/map_serial_training_input_v1_independent_v1.json).

## 완료·준비·미구현을 구분하면

| 상태 | 내용 |
| --- | --- |
| 완료 | 단일 프로세스 입력 어댑터, 두 crop 방식, 실제 NumPy 값 검사, ACK 연결, 8배치 단위 기록, 실패·종료 처리, 실제 제한 실행·독립 저장 검산 |
| 준비만 됨 | 20,000배치를 8개씩 나누는 2,500개 묶음의 **메타데이터 계획 함수**. 전체 배열 실행·파일 생성·학습 허가는 아님 |
| 아직 미구현·미검증 | Torch tensor 연결과 수치 등가성, 실제 trainer 연결, 여러 worker와 prefetch, 전체 20,000배치 처리량·용량, 새 학습·인식 개선·지도 기능 승인 |

현재 선언의 2MiB 한도로 전체 20,000배치 기록까지 저장할 수 있다고 말하지 않습니다. 여러 worker를 추가할 때는 부모 메모리만 보는 것으로 충분하지 않으며, 자식·prefetch 버퍼·총 기록 용량·소유권을 함께 제한해야 합니다. 먼저 단일 프로세스 경로를 유지합니다.

## 다음 진행 순서

<!-- HH_261008 - Make the remaining tensor/trainer boundary explicit instead of repeatedly describing a dry consumer as completed learning. -->

1. **비학습 입력 연결:** 기존 허용 환경에 Torch가 있는지와 실제 trainer가 요구하는 형식을 먼저 확인합니다. 설치부터 하지 않습니다. 새 revision에서 NumPy→Torch 경계를 검사하고, 동일 구조 crop A/B의 입력 순서·손실·seed·초기 가중치·종료점·평가 범위를 고정합니다. 기존 frozen trainer는 보존합니다.
2. **PRO6000 후속 학습:** 접속 복구와 개인 hwanhong venv, 실행 직전 물리 GPU 0 가용성을 확인한 뒤 진행합니다. 승인된 TRAIN 범위만 사용하고 crop 하나만 바꾼 두 모델을 학습·평가합니다. GPU 1과 다른 사용자의 작업은 사용하지 않습니다. 이번에는 접속이나 학습을 실행하지 않았습니다.
3. **지도 성능 판단:** 기존 350장·23/27 통과·4개 실패·후보 미채택을 유지한 상태에서 표지판 누락·오탐, 다른 정적 분류의 회귀, 독립 평가를 확인합니다. 입력 보존 효과가 실제 모델 인식에 도움이 되는지는 여기서 판단합니다. 이미 노출된 DEV를 새 TEST로 부르지 않습니다.
4. **주행은 별도:** RTE-01과 SAF-01부터, 같은 모델의 실행 지연과 실제 폐루프를 검증합니다. 기본 직진·좌우회전·속도 추종·종점 정지가 안정된 뒤 신호 준수·ACC·회피·차선변경을 붙입니다. 지도 없이도 주행하도록 유지하고 지도 보조는 별도 있는/없는 비교로 평가합니다.

이번 제한 실행의 75.26초를 전체 학습 시간으로 확대하지 않습니다. Tensor 연결 개발·전체 loader·서버 가용성·학습 시간은 아직 측정하지 않았습니다. 크레딧 잔액과 정확한 미래 토큰 비용도 조회 근거가 없어 확정하지 않습니다. 기존 모델과 동급 성능이라는 비교 결과도 없습니다.

## 코드·기록을 찾는 방법

[입력 어댑터](../../../../../../learning/portable_e2e/map_serial_training_input_adapter_20261008_v1.py) · [어댑터 시험](../../../../../../learning/tests/test_map_serial_training_input_adapter_20261008_v1.py) · [실제 실행기](../../../../../../tools/e2e/verify_map_serial_training_input_20261008_v1.py) · [독립 저장 검사기](../../../../../../tools/e2e/check_map_serial_training_input_saved_20261008_v1.py).

[새 원문 ROOT 계획](../../../../../../storage/experiments/plans/2026-10-08/map_serial_training_input_v1/ROOT_plan.json) · [이번 제한 권한](../../../../../../storage/experiments/plans/2026-10-08/map_serial_training_input_v1/ROOT_authority.json) · [전체 소스·범위·한도 검토](../../../../../../storage/experiments/plans/2026-10-08/map_serial_training_input_v1/ROOT_execution_review_v1.json).

실제 기록은 원본 방식과 표지 보존 방식의 폴더 아래에 각각 `shard_0000_exposures.jsonl`, `shard_0001_exposures.jsonl`, 대응하는 `host_receipts.jsonl` 두 파일로 나뉩니다. 전체 결과와 실행 명령은 위 링크에서 확인할 수 있습니다. 이미 종료된 명령을 그대로 다시 실행하면 고정 출력이 있어 거절됩니다. **새 revision·출력·해시·자원 확인·제한 권한**으로 재현하며, 결과를 삭제하고 재시도하지 않습니다.

이번에는 PNG/GIF를 새로 만들지 않았습니다. [앞선 실제 원본·crop·복구 진단 PNG](../02_actual_rescue_and_reader_ab/README.md)는 그때 기록한 입력 화면이며, 이번 새 주행이나 RViz·모델 인식 결과가 아닙니다.

`storage/experiments/`는 Git 제외 대상입니다. 코드·문서 clone만으로 실제 데이터·가중치·과거 결과가 생기지는 않습니다. 원본·기존 결과·레거시 제거 상태를 유지했고 영구 삭제·commit·push는 하지 않았습니다.

<!-- HH_261008 - Retain the known missing frozen source link without claiming the whole workspace passes navigation validation. -->

[이번 읽기 전용 링크 검사](../../../../../../storage/experiments/diagnostics/2026-10-08/navigation_links_after_serial_training_input_v1.json)는 검사 시점 684개 문서·8,027개 로컬 링크에서 레거시0·기존 고정 자료 누락1개·범위 밖18개를 확인해 종료값1을 보존했습니다. **이번 새 문서의 잘못된 링크는0개**입니다. 누락은 앞서 알려진10월1일 고정 소스의 `WEIGHT_PROVENANCE_CORRECTION.md`이고 추정 원본을 만들지 않았습니다. 링크 검사를 실행 환경·모델 성능 합격으로 세지 않습니다.

[오늘 전체 현황](../../README.md) · [앞선 실제 복구·캐시 비교](../02_actual_rescue_and_reader_ab/README.md).
