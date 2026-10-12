# 실제 표지판 복구와 데이터 읽기 개선

<!-- HH_261008 - Separate measured input rescue and bounded-reader acceleration from untrained recognition and driving capability claims. -->

**실제 입력에서 표지판 정답 복구 2회를 확인했고, 같은 결과를 유지하면서 읽기 시간을 줄였습니다.** 동일한 첫 16배치에서 캐시를 끈 실행은 237.06초, 켠 실행은 34.28초였습니다. 두 설정 모두 16개 통과·실패 0·미실행 0이며, RGB와 정답 결과가 전부 일치했습니다. 실행과 독립 저장 검산은 모두 종료했습니다.

이것은 **자동 지도 모델의 학습 입력 준비**에서 얻은 개선입니다. 모델 가중치·인식 성능·차량 주행 능력을 개선했다고 판단한 결과는 아닙니다. 전체 목표는 자체 주행 모델의 메인 9개 영역·서브 30개 기능과 별도 자동 지도 모델이며, 정식 완료는 **메인 0/9·서브 0/30**입니다. 주행 첫 목표 RTE-01 직진 차로 유지와 SAF-01의 100ms 지연 미달은 유지합니다. [전체 합격 기준](../../../../../portable-e2e-feature-roadmap.md), [두 모델의 현재 구조와 범위](../../model_scope/README.md)를 따릅니다.

## 무엇을 바꿨나요

큰 압축 파일에서 임의의 영상을 읽으면 앞부분을 반복해서 압축 해제해야 합니다. 새 reader는 중간 해제 상태를 제한된 메모리에 기억해 다음 읽기의 출발점으로 사용합니다. **이미지 전체를 캐시에 보관하거나 디스크에 전체 배열을 풀어 놓지는 않습니다.**

두 설정 모두 같은 V3 reader·crop worker·정답 보존 정책을 사용했습니다. 바꾼 것은 체크포인트 캐시의 켜짐/꺼짐뿐입니다. 순서·원래 crop 영역·FALSE 입력·RGB BILINEAR·정답 NEAREST·원본 해시 검사는 동일했습니다. 측정 구간에 같은 작업을 넣었고, 두 결과의 비교와 PNG 생성은 측정 구간 밖에서 수행했습니다.

캐시는 최대 48개 상태에 상태당 256KiB를 예약하는 방식으로 총 12MiB로 제한했습니다. 이 예약값은 native decoder의 정확한 실제 할당량을 증명하는 수치가 아닙니다. 파일 소유 세대 검사는 유지하고, 비용이 큰 외부 자원 검사는 같은 코드에서 16회마다 수행합니다. 이 관측 검사는 OS 강제 차단 보장이 아닙니다.

## 실제 측정 결과

| 항목 | 캐시 없음 | 제한 캐시 사용 |
| --- | ---: | ---: |
| 선언한 배치 | 첫 1–16번, 각 8개 입력 | 동일 |
| 통과 / 실패 / 미실행 | 16 / 0 / 0 | 16 / 0 / 0 |
| 입력 표본 노출 | 128회 | 같은 128회 재생 |
| 실제 RGB·정답 복구 | 2회 | 같은 2회 |
| 시작·처리·정리를 포함한 설정별 시간 | 237.06초 | 34.28초 |
| 반복해서 새로 압축 해제한 앞부분 | 38,019,686,400바이트 | 5,283,799,938바이트 |
| 반환한 native RGB·정답 바이트 | 117,964,800바이트 | 동일 |
| 캐시 복원 hit | 0 | 213회; RGB/정답 member별 횟수 |
| 관측 최대 체크포인트 상태 | 0 | 34개 |
| 캐시 예약 관측 최대 | 0 | 8,912,896바이트, 8.5MiB |
| 종료 후 캐시 상태 / 예약 | 0 / 0 | 0 / 0 |

**이번 실행 시간은 약 85.5%, 반복 prefix 압축 해제량은 약 86.1% 감소했습니다.** 두 설정의 RGB·정답 결과 비교 16개도 전부 통과했습니다. 전체 실행은 274.11초, 관측 최대 VmHWM은 62,889,984바이트, 약 60MiB였습니다. 종료 seal 57개가 모두 일치했고 정리 오류는 0개입니다.

이 시간은 한 번의 고정된 순차 실행 결과입니다. OS 페이지 캐시와 실행 순서를 통제한 반복 성능 시험은 아니므로 **항상 6.9배 빠르다거나 전체 학습이 그만큼 빨라졌다고 보장하지 않습니다.** 앞선 다른 V2 reader의 8배치·74.84초를 같은 범위의 대조군으로 사용하지 않았습니다. 전체 20,000배치 처리량·실제 trainer 속도는 아직 측정하지 않았습니다.

두 실행은 같은 128회 노출을 반복했습니다. 256개의 새로운 독립 장면이 아닙니다. 원래 20,000배치 중 **각 설정에서 19,984배치는 범위 밖**입니다. 첫 16개 범위와 메타데이터 근거는 새 영상 해독 전에 고정했고, 성공 사례만 골라 분모를 바꾸지 않았습니다.

38GB와 5.28GB는 반복 처리한 압축 해제 바이트의 누적값이며 **그만큼 디스크 파일을 새로 만든 것이 아닙니다.** 계획·보존 소스·두 journal·결과·PNG를 합친 이 실험의 실제 파일 크기는 1,140,412바이트입니다. 모의 검사 임시 파일과 별도 검산 영수증은 이 합계와 구분합니다. 기본 CARLA·Unreal·Autoware·RoadRunner·설치 환경·데이터·이전 실패를 삭제하지 않았습니다.

## 복구가 실제로 어떻게 됐나요

여기서 복구란 인식 모델의 예측을 고쳤다는 뜻이 아닙니다. **학습용 화면을 잘라 확대할 때 표지판 정답이 완전히 사라지는 경우, RGB와 정답을 함께 원래 전체 영역으로 돌리는 처리**입니다. 과거 RGB·정답 배치 해시를 먼저 둘 다 재현한 뒤 같은 선택 영역을 적용했습니다.

| 발생 위치 | 원래 전체 정답 | 원래 잘림 후 정답 | 새 전체 영역 선택 후 정답 |
| --- | ---: | ---: | ---: |
| 배치 6, 위치 2, 슬롯 1036 | 표지판 56픽셀 | 0픽셀 | 56픽셀 |
| 배치 10, 위치 4, 슬롯 344 | 표지판 37픽셀 | 0픽셀 | 37픽셀 |

위치는 0부터 시작하는 기록 번호입니다. 두 설정에서 같은 복구·같은 RGB/정답 해시를 얻었습니다. 보호 대상이 아닌 입력을 임의로 복구하거나 정책을 바꾸지는 않았습니다. 앞선 8배치의 복구 0회 결과도 [별도 기록](../README.md)에 보존합니다.

![원본·잘림·복구 영상과 실제 표지판 정답 비교](../../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_rescue_reader_ab_v1/first_actual_rescue.png)

화면 왼쪽은 원본 전체 입력, 가운데는 원래 잘림, 오른쪽은 실제 선택된 복구 입력입니다. 아래의 흰색은 **정답 label 5**이며 모델 예측이 아닙니다. 첫 복구의 전체 영상·정답 바이트 해시와 56→0→56픽셀을 검사한 뒤 생성했고 직접 열람했습니다. 1920×768 PNG, 501,022바이트로 기존 512KiB 한도 안입니다.

**실제 기록 입력으로 만든 진단 화면이지 새 CARLA 촬영·Autoware/RViz 스크린샷이 아닙니다.** 이 입력에서는 ego 차량·경로가 관측되지 않았으므로 차량과 궤적을 만들어 그리지 않았습니다. [별도 차량 중심 기록 진단 GIF·PNG](../../recorded_visuals/README.md)는 주행 화면 자료입니다. 이번 입력 검사에 새로운 주행 GIF가 있다고 설명하지 않습니다.

## 검산과 보존 범위

새 reader의 모의 테스트 79개, 실행기의 스칼라 테스트 44개, 독립 저장 검산기의 스칼라 테스트 49개가 각각 통과했습니다. 이는 자율주행 기능이나 학습 횟수가 아닙니다. 앞선 검사들과 겹치는 항목을 새 개선 횟수로 합산하지 않습니다.

별도 작성한 표준 라이브러리 검산기는 실제 **3.09초에 파일 68개**의 앞뒤 바이트 해시·파일 세대를 확인했습니다. 현재 소스 18개·보존 복사 18개·입력 8개·보조 메타데이터·선택 scientific provider 6개·그래픽 소스 3개·계획·권한·결과·journal·PNG와 검산 소스를 포함합니다. 원래 20,000행 메타데이터와 선택 16행, 두 journal의 128개씩 원본 witness, 실제 복구 지원·영역·캐시/종료 통계를 연결했습니다.

**독립 검산은 저장된 바이트와 메타데이터의 검증입니다.** 후보 픽셀을 두 번째로 재생해 해시를 독립 재계산한 것은 아니며, PNG도 컨테이너·IHDR·chunk CRC를 확인했습니다. 정답의 현실 정확성·모델 인식 성능·모든 native stream의 전체 CRC를 증명하지 않습니다. 필요한 부분 앞의 압축 해제도 수행했고, TRUE·DEV·HOLDOUT·TEST·가중치·거절된 RAW 캡처를 추가 해독하지 않았습니다.

최종 소스 검토 전 초안에서는 실패 분모 중복, 비교 비용 비대칭, 종료 seal·PNG 연결 문제를 수정했습니다. SOURCE_READY 후 고정 실험을 덮어쓰지 않았습니다. ROOT 권한 생성의 첫 명령은 지원하지 않는 `-c` 옵션으로 거절됐으며, 실제 데이터 실행은 0회였습니다. 그 호출 기록도 보존하고 지원하는 별도 Python 호출로 권한을 만든 뒤 실행했습니다. 이전 실험의 실패와 원본은 유지했습니다.

## 현재 상태와 다음 순서

| 상태 / 환경 | 내용 |
| --- | --- |
| 완료 / 로컬 CPU | 실제 복구 2회, 동일 16배치 캐시 비교, 바이트 일치, PNG 열람, 독립 저장 검산 |
| 미구현·미실행 / 로컬 후속 | 새 trainer 입력 경로와 이 worker/writer 연결, 실제 학습 처리량 검사 |
| 미실행 / PRO6000 | 새 지도·주행 학습. 이번 추가 작업에서 SSH 접속도 조회하지 않음 |
| 미달 / 주행 | SAF-01 100ms 지연, RTE-01 독립 직진 및 회전·목표 정지 승인 |

다음 비학습 순서는 **새 revision의 학습 입력 어댑터 구현→고정 입력 순서·같은 crop·실제 journal·실패 처리 검사→제한된 실제 입력 검증**입니다. 기존 frozen trainer와 결과를 수정하지 않고, 전체 배열을 펼치는 기존 경로를 그대로 빠른 loader라고 주장하지 않습니다. 본 실험의 16개 결과를 전체 처리량으로 확대하지 않습니다.

<!-- HH_261008 - Ground the unimplemented next adapter in source-audited loader, journal and process-ownership constraints. -->

소스 검토로 다음 연결의 장애물도 확인했습니다. [기존 CropBatches·loader](../../../../../../tools/e2e/train_map_native_fusion_pair_20261002.py)는 [train_package](../../../../../../tools/e2e/map_native_fusion_contract_20261002.py)와 [read_npz](../../../../../../tools/e2e/map_paired_balanced_contract_20261002.py)를 통해 전체 배열을 읽습니다. 사용하지 않는 TRUE 배열까지 펼치는 경로가 있지만 **이번 실행은 그 경로를 호출하거나 TRUE를 해독하지 않았습니다.** 기존 실험은 구조가 다른 두 모델을 비교하는 권한이므로, 같은 모델에서 crop만 바꾸는 후속 학습 권한으로 재사용하지 않습니다.

새 연결은 먼저 **단일 프로세스·worker 0**에서 검증할 예정입니다. 원래 crop 대조군과 정답 보존 후보의 기록 형식을 구분해야 하며, 현재 후보 전용 journal에 대조군을 억지로 넣지 않습니다. 전체 20,000배치 기록은 기존 단일 journal 한도를 넘으므로 사전에 고정한 연속 분할·전체 용량·실패/미실행 분모가 필요합니다. 두 worker를 추가하려면 각각의 새 PID·reader·종료와 자식 메모리/prefetch 합산을 별도로 검사해야 합니다. 현재 부모 프로세스 VmHWM만으로 다중 프로세스 제한을 입증하지 않습니다. 원격에는 기존 flat source에 무작정 파일을 추가하지 않고 실제 import 의존성을 보존한 새 패키지 구성을 준비해야 합니다. **이 어댑터·병렬 연결·새 trainer는 아직 구현/실행하지 않았습니다.**

학습 재개는 PRO6000 접속·개인 hwanhong venv·물리 GPU 0 가용성을 즉시 확인한 뒤 진행합니다. 지도는 같은 구조·초기 가중치·손실·optimizer에서 crop 정책만 바꾸는 A/B부터, 주행은 지연·비구동 연결과 첫 RTE-01 행동 실패에 맞춘 별도 허용 TRAIN 보강부터 이어갑니다. 다른 사용자 작업을 종료하거나 GPU 1을 쓰지 않습니다. 인식·폐루프 평가와 기존 기능 회귀를 통과하기 전 모델을 자동 채택하지 않습니다.

이번 두 설정의 실제 계산은 약 4분34초, 저장 검산은 약 3초였습니다. 다음 입력 어댑터 구현·서버 복구·새 학습·폐루프의 완료 시간이나 정확한 토큰/크레딧 비용은 이 측정으로 확정하지 않습니다. 자동 HD map 없이도 동작하는 주행 모델 원칙을 유지하고, 지도 보조 입력은 별도로 검증합니다.

## 결과와 재현 근거

[실제 전체 결과](../../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_rescue_reader_ab_v1/result.json) · [캐시 없는 전체 journal](../../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_rescue_reader_ab_v1/CONTROL_V3_CACHE0_RGB_GT_batches.jsonl) · [캐시 사용 전체 journal](../../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_rescue_reader_ab_v1/CANDIDATE_V3_CACHE12M_RGB_GT_batches.jsonl) · [실제 명령·전체 stdout](../../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_rescue_reader_ab_ROOT_run_v1.json) · [독립 저장 검산](../../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_rescue_reader_ab_v1_independent_v1.json) · [독립 검산 명령](../../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_rescue_reader_ab_independent_ROOT_run_v1.json).

[reader 테스트](../../../../../../storage/experiments/diagnostics/2026-10-08/map_reader_v3_synthetic_tests_v1.json) · [실행기 테스트·prepare·호출 거절 기록](../../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_rescue_reader_ab_source_tests_v1.json) · [독립 소스 검토·49개 테스트](../../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_rescue_reader_ab_independent_ROOT_review_v1.json) · [실행 전 검토](../../../../../../storage/experiments/plans/2026-10-08/map_crop_rescue_reader_ab_v1/ROOT_execution_review_v1.json) · [원문 ROOT 계획](../../../../../../storage/experiments/plans/2026-10-08/map_crop_rescue_reader_ab_v1/ROOT_plan.json) · [별도 실행 권한](../../../../../../storage/experiments/plans/2026-10-08/map_crop_rescue_reader_ab_v1/ROOT_authority.json).

[reader V3](../../../../../../learning/portable_e2e/map_native_pair_reader_20261008_v3.py) · [실행기](../../../../../../tools/e2e/verify_map_crop_rescue_reader_ab_20261008_v1.py) · [독립 검산기](../../../../../../tools/e2e/check_map_crop_rescue_reader_ab_saved_20261008_v1.py).

저장 명령은 **종료된 고정 실행의 재현 근거**입니다. 그대로 재실행하면 기존 출력 때문에 거절됩니다. 재실행에는 새 revision·경로·해시·자원 확인·권한이 필요하며 폴더를 지우고 재시도하지 않습니다. `storage/experiments/`는 Git 제외 대상이므로 clone만으로 실제 데이터·PNG·결과가 생기지는 않습니다. 이번 작업은 설치·삭제·원격 접속·GPU·CARLA·ROS·학습·commit/push를 수행하지 않았습니다.

<!-- HH_261008 - Keep the readonly navigation outcome separate from model and runtime validation. -->
[읽기 전용 링크 검사](../../../../../../storage/experiments/diagnostics/2026-10-08/navigation_links_after_rescue_reader_ab_v1.json)는 검사 당시 683개 문서·8,002개 링크에서 레거시 0·누락 1·범위 밖 홈 링크 18개를 기록했습니다. 새 자료의 누락은 없고, 남은 1건은 10월 1일 보존 소스의 `WEIGHT_PROVENANCE_CORRECTION.md`입니다. 추정 원본을 만들거나 고정 자료를 덮어쓰지 않았습니다. `git diff --check`도 이번 수정 문서에서 통과했습니다.

[상위 연결부 기록](../README.md) · [오늘 전체 현황](../../README.md).
