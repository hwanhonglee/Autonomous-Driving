# 학습 없이 연결부 구현: 실제 영상·정답 배치와 센서 수집 검사

<!-- HH_261008 - Report implemented byte processing and fictional callbacks without claiming new training, live capture or driving approval. -->

**실제 RGB·정답을 읽고, 같은 영역으로 자르고, 새 기록을 저장하는 연결부를 구현했습니다.** 실제 데이터 8개 배치는 모두 검증을 마쳤습니다. 센서 수집 연결부도 모의 입력으로 검사했습니다. 모델 가중치는 바꾸지 않았고, 주행·인식 성능이 좋아졌다는 결과는 아직 없습니다.

정식 완료는 **메인 0/9, 서브 0/30**입니다. 주행의 첫 목표는 RTE-01 직진 차로 유지이며, 앞선 SAF-01 처리 지연 실패는 그대로입니다. 이번 작업은 그와 병렬인 **자동 지도 모델의 입력 연결**입니다. [전체 목표와 합격 기준](../../../../portable-e2e-feature-roadmap.md), [현재 주행·지도 모델이 하는 일](../model_scope/README.md)을 따릅니다.

## 최신 완료: 입력 형식·기록·수신 확인 연결

<!-- HH_261008 - Index the ended real-data serial adapter and retain prior rescue and collection results as history. -->

새 입력 어댑터를 구현하고 실제 NumPy 값 검사→기록 저장→ACK까지 검증했습니다. 원본 방식과 표지 보존 방식 각각16개 통과·실패/미실행0, 같은128회 노출이며 후보 복구2회입니다. 전체75.26초·관측 최대약130MiB·종료seal41개 일치, 별도 코드의 지정 파일57개 저장 검산3.11초입니다. 두 작업 모두 종료했고 **Torch·실제 trainer 연결·학습은 아직 없습니다.**

[읽기 쉬운 새 결과와 다음 순서](03_serial_training_input/README.md)를 먼저 보세요. 완료0/9·0/30과 주행지연 실패는 유지합니다. 다음은 기존 허용 환경의 Torch/trainer 경계 확인이며, 아래는 앞선 종료 실험의 기록입니다.

## 앞선 완료: 실제 복구와 읽기 속도 비교

<!-- HH_261008 - Index the fresh completed rescue/cache A/B without rewriting the preceding eight-batch experiment. -->

**13:09 KST 추가 확인:** 새 첫 16배치 실험에서 실제 RGB·정답 복구 2회를 확인했습니다. 같은 V3 reader의 캐시 꺼짐/켜짐만 비교해 양쪽 모두 16개 통과·실패/미실행 0·RGB/정답 전부 일치를 얻었고, 설정별 시간은 **237.06→34.28초**, 반복 압축 해제량은 약 **86.1% 감소**했습니다. 최대 관측 메모리 약 60MiB·전체 실행 274.11초·독립 저장 검산 68개 파일·3.09초입니다. 실제 입력 진단 PNG도 직접 확인했습니다.

[새 실제 결과·PNG·비교 범위·다음 단계](02_actual_rescue_and_reader_ab/README.md)를 먼저 보세요. 한 번의 순차 비교이며 전체 trainer 속도·인식/주행 성능 향상·새 학습은 미검증입니다. 공식 승인 0/9·0/30과 주행 지연 미달은 유지합니다. **아래는 앞선 8배치와 모의 수집의 종료 기록**이며 당시 복구 0회와 실패 원본을 그대로 보존합니다.

## 앞선 구현에서 달라진 것

| 대상 | 완료한 것 | 아직 하지 않은 것 |
| --- | --- | --- |
| 실제 RGB·정답 배치 | 제한 메모리 reader, 같은 영역 crop worker, 실제 바이트 해시 journal writer 구현. 실제 8개 배치에서 양쪽 원래 해시 재현 | 실제 복구 조건이 발생한 RGB 배치, 전체 20,000개 RGB 검증, trainer 연결, 학습·인식 개선 |
| 센서 수집 연결 | 생성→tick→센서 응답→제공된 저장 바이트 확인→개별 정리 callback 연결. 정상·응답 중단·소유권 변경 등 검사 | 실제 커널 소유권, 실제 CARLA 최초 RPC/촬영, 실제 파일 저장 인증, 프레임 간 주기 인증 |
| 기존 주행 모델 | 변경 없음 | 10Hz 지연 통과, 독립 직진·회전·목표 정지 합격, 새 학습 |

**모두 로컬 CPU 작업입니다.** PRO6000 접속·패키지 설치·GPU·CARLA·ROS·실차 구동·모델 승격·commit/push는 하지 않았습니다. 다른 테스트의 시뮬레이터를 사용하거나 종료하지 않았습니다. 실행은 기존 공유 잠금으로 직렬화했습니다.

## 1. 실제 영상·정답: 8개 배치 검증 완료

새 reader는 승인된 원본 TRAIN 729개와 paired FALSE 480개의 RGB·정답만 대상으로 합니다. 큰 배열 전체를 메모리에 펼치거나 원본을 수정하지 않고, 필요한 배치의 배열을 반환합니다. worker는 먼저 **과거 RGB와 정답의 원래 배치 해시를 둘 다 재현**한 다음 새 정책의 선택 영역을 적용합니다. RGB는 원래 BILINEAR, 정답은 원래 NEAREST를 유지합니다.

| 실제 측정 | 결과 |
| --- | ---: |
| 고정 배치 번호 | 1, 2, 50, 100, 500, 1,000, 10,000, 20,000 |
| 선언 / 통과 / 실패 / 미실행 | 8 / 8 / 0 / 0 |
| 실제 배치 표본 노출 | 64회; 독립 장면 수와 다름 |
| 전체 과거 배치 중 이번 범위 밖 | 19,992개 |
| 실제 실행 시간 | 74.84초 |
| 관측 최대 VmHWM | 54,980,608바이트, 약 55MB |
| 원래 양쪽 배치 해시 재현 | 8개 모두 일치 |
| 선택한 표본의 복구 조건 발생 | 0회 |
| 종료 소스·입력·선택 provider seal 오류 / 정리 오류 | 0 / 0 |
| NN / optimizer 갱신 / 기능 승인 | 0 / 0 / 0 |

**복구 0회는 중요한 제한입니다.** 이번 표본에서는 정책이 원래 crop을 그대로 선택했습니다. 따라서 실제 데이터에서 변경 없는 경로와 배치 기록 연결을 확인했지만, 실제 RGB의 복구 분기까지 입증하지는 못했습니다. 합성 검사에서는 표지판이 완전히 잘리는 조건을 따로 시험했습니다. 그것을 실제 촬영의 개선으로 바꾸어 설명하지 않습니다.

앞선 정답 전용 20,000배치 재생의 완전 잘림 **3,626→1,821회**와 이번 RGB 검증은 다른 실험입니다. 앞선 결과는 실제 정답의 보존 효과이고, 이번 8개 결과를 전체 RGB·모델 효과로 확장하지 않습니다. [앞선 정답 보존 결과](../mapping_review/crop_preservation/README.md)를 유지합니다.

압축 파일의 임의 위치를 읽으면 앞부분도 반복해서 압축 해제됩니다. **64개 표본만 반환했다고 64개 프레임의 압축만 풀었다고 말할 수 없습니다.** 기록된 prefix 정보는 배치마다 마지막 표본의 8개 witness이며, 전체 64개 prefix 기록은 아닙니다. 각 표본의 원본 RGB·정답 바이트 해시 64개는 새 journal에 남겼습니다. TRUE view는 해독하지 않았으며, whole-file SHA에 포함된 압축 바이트 읽기와 배열 해독은 구분합니다.

현재 reader는 제한 검증용입니다. 이 반복 읽기 비용을 그대로 두고 빠른 학습 loader가 완성됐다고 할 수 없습니다. 전체 20,000개 처리량과 900초 충족은 미측정이며, 이번 작은 실행 시간을 학습 완료 시간으로 적용하지 않습니다. 메모리·시간 검사는 관측값에 대한 검사이고 OS 강제 차단 보장은 아닙니다. provider 스레드 환경은 1로 설정했지만 모든 native 스레드를 독립 전수 조사한 것은 아닙니다.

[실제 결과](../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_batch_bytes_v3/result.json) · [8개 전체 배치 journal](../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_batch_bytes_v3/candidate_RGB_GT_batches.jsonl) · [실제 실행 명령·전체 stdout](../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_batch_bytes_v3_ROOT_run_v1.json) · [새 원문 ROOT 계획](../../../../../storage/experiments/plans/2026-10-08/map_crop_batch_bytes_v3/ROOT_plan.json) · [별도 제한 실행 권한](../../../../../storage/experiments/plans/2026-10-08/map_crop_batch_bytes_v3/ROOT_authority.json) · [실행 전 검토](../../../../../storage/experiments/plans/2026-10-08/map_crop_batch_bytes_v3/ROOT_execution_review_v1.json).

**독립 저장 검산도 완료했습니다.** 별도 코드의 스칼라 검사 20개를 통과한 뒤 실제 3.05초에 지정 파일 50개의 앞뒤 바이트 해시·세대를 확인했습니다. 현재 소스 14개와 보존 복사 14개, 입력 7개, 선택 provider 6개, 원문 계획·권한·결과·journal을 포함합니다. 원래 20,000행 메타데이터의 선택 8행, 새 8행·64개 원본 해시 witness, 마지막 표본의 prefix 정보와 실패/미실행 분모를 연결했습니다. **픽셀을 두 번째로 재생하거나 후보 픽셀 해시를 독립 재계산한 것은 아닙니다.**

[독립 전체 저장 검산 결과](../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_batch_bytes_v3_independent_v1.json) · [전체 소스 검토·20개 모의 검사·제한 실행 범위](../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_batch_bytes_v3_independent_ROOT_review_v1.json) · [독립 검산 코드](../../../../../tools/e2e/check_map_crop_batch_bytes_saved_20261008_v1.py).

## 2. 센서 수집: 실제 촬영이 아닌 모의 연결 검사

수집 어댑터는 각 동작 직전에 새로 제공된 소유자·프로세스 세대·listener·actor 정보의 계약을 확인하고 callback을 호출합니다. actor의 실제 객체 ID와 제공된 등록 ID를 비교하고, RGB·깊이·semantic의 frame·시각·pose·원시 바이트 해시를 연결합니다. 원래 실패와 후속 정리 실패를 함께 보존합니다.

이 단계는 **제공된 메타데이터를 검사하는 어댑터**입니다. 제공자가 진짜 커널 상태를 읽었다는 인증, 실제 최초 RPC, 실제 CARLA 실행기 연결, 파일의 inode·경로·내용 인증까지 구현한 것은 아닙니다. 제공된 저장 바이트의 SHA가 맞는 것과 실제 파일이 올바르게 저장됐다는 것은 다릅니다.

실제 실행한 모의 기록 두 가지는 다음과 같습니다.

- 정상 조건: 선언한 2프레임 모두 통과, 카메라 응답 6개, 제공된 저장 바이트 확인 4회. 소유한 모의 actor를 모두 정리했습니다.
- 깊이 응답 중단 조건: 5프레임 중 첫 프레임 실패, 나머지 4프레임 미실행. RGB 응답 1개·실패한 깊이 응답 1개·미실행 카메라 13개를 모두 남겼고, 뒤의 저장 callback은 호출하지 않았습니다. 모의 actor는 정리했습니다.

두 기대 동작의 검사는 통과했지만 **실패 조건의 촬영 결과를 성공 프레임으로 세지는 않습니다.** 원시 BGRA는 합성 버퍼이고, 제공된 encoded 바이트도 PNG·NPZ 형식을 갖춘 실제 영상이 아닙니다. 실제 촬영·진짜 저장 화면·모델 제어는 0회입니다. 이 자료로 화면 PNG나 CARLA 촬영을 만들어 보여주지 않습니다.

[두 조건의 전체 모의 결과·실패 꼬리](../../../../../storage/experiments/diagnostics/2026-10-08/mapping_capture_callback_record_v1.json) · [실제 모의 명령·범위](../../../../../storage/experiments/diagnostics/2026-10-08/mapping_capture_callback_ROOT_run_v1.json).

## 3. 검사는 어떻게 진행됐나요

<!-- HH_261008 - Retain failed revisions and avoid adding overlapping unit-test totals as new features. -->

첫 통합 검사는 **143개 통과·18개 실패**였습니다. reader가 Python 3.10의 정상적인 inline ZIP64 헤더를 거절하는 문제였습니다. 실패 원본을 보존하고, 정상 형식과 잘못된 형식을 구분하는 새 reader V2에서 수정했습니다. 메모리·시간·원본 해시 기준을 완화하지 않았습니다.

이후 182개가 통과했고, Pillow의 실제 RGB resize core와 선택 native plugin을 구분하는 3개 검사를 추가해 최종 **185개 통과·0개 실패, 7.45초**를 확인했습니다. 182개는 185개에 포함되므로 합산하지 않습니다. 검사 수는 학습 횟수나 자율주행 기능 수가 아닙니다.

스칼라 계약, 실제 합성 RGB·정답의 같은 crop, 해시 불일치·버퍼 변경 거절, create-only journal, 같은 inode/같은 길이 변조, ZIP 헤더·크기·CRC 경계, 센서 timeout·actor/listener 변경·부분 저장·정리 실패를 포함합니다. 실제 원본 영상은 이 합성 suite 뒤의 별도 8배치 실행에서만 검증했습니다.

[첫 실패 기록](../../../../../storage/experiments/diagnostics/2026-10-08/nontraining_batch_capture_synthetic_suite_v1.json) · [182개 통과 기록](../../../../../storage/experiments/diagnostics/2026-10-08/nontraining_batch_capture_synthetic_suite_v2.json) · [최종 185개 명령·전체 콘솔](../../../../../storage/experiments/diagnostics/2026-10-08/nontraining_batch_capture_synthetic_suite_v3.json).

최초 실패 콘솔은 도구 전송에서 일부 잘렸고 18개 실패 요약은 보존돼 있습니다. 마지막 suite의 콘솔은 전량 보존했습니다. 소스 검토에서 발견한 worker·verifier의 중간 revision도 그대로 남겼으며, 실행하지 않은 초안의 SOURCE 문제를 실제 데이터 시험 실패로 세지 않습니다.

## 앞선 8배치 종료 당시의 다음 순서

1. **로컬·비학습:** 실제 표지판 복구 조건이 발생한 배치를 새 선언으로 검증합니다. 원래 양쪽 해시를 재현하고, 변경된 RGB·정답에 같은 전체화면 영역을 적용했는지 확인해야 합니다. 이번 실행을 이어 쓰거나 원본 journal을 덮어쓰지 않습니다.
2. **로컬·비학습:** 제한 메모리와 공간을 유지하면서 반복 압축 해제 비용을 줄이는 reader 후보를 한 가지씩 비교합니다. 실제 처리량·바이트 동일성을 계측한 뒤 학습용 연결 방식을 결정합니다. 실패 원본을 학습 가능으로 바꾸지 않습니다.
3. **PRO6000 접속 복구 후:** 개인 hwanhong 환경과 물리 GPU 0 가용성을 확인합니다. 지도는 같은 구조·초기 가중치·손실·optimizer에서 crop 한 가지만 A/B 변경하고, 원래 인식 평가·누락/오탐·기존 기능 회귀를 유지합니다. 실제 trainer 연결과 새 학습은 아직 미실행입니다.
4. **주행은 별도:** 공간·소유 자원 조건을 충족한 뒤 기존 GOOD 모델의 GPU 출력·지연을 비교하고, 소유한 CARLA에서 직진→회전→목표 정지의 독립 반복으로 RTE-01부터 판단합니다. 지도 입력 검사가 주행의 SAF-01 지연 실패를 해결한 것은 아닙니다.

이후 위 1·2번의 제한 실제 복구·캐시 A/B는 [새 선언으로 완료](02_actual_rescue_and_reader_ab/README.md)했습니다. 이제 다음 비학습 작업은 새 revision의 trainer 입력 어댑터 연결과 순서·바이트·실패 검증입니다. 실제 trainer 처리량·새 학습은 아직 없습니다. 앞선 최종 합성 suite 7.45초·8배치 74.84초와 이번 다른 범위의 시간을 혼합하지 않습니다. 원격 가용성, 전체 loader 처리량, 학습·폐루프 종료 시간과 정확한 토큰/크레딧 비용은 이 근거로 확정할 수 없습니다.

## 코드와 재현

[제한 메모리 reader V2](../../../../../learning/portable_e2e/map_native_pair_reader_20261008_v2.py) · [배치 worker·writer V3](../../../../../learning/portable_e2e/map_crop_batch_worker_20261008_v3.py) · [실제 검증 실행기 V3](../../../../../tools/e2e/verify_map_crop_batch_bytes_20261008_v3.py) · [수집 어댑터 V1](../../../../../learning/portable_e2e/mapping_capture_adapter_20261008_v1.py) · [모의 수집 검사](../../../../../learning/tests/test_mapping_capture_adapter_20261008_v1.py).

실제 검증은 별도의 새 권한과 출력에서 실행했습니다. 이전 정답 전용 권한을 RGB 읽기 권한으로 늘려 쓰지 않았습니다. 기존 계획·소스·모델·원본·실패 결과는 덮어쓰지 않았고 레거시 링크도 만들지 않았습니다. `storage/experiments/`는 Git 제외 대상이므로 clone만으로 이 실제 데이터와 결과가 생기지는 않습니다.

저장된 명령은 이 종료 실행을 설명하는 재현 근거입니다. 그대로 다시 실행하면 이미 존재하는 고정 출력 때문에 거절됩니다. 재실행하려면 **새 revision·출력·해시·자원 확인·제한 실행 권한**이 필요하며, 결과 폴더를 지우고 재시도하지 않습니다.

<!-- HH_261008 - Preserve the known frozen missing destination and report navigation separately from runtime/model validation. -->
[마지막 읽기 전용 링크 검사](../../../../../storage/experiments/diagnostics/2026-10-08/navigation_links_after_batch_capture_v1.json)는 682개 문서·7,970개 로컬 링크에서 레거시 0개·기존 고정 자료 누락 1개를 확인했습니다. 이번 새 링크의 누락은 없습니다. 남은 것은 10월 1일 보존 소스의 `WEIGHT_PROVENANCE_CORRECTION.md`이며 원본을 추정해서 만들지 않았습니다. 검사 범위 밖 홈 링크 18개는 이번 도구의 검증 대상이 아닙니다. 이 링크 검사를 실행 환경이나 모델 성능 합격으로 세지 않습니다.

[오늘 전체 현황](../README.md) · [차량 중심 실제 기록 진단 영상](../recorded_visuals/README.md) · [지도 정답 보존과 A/B 설계](../mapping_review/crop_preservation/README.md).
