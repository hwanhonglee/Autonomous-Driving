# 오전 10시까지 — 정답 수집 품질 개선과 후속 학습

<!-- HH_260906 - Track the explicitly authorized overnight continuation without claiming unperformed experiments or model promotion. -->

사용자의 추가 요청으로 **2026-09-09 약 01:56 KST부터 10:00 KST까지** 작업을 이어갑니다.
종료 경계는 `2026-09-09T01:00:00Z`입니다. 이 문서는 새 작업의 진행 기록이며,
[앞서 완료한 6회 학습](../01_no_accel_development_ab/README.md)을 계속 실행 중이라고 표시하지 않습니다.

## 작업 순서와 환경

| 환경 | 이번 작업 | 제한 |
|---|---|---|
| 로컬 | 기존 원본의 원인 가설 대조, 소유한 CARLA에서 별도 expert 수집 실험, 코드 검사와 그림 정리 | 다른 CARLA/Autoware 작업 종료 금지, 새 Portable 차량 제어 금지 |
| SSH Pro6000 | 고정한 새 실험 조건으로 실제 모델 학습 후 같은 validation·runtime 검증 | physical GPU 0만, hwanhong 개인 venv만; 다른 작업·GPU 1·시스템 환경 불변 |
| 공통 | 실패도 보존, 원본·실행 코드·출력 해시 연결, 날짜/범주별 문서화 | 원본 라벨 덮어쓰기·미승인 데이터 편입·test로 설정 선택·자동 모델 승격 금지 |

1. 현재 로컬·원격 사용 상태와 남은 공간을 확인합니다.
2. 출발·정지 저속 곡률의 원인 가설을 좁히고 새 수집 비교 조건을 실행 전에 고정합니다.
3. 새 수집이 품질 조건을 충족하면 별도 데이터 버전으로 검증하고 학습합니다.
4. 새 데이터가 미승인이면 이를 숨기지 않습니다. 기존 v3 기반 모델 개선 연구는
   새 데이터 승인과 구분한 탐색적 실험으로만 실행하며 조건·목적을 별도로 기록합니다.
5. 학습 후 전체 고정 seed·평가·경로 검사를 정리하고, 오전 10시 전에 실행 종료·자원 반환을 확인합니다.

학습 후보 비교 기준과 안전 한계는 결과를 본 뒤 완화하지 않습니다. 새 수집·학습의
구체적인 실행 수·모델 변경은 검토 후 각각 별도 계획에 고정합니다. 아직 실행하지 않은
모델 학습이나 수집을 완료 수에 넣지 않습니다.

## 종료 정책

<!-- HH_260906 - Reserve wall-clock budgets for training, evaluation and owned cleanup before the user deadline. -->

새 CARLA 실행은 소유 helper의 `--finish-before-utc 2026-09-09T01:00:00Z`와
개별 timeout을 사용합니다. 원격 학습도 평가·종료 확인 시간까지 남겨두고 시작합니다.
권한 확대·새 라이선스 동의·개인 venv 외 설치가 필요하면 그 작업은 진행하지 않고 기록합니다.
시험 실패는 다음 원인 분석의 근거이며, 좋은 결과가 나올 때까지 같은 seed를 무제한 재시도하지 않습니다.

## 현재 상태

01:56 KST: 시간 경계와 기존 완료 상태를 확인했습니다. 로컬 실행 자원,
미세 이동·가감속 원인, 새 학습 목적을 병렬 검토 중입니다. 이 후속 작업의 새 학습·주행은 아직 0회입니다.

02:21 KST: [선택 손실 A/B의 고정 계획](02_cost_aware_learning/README.md)으로 원격 GPU 0의
실제 학습을 시작했습니다. 02:23 KST 확인에서 첫 A 모델의 optimizer step 437→438과
유한 손실을 확인했습니다. 이는 6회 전체 학습의 완료 판정이 아니라 시작 증거입니다.
로컬은 [물리 적분 상한 A/B](01_substeps/README.md)의 첫 A 수집을 별도로 시작했습니다.
학습 데이터는 기존 v3 그대로이며 새 수집 결과는 아직 편입하지 않았습니다.

02:44 KST: 새 원격 6회 학습·6회 평가·6회 감사를 모두 완료했고 미채택입니다.
상대 기준은 FAIL/PASS/PASS, 절대 기준은 세 seed 모두 FAIL입니다. 로컬 4회 수집도 완료했고
A의 저속 곡률과 B의 목표 정차·가감속 문제 때문에 모두 데이터 미승인입니다.
현재 모델·데이터를 승격하지 않고, 결과 게시·학습 처리 비용 계측·정지 후보를 추가하는
다음 별도 모델 비교 준비를 이어갑니다.

03:09 KST: [정지 후보 추가 모델 비교](04_stopmix_learning/README.md)의 실제 6회 학습을
GPU 0에서 시작했습니다. 첫 A 모델의 176 step을 확인했으며 나머지는 아직 완료로 세지 않습니다.
별도 [학습 처리 비용 계측](03_training_cost/README.md)은 80회 측정을 완료했지만
실제 전체 학습 처리량 개선이나 production 코드 변경으로 해석하지 않습니다.

03:41 KST: 정지 후보 비교의 학습·평가·감사 18개와 별도 행동 분석 6개가 모두 끝났습니다.
상대 비교는 FAIL/PASS/FAIL, 절대 기준은 세 seed 모두 FAIL로 모델은 미채택입니다.
행동 분석의 전체 원본을 가져와 각 파일·checkpoint 해시를 대조하고 있습니다.

03:44 KST: GPU 0에서 원본 trainer를 사용하는 16-step × 2회 계측 학습을 시작했습니다.
두 실행은 전체 캠페인 학습 횟수에 포함하지 않는 진단입니다. 기존 유한값 검사·데이터·손실·
loader를 바꾸지 않고 계측 비용과 결과 동일성을 확인합니다. 로컬 CARLA는 현재 종료 상태입니다.

03:45 KST: 16-step 계측 2회를 완료했습니다. 모든 32개 학습 기록과 creation timestamp만 제외한
checkpoint의 가중치·Adam·RNG·ABI 내용이 정확히 같습니다. 계측기는 원본 trainer와
데이터 준비·다수의 작은 CUDA 연산 호출 비용을 드러냈으며, 아직 속도 개선을 적용한 결과는 아닙니다.

03:52 KST: [새 전체 코드 검사](07_code_verification/README.md)는 고정 commit `99902f3`에서
**5,607 passed / 6 skipped / 0 failed**, 소스·설정 553개 전후 동일입니다.
이전 b478 검사 1건 실패와 테스트만 수정한 경위도 함께 보존했습니다.
다음은 기존 6개 모델의 원본 checkpoint를 보존한 고정 10-epoch 이어학습 비교를 준비합니다.
아직 시작 전이며 새 loss나 seed 선택 없이 학습량 부족 가설만 분리해 확인할 계획입니다.

04:07 KST: [고정 10-epoch 이어학습](08_stopmix_ten_epoch_continuation/README.md)을 시작했습니다.
04:23 KST에는 24단계 중 16단계 완료와 다섯 번째 모델의 실제 step 증가를 확인했습니다.
원본 6개 checkpoint를 독립 복사해 학습량만 늘린 비교로, 새 초기화 6회나 새 데이터 수집으로
세지 않습니다. 완료 후 모든 종점과 행동 지표를 함께 비교합니다.

<!-- HH_260906 - Keep timestamped in-progress observations but add the actual completed endpoints without inflating fresh-fit counts. -->
04:30:32 KST: 이어학습 24단계를 완료했습니다. 6개 모델에 각각 1,330 step을 추가했고
기존 이력 1,540행은 모두 부모 원본과 같습니다. 자기 부모 대비 상대 조건은 3/6 통과했지만
절대 조건은 0/6 통과, **1초·3초 ADE는 6개 모두 악화**했습니다. 모델은 미채택입니다.

04:35:40 KST: 별도 DataLoader ABBA 4회도 완료했습니다. 각각 TRAIN 1,147개를
287 step으로 한 번 순회한 진단입니다. 모든 metrics 원본과 선언한 두 필드 외 checkpoint
내용이 일치했습니다. worker 2의 평균 학습 구간 시간은 약 33.8% 짧았지만 표본은 조건당
2회이며 장기 학습·추론 성능이나 기본 설정 변경을 뜻하지 않습니다.

04:41:33 KST: nuPlan의 앞서 보존한 32개 calibration BLOB을 별도 사전 계획으로
로컬에서 해석했습니다. 실행형 역직렬화 없이 정확히 고정한 바이트 틀과 숫자 위치만 읽었고
8카메라의 기본 수치 검사를 통과했습니다. 실제 영상 투영·TF·물리 센서 배치·학습 승인은
여전히 확인되지 않았습니다. 원래 opcode-only 진단 결과는 변경하지 않았습니다.

<!-- HH_260906 - Append actual follow-up observations without relabeling earlier runs or the rejected pre-GPU launch. -->
04:59:48 KST: commit `7f1141b`의 새 전체 코드 회귀를 완료했습니다.
[5,960 통과 / 6 건너뜀 / 0 실패](07_code_verification/frozen_7f1141/README.md)이며,
소스·설정 567개 전후 동일입니다. 이후 추가한 oracle·선택기 코드를 포함한 수치는 아닙니다.

05:04:10–05:06:40 KST: [TRAIN oracle 진단](09_train_oracle_diagnosis/README.md)을
GPU 0에서 완료했습니다. 3개 seed의 부모/이어학습 체크포인트를 TRAIN 1,147개씩 진단한
6회 forward pass이며 새 학습은 아닙니다. 저장된 6,882행·82,584 cost를 로컬에서 다시
계산했습니다. 최소 cost 동률은 모두 이미 올바른 STOP 후보를 선택해 regret 0이므로,
현재 오차를 동률 문제로만 설명하지 않습니다. 동일 입력·다른 미래 정답의 11쌍도 보존했습니다.

05:34:23 KST: [고정 생성기 위 선택기 A/B 6회](10_frozen_stopmix_selector/README.md)의
실제 실행을 시작했습니다. 05:35 KST에는 첫 선택기의 1,540 step·6,155회 표본 노출을
확인했습니다. 앞선 05:33 실행 요청은 출력 경로 검사에서 GPU 사용 전에 거절됐고,
허용된 개인 진단 폴더로 경로만 수정했습니다. 실패 기록은 남겼으며 학습 조건·소스·계획은
바꾸지 않았습니다. 전체 6회 완료나 모델 채택을 아직 의미하지 않습니다.

05:40:53 KST: 고정 생성기 위 선택기 6회와 전체 진단을 정상 종료했습니다.
학습 기록 9,240행·36,930회 표본 노출이며 부모 생성기·캐시·원래 후보 비용이 유지됐습니다.
평균 경로 오차 개선과 정지 유지 그룹의 퇴행이 함께 나타나 모델은 미채택입니다.
모든 seed의 원본·그림을 수집하고 상세 비교를 게시하는 중입니다.

05:40:56 KST: commit `cf3bfc2`의 [추가 전체 코드 검사](07_code_verification/frozen_cf3bfc2/README.md)를
완료했습니다. **6,095 통과 / 6 건너뜀 / 0 실패**, 소스·설정 573개 전후 동일입니다.
최신 oracle·선택기·실행기 테스트를 포함하지만 이후 작성한 결과 게시·후속 연구 코드는 포함하지 않습니다.

## 결과를 찾는 곳

<!-- HH_260906 - Keep actual training, expert collection and read-only data readiness in separate discoverable categories. -->

| 폴더 | 내용 | 현재 판단 |
|---|---|---|
| [01 물리 적분과 차량 중심 촬영](01_substeps/README.md) | C-track expert 수집 4회, 실제 PNG·GIF와 원본 품질 검사 | 4회 모두 데이터 미승인 |
| [02 후보 선택 손실 비교 학습](02_cost_aware_learning/README.md) | GPU 0의 새 학습 6회, 고정 seed별 성능·경로 그림 | 미채택 |
| [03 학습 처리 비용](03_training_cost/README.md) · [실제 학습 계측](03_training_cost/actual_training_profile/README.md) | 성분 80회 계측, 실제 16-step 학습 2회와 CPU/CUDA 추적 | 진단 완료·결과 동일성 통과; 속도 개선 판정 아님 |
| [03 부록: DataLoader 고정 ABBA](03_training_cost/dataloader_abba/README.md) | 287-step 학습 4회·worker 0/2명·전체 원본 이력과 실측 그림 | 결과 동일·조건당 2회 평균 단축; 기본 설정 불변 |
| [04 주행·정지 후보 비교 학습](04_stopmix_learning/README.md) | 새 학습 6회와 별도 행동 분석 6회, 고정 시점 PNG 72장 | 결과 게시·해시 검증 완료, 미채택 |
| [05 실제 데이터의 준비 상태](05_real_data_readiness/README.md) | nuPlan 단일 DB의 native 센서·경로 메타데이터 검사 | 학습 승인 아님 |
| [05 부록: 6카메라 시간차 하한](05_real_data_readiness/subset_interval_analysis/README.md) | 동일 매칭에 대한 28개 조합의 보수적 하한 | 현재 매칭으로는 기존 20 ms 조건 미달 |
| [05 부록: 보정 메타데이터 구조](05_real_data_readiness/calibration_opcode_inventory/README.md) | 32개 BLOB의 실행 없는 opcode 구조 검사 | 완료; 원래 진단은 숫자 해석·투영 검증 아님 |
| [05 부록: 실행 없는 보정 숫자 진단](05_real_data_readiness/calibration_literal_diagnostic/README.md) | 별도 계획으로 고정 바이트 틀의 32개 값 해석, 8카메라 기본 수치 검사 | 수치 진단 완료; 투영·TF·학습 승인 아님 |
| [06 Town07 저속 정지 원인 분석](06_town07_stop_conditioning/README.md) | 원본 1,337창 재구성, 고유 실패 119구간·실제 수치 PNG | 원본·실패 보존, 원인 단정·승인 없음 |
| [07 전체 코드 검사와 이전 실패](07_code_verification/README.md) · [cf3bfc2 후속 검사](07_code_verification/frozen_cf3bfc2/README.md) | 고정한 코드 전체 재검사, 원본 로그·SHA·미실행 사유 | 후속 6,095 통과 / 6 건너뜀 / 0 실패 |
| [08 고정 10-epoch 이어학습](08_stopmix_ten_epoch_continuation/README.md) | 기존 6개 모델의 학습량만 늘리고 전체 종점·행동 비교 | 24/24단계 완료; 절대 조건 0/6 통과·미채택 |
| [09 TRAIN의 정답 후보와 선택 오차 진단](09_train_oracle_diagnosis/README.md) | 6개 체크포인트의 전체 6,882행 cost·동률·그룹·11개 동일 입력 쌍 | 진단 완료; 새 학습·원인 단정·모델 승격 없음 |
| [10 생성기 고정 선택기 A/B 학습](10_frozen_stopmix_selector/README.md) | 기존 3개 생성기 위 두 선택기를 각각 새로 학습 | 05:40 6회 완료; 전체 평균 개선과 정지 유지 퇴행, 미채택 |
