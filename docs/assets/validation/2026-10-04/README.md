# 10월 4일 — 기본 주행을 유지하면서 개선하기

<!-- HH_261004 - Separate real optimizer activity, completed evidence and future feature targets. -->

목표는 **주행 메인 9개·하위 30개 기능**과 별도 자동 지도 모델이다. 현재 정식 완료는 **메인 0/9·하위 0/30**이다. 지금은 직진·좌회전·우회전·종점 정지·속도 추종의 **5개를 우선 개선**하고 있다. 다섯 기능을 완료했다는 뜻은 아니다.

## 10월 5일 이어서 진행 — 정리 완료, 실제 V5 종료, 지연 개선 계속

<!-- HH_261005 - Route readers to the consolidated measured tradeoff and the explicit pending-resource boundary. -->

### 12:27 KST — 비교·중복 캐시 가설 확인 완료, 다음 실제 GPU 작업은 추가 공간 필요

현재 학습/GPU 평가는 끝났다. 새3개는 HOLDOUT에서 속도 오차가 줄었지만 경로 오차가 모두 증가해 기존 GOOD를 바꾸지 않는다. 실제883회 기록으로 기존 raw 영상 해시 캐시가 이미 적용된 것을 확인했고, 새 중복 캐시를 속도 개선이라 주장하거나 NN 비교에 쓰지 않았다. 승인된 `.o` 정리는 끝났으며 추가 캐시 정리는 별도 승인을 기다린다. 첫 목표 **RTE-01**, 정식 완료 **0/9·0/30**이다. [측정 수치·계속할 순서·그 다음 학습 설계](../2026-10-05/README.md)를 먼저 읽으면 된다. 아래는 당시 기록이다.

<!-- HH_261005 - Lead with the completed evaluation and its remaining quality failures, not obsolete running process claims. -->

### 12:10 KST — 실제 학습·평가 완료, 전체 품질 미달·지연 개선 계속

새3개 학습과 고정5모델 **35,315개 평가·독립 전량검산이 끝났다**. 원래497개 비교 중64개 미달이다. 검산은 전체 기록·순서·판정 계산이 정확하다는 뜻이며 주행 기능 승인이나 자동 모델 채택이 아니다. 현재 학습/GPU 평가 작업은 없다. 첫 목표 **직진 차로 유지(RTE-01)**, 정식 완료 **0/9·0/30**을 유지하며 동일 기존 모델의 입력 처리·추론 지연을 개선한다. 로컬 쓰기 가능 약4.74GB로 새 실주행은 아직 미실행이다. [현재 결론·기능 위치·다음 순서·결과 링크](../2026-10-05/README.md)를 먼저 읽으면 된다. 아래는 당시 시점의 과거 기록이다.

<!-- HH_261005 - Put the real five-endpoint evaluation and the remaining first-function gates in a readable current-state table. -->

### 11:31 KST — 학습 완료, 실제 5모델 비교 실행 중

첫 완료 목표는 **직진 차로 유지(RTE-01)**다. 현재 정식 완료는 **메인 0/9·하위 0/30**이며, 학습 횟수나 오프라인 검사 통과 수를 기능 완료로 바꾸지 않는다.

| 장소 | 현재 상태 | 이번에 하는 일 |
| --- | --- | --- |
| PRO6000 GPU0 | 학습 완료 | 새 모델 3개가 각각 6,000회·입력 24,000개 학습을 마쳤고 전량 검산도 완료했다. 현재 optimizer 학습은 없다. |
| PRO6000 GPU0 | 실제 평가 실행 중 | 11:20:23에 시작했다. 기존 2개·새 3개 최종 모델의 같은 입력 35,315개와 원래 기준을 비교한다. 첫 모델 7,063개는 끝났고 두 번째 모델로 진행 중이다. 최종 판정·모델 채택은 아직 없다. |
| 로컬 | 연결·가속 코드 준비 | 저장된 실제 JPEG·상태·TF를 비구동 Autoware 입력에 연결하고, 같은 가중치의 GPU 추론을 준비한다. 실제 ROS 동등성·GPU 속도 개선은 아직 검증하지 않았다. |
| 로컬 | 새 주행 미실행 | 쓰기 가능 약 4.76GB다. 승인된 빌드 중간 파일 정리는 끝났으며 복구 가능하다. 추가 정리 승인을 받지 않은 JPEG·컴파일 캐시는 삭제하지 않는다. |
| 자동 지도 | 새 학습 미실행 | 별도 모델을 유지한다. 현재는 첫 주행 기능의 안정화가 우선이다. |

이번 평가의 첫 모델은 실제 약 9분이었다. 같은 진행 속도라면 전체 평가 자체는 약 45분 규모지만, 후속 전량 검산과 실제 주행 시간은 별도다. 오류 로그가 비어 있다는 사실만으로 품질 통과를 주장하지 않는다. 저장된 JPEG 입력의 ROS 동등성도 원래 실시간 BGRA 입력의 동등성과 구분한다.

**다음 순서:** 전체 평가·독립 검산 → 개선과 퇴행 판독 → 같은 가중치의 출력 동등성·추론 지연 확인 → 실제 직진 → Autoware 연결·독립 장소 반복 시험이다. 기존 실제 209m의 **충돌·차체 침범 0**은 개선 근거지만, **8.62Hz·최대 명령 확인 109.60ms** 미달과 독립 장소의 반복 검증이 남아 있다. 기준을 낮추거나 실패 꼬리를 지우지 않는다.

[실제 평가 승인](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_final_evaluation_20261005_v1/ROOT_final_evaluation_execution_review_v2.json), [세 번째 학습 전량검산](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/seed_2026100503_fit_supervisor_v2/ROOT_whole_seed3_training_postcheck_v1.json)을 보존한다. 첫 준비 오류와 수정 전 원본도 유지한다. 아래는 각 시점의 과거 기록이다.

<!-- HH_261005 - Record all completed real learning and the failed local authority preparation separately from NN evaluation. -->

### 11:14 KST — 3개 학습·전량검산 완료, 실제5모델 평가 연결 준비

**완료:** PRO6000 GPU0·개인 venv에서 새3개 모델이 각각6,000회·입력24,000개 학습을 끝냈다. 세 번째 학습 계산은1,917.38초(31분57초)였고 오류 로그0이다. [세 번째 실제 종료 원본](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/seed_2026100503_fit_supervisor_v2/SSH.stdout), [독립 전량검산](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/seed_2026100503_fit_supervisor_v2/ROOT_whole_seed3_training_postcheck_v1.json)에서 전체6,000행·12개 체크포인트·최종101상태/동결47개·TRAIN 영상34,992개·원본/소스·프로세스 종료를 확인했다. 현재 실행 중인 학습은 없다. 세 모델은 같은 선행학습에서 마지막 단계만 독립 반복한 것이며 전체 선행학습의 독립3회나 설계A/B6회는 아니다.

**수정 중:** 평가 감시 코드의 첫39통과·3실패를 보존하고 수정본54개 검사는 통과했다. 이후 ROOT가 새 승인 JSON을 만들면서 원래 학습 조건의`5.0`을`5`로 저장해 해시를 바꾸는 실수를 냈다. [로컬 승인 준비 실패](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_final_evaluation_20261005_v1/ROOT_initial_authority_validation_failure_v1.json)는 실제평가 전에 거부됐다. 실패승인·고정코드를 덮지 않고 원본 자료형을 유지하는 새V2 승인/감시 연결로 진행한다. **아직 실제 평가·원격 전송·새 로컬 주행은 하지 않았다.** 이 준비 오류를 모델 주행 실패나 새 학습으로 세지 않는다.

**다음:** 기존2개+새3개 최종 모델의35,315개 기록·원래71기준 평가 → 개선/퇴행 판독 → 동일 가중치의 추론 지연 검증 → 실제 직진·독립 장소 반복·Autoware 연결이다. 로컬 추가 공간 확보의 승인 문제는 별도로 남는다. 첫 목표 **직진 차로 유지(RTE-01)**, 정식 완료 **메인0/9·하위0/30**을 유지한다. 지도 새 학습은 실행하지 않는다. 아래는 각 시점의 과거 기록이다.

<!-- HH_261005 - Keep real remote learning, source-only failures and blocked local driving distinct. -->

### 10:59 KST — 세 번째 모델 실제 학습 중, 평가 실행 오류 수정 중

| 장소 | 상태 | 확인한 내용 |
| --- | --- | --- |
| PRO6000 GPU0 | 첫 두 학습 완료 | 각각6,000회·입력24,000개. 전체 이력·원본·최종 가중치의 독립 검산 완료. |
| PRO6000 GPU0 | 세 번째 학습 중 | 최근4,400/6,000회·입력17,600개·오류 로그0. 공통 시작 가중치·새 optimizer·원래 TRAIN만 사용. |
| 로컬 소스 검사 | 첫 실패 보존·수정 중 | 평가 감독자의 실행 구문 선택 오류로 첫 검사39통과·3실패. 학습 번호·모델 바인딩·전량검산의 연결 검사도 보강. 실제 평가/원격 전송은 하지 않음. |
| 로컬 주행 | 새 실행 미실행 | 마지막 쓰기 가능약4.75GB. 승인된 빌드 중간 파일 정리는 완료·복구 가능. 추가 캐시/JPEG는 승인 없이 삭제하지 않음. |
| Autoware 연결 | 별도 코드 준비 중 | 저장된 실제6카메라 JPEG·상태·TF를 비구동 shadow 입력으로 연결하는 최소 경로 준비. 실제 ROS·동등성·전체 지연 검증은 아직 없음. |
| 자동 지도 | 새 학습 미실행 | 별도 모델 유지. 현재는 첫 주행 기능 안정화가 우선. |

다음은 **세 번째 학습 종료·전량검산 → 기존2개와 새3개 최종 모델의 전체35,315개 기록·71기준 비교 → 추론 지연 검증 → 실제 직진·독립 장소 반복 시험**이다. 첫 평가 코드 실패는 [원본 실패 기록](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/evaluation_supervisor_scalar_suite_main_v1/actual_result_exact.json)에 보존한다. 모의 검사·학습 완료·개발 자료의 오차 감소를 주행 기능 승인으로 세지 않는다. 첫 목표 **RTE-01 직진 차로 유지**, 현재 정식 완료 **메인0/9·하위0/30**이다. 아래는 각 시점의 과거 기록이다.

<!-- HH_261005 - Preserve the actual postcheck bookkeeping failure and corrected independent result before starting the third fixed-final continuation. -->

### 10:32 KST — 두 모델 학습·전량검산 완료, 세 번째 초기 검사 중

**PRO6000:** 첫 번째와 두 번째 모델은 각각6,000회·입력24,000개 학습을 종료했다. 두 번째 학습 자체는1,893.10초(31분33초)였고 오류 로그0이다. [전체 실제 종료 응답](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/seed_2026100502_fit_supervisor_v1/SSH.stdout)과 서버 개인 폴더의 전체 학습 로그·이력·모든 체크포인트를 보존한다.

두 번째 모델의 [첫 사후검산 실패](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/seed_2026100502_fit_supervisor_v1/ROOT_whole_seed2_training_postcheck.json)는 **ROOT 승인 해시의 구분 실수**다. JSON 내용의 해시를 원본 파일 마지막 줄바꿈까지 포함한 바이트 해시로 잘못 넣었다. 학습 파일이 바뀐 것은 아니며 검사 코드가 올바르게 거부했다. 기존 실패·소스·승인을 덮지 않고, 파일명만 별도V2로 바꾼 같은 검사와 올바른 원본 해시로 [전량검산을 완료](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/seed_2026100502_fit_supervisor_v1/ROOT_whole_seed2_training_postcheck_v2.json)했다. 전체6,000행·실제 입력 순서·12개 체크포인트 해시·최종101개 수치/동결47개·TRAIN 영상34,992개·소스33개/런타임31개·원본 로그와 소유 세션 종료를 확인했다. 실제 NN/평가/재학습은 이 검산에 포함하지 않았다.

[별도 세 번째 실행 검토](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/ROOT_seed3_GPU0_execution_review_v2.json) 뒤 **10:30:43에 세 번째 프로세스를 시작했다**. GPU0 전용·개인 기존venv·OMP4/MKL4/OpenBLAS1이며 원래 공통 시작 가중치와 새optimizer·고정6,000회/24,000입력을 유지한다. 현재 초기 검사 단계의stdout/stderr0만 확인했으므로 optimizer 업데이트가 진행됐다고 아직 주장하지 않는다. 앞 두 학습에 근거한 학습 자체 예상은31–33분이고 평가·검산은 별도다.

**다음:** 세 번째 종료·전량검산 → **공통3000·기존GOOD6000·세 개 최종6000 모델의 같은 입력 평가**다. 전체35,315개의 완성/미완성 기록과 원래71개 기준을 보존하며 좋은 중간 체크포인트 선택·자동 채택·TEST 튜닝은 하지 않는다. 이전 실제3모델 평가는약27분이었지만 새5모델 작업의 완료 시간은 아직 계측하지 않았다. 최종 평가가 주행 기능의 승인은 아니다.

**로컬:** 10:21 쓰기 가능약4.75GB로 새GPU·전체주행은 미실행이다. 승인된 빌드5,496개는 이미 백업·검증 후 정리했고 복구 가능하다. 추가JPEG·ccache는 답변 전에는 지우지 않는다. 특히 이전V2의전체901카메라묶음은 준비한GPU 출력 비교의 필수 입력이므로, 추가 정리 승인도 사용 중인 입력의 검증 순서를 먼저 따른다. 실제GPU 출력 동등성/가속, 현재 모델의 실제ROS 연결·독립3개 장소의 반복평가는 남는다. 지도 새 학습은 실행하지 않는다. 첫 목표 **직진 차로 유지(RTE-01)**, 정식 완료 **메인0/9·하위0/30**이다. 실제209m에서 충돌·차체침범0을 확인했지만 **8.62Hz·최대109.60ms** 미달은 해결됐다고 아직 보고하지 않는다.

아래는 각 시점의 과거 기록이다.

<!-- HH_261005 - Report whole first-fit completion and actual second-fit progress without promoting a trained endpoint to a driving capability. -->

### 09:42 KST — 1차 학습·전량검산 완료, 2차 실제 학습 중

**PRO6000: 첫 모델은 종료했고 두 번째 모델을 학습 중이다.** 08:58에 시작한 첫 학습은 **6,000회·입력24,000개·30.9분**으로 정상 종료했다. 전체6,000행과 실제 입력 순서, 12개 체크포인트의 해시, 최종101개 상태/동결47개, TRAIN 영상34,992개, 소스와 승인·원본 보존을 독립 검사했다. [실제 종료 원본](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/seed_2026100501_fit_supervisor_v1/SSH.stdout), [독립 전량검산](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/seed_2026100501_fit_supervisor_v1/ROOT_whole_seed1_training_postcheck.json)을 보존한다. 전체 stdout·stderr·학습 이력·모든 체크포인트는 서버 개인 폴더에 남는다. 검사 범위는 중간 체크포인트의 해시와 최종 가중치 수치이며, 중간12개 모두의 수치 검사나 학습률 공식의 독립 재계산까지 했다는 뜻은 아니다.

GPU0 가용성·기존 잠금·개인 venv를 다시 확인하고 **09:36에 두 번째 실제 optimizer 학습을 시작했다. 최근 확인은600/6,000회·입력2,400개·오류 로그0**이다. [두 번째 실제 실행 검토](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/ROOT_seed2_GPU0_execution_review.json), [GPU0·TRAIN 전용 승인](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/ROOT_GPU0_TRAIN_only_authorization.json)을 따른다. 같은 공통 선행 가중치에서 새 optimizer와 서로 다른 입력 순서로 마지막 공동 주행 학습을 반복한다. 첫 번째 결과를 이어 학습하거나 전체 선행학습까지 독립인 세 모델을 만든다는 뜻은 아니다. 각6,000회·입력24,000개이며 기존 직진·복귀·저속 자료를 함께 사용한다. 첫 완료 시간을 참고한 **학습 자체 예상은약31–33분/모델**, 후속 평가·검산 시간은 별도다. 세 번째 모델은 두 번째 종료·전량검산 후 별도 실행 검토를 거친다.

**로컬: 새 실제 주행과 GPU 추론은 실행 중이지 않다.** 마지막09:11 조회의 쓰기 가능 공간은약4.81GB다. 승인된 빌드 중간5,496개 백업·정리는 완료했고 복구 가능하다. 추가 JPEG/컴파일 캐시 정리는 답변 전에는 하지 않는다. 같은 CNN을 GPU에서 계속 실행하는 별도 코드를 준비·검토 중이며 **실제 엔진 성공·전체 출력 동등성·10Hz 가속은 아직 미검증**이다. 현재 준비한 가속 연결은 기존GOOD6000에 묶여 있어 새 학습 모델에 자동 적용하지 않는다. Autoware 비구동 연결 코어도 준비 단계이며 실제 ROS/TF/런치 검증은 남는다. 지도 모델의 새 학습은 실행하지 않는다.

다음 순서는 **두 번째 종료·전량검산 → 세 번째 순차 학습 → 고정 최종 모델 평가**와 로컬의 **추가 정리 승인·공간 확인 → 같은 모델의 GPU 출력/지연 검증 → 새 폐루프 직진 → 독립 경로 반복·Autoware 연결**이다. 메인9개·세부30개 중 첫 완료 목표는 **직진 차로 유지(RTE-01)**이며 현재 공식 충족은 **0/9·0/30**이다. 실제209m의 충돌·차체 침범0과 작은 차선 오차는 확인했지만 **8.62Hz·최대109.60ms** 미달이 남아 있다. 아직 독립3개 장소의 반복 평가도 완료하지 않았다. 새 head를 추가하거나 안전 기준·실패 분모를 낮춰 완료로 세지 않는다.

<!-- HH_261005 - Publish completed actual CPU admission and the remaining remote-fit/local-resource boundaries without claiming a running job. -->

### 08:46 KST — 지금 무엇을 하는가

첫 목표는 **직진 차로 유지(RTE-01)**다. 차선 오차가 작았던 실제209m 시험도 처리 주기 **8.62Hz**, 최대 명령 확인 **109.60ms** 때문에 FAIL이었다. 이 시간 기준을 만족시키고 독립 경로의 반복 주행까지 확인해야 완료로 센다. 현재는 **메인0/9·세부0/30**이다.

| 장소 | 상태 | 이번 단계 |
| --- | --- | --- |
| PRO6000 | 실제 CPU 검사 완료 | 승인된 TRAIN·서로 다른 3개 학습 순서·동일6000회/B4·공통101개 상태·소스33개를 전량 확인했다. 아직 optimizer 학습은 시작 전이다. |
| PRO6000 | 학습 실행 준비 | CPU와 별개의 GPU0 승인 파일을 만들었다. 기존 개인 venv·물리 GPU0의 직전 가용성/잠금·전체 로그/실패 보존을 검사하고 seed1부터 순차 실행한다. |
| 로컬 | 새 GPU 코드 준비 | V6 실제 실행은 소유권 검사에서 종료했다. V7은 거부한 조회도 보존하며 모의424개 검사를 통과했다. 실제 GPU 엔진·출력 동등성·가속은 아직 미검증이다. |
| 로컬 | 실제 실행 공간 부족 | 08:38의 쓰기 가능 공간은 약4.80GB다. VS Code 업데이트 파일 약801MB가 같은 시간대에 생성됐고 대화 기록도 증가했다. 감소 전체의 원인은 아직 확정하지 않는다. 승인된 빌드5496개 정리는 완료·복구 가능하며, 추가 JPEG/컴파일 캐시 정리는 승인 전에는 하지 않는다. |
| 로컬 | Autoware 준비 코어 | 현재 모델의 실제 프레임·시간·calibration을 원래 정책에 넘기는 별도 비구동 코어를 준비했다. 실제 ROS 메시지/TF 연결·런치·수치/전체 지연 검증은 아직 없다. |
| 지도 모델 | 새 학습 미실행 | 별도 모델을 유지하되 지금은 첫 주행 기능의 안정화가 우선이다. |

[실제 CPU 입장 전체 원본](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/SOURCE_transfer_and_CPU_admission/remote_CPU_admission_result.json), [원본·배치6000행·모든 소스의 독립 검산](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/SOURCE_transfer_and_CPU_admission/independent_whole_CPU_admission_postcheck.json), [별도 GPU0 학습 승인](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/ROOT_GPU0_TRAIN_only_authorization.json).

다음은 **GPU0 seed1 실제6000회 학습 → 최종 출력·원본/동결 상태 확인 → seed2/3 순차 학습**, 로컬에서는 **추가 공간 승인 → 같은 CNN의 GPU 엔진/전체 원출력·지연 검증 → 새 폐루프 직진 → Autoware 연결과 독립 반복 평가**다. 기존 같은6000회 학습은 약30분/모델이었으며 실제 진행 속도를 보고 예상을 갱신한다. 같은 선행학습에서 마지막 단계를 3번 독립 반복하는 것이지 전체 선행학습까지 독립인 모델이나 설계 A/B6회는 아니다. 학습 완료·오프라인 loss·모의 검사를 기능 완료로 세지 않고, 새 head·자동 배포·실패 삭제는 하지 않는다.

이 아래의 상태와 시간은 각 시점의 기록이다.

<!-- HH_261005 - Keep actual stationary failure, completed independent checks and the currently running CPU-only comparison distinct from feature approval. -->

**08:22 KST 현재:** 첫 목표는 **RTE-01 직진 차로 유지**, 정식 완료는 여전히 **메인 0/9·하위 0/30**이다.

| 장소 | 현재 상태 | 이번 작업과 판단 |
| --- | --- | --- |
| 로컬 | 종료·전량 검산 완료 | OpenBLAS 1/6 비교 1,776개 문맥·8,880개 원출력·60회 계측을 확인했다. 평균 32.44/32.94ms로 실제 10Hz 개선 근거는 없다. 거부 10건·종점 미검증 126건·원래 FAIL을 보존했다. |
| 로컬 | 종료·미채택 | CPU 코어 제한 비교는 속도 이득이 없었다. 전체 3,552개 문맥·120회 시간·원본을 독립 검산했고 기존 FAIL을 보존했다. |
| 로컬 | 종료·FAIL | 정지 진단 V5/V6는 최초 각속도 검사에서 거부됐다. V6의 실제 각속도 0 요청은 반환됐지만 첫 관측의 미세 각속도는 그대로였다. 바퀴 조회 0회·60회 미실행이며 정지·주행 성공으로 세지 않는다. |
| 로컬 | 실제 GPU 준비 종료·미채택 | V6는 컴파일0.48초·실제 실행 프로세스 생성까지 통과했다. 그러나 GPU의 두 프로세스 목록 불일치로 종료돼 엔진2개 모두 미검증이다. 거부된 중간 조회값이 미보존이므로 외부 작업/조회 시점 차이 중 원인은UNKNOWN이다. 전량 검산과 모든 조회를 기록하는 새 V7을 준비한다. |
| PRO6000 | CPU 입장 준비·학습 미실행 | 새 V2의 CPU/GPU 권한 파일을 분리하고 ROOT의 V1/V2 모의 검사237개를 통과했다. 정확한 개인 새 작업폴더와 기존 py312 venv에서 승인된 TRAIN만 확인하는 CPU 입장을 허가했다. 실제3개 배치순서 SHA 확인 전에는 GPU 승인·학습을 하지 않는다. 07:39의 가용 조회는 과거 관측으로만 남긴다. |

[CPU 비교의 사전 선언·ROOT 검토](../../../../storage/experiments/plans/2026-10-05/full209_blas16_controlled_counterfactual_v1/ROOT_execution_review.json), [코어 제한 비교 독립 전량 검산](../../../../storage/experiments/diagnostics/2026-10-05/full209_pcore_affinity_serial_ABBA_v1_independent_postcheck/PASS_WITH_ORIGINAL_FAIL_RETAINED.json), [V5 최초 실제 상태·전체 검산](../../../../storage/experiments/diagnostics/2026-10-05/owned_town03_stationary_wheel_rpc_v5_failure_independent_postcheck/ROOT_actual_failure_and_all_sources.json), [V6 실제 실패](../../../../storage/experiments/validation/2026-10-05/owned_town03_stationary_wheel_rpc_v6/capture_result.json).

[OpenBLAS 전량 독립 검산·과거 PID 세대 미기록 한계](../../../../storage/experiments/diagnostics/2026-10-05/full209_blas16_controlled_counterfactual_v1_independent_postcheck/PASS_WITH_ORIGINAL_FAIL_AND_PROCESS_HISTORY_GAP.json), [V6 전체 86파일·정리·불변 독립 확인](../../../../storage/experiments/diagnostics/2026-10-05/owned_town03_stationary_wheel_rpc_v6_failure_independent_postcheck/ROOT_actual_failure_and_all_sources.json). GPU 진단의 새 출력 상한은512MiB로 더 엄격히 줄이고 원래5GiB 최소 잔여·64MiB 공유·16MiB 종료 보호는 유지한다. 실제 복사량·제한·컴파일 중 감시를 새 선언으로 확인하며 기존768MiB 선언을 덮어쓰지 않는다. 임시 파일 전체의 절대 상한이나 전체 SDK ABI 검증까지 주장하지 않는다.

<!-- HH_261005 - Preserve the actual compile failure before any CUDA initialization or native engine build. -->

[GPU V5 실제 실패·컴파일 명령·소유 프로세스 기록](../../../../storage/experiments/diagnostics/2026-10-05/good6000_cnn_tensorrt_build_v5/result.json), [실제 컴파일 오류](../../../../storage/experiments/diagnostics/2026-10-05/good6000_cnn_tensorrt_build_v5/compile.stderr.log). 6장·30장 모두 미검증으로 남기고 실패를 성공 분모에서 빼지 않는다. 이는 실행 코드 오류이며 모델 주행 성능 또는 GPU 속도 판정이 아니다.

<!-- HH_261005 - Keep successful compilation distinct from the subsequent GPU ownership admission failure and private TRAIN-only CPU grant. -->

[GPU V6 실제 종료·처음/마지막 조회·원래 거부](../../../../storage/experiments/diagnostics/2026-10-05/good6000_cnn_tensorrt_build_v6/result.json), [3-seed V2 ROOT 검토·CPU만 승인](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/ROOT_source_and_CPU_admission_review.json). V6는 모델 추론까지 실행하지 않았다. 다음 V7은 두 조회의 시점 차이라는 가설을 확인할 수 있게 실제 거부 관측도 기록하고, 외부/불명 프로세스를 발견하면 즉시 거부하는 원칙을 유지한다. 새 코드·선언·권한·출력으로 실행하며 과거 실패를 덮어쓰지 않는다.

새 세 모델은 **같은 선행학습 계보에서 공동 학습 마지막 단계를 독립 반복**하는 것이며 전체 선행학습까지 독립인 세 모델 또는 설계 A/B6회가 아니다. 기존 모델당6000회 약30분을 참고하면 세 모델의 학습은 약1.5–2시간이나 준비·검산·후속 평가는 별도이고 확정 시간이 아니다. GPU0는 실행 직전에 실제 잠금과 가용성을 다시 검사한다. 새 head·TEST 학습·실패 자료 재활용·자동 배포는 하지 않는다.

다음 순서는 **CPU 환경 비교 종료·전량 검산 → 동일 CNN GPU 실행 가능성 → 전체 원출력·시간 동등성 → 새 폐루프 주행**이다. 이후 별도 현재 모델의 Autoware 연결·독립 TEST 범위·반복 평가가 남는다. 전체 주행 예약 8.03GB는 현재 쓰기 가능 약6.24GB보다 크며, 추가 JPEG 정리는 아직 승인되지 않았다. 기준 완화·실패 삭제·자동 모델 채택은 하지 않는다. 아래 시간 표시는 당시의 진행 기록이다.

<!-- HH_261005 - Separate the ended unfavorable offline comparison from a ROOT command invocation error before any stationary capture. -->

**07:14 KST 현재:** CPU 코어 비교4조건은 모두 종료했다. 평균은 A1 **229.79ms**, B1 **291.51ms**, B2 **267.25ms**, A2 **236.76ms**이며 코어 제한 후보는 **미채택**이다. 이전32.45ms 시험과 이번 시험은 OMP/MKL/OpenBLAS 환경값도 달라 직접 속도 개선으로 비교할 수 없다. 전체3,552개 출력·120회 계측을 독립 검산 중이며 초기 출력 거부와 종점 증거 부족을 없애지 않는다. [전체 종료·FAIL 원본](../../../../storage/experiments/diagnostics/2026-10-05/full209_pcore_affinity_serial_ABBA_v1/summary.json).

실제 정지 진단V4도 종료했다. **ROOT의 실행 명령 경로 오류**로 선언의`/usr/bin/python3`와 실제`/usr/bin/python3.10`가 달라 계측 프로세스 생성 전에 거부됐다. 모델 성능·바퀴 조회·맵 오류가 아니며 실제 최초 속도는 여전히UNKNOWN이다. 서버/PID·포트·69개 파일·소스27개·원본/보존SDK·정상 캐시 불변을 [독립 확인](../../../../storage/experiments/diagnostics/2026-10-05/owned_town03_stationary_wheel_rpc_v4_failure_independent_postcheck/ROOT_actual_failure_and_all_sources.json)했다. 기록과 정확한 명령 검사를 보존하고, 같은 물리 조건의 새V5를 준비한다. 실행된 바퀴 조회는0회이며 실제60회 ledger 자체가 없음을 구분한다.

같은 CNN 변환의 [155개 전체 파일·101개 원래 가중치 독립 검산](../../../../storage/experiments/plans/2026-10-05/good6000_cnn_legacy_onnx_v3/independent_complete_postcheck_v1_20261005.json)과 [두 ONNX의14개 연산 가중치 바이트·고정 입력/출력 확인](../../../../storage/experiments/plans/2026-10-05/good6000_cnn_legacy_onnx_v3/independent_exact14_initializers_v1_20261005.json)은 종료·통과했다. **GPU 실행·전체 출력 동등성·실제 주행 가속은 아직 미검증**이다. 현재 학습·주행·GPU 작업은 실행 중이 아니며 기능 승인은 **0/9·0/30**이다. 다음 실제 작업은 새 정지 진단과 GPU 변환의 실행 가능성 확인이며, 원래 주행 합격 기준은 유지한다.

아래 시간 표시는 당시 진행 기록이다.

<!-- HH_261005 - Report only completed affinity prefixes and preserve the anomalous timing instead of adopting an unmeasured acceleration. -->

**07:03 KST 현재:** CPU 비교 4조건 중 A1·B1·B2는 종료했고 마지막 A2를 실행 중이다. 먼저 끝난 A1·B1의 고정30회 평균은 각각 **229.79ms·291.51ms**로, 코어 제한의 이득이 없고 이전 CPU6 측정과도 큰 차이가 있다. 전체4조건·3,552개 출력·120회 계측의 독립 검산 전에는 원인을 단정하거나 설정을 채택하지 않는다. 현재 실제 작업은 **기록 입력의 로컬 추론 비교**이며 주행·학습이 아니다. 원래 실행 주기·지연·실패 기준과 **0/9·0/30** 완료 수는 유지한다.

현재 GOOD6000은 기존 Autoware shadow 연결 코드의 단일6카메라 입력·모델 ABI와 다르다. 현재 모델의5시점×6카메라 이력과5개 원출력을 위한 별도 연결·동등성·시간·거부 처리가 필요하다. 따라서 이전 VAD/Autoware 화면이나 기존 shadow 출력을 현재 모델의 Autoware 주행 성공으로 세지 않는다. 다음은 **전체 CPU 비교 종료·검산 → 최초 정지 상태의 실제 기록 → 같은 CNN의 GPU 실행 가능성·원출력 동등성 → 공간과 범위를 승인받은 새 폐루프 시험** 순서다. 아직 GPU 실행·새 학습·기능 승인은 없다.

아래 06:50 KST 이하는 해당 시점의 기록이다.

<!-- HH_261005 - Keep actual export success, the failed stationary admission and the running CPU comparison separate from driving functionality. -->

**06:50 KST 업데이트:** 첫 주행 목표는 여전히 **RTE-01 직진 차로 유지**, 정식 완료는 **메인 0/9·하위 0/30**이다. 로컬에서 같은 GOOD6000의 **CPU 코어 배치 비교를 실제 실행 중**이다. 전체 코어 A·빠른 코어 B·B·A의 새 프로세스 4개가 각각 원래 입력 888개와 고정 추론 30회를 처리한다. 30쌍의 교차 A/B 시험이 아니며, JPEG 복원 입력·OS 캐시·순서 효과의 한계를 남긴다. [고정 사전 선언](../../../../storage/experiments/plans/2026-10-05/full209_pcore_affinity_serial_ABBA_v1/declaration.json), [ROOT 전체 소스 검토·순수 검사 24개](../../../../storage/experiments/plans/2026-10-05/full209_pcore_affinity_serial_ABBA_v1/ROOT_execution_review.json). 현재 첫 조건을 처리 중이며 학습이나 실제 주행은 아니다.

**같은 CNN의 실제 CPU 변환은 종료·통과했다.** 원래 가중치 101개와 카메라 연산 14개, callback·캐시·입력 바이트 불변을 확인한 채 6장·30장 두 ONNX 파일을 만들었다. 실행은 약 0.96초였고 새 패키지 설치·GPU 실행은 없었다. [고정 계획](../../../../storage/experiments/plans/2026-10-05/good6000_cnn_legacy_onnx_v3/ROOT_plan.json), [실제 두 조건과 종료 결과](../../../../storage/experiments/diagnostics/2026-10-05/good6000_cnn_legacy_onnx_v3/result.json). 이는 **변환 가능성**이며 GPU 속도·전체 모델 출력 일치·10Hz·주행 기능 합격은 아직 증명하지 않는다.

정지 계측 V3은 **실제 Town03과 전체 OpenDrive 내용 확인을 통과**했고, 맵 변경 없이 시작했다. 그러나 차량 생성 후 첫 정지 속도 검사에서 종료해 바퀴 조회는 0회·선언된 60회 모두 미실행이다. 최초 속도값이 검사 전에 기록되지 않아 그 값은 UNKNOWN으로 남긴다. [원본 실제 실패](../../../../storage/experiments/validation/2026-10-05/owned_town03_stationary_wheel_rpc_v3/capture_result.json). 소유 차량 삭제·설정 복구·서버/PID·포트 정리는 확인했다. 다음 V4는 첫 실제 상태를 거부 전에 기록하는 최소 보완만 준비하며, 초기 속도를 만들어 넣거나 기준을 낮추지 않는다. PRO6000 학습·지도 새 학습은 실행 중이 아니다.

아래 06:29/06:02 KST 표시는 이전 진행 기록이다.

<!-- HH_261005 - Supersede the completed V2 setup failure with the bounded next owned-map diagnostic without claiming driving approval. -->

**06:29 KST 업데이트:** 정지 차량 V2도 종료했다. 정상 라이브러리를 실제로 로딩했고 CARLA 연결까지 진행했으므로 V1의 SIGBUS는 재현되지 않았다. 그러나 맵 검사에서 계측 전에 거부돼 **실제 비교 0회, 선언된 60회 모두 미실행**이다. 당시 맵 문자열을 기록하기 전에 거부했으므로 실제 맵은 UNKNOWN으로 남긴다. [V2 전체 62개 파일·소스 19개·자산·PID/포트 정리의 독립 확인](../../../../storage/experiments/diagnostics/2026-10-05/owned_town03_stationary_wheel_rpc_v2_failure_independent_postcheck/ROOT_actual_failure_and_all_sources.json).

다음 V3은 **새로 만든 ROOT 소유 시뮬레이터만** 사용한다. 시작 맵의 이름·월드 ID·활성 차량/보행자/센서 수만 확인한 뒤, 필요하면 배우·조회 작업자 생성 전에 Town03을 한 번 초기화하는 범위를 새 선언으로 고정한다. Town03의 실제 OpenDrive 전체 크기·SHA 확인 후에만 계측한다. 다른 시험의 시뮬레이터에 접속·맵 변경하지 않는다. 현재는 소스 검토·모의 시험 단계이며 실제 V3 실행 전이다.

현재 주행 기능의 남은 문제는 **실제 10Hz 처리와 원래 100ms 지연 기준**이다. 같은 모델의 CPU 배치·GPU CNN 실행 후보도 준비 중이며 실제 가속이나 모델 성능 개선을 아직 주장하지 않는다. PRO6000 학습은 실행 중이 아니다. 메인 **0/9·하위 0/30**이다. 쓰기 가능 공간은 06:28 KST 약 **6.50GB**로 작은 정지 계측 예약 6.24GB는 가능하지만 전체 주행 예약 8.03GB는 부족하다. 추가 JPEG 정리는 아직 승인받지 않았다.

아래 06:02 KST의 V2 준비 표시는 당시 기록이다.

<!-- HH_261005 - Put the latest actual fault and resource boundary before historical preparation and test evidence. -->

**06:02 KST 현재 요약:** 첫 목표는 RTE-01 직진 차로 유지이며 완료 수는 여전히 **0/9·0/30**이다. 실제 V5 직진은 차선·충돌 검사에 문제가 없었지만 **8.62Hz·최대 109.60ms**로 종합 FAIL이었다. 뒤이은 정지 차량 RPC V1은 **비교 측정 전에 SIGBUS로 종료**됐고, 소유 서버·포트 정리는 정상이다. 실제 자식의 전용 캐시 라이브러리가 필요한153,958,688B 대신 정확히16,777,216B에서 잘렸고, 원본 라이브러리의 첫16MiB와 해시가 같다. ELF의 일부 적재 구간도 이 파일 끝 뒤에 있어 **잘린 라이브러리의 로딩 오류가 유력**하다. 당시 fatal instruction은 기록되지 않아 정확한 고장 명령까지 확정하지 않는다. [V1 전체45개 파일·소스13개·자산5개·60개 미실행 분모의 독립 확인](../../../../storage/experiments/diagnostics/2026-10-05/owned_town03_stationary_wheel_rpc_v1_failure_independent_postcheck/ROOT_actual_failure_and_all_sources.json).

**서버 연결 없는 실제 CARLA 로딩 진단은 종료·통과했다.** 기존16MiB 파일 제한을 유지하고 검증한 정상 라이브러리를 새 전용 캐시에 준비한 뒤 원래 CARLA bootstrap 전체를 로딩했다. before/after 로그·실제 syscall 추적·정상 종료·최종 자산 검사를 통과했고 native endpoint 호출도 없었다. [사전 선언](../../../../storage/experiments/plans/2026-10-05/carla_import_only_v1/ROOT_plan.json), [실제 결과](../../../../storage/experiments/diagnostics/2026-10-05/carla_import_only_v1/result.json). 이는 로딩 문제의 개선이며 실제 주행·10Hz·기능 합격이 아니다. 전용 캐시의 접근 권한 경고도 보존하고 후속 사용 전에 검토한다.

지금은 이 정상 캐시를 사용하는 **새 정지 차량 A/B V2**와 **같은 CNN의 GPU 실행 후보**를 준비하고 있다. 실제V2·GPU 후보는 아직 실행하지 않았고, PRO6000 학습도 실행 중이 아니다. 현재 로컬 PyTorch는 CPU 전용이다. 새 설치·모델 교체 없이 가능한 GPU 경로를 먼저 검토하며, 실제 출력·캐시·시간 비교 전에는 채택하지 않는다. 쓰기 가능 공간과 전체 주행 예약은 매 실행 직전에 다시 확인한다. 추가 JPEG 정리 질문에는 아직 답변이 없어 해당 파일을 제거하지 않았다.

<!-- HH_261005 - Record the new exact cleanup/TEST preparation authority without rewriting frozen trials or claiming a running training job. -->

사용자가 오래된 `autonomy/build/**/CMakeFiles/**/*.o` 약1.29GB의 **서버 개인 폴더 백업·전량 검증 후 정리**와 독립 TEST-only 범위 준비를 승인했다. 04:00 KST에 승인된5,496개만 정리를 완료했고, 뒤이어 모든 대상의 부재·삭제 전후 영수증·설치 참조·소스 불변을 독립 확인했다. 복구 원본은 PRO6000 개인 폴더의 `portable_e2e/archives/autonomy_build_objects_20261005_v2`에 남아 있다. 소스·설치 파일·canonical rebuild·모델·데이터·성공/실패 기록은 제거하지 않았다. [삭제 결과](../../../../storage/experiments/maintenance/2026-10-05/autonomy_build_objects_v2/removal_result.json), [독립 확인과 복구 위치](../../../../storage/experiments/maintenance/2026-10-05/autonomy_build_objects_v2/independent_local_postcheck.json). 첫 기능 목표는 여전히 **RTE-01 직진 차로 유지**다.

<!-- HH_261005 - Report actual V5 failure and measured behavior separately from source tests, offline savings and formal feature approval. -->

**04:12 KST 실제 V5 종료:** 같은 GOOD6000으로 실제209m 경로를 따라 목표점에 도달했고, 학습 모델의 실제 주행 효과는206.39m였다. 차량 차체 중심 오차는p95 **3.67cm**, 최대 **5.00cm**, 전체 차체1,836개·상태 간 sweep1,835개의10cm 차선 inset 침범과 충돌은0이었다. 목적지 도달 후 정리 브레이크이며 **학습한 정차 기능의 성공은 아니다**.

그러나 원 판정은 **FAIL**이다. 영상 첫 콜백→계획 최대108.39ms, →ACK 최대109.60ms, 실제 계획8.62Hz로100ms 및10Hz health를 만족하지 못했다. 새CPU6·masked-copy·native 전체 파일 검사는 실제 호출·원본 보호를 통과했지만 **실제 처리 속도가 개선됐다는 증거는 없었다**. V4/V5의883개 추론과918개 저장 구간을 빠짐없이 비교하며 전처리·바퀴 조향각 계측 대기를 다음 개선 대상으로 분리한다. 이전 실패와 원본 기준을 그대로 남긴다.

[실제 전체91.8초 GIF·차량 중심 PNG5장](../2026-10-05/portable_e2e_learning_cycle_v1/01_town03_full_route209_v5/README.md), [실제 원 판정](../../../../storage/experiments/validation/2026-10-05/temporal_route_joint_town03_known_dev_straight_full209_v5/diagnostic_report.json), [7,507개 원본·JPEG5,508장·미디어6개 전량 검산](../../../../storage/experiments/diagnostics/2026-10-05/town03_full209_v5_publication_postcheck/ROOT_actual_whole_run_and_media.json). root가 최종PNG도 직접 확인했다. 이 화면은 **기록 기반 CARLA 진단이며 실제 Autoware/RViz UI 캡처가 아니다**. 준비·주행·정리는152.04초였고, 현재 CARLA·PRO6000 학습은 종료/미실행이다.

로컬 저장 폴더 전체 검사 후보 비교는 종료했다. 이전 V4 원본7,486개를 전후 재해시했고, 실제 episode의 파일5,533개·전체 entry5,540개를30쌍 모두 빠짐없이 검사했다. 논리 용량584,053,635B와 모든 파일/디렉터리 수가 같았다. 평균 검사 시간은 **8.947→5.343ms**, 약3.603ms 절감이다. 고정 전체 폴더의 warm-cache 비교이며 실제 센서→명령 지연 개선은 아직 증명하지 않았다. 처음V1의 출력 상위 폴더 누락 실패도 보존했다. [새V2 사전 선언](../../../../storage/experiments/plans/2026-10-05/full209_native_quota_v2/declaration.json), [모든60회 결과](../../../../storage/experiments/diagnostics/2026-10-05/full209_native_quota_v2/summary.json), [독립 전량 검산](../../../../storage/experiments/diagnostics/2026-10-05/full209_native_quota_v2/ROOT_complete_independent_postcheck.json).

독립 TEST 준비는 **메타데이터 확인까지 종료**했다. Town04·Town10HD_Opt·Town15가 GOOD6000의 선언된 TRAIN 계보에 없고, 기존 예약 guard가 바뀌지 않았음을 확인했다. 별도V2에서는 고정한1,069개 문서의 현재 모델 결합과 실제manifest 장소가Town01·Town03·Town07임을 확인했다. 기존 파싱 오류1개는UNKNOWN으로 남긴다. 이것은 범위가 명시된 과거 메타데이터 검사이며, 미기록 과거의 전역 부재나 실제 renderer/6카메라 품질의 증명은 아니다. 세 후보를 독립 TEST로 자동 승인하지 않았고 TEST 영상·상태·정답·NN·loss는 열지 않았다. [V1 제한된 권한](../../../../storage/experiments/plans/2026-10-05/rte01_test_only_metadata_qualification_v1/README.md), [V2 노출 이력·실패 보존·남은 자격 검증](../../../../storage/experiments/plans/2026-10-05/rte01_good6000_exposure_metadata_v2/README.md). root의52개 순수 시험과 저장 없는 메타데이터 재검산도 통과했다.

<!-- HH_261005 - Reject a measured no-gain preprocessing candidate and distinguish synthetic correctness from actual driving progress. -->
**04:49 KST 새 전처리 비교 종료·미채택:** 6카메라를 한꺼번에 정규화하는 후보를 로컬CPU6·interop1에서 합성BGRA 3종류×30쌍으로 비교했다. 최초 호출·느린 호출까지180회와90쌍을 모두 보존했고, 전체6개 입력 tensor의 바이트·형상·dtype·SHA가 같았다. 그러나 평균은 기존 **10.025ms → 후보10.066ms**, 중앙값은9.811→9.983ms로 실질적인 절감이 없었다. 실제 주행에 활성화하지 않는다. 이것은 **합성 입력의 전처리 비교**이며 실제 CARLA 원본·NN·모델 가중치·학습·센서 지연·기능 승인을 증명하지 않는다. [사전 선언](../../../../storage/experiments/plans/2026-10-05/temporal_batched_tensorization_synthetic_v1/declaration.json), [고정180회 비교 결과](../../../../storage/experiments/diagnostics/2026-10-05/temporal_batched_tensorization_synthetic_v1/summary.json).

V4/V5 실제883개 추론·918개 저장·1,836개 차량 상태의 시간 구간 비교도 종료했다. 저장 폴더 검사는 평균7.563→5.027ms로 줄었지만, 실제 전체 추론 주기는113.061→115.999ms로 늘었다. 두 앞바퀴 각도를 순서대로 기다리는 원래 차량 상태 조회를 분리해 다음 후보로 다룬다. 별개 실제 시험 사이의 시간 차이를 엄밀한 A/B 인과 효과로 단정하지 않는다. [전체 시간 구간 비교](../../../../storage/experiments/diagnostics/2026-10-05/full209_actual_v4_v5_critical_path_readonly_v1/summary.json), [ROOT 전체 재계산](../../../../storage/experiments/diagnostics/2026-10-05/full209_actual_v4_v5_critical_path_readonly_v1/ROOT_whole_scalar_recomputation.json).

**현재 공간과 실행 상태:** 쓰기 가능 공간은04:51 KST 약 **7.10GB**, 다음 전체209m 시험 예약은 약 **8.03GB**다. 디스크의 전체 미사용 공간과 일반 사용자가 쓸 수 있는 공간은 다르며 예약 설정을 바꾸지 않는다. 이전V1·V2·V4의JPEG 폴더만 약1.47GB를 서버 개인 폴더에 전량 백업·해시 검증한 뒤 로컬 사본을 정리하는 추가 승인을 질문했다. 아직 승인 답변이 없어 JPEG는 삭제하지 않았다. 로컬GIF·PNG·판정·실패·상태·소스·모델·신호등 복구 증거는 그 질문의 삭제 범위가 아니다.

다음은 **바퀴 조회 후보의 소유권·실패·정리 순수 검사 → 추가 승인과 예약 공간 확인 → 새 선언의 실제RPC 계측·전체 주행 → 독립 TEST 자격·3-seed·현재 모델 Autoware adapter·P1–P4**다. 새 바퀴 후보는 준비 단계이며 실제 효과는 미검증이다. CARLA 시험·PRO6000 주행 학습·지도 새 학습은 현재 실행 중이 아니다. 정식 기능 승인은0/9·0/30 그대로이고, 기존100ms·차선/차체·실패 분모 기준을 유지한다. 지도 모델은 별도 개발하되 지금은 첫 주행 기능의 안정화가 우선이다.

<!-- HH_261005 - Link the independent complete scalar postcheck without claiming an additional tensor-byte rerun. -->
위 전처리 비교의 [독립 전체 검산](../../../../storage/experiments/diagnostics/2026-10-05/temporal_batched_tensorization_synthetic_v1_independent_postcheck/ROOT_all180_scalar_rows_and_sources.json)도 종료했다.180회·90쌍·소스37개·출력4개·tensor기록1,080개를 모두 확인했고 후보는90쌍 중74쌍에서 느렸다. 독립 검산은 원래의 바이트 비교 판정과 전체SHA/시간 기록을 확인한 것이며, tensor를 새로 생성해 속도 비교를 반복한 것은 아니다.

<!-- HH_261005 - Keep reviewed wheel source and fake-process lifecycle evidence separate from real RPC speed and driving approval. -->
**05:02 KST 바퀴 조회 후보 준비 검사 종료:** 두 앞바퀴 읽기 작업을 별도 소유 프로세스로 나누는 코드의 모의 검사31개를 통과했고 ROOT도31개를 다시 통과했다. 프로세스 시작 전환의 최초7개 실패와 당시 소스를 보존했다. 새 성공·오류·지연·시작 실패 사례는 실제로 만든 모의 자식 PID 세대의 종료를 확인했다. 실제CARLA·NN 연결이나 지연 이득은 아직 확인하지 않았으며, 실제 접속 허가 파일도 만들지 않았다. [동결 준비 기록](../../../../storage/experiments/plans/2026-10-05/owned_parallel_wheel_reads_v1/source_prepared_review.json), [ROOT 소스 검토·모의 검사 재확인](../../../../storage/experiments/plans/2026-10-05/owned_parallel_wheel_reads_v1/ROOT_source_review_only.json). 기존5GiB 여유 공간 기준을 유지하면서 작은 별도 소유 정지차량RPC 진단이 가능한지 검토 중이다. 이 진단은 전체 주행이나 기능 완료를 대신하지 않는다.

<!-- HH_261005 - Supersede source readiness with an independent pre-activation asset-admission blocker without rewriting the frozen V1 review. -->
**05:10 KST 독립 검토의 추가 발견:** 위V1의 실제CARLA 라이브러리 경로·SHA가 호출자가 준 값끼리만 확인되고, 시험 선언과ROOT 허가의 고정asset까지 연결되지 않은 누락을 발견했다. 실제 접속이나 허가 파일 생성 전이며 **V1은 실제 활성화 금지**다.31개 모의 검사 통과를 실제 접속 승인으로 바꾸지 않는다. 동결V1과 검토 기록을 보존하고, 별도V2에서 선언·허가·실제/보존 라이브러리 바이트를 모두 연결하는 검사를 보완 중이다. 이 수정·독립 검토 후에도 실제RPC 속도 계측과 전체 주행 검증은 남는다.

<!-- HH_261005 - Separate the new V2 asset-binding source checks from the pending owned stationary actual comparison. -->
**05:29 KST 새 V2 준비 검사 종료:** 같은 실제 V5의 CARLA 라이브러리를 선언·ROOT 허가·현재 파일·보존본에 연결하도록 보완했다. 모의 검사 42개를 통과했고 ROOT도 42개를 다시 통과했다. 기존 V1과 실패 기록은 그대로 보존한다. [새 준비 기록](../../../../storage/experiments/plans/2026-10-05/owned_parallel_wheel_reads_v2/source_prepared_review.json), [ROOT 소스 검토](../../../../storage/experiments/plans/2026-10-05/owned_parallel_wheel_reads_v2/ROOT_source_review_only.json).

다음 작은 진단은 같은 Prius 차량을 정지시킨 뒤, 같은 프레임에서 순차 A·병렬 B 조회를 **30회씩** 비교하는 로컬 CARLA 시험이다. 현재는 실행·정리 코드와 실패 보존 검사를 준비 중이며, **아직 실제 실행하지 않았다**. 기존 5GiB 여유 기준에 시도 예약 768MiB와 공유 여유 64MiB를 더한 **6.24GB**가 필요하다. 전체 209m 주행 예약 **8.03GB는 그대로**다. 이 진단이 통과해도 움직이는 차량의 지연 개선이나 기능 완료를 뜻하지 않는다.

## 앞선 10월 4일 상태 — V4 주행 종료, 처리 속도 개선 중

<!-- HH_261004 - Put the current actual outcome and immediate next action ahead of the historical experiment log. -->

**최신 V4는 실제209m 경로의 목표 도달·충돌/침범0, 전체 영상 저장과 신호등 복구를 확인하고 종료했다.** 차량 차체 중심 오차 p95는3.65cm·최대5.00cm다. 하지만 영상 첫 수신→ACK 최대108.27ms와 실제 계획8.84Hz가 남아 원본 종합 판정은 FAIL이다. 현재 `RTE-01` 직진 차로 유지의 **처리 속도**를 개선한다. 기능 완성이나 모델 가중치 개선으로 세지 않으며, 우선순위5개와 전체9개·30개 승인 수는 늘리지 않았다. [읽기 쉬운 최신 결과·남은 원인·다음 순서](portable_e2e_learning_cycle_v1/14_town03_full_route209_v4/OUTCOME.md), [전체91.8초 실제 기록 기반 GIF·PNG5장](portable_e2e_learning_cycle_v1/14_town03_full_route209_v4/README.md).

<!-- HH_261004 - Put complete V4 outcomes and the completed offline comparator before earlier failures without changing formal counts. -->
실제 상태1,836개·카메라918묶음·NN883회·ACK1,840개를 보존했다. 실제38개 신호등 원복과 원본/불변 복사/인코딩 후 해시 검증이 모두 통과했다. 원본7,486개·JPEG5,508장·미디어6개·GIF185장/91.8초를 전량 확인했다. 정식 기능 승인은 아직0/9·0/30이다. 이 화면은 기록 기반 CARLA 진단이며 실제 Autoware/RViz UI 캡처가 아니다.

영상 메모리 복사 절감의 **로컬 CPU 비교는 종료**했다. 이전 V2의23개 준비 추론+865개 실제 모델 추론 시점에서 모든 출력·캐시가 바이트까지 같았다. 고정30쌍 평균은36.271→35.194ms로 약1.076ms 줄었다. 전체 원본7,366개·소스143개·가중치/메타데이터7개와 모든 시점·시간을 별도로 검산했다. JPEG 복원 입력의 비교이며 실제 센서 지연 개선 증거는 아니다. [전체 수치·한계·재현 근거](../../../../storage/experiments/diagnostics/2026-10-04/full209_masked_copy_JPEG_counterfactual_v1/README.md).

앞선 CPU4/6/8 V1은 **추론 전 종료·모델 추론0회**였다. 형상 튜플의 JSON 배열 표현 차이를 확인했고 원래 선언·실패를 보존했다. [원래 실패·정확한 원인](../../../../storage/experiments/plans/2026-10-04/full209_thread468_JPEG_counterfactual_v1/ROOT_precall_failure.json).

**17:47 KST 별도V2 비교 종료·전량 검산 완료:** CPU4·6·8 각888개, 총2,664개와90개 시간 기록을 모두 확인했다. 원출력과 비교 가능한 경로가 모두 같았고 평균 추론은35.011→32.448→32.029ms로 약3ms 개선됐다. 준비 추론 처음5개씩은 원래 후진 상태로GO검사를 거부했으며, 종점63개씩은 전체 엔진 미검증이다. 원래 종료 코드2·15개 거부·실패 분모를 그대로 남긴다. 소스147개·원본7,366개·가중치/메타데이터7개도 전량 재해시했다. [세 설정의 전체 결과·통과/거부 구분·재현](../../../../storage/experiments/diagnostics/2026-10-04/full209_thread468_JPEG_counterfactual_v2/README.md).

실제 주기에서 평균 약7.8ms를 줄여야9.5Hz에 도달하므로 위NN시간만으로 실제 처리율을 보장하지 않는다. **10월4일 시점 CPU비교·CARLA·PRO6000학습·지도 새 학습은 모두 종료/미실행이었다.** 당시 쓰기 가능약7.67GB < 다음예약8.03GB로, 재생성 가능한 빌드중간 `.o` 약1.29GB 정리와 독립 TEST 범위 답변을 기다렸다. **10월5일 새 승인은 위 최신 상태에 반영했다.** [정식 첫 기능까지 남은 구체적인 단계·현재 모델 계보](../../../../storage/experiments/plans/2026-10-04/rte01_independent_test_metadata_proposal_v1/README.md). 기존 실패와 기능 승인0/9·0/30은 그대로다.

### 아래는 앞선 실행의 결과입니다

<!-- HH_261004 - Label previous runtime measurements as historical rather than competing current-state claims. -->

종점·ACK·녹화·거리의 **새 검사기 원본 검산은 완료·통과**했다. 기존 전체7,461파일·상태1,836개·영상918묶음·ACK1,840개를 보존했고 원래 지연 FAIL은 그대로다. [새 검산·원래 실패 구분](../../../../storage/experiments/diagnostics/2026-10-04/full209_v1_bindings_readonly_v3/README.md).

영상 처리 후보는 전체5,508장의 결과가 같았지만 중앙값11.557→11.679ms로 빨라지지 않아 **미채택**이다. [전체 입력 비교·시간 측정](../../../../storage/experiments/diagnostics/2026-10-04/full209_JPEG_normalization_counterfactual_v1/README.md). 지연을 넘은883회 중1회는 종점 계산에 집중됐다. [전체 지연 분석](../../../../storage/experiments/diagnostics/2026-10-04/full209_actual_callback_critical_path_readonly_v1/README.md).

**새 full209 V2는 종료·전체 화면 검증 완료, 원본 FAIL**이다. 실제 상태1,801개·6카메라901묶음·ACK1,805개에서 원래 목표점까지 도달했고 충돌/침범0이었다. 수신→ACK 최대105.11ms·실제 계획8.83Hz로 미달이며 주행 뒤 신호등 원복 검사도 실패했다. 차량 차체 중심 오차 p95는7.18cm로 이전3.65cm보다 커졌으므로 모델 성능 개선을 주장하지 않는다. 후륜 base_link 기준6.93cm와 차체 중심을 혼용하지 않는다. 전체 원본7,366개·JPEG5,406장·GIF181장/90.10초·PNG5장을 검증했고 실패 구간을 자르지 않았다. [실제 V2 전체 화면·원본 오류](portable_e2e_learning_cycle_v1/12_town03_full_route209_v2/README.md), [전량 게시 검증](../../../../storage/experiments/diagnostics/2026-10-04/town03_full209_v2_publication_postcheck/ROOT_actual_whole_run_and_media.json).

**JPEG 병렬 후보의 오프라인 전량 비교는 종료됐다.** 901묶음·5,406장 결과가 같고 고정30쌍 실제 저장 호출의 평균은7.238→4.765ms였다. 첫 작업자 시작 비용도 포함하며 최대는7.543→8.114ms로 새 후보가 더 컸다. JPEG에서 다시 만든 입력이므로 실제 센서 BGRA 동일성이나10Hz 개선을 증명하지 않는다. [전량 비교·수치·한계](../../../../storage/experiments/diagnostics/2026-10-04/full209_parallel_JPEG_counterfactual_v1/README.md).

**새 V3는 실제 실행·종료했으나 모델 주행 전 실패**했다. 상태1개·카메라1묶음·NN/학습 모델 명령0·JPEG0이다. 저장 입력 검사가 여섯 카메라를 모두 거부했고, 원복 코드가 실제 신호등38개를43개로 잘못 요구한 오류도 확인했다. 전체1,958파일·게시 소스14개를 보존·재검산했으며 실제 픽셀이 저장되지 않아 PNG/GIF는 만들지 않았다. [V3 전체 실패·다음 수정·없는 화면의 구분](portable_e2e_learning_cycle_v1/13_town03_full_route209_v3/README.md).

당시의 다음 단계였던 V4는 위 최신 결과처럼 실제 실행·종료했다. 기존/신규 신호등 CPU85개·실제 실행 연결116개·저장/게시140개 검사는 통과했고, 모델·제어·속도·100ms·10cm 기준을 유지했다. 이전 V2의 JPEG 저장은 평균7.95ms이며 종점의 실제 메모리 정리21.68ms도 관측됐지만, 그것을 새 V4의 최악 지연 원인으로 옮겨 단정하지 않는다. V4에서는 일반 구간의 계획 생성 뒤 약29ms 추가 지연과 지속적인 전체 저장 폴더 검사 비용이 남아 별도로 분석한다.

## 지금 진행 중인 작업과 오늘 끝난 작업

| 장소 | 상태 | 작업 | 증명하려는 것 |
| --- | --- | --- | --- |
| 로컬 CPU | **고정 비교 종료·출력 동일·학습 아님** | 이전 V2의888개 실제 추론 시점을 JPEG에서 복원해 기존/복사 절감 후보 비교 | 출력·캐시 전량 동일, 평균36.271→35.194ms; 실제 주행 지연은 별도 검증 |
| 로컬 CPU | **CPU4/6/8 새V2 종료·전량 검산·학습 아님** | 각888개·총2,664개·시간90개·원래 기준불변 | 평균35.011→32.029ms; 준비추론거부15개/종점미검증189개 보존·실주행개선은별도 |
| 로컬 CARLA | **최신 V5 주행·전체 화면 검증 완료, 지연 FAIL** | 같은 GOOD6000로 원래 209m 경로 | 차체 중심 p95 3.67cm·충돌/침범 0·저장/정리 정상; 최대 109.60ms·8.62Hz로 미달 |
| PRO6000 개인 venv·GPU0 | **두 학습·최종 검증 완료, 후보 미채택** | 동일 GOOD6000에서 대조군·기존 주행 보존 후보 각 1,000회 | 직진 오차 퇴행이 남아 새 가중치를 주행에 넣지 않음 |
| PRO6000 개인 venv·GPU0 | **TRAIN 손실 방향 분석 완료·종료** | 고정 160개·1,152회 gradient 계산·가중치 업데이트 0회 | 일부 직진/회전 방향 충돌과 수정 힘 차이를 계측; 모델 상태·41,256 JPEG 전후 동일 |
| PRO6000 개인 venv·GPU0 | **경로 분기 학습·검산 완료, 후보 미채택** | 기존 보정 4개 텐서·1,409개 숫자만 고정 1,000회·5,000개 노출 | 좌회전 평균 오차 30.36% 개선, 저속 직진 14.01% 악화; 사전 기준 9/18로 미채택·46개 결과 전량 미러 검증 |
| 로컬 CARLA | **v3 전체 기록·예비 기준 통과·종료** | 같은 GOOD6000 설정으로 Town03 직진 45초 | 모델 제어 약 96.37m·차량 CTE p95 2.63cm·충돌/침범 0; 독립 기능 승인 아님 |
| 로컬 CARLA | **callback45 v3 전체 기록·게시 완료, 지연 FAIL** | 실제 카메라 수신→계획→ACK 100ms 검사 | 영상 연결·7,200개 처리 시각 통과; 수신→ACK 최대107.16ms·10/415회 초과로 미달 |
| 로컬 코드 | **callback45 v3 CPU·실제 전량 재검증 완료** | 별도 revision·모델/제어/안전 기준 불변 | 수집 CPU97개·게시105개 통과; 실제 원본4,642개·JPEG2,700장·미디어6개 확인 |
| 로컬 CPU | **종점 수치 수정·전체 비교 완료** | 같은415개 실제 원출력 ×4개 수치 종점 조건 | 1,239/1,660→1,660/1,660; 기존421개 실패 보존, 수치 검사와 실제 주행 구분 |
| 로컬 CARLA | **전체 경로 종료·목표 도달, 종합 FAIL** | 같은 GOOD6000로 원래209.37m 경로, 실제91.8초 | 충돌/침범0·차체 중심 p95 3.65cm; 수신→ACK 최대113.31ms·처리율9.00Hz와 검사기 오류가 남음 |
| 로컬 CPU | **용량 순회 비교·실주행 계측 완료** | 매 호출 전체 파일을 검사하는 scandir 후보 | 고정 폴더 median21.55→4.53ms; 실제918회 median6.80ms·전량PASS, 전체 센서 지연은 아직 미달 |
| 지도 모델 | **지난 학습·평가 완료, 새 학습 미실행** | 이전 구조 비교 결과를 유지 | 23/27 통과·recall 4개 미달로 자동 교체하지 않음 |

03:10 KST 확인: 양쪽 실제 **1,000/1,000회·각 5,000개 입력 노출**을 완료했다. 학습 합계 **22.31분**이며 종료 세대·최종 요약·소스와 데이터 전후 동일성을 검사했다. 기존 저속 직진 TRAIN 평균 위치 오차는 **시작 0.039 → 대조군 0.100 → 보존 후보 0.095m**였다. 보존 후보는 대조군보다 약 5.4% 낫지만 시작 모델보다 143.5% 나빠 **채택하지 않았다**. [전체 비교 결과·학습 근거](portable_e2e_learning_cycle_v1/01_retention_learning_pair/README.md)를 참고한다. 학습은 현재 실행 중이 아니다.

로컬 첫 시험은 **45초·900개 상태·450개 6카메라**를 완전히 기록했다. 교사 출발 뒤 실제 NN 효과 829개를 관측했고 약 96m를 주행했다. 하지만 저장 ACK 순서 증빙 누락과 출발 지점 연결 차로의 검사 범위 누락으로 원래 판정은 FAIL이다. [읽기 쉬운 결과·원인](portable_e2e_learning_cycle_v1/02_town03_known_dev_straight/OUTCOME.md), [실제 PNG·GIF](portable_e2e_learning_cycle_v1/02_town03_known_dev_straight/README.md)를 보존했다. 화면은 차량 중심 기록 기반 CARLA 진단이며 Autoware/RViz 실제 캡처가 아니다.

03:28 KST에 수정 v2를 실행했지만 **36.52초 후 주행 전에 종료**됐다. 새 선언과 기존 fixture의 소유권 식별값 연결이 빠져 보호 검사가 거부했다. 0개 상태·카메라·ACK이므로 모델 행동을 평가하지 않는다. [v2 원인·원본 로그](portable_e2e_learning_cycle_v1/03_v2_startup_failure/README.md)에 정리했다. 실제 보호 검사를 없애거나 동결 v2를 덮어쓰지 않고 별도 v3로 보완한다.

**03:47 KST 최종 확인: 새 v3는 전체 45초·900개 상태·450개 6카메라·904개 ACK의 예비 기준을 통과했다.** 교사 출발 뒤 실제 NN 효과 829개에서 약 **96.37m**, 차량 기준 중심 오차 p95 **2.63cm**, 최대 **3.69cm**, 충돌·차선 침범 0이었다. 측정한 영상 전달→ACK 최대 **78.46ms**, 차체·상태 사이 sweep의 10cm inset 검사 모두 침범 0이다. ACK 뒤 이미지 저장 450묶음·JPEG 2,700장과 fixture 원복도 확인했다. 가중치나 안전 기준을 바꾼 성능 향상이 아니라, **기존 모델을 정상 계측하도록 시험 연결과 증빙을 보완한 결과**다. v1 FAIL·v2 시작 FAIL은 남긴다. 약 8.64km/h이며 정차·30km/h·독립 평가·전체 센서→명령 지연은 미검증이다.

[읽기 쉬운 v3 결과·다음 순서](portable_e2e_learning_cycle_v1/04_town03_known_dev_straight_v3/OUTCOME.md), [실제 전체 45초 GIF·차량 중심 PNG 5장](portable_e2e_learning_cycle_v1/04_town03_known_dev_straight_v3/README.md)을 게시했다. 게시 뒤 원본 **4,632개 파일**·모든 게시 입력/소스·미디어 6개를 전량 재해시해 변화 0을 확인했다. [전체 검증 영수증](../../../../storage/experiments/diagnostics/2026-10-04/town03_straight_v3_publication_postcheck/ROOT_actual_whole_run_and_media.json). 준비·주행·정리 90.19초이며 현재 로컬 시험도 종료됐다. 정식 완료 수는 0/9·0/30 그대로다.

## 왜 학습 방법을 바꾸는가

직전 좌우회전 균형 학습은 **20,000회·100,000개 노출까지 끝났다. 계속 학습 중인 것이 아니다.** 같은 기존 74개 검사에서 이전 좌회전 모델의 60개 통과가 59개로 줄었고, 우회전 TRAIN 검사 3개를 더한 전체는 62/77이다. 기존 실패 14개도 해결되지 않았다. 회전 TRAIN 오차는 감소했지만 전체적으로 채택할 결과는 아니었다. [완료 당시 원본 요약](../../../../storage/experiments/diagnostics/2026-10-04/prior_balanced20k_audit_v1/original_summary.json)을 서버 원본과 같은 SHA로 보존했다.

따라서 단순히 학습 횟수나 head 수를 늘리지 않는다. 이번에는 **같은 학습 순서·데이터·초기 가중치·학습률**을 두 모델에 적용하고, 한쪽에만 이전 모델의 정상 TRAIN 예측을 지나치게 바꾸지 않도록 하는 벌점을 추가한다. 이전 데이터도 함께 복습한다. 이전 모델의 잘못된 예측까지 보존할 위험이 있으므로, 보존 벌점뿐 아니라 **실제 정답 위치·속도 오차**도 함께 비교한다.

이번 1,000회 비교는 비용이 큰 후속 학습을 하기 전에 방법의 영향을 보는 실험이다. 학습 데이터의 160개 고정 점검 시점만 쓰므로 **독립 평가·기능 완성·모델 채택 근거가 아니다.** 새 head는 추가하지 않는다.

## 기능을 하나씩 늘리는 방법

가능하다. 다만 기능이 서로 독립된 블록처럼 완전히 분리되는 것은 아니다.

1. **기본 주행:** 직진·속도 유지가 되도록 경로 곡률과 속도를 학습하고 실제 차량 상태로 검사한다.
2. **회전·정차:** 같은 경로·속도 출력으로 학습할 수 있다. 새 데이터와 손실을 추가하되 직진 데이터를 복습하고 기존 시험도 다시 수행한다.
3. **신호·ACC·회피:** 신호 상태·선행차·장애물처럼 새로운 정보가 필요하면 인식 출력 head나 보조 모델을 추가한다. 그 결과가 실제 주행 판단에 연결되어야 기능이 된다.
4. **차선변경·합류·주차:** 필요한 센서 범위·상황 데이터·안전 조건을 별도로 만들고, 기존 모든 승인 기능의 회귀 시험과 함께 평가한다.

현재 자체 모델은 카메라 시계열·차량 상태·내비게이션 경로를 받아 CNN 영상 특징과 attention으로 정보를 합친다. 곡률·선호 속도·의도·정지 거리·가속도 출력이 코드에 있지만 **출력이 존재한다고 그 기능이 검증된 것은 아니다.** 지금 실제 주행 시험에서 확인하는 주된 출력은 곡률과 속도다. 지도 모델은 별도로 개발하며, 주행이 자동 생성 HD 지도에 필수 의존하도록 바꾸지 않는다.

기존 응답을 보존하면서 새 작업을 학습하는 원리는 [Learning without Forgetting 원 논문](https://arxiv.org/abs/1606.09282)과 관련된다. 하지만 그 논문이 이 주행 모델의 안전성을 보장하지는 않는다. 모델의 행동이 다음 입력을 바꾸므로 오프라인 오차만으로는 부족하다는 근거는 [DAgger 원 논문](https://proceedings.mlr.press/v15/ross11a.html)에서 확인할 수 있다. 이 프로젝트에서는 실제 폐루프 시험으로 별도 확인한다.

## 다음 순서와 남은 범위

**13:05 KST 확인:** GPU0 TRAIN 분석과 로컬 callback45 v2는 모두 종료됐다. [분석 결과·전체 원본](portable_e2e_learning_cycle_v1/07_TRAIN_gradient_diagnosis/README.md), [v2 전체 실패·새 원인 구분](portable_e2e_learning_cycle_v1/06_town03_callback_to_plan_v2/OUTCOME.md), [실제 45초 GIF·PNG](portable_e2e_learning_cycle_v1/06_town03_callback_to_plan_v2/README.md)을 보존한다. v2는 모델 제어 약 96.37m·차량 CTE p95 2.83cm·차체/충돌/침범 검사를 통과했지만 콜백 연결은 FAIL이다. 같은 프레임 번호의 서로 다른 영상 객체를 계측 코드가 거부했다. 정확히 소비한 객체로 연결한 **별도 읽기 전용 원인 분석**에서도 수신→계획 최대 109.83ms·수신→ACK 최대 111.28ms로 100ms에 미달했다. 원래 v2를 PASS로 다시 쓰지 않는다. 따라서 연결 수정뿐 아니라 실제 지연 구간 개선도 필요하다.

**13:37 KST 종료·독립 검산 확인:** [종점 수치 수정](portable_e2e_learning_cycle_v1/08_terminal_numeric_repair/README.md)은 같은 1,660개 전체 검사에서 이전 421개 실패를 해결했고 기존 통과 사례의 퇴행은 없었다. 이는 종점 위치를 가정한 수치 검사이며 실제 주행이 아니다. 전체 209.37m 실행기 연결과 목표점 도달은 아직 미검증이다. [경로 보정만 학습한 새 후보](portable_e2e_learning_cycle_v1/10_branch_only_learning/README.md)는 **1,000회·5,000개 노출·11분 29초 학습을 완료**했다. 좌회전 위치 오차는 30.36%, 우회전은 7.46% 줄었지만 저속 직진은 14.01% 나빠졌다. 사전 기준 **9/18**이므로 미채택이다. 다른 97개 상태·모든 원본 입력 보호와 46개 결과 전량 미러도 확인했다. 현재 이 학습은 실행 중이 아니다. 실제 후속 시험은 GOOD6000을 유지하며 원본·독립 평가 접근 제한·안전 기준을 지킨다.

**13:47 KST 새 실주행 종료:** [callback45 v3 결과·다음 순서](portable_e2e_learning_cycle_v1/09_town03_callback_to_plan_v3/OUTCOME.md)에서 정확한 원본 영상 연결과 전체900tick/7,200시계를 통과했다. 차량 중심 오차 p95 **3.04cm**이며 기존 주행·차체 검사는 통과했다. 하지만 수신→계획 최대 **105.94ms·9/415회**, 수신→ACK **107.16ms·10/415회**로 종합 FAIL이다. 매 영상 묶음 전 전체 저장 폴더 순회가 늦은 구간에 있는지 원래 안전 검사를 유지한 새 구현으로 비교한다. [전체45초 GIF·PNG](portable_e2e_learning_cycle_v1/09_town03_callback_to_plan_v3/README.md)와 실제 원본4,642개·JPEG2,700장·미디어6개 전량 SHA를 확인했다. 이전 실패는 보존하며 기능 승인 수는 바꾸지 않는다.

<!-- HH_261004 - Record measured quota-accounting improvement without substituting it for actual sensor latency or a driving approval. -->
**14:22 KST 폴더 순회 비교 종료:** 같은 최종 폴더 2,723개·276,100,879바이트를 30쌍·60회 전량 계산했다. 기존 순회 median/p95/max **21.554/22.683/23.363ms**, 새 순회 **4.532/4.783/4.926ms**였다. 모든 호출의 바이트 수가 같고, 전체 원본4,642개·소스 전후 SHA도 동일하다. [전체 측정·한계](../../../../storage/experiments/diagnostics/2026-10-04/callback45_v3_complete_quota_scandir_counterfactual_v1/README.md). 이는 무결성 검사가 캐시를 데울 수 있는 최종 폴더 비교이며, 실제 주행 중 센서 지연 개선을 증명하지 않는다. 새 전체209m 실행은 기존 용량·여유 공간·정리 예약 검사와 매번 전체 파일 순회를 유지한다.

**완료한 순서:** 두 학습 종료 → 고정 TRAIN 시점의 정답 오차·기존 출력 변화 비교 → 로컬 Town03 직진 전체 45초 → 실제 원본·차량 중심 PNG/GIF. GPU/CARLA는 직렬 실행했다. Town03은 known DEV이며 독립 TEST가 아니다.

**이번에 마친 순서:** 원본 v1·v2 FAIL 보존 → 별도 v3 소유권 연결을 실제 보호 함수로 검사 → CPU 회귀·새 사전 선언·전체 화면 선택 규칙 고정 → 같은 GOOD6000로 전체 45초 재시험 → 실제 수치·전체 PNG/GIF 확인·원본 재해시. 기존 모델·제어·속도·10cm 차체 inset·100ms 기준은 유지했다. CPU 검사 통과는 주행 기능 승인을 뜻하지 않는다.

**그 뒤의 가중치 개선:** 보존 후보를 자동 확장하지 않는다. 기존 직진과 회전 정답의 곡률·속도·공유 특징 충돌을 계측해, 필요한 학습 분리 또는 작은 전용 보정 구조를 결정한다. 실제 관측된 정상/복구 데이터만 학습 후보로 다루며, 이번 FAIL 원본이나 known DEV를 TRAIN으로 바꾸지 않는다. 독립 평가를 준비하고 RTE-01을 먼저 검증한 뒤 정지·속도·회전의 회귀 범위를 넓힌다. 지도 모델의 새 학습은 아직 시작하지 않았다.

[실제 주행 선언](../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v1_plan.json)과 [화면 선택 사전 선언](../../../../storage/experiments/plans/2026-10-04/town03_known_dev_straight_v1/ROOT_prospective_publication_policy.json)을 시험 전에 고정했다. 로컬 새 코드의 최초 30개 CPU 검사와 실제 사전검사는 통과했다. 선언을 만든 뒤의 재검사에서는 같은 v1 선언을 새로 만들려는 합성 테스트 1개가 create-only 조건에 걸렸다. [118 PASS·해당 1회 실패 기록](../../../../storage/experiments/plans/2026-10-04/town03_known_dev_straight_v1/ROOT_post_declaration_CPU_regression.txt)을 보존하고, 실제 선언을 삭제하지 않은 채 사전 선언 전용 검사만 제외한 나머지 118개를 통과했다. 최종 화면 게시 코드의 CPU 검사 42개도 통과했다. 이는 주행 성능 검사가 아니다.

**첫 기능의 정식 목표:** `RTE-01` 직진 차로 유지. Town07 제한 시험3/3과 Town03 전체 경로 목표 도달은 제한된 개선 근거다. 독립 TEST·정식 반복 분모·실제 어댑터·전체 센서→계획 시간과 실제 처리율 검증이 남아 있다. TRAIN/VAL/미노출 TEST를 합친 최소3개 사이트, 고정 seed/날씨, 기존 안전 기준을 지킨 평가가 필요하다. 3개 독립 TEST 맵을 새로 요구하는 것은 아니며 RTE-04의 정차 속도·유지 시간 기준을 RTE-01에 붙이지 않는다. 로드맵의 안전 핵심 글로벌30회 조건을 임의로 낮추지 않는다. 기존 TEST 접근 금지는 유지 중이며 새 독립 TEST 전용 사전 계획에 대한 사용자 승인도 필요하다. RTE-01 하나가 승인되어도 **하위1/30·메인0/9**다.

메인 목표는 센서·상태 이해(4), 경로·차로 주행(4), 속도·승차감(3), 교통규칙(4), ACC(3), 장애물 대응(4), 차선변경·합류(3), 정차·주차(2), 런타임·실차 전환(3)이다. [전체 기능 정의·기존 기준](../../../portable-e2e-feature-roadmap.md)은 바꾸지 않는다.

## 최신 전체 경로 결과와 바로 다음 작업

<!-- HH_261004 - Replace the stale running label with the complete actual outcome and retain formal approval limits. -->

**14:43 KST 전체 경로·화면 검증 완료:** 실제91.8초 주행으로 원래 목표점에 도달했다. 모델 제어 차체 중심 오차 p95 **3.65cm**, 충돌·차선/차체 침범0이었다. 그러나 수신→ACK **113.31ms** 한 번과 실제 처리율 **9.00Hz**가 남아 종합 FAIL이다. 종점 기록 이름·기어·독립 거리 비교의 검사기 오류도 별도로 수정 중이다. [쉽게 읽는 결과·다음 실험](portable_e2e_learning_cycle_v1/11_town03_full_route209/OUTCOME.md), [전체 실제 GIF·PNG](portable_e2e_learning_cycle_v1/11_town03_full_route209/README.md). 원본7,461개·JPEG5,508장·미디어6개를 전량 재검증했고 기존 FAIL은 수정하지 않았다. 현재 CARLA 시험과 PRO6000 학습은 모두 종료됐다. 정식 완료는 여전히0/9·0/30이다.

다음은 **종점 계산 단계별 계측 → 정확한 실제 기록에 맞춘 새 검사기 → 영상 정규화 전량 동일성·시간 비교 → 새 선언의 실제 전체 경로 재시험**이다. 새 source/revision과 새 결과만 만들며 기존 가중치·안전 조건·실패 분모를 유지한다. 인위적으로 NN 횟수·평가 기준을 줄이거나 TEST를 TRAIN으로 바꾸지 않는다.

## 기존 실제 결과를 읽는 위치

- [Town07 0.5초 설정: 제한 주행 3회 성공, 이전 6회 실패 포함](../2026-10-02/portable_e2e_learning_cycle_v1/07_preview05_actual_signed45/README.md)
- [좌회전: 약 89° 완료했지만 중심 오차 0.414→0.386m로 여전히 0.35m 미달](../2026-10-02/portable_e2e_learning_cycle_v1/24_actual_left_joint_lateral_lookahead_pair/OUTCOME.md)
- [지도: 학습 완료·표지 인식 퇴행 포함 실제 비교 화면](../2026-10-02/portable_e2e_learning_cycle_v1/25_map_native_fusion_v3_results/README.md)

실제 개선과 기능 승인을 구분한다. 실패를 지우거나 안전 기준을 완화하지 않는다. 설치·재부팅·GPU1·다른 사용자의 시험·실차 구동·자동 배포는 하지 않는다. 크레딧 잔액·확정 토큰 비용은 조회할 수 없다.
