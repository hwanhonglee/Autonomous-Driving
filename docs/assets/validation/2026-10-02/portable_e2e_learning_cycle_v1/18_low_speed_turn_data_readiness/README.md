# 저속 회전 학습 자료 — 경로 전수 검사와 다음 수집

<!-- HH_261002 - Keep static geometry, actual teacher qualification, and neural closed-loop performance as separate evidence. -->

현재 모델의 회전 학습 자료 328개 중 **미래 6.4초 전체가 2.5m/s 이하인 자료는 0개**다. 실제 저속 회전 정답을 새로 수집할 필요가 있다. 기존 정답의 시간을 늘리거나 속도·곡률을 임의로 바꾸지 않는다.

## 좌회전 학습 완료와 새 좌우회전 균형 학습 진행

<!-- HH_261002 - Distinguish actual qualified data conversion and GPU optimizer progress from a teacher-only capability. -->

실제 좌회전 347개 전량의 자격 검사와 학습 형식 변환을 완료했다. 프로그램의 tuple과 저장 JSON의 list가 다른 문제는 새 v3 어댑터에서 **컨테이너 형식만** 정규화했다. 수치·시간·속도·곡률·원래 실패 기준은 바꾸지 않았고, 저장된 347개 모든 창이 다시 계산한 값과 일치함을 확인했다. 실패한 v2 변환 시도와 최초 v1 수집은 보존한다.

서버 개인 폴더로 옮긴 2,095개 파일의 전체 SHA·크기 검사를 통과했다. 19개 TRAIN CPU 검사 다음으로 **GPU0에서 13:09:40 실제 학습을 시작했고, 고정 20,000회 업데이트·80,000회 입력 노출과 15:12 전체 평가까지 완료**했다. 새 좌회전과 기존 직진·복귀·속도를 함께 학습했다. 평가 60/74·미채택이며, 실제 좌회전에서는 약 89° 회전·출구 후 10m 및 3초 안정 주행·침범 0을 관측했지만 중심 오차 p95 0.413m가 기존 0.35m 기준을 넘었다. [학습 구성·실제 개선과 미달·다음 제어 비교](../20_actual_low_speed_left_joint_learning/README.md).

<!-- HH_261002 - Admit only the separately captured full RIGHT80 after all697 genuine futures pass; preserve RIGHT70 denial. -->
**새 80초 우회전은 수집·전량 미래 검사·학습 형식 변환을 완료했고 새 균형 학습에 사용 중이다.** 실제 약 −89.97° 회전, 출구 뒤 10m와 3초 안정 주행, 상태 1,755개·카메라 878묶음·JPEG 5,268개·ACK 1,760개를 보존했다. 차체 1,755개·이동 영역 1,754개와 전 구간 가감속 검사를 통과했다. 고정 **697개 창·44,608개 실제 미래 점**은 실패 창 0개다. 이전 점이 없는 첫 곡률 697개는 미평가로 보존하며 0 정답으로 만들지 않았다. ROOT가 전량 연구 TRAIN 투입을 승인한 뒤 697개를 모두 변환했다. 별도 새 모델의 실제 가중치 업데이트가 시작됐으며 교사 성공을 모델 우회전 성공으로 세지 않는다.

[80초 실제 교사 수집](../../../../../../storage/experiments/validation/2026-10-02/town01_low_speed_right80_teacher_v1/pilot_00/diagnostic_report.json), [697창 전량 미래 검사](../../../../../../storage/experiments/diagnostics/2026-10-02/town01_low_speed_right80_future64_v1/summary.json), [별도 전량 TRAIN 승인](../../../../../../storage/experiments/plans/2026-10-02/town01_low_speed_right80_v1/ROOT_entire697_TRAIN_conversion_authority.json), [변환 영수증](../../../../../../storage/datasets/prepared/town01-low-speed-right80-train-20261002-v1/moving_conversion_qualification.json)을 구분한다. **서버 개인 `datasets/prepared`로 복사한 4,195개 파일도 전체 SHA·크기 검증 완료**했다. [실제 서버 전량 검증](../../../../../../storage/experiments/training/2026-10-02/town01_right80_prepared4195_transfer_v1/ROOT_actual_remote_all4195_verification.json). 지도 GPU0 작업의 실제 종료·350장 평가·독립 재검산 후, 19:06:29에 기존 직진·복귀·LEFT347과 새 균형 학습을 시작했다. 19시 38분 확인에서 **4,638/20,000회·23,190개 입력 노출**, 오류 없음이다. [실제 새 학습과 다음 전체 평가·좌우회전 시험](../26_actual_left_right_joint_learning/README.md)에 구분해 정리했다.

<!-- HH_261002 - Preserve the completed RIGHT70 whole-capture and global dwell failure; never admit its clear future windows alone. -->
**새 우회전 v2 수집과 전량 미래 검사도 완료했다.** 상태 1,555개·6카메라 778묶음·ACK 1,560개·차체 1,555개·이동 영역 1,554개를 보존했다. 침범 0이며 10cm 안쪽 최소 여유는 차체 0.247m·이동 0.188m, 전 구간 가감속 −2.026~+2.348m/s²다. 고정 597개 창의 실제 미래 38,208점도 위반 0이다. 그러나 출구 후 10m는 62.95초에 도달했고 고정 마지막 시점은 63.60초여서, 그 안에서 필요한 3초 안정 주행을 확인할 수 없었다. **전체 FAIL을 유지하고 597개 일부·전부를 TRAIN에 넣지 않는다.** 고정 기준을 낮추지 않고 별도의 충분한 길이 수집을 준비한다. [수집 전체 원본](../../../../../../storage/experiments/validation/2026-10-02/town01_low_speed_right70_teacher_v2/pilot_00/diagnostic_report.json), [597창 전체 미래 검사·학습 금지](../../../../../../storage/experiments/diagnostics/2026-10-02/town01_low_speed_right70_future64_v2/summary.json).

[새 347개 전량 자격 영수증](../../../../../../storage/experiments/training/2026-10-02/town01_left45_admission_v3/qualification.json), [실제 학습 형식 변환 영수증](../../../../../../storage/datasets/prepared/town01-low-speed-left45-train-20261002-v3/moving_conversion_qualification.json), [이전 자료 형식 오류 보존](../../../../../../storage/experiments/diagnostics/2026-10-02/town01_left45_failed_v2_admission/root_observed_failure.json)을 구분한다.

## 12:15 KST 기록 — 새 좌회전 데이터의 전체 미래 정답 검사 통과

<!-- HH_261002 - Record real fresh V2 qualification without rewriting the failed V1 or claiming neural turn completion. -->

새 v2를 같은 물리 조건으로 실제 재수집했다. **약 89.88° 좌회전과 출구 뒤 10m 복귀**, 실제 상태 1,055개·6카메라 528묶음·JPEG 3,168개·ACK 1,060개를 보존했다. 차체 1,055개와 이동 영역 1,054개에서 침범 0이고, 10cm 안쪽 차선의 최소 여유는 0.440m·0.371m였다. 실제 가감속은 전 구간 −1.929~+2.550m/s²로 기준 안이다. 종료 제동과 제로 페달 coast 문맥을 실제 기록대로 읽는 새 검증기에서 전체 수집 검사를 통과했다.

**고정 347개 시점·22,208개 실제 미래 점의 검사도 통과했다. 실패 창은 0개다.** 원래 미래 정답 9개 기준과 추가 저속 곡률·횡가속·헤딩 검사를 유지했다. 계산에 필요한 이전 점이 없는 각 창의 첫 곡률 347개는 미평가로 남기며 0 곡률 정답으로 바꾸지 않았다. 원래 학습 금지 항목·보호 구간 실패는 없고, 속도나 시간을 바꾸거나 좋은 창만 골라내지 않았다. [새 수집 원본 판정](../../../../../../storage/experiments/validation/2026-10-02/town01_low_speed_left45_teacher_v2/pilot_00/diagnostic_report.json), [전체 347창 미래 검증](../../../../../../storage/experiments/diagnostics/2026-10-02/town01_low_speed_left45_future64_v2/summary.json), [모든 창의 실제 정답](../../../../../../storage/experiments/diagnostics/2026-10-02/town01_low_speed_left45_future64_v2/all347_true_future_windows.jsonl)을 보존한다.

12:15 당시에는 변환·학습이 남아 있었다. 이후 전량 변환과 서버 CPU 검사를 마쳤고 실제 모델의 고정 20,000회 학습에 사용했다. **교사 좌회전은 모델 성공이 아니다.** 바로 전 주행 모델의 회전 손실 비교는 대조군 약 48°·후보 약 38°에서 둘 다 이탈했다. 실제 저속 회전 정답이 없는 상태에서 공간 손실만 추가하는 방법은 이번 시험에서 개선으로 이어지지 않았다. 메인 0/9·하위 0/30은 그대로다.

## 11:42 KST 첫 수집의 실패 기록 — 그대로 보존

<!-- HH_261002 - Preserve the first real pilot's integration failure even when separate native motion and body diagnostics pass. -->

첫 교사 수집은 실제 완료했다. **상태 1,055개·6카메라 묶음 528개·JPEG 3,168개**, 정상 교사 제안 901개·진단 coast 관측 130개·전체 ACK 1,060개를 보존했다. 약 89.88° 좌회전과 회전 출구 이후 10m 복귀를 관측했다. 전 구간 실제 가감속은 −1.929~+2.550m/s²였다.

원래 전체 판정은 **FAIL**이다. 기존 ACK 판정기가 새로 선언한 `left45_post_capture_cleanup_full_brake` 기록명을 몰라 차체 검사 전에 거부했다. 원래 실패 판정과 원자료는 덮지 않았다. 별도의 읽기 전용 계산에서 실제 차체 1,055개·중간 이동 1,054개는 모두 침범 0이고, 안쪽 10cm 기준 최소 여유는 0.440m·0.371m였다. [원본 실패](../../../../../../storage/experiments/validation/2026-10-02/town01_low_speed_left45_teacher_v1/pilot_00/diagnostic_report.json), [별도 차체·통합 원인 검사](../../../../../../storage/experiments/diagnostics/2026-10-02/town01_left45_v1_root_read_only/independent_body_and_integration_audit.json)를 구분한다.

**실패한 첫 수집은 TRAIN에 넣지 않는다.** 새 실행을 사전에 고정해 다시 수집한 v2와 구분한다. v1 원본 FAIL은 그대로이며 전체 시도 분모에서 제거하지 않는다. 교사 데이터 검증과 신경망 회전 성공은 별개다.

## 실제 완료한 경로 검사

Town01 TRAIN 카탈로그의 기존 좌·우회전 **32개 전체**를 오프라인 CARLA 지도와 대조했다. 경로 좌표와 종점을 그대로 보존하고 차체 8개 꼭짓점, 차체 사이의 이동 영역, 차선 안쪽 10cm 여유를 검사했다.

| 결과 | 경로 수 | 의미 |
| --- | ---: | --- |
| 명목 경로 여유 통과 | 1 | 경로 중심을 정확히 따라간다는 가정의 정적 검사 통과. 실제 운전 성공이 아님. |
| 차체·이동 영역 침범 | 6 | 기존 원본을 보존하고 수집 후보로 사용하지 않음. |
| 지도 일치 거부 | 25 | 원래 경로의 차선·위치 일치를 확인하지 못함. 침범 없음으로 세지 않음. |

[32개 전체 원본 결과](../../../../../../storage/experiments/diagnostics/2026-10-02/town01_train_turn_nominal_census_v2/summary.json), [사전 고정 소스·입력 해시](../../../../../../storage/experiments/plans/2026-10-02/town01_nominal_all32_v2/caller_pins.json), [처음 선택한 우회전 경로의 거부 기록](../../../../../../storage/experiments/plans/2026-10-02/town01_nominal_train_curve_v1/root_actual_failure_receipt.json)을 남겼다. 실제 서버·GPU·차량은 이 검사에 사용하지 않았다.

<!-- HH_261002 - Preserve the sharper RIGHT failure and distinguish the separately cleared gentle route from a physical drive. -->
그 뒤 급한 우회전 후보는 이동 여유 2,018개 중 72개가 실패해 보존하고 제외했다. 별도 완만한 우회전 경로는 차체 1,941개·이동 1,940개 전량 통과했고, 10cm 안쪽 기준 최소 간격은 각각 0.237·0.179m다. [완만한 경로 전량 검증](../../../../../../storage/experiments/training/2026-10-02/town01_train_right_gentle_centerline_catalog_v1/ROOT_actual_nominal_review.json)은 정적 검사이지 실제 운전 성공은 아니다. 같은 기준의 70초 교사 수집과 597×64 미래 정답 검사로 이어간다. 첫 v1의 출발 전 선언 형식 오류는 [실패 기록](../../../../../../storage/experiments/diagnostics/2026-10-02/town01_right70_v1_preflight_failure/ROOT_observed_failure.json)에 보존했고, CARLA·차량은 시작하지 않았다.

## 좌회전 수집 전 사전 조건 기록 — 이후 v2 수집 완료

유일하게 통과한 경로는 **Town01 좌회전 `town01_left_s0002_p03`**다. 전체 원본 길이는 약 220.94m, 좌회전은 약 89.97°다. 차체 1,988개와 이동 영역 1,987개를 검사했으며, 10cm 안쪽 차선 기준의 최소 여유는 각각 약 0.316m·0.265m였다. 이 수치는 실제 움직이는 차량의 여유가 아니라 명목 경로 검사값이다.

새 수집은 먼저 한 번의 **저속 교사 주행**으로 진행한다. 신경망 성공으로 표시하지 않는다. 45초 주행과 뒤의 6.5초 진단 문맥을 보존하고, 사전에 정한 347개 시점의 미래 64점씩 **22,208개 실제 관측 점**을 전량 검사한다. 신호 Green 고정은 통제된 기본 회전 수집 조건이며 신호 준수 기능 증거가 아니다. 원래 위험 감지는 유지하고 신호 상태는 복원한다.

말미는 정지 학습이 아니라 수동 1단·제로 페달의 **진단 coast 문맥**이다. 정지했다고 가정하지 않는다. 전 구간 가감속·차체·ACK·카메라 검사를 유지하고, 말미도 실패 분모에서 제외하지 않는다. 캡처 이후 별도 종료 제동 ACK는 관측되지 않은 정지 상태로 만들지 않는다.

**학습 투입 조건:** 실제 회전 완료와 회전 후 복귀, 모든 차체·이동 영역 여유, 원래 카메라·ACK·처리 시간 기준, 전 구간 +3/−6m/s², 미래 정답의 속도·곡률·가감속 표현 범위를 모두 확인한 뒤 별도 변환·학습 허가를 낸다. 하나라도 미달이면 원자료만 보존하고 TRAIN에 넣지 않는다. 새 좌회전 v2는 이 조건과 전량 변환을 통과했고 이번 학습에 사용했다. 실패한 v1은 투입하지 않는다.

이번 주행 비교 학습의 원래 대조군·후보와 데이터 보강은 별개다. 두 모델을 먼저 같은 우회전·직진 조건에서 평가한 뒤, 합격한 새 저속 회전 자료로 다음 혼합 학습을 진행한다.
