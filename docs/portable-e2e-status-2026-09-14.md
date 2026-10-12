# Portable E2E — 9월 14일 재개 지점·목표·학습 계획

<!-- HH_260906 - Reconcile the interrupted September 9 experiments with September 14 observations without claiming continuous training or learned vehicle control. -->

## 먼저 알아둘 결론

우리는 **VAD를 재학습하는 것이 아니라 자체 Portable E2E 경로 예측 모델**을 개발하고 있다.
Autoware에는 현재 shadow로 연결되어 있고, 향후 제어 연동을 검증할 예정이다. 현재 목표는 30 km/h 이하에서
직진·좌회전·우회전·종점 정지·속도 추종을 먼저 안정화하는 것이다.
ACC·장애물 회피·차선변경까지 이미 학습한 완성 모델은 아니다.

<!-- HH_260906 - Lead with the complete negative hold-brake ramp experiment while retaining earlier chronology. -->
**최신 후속 — 정지 제동 램프 4회 완료:** [09번 실험](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/09_hold_brake_ramp/README.md)에서
Town07 직진의 정상 정지 제동을 즉시/0.5초 램프로 ABBA 비교했다. 이번에는 **4회 모두 수집 완료**이며,
이전 v6의 미실행 A2를 채운 것이 아닌 별도 실험이다. 제동 요청·읽기는 바뀌었지만 실제 관측 물리 상태와
곡률 실패 **115/671구간·3시점**은 네 회차 모두 같아 개선안으로 채택하지 않았다.
5,888상태·17,664 JPEG를 검산하고 PNG 25장·GIF 4개를 정리했다. 회귀는 **958 passed / 1 skipped**다.
새 학습·SSH 명령·설치·빌드는 하지 않았다. 다음은 출발/제동 경계의 위치·속도·좌표 기준 분리 진단이며,
새 데이터 QA 통과 뒤 독립 TRAIN/VAL 확대와 GPU 0 전체 모델 학습으로 이어간다.

<!-- HH_260906 - Retain the earlier local three-capture diagnostic and its blocked fourth attempt as history. -->
**17시 후속 — 저속 조향 실측:** [08번 새 실험](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/08_low_speed_steering_abba/README.md)에서
Town07 직진의 기존 조향 A1과 저속 조향 0인 B1/B2를 실제 수집했다. 마지막 A2는 포트 2101
사전 점유 검사에서 차단되어 자동 재시도하지 않았다. **4회 계획·3회 수집·1회 실행 전 차단**이다.
곡률 실패가 134/666구간·39시점에서 B 두 회차 각각 115/671구간·3시점으로 줄었지만,
출발 2시점·제동 경계 1시점이 남고 최대 곡률은 증가했다. 완전 ABBA 판정·학습 편입은 없다.
원본 13,218 JPEG, 차량 중심 6카메라 PNG 15장과 실제 프레임 요약 GIF 3개를 정리했다.
이는 **로컬 BasicAgent 수집**이며 Autoware UI·학습 모델 직접 운전·원격 새 학습은 아니다.
다음은 포트 사전 상태 기록 보강과 출발/제동 전환 중 한 요인만 분리한 새 계측이다.

<!-- HH_260906 - Correct training sufficiency and all-map interpretations using the new independent coverage and raw replay audits. -->
**후속 확인 — 6회는 충분한 학습·모든 맵 성공의 증거가 아니다.** 실제 TRAIN은 3 episode·3맵,
원본 길이 약 2분 14초이고 VAL은 Town03 우회전 1개다. 동일 데이터 손실 A/B × 3 seed를
반복한 실험이며 새 장면이 추가된 것은 아니다. [07번 후속 검사](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/07_coverage_and_curvature/README.md)에서
학습량 검사·19맵/95장면 계획·원시 곡률 재현 코드를 구현하고 실제 기록으로 검증했다.
통합 회귀는 **636 passed / 1 skipped**, 이전 01–06 공개 payload 146개와 native 학습 코드 18개는
변경 없이 보존했다. 이 후속 단계에서 추가 학습이나 새 CARLA 주행은 하지 않았다.
곡률 실패는 출발 6·감속 32·제동 경계 1개 시점이며 134/666 창의 원래 실패를 유지한다.
[전체 맵 승인표](portable-e2e-map-acceptance-2026-09-14.md)는 기존 VAD 성공과 새 Portable 미검증을 구분한다.

<!-- HH_260906 - Lead with the completed September 14 full-model campaign and keep local collection separate. -->
**15:58 KST 완료 — 실제 학습:** 9월 14일 15:29:30~15:58:47 KST에 PRO6000 GPU 0에서
자체 STOPMIX 전체 모델 6회와 후속 검사 18회를 수행하고 결과 회수·독립 검산을 마쳤다.
총 9,240 step·36,930회 표본 노출이며, 선택기만의 학습이 아니라 1,056,362개 파라미터 전체를
새로 학습했다. 후보 비용 보조 손실 B는 동일 seed의 A보다 두 seed에서 상대 기준을 통과했지만,
seed04 ADE는 소폭 악화했고 세 seed 모두 장기 오차 목표에 미달해 **운용 모델은 미채택**이다.
이전 계획의 새 데이터 수집과 병행한 **기존 TRAIN/VAL 연구**이며 TEST나 미승인 수집은 넣지 않았다.
[전체 학습 결과·경로 PNG 72장](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/05_stopmix_regret_learning/README.md)을 확인할 수 있다.

로컬에서는 [Town07 v5 자연 감속→브레이크 유지](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/06_brake_hold_pilot/README.md)를
1회 실행했다. 측정 정지 2초와 정지 후 6.5초 기록은 성공했지만 미래 곡률 134/666구간이
실패해 새 데이터는 학습에 편입하지 않았다. 원본 4,386 JPEG와 ego 중심 카메라/경로 대시보드를
검증·보존했다. 당시 통합 회귀는 539 passed / 1 skipped이며 로컬의 보안 exact-resume 조건에
해당하는 1개만 건너뛰었다. 학습 실행 성공과 데이터 승인·자율주행 기능 통과는 별개다.

전체 로드맵은 **상위 9개 영역·하위 30개 요구사항**이다. 10개 항목에 부분적인 데이터나
계측 근거가 있지만, **학습 모델이 직접 운전해서 폐루프 기준을 통과한 기능은 0/30**이다.
학습 실행 횟수, 코드 테스트 수, 전문가 주행 영상은 이 기능 통과 수와 별개다.

<!-- HH_260906 - Keep the new external-reference review separate from recovered experimental results and actual training. -->
**이후 요청 반영 — METEOR:** [공식 소스 검토와 적용 계획](portable-e2e-meteor-review-2026-09-14.md)을 추가했다.
자체 모델을 유지하면서 보정 기반 BEV·차로/깊이 보조 학습을 검토한다. METEOR 가중치를
설치·실행하거나 기존 모델을 교체한 것은 아니다. 먼저 현재 TRAIN/VAL의 장면과 label 부족분을
읽기 전용으로 집계하고, 새 데이터·보조 정답을 검증한 뒤 다음 전체 모델 A/B로 넘어간다.

11:55 KST에 [실제 데이터 감사](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/01_data_readiness/README.md)를
완료했다. TRAIN/VAL 정지 유지 118/0개이며 118개도 Town01 한 episode에 집중되어 있다.
VAL은 우회전 한 episode이고, 차로·깊이·신호 정답은 미기록이다. 143개 CPU 테스트를 통과했지만
학습 허가·기능 통과·새 모델 채택은 아니다. TEST 정답과 영상은 열지 않았다.

<!-- HH_260906 - Append measured stationary controls without replacing prior data audit or claiming new training. -->
12:39 KST에는 [정지 상태 서비스·주차 브레이크 ABBA 4회](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/02_stationary_motion/README.md)의
분석을 완료했다. 총 1,880상태·940카메라 묶음·5,640 JPEG와 PNG 8장이다.
초기 안착 후 hold 400표본은 네 회차 모두 변위·보고 속도가 0이었고 주차 브레이크 추가 효과는
관찰되지 않았다. 초기화 코드 실패 2회는 별도 보존했다. 관련 코드 303 tests passed.
이는 움직이다 멈추는 문제의 해결이나 학습 데이터 추가가 아니며, 다음은 coast/제동 전환 계측이다.
이번 단계도 원격 학습·새 모델 채택·Autoware 주행 없이 로컬 CARLA 진단만 수행했다.

<!-- HH_260906 - Append the new coast observations and parity limitation without relabeling low-speed dwell as standstill. -->
후속 [저속 무페달 감속 2회](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/03_low_speed_coast/README.md)도
완료했다. 합계 2,026상태·6,072 JPEG, 분석 PNG 6장이다. 두 회차 모두 45초 뒤 속도
0.018527 m/s가 남았고 마지막 약 9.95초간 순변위는 52.741 mm였다. 이때 관측 위치는
전진하지만 보고 전후 속도의 부호가 매 native tick마다 교대하여 저장 10 Hz 시점은 음수 쪽과
겹쳤다. 원인은 아직 확정하지 않았고 표본/속도/판정 기준을 수정하지 않았다. 관련 최종 회귀
373 tests passed. 다음은 저속 속도·위치의 샘플링 일관성과 제동 전환 계측이며, 새 학습은 미실행이다.

<!-- HH_260906 - Record matched brake-response evidence and its transition residual without claiming model or data approval. -->
14:59 KST까지 [저속 감속→서비스 제동 ABBA 4회](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/04_coast_brake_transition/README.md)의
실측을 완료했다. 총 4,052상태·12,144 JPEG·분석 PNG 12장이다. 같은 감속 이력 뒤 15초 Actor
순변위는 무페달 81.340 mm, 제동 0.488 mm로 각각 반복됐다. 제동 두 회차 모두 마지막 200표본은
위치 변화·보고 속도 0이었다. 단, 첫 제동 50 ms에는 약 0.484 mm 후방 위치 변화와 0.960 mm
Actor 위치−속도 적분 잔차가 있어 전환 표본의 물리 일관성까지 승인하지 않는다.
관련 423+16개 테스트 통과; 데이터 편입·원격 학습·Autoware 주행·모델 교체는 없다.
다음은 자연 감속→제동 유지→재출발 프로파일 및 독립 TRAIN/VAL 수집이며 기존 판정 기준은 유지한다.

## 어디서 중단됐나?

1. 9월 9일 06:14 KST: 같은 크기의 잔차 선택기 A/B 학습 6회를 완료했다.
   TRAIN 정지 유지 118개를 보존했으나 VAL 오차는 두 seed 개선·한 seed 악화로 미채택이다.
2. 06:23 KST: 고정 commit `9d03bb2`의 회귀 검사 6,312 passed / 6 skipped를 완료했다.
   이후 작성한 코드까지 검사했다는 뜻은 아니다.
3. 06:35:59 KST: **12번 주행 후보 순위 보조학습 비교**를 시작했다.
   세 부모 선택기 각각에서 기존 손실 A와 순위 보조 손실 B를 비교하는 6회 이어학습이다.
4. 실제 worker는 **06:41:07 KST**, SSH 실행기는 **06:41:08 KST에 exit 0으로 종료**했다.
   문서에는 아직 전체 완료 미확인이라고 남아 있었고 결과 회수·독립 검산·게시가 미완료였다.

**9월 14일 재개 때 직접 확인한 사항:** SSH 연결 정상, physical GPU 0 사용 메모리
0 MiB·사용률 0%, 해당 학습 프로세스 없음, 서버 파일시스템 약 2.9 TiB 여유.
로컬에서 해당 CARLA/학습 프로세스도 발견되지 않았다. 이 수치는 확인 시점의 관측이지
이후에도 자원이 예약되어 있거나 계속 학습 중이라는 뜻이 아니다.

완료된 12번 원격 원본 197개 파일과 이전 실험 205/195개 및 부모 입력 17개를 대조했다.
메타데이터·그림 185개와 전달 검증서 1개를 로컬에 회수했다. 새 가중치/점수 PT 12개는
서버에 그대로 두고 무결성 해시만 확인했으며, 가중치 역직렬화·추론·재학습은 하지 않았다.
실행 보고서·계획·소스·원본을 수정하지 않았다.

독립 검산 결과는 `COMPLETE_DIAGNOSTIC_NOT_PROMOTED`다. B는 세 부모보다 VAL 경로 오차가
감소했지만, 동일 추가 학습량의 A보다 seed03 ADE가 조금 악화했다. 또한 seed05의
미래 속도 0 이후 재가속 지표는 A 7개에서 B 66개로 늘었다. TRAIN 정지 유지 118개 보존과
평균 오차 개선만으로 정지 기능을 승인하지 않는다. 기존 shadow 모델은 교체하지 않았다.

누적 후속 캠페인 실적은 새 전체 모델 학습 **39회**(복구 시점 33회 + 9월 14일 6회), 전체 모델 이어학습 **6회**,
선택기 전용 학습 **21회**, 이번에 완료를 복구한 선택기 이어학습 **6회**로 분리한다.
짧은 계측용 학습·DataLoader 시험·forward 진단은 이 수에 더하지 않는다.
이는 프로젝트 최초 학습까지 포함한 모든 optimizer 호출의 총계가 아니라 문서로 추적하는
후속 비교 캠페인 집계다. 여러 번 학습했다는 사실이 여러 기능을 통과했다는 뜻은 아니다.

- [12번의 실행 계획 및 후속 결과](assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/02_overnight_data_and_learning/12_drive_ranking_continuation/README.md)
- [11번 잔차 선택기의 전체 결과·원본 경로 그림](assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/02_overnight_data_and_learning/11_matched_residual_selector/results/README.md)
- [9월 9일 전체 기록](assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/README.md)

이번 결과 회수·요약 관련 CPU 테스트는 94개, 복구 게시자는 24개를 통과했다.
직전 잔차 결과 게시자 및 공통 도구 테스트 76개도 재확인했다. 실제 12번 결과의
122개 공개 파일·121개 링크·원본 PNG 108장을 검증했다. 코드 테스트 수는 기능 통과 수가 아니다.

## 두 환경에서는 무엇을 하나?

| 구분 | 로컬 Autoware 저장소 | SSH PRO6000 |
| --- | --- | --- |
| 주 역할 | CARLA 시나리오·원본 수집, 데이터 변환, Autoware 연결, 추론·화면·제어 검증 | 데이터 재검증, 모델 학습, 고정 평가, checkpoint 보관 |
| 현재 상태 | v7 제동 램프 ABBA 4회 완료·개선 없음; 출발/제동 3시점 곡률 실패로 미편입; 새 모델 자율주행 없음 | 기존 TRAIN/VAL로 전체 모델 6회 및 후속 검사 완료, checkpoint 6개 서버 보관; 이번 v7 단계에서 원격 작업 미실행 |
| 다음 할 일 | 출발/제동 경계의 위치·속도·좌표 기준 분리 진단 후, 독립 직진/회전/정지/재출발 episode·의도 정답 확보 | 새 데이터가 QA와 분할 검토를 통과하면 다음 전체 모델 비교; v6/v7 자료 자동 편입 없음 |
| 제한 | 다른 실행을 임의 종료하지 않음; 실차 제어·모델 자동 교체 없음 | GPU 0만, 지정한 개인 venv만; GPU 1·Conda·시스템 변경 없음 |

서버 학습 작업공간은 개인 폴더의 `$PORTABLE_E2E_ROOT`, 실제 dataset 루트는
`$PORTABLE_E2E_DATA_ROOT`로 구분한다. 공개 문서에는 계정별 절대 경로를 적지 않는다.
서버에서 Autoware를 빌드·주행하려는 작업이 아니다.
결과 복구·초기 진단 단계에서는 새 학습을 실행하지 않았고, 이후 05번에서 실제 GPU 0 학습까지 진행했다.
이번 작업에서 설치·재부팅·Git push·원격 Autoware 빌드는 하지 않았다.

## 9개 영역과 30개 기능

| 상위 영역 | 하위 수 | 현재 위치 |
| --- | ---: | --- |
| 센서·상태 이해 | 4 | 카메라 동기, 보정/TF, ego 상태의 CARLA 입력 계약 일부 통과; 차로/주행 가능 영역 미시작 |
| 경로·차로 주행 | 4 | 직진·좌/우회전·종점 정지가 최우선; 데이터 일부 및 예측 평가, 직접 운전 미통과 |
| 속도·승차감 | 3 | 30 km/h 속도 예측·추종 품질 개선 중; 승차감 전체 검증 미완료 |
| 교통규칙·교차로 | 4 | 전문가가 신호를 기다린 증거만 있음; 모델의 신호 이해 성공이 아님 |
| 선행차 추종·ACC | 3 | 기능별 학습·검증 미시작 |
| 충돌위험·장애물 대응 | 4 | 기능별 학습·검증 미시작 |
| 차선변경·합류 | 3 | 기능별 학습·검증 미시작 |
| 저속 정차·주차 | 2 | 기능별 학습·검증 미시작 |
| 런타임 안전·실차 전환 | 3 | 기존 모델의 10 Hz shadow 계측; fallback·직접 제어·실차 검증은 미완료 |
| **합계** | **30** | **learned closed-loop 통과 0** |

하위 상태를 중복 없이 세면 센서 기반 3, 데이터 일부 3, open-loop 측정·목표 미달 2,
expert 행동만 1, shadow 계측 1, 미시작 20이다. 이 합계의 10개는 “완료한 기능”이 아니다.
그중 이번에 우선 해결하는 **행동 5개**는 직진·좌회전·우회전·종점 정지·30 km/h 속도 추종이다.
나머지 기반 작업과 신호 expert 기록을 더해 10개가 되는 것이지 기능 10개를 한꺼번에 학습한 게 아니다.

기능별 세부 요구와 판정은 [30개 기능 로드맵](portable-e2e-feature-roadmap.md)과
[기계 판독 기능표](../learning/config/portable_e2e_feature_matrix.yaml)를 따른다.
기능표의 기준일은 9월 8일이며 이후 연구 횟수가 증가했어도 기능 승인 상태는 변하지 않았다.

## 현재 모델은 어떻게 학습하는가?

### 입력 → 모델 → 출력

`현재 6카메라 영상 + 카메라 보정값 + 약 1초 ego 상태 이력 + 예정 경로`
→ `소형 CNN + 상태/경로 GRU + feature 결합`
→ `미래 6.4초의 후보 경로·속도 + 각 후보 점수`.

기존 physical-v1은 6개 후보, 최근 STOPMIX 연구 모델은 주행 6개+정지 6개 후보를 낸다.
현재 카메라 입력에는 과거 영상 시퀀스가 없고 객체·신호·점유공간을 출력하는 별도 head도 없다.
가속·제동·조향 제어 명령을 직접 출력하는 모델이 아니라 **경로와 속도를 예측하는 planning core**다.
소형 baseline을 검증하는 단계이며 Tesla급 완성 모델이나 VAD backbone이라고 부를 수 없다.

CARLA의 BasicAgent/expert가 생성한 주행·영상·미래 궤적을 정답으로 사용하는 **지도 모방학습**이다.
XY·속도·방향·종점·운동 일관성 오차와 후보 선택 손실을 줄인다. 현재 강화학습이나
모델이 스스로 운전한 데이터를 무제한으로 모아 재학습하는 자동 루프는 아니다.

9월 9일의 12번에서는 영상 인코더·경로 생성기가 만든 입력과 12후보를 고정한 채 **선택기만** 학습했다.
좋은 후보를 만들어도 잘못 선택하는 문제를 분리해 보기 위한 통제 실험이다.
“전체 모델을 새로 학습”, “기존 모델을 이어학습”, “선택기만 학습”은 횟수를 분리한다.
9월 14일 05번에서는 이와 달리 **전체 모델을 처음부터 학습**했다. 같은 STOPMIX 구조에서
기존 손실과 후보 비용 보조 손실을 비교했으며, 새 대형 backbone이나 METEOR를 학습한 것은 아니다.

### 실제 학습 데이터

| split | 내용 | 표본 수 |
| --- | --- | ---: |
| TRAIN | Town07 직진 309 + 기존 C-track 좌회전 304 + Town01 우회전 534 | 1,147 |
| VAL | Town03 우회전, 반복해서 확인한 개발 validation | 337 |
| TEST | Town04 직진, 모델 추론·학습·선택에 사용하지 않은 held-out | 309 |

총 1,793개는 1,793개의 독립 주행이 아니라 **5개 episode에서 만든 서로 겹치는 시간 표본**이다.
데이터 계약을 통과한 기존 v3라도 정답의 일부 가감속은 모델의 운동 제한과 맞지 않았다.
정답 품질과 coverage가 충분하다는 뜻으로 읽으면 안 된다. 최근 추가 수집은 품질 미달로
기존 데이터에 편입하지 않았다. 원본을 삭제·보간·시간 조작해 통과시키지 않는다.

실제 차량 공개 데이터 일부는 확보·메타데이터 조사 단계다. **현재 모델을 실제 데이터와
혼합 학습한 완료 실적은 없다.** 타임스탬프, 센서 매칭, 보정·TF, 경로/정답, 이용 조건과
유효 mask를 검증한 별도 adapter가 먼저 필요하다. 기존 10 Hz 기준에 맞추기 위해 원래
시각을 바꾸지 않는다. 비동기 센서를 지원해야 한다면 명시적인 새 입력 계약·검증 문제다.

모델·학습 코드와 정확한 ABI: [모델 설계](portable-e2e-model-v0.md),
[학습 운용 가이드](portable-e2e-training.md), [데이터 현황](portable-e2e-datasets.md).

## 앞으로 진행할 순서

1. **완료 — 중단된 결과 복구·독립 검산:** 12번의 전 seed·정지 subgroup·학습 기록을 확인했다.
   일부 좋아진 평균만으로 승자를 고르지 않았으며 원래 모델과 실패 기록을 보존했다.
2. **정답 수집과 평가 범위 보강:** 직진/좌/우회전/자연 감속/정지 유지/재출발을 독립 episode로
   확보한다. 신호·장애물·선행차 유무와 원인을 구분할 수 있는 입력/label도 설계한다.
   현재 VAL의 stationary-hold는 0개이므로 TRAIN 118/118 보존만으로 정지 검증을 대신하지 않는다.
3. **다음 새 데이터 전체 모델 학습:** 기존 자료에 대한 05번의 전체 모델 6회 비교는 완료했다.
   다음은 새 데이터 버전·split을 고정 → 소수 TRAIN 표본에서 학습 가능성 확인
   → 동일 조건 3개 seed A/B → 전체 경로/속도/정지 subgroup 평가. 현재 TEST는 계속 잠근다.
   같은 작은 데이터에서 epoch만 무작정 늘리지 않는다. 이전 이어학습 6개 모두 단기 ADE가 악화했다.
4. **모델 설계 확장:** 과거 영상 처리와 공간 정보 결합, 필요 시 차로·점유공간·신호/객체 보조
   출력을 비교한다. 이들은 앞으로 구현·실험할 후보이지 현재 완료 기능이 아니다.
   출력 경로 ABI를 유지해 Autoware와 일반 controller adapter에 분리 연결한다.
5. **로컬 실행 검증:** 가중치 export 동등성 → 10 Hz shadow → 이상 입력 거부/fallback →
   안전 조건을 갖춘 제한된 CARLA learned closed-loop. 첫 목표는 모델이 직접 운전하는
   기초 5개 행동의 검증이다. 30 km/h는 목표 상한이지 모든 회전 구간에서 유지할 속도가 아니다.
6. **기능 확장:** 교통규칙·교차로 → 선행차/ACC → 장애물 대응 → 차선변경/합류 → 저속/주차.
   안전 감시·정지·fallback은 처음부터 병행한다. AEB를 학습 planner 하나에만 맡기지 않는다.
7. **실차 전환:** 검증된 real 데이터의 학습/혼합 비율 A/B, real-domain 별도 평가,
   실제 센서 녹화 replay·actuator-off shadow를 먼저 한다. 폐쇄 시험장과 공도 운용은 별도 승인 단계다.

실차와 CARLA를 함께 학습하는 것은 설계 가능하다. 동일 표현으로 변환하고 domain별 샘플링·loss
유효 mask와 별도 평가를 고정해야 한다. 센서 위치 TF만 맞춘다고 화각·가림·시간·영상 분포 차이가
자동 해결되는 것은 아니다. 현재 단계에서 완성 시기나 실차 안전성을 보장하지 않는다.

## 자료를 읽을 때 구분할 것

- **Expert CARLA 영상:** 학습 정답 수집을 위한 전문가 controller 주행.
- **Offline 경로 PNG:** 저장된 영상 입력에서 예측한 후보와 정답의 비교. 실제 차량 제어가 아님.
- **Portable shadow:** 실시간 센서로 예측하지만 차량을 제어하지 않음.
- **과거 Autoware VAD 영상:** 별도 stack의 주행. 새 Portable checkpoint의 직접 운전 증거가 아님.
- **Learned closed-loop:** 새 모델의 출력이 실제 ego 제어에 쓰이고 사전 기준을 통과해야 실적으로 집계.

이번 복구의 목적은 **완료된 결과를 중복 학습하지 않고, 다음에 무엇을 개선할지 확인 가능한
상태로 돌려놓는 것**이다. 새 모델의 자동 승격·실차 제어·데이터 기준 완화는 하지 않는다.
