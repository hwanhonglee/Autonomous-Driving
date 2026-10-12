# 주행 실행 속도: 무엇이 개선됐고 무엇이 실패했나요?

<!-- HH_261008 - Separate actual recorded inference, diagnostic rendering, failed source checks and unapproved driving capabilities. -->

**최신 CNN 컴파일 비교까지 10Hz 기준을 만족하지 못했습니다.** V21d는 실제105.41초 후 종료했고 독립 전량 검산을 마쳤습니다. 이전 V20과 같은 입력의 완전한12쌍에서 전체 ACK 중앙값은104.03→104.31ms였으며 모두100ms를 넘었습니다. 따라서 컴파일 변경을 채택하지 않습니다. 메인0/9·서브0/30, 첫 목표는 직진 차로 유지(RTE-01)입니다.

앞선 V18c의 공통 완전15쌍은109.91→106.20ms, V19c↔V20의 별도14쌍은108.76→104.85ms였습니다. 서로 다른 구간의 값이므로 하나의 개선 추세로 합치지 않습니다. 어느 조건도 안정적인10Hz 주행 성공이 아닙니다.

이번 실행은 **로컬 CPU에서 같은 GOOD6000 모델에 기록된6카메라 입력을 넣은 직접 추론→Autoware ROS 메시지 비교**입니다. teacher 주행·새 CARLA 주행·실차 구동·학습은 아닙니다. 차량 제어 명령은 내보내지 않았습니다.

## 실제 결과

| 조건 | 직접 추론 | ROS 관측 / 원래918개 | 완전한 궤적·Path 쌍 | 그중 전체 ACK100ms 충족 | 미실행 |
| --- | ---: | ---: | ---: | ---: | ---: |
| V13: 상세 기록 압축 | 918/918 | 26/918 | 11 | 0/11 | 892 |
| V14: 수신 실행기 등록 유지 | 918/918 | 28/918 | 13 | 0/13 | 890 |
| V15: 감사 해시 바이트 복사 제거 | 918/918 | 26/918 | 11 | 0/11 | 892 |
| V16b: 과거 입력의 중복 복사 제거 | 918/918 | 31/918 | 15 | 0/15 | 887 |
| V17b: 자체 실행기 CPU4코어 고정 | 918/918 | 10/918 | 4 | 0/4 | 908 |
| V18c: 수신 후 원래 poll 즉시 호출 | 918/918 | 146/918 | 129 | 0/129 | 772 |
| V19c: 시작 파일 반복 파싱 캐시 | 918/918 | 29/918 | 14 | 0/14 | 889 |
| V20: CNN 가중치 메모리 배치 | 918/918 | 43/918 | 27 | 0/27 | 875 |
| V21d: 원래 CNN feature 컴파일 | 918/918 | 27/918 | 12 | 0/12 | 891 |

모두 지연 실패로 종료했습니다. V13~V18c의 마지막 입력은 Trajectory 일부가 실제 전달됐지만 뒤의 Path 진입이 100ms를 넘었습니다. V19c의 마지막 순번 28은 **첫 Trajectory 진입 101.46ms에서 거절**, 실제 출력 수신 0이며 전체 ACK 116.51ms입니다. V20의 마지막 순번42는 Path 진입99.74ms였으나 완료100.012613ms로 늦었습니다. 두 출력은 실제 수신돼 전체 수신쌍은28개지만, 마지막 실패를 제외한 비교 가능한 성공쌍은27개입니다. 실패·부분 출력·미실행을 성공으로 세지 않습니다. V20의 직접918개는 V19c·V18c와 입력9종·원시5출력·상태가 전량 동일했습니다.

V14/V15의 공통 완전 출력11개는 순번5–9와19–24입니다. 그 구간의 감사 평균은10.58→8.49ms, 전체 ACK는110.57→108.59ms입니다. 앞서 사용한 순번14–24의11개는 워밍업·거절을 포함한 다른 집계이며, 완전 출력11개와 혼동하지 않습니다. [원본 표기 오류와 두 집계를 함께 보존한 정정](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v14_v15_clock_label_correction_v2.json)을 따릅니다. 단일 측정끼리의 사후 비교라 지속적·인과적 개선을 입증하지 않습니다.

## 실제 기록에서 만든 지연 그래프

이 PNG는 **저장된 실제 시각의 진단 그래프**이며 RViz·Autoware 화면 캡처나 새 주행 촬영이 아닙니다. 차량 위치를 그린 그림도 아닙니다.

![V18c 실제 관측146개와 마지막 실패·부분 출력의 지연](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_clock_comparison_v4/V18c_all_actual_prefix_clocks.png)

위의 파란 점은 원래 ACK 완료, 주황색 X는 그 뒤 원래 콘솔 출력 완료입니다. 아래 막대는 같은 입력의 실제 처리 구간입니다. 빨간 선은 변경하지 않은100ms 기준이며, 마지막 검은 X는 실제 거절된 publish 진입입니다. 부족한 시각은0으로 채우지 않았고, 원시 NN·감사 시간은 다른 구간 안에 포함되므로 다시 더하지 않습니다. 콘솔 뒤의 출처 연결·최종 결과 저장 비용까지 모두 포함됐다는 뜻은 아닙니다.

[V16b 그래프](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_clock_comparison_v4/V16b_all_actual_prefix_clocks.png)도 전체 관측 31개와 실패를 보존했습니다. [전체 CSV, 918개×두 조건](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_clock_comparison_v4/complete918_per_arm.csv.gz)에는 미실행·누락 시각도 포함됩니다. 그래프 생성 4.89초·NN 0회·학습 0회입니다. 두 실행의 확인 문자열 2항목은 실행마다 달라야 하므로 따로 보존하고, 실제 입력 17항목·원본 출처·완전 출력·9입력 텐서가 같은 15쌍을 구분했습니다. 그림 완료가 모델 성능 통과는 아닙니다. [이전 V13/V14 그림과 원본](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_clock_comparison_v2/result.json)도 유지합니다.

## 다음에 무엇을 바꾸나요?

1. **추론 전 중복 복사 — 종료·미채택:** V16의 모의 검사338개 통과·3개 실패와 [잘못 이어 호출한 준비 단계](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_v16_source_test_failure_v1.json)를 보존했습니다. 새 V16b는344개 검사를 통과한 뒤 실제106.54초 실행 후 지연 실패로 종료했습니다. 마지막 Path 진입102.89ms·전체 ACK117.29ms, ROS887개 미실행입니다. [독립 전량 검산](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_v16b_independent_postcheck_v1.json)에서 V15↔V16b 직접918개의 입력9종·원시5출력·상태가 모두 같았습니다. 공통 완전 출력11개의 전체 ACK 중앙값은108.45→109.66ms로 개선되지 않았습니다. 입력 동일성이 속도 개선을 뜻하지 않습니다.
2. **CPU 배치 — 종료·미채택:** 실제 물리 코어 번호 가정이 틀린 V17은 준비 단계에서 거절됐고 [원본 실패](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_v17_prepare_failure_v1.json)를 남겼습니다. 새 V17b는385개 모의 검사 후 실제103.93초 실행으로 종료했습니다. 시작·로드 후·종료의 자체 스레드 설정16개 checkpoint는 모두 통과했지만, ROS10개 관측·908개 미실행, 완전4쌍의 전체 ACK 통과0입니다. [실제 결과](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v17b/result.json)를 보존하고 미채택합니다. 이전 작업의 실제 코어 위치는 미확인이고 이번 단일 측정도 원인 전체를 설명하지 않습니다. 다른 프로세스·시스템 설정은 변경하지 않았습니다.
3. **검사 임시 메모리 — 합성 실험 종료·미채택:** [실제 NumPy 비교](../../../../../storage/experiments/diagnostics/2026-10-08/cpu_finite_scratch_v1_ROOT_scope_v1.json)는768회 A/B와48회 overwrite 검사 모두 일치했습니다. 첫 메모리 할당을 포함해도 일관된 속도 이득이 없어 모델에 넣지 않습니다. 실제 Torch와 두 번의 원래 모델 검사를 실행한 것은 아닙니다.
4. **입력 준비 후 타이머 대기 — 종료·검산 완료, 기준 미달:** V18의 작성단계 문제와 [V18b 모의 검사356개 통과·5개 실패](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_v18b_source_test_failure_v1.json)를 보존했습니다. 새 V18c는366개 검사 후 실제119.61초 실행으로 종료했습니다. ROS146개 관측·완전129쌍까지 진행했지만 전체 ACK 적시0이며772개 미실행입니다. [독립 전량 검산](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_v18c_independent_postcheck_v1.json)에서 직접918개는 V16b와 입력9종·원시5출력·상태·메시지가 모두 같았습니다. 공통 완전15쌍의 입력 준비 중앙값은31.66→27.81ms, 전체 ACK는109.91→106.20ms입니다. 원시 NN은32.40→33.28ms여서 모델 자체가 빨라진 것은 아닙니다. CPU 고정은 섞지 않았고 타이머·전체 검사·100ms를 유지했습니다. 마지막 부분 출력과 출처 미확인도 그대로 남겼습니다.
5. **시작 파일 반복 파싱 — 실제 종료·미채택:** V19의 사전 실패와 V19b의 실행 대상 오류는 보존했습니다. 새 V19c는 413개 검사와 실제 새 버전 PID·명령 확인 뒤 105.04초 실행 후 종료했습니다. [독립 전량 검산](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v19c_independent_postcheck_v1.json)에서 직접 918개의 입력·원시 출력·상태가 V18c와 동일하고 일곱 캐시 마감 기록도 정상이었습니다. 실제 파싱 8회·캐시 적중 4,557회였지만 공통 완전 14쌍의 ACK 중앙값은 105.67→108.76ms로 늘었습니다. 원시 NN은 33.15→31.65ms로 변동했으나 전체 속도 개선 근거는 없습니다. 캐시 적중을 성능 개선으로 세지 않습니다.
6. **CNN 메모리 배치 — 실행·독립 전량 검산 완료, 미채택:** 모의 검사478개 통과 후 V20을 실행했습니다. 101개 가중치 값·shape·dtype·device와 원래 모듈은 유지했고 Conv5개의 stride만 바꿨습니다. 변환0.285ms·layout 준비10.467ms·추가 NN 워밍업0회이며 시작 준비 비용은 원래 전체 실행에 포함합니다. 실제100.75초 뒤 지연 실패로 종료했습니다. [독립 전량 검산](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_v20_independent_postcheck_v1.json)에서 직접918개는 V19c·V18c와 모두 동일했고, ROS43개에서도 입력9종·원시5출력은 같지만 상태는41개만 같았습니다. V19c와 같은 입력의 완전14쌍에서 ACK 중앙값108.76→104.85ms, 원시 NN31.65→32.14ms입니다. NN 자체가 빨라졌다는 증거는 없으며 완전27쌍의 적시ACK0이라 채택하지 않습니다. [실제 결과](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v20/result.json), [원문 계획](../../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v20/ROOT_plan.json), [실행 전 검토](../../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v20/ROOT_execution_review_v1.json)를 보존합니다.
7. **원래 CNN의 실행 방식 — 실제 종료·검산 완료, 미채택:** V21/V21b의 소스 단계 중단과 V21c의71개 통과·1개 실패를 보존했습니다. 새V21d는76개 검사 후 실제 컴파일·추론·ROS 비교를 수행했습니다. [독립 전량 검산](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_v21d_independent_postcheck_v1.json)에서 원래101개 상태·compiled12개 parameter의 연결·복원과 직접918개의 입력9종·원시5출력·상태가 V20과 전량 동일함을 확인했습니다. 공통12쌍의 ACK 중앙값104.03→104.31ms, 원시NN 평균32.65→31.68ms여서 전체 지연 개선은 입증되지 않았습니다. 평균과 중앙값을 혼동하지 않습니다. 마지막 순번26은 Trajectory가 실제 전달됐지만 Path 진입103.58ms에서 거절됐고 전체 ACK116.88ms였습니다. ROS891개 미실행·부분 출력·출처 미확인도 보존합니다. 설치·추가 워밍업·학습은0입니다. [실제 결과](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v21d/result.json), [원문 계획](../../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v21d/ROOT_plan.json)을 따릅니다. 독립 검산기의 첫 null 처리 실수도 원본을 남겼으며, 모델 실행 실패와 구분합니다.

위의 최신 그림은 V4의 84개 모의 검사·별도 계획·독립 소스 검토 후 실제 기록으로 생성했습니다. [V3의 사전 발견](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_clock_v3_source_finding_v1.json)은 실행별 확인 정보 2항목까지 같게 비교했던 문제이며, 원본 계획·70개 검사 통과와 실행 전 중단 상태를 유지합니다. 이전 V13/V14를 최신 시험이라고 바꾸어 부르지 않습니다.

직전 실제 직접→ROS 실행은 약100–110초였습니다. 새 조건도 같은918개를 선언하고 실패 꼬리를 보존하며, 소스 검토·실패 수정 시간은 별도입니다. 오전9시까지 비학습 작업을 이어가지만 공유 CARLA·새 GPU 작업·학습·자동 승격은 하지 않습니다.

## 근거와 재현

- 실제 실행: [V13 결과](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v13/result.json), [V14 결과](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v14/result.json), [V15 결과](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v15/result.json)
- 원문 계획: [V15 ROOT 계획](../../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v15/ROOT_plan.json), [실행 전 검토](../../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v15/ROOT_execution_review_v1.json)
- 기록 보존 검산: [V13](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v13_independent_postcheck_v1.json), [V14](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v14_independent_postcheck_v1.json), [V15](../../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v15_independent_postcheck_v1.json)
- 실제 최신 그래프: [V4생성결과·전체입력/소스·선택한11개provider](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_clock_comparison_v4/result.json), [그림만허용한ROOT검토](../../../../../storage/experiments/plans/2026-10-08/recorded_ros_clock_comparison_v4/ROOT_execution_review_v1.json)

[오늘 전체 상태](../README.md) · [지도 정답 확인](../mapping_review/README.md) · [9개 영역·30개 기능·합격 기준](../../../../portable-e2e-feature-roadmap.md)
