# 2026-09-08 Portable E2E 학습·수집 개선

<!-- HH_260906 - Link the dated work ledger without presenting an ongoing campaign as deployment completion. -->

[최신 결과 폴더](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/README.md)에
정답 데이터 진단, 생성기를 고정한 선택기 9회 학습, 로컬 자연 정지 수집 개선을 분리해 기록합니다.

<!-- HH_260906 - Add completed synthetic research and preflight work while keeping remote training status unchanged. -->
**18:27 KST 추가 완료:** [25 정지 출력 구조](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/25_stop_primitive_research/README.md)를
별도 연구 모듈로 구현하고 합성 후보 600개 모두 기존 출력 검증·독립 수식 대조를 통과했습니다.
실제 주행·학습한 정지 판단이 아니며 기존 모델과 데이터는 바꾸지 않았습니다.
[26 VisionPilot 사전 검사](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/26_visionpilot_preflight/README.md)도
완료했지만 현재 로컬 추론 환경·센서·출력 어댑터가 미준비라 모델 실행은 하지 않았습니다.
원격 접근·설치·새 GPU 학습 없이 로컬 CPU만 사용했습니다.
최종 그래프 보완을 포함한 전체 코드 검사는 **4,795 passed / 6 skipped**입니다.
[25 폴더](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/25_stop_primitive_research/README.md)에
실제 수치 PNG 2장·모든 후보 원본·재실행 명령·검사 로그·소스 해시를 모았습니다.

<!-- HH_260906 - Distinguish the independent Portable model from existing and proposed external baselines. -->
자체 **Portable E2E 모델을 설계·학습하는 프로젝트**이며 VAD 재학습과는 다릅니다.
VAD는 기존 연결된 비교 기준, VisionPilot은 아직 실행하지 않은 외부 비교 후보입니다.
[역할·센서 차이·공정한 비교 계획](portable-e2e-visionpilot-comparison.md)을 추가했습니다.

<!-- HH_260906 - Link the completed read-only motion study without claiming fresh training or raw-data admission. -->
**15:28 KST 후속 완료:** [24 미세 이동 분석](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/24_micro_motion_study/README.md)에서
원본 구간 30,908개와 미래 지점 461,824개를 처리했습니다. 원본 해시와 기존 곡률 실패
1,560개 창은 유지됐습니다. 위치–속도 적분 잔차와 100/200/500 ms 방향 차이를 수치·그래프로
기록했으며, 원인 확정·라벨 수정·새 학습이나 모델 교체를 하지 않았습니다.
해당 분석과 표시 보완 당시의 코드 회귀는 **4,577 passed / 6 skipped**이며,
원본 로그와 소스 해시는 24 폴더에 있습니다. 아래 23의 4,518개 결과는 앞선 시점의 기록입니다.

<!-- HH_260906 - Show the latest bounded execution outcome separately from the overnight history. -->
오후 재개한 C-track 출발 페달 비교는 **14:11 KST에 8회 모두 종료**했습니다.
목표 정차는 8/8, 20 Hz 가감속 기준은 2/8 충족이며 통과한 조건은 페달 0.13의 두 반복입니다.
전체 영상 46,416장과 6.4초 미래 창 7,216개 검사를 완료했으며, XY 곡률 초과는
1,560개 창에 남았습니다. 가감속을 통과한 0.13도 각 반복 905개 중 161개 창이
곡률 초과로 **8회 모두 학습 데이터 미승인**입니다. 겹치는 창을 독립 사건 수로 세거나,
실패 자료를 제외하거나 데이터·모델을 승격하지 않았습니다.

<!-- HH_260906 - Report the final two-directory regression independently of driving and training status. -->
[23 최신 코드 회귀 검사](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/23_code_validation_final/README.md)는
일반 테스트 4,145개와 Autoware 런치 테스트 373개를 통과했습니다. 합계 **4,518 passed /
6 skipped**이며, 초기 환경 미설정 실행과 부동소수점 완전 동일성 비교 실패도 보존했습니다.
테스트의 수치 비교만 보완했으며 모델 계산·물리 한계·데이터 승인 기준을 완화하지 않았습니다.

<!-- HH_260906 - Add completed pedal measurements and corrected warmup accounting without promoting data or models. -->
로컬의 [고정 페달 12개 계측](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/04_pedal_response_calibration/README.md)은
완료됐습니다. 일정한 페달에서도 출발 가속과 저속 급정지가 남아 기존 제어기 전환만을 원인으로
단정할 수 없습니다. 특정 물리 내부 원인은 미확정이며, 앞선 자연 정지 수집 두 시도는 모두 품질 FAIL로
학습에 채택하지 않았습니다. 다음은 이 계측에 근거한 원인 분리·수집 제어 개선과 재검증입니다.

[Warmup·과거 입력 감사](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/06_warmup_history_audit/README.md)는
v3의 warmup 앵커 train105/1,147·val35/337 포함을 확인했습니다. 9월 5일 문서의 잘못된 제외
설명을 정정했으며, 기존 데이터·평가 분모는 유지합니다. 향후 history-only 방안은 미채택 제안입니다.

원격은 개인 venv·GPU0의 학습/평가 전용, 로컬은 CARLA 데이터 수집·추론/제어 검증 전용입니다.
새 선택기들은 세 seed 전체 기준을 통과하지 못해 승격하지 않았습니다. 학습 모델의
폐루프 주행·실차 제어 승인은 없으며 기존 shadow 모델을 유지합니다.

원래 사용자 요청에 따른 작업 경계는 **2026-09-08 오전 10시 KST**였습니다.
야간 작업은 마지막 시험이 05:56 KST에 종료된 뒤 중단됐고, 오전 10시까지 연속 수행하지
못했습니다. 사용자 요청으로 13:53 KST 이후 재개했으며 원래 결과와 별도 재개 기록을 구분합니다.
실제 완료·실패·진행 상태와 다음 순서는 위 결과 폴더의 기록을 확인합니다.

<!-- HH_260906 - Report completed raw driving evidence without interpreting scalar success as dataset or model approval. -->
[Town07 전체 경로 재시험과 실제 화면](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/10_comfortable_v3_pilot/README.md)에
2회 전체 결과와 PNG 12장·GIF 2개를 추가했습니다. 최고 속도 약 28.6 km/h·8.6초 이상
순항·목표 앞 자연 정지는 확인했지만, 1회 급감속 초과와 두 실행의 제어 기록 시점 불일치가
있어 모두 미승인입니다. 이후 비교와 남아 있는 문제는 아래처럼 구분합니다.

<!-- HH_260906 - Update the chronological handoff after completed ACK, brake-free and full raw-geometry audits. -->

| 후속 검사 | 확인된 결과 | 아직 확인되지 않은 것 |
|---|---|---|
| [11 제어 수신 확인](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/11_acknowledged_control_protocol/README.md) | 두 실행 모두 제어 기록 불일치 0건 | 실제 액추에이터 적용 시각; 접근 급감속은 여전히 실패 |
| [12 정지 상태 Low/Epic 비교](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/12_stationary_camera_quality/README.md) | 같은 위치에서 Low 체크무늬와 Epic 차선·아스팔트 차이 확인 | GUI FPS·정확한 렌더링 내부 원인; 엄격한 사전 등록 시각 증명은 미충족 |
| [14 정상 브레이크 제거](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/14_brake_free_goal_stop/README.md) | 같은 조건 두 회 모두 기존 스칼라·정지·제어 기록 기준 충족 | 전체 궤적 품질과 학습 데이터 승인 |
| [15 원본 전체 미래와 좌표 기준](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/15_data_reference_and_raw_geometry/README.md) | JPEG 8,802장·미래 앵커 1,337개 확인; 원본 3D에서 기존 좌표 기록 재현 | 저속 XY 곡률 실패가 남음; 물리 COM/후륜 기준점 동일성 미입증 |

따라서 현재 결과는 **정답 생성용 expert 수집 개선**이며, 새 모델의 자율주행 성공이
아닙니다. 새 원본을 학습 데이터로 채택하거나 모델·안전 한계·차량 TF를 바꾸지 않았습니다.
이후 같은 수집기의 Low/Epic 실제 시간 계측, 현재 decoder의 표현 가능성 진단,
별도로 선언한 낮은 속도의 C-track 회전 개발 시험을 아래 순서로 진행했습니다.
실험 완료와 품질 통과는 각 결과 폴더의 실행 기록을 기준으로 구분합니다.

<!-- HH_260906 - Record the completed prospective quality/timing pair without attributing unmeasured GUI or inference latency. -->
[16 Low/Epic 동일 경로 실제 계측](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/16_wall_timing_quality/README.md)은
각 1회 완료됐습니다. 각각 상태 1,462개·카메라 묶음 731개·ACK 1,466개가 모두 일치하고,
실제 시간 기준 카메라 묶음 처리량은 36.5/40.6 Hz였습니다. 전체 준비·GUI·Autoware·추론을
측정한 것이 아니며, 단일 순차 비교로 렌더 품질의 일반적 성능 차이를 단정하지 않습니다.
원본 6카메라·차량 중심 경로 PNG 13장·GIF 2개와 모든 tick 지연을 함께 공개했습니다.

<!-- HH_260906 - Separate completed decoder-only numerical research from the failed native low-speed turn. -->
[17 디코더 표현 가능성](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/17_decoder_representability/README.md)은
GPU0에서 05:05:03–05:07:27 KST에 완료했습니다. 모델 가중치가 아닌 잠재 입력만 최적화했고,
전체 8,022개 후보의 잔차·게이트를 재계산했습니다. 출력 실패 34개와 초기 목적함수
재계산 불일치 22개를 보존했습니다. 평균 근사 오차를 학습 모델의 예측 성능으로 해석하지 않습니다.

[18 C-track 저속 좌회전](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/18_c_track_low_speed_turn/README.md)은
실제 최고 14.4086 km/h·목표 앞 0.9386m 정지와 2초 유지를 확인했지만, 20Hz 출발
한 구간 +3.654933 m/s² 초과로 실패했습니다. 차량 중심 경로와 전체 6카메라 화면을
PNG 7장·GIF 1개로 정리했습니다. 30km/h 검증·새 학습 데이터 승인·모델 제어 영상이 아닙니다.

<!-- HH_260906 - Link completed initialization and decoder verification without hiding retained failures or the interrupted session. -->
[19 최초 프레임 이후 초기화](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/19_agent_initialization_comparison/README.md)는
첫 조향을 거의 0으로 정상화했지만, 출발 최대 +3.467811 m/s² 초과는 유지됐습니다.
[20 두 CPU 최종 전방계산](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/20_decoder_forward_verification/README.md)은
각 1,540,224개 수치가 원래 허용량 내에서 일치했습니다. 초기 목적함수 22개 미확인과
원격 그래프 생성의 패키지 부재 실패도 보존했으며, 추가 설치·모델 학습으로 표시하지 않습니다.

<!-- HH_260906 - Link the completed resumed matrix and all-future diagnosis without calling expert data research learned driving. -->
[21 출발 페달 8회 비교](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/21_turn_launch_matrix/README.md)는
차량 중심 경로·전체 6카메라 PNG 26장, 출발 0–8초 구간의 2배속 GIF 8개, 전체 주행
계측을 함께 제공합니다. 정차·제어 기록·초기화는 8/8 충족이고 native 가감속은 2/8
충족입니다. 카메라 10 Hz 차분에서는 가려지는 출발 초과가 20 Hz 원본에 남아 있습니다.
첫 화면 생성의 경로 오류는 수정했고, 당시 부분 출력은 원본 그대로 별도 보존했습니다.

[22 전체 미래·미세 변위 분석](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/22_turn_launch_raw_geometry/README.md)은
미래 창 7,216개를 모두 검사하고 실제 계측 그래프 3장을 추가했습니다. 곡률 위반은 XY
위치 차분 평균속도 1 m/s 미만에 있으며, 극소 변위의 방향 계산과 실제 회전 제어를
구분해야 합니다. 노이즈나 물리 기준점의 원인을 확정한 것은 아닙니다. 스칼라를 통과한
0.13도 두 번 모두 곡률 초과가 있어 미승인이고, 학습 데이터·기존 모델·판정 기준은 유지합니다.
