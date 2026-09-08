# Portable E2E — 선택기 분리 학습과 자연스러운 정지 데이터 개선

<!-- HH_260906 - Keep the overnight work ledger separate from model deployment and expert driving evidence. -->

2026-09-08 작업 자료입니다. 사용자가 요청한 작업 경계는 **오늘 오전 10시 KST**입니다.
아래는 진행 중인 기록이며, 이 문서가 학습 모델의 차량 제어 승인이나 모든 기능 완료를
뜻하지 않습니다. 기존 shadow 운용 checkpoint는 유지합니다.

<!-- HH_260906 - Disclose the interrupted overnight session and separately authorized afternoon resumption. -->
야간 작업은 마지막 시험이 **05:56 KST**에 종료된 뒤 중단되어 오전 10시까지 연속으로
진행되지 못했습니다. 사용자 요청으로 **13:53 KST 이후 재개**했습니다. 원래 계획과
완료 기록은 보존하고, 미실행 페달 비교 6개에만 별도 재개 기록과 운영상 시간 상한
17:00 KST를 적용합니다. 시험 순서·반복 수·제어·판정 기준은 바꾸지 않습니다.

<!-- HH_260906 - Put the actual latest execution state before the long historical ledger; scalar success is not admission. -->
**최신 실행 상태:** C-track 출발 페달 비교 8회는 **14:11 KST에 모두 종료**했고,
각 실행이 사용한 CARLA 프로세스와 포트 정리도 확인했습니다. 목표 정차는 8회 모두
도달했으며, 20 Hz 가감속 기준은 페달 0.13의 두 반복만 충족했습니다. 나머지 6회는
출발 가속 초과로 실패 자료를 보존합니다. 전체 JPEG 46,416장과 미래 창 7,216개를
검사했으며, 미래 XY 곡률은 1,560개 창에서 기준을 넘었습니다. 0.13도 각 반복의
905개 창 중 161개가 곡률 초과이므로 **8회 모두 학습 데이터로 승인하지 않았습니다**.
창은 서로 겹치므로 초과 창 수를 독립적인 주행 사건 수로 해석하지 않습니다.
이는 목표 14.4 km/h의 expert 개발 시험이며, 30 km/h 검증이나 학습 데이터 승인이 아닙니다.
원격 GPU0은 14:08 KST 확인 시 비어 있었으며 새 학습 작업은 시작하지 않았습니다.

<!-- HH_260906 - Separate final unit-regression evidence from expert driving and model qualification. -->
앞선 코드 회귀 검사는 일반 테스트 **4,145 passed / 6 skipped**, Autoware 런치 패키지
**373 passed / 0 skipped**로 종료했습니다. 합계 **4,518 passed / 6 skipped**이며,
초기 실행 실패·수정 근거·건너뛴 이유·검사한 소스 해시는 [23 검증 기록](23_code_validation_final/README.md)에 보존합니다.
이는 코드 검사 결과이며 실제 Autoware 주행 성공이나 새 모델 학습 완료를 뜻하지 않습니다.

<!-- HH_260906 - Distinguish the completed later diagnostic and final-source regression from the earlier preserved code-validation snapshot. -->
**후속 완료:** [24 미세 이동 분석](24_micro_motion_study/README.md)은 15:28 KST에
원본 구간 30,908개·미래 지점 461,824개를 처리했습니다. 표시를 보완한 PNG 3장과
최종 코드 검사 **4,577 passed / 6 skipped**를 추가했습니다. 기존 곡률 실패 1,560개 창은
그대로이며 새 라벨·모델 학습·VisionPilot 실행은 하지 않았습니다.

<!-- HH_260906 - Separate the later synthetic stop representation and local prerequisite discovery from real driving and training. -->
**18:27 KST 후속 완료:** [25 정지 출력 구조](25_stop_primitive_research/README.md)의
고정 합성 후보 **600/600**이 기존 출력 검증과 독립 수식 대조를 통과했습니다.
정지 판단을 학습한 모델이나 실제 주행은 아니며 원본 데이터 미승인은 유지합니다.
[26 VisionPilot 사전 검사](26_visionpilot_preflight/README.md)도 실행했으며, 현재 로컬
ONNX Runtime 미발견·센서/출력 차이 등으로 모델 추론은 아직 준비되지 않았습니다.
이번 두 작업에서 원격 서버·GPU 작업·설치·기존 모델 교체는 하지 않았습니다.
차량 기준점 중심의 합성 궤적·감속 PNG 2장과 모든 후보의 수치를 25 폴더에 게시했습니다.
그림 표시 보완까지 반영한 최종 전체 코드는 **4,795 passed / 6 skipped**이며,
[실제 명령·로그·소스 해시](25_stop_primitive_research/verification.json)를 보존합니다.
앞선 23·24 검사 횟수와 합산하지 않으며 이번에도 실제 주행·모델 학습 완료는 아닙니다.

| 카테고리 | 내용 | 현재 판정 |
|---|---|---|
| [01 정답 데이터의 감속 한계 분석](01_target_feasibility/README.md) | 기존 train/val의 실제 속도·XY를 모델의 그대로인 한계와 대조 | 진단 완료; 원본 라벨 유지 |
| [02 경로 생성기 고정·선택기 학습](02_frozen_selector/README.md) | C 생성기 3개 각각에 선택기 3종, 총 9회 추가 학습·고정 시점 경로 PNG | 전부 완료, 세 방식 모두 종합 상대·절대 FAIL |
| [03 초기 자연 정지 수집 보정](03_goal_stop_calibration_initial/README.md) | Town07 두 시도의 실제 속도·가감속·목표 정지 분석 | 두 번째 목표 정지는 성공, 두 시도 모두 데이터 품질 FAIL |
| [04 고정 페달 저속 응답 계측](04_pedal_response_calibration/README.md) | 일정 가속 6개·제동 6개, 실제 20 Hz 상태 4,176개 | 12개 계측 완료; 제동 6개 모두 저속 급정지, 제어·데이터 승인 아님 |
| [05 초기 코드 전체 회귀 검사](05_code_validation_initial/README.md) | 선택기·진단·수집 보정·고정 페달 계측 v1 포함 원본 로그 | 2,633 passed / 6 skipped |
| [06 Warmup·과거 입력 감사](06_warmup_history_audit/README.md) | v3 train/val 앵커 phase·history와 원본 30개 해시 | warmup train105·val35 포함 확인; 과거 문서 정정, 정책 미채택 |
| [07 타력 주행·출발 반복·점진 가속](07_coast_ramp_identification/README.md) | 고정한 9개 조건, 실제 상태 2,742개·그래프 3장 | 계측 완료; 타력 20초는 미정지, 움직인 ramp 3개 모두 가속 초과 |
| [08 확장 타력 주행·충돌 실패 보존](08_extended_coast_partial_failure/README.md) | 예정 6개 중 완료 2·충돌 실패 1·미실행 3, 실제 상태 2,451개 | 전체 실패; 충돌 구간·출발 초과 모두 보존 |
| [09 가속도 입력 기준점·연구용 모델](09_acceleration_input_contract/README.md) | 기존 입력 정의 감사·실제 수치 그래프·가속도 값 제외 코드 | 131 passed / 5 skipped; 새 ID 실제 학습·배포는 미실행 |
| [10 Town07 순항·자연 정지 재시험과 실제 화면](10_comfortable_v3_pilot/README.md) | 두 번 전체 계측, 차량 중심 경로·6개 카메라 PNG 12장·GIF 2개 | 스칼라 1회 충족/1회 실패; 제어 기록 정합성은 모두 실패·미승인 |
| [11 제어 수신·관측 정합성 비교](11_acknowledged_control_protocol/README.md) | 기존 async 2회·새 ACK 2회 모두 비교, 원본 카메라·차량 중심 경로 PNG/GIF | ACK 정합성 2/2 충족; 접근 급감속은 2/2 실패, 미승인 |
| [12 같은 위치의 Low/Epic 실제 화질 비교](12_stationary_camera_quality/README.md) | 정지 상태 320개·원본 JPEG 48장 검증, 전방 PNG 8장·6카메라 비교표 4장 | Low 체크무늬/Epic 아스팔트·차선 확인; FPS·주행·학습 승인 아님 |
| [13 원본 영상·저속 곡률 진단](13_raw_pre_admission_diagnostics/README.md) | 초기 v3 JPEG 8,838장·전체 미래 1,343개, 실제 곡률·위치 변화 분석 | 저속 XY 곡률 불일치 유지; 원인 단정·라벨 수정 없음 |
| [14 정상 브레이크 제거 A/B와 실제 화면](14_brake_free_goal_stop/README.md) | 기존 ACK 실패 2회 + 정상 brake=0 반복 2회, 실제 PNG 13장·GIF 2개 | V4 두 회 모두 스칼라·정지·제어 기록 충족; 데이터 미승인 |
| [15 데이터 기준점과 남은 궤적 문제](15_data_reference_and_raw_geometry/README.md) | V4 JPEG 8,802장·미래 1,337개 전체 검사, 바퀴·virtual base 기준 대조 | 저속 곡률 실패 유지; 좌표 기록 재현과 물리 기준점 승인은 별개 |
| [16 Low/Epic 주행 처리 시간과 실제 화면](16_wall_timing_quality/README.md) | 같은 Town07 경로 각 1회, 모든 tick·카메라·ACK, PNG 13장·GIF 2개 | 수집 묶음 벽시계 36.5/40.6 Hz; GUI FPS·추론 측정이나 전체 끊김 해결 주장은 아님 |
| [17 디코더 표현 가능성과 정지 직전 출력](17_decoder_representability/README.md) | GPU0에서 고정 6초기값, 1,337개 시작점·8,022개 후보 수치 근사와 독립 검사 | 학습 모델 가중치 변경 없음; 34개 출력 실패·22개 초기 계산 불일치 유지 |
| [18 C-track 저속 좌회전과 실제 화면](18_c_track_low_speed_turn/README.md) | 별도 목표 14.4 km/h, 1,907개 native 상태·5,724개 영상, PNG 7장·GIF 1개 | 목표 정차 완료; 출발 한 구간 +3.654933 m/s² 초과로 실패·미승인 |
| [19 최초 프레임 이후 제어기 초기화 비교](19_agent_initialization_comparison/README.md) | 생성·첫 명령·관측까지 독립 프레임/ACK 검사, PNG 8장·GIF 1개 | 첫 조향 거의 0으로 정상화; 출발 +3.467811 m/s² 초과는 유지 |
| [20 두 CPU의 최종 디코더 계산 대조](20_decoder_forward_verification/README.md) | 각 환경 1,540,224개 저장 출력 비교, 원격 초기 검사 부분 실패도 보존 | 최종 출력 허용량 내 일치; 초기 목적함수 22개 미확인·기존 게이트 실패 유지 |
| [21 C-track 출발 페달 8회와 실제 화면](21_turn_launch_matrix/README.md) | 15,466개 상태·46,416장 영상, 실제 6카메라·차량 중심 PNG 26장·출발 GIF 8개 | 정차·ACK·초기화 8/8, native 가감속 2/8 충족; 학습 미승인 |
| [22 C-track 전체 미래와 미세 변위 분석](22_turn_launch_raw_geometry/README.md) | 전체 7,216개 미래 창·8회 실제 궤적·저변위 곡률 그래프 3장 | 곡률 초과 1,560개 창 유지; 0.13도 미승인, 원본 수정 없음 |
| [23 앞선 코드 회귀와 실패 원인](23_code_validation_final/README.md) | 일반·런치 검사 원본 로그, 초기 실패 보존, 부동소수점 비교 보완과 소스 해시 | 당시 합계 4,518 passed / 6 skipped; 실제 차량·학습 모델 승인 아님 |
| [24 미세 이동의 위치·속도·방향 분석](24_micro_motion_study/README.md) | 원본 구간 30,908개·미래 지점 461,824개 전체 분석과 실제 수치 그래프 | 계산 완료; 기존 초과 창 1,560개 유지, 라벨 보정·학습 미실행 |
| [25 재가속 없는 정지 출력 연구](25_stop_primitive_research/README.md) | 별도 연구용 구조, 고정 합성 100조건·600후보·독립 수식 대조 | 합성 출력 검사 600/600; 정지 의도 학습·실제 주행 아님 |
| [26 VisionPilot 환경·입출력 사전 검사](26_visionpilot_preflight/README.md) | 설치 없는 로컬 모듈·라이브러리 발견과 선언한 센서/과제/출력 차이 | 사전 검사 완료, 모델 추론·10 Hz·연동은 미검증 |

## 두 환경에서 실제로 한 일

<!-- HH_260906 - Explain model ownership and distinguish researching an external baseline from replacing the independent planner. -->
현재 학습 연구는 VAD 재학습이 아닌 **자체 Portable E2E 모델 개발**입니다.
VAD는 기존 Autoware/CARLA 기준선이며, VisionPilot은 공통 기능에서 비교할 외부 후보입니다.
[세 모델의 역할과 공정한 비교 순서](../../../../portable-e2e-visionpilot-comparison.md)에
공식 소스·센서/출력 차이·설치 제한을 정리했습니다. VisionPilot 다운로드·추론·학습·주행
실행이나 기존 모델 교체는 아직 하지 않았습니다.

- **원격 Pro6000:** 기존 개인 py312 venv와 GPU0만 사용해 선택기 전용 학습 9회를
  완료했습니다. 학습 시간은 01:29:13–01:32:00 KST입니다. 이는 전체 모델을 처음부터
  9번 학습한 것이 아닙니다. 이후 05:05:03–05:07:27 KST에 GPU0에서 고정 decoder의
  잠재 입력만 수치 최적화했습니다. 이는 미래 정답을 사용하는 표현 가능성 진단이며
  모델 가중치 학습·주행 성능이 아닙니다. 두 작업은 모두 종료됐고, 자동 반복 학습은 없습니다.
  시스템 설치·Conda 변경·GPU1
  사용·다른 작업 종료·재부팅·CARLA/Autoware 빌드는 하지 않았습니다.
- **로컬:** 원본 결과 검증, 계측 코드와 그래프·경로 PNG 정리, Town07 직진 BasicAgent
  수집 제어 보정을 진행합니다. 이 BasicAgent는 학습 정답을 만드는 expert입니다.
  새 Portable 모델이나 Autoware VAD가 이 수집 차량을 운전한 것이 아닙니다.
  이후 원인 분리를 위해 **일정 페달 12개와 타력·반복·ramp 9개, 총 21개 계측**도 완료했습니다.
  확장 타력·가속 전환 6개 시험은 2개 완료 뒤 1개 충돌로 실패해 나머지 3개를 실행하지 않았습니다.
  이어 조향을 유지한 `comfortable_v3` 전체 경로 재시험 두 번과 실제 화면 기록을 완료했습니다.
  수신 확인 방식 2회와 정상 브레이크만 제외한 2회도 완료했습니다. 후자는 두 번 모두
  기존 속도·정지·제어 기록 검사를 충족했지만, 후속 전체 미래 검사에서 저속 XY 곡률
  문제가 남아 데이터로 채택하지 않았습니다. 별도의 warmup/history 감사는 원본
  메타데이터·코드만 읽었고 새 데이터를 만들지 않았습니다.

## 현재 원인과 다음 순서

기존 정답 데이터에는 모델이 허용하는 감속보다 급한 속도·위치 변화가 있습니다.
6.4초 미래 창을 검사하면 train 1,147개 중 속도 라벨 294개 / 독립 XY 263개,
val 337개 중 각각 108개가 감속 한계를 넘습니다. 서로 겹치는 창의 수이며 독립적인
급정지 횟수가 아닙니다. 최종 강제 제동뿐 아니라 그 이전의 주행에도 문제가 있습니다.

선택기만 개선하는 통제 실험에서도 모든 seed에 걸친 개선은 확인하지 못했습니다.
따라서 좋은 seed 하나를 골라 배포하지 않고, 먼저 정답을 만드는 주행 제어를 개선합니다.

<!-- HH_260906 - Attribute the shared selected-path rejection to its measured input-speed precondition without discarding the sample or relaxing the gate. -->
02의 원래 C 3개·학습 head 9개는 모두 selected gate `336/337`이며, 유일한 selected 실패는
val index108의 `speed`입니다. 이 원본 sample의 현재 종방향 속도는 `8.3409921056 m/s`
(`30.02757158 km/h`)로 gate의 `8.3333333333 m/s`보다 높습니다. 실행 소스는 이 입력
조건을 경로 형상 검사 전에 거절합니다. 따라서 이 공통 실패를 곧바로 새 head의 경로 모양
결함으로 해석하지 않습니다. 거절 뒤의 형상까지 통과한다는 뜻도 아니며 판정은 유지합니다.
새 수집은 명령값만 아니라 실제 입력 속도도 계약 안에 드는지 확인해야 합니다.
[실제 geometry 예시](02_frozen_selector/routes/seed_20260903/original_c_same_cache/geometry.json)의
원본 SHA는 02 manifest에, sample JSONL SHA는
[06 원본 출처](06_warmup_history_audit/provenance.json)에 있습니다.
당시 실행 commit과 동일한 `portable_e2e/runtime_contract.py` SHA-256은
`38e993278ef84b149efc90931423cd90b1562d86c9eb1585260d50e03b2ae0d3`입니다.

Town07 동일 학습 경로의 첫 두 보정 시험은 모두 실패 자료로 보존했습니다.
첫 번째는 목표 1.84 m 앞에서 BasicAgent가 완료를 선언했고, 출발 가속도도 초과했습니다.
두 번째는 실제 목표 0.97 m 앞 정지·2초 유지·정지 후 꼬리 기록까지 됐지만, 정상 제동
중 저속 급감속과 출발 가속 때문에 품질 검사에 실패했습니다. 두 번째 실제 최고 속도는
약 16.7 km/h로, 명령값 30 km/h와 구별합니다. **둘 다 학습 데이터로 채택하지 않았습니다.**

<!-- HH_260906 - Separate completed fixed-pedal measurements from pending controller qualification and unadopted history policy. -->
[고정 페달 분리 계측](04_pedal_response_calibration/README.md)은 가속 6개·제동 6개를
완료했습니다. 일정한 페달에서도 출발 지연·순간 가속과 저속 급정지가 남았으며, 제동
여섯 단계의 최소 가속은 약 −16.32 ~ −24.63 m/s²였습니다. 이전 제어기의 페달 전환만을
원인으로 단정할 수 없습니다. 내부 엔진·바퀴 RPM이나 타이어 상태는 측정하지 않아 특정
물리 로직을 원인으로 확정하지 않았습니다. 이 계측은 30 km/h 주행이나 데이터 품질 PASS가 아닙니다.

<!-- HH_260906 - Include every predeclared coast/ramp case and retain preparation-phase violations. -->
[후속 9개 계측](07_coast_ramp_identification/README.md)에서 타력 구간은 급정지 없이
감속했지만, 20초 후에도 0.1864 m/s가 남았습니다. 고정 throttle 0.15 반복은 같은 응답을
보였으나 가속 한계까지 여유가 작고, 움직인 ramp 세 가지는 가속 초과를 없애지 못했습니다.
긴 타력 주행·측정 속도 기반 두 단계 출발 시험은 별도 조건으로
[08 결과](08_extended_coast_partial_failure/README.md)에 분리했습니다.

<!-- HH_260906 - Preserve both moving pilot outcomes and the newly detected control-observation alignment failure. -->
[Town07 전체 경로 재시험](10_comfortable_v3_pilot/README.md) 두 번은 각각 최고
28.55·28.61 km/h, 연속 순항 8.65·8.60초, 목표 앞 0.83·0.85 m 정지를 기록했습니다.
첫 번째는 접근 중 한 구간의 급감속으로 스칼라 검사에 실패했고 두 번째는 충족했습니다.
그러나 API가 보고한 제어값과 직전 요청의 불일치가 각각 79·85개 발견되어 **둘 다
학습 데이터로 채택하지 않았습니다.** 실제 물리 지연인지 관측 시점 문제인지는 아직
입증되지 않았습니다. 다음은 명령 수신 확인·동일 프레임 관측의 전송/계측 비교입니다.

<!-- HH_260906 - Record completed protocol outcomes separately from the following one-parameter brake experiment. -->
[수신 확인 방식의 후속 두 실행](11_acknowledged_control_protocol/README.md)은 독립 검사에서
제어 불일치가 모두 0건이었지만, 접근 중 최소 속도 변화율이 −8.86·−7.73 m/s²로
여전히 기준을 넘었습니다. 두 실행 모두 목표 정지에는 도달했지만 학습 데이터로 채택하지
않았습니다. 이후 정상 brake 상한만 0.10에서 0.00으로 바꾸는 `comfortable_v4` 비교를
별도로 시작했습니다. 비상 제동·조향·정지 조건·품질 기준은 유지합니다.

[V4 실제 반복 결과](14_brake_free_goal_stop/README.md)는 두 번 모두 스칼라 검사와
목표 정지를 충족했고, 기존 두 ACK 실패도 함께 보존했습니다. 그러나 [전체 원본 추가
검사](15_data_reference_and_raw_geometry/README.md)에서 저속 XY 곡률 초과가 각각
145/671개, 154/666개 미래 시작점에 남았습니다. 8,802장의 영상과 전체 85,568개 미래
점을 포함했으며, 좋은 구간만 골라 데이터로 승인하지 않았습니다. 이후 Epic 수집의
실제 처리시간과 원본에 대한 디코더의 표현 오차를 [16](16_wall_timing_quality/README.md)·
[17](17_decoder_representability/README.md)에서 분리 계측했습니다.

<!-- HH_260906 - Distinguish the recorded low-speed failure from the prospective initialization-order experiment. -->
[C-track 저속 좌회전](18_c_track_low_speed_turn/README.md)은 목표 정차·제어 정합성을
충족했지만, 실제 20 Hz 출발 한 구간의 가속도 초과로 실패했습니다. 첫 조향 요청이
−0.8에서 시작한 점과 BasicAgent가 최초 bootstrap tick 전에 생성되는 코드를 확인했습니다.
[후속 비교](19_agent_initialization_comparison/README.md)는 기존 tick 이후 제어기를 생성하고
실제 프레임·제어·PID 이력을 기록했습니다. 초기 조향은 정상화됐지만 출발 가속도는
여전히 실패했습니다. PID 값을 강제로 초기화하거나 실패 구간을 제거하지 않았습니다.
생성자의 위치·조향 등 여러 초기 관측 시점이 함께 바뀌므로 조향 이력만의 인과 효과로
해석하지 않습니다. 기존 기본 동작·물리 한계·학습 승인 상태는 유지합니다.

이후 출발 페달 0.12·0.13·0.14·0.15를 각 두 번, 미리 정한 순서로 비교했습니다.
출발 이후의 상한 변화는 모두 0.15에서 같은 비율로 시작합니다. 낮은 페달은 이 전환에서
더 큰 명령 차이를 만들 수도 있으므로 출발뿐 아니라 전환·전체 회전·종점 정차까지 검사했습니다.
8회 모두 전환 주변 가속은 기준 안에 있었고 실패한 6회의 초과는 그 이전 출발에 있었습니다.
원본 20 Hz 가감속·정지·제어 정합성은 0.13의 두 반복만 모두 충족했지만, 전체 6.4초
미래 XY 곡률 검사는 이 두 반복도 실패했습니다. 결과에 따라 반복 횟수를 늘리거나
좋은 구간만 골라 학습에 넣지 않았으며, 동일 경로·seed의 반복을 독립적인 환경 검증으로
세지 않습니다. 추가 원인 분리와 새 데이터 승인 절차가 필요합니다.

<!-- HH_260906 - Describe low-displacement conditioning without changing physical limits, target masks or historical verdicts. -->
[22의 추가 집계](22_turn_launch_raw_geometry/README.md)에서 곡률 위반은 모두 XY 위치
차분으로 계산한 구간 평균속도 1 m/s 미만에 있었습니다. 0.13의 각 반복은 겹치는
미래 창을 포함한 위반 지점 3,545개 중 3,544개가 0.1 m/s 미만이며, 고유 100 ms
target tick은 64개입니다. 최대 곡률 사례는 warmup의 약 0.125 mm 변위입니다.
이처럼 작은 변위에서는 방향 기반 곡률이 불안정해질 수 있으나, 센서 노이즈나 특정
물리 엔진 동작이 원인이라는 증명은 아닙니다. 첫 미래점 이후에도 위반이 있으므로
첫 점만 고치면 해결됐다고 하지 않습니다. 기존 라벨·warmup·tail·판정 기준은 유지합니다.

다음 수정은 출발 가감속과 별도로 **정지/미세 이동의 궤적 표현·계측 기준을 분리 검증**하는
단계입니다. 기준을 바꾸거나 라벨을 보정하려면 별도 설계·버전·검증이 필요하며, 기존
미승인 표시를 지워 학습에 넣지 않습니다. 이후 독립 경로 수집·데이터 버전 검증·전송을
거쳐 고정 조건으로 다시 학습합니다. GPU가 비어 있다는 이유만으로 새 학습이 실행되지는 않습니다.

<!-- HH_260906 - Record the completed CPU-only study without authorizing label replacement or a new capture retry. -->
[후속 CPU 분석](24_micro_motion_study/README.md)은 **15:28 KST에 완료**했습니다.
기존 8회 원본의 20 Hz 위치 차이와 기록 속도의 적분을 비교하고,
전체 미래 인덱스에서 0.1·0.2·0.5초 변위 방향의 민감도를 확인했습니다.
API 기준점과 변환된 후륜 기준을 구분하며 어느 쪽도 물리 무게중심이라고 가정하지 않습니다.
긴 시간 간격에서 지표가 안정되더라도 기존 0.1초 곡률 실패를 지우지 않습니다.
추후 제한을 지키는 명목 궤적과 원본의 잔차를 별도 표현으로 연구하되, 이번 실패에 맞춰
오차 허용폭을 정하거나 실제 미속 이동을 정지로 숨기지 않는 검증이 먼저입니다.
계산은 완료됐지만 표현 변경은 아직 채택하지 않았으며, 새 모델 학습도 시작하지 않았습니다.

<!-- HH_260906 - Keep the failed visual-only publication attempt recoverable and separate from the eight driving cases. -->
21 화면의 첫 생성은 상대경로/절대경로 바인딩 오류로 첫 시도의 PNG 3장·GIF 1개 생성 후
실패했습니다. 당시 소스 `17dd947…`와 부분 출력은 private
`artifacts/training/2026-09-08/turn_launch_matrix_publication_partial_v1`에 보존했습니다.
경로 처리만 수정한 `b62b05e…`로 새 공개 폴더를 생성했으며, 원본 주행은 재실행하지
않았습니다. 이 화면 생성 재시도를 아홉 번째 주행으로 세지 않습니다.

[Warmup 감사](06_warmup_history_audit/README.md)에서는 v3 train 1,147개 중 105개,
val 337개 중 35개가 warmup 앵커임을 확인했습니다. 출발 직후 첫 9개 주행 앵커가 warmup
상태를 과거 입력으로도 사용합니다. 과거 문서의 제외 설명만 정정했고, history-only 방안은
새 인덱스·문맥 저장소 계약이 필요한 미채택 제안으로 남겼습니다.

<!-- HH_260906 - Keep the untrained acceleration-value ablation separate from a physical input correction or deployed model. -->
[가속도 입력 감사](09_acceleration_input_contract/README.md)에서는 기존 수집 가속도의
기준점과 실시간 가속도 생성 방식도 비교했습니다. 수집기의 가속도만 수정하지 않고,
가속도 두 입력값의 영향만 제외하는 별도 연구 모델 코드를 준비했습니다. 기존 데이터·
모델 ID·배포 bundle은 유지하며, 새 ID는 현재 secure runtime bundle이 지원하지 않습니다.
실제 비교 학습은 동일한 새 검증 corpus와 고정 seed 조건을 먼저 마련한 뒤 진행합니다.

다음은 완료한 고정 페달 계측에 근거한 저속 응답 원인 추가 분리·수집 제어 개선 →
30 km/h 직진의 자연스러운 정지 재검증 → 회전·독립 episode 확장 →
새 데이터 버전 검증·전송 → 고정 조건 재학습·평가 순서입니다.
원래 데이터·test·기준값은 덮어쓰거나 결과에 맞춰 완화하지 않습니다.

## 자료 해석과 보존

01의 PNG는 실제 JSON 수치를 그린 그래프, 02의 경로 PNG는 고정 입력에 대한
오프라인 예측입니다. 둘 다 CARLA/Autoware 화면 녹화나 학습 모델의 폐루프 주행 영상이
아닙니다. 실제 카메라 자료를 추가할 때도 expert 수집 영상인지 모델 제어 영상인지 구분합니다.
04·07의 PNG도 실제 상태 수치로 그린 그래프입니다. 두 계측의 10 Hz 표시는 20 Hz 상태열의
짝수/홀수 간격 추출이며 카메라 cadence나 실시간 모델 성능을 뜻하지 않습니다.
06은 메타데이터 감사이며 촬영·모델 추론 자료가 아닙니다.
09의 PNG는 가속도 내부 일관성 감사 수치이며 모델 성능 개선이나 실제 주행 결과가 아닙니다.
08도 카메라를 사용하지 않은 실제 계측 그래프입니다. 10은 원본 카메라와 관측 궤적의
합성 대시보드이며, 라이브 Autoware 화면이나 학습 모델의 폐루프 주행 영상이 아닙니다.
10의 체크무늬 도로는 원래 Low 렌더링 영상에 남아 있으며 별도 시각 품질 검사 대상입니다.
12는 같은 시작점에 차량을 정지시켜 실제 촬영한 화질 비교입니다. 해당 조건에서는 Epic의
도로 표면과 차선 표시를 확인했지만, 주행 성능·프레임 속도나 특정 엔진 내부 원인까지
입증하지 않습니다. 원본 JPEG를 디코딩한 픽셀을 그대로 PNG에 보존했습니다. 계획 JSON의
기재 시각 오류는 해당 보고서에 공개했고, 엄격한 사전등록 검증 통과로 표시하지 않았습니다.

전체 corpus 형식·hash 무결성 검사는 test 파일도 읽을 수 있습니다. test를 모델 학습·
성능 평가·선택에 사용하지 않았다는 것과 파일을 전혀 읽지 않았다는 것은 다릅니다.
선택기 학습에서는 전자만을 주장합니다. 01의 별도 라벨 진단은 test 에피소드 메타데이터로
분할만 확인하고 test 샘플·이미지·궤적 통계는 열지 않았습니다.

대용량 checkpoint·캐시·실패 raw 수집은 private artifacts에 보존하고 Git에는 넣지 않습니다.
공개 metadata의 개인 경로 치환 여부와 원본 SHA는 각 카테고리에 기록합니다.

[전날 결과](../../2026-09-07/portable_e2e_learning_cycle_v1/README.md) ·
[선택기 실험 설계](../../../../portable-e2e-frozen-selector.md) ·
[로컬 정지 보정 실행 안내](../../../../portable-e2e-natural-goal-stop.md) ·
[두 PC의 운용 순서](../../../../portable-e2e-learning-loop.md) ·
[9개 상위·30개 하위 기능표](../../../../portable-e2e-feature-roadmap.md)
