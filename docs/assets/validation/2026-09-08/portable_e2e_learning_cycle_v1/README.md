# Portable E2E — 선택기 분리 학습과 자연스러운 정지 데이터 개선

<!-- HH_260906 - Keep the overnight work ledger separate from model deployment and expert driving evidence. -->

2026-09-08 작업 자료입니다. 사용자가 요청한 작업 경계는 **오늘 오전 10시 KST**입니다.
아래는 진행 중인 기록이며, 이 문서가 학습 모델의 차량 제어 승인이나 모든 기능 완료를
뜻하지 않습니다. 기존 shadow 운용 checkpoint는 유지합니다.

| 카테고리 | 내용 | 현재 판정 |
|---|---|---|
| [01 정답 데이터의 감속 한계 분석](01_target_feasibility/README.md) | 기존 train/val의 실제 속도·XY를 모델의 그대로인 한계와 대조 | 진단 완료; 원본 라벨 유지 |
| [02 경로 생성기 고정·선택기 학습](02_frozen_selector/README.md) | C 생성기 3개 각각에 선택기 3종, 총 9회 추가 학습·고정 시점 경로 PNG | 전부 완료, 세 방식 모두 종합 상대·절대 FAIL |
| [03 초기 자연 정지 수집 보정](03_goal_stop_calibration_initial/README.md) | Town07 두 시도의 실제 속도·가감속·목표 정지 분석 | 두 번째 목표 정지는 성공, 두 시도 모두 데이터 품질 FAIL |
| [04 고정 페달 저속 응답 계측](04_pedal_response_calibration/README.md) | 일정 가속 6개·제동 6개, 실제 20 Hz 상태 4,176개 | 12개 계측 완료; 제동 6개 모두 저속 급정지, 제어·데이터 승인 아님 |
| [05 초기 코드 전체 회귀 검사](05_code_validation_initial/README.md) | 선택기·진단·수집 보정·고정 페달 계측 v1 포함 원본 로그 | 2,633 passed / 6 skipped |
| [06 Warmup·과거 입력 감사](06_warmup_history_audit/README.md) | v3 train/val 앵커 phase·history와 원본 30개 해시 | warmup train105·val35 포함 확인; 과거 문서 정정, 정책 미채택 |
| [07 타력 주행·출발 반복·점진 가속](07_coast_ramp_identification/README.md) | 고정한 9개 조건, 실제 상태 2,742개·그래프 3장 | 계측 완료; 타력 20초는 미정지, 움직인 ramp 3개 모두 가속 초과 |
| [09 가속도 입력 기준점·연구용 모델](09_acceleration_input_contract/README.md) | 기존 입력 정의 감사·실제 수치 그래프·가속도 값 제외 코드 | 131 passed / 5 skipped; 새 ID 실제 학습·배포는 미실행 |

## 두 환경에서 실제로 한 일

- **원격 Pro6000:** 기존 개인 py312 venv와 GPU0만 사용해 선택기 전용 학습 9회를
  완료했습니다. 학습 시간은 01:29:13–01:32:00 KST입니다. 이는 전체 모델을 처음부터
  9번 학습한 것이 아닙니다. 완료 후 GPU0가 비어 있음을 확인했으며, 새 작업을 시작하기
  전까지 자동으로 계속 학습하는 프로세스는 없습니다. 시스템 설치·Conda 변경·GPU1
  사용·다른 작업 종료·재부팅·CARLA/Autoware 빌드는 하지 않았습니다.
- **로컬:** 원본 결과 검증, 계측 코드와 그래프·경로 PNG 정리, Town07 직진 BasicAgent
  수집 제어 보정을 진행합니다. 이 BasicAgent는 학습 정답을 만드는 expert입니다.
  새 Portable 모델이나 Autoware VAD가 이 수집 차량을 운전한 것이 아닙니다.
  이후 원인 분리를 위해 **일정 페달 12개와 타력·반복·ramp 9개, 총 21개 계측**도 완료했습니다.
  별도의 warmup/history 감사는 원본 메타데이터·코드만 읽었고 새 데이터를 만들지 않았습니다.

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
다음 긴 타력 주행·측정 속도 기반 두 단계 출발 시험은 별도 조건이며, 이 결과에 포함하지 않습니다.

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
