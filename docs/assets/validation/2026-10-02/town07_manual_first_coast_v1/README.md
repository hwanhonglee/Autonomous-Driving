# 수동 1단에서 브레이크 없이 감속했을 때

<!-- HH_261002 - Separate the measured zero-pedal response from goal-stop training, internal physics causation and unmatched AUTO comparisons. -->

**이번 한 번의 실제 계측에서는 급감속 품질 위반이 없었다.** 같은 Town07·Prius에서 수동 1단을 유지하고 가속·브레이크·조향을 모두 0으로 둔 45초를 전량 확인했다. 이전 목적지 정지 v1/v2의 급감속을 “수동 1단이면 무조건 생기는 문제”로 설명할 수는 없다는 근거다. 그러나 이것만으로 내부 물리 원인을 확정하거나 목적지 정지 기능을 승인하지 않는다.

로컬 CARLA의 **차량 응답 계측**이다. 학습 모델·BasicAgent·PID·카메라를 사용하지 않았으며 학습 자료로 승인하지 않았다. 정식 완료는 **메인 0/9, 하위 0/30** 그대로다.

## 실제 결과

| 확인 항목 | 전량 원자료에서 확인한 값 |
| --- | --- |
| 코스트 진입 속도 | 첫 관측 1.528009m/s. 1.5로 덮어쓰거나 잘라내지 않음 |
| 무페달 구간 | frame 1658–2557, 900틱, 실제 45.00000067초 |
| 실제 제어·ACK | 900틱 모두 throttle/brake/steer=0, manual=True, gear=1 |
| 45초 동안 전진 | 경로 기준 10.960053m; actor 실제 위치 누적 10.961131m |
| 종료 속도 | 0.021375m/s. 정확히 0이라는 뜻은 아님 |
| 코스트만의 속도 변화율 | −1.014323 ~ +1.028612m/s², 위반 0 |
| 준비 포함 속도 변화율 | −1.981288 ~ +2.474315m/s², 1,060구간 전부 ±2.9 및 +3/−6 기준 이내 |
| 차체·10cm 여유 | 준비·코스트·정리 1,062상태와 1,061 snapshot sweep 전부 통과; inset 최소 여유 0.334806m |
| 기록·소유권 | 일반 1,061상태 + 별도 정리 1상태, 실제 ACK 1,062개, 오류·충돌·침범 0; 원 소스·자료 SHA와 종료/포트 해제 확인 |

속도 변화율은 원 collector의 **rear-base `hypot(vx,vy)` 차분**이다. actor world 속도나 CARLA의 가속도 벡터 API와 같은 측정이라고 부르지 않는다. 진입 actor world 속도도 1.528009m/s로 확인했다. snapshot sweep 통과는 연속 물리 시간의 모든 위치를 증명한 것은 아니다.

**정지 판정에서 중요한 차이:** 속도 ≤0.05m/s를 처음 관측한 것은 코스트 시작 후 22.45초였지만 이후 작은 재상승이 있었다. 처음으로 **연속 40구간·2초**를 유지한 구간은 **26.25→28.25초**다. 그동안 경로 전진은 10.707385→10.769872m였다. 자연 감속이 완전히 단조롭거나 22.45초에 영구 정지했다고 쓰면 안 된다.

측정 900틱이 끝난 **다음 frame 2558**에서만 정리용 full brake를 적용했다. 이 별도 상태와 ACK도 보존했다. 준비는 bootstrap 1틱·정지 warmup 24틱·명시한 open-loop 출발 136틱이며, 불리한 출발 구간을 빼서 품질을 통과시키지 않았다.

## 무엇을 알았고, 아직 무엇을 모르는가

- 같은 manual1 설정에서도 **정상 brake=0**일 때 이번 저속 감속은 품질 범위 안이었다. [목적지 정지 v2](../town07_natural_goal_stop_teacher_v2/README.md)의 brake 0.01·가속/제동 반복과 실제 속도 급락은 그대로 실패 증거다.
- ECU 내부 토크·엔진/바퀴 RPM·PhysX 필터·타이어 sticky 상태는 이 API로 측정하지 않았다. “브레이크가 모든 원인의 유일한 원인”이라고 단정하지 않는다.
- [이전 AUTO 코스트·고정 브레이크·브레이크 없는 종점 결과](PRIOR_EVIDENCE.md)는 다른 진입 속도와 준비 제어를 썼다. AUTO 대 manual1의 정확히 맞춘 A/B가 아니며, 이전 준비 실패와 전체 캠페인 실패도 그대로 남긴다.
- **1.528m/s에서 측정한 약 11m를 2.4m/s 정지거리로 외삽하지 않는다.** 다음 종점 교사는 새로운 사전 선언에서 실제 진입 속도·거리와 일회성 coast latch를 정하고, 다시 가속하지 않는 감속을 검증해야 한다. 아직 그 교사를 구현·실행하거나 이 자료에 TRAIN 권한을 부여한 것은 아니다.

## 재현 근거

[선언](../../../../../storage/experiments/validation/2026-10-02/town07_manual_first_zero_pedal_coast_v1/declaration.json) · [원 결과](../../../../../storage/experiments/validation/2026-10-02/town07_manual_first_zero_pedal_coast_v1/calibration/result.json) · [독립 원자료/ACK·차체·소유권 진단](../../../../../storage/experiments/validation/2026-10-02/town07_manual_first_zero_pedal_coast_v1/diagnostic_report.json) · [전체 상태](../../../../../storage/experiments/validation/2026-10-02/town07_manual_first_zero_pedal_coast_v1/calibration/states.jsonl) · [전체 ACK](../../../../../storage/experiments/validation/2026-10-02/town07_manual_first_zero_pedal_coast_v1/calibration/control_receipts.jsonl)

독립 확인에서는 모든 상태·정리·ACK를 읽어 actor/frame/실제 7개 제어값을 결합하고, 900틱 제로 제어·시간·거리·속도 차분·연속 dwell를 원본에서 다시 계산했다. 카메라가 없는 계측이므로 주행 PNG/GIF나 Autoware 화면을 만들어 붙이지 않았다.

| 증거 | SHA256 |
| --- | --- |
| 선언 | `c0377bfbde57269cf2679c1e2dbfdb1d3c5cac3b491627680e039dfd3f252617` |
| 원 결과 | `f164e7a9f1e48c43a07f17931076f84386bc45840b5dfe1581b331b5c2eb1dc8` |
| 전체 상태 | `c1fbd84215e0089ff6ca0d37f127e5fc6541d64664e480a15d822d5198c59d4e` |
| 전체 ACK | `dc9383d3ec4cd53761cbe6b8ba00227c67e7b9c1c2328f12feefda87427f1684` |
| 독립 진단 | `60772355a608afb7349eec0fe45cf9f5934bbd08d3cd0a11641a7a663cda4347` |

원 코드·테스트·`PRIOR_EVIDENCE.md`는 선언 당시 동결본을 유지했다. 이 문서만 실제 실행 이후 새로 추가했다.
