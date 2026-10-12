# 왜 수동 1단 무페달 계측을 한 번 분리하는가

<!-- HH_261002 - Reconcile actual old coast/brake evidence and failed denominators before a new manual-first experiment. -->

**예전 자동기어 기록에서는 페달을 완전히 떼었을 때 이번 같은 급감속이 없었다. 작은 브레이크를 일정하게 유지한 여섯 기록에서는 모두 급감속했다.** 현재 목적지 정지 교사는 수동 1단이므로, 예전 자동기어 결과만으로 현재 무페달 응답을 확정할 수 없다. 새 실험은 이 빈칸 하나를 확인하려는 원인 분리 계측이며 학습·정지 기능 승인 시험이 아니다.

실제 원자료를 읽고 저장된 SHA와 대조했다. 새 시뮬레이터·모델·GPU는 이 과거 자료 검토에 사용하지 않았다. 센서 Hz로 희석하지 않고 모든 실제 20Hz 속도 차분 `Δhypot(vx,vy)/Δt`를 계산했다. CARLA가 기록한 API 가속도 크기는 이 차분과 구별한다.

## 1. 자동기어 무페달 4회

Town07 직선·Toyota Prius·ClearNoon, 동기식 0.05초, substep 0.01초×10 조건이다. 무페달 구간의 요청·관측 throttle/brake는 전량 0/0이며 관측 `manual_gear_shift=False, gear=1`이다. 3m/s로 속도를 강제 설정하지 않고 throttle 0.30 준비 중 처음 기준을 넘은 실제 **3.039646m/s**에서 시작했다.

| 원래 실험 | 전체 상태 / 무페달 상태 | 종료 실제 속도 | 무페달 최소 가속도 | 무페달 ±2.9 위반 |
| --- | ---: | ---: | ---: | ---: |
| low_speed_response_v2 / coast 01 | 566 / 400 | 0.186413m/s, 20초 | −1.217907m/s² | 0 |
| 같은 v2 / coast 02 | 566 / 400 | 0.186413m/s, 20초 | −1.217907m/s² | 0 |
| low_speed_response_v3 / coast 01 | 1,066 / 900 | 0.021451m/s, 45초 | −1.217907m/s² | 0 |
| 같은 v3 / coast 02 | 1,066 / 900 | 0.021451m/s, 45초 | −1.217907m/s² | 0 |

네 기록에서 직전 실제 속도 ≤0.5m/s인 무페달 구간의 최소 차분도 **−0.867592m/s²**였다. 45초 기록은 속도 ≤0.1m/s가 연속 2초였음을 26.85초 시점에 확인했고, 그때 약 21.0715m 전진했다. 이는 목표 위치에서 완전히 정지한 성공이 아니다.

**준비 실패를 빼서 전체 통과로 만들지 않는다.** 두 실험의 준비 throttle 0.30에는 최대 +3.654260m/s²가 남아 있다. v2는 9/9 case·총 2,742개 상태를 완료했지만 5개 case의 전체 스칼라 품질이 실패했다. v3는 6개 예정 중 coast 2개 완료, 세 번째 handoff 319개 상태에서 vegetation 충돌, 나머지 3개 NOT_RUN으로 **campaign FAIL**이었다. 완료 coast 둘만으로 campaign 성공을 주장하지 않는다.

[기존 v2 전체 설명](../../2026-09-08/portable_e2e_learning_cycle_v1/07_coast_ramp_identification/README.md), [기존 v3 실패·미실행 분모](../../2026-09-08/portable_e2e_learning_cycle_v1/08_extended_coast_partial_failure/README.md).

## 2. 고정 브레이크 6회는 이미 모두 급감속했다

같은 조건에서 직접 일정한 페달을 적용한 실험이며 PID·학습 모델이 없다. 각 case 전체 466개 상태 중 brake 유지 300개를 전수 확인했다. 요청·관측 브레이크의 실제 float32 값은 원자료대로 비교했으며 `manual=False, gear=1`이다.

| 선언 brake | 최소 실제 가속도 | 최악 실제 frame 쌍 | 최악 구간 속도 변화 | brake 유지 ±2.9 위반 |
| ---: | ---: | --- | --- | ---: |
| 0.02 | −19.796215m/s² | 4203→4204 | 1.402317→0.412506m/s | 2 |
| 0.04 | −19.822786 | 4656→4657 | 1.418995→0.427856 | 2 |
| 0.06 | −24.626103 | 5114→5115 | 1.402858→0.171553 | 2 |
| 0.08 | −19.701212 | 5575→5576 | 0.986344→0.001283 | 2 |
| 0.10 | −16.318412 | 6036→6037 | 1.546270→0.730349 | 2 |
| 0.12 | −16.377731 | 6499→6500 | 1.564541→0.745655 | 2 |

원래 12/12 페달 case·총 4,176개 상태의 분모와 준비 구간까지 남아 있으며, 전체 12개 중 10개가 ±2.9를 넘었다. **작은 brake가 작은 감속을 보장하지 않았고, PID 페달 전환만이 원인이라는 설명은 근거가 부족하다.** [기존 12개 전체 결과](../../2026-09-08/portable_e2e_learning_cycle_v1/04_pedal_response_calibration/README.md).

## 3. 실제 목적지 비교도 있었지만 학습 가능한 자료는 아니었다

옛 `brake_free_goal_stop_v4` 두 실행은 정상 brake=0과 자동기어로 달렸다. 전체 상태 1,472/1,462개, driving 1,272/1,262개, 실제 6카메라 묶음 736/731개, ACK 1,476/1,466개였다. driving 최소/최대 가속도는 각각 −1.601662/+2.688967, −1.746143/+2.803371m/s²로 스칼라 ±2.9 위반은 0이었다. 마지막 driving의 next full-brake 명령은 이후 tail용이므로 정상 감속 페달과 구별했다.

하지만 실제 미래 XY 곡률이 **145/671, 154/666개 창에서 실패**하여 TRAIN 권한은 false 그대로다. camera 전체 1,467묶음=8,802JPEG가 있고, scalar PASS를 미래 궤적·편안한 목표 정지 학습 승인으로 바꾸지 않았다. 이전 positive-brake 비교의 −8.860/−7.731m/s²는 [당시 독립 요약](../../2026-09-08/portable_e2e_learning_cycle_v1/14_brake_free_goal_stop/README.md)에 남아 있으며 이번 검토에서 그 baseline 원시 상태를 다시 재검산하지는 않았다.

별도 Town12 `pi_coast_hold_v1`은 20초 진입 안정화 timeout으로 coast 0회·hold 0회였다. 400개 native 간격 중 +3.253028m/s² 위반도 있다. 현재 수동 1단 pure coast의 성공 근거로 쓰지 않는다.

## 무엇이 측정됐고 무엇이 아직 미측정인가

과거 실제 `get_physics_control` 스냅샷에는 질량 1,375kg, wheel 4개, 원시 radius 37, max brake torque 800, tire friction 3.5, auto gearbox=true, forward ratio 1, final ratio 약 4.17, zero-throttle damping clutch-engaged 2.0 / disengaged 약 0.35가 있다. 원시 단위·값을 임의로 바꾸지 않았다. 당시 client/server 문자열은 `0.9.15-dev-dirty`였다.

그러나 **엔진·wheel RPM, 내부 필터 입력, sticky tire 상태는 이 API로 측정되지 않았다.** 옛 직접 페달 기록은 API applied-control readback이며 현재 ACK 운송 증명과 다르다. 현재 goal v2의 동일한 physics 전체 스냅샷 증명도 없으므로, 특정 내부 물리 함수나 동일 내부 상태가 원인이라고 확정할 수 없다.

새 계측은 현재 manual1에서 실제 첫 1.5m/s 진입 뒤 45초·900tick 동안 throttle/brake/steer=0/0/0을 유지한다. 준비·무페달·별도 cleanup 원시 상태, ACK, 실제 actor 세계 속도·좌표, 세 physics 스냅샷, 차체·10cm 여유·sweep·events를 남긴다. PID·goal·NN·카메라·GPU·학습은 없다. 기준 미달은 그대로 FAIL이며 메인 0/9·하위 0/30 상태를 바꾸지 않는다.

## 정확한 원본 핀

다음 상대 경로는 저장소의 현재 `storage/experiments/training/2026-09-08/` 아래다. 원자료나 과거 소스는 수정하지 않았다. 당시 원래 실행 소스 archive+unchanged 증명은 v1 9개, v2/v3 10개이며 v3 owner exit=1이다.

| 파일 | SHA256 |
| --- | --- |
| `low_speed_response_v1/run_001/actuation/manifest.json` | `5d93dda67b97f1921dec26d07c1794ab0af9817405a43bceaa571305b7d28680` |
| `low_speed_response_v2/run_001/actuation/manifest.json` | `67940e3f05a491f632d137586f451a9093fb057739754cf4ed1b42231c5cb0be` |
| `low_speed_response_v3/run_001/actuation.partial/manifest.json` | `a449c9bd264093c068147b7fc4fcc7a4180e776fe236d7b3761945b10bb41699` |
| `low_speed_response_v2/run_001/actuation/01_coast_repeat_01/states.jsonl` | `9c57039422961e8e108398b9d9b7451d53430f119d1c1a4d803c9b56bb3d51cf` |
| `low_speed_response_v2/run_001/actuation/02_coast_repeat_02/states.jsonl` | `f100ad884d4260903b302be817b14a586f4c6928d42e8cb05fbaf6be22d69e09` |
| `low_speed_response_v3/run_001/actuation.partial/01_coast_45s_repeat_01/states.jsonl` | `4337d2bb2e76c39dae9e7a83b7c1723ed357aa69b833c380a9fa0f2eec58648a` |
| `low_speed_response_v3/run_001/actuation.partial/02_coast_45s_repeat_02/states.jsonl` | `1e1b7c220d6c451d22392beb452db5100dcbc335e1edeb1b9fd6155e30d50ec2` |
| `low_speed_response_v1/run_001/actuation/07_brake_0.02/states.jsonl` | `8b5999490437f3a92d8b0155f64fb36d8067745cb906481c487516adbeaa1cd6` |
| `low_speed_response_v1/run_001/actuation/08_brake_0.04/states.jsonl` | `e0a39c194b2ea557e47c0a16ba1b0593fd3b20774b782bd6e127e9283c8830dd` |
| `low_speed_response_v1/run_001/actuation/09_brake_0.06/states.jsonl` | `04406b408ad6c7f38186634ac51acc1480e9302b125b40b1a30ad2a843c2b7da` |
| `low_speed_response_v1/run_001/actuation/10_brake_0.08/states.jsonl` | `f24f53dc0815976255ee9e1800465343d1d727076a15555d19432105b85e773b` |
| `low_speed_response_v1/run_001/actuation/11_brake_0.10/states.jsonl` | `4bbeb83a25ef92115bcce3a6b35b0660e1adbe91b40e754700fc177d405e1ea4` |
| `low_speed_response_v1/run_001/actuation/12_brake_0.12/states.jsonl` | `0144e6d38925ca88aa2f1fb54ab85b768622b6e68e7f2fe65627623ceec41636` |
| `brake_free_goal_stop_v4/town07_straight_calibration/run_001/episode/states.jsonl` | `8df19ac93f4399c86bcd6d46ffb18c78630d1618177b7457db73b70e5edb56c1` |
| `brake_free_goal_stop_v4/town07_straight_calibration/run_002/episode/states.jsonl` | `0021f6d7dad86f0bf56160a8f14d9b4432ac165a37c31a1c60ba748be25d05ca` |
| `brake_free_goal_stop_v4/town07_straight_calibration/run_001/episode/control_receipts.jsonl` | `10fe981c01e4f6902888082dad53ed2d29146c16aa78de3e5878abb909497ffa` |
| `brake_free_goal_stop_v4/town07_straight_calibration/run_002/episode/control_receipts.jsonl` | `285ee2b4e559ffa38308f7398a9c8897e4c7fc57eefa3f341cad96d9d2c17b90` |
