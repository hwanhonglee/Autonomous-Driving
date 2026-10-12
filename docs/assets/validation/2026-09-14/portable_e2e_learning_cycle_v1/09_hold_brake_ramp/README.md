# 09 — 정지 브레이크 램프 ABBA: 4회 완료·궤적 개선 없음

<!-- HH_260906 - Report the complete prospective negative result without admitting failed labels or claiming learned driving. -->

## 결론

로컬 CARLA **Town07 직진·BasicAgent 전문가 제어**로 `즉시 제동 A1 → 0.5초 램프 B1 →
램프 B2 → 즉시 제동 A2`를 각각 한 번씩 실제 실행했다. **4회 모두 수집·독립 검산 완료**다.
이번에는 포트 점검에서 차단되지 않았다. 이전 08번의 미실행 A2를 재시도하거나 채운 것이 아니라,
별도로 사전 선언한 v7 실험이다.

브레이크 요청과 실제 읽기 값은 바뀌었지만 **관측한 차량 움직임과 원래 곡률 실패는 개선되지 않았다.**
이번 램프를 개선안으로 채택하지 않으며, 새 데이터의 학습 편입도 승인하지 않는다.

| 실행 순서 | 정상 정지 제동 | 미래 곡률 실패 / 완전한 미래 구간 | 중복 실패 step / 고유 시점 | 최대 곡률 rad/m |
| --- | --- | ---: | ---: | ---: |
| A1 | 즉시 1.0 | 115 / 671 (17.14%) | 115 / 3 | 18.444801308 |
| B1 | 0.1부터 1.0까지 램프 | 115 / 671 (17.14%) | 115 / 3 | 18.444801308 |
| B2 | 램프 반복 | 115 / 671 (17.14%) | 115 / 3 | 18.444801004 |
| A2 | 즉시 1.0 반복 | 115 / 671 (17.14%) | 115 / 3 | 18.444801308 |

다른 원래 미래 품질 지표 8개는 네 회차 모두 실패 구간 0이다. 그렇더라도 곡률 실패를 무시하지 않는다.
사전 기준인 **B 두 회차가 A 두 회차 모두보다 실패 수와 비율을 줄일 것**을 만족하지 못했다.
`MIXED_OR_NO_CONSISTENT_IMPROVEMENT_NOT_ADMITTED`가 최종 비교 판정이다.
B2 최대값의 작은 수치 차이를 유의미한 개선으로 해석하지 않는다.

## 무엇을 바꿨나

[실행 전 고정 계획](plan.json)에 따라 **정상 정지 유지 제동의 시작 크기만** 비교했다.

- 공통: Town07 TRAIN 직진 210.598 m, Prius, ClearNoon, seed 0, Epic.
- 공통: 물리 20 Hz·6카메라 10 Hz, 목표 28.8 km/h, 안착 3.5초,
  기존 정지 판정 0.1 m/s 이하·2초 유지·후속 기록 6.5초.
- 공통: v6에서 사용한 0.75 m/s 이하 정상 출력 조향 0, 같은 BasicAgent/PID와 실행 순서.
- A: 정상 정지 유지 명령의 brake를 즉시 1.0으로 요청한다.
- B: 첫 정지 유지 소스 시각을 `t0`로 고정하고, 측정 명령 소스 시각 `t`에서
  `min(1, (t - t0 + 0.05) / 0.5)`를 요청한다.
- 첫 B 요청은 0.1이다. 완전 제동 요청은 소스 기준 약 +0.45초,
  그 요청에 대응하는 첫 실제 제어 읽기는 약 +0.50초다. 이를 모두 +0.5초의 동일 사건이라고 쓰지 않는다.
- BasicAgent의 원래 비상 제동 설정 0.5는 변경하지 않았다. 수집기의 준비·tail·중단 제동 1.0과는
  별도 경로다. 이번 네 회차에서 BasicAgent 비상 제동 발생은 없었으며, 비상 제동을 램프로 바꾸지 않았다.

물리 설정, 차량/센서 보정, 라벨 시각, 원래 품질 기준과 정지 기준은 바꾸지 않았다.
구간 삭제·유리한 반복 선택·PID 초기화·위치/속도 강제 초기화도 하지 않았다.
제어 값은 CARLA의 float32 저장값과 수학적 요청값을 구분해 검산했다.
요청·ACK·읽기 확인은 물리 substep의 실제 브레이크 토크를 직접 측정했다는 뜻이 아니다.

## 제어는 달라졌는데 왜 실패가 같나

![네 회차 실제 브레이크 읽기·속도·진행 방향 변위](brake_comparison/brake_readback_and_motion.png)

위 그림의 첫 패널은 A의 즉시 1.0과 B의 계단식 증가를 보여준다. 아래 두 패널의
**실제 보고 속도와 위치 변화는 겹친다.** 각 회차의 정지 유지 소스 시각을 그림의 0초로만
표시했으며, 원시 시각과 학습 정답을 retiming하지 않았다. [원본 시각·frame·수치](brake_comparison/manifest.json)를 함께 보존했다.

반복 A1/A2와 B1/B2는 검사한 물리 상태 및 현재/다음 제어 필드가 일치했다.
서로 다른 제동을 비교한 A1/B1과 B2/A2는 제동 전까지의 물리·제어 이력이 일치하고,
정지 유지 시작 이후의 **제어를 제외한 관측 물리 필드도 일치**했다.
절대 시각·frame ID는 비교 대상이 아니며 제동 시작 소스의 다음 명령은 이미 개입 대상이라
개입 전 비교에서 제외했다. 모든 필드·모든 미래 보간값이 byte 단위로 같다는 주장은 아니다.

원래 고유 실패는 네 회차 모두 출발의 **50·51번째 100 ms 시점**과 제동 경계의 **651번째 시점**이다.
겹치는 6.4초 미래 창 때문에 이 3시점이 115개 실패 구간으로 집계된다.
첫 제동 50 ms의 Actor 위치 변화와 보고 속도 사다리꼴 적분 사이에는 네 회차 모두
약 **1.365 mm 잔차**가 있다. 이는 원시 관측의 일관성 분석이며 실제 substep 운동의 정답은 아니다.

**현재 증거만으로 브레이크 포화, 접촉/타이어 물리, 샘플링 또는 좌표 기준 중 하나를 원인으로 확정할 수 없다.**
단순히 램프 크기만 바꾸면 해결된다는 가설은 이번 조건에서 지지되지 않았다.
[A1/B1 상세 잔여 경계 분석](residual_analysis.md)은 첫 쌍에 한정된 독립 분석이고,
네 회차의 전체 비교는 [회차 간 비교 JSON](analysis/paired_comparisons.json)에 있다.

## 수집·증거 검증 범위

총 **5,888개 상태·2,944개 6카메라 묶음·17,664개 원본 JPEG**를 검사했다.
회차마다 1,472개 상태·736개 카메라 묶음 중 완전한 미래 6.4초 구간은 671개다.
회차별 나머지 65개 tail-context 카메라 묶음도 분석 파일에 보존했으며 학습 가능한 미래 정답으로 바꾸지 않았다.
정지 유지 2초와 tail 6.5초, 명령 연결, 카메라 파일·픽셀, 소유 실행기의 종료 기록을 확인했다.
이는 이 경로의 진단 완료이지 모든 Town·실차 안전·학습 데이터 승인이 아니다.

- [독립 검산 요약](analysis/summary.json)
- [전체 미래 창 2,944개 — 무손실 gzip](analysis/future_windows.jsonl.gz)
- [개입 전후 제어·최종 명령·실제 상태 연결 5,888개](analysis/control_policy_witnesses.jsonl)
- [원본 JPEG 검사 목록](analysis/image_inventory.jsonl)
- [4회 실제 실행 시도 기록](execution_attempts.json)

포트 2100–2102의 TCP/UDP 여섯 조건을 2초 이상 간격으로 3회 연속 관측하는 사전 점검을 추가했다.
네 회차 모두 약 4초의 연속 CLEAR 관측 뒤 기존 소유 실행기 검사도 통과했다.
[A1](execution/01_immediate_A1/readiness.json) · [B1](execution/02_ramp_B1/readiness.json) ·
[B2](execution/03_ramp_B2/readiness.json) · [A2](execution/04_immediate_A2/readiness.json)에 시각과 관측을 보존했다.
이는 포트 예약이나 향후 충돌 방지 보증이 아니다. 이전 08번 차단 원인을 소급 확정하지 않으며,
다른 프로세스 종료·포트 가드 우회·자동 주행 재시도를 하지 않았다.

## 차량 중심 카메라 화면·GIF

각 PNG는 원본 6카메라 전체 화각, **차량 중심 ±20 m 경로와 과거 10초 실제 궤적**,
속도·제어 값을 함께 보여준다. **Autoware UI나 학습 모델 운전 화면은 아니다.**
카메라 PNG 24장의 원본 타일 144개를 픽셀 단위로 대조했고, GIF 4개의 전체 96프레임을 디코딩 확인했다.
위 제어 비교 그래프까지 포함하면 **PNG 25장·GIF 4개**다.

| 회차 | 준비 | 정상 PID | 감속 시작 | 정지 제동 시작 | 완전 제동 읽기 | 마지막 | GIF |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A1 | [PNG](visuals/01_immediate_A1/01_first_saved_camera_anchor.png) | [PNG](visuals/01_immediate_A1/02_launch_to_normal_pid.png) | [PNG](visuals/01_immediate_A1/03_normal_pid_to_zero_pedal_coast.png) | [PNG](visuals/01_immediate_A1/04_zero_pedal_coast_to_service_brake_hold.png) | [PNG](visuals/01_immediate_A1/05_first_full_hold_brake_readback.png) | [PNG](visuals/01_immediate_A1/06_last_saved_camera_anchor.png) | [보기](visuals/01_immediate_A1/sampled_actual_camera_timelapse.gif) |
| B1 | [PNG](visuals/02_ramp_B1/01_first_saved_camera_anchor.png) | [PNG](visuals/02_ramp_B1/02_launch_to_normal_pid.png) | [PNG](visuals/02_ramp_B1/03_normal_pid_to_zero_pedal_coast.png) | [PNG](visuals/02_ramp_B1/04_zero_pedal_coast_to_service_brake_hold.png) | [PNG](visuals/02_ramp_B1/05_first_full_hold_brake_readback.png) | [PNG](visuals/02_ramp_B1/06_last_saved_camera_anchor.png) | [보기](visuals/02_ramp_B1/sampled_actual_camera_timelapse.gif) |
| B2 | [PNG](visuals/03_ramp_B2/01_first_saved_camera_anchor.png) | [PNG](visuals/03_ramp_B2/02_launch_to_normal_pid.png) | [PNG](visuals/03_ramp_B2/03_normal_pid_to_zero_pedal_coast.png) | [PNG](visuals/03_ramp_B2/04_zero_pedal_coast_to_service_brake_hold.png) | [PNG](visuals/03_ramp_B2/05_first_full_hold_brake_readback.png) | [PNG](visuals/03_ramp_B2/06_last_saved_camera_anchor.png) | [보기](visuals/03_ramp_B2/sampled_actual_camera_timelapse.gif) |
| A2 | [PNG](visuals/04_immediate_A2/01_first_saved_camera_anchor.png) | [PNG](visuals/04_immediate_A2/02_launch_to_normal_pid.png) | [PNG](visuals/04_immediate_A2/03_normal_pid_to_zero_pedal_coast.png) | [PNG](visuals/04_immediate_A2/04_zero_pedal_coast_to_service_brake_hold.png) | [PNG](visuals/04_immediate_A2/05_first_full_hold_brake_readback.png) | [PNG](visuals/04_immediate_A2/06_last_saved_camera_anchor.png) | [보기](visuals/04_immediate_A2/sampled_actual_camera_timelapse.gif) |

6개 화면 선택 규칙을 실행 전에 고정했다. 전환이 카메라 사이에서 발생하면 실제 선택한
카메라와 참조 시점의 차이를 표시했다. 즉시 제동 A에서는 제동 시작/완전 제동 읽기가
같은 저장 카메라에 해당할 수 있어 같은 원본 프레임을 두 참조에 정직하게 사용했다.
각 GIF는 실제 시뮬레이터 약 73.5초에서 균등 선택한 24프레임을 400 ms씩 보여주는 **9.6초 요약**이다.
실시간 10 Hz 재생이나 UI 끊김 측정 영상이 아니다.

## 학습과 다음 순서

이번 작업은 **로컬 제어 실험·데이터 진단**이며, SSH 명령·새 원격 학습·설치·빌드·push는 하지 않았다.
[05번 GPU 0 전체 모델 6회 학습](../05_stopmix_regret_learning/README.md)은 이미 종료된 별도 작업이다.
백그라운드 학습을 계속 실행 중이라고 보고하지 않는다.

다음은 제동 세기를 반복 변경하기보다 **출발/제동 경계의 Actor 위치·속도·가상 rear-axle 좌표를
원시 물리 시각에 맞춰 분리 진단**하는 것이다. 필요하면 출발 프로파일 또는 물리 계측의 한 요인만
별도 계획으로 비교한다. 기존 정답을 유리한 좌표로 교체하거나 기준을 완화해 PASS로 만들지 않는다.
이후 QA를 통과한 독립 TRAIN/개발 VAL 직진·좌/우회전·정지·재출발 데이터를 확보하고,
PRO6000 **GPU 0·개인 venv**에서 전체 모델 A/B 학습과 고정 검증을 진행한다.
TEST 잠금과 학습 모델 폐루프 기능 **0/30**은 유지한다.

## 회귀검사·재현 방법

<!-- HH_260906 - Keep code-test success separate from failed geometry and preserve original source snapshots. -->
기존 v6 회귀 범위와 새 v7 제어·통합·포트·분석·실행기 테스트를 합쳐
**958 passed / 1 skipped (56.37초)**다. [JUnit 원문](regression.xml)을 보존했다.
skip 1개는 기존 로컬 안전한 weights-only exact-resume 조건이며 우회하지 않았다.
4회 원본에 대한 별도 읽기 전용 검산도 성공했다. 성공한 것은 증거 검증이지 곡률 기준 통과가 아니다.

이전 01–08 공개 파일 196개, 보관한 v6 소스 24개와 v7 실행/입력/검사/오케스트레이션 pin 23개를
변경 없이 확인했다. 원본 수집은 `artifacts/training/2026-09-14/hold_brake_ramp_abba_v7`에 보존한다.
126,655,240바이트의 미래 창 JSONL은 공개 사본만 무손실 gzip으로 저장했다.
[압축/원본 해시 기록](analysis/payload_encodings.json)으로 복원 일치를 확인할 수 있다.
이 압축은 실패 데이터 삭제나 필터링이 아니다.

원본이 있는 저장소에서 아래 명령은 CARLA를 새로 실행하지 않고 기존 자료만 다시 검사한다.
Git clone에는 비공개 전체 JPEG/상태 원본이 포함되지 않는다. 재검산에는 해당 원본과 계획에 고정된
소스가 필요하다. [보관 실행기](code_snapshot/run_case.py)와 [화면 생성기](code_snapshot/render_evidence.py)는
원래 작업 경로를 전제로 한 소스 증거이며 공개 폴더에서 바로 실행하는 독립 설치 도구가 아니다.

```bash
python3 scripts/e2e/summarize_carla_hold_brake_ramp_abba.py \
  --campaign-root artifacts/training/2026-09-14/hold_brake_ramp_abba_v7 \
  --plan artifacts/training/2026-09-14/hold_brake_ramp_abba_v7/plan.json \
  --plan-sha256 a7708081681efb7dddf9fc52e547a0950b500dcb2c36f5be9f1deaa2402d0a48 \
  --attempts artifacts/training/2026-09-14/hold_brake_ramp_abba_v7/execution_attempts.json \
  --attempts-sha256 fef298651e9e419ebcb841ad956ec23852fa955d54b718bfbb44b4e1629f8cda \
  --verify-only
```

[이전 저속 조향 실험](../08_low_speed_steering_abba/README.md) · [오늘 전체 결과](../README.md)
