# 01 — 출발·제동의 원시 운동 분석과 10 ms/5 ms 비교

<!-- HH_260906 - Preserve the failed fine-setting capture and incomplete experiment; smaller curvature counts do not establish success. -->

## 결론: 물리 설정에 따라 움직임이 달라졌지만, 5 ms는 미채택

이전 네 회차를 원시 자료로 재분석하고, **로컬 CARLA Town07 직진·BasicAgent**에서 최대 물리 계산
간격 설정 10 ms/5 ms 비교를 실제 시작했다. 10 ms A1은 정상 수집을 완료했다.
5 ms B1은 **목표 약 1.75 m 앞에서 멈춘 뒤 기존 45초 감속 제한시간으로 실패**했다.
정상 정지 유지 제동에 진입하지 못했다. 나머지 B2/A2는 사전 중단 정책에 따라 실행하지 않았다.

이번 실패는 **포트 점유나 서버 연결 끊김이 아니다.** 두 회차 모두 준비·수집 후 서버 검사와
소유 서버 종료 확인을 통과했다. 운영 실패를 숨기거나 시간/정지 기준을 늘려 재시도하지 않았다.
최종 판정은 `INCOMPLETE_OR_FAILED_NO_COMPARATIVE_VERDICT`이며 **데이터 미편입·모델 미채택**이다.

| 사전 순서 | 실제 상태 | 완전한 미래 창의 곡률 실패 | 고유 실패 시점 | 최대 곡률 rad/m |
| --- | --- | ---: | ---: | ---: |
| A1 최대 10 ms | 수집 완료·미편입 | 115 / 671 (17.14%) | 3 | 18.444801308 |
| B1 최대 5 ms | **감속 timeout·실패 원본 보존** | 63 / 788 (7.99%) | 1 | 0.335074365 |
| B2 최대 5 ms 반복 | 미실행 | 미측정 | 미측정 | 미측정 |
| A2 최대 10 ms 반복 | 미실행 | 미측정 | 미측정 | 미측정 |

B1의 낮은 수치는 성공한 동일 정지 장면의 개선이 아니다. 종점 정지·유지·tail을 못 만들었고
관측 길이/분모도 다르므로 **정지 실패를 곡률 숫자 감소로 상쇄하지 않는다.**

## 실제로 무엇이 달랐나

![실제 감속 시작부터 마지막 원본까지의 거리·속도·브레이크·측방 변위](motion_comparison/coast_to_stop_motion.png)

파란색은 10 ms, 주황색은 5 ms다. 각 실제 감속 시작 시각을 그림의 0초로 표시했지만 원시 시각이나
정답은 바꾸지 않았다. 각 회차의 **감속 시작부터 마지막까지 모든 원시 상태**를 사용했다.
[그림 원본 수치·시각·해시](motion_comparison/manifest.json)도 함께 있다.

- A1은 원래 목표 창 1 m 안에 들어와 정지 제동을 요청하고 2초 유지·6.5초 tail을 기록했다.
- B1은 감속 약 19.8초 후, 페달 요청/읽기 모두 0인 상태에서 속도가
  **0.173542 → 0.000196 m/s**로 한 50 ms 관측 간격에 감소했다.
- B1은 이후 정확한 보고 속도 0과 XY 위치 불변 상태를 **497개 관측·약 24.8초** 기록했지만,
  목표 거리는 약 1.7503 m라 정지 유지 조건에 들어가지 못했다. hold latch/hold counter/tail은 모두 0이다.
- B1의 관측 throttle/steer/brake는 그 정지 이후 마지막까지 모두 0이었다.
  마지막의 다음 brake 1.0은 **실패 중단 명령**이다. 그 이후의 실제 물리 반응을 기록했다고 주장하지 않으며,
  앞선 정지를 정상 hold 제동이 일으켰다고 해석하지 않는다.
- B1은 원래 native 감속도 기준도 1개 간격에서 초과했다(약 −3.4669 m/s²).
  10 Hz 미래 창의 다른 지표 8개가 실패 0이어도 이 20 Hz 원시 검사 실패를 제외하지 않는다.

B1의 63개 곡률 실패는 **전체 기록의 100 ms tick 600(감속 구간)**에 겹친다. A1의 출발 50·51 및
정지 제동 651시점과 같은 사건으로 취급하지 않는다. [상세 원시 경계 분석](residual_analysis.md)에
프레임·위치·속도 적분·페달·휠 각도·해시를 보존했다.

## 변경한 것은 한 가지

[실행 전 고정 계획](plan.json)의 순서는 A1 → B1 → B2 → A2, 회차당 최대 한 번이다.

- 공통: 물리 상태/제어 20 Hz, 원본 6카메라 10 Hz, Town07 TRAIN 직진 210.598 m,
  Prius, ClearNoon, seed 0, Epic, 목표 28.8 km/h.
- 공통: 기존 v7 즉시 정상 정지 제동·0.75 m/s 이하 정상 조향 0, 같은 PID/안착/정지/시간 제한.
- 공통: 최대 substep 개수 10, 기존 명령 ACK·초기화·비상 제동·품질 기준.
- 유일한 설정 요인: `max_substep_delta_time`을 명시적으로 **0.01 / 0.005초**로 설정한다.
  원래 환경·요청·적용 후 읽기의 WorldSettings 11개와 기존 runtime 6개를 교차 확인했다.

이는 **최대 계산 간격 설정**이며 실제 내부 substep의 시각/횟수나 물리 정확도를 직접 계측한 값이 아니다.
11개 적용 후 읽기를 11개 원상복구 독립 증명이라고 쓰지 않는다. 기존 소유 실행기의 정리 기록은 보존했다.
카메라나 제어 주파수를 올리지 않았으므로 이를 실시간 화면 FPS 개선 실험으로 부르지 않는다.

수치 설정은 안착 전부터 적용된다. B1은 첫 안착 관측에서 이미 A1과 물리 상태가 달랐고,
감속 진입 시각/속도/남은 거리도 완전히 같지 않았다. 따라서 **개입 전 상태가 동일한 제동 순간 비교가 아니다.**
반면 새 A1의 검사한 전체 물리/제어 상태는 이전 v7 즉시 제동 회차와 일치했다.

## 이전 4회 원시 분석에서 확인한 것

새 주행과 별개로, 이전 v7 네 회차의 **5,888상태·5,884개 native 간격·2,944개 미래 창**을 읽기 전용 재분석했다.
회차별 평가된 원래 곡률 값 38,423개와 실패 flag를 정확히 재현했고,
115개 중복 실패·고유 3시점씩, 네 회차 합계 460개 실패를 그대로 보존했다.
다른 지표의 기존 실패 수는 대조했으나 이 보조 분석기가 다른 모든 지표까지 다시 계산했다고 주장하지 않는다.

| 이전 실패 시점 | 계산상 확인한 구분 |
| --- | --- |
| 출발 50 | 움직임이 극히 작던 구간 뒤, 유지되던 차량 전방 기준과 첫 충분한 이동 방향을 비교 |
| 출발 51 | 해당 시점을 첫 미래점으로 삼는 창에서만 실패; 세 점 궤적의 동일 굽힘을 뜻하지 않음 |
| 제동 651 | 실제 세 점과 이전 평가 방향이 존재; 출발의 방향 유지 규칙만으로 설명되지 않음 |

코드에는 이미 뒤 차축 기준의 **평면 회전 속도 보정**이 들어 있다. 누락된 보정을 넣었다고 주장하지 않는다.
Actor/가상 rear-axle 변위와 속도 적분, yaw 먼저/pitch 나중의 명시적 좌표 오프셋 변화를 분리했다.
첫 제동 간격의 Actor 진행 방향 변위/진입 속도가 약 0.0100001095초인 것은
수치 설정 비교의 동기이지 실제 10 ms 내부 동작의 증명이 아니다.

[원시 분석 요약](native_analysis/summary.json) · [실패 12개 회차-시점의 원본 증거](native_analysis/boundary_witnesses.jsonl) ·
[5,884개 간격 전체·무손실 gzip](native_analysis/native_intervals.jsonl.gz)

현재 확인한 것은 **설정에 따른 관측 궤적 민감성과 실패 양상**이다. 엔진의 sleep·접촉·타이어·적분 순서
등 특정 내부 원인을 확정하지 않았다. 유리한 좌표나 시각으로 정답을 고쳐 PASS를 만들지 않았다.

## 새 수집 범위와 원본

실제 새 시도는 두 번, 합계 **3,176개 상태·1,588개 6카메라 묶음·9,528개 JPEG**다.
A1은 671개 완전한 미래 창과 65개 tail-context 묶음, B1은 788개 완전한 창과
**64개 불완전한 미래 창**이다. 실패 회차와 불완전한 창도 버리지 않았다.
두 회차의 제어 ACK 총 3,185개는 모두 수락됐다. 명령 수락은 물리 작동이나 학습 품질 승인과 다르다.

[전체 독립 검산](substep_analysis/summary.json) · [미완성 비교 판정](substep_analysis/paired_comparisons.json) ·
[모든 미래 창·무손실 gzip](substep_analysis/future_windows.jsonl.gz) ·
[개입/명령/실제 상태 연결](substep_analysis/control_policy_witnesses.jsonl) ·
[새 수집의 native 잔차 전체](substep_analysis/native_residuals.jsonl) ·
[원본 JPEG 검사 목록](substep_analysis/image_inventory.jsonl)

실제 시도·포트 기록: [A1](execution/01_reference_A1/result.json), [B1](execution/02_fine_B1/result.json).
B2/A2에 대한 실행/카메라 파일은 만들지 않았다. 기록된 소유 프로세스만 정리했으며 다른 작업을 종료하지 않았다.

## 차량 중심 실제 화면 — PNG 11장·GIF 2개

아래는 원본 6카메라 전체 화각과 **차량 중심 ±20 m 경로·과거 10초 실측 궤적**,
속도·제어를 함께 그린 증거다. **Autoware UI나 학습 모델 직접 운전 화면은 아니다.**
카메라 PNG 10장(원본 타일 60개)과 위 비교 그래프 1장, 실제 프레임 GIF 2개다.

| 회차 | 준비 | 정상 PID | 감속 시작 | 정상 hold 시작 | 완전 hold 읽기 | 마지막 | GIF |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A1 10 ms | [PNG](visuals/01_reference_A1/01_first_saved_camera_anchor.png) | [PNG](visuals/01_reference_A1/02_launch_to_normal_pid.png) | [PNG](visuals/01_reference_A1/03_normal_pid_to_zero_pedal_coast.png) | [PNG](visuals/01_reference_A1/04_zero_pedal_coast_to_service_brake_hold.png) | [PNG](visuals/01_reference_A1/05_first_full_hold_brake_readback.png) | [PNG](visuals/01_reference_A1/06_last_saved_camera_anchor.png) | [보기](visuals/01_reference_A1/sampled_actual_camera_timelapse.gif) |
| B1 5 ms **실패** | [PNG](visuals/02_fine_B1/01_first_saved_camera_anchor.png) | [PNG](visuals/02_fine_B1/02_launch_to_normal_pid.png) | [PNG](visuals/02_fine_B1/03_normal_pid_to_zero_pedal_coast.png) | **미관측** | **미관측** | [실패 전 마지막](visuals/02_fine_B1/06_last_saved_camera_anchor.png) | [실패 기록 보기](visuals/02_fine_B1/sampled_actual_camera_timelapse.gif) |

B1 마지막 화면에서 속도 0이어도 목표 밖이므로 성공한 정지 장면이 아니다. 미관측 hold 화면을
중단 브레이크나 다른 프레임으로 대체하지 않았다. 각 GIF는 균등 선택 24프레임·프레임당 400 ms로
9.6초간 보여주는 요약이다. A1 약 73.5초, B1 약 85.1초의 실제 시뮬레이터 시간을 압축한 것이며
실시간 10 Hz 재생이나 화면 끊김 측정이 아니다.

## 다음 순서와 두 환경의 상태

로컬에서는 이번 원시 분석과 두 번의 수집·실패 검산을 완료했다. **5 ms 설정을 개선안으로 채택하지 않는다.**
다음은 물리 설정을 고정한 뒤, 일정한 경험적 무페달 감속 거리만으로 종점 도달을 기대하는 방식의
한계를 다룬다. 남은 거리와 실측 속도를 보고 저속 접근을 조절하는 별도 프로파일을 설계·비교하고,
출발/정지 경계의 원래 품질 기준까지 통과해야 독립 TRAIN/개발 VAL 수집을 확대한다.
시간·허용 거리·곡률 한계를 늘리거나 실패 구간을 삭제해서 승인하지 않는다.

PRO6000에서는 이번에 **SSH 명령·새 학습을 실행하지 않았다.** 9월 14일의 전체 모델 6회 학습은
이미 완료된 별도 작업이다. 현재 GPU 사용량을 새로 확인했다고 주장하지 않는다.
검증된 새 데이터가 준비되면 GPU 0·개인 venv에서 다음 전체 모델 A/B와 고정 평가를 진행한다.
새 패키지 설치·재부팅·원격 Autoware 빌드·Git push·실차 제어·TEST payload 접근은 하지 않았다.
학습 모델의 폐루프 기능 통과 수는 여전히 **0/30**이다.

## 검증·재검산

<!-- HH_260906 - Separate regression success from failed data quality and retain source/version evidence. -->
기존 v7 전체 검사에 새 원시 분석·물리 설정·분석기·실행기 검사를 합쳐
**1,102 passed / 1 skipped (58.02초)**다. [JUnit 원문](regression.xml)을 보존한다.
기존 로컬 안전한 weights-only exact-resume 조건에 따른 1개 skip은 우회하지 않았다.
원시 분석과 새 실제 수집에 대한 별도 읽기 전용 검산도 완료했다.
이는 코드/증거 검증의 성공이며 실패한 B1을 통과로 바꾸는 뜻이 아니다.

이전 결과 원본과 이번 실패 `episode.partial`은 보존한다. 큰 분석 파일은 공개 사본만 무손실 압축했고,
이미 공개된 이전 미래 창은 중복 저장하지 않고 그대로 참조한다. [압축·원본 해시 기록](payload_encodings.json)에
복원 일치를 기록했다. [소스 사본](code_snapshot/run_case.py)은 원래 작업 경로를 전제로 한 실행 증거이며
공개 폴더에서 바로 실행하는 독립 설치 도구가 아니다. Git clone에 비공개 전체 JPEG/상태 원본은 포함되지 않는다.

원본과 고정 소스가 있는 이 저장소에서 새 수집은 아래 명령으로 재검산한다. CARLA를 새로 실행하지 않는다.

```bash
python3 scripts/e2e/summarize_carla_hold_substep_abba.py \
  --campaign-root artifacts/training/2026-09-15/native_boundary_and_substeps/substep_abba \
  --plan artifacts/training/2026-09-15/native_boundary_and_substeps/substep_abba/plan.json \
  --plan-sha256 dcfb17fa955208879606dce4c618ef0720522f780e046d4c5fde3aeabfd937e0 \
  --verify-only
```

[오늘 전체 결과](../README.md) · [이전 제동 램프 실험](../../../2026-09-14/portable_e2e_learning_cycle_v1/09_hold_brake_ramp/README.md)
