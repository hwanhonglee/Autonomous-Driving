# 08 — 저속 조향 분리 실험: 3회 수집 완료·마지막 1회 실행 전 차단

<!-- HH_260906 - Preserve an incomplete prospective ABBA and its remaining label failures, without declaring model or dataset approval. -->

## 결론

로컬 CARLA **Town07 직진·BasicAgent 전문가 제어**에서 저속 조향 출력만 분리해 비교했다.
기준 A1과 변경 B1/B2는 실제 수집·정지·명령 전달 검사를 완료했다. 마지막 A2는 실행 전
포트 2101 점유 검사에서 차단됐다. **4회 중 3회 수집이며, 완성된 ABBA 비교가 아니다.**
자동 재시도·유리한 회차 대체·기존 실패 삭제를 하지 않았다.

| 선언 순서 | 실제 결과 | 미래 곡률 실패 구간 / 전체 구간 | 중복 실패 step / 고유 실패 시점 |
| --- | --- | ---: | ---: |
| A1 기존 조향 | 수집·운영 검사 완료 | 134 / 666 (20.12%) | 612 / 39 |
| B1 저속 조향 0 | 수집·운영 검사 완료 | 115 / 671 (17.14%) | 115 / 3 |
| B2 저속 조향 0 반복 | 수집·운영 검사 완료 | 115 / 671 (17.14%) | 115 / 3 |
| A2 기존 조향 반복 | **실행 전 차단·카메라 수집 없음** | 미측정 | 미측정 |

겹치는 6.4초 미래 창 때문에 하나의 실제 경계가 여러 실패로 집계된다. B의 실제 기록이
A보다 0.5초 길어 분모도 다르며, 길이를 맞추려고 구간을 삭제하지 않았다.
관측한 실패 수는 줄었지만 **A2 부재로 사전 선언한 4회 비교 판정은 보류**다.
세 회차 모두 곡률 실패가 남아 **학습 미편입·모델 미채택**이다.

## 무엇을 바꿨고 무엇을 유지했나

[실행 전 고정 계획](plan.json)의 순서는 `A1 → B1 → B2 → A2`다.
새 프로파일은 `comfortable_v6_low_speed_steering`이며 이전 v5 원본을 재작성하지 않았다.

- 공통: Town07 기존 TRAIN 직진 경로 210.598 m, Prius, ClearNoon, seed 0, Epic.
- 공통: 물리 20 Hz·6카메라 10 Hz, 목표 28.8 km/h, 최초 안착 3.5초,
  측정 정지 2초·후속 정지 기록 6.5초, 기존 물리·속도·경로·미래 정답 기준.
- A: 원래 BasicAgent 조향을 반환한다.
- B: 측정 평면 속도 **0.75 m/s 이하**의 정상 주행 명령에서만 조향 출력을 0으로 바꾼다.
  가속·제동·기어 등 다른 필드는 건드리지 않는다.
- 원래 PID는 매번 실행한다. PID 기억 초기화, 최초 조향 검사 우회, 비상 제어 수정,
  위치/속도 초기화, 정답 보정·retiming·허용 기준 완화는 하지 않는다.
- 최초 원래 조향 검사 뒤, 기존 정지 제동 가드 앞에 개입한다. 개입 전/후 7개 제어 필드,
  최종 명령, 서버 ACK, 다음 실제 상태를 별도로 보존·검산한다.

0.75 m/s는 학습 품질의 새 허용 기준이 아니라 **실험 개입 조건**이다. 경계 재진입도
그대로 기록한다. 기존 차량/센서에 더해 보행자/보행자 제어기 없는 환경을 확인하고,
관측된 충돌·차선 이벤트는 다음 검사에서 중단한다. 비동기 센서이므로 즉시 대응 보증은 아니다.

## 실제 수집·검사 범위

총 **4,406개 20 Hz 상태·2,203개 6카메라 묶음·13,218개 원본 JPEG**다.
완료 3회 모두 최고속도 약 28.55 km/h, 충돌·차선침범 기록 0,
목표 지점 오차 0.9911 / 0.9967 / 0.9967 m, 정지 유지 2초·tail 6.5초를 확인했다.
이는 해당 조건의 수집 QA이지 실차 안전이나 모든 맵 성공의 증거가 아니다.

[독립 검사 요약](analysis/summary.json) · [회차 간 비교](analysis/paired_comparisons.json) ·
[전체 원시 미래 창·무손실 gzip](analysis/future_windows.jsonl.gz) ·
[조향 개입·최종 명령 연결 증거](analysis/steering_policy_witnesses.jsonl) ·
[원본 JPEG 검사 목록](analysis/image_inventory.jsonl)

제어 ACK **4,418개 전부 수락**, 상태 연결 불일치 0을 독립 확인했다.
B1/B2의 1,472개 상태는 검사한 물리 필드·현재/다음 제어가 동일했다. 절대 시각·frame ID는
비교에서 제외했으며, 나노초 보간의 최종 곡률 값까지 byte 동일하다는 의미는 아니다.
A1/B1 개입 전 준비 상태 70개도 물리 필드가 일치했다. 마지막 준비 상태의 다음 명령은
이미 개입 대상이므로 개입 전 비교에서 제외했다.

## 남은 실패는 어디인가

B1의 고유 실패 3시점은 출발 직후 **50·51번째 100 ms 시점**, 제동 경계 **651번째 시점**이다.
출발 2시점과 제동 경계는 조향 출력과 측정 휠 각도가 0인 상태에서도 남는다.
따라서 조향 제한 범위를 더 넓히면 해결된다고 결론내릴 수 없다.

기준 A1의 감속 중 32시점·정상 PID 구간 3시점 실패는 B1에서 관측되지 않았지만,
최대 곡률은 오히려 **16.7901 → 18.4448 rad/m**로 증가했다.
실패 수 감소를 모든 수치 개선이나 해결 완료라고 부르지 않는다.
Actor 기준과 가상 rear-axle 기준의 차이는 원인 진단용이며, 유리한 좌표로 학습 정답을
바꿔 PASS를 만들지 않았다. 자세한 변위·방향·제동 기록은 [잔여 경계 분석](residual_analysis.md)을 참고한다.

## 왜 마지막 주행이 없나

[실행 시도 기록](execution_attempts.json)에 실제 오류
`CARLA port 2101 is already occupied`와 exit 1을 보존했다.
A2 출력 폴더·소유 서버·수집기가 생성되기 전 차단됐으므로 센서/주행 실패 회차로 꾸미지 않는다.
17:01:28 KST 후속 읽기 전용 검사에서는 2100–2102 리스너·TCP 연결·CARLA 프로세스가 없었다.
**이후 점유가 사라졌다는 관측만으로 당시 다른 사용자나 TIME_WAIT가 원인이라고 확정할 수 없다.**
다른 프로세스를 종료하거나 포트 가드를 우회하지 않았고, 선언한 단일 시도·자동 재시도 금지를 지켰다.

## 차량 중심 화면과 실제 카메라 GIF

아래 그림은 실제 원본 6카메라 전체 화각과 **ego 중심 ±20 m 기준 경로·과거 10초 실측 궤적**,
속도·제어를 함께 표시한다. **Autoware UI나 학습 모델 주행 영상이 아니다.**
PNG마다 원본 640×360 카메라 타일을 픽셀 단위로 대조했다.

| 회차 | 첫 카메라·준비 | 정상 PID | 감속 시작 | 제동 경계 | 마지막 카메라 | GIF |
| --- | --- | --- | --- | --- | --- | --- |
| A1 | [PNG](visuals/01_native_A1/01_first_saved_camera_anchor.png) | [PNG](visuals/01_native_A1/02_launch_to_normal_pid.png) | [PNG](visuals/01_native_A1/03_normal_pid_to_zero_pedal_coast.png) | [PNG](visuals/01_native_A1/04_zero_pedal_coast_to_service_brake_hold.png) | [PNG](visuals/01_native_A1/05_last_saved_camera_anchor.png) | [실제 프레임](visuals/01_native_A1/sampled_actual_camera_timelapse.gif) |
| B1 | [PNG](visuals/02_zero_B1/01_first_saved_camera_anchor.png) | [PNG](visuals/02_zero_B1/02_launch_to_normal_pid.png) | [PNG](visuals/02_zero_B1/03_normal_pid_to_zero_pedal_coast.png) | [PNG](visuals/02_zero_B1/04_zero_pedal_coast_to_service_brake_hold.png) | [PNG](visuals/02_zero_B1/05_last_saved_camera_anchor.png) | [실제 프레임](visuals/02_zero_B1/sampled_actual_camera_timelapse.gif) |
| B2 | [PNG](visuals/03_zero_B2/01_first_saved_camera_anchor.png) | [PNG](visuals/03_zero_B2/02_launch_to_normal_pid.png) | [PNG](visuals/03_zero_B2/03_normal_pid_to_zero_pedal_coast.png) | [PNG](visuals/03_zero_B2/04_zero_pedal_coast_to_service_brake_hold.png) | [PNG](visuals/03_zero_B2/05_last_saved_camera_anchor.png) | [실제 프레임](visuals/03_zero_B2/sampled_actual_camera_timelapse.gif) |

고정한 5개 시점만 선택했고, 제어 전환이 카메라 사이에서 발생하면 실제 다음 카메라와의
시간 차이를 표시했다. GIF는 전체 episode에서 균등 선택한 **24프레임·프레임당 400 ms**의
9.6초 요약이다. 10 Hz 실시간 재생이나 화면 끊김 측정 영상이 아니며 실제 시뮬레이터 시간이 표시된다.
미실행 A2에 대한 그림은 만들지 않았다. 초기 Pillow API 렌더 실패 산출물은 비공개 원본 작업
폴더에 별도 보존했고, 패키지를 설치하지 않고 호환 처리 후 새 출력으로 생성했다.

## 서버 학습과 다음 단계

이번 단계는 **로컬 수집·제어 분리 실험**이다. 원격 SSH 명령·패키지 설치·새 학습·운용 모델
교체를 수행하지 않았다. [05번 서버 전체 모델 6회 학습](../05_stopmix_regret_learning/README.md)은
이미 완료된 별도 작업이며, 현재 백그라운드에서 계속 학습 중이라는 뜻이 아니다.

다음은 포트 차단 당시 상태를 남길 수 있도록 사전 점검을 보강하고, **출발 페달/램프 또는
정지 제동 시작 램프 중 한 요인만** 새 계획에서 분리해 검증하는 것이다.
비상 제동과 기존 정지/품질 기준은 유지한다. 새 데이터가 독립 QA를 통과한 뒤에만
TRAIN/개발 VAL의 별도 직진·좌/우회전·정지·재출발 episode를 확대하고,
PRO6000 **GPU 0·개인 venv**에서 다음 전체 모델 학습을 진행한다.
현재 TEST 잠금과 학습 모델 폐루프 기능 **0/30**은 그대로다.

## 코드 검사·원본 검증·재검산

<!-- HH_260906 - Distinguish successful evidence verification from failed geometry and preserve the one secure-resume skip. -->
관련 통합 회귀는 **783 passed / 1 skipped (55.62초)**다. 새 v6 제어/통합 검사 113개,
분석기 32개, 보조 포트 차단 회귀 2개를 포함한다. skip 1개는 로컬 환경의 안전한
weights-only exact-resume 조건이며 우회하지 않았다. [JUnit 원문](regression.xml)을 보존했다.
이 테스트 수는 자율주행 기능 통과 수가 아니다.

이전 01–07 공개 파일 158개와 기존 v5 곡률 재현 결과는 변경 없이 다시 확인했다.
원본 수집·실패 자료는 `artifacts/training/2026-09-14/low_speed_steering_abba_v6`에 그대로 있다.
공개 미래 창은 크기를 줄이기 위해 무손실 gzip으로만 저장했으며 원본 JSONL 해시와 압축
해시를 [인코딩 기록](analysis/payload_encodings.json)에 함께 남겼다. 원본 데이터 삭제가 아니다.

로컬 원본이 있는 같은 저장소에서 다음 명령으로 다시 읽기 전용 검산할 수 있다.
Git clone만으로 비공개 원본 수집 파일까지 내려오는 것은 아니며 해당 원본과 고정 소스가 필요하다.

```bash
python3 scripts/e2e/summarize_carla_low_speed_steering_abba.py \
  --campaign-root artifacts/training/2026-09-14/low_speed_steering_abba_v6 \
  --plan artifacts/training/2026-09-14/low_speed_steering_abba_v6/plan.json \
  --plan-sha256 f0d4ee95bb1ab9f46117e42107a2b396950e16cf9f2b918bd4f6d46718fe131a \
  --execution-attempts artifacts/training/2026-09-14/low_speed_steering_abba_v6/execution_attempts.json \
  --execution-attempts-sha256 f2025e1c2b8fa7d711a80ab943b309632e0fd5f6dbda6807347afac1c38b02b4 \
  --verify-only
```

명령 성공은 증거를 올바르게 검산했다는 뜻이다. 결과의 `PRELAUNCH_BLOCKED_NOT_CAPTURED`,
곡률 실패와 `training_data_approved: false`가 PASS로 바뀌는 것은 아니다.

[이전 학습량·전체 맵 현황](../07_coverage_and_curvature/README.md) ·
[오늘 전체 결과](../README.md)
