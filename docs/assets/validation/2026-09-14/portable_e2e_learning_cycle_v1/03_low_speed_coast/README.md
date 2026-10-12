# 03 — 저속 주행 후 무페달 감속: 움직임 이력 분리 계측

<!-- HH_260906 - Declare the bounded coast diagnostic separately from held-brake evidence and training approval. -->

**실제 2회 완료 — 정지 성공·학습 편입은 아님.** 두 회차에서 같은 저속 움직임과 속도 부호 교대가 재현됐다.
이 실험은 이전 [정지 브레이크 ABBA](../02_stationary_motion/README.md)와 달리 차량을 매우 낮은
속도로 움직인 뒤 가속·브레이크를 모두 놓는 과정이다. 기존 학습 모델이나 METEOR가 운전하지 않는다.

## 다음에 이것을 하는 이유

앞선 네 회차에서는 안착 후 정지 유지 중 움직임이 0이었다. 하지만 “움직이던 차가 페달을
놓은 뒤 멈추는 과정”은 검사하지 않았다. 두 상황을 분리해야 작은 변위·속도·방향 문제를
모두 정지 노이즈로 처리하거나, 주차 브레이크를 해결책으로 잘못 선택하지 않을 수 있다.

이번 실험은 **무페달 감속만** 분리한다. 제동 페달을 단계적으로 바꾸는 비교, 목표 지점에
정확히 정차하는 시험, 30 km/h 주행 시험은 아니다. 이후 제동 전환·독립 경로 수집에 앞선
진단이며 학습 데이터 자동 편입이나 기능 승인으로 연결하지 않는다.

## 실행 전 고정 조건

| 항목 | 조건 |
| --- | --- |
| 실행 위치 | 로컬 CARLA C_track_1_0_7, 매 회차 새 서버·Prius·같은 시작점 |
| 반복 | 2회, 첫 회차 실패 시 나머지 미실행; 자동 재시도 없음 |
| 물리·영상 | 기존 20 Hz / substep 최대 10 ms × 10 / Epic / ClearNoon / 동일 6카메라 |
| 안착 | 70 tick(3.5초), 서비스 brake 1, handbrake false |
| 출발 | throttle 0.13, brake·steer 0; 처음 관측 속도가 0.5 m/s 이상이면 다음 tick부터 coast |
| 출발 제한 | 최대 160 tick(8초); 목표 속도에 도달하지 않으면 실패 |
| 무페달 관측 | 정확히 900 tick(45초), throttle·brake·steer 0, handbrake false |
| 원본 저장 | 매 50 ms 상태·6카메라 수신 검증, 짝수 tick 영상 저장(10 Hz); 최대 1,130상태·565묶음/회 |
| 중단 기준 | 충돌·프로토콜 오류, 속도 >0.75 m/s, 후진 vx <−0.1 m/s, spawn에서 XY >10 m, 경로 횡오차 >1 m |

0.5 m/s는 약 **1.8 km/h**다. 속도를 강제로 주입하지 않고 실제 관측값이 기준을 넘는지 확인한다.
관측 도중 0.1 m/s 아래로 떨어져도 일찍 끝내거나 제동하지 않는다. 안착 70번째 tick에는 이전과
같은 0.1 m/s / 0.5 m 한계를 적용한다. 도로·장애물 사전 확인은 제한된 표본 검사이며,
전 구간 장애물 부재나 실차 안전을 보증하지 않는다. 충돌·범위 위반 표본도 남기고 종료한다.

## 무엇을 분석하는가

- 실제 Actor 3D 위치·속도·가속도·각속도와 기존 virtual-base 값을 따로 보존한다.
- 명령 ACK, 적용값 readback, front wheel angle, phase 경계, 프레임·타임스탬프를 확인한다.
- 원래 50 ms와 두 시작 parity의 100 ms 간격에서 위치 변화와 속도 적분 차이를 비교한다.
- 첫 coast 진입, 처음 0.1 m/s 이하가 되는 시점, 2초 연속 저속 유지 여부, 마지막 속도·이동거리를
  실제 관측으로 계산한다. 낮은 속도 기준의 만족은 완전 정지나 목표 위치 정차 성공이 아니다.
- 실제 센서 6카메라 전체 화각과 ego 중심 위치 그래프, 전체 구간 속도·잔차 그림을 함께 남긴다.
  카메라 표시 시점은 첫 coast의 짝수 tick과 마지막 예정 짝수 tick으로 고정하고 예쁜 장면을 고르지 않는다.

원본을 정지 위치에 맞춰 붙이거나, 속도/시간을 보정하거나, 문제가 있는 표본을 지워 통과시키지 않는다.
현재 TEST 데이터는 열지 않고, 기존 model/runtime/decoder 계약은 변경하지 않는다.

## 실행 결과

두 회차 모두 안착 70 + 출발 43 + coast 900 = **1,013상태**, 카메라 **506묶음·3,036 JPEG**다.
합계는 **2,026상태·1,012묶음·6,072 JPEG**이며, 추가 재시도나 실패 회차는 없다.
충돌·운영 한계 위반 없이 소유 actor 8개를 정리하고 설정·날씨·서버 종료를 확인했다.
15개 도로 표본과 6개 광선 검사도 통과했지만 전체 장애물 부재를 보증하는 자료는 아니다.
두 회차는 같은 시작점의 반복이며 독립 경로 두 개가 아니다.

| 측정 | 반복 1 | 반복 2 |
| --- | ---: | ---: |
| 무페달 전환 기준을 처음 넘은 속도 | 0.517824 m/s | 동일 |
| 전환 시점 | 관측 tick 113 → 명령 tick 114 | 동일 |
| 처음 0.1 m/s 이하가 된 시점¹ | 11.05초 | 동일 |
| 이후 연속 저속 유지가 시작된 시점¹ | 12.85초 | 동일 |
| 2초 연속 유지 확인 시점¹ | 14.85초 | 동일 |
| 45초 관측 마지막 속도 | **0.018527 m/s ≈ 0.0667 km/h** | 동일 |
| 무페달 첫 관측→마지막 Actor 순변위 | 3.810385 m | 동일 |
| 명령 전환 경계를 포함한 Actor 순변위 | 3.834742 m | 동일 |
| 요청 spawn에서 최종 XY 거리 | 3.963121 m | 동일 |
| 최대 경로 횡오차 | 0.003999 m | 동일 |

¹ 시간 기준은 마지막 launch 관측 직후의 coast 명령 경계다. 11.05초의 첫 저속 관측 직후에는
다시 0.1 m/s를 넘으므로 그것을 연속 유지 시작으로 계산하지 않는다. 타임스탬프의 1 ns 수준
차이 때문에 일부 소수 자리는 다르지만, 두 회차의 **물리·제어·phase 관련 31개 필드가
1,013 tick 전체에서 일치**했다. 전체 최대 속도는 0.552125 m/s였다.

[전체 결과·6,135개 입력 해시](evidence/summary.json) · [실행 전 고정 계획](provenance/plan.json) ·
[검증서](verification.json)

### 핵심 단서: 저속에서 보고 속도와 위치 변화가 다르게 보인다

<!-- HH_260906 - Describe observed signed-velocity parity without silently changing the stored ten-hertz state or inferring a physical cause. -->

마지막 10초 안에 들어오는 200표본(실제 표본 간 길이 약 9.95초, tick 814–1013)을 추가 분석했다.
두 회차 모두 Actor 위치는 순방향으로 **52.741 mm** 이동했고, 199개 관측 간격의 전방 변위가
모두 양수였다. 그러나 보고된 전후 속도 `vx`는 **199번 부호가 교대**했다.

| native 표본 | 표본 수 | `vx` 부호 | 평균 `vx` |
| --- | ---: | --- | ---: |
| 짝수 tick — 실제 저장되는 10 Hz 카메라 시점 | 100 | 모두 음수 | −0.007683 m/s |
| 홀수 tick — 카메라 수신은 검증하지만 이미지는 저장하지 않는 시점 | 100 | 모두 양수 | +0.018762 m/s |

**이번 저속 구간에서는 10 Hz의 어느 시점만 선택하느냐에 따라 순간 속도 해석이 달라진다.**
이는 화면 끊김의 원인을 특정한 결과도, 실제 바퀴가 서브스텝마다 뒤로 움직였다는 증거도 아니다.
홀수 표본으로 바꾸거나 속도를 0으로 덮어쓰면 해결된다고 판단하지 않는다.

속도 크기의 RMS만 이동거리/시간과 비교하면 부호 교대를 놓친다. 같은 199개 간격에서 **부호를
유지한 벡터 속도 적분**과 위치를 비교한 결과는 다음과 같다.

| 마지막 9.95초의 측정 | Actor 기준 | 기존 virtual-base 기준 |
| --- | ---: | ---: |
| 위치의 누적 이동거리 | 52.976 mm | 57.057 mm |
| 처음→마지막 순변위 | 52.741 mm | 52.749 mm |
| 부호 유지 속도 적분 벡터의 크기 | 55.127 mm | 55.131 mm |
| 순변위 벡터 − 속도 적분 벡터의 크기 | 2.476 mm | 2.498 mm |
| 각 50 ms 위치−적분 잔차 RMS / 최대 | 0.140 / 0.183 mm | 0.169 / 0.211 mm |

보고 기어는 이 구간에서 1로 일정했다. 요청 기어 0은 자동 기어 모드의 요청값이므로 실제 기어를
0으로 강제한 실험이 아니다. 접촉·구동계·미세 자세 변화·수치 계산 중 무엇이 원인인지는
아직 분리되지 않았다. 보편적인 noise 허용치나 데이터 통과 기준으로 사용하지 않는다.
[추가 분석 JSON](tail_observation.json)과 [읽기 전용 재계산 코드](tail_observation.py)를 함께 보존했다.

### 실제 카메라·차량 중심 궤적 PNG 6장

| 회차 | 전체 속도·변위·잔차·차체 방향 | coast 진입 6카메라·위치 | 마지막 6카메라·위치 |
| --- | --- | --- | --- |
| 반복 1 | [계측 그래프](evidence/01_coast_repeat_1_measured_traces.png) | [tick 114](evidence/01_coast_repeat_1_coast_entry_six_camera_trace.png) | [tick 1012](evidence/01_coast_repeat_1_scheduled_final_six_camera_trace.png) |
| 반복 2 | [계측 그래프](evidence/02_coast_repeat_2_measured_traces.png) | [tick 114](evidence/02_coast_repeat_2_coast_entry_six_camera_trace.png) | [tick 1012](evidence/02_coast_repeat_2_scheduled_final_six_camera_trace.png) |

![반복 1의 전체 계측](evidence/01_coast_repeat_1_measured_traces.png)

![반복 2의 마지막 6카메라·차량 중심 위치](evidence/02_coast_repeat_2_scheduled_final_six_camera_trace.png)

카메라 원본의 전체 640×360 화각을 유지하고, 그 시점의 virtual-base를 위치 그래프 중앙에 둔다.
Actor의 약 1.425 m 기준점 차이를 유지하며 예측 경로를 그려 넣지 않는다. 최종 native tick은
1013, 예정된 마지막 저장 영상은 짝수 tick 1012다. **약 50 ms 차이**를 제목에 표시했으며,
영상 패널의 궤적도 그 영상 시점까지만 그렸다. 전체 1013번째 상태는 그래프·JSON에 남아 있다.
이것은 Autoware/RViz 데스크톱이나 학습 모델이 운전하는 화면이 아니다. 밝은 하늘·어두운 도로를
보정하지 않았고 시각 품질을 학습용으로 승인하지 않았다. 새 GIF는 생성하지 않았다.

### 검증과 재현

최종 9개 테스트 파일 회귀 검사는 **373 passed / 58.33초**다. 원시 모든 이미지의 해시·디코딩,
상태/ACK/카메라 frame과 시간, source archive, phase 전환과 종료를 확인했다. 독립 검산에서는
복사된 route로 두 회차 모든 표본의 경로 projection도 정확히 재계산했다. 테스트 수는 기능 승인 수가 아니다.

요약기 검토 중 “COMPLETE와 오류 표시가 동시에 있는 기록” 및 “70번째 안착 표본의 더 엄격한
제한”을 거부하는 검사를 보강하고 재분석했다. 실제 두 회차는 추가 검사도 통과했고 수치는
바뀌지 않았다. 최초 분석본·당시 소스는 `artifacts/analysis/2026-09-14/low_speed_coast_v1_initial_summary`
에 보존했다. 차량을 다시 실행한 것이 아니며, 실제 raw·worker·계획은 수정하지 않았다.

원본은 저장소 루트의 `artifacts/training/2026-09-14/low_speed_coast_v1`에 있다. 회차별
`coast_probe`에 모든 상태·ACK·카메라가 있고 상위에 소유자 로그·lifecycle·16개 실행 소스 archive가 있다.
이번 실행에는 미커밋 worker/wrapper가 포함되어 HEAD만으로 소스를 식별할 수 없다. 계획의 바이트
해시와 archive가 기준이며 이번 단계에서 commit/push는 하지 않았다.

기존 의존성과 원본이 있는 저장소 루트에서 아래 명령으로 읽기 전용 분석을 재현한다. 첫 명령의
출력은 기존에 없는 폴더여야 한다. raw는 Git clone만으로 생기지 않는다.

```bash
CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 python3 \
  scripts/e2e/summarize_carla_low_speed_coast.py \
  --campaign-root artifacts/training/2026-09-14/low_speed_coast_v1 \
  --output-dir artifacts/analysis/low_speed_coast_recheck_01

PYTHONDONTWRITEBYTECODE=1 python3 \
  docs/assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/03_low_speed_coast/tail_observation.py
```

[SHA256SUMS](SHA256SUMS)는 이 게시 폴더의 파일을 검증한다. 기존 01 데이터 감사·02 정지 ABBA와
9월 9일 결과는 변경하지 않았다.

## 두 환경의 역할과 이후 순서

- **지금 로컬:** 제한된 2회 관측·원본 검산·그림·보고서 완료. 해당 CARLA 작업은 종료했다.
- **지금 PRO6000:** 이번 단계에서는 접속·설치·학습 실행 없음. GPU가 사용 중이라고 표시하지 않는다.
- **다음:** 같은 저속 구간의 관측 주기·속도/위치 일관성과 제동 전환을 분리 계측한다. 기존 10 Hz
  표본을 다른 parity로 바꾸거나 데이터 기준을 완화하지 않는다. 그 뒤 운동·영상 품질을 통과한 독립 TRAIN/VAL episode 확보
  → 데이터 버전 고정 → PRO6000 physical GPU 0에서 소수 TRAIN 점검 및 3-seed A/B
  → 로컬 export·10 Hz shadow·제한된 learned closed-loop 검증.

서버 패키지는 기존 제한대로 개인 hwanhong venv에서만 관리한다. 시스템/Conda·GPU 1·다른 작업은
변경하지 않는다. 이번 진단은 9개 영역/30개 하위 기능 중 새로운 learned closed-loop 승인 실적으로
세지 않는다.

[9월 14일 전체 목차](../README.md)
