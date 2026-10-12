# 04 — 저속 감속 후 서비스 브레이크 적용: 무페달과 ABBA 비교

<!-- HH_260906 - Separate measured brake intervention from training, learned driving, and physical-cause claims. -->

**로컬 CARLA ABBA 4회 완료.** 같은 저속 감속 이력 뒤 무페달을 유지하면 미세 전진·속도 부호
교대가 남았고, 서비스 브레이크를 적용한 두 회차에서는 정지 상태가 관측됐다.
실행 전 고정한 [계획](provenance/plan.json)을 따른 제한된 진단이며 학습 데이터 편입·모델 채택은 아니다.

이전 [저속 무페달 감속](../03_low_speed_coast/README.md)에서는 관측 위치가 전진하는 동안
보고된 전후 속도 부호가 20 Hz tick마다 교대했다. 이번에는 같은 출발·30초 감속 이력 뒤에
서비스 브레이크를 적용했을 때 이 현상이 남는지 확인한다. 내부 원인을 확정하거나 학습용
정지 정답으로 승인하는 실험이 아니다.

| 순서 | 회차 | 공통 이력 뒤 15초 |
| --- | --- | --- |
| 1 | A1 | 무페달 유지 |
| 2 | B1 | 서비스 brake 1 |
| 3 | B2 | 서비스 brake 1 |
| 4 | A2 | 무페달 유지 |

매번 새 서버·Prius를 같은 C_track_1_0_7 시작점에 배치한다. Epic·ClearNoon·6카메라,
20 Hz 물리/수신 검증 및 전역 짝수 tick 영상 저장(10 Hz), substep 최대 10 ms×10은 동일하다.
70 tick 서비스 제동 안착 → throttle 0.13으로 처음 관측 속도 0.5 m/s 이상(약 1.8 km/h)
→ 다음 tick부터 600 tick 무페달 감속 → 다음 tick부터 300 tick 비교 구간이다.
두 arm 모두 600번째 감속 표본에서 속도 ≤0.1 m/s여야 비교 구간으로 넘어간다.
handbrake·reverse·manual gear는 항상 false이며 속도·위치를 강제로 넣지 않는다.

충돌·프로토콜 오류·속도 >0.75 m/s·전후 vx <−0.1 m/s·시작점 거리 >10 m·횡오차 >1 m에서
위반 표본을 보존하고 중단한다. 70번째 안착 표본에는 속도 ≤0.1 m/s, 거리 ≤0.5 m도 적용한다.
어느 회차든 실패하면 이후 회차를 실행하지 않고 재시도하지 않는다. 사전 도로·광선 표본 검사는
전체 영역의 안전 보증이 아니다. 30/60 km/h 주행이나 목표점 정차 시험도 아니다.

서비스 제동을 명령한 첫 tick 전의 위치부터 마지막 tick까지 15초 변위를 계산한다.
마지막 10초 구간의 부호 유지 속도 적분·위치 변화·짝수/홀수 tick 속도를 함께 비교한다.
영상은 실제 6카메라 전체 화각과 해당 시점 차량 중심 위치 그래프를 사용하며, Autoware 화면이나
학습 모델의 예측 경로로 표시하지 않는다. 원본 표본·시간·속도·판정 기준을 수정하지 않는다.

PRO6000 접속·설치·학습, 데이터 편입, 새 모델 채택은 이번 단계에 포함되지 않는다.
9개 영역/30개 하위 기능의 learned closed-loop 통과 수는 이 진단으로 증가하지 않는다.

## 실제 결과

각 회차는 안착 70 + 출발 43 + 감속 600 + 비교 300 = **1,013상태**, 카메라 506묶음·3,036 JPEG다.
합계 **4,052상태·2,024묶음·12,144 JPEG**다. 네 회차 모두 exit 0이며, 충돌·운영 한계 위반 없이
소유 actor 8개 정리, 설정·날씨 복원, 서버 종료를 확인했다. 실패·재시도 회차는 없다.

비교 시작 기준은 coast의 마지막 **tick 713**, 실제 다른 제어 명령은 다음 **tick 714**부터다.
그 기준 속도는 네 회차 모두 **0.0189401 m/s**였다. 아래 15초 변위는 tick 713→1013의 300개
간격을 포함한다. 첫 제동 후 관측인 tick 714부터 계산해 첫 움직임을 누락하지 않았다.

| 측정 | A1 무페달 | B1 서비스 제동 | B2 서비스 제동 | A2 무페달 |
| --- | ---: | ---: | ---: | ---: |
| 15초 Actor XY 순변위 | 81.340 mm | **0.488 mm** | **0.488 mm** | 81.340 mm |
| 최종 보고 속도 | 0.018527 m/s | **0 m/s** | **0 m/s** | 0.018527 m/s |
| 비교 구간 300표본 중 속도 0 | 0 | 292 | 292 | 0 |
| 마지막 200표본 Actor XY 순변위 | 52.741 mm | **0 mm** | **0 mm** | 52.741 mm |
| 마지막 200표본 전후 vx 부호 전환 | 199회 | 0회 | 0회 | 199회 |

마지막 200표본은 tick 814–1013, 실제 표본 사이 길이는 약 **9.95000015초**다.
마지막 timestamp에서 10초 이내에 든 표본을 고른 것이며, 딱 10초라고 시간을 다시 맞추지 않았다.
무페달에서는 저장 10 Hz 짝수 tick의 vx가 모두 음수, 홀수 tick은 모두 양수였다. 제동에서는
이 구간의 모든 3D 위치·보고 속도가 일정/0이었다. Actor와 virtual-base 기준, 벡터 속도 적분과
위치 잔차는 [전체 결과 JSON](evidence/summary.json)에 따로 남겼다.

두 제동 회차 모두 tick 722(비교 9번째 표본, 명령 경계로부터 약 0.45초)부터 끝까지
**3D 보고 속도 0이 292표본 유지**됐다. 이것은 50 ms 간격 관측 결과이지 실제 액추에이터의
물리적 응답 지연이 0.45초라는 뜻은 아니다. 처음 8표본의 작은 속도도 그대로 보존했다.

**전환 순간의 차이도 남아 있다.** 두 제동 회차의 첫 50 ms(tick 713→714)에서는 Actor 관측
위치가 차량 전방 축 기준 약 **0.484 mm 후방**으로 변했다. 같은 간격의 위치 변화와 부호 유지
속도 사다리꼴 적분 차이는 Actor **0.960 mm**, virtual-base **1.074 mm**다. 마지막 정지 상태만
보고 이 전환 표본의 운동 일관성까지 통과했다고 판단하지 않는다. 순간 제동 중 끝점 속도 적분의
한계와 내부 물리 계산 원인을 아직 분리하지 않았으며 해당 표본을 제거하거나 0으로 바꾸지 않았다.

### 무엇이 확인됐고, 무엇은 아직 아닌가

이번 조건에서는 서비스 브레이크 적용 뒤 미세 전진과 속도 부호 교대가 사라지는 반응이 재현됐다.
따라서 무페달 상태의 잔류 움직임과 제동 유지 상태를 구분해야 한다. **부호가 교대했다는 이유로
차량이 실제로 앞뒤로 왕복했다고 단정하거나, 카메라 Hz·PC 부하를 원인으로 확정하지 않는다.**
구동계 creep, 접촉, 서브스텝 계산 등 내부 원인은 아직 분리하지 않았다.

매번 같은 시작점·설정으로 다시 실행한 반복이다. 여러 Town의 독립 검증이나 일반적인 정지
성능 검증이 아니다. brake 1을 30 km/h 주행 제어에 바로 적용하는 개선안도 아니다.
목표 위치 정확도, 자연스러운 감속·jerk, 재출발, 10 Hz 모델 추론/제어는 별도 검증이 필요하다.

## 실제 6카메라·차량 중심 궤적 PNG

| 회차 | 전체 속도·후반 변위·잔차 | 비교 구간 진입 6카메라 | 마지막 예정 6카메라 |
| --- | --- | --- | --- |
| A1 무페달 | [전체 계측](evidence/01_coast_A1_measured_traces.png) | [tick 714](evidence/01_coast_A1_tail_entry_six_camera_trace.png) | [tick 1012](evidence/01_coast_A1_scheduled_final_six_camera_trace.png) |
| B1 서비스 제동 | [전체 계측](evidence/02_service_B1_measured_traces.png) | [tick 714](evidence/02_service_B1_tail_entry_six_camera_trace.png) | [tick 1012](evidence/02_service_B1_scheduled_final_six_camera_trace.png) |
| B2 서비스 제동 | [전체 계측](evidence/03_service_B2_measured_traces.png) | [tick 714](evidence/03_service_B2_tail_entry_six_camera_trace.png) | [tick 1012](evidence/03_service_B2_scheduled_final_six_camera_trace.png) |
| A2 무페달 | [전체 계측](evidence/04_coast_A2_measured_traces.png) | [tick 714](evidence/04_coast_A2_tail_entry_six_camera_trace.png) | [tick 1012](evidence/04_coast_A2_scheduled_final_six_camera_trace.png) |

![B1 서비스 제동의 전체 계측](evidence/02_service_B1_measured_traces.png)

![B2 최종 6카메라·차량 중심 위치](evidence/03_service_B2_scheduled_final_six_camera_trace.png)

640×360 원본 전체 화각을 유지했고 위치 그래프 중앙은 해당 영상 시점의 virtual-base다.
Actor 기준점의 차이도 보존한다. 최종 native 상태는 tick 1013, 예정된 마지막 짝수 tick 영상은
1012여서 약 50 ms 차이가 있다. 제목에 이를 표시하고 궤적도 영상 시점까지만 그렸다.
예측 경로를 그려 넣거나 밝기/영상 품질을 보정하지 않았다. Autoware/RViz 데스크톱 캡처나
학습 모델 제어 화면이 아니며 새 GIF는 만들지 않았다.

## 검증·원본·재현

관련 10개 파일 회귀 **423 passed / 63.52초**, 새 요약기 별도 **16 passed / 13.11초**다.
합계 11개 파일의 439개 테스트이며 전체 저장소 테스트나 기능 승인 수는 아니다.
새 worker/요약기는 실제 실행·게시 전에 독립 검토했다. 모든 JPEG 해시·디코딩, 원본·ACK·프레임,
제어 전환, 소스 17개 archive, 설정·날씨·종료 증거를 확인했다.
독립 검산에서 네 회차의 앞 713표본, 같은 arm 반복의 전체 1,013표본은 물리·제어 관련 25개
필드가 정확히 일치했다(절대 frame/time 제외). 4,052표본 전체의 base 변환·경로 projection도
재계산해 원본과 일치했다. 같은 seed 반복이라는 범위는 그대로다.
[검증서](verification.json)와 [SHA256SUMS](SHA256SUMS)를 함께 제공한다.

원본은 저장소 루트의 `artifacts/training/2026-09-14/coast_brake_transition_v1`에 있다.
회차별 `brake_probe`가 상태·영상·ACK·manifest, 상위 `provenance`가 실행 소스 archive다.
실행에는 미커밋 코드가 포함돼 HEAD만으로 식별할 수 없으며 [계획의 SHA256](provenance/plan.json)이
기준이다. 과거 01/02/03 결과·원본과 기존 요약기는 바꾸지 않았다. commit/push도 하지 않았다.

기존 의존성과 실제 raw가 있는 저장소 루트에서 읽기 전용 분석을 재현한다. 출력 폴더는 새 경로여야
한다. raw는 Git clone만으로 생기지 않는다.

```bash
CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 python3 \
  scripts/e2e/summarize_carla_coast_brake_transition.py \
  --campaign-root artifacts/training/2026-09-14/coast_brake_transition_v1 \
  --output-dir artifacts/analysis/coast_brake_transition_recheck_01
```

## 다음 순서와 두 환경

1. **로컬 다음:** 이번 결과를 근거로 자연 감속→서비스 제동 유지→재출발의 제한된 제어 프로파일을
   검증한다. 목표 위치·가감속/jerk·저속 위치/속도 일관성·카메라 품질을 함께 확인하며 기존 기준을
   완화하지 않는다. 이번 진단의 brake 1을 학습 정답으로 곧바로 편입하지 않는다.
2. **로컬 데이터:** 품질을 통과한 독립 직진·좌/우회전·정지 episode를 TRAIN/VAL로 확보하고 버전을 고정한다.
   같은 시작점 반복 네 회를 독립 학습/검증 데이터로 나누지 않는다. TEST는 계속 열지 않는다.
3. **PRO6000 학습:** 검증된 새 데이터로 소수 TRAIN 학습 점검 → 동일 조건 3-seed A/B.
   physical GPU 0만, 패키지는 hwanhong 개인 venv만 사용한다. 이번 단계에서 서버 접속·학습은 없었다.
4. **로컬 모델 실행:** export 동등성 → 10 Hz shadow → 제한된 CARLA learned closed-loop.
   30개 기능의 학습 모델 직접 제어 통과 수는 아직 **0/30**이다.

[9월 14일 전체 목차](../README.md)
