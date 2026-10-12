# Town07: 새 회복 모델 — 실제 45초 × 3회

<!-- HH_261001 - Keep all prospective trials and distinguish basic driving from official feature approval. -->

기본 주행 통과 3/3 · 차체/10 cm 여유 3/3 · 관측 bundle→ACK 100 ms 3/3 · 모두 동시 통과 3/3.

| 회차 | 실제 전진 | CTE 95% / 최대 | 최대 수신→ACK | 기본 주행 | 벽시계 |
|---|---:|---:|---:|---|---|
| 1 | 103.657 m | 0.223 / 0.236 m | 87.191 ms | PASS | PASS |
| 2 | 96.320 m | 0.153 / 0.157 m | 90.553 ms | PASS | PASS |
| 3 | 100.980 m | 0.141 / 0.145 m | 91.567 ms | PASS | PASS |

학습한 것은 새 실제 복귀 데이터의 주행 4.0–34.1초 구간입니다. 초기 출발 회복을 학습했다고 주장하지 않습니다. 원래 97개 backbone tensor는 고정하고 4개 회복 branch tensor만 3,000 update 학습했습니다. 선택된 내비게이션 경로를 모델 출력으로 대체하지 않았습니다.

전체 2,700 native 관측, 1,350개 실제 6카메라 묶음/8,100 JPEG, 2,712 ACK를 확인합니다. 실패 후 정지 구간도 포함하고 세 회차 중 좋은 결과만 골라 성공으로 판단하지 않습니다.

100 ms 정책 처리와 관측 bundle delivery→ACK 벽시계 처리는 별도 기준입니다. 센서 노출/callback 진입부터의 완전한 10 Hz는 미측정입니다. 2.40 m/s 제어 headroom 보정도 모델 자체 성능과 구분합니다.

지연 분해는 실제 수락된 fresh 모델 제안 전체를 분모로 합니다. 거절된 첫 추론과 native 안전 거절/latched 비상제동을 수락된 추론처럼 넣지 않습니다. 원래 거절 명령·벽시계 기록은 모두 별도로 보존하며, 수락된 추론이 0개인 경우 정책·벽시계 기준은 미증명/FAIL입니다.

정식 기능 승인: 9개 중 0개 / 하위 30개 중 0개. 이 결과는 제한된 직진 기본 주행 예비 시험이며 모든 맵·회전·실차·정상 정지 기능의 승인이 아닙니다.

[전체 독립 확인 JSON](batch_outcome.json) · [원자료·소스 SHA256](source_evidence.json)

## 실제 전체 화면

### 1회차 — 전체 45초

![실제 6카메라와 차량 중심 지도](trial_00/full45s_actual_six_cameras_2hz.gif)

[실제 마지막 카메라](trial_00/05_last.png)

### 2회차 — 전체 45초

![실제 6카메라와 차량 중심 지도](trial_01/full45s_actual_six_cameras_2hz.gif)

[실제 마지막 카메라](trial_01/05_last.png)

### 3회차 — 전체 45초

![실제 6카메라와 차량 중심 지도](trial_02/full45s_actual_six_cameras_2hz.gif)

[실제 마지막 카메라](trial_02/05_last.png)

모든 지도는 측정 차량 위치를 중심으로 고정합니다. 보라색은 가장 최근 이전 ACK가 실제 모델 제어일 때만 표시하며 emergency ACK 뒤에는 지웁니다. 실제 CARLA 원 카메라와 기록 기반 진단이며 Autoware/RViz 화면 촬영이 아닙니다. GIF는 고정 0.5초 간격 90개 구간을 대표하고 마지막 원 카메라 449번도 별도 PNG에 남깁니다.
