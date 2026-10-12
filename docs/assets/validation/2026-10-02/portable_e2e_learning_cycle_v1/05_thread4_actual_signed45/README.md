# Town07: 같은 모델의 CPU1 → CPU4 실제 주행 비교

<!-- HH_261002 - Retain strict numerical parity FALSE and every unchanged physical threshold/outcome. -->

전체 기준: CPU1 0/3 → CPU4 0/3. 좌·우 복귀 2/2, 중앙 비교는 별도입니다.

| 조건 | CPU1 → CPU4 전체 기준 | 전진 | CTE95% | 관측→ACK 최대 |
|---|---|---:|---:|---:|
| negative_outward | FAIL → FAIL | 0.446 → 87.944 m | 0.256 → 0.242 m | 105.516 → 76.741 ms |
| positive_outward | FAIL → FAIL | 89.359 → 89.362 m | 0.231 → 0.231 m | 91.521 → 77.167 ms |
| centered_setup_comparator | FAIL → FAIL | 88.705 → 88.767 m | 0.035 → 0.035 m | 84.966 → 77.867 ms |

가중치·출발 위치/seed·전문가 출발·PI2.40·90m·native 속도2.5·차체900/sweep899 및10cm·정책/관측→ACK100ms는 그대로입니다. 바뀐 것은 CPU intra-op4/interop1 preset이며 실제 모든 normal 제안(초기화/인계/새 추론/held)의 설정과 원래 wall-clock 구간을 독립 확인합니다.

이 변경은 엄격한 수치 동등성 통과가 아닙니다. 기존 고정128개 JPEG 복원 입력 benchmark의 동일 tensor SHA/raw1e-6는 통과했지만 decoded/selected1e-6는 실패했습니다. 관측된 차이는 XY최대3.46µm·속도1.94e-5m/s입니다. 원래 FALSE를 보존하고 예측/안전 기준을 교체하거나 허용오차를 완화하지 않습니다. Offline warm-window 결과는 실제 센서/ACK 성능의 승인이 아닙니다.

세 실행의 전체45초·2700상태·1350카메라묶음·8100JPEG·2712ACK와 모든 실패 꼬리를 남깁니다. 센서 노출/callback 진입부터의 완전한10Hz는 미측정입니다. 정식 MAIN9 승인0/9·하위0/30, 완전한 도착/정지·회전·다른 맵·실차 승인이 아니며 개발을 계속합니다.

[독립 결과 JSON](batch_outcome.json) · [원자료/소스SHA](source_evidence.json)

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
