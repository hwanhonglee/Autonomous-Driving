# Town07 실제 비교 — 기존 회복 모델 vs 새 공동 학습 모델

<!-- HH_261002 - Compare all actual arms under the same unchanged gates; learning/loss is not physical completion. -->

기존 모델의 전체 기준 통과 0/3 → 새 모델 0/3. 새 모델의 좌·우 복귀 조건 통과 1/2이며 중앙 비교는 별도입니다.

| 조건 | 기존 → 새 전체 기준 | 기존 → 새 전진 | 기존 → 새 native base 속도 최대 | 기존 → 새 관측→ACK 최대 |
|---|---|---:|---:|---:|
| negative_outward | FAIL → FAIL | 96.172 → 0.446 m | 2.516 → 0.655 m/s | 91.356 → 105.516 ms |
| positive_outward | FAIL → FAIL | 96.183 → 89.359 m | 2.515 → 2.471 m/s | 97.053 → 91.521 ms |
| centered_setup_comparator | FAIL → FAIL | 96.161 → 88.705 m | 2.516 → 2.472 m/s | 97.567 → 84.966 ms |

새 모델은 같은 101-tensor 구조에서 fusion·곡률·속도 등 **54개 tensor를 6,000 update 공동 학습**하고 47개 입력/시각/semantic-head tensor를 고정합니다. 기존 모델의 ‘97개 backbone 전체 고정’과 다릅니다. semantic-head 파라미터가 고정되어도 공유 특징이 바뀌어 semantic 출력은 보존됐다고 주장하지 않습니다. STOP은 학습하지 않았습니다.

모델 가중치만 바뀌었고, 실제 spawn ±0.20 m·±1.5°/중앙·seed·manual1 전문가 출발·PI 목표 2.40 m/s·원래 제어/안전 기준은 같습니다. 원본 상태를 만든 이전 모델 ACK부터 부호 CTE/yaw를 독립 계산합니다. 전문가가 출발 중 복귀시킨 것을 모델 복귀로 세지 않습니다.

복귀 조건: 첫 실제 모델 효과의 오차가 0.10 m·0.5° 이상이고 부호가 맞아야 합니다. 10초 안에 두 오차 밴드로 들어가 연속 2초 유지합니다(확인은 최대 12초). 중앙 조건은 복귀 2조건에 더하지 않습니다. 복귀만 통과했어도 속도/주행/차체/지연 중 하나라도 미달이면 전체 기준은 FAIL입니다.

모든 세 실행 전체 45초·2,700 native 상태·1,350 실제 6카메라 묶음·8,100 JPEG·2,712 ACK를 남깁니다. 각 실행 전진 ≥90 m, native base 속도 ≤2.5 m/s, world 속도 ≤2.7 m/s, 기존 CTE·무충돌·차체900개/sweep899개 및 10 cm 여유, 정책100 ms와 관측 bundle→ACK100 ms 기준을 완화하지 않습니다. 실패 꼬리와 미측정 값도 그대로 보존합니다.

최종6000 NPZ/metadata/checkpoint/학습 선언의 네 SHA와 두 부모 가중치·metadata를 별도로 묶습니다. 이 감사는 pickle/.pt나 Torch를 불러 추론하지 않으며, 47개 고정값 동등성은 선언된 numeric loader가 강제합니다. 감사가 직접 tensor 전체를 다시 로딩했다고 주장하지 않습니다.

센서 노출/callback 진입부터의 완전한 센서→ACK 10 Hz는 아직 미측정입니다. 고정 TRAIN 회랑의 저속 예비 시험이며 다른 맵·완전한 목적지 정지·회전·실차/30–60 kph의 승인이 아닙니다. **정식 MAIN9 승인 0/9·하위 기능 0/30이며 개발은 계속됩니다.**

[전체 비교·독립 확인 JSON](batch_outcome.json) · [원자료·소스 SHA256](source_evidence.json)

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
