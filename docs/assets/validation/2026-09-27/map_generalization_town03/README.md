# Town03 지도 TRAIN 후보 원본 감사

<!-- HH_260906 - The owned CARLA semantic source is training evidence, not a learned driving or HD-map result. -->

**결론: QUALIFIED_TRAIN_CANDIDATES_ONLY.** 이는 두 Town03 경로의 수집 품질·중복 위치·라벨 분포를 확인한 결과이며, 지도 모델 학습이나 성능 향상을 뜻하지 않습니다. 차량은 CARLA BasicAgent가 움직였고 모델이 조향하지 않았습니다.

| 원본 640×360 | 직진 p03 | 우회전 p03 |
|---|---:|---:|
| 저장된 RGB/시맨틱 쌍 | 240 | 240 |
| 각 경로 자체 0.25m 중복 제거 | 232 | 184 |
| **두 경로 전체에서 보존** | **232** | **184** |
| 최종 보존 최소 120장 | 통과 | 통과 |

두 경로는 같은 Town에서 위치가 겹칠 수 있으므로 순서대로(`straight` 뒤 `right`) **이전 모든 보존 XY와 0.25m 이상** 떨어진 프레임만 계산했습니다. 전체 보존 수는 **416장**입니다. 다른 Town과 XY 거리를 계산하거나, 이 프레임을 서로 독립 TEST 사례로 세지 않습니다.

[전방 전체 RGB·CARLA 의미 라벨 PNG](town03_rgb_oracle_full_fov.png) · [파일별 SHA256·프레임 목록·클래스 픽셀 감사 JSON](summary.json)

후속 [독립 재검산된 913장 TRAIN 포장 기록](TRAIN_PACKAGE.md)은 이 감사와 별개로 원본·보존 목록·SHA를 다시 검증한 결과입니다.

각 owner의 exit 0, 원본 소스 archive 해시, 고정 route SHA/월드, CARLA lifecycle ready/after/stopped, 240장 센서 쌍 디코드, 충돌 0, live/replay 일치, cleanup 0을 검증했습니다. `traffic_sign`을 포함한 7개 클래스 픽셀 수는 **CARLA oracle 정답**으로 계산했고, 그림에 모델 예측은 없습니다. 이 감사는 원본을 변경하지 않았으며 `dataset_admitted=false`로 남습니다. 추가 TRAIN 패키지는 별도 포장기가 같은 프레임·해시·중복 규칙을 독립 재검산해야 합니다.

Town06 p06의 사전 소형 표지판 조건은 전체 프레임 1–4 px **0/232로 실패**한 채 유지됩니다. 이 Town03 감사가 그 판정을 뒤집거나 작은 표지판 검출력·자동 정밀지도·실차 준비를 입증하지 않습니다. Town07 평가 원본은 여기서 열거나 사용하지 않았습니다.
