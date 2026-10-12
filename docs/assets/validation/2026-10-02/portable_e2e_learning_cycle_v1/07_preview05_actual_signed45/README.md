# 같은 주행 모델: 속도 추종 시점 0.3·0.5·0.8초 전체 비교

<!-- HH_261002 - Retain all nine actual attempts and every fixed45s failure while comparing one controller scalar. -->

전체 기준: 0.3초 0/3 · 0.5초 3/3 · 0.8초 0/3. 정식 메인 0/9 · 하위 0/30입니다.

| 출발 조건 | 추종 시점 | 전체 기준 | 전진 거리 | 실제 속도 최대 | 수신→ACK 최대 |
|---|---:|---|---:|---:|---:|
| negative_outward | 0.3초 | FAIL | 87.944 m | 2.475 m/s | 76.741 ms |
| negative_outward | 0.5초 | PASS | 90.591 m | 2.474 m/s | 82.410 ms |
| negative_outward | 0.8초 | FAIL | 92.926 m | 2.516 m/s | 77.980 ms |
| positive_outward | 0.3초 | FAIL | 89.362 m | 2.471 m/s | 77.167 ms |
| positive_outward | 0.5초 | PASS | 91.614 m | 2.473 m/s | 76.018 ms |
| positive_outward | 0.8초 | FAIL | 93.550 m | 2.514 m/s | 79.627 ms |
| centered_setup_comparator | 0.3초 | FAIL | 88.767 m | 2.472 m/s | 77.867 ms |
| centered_setup_comparator | 0.5초 | PASS | 91.206 m | 2.473 m/s | 79.609 ms |
| centered_setup_comparator | 0.8초 | FAIL | 93.266 m | 2.515 m/s | 80.368 ms |

가중치·NN/디코더·실제 v0·CPU4/1·PI2.4·좌우/중앙 출발·seed·45초·90m·속도2.5·차체10cm·100ms는 같습니다. 바뀐 것은 예측 속도에서 제어기가 따라갈 시점뿐입니다. 어느 비교 시점이든 0.05m/s 이하 또는 명시적 non-GO이면 원래0.3초 tracker를 통째로 사용합니다.

모든 fresh 선택64점·원 tracker·PI 적분·ACK 5개 필드를 독립 재계산합니다. held 명령은 PI를 다시 적분하지 않습니다. 추가20Hz actor world 속도 변화율 +3/−6 검사는 첫 driving 직전 warmup 경계도 포함합니다. 벡터 가속도와 abs2.9 관측은 별도이며 학습 승인으로 바꾸지 않습니다.

새0.5초의 3조건·2,700상태·1,350 실제6카메라 묶음·8,100JPEG·2,712ACK·실패 꼬리와 기존0.3/0.8의 모든 결과를 보존합니다. 좋은 실행 한 번을 골라 전체 성공으로 세지 않습니다. 제한된 Town07 저속 직진 비교이며 회전·종점 정지·30km/h·다른 맵·실차 승인, 센서 노출부터 전체10Hz의 증거가 아닙니다.

[전체 결과 JSON](batch_outcome.json) · [원자료/소스SHA](source_evidence.json)

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
