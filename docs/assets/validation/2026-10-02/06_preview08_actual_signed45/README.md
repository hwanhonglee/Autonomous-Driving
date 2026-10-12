# 같은 주행 모델: 속도 예측을 0.3초 → 0.8초 앞에서 따라본 결과

<!-- HH_261002 - Report all actual arms, complete native-rate failures and the unchanged learned model. -->

전체 기준: 이전 0/3 → 새 비교 0/3. 좌·우 복귀는 2/2이며 중앙 조건은 별도입니다.

| 조건 | 이전 → 새 전체 기준 | 전진 거리 | 속도 최대 | 수신→ACK 최대 | 새 가감속 검사 |
|---|---|---:|---:|---:|---|
| negative_outward | FAIL → FAIL | 87.944 → 92.926 m | 2.475 → 2.516 m/s | 76.741 → 77.980 ms | PASS (0위반) |
| positive_outward | FAIL → FAIL | 89.362 → 93.550 m | 2.471 → 2.514 m/s | 77.167 → 79.627 ms | PASS (0위반) |
| centered_setup_comparator | FAIL → FAIL | 88.767 → 93.266 m | 2.472 → 2.515 m/s | 77.867 → 80.368 ms | PASS (0위반) |

바뀐 것은 제어기가 이미 예측된 속도의 어느 시점을 따르는지(0.3→0.8초)뿐입니다. 가중치·모델 궤적·실제 v0·CPU4/1·PI2.4·출발/seed·45초·90m·속도2.5·차체/10cm·100ms는 그대로입니다. 두 시점 중 어느 속도든 0.05m/s 이하이거나 명시적 non-GO이면 원래 0.3초 tracker를 통째로 사용합니다. STOP 학습 완료나 새 모델 개선을 뜻하지 않습니다.

추가로 원자료 20Hz의 actor XY 속도 변화율 +3/−6m/s²를 전체 driving 구간(직전 warmup 경계 포함)에 확인합니다. 이전 세 실행에도 같은 추가 검사를 적용하되 기존 결과는 수정하지 않습니다. 가속도 벡터 크기와 절대 변화율 2.9는 서로 다른 별도 관측이며 데이터 학습 승인으로 바꾸지 않습니다.

전체 3조건·2,700상태·1,350 실제 6카메라 묶음·8,100JPEG·2,712ACK와 실패 꼬리를 남깁니다. 매 normal 설정, 모든 fresh 선택 속도·원 tracker·PI 적분·실제 ACK를 독립 재계산했고 held 명령은 PI를 다시 적분하지 않았습니다. 센서 노출부터 완전한10Hz는 미측정입니다.

정식 메인 0/9 · 하위 0/30. 제한된 Town07 저속 직진 비교이며 회전·종점 정지·30km/h·다른 맵·실차 승인이 아닙니다.

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
