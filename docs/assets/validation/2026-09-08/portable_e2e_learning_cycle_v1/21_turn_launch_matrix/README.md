# C-track 좌회전: 출발 페달 4종 × 같은 조건 2회

CARLA의 **BasicAgent 전문가 주행 수집**입니다. 사람이 수동 운전한 영상도, 학습 모델 또는 Autoware 자율주행 폐루프 시험도 아닙니다.

총 8개를 모두 남겼습니다. 스칼라 품질만 2/8 PASS, 6/8 FAIL이며 **학습 데이터 승인·자동 최적 설정 선택은 0건**입니다. 목표는 14.4 km/h로, 30 km/h 검증 결과가 아닙니다.

설정: 같은 C-track 경로/시드, Epic, 물리 20 Hz, 카메라 10 Hz, 640×360 카메라 6개. 매번 새 소유 CARLA를 실행했지만 반복별 요약값이 같으며 다양한 경로나 무작위 실험 8개를 의미하지 않습니다.

## 전체 결과

| 순서 | 출발 throttle / 반복 | 스칼라 | 20 Hz native 최대 / 10 Hz camera 최대 (m/s²) | 목표 오차(m) | 화면 폴더 |
|---|---|---|---|---|---|
| 1 | 0.15 / 1 | FAIL | 3.467811 / 2.715636 | 0.952436 | [01_turn_launch_015_v1_r1](./01_turn_launch_015_v1_r1/) |
| 2 | 0.13 / 1 | PASS | 2.399301 / 2.237004 | 0.949681 | [02_turn_launch_013_v1_r1](./02_turn_launch_013_v1_r1/) |
| 3 | 0.14 / 1 | FAIL | 3.625722 / 2.478957 | 0.949947 | [03_turn_launch_014_v1_r1](./03_turn_launch_014_v1_r1/) |
| 4 | 0.12 / 1 | FAIL | 3.987333 / 2.001571 | 0.940470 | [04_turn_launch_012_v1_r1](./04_turn_launch_012_v1_r1/) |
| 5 | 0.12 / 2 | FAIL | 3.987333 / 2.001571 | 0.940470 | [05_turn_launch_012_v1_r2](./05_turn_launch_012_v1_r2/) |
| 6 | 0.14 / 2 | FAIL | 3.625722 / 2.478957 | 0.949947 | [06_turn_launch_014_v1_r2](./06_turn_launch_014_v1_r2/) |
| 7 | 0.13 / 2 | PASS | 2.399301 / 2.237004 | 0.949681 | [07_turn_launch_013_v1_r2](./07_turn_launch_013_v1_r2/) |
| 8 | 0.15 / 2 | FAIL | 3.467811 / 2.715636 | 0.952436 | [08_turn_launch_015_v1_r2](./08_turn_launch_015_v1_r2/) |

모두 목표 도달·2초 정지 및 governor/ACK/초기화 검사 통과. 출발 페달 .13 두 회만 native 가속도 스칼라 검사 통과했습니다. .12/.14/.15의 실패는 더 이른 출발 구간에 기록되었고, 인계 주변 최대값은 모두 2.9 m/s² 아래입니다. 이후 throttle ramp 시작값은 모든 설정에서 .15로 동일하며, 이 인계가 실패 원인이라고 단정하지 않습니다.

[전체 주행 속도·가속도](01_whole_record_speed_acceleration.png) · [출발 0–8초 계측](02_launch_window_acceleration_throttle.png) · [8개 독립 원자료 감사](audit.json)

실제 카메라 10 Hz 간격의 속도 차분은 8회 모두 스칼라 기준 이내지만 native 20 Hz에서는 6회가 실패합니다. 관측 간격이 순간 변화의 크기를 다르게 보이게 할 수 있습니다. 화면이 부드러워 보여도 물리 검사 통과가 아니며, 이 결과만으로 모든 GUI 끊김의 원인을 카메라 Hz라고 결론내리지 않습니다.

## 화면 보는 법

각 폴더의 01은 첫 driving 카메라, 02는 원래 경로의 LEFT 구간 거리 중간점에 가장 가까운 실제 카메라, 03은 goal-complete가 실제 기록된 첫 카메라입니다. 차량은 지도 중앙, 6개 원본 카메라는 화각을 자르지 않은 letterbox입니다. 궤적은 관측된 과거/미래이지 모델 예측이 아닙니다. 정지 끝부분의 없는 미래를 채우지 않았습니다.

04 GIF는 **첫 driving 카메라부터 0–8 시뮬레이션 초만** 담은 **2배속 출발 미리보기**입니다. 전체 경로 영상이나 실제 20 Hz 화면 재생 증명이 아닙니다. 원본 10 Hz 카메라를 stride 2로 골라 10 fps로 재생합니다. 전체 구간 수치·실패는 위 전체 그래프와 audit.json에 별도 보존했습니다.

전체 분모: native 15,466개, 카메라 시점 7,736개, JPEG 46,416개. 모든 JPEG의 SHA와 640×360 디코딩 검사를 독립 감사에서 확인했습니다. 디코딩은 장면/재질 품질 승인이나 전체 미래 XY 품질 승인이 아닙니다. 제어값은 reported/ACK 관측이며 물리 토크 적용 시점 증명이 아닙니다.

## 출처와 재생성

audit.json은 검토된 원본 SHA와 동일한 경로 비식별 감사입니다. 각 image_hashes.json은 원본 JPEG 전체 SHA 목록이며 원본 JPEG 자체는 배포하지 않습니다. publication_manifest.json에는 감사·렌더러 소스 SHA, 시점 선택과 출력 SHA가 들어 있습니다. 원본 실패, 이전 7개 진행 검토서의 당시 제한 사항, 별도 재개 승인을 덮어쓰지 않았습니다.

git clone만으로 원본 수집 데이터를 얻거나 시험이 자동 실행되지는 않습니다. 아래 재생성에는 개인 보관 원본 8회, 연결된 이전 감사 입력, 해당 기록에 필요한 Git 소스 객체가 있어야 합니다. 설치·학습·CARLA 재실행 없이 읽기 전용으로 새 폴더에 렌더링하며, 기존 출력 경로는 거부합니다. 현재 Python 환경의 Pillow/Matplotlib와 ffmpeg가 필요합니다.

```bash
python3 -m scripts.e2e.curate_carla_turn_launch_matrix \
  --campaign-root artifacts/training/2026-09-08/turn_launch_matrix_v1 \
  --audit-root artifacts/training/2026-09-08/turn_launch_matrix_audit_v1 \
  --output-dir artifacts/training/2026-09-08/turn_launch_matrix_publication_replay_v1
```

정식 학습 편입·모델 개선·실차 운용 승인은 별도입니다. 이번 자료를 통과한 학습 데이터나 새 모델 성능으로 사용하지 않습니다.
