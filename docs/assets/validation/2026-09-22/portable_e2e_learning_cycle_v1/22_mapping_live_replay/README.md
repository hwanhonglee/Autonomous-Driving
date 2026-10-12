# 자동 지도 생성 01 — 실시간·기록 재생 공통 기반

<!-- HH_260906 - Lead with real owned sensor results while retaining the earlier implementation record. -->
## 이번에 실제로 확인한 결과

Town01 직진·좌회전 **각 32개, 총 64개 센서 묶음**을 새로 수집했다.
RGB/depth/semantic의 프레임 번호·시간·센서 pose를 맞췄고, 저장된 원시 센서에서 재구성한 지도는
현장 처리 지도와 두 구간 모두 정확히 일치했다. 두 전용 CARLA 서버는 소유 wrapper가 정상 종료했고
2180–2182 포트도 해제됐다. 다른 시험이나 원격 학습 작업은 변경하지 않았다.

**이것은 CARLA 정답 센서와 정답 위치를 이용한 지도 배선 시험이다. 카메라 인식 모델 학습,
자율주행, 실차 지도 정확도 검증이 아니다. 차량은 물리 주행하지 않고 지정 위치로 옮겼다.**

| 구간 | 수집 프레임 | 관측 격자 (0.2m) | 현장/재생 일치 | 시각 자료 |
|---|---:|---:|---|---|
| 직진 | 32 | 84,563 | 일치 | [PNG](straight_v1/comparison_0016.png) · [GIF](straight_v1/ego_centered_mapping.gif) |
| 좌회전 | 32 | 30,517 | 일치 | [PNG](turn_v1/comparison_0031.png) · [GIF](turn_v1/ego_centered_mapping.gif) |

![회전 구간 카메라·정답 분류·현재 관측·누적 지도](turn_v1/comparison_0031.png)

### 화면 읽는 법

- 위 왼쪽: 실제 렌더링 RGB. 위 오른쪽: CARLA 정답 분류, 모델 예측 아님.
- 아래 왼쪽: 현재 센서 관측. 아래 오른쪽: 누적 지도. 두 패널 모두 차량 중심, 전방은 위.
- 노랑은 노면 표시, 파랑은 도로, 보라는 보도. 흰 점은 별도로 겹친 논리 차선 중심 참고선.
- 주황 사각형은 차량, 청록 점은 이동한 센서 위치. 관측하지 않은 공간을 도로로 채우지 않음.
- GIF는 수집된 0.3초 간격의 묶음을 0.15초씩 재생하는 약 2배속 시각화다. 실시간 10Hz 달성 증거가 아니다.

### 품질 한계와 검사 범위

원거리의 줄무늬/빈틈은 영상 점을 성기게 투영한 관측이다. 벡터 차선 연결·누락 보완은 아직 없다.
현재 API의 RoadLines는 일반 노면 표시이며 횡단보도/정지선/차선의 별도 인식 완료를 의미하지 않는다.
신호등 설치물은 관측되지만 신호 색이나 통제 차선은 추정하지 않았다.

별도 논리 지도 중심선과 차선 폭으로 만든 직사각형 합집합을 참고하여 road 점 일부를 비교했다.
직진 10,197점 중 84점, 회전 10,809점 중 168점이 참고 영역 밖이었다.
이는 **도로 인식 정확도 또는 센티미터급 정밀지도 인증 수치가 아니다.** 논리 지도와 렌더링 노면의
정의 차이, 직사각형 근사, 중복 관측, 정답 pose/depth 사용이 포함된 진단이다.
원시 측정값: [직진](straight_v1/metrics.json), [회전](turn_v1/metrics.json).

설치된 client/server는 모두 `0.9.15-dev-dirty`다. 설치 enum의 Roads=1, RoadLines=24를 사용했다.
일부 [0.9.15 온라인 센서 문서](https://carla.readthedocs.io/en/0.9.15/ref_sensors/)의 이전 분류표와
달라 하드코딩하지 않고 실제 enum을 기록했다. 깊이 BGRA 해석·UE 좌표축도 별도 단위 검사했다.

### 재현과 원본

원본 루트: `artifacts/validation/2026-09-22/mapping_sensor_probe/{straight_v1,turn_v1}`.
각 루트의 owner_plan/result, lifecycle, provenance 및 mapping/ 아래 센서 NPZ·RGB·pose·지도·summary를 보존했다.
새 실행은 존재하지 않는 출력 디렉터리를 지정해야 하며 기존 결과를 덮어쓰지 않는다.

```bash
# HH_260906 - Run only in an idle owned workspace; this command performs a kinematic oracle scan.
bash scripts/e2e/run_owned_carla_expert_trial.sh \
  artifacts/validation/2026-09-22/mapping_sensor_probe/turn_v2 \
  artifacts/validation/2026-08-31/town01_left_export/route.json \
  --capture-mode mapping-probe --port 2180 --quality Epic --wall-timeout-sec 240 -- --frames 32
```

오프라인 원시 센서 재생과 자료 생성:

```bash
python3 scripts/e2e/render_mapping_probe.py \
  artifacts/validation/2026-09-22/mapping_sensor_probe/turn_v1/mapping \
  /tmp/mapping_replay_new_output
```

추가 구현: `mapping_projection.py`, `collect_carla_mapping_probe.py`, `render_mapping_probe.py`.
최종 합동 회귀 검사 **102개 통과 (55.54초)**: 공통 누적·투영·수집·오차분해 27개와 owner wrapper 75개.
별도 패키지 설치 없음. PNG 6개·GIF 2개이며 RGB 인식 모델 학습은 시작하지 않았다.

## 다음 우선순위

1. 주행: AD 우회전 후보 선택 gap과 직진 후보 퇴행을 함께 개선할 학습 설계. 지도 작업을 선행 조건으로 두지 않음.
2. 지도: 실제 물리 주행 기록에서도 동일한 동기화/좌표 검사를 통과시키기. 현재 위치 이동 스캔을 학습 주행 기록으로 둔갑시키지 않음.
3. RGB 인식 학습용 독립 route/Town 분할과 라벨 준비. 현재 64개는 배선 개발용이며 모델 학습·일반화 입증에 충분하지 않음.
4. 도로/노면 인식 baseline 학습 후 지도 품질 비교. 신호등 상태·차선 관계 등은 별도 라벨과 평가를 추가.

---

## 아래는 최초 기반 구현 당시 기록

<!-- HH_260906 - Report tested observation fusion separately from perception, simulator capture and HD-map quality. -->

## 이번 구현

`portable_e2e/mapping.py`에 동일한 update()를 사용하는 실시간 관측 누적과 replay()를 구현했다.
입력은 이미 보정·동기화·지도 좌표 변환된 (x, y, 분류, 신뢰도) 관측이다.
RGB 인식기나 카메라 역투영은 아직 없다. 명칭은 자체 지도 연구 트랙이며 ASSURE AMG 구현이나 제품 연동이 아니다.

- 도로, 노면 표시, 도로 경계, 보도, 횡단보도, 정지선, 신호등 설치물, 표지판, 기둥을 정적 정보 분류로 선언.
- 차량, 보행자, 자전거, 신호 상태는 현재 프레임 정보로 분리. 신호 상태의 색상·설치물 ID 연결은 아직 별도 계약이 필요하다.
- oracle_sensor와 learned_prediction 출처를 분리하고 혼합 거부.
- 시간 역행·중복, 좌표계 불일치, 비정상 값 및 용량 초과 거부. 실패 프레임은 기존 지도를 변경하지 않음.
- 한 프레임의 빽빽한 점들을 독립 관측 횟수로 중복 계산하지 않음.
- 상충 분류는 그대로 보존. 합산 confidence를 보정된 확률이라고 해석하지 않음.
- 미관측은 unknown. 정적 정보의 시간 감쇠·지도 변경 반영·객체 association은 미구현.

검증: `python3 -m pytest -q tests/test_mapping.py` → **11 passed**.
합성 점 관측으로 처리 논리를 검사한 결과이며 실시간 성능·인식 정확도·지도 정확도 검증이 아니다.
신규 설치, GPU 실행, CARLA 센서 수집, 이미지 생성은 하지 않았다.

## 다음 실험

1. 별도 소유 CARLA 세션에서 RGB/semantic/depth/pose를 동기 수집하는 adapter 구현.
2. metric 좌표로 투영해 공통 update()와 기록 replay() 결과를 비교.
3. Town01 짧은 직진·회전의 RGB/관측 BEV/누적 지도/정답 오차 PNG와 GIF 생성.
4. 정답 센서 기반 배선 확인 후, RGB 인식 모델 학습으로 교체. 여러 학습 job은 GPU 0에서 순차 실행.

첫 버전은 노면 분류 지도로 제한한다. 신호등 위치·색상·차선 연결, 표지판 내용 등은 별도 라벨과 인식 시험을 추가한다.
원하는 모든 항목을 선언했다고 인식 기능을 구현한 것은 아니다.

전체 방향: [주행과 지도 분리 계획](../../../../../portable-e2e-driving-and-mapping-plan.md).
