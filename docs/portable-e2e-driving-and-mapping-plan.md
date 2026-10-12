# 주행 모델과 자동 지도 생성 — 분리 개발 계획

<!-- HH_260906 - Separate proposed mapping work from implemented driving capability and measured results. -->

초안: 2026-09-22. 현황 갱신: 2026-10-01. 이 문서는 개발 방향이며 개별 결과와 승인 범위는 [최신 검증 요약](assets/validation/2026-10-01/README.md)과 각 실험 기록을 따른다.

## 10월 1일: 실제로 달라진 점

<!-- HH_261001 - Keep historical failures while linking new training and genuine closed-loop evidence. -->

**후속 갱신:** [최신 주행 결과](assets/validation/2026-10-01/warmup_supervision_ab/README.md)는
W1 20초 29.76m, 추가 150초 200.51m입니다. 반복 정지·종점 미완주·안전 여유 미달이
남아 있습니다. [지도 밝기·색 변화 비교](assets/validation/2026-10-01/map_photometric_endpoints/PLAN.md)는
각 10,000회 학습을 완료하고 4개 장면을 전수 평가 중입니다. 아래 항목은 10월 1일
초기 조사 기록이며, 새 학습·실제 제어가 없었다는 뜻으로 읽지 않습니다.

- **지도 학습 개선:** GPU 0에서 기존 구조 A와 넓은 문맥 구조 B를 각각 추가 10,000 step 학습했다. B는 두 평가 경로의 7개 클래스 모두 A보다 높았고, 이전 기준 대비 비교 조건 5개도 통과했다. 700개 예측 재집계·CPU 실제 가중치 재추론 검산까지 완료했다. [전체 지표·PNG/GIF](assets/validation/2026-10-01/map_context_recovery/README.md). 새 지도 일반화, 작은 객체 품질, CPU 10Hz는 아직 해결 전이며 자동 채택하지 않았다.
- **실제 모델 주행 연결:** AH 모델 출력이 CARLA 실제 명령으로 적용되는 저속 시험을 실행했다. 첫 시험에서 약 14.6m를 주행한 뒤 모델이 가까운 미래 속도를 0으로 예측해 정지했다. 전문가가 대신 주행한 기록이 아니다. 고정 소스로 재현하고, 현재 실행할 속도와 학습된 미래 궤적의 불일치를 개선한다. 목표 도달·기능 합격은 아니다.
- **새 우회전 수집 기반:** Town12 에디터 시작과 정확한 지도·노면·제어 신호 확인은 성공했다. 첫 6카메라 촬영은 시작점 차량 생성 실패로 원본을 보존하고 수정 중이다. 새 우회전 TRAIN은 아직 승인되지 않았다.
- **C-track 복구:** 읽기 전용 원본을 보호하는 격리 실행에서 도로/차선 semantic 라벨과 정상 조명 RGB를 확보했다. 좌회전·직진 연속 수집 후 이번 지도 A/B의 외부 진단에 사용한다. 이 평가 자료는 TRAIN에 편입하지 않는다.
- **기능 현황:** 9개 상위 영역·30개 하위 기능 가운데 승인된 자체 모델 폐루프 기능은 여전히 **0/30**이다. 이번에는 기본 차로 주행·속도·센서 입력·실행 안전을 개선하고 있다. 작업 중단이나 학습 미실행을 의미하지 않는다.

## 9월 29일까지의 상태 — 당시 실패 기록 보존

- 주행: [현재 소스](../learning/portable_e2e/model.py)의 6-camera, 차량 상태 이력, 제공 경로 입력으로 미래 궤적을 출력하는 자체 모델. 제어와 안전 검사는 모델 밖에 있다. 9월 29일에는 새 주행 학습을 하지 않았다. Town12 오프라인 전수 탐색은 [서로 다른 교차로 3곳의 명목 차체 우회전 후보](assets/validation/2026-09-29/drive_town12_right_static/README.md)를 찾았지만, [첫 소유 에디터 시작이 렌더 스레드 충돌로 실패](assets/validation/2026-09-29/drive_town12_right_static/LIVE_PROBE_FAILURE.md)하여 실제 수집이나 폐루프 성능으로 세지 않는다.
- 지도: 기존 AI v2 RGB 의미분할과 oracle 깊이·위치를 결합해 [서로 다른 출처의 누적 격자](assets/validation/2026-09-29/map_learned_projection_bridge/README.md)를 3프레임만 재현했다. [Town05 반복 DEV의 동일 배선](assets/validation/2026-09-29/map_town05_projection_hardcase/README.md)에서는 어두운 도로·회색 차선 오류가 누적 지도까지 이어졌다. [사진측광 후보](assets/validation/2026-09-29/map_photometric_pilot/README.md)와 [Town02 p00 추가 A/B](assets/validation/2026-09-29/map_town02_p00_route_holdout/AUDIT.md)를 실제 학습했으나 둘 다 Town01 보호 관문에 실패해 미채택이다. 기존 AI v2를 유지하며 새 일반화 품질·실제 센서 깊이·위치·HD map은 승인하지 않았다.
- 기능 원장: 9개 상위 영역·30개 하위 기능. 자체 모델 폐루프 승인 0/30. 지도 제작을 기존 기능의 완료로 세지 않는다.
<!-- HH_260929 - Report the real C-track capture failure without widening V2 data admission. -->
- [기존 지도 probe](../tools/e2e/collect_carla_mapping_probe.py)는 일부 고정된 Town에서 RGB/semantic/depth를 동기 수집한다. C-track에는 [별도 V2 정책](../learning/config/site_split_guard_20260929_v2.json)과 [전용 수집기](../tools/e2e/collect_carla_mapping_ctrack_v2.py)로 소유 촬영 후보 역할만 열었다. [실제 좌회전 Epic 240프레임](assets/validation/2026-09-29/map_ctrack_capture_readiness/LEFT_OWNED_01_RESULT.md)은 물리 주행·위치·깊이·해시·소유 절차는 통과했지만 `Roads`/`RoadLines` semantic 라벨이 전량 0개여서 [전수 QA](assets/validation/2026-09-29/map_ctrack_capture_readiness/left_owned_01_qa.json)가 실패했다. C-track 영상의 TRAIN 편입·독립 DEV는 없으며, 동일 맵의 직진 반복 대신 메시 태깅 원인을 먼저 조사한다. Town12는 여전히 오프라인 주행 경로 조사만 허용한다.
- 현재 dense route 입력을 쓰는 모델을 정밀지도 비의존 모델로 부르지 않는다. 미래 map-optional 비교에서는 양쪽에 동일한 거친 경로 안내를 주어야 한다.

## 트랙 A — 주행 능력

목표는 센서와 목적지 방향으로 시간별 XY·속도 궤적을 예측하는 portable planner다. Autoware 및 다른 제어기에는 adapter로 연결한다. 10 Hz는 전체 입력→출력 지연까지 검증할 목표이며 새 모델의 달성 결과가 아니다.

1. 직진·좌우 회전·차선 유지·속도 추종·종점 정지의 기본 품질 복구.
2. 정지 유지·재출발, 선행차 추종/ACC, 신호·정지선·보행자 대응.
3. 정적/동적 장애물 대응, 양보·합류·차선 변경. 기능별 데이터와 폐루프 회귀 시험 추가.

위 묶음은 개발 순서이지 기존 30개 원장을 대체하는 새 기능 수가 아니다. 기능마다 head를 하나씩 붙이는 것은 전제하지 않는다. 공통 공간 표현과 궤적 출력으로 여러 행동을 학습하고 필요성이 확인된 보조 출력만 별도 비교한다.

다음 주행 학습 전 기존 AH/K1 대비 직진/우회전과 속도 지표를 동일 조건에서 확인한다. 같은 DEV 개선을 독립 시험 통과로 간주하지 않는다. 우회전 성능이 부족하면 데이터 다양성·후보 생성/선택을 구분해 다음 학습을 설계한다. 지도 작업으로 이 후속 학습을 차단하지 않는다.

## 트랙 B — CARLA 지도 생성 첫 실험

첫 범위: Town01 짧은 직진 1구간과 회전 1구간. 전체 Town·신호등 연결·완전한 Lanelet2 생성은 첫 범위에서 제외한다.

산출물은 관측된 도로/노면 표시의 BEV raster 초안과 신뢰도·관측 횟수다. 이것을 완성된 차선 연결 그래프나 HD map으로 부르지 않는다. 관측되지 않은 영역은 unknown으로 남긴다.

### 구분해야 하는 네 단계

1. **정답 센서 기반 배선 실험:** 동기화된 RGB, CARLA semantic/depth, 카메라 보정, 차량 pose를 수집한다. semantic/depth 정답을 역투영·누적해 좌표 처리 및 시각화를 검사한다. 학습된 지도 모델이 아니다.
2. **중간 배선 실험:** RGB 학습 모델의 semantic 예측을 oracle depth/pose로 역투영한다. [Town01 배선 확인](assets/validation/2026-09-29/map_learned_projection_bridge/README.md)과 [Town05 반복 DEV 오류](assets/validation/2026-09-29/map_town05_projection_hardcase/README.md)가 각각 3프레임으로 여기에 해당한다. 학습 예측이라는 점은 1보다 나아갔지만, 정답 깊이·위치에 의존하므로 실차 지도 모델이라고 부르지 않는다.
3. **영상 기반 지도 모델:** RGB를 입력하고 정답 semantic/depth/지도는 학습 target 및 평가에만 사용한다. 실차에서 얻을 수 있는 깊이/측위 센서 또는 학습된 geometry를 결합해 CARLA 정답 geometry/depth 없이 추론해야 한다. 위치를 oracle pose로 받는다면 그 조건을 결과에 표시한다.
4. **지도 정밀화:** 위치 오차·가림·날씨 변화, 벡터화·차선 연결·규칙 관계를 별도 검증한다. 검토한 결과에 한해 Autoware 지도 변환을 진행한다.

CARLA OpenDRIVE/API 지도는 별도 reference로 보관한다. 이를 그대로 변환한 파일을 카메라 추정 결과로 제출하지 않는다. 렌더링 노면과 논리 지도 정의가 다를 수 있어 같은 종류의 대상끼리 비교한다.

### 비교와 자료

- RGB / 현재 프레임 추정 / 누적 지도 / 정답 / 오차를 같은 위치·축·범례로 표시한 PNG.
- 차량 중심 누적 과정을 보여주는 GIF. 정답 센서 기반인지 RGB 모델 기반인지 화면에 표시.
- 관측 영역의 도로·노면 표시 IoU, 경계 거리 오차, 관측 범위 및 unknown 비율, 처리 지연, 위치 누적 오차.
- 겹치는 연속 프레임을 무작위로 train/test 분리하지 않는다. 첫 두 구간은 plumbing 개발용이며 독립 test는 별도 경로 또는 Town으로 고정한다.
- 첫 지도 품질을 측정한 다음 주행 채택 기준과 별개의 지도 실험 기준을 고정한다. 시각화 생성 성공과 지도 정확도 통과는 구분한다.

## 두 트랙 연결은 나중에 별도 A/B

동일한 경로 안내, 장면, 제어기로 (A) 정밀지도 없음, (B) 검증 지도, (C) 자동 생성 지도, (D) 누락/오차 지도를 비교한다. 모델이 입력을 활용하도록 각각 학습하고 학습 예산도 기록한다. 경로 완료율·충돌·차선 이탈·정지·추론 지연을 평가한다. 지도와 관측의 불일치에는 감속/정지 등 불확실성 대응이 필요하다.

## 환경과 실행 경계

- 로컬: CARLA 수집·지도 재구성·PNG/GIF·추론/주행 시험. 실행 전 다른 테스트의 프로세스·GPU·포트 소유권을 확인한다.
- PRO6000: 주행 및 향후 지도 모델 학습. 물리 GPU 0만 사용하며 학습 job은 순차 실행한다.
- 설치는 hwanhong 개인 venv에만 가능. 9월 29일 원격 지도 A/B는 GPU 0 lease 안에서 실행했고 모두 미채택했으며, RGB→위치 지도 연결은 로컬 CPU만 사용했다. 이 문서 갱신 자체는 설치·시뮬레이터 실행·학습 시작·기존 작업 중단을 하지 않았다.
- 첫 수집/재구성 prototype 예상은 호환 환경이 준비된 경우 0.5–2 작업일이며 일정 보장이 아니다. RGB 지도 모델 학습·일반화 평가·실차 적용은 포함하지 않는다. 토큰 잔액이나 확정 소모량은 확인할 수 없으며 단계별 범위를 제한한다.

관련 실행 기록: [AD 전체 모델 추가 학습](assets/validation/2026-09-22/portable_e2e_learning_cycle_v1/21_full_continuation_ad/README.md).
