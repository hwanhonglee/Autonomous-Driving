# METEOR 검토와 Portable E2E의 다음 개발 단계

<!-- HH_260906 - Use pinned primary METEOR references to inform our own model while keeping release claims, local measurements and future proposals separate. -->

검토일: **2026-09-14**. 결론은 **설계·학습·평가의 참고 대상으로 반영하되, 기존 모델을 즉시 교체하지 않는다**이다.
METEOR를 설치하거나 가중치를 실행한 성능 비교는 아직 하지 않았다. 공식 문서·배포 입력 규격·
평가 코드를 읽은 결과이며, 아래 개선안은 우리 프로젝트의 판단이다.

검토 기준은 TIER IV 저장소 commit
[`dc193a81e9959de57b751e269127ae9b67e3f300`](https://github.com/tier4/METEOR/commit/dc193a81e9959de57b751e269127ae9b67e3f300),
공식 모델 artifact revision `01a5f6d71df5ecbbb5853ec600825481d57b9c6b`다.
GitHub 소스 버전과 모델 artifact의 `v1.0` 표기를 혼동하지 않는다.

## 1. 무엇이 참고할 만한가?

METEOR는 영상의 공통 특징과 기하학적인 BEV 표현을 공유하면서 차로·깊이·객체·점유공간·
경로 등 여러 출력을 함께 학습하는 연구 모델이다. ResNet-34/FPN 기반 v52 모델과 v157
가중치가 공개되어 있다. 우리는 이 **공유 공간 표현 + 보조 정답 학습** 방식을 참고한다.
현재의 작은 CNN/GRU 경로 예측기에 설명 가능한 공간 정보가 부족하다는 설계상 차이를
다루기 위한 후보이지, METEOR가 우리 데이터에서 더 낫다고 측정한 결론은 아니다.
[고정 모델 카드](https://huggingface.co/AutowareFoundation/meteor/blob/01a5f6d71df5ecbbb5853ec600825481d57b9c6b/README.md)

우리의 첫 적용 순서는 차로/주행 가능 영역·깊이 정답의 확보와 검증, 작은 보조 head,
그다음 객체/점유공간·신호 정보다. 출력 head를 만든 것과 그 행동을 안전하게 학습한 것은
구분한다. METEOR의 12개 학습 작업도 우리 로드맵의 30개 운전 기능과 일대일 대응하지 않는다.
공개 ONNX의 19개 출력 tensor는 작업 수와도 다른 단위이며, occupancy flow 등 일부 checkpoint
출력은 배포 graph에서 제외된다.
[실제 export 경로](https://github.com/tier4/METEOR/blob/dc193a81e9959de57b751e269127ae9b67e3f300/deploy/export_onnx.py)

## 2. 바로 연결할 수 없는 이유

| 항목 | 현재 Portable E2E | METEOR 공개 camera-only ONNX |
| --- | --- | --- |
| 카메라 | 6대, 고정된 자체 순서 | 광각/측면/협각을 포함한 8대, 별도 고정 순서 |
| 영상 tensor | 정규화 RGB float, 320×180 | RGB uint8, 768×432; 정규화는 graph 내부 |
| 외부 보정 방향 | camera → base 성분을 embedding | ego → camera 전체 행렬 |
| 주행 의도 | route XY + command + ego 상태 이력 | 공개 graph 입력은 영상·K·외부 보정·현재 속도; route/command 입력 없음 |
| 후보 경로 | 6 또는 12개, 0.1초 간격 64점 / 6.4초 | 3개, 0.5초 간격 6점 / 3초 |
| 시간 정보 | 현재 영상 + 차량 상태 이력 | 배포본은 단일 영상 시점 |

METEOR 쪽 규격 근거:
[고정 artifact parameter](https://huggingface.co/AutowareFoundation/meteor/blob/01a5f6d71df5ecbbb5853ec600825481d57b9c6b/meteor_v157.param.yaml),
[실제 ONNX 입력 구성](https://github.com/tier4/METEOR/blob/dc193a81e9959de57b751e269127ae9b67e3f300/deploy/export_onnx.py#L657).
Portable 쪽은 [현재 구현 설명](portable-e2e-model-v0.md)과 [모델 코드](../learning/portable_e2e/model.py)를 따른다.

따라서 **6개 영상을 8개 슬롯에 복제하거나 TF만 바꾸고 호환된다고 판단하지 않는다**.
3초 경로를 보간해도 6.4초 예측 능력이 생기는 것은 아니다. checkpoint에 있는 command 기능과
현재 공개 ONNX의 입력도 다르다. 우리 route-aware planner와 10 Hz 출력 ABI는 유지한다.

## 3. 공개 결과에서 주의할 부분

- **Autoware 완성 노드가 아니다.** 공식 카드는 별도 runtime을 가진 reference model이며 ROS 2
  노드는 아직 없다고 설명한다. 다른 센서 rig의 정확도와 공도 운용을 보장하지 않는다.
  [모델 제한](https://huggingface.co/AutowareFoundation/meteor#limitations)
- **평가 이름만으로 폐루프 성공을 세지 않는다.** 공식 `closed_loop_eval.py`는 스스로
  quasi-closed-loop라고 설명한다. 기록 영상에서 궤적 오차를 누적하고 보정값을 바꾸는 진단이며,
  모델의 행동으로 다음 영상과 차량 상태가 바뀌는 CARLA 시험이 아니다.
  [평가 코드](https://github.com/tier4/METEOR/blob/dc193a81e9959de57b751e269127ae9b67e3f300/bevlane/closed_loop_eval.py#L1)
- **속도는 장비·변환 조건부다.** 공식 Orin 약 67.4 ms는 INT8 sparse·CUDA Graph 등의 조건에서
  잰 추론 중앙값이다. 우리 PC의 센서 동기화부터 제어까지 걸리는 시간이나 p99 보장이 아니다.
  공개 benchmark 정확도와 전체 학습 corpus도 이번 release에 제공되지 않는다.
  [배포 결과](https://github.com/tier4/METEOR/blob/dc193a81e9959de57b751e269127ae9b67e3f300/README.md#edge-deployment-jetson-agx-orin)
- **과거 영상 학습은 그대로 따라 하지 않는다.** 공식 README는 history cache 오류로 실제
  과거 영상이 학습에 들어가지 않았고 release에서 history를 제거했다고 기록한다. 우리도
  영상 이력을 추가할 때 입력 존재·시간 정렬·교란 반응을 먼저 검사한다.
  [시간 정보에 대한 공식 설명](https://github.com/tier4/METEOR/blob/dc193a81e9959de57b751e269127ae9b67e3f300/README.md#temporal-memory--present-in-the-model-not-in-the-deployment-honest-note)
- **문서의 모델 버전이 섞여 있다.** `docs/ARCHITECTURE.md`는 과거 v26 설명이다. 그 표의
  파라미터/출력 수를 현재 v52 artifact의 규격으로 사용하지 않는다.
  [과거 구조 문서](https://github.com/tier4/METEOR/blob/dc193a81e9959de57b751e269127ae9b67e3f300/docs/ARCHITECTURE.md)

## 4. 실제 데이터와 자동 정답은 어떻게 참고하나?

METEOR의 정답 생성은 기록된 LiDAR·pose·panoptic 정보 등을 사용한다. “자동 정답”은
정답 없이 학습한다는 뜻이 아니다. 궤적·제동 추정, 가려진 객체 제외, 정지 장면의 coverage
필터도 목적에 맞는지 따로 살펴봐야 한다. 우리 정지 학습에서 해당 필터를 그대로 쓰면
필요한 장면을 잃을 수 있다는 점은 **우리의 위험 분석**이다.
[정답 생성 문서](https://github.com/tier4/METEOR/blob/dc193a81e9959de57b751e269127ae9b67e3f300/docs/DATA_PIPELINE.md)

공식 demo card는 전체 정답 디렉터리를 배포하지 않는다고 설명한다. 또한 모든 demo가
검증용이라는 첫 설명과 두 장면은 TRAIN이라는 후속 설명, 익명화 여부, 이용 조건에 서로
충돌하는 문장이 있다. 이번에는 demo를 학습/test에 편입하거나 영상을 재배포하지 않았다.
모델의 Apache-2.0 표기와 별도 demo 데이터의 조건은 분리해 확인한다.
[공식 demo 데이터 카드](https://huggingface.co/datasets/AutowareFoundation/meteor-demo-scenes)

우리 계획은 CARLA 원본의 semantic/depth/actor 정보와 실측 데이터의 이용 가능한 정답을
각각 검증해 동일 표현으로 변환하는 것이다. 출처가 없는 항목은 **NOT_RECORDED / unavailable**로
남긴다. 검출값을 검증된 정답으로 승격하거나, 없는 객체 label을 “객체 없음”으로 채우지 않는다.
현재 Common10은 objects/occupancy payload가 정의되기 전 `available=true`를 거부하므로,
추가 정답은 별도 버전·유효 mask·검증기를 먼저 설계해야 한다.

## 5. 이번에 반영하는 작업과 다음 학습

<!-- HH_260906 - Prioritize independent supervision coverage before introducing new multitask losses or adopting a reference checkpoint. -->

**실제 완료:** [9월 14일 데이터 준비 상태 검사](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/01_data_readiness/README.md)를
기존 v3에 실행했다. TRAIN/VAL 1,147/337개, 정지 유지 118/0개이며 TRAIN의 118개도
Town01 한 episode에 몰려 있다. 차로·깊이·신호 정답은 미기록, 객체·점유공간은 unavailable이다.
관련 CPU 테스트 143개 통과. 모델 설치·추론·새 학습이 아니라 다음 학습의 부족분을 확인한 결과다.

1. **완료 — 현재 데이터의 split별 준비 상태 계측:** TRAIN/VAL을 episode·route-group·날씨와 함께
   집계하고 정지 유지·감속 정지·이동의 시간 표본 수를 분리한다. 같은 경로 반복이나 겹치는
   미래 창을 독립 장면 수로 세지 않는다. TEST의 행동·성능 분석은 하지 않는다.
2. **독립 회전·정지 수집:** 실패했던 원본 가감속·저속 곡률 문제는 그대로 미승인이다.
   품질을 충족한 새로운 episode와 정지 이유/신호/선행차 정보가 필요하다. 기존 판정은 완화하지 않는다.
3. **작은 다중 작업 모델 A/B 설계:** 기존 route-aware planner를 A로 보존하고,
   먼저 B의 보정 기반 공간 특징만 비교한다. 이어 고정한 구조에서 검증된 차로·깊이 보조 loss를
   각각 비교한다. 구조와 보조 손실을 한 번에 바꾸지 않으며 아직 head 구현·학습 완료는 아니다.
4. **학습 전 점검:** 소수 TRAIN overfit, missing-label loss mask, 실제 optimizer step·유한 gradient,
   후보 최선 오차와 선택 오차, 직진/회전·정지 subgroup을 함께 검사한다. 구조·손실 변경은 하나씩 분리한다.
   [참고한 학습·평가 분해](https://github.com/tier4/METEOR/blob/dc193a81e9959de57b751e269127ae9b67e3f300/docs/TRAINING.md#validation-metrics-printed-every-epoch)
5. **원격 3-seed 학습 → 로컬 실행 검사:** GPU 0·지정 개인 venv만 사용한다. 원본 데이터는
   학습 중 바꾸지 않는다. export 동등성·실측 10 Hz·안전 정지·CARLA 직접 주행은 별도 통과 단계다.
6. **METEOR 자체 실행 비교는 별도:** artifact/환경/센서 계약을 갖춘 뒤 격리된 offline reference로
   측정한다. 공식 설치 명령·다중 GPU 설정·안전하지 않은 checkpoint loader를 그대로 실행하지 않는다.

최종 방향은 **METEOR로 갈아타기**가 아니라 **우리 모델을 유지하면서 공간 이해·정답 준비·평가
방법을 개선하고, 동등 조건의 비교 대상으로 활용하기**다. 데이터 품질이나 독립 시험이 없는
상태에서 모델 크기만 키우는 것은 이번 단계의 완료 조건이 아니다.

[기존 재개 현황](portable-e2e-status-2026-09-14.md) · [30개 기능 로드맵](portable-e2e-feature-roadmap.md)
