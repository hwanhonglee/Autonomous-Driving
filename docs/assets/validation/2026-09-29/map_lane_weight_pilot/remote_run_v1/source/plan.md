# TRAIN 경로 holdout 차선 손실 2× A/B — 실행 전 계약

<!-- HH_260929 - Freeze nine paired route-holdout comparisons before opening GPU0; stop after the first failed pair. -->

**상태: 2026-09-29 사전 등록.** [TRAIN 감사](../map_lane_train_audit/README.md)에서 Town01 497장과 Town02_Opt p02 232장의 원본 RGB·semantic·depth·pose 전수 대조가 통과했다. 이번 실험은 고정 D1 자료 안의 세 Town01 경로를 번갈아 통째로 제외하고, `lane_marking` 손실 계수 하나만 2.0배 하는 후보를 같은 조건의 기준 팔과 비교한다. **기존 Town05 DEV와 모든 DEV/TEST 배열은 이 실행에서 열지 않는다.** 결과가 좋아도 모델을 승격하지 않는다.

## 불변 입력과 분할

- 데이터는 [D0/D1 manifest](../../../../../storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2/manifest.json) SHA-256 `313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6`의 `train_d0.npz` SHA-256 `69dae433616cf27690310af9ff1923ef3b2e67641fa935e2a2f77116eea02ace`와 `train_d1.npz` SHA-256 `4e2f7df61814a7e8aade88877482506a30e41c9fff0b5e6b7c3182193ff7d8f8`뿐이다. D1 index `[0,497)`의 RGB·라벨이 D0와 정확히 같아야 한다. DEV 파일은 manifest의 이름·해시 메타데이터만 확인하고 파일 자체에는 접근하지 않는다.
- [Town01 AI v2 부모 manifest](../../../../../storage/datasets/prepared/map_expanded_20260926_ai_v2/manifest.json) SHA-256 `e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c`의 `source_records`는 index `[0,114)` 기존 직진, `[114,301)` p03, `[301,497)` p01이다. [TRAIN 감사 문서](../map_lane_train_audit/README.md) SHA-256 `9dd002dee9ae4829724a2b073163b22c35cb97c25453652d384374f56a065375`의 원본 대조를 전제로 한다. 세 경로의 최소 ego XY 거리는 각각 212.608m, 171.887m, 212.710m다. 물리 구간 분리에 도움을 주지만 영상 시야 독립성이나 Town 일반화를 증명하지 않는다.
- fold `historical`은 `[0,114)` holdout, `[114,497)∪[497,729)` TRAIN; `p03`은 `[114,301)` holdout, `[0,114)∪[301,729)` TRAIN; `p01`은 `[301,497)` holdout, `[0,301)∪[497,729)` TRAIN이다. Town02 p02는 항상 TRAIN이고 holdout에는 없다. 배열은 메모리에서 index view로만 나누며 원본 NPZ·manifest를 수정하거나 새 데이터셋으로 덮어쓰지 않는다.

## 정확히 한 요인과 판정

- 각 fold·seed에서 기준 `B`와 후보 `W2`는 동일한 7클래스 `MapPerception`, RGB 640×360, scratch 초기화, AdamW learning rate `3e-4`/weight decay `1e-4`, batch 8, 2,000 step, gradient clipping 5.0, 균등 프레임 샘플러, TRAIN-only 동일 `paired_crop`(증강 RNG `20260926`)을 사용한다. seed는 순서대로 `20260922`, `20260923`, `20260924`다. 팔마다 같은 seed로 초기화하고 초기 state hash의 일치를 확인한다. 기존 D1 checkpoint에서 이어 학습하지 않는다.
- `sqrt_frequency` 기본 클래스 가중치는 **각 fold의 학습용 Town01 두 경로에서만** 계산한다. holdout 라벨은 가중치·샘플링·학습에 사용하지 않는다. 기준과 후보는 이 동일한 기본 벡터를 사용하며 W2만 index 3 `lane_marking` 항을 정확히 `2.0`배 한다. 다른 여섯 계수·손실식·데이터·평가 후처리는 같게 둔다. 최종 full-D1 기존 실험의 D0 497장 가중치 계약은 이 별도 탐색에서 변경하지 않는다.
- 평가에는 제외한 Town01 경로의 원본 640×360 라벨만 사용한다. 7클래스 혼동행렬과 IoU를 두 팔 모두 기록한다. **각 짝의 통과 조건:** W2 차선 IoU − B 차선 IoU `≥0.05`, W2 도로 IoU − B 도로 IoU `≥−0.01`. 계산은 보관된 전체 픽셀 혼동행렬에서 한다. 둘 중 하나가 실패하면 그 짝을 완결 기록하고 **즉시 중단**하여 다음 fold/seed를 실행하지 않는다. 9개 짝이 모두 통과해야만 이 내부 탐색을 통과한 것으로 표시한다. 그 경우에도 자동 승격은 `false`다.

실행 순서는 fold-major: `historical` seed 20260922/23/24, `p03` 같은 세 seed, `p01` 같은 세 seed. 각 짝의 B와 W2는 모두 완료해 비교하고, 다음 짝은 직전까지의 모든 통과 결과를 확인한 뒤에만 시작한다. 실패 후 새로운 계수·다른 순서·임계값을 탐색하지 않는다. 이전 [D0/D1 사전 계약](../map_road_lane_ab/PLAN.md)의 Town05 도로·차선 절대/상대 목표와 Town01 DEV 보존 기준은 그대로이며, 이 내부 holdout이 그 검증을 대신하지 않는다.

## 실행 경계와 보존

원격 개인 `/home/pro6000/personal/hwanhong`의 기존 `venvs/py312`, 물리 GPU 0 UUID `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`만 사용한다. 개인 venv 밖의 설치, GPU 1, 타인 프로세스 종료/점유, 공유 데이터 변경은 하지 않는다. GPU lease와 0 MiB/0% idle 확인 중 하나라도 실패하면 훈련하지 않는다. 먼저 CPU 사전 검사가 데이터·출처·fold·소스·새 출력 경로를 통과해야 한다. 매 짝은 별도 새 디렉터리에 두 checkpoint, 초기 state hash, 학습 로그, 두 혼동행렬, 판정, 해시를 기록한다. 기존 AI v2, D0/D1, 로컬 데이터와 기존 원격 증거는 덮어쓰지 않는다. 원격과 로컬의 새 결과는 종료 후 해시로 대조한다.

이 실험은 같은 Town01의 세 경로를 검증에 재사용하는 **내부 탐색**이다. 순차 프레임 114/187/196장을 독립 반복으로 세지 않는다. Town02 밖의 새 장소·날씨, 봉인 TEST, 누적 HD 지도, 자율주행 폐루프·실차 결과는 없다. 모든 결과의 `model_promotion`, `test_used`, `deployment_approved`는 `false`다.
