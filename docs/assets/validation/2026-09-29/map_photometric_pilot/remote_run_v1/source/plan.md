# D1 사진측광 단일 후보: 실행 전 계약

<!-- HH_260929 - Freeze one image-only policy before the candidate training or any candidate score. -->

**상태: 후보 학습 전 사전 등록.** 이것은 어둡고 무채색인 반복 관찰 Town05_Opt DEV의 [외관 진단](../map_road_lane_ab/APPEARANCE_SHIFT.md)에서 동기를 얻은 탐색이다. 따라서 그 DEV에서 점수가 좋아져도 새로운 독립 검증, 인과 원인 규명, 지도 모델 승격을 뜻하지 않는다. 정책의 흑백 혼합과 밝기 변경은 함께 켜지는 **한 묶음**이며 각 성분의 효과는 분리하지 않는다. 점수를 보고 범위·확률·seed를 바꾸거나 추가 후보를 실행하지 않는다.

## 고정 입력과 유일한 변경

- 기준 팔은 [기존 D1 실행](../../../../../storage/experiments/training/2026-09-29/map_road_lane_ab_run_v1/summary.json) 그대로다. 기준 훈련 소스 SHA256 `ac3c71c071cc9db9f6b36fe0948ece9d034313c00853f986f843bb13a40e3c12`, D1 checkpoint `c583be21a082fbfaa0a9ed1346dc667f032ef928582bd8c537b949039404c38f`, `started.json` `05b29737a73c258444931b9dfd7a6ff7ebd462f7c48dee3486b0f87c2f90eb20`, D1 `metrics.json` `32678cd5225532b1700bb90ddc385d44a516db3fd6ad936e798539231cb48624`를 확인한다. 기존 팔은 다시 훈련하지 않는다.
- 고정 데이터 [manifest](../../../../../storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2/manifest.json) SHA256 `313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6`, D1 TRAIN 729장 NPZ SHA256 `4e2f7df61814a7e8aade88877482506a30e41c9fff0b5e6b7c3182193ff7d8f8`. 클래스 가중치는 원래대로 Town01 D0 TRAIN 497장의 `sqrt_frequency`이며 바꾸지 않는다.
- 새 후보는 동일 `MapPerception` 7클래스, 원본 640×360, scratch seed `20260922`, AdamW `3e-4`/weight decay `1e-4`, batch 8, 2,000 step, 동일 프레임 샘플러 seed `20260922`, 동일 `paired_crop` RNG seed `20260926`과 gradient clipping 5.0을 쓴다. label, sampler, crop, loss, architecture, model 초기화는 변경하지 않는다.
- **유일한 변경:** `paired_crop` 직후 각 TRAIN RGB crop마다 별도 NumPy RNG seed `20260929`에서 확률 `0.5`로 사진측광 정책을 적용한다. 켜지면 `lambda ~ Uniform[0,1)`, `scale ~ Uniform[0.35,1.0)`을 순서대로 뽑고, BT.601 회색값 `Y=0.299R+0.587G+0.114B`, `RGB' = round_to_nearest_even(clip(scale * ((1-lambda)*RGB + lambda*Y), 0, 255))`로 `uint8` 변환한다. 꺼지면 원 RGB crop의 바이트가 그대로다. target crop은 어떠한 변환도 하지 않는다. 평가 입력은 변형하지 않는다.

## 순서와 판정

1. 로컬 테스트와 원격 개인 venv CPU preflight에서 위 해시, 훈련 배열·가중치, 기준 실행 계약, 새 출력 경로를 확인한다. 이 시점에 DEV/TEST 배열은 **열지 않는다**. `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5` 물리 GPU 0만 지정하고 lease 및 0 MiB/0% idle을 확인한다. 점유 중이면 학습을 시작하지 않는다. 개인 기존 `venvs/py312`만 쓰고 설치, GPU 1, 타인 프로세스 조작은 금지한다.
2. 새 후보 하나만 학습·checkpoint 봉인한 뒤 Town01 DEV 118장과 반복 관찰 Town05_Opt DEV 232장을 평가한다. TEST는 열거나 실행하지 않는다. 두 DEV 모두 7×7 혼동행렬과 7클래스 IoU, 기준 대비 차이를 기록한다. 비교 기준은 고정 D1 팔이며 D0/AI v2와 바꿔치기하지 않는다.
3. **탐색 진단 게이트:** Town05 차선 IoU 후보−D1 `≥+0.05`, Town05 도로 IoU 후보−D1 `≥−0.01`, Town01 도로·차선 IoU 후보−D1 각각 `≥−0.01`. 기준 D1의 Town05 lane 0.07325/road 0.52302와 Town01 lane 0.89364/road 0.98383에서 이 수학적 하한은 달성 가능하다. 다른 5개 클래스의 증감도 공개한다. 게이트는 반복 DEV에 맞춘 탐색 표시일 뿐 승인 관문이 아니다.

기존 [D0/D1의 절대 목표와 7클래스 비퇴행 조건](../map_road_lane_ab/PLAN.md)은 완화하지 않는다. 위 게이트가 통과해도 한 seed와 이미 반복 관찰한 DEV만 있으므로 `model_promotion=false`, 독립 검증 수 `0`, 정밀지도·온디바이스 배포 승인 `false`다. 기존 체크포인트·데이터·결과는 보존하고 새 원격/로컬 디렉터리와 SHA256 증거만 추가한다.
