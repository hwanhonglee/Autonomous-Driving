# D1 사진측광 단일 후보 — 반복 DEV 탐색 실패

<!-- HH_260929 - Report one frozen image-only candidate without promotion or post-hoc retuning. -->

**결론: 사전 탐색 게이트 FAIL, 모델 미채택.** [실행 전 계약](PLAN.md) SHA-256 `79c14b50b268fa3a51f023baf34f602faa95cd687b7d90158911ca1372609579`에 따라 전체 D1 TRAIN 729장에 사진측광 정책 하나만 더한 새 모델을 scratch seed `20260922`로 2,000 step 학습했다. 기준은 해시로 확인한 기존 D1 팔이며 재학습하지 않았다. 반복 관찰 Town05_Opt DEV의 차선은 **+6.158pp**, 도로는 **+0.638pp**였지만 Town01 DEV 차선은 **−5.485pp**로 사전 보존 하한 `−1pp`를 넘어서 악화됐다. 결과를 보고 강도·확률·게이트를 조정하거나 다른 후보를 실행하지 않았다.

| 클래스 IoU | Town01 반복 DEV: D1 → 후보 (Δpp) | Town05_Opt 반복 DEV: D1 → 후보 (Δpp) |
| --- | ---: | ---: |
| other | 95.527% → 95.968% (+0.442) | 52.227% → 54.332% (+2.105) |
| road | 98.383% → 98.463% (+0.079) | 52.302% → 52.940% (+0.638) |
| sidewalk | 91.573% → 91.559% (−0.015) | 1.857% → 2.294% (+0.437) |
| lane_marking | **89.364% → 83.880% (−5.485)** | **7.325% → 13.483% (+6.158)** |
| traffic_light_fixture | 10.791% → 9.015% (−1.776) | 5.717% → 4.363% (−1.354) |
| traffic_sign | 3.976% → 2.337% (−1.639) | 0% → 0.005% (+0.005) |
| pole | 29.011% → 30.437% (+1.426) | 10.123% → 8.297% (−1.826) |

사전 탐색 조건은 Town05 차선 `≥+5pp`, 도로 `≥−1pp` 및 Town01 도로·차선 각각 `≥−1pp`였다. 앞의 세 조건만 통과했고 Town01 차선이 실패했다. Town05의 **원래 절대 목표인 도로 ≥80%, 차선 ≥50%도 여전히 크게 미달**한다. Town01의 차선은 원래 보존 절대 하한 `89.233%`도 미달하며, 다른 클래스 일부도 퇴행했다. 따라서 기존 D0/D1의 승인 기준을 완화하지 않고 `model_promotion=false`다.

사진측광 정책은 TRAIN의 `paired_crop` 직후 별도 seed `20260929`로 샘플별 확률 0.5에 적용했다. 켜지면 BT.601 흑백 혼합 `λ∼U[0,1)`과 밝기 배율 `U[0.35,1.0)`을 **같은 정책 안에서 함께** 적용했다. 실제 적용은 8,052/16,000 TRAIN crop이었다. `MapPerception` 7클래스, D1 프레임 구성, D0 출처 `sqrt_frequency` 가중치, sampler/crop RNG, 라벨, 손실, optimizer, seed와 step은 그대로다. 초기 state SHA-256 `e78098947d72df35bcee1eb8719ad0e9b0acd67ec6a1e393197d477823da2058`을 확인했다. 이 실험으로 흑백화와 어둡게 하기 각각의 효과나 Town 간 퇴행의 원인을 분리할 수 없다.

이 정책 범위 자체가 이미 여러 차례 본 [Town05 외관 진단](../map_road_lane_ab/APPEARANCE_SHIFT.md)에서 나왔다. 또한 Town01과 Town05 지표 모두 반복 관찰한 DEV다. 따라서 Town05 상대 이득은 독립 일반화가 아니라 **탐색 신호**이며, 원본 수집기의 맑은 낮/해상도/FOV 유사성만으로 노출·재질·표지 구성 차이를 확정할 수 없다. [저장 예측의 별도 공간 진단](SPATIAL_DIAGNOSTIC.md)은 Town05 차량 전방 중앙의 차선 TP 증가와 Town01 상단 먼 영역의 차선 오탐 증가를 보였다. 이 분석은 사후적이며 새로운 판정 게이트가 아니다.

[Town01 DEV 고정 첫 프레임](visuals/town01_dev_000_rgb_oracle_d1_photo.png)과 [Town05 DEV 기존 대표 115번](visuals/town05_dev_115_rgb_oracle_d1_photo.png)은 원본 전체 640×360 RGB, CARLA oracle, 기준 D1, 후보를 나란히 보여 준다. 도로는 파랑, 차선은 노랑이다. Town05 PNG는 기존 보고서에 사용한 프레임으로 새로 좋은 장면을 골라낸 것이 아니다. 두 그림은 반복 DEV의 정지 영상이지 Autoware 화면·정밀지도·주행 성능 증거가 아니다.

[Town05 반복 DEV 3프레임 GIF](visuals/town05_dev_000_115_231_four_panel_near_ego.gif)는 **고정 원본 순서 000/115/231**만 각 1.8초씩 보여 준다. 매 프레임 위쪽에 네 개의 전체 640×360 화각(RGB·CARLA oracle·고정 D1·후보), 아래쪽에 동일 좌표 `x=160..479, y=180..359`의 차량 전방 중앙 확대를 둔다. 프레임에는 `EXPLORATORY | NOT AUTOWARE | NOT HD MAP`을 표시했다. SHA-256 `cc7dae21d79241444e84a1bf6221ab94b6c1672f60b3a3d3515558c7ed642fe5`이며 [시각화 해시 목록](VISUAL_SHA256SUMS)에 기존 PNG와 함께 고정했다. 이는 **띄엄띄엄 고른 보존 스냅샷**이지 연속 주행 영상이 아니다. 확대 영역은 원본 픽셀의 표시일 뿐 별도 검증 표본이 아니다. GIF는 표시를 위해 256색으로 양자화했으며 지표는 GIF가 아닌 원본 라벨·예측 배열에서 계산했다.

## 재현·자원·보존

- [CPU 사전 검사 영수증](cpu_preflight_receipt.json)이 D0/D1 TRAIN 배열과 고정 baseline 소스·checkpoint·metrics·summary 해시를 확인했다. **후보 checkpoint가 먼저 봉인된 후에만** runner가 DEV 배열을 열었다. TEST 배열은 어느 단계에서도 열지 않았다. [실행 소스](../../../../../tools/e2e/train_map_photometric_pilot.py) SHA-256 `4de567323d3964ec877da831ec2acb08e445fe77c9da659f052edcf52ca53484`와 [단위 테스트](../../../../../learning/tests/test_train_map_photometric_pilot.py)로 정책·RNG 격리·판정·사전 검사 순서를 고정했다.
- 기존 개인 `venvs/py312`와 물리 GPU 0 UUID `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`만 사용했다. 시작 전 GPU0 0 MiB/0%, lease 가능을 확인했고 실행 후에도 GPU0 **0 MiB/0% 및 lease 해제**를 확인했다. 학습 구간은 84.96초다. 설치, GPU 1, 타인 프로세스 조작은 없었다. 원격 D0/D1 TRAIN·manifest 해시도 종료 후 동일했다.
- [원격 실행 전체 복사본](remote_run_v1/)의 checkpoint `a849f51d5b11b223a2fe291ddc37ecac7804293fadac87754d540c2548219477`, 20행의 2,000-step 로그, 양쪽 DEV 예측·7×7 혼동행렬·시작/종료 기록과 소스 스냅샷을 보존했다. 새 로컬 복사본의 **15개 파일 SHA-256이 원격과 전부 일치**했고 [SHA256SUMS](SHA256SUMS) 15/15 PASS했다. 기존 D1 실행과 데이터 패키지를 덮어쓰지 않았다.
- [독립 재계산기](verify_result.py)와 [시험](../../../../../learning/tests/test_verify_map_photometric_pilot.py)은 보관한 D1/후보 예측과 고정 DEV 라벨을 읽어 Town01/Town05 **네 7×7 혼동행렬과 모든 7클래스 IoU**를 픽셀에서 다시 계산했다. 사전 판정과 run/source/checkpoint 해시도 일치해 `INDEPENDENT_D1_AND_PHOTO_DEV_PIXEL_REPLAY_PASS`; 신규 시험 7개 PASS다. GIF의 3프레임·재생 간격·서로 다른 장면·전체 화각과 동일한 확대 픽셀도 시험했다. 이는 산술·보존 확인이지 독립 검증을 만들어 내는 절차는 아니다. 그림 [생성기](render_result.py)는 이 재검사를 먼저 수행한다.

차기 작업은 이 반복 DEV에 맞춰 사진측광 확률·밝기 범위를 즉시 재탐색하는 것이 아니다. 실제 어둡고 무채색인 **허용된 새 TRAIN 현장**과, 모델 결과를 보기 전에 분할·원본·기준을 잠근 미사용 DEV가 필요하다. 이 한 seed는 교차-Town 원인, 온디바이스 지연, 정밀지도 누적 또는 폐루프 주행을 검증하지 않았다.

```bash
cd docs/assets/validation/2026-09-29/map_photometric_pilot
sha256sum -c SHA256SUMS
sha256sum -c VISUAL_SHA256SUMS
python3 verify_result.py --data ../../../../../storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2 \
  --baseline ../../../../../storage/experiments/training/2026-09-29/map_road_lane_ab_run_v1 \
  --run remote_run_v1 --sums SHA256SUMS
# HH_260929 - Use a fresh path; the evidence GIF is never overwritten.
python3 render_result.py --gif-only --gif-output /tmp/map_photo_town05_replay.gif
```
