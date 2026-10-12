# TRAIN 경로 holdout 차선 2× 손실 파일럿 — 첫 짝 중단

<!-- HH_260929 - Report the frozen first-pair gate and its ceiling limitation without treating TRAIN holdout as external generalization. -->

**결과: 첫 짝 `PAIR_FAIL_STOP`, 나머지 8짝 미실행, 모델 미채택.** [실행 전 계약](PLAN.md)에 따라 Town01 기존 직진 경로 114장을 통째로 holdout으로 두고, 나머지 Town01 383장과 Town02 p02 232장만으로 기준 `B`와 차선 손실 가중치 2× `W2`를 각각 scratch 2,000 step 학습했다. 두 팔의 초기 모델 state SHA-256은 동일한 `e78098947d72df35bcee1eb8719ad0e9b0acd67ec6a1e393197d477823da2058`이다. seed는 `20260922`이며 같은 균등 샘플링·TRAIN crop RNG와 모델/optimizer 설정을 사용했다. DEV/TEST 배열이나 이전 Town05 예측은 열지 않았다.

| holdout 114장 전체 픽셀 IoU | B | W2 | W2 − B |
| --- | ---: | ---: | ---: |
| 차선 표시 | 95.258244% | 95.377765% | **+0.119520pp** |
| 도로 | 91.546782% | 91.513453% | −0.033329pp |

사전 조건은 매 짝 **차선 ≥+5pp, 도로 ≥−1pp**였다. 도로 보존은 통과했지만 차선 개선은 미달했다. [원격 보관 summary](remote_run_v1/pair_00_historical_20260922/summary.json)의 `next_pair_authorized=false`이고 이후 짝 디렉터리는 없다. 기준 B의 차선 IoU가 이미 **95.258244%**였으므로 이 짝의 가능한 최대 개선폭은 **4.741756pp**다. 즉 +5pp 조건은 W2 결과와 관계없이 이 경로·seed에서 수학적으로 통과 불가능했다. 사전 조건을 결과를 보고 고치거나 계속 학습하지 않았다. 관측된 W2 이득은 작지만, 이 중단 판정을 Town05의 차선 실패 원인이나 다른 Town에서의 W2 효과로 해석할 수 없다.

전체 7클래스 IoU와 7×7 혼동행렬은 [B metrics](remote_run_v1/pair_00_historical_20260922/B_metrics.json), [W2 metrics](remote_run_v1/pair_00_historical_20260922/W2_metrics.json)에 있다. 참고로 7클래스 IoU는 다음과 같다. 소수 클래스의 단일 경로 변화는 독립 일반화 신호로 취급하지 않는다.

| 클래스 | B | W2 |
| --- | ---: | ---: |
| other | 87.5095% | 88.1054% |
| road | 91.5468% | 91.5135% |
| sidewalk | 86.2207% | 88.2558% |
| lane_marking | 95.2582% | 95.3778% |
| traffic_light_fixture | 15.7177% | 13.1996% |
| traffic_sign | 11.6250% | 26.6062% |
| pole | 22.0131% | 26.1676% |

[첫 heldout 프레임](visuals/heldout_000_rgb_oracle_b_w2.png)과 [마지막 프레임](visuals/heldout_113_rgb_oracle_b_w2.png)은 성능을 보고 고르지 않았다. 각 PNG는 원본 640×360 전체 화각 RGB, CARLA oracle, B, W2를 나란히 둔다. 도로는 파랑, 차선은 노랑이고 다른 명명 클래스도 색으로 표시했다. 그림은 내부 **TRAIN 경로 holdout**의 정지 영상이며 Autoware 화면, 완성된 HD 지도, 주행 시험이 아니다.

## 재현·보존

- 원격 개인 `venvs/py312`, 물리 GPU 0 UUID `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`에서만 실행했다. 시작 전과 종료 후 GPU0은 **0 MiB/0%**, 공유 개인 GPU0 lease는 획득·해제 확인했다. GPU1·설치·다른 사용자 작업은 건드리지 않았다. 두 팔의 GPU 학습 구간은 각각 **75.56초, 74.79초**이며 사전 검사·평가·전송 시간은 별도다.
- [CPU 사전 검사 영수증](cpu_preflight_pair00.json)은 출력 디렉터리 생성 전에 PASS했다. 615 TRAIN/114 holdout, 클래스 가중치 원천 383장, 원본 해시·index 목록·DEV/TEST 미사용 상태를 기록한다. [로컬 8개 검사와 원격 소스 해시 로그](local_validation.txt)도 보존했다.
- 첫 실패 후 원격 `--pair-index 1 --validate-only`를 읽기 전용으로 호출하자 exit 1과 `ValueError: prior pair failed or source contract changed`가 발생했다. 즉 실행기는 다음 짝의 CPU 단계에서도 중단을 강제한다.
- [원격 실행 전체 복사본](remote_run_v1/)에는 두 checkpoint, 2,000-step 학습 로그, holdout 예측 NPZ, 전체 metrics/혼동행렬, 시작·종료 JSON과 소스 스냅샷이 있다. 새 로컬 복사본의 **17개 파일 SHA-256이 원격과 모두 정확히 일치**했고 [SHA256SUMS](SHA256SUMS) 검사도 17/17 PASS했다. 기존 D0/D1 실행과 데이터 패키지는 변경하지 않았다.
- [독립 픽셀 재계산기](verify_pair.py)는 보관된 두 예측과 **TRAIN `train_d0.npz`의 첫 114장**만 읽어 두 혼동행렬·7클래스 IoU·판정·초기 state/checkpoint/예측 해시를 재검사해 `INDEPENDENT_TRAIN_HOLDOUT_PIXEL_REPLAY_PASS`를 냈다. 이미지 생성 [스크립트](render_pair.py)는 이 재검사를 먼저 통과해야 PNG를 만든다. PNG SHA-256은 `c9a38c380aa907336e59c7567db367c88790ceb02646d95b53cccb3a28656670`(000), `db22942588c55c9561e23e8eddf02266a39fbcd941e91fceafecfdcc9f7405a5`(113)다.

같은 Town의 경로 holdout은 프레임 랜덤 분할보다 위치 중복을 줄이지만, 경로·렌더 조건이 제한되고 시간상 인접 프레임은 상관되어 있다. 이 한 짝은 원래 D1 전체 729장 훈련이나 Town 밖의 새 DEV를 대체하지 않는다. 본 후보의 추가 8짝, full-D1 비교, 독립 DEV/TEST, 누적 지도, 폐루프 주행은 실행하지 않았다. 새로운 후속 실험이 필요하면 이 실패/ceiling 한계를 먼저 반영한 별도 사전 계약과 사용하지 않은 검증 장소가 필요하다.
