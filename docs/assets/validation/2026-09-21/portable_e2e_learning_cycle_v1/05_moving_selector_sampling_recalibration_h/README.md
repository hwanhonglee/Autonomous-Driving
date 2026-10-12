# 05 — 성능 우선 H selector sampling 재보정

<!-- HH_260906 - Record the failed selector-only recovery without claiming promotion, closed-loop validation, or new functionality. -->

## 결론

H는 **성능을 회복하지 못했다.** 최종 판정은
`FAIL_SELECTOR_SAMPLING_RECOVERY_SIGNAL`이며 H0/H1 모두 미승격이다. H1의 TRAIN 균등 replay는
같은 조건의 H0보다 직진·전체 6개 지표를 모두 조금 개선했지만, 절대 기준에서는 우회전 3개만
통과하고 직진·전체 6개를 모두 실패했다. 따라서 새 기능 head 추가, runtime checkpoint export,
폐루프 주행, 차량 제어 및 배포 승인은 계속 동결한다.

이 단계는 F의 후보 생성 성능은 남아 있는데 selector가 잘못 고르는 현상을 최소 변경으로
분리한 실험이다. F checkpoint SHA-256
`a551570aa6fc1697e9dcfd9fa714b2c1066f95e2194a5fefa9b0d9f3c20baf8c`에서 시작해 image/route/ego
encoder, fusion, trajectory/stop 후보 생성부와 모든 cached candidate를 고정했다. 새 구조나 새
기능 head를 만들지 않고 기존 `candidate_head`와 `stop_candidate_head` 합계 3,084개 parameter만
학습했다.

- H0: F와 같은 우회전 replay identity
- H1: 추가 181개/epoch만 전체 TRAIN 1,475개에 cyclic-uniform으로 분배
- 양쪽 모두 20 epoch, 8,280 optimizer step, 33,120 exposure, batch 4
- 동일한 F head 초기값 SHA-256 `e4322282697bc83d81c67664fb39ffddef0700e5cf559cb1dbbc3c9910ea67ac`
- 동일한 base position order SHA-256 `02247a0986b5df7831abb05c9c9f44cbf60189a8946441785a63215582a95ed8`
- 동일한 AdamW, learning rate `1e-4`, weight decay `1e-4`, gradient clip `5.0`
- loss: `0.1 × hard composite-oracle CE + 0.1 × expected detached composite regret`
- Town03 DEV는 마지막 step 이후 각 arm당 한 번만 평가; 학습·early stopping·선택에 사용하지 않음
- TEST 접근 0, GPU 1 사용 0, optimizer resume 0, 새 기능 head 0

## 고정 DEV 절대 gate

ADE/FDE/속도 MAE는 모두 6.4초 지평이며 낮을수록 좋다. 우회전은 C, 직진과 전체는 D의 고정
수치를 상한으로 사용했다.

| 범위 | 절대 상한 ADE / FDE / 속도 | H0 ADE / FDE / 속도 | H1 ADE / FDE / 속도 | H1 판정 |
| --- | --- | --- | --- | --- |
| Town03 우회전 337 | C: 2.935 / 7.794 m / 1.108 m/s | 2.849 / 7.364 m / 0.968 m/s | **2.861 / 7.369 m / 0.966 m/s** | 3/3 PASS |
| Town03 직진 328 | D: 3.765 / 11.027 m / 1.788 m/s | 7.117 / 16.558 m / 2.694 m/s | **7.082 / 16.448 m / 2.681 m/s** | 0/3 FAIL |
| 전체 VAL 665 | D: 3.563 / 10.354 m / 1.635 m/s | 4.954 / 11.899 m / 1.819 m/s | **4.943 / 11.847 m / 1.812 m/s** | 0/3 FAIL |
| geometry | selected/any/all ≥664/665 | 664/665, 664/665, 664/665 | 664/665, 664/665, 664/665 | 3/3 PASS |

H1은 D보다 직진 ADE `+88.11%`, FDE `+49.15%`, 속도 MAE `+49.96%` 나쁘다. 전체도 D보다
ADE `+38.72%`, FDE `+14.42%`, 속도 MAE `+10.83%` 나쁘다. 우회전 기준을 통과했다는 한
부분만 골라 모델을 채택할 수 없다. 절대 metric gate 결과는 H0와 H1 모두 `3/9`이며, geometry
통과는 이 여섯 성능 실패를 상쇄하지 않는다.

## paired H1 대 H0 해석

H1은 같은 초기 head, base 순서·위치, step 수, optimizer와 objective를 사용한 H0보다 직진과
전체 6개 지표에서 모두 비퇴행했다.

| 범위 | 지표 | H0 | H1 | H1−H0 | paired 판정 |
| --- | --- | ---: | ---: | ---: | --- |
| 직진 | ADE (m) | 7.117216 | 7.081695 | -0.035521 | PASS |
| 직진 | FDE (m) | 16.558229 | 16.447721 | -0.110507 | PASS |
| 직진 | 속도 MAE (m/s) | 2.694012 | 2.680835 | -0.013177 | PASS |
| 전체 | ADE (m) | 4.954436 | 4.943023 | -0.011413 | PASS |
| 전체 | FDE (m) | 11.898687 | 11.846783 | -0.051904 | PASS |
| 전체 | 속도 MAE (m/s) | 1.819314 | 1.812064 | -0.007250 | PASS |

이 `6/6`은 균등 sampling 방향이 H0보다 약간 낫다는 paired 결과일 뿐, 회복 성공이 아니다.
H1의 절대 gate가 `3/9`이므로 `recovery_signal=false`다. 후보 생성부를 고정했기 때문에 oracle
ADE는 F와 동일하게 직진 `0.536596 m`, 전체 `1.024447 m`인데, selected ADE는 각각
`7.081695 m`, `4.943023 m`다. 즉 3,084개 기존 linear selector parameter의 sampling 변경만으로
남은 선택 오류를 해소하지 못했다.

## 다음 성능 회복 순서

현재 결과로 신규 기능 head를 넣지 않는다. 초기 G 우회전-only 수집 제안은
[후속 원인 분석](analysis/README.md)으로 보류했다. 우회전 gate는 이미 통과했고 즉시 병목은
candidate geometry를 보지 못한 selector의 domain shortcut이었다. 따라서 다음 단계는 같은 F
candidate와 H1 순서를 사용하는 I geometry-aware DRIVE-mode residual A/B다. I도 실패하면
우회전 frame을 더 반복하지 않고 TRAIN 역할 Town의 서로 다른 물리 경로에서 신규 15/30 km/h
straight episode를 수집한다. C 우회전과 D 직진·전체 9개 기준을 동시에 회복하기 전에는 기능
확장이나 3-seed 확증으로 넘어가지 않는다.

## 증거와 제한

- [전체 H summary](evidence/summary.json)
- [H0 exact report](metrics/H0_f_right_replay/report.json)
- [H1 exact report](metrics/H1_train_cyclic_uniform_replay/report.json)
- [원격→로컬 exact 전달 검증](evidence/exact_transfer_verification.json)
- [exact artifact summary](../../../../../../storage/experiments/training/2026-09-21/moving_selector_sampling_recalibration_h_v1/summary.json)
- [exact artifact checksum](../../../../../../storage/experiments/training/2026-09-21/moving_selector_sampling_recalibration_h_v1/SHA256SUMS)

이 결과는 CARLA에서 이미 수집한 TRAIN/DEV candidate cache를 사용한 single-seed open-loop
진단이다. 라이브 CARLA 주행, Autoware/RViz 화면, GIF/PNG, closed-loop 제어, 모든 Town,
30/60 km/h, 장애물 회피, ACC, 차선 변경 또는 실제 차량 성능 검증이 아니다. 고정 DEV에는
봉인된 stationary episode가 없으므로 stop guard는
`NOT_EVALUABLE_NO_SEALED_STATIONARY_DEV_EPISODE`다. 따라서 이동 지표를 모두 통과했더라도 이
실험만으로는 승격할 수 없다.

[직전 F 단계](../04_moving_direction_balanced_replay_f_pilot/README.md) ·
[다음 I 단계](../06_moving_drive_mode_residual_recovery_i/README.md) ·
[cycle 목차](../README.md)
