# 2026-09-21 portable E2E learning cycle v1

<!-- HH_260906 - Keep the learning-cycle index chronological and make the latest non-promotion decision explicit. -->

이 폴더는 이동 데이터 추가와 회전 replay, selector 복구 효과를 같은 seed에서 단계적으로
검증한 연구용 기록이다. 일곱 단계 모두 PRO6000 물리 GPU 0의 실제 학습 결과지만, CARLA 데이터 기반
open-loop DEV 진단이며 모델 승격이나 차량 제어 승인 기록이 아니다.

| 단계 | 비교 | 결과 | 판정 |
| --- | --- | --- | --- |
| [01 — 5,740-step A/B](01_moving_data_ab_pilot/README.md) | 기존 TRAIN A vs Town07 직진 증강 B | 직진 개선, 기존 우회전 악화 | `COMPLETE_NOT_PROMOTED` |
| [02 — 7,380-step C/D](02_moving_data_high_budget_cd_pilot/README.md) | 기존 TRAIN C vs Town07 직진 증강 D | 직진 개선 확대, 우회전 퇴행 축소됐으나 잔존 | `FAIL_SINGLE_SEED_DIAGNOSTIC`, 미승격 |
| [03 — 9,020-step E 회전 replay](03_moving_turn_replay_e_pilot/README.md) | C vs 직진 증강 + high-curvature replay E | 직진·전체 평균 개선, 우회전 3개 gate 실패 | `FAIL_SINGLE_SEED_DIAGNOSTIC`, 미승격 |
| [04 — 8,280-step F 방향 균형 replay](04_moving_direction_balanced_replay_f_pilot/README.md) | C 우회전 + D 직진/전체 Pareto 회복 | 우회전 3개 개선, 직진·전체 6개 퇴행 | `FAIL_SINGLE_SEED_DIAGNOSTIC`, 미승격 |
| [05 — 각 arm 8,280-step H selector sampling](05_moving_selector_sampling_recalibration_h/README.md) | F 후보 고정, H0 우회전 replay vs H1 TRAIN 균등 replay | paired 6/6 소폭 개선, 절대 gate 3/9 | `FAIL_SELECTOR_SAMPLING_RECOVERY_SIGNAL`, 미승격 |
| [06 — 각 arm 8,280-step I DRIVE residual](06_moving_drive_mode_residual_recovery_i/README.md) | 동일 용량 I0 문맥-only vs I1 candidate geometry | 직진 개선 신호, 우회전·combined 비퇴행 실패 | `FAIL_DRIVE_MODE_RESIDUAL_RECOVERY_SIGNAL`, 미승격 |
| [07 — J causal route ranker](07_moving_causal_route_ranker_j/README.md) | F 후보 고정, causal route와 candidate geometry의 4-parameter shared cost | 직진 FDE·속도 회복, 우회전 3개와 ADE 2개 실패 | `FAIL_CAUSAL_ROUTE_RANKER_RECOVERY_SIGNAL`, 미승격 |

최신 결론은 07 단계다. J는 target·미래 ego·image·fused context·map/episode ID를 ranker
입력에서 제외하고, raw causal route와 현재 ego 상태를 frozen candidate geometry에 대응시켰다.
4개 shared parameter를 3-fold와 final fit에서 총 `946,176` row exposure로 학습했다. 직진과
combined의 FDE·속도 MAE는 고정 gate를 통과했지만, 우회전 `0/3`, 직진 ADE 실패, combined
ADE 실패로 metric은 `4/9`다. combined ADE는 상한을 `0.000360 m` 넘었지만 사후 완화하지
않는다. geometry `3/3`, 보존 `2/2`와 별개로 J도 채택·export하지 않는다.

다음은 기능 head가 아니라 독립 TRAIN straight route 데이터다. Town03 DEV와
Town04/Town15 TEST를 계속 봉인하고, create-only 15 km/h 계약으로 서로 다른 물리 경로를
수집한다. C-track 후보의 exact corridor 중복은 먼저 QA하며 동일 경로 변형을 독립 group으로
세지 않는다. 현재 약 210 m route는 30 km/h의 unchanged 30초+6.4초 계약에 필요한
`333.333 m`보다 짧으므로 30 km/h는 긴 route 설계가 먼저다. NuRec은 이 selector 실패를
고치는 수단이 아니며 성능 기준선 회복 뒤 별도 CARLA 0.9.16+ real-to-sim 파일럿으로 다룬다.

[현재 상태 요약](CURRENT.md) ·
[최신 J exact artifact](../../../../../storage/experiments/training/2026-09-21/moving_causal_route_ranker_j_v1/summary.json) ·
[NuRec real-to-sim 검토](../../../../nurec-real2sim-feasibility-ko.md)
