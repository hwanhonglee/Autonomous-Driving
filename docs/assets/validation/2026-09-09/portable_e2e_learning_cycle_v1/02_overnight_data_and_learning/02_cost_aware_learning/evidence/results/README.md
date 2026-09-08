# Portable E2E Composite-regret 보조 손실 · 새 학습 3-seed A/B

상태: `COMPLETE_NOT_PROMOTED`
후보 상대 비교: `FAIL` · 절대 품질: `FAIL`

모델 자동 승격·차량 제어 승인 없음. Train1147 학습·val337 평가만 사용합니다. 전체 corpus 무결성 검사에서 test 파일을 읽을 수 있지만 test 추론·최적화·모델 선택은 금지됩니다.

| Seed | Arm | ADE m | FDE m | Oracle ADE m | Selection regret m | Speed MAE m/s | Geometry pass | c0–c5 선택수 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 20260903 | A_original_loss | 3.87433 | 9.82771 | 1.55218 | 2.32215 | 1.45993 | 336/337 | 0, 0, 294, 43, 0, 0 |
| 20260903 | B_cost_aware_selector | 4.21325 | 9.75842 | 1.51251 | 2.70075 | 1.43267 | 336/337 | 0, 30, 17, 51, 239, 0 |
| 20260904 | A_original_loss | 6.25855 | 12.89745 | 1.48367 | 4.77487 | 1.99979 | 336/337 | 150, 0, 0, 0, 100, 87 |
| 20260904 | B_cost_aware_selector | 5.42182 | 12.48005 | 1.61793 | 3.80389 | 2.05640 | 336/337 | 12, 195, 0, 0, 55, 75 |
| 20260905 | A_original_loss | 4.78955 | 11.86629 | 1.55011 | 3.23944 | 1.61377 | 336/337 | 333, 0, 4, 0, 0, 0 |
| 20260905 | B_cost_aware_selector | 3.45695 | 7.58843 | 1.71904 | 1.73791 | 0.96950 | 336/337 | 209, 68, 50, 0, 0, 10 |

Seed 20260903: 상대 비교 `FAIL`, 절대 품질 `FAIL`.

- selected_ade_improves: FAIL
- selected_fde_improves: PASS
- geometry_does_not_regress: PASS
- speed_mae_within_5_percent: PASS
- ade_1p0s_m: PASS
- ade_3p0s_m: FAIL
- ade_6p4s_m: FAIL
- fde_6p4s_m: FAIL

Seed 20260904: 상대 비교 `PASS`, 절대 품질 `FAIL`.

- selected_ade_improves: PASS
- selected_fde_improves: PASS
- geometry_does_not_regress: PASS
- speed_mae_within_5_percent: PASS
- ade_1p0s_m: PASS
- ade_3p0s_m: FAIL
- ade_6p4s_m: FAIL
- fde_6p4s_m: FAIL

Seed 20260905: 상대 비교 `PASS`, 절대 품질 `FAIL`.

- selected_ade_improves: PASS
- selected_fde_improves: PASS
- geometry_does_not_regress: PASS
- speed_mae_within_5_percent: PASS
- ade_1p0s_m: PASS
- ade_3p0s_m: FAIL
- ade_6p4s_m: FAIL
- fde_6p4s_m: FAIL

Selection regret는 동일 분모의 selected ADE − oracle ADE입니다. 후보의 기하학적 다양성 판정은 아닙니다.
절대 기준: ADE 1/3/6.4초 ≤ 0.5/1/2 m, FDE 6.4초 ≤ 4 m.
Checkpoint가 내려받아지지 않은 경우 평가·감사 간 hash 일치만 확인하며 로컬 byte 검증 여부는 JSON에 기록합니다.

A λ=0 / B λ=0.1, 각각 새 학습 1540 step·6155 sample 노출입니다. 보조 손실은 composite-cost regret이며 ADE regret과 다릅니다. B total loss에만 새 항이 있으므로 A/B total loss 감소를 같은 목적함수의 개선으로 해석하면 안 됩니다. 기존 기준으로 세 seed를 모두 비교하고 자동 승격·새 데이터 승인·자율주행 완료를 주장하지 않습니다.
