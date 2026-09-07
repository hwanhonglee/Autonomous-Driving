> 공개 metadata의 계정별 경로를 치환했습니다. 독립 재실행에는 원본 SHA로 식별한 private 원본 자료가 필요합니다.

# Portable E2E 학습·검증 3-seed A/B

상태: `COMPLETE_NOT_PROMOTED`
후보 상대 비교: `FAIL` · 절대 품질: `FAIL`

모델 자동 승격·차량 제어 승인 없음. 이번 캠페인은 validation만 사용하며 독립 test는 열지 않습니다.

| Seed | Arm | ADE m | FDE m | Oracle ADE m | Selection regret m | Speed MAE m/s | Geometry pass | c0–c5 선택수 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 20260903 | A_baseline | 4.26151 | 10.91724 | 1.76066 | 2.50085 | 1.39329 | 336/337 | 0, 0, 337, 0, 0, 0 |
| 20260903 | B_lower_lr | 4.15092 | 10.72309 | 1.63546 | 2.51546 | 1.39097 | 336/337 | 0, 0, 0, 0, 337, 0 |
| 20260904 | A_baseline | 4.99626 | 12.29488 | 1.75170 | 3.24456 | 1.61823 | 336/337 | 58, 0, 0, 279, 0, 0 |
| 20260904 | B_lower_lr | 4.20777 | 10.87828 | 1.67930 | 2.52847 | 1.38473 | 336/337 | 16, 321, 0, 0, 0, 0 |
| 20260905 | A_baseline | 4.25799 | 11.09341 | 1.63040 | 2.62759 | 1.42067 | 336/337 | 0, 337, 0, 0, 0, 0 |
| 20260905 | B_lower_lr | 4.59215 | 11.56600 | 1.65975 | 2.93240 | 1.48970 | 336/337 | 337, 0, 0, 0, 0, 0 |

Seed 20260903: 상대 비교 `PASS`, 절대 품질 `FAIL`.

- selected_ade_improves: PASS
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

Seed 20260905: 상대 비교 `FAIL`, 절대 품질 `FAIL`.

- selected_ade_improves: FAIL
- selected_fde_improves: FAIL
- geometry_does_not_regress: PASS
- speed_mae_within_5_percent: PASS
- ade_1p0s_m: PASS
- ade_3p0s_m: FAIL
- ade_6p4s_m: FAIL
- fde_6p4s_m: FAIL

Selection regret는 동일 분모의 selected ADE − oracle ADE입니다. 후보의 기하학적 다양성 판정은 아닙니다.
절대 기준: ADE 1/3/6.4초 ≤ 0.5/1/2 m, FDE 6.4초 ≤ 4 m.
Checkpoint가 내려받아지지 않은 경우 평가·감사 간 hash 일치만 확인하며 로컬 byte 검증 여부는 JSON에 기록합니다.
