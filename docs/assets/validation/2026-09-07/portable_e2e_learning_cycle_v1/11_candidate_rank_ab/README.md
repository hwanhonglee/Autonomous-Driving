> 공개 metadata의 계정별 경로를 치환했습니다. 독립 재실행에는 원본 SHA로 식별한 private 원본 자료가 필요합니다.

# Portable E2E 후보 경로 인지 점수 모델 · 동일 v3 · 3-seed C/E

상태: `COMPLETE_NOT_PROMOTED`
후보 상대 비교: `FAIL` · 절대 품질: `FAIL`

모델 자동 승격·차량 제어 승인 없음. 이번 캠페인은 validation만 사용하며 독립 test는 열지 않습니다.

| Seed | Arm | ADE m | FDE m | Oracle ADE m | Selection regret m | Speed MAE m/s | Geometry pass | c0–c5 선택수 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 20260903 | C_expanded_data | 3.87433 | 9.82771 | 1.55218 | 2.32215 | 1.45993 | 336/337 | 0, 0, 294, 43, 0, 0 |
| 20260903 | E_candidate_rank | 4.73573 | 9.41304 | 1.63475 | 3.10098 | 1.39987 | 336/337 | 0, 0, 131, 65, 141, 0 |
| 20260904 | C_expanded_data | 6.25855 | 12.89745 | 1.48367 | 4.77487 | 1.99979 | 336/337 | 150, 0, 0, 0, 100, 87 |
| 20260904 | E_candidate_rank | 4.53911 | 9.70931 | 1.52800 | 3.01111 | 1.52334 | 336/337 | 261, 0, 0, 76, 0, 0 |
| 20260905 | C_expanded_data | 4.78955 | 11.86629 | 1.55011 | 3.23944 | 1.61377 | 336/337 | 333, 0, 4, 0, 0, 0 |
| 20260905 | E_candidate_rank | 4.22841 | 9.34865 | 1.62355 | 2.60486 | 1.25132 | 336/337 | 139, 174, 24, 0, 0, 0 |

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

서로 다른 모델 구조의 비교입니다. 모델 ID·parameter 수가 다르며, 같은 seed·step이 같은 초기 가중치나 학습 연산량을 의미하지 않습니다.
