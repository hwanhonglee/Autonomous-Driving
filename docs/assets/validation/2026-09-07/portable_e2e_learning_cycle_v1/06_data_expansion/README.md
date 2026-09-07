> 공개 metadata의 계정별 경로를 치환했습니다. 독립 재실행에는 원본 SHA로 식별한 private 원본 자료가 필요합니다.

# Portable E2E 추가 우회전 데이터 학습 · 3개 seed

상태: `COMPLETE_NOT_PROMOTED` · 절대 품질: `FAIL`

Train 1147 / Town03 val337, 1540 optimizer step: 약 5.37 epoch입니다. 모델 자동 승격·차량 제어 승인은 없습니다.
Town04 test는 이번 캠페인에서 평가하지 않습니다. 기존 v2와 v3 간 자동 PASS 판정이나 공정 비교 승인은 하지 않습니다.

| Seed | ADE m | FDE m | Oracle ADE m | Selection regret m | Speed MAE m/s | Geometry pass | c0–c5 선택수 | 절대 품질 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 20260903 | 3.87433 | 9.82771 | 1.55218 | 2.32215 | 1.45993 | 336/337 | 0, 0, 294, 43, 0, 0 | FAIL |
| 20260904 | 6.25855 | 12.89745 | 1.48367 | 4.77487 | 1.99979 | 336/337 | 150, 0, 0, 0, 100, 87 | FAIL |
| 20260905 | 4.78955 | 11.86629 | 1.55011 | 3.23944 | 1.61377 | 336/337 | 333, 0, 4, 0, 0, 0 | FAIL |

Seed 20260903 절대 기준:

- ade_1p0s_m: PASS
- ade_3p0s_m: FAIL
- ade_6p4s_m: FAIL
- fde_6p4s_m: FAIL

Seed 20260904 절대 기준:

- ade_1p0s_m: PASS
- ade_3p0s_m: FAIL
- ade_6p4s_m: FAIL
- fde_6p4s_m: FAIL

Seed 20260905 절대 기준:

- ade_1p0s_m: PASS
- ade_3p0s_m: FAIL
- ade_6p4s_m: FAIL
- fde_6p4s_m: FAIL

절대 기준: ADE 1/3/6.4초 ≤ 0.5/1/2 m, FDE 6.4초 ≤ 4 m. Selection regret는 경로 다양성 지표가 아닙니다.
Checkpoint byte 검증 여부는 JSON에 명시합니다. 미다운로드 checkpoint는 평가·감사 간 hash 일치까지만 확인합니다.
