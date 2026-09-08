# Portable E2E 제공 가속도 입력 유지/제거 · 새 학습 3-seed A/B

상태: `COMPLETE_NOT_PROMOTED`
후보 상대 비교: `FAIL` · 절대 품질: `FAIL`

모델 자동 승격·차량 제어 승인 없음. 학습은 train1147, 평가는 val337입니다. 전체 corpus 무결성 검사에서 test 파일을 읽을 수 있으나 test 추론·학습·모델 선택은 하지 않습니다.

| Seed | Arm | ADE m | FDE m | Oracle ADE m | Selection regret m | Speed MAE m/s | Geometry pass | c0–c5 선택수 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 20260903 | A_physical_input | 3.87433 | 9.82771 | 1.55218 | 2.32215 | 1.45993 | 336/337 | 0, 0, 294, 43, 0, 0 |
| 20260903 | B_no_accel_input | 5.58615 | 12.53533 | 1.56252 | 4.02363 | 2.16971 | 336/337 | 0, 0, 0, 108, 229, 0 |
| 20260904 | A_physical_input | 6.25855 | 12.89745 | 1.48367 | 4.77487 | 1.99979 | 336/337 | 150, 0, 0, 0, 100, 87 |
| 20260904 | B_no_accel_input | 4.62077 | 11.11497 | 1.44866 | 3.17211 | 1.89877 | 336/337 | 0, 253, 0, 0, 0, 84 |
| 20260905 | A_physical_input | 4.78955 | 11.86629 | 1.55011 | 3.23944 | 1.61377 | 336/337 | 333, 0, 4, 0, 0, 0 |
| 20260905 | B_no_accel_input | 6.24394 | 13.17265 | 1.56619 | 4.67775 | 2.07880 | 336/337 | 0, 244, 93, 0, 0, 0 |

Seed 20260903: 상대 비교 `FAIL`, 절대 품질 `FAIL`.

- selected_ade_improves: FAIL
- selected_fde_improves: FAIL
- geometry_does_not_regress: PASS
- speed_mae_within_5_percent: FAIL
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
- speed_mae_within_5_percent: FAIL
- ade_1p0s_m: PASS
- ade_3p0s_m: FAIL
- ade_6p4s_m: FAIL
- fde_6p4s_m: FAIL

Selection regret는 동일 분모의 selected ADE − oracle ADE입니다. 후보의 기하학적 다양성 판정은 아닙니다.
절대 기준: ADE 1/3/6.4초 ≤ 0.5/1/2 m, FDE 6.4초 ≤ 4 m.
Checkpoint가 내려받아지지 않은 경우 평가·감사 간 hash 일치만 확인하며 로컬 byte 검증 여부는 JSON에 기록합니다.

ID 외 설정 일치와 실제 보고된 파라미터 수의 검증 결과는 JSON source_proof에 기록합니다. 실제 초기 텐서 해시는 저장되지 않았습니다. 기존 v3 개발용 데이터에서 같은 1540 step(약 5.37 epoch)으로 새로 학습하는 탐색적 비교이며, 새 데이터 승인이나 실차 준비 완료를 뜻하지 않습니다.
