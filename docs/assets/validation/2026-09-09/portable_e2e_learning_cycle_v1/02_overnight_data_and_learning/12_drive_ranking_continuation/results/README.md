# 주행 후보 순위 이어학습 — 전체 저장 결과

<!-- HH_260906 - Separate the September 9 execution from September 14 evidence publication and retain every regression. -->

실제 실행일은 **2026-09-09 KST**, 중단 이후 로컬 검증·게시일은 **2026-09-14**입니다. 실행 UTC: `2026-09-08T21:36:00.424783+00:00` → `2026-09-08T21:41:07.668975+00:00`. 새 학습을 실행한 결과가 아닙니다.

동일한 잔차 선택기 부모에서 A(기존 CE)와 B(CE + DRIVE 순위 손실)를 비교한 6회 선택기 이어학습입니다. 각각 1,540 step·6,155회 노출, 합계 9,240 step·36,930회 노출입니다. 최적화기는 새로 시작했습니다. 생성기 forward 0회이며 전체 모델 새 학습으로 합산하지 않습니다. **모델 미채택·데이터 미승인·학습 차량 제어 0회**를 유지합니다.

| seed | endpoint | VAL ADE m | VAL FDE m | speed MAE m/s | TRAIN hold STOP | VAL reacceleration / 337 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 20260903 | parent_candidate_residual | 5.267210 | 11.296644 | 1.705659 | 118/118 | 6 |
| 20260903 | ce_continue | 4.862281 | 10.712870 | 1.608461 | 118/118 | 15 |
| 20260903 | drive_pair_continue | 4.884033 | 10.708413 | 1.611411 | 118/118 | 11 |
| 20260904 | parent_candidate_residual | 5.449730 | 11.719451 | 1.879367 | 118/118 | 14 |
| 20260904 | ce_continue | 4.326678 | 10.248975 | 1.624205 | 118/118 | 97 |
| 20260904 | drive_pair_continue | 4.192644 | 9.908399 | 1.571568 | 118/118 | 90 |
| 20260905 | parent_candidate_residual | 4.535711 | 9.443228 | 1.328209 | 118/118 | 10 |
| 20260905 | ce_continue | 4.481143 | 9.408322 | 1.390810 | 118/118 | 7 |
| 20260905 | drive_pair_continue | 3.783201 | 8.222826 | 1.249053 | 118/118 | 66 |

B의 seed03 VAL ADE는 A보다 조금 악화됐습니다. seed05의 저장된 재가속 진단은 부모 10 → A 7 → B 66입니다. 이는 미래 예측 속도가 정확히 0이 된 뒤 다시 증가하는 저장 행의 수이며 새로운 통과 기준이나 실제 차량 사건 수가 아닙니다. **VAL stationary_hold는 0개**이므로 TRAIN 정지 유지 118/118을 독립 정지 성능으로 해석하지 않습니다. 모든 TRAIN/VAL 그룹·회귀·분모·후보 비용 진단은 전체 요약에 보존했습니다.

[전체 독립 요약](independent_summary.json) · [원본 185파일 inventory와 receipt 근거](input_inventory.json) · [원본/공개 SHA provenance](publication_manifest.json) · [공개 체크섬](SHA256SUMS)

단일 VAL337은 반복 사용한 개발 비교입니다. TEST 추론·최적화·모델 선택은 수행하지 않았습니다. 저장된 13,356개 분석 행과 9,240개 학습 기록의 기존 독립 검증을 SHA로 연결하며, 게시자는 새 추론·궤적 gate 재실행·미래 그룹 재분류를 하지 않습니다.

## 전체 9개 endpoint × 고정 12개 그림

- [20260903 / parent_candidate_residual: 원본 12장](galleries/20260903_parent_candidate_residual.md)
- [20260903 / ce_continue: 원본 12장](galleries/20260903_ce_continue.md)
- [20260903 / drive_pair_continue: 원본 12장](galleries/20260903_drive_pair_continue.md)
- [20260904 / parent_candidate_residual: 원본 12장](galleries/20260904_parent_candidate_residual.md)
- [20260904 / ce_continue: 원본 12장](galleries/20260904_ce_continue.md)
- [20260904 / drive_pair_continue: 원본 12장](galleries/20260904_drive_pair_continue.md)
- [20260905 / parent_candidate_residual: 원본 12장](galleries/20260905_parent_candidate_residual.md)
- [20260905 / ce_continue: 원본 12장](galleries/20260905_ce_continue.md)
- [20260905 / drive_pair_continue: 원본 12장](galleries/20260905_drive_pair_continue.md)

## 원본 보존과 검증 범위

원격 197파일 중 PT 12개를 제외한 185파일과 수집 receipt 1개를 로컬에 보존했습니다. 이 게시물에는 전체 요약과 원본 PNG 108장만 바이트 그대로 복사합니다. 약 125 MB의 raw JSONL을 중복 게시하지 않으며 개인 원본의 전체 파일명·SHA·크기를 inventory로 연결합니다. PT는 전송하거나 읽지 않았습니다.

원본 185파일과 receipt를 포함한 전체 로컬 186파일의 SHA는 독립 요약의 `input_sha256`과 게시 전후 대조합니다. receipt proof는 그 SHA에 연결된 원본에서 공개 가능한 검증 필드만 추출하며 원본 receipt 자체는 개인 폴더에 보존합니다. 99a9e51 독립 요약 소스와 d9a877/b6d32f 의존 소스도 고정 SHA로 검증합니다. 공개 파일 검증은 이 폴더의 SHA256SUMS를 사용하며 inventory에는 공개하지 않은 원본도 포함됩니다.
