# 동일 크기 잔차 선택기 — 전체 실제 결과

<!-- HH_260906 - Preserve all outcomes, including regressions; this is a frozen-generator information ablation, not driving or admission. -->

기존 3개 생성기·12후보·입력 cache·composite teacher를 그대로 두고 **head 전용 6회**를 학습했습니다. 전체 모델을 다시 학습한 것이 아닙니다. 각각 1,540 step·6,155회 노출, 합계 9,240 step·36,930회 노출입니다. 3 baseline+6 head의 TRAIN1,147/VAL337 저장 행 13,356개와 원본 PNG108장을 검증했습니다.

| seed | 선택기 | VAL ADE m | VAL FDE m | TRAIN hold ADE m | hold STOP |
| --- | --- | ---: | ---: | ---: | ---: |
| 20260903 | original | 5.405720 | 11.599149 | 0.00003685 | 118/118 |
| 20260903 | prior_only_residual | 5.405720 | 11.599149 | 0.00003685 | 118/118 |
| 20260903 | candidate_residual | 5.267210 | 11.296644 | 0.00003685 | 118/118 |
| 20260904 | original | 5.205849 | 10.981457 | 0.00003685 | 118/118 |
| 20260904 | prior_only_residual | 5.205849 | 10.981457 | 0.00003685 | 118/118 |
| 20260904 | candidate_residual | 5.449730 | 11.719451 | 0.00003685 | 118/118 |
| 20260905 | original | 4.842267 | 10.030749 | 0.00003685 | 118/118 |
| 20260905 | prior_only_residual | 4.842267 | 10.030749 | 0.00003685 | 118/118 |
| 20260905 | candidate_residual | 4.535711 | 9.443228 | 0.00003685 | 118/118 |

![All validation outcomes](all_validation_endpoints.png)

A(prior_only_residual)와 B(candidate_residual)는 같은 449→256→1 구조, 할당 파라미터115,457개, 같은 초기 head 상태를 사용합니다. original score에 마지막 층이 0으로 초기화된 잔차를 더합니다. A의 geometry192개 입력만 0이며 B는 실제 고정 후보 geometry를 받습니다. 후보·순서·원래 CE×0.1·학습률·optimizer는 같지만 A의 비활성 geometry 가중치와 유효 정보/gradient는 다릅니다. 따라서 같은 파라미터 수가 같은 유효 용량을 뜻하지 않습니다.

B는 TRAIN 정지 유지118개에서 세 seed 모두 STOP118/118을 유지했지만, VAL seed20260904의 ADE/FDE는 원래 선택기보다 악화됐습니다. seed03·05의 두 지표는 감소했습니다. **VAL stationary_hold는 0개**이며 평균 VAL 개선으로 정지 성능을 검증할 수 없습니다. 승자·승격을 정하지 않았습니다.

아래 선택 변화는 평균 metric 일치에서 추정하지 않고 동일한 saved sample의 candidate index를 전수 비교했습니다. A의 변경은 전체 TRAIN/VAL에서 0개입니다. 초기 logit 바이트 일치는 독립 요약의 별도 실행 근거이고, 최종 score 자체가 같다는 뜻은 아닙니다.

| seed | split | arm | 변경 / 전체 선택 |
| --- | --- | --- | ---: |
| 20260903 | train | prior_only_residual | 0 / 1147 |
| 20260903 | train | candidate_residual | 103 / 1147 |
| 20260903 | val | prior_only_residual | 0 / 337 |
| 20260903 | val | candidate_residual | 20 / 337 |
| 20260904 | train | prior_only_residual | 0 / 1147 |
| 20260904 | train | candidate_residual | 95 / 1147 |
| 20260904 | val | prior_only_residual | 0 / 337 |
| 20260904 | val | candidate_residual | 38 / 337 |
| 20260905 | train | prior_only_residual | 0 / 1147 |
| 20260905 | train | candidate_residual | 56 / 1147 |
| 20260905 | val | prior_only_residual | 0 / 337 |
| 20260905 | val | candidate_residual | 27 / 337 |

[전수 독립 요약·이전 선택기와 비교](independent_summary.json) · [선택 변화 전체 witness](selection_changes.json) · [실제 report](raw/report.json) · [실행 전 계획](raw/plan.json) · [전달 검증](raw/transport_verification.json)

stationary_hold/moving_to_stop/continuing_motion/other_motion/unavailable_masks 전체 그룹 및 episode별 결과를 보존합니다. 이 그룹은 미래 정답을 이용한 사후 진단이며 NN 입력이나 실제 신호등 상태가 아닙니다. TRAIN은 in-sample, VAL은 반복 사용한 단일 개발 episode로 독립 holdout이 아닙니다. 기존 test 파일은 full-corpus 무결성 검사에서 읽힐 수 있지만 test NN·loss·metric·선택은 없습니다.

## 원본 9개 variant × 고정 12개 VAL 그림

- [20260903 / original: 모든 TRAIN/VAL행·PNG12장](galleries/20260903_original.md)
- [20260903 / prior_only_residual: 모든 TRAIN/VAL행·PNG12장](galleries/20260903_prior_only_residual.md)
- [20260903 / candidate_residual: 모든 TRAIN/VAL행·PNG12장](galleries/20260903_candidate_residual.md)
- [20260904 / original: 모든 TRAIN/VAL행·PNG12장](galleries/20260904_original.md)
- [20260904 / prior_only_residual: 모든 TRAIN/VAL행·PNG12장](galleries/20260904_prior_only_residual.md)
- [20260904 / candidate_residual: 모든 TRAIN/VAL행·PNG12장](galleries/20260904_candidate_residual.md)
- [20260905 / original: 모든 TRAIN/VAL행·PNG12장](galleries/20260905_original.md)
- [20260905 / prior_only_residual: 모든 TRAIN/VAL행·PNG12장](galleries/20260905_prior_only_residual.md)
- [20260905 / candidate_residual: 모든 TRAIN/VAL행·PNG12장](galleries/20260905_candidate_residual.md)

## 전체 6개 학습 history

- [20260903 / prior_only_residual: 1,540행](raw/seed_20260903/prior_only_residual/metrics.jsonl) · [head report](raw/seed_20260903/prior_only_residual/report.json)
- [20260903 / candidate_residual: 1,540행](raw/seed_20260903/candidate_residual/metrics.jsonl) · [head report](raw/seed_20260903/candidate_residual/report.json)
- [20260904 / prior_only_residual: 1,540행](raw/seed_20260904/prior_only_residual/metrics.jsonl) · [head report](raw/seed_20260904/prior_only_residual/report.json)
- [20260904 / candidate_residual: 1,540행](raw/seed_20260904/candidate_residual/metrics.jsonl) · [head report](raw/seed_20260904/candidate_residual/report.json)
- [20260905 / prior_only_residual: 1,540행](raw/seed_20260905/prior_only_residual/metrics.jsonl) · [head report](raw/seed_20260905/prior_only_residual/report.json)
- [20260905 / candidate_residual: 1,540행](raw/seed_20260905/candidate_residual/metrics.jsonl) · [head report](raw/seed_20260905/candidate_residual/report.json)

## 원본·전송·해석 범위

원격195개 중183개 원본과 transport receipt1개를 로컬에 보존했습니다. 새 head/logits PT12개는 원격에만 있고 SHA·크기로 연결합니다. 원래 cache 실행205개와 그 PT18개 및 continued parent 입력17개의 불변 검증도 유지합니다. 이 게시자는 어떤 PT도 읽거나 새 NN을 실행하지 않습니다. 실제 worker의 원래 cache6개 CPU weights-only 로드는 새 생성기 forward나 parent 모델 재학습이 아닙니다.

원본 PNG108장·samples JSONL18개·전체 metrics JSONL6개는 바이트 그대로 복사합니다. 개인 경로/GPU UUID를 가린 metadata·source·log는 원본/공개 SHA를 각각 기록합니다. 일부 공개 source의 placeholder는 실행 가능한 배포본이 아니라 감사용 참조입니다. numeric chart만 기존 저장 숫자로 렌더링하며 원본 그림을 수정하지 않습니다.

[실행 로그](execution/frozen_stopmix_residual_v1_console.log) · [종료 receipt](execution/frozen_stopmix_residual_v1_ssh_completion.json) · [이전 fresh-head 결과와 최초 launcher 거절 보존](../../10_frozen_stopmix_selector/results/README.md)

compact 행으로 비용 argmin·정확한 동률·regret·모든 집계를 재검산했으나 전체 XY/speed/target 배열이 없어 새 trajectory gate 실행이나 그룹 재분류를 하지 않았습니다. raw current vx를 사용하는 원래 gate 결과는 source와 저장 행으로 연결합니다. GPU float32 core metric과 Python 저장 행 집계를 별도 보존합니다. legacy 개발 데이터·물리 한계·runtime allowlist·정답·새 원본 데이터 승인 상태는 바꾸지 않았습니다.

[원본/공개 SHA provenance](publication_manifest.json) · [공개 체크섬](SHA256SUMS) · [원본 원격 체크섬 참조](raw/original_remote_SHA256SUMS.txt)

원본 원격 체크섬에는 생략 PT와 비식별화 전 파일이 있으므로 공개 tree 검증에는 이 폴더의 SHA256SUMS만 사용합니다. 실행 native source b478, 외부 residual worker c1572/core d4bf, 새 요약 d9a877을 구분합니다. 실제 차량 주행·실시간 추론·배포 성능이나 데이터 승인 결과가 아닙니다.
