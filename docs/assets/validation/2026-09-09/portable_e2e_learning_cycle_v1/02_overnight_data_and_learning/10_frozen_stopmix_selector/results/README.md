# 생성기 고정 선택기 — 실제 6개 head 학습의 전체 결과

<!-- HH_260906 - Report fixed saved outcomes and group regressions without promotion or causal interpretation. -->

원래 3개 STOPMIX 생성기·12후보를 고정하고 A/B 선택기만 각각 새로 학습했습니다. **전체 모델 6회가 아니라 head 전용 6회**이며, 각 1,540 step·6,155회 표본 노출입니다. 합계 9,240 step·36,930회 노출, 3 baseline+6 head의 TRAIN/VAL 총 13,356개 저장 행을 검증했습니다.

| seed | 선택기 | VAL337 ADE m | VAL FDE m | TRAIN hold118 ADE m | hold STOP 선택 |
| --- | --- | ---: | ---: | ---: | ---: |
| 20260903 | original | 5.405720 | 11.599149 | 0.00003685 | 118/118 |
| 20260903 | linear_pair_reset | 4.976841 | 10.991894 | 0.00003685 | 118/118 |
| 20260903 | candidate_reset | 4.458232 | 10.172761 | 1.08803268 | 0/118 |
| 20260904 | original | 5.205849 | 10.981457 | 0.00003685 | 118/118 |
| 20260904 | linear_pair_reset | 4.397637 | 9.529405 | 0.00003685 | 118/118 |
| 20260904 | candidate_reset | 4.238749 | 9.658102 | 0.91999198 | 0/118 |
| 20260905 | original | 4.842267 | 10.030749 | 0.00003685 | 118/118 |
| 20260905 | linear_pair_reset | 4.096361 | 8.782228 | 0.00003685 | 118/118 |
| 20260905 | candidate_reset | 4.393915 | 9.516988 | 0.41631564 | 0/118 |

![All endpoints and retained hold regression](all_endpoints_and_hold_regression.png)

B(candidate_reset)의 VAL ADE는 세 seed 모두 원래 선택기보다 줄었지만, TRAIN 정지 유지118개에서는 세 seed 모두 STOP 선택이 0/118로 퇴행했습니다. 원래 선택기와 A는 118/118을 유지합니다. **VAL에는 stationary_hold 표본이 0개**이므로 VAL 평균만으로 정지 성능 개선을 주장할 수 없습니다. B는 A보다 seed05의 VAL ADE가 나쁘고, seed04·05의 VAL FDE도 나쁩니다. 승자·승격은 정하지 않았습니다.

A는 두 Linear(256,6), 총3,084 parameters이고 B는 공유448→256→1 MLP, 총115,201 parameters입니다. 후보 정보와 구조·용량 효과가 섞여 있습니다. hard-argmin composite teacher·CE×0.1·학습 순서·후보·정답·mask는 바꾸지 않았습니다. 정확한 동률만 사용하며, 동률이나 loss floor만으로 퇴행 원인·해결책을 단정하지 않습니다.

미래 정답으로 분류한 stationary_hold/moving_to_stop/continuing_motion/other_motion/unavailable_masks 전체 그룹을 별도 보존합니다. TRAIN은 in-sample, VAL은 반복 사용한 단일 개발 episode로 독립 holdout이 아닙니다. 그룹은 입력이 아니라 사후 진단이며 실제 신호등 상태를 알려주지 않습니다. 기존 full-corpus 무결성 검사는 test 파일을 읽을 수 있지만 test NN forward·학습·metric·선택은 없습니다.

[독립 전수 요약](independent_summary.json) · [실제 report](raw/report.json) · [실행 전 계획](raw/plan.json) · [원격/로컬 전달 검증](raw/transport_verification.json)

## 모든 저장 행·고정 그림

- [20260903 / original: 고정 PNG12장·TRAIN/VAL 전체행](galleries/20260903_original.md)
- [20260903 / linear_pair_reset: 고정 PNG12장·TRAIN/VAL 전체행](galleries/20260903_linear_pair_reset.md)
- [20260903 / candidate_reset: 고정 PNG12장·TRAIN/VAL 전체행](galleries/20260903_candidate_reset.md)
- [20260904 / original: 고정 PNG12장·TRAIN/VAL 전체행](galleries/20260904_original.md)
- [20260904 / linear_pair_reset: 고정 PNG12장·TRAIN/VAL 전체행](galleries/20260904_linear_pair_reset.md)
- [20260904 / candidate_reset: 고정 PNG12장·TRAIN/VAL 전체행](galleries/20260904_candidate_reset.md)
- [20260905 / original: 고정 PNG12장·TRAIN/VAL 전체행](galleries/20260905_original.md)
- [20260905 / linear_pair_reset: 고정 PNG12장·TRAIN/VAL 전체행](galleries/20260905_linear_pair_reset.md)
- [20260905 / candidate_reset: 고정 PNG12장·TRAIN/VAL 전체행](galleries/20260905_candidate_reset.md)

## 6개 전체 학습 기록

- [20260903 / linear_pair_reset: 전체1,540 step](raw/seed_20260903/linear_pair_reset/metrics.jsonl) · [head report](raw/seed_20260903/linear_pair_reset/report.json)
- [20260903 / candidate_reset: 전체1,540 step](raw/seed_20260903/candidate_reset/metrics.jsonl) · [head report](raw/seed_20260903/candidate_reset/report.json)
- [20260904 / linear_pair_reset: 전체1,540 step](raw/seed_20260904/linear_pair_reset/metrics.jsonl) · [head report](raw/seed_20260904/linear_pair_reset/report.json)
- [20260904 / candidate_reset: 전체1,540 step](raw/seed_20260904/candidate_reset/metrics.jsonl) · [head report](raw/seed_20260904/candidate_reset/report.json)
- [20260905 / linear_pair_reset: 전체1,540 step](raw/seed_20260905/linear_pair_reset/metrics.jsonl) · [head report](raw/seed_20260905/linear_pair_reset/report.json)
- [20260905 / candidate_reset: 전체1,540 step](raw/seed_20260905/candidate_reset/metrics.jsonl) · [head report](raw/seed_20260905/candidate_reset/report.json)

## 첫 launcher 거절과 원본 보존

첫 launcher는 잘못된 output namespace 때문에 GPU lease·모델 import·데이터 로딩·optimizer 생성 전에 거절됐습니다. head fit0/NN forward0이며 실패한 학습으로 더하지 않습니다. 둘째는 output 경로만 수정해 같은 worker/core/plan으로 실행했습니다. 최초 실패의 로그와 receipt도 보존했습니다.

[첫 거절 로그](execution/frozen_stopmix_selector_v1_console.log) · [첫 종료 receipt](execution/frozen_stopmix_selector_v1_ssh_completion.json) · [둘째 로그](execution/frozen_stopmix_selector_v2_console.log) · [둘째 종료 receipt](execution/frozen_stopmix_selector_v2_ssh_completion.json)

원격205개 파일 중187개 원본을 로컬에 받았고 transport receipt1개를 더했습니다. **18개 cache/head/logits PT는 원격에만 있으며 SHA·크기만 연결합니다.** 이 게시자는 PT를 읽거나 모델을 실행하지 않습니다. 원본 108 PNG, 18개 samples JSONL, 6개 metrics JSONL 및 cache journal은 그대로 복사합니다. JSON metadata/source/log에서 개인 경로·GPU UUID만 가린 경우 원본 SHA와 공개 SHA를 함께 기록합니다. 공개 source는 감사용 참조이며 일부 placeholder 때문에 실행 가능한 배포본이 아닙니다.

저장된 candidate 비용 argmin·정확한 동률·regret·전체 행 집계는 재검산했습니다. compact JSONL에는 전체 XY/speed·target speed 배열이 없으므로 이 단계가 새로운 full-trajectory gate 실행이나 그룹 재분류를 했다고 주장하지 않습니다. 원래 raw current vx를 사용한 gate는 실행 source와 저장된 후보별 gate 결과로 연결됩니다. GPU float32 core 집계는 Python 저장 행 집계와 별도로 보존합니다.

[원본/공개 provenance](publication_manifest.json) · [공개 체크섬](SHA256SUMS) · [원본 원격 체크섬 참조](raw/original_remote_SHA256SUMS.txt)

원본 원격 체크섬은 생략 PT와 비식별화 전 파일을 포함하므로 공개 tree의 검증 명령으로 쓰지 않습니다. 이 폴더의 SHA256SUMS만 공개 파일에 해당합니다. source b478과 외부 worker469b/core4716을 구분하며 기존 데이터·runtime gate·모델 allowlist·정답을 변경하거나 새 데이터를 승인하지 않았습니다.

## 게시 도구 v1의 실패 보존

첫 게시 도구는 211개 파일을 만든 뒤, 소스 안의 비밀키 검사 문자열 자체를 실제 키로 오인하여 마지막 개인정보 검증에 실패했습니다. 실제 비밀키 노출은 발견되지 않았으며 211개 원본과 210개 payload SHA를 모두 private 경로에 그대로 보존했습니다. 이 v2는 완전한 PEM private-key header를 검출하도록 수정하고 generic·RSA·OpenSSH·encrypted 헤더 거절 및 자체 소스 검증 회귀 테스트를 추가했습니다. 최초 게시를 성공으로 바꾸거나 학습·그림을 다시 생성한 실험으로 세지 않습니다.

[최초 게시 실패 보존 receipt](publication_attempt_001_receipt.json). 이 receipt는 root가 반환 traceback을 기록한 설명 자료이며 raw console stream이 아닙니다. 최초 publisher SHA975b927c…와 보존 tree 전체 SHA는 provenance에 연결합니다.
