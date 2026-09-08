# TRAIN oracle 진단 — 저장된 6개 체크포인트 결과

<!-- HH_260906 - Separate stored-cost diagnosis from new fitting, causal claims, and dataset admission. -->

기존 STOPMIX 3seed의 1540/2870 update 체크포인트를 TRAIN 1,147행씩 진단했습니다. 이 게시 검증은 저장된 **6,882행·82,584개 cost**만 다시 계산했으며 모델·optimizer·원본 데이터·이미지를 실행하거나 읽지 않았습니다. 모델 승격·새 데이터 승인도 없습니다.

| seed | update | selected ADE m | composite oracle ADE m | mean regret (loss units) | exact tie rows |
| --- | ---: | ---: | ---: | ---: | ---: |
| 20260903 | 1540 | 2.847000 | 1.017788 | 4.760657 | 120 |
| 20260903 | 2870 | 2.322302 | 0.857797 | 3.442241 | 121 |
| 20260904 | 1540 | 3.260007 | 0.967869 | 5.874873 | 123 |
| 20260904 | 2870 | 2.289605 | 0.867487 | 3.650767 | 127 |
| 20260905 | 1540 | 3.114266 | 0.971021 | 5.552209 | 120 |
| 20260905 | 2870 | 2.447421 | 0.827204 | 3.829348 | 124 |

![6개 저장 결과의 ADE와 regret](train_oracle_costs.png)

각 1,147행의 future-derived 분류는 stationary hold118 / moving-to-stop202 / continuing611 / other216 / unavailable0입니다. Warmup105는 별도 phase 분류로 위 그룹과 겹치므로 더하지 않습니다. 이는 미래 정답을 보고 나눈 진단 그룹이며 모델의 causal 입력이나 실제 신호등 상태가 아닙니다.

동률 판정은 저장된 float32 cost의 정확한 equality만 사용했습니다. 최소값 동률은 모두 STOP 후보6–11이며 이미 후보6이 선택돼 regret0입니다. 따라서 이 동률만으로 현재 선택 오차를 설명하지 않습니다.

같은 모델 입력 hash를 가진 Town01 11쌍(22행)은 원본6장 JPEG SHA와 저장된 logits도 같지만 target hash와 최소-cost 후보 집합이 다릅니다. 해당 관측 입력만 받는 결정적 선택기는 두 개의 서로 겹치지 않는 정답 집합을 동시에 만족시킬 수 없습니다. **라벨 오류·신호등 상태·유일한 성능 저하 원인·미래 예측 불가능성 일반론의 증거는 아닙니다.** 원본을 제거하거나 정답을 대체하지 않았습니다.

<!-- HH_260906 - Link the separately sealed original-camera illustration without changing the full saved-row diagnosis. -->
[원본 카메라 6장과 미래 궤적·속도 그림](input_ambiguity_illustration/README.md)은
11쌍 중 첫 번째를 설명하기 위한 별도 자료입니다. 원본 JPEG·두 샘플의 바이트를 보존했고,
새 촬영·Autoware 화면·학습 모델의 주행 성공으로 표시하지 않습니다.

Teacher index는 같은 lineage의 두 checkpoint 사이에 seed별 155, 109, 129행에서 바뀌었습니다. 후보 생성기와 selector가 함께 바뀌므로 index 변경 자체를 학습 불안정이나 잘못된 정답으로 해석하지 않습니다. 전체 episode/group와 모든 쌍 witness는 독립 요약에 있습니다.

[독립 전수 재계산](independent_summary.json) · [실제 실행 report](raw/report.json) · [실제 v2 계획](raw/plan.json) · [원격 전후 전송 검증](raw/transport_verification.json)

[계획 준비 경위](preparation/stopmix_training_oracles_plan_preparation_v1.json) · [미실행 v1](preparation/stopmix_training_oracles_plan_v1.json) · [실행 receipt](preparation/stopmix_training_oracles_v1_launch.json) · [종료 receipt](preparation/stopmix_training_oracles_v1_ssh_completion.json)

로컬 Python3.10 준비본 v1은 실행하지 않았습니다. Python3.12의 AST type_params 필드 차이 때문에 별도 v2를 실행 전에 선언했습니다. loss 원문·prefix bytes는 동일하며 AST hash를 사후 정규화하거나 v1이 실행됐다고 바꾸지 않았습니다.

## 6개 전체 저장 행

- [20260903 / parent1540 전체1,147행](raw/seed_20260903/parent1540/samples.jsonl)
- [20260903 / continued2870 전체1,147행](raw/seed_20260903/continued2870/samples.jsonl)
- [20260904 / parent1540 전체1,147행](raw/seed_20260904/parent1540/samples.jsonl)
- [20260904 / continued2870 전체1,147행](raw/seed_20260904/continued2870/samples.jsonl)
- [20260905 / parent1540 전체1,147행](raw/seed_20260905/parent1540/samples.jsonl)
- [20260905 / continued2870 전체1,147행](raw/seed_20260905/continued2870/samples.jsonl)

기존 full-corpus 무결성 검사는 test metadata/JSON/JPEG를 읽을 수 있지만 실제 진단 NN forward는 TRAIN만입니다. native oracle/regression의 일치는 원래 실행의 source-bound 증거이며, 이 도구는 native loss나 checkpoint를 재실행하지 않습니다. checkpoint는 원격 원본 SHA만 연결합니다.

[원본/공개 SHA·소스 provenance](publication_manifest.json) · [공개 파일 체크섬](SHA256SUMS) · [앞선 10epoch 결과](../08_stopmix_ten_epoch_continuation/results/README.md)

raw/SHA256SUMS는 원본 private 파일용 참조로 이름을 바꿔 보존했습니다. 공개 JSON은 account 경로만 가린 metadata view이며 raw SHA를 포함합니다. samples JSONL은 원본 바이트 그대로입니다. 공개 source는 감사를 위한 보관본이며 실행 요청이 아닙니다.
