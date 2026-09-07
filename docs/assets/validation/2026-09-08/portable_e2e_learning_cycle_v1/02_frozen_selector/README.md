# 생성기를 고정한 선택기 9회 학습 — 실제 결과

<!-- HH_260906 - Preserve failed outcomes, unchanged frozen candidates and original evidence hashes in public views. -->

> 공개 JSON은 계정별 경로를 치환한 metadata view입니다. 원본 SHA를 보존했지만 독립 재실행에는 private 원본이 필요합니다.

실제 완료: `2026-09-07T16:32:00.964042+00:00`. 원격 기존 개인 py312 venv의 **GPU0만** 사용했습니다. 로컬에서 검증·그래프·이 자료를 생성했습니다.
전체 모델 재학습이 아니라 **고정 C 생성기 3개 × 선택기 3종 = head 전용 학습 9회**입니다.
3-seed 종합 판정: `linear_continue` 상대 **FAIL**, 절대 **FAIL** / `linear_reset` 상대 **FAIL**, 절대 **FAIL** / `candidate_reset` 상대 **FAIL**, 절대 **FAIL**. 운용 checkpoint는 교체하지 않았습니다.
learned closed-loop 주행, 실시간 10 Hz 성능, 장애물 회피·차선변경 또는 실차 안전을 증명하지 않습니다.

## 먼저 볼 그림

![실제 선택기 학습 기록](visuals/01_measured_scorer_training.png)

![원래 C와 최종 선택기의 동일 캐시 비교](visuals/02_same_cache_validation.png)

학습 그래프의 옅은 선은 1,540개 실제 batch, 진한 선은 노출 표본 수로 가중한 epoch 평균입니다.
batch4 중 매 287번째는 3개입니다. 5개 full epoch + 420개 노출의 partial epoch = **6,155 exposures**이며 6개 full epoch가 아닙니다.
분류 loss와 clipping 전 gradient norm만 그렸습니다. 최적화 loss는 분류 loss의 0.1배이며 중간 val 측정점은 만들지 않았습니다.

## 모든 seed 결과

| Seed | 모델/head | val ADE m | val FDE m | 속도 MAE m/s | selected geometry | 상대 | 절대 |
|---|---|---:|---:|---:|---:|---|---|
| 20260903 | original_c_same_cache | 3.874328 | 9.827706 | 1.459928 | 336/337 | 기준 | 기준 |
| 20260903 | linear_continue | 4.000330 | 9.653150 | 1.437095 | 336/337 | FAIL | FAIL |
| 20260903 | linear_reset | 4.094595 | 10.082237 | 1.532413 | 336/337 | FAIL | FAIL |
| 20260903 | candidate_reset | 3.852200 | 8.844149 | 1.302575 | 336/337 | PASS | FAIL |
| 20260904 | original_c_same_cache | 6.258545 | 12.897454 | 1.999795 | 336/337 | 기준 | 기준 |
| 20260904 | linear_continue | 6.178727 | 12.812708 | 2.083774 | 336/337 | PASS | FAIL |
| 20260904 | linear_reset | 5.976796 | 12.701923 | 2.140251 | 336/337 | FAIL | FAIL |
| 20260904 | candidate_reset | 5.228551 | 11.255055 | 1.803051 | 336/337 | PASS | FAIL |
| 20260905 | original_c_same_cache | 4.789547 | 11.866288 | 1.613773 | 336/337 | 기준 | 기준 |
| 20260905 | linear_continue | 5.226921 | 12.644825 | 1.840722 | 336/337 | FAIL | FAIL |
| 20260905 | linear_reset | 5.314008 | 12.805577 | 1.896471 | 336/337 | FAIL | FAIL |
| 20260905 | candidate_reset | 5.154170 | 11.271210 | 1.765101 | 336/337 | FAIL | FAIL |

각 방식은 모든 seed에서 ADE/FDE 개선, selected geometry 비악화, 속도 MAE 증가 5% 이하를 동시에 만족해야 합니다.
별도 절대 기준은 1/3/6.4초 ADE ≤ 0.5/1/2 m, 6.4초 FDE ≤ 4 m입니다. 좋은 seed만 골라 성공으로 표시하지 않았습니다.
깊이·용량·입력·초기화가 함께 달라졌으므로 후보 XY 입력만의 인과 효과로 단정할 수 없습니다.

## 원본 경로 그림 72장

아래 링크마다 val337의 고정 index **0, 67, 134, 201, 268, 336** PNG 6장이 있습니다.
좌표는 ego 중심이며 모든 후보와 정답 궤적을 보여주는 **오프라인 예측**입니다. CARLA 주행 녹화나 Autoware 실시간 화면이 아닙니다.

| Seed | 원래 C | Linear 이어 학습 | Linear 새 초기화 | 후보 인지 MLP |
|---|---|---|---|---|
| 20260903 | [original_c_same_cache](routes/seed_20260903/original_c_same_cache/README.md) | [linear_continue](routes/seed_20260903/linear_continue/README.md) | [linear_reset](routes/seed_20260903/linear_reset/README.md) | [candidate_reset](routes/seed_20260903/candidate_reset/README.md) |
| 20260904 | [original_c_same_cache](routes/seed_20260904/original_c_same_cache/README.md) | [linear_continue](routes/seed_20260904/linear_continue/README.md) | [linear_reset](routes/seed_20260904/linear_reset/README.md) | [candidate_reset](routes/seed_20260904/candidate_reset/README.md) |
| 20260905 | [original_c_same_cache](routes/seed_20260905/original_c_same_cache/README.md) | [linear_continue](routes/seed_20260905/linear_continue/README.md) | [linear_reset](routes/seed_20260905/linear_reset/README.md) | [candidate_reset](routes/seed_20260905/candidate_reset/README.md) |

## 데이터·검증·재현 범위

- Train 1,147 / 개발용 val 337을 분리했습니다. Test는 전체 corpus 무결성 검사 대상일 수 있지만 모델 예측·학습·선택·성능 분석에 사용하지 않았습니다.
- 원본 cache·생성기·선택 head의 전후 hash와 후보/oracle 불변성을 검증했습니다. 요약기는 tensor를 재로드하지 않고 bytes hash와 고정 소스 worker의 증명을 확인합니다.
- [summary.json](summary.json): 엄격한 146개 원본 입력의 SHA와 모든 판정. [plot_inputs.json](visuals/plot_inputs.json): 실제 학습 기록 9개·요약·그래프·현재 발행 스크립트의 SHA.
- [execution_record.json](provenance/execution_record.json): 원격 실행 commit/runner SHA와 별도의 로컬 발행 코드 SHA. 발행 시 미커밋 코드의 SHA를 실행 commit으로 가장하지 않습니다.
- [publication_manifest.json](publication_manifest.json): 원본 SHA / 공개 파일 SHA와 변환 종류. [SHA256SUMS](SHA256SUMS): 이 폴더의 모든 다른 파일 검증.
- 원격 raw runner·checkpoint·캐시·logit tensor는 공개하지 않았습니다. PNG 72장과 숫자 metrics.jsonl은 bytes 그대로이며 head report의 중복 history/sample IDs는 원본 SHA를 남기고 view에서 제외했습니다.

로컬 공개 파일 무결성 확인:

```bash
cd docs/assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/02_frozen_selector
sha256sum -c SHA256SUMS
```
