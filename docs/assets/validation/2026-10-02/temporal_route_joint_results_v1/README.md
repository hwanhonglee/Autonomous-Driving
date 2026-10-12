# 주행 곡률·속도 공동 학습 — 최종 6000회

사전 기준71개 중 **63개 통과 / 8개 미달**입니다. 실패를 숨기거나 중간 checkpoint를 고르지 않았습니다.

정식 주기능 **0/9**, 하위 기능 **0/30**. 이번 결과는 학습과 오프라인 평가입니다. 폐루프 주행·전체 기능·실차 사용 승인이 아닙니다.

## 무엇을 개선했나요?

이전에는 차선 복귀 branch만 학습해 속도 오차를 줄일 수 없었습니다. 이번에는 같은101-tensor 모델에서 경로·속도·fusion/trunk54개 tensor를 함께 학습했습니다. 영상 입력부와 미학습 의미 head 등을 포함한47개 tensor는 부모와 바이트까지 같습니다.

- **MF02 경로 주행:** 직진·차선 복귀의 궤적 개선 대상입니다. 기존 회전 자료도 raw 곡률로 재학습했지만 실제 회전 성공을 입증한 것은 아닙니다.
- **MF03 속도·승차감:** 실제 속도 label로 선호 속도를 학습했습니다. decoder는 그대로2.5m/s 이하이며30kph SPD01 검증이 아닙니다.
- STOP/의도 head의 가중치는 동결했지만 공유 trunk 변화로 출력까지 같다는 보장은 없습니다. STOP은 아직 학습·승인하지 않았습니다.
- 실제18TRAIN: 새 회복9개 +기존 저속1개 +기존 직진·회전8개. 같은 Town07 경로의 HOLDOUT2개는 학습과 checkpoint 선택에서 제외했습니다.
- 고정6000 updates/24000회 노출: 회복12000 +기존 저속6000 +기존 직진·회전6000. 신규 회복은4.0~34.1초 구간이며 정지·출발 회복 학습은 아닙니다.
- 기존 known DEV599는 이전부터 사용한 진단 자료입니다. 독립 TEST나 모든 맵 일반화로 부르지 않습니다.

## 전후 요약

| 자료 | ADE 부모→최종(m) | FDE 부모→최종(m) | 순간 속도 MAE 부모→최종(m/s) |
|---|---:|---:|---:|
| 새 회복 TRAIN2718 | 0.643131 → 0.046899 | 1.184430 → 0.103205 | 0.213179 → 0.020267 |
| 미학습 HOLDOUT604 | 0.641233 → 0.045903 | 1.181464 → 0.102585 | 0.212704 → 0.020594 |
| 기존 저속 TRAIN327 | 0.078348 → 0.034600 | 0.176099 → 0.080718 | 0.024229 → 0.024056 |

## 미달 기준 — 전부 공개

| 기준 | 최종 | 허용 상한 |
|---|---:|---:|
| known599/carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320/raw_curvature | 0.003733910 | 0.003260269 |
| legacy/carla_town01_town01_straight_s0000_p02_ClearNoon_s0101_1662f5a7e646/raw_curvature | 0.001739892 | 0.001529462 |
| legacy/carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7/raw_speed_MAE5percent_preservation | 0.203362772 | 0.182242754 |
| legacy/carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p03_CloudyNoon_s0102_f586e8899633/raw_speed_MAE5percent_preservation | 0.063518307 | 0.057749394 |
| legacy/carla_town07_town07_straight_s0000_p02_CloudyNoon_s0102_384d262a3bf2/raw_speed_MAE5percent_preservation | 0.121097650 | 0.080136218 |
| known599/aggregate/raw_speed_MAE5percent_preservation | 0.385127660 | 0.331510308 |
| known599/carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9/raw_speed_MAE5percent_preservation | 0.408089347 | 0.383054745 |
| known599/carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320/raw_speed_MAE5percent_preservation | 0.366156267 | 0.288923289 |

[모든71개 판정·계보](result.json), [9개 TRAIN·2개 HOLDOUT·기존327/2487/599 전체 전후 비교](comparisons.md), [원자료SHA](evidence_manifest.json), [숫자 전용 검사](numeric_loader_receipt.json).

숫자 검사 `PASS_FULL101_NUMERIC_CPU_ONLY_NOT_CONTROL`. 12개 checkpoint는 SHA만 확인하고 `.pt`를 역직렬화하지 않았습니다. 모든 export의101개 float32 tensor 및47개 원본 바이트를 검사했습니다.

GPU 소유 2748.8초: baseline 포함 fit 2223.0초, 최종 평가 525.8초. 사전 fit 예산90분, 최종 평가는 별도입니다.

다음은 고정 최종 가중치를 동일 조건의 실제 폐루프 직진·회복·회전 시험에 적용해 실제 성능과 기존 성능 보존을 확인하는 단계입니다. 이 도구는 추가 학습이나 simulator를 실행하지 않습니다.

원자료: `/home/a/autoware_e2e/storage/experiments/training/2026-10-02/temporal_route_joint_v1`
