# 잘못 고른 경로의 손해 크기까지 반영하는 학습 A/B

<!-- HH_260906 - Predeclare an optional cost-aware training objective without replacing the physical decoder or accepting denied data. -->

실행 전 고정한 [계획](../../../../../../../config/portable_e2e_candidate_regret_20260909.json)입니다.
학습 소스는 `05c5847`로 고정합니다. 이 선언 시점에는 아직 새 모델을 학습하지 않았습니다.

앞선 모델은 여섯 경로 후보 중 고른 경로와 정답으로 사후 고른 최선 후보 사이에 오차가 남았습니다.
이번에는 기존 정답 후보 분류 손실을 유지하면서, **큰 손해를 내는 후보에 높은 선택 확률을 주면
추가 손실을 받도록** 학습합니다. 가속도 제거·선택기만 재학습·후보 수 변경 실험이 아닙니다.

| 조건 | A 기존 손실 | B 손해 크기 추가 |
|---|---|---|
| 모델 | 같은 physical.v1, 954,590 파라미터, 후보 6개 | 동일 |
| 기존 best-of-K 복합 회귀·분류 손실 | 그대로, 분류 계수 0.1 | 동일 |
| 확률 가중 복합 손해 추가 계수 | 0 | 0.1 |
| 학습 | 전체 파라미터 새 학습, 3개 고정 seed | 동일 seed별 새 학습 |
| 학습량 | 1,540 step·batch 4·6,155 sample exposure | 동일 |
| 데이터 | 기존 v3 train1,147 / val337 | 동일 |

추가 항은 `평균[Σ softmax(logit) × detach(후보 복합 비용 − 최소 복합 비용)]`입니다.
후보의 비용은 기존 회귀·분류 정답에 사용한 동일한 복합 비용입니다. XY ADE만으로 정답 후보를
바꾸지 않습니다. 추가 항의 직접 gradient는 선택 logit에만 흐르지만, 공유 encoder가 학습되므로
간접적으로 후보 경로도 변할 수 있습니다.

추가 복합 손해 값과 `selected ADE − oracle ADE`의 미터 오차는 다른 지표입니다.
A/B의 **총 loss 정의도 다르므로** 원래 loss 숫자만 비교해 모델이 좋아졌다고 판정하지 않습니다.
실제 채택 기준은 세 seed 모두의 selected ADE/FDE 개선, geometry 통과 수 비감소,
속도 MAE 5% 이내와 기존 절대 오차 조건입니다. 전체 6회 마지막 checkpoint만 비교합니다.

원본 v3의 warmup 포함·감속 정답 한계는 그대로 남습니다. 9월 8일 미승인 수집과 이번 새
substep 수집을 넣지 않으며, 새 데이터가 승인됐다고 설명하지 않습니다. 전체 corpus 무결성
검사는 test 파일도 읽지만 test의 학습 loss·추론·평가·모델 선택 사용은 하지 않습니다.

GPU 0·기존 개인 venv만 사용합니다. 학습 단계별 최대 600초, 평가·감사 각각 최대 120초와
전체 정리 여유 300초를 합산해 오전 10시 경계 전에 완료할 시간이 있는지 검사합니다.
기본 loss 계수 0의 기존 동작·6개 설정 필드는 보존했고, 계수 0.1은 별도 7번째 필드로 기록됩니다.
기존 secure runtime bundle은 이 새 연구 loss 설정을 거절하며 현재 차량·shadow 모델을 바꾸지 않습니다.

## 실제 학습·평가 결과

<!-- HH_260906 - Count complete optimizer runs and retain every failed paired seed; do not compare unlike total objectives. -->

**02:21:35–02:44 KST에 전체 모델 6회·평가 6회·gate 감사 6회를 완료했습니다.**
총 9,240 optimizer step이며, 02:44:20 KST 원격 확인에서 GPU 0 사용 메모리 0 MiB,
해당 학습 작업 종료·잠금 해제를 확인했습니다. 완료한 학습을 계속 실행 중이라고 표시하지 않습니다.

| Seed | A selected ADE / FDE (m) | B selected ADE / FDE (m) | 상대 기준 | B 절대 기준 |
|---|---:|---:|---|---|
| 20260903 | 3.8743 / 9.8277 | 4.2133 / 9.7584 | FAIL: ADE 악화 | FAIL |
| 20260904 | 6.2585 / 12.8975 | 5.4218 / 12.4800 | PASS | FAIL |
| 20260905 | 4.7895 / 11.8663 | 3.4569 / 7.5884 | PASS | FAIL |

B는 두 seed에서 상대 기준을 통과했지만 **세 seed 모두 통과 조건과 절대 오차 조건에 미달해 미채택**입니다.
각 실행의 selected geometry는 336/337입니다. 공통 거절 1개는 기존 val index108의
현재 속도 약 30.02757 kph가 30 kph 입력 gate를 초과한 사전 조건 실패이며, 해당 sample을
제거하거나 검사하지 못한 경로 형상을 통과로 간주하지 않습니다.

![동일한 회귀 지표와 별도 보조 손실 학습 곡선](evidence/plots/01_common_learning_and_auxiliary.png)

![세 seed의 최종 validation 비교](evidence/plots/02_paired_validation.png)

학습 곡선은 전체 배치와 마지막 40 step을 포함합니다. B만 존재하는 보조 손실은 별도 축입니다.
실제 val 카메라와 예측 경로 PNG도 **모든 6개 모델에 12장씩, 총 72장**을 보존했습니다.
[seed03 A](evidence/runs/seed_20260903/A_original_loss/evaluation/trajectories/)와
[seed03 B](evidence/runs/seed_20260903/B_cost_aware_selector/evaluation/trajectories/)를 포함해
각 seed/arm 폴더에서 확인할 수 있습니다. 이 그림은 오프라인 예측이며 Autoware 주행 화면이 아닙니다.
현재 평가기의 기본 선택은 각 실행의 **첫 12개 validation sample**이므로 이 PNG들은 시작 warmup
구간에 집중되어 있고 전체 주행의 대표 시각 자료가 아닙니다. 수치 평가는 337개 전체를 사용했습니다.
다음 구조 비교에는 전체 validation 구간에 고르게 배치한 동일 12개 index의 그림을 별도로 추가합니다.

[전체 판정](evidence/results/summary.json) · [원격 종료·전송 대조](evidence/provenance/remote_completion.json) ·
[게시 원본/파생본 해시](evidence/publication_manifest.json) · [전체 SHA-256](evidence/SHA256SUMS).

원격 보고·로그·PNG 116개, 5,723,735 bytes와 마지막 checkpoint 6개의 원격 바이트 해시를 확인했습니다.
checkpoint 텐서를 로컬에 내려받거나 실행하지 않았습니다. 게시 108개 파일 중 실행 메타데이터는
개인 경로를 치환한 파생본임을 표시했고, 학습 history와 그림은 원본 바이트를 유지합니다.
게시 단계는 단순한 보고서 내부 해시 일치뿐 아니라 실행 전 커밋된 계획 `10ee27de…`와
실제 학습 소스 `05c5847…`, 원격 launch 기록을 명시적으로 대조했습니다.

다음 구조 비교에서는 이번 미채택 보조 손실을 함께 바꾸지 않고 원래 손실을 유지합니다.
기존 주행 후보 6개를 보존한 채 정지 후 재가속하지 않는 후보 6개를 더하는 별도 연구 모델을
준비합니다. 후보 선호를 학습한다는 것과 신호·장애물 때문에 정지할 의도를 학습한다는 것은 다릅니다.
