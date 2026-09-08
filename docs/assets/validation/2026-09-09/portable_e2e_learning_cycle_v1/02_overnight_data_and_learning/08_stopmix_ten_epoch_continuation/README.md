# 08 — 같은 6개 모델의 고정 10-epoch 이어학습

<!-- HH_260906 - Predeclare a duration-only continuation and distinguish inherited history from new optimizer updates. -->

04:07:59–04:30:32 KST에 원격 GPU 0에서 **6개 이어학습·평가·감사·행동 분석 24단계**를
완료했습니다. 04:31 KST에 원본·결과 전후 해시, GPU 0 작업 종료와 lease 반환을 확인했습니다.
모든 모델은 절대 품질에 미달하므로 **미채택**입니다.

[04의 원래 실험](../04_stopmix_learning/README.md)에서 학습한 A/B 각 세 seed의
checkpoint·Adam·RNG 상태를 새로운 출력 폴더로 독립 복사한 뒤 모두 같은 양만큼
이어학습합니다. 원본 파일은 변경하지 않습니다. **새 초기화 6회가 아니라 기존 6개 모델의
이어학습**이며, 기존 33회 전체 모델 학습 수와 구분합니다.

## 고정한 비교

| 항목 | 기존 종점 | 이번 고정 종점 |
|---|---|---|
| 모델당 optimizer step | 1,540 | 2,870 |
| 모델당 누적 sample 노출 | 6,155 | 11,470 |
| 데이터 전체 반복 | 약 5.37 epoch | 정확히 10 epoch |
| 이번 추가 학습량 | 해당 없음 | 1,330 step · 5,315 노출 |

train 1,147개는 기존 데이터 그대로입니다. 11,470개 신규 장면을 수집한 뜻이 아닙니다.
모델·loss·learning rate·seed·batch 4·worker 0·checkpoint 간격 154는 유지하고
`max_steps`만 바꿉니다. 세 seed는 20260903 / 20260904 / 20260905이며 각각
physical DRIVE 6후보와 DRIVE+STOP 12후보를 모두 평가합니다.

앞선 학습에서 완전한 epoch 간 학습 손실 감소가 관측되어 학습량 부족 가설을 분리해
확인합니다. 더 오래 학습하면 validation도 좋아진다고 가정하지 않습니다. 모든 기존 종점과
새 종점을 함께 보고하며, 좋은 seed·checkpoint만 고르거나 통과할 때까지 재시도하지 않습니다.

## 실행과 증거 경계

- 원격 학습 소스는 clean commit `b478f02`로 고정합니다. 로컬 문서·진단 도구의 최신
  commit을 학습 중인 원격 저장소에 덮어쓰지 않습니다.
- 각 모델은 train → validation 평가 → 경로 감사 → 행동 분석의 순서로 진행하며 총 24단계입니다.
- validation은 기존 337개 전부입니다. 행동 그림의 12개 인덱스도 실행 전에 고정했고,
  전체 수치에는 337개를 사용합니다. test를 신경망 평가·학습·모델 선택에 사용하지 않습니다.
- 기존 metrics 1–1,540행은 복사 이력, 1,541–2,870행만 이번 optimizer update입니다.
  원본 metrics 접두부와 부모 checkpoint 해시를 전후 확인합니다.
- GPU 0만 사용하고 기존 개인 venv 밖 설치, GPU 1 접근, runtime 교체, 차량 제어는 하지 않습니다.
- 각 단계에 timeout이 있고 오전 10시 전 종료 확인 예산을 확보했습니다. 실패도 별도 보존합니다.

사전 계획 [plan.json](plan.json)의 SHA256은
`bcdf31f09838750cd24fcabf063a9baeac8fb317ed045e6a41d734fff2e6233b`입니다.
선언은 04:05:50 KST이며 학습 시작보다 앞섭니다. 실제 실행 worker SHA256은
`e51d2c4e02ba15a24a376944c08a387d616b8d749585562732ee483b913958e3`입니다.
계획 원본은 변경하지 않고 결과·그림·원본 검증 내역을 별도 추가합니다.

## 완료 결과

<!-- HH_260906 - Retain every lineage and report short-horizon regression even when final displacement improves. -->

| 계보 | ADE 기존 → 10 epoch (m) | FDE 기존 → 10 epoch (m) | 자기 기존 종점 대비 상대 조건 | 절대 조건 |
|---|---:|---:|---|---|
| seed 03 · A DRIVE | 3.87433 → 4.48201 | 9.82771 → 9.80604 | FAIL | FAIL |
| seed 03 · B DRIVE+STOP | 6.21767 → 5.40572 | 15.03221 → 11.59915 | PASS | FAIL |
| seed 04 · A DRIVE | 6.25855 → 6.45213 | 12.89745 → 13.41993 | FAIL | FAIL |
| seed 04 · B DRIVE+STOP | 5.16484 → 5.20585 | 12.46730 → 10.98146 | FAIL | FAIL |
| seed 05 · A DRIVE | 4.78955 → 4.43988 | 11.86629 → 9.24257 | PASS | FAIL |
| seed 05 · B DRIVE+STOP | 4.91565 → 4.84227 | 12.13344 → 10.03075 | PASS | FAIL |

위 상대 조건은 각 모델의 **자기 기존 종점 대비** ADE·FDE 감소, 속도 MAE 5% 이내,
경로 gate 비퇴행입니다. 새 B가 A를 이겼다는 뜻도 차량 배포 통과라는 뜻도 아닙니다.
**6개 모두 1초·3초 ADE는 오히려 증가했습니다.** 장기 FDE 개선 일부만 보고 전체 주행이
좋아졌다고 판단하지 않습니다. 경로 gate는 전부 336/337로 같고 기존 1개 초과도 유지합니다.

추가 학습은 전체 합계 7,980 optimizer step·31,890 sample 노출입니다.
원격 232개 파일을 해시 확인했고 checkpoint 6개는 원격에 남겨 해시로 연결했습니다.
나머지 226개 파일(21,723,125 bytes)은 로컬에 원본 그대로 수집했습니다.
이 검증은 checkpoint tensor를 로컬에서 다시 로드한 결과가 아닙니다.

## 전체 결과와 그림

<!-- HH_260906 - Link the verified complete publication rather than selecting favorable seeds or frames. -->
[전체 6개 계보의 학습·평가·행동 분석](results/README.md)에 원본 학습 이력 17,220행,
행동 분석 2,022행, 실행 전 고정한 경로 PNG 72장과 실제 지표 그래프 2장을 모았습니다.
공개 173개 파일은 [SHA256SUMS](results/SHA256SUMS)로 확인할 수 있습니다.
이미지는 오프라인 모델 예측이며 차량 자율주행 촬영이 아닙니다.
공개 생성 중 링크 검증 순서 오류가 한 번 있었고, 미완성 생성물과 실패 기록도 보존했습니다.
이 수정은 재학습·재추론이나 지표 변경이 아닙니다.
