# 실제 저속 회전 자료를 넣은 20,000회 학습

<!-- HH_261002 - State the actual remote optimizer start and bounded objective, not an untrained capability claim. -->

PRO6000 GPU0에서 **2026-10-02 13:09:40 KST에 실제 학습을 시작**했다. **20,000회 업데이트·80,000회 입력 노출을 완료했고, 15:12에 최종 전체 평가까지 끝났다.** 학습은 102.74분, 이후 평가는 18.22분 걸렸다. 학습 소스 30개와 전체 체크포인트·업데이트 기록을 보존하고 로컬 전량 동일성 검사를 완료했다. **74개 중 60개 통과·14개 미달로 미채택**이다. 아래처럼 좌회전 TRAIN 예측은 좋아졌지만 기존 성능 보호도 함께 충족해야 한다.

<!-- HH_261002 - Show matched final-versus-parent TRAIN errors separately from actual driving and unchanged regression failures. -->
| 같은 좌회전 TRAIN 347개 / 작을수록 좋음 | 기존 final6000 | 새 final20000 |
| --- | ---: | ---: |
| 6.4초 미래 위치 평균 오차 | 0.3471m | 0.0619m |
| 6.4초 마지막 위치 오차 | 0.8054m | 0.2339m |
| 순간 속도 평균 오차 | 0.0462m/s | 0.0113m/s |

**미달 14개:** 기존 저속 위치 보호 2개, 복귀 속도 오차 20% 개선 목표 11개, 알려진 Town03 우회전의 원래 속도 보호 1개다. 기존 71개만 비교하면 새 모델은 57/71이며, 이전 정상 모델의 63/71보다 낮다. 새 좌회전 3개를 더한 60/74를 이전 63/71과 같은 점수처럼 비교하지 않는다. 이는 학습 자료 재평가이지 독립 장소나 실제 주행 합격이 아니다.

<!-- HH_261002 - Replace the old running capture claim with the completed physical comparison and unchanged accuracy failure. -->
두 모델의 실제 좌회전 비교도 끝났다. 기존 모델은 차선 침범 1회로 약 30초에서 종료됐고, 새 모델은 **89.13° 회전·출구 뒤 10.70m·3초 안정 주행과 충돌·침범 0**을 기록했다. 그러나 새 모델의 중심 오차 p95 **0.413m > 기존 0.35m**여서 FAIL이다. 두 선언은 어느 주행보다 먼저 함께 고정했다. [실제 전체 GIF·PNG와 같은 처음 513개 상태의 비교](../23_actual_left_joint_left45_comparison/README.md)를 확인할 수 있다. 같은 새 가중치로 추종 거리만 바꾸는 두 조건을 별도 사전 고정했으며, 공간 예약 검사에서 차단돼 아직 실제 실행 전이다. 교사 자료의 [전체 화면](../21_actual_low_speed_left_teacher_v2/README.md)과 신경망 주행을 구분한다.

## 이번에 무엇을 바꾸는가

기존 직진·복귀·속도 자료와 실제 저속 좌회전 347개를 함께 학습한다. 회전 손실 후보의 실패한 가중치가 아니라, 이전 정상 공동 학습 final6000에서 시작한다. 새로운 기능 head·속도 제한·평가 기준은 추가하거나 바꾸지 않는다.

| 매 업데이트의 입력 4개 | 총 입력 노출 | 목적 |
| --- | ---: | --- |
| 실제 경로 복귀 | 20,000 | 좌우 편차에서 직진 복귀 보호 |
| 기존 저속 속도 유지 | 20,000 | 속도·가감속 성능 보호 |
| 기존 직진·회전 | 20,000 | 기존 맵·경로 성능 보호 |
| 새 실제 저속 좌회전 | 20,000 | 시험 속도와 맞는 회전 정답 학습 |

19개 TRAIN 주행 기록·총 6,179개 원 샘플을 보존하고, 사전 조건에 맞는 5,879개가 입력 후보다. 새 좌회전 347개를 약 57~58번 반복 노출하지만 **새로운 347개 경로나 57번의 실제 주행을 뜻하지 않는다.** 회전 정답 22,208점과 영상·시간·현재 속도·경로는 원자료 그대로다. 교사 주행 성공은 모델 기능 성공이 아니다.

데이터·노출 순서·학습 기간을 함께 바꾼 개선 실행이므로, 성공하더라도 한 요인의 효과를 분리한 A/B 결과라고 주장하지 않는다.

## 검사와 다음 순서

1. 완료: 347개 전량 자격 검사 → 학습 형식 변환 → 서버 복사 2,095개 파일 전량 동일성 검사 → 19개 TRAIN CPU 사전검사.
2. 완료: 개인 `py312` venv·물리 GPU0에서 고정 20,000회 학습. 1,000회마다 체크포인트와 전량 업데이트 기록 보존.
3. 학습 후: 최종 모델의 전체 101개 상태·고정 47개 동일성·20,000개 실제 업데이트를 확인한 뒤에만 기존 DEV/HOLDOUT을 연다. 중간 체크포인트를 골라 채택하지 않는다.
4. 완료: 기존 71개 진단을 유지하고 새 좌회전 TRAIN 진단 3개를 추가한 전체 평가. 60/74이며 미채택이다.
5. 완료: 같은 CARLA 좌회전의 두 모델 비교와 전체 실패 꼬리·차체·가감속·100ms 검사. 새 모델의 회전은 완료했지만 정확도는 미달이다. 다음은 동일 가중치의 추종 거리 A/B → 새 우회전 자료를 포함한 균형 학습 → 직진 보호·양방향 회전 및 새로운 경로·날씨·초기 조건이다.

학습·전체 평가는 실제 120.96분 걸렸다. 같은 경로 실제 주행 두 번과 화면 게시에는 약 10~20분을 예상하되 성공을 보장하지 않는다. 현재 메인 0/9·세부 0/30이며, 30km/h와 60km/h는 기본 저속 주행을 확보한 다음 단계다. 크레딧 잔액과 정확한 토큰 비용은 조회할 수 없다.

## 원본과 재현 근거

- [고정 학습 조건·ROOT 실행 권한](../../../../../../storage/experiments/plans/2026-10-02/temporal_left_joint_v1/root_fixed20k_research_fit_authorization.json)
- [최종 전체 평가 원본 — 60/74·미채택](../../../../../../storage/experiments/training/2026-10-02/temporal_left_joint_v1/summary.json)
- [서버 전후·로컬 전량 동일성 근거](../../../../../../storage/experiments/training/2026-10-02/temporal_left_joint_mirror_v1/completed_run_for_root_review.json)
- [ROOT 완료 확인·별도 연구 실행 범위](../../../../../../storage/experiments/plans/2026-10-02/temporal_left_joint_v1/ROOT_completed_run_authority.json)
- [주행 전에 고정한 두 모델의 비교 선언](../../../../../../storage/experiments/validation/2026-10-02/temporal_left_joint_left45_v1/batch_plan.json)
- [실제 서버 CPU 검사·전체 복사 영수증](../../../../../../storage/experiments/training/2026-10-02/temporal_left_joint_dispatch_v1/actual_CPU_and_full_transfer_receipt.json)
- [실제 20,000회 완료·평가 시작 확인 — 14:52](../../../../../../storage/experiments/training/2026-10-02/temporal_left_joint_dispatch_v1/ROOT_actual_progress_1452.json)
- [이전 실제 학습 프로세스·업데이트 확인 — 14:08](../../../../../../storage/experiments/training/2026-10-02/temporal_left_joint_dispatch_v1/ROOT_actual_progress_1408.json)
- [실제 새 자료와 자격 검사](../18_low_speed_turn_data_readiness/README.md)
- [바로 전 실제 우회전 실패 비교](../17_intrinsic_curve_pair_actual_town03_right/README.md)

원격 원본은 `/home/pro6000/personal/hwanhong/portable_e2e/runs/training/temporal_left_joint_20261002_v1`에 보존한다. 설치·재부팅·GPU1·다른 테스트의 시뮬레이터 변경·실차 제어·자동 모델 교체는 하지 않는다.
