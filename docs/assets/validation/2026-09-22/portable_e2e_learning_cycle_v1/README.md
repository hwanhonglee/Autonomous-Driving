# 2026-09-22 — 전체 모델 성능 복구 학습·검증

<!-- HH_260906 - Link the latest iteration before preserving the historical K/L snapshot. -->
최신: [루트 구조 정리·주행 AF·지도 클래스 균형 학습](24_root_cleanup_and_controlled_learning/README.md).
아래 완료 횟수는 K/L 단계 당시 기록이며 이후 학습을 합산한 최신 총계가 아닙니다.

<!-- HH_260906 - Publish the complete K learning cycle without promoting a regressed model. -->

- [K0/K1 전체 학습·최종 평가·원인 분석·경로 PNG](01_full_model_recovery_k/README.md)
- [TRAIN 전수 진단·L 구조 비교 설계](02_train_selection_and_geometry_recovery/README.md)
- [L 전체 학습 결과·비교 PNG·다음 작업](03_geometry_recovery_l_results/README.md)
- [M 우회전 수집 6회·487개 후보·물리 라벨 검사](04_right_diversity_m/README.md)
- [실행 위치와 시간 기록](../../../../portable-e2e-k-recovery-execution-2026-09-22.md)
- [전체 9영역·30기능과 학습 계획](../../../../portable-e2e-status-and-training-plan-2026-09-21.md)

실제 완료: K0/K1/L 전체 모델 학습 3회·총 45,000 step, 최종 DEV 665개 × 3회,
K 사후 DEV 후보 선택 진단 2회와 TRAIN 전수 진단 2회.
결론: 직진 개선에도 우회전 문제가 남아 세 모델 모두 미채택. 자체 모델의 폐루프 승인 기능 0/30.

후속 M: 로컬 전문가 수집 6회, 지면 생성 수정 후 Common10 487개 검증 통과.
다만 실제 타깃의 기존 런타임 검사에서 221개가 실패해 학습 편입을 보류했다. M 새 학습은 0회다.
