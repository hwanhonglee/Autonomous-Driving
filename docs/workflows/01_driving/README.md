# 01 — 주행 모델

<!-- HH_260906 - Expose driving entrypoints without moving model imports or frozen training dependencies. -->

목표: 카메라·차량 상태·경로로 미래 궤적을 출력하고 Autoware 또는 별도 제어기와 연결.
현재 우선순위는 직진 보존과 우회전 개선입니다. 승인된 자체 모델 폐루프 기능은 0/30입니다.

- [모델 원본](../../../learning/portable_e2e/model.py)
- [AE/AF/AG 학습 실행 코드 — 원격 전용](../../../tools/e2e/run_recovery_ae.py)
- [AE 추가 손실](../../../learning/portable_e2e/recovery_objective.py)
- [9개 영역·30개 기능 원장](../../../learning/config/portable_e2e_feature_matrix.yaml)
- [최신 후속 — 출발·정지 제어 AJ, 재학습 미실행](../../assets/validation/2026-09-27/drive_control_aj/README.md)
- [완료된 AH 주행/지도 개선 학습](../../assets/validation/2026-09-26/dual_model_recovery_ah/README.md)
- [이전 AG 실패 결과](../../assets/validation/2026-09-22/portable_e2e_learning_cycle_v1/27_driving_continuation_ag/README.md)

조회: 저장소 루트에서 `python3 tools/workspace.py status --remote`.
학습 재시작은 새 출력 경로·GPU 잠금·데이터 해시 확인이 필요합니다. 진행 중인 학습을 중복 실행하지 마세요.
