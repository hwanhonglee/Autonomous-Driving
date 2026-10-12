# 02 — 자동 지도·도로 인식

<!-- HH_260906 - Separate RGB perception training from oracle-sensor mapping and map fusion. -->

1. RGB 인식: [모델](../../../learning/portable_e2e/map_perception.py) · [원격 학습](../../../tools/e2e/train_map_perception_pilot.py)
2. 센서 수집: [CARLA 수집기](../../../tools/e2e/collect_carla_mapping_probe.py)
3. 학습 자료: [데이터 준비](../../../tools/e2e/prepare_map_perception_pilot.py)
4. 지도 누적: [공통 live/replay](../../../learning/portable_e2e/mapping.py) · [좌표 투영](../../../learning/portable_e2e/mapping_projection.py)
5. 자료 생성: [센서 지도 시각화](../../../tools/e2e/render_mapping_probe.py)

[실제 센서 시험 결과](../../assets/validation/2026-09-22/portable_e2e_learning_cycle_v1/22_mapping_live_replay/README.md)

[현재 개선 학습 — 원본 해상도·확대 증강 AH](../../assets/validation/2026-09-26/dual_model_recovery_ah/README.md)

[이전 학습 비교 — 클래스 균형 조정 실패 결과](../../assets/validation/2026-09-22/portable_e2e_learning_cycle_v1/24_root_cleanup_and_controlled_learning/README.md)

지도 센서 그림은 정답 depth/분류/pose 기반이고, RGB 학습 모델 예측은 별도 결과입니다.
픽셀 분류 성공을 신호 해석·차선 연결·완성된 정밀지도 승인으로 해석하지 않습니다.
