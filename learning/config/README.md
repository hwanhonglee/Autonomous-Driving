# 연구 설정 — 기능 목표와 실험 조건

<!-- HH_260906 - Distinguish research configuration from ROS launch and runtime model contracts. -->

- [portable_e2e_feature_matrix.yaml](portable_e2e_feature_matrix.yaml): 전체 기능 요구사항·검증 기준. 구현/승인 개수와 같지 않습니다.
- `portable_e2e_*202609*.json`: 날짜별 연구 계획·학습/샘플링 비교 조건. 과거 파일을 최신 기본 설정으로 간주하지 마세요.
- 실제 Autoware 노드 설정: [autoware_e2e_vad_launch/config/](../../autonomy/autoware_e2e_vad_launch/config).
- 자체 모델 내부 설정: [portable_e2e/config/](../portable_e2e/config).

과거 연구 설정은 재현에 필요하므로 날짜·파일명을 유지합니다. 현재 실행 조건은 최신 실험의
`declaration.json`, `started.json`, 학습 데이터 manifest가 기준입니다.
