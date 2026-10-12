# 자체 주행·지도 모델 연구

<!-- HH_260906 - Keep the learning core separate from ROS workspace and large data. -->

- `portable_e2e/`: 주행 궤적 모델, 지도 RGB 인식 모델, 손실·데이터·추론 코드.
- `config/`: 기능 요구사항과 연구 실험 설정.
- `tests/`: 코드 회귀 검사. 자율주행 기능 승인 개수와 다름.

로컬 기본 검사 예: `python3 -m pytest -q learning/tests/test_map_perception.py`.
루트의 옛 Python 패키지 이름은 호환 링크로 유지 중입니다. 링크 제거 전 학습 패키지 경로와
도구 import를 모두 이전해야 합니다. 원격은 개인 py312 venv/GPU 0 제약을 유지합니다.

[모델 안내](portable_e2e/README.md) · [전체 안내](../WORKSPACE.md)
