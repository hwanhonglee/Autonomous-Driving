# 학습 예측 → 위치 누적 지도 배선: 실행 전 계약

<!-- HH_260929 - Freeze a three-frame CPU-only learned-projection demonstration without HD-map or generalization claims. -->

이 실험은 RGB 의미분할의 **학습된 예측**을 깊이·위치와 결합해 기존 `SemanticEvidenceMap`에 넣는 최소 배선 검사다. 도로/차선 검출 품질, 미사용 DEV, 실시간 10Hz, 지도 연결, 신호 규칙, Autoware 주행을 검증하는 실험이 아니다. 입력 3장은 Town01의 기존 BasicAgent 수집 중 원본 순서 **0000/0090/0179**로, 모델/시각 점수를 보고 고르지 않았다. 연속 영상 재생도 아니다.

- 기존 AI v2 지도 checkpoint SHA-256 `ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380`, 원본 RGB 640×360·FOV90·CARLA depth/semantic·`world_from_camera`를 사용한다. [원본 수집 요약](../../../../../storage/experiments/validation/2026-09-22/mapping_sensor_probe/moving_straight_v1/mapping/summary.json) SHA-256 `95e30aa5b121d668c03bae2863f7208b961db13caea44724c66ba76855d3591e`, 기록 SHA-256 `51ffdcd08f8aa89d95c64bef664b352419db1e35437ed8a8079eb11336811a65`다.
- 세 raw NPZ SHA-256은 순서대로 `dd1a1004b22e8ee03c2e5a2672bbd99846f6bf1a2afe4098d36af3e9371873b8`, `21a897942101c4fbe6fd0224ae0dd3b7497deeaa36b3532ee366bc693f36602e`, `19a556be9a39bb1e275c96a4bf623a9a43a0c79241624f4aef3fcbe93575dd69`이다. RGB PNG SHA-256은 `059daf073396ec7c1a2f6035e0d3ecb65091ba1dfd9475683d139ae24969b9ea`, `94bcc362a3736b60e8511118eee26df6b1fabae2d449be56b85f90893606f00b`, `fbcc52650c26144d5881226bf024664010c15938f3eab591f7d840ba2fed67ec`다.
- [투영 함수](../../../../../learning/portable_e2e/mapping_projection.py) SHA-256 `c42ab7d20454475f544b8073d6a23a104a0a2bb1cc2cbb7be092fa3e871cbbee`, 기존 [의미 증거 누적기](../../../../../learning/portable_e2e/mapping.py) `264f9dda759f6986bde9c614e5cf3ae21680a22ae58d855db11564ceb3b3b867`, 모델 구조 [소스](../../../../../learning/portable_e2e/map_perception.py) `1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b`다. 실행 코드를 작성·검사한 뒤 코드 SHA를 별도로 기록한다.

실행은 **로컬 CPU만** 쓰고 GPU·CARLA 서버·Autoware·SSH·설치를 쓰지 않는다. `torch.softmax` 최대 확률을 클래스 confidence로 사용한다. 각 프레임에서 stride3, 광축 깊이 `0.1–40m`, confidence `≥0.5`인 road/sidewalk/lane/light fixture/sign/pole만 0.2m 격자에 누적한다. `other`는 투영하지 않는다. 비교 기준은 같은 픽셀/깊이/pose에 CARLA semantic을 적용한 oracle 센서 배선이며, learned 경로의 추론에는 oracle semantic을 공급하지 않는다. 깊이와 pose는 **CARLA oracle**인 것을 모든 산출물에 표시해야 한다. 출력은 별도 새 폴더의 3프레임 별 7클래스 픽셀 비교·누적 cell 수·차량 중심 PNG와, 예측/oracle provenance 분리 결과다. 세 프레임 간에는 건너뛴 177장을 관측한 척하지 않는다.

성공 관문은 파일 출처·투영 축/확률·source 분리·재생 결과의 재현뿐이다. IoU·관측 cell 수가 좋아도 HD map 승격은 없다. 기본 지도 후보 AI v2를 변경하거나 새 학습하지 않는다.
