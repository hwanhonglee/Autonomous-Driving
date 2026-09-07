# 2026-09-07 Portable E2E 반복 학습·검증

<!-- HH_260906 - Point users to a categorized evidence bundle and keep deployment approval separate. -->

[전체 결과 폴더](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/README.md)에서
학습 A/B, 새 우회전·직진 데이터, 차량 중심 PNG/GIF, 경로 후보 분석, 원격 전송 검증을
카테고리별로 확인할 수 있다.

이번 작업은 기존 VAD를 계속 학습하는 작업이 아니라 **새 Portable physical-v1 모델**의
반복 학습·검증이다. 로컬에서는 CARLA BasicAgent로 정답 데이터를 만들고, Pro6000
개인 venv의 GPU0에서 학습·평가한다. GPU1·시스템 패키지·Conda·타 사용자 작업은
변경하지 않았다.

- 1차 실험: 기존 train613, val337로 학습률 2개 × seed 3개.
- 새 데이터: Town01 우회전 train534 추가, Town04 직진 test309 신규. 총 train1147,
  val337, test309. 기존 train/val은 파일 hash로 불변성을 확인했다.
- 2차 실험: 새 train1147로 동일 1,540 step × seed 3개 완료.
- 3차 실험: 같은 새 데이터에서 후보 선택 손실 가중치만 0.1 → 0.5로 바꿔 seed 3개 완료.
- 후보 선택: 첫 A/B 쌍의 전체 val337와 고정된 여섯 시점에서 경로 다양성·선택 오차를
  따로 계측했다. 특정 후보만 선택하는 것과 후보 경로가 모두 같은 것은 다르다.

**총 12회 학습과 각각의 val 평가·경로 형상 검사, 총 36개 단계를 완료했다.** 마지막
실험은 12:02:16 KST에 끝났으며, 현재 이 캠페인의 학습은 실행 중이 아니다.
학습률 변경과 선택 손실 가중치 변경 모두 세 seed 중 두 개만 상대 개선을 보였다.
새 데이터 실험도 포함해 모든 모델이 절대 경로 품질 목표에는 미달해 주행용 모델을
교체하지 않았다. 가장 좋은 한 번의 결과만 골라 성공으로 표시하지 않는다.

전체 코드 회귀 테스트는 **2,155 passed / 6 skipped**다. 코드 테스트 통과는 학습 모델의
주행 성능 합격과 별개다. 실제 학습 곡선, 12개 모델의 수치, 후보 경로 PNG,
새 expert 주행 PNG/GIF와 전송 검증을 위 결과 폴더에 보존했다.

다음 개선 대상은 후보 선택 목표와 경로 품질의 정렬, 후보 경로 정보를 활용하는 선택
구조, 더 다양한 독립 episode다. 현재 ADE oracle 진단은 speed 등을 포함하는 학습 loss
oracle과 다르므로, 학습 목표의 일치율을 계측한 뒤 한 항목씩 수정·재학습한다.

모델 제어 승인과 30개 기능 완료를 의미하지 않는다. 미사용 test, 기능별 시나리오,
로컬 10 Hz shadow 회귀, learned closed-loop 검증이 아직 필요하다. BasicAgent 수집
영상은 새 모델의 자율주행 성공 증거로 쓰지 않는다.

[두 PC에서 실행 상태 보는 방법과 반복 순서](portable-e2e-learning-loop.md) ·
[전체 기능 정의](portable-e2e-feature-roadmap.md)
