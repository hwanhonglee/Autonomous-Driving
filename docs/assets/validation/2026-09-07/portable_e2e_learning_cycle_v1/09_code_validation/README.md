# 코드 회귀 검증 — 2026-09-07 반복 학습 도구

<!-- HH_260906 - Separate repository regression checks from learned driving quality. -->

로컬에서 저장소의 ROS 환경을 활성화한 뒤 전체 테스트를 실행했다.

```bash
source scripts/e2e/env.sh
python3 -m pytest -q tests
```

결과: **2,155 passed, 6 skipped in 141.11 s**. [원본 실행 출력](pytest_tests_final.log)을
보존했다. 6개 skip을 성공으로 합산하지 않는다. 이 로컬의 오래된 PyTorch에서 지원하지
않는 secure checkpoint API 등을 필요로 하는 테스트는 환경 제약으로 건너뛴다.
원격 실제 학습·평가는 기존 개인 venv의 PyTorch로 별도 수행했다.

대상에는 학습 CLI, 고정된 실험 계획·GPU 범위, 중단 처리, 결과 비교의 분모·hash·완료
검사, 선택 오차 진단, 학습 곡선, 공개 metadata 경로 치환·원본 검증이 포함된다.
코드 테스트 통과는 모델 경로 오차·충돌 회피·30개 주행 기능의 합격이 아니다.
이번 12회 모델 학습의 절대 품질 목표는 모두 미달이며 주행용 교체는 없었다.

환경 설정 없이 처음 실행한 pytest가 ROS 의존 모듈에서 수집을 건너뛴 출력은 private
실패 기록으로 보존했고, 위 전체 테스트 통과 기록으로 대신 계산하지 않았다.
