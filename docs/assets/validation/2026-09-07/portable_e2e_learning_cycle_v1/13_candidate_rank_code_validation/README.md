# 후보 경로 인지 모델·진단 도구 — 전체 코드 회귀 검증

<!-- HH_260906 - Preserve the full regression output and distinguish code correctness from model quality. -->

로컬 저장소에서 다음과 같이 전체 테스트를 실행했다. 이번 변경의 모델·실험 runner·
엄격한 집계·학습 곡선·원본 검증·공개 자료 생성 코드가 포함된 상태다.

```bash
source scripts/e2e/env.sh
python3 -m pytest -q -rs tests
```

결과: **2,333 passed, 6 skipped in 145.37 s**. [원본 출력](pytest_full.log)은 private
실행 로그와 bytes가 같다. 모델 구현은 `5c50353480a51f11b970c7d0b9ec8dcd91f7b056`이며,
진단·집계·테스트 도구는 이 문서와 함께 발행하는 후속 커밋의 작업 트리에서 검증했다.

건너뛴 6개를 성공으로 합산하지 않는다.

- 5개: 로컬의 오래된 PyTorch가 secure `weights_only` checkpoint 읽기를 지원하지 않음.
- 1개: 고정된 Woraksan route/map bundle이 이 환경에 없음.

정확한 테스트 이름과 사유는 로그 마지막에 있다. 보안 옵션을 풀거나 패키지를 설치해
우회하지 않았다. 원격의 기존 개인 venv에서는 실제 E 모델 3개의 학습·평가·감사를
별도로 완료했다. 코드 테스트 통과는 모델 성능 합격이 아니며, E의 상대·절대 품질은
모두 FAIL로 운용 모델을 교체하지 않았다.

원본 출력 SHA-256: `1d59fcb504f2d37d4da99d058a9a84f5871debb81e0ae350a0a91c03d0bb0173`.
이전 2,155 passed 기록은 [09의 과거 실행](../09_code_validation/README.md)에 그대로 보존한다.
