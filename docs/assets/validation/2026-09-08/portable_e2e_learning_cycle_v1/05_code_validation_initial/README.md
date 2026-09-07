# 05 · 초기 수집 보정·페달 계측 코드 회귀 검사

<!-- HH_260906 - Retain the actual full-suite output without conflating code tests with measured driving quality. -->

결과: **2,633 passed / 6 skipped, 185.38초**. [원본 로그](pytest_full.log)는 private
실행 로그와 바이트가 같습니다. 9월 8일 초기 선택기·정답 진단 도구, comfortable v1/v2
수집기, 소유권 실행 wrapper, 12개 고정 페달 계측 v1과 초기 정지 집계기를 포함했습니다.
이후 추가하는 타력 주행·ramp v2 계측과 페달 결과 집계기의 테스트를 소급 포함하지 않습니다.

```bash
source scripts/e2e/env.sh
export PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES=''
python3 -m pytest -q -rs tests
```

건너뛴 5개는 로컬 PyTorch의 안전한 `weights_only` checkpoint 로딩 API 미지원,
1개는 고정 Woraksan map/route bundle 부재입니다. 성공으로 합산하지 않았으며,
패키지 설치나 안전 옵션 해제로 우회하지 않았습니다.

이번 테스트는 후속 커밋에 담는 검토된 작업 트리에서 실행했습니다. 실제 12개 페달 계측의
소스 SHA는 `e5f32d74fd68c52b601230bb5a907b471acee9c1cfc3088b5d483f01421ec38d`이며,
해당 실행의 private owner 자료에 실행 전 원본 소스와 종료 후 불변성 증거가 있습니다.

로그 SHA-256: `ccedc944cf32fa3cafee00e1827a59fadd868809d2803d71fec4ff74666c9f42`.
코드 검사 통과가 수집 데이터 품질이나 학습 모델 주행의 합격을 뜻하지 않습니다.
