# GPU 0 학습 검사 비용 — 합성 미세 벤치마크

<!-- HH_260906 - Measure one safety-check component without changing production guards or inferring end-to-end training throughput. -->

02:45:22–02:45:23 KST에 기존 개인 venv·GPU 0에서 실제 측정했습니다.
모델 학습, checkpoint 로딩, 데이터 읽기나 배포는 하지 않았습니다. 앞선 6회 학습이 종료되고
GPU 잠금이 반환된 뒤 독립 실행했으며, 실제 학습의 안전 검사 코드는 변경하지 않았습니다.

| 방법 | 반복 수 | 검사 함수 평균 | 중앙값 | p95 |
|---|---:|---:|---:|---:|
| 기존 텐서별 결과 읽기 | 40 | 4.102 ms | 4.121 ms | 4.145 ms |
| 장치별 결과 묶기 | 40 | 3.031 ms | 3.029 ms | 3.056 ms |

차이는 이 **검사 함수에서 평균 약 1.07 ms**입니다. 전체 학습이 같은 비율로 빨라진다는
뜻도, GPU 사용률이 낮은 주원인이 확정됐다는 뜻도 아닙니다. 별도 데이터 JPEG 읽기·해시·
디코딩·전송과 forward/backward, 다른 `.item()` 호출은 측정 범위 밖입니다.

실제 physical-v1의 47개 파라미터 모양·954,590개 원소와 Adam 형태의 두 GPU moment,
47개 CPU step scalar를 **합성값**으로 만들었습니다. 모델 검사와 optimizer 검사의 두 호출
경계를 유지하고, 매번 모든 float/complex 텐서를 검사했습니다. 코드상 scalar 읽기 지점은
188개(141 GPU·47 CPU)에서 3개(2 GPU·1 CPU)로 바뀌지만 이것은 실제 CUDA fence 횟수 계측이 아닙니다.
NaN·Inf·복소수·정수·빈 구조 등 12개 판정 대조도 모두 일치했습니다.

각 방법 5회 준비 후 ABBA 순서 20묶음, 총 80회를 측정했습니다. 입력·소스의 전후 해시가
동일하며 원래 측정 결과·실행 worker `4fcdbaff…`를 그대로 보존했습니다.

실행 후 검토에서, 유효한 JSON 줄 전체가 누락되는 경우 최초 worker가 완료를 잘못 표시할 수
있는 방어 누락을 발견했습니다. **이번 실제 80줄은 순서·수·유한 시간·통계·해시를 독립 재검증해
문제가 없음을 확인했습니다.** 이후 코드에 완전한 journal 검사를 추가했으며, 과거 실행을
수정된 코드로 실행했다고 바꿔 쓰지 않았습니다. 새 코드 CPU 테스트는 69개를 통과했습니다.

[실제 원본 결과](evidence/raw/summary.json) · [측정 80개](evidence/raw/measurements.jsonl) ·
[사후 독립 검사](evidence/independent_journal_audit.json) · [실행한 원본 코드](evidence/source/executed_worker.py) ·
[실행 전 계획](evidence/prospective_launch_plan.json) · [SHA-256](evidence/SHA256SUMS).

현재는 이 결과만으로 학습 루프를 바꾸지 않습니다. 학습 속도 개선은 검사 항목을 제거하지 않고
전체 입력·학습 경로에서 동일 결과와 시간을 비교한 후 별도로 판단합니다.

## 이후 별도로 실행한 전체 경로 진단

<!-- HH_260906 - Separate the earlier synthetic component timings from later native-trainer measurements. -->

- [원본 trainer의 16-step 계측 비교](actual_training_profile/README.md): 계측 유무 2회,
  모든 32개 학습 기록과 선언한 생성 시각 외 checkpoint 내용의 동일성을 확인했습니다.
- [DataLoader worker 0→2→2→0 실제 비교](dataloader_abba/README.md): 각각 새 초기화에서
  TRAIN 1,147개를 한 번 순회한 287-step 진단 4회입니다. 모든 학습 기록이 바이트 동일했고
  조건당 두 번의 평균 학습 구간은 34.631→22.936초였습니다. 장기 캠페인·추론 FPS 개선
  증거가 아니며 production 설정은 그대로입니다.
