# CPU 최종 전방계산 검증과 초기값 검사 미완료 기록

<!-- HH_260906 - Distinguish final arithmetic verification from unchanged initial disagreement, optimization and data admission. -->

기존 GPU 수치 근사의 **저장된 최종 잠재값**을 두 CPU 환경에서 동일 물리 decoder에 한 번씩 넣었습니다. 1,337개 시작점 × 6후보 × 64점의 X·Y·속도 1,540,224개 값을 각각 비교했습니다. 최적화·학습·GPU 실행·게이트 변경 없이 `no_grad` 전방계산만 했습니다.

| CPU 검사 | Torch | 비교 스칼라 수 | 허용량 밖 | 최대 X 차이 (m) | 최대 Y 차이 (m) | 최대 속도 차이 (m/s) |
|---|---|---:|---:|---:|---:|---:|
| 로컬 | 1.8.0a0 | 1,540,224 | 0 | 2.28881836e-05 | 1.43051147e-06 | 1.43051147e-06 |
| Pro6000 개인 venv | 2.13.0+cu130 | 1,540,224 | 0 | 2.28881836e-05 | 1.43051147e-06 | 1.43051147e-06 |

허용량은 처음과 같은 절대·상대 `1e-5`입니다. 판정식은 `|CPU-GPU| <= max(1e-5, 1e-5*max(|CPU|,|GPU|))`입니다. 따라서 최대 X 절대 차이가 1e-5 m보다 커도 상대 오차 조건으로 통과할 수 있으며, 모든 절대 차이가 1e-5 이하라는 뜻은 아닙니다. 이는 저장된 수치 결과의 전방계산 일치이지 원본 정답 오차·자율주행 성능·데이터 승인 PASS가 아닙니다. 원래 출력 게이트 실패 34개와 원본 곡률 실패 시작점 299개는 그대로 보존했습니다.

두 CPU 실행의 최적화 잠재값 해시는 6개 배치 모두 같습니다. CPU 출력 바이트까지 동일한 배치는 XY 0/6, 속도 0/6이므로 허용량 내 일치와 bit-identical은 구분합니다.

## 초기값 검사는 여전히 별개로 확인 미완료

같은 Torch 2.13.0+cu130의 원격 CPU 초기값 재감사에서도 22개 초기 목적함수 차이가 남았습니다. 최대 CPU–GPU 차이는 8.25237248e-05 m²이며 기존 로컬 검사와 동일한 후보 인덱스입니다. 전체 초기 목적함수의 두 CPU 환경 사이 최대 차이는 2.26108406e-05 m²입니다. Torch 버전을 맞춰도 차이가 없어지지 않았으므로 버전 차이만을 원인이라고 할 수 없습니다. CPU/GPU 연산 차이의 구체적 원인은 아직 확정하지 않았습니다.

이 원격 초기값 CLI는 수치 검사 JSON 3개를 쓴 뒤 PNG 생성에서 `ModuleNotFoundError: No module named 'matplotlib'`로 종료 코드 1을 반환했습니다. 패키지를 설치하거나 재시도하지 않았고, 원격 README·PNG·정상 SHA256SUMS는 생성되지 않았습니다. [부분 실행 기록](03_remote_cpu_initial_partial/EXECUTION_STATUS.md)과 JSON 3개를 원본 바이트 그대로 보관했습니다. 이 폴더의 새 공개 해시 목록은 전송된 부분 파일을 검증할 뿐, 과거 CLI를 완료로 바꾸지 않습니다.

## 자료

[로컬 최종 전방계산](01_local_cpu_final_forward/summary.json), [원격 최종 전방계산](02_remote_cpu_final_forward/summary.json), [환경 비교](comparison.json), [복사 원본 해시](publication_manifest.json), [전체 SHA256SUMS](SHA256SUMS). 각 최종 검사 폴더의 JSONL에는 모든 후보 비교 수와 최대 차이·불일치 목록이 있습니다. 초기 GPU 최적화와 기존 실패 설명은 [변경하지 않은 category17](../17_decoder_representability/README.md)을 참조합니다. 새 최종 전방계산 코드 SHA와 실행 날짜는 각 JSON에 기록했습니다. 원본 학습 데이터나 모델 체크포인트를 만들거나 변경하지 않았습니다.
