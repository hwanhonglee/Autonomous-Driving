# DataLoader 0명/2명 · 고정 ABBA 실제 계측

<!-- HH_260906 - Four one-epoch diagnostic fits are not four full training campaigns or an adopted performance change. -->

같은 physical-v1 모델·TRAIN 1,147개·seed·loss·학습 순서를 유지하고 DataLoader CPU worker 수만 0→2→2→0으로 비교했습니다. 각자 새 프로세스·새 초기화에서 287 update(1 epoch)를 실행한 진단 4회이며, 장기 학습 캠페인 4개가 아닙니다.

| 순서 | CPU workers | 학습 함수 시간 s | 전체 fit 프로세스 시간 s |
|---|---:|---:|---:|
| A1_workers0 | 0 | 34.043344 | 45.087045 |
| B1_workers2 | 2 | 23.159783 | 34.123091 |
| B2_workers2 | 2 | 22.712539 | 33.520451 |
| A2_workers0 | 0 | 35.218968 | 46.089320 |

이 진단에서 관찰한 평균: 학습 함수 34.631156→22.936161s (33.77% 감소), 전체 fit 프로세스 45.588182→33.821771s (25.81% 감소). 두 조건 각각 2회뿐인 고정 순서 반복입니다. 일반 서버 성능·장기 학습 시간·추론 FPS 개선을 증명하지 않습니다.

학습 함수 시간에는 worker 생성, 데이터 읽기·전처리, forward/backward/update, checkpoint 저장과 최종 GPU 동기화가 포함됩니다. 전체 fit 프로세스는 여기에 Python/CUDA 시작과 각 fit 전후 corpus 무결성 검사 등을 포함합니다. 전체 ABBA 감독 프로세스의 시간과는 다르며, 디스크/page cache·spawn 비용을 제거한 순수 연산 benchmark가 아닙니다.

4개 metrics.jsonl은 각 287행, 합계 1,148행으로 원본 바이트가 모두 같습니다. 노출은 1,147×4=4,588회이며 고유 데이터가 늘어난 것은 아닙니다. 원격 실행은 checkpoint를 안전한 CPU loader로 비교하여 `created_at_utc`, `train_config.num_workers` 두 필드만 제외하고 일치했다고 기록했습니다. 공개 helper는 checkpoint tensor를 다운로드하거나 다시 읽지 않았고, 원격 검사 기록과 SHA를 대조했습니다.

TRAIN만 신경망에 사용했습니다. 기존 전체 corpus 무결성 검사는 TEST metadata/JPEG를 읽을 수 있지만 VAL/TEST 추론·loss·모델 선택은 하지 않았습니다. 실제 GPU 이용 기록과 worker의 CUDA 비노출 시작 증거를 보존하되 장치 UUID·개인 경로는 공개 view에서 치환합니다. 데이터 승인·모델 승격·프로덕션 worker 설정 변경은 없습니다.

- [실측 시간 그림](01_measured_abba_timing.png)
- [재계산한 시간·분모](timing_summary.json)
- [원격 완료 보고서 view](metadata/report.json)
- [사전 고정 계획 view](metadata/plan.json)
- [수집·원격 SHA 검증 receipt view](metadata/transport_verification.json)
- [원본→공개 SHA 및 제외 항목](publication_manifest.json)

각 arm 폴더의 전체 metrics.jsonl은 원본 바이트입니다. fit/run report와 source_views는 명시적인 metadata/source text view이며 실행용 원본이 아닙니다. 원본 44파일 중 checkpoint 4개는 원격에 유지했고, 로컬 수집 40파일의 비어 있는 lock/log 8개는 중복 공개하지 않습니다. 원본은 삭제하지 않았습니다. 재현에는 원본 private 자료와 고정 실행 소스가 필요하며 git clone만으로 데이터·실행 승인이 생기지는 않습니다.
