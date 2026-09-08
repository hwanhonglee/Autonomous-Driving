# 실제 결과 — 6개 기존 모델의 고정 10 epoch 이어학습

<!-- HH_260906 - Report all fixed endpoints, inherited updates and short-horizon regressions without promotion. -->

24/24 단계와 6/6 계보가 완료되었습니다. 상대조건은 3/6, 절대조건은 0/6 통과입니다. 자동 승격·새 데이터 승인·차량 제어 승인은 없습니다.

새 모델6개를 처음부터 학습한 것이 아닙니다. 각 부모1540 step/6155회 노출에서1330 step/5315회를 추가해 총2870 step/11470회, 정확히10 epoch까지 이어갔습니다. 데이터는 기존 train1147개·validation337개 그대로입니다.

공개 이력은 총17,220행입니다. 이 중9,240행은 복사한 부모 이력이고, 7,980행만 이번 새 update입니다. 행동 기록은6×337=2,022행 전부입니다.

중요: 1초 ADE는 6/6, 3초 ADE는 6/6 계보에서 오히려 증가했습니다. 일부 전체6.4초 ADE가 개선되어도 짧은 구간의 성능 향상을 의미하지 않습니다.

| 계보 | 6.4초 ADE 이전→현재 m | 1초 ADE 이전→현재 m | 3초 ADE 이전→현재 m | 기하 PASS /337 | 상대조건 |
| --- | ---: | ---: | ---: | ---: | --- |
| [seed_20260903/A_physical_drive](campaign/seed_20260903/A_physical_drive/behavior/README.md) | 3.8743→4.4820 | 0.2164→0.2649 | 1.2099→1.5973 | 336→336 | False |
| [seed_20260903/B_drive_stop_mix](campaign/seed_20260903/B_drive_stop_mix/behavior/README.md) | 6.2177→5.4057 | 0.3129→0.3497 | 1.8250→1.9241 | 336→336 | True |
| [seed_20260904/A_physical_drive](campaign/seed_20260904/A_physical_drive/behavior/README.md) | 6.2585→6.4521 | 0.3048→0.3490 | 2.1356→2.2458 | 336→336 | False |
| [seed_20260904/B_drive_stop_mix](campaign/seed_20260904/B_drive_stop_mix/behavior/README.md) | 5.1648→5.2058 | 0.2103→0.3277 | 1.5363→1.9295 | 336→336 | False |
| [seed_20260905/A_physical_drive](campaign/seed_20260905/A_physical_drive/behavior/README.md) | 4.7895→4.4399 | 0.2251→0.2586 | 1.3741→1.5836 | 336→336 | True |
| [seed_20260905/B_drive_stop_mix](campaign/seed_20260905/B_drive_stop_mix/behavior/README.md) | 4.9157→4.8423 | 0.1960→0.2772 | 1.4222→1.7665 | 336→336 | True |

## 실제 저장 수치와 그림

![전체 학습 이력](plots/01_continued_learning_curves.png)

회색은 부모1–1540 update, 이후가 이번 추가 학습입니다. 전체 batch 점을 보존했고50-update 평균은 샘플 수로 가중했습니다. 1501–1550 bin은 부모40개와 새10개가 섞이며 마지막 bin은20개입니다.

![고정 종점 validation 비교](plots/02_fixed_endpoint_validation.png)

[엄격한24단계 감사](audit/summary.json) · [상태와 모든 명령](campaign/status.json) · [전송 검증](receipts/collection.json) · [공개파일/원본SHA 대응](publication_manifest.json) · [공개SHA256SUMS](SHA256SUMS)

[원래1540-step 결과](../../04_stopmix_learning/README.md) · [이번 사전 계획](../plan.json)

## 보존·비공개·안전 경계

- 6개 계보마다 사전 고정12장, 총72개의 실제 저장 행동 PNG를 모두 표시합니다. 이것은 오프라인 예측이며 자율주행 촬영이나 실차 검증이 아닙니다.
- 별도 수치 그래프2장도 실제 저장 지표에서 생성한 원본 PNG입니다. 초기12개만 고른 일반 evaluation PNG72장은 private 원본에 그대로 보존하고 공개 갤러리에서는 중복을 제외했습니다.
- 원격232개 파일은 전후 해시가 같았고, 그중checkpoint6개를 제외한226개·21,723,125byte를 로컬에서 확인했습니다. checkpoint는 원격SHA 영수증만 제공하며 로컬 복사·tensor load를 하지 않았습니다.
- 모든 stage log/metric, 원격 실행 worker, 원본 체크섬 참조를 포함합니다. `*.private_reference.txt`는 private 원본용 체크섬이며 공개파일 검증에는 이 폴더 최상위SHA256SUMS를 사용하세요. 0-byte run-lock6개도 감사용 원본 inventory로 보존했습니다.
- JSON은 계정경로·GPU UUID를 가린 metadata view이며 원본 SHA를 별도 기록합니다. worker의 redacted reference는 실행 가능한 원본과 동일한 코드가 아닙니다. 원본 실행 source b478, worker e51의 SHA와 공개 source를 혼동하지 마세요.
- 기존 모델·loss·학습률·sampling·gate는 그대로입니다. 후보 수/모델 용량이 다른 A6와B12의 전체 loss를 같은 난이도의 절대 점수로 해석하지 않습니다. 더 오래 학습한 결과만 비교하며 최적 epoch/seed를 고르지 않았습니다.
- 기존 v3의 label/저속 문제는 해결되거나 승인된 것이 아닙니다. STOP 후보 학습은 실제 정지 의도·차선변경·장애물 회피 능력 검증이 아닙니다. 새 C-track 원본8회도 학습 데이터로 승인하지 않았습니다.
- test 신경망 추론·최적화·모델 선택은 하지 않았습니다. 변함없는 전체 corpus 무결성 검사에서 test metadata/JPEG를 읽을 수는 있습니다.

첫 공개 생성은 최종 SHA 파일을 만들기 전 README 링크를 검사하는 순서 문제로 실패했습니다. [실패 기록](receipts/publication_attempt_001_failure.json)과 원래 생성물171개는 private에 보존했고, 검증 순서만 수정했습니다. 재학습·재추론·이미지 재생성은 하지 않았습니다.
