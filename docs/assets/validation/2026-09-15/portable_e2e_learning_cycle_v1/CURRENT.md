# 9월 15일 — 최신 실행 결과 목록

<!-- HH_260906 - Add a current index without rewriting the previous sealed stage or its day manifest. -->

**03 전체 모델 이어학습**과 **04·05 신규 주행 데이터 확보·변환·원격 수신**을 완료했다.
PRO6000 GPU 0에서 19:29:38–21:30:27 KST에 6개 모델 모두 20 epoch까지 실제 학습하고,
매 epoch 개발 VAL·최종 행동 평가를 마쳤다. 추가 25,200 step·100,710회 표본 노출이다.
로컬 수집은 20:16 KST, 새 TRAIN 328개 원격 수신·검증은 20:48 KST에 완료했다.

같은 VAL에서 6.4초 경로 평균 오차의 세 seed 평균은 A **22.2%**, B **27.7%** 감소했다.
그러나 B/seed03의 장기 오차는 악화했고, 네 가지 경로 목표를 모두 만족한 모델은 **0/6**이다.
**실제 학습 완료와 모델 채택·직접 주행 성공은 다르며, 새 모델은 채택하지 않았다.**

| 순서 | 결과 | 화면 |
| --- | --- | --- |
| [05 — 신규 TRAIN 데이터 변환·서버 수신](05_moving_train_dataset/README.md) | 실제 TRAIN 328개·영상 1,968장; 로컬/원격 파일·planning·loader 검증 통과; 아직 학습 미사용 | 04의 실제 원본 화면에 연결 |
| [04 — Town07 신규 이동 데이터·실제 촬영](04_moving_data_pilot/README.md) | 328개 사전 선택 anchor·32.7초, 9개 필요조건 위반 0; 당시 원본 진단 그대로 보존, 후속 변환은 05 | 차량 중심 6-camera PNG 6장·실제 기록 요약 GIF |
| [03 — 전체 모델 이어학습·학습 곡선](03_duration_learning/README.md) | 6개 모델 모두 20 epoch 완료·독립 검산 통과; 미채택 | 학습 곡선 4장·고정 offline 경로 144장 |
| [02 — 저속 종점 접근 제어 A/B](02_terminal_approach_feedback/README.md) | 고정 10 ms에서 두 회차 정지 완료; B 품질 악화·학습 미편입 | PNG 14장·GIF 2개 |
| [01 — 원시 경계 분석·10 ms/5 ms 비교](01_native_boundary_and_substeps/README.md) | A1 완료, 5 ms B1 목표 밖 정지·실패; B2/A2 미실행 | PNG 11장·GIF 2개 |

기존 [당시 일일 요약](README.md)과 체크섬은 당시 상태 그대로 보존한다.
새 02·03/results·04·05 폴더는 자체 `SHA256SUMS`로 검증하며 이전 일일 체크섬의 새 항목인 것처럼 주장하지 않는다.
02 단계의 당시 회귀는 **1,300 passed / 1 skipped**, 이번 03–05 관련 최종 회귀는 **257 passed**다.
자체 학습 모델의 직접 운전 검증 통과는 **0/30**이다.

04 화면은 CARLA BasicAgent 전문가가 운전한 기록이다. Portable 모델·VAD·Autoware가
운전한 화면으로 표시하지 않는다. 신규 데이터는 이번 03의 고정 학습 데이터에 섞지 않았다.
