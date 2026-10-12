# 9월 15일 — 출발·제동 경계 분석과 물리 계산 간격 비교

<!-- HH_260906 - Keep the new numerical-sensitivity work separate from the sealed September 14 evidence and from model training. -->

이번 작업은 로컬 CARLA의 **데이터 수집 품질 진단**이다. 학습 모델 직접 운전이나
Autoware UI 실행, 서버 GPU 학습과 구분한다. 이전 날짜의 결과·실패 기록은 수정하지 않는다.

| 분류 | 내용 | 상태 |
| --- | --- | --- |
| 01 경계·물리 설정 | [원시 위치/속도·곡률 재현 및 10 ms/5 ms 비교](01_native_boundary_and_substeps/README.md) | 이전 4회 원시 재현 완료; 새 A1 완료·B1 감속 timeout·2회 미실행; PNG 11장·GIF 2개 |

기존 카메라 10 Hz·물리 상태/제어 20 Hz와 즉시 정지 제동·저속 조향 0을 고정하고,
명시적인 최대 물리 계산 간격 설정만 10 ms/5 ms로 비교한다. 설정값은 내부 실제 계산 시각을
직접 계측한 값이 아니다. 원래 정답·정지 기준·학습 편입 기준을 바꾸지 않는다.

5 ms 회차는 정상 정지 제동 전에 목표 약 1.75 m 앞에서 정지했고 기존 45초 감속 제한으로 종료됐다.
포트/서버 연결 실패가 아니며, 사전 선언한 운영 실패 시 중단 정책에 따라 B2/A2는 실행하지 않았다.
수치상 곡률 실패 감소를 성공으로 판정하지 않는다. 새 자료는 학습 미편입이고 새 원격 학습도 없다.
관련 전체 회귀는 **1,102 passed / 1 skipped**다.

[이전 9월 14일 전체 결과](../../2026-09-14/portable_e2e_learning_cycle_v1/README.md)
