# Portable E2E — 9월 15일 저속 종점 접근 후속 현황

<!-- HH_260906 - Report completed local diagnostics separately from worsened data quality and unchanged remote training. -->

**저속 종점 접근 제어를 구현하고 실제 A/B 수집까지 완료했다. 후보 B는 궤적 품질 악화로 미채택이다.**

| 환경 | 이번 작업 | 결과·다음 순서 |
| --- | --- | --- |
| 로컬 | 고정 10 ms·20 Hz 상태/제어·10 Hz 6카메라에서 기존 coast A와 terminal feedback B 실제 수집 | 두 회차 모두 정지·유지·후속 관측 완료; B는 곡률·횡가속도 기준 실패 증가로 학습 미편입 |
| PRO6000 | 이번에는 SSH·학습·설치 없음 | 데이터 품질·독립 TRAIN/VAL 검증 후 GPU 0·hwanhong 개인 venv에서 다음 학습 |

A/B 곡률 실패는 **115/671 → 201/691개**, B의 새로운 XY 기반 횡가속도 기준 실패는 **64개**다.
최종 목표 거리는 **0.996705 → 0.293478 m**로 가까워졌지만, 이것만으로 개선 성공이라 판단하지 않는다.
두 회차 통신·제어·카메라·원시 속도 검사 통과와 미래 궤적 품질 실패를 분리해 기록했다.

[전체 분석·차량 중심 PNG 14장·GIF 2개](assets/validation/2026-09-15/portable_e2e_learning_cycle_v1/02_terminal_approach_feedback/README.md)

관련 회귀 **1,300개 통과·1개 건너뛰기**. 이전 공개 자료 350개는 변경하지 않았다.
이번 주행은 CARLA BasicAgent 기반 전문가 데이터 수집이며 Autoware UI·학습 모델 직접 주행이 아니다.
모델 직접 운전으로 통과한 기능은 여전히 **0/30**이고, 실제 차량용 승인은 없다.

다음은 저속 피드백 구간의 페달·위치/yaw 변동과 정지 경계 실패를 분리하여 새 제어 후보를 설계·검증하는 것이다.
기존 데이터 정답·허용 기준을 유지하며, 실패를 숨겨 학습을 시작하지 않는다.

[오늘 최신 목록](assets/validation/2026-09-15/portable_e2e_learning_cycle_v1/CURRENT.md) ·
[9개 영역·30개 기능의 기존 상세 현황](portable-e2e-status-2026-09-14.md)
