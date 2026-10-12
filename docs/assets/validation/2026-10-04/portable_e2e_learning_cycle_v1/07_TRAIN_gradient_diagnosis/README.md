# 추가 학습이 기본 주행을 해친 이유 — 실제 GPU 분석

<!-- HH_261004 - Report TRAIN gradient measurements without asserting causality, optimizer updates, or independent performance approval. -->

PRO6000 개인 venv의 **물리 GPU 0**에서 분석을 완료했다. 기존 학습이 계속 돌고 있는 것은 아니다. 고정된 TRAIN 시점 160개에 대해 손실의 수정 방향을 1,152회 계산했다. 이 분석에서 가중치 업데이트는 **0회**이며, 모델을 바꾸거나 채택하지 않았다. 측정 루프는 77.62초였다.

서로 다른 손실을 함께 줄이려 할 때 같은 가중치를 반대 방향으로 움직이려는지 확인했다. 아래 방향 일치도는 +1이면 같은 방향, −1이면 반대 방향이다. 이는 TRAIN 표본의 진단이지 독립 평가 점수나 주행 성공률이 아니다.

| 비교 | 측정값 | 의미 |
| --- | --- | --- |
| 기존 저속 주행 ↔ 우회전, 전체 손실 방향 | −0.392 | 두 그룹을 개선하려는 방향에 충돌이 있다. |
| 좌회전 ↔ 우회전, 곡률 손실 방향 | −0.515 | 회전 곡률을 같은 가중치로 수정할 때 서로 방해할 수 있다. |
| 좌회전 ↔ 우회전, 위치 손실 방향 | −0.567 | 곡률뿐 아니라 계획 위치에도 충돌이 보인다. |
| 각 그룹의 평균 전체 수정 힘: 기존 저속 / 좌회전 / 우회전 | 0.422 / 6.214 / 7.949 | 회전 쪽 수정 힘이 훨씬 크다. 실제 학습의 그룹별 계수·동일 배치 합성까지 따로 확인해야 한다. |

이 결과는 **학습 횟수만 늘리면 해결된다는 근거가 아니다.** 충돌이 모든 실패의 단독 원인이라는 증명도 아니다. 그룹마다 입력이 다르고, 그룹 평균은 기존 5영역 학습 배치의 합성 gradient와 동일하지 않다. 변경 구조를 고르기 위한 계측으로 사용한다.

이전 모델 출력 보존 벌점은 시작 시점에 gradient 0이었다. 학생과 기준 모델이 같은 GOOD6000 가중치이므로 정상이다. 원래 attention 설정에서 첫 실제 표본도 출력이 정확히 같았으며, 별도 맞춤 attention 분석의 설정은 종료 후 원복했다. 시작의 0이 학습 후 보존을 보장하지는 않는다.

확인한 범위는 고정 TRAIN 5영역의 각 32개, 기존 모델·데이터·허가 32개 SHA, 원본 학습 JPEG 41,256장의 전후 동일성이다. 모델 상태 101개 텐서의 전후 바이트도 같다. 원본 결과 41개 파일을 로컬에 복사해 서버 SHA와 전량 대조했다. TEST/HOLDOUT 영상·라벨은 열지 않았다. 설치·GPU 1·CARLA 구동·모델 배포도 하지 않았다.

다음 가중치 변경은 기본 곡률·속도 경로의 충돌을 줄이는 **새 TRAIN 전용 비교**로 설계한다. 학습 후 기존 기본 주행의 고정 점검 시점까지 다시 평가하고, 좋아지지 않은 후보는 주행에 넣지 않는다. 새 head 추가보다 기본 주행 보존을 먼저 한다. 현재 정식 완료는 메인 0/9·하위 0/30이다.

- [실제 분석 요약](../../../../../../storage/experiments/diagnostics/2026-10-04/temporal_route_joint_train_gradient_audit_v1/summary.json)
- [160개 전체 표본 원본](../../../../../../storage/experiments/diagnostics/2026-10-04/temporal_route_joint_train_gradient_audit_v1/probe_gradients.jsonl)
- [그룹 간 전체 비교 원본](../../../../../../storage/experiments/diagnostics/2026-10-04/temporal_route_joint_train_gradient_audit_v1/paired_domain_gradients.jsonl)
- [실행 전 TRAIN 전용 선언](../../../../../../storage/experiments/plans/2026-10-04/temporal_route_joint_train_gradient_audit_v1/ROOT_TRAIN_gradient_authorization.json)
- [실제 개인 환경 사전 검사](../../../../../../storage/experiments/plans/2026-10-04/temporal_route_joint_train_gradient_audit_v1/ROOT_remote_CPU_metadata.json)
