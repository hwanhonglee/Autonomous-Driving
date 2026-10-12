# Town03 v2 — 소유권 설정 연결 오류로 주행 전 중단

<!-- HH_261004 - Preserve a zero-frame infrastructure failure without fabricating images or counting model behavior. -->

별도 v3의 실제 재시험은 종료됐고 제한 예비 기준을 통과했습니다. [v3 결과·전체 화면](../04_town03_known_dev_straight_v3/OUTCOME.md)을 참고하세요. 아래 v2 실패 기록은 그대로 유지합니다.

**완료 상태: 시작 실패, 모델 주행 판정 불가.** 소스·선언은 사전에 동결했고 실제 로컬 owner를 실행했다. 36.52초 뒤 소유권 검사가 새 v2 선언과 신호등 fixture의 기존 v1 식별값 불일치를 발견해 거부했다.

- 실제 차량 상태 **0개**, 카메라 묶음 **0개**, 제어 ACK **0개**.
- 원본 오류: `CollectionError: fixture is restricted to the declared original Town03 owner`.
- 모델 추론·NN 주행 없음. 이것을 모델의 직진 실패 또는 학습 성능 퇴행으로 합산하지 않는다.
- owned 프로세스 정리·원래 TCP/UDP 포트 해제·소스/외부 자산 SHA 검사는 통과했다.
- 45초 전체 시험은 **완료하지 못했다**. 없는 화면·차량 자세·ACK를 사후에 만들어 넣지 않는다.

CPU 검사에서 사용한 합성 fixture가 이 실제 선언 식별값 연결을 검사하지 못했다. 작성 코드의 누락이며, 실제 보호 검사를 제거해 해결하지 않는다. 다음 별도 revision에서는 실제 선언과 동일한 식별값을 scoped fixture에 연결하고, 잘못된 소유권은 여전히 거부하는 양성·음성 검사를 추가한다. 모델·제어·차로·지연 임계값은 그대로 유지한다.

[실제 실행 선언](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v2_plan.json), [원래 진단](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v2/diagnostic_report.json), [owner 결과](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v2/owner_result.json), [수집기 로그](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v2/collector.log)를 보존한다. v2의 원본·동결 소스·선언은 덮어쓰지 않는다.

앞선 v1의 실제 45초 화면은 [별도 기록](../02_town03_known_dev_straight/README.md)에 있다. 이번 0프레임 실행의 화면이라고 바꾸어 부르지 않는다. 모든 시도는 남기되, 정식 기능 승인은 여전히 메인 0/9·하위 0/30이다.
