# 카메라 시간 계측 첫 시도 — 실패 원인과 다음 수정

<!-- HH_261004 - Preserve the actual failed interval and distinguish configured preview from the incorrectly bound calculation. -->

결론은 **FAIL**이다. 전체 주행 기록은 남았지만, 계측을 추가하면서 실제 0.5초 미리보기 계산 함수 연결을 놓쳤다. 이 결과를 정상 설정의 주행 성공이나 이전 v3보다 좋아진 성능으로 세지 않는다. 정식 메인 0/9·하위 0/30이다.

실제 기록은 45초·상태 900개·6카메라 450묶음·명령 ACK 904개다. 카메라 콜백 5,405개와 새 NN 계산 415개의 단계 표시 1,245개도 보존했다. 주행 원본을 학습 데이터로 사용하지 않는다.

| 문제 | 확인한 원인 | 별도 수정본에서 할 일 |
| --- | --- | --- |
| 선택 속도의 미리보기 증빙 누락 | 계측용 함수가 실제 설정 활성화 **전**의 제어 함수를 복사했다. 저장된 설정값 0.5초와 실제 연결은 다른 문제다. | 설정 활성화가 모두 끝난 뒤 함수를 연결하고, 그 안에서 원본과 수치 동일성을 검사한다. |
| 저장기 증빙 판정 실패 | 실제 450묶음·2,700 JPEG·904 ACK는 존재한다. 앞의 미리보기 검사에서 종료돼 저장기 결과를 붙이지 못했다. | 원래 저장기 provider를 유지하고 영상 묶음 반환 직후 관찰만 추가한다. |
| 시간 기록을 다시 읽지 못함 | 새 콜백 JSON은 5,692,244바이트인데 기존 일반 JSON 제한은 1MiB였다. | 콜백 파일에만 16MiB·SHA·스키마·행 수·일반 파일 검사를 적용한다. 안전 시간 기준 100ms는 그대로다. |

기존 v1 코드·결과·실패 구간을 덮어쓰지 않는다. v2는 새 선언·새 소스·새 출력으로 실행한다. PNG에 표시한 `preview0.5`는 이 v1에서 선언된 설정이며, 제대로 연결된 0.5초 계산의 증명이 아니다.

[실제 전체 GIF·차량 중심 PNG](README.md), [원본 진단 보고서](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_callback45_v1/diagnostic_report.json), [다음 수정본의 사전 선언](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_callback45_v2_plan.json)을 함께 본다. 화면은 기록된 CARLA 진단 렌더링이며 실제 Autoware/RViz 캡처가 아니다.
