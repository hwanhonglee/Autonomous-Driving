# Town03 직진 — 실제로 한 것과 실패 원인

<!-- HH_261004 - Keep the original failed attempt separate from corrected future instrumentation and geometry. -->

후속 v3는 별도 새 선언으로 실제 45초를 완료하고 예비 기준을 통과했습니다. [현재 결과·남은 범위](../04_town03_known_dev_straight_v3/OUTCOME.md)를 참고하세요. 아래 첫 시험의 원본 FAIL은 변경하지 않았습니다.

**판정: 첫 시험 FAIL, 기존 모델 GOOD6000 유지.** PRO6000에서 새로 학습한 두 후보를 운전시킨 것이 아니다. Town07에서 사용하던 동일 GOOD6000·preview 0.5초·CPU 4+1·PI 2.4 설정으로, 로컬 CARLA Town03의 이미 알려진 DEV 직진 경로를 1회 시험했다. 목표 속도는 **9km/h**이며 30km/h 성능을 주장하지 않는다.

| 확인 항목 | 실제 기록 | 의미 |
| --- | --- | --- |
| 전체 구간 | 45초, 상태 900개·6카메라 묶음 450개 | 실패 꼬리까지 유지 |
| 주행 제어 | 시작 교사 47명령, NN 415회 추론·830 ACK | 출발 보조와 모델 제어를 구분 |
| 다음 상태에서 관측된 NN 효과 | 829개 | 마지막 ACK 뒤 없는 상태를 만들지 않음 |
| NN 구간 차량 기준 진행 | 96.44m | 제한 직진 실제 움직임, 종점 정지 아님 |
| NN 구간 차량 기준 CTE p95 / 최대 | 0.0371 / 0.0509m | 차량 기준과 rear-base 기준을 섞지 않음 |
| 실제 새 영상 수신→NN ACK | 415개, 최대 85.27ms | 카메라 노출부터의 전체 10Hz 증명은 아님 |
| 충돌·차선 센서 이벤트 | 각각 0 | 차체 전체 경계 검사를 대신하지 않음 |
| 원래 차체 검사 | 900개 차체·899개 sweep 중 FAIL | 원래 FAIL을 바꾸지 않음 |

## 왜 통과하지 못했는가

**1. 영상 저장 증빙 연결 누락.** 실제 영상 2,700장과 ACK 904개는 모두 있었지만 `recording_after_ack.jsonl`을 생성하는 deferred recorder가 이 실행 경로에 연결되지 않았다. 따라서 “ACK 후 저장했다”는 순서를 증명할 수 없어 검사기가 거부했다. 영상 손실이나 중복 카메라로 판정한 것이 아니다. 원본에는 없던 저장 시간을 사후에 만들어 넣지 않는다.

**2. 출발 지점의 차체 검사 영역 누락.** 차량은 차로 중심에 있었으나 뒷부분이 경로 시작점 이전으로 약 0.83m 나갔다. 검사 영역에는 경로 첫 차로 `(1426,0,2)`만 있어, 물리적으로 직접 이어진 출발 뒤 차로 `(67,0,2)`를 포함하지 못했다. 첫 82개 상태의 10cm inset 실패에는 warmup 24개·출발 교사 47개·NN 첫 11개가 모두 포함된다. 이 82개를 잘라서 성공으로 세지 않는다. 연결 차로의 실제 XODR·오프라인 CARLA topology를 독립 검증한 뒤, 새 버전의 사전 선언과 전체 차체 검사로 확인해야 한다.

고정 45초의 원래 수집기는 209m 경로 전체를 끝내지 못해 driving-budget 오류도 남겼다. 이는 사전에 종점 정지를 끄고 전체 45초를 기록하기로 한 제한 시험의 원본 상태다. **전체 경로 완료나 정지 성공이 아니다.**

## 보존된 화면과 원본

[PNG 5장·전체 45초 GIF](README.md)는 실제 6카메라 JPEG와 기록 차량 상태를 사용한다. 차량을 중심에 두고 원 경로·이미 주행한 궤적·이전 ACK가 적용된 NN 계획을 표시한다. **실제 Autoware/RViz 스크린샷이 아닌 기록 기반 CARLA 진단 화면**이다. 실행 전에 PNG 최초/25%/50%/75%/마지막과 GIF 0.5초 간격을 고정했다. GIF는 91프레임·45,000ms이며 마지막 실제 카메라와 실패 꼬리를 포함한다.

[원래 FAIL 진단](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v1/diagnostic_report.json), [실제 learned 요약](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v1/episode.partial/learned_summary.json), [전체 입력 caller seal](../../../../../../storage/experiments/plans/2026-10-04/town03_known_dev_straight_v1/ROOT_actual_publication_caller.json), [게시 provenance](publication_provenance.json)를 보존했다. 렌더 후 전체 4,624개 원본 파일과 모든 게시 이미지 SHA가 그대로임을 확인했다.

**다음:** 모델·제어·안전 임계값은 유지하고, 새 버전에서 실제 저장 ACK 연결 및 출발 연결 차로의 계측을 고친 뒤 같은 전체 구간을 다시 시험한다. 첫 실패와 다음 시도를 모두 분모에 남긴다. 이 DEV 예비 시험이 통과하더라도 정식 기능은 여전히 메인 0/9·하위 0/30이다.
