# Town03 직진 v3 — 제한 예비 기준 통과

<!-- HH_261004 - Lead with the actual fixed45 result while separating infrastructure repair from learned weight improvement and formal approval. -->

## 이번에 확인한 결과

**기존 GOOD6000 모델이 Town03 직진 45초 예비 시험을 통과했습니다.** 출발 교사 이후 모델 제어로 약 96.37m를 주행했고, 전체 상태 900개·6카메라 450묶음·JPEG 2,700장·ACK 904개를 보존했습니다. 정식 기능 완료가 아니라 known DEV 한 경로의 제한 시험입니다.

| 검사 | 실제 결과 | 기존 기준·판정 |
| --- | ---: | --- |
| 모델 제어 이후 차량 기준 진행 거리 | 96.37m | 90m 이상·통과 |
| 차량 기준 중심 오차 p95 / 최대 | 2.63 / 3.69cm | 35 / 75cm 이하·통과 |
| 최대 실제 속도 | 8.64km/h | 2.7m/s 상한·통과; 30km/h 시험 아님 |
| 충돌·차선 침범·긴급제동 명령 | 모두 0 | 통과 |
| 10cm inset 뒤 차체 / 상태 사이 sweep 최소 여유 | 0.530 / 0.510m | 전체 900 / 899개·침범 0 |
| 측정된 영상 전달 → 명령 ACK 최대 시간 | 78.46ms | 100ms 이하·통과 |
| 실제 가감속 범위 | -2.02 ~ +2.92m/s² | −6 ~ +3·통과 |

차량 기준은 CARLA actor API 위치를 원 경로에 투영한 값이며 질량중심으로 확인된 것은 아닙니다. 후륜 base_link 기준 p95/최대는 2.61/3.33cm로 별도 계산했습니다. 두 기준을 섞지 않습니다.

## 무엇을 바꿨고, 무엇은 바꾸지 않았나

- v1은 저장 ACK 순서 증빙과 출발 연결 차로의 검사 범위가 빠져 FAIL이었습니다.
- v2는 소유권 식별값이 실제 신호등 fixture에 연결되지 않아 주행 전 0프레임에서 중단됐습니다.
- 별도 v3에서 두 소유권 schema 별칭만 실행 범위 안에서 일치시키고 항상 원복했습니다. 실제 보호 함수의 정상·잘못된 SHA/schema/PID를 먼저 검사했습니다.
- **가중치·모델 구조·제어기·경로·맵·속도·안전 기준은 바꾸지 않았습니다.** 따라서 이번 결과를 새 가중치의 성능 향상으로 주장하지 않습니다. 실제 주행을 정상 계측하고 판정할 수 있게 개선한 것입니다.

같은 10cm inset을 유지하면서 출발 뒤쪽에 실제 연결된 동일 주행 차로만 포함했습니다. 원 경로는 변경하지 않았고, 기존 원본 FAIL을 PASS로 덮어쓰지 않았습니다. 전체 3회 시도는 **v1 FAIL / v2 시작 FAIL / v3 제한 PASS**로 그대로 남깁니다.

## 교사인가, 모델인가

교사는 출발 명령 47개만 담당했습니다. 이후 실제 NN 추론 415회·ACK 명령 830개와 다음 상태에 반영된 **829개 NN 효과**를 확인했습니다. 마지막 명령 뒤의 존재하지 않는 상태는 만들지 않았습니다. 모델 전환 이후 교사로 되돌아가지 않았습니다.

신호등은 소유한 시험 세계에서만 녹색 고정 후 원복했습니다. 신호 준수·장애물 대응 기능을 검증한 시험이 아닙니다. 종점 정지는 비활성화했고 전체 209m 경로의 완주도 시험하지 않았습니다.

## 원본에 failed라고 적혀 있는 이유

수집기는 45초 안에 전체 경로를 완주하지 않으면 원래 budget 오류를 남깁니다. 이번 시험은 실행 전에 **45초·900개 상태 전체를 보존하는 별도 직진 예비 검사**로 선언했습니다. 그래서 manifest의 failed 상태와 오류를 그대로 보존하면서, 별도 진단에서 예비 기준을 평가했습니다. 숨긴 조기 종료나 사후 기준 완화가 아닙니다.

owner 준비·주행·정리 시간은 **90.19초**였습니다. 소유 프로세스·포트 정리와 전후 소스 동일성 검사를 통과했고 현재 시험은 종료됐습니다.

## 아직 증명하지 못한 것과 다음 순서

현재 정식 완료는 여전히 **메인 0/9·하위 0/30**입니다. 독립 경로·여러 사이트·선언된 반복 횟수, 종점 정지, 30km/h, 센서 callback/노출부터 ACK까지의 전체 지연, 실제 Autoware 연동이나 실차 안전성을 증명하지 못했습니다. 지금의 78.46ms를 전체 센서→제어 지연으로 확대하지 않습니다.

다음은 이 정상 계측 경로로 기존 기능의 회귀 시험을 유지하면서, 직진과 회전 학습의 가중치 충돌을 먼저 분석하는 것입니다. 완료된 보존 비교 후보도 직진 오차 퇴행이 남아 미채택입니다. 실패 원본·known DEV를 TRAIN으로 바꾸지 않고 기존 승인 TRAIN만 사용합니다. 신호·ACC·회피 등의 새 head는 기본 주행을 확보한 뒤 실제 상황 데이터와 행동 검사를 함께 추가합니다.

## 화면과 재현 근거

[전체 45초 GIF·차량 중심 PNG 5장](README.md)은 기록된 실제 CARLA 6카메라와 원 경로·주행 궤적·이전 ACK 적용 NN 계획을 보여줍니다. **기록 기반 진단 렌더링이며 Autoware/RViz 실제 캡처가 아닙니다.**

게시 후 원본 4,632개 파일과 모든 미디어 SHA를 전량 재검사했습니다. 원래 진단의 JPEG decode 미실행 표기는 변경하지 않았고, 후속 읽기 전용 검사에서 실제 JPEG 2,700장을 모두 640×360 형식으로 디코딩하고 원본 SHA와 대조했습니다. 이것은 파일 무결성 확인이며 인식 정확도나 센서 노출→제어 지연 검증이 아닙니다. [전체 원본·미디어 영수증](../../../../../../storage/experiments/diagnostics/2026-10-04/town03_straight_v3_publication_postcheck/ROOT_actual_whole_run_and_media.json), [전체 JPEG 디코딩 검사](../../../../../../storage/experiments/diagnostics/2026-10-04/town03_straight_v3_publication_postcheck/ROOT_all_actual_JPEG_decode.json).

- [실행 전 고정 선언](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v3_plan.json)
- [원본 전체 진단](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v3/diagnostic_report.json)
- [실행 전 화면 선택 규칙](../../../../../../storage/experiments/plans/2026-10-04/town03_known_dev_straight_v3/ROOT_prospective_publication_policy.json)
- [전체 원본 파일 고정 해시](../../../../../../storage/experiments/plans/2026-10-04/town03_known_dev_straight_v3/ROOT_actual_publication_caller.json)
- [코드·사전 CPU 검사 결과](../../../../../../storage/experiments/plans/2026-10-04/town03_known_dev_straight_v3/ROOT_preflight_and_final_source_receipt.json)

재현하려면 새 시도 revision과 새 선언이 필요합니다. 같은 v3를 재실행하거나 기존 원본을 덮어쓰지 않습니다.
