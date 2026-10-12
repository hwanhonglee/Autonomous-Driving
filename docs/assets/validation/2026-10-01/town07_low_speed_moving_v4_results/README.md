# Town07 저속 순항 — 327개 창 검사 통과·실제 6카메라 기록

<!-- HH_261001 - Report the fixed cruise data separately from learned-model driving and admission. -->

**9km/h 목표 전문가 주행을 실제 수집했고, 고정된 327개 창 전부가 영상·제어·차체·미래 궤적 검사에 통과했다.** 앞선 실패 자료를 덮어쓰거나 실패 프레임을 잘라내지 않았다. 이번에는 준비된 V4의 출처 보관·제어 속도 참조점·미래 감사 오류 수정이 실제 기록에서도 작동했다.

이것은 BasicAgent 전문가 주행이다. 새 학습 모델의 주행, Autoware 화면, 전체 경로 완주·종점 정지 성공이 아니다. 목표 구간은 미리 정한 **40–120m 일정 속도 주행**이며, 기존 모델의 정체가 나타난 10–30m와 다르다. QA의 결과는 `NECESSARY_CHECKS_CLEAR_PENDING_AUTHORITY_REVIEW`; 데이터 편입·변환은 아직 하지 않았다.

## 화면과 기록

차량 위치를 고정한 오른쪽 패널에는 원래 경로, 실제 이동 궤적, 검산한 차체가 함께 보인다. GIF는 10Hz로 수집한 실제 영상 중 약 2Hz를 표시한다. 영상 중간 장면을 생성하거나 보간하지 않았다.

![6카메라와 차량 중심 경로](six_camera_ego_centered.png)

- [실제 6카메라 GIF](six_camera_ego_centered_2hz.gif): 67장면·402 JPEG, 원본 시간에 맞춘 32.7초 재생. 마지막 0.1초 표시 유지가 포함된다.
- [속도·진행 분석 PNG](speed_progress_summary.png): 카메라 10Hz 표본과 원본 20Hz 표본을 구분한다.
- [표시 출처·타임스탬프·해시](visual_metadata.json), [전량 QA 원본](../town07_low_speed_moving_v4_qa_v1/summary.json).

## 무엇을 확인했나

| 항목 | 결과 |
| --- | --- |
| 고정 구간 | 실제 진행 40.0108–119.9776m, 영상 앵커 시간 폭 32.6000초 |
| 선택 창 | 327/327 통과; 사후 필터링·조건 완화 없음 |
| 전체 기록 | 원본 상태 1,395개/20Hz, 영상 묶음 698개/10Hz, 4,188개 JPEG 전량 해독 성공 |
| 순항 속도 | 선택 시간 안의 원본 653 tick 최소/평균/최대 2.4196/2.4528/2.4847m/s, 약 8.71/8.83/8.94km/h |
| 제어 명령 | 1,399개 전부 ACK; 관측 제어 불일치·정렬 실패 0 |
| 미래·과거 | 각 선택 창의 필요한 과거 영상·원본 guard·6.4초 미래를 포함해 검사 |
| 차체·궤적 | 모든 선택 창의 차체/구간 이동 여유·기존 9개 미래 운동 기준 통과 |
| 이벤트 | 전체 기록의 충돌/차선 침범 이벤트 0 |
| 소스 | 선언한 workspace 21개와 외부 agents 8개 보관·실행 출처 검사 통과; OS/서버 바이너리 전체 보증 아님 |
| 종료 | 수집 exit 0, cleanup 오류 없음, 소유한 서버 종료·2440–2442 포트 해제 |

**전체 녹화가 모두 좋은 학습 자료인 것은 아니다.** 선택 창과 그 과거·미래를 보호한 영역 밖에는 출발 +3.194m/s²와 강제 종료 제동 −5.804/−20.996/−19.277m/s²가 남는다. 원본을 보존했고 훈련 앵커로 선택하지 않았다. 강제 종료를 정상 종점 정차 라벨로 쓰지 않는다. 기존 전체 종점 품질 계산의 `'complete'` 오류도 원본의 `UNAVAILABLE_RETAINED`로 유지했다. V4 전체 경로·출발·정차를 통과시킨 결과가 아니다.

## 다음 단계

우선 같은 모델이 멈추던 **10–30m 저속·재가속 구간을 영상과 함께 수집**해야 한다. 앞선 4km/h 전문가 예비 주행은 가능했지만 카메라와 미래 정답을 기록하지 않았다. 그 다음 좌/우회전과 새로운 독립 평가 장소를 확보하고, 적격 자료를 새 V2의 경로·시간 공동 학습에 연결한다. 이 327개는 겹치는 한 episode의 창이며 327개의 독립 주행 성공이 아니다.

[같은 블록의 새 V2 실제 학습 결과](../temporal_v2_raw_bootstrap_v1/README.md): 이번 V4를 학습한 것이 아니라 기존 자료로 별도 GPU 학습 진단을 했다.

원본은 [owner 결과](../../../../../storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v4/owner_moving_result.json)와 [episode manifest](../../../../../storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v4/episode/manifest.json)이다. [V4 고정 QA 규약](../town07_low_speed_moving_V4_QA_PROTOCOL.md)을 변경하지 않았다. 실행·표시 파일은 [소유 실행기](../../../../../tools/e2e/run_owned_town07_low_speed_moving_v4_20261001.py), [감사기](../../../../../tools/e2e/audit_town07_low_speed_moving_v4_20261001.py), [화면 생성기](../../../../../tools/e2e/publish_town07_low_speed_v4_gallery_20261001.py)에 있다.
