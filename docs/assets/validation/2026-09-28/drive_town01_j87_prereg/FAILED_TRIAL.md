# Town01 j87 우회전 단회 원본: 회전은 관찰, 수집은 실패

<!-- HH_260928 - Report the failed complete-trial gate without trimming the raw signal stop or goal miss. -->

**판정 `FAILED_DIAGNOSTIC_ONLY` — 학습 편입·모델 개선·폐루프 승인이 아니다.** 이 화면과 궤적의 운전자는 학습 E2E나 Autoware가 아닌 CARLA `BasicAgent`다. [사전 고정 계획](CAPTURE_PLAN.md)에 따라 9km/h 목표, Epic/ClearNoon, Prius, 20Hz 차량 상태/10Hz 6카메라, 같은 출처의 Town01에서 단회 실행했다. [owner 결과](../../../../../storage/experiments/training/2026-09-28/drive_town01_j87_right_9kph_expert_trial_v1/owner_result.json)의 exit code는 **1**, [원본 manifest](../../../../../storage/experiments/training/2026-09-28/drive_town01_j87_right_9kph_expert_trial_v1/episode.partial/manifest.json)는 `status=failed`다. 다른 결과를 고르기 위한 재실행이나 관측 구간 삭제는 하지 않았다.

![원본 전방 영상 한 프레임과 같은 시각 차량 중심 회전 경로](visuals/turn_camera_and_ego_context.png)

[전체 카탈로그 경로와 실제 기록 궤적·최종 목표 차이 PNG](visuals/route_vs_recorded_path.png). 첫 이미지는 원본 전방 frame 3081과 같은 시각의 ego 중심 ±24m 경로 화면이다. 두 번째는 전체 원본 경로를 보여 주되 **차로·차체 통과 판정 그림이 아니다**. [재현 스크립트](../../../../../tools/e2e/render_failed_expert_drive_visuals.py)는 실패한 `episode.partial`만 읽고 기존 이미지 출력을 덮어쓰지 않는다.

| 사전 고정/진단 항목 | 실제 원본 | 판단 |
| --- | --- | --- |
| live OpenDRIVE | 차량 생성 전 SHA256 `97a7f6ac...709` 일치 | 출처 검사 통과 |
| 촬영·제어 | 20Hz 상태 1,940개, 10Hz 6카메라 anchor 970개, ACK 1,945/1,945, 제어 정렬 실패 0 | 실패 원본은 보존. 정상 종료·학습 자격을 뜻하지 않음 |
| 고정 경로 30–120m의 이동 | 20Hz 상태 1,374개, `>0.5m/s` 연속 이동 최장 **567 tick = 28.35초**(timestamp 양 끝 간격 28.30초) | 요구한 30초에 미달. 사후 window 이동·정지 프레임 삭제 금지 |
| 중간 저속 구간 | progress 약 50.419–50.469m, 639 tick = **31.95초** `≤0.5m/s`; 그중 632 tick 제어값 throttle 0/brake 0.5 | 전방 신호가 바뀌는 모습과 BasicAgent의 기본 신호 준수 제동 방식에 부합하지만, 원본에 신호 상태/원인 ID가 없어 **적색 신호 원인 확정은 아님** |
| 우회전 형상 | progress 59.970–74.231m에서 112개 상태, 방향각 약 **−75.3°** 변화, 최대 경로 중심 오차 약 0.600m | 우회전이 실제 발생. 차체의 의도 차로 10cm inset 전량 통과를 의미하지 않음 |
| 종점 | BasicAgent `done=true`, route progress 149.535/152.106m, 목표까지 평면 base_link **2.552m** | 허용 2.500m를 **0.052m 초과**하여 `goal_reached=false`; collector가 의도대로 실패 처리 |
| 충돌/침범 센서·정지 tail | 센서 이벤트 각 0, 완료 후 정지 tail 0 tick | 센서 이벤트 0은 차체 corridor 통과 증거가 아니며, tail 부재도 완료 원본 요건 미달 |

중간 정지 구간의 첫/끝 [전방 원본 frame 2301](../../../../../storage/experiments/training/2026-09-28/drive_town01_j87_right_9kph_expert_trial_v1/episode.partial/images/CAM_FRONT/00002301.jpg) · [frame 2935](../../../../../storage/experiments/training/2026-09-28/drive_town01_j87_right_9kph_expert_trial_v1/episode.partial/images/CAM_FRONT/00002935.jpg)은 같은 위치에서 신호 표시가 바뀌는 장면이다. 기록된 제어와 시각적 변화로 신호 정지가 가장 유력하지만 원인 필드가 없으므로 확정하지 않는다. 이처럼 **합법적인 신호 대기일 수 있는 구간**을 포함한 고정 창에 연속 30초 이동을 요구하면, 운전 품질이 좋더라도 해당 자료는 통과할 수 없다. 그러나 이 실행의 기준을 사후 변경하지 않는다.

이번 진단의 실패 종료 후 소유 `ready/after_capture/stopped` 영수증, 출처 소스의 시작·종료 해시, 충돌 0, 2100–2102 포트·CARLA 프로세스 해제를 확인했다. 정상 센서 plumbing과 회전 관찰은 남기되, **부분 원본에서 운동 9항목·64점 미래·차체/sweep 전량 QA를 성공 판정으로 돌리지 않는다.** 본 실행은 `episode.partial` 그대로 보관하고 TRAIN/DEV 패키지·GPU 학습 입력으로 사용하지 않는다.

다음 주행 실험은 (1) 독립 물리 경로·site 및 TRAIN/DEV 분할을 다시 확인하고, (2) 촬영 *전* 신호 대기가 평가창에 들어오는지 정적 지도와 짧은 별도 정찰로 확인하며, (3) endpoint/목표 종료 계약을 사전 등록하고 (4) 같은 30초 이동·미래·몸체 기준으로 **새 출력 경로**에서 한 요인만 바꾸어 검증한다. 신호 준수를 끄거나 현재 목표 허용치를 늘려 이 실패를 성공으로 바꾸지 않는다. 정지·ACC 등 동작은 이후 별도 정지 인식형 기준과 실제 표지/신호 상태 기록으로 평가한다.

이 실패 분석 뒤 [collector](../../../../../tools/e2e/collect_carla_vad_expert.py)에 선택형 `--observe-traffic-lights`를 추가했다. **다음 새 수집에서만** 프레임별 차량-신호 연결 및 BasicAgent가 마지막으로 본 신호 액터/색을 읽어 기록한다. 수집 제어·기본 출력은 바꾸지 않고, 신호 API나 프레임 정합이 실패하면 기록도 실패 처리한다. 이 읽기값만으로 BasicAgent 제동 원인이 증명되는 것은 아니며 **이미 보존한 v1에는 소급 신호 데이터가 없다.** 관련 fake-world 테스트 49개는 통과했으나 새 CARLA 실행 검증은 아직 하지 않았다.
