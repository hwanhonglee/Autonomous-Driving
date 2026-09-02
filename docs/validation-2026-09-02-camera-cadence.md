# CTrack 30 kph camera-source cadence 검증

2026-09-02 공식 재실행에서 CTrack 직진과 좌회전은 모두 full-stack `PASS`했다.
Candidate는 CARLA의 여섯 RGB camera source를 `sensor_tick=0.2 s`로 제한하고 ROS
publish는 5 Hz로 유지한다. VAD route manager에만 BLAS 계열 thread를 각각 1로
고정했으며, baseline과 candidate가 동일한 route를 사용했음을 byte SHA와 canonical
payload로 확인했다.

이 보고서는 CTrack 한 맵의 기능·무결성 A/B이다. 해당 matrix의 `INCOMPLETE 1/9`는
오류가 아니라 CTrack만 선택했기 때문이며, 9개 실행 가능 맵 전체의 기준 결과는
[`validation-2026-09-01.md`](validation-2026-09-01.md)에 그대로 유지된다.

## 결론

- Candidate 직진·좌회전 모두 goal reached, route/speed gate PASS.
- 여섯 camera 모두 stamp rate `5.0 Hz`, 최대 stamp gap `0.200 s`, bundle coverage
  `100%`였다. VAD supersession, pruning, coalescing은 0이고 published와 mailbox taken
  count가 일치했다.
- Candidate의 wall receipt rate는 직진 `4.992644 Hz`, 좌회전 `4.992597 Hz`였고
  RTF는 각각 `0.999625`, `0.999539`였다. 따라서 candidate 공식 화면은 거의
  실시간으로 갱신됐다.
- 화면에서 보인 끊김의 직접 지표는 camera stamp 누락이 아니라 낮은 RTF다. 시뮬레이션
  시간의 camera frame은 5 Hz로 정상이어도 RTF가 0.25이면 벽시계에서는 약 1.25 Hz처럼
  보이며 같은 frame이 오래 유지된다.
- 이전 실패 재현에서는 `vad_route_manager`가 OpenBLAS 24 thread를 사용해
  `776-959%` CPU를 점유하고 camera stamp gap이 `3.2-3.6 s`까지 증가했다. 이를
  해당 노드에만 1 thread로 제한하여 제거했다. 시스템 전체 환경변수는 바꾸지 않았다.

## 결과 수치

| Trial | Arm | 최고속도 kph | 30 kph 노출 | 최대 CTE m | RTF | wall/stamp Hz | 최대 gap s | bundle | inference p95 ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 직진 | baseline | 27.93 | 7.65 s | 0.519 | 0.319011 | 1.736707 / 5.0 | 0.200 | 100% | 38.074 |
| 직진 | candidate | 27.76 | 6.25 s | 0.529 | 0.999625 | 4.992644 / 5.0 | 0.200 | 100% | 31.317 |
| 좌회전 | baseline | 16.93 | 곡률 제한 | 0.513 | 0.998307 | 4.992801 / 5.0 | 0.200 | 100% | 38.576 |
| 좌회전 | candidate | 17.01 | 곡률 제한 | 0.514 | 0.999539 | 4.992597 / 5.0 | 0.200 | 100% | 31.493 |

30 kph는 target profile이다. 직진 합격 조건은 actual speed `>=7.5 m/s (27.0 kph)`를
1초 이상 유지하는 것이고, 회전은 `curvature_limited_turn`이라 30 kph 유지 요구가
없다. 따라서 이 결과를 “실제 30 kph 달성”으로 해석하면 안 된다.

Candidate/front count는 직진 `277/277`, 좌회전 `231/230`이다. 좌회전의 +1은 whole-bag
시작·종료 경계 허용치이며 causal frame exactness 주장이 아니다. 모든 camera bundle은
동일 stamp로 100% 구성됐다.

## 화면 증거

두 대표 화면은 owned RViz window만 캡처한 1920×1080 PNG다. 차량은 오른쪽 map
viewport 중앙에 있고 기준 경로, VAD 후보, 최종·실주행 궤적, 전방 camera,
Autoware Routing/Localization/Autonomous 상태를 동시에 볼 수 있다.

| 직진 | 좌회전 |
|---|---|
| ![CTrack straight centered](assets/validation/2026-09-02/c_track_1_0_7/autoware_vad/straight/autoware_rviz_fullscreen.png) | ![CTrack left turn centered](assets/validation/2026-09-02/c_track_1_0_7/autoware_vad/turn/autoware_rviz_fullscreen.png) |

- 직진: [drive GIF](assets/validation/2026-09-02/c_track_1_0_7/autoware_vad/straight/autoware_rviz_drive.gif), [경로 분석](assets/validation/2026-09-02/c_track_1_0_7/autoware_vad/straight/autoware_vad_route_result.png), [속도 분석](assets/validation/2026-09-02/c_track_1_0_7/autoware_vad/straight/speed_profile.png), [5-frame 중앙 유지 감사](assets/validation/2026-09-02/visual_audit/straight_contact.png)
- 좌회전: [drive GIF](assets/validation/2026-09-02/c_track_1_0_7/autoware_vad/turn/autoware_rviz_drive.gif), [경로 분석](assets/validation/2026-09-02/c_track_1_0_7/autoware_vad/turn/autoware_vad_route_result.png), [경로·제어 GIF](assets/validation/2026-09-02/c_track_1_0_7/autoware_vad/turn/autoware_vad_turn_path_control.gif), [5-frame 중앙 유지 감사](assets/validation/2026-09-02/visual_audit/turn_contact.png)

전체 발행 자산과 provenance는
[`assets/validation/2026-09-02/README.md`](assets/validation/2026-09-02/README.md), A/B 원시
요약은
[`camera_cadence_ab.md`](assets/validation/2026-09-02/camera_cadence_ab/camera_cadence_ab.md)에 있다.

## 끊김 원인 분석

공식 route-window host telemetry에서 CPU idle 최솟값은 `81-84%`였고 GPU SM 평균은
`16.7-34.3%`였다. 즉 전체 PC의 지속적인 CPU/GPU 포화 증거는 없다. 다만 프로세스
단위 thread 과구독은 전체 평균이 낮아도 특정 scheduling path와 CARLA sync loop를
막을 수 있으며, 실제로 BLAS cap 이전 candidate retry에서 이를 재현했다.

Baseline 직진만 RTF `0.319`인 현상도 단순 camera 설정의 인과 효과로 단정할 수 없다.
이 trial은 실행 순서상 첫 CARLA cold start였고 wall `122.2 s` 부근까지 RTF 약 0.249로
진행하다 마지막 약 14초에 1:1로 회복했으며 동시에 GPU SM이 상승했다. 엔진·shader·driver
cache 또는 일시적인 renderer/scheduler 상태가 강한 순서 교란 요인이다. Candidate의
직진 RTF `+213.352%`와 inference p95 약 `17.7-18.4%` 감소는 관찰값일 뿐 최적화 채택
근거로 쓰려면 ABBA 반복, 실행 순서 무작위화, 사전 정의한 threshold가 필요하다.

## 검증 경계와 다음 단계

현재 architecture는 `vad_route_manager_hybrid`이고 종방향 속도는 explicit CARLA
simulation profile이 공급한다. VAD geometry는 평가했지만 raw VAD velocity는 평가하지
않았고 `real_vehicle_ready=false`다. 이번 CTrack 검증을 기준으로 동일 candidate profile을
9개 실행 가능 맵의 직진·회전에 확장한 별도 전체 matrix가 다음 단계다.
