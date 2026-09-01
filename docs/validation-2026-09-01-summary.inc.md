## 2026-09-01 최종 요약

- 최종 v16은 실행 가능한 CARLA/Autoware full-map 9개에서 직진·회전 두 경로씩
  `18/18 PASS`했다. 19개 canonical inventory 중 나머지 10개는 검증된
  Lanelet2+PCD/runtime asset이 없어 `BLOCKED`로 보존했다.
- 이는 nominal `speed_30kph` 기능·안전 simulation screening 결과다. 직진은
  `7.5 m/s` 이상을 1초 이상 유지해야 PASS하며 실제 최고속도는
  `27.69-27.94 kph`였다. 회전은 곡률 제한으로 `17.62-24.02 kph`였고 별도
  최소 지속속도 gate가 없다. 따라서 모든 경로가 정확히 30 kph에 도달했다는 뜻은
  아니다.
- 전 trial의 최대 CTE는 `0.735 m < 1.0 m`, 최대 경로 correction은
  `14.189 m < 15 m`, 최대 횡가속은 `0.991 m/s² < 1.8 m/s²`였다. 진단 분포는
  `path_dominant=14`, `within_thresholds=3`, `mixed_path_and_control=1`이다.
- 18개 대표 PNG와 MKV/GIF 전 frame 감사를 통과했다. 차량은 우측 RViz 지도 viewport
  중앙에 유지되고 기준 경로, VAD 후보, 최종/실주행 궤적, 전방 camera와
  Routing/Localization/Autonomous 상태가 함께 보인다. root 화면이 아니라 소유한 RViz
  창만 녹화해 shell, dock, tooltip과 알림을 제외했다.
- 이 결과의 전체 claim boundary는 `vad_route_manager_hybrid` simulation screening이다.
  VAD geometry는 평가했지만 종방향 속도는 explicit CARLA overlay가 공급하며, raw VAD
  velocity는 평가하지 않았다. `real_vehicle_ready=false`이고 순수·무보조 E2E 성공
  주장이 아니다.
- Town10HD_Opt 우회전은 PASS지만 `mixed_path_and_control`이고 correction
  `13.286/15 m`; CTrack 좌회전은 `path_dominant`이고 `12.679/15 m`다. 두 경로가
  raw VAD planning 및 제어 강건성 고도화의 우선순위다.

### 핵심 수치

| Map | Trial | 최고속도 kph | 최대 CTE m | 최대 correction m | 횡가속 p95/max m/s² | RTF | 진단 |
|---|---|---:|---:|---:|---:|---:|---|
| `c_track_1_0_7` | `straight` | 27.86 | 0.540 | 1.240 | 0.355/0.461 | 0.249 | `path_dominant` |
| `c_track_1_0_7` | `turn` | 17.62 | 0.508 | 12.679 | 0.623/0.745 | 0.250 | `path_dominant` |
| `town01` | `straight` | 27.86 | 0.538 | 0.545 | 0.409/0.507 | 0.248 | `path_dominant` |
| `town01` | `turn` | 20.43 | 0.549 | 7.120 | 0.514/0.639 | 0.248 | `path_dominant` |
| `town02_opt` | `straight` | 27.87 | 0.470 | 0.490 | 0.334/0.465 | 0.251 | `within_thresholds` |
| `town02_opt` | `turn` | 20.17 | 0.601 | 14.189 | 0.734/0.845 | 0.250 | `path_dominant` |
| `town03` | `straight` | 27.69 | 0.503 | 2.206 | 0.188/0.382 | 0.248 | `path_dominant` |
| `town03` | `turn` | 18.06 | 0.501 | 8.767 | 0.600/0.808 | 0.249 | `path_dominant` |
| `town04` | `straight` | 27.76 | 0.382 | 0.348 | 0.086/0.315 | 0.251 | `within_thresholds` |
| `town04` | `turn` | 18.49 | 0.632 | 5.969 | 0.654/0.808 | 0.250 | `path_dominant` |
| `town05_opt` | `straight` | 27.83 | 0.513 | 0.939 | 0.192/0.334 | 0.245 | `path_dominant` |
| `town05_opt` | `turn` | 18.31 | 0.562 | 3.851 | 0.620/0.734 | 0.244 | `path_dominant` |
| `town06` | `straight` | 27.94 | 0.546 | 0.339 | 0.236/0.365 | 0.248 | `within_thresholds` |
| `town06` | `turn` | 24.02 | 0.627 | 6.581 | 0.703/0.808 | 0.248 | `path_dominant` |
| `town07` | `straight` | 27.76 | 0.534 | 1.617 | 0.355/0.431 | 0.250 | `path_dominant` |
| `town07` | `turn` | 19.29 | 0.484 | 8.553 | 0.599/0.680 | 0.245 | `path_dominant` |
| `town10hd_opt` | `straight` | 27.76 | 0.510 | 1.526 | 0.159/0.365 | 0.248 | `path_dominant` |
| `town10hd_opt` | `turn` | 18.80 | 0.735 | 13.286 | 0.918/0.991 | 0.248 | `mixed_path_and_control` |

### 전체화면 빠른 확인: 커스텀 CTrack 직진·좌회전

직진과 좌회전은 서로 다른 trial이며, 아래 이미지는 각 route 평가의 정확한 시간
중간점에서 owned RViz 창만 추출한 1920x1080 PNG다. 전체 9개 맵의 18개 PNG/GIF와
경로 분석은 아래 `Published PASS trials` 표에 각각 링크되어 있다.

| 직진 | 좌회전 |
|---|---|
| [![CTrack straight full Autoware](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/autoware_rviz_fullscreen.png)](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/autoware_rviz_fullscreen.png) | [![CTrack left turn full Autoware](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/autoware_rviz_fullscreen.png)](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/autoware_rviz_fullscreen.png) |

### 화면이 끊겨 보이는 이유

입증된 직접 메커니즘은 camera의 **wall-time 유효 갱신율**이다. CARLA physics는
simulation 기준 20 Hz이고, 6대 camera source는 `sensor_tick=0.0`이라 매 physics
frame에 렌더된다. bridge는 같은 CARLA frame stamp의 완전한 6-camera bundle을
simulation 기준 5 Hz로 발행한다. v16 RTF가 `0.244-0.251`이므로 실제 시간에서 front
camera receipt는 약 `1.22-1.24 Hz`, 즉 약 `0.8초`마다 새 영상으로 보인다. VAD
inference는 대개 `34-39 ms`, candidate acceptance는 약 100%, coalesced drop은 0이었다.
odometry와 control의 simulation-time 최대 간격도 각각 `0.1/0.05 s`여서 controller가
wall time으로 0.8초씩 명령을 멈춘 증거는 없다. 현재 보이는 현상은 주로 camera frame
hold에 의한 시각적 계단감이다.

v16 실행 중 확인한 12회 resource 표본은 CPU idle 중앙값 84%, GPU utilization 중앙값
10%, swap 0이었다. 이 표본의 원시 로그와 timestamp/수집 명령은 발행하지 않았으므로
재현 가능한 root-cause 증거가 아니라 보조 관찰로만 쓴다. 지속적인 호스트 전체 포화
증거는 없지만 순간적인 다른 process 간섭, CARLA render thread 포화 또는 국소 직렬
대기를 배제하지는 않는다. 따라서 낮은 RTF의 선행 가설은 synchronous six-camera
render와 reliable exact-bundle 경로이지만, 그 인과는 아직 입증되지 않았다.

다음 성능 A/B는 동일 route/weather/Epic/UI/physics 조건에서 현행 reliable+IMU mapping을
복제하고 camera source `sensor_tick`만 `0.0 -> 0.2`로 바꾼다. ROS publish는 5 Hz로
유지하고 route PASS, six-camera coverage `>=99%`, candidate acceptance 100%, drop 0과
RTF를 함께 비교한다. 기존 `sensor_mapping_vad_fast.yaml`은 best-effort이고 IMU가 없어
단일변수 대조군으로 쓰지 않는다. 이 A/B 전에 ROS camera publish Hz부터 올리지는 않는다.

### Town10HD 안전 이력

v14의 동일 우회전 경로에서는 신뢰도 `0.512` CAR 객체 한 frame이 AEB/MRM을 정상
작동시켜 17/18로 끝났다. 같은 source-route SHA의 v15와 최종 v16에서는 주행 중 RSS,
AEB brake/virtual wall, MRM 개입이 모두 0이고 직진·우회전이 PASS했다. v16 turn에는
dynamic forward CAR가 없었다. v16 straight에는 off-corridor 한 frame velocity spike가
있었지만 OBB lateral overlap이 없고 안전 개입도 없었다. 따라서 오탐 재발 가능성을
0으로 보지 않으며 confidence threshold를 일괄 상향하지 않고, 후속 temporal
confirmation을 별도 safety A/B로 검증한다.

### 호스트·발행 검증

- 전체 repository pytest: `698 passed, 1 skipped`
- `autoware_e2e_vad_launch` build/CTest: `13/13` target PASS,
  xUnit 합계 `245 tests, 0 failures`
- owned-window 시각 감사: `PASS`, 9 maps / 18 trials, 대표 PNG의 MKV frame
  pixel-exact 재추출과 전체 MKV/GIF decode/변화 검사 통과
- 발행 자산: `337 files`; `SHA256SUMS` 336줄 전체 PASS
- 발행된 전체 장면 시각 감사:
  [JSON](assets/validation/2026-09-01/owned_window_visual_audit/v16_owned_window_visual_audit.json),
  [Markdown](assets/validation/2026-09-01/owned_window_visual_audit/v16_owned_window_visual_audit.md),
  [review JSON](assets/validation/2026-09-01/owned_window_visual_audit/v16_owned_window_visual_review.json),
  [contact sheet](assets/validation/2026-09-01/owned_window_visual_audit/v16_owned_window_contact_sheet.png)

이 발행은 2026-08-31 BasicAgent aggregate와 VS Code IME proof를 재사용하고,
2026-09-01 v16 Autoware VAD matrix를 결합한다. 호스트에는 `/home/a/.local/bin/code`
wrapper와 사용자 desktop launcher를 적용했다. 이미 실행 중인 Electron process는 새
환경을 상속하지 않으므로 모든 VS Code 창을 닫은 뒤 해당 wrapper로 한 번 다시 실행해야
한글 입력 설정이 적용된다.

원본 matrix validation은 route/result/speed/bag/lifecycle/visual/config provenance를
결합한다. 일부 후처리 analysis 파일은 원본 validation에서 존재·내용만 검사하고 별도
digest field를 두지 않지만, 발행본에서는 모든 published analysis 파일을 최상위
`SHA256SUMS`가 결합한다. 미래 matrix schema에서는 후처리 digest도 원본
`matrix_validation.json`에 직접 넣는 것이 남은 provenance 개선점이다.
