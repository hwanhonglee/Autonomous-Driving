# Validation evidence

> The following campaign summary is an operator-reviewed, SHA-bound narrative supplement. The generated provenance and result tables below remain authoritative.

> 2026-09-02 후속 검증: CTrack A/B를 거쳐 여섯 CARLA camera source를 5 Hz로
> 제한한 30 kph 전체 9-map/18-trial 결과는
> [`validation-2026-09-02-all-towns-camera-source-5hz.md`](validation-2026-09-02-all-towns-camera-source-5hz.md),
> CTrack A/B와 끊김 원인 분석은
> [`validation-2026-09-02-camera-cadence.md`](validation-2026-09-02-camera-cadence.md)에
> 있다. 아래 내용은 camera-source 변경 전 `speed_30kph` 기준선으로 보존한다.

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

Source: [archived CARLA BasicAgent aggregate JSON](assets/validation/2026-09-01/carla_basicagent_sweep_aggregate.json) (`sha256:e2b1da4cba11f0ae5b663d6783737d6819149441637b8cc0a7768976ce609dfb`) — overall **COMPLETE**, selected success **9/9**.

> Scope boundary: the all-map sweep and `expert_*` media are CARLA BasicAgent six-camera route-smoke evidence. They are not Autoware VAD inference or closed-loop-control evidence.

![All-map CARLA BasicAgent status](assets/validation/2026-09-01/all_maps_basicagent_status_1920x1080.png)

## CARLA BasicAgent packaged-map sweep

| Map | Status | Stage | Published media |
|---|---|---|---|
| `town01` (Town01) | **PASS** | complete | [PNG 1920x1080](assets/validation/2026-09-01/town01/expert_overview_1920x1080.png), [GIF 960x540](assets/validation/2026-09-01/town01/expert_drive.gif) |
| `town02` (Town02) | **EXCLUDED** | not_run | — |
| `town02_opt` (Town02_Opt) | **SKIP_RESUME_VALIDATED** | resume_validated | [PNG 1920x1080](assets/validation/2026-09-01/town02_opt/expert_overview_1920x1080.png), [GIF 960x540](assets/validation/2026-09-01/town02_opt/expert_drive.gif) |
| `town03` (Town03) | **SKIP_RESUME_VALIDATED** | resume_validated | [PNG 1920x1080](assets/validation/2026-09-01/town03/expert_overview_1920x1080.png), [GIF 960x540](assets/validation/2026-09-01/town03/expert_drive.gif) |
| `town04` (Town04) | **SKIP_RESUME_VALIDATED** | resume_validated | [PNG 1920x1080](assets/validation/2026-09-01/town04/expert_overview_1920x1080.png), [GIF 960x540](assets/validation/2026-09-01/town04/expert_drive.gif) |
| `town05` (Town05) | **EXCLUDED** | not_run | — |
| `town05_opt` (Town05_Opt) | **SKIP_RESUME_VALIDATED** | resume_validated | [PNG 1920x1080](assets/validation/2026-09-01/town05_opt/expert_overview_1920x1080.png), [GIF 960x540](assets/validation/2026-09-01/town05_opt/expert_drive.gif) |
| `town06` (Town06) | **SKIP_RESUME_VALIDATED** | resume_validated | [PNG 1920x1080](assets/validation/2026-09-01/town06/expert_overview_1920x1080.png), [GIF 960x540](assets/validation/2026-09-01/town06/expert_drive.gif) |
| `town07` (Town07) | **SKIP_RESUME_VALIDATED** | resume_validated | [PNG 1920x1080](assets/validation/2026-09-01/town07/expert_overview_1920x1080.png), [GIF 960x540](assets/validation/2026-09-01/town07/expert_drive.gif) |
| `town08` (Town08) | **EXCLUDED** | not_run | — |
| `town09` (Town09) | **EXCLUDED** | not_run | — |
| `town10hd` (Town10HD) | **EXCLUDED** | not_run | — |
| `town10hd_opt` (Town10HD_Opt) | **SKIP_RESUME_VALIDATED** | resume_validated | [PNG 1920x1080](assets/validation/2026-09-01/town10hd_opt/expert_overview_1920x1080.png), [GIF 960x540](assets/validation/2026-09-01/town10hd_opt/expert_drive.gif) |
| `town11` (Town11) | **SOURCE_EDITOR_REQUIRED** | not_run | — |
| `town12` (Town12) | **SOURCE_EDITOR_REQUIRED** | not_run | — |
| `town13` (Town13) | **SOURCE_EDITOR_REQUIRED** | not_run | — |
| `town15` (Town15) | **SOURCE_EDITOR_REQUIRED** | not_run | — |
| `c_track_1_0_7` (C_track_1_0_7) | **SKIP_RESUME_VALIDATED** | resume_validated | [PNG 1920x1080](assets/validation/2026-09-01/c_track_1_0_7/expert_overview_1920x1080.png), [GIF 960x540](assets/validation/2026-09-01/c_track_1_0_7/expert_drive.gif) |
| `woraksan_1_0_3` (Woraksan_v1_0_3_parking_lot_hegiht_fit) | **BLOCKED** | not_run | — |

Only PASS/SKIP_RESUME_VALIDATED rows with a complete episode, validated export, matching manifests, and all six camera images are published.

## Autoware VAD full-stack trials

> **30 kph evidence boundary — simulation screening only.** `carla_vad_30kph_v2` evaluates VAD candidate geometry through `vad_route_manager_hybrid` while an explicit CARLA simulation speed overlay supplies longitudinal cruise velocity. It does not evaluate raw VAD cruise velocity and `real_vehicle_ready=false`. A PASS is therefore not a real-vehicle-readiness claim and does not by itself claim that measured speed reached exactly 30 kph.

Matrix campaign status: **COMPLETE**; runnable map PASS **9/9**; BLOCKED=10, PASS=9.

Provenance: [archived matrix plan JSON](assets/validation/2026-09-01/autoware_vad_matrix_plan.json) (`sha256:572ca8aa537491e8bb1c29092a9e4ef494484764691c72516559cf9d16e64c1c`, generated `2026-09-01T11:55:19.562950+00:00`); [archived terminal aggregate JSON](assets/validation/2026-09-01/autoware_vad_matrix_aggregate.json) (`sha256:48de1db07c7fd8d031c0f51f81a2f027b760a744499b34b138fc15f554168685`, generated `2026-09-01T13:44:14.732415+00:00`).

The terminal table below is immutable publication provenance. Matrix PASS trials are revalidated from status + `matrix_validation.json`. Explicit supplemental PASS rows are reported separately with capture timing; they never change a matrix `FAILED` or `BLOCKED` row.

Map identities are not aliases: `town05` (Town05) remains distinct from `town05_opt` (Town05_Opt), and `town10hd` (Town10HD) remains distinct from `town10hd_opt` (Town10HD_Opt). Evidence for an optimized variant does not validate its standard-map counterpart.

### Original terminal matrix: all 19 maps

| Map | Runnable | Matrix status | Stage | Straight | Turn | Recorded reason |
|---|---|---|---|---|---|---|
| `town01` (Town01) | yes | **PASS** | complete | PASS | PASS | Straight and turn full-stack trials both passed. |
| `town02` (Town02) | no | **BLOCKED** | admission | BLOCKED | BLOCKED | No locally validated Lanelet2 + PCD full-map bundle is declared; CARLA/BasicAgent assets alone do not authorize an Autoware VAD closed-loop claim. Suite status=unavailable: Runtime validation showed that the standard packaged Town02 renderer crashes; use town02_opt for collection. |
| `town02_opt` (Town02_Opt) | yes | **PASS** | complete | PASS | PASS | Straight and turn full-stack trials both passed. |
| `town03` (Town03) | yes | **PASS** | complete | PASS | PASS | Straight and turn full-stack trials both passed. |
| `town04` (Town04) | yes | **PASS** | complete | PASS | PASS | Straight and turn full-stack trials both passed. |
| `town05` (Town05) | no | **BLOCKED** | admission | BLOCKED | BLOCKED | No locally validated Lanelet2 + PCD full-map bundle is declared; CARLA/BasicAgent assets alone do not authorize an Autoware VAD closed-loop claim. Suite status=unavailable: Packaged expert collection crashes in SceneCapture2D skeletal-mesh rendering; use town05_opt. |
| `town05_opt` (Town05_Opt) | yes | **PASS** | complete | PASS | PASS | Straight and turn full-stack trials both passed. |
| `town06` (Town06) | yes | **PASS** | complete | PASS | PASS | Straight and turn full-stack trials both passed. |
| `town07` (Town07) | yes | **PASS** | complete | PASS | PASS | Straight and turn full-stack trials both passed. |
| `town08` (Town08) | no | **BLOCKED** | admission | BLOCKED | BLOCKED | No locally validated Lanelet2 + PCD full-map bundle is declared; CARLA/BasicAgent assets alone do not authorize an Autoware VAD closed-loop claim. Suite status=unavailable: Town08 is a non-distributed Leaderboard unseen map and no local level or OpenDRIVE asset exists. |
| `town09` (Town09) | no | **BLOCKED** | admission | BLOCKED | BLOCKED | No locally validated Lanelet2 + PCD full-map bundle is declared; CARLA/BasicAgent assets alone do not authorize an Autoware VAD closed-loop claim. Suite status=unavailable: Town09 is a non-distributed Leaderboard unseen map and no local level or OpenDRIVE asset exists. |
| `town10hd` (Town10HD) | no | **BLOCKED** | admission | BLOCKED | BLOCKED | No locally validated Lanelet2 + PCD full-map bundle is declared; CARLA/BasicAgent assets alone do not authorize an Autoware VAD closed-loop claim. Suite status=unavailable: Standard Town10HD is unverified and the packaged server boots Town10HD_Opt instead; use town10hd_opt for collection. |
| `town10hd_opt` (Town10HD_Opt) | yes | **PASS** | complete | PASS | PASS | Straight and turn full-stack trials both passed. |
| `town11` (Town11) | no | **BLOCKED** | admission | BLOCKED | BLOCKED | No locally validated Lanelet2 + PCD full-map bundle is declared; CARLA/BasicAgent assets alone do not authorize an Autoware VAD closed-loop claim. Suite status=source_editor_required: The complete source map exists, but it is an undecorated large-map proof of concept and is excluded from visual training. |
| `town12` (Town12) | no | **BLOCKED** | admission | BLOCKED | BLOCKED | No locally validated Lanelet2 + PCD full-map bundle is declared; CARLA/BasicAgent assets alone do not authorize an Autoware VAD closed-loop claim. Suite status=source_editor_required: Complete level and OpenDRIVE assets exist only in the source checkout used by UE4Editor. |
| `town13` (Town13) | no | **BLOCKED** | admission | BLOCKED | BLOCKED | No locally validated Lanelet2 + PCD full-map bundle is declared; CARLA/BasicAgent assets alone do not authorize an Autoware VAD closed-loop claim. Suite status=source_editor_required: Complete level and OpenDRIVE assets exist only in the source checkout used by UE4Editor. |
| `town15` (Town15) | no | **BLOCKED** | admission | BLOCKED | BLOCKED | No locally validated Lanelet2 + PCD full-map bundle is declared; CARLA/BasicAgent assets alone do not authorize an Autoware VAD closed-loop claim. Suite status=source_editor_required: This additional campus map is held out and requires the complete source-editor asset set. |
| `c_track_1_0_7` (C_track_1_0_7) | yes | **PASS** | complete | PASS | PASS | Straight and turn full-stack trials both passed. |
| `woraksan_1_0_3` (Woraksan_v1_0_3_parking_lot_hegiht_fit) | no | **BLOCKED** | admission | BLOCKED | BLOCKED | The canonical manifest entry is retained, but the current packaged CARLA installation lacks the required Woraksan runtime assets and no admitted Autoware full-map bundle is present. |

### Final paired coverage across executable variants

This is a union of status-backed matrix PASS trials and separately validated explicit supplements. It is a publication coverage view, not a rewritten matrix result. Only exact `straight` and `turn` trial IDs fill the paired slots; the historical C-track `lane_follow` row is supplemental and does not count as C-track straight coverage.

| Executable map variant | Straight | Turn | Final paired outcome |
|---|---|---|---|
| `town01` (Town01) | PASS (matrix) | PASS (matrix) | **PASS/PASS** |
| `town02_opt` (Town02_Opt) | PASS (matrix) | PASS (matrix) | **PASS/PASS** |
| `town03` (Town03) | PASS (matrix) | PASS (matrix) | **PASS/PASS** |
| `town04` (Town04) | PASS (matrix) | PASS (matrix) | **PASS/PASS** |
| `town05_opt` (Town05_Opt) | PASS (matrix) | PASS (matrix) | **PASS/PASS** |
| `town06` (Town06) | PASS (matrix) | PASS (matrix) | **PASS/PASS** |
| `town07` (Town07) | PASS (matrix) | PASS (matrix) | **PASS/PASS** |
| `town10hd_opt` (Town10HD_Opt) | PASS (matrix) | PASS (matrix) | **PASS/PASS** |
| `c_track_1_0_7` (C_track_1_0_7) | PASS (matrix) | PASS (matrix) | **PASS/PASS** |

### Published PASS trials

Every row passed the full-stack, route-assisted HYBRID `vad_route_manager_hybrid` evaluator; these rows are not unassisted end-to-end VAD claims. Every labeled row also requires a candidate-gated owned-window Autoware/RViz capture and recorded route-analysis bundle.

Timing compares `desktop_capture.json.captured_at` with the archived matrix plan and terminal aggregate timestamps for ordinary rows. For a visual-refresh row, timing remains tied to the original authoritative validation capture, which is retained separately in the manifest and archive. `after terminal matrix` is post-matrix evidence; `during matrix window (external)` and `before matrix plan` are still explicit supplements outside the matrix status chain. None rewrites the terminal table above.

| Map | Trial | Source and matrix relation | Evidence timing | Route | Result | Owned RViz window | Drive GIF | Route/control analysis |
|---|---|---|---|---|---|---|---|---|
| `c_track_1_0_7` | `straight` | matrix PASS | matrix status-backed (`2026-09-01T13:35:45.296952+00:00`) | `189→377` (straight, 209.47 m; [JSON](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/diagnosis.json), [tracking](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/path_vs_control.png), [steering](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/steering_tracking.png), [latency](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/speed_profile.json), [speed plot](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/speed_profile.png), observed max `7.740 m/s` (`27.86 kph`), [capture runtime](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/runtime.env), [pinned RViz config](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/straight/rviz_capture_provenance/SHA256SUMS) |
| `c_track_1_0_7` | `turn` | matrix PASS | matrix status-backed (`2026-09-01T13:41:24.786773+00:00`) | `71→171` (left, 73.86 m; [JSON](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/diagnosis.json), [tracking](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/path_vs_control.png), [steering](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/steering_tracking.png), [latency](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/speed_profile.json), [speed plot](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/speed_profile.png), observed max `4.895 m/s` (`17.62 kph`), [capture runtime](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/runtime.env), [pinned RViz config](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/c_track_1_0_7/autoware_vad/turn/rviz_capture_provenance/SHA256SUMS) |
| `town01` | `straight` | matrix PASS | matrix status-backed (`2026-09-01T11:57:57.342397+00:00`) | `waypoint 116→143` (straight, 210.18 m; [JSON](assets/validation/2026-09-01/town01/autoware_vad/straight/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town01/autoware_vad/straight/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town01/autoware_vad/straight/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town01/autoware_vad/straight/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town01/autoware_vad/straight/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town01/autoware_vad/straight/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town01/autoware_vad/straight/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town01/autoware_vad/straight/diagnosis.json), [tracking](assets/validation/2026-09-01/town01/autoware_vad/straight/path_vs_control.png), [steering](assets/validation/2026-09-01/town01/autoware_vad/straight/steering_tracking.png), [latency](assets/validation/2026-09-01/town01/autoware_vad/straight/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town01/autoware_vad/straight/speed_profile.json), [speed plot](assets/validation/2026-09-01/town01/autoware_vad/straight/speed_profile.png), observed max `7.740 m/s` (`27.86 kph`), [capture runtime](assets/validation/2026-09-01/town01/autoware_vad/straight/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town01/autoware_vad/straight/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town01/autoware_vad/straight/rviz_capture_provenance/SHA256SUMS) |
| `town01` | `turn` | matrix PASS | matrix status-backed (`2026-09-01T12:03:08.559021+00:00`) | `28→22` (left, 81.27 m; [JSON](assets/validation/2026-09-01/town01/autoware_vad/turn/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town01/autoware_vad/turn/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town01/autoware_vad/turn/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town01/autoware_vad/turn/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town01/autoware_vad/turn/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town01/autoware_vad/turn/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town01/autoware_vad/turn/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town01/autoware_vad/turn/diagnosis.json), [tracking](assets/validation/2026-09-01/town01/autoware_vad/turn/path_vs_control.png), [steering](assets/validation/2026-09-01/town01/autoware_vad/turn/steering_tracking.png), [latency](assets/validation/2026-09-01/town01/autoware_vad/turn/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town01/autoware_vad/turn/speed_profile.json), [speed plot](assets/validation/2026-09-01/town01/autoware_vad/turn/speed_profile.png), observed max `5.674 m/s` (`20.43 kph`), [capture runtime](assets/validation/2026-09-01/town01/autoware_vad/turn/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town01/autoware_vad/turn/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town01/autoware_vad/turn/rviz_capture_provenance/SHA256SUMS) |
| `town02_opt` | `straight` | matrix PASS | matrix status-backed (`2026-09-01T12:09:05.784590+00:00`) | `waypoint 40→42` (straight, 177.13 m; [JSON](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/diagnosis.json), [tracking](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/path_vs_control.png), [steering](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/steering_tracking.png), [latency](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/speed_profile.json), [speed plot](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/speed_profile.png), observed max `7.742 m/s` (`27.87 kph`), [capture runtime](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town02_opt/autoware_vad/straight/rviz_capture_provenance/SHA256SUMS) |
| `town02_opt` | `turn` | matrix PASS | matrix status-backed (`2026-09-01T12:13:56.633433+00:00`) | `8→53` (left, 83.61 m; [JSON](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/diagnosis.json), [tracking](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/path_vs_control.png), [steering](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/steering_tracking.png), [latency](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/speed_profile.json), [speed plot](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/speed_profile.png), observed max `5.604 m/s` (`20.17 kph`), [capture runtime](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town02_opt/autoware_vad/turn/rviz_capture_provenance/SHA256SUMS) |
| `town03` | `straight` | matrix PASS | matrix status-backed (`2026-09-01T12:20:16.123202+00:00`) | `waypoint 188→225` (straight, 209.52 m; [JSON](assets/validation/2026-09-01/town03/autoware_vad/straight/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town03/autoware_vad/straight/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town03/autoware_vad/straight/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town03/autoware_vad/straight/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town03/autoware_vad/straight/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town03/autoware_vad/straight/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town03/autoware_vad/straight/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town03/autoware_vad/straight/diagnosis.json), [tracking](assets/validation/2026-09-01/town03/autoware_vad/straight/path_vs_control.png), [steering](assets/validation/2026-09-01/town03/autoware_vad/straight/steering_tracking.png), [latency](assets/validation/2026-09-01/town03/autoware_vad/straight/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town03/autoware_vad/straight/speed_profile.json), [speed plot](assets/validation/2026-09-01/town03/autoware_vad/straight/speed_profile.png), observed max `7.692 m/s` (`27.69 kph`), [capture runtime](assets/validation/2026-09-01/town03/autoware_vad/straight/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town03/autoware_vad/straight/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town03/autoware_vad/straight/rviz_capture_provenance/SHA256SUMS) |
| `town03` | `turn` | matrix PASS | matrix status-backed (`2026-09-01T12:25:44.284119+00:00`) | `236→92` (left, 73.12 m; [JSON](assets/validation/2026-09-01/town03/autoware_vad/turn/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town03/autoware_vad/turn/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town03/autoware_vad/turn/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town03/autoware_vad/turn/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town03/autoware_vad/turn/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town03/autoware_vad/turn/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town03/autoware_vad/turn/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town03/autoware_vad/turn/diagnosis.json), [tracking](assets/validation/2026-09-01/town03/autoware_vad/turn/path_vs_control.png), [steering](assets/validation/2026-09-01/town03/autoware_vad/turn/steering_tracking.png), [latency](assets/validation/2026-09-01/town03/autoware_vad/turn/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town03/autoware_vad/turn/speed_profile.json), [speed plot](assets/validation/2026-09-01/town03/autoware_vad/turn/speed_profile.png), observed max `5.015 m/s` (`18.06 kph`), [capture runtime](assets/validation/2026-09-01/town03/autoware_vad/turn/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town03/autoware_vad/turn/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town03/autoware_vad/turn/rviz_capture_provenance/SHA256SUMS) |
| `town04` | `straight` | matrix PASS | matrix status-backed (`2026-09-01T12:34:45.263122+00:00`) | `waypoint 1408→1435` (straight, 210.01 m; [JSON](assets/validation/2026-09-01/town04/autoware_vad/straight/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town04/autoware_vad/straight/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town04/autoware_vad/straight/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town04/autoware_vad/straight/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town04/autoware_vad/straight/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town04/autoware_vad/straight/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town04/autoware_vad/straight/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town04/autoware_vad/straight/diagnosis.json), [tracking](assets/validation/2026-09-01/town04/autoware_vad/straight/path_vs_control.png), [steering](assets/validation/2026-09-01/town04/autoware_vad/straight/steering_tracking.png), [latency](assets/validation/2026-09-01/town04/autoware_vad/straight/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town04/autoware_vad/straight/speed_profile.json), [speed plot](assets/validation/2026-09-01/town04/autoware_vad/straight/speed_profile.png), observed max `7.712 m/s` (`27.76 kph`), [capture runtime](assets/validation/2026-09-01/town04/autoware_vad/straight/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town04/autoware_vad/straight/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town04/autoware_vad/straight/rviz_capture_provenance/SHA256SUMS) |
| `town04` | `turn` | matrix PASS | matrix status-backed (`2026-09-01T12:40:04.221842+00:00`) | `363→166` (left, 70.39 m; [JSON](assets/validation/2026-09-01/town04/autoware_vad/turn/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town04/autoware_vad/turn/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town04/autoware_vad/turn/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town04/autoware_vad/turn/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town04/autoware_vad/turn/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town04/autoware_vad/turn/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town04/autoware_vad/turn/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town04/autoware_vad/turn/diagnosis.json), [tracking](assets/validation/2026-09-01/town04/autoware_vad/turn/path_vs_control.png), [steering](assets/validation/2026-09-01/town04/autoware_vad/turn/steering_tracking.png), [latency](assets/validation/2026-09-01/town04/autoware_vad/turn/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town04/autoware_vad/turn/speed_profile.json), [speed plot](assets/validation/2026-09-01/town04/autoware_vad/turn/speed_profile.png), observed max `5.136 m/s` (`18.49 kph`), [capture runtime](assets/validation/2026-09-01/town04/autoware_vad/turn/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town04/autoware_vad/turn/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town04/autoware_vad/turn/rviz_capture_provenance/SHA256SUMS) |
| `town05_opt` | `straight` | matrix PASS | matrix status-backed (`2026-09-01T12:47:17.510886+00:00`) | `waypoint 547→1645` (straight, 209.77 m; [JSON](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/diagnosis.json), [tracking](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/path_vs_control.png), [steering](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/steering_tracking.png), [latency](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/speed_profile.json), [speed plot](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/speed_profile.png), observed max `7.730 m/s` (`27.83 kph`), [capture runtime](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town05_opt/autoware_vad/straight/rviz_capture_provenance/SHA256SUMS) |
| `town05_opt` | `turn` | matrix PASS | matrix status-backed (`2026-09-01T12:52:44.242481+00:00`) | `103→213` (left, 75.02 m; [JSON](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/diagnosis.json), [tracking](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/path_vs_control.png), [steering](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/steering_tracking.png), [latency](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/speed_profile.json), [speed plot](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/speed_profile.png), observed max `5.086 m/s` (`18.31 kph`), [capture runtime](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town05_opt/autoware_vad/turn/rviz_capture_provenance/SHA256SUMS) |
| `town06` | `straight` | matrix PASS | matrix status-backed (`2026-09-01T13:03:28.467873+00:00`) | `waypoint 231→694` (straight, 210.48 m; [JSON](assets/validation/2026-09-01/town06/autoware_vad/straight/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town06/autoware_vad/straight/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town06/autoware_vad/straight/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town06/autoware_vad/straight/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town06/autoware_vad/straight/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town06/autoware_vad/straight/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town06/autoware_vad/straight/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town06/autoware_vad/straight/diagnosis.json), [tracking](assets/validation/2026-09-01/town06/autoware_vad/straight/path_vs_control.png), [steering](assets/validation/2026-09-01/town06/autoware_vad/straight/steering_tracking.png), [latency](assets/validation/2026-09-01/town06/autoware_vad/straight/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town06/autoware_vad/straight/speed_profile.json), [speed plot](assets/validation/2026-09-01/town06/autoware_vad/straight/speed_profile.png), observed max `7.761 m/s` (`27.94 kph`), [capture runtime](assets/validation/2026-09-01/town06/autoware_vad/straight/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town06/autoware_vad/straight/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town06/autoware_vad/straight/rviz_capture_provenance/SHA256SUMS) |
| `town06` | `turn` | matrix PASS | matrix status-backed (`2026-09-01T13:08:35.029172+00:00`) | `325→271` (right, 88.11 m; [JSON](assets/validation/2026-09-01/town06/autoware_vad/turn/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town06/autoware_vad/turn/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town06/autoware_vad/turn/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town06/autoware_vad/turn/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town06/autoware_vad/turn/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town06/autoware_vad/turn/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town06/autoware_vad/turn/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town06/autoware_vad/turn/diagnosis.json), [tracking](assets/validation/2026-09-01/town06/autoware_vad/turn/path_vs_control.png), [steering](assets/validation/2026-09-01/town06/autoware_vad/turn/steering_tracking.png), [latency](assets/validation/2026-09-01/town06/autoware_vad/turn/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town06/autoware_vad/turn/speed_profile.json), [speed plot](assets/validation/2026-09-01/town06/autoware_vad/turn/speed_profile.png), observed max `6.672 m/s` (`24.02 kph`), [capture runtime](assets/validation/2026-09-01/town06/autoware_vad/turn/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town06/autoware_vad/turn/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town06/autoware_vad/turn/rviz_capture_provenance/SHA256SUMS) |
| `town07` | `straight` | matrix PASS | matrix status-backed (`2026-09-01T13:13:55.517941+00:00`) | `waypoint 51→70` (straight, 210.60 m; [JSON](assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town07/autoware_vad/straight/diagnosis.json), [tracking](assets/validation/2026-09-01/town07/autoware_vad/straight/path_vs_control.png), [steering](assets/validation/2026-09-01/town07/autoware_vad/straight/steering_tracking.png), [latency](assets/validation/2026-09-01/town07/autoware_vad/straight/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town07/autoware_vad/straight/speed_profile.json), [speed plot](assets/validation/2026-09-01/town07/autoware_vad/straight/speed_profile.png), observed max `7.710 m/s` (`27.76 kph`), [capture runtime](assets/validation/2026-09-01/town07/autoware_vad/straight/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town07/autoware_vad/straight/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town07/autoware_vad/straight/rviz_capture_provenance/SHA256SUMS) |
| `town07` | `turn` | matrix PASS | matrix status-backed (`2026-09-01T13:19:13.204395+00:00`) | `76→3` (left, 82.61 m; [JSON](assets/validation/2026-09-01/town07/autoware_vad/turn/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town07/autoware_vad/turn/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town07/autoware_vad/turn/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town07/autoware_vad/turn/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town07/autoware_vad/turn/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town07/autoware_vad/turn/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town07/autoware_vad/turn/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town07/autoware_vad/turn/diagnosis.json), [tracking](assets/validation/2026-09-01/town07/autoware_vad/turn/path_vs_control.png), [steering](assets/validation/2026-09-01/town07/autoware_vad/turn/steering_tracking.png), [latency](assets/validation/2026-09-01/town07/autoware_vad/turn/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town07/autoware_vad/turn/speed_profile.json), [speed plot](assets/validation/2026-09-01/town07/autoware_vad/turn/speed_profile.png), observed max `5.358 m/s` (`19.29 kph`), [capture runtime](assets/validation/2026-09-01/town07/autoware_vad/turn/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town07/autoware_vad/turn/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town07/autoware_vad/turn/rviz_capture_provenance/SHA256SUMS) |
| `town10hd_opt` | `straight` | matrix PASS | matrix status-backed (`2026-09-01T13:24:50.627669+00:00`) | `waypoint 5207→1220` (straight, 171.22 m; [JSON](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/diagnosis.json), [tracking](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/path_vs_control.png), [steering](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/steering_tracking.png), [latency](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/speed_profile.json), [speed plot](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/speed_profile.png), observed max `7.712 m/s` (`27.76 kph`), [capture runtime](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town10hd_opt/autoware_vad/straight/rviz_capture_provenance/SHA256SUMS) |
| `town10hd_opt` | `turn` | matrix PASS | matrix status-backed (`2026-09-01T13:29:43.477003+00:00`) | `17→28` (right, 79.76 m; [JSON](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/autoware_vad_route.json)) | [JSON](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/autoware_vad_result.json) | [PNG 1920x1080](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/autoware_rviz_fullscreen.png), [vehicle-centered candidate still](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/autoware_rviz_candidate.png) | [GIF 960x540](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/autoware_rviz_drive.gif) | [route PNG](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/autoware_vad_route_result.png), [control GIF](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/autoware_vad_turn_path_control.gif), [diagnosis](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/diagnosis.json), [tracking](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/path_vs_control.png), [steering](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/steering_tracking.png), [latency](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/latency/e2e_latency.json), [speed-source JSON](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/speed_profile.json), [speed plot](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/speed_profile.png), observed max `5.222 m/s` (`18.80 kph`), [capture runtime](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/runtime.env), [pinned RViz config](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/rviz_capture_provenance/autoware_vad_carla.rviz), [RViz checksum](assets/validation/2026-09-01/town10hd_opt/autoware_vad/turn/rviz_capture_provenance/SHA256SUMS) |

## Owned-window vehicle-centered visual audit

**PASS**: 9 maps / 18 straight+turn trials passed both the mechanical centered-frame contract and the operator visual review. Every representative PNG is pixel-exact with its route-midpoint recording frame.

[Contact sheet 1920x2412](assets/validation/2026-09-01/owned_window_visual_audit/v16_owned_window_contact_sheet.png), [audit JSON](assets/validation/2026-09-01/owned_window_visual_audit/v16_owned_window_visual_audit.json), [audit Markdown](assets/validation/2026-09-01/owned_window_visual_audit/v16_owned_window_visual_audit.md), [operator review JSON](assets/validation/2026-09-01/owned_window_visual_audit/v16_owned_window_visual_review.json).

![All owned-window Autoware VAD scenes](assets/validation/2026-09-01/owned_window_visual_audit/v16_owned_window_contact_sheet.png)

## VS Code Korean IME desktop proof

These screenshots document the editor input-method fix only. They are not CARLA driving or Autoware VAD evidence.

- [Before / ASCII input](assets/validation/2026-09-01/vscode/vscode_ime_before_ascii.png) (1200x800)
- [After / Hangul input](assets/validation/2026-09-01/vscode/vscode_ime_after_hangul.png) (1200x800)

## Integrity and reproduction

Run the checksum command from the published asset directory:

```bash
cd /home/a/autoware_e2e/docs/assets/validation/2026-09-01
sha256sum -c SHA256SUMS
```

Rebuild with:

```bash
python3 scripts/e2e/publish_validation_assets.py \
  --artifact-root /home/a/autoware_e2e/artifacts/validation/2026-08-31 \
  --docs-assets-root /home/a/autoware_e2e/docs/assets/validation/2026-09-01 \
  --expected-map-count 19 \
  --expected-selected-map-count 9 \
  --report /home/a/autoware_e2e/docs/validation-2026-09-01.md \
  --report-preamble /home/a/autoware_e2e/docs/validation-2026-09-01-summary.inc.md \
  --owned-window-visual-audit-dir /home/a/autoware_e2e/artifacts/validation/2026-09-01 \
  --vad-matrix-root /home/a/autoware_e2e/artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final \
  --vad-runtime-profile-selector speed_30kph \
  --vscode-ime-before /home/a/autoware_e2e/artifacts/validation/2026-08-31/vscode_ime_before.png \
  --vscode-ime-after /home/a/autoware_e2e/artifacts/validation/2026-08-31/vscode_ime_after_hangul.png
```
