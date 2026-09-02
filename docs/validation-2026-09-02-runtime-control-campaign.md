# Autoware VAD runtime·제어 캠페인 검증

이 문서는 2026-09-02 canonical 캠페인에 모은 화면 끊김 진단, 30 kph 제어 A/B,
수정 프로필 회귀, 60 kph 탐색 주행을 한 곳에서 해석한다. 원본은 아래 폴더이며,
이전 전체 Town 9-map/18-trial 결과를 대체하지 않는다.

- Canonical root:
  [`artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/)
- Git에 포함되는 정리 화면(PNG/GIF/경로·제어 분석):
  [`docs/assets/validation/2026-09-02-runtime-control-campaign-v1/README.md`](assets/validation/2026-09-02-runtime-control-campaign-v1/README.md)
- 전체 Town 30 kph 결과:
  [`validation-2026-09-02-all-towns-camera-source-5hz.md`](validation-2026-09-02-all-towns-camera-source-5hz.md)
- 이전 동일 Town06 30/60 kph 비교:
  [`validation-2026-09-02-speed-30-60-longitudinal.md`](validation-2026-09-02-speed-30-60-longitudinal.md)

## 현재 판정

| 항목 | 판정 | 의미 |
|---|---|---|
| 화면 끊김 재현 | **REPRODUCED** | fresh CARLA 3회 모두 engage 전에 RTF 약 0.244, camera wall rate 1.25 Hz로 실패 |
| camera transport 수정 | **REGRESSION PASS 4/4** | 30 kph 3개 시나리오와 60 kph v3가 첫 cold-start 시도에서 health·graph PASS |
| 전체 Town 30 kph | **RUNNABLE PASS 9/9** | 직진·회전 18/18 PASS; 나머지 10개는 주행 실패가 아니라 map bundle unavailable/not admitted로 BLOCKED |
| 30 kph 제어 A/B | **HOLD 3/3** | B 후보를 채택하지 않고 A baseline 유지 |
| 30 kph 선택 baseline 회귀 | **PASS 3/3** | `Town07/straight`, `C_track_1_0_7/turn`, `Town03/turn` 모두 goal reached |
| 60 kph 현재 후보 | **FAILED** | Town06 goal에는 도착했지만 최고 36.362 kph로 속도 노출 계약 미달 |
| 실제 차량 준비 | **NO** | 모든 결과는 CARLA simulation screening, `real_vehicle_ready=false` |

여기서 30/60 kph는 요청한 simulation profile의 명칭이다. 회전 구간은 곡률·횡가속
제한을 적용하므로 30 kph를 계속 유지하지 않으며, 60 kph도 목표값을 설정했다는 사실과
실제 차량 속도 달성을 구분한다.

## Canonical 폴더 구조

```text
autoware_vad_runtime_control_campaign_v1/
├── 00_index/             # 캠페인 인덱스와 안내
├── 10_runtime_stutter/   # engage 전 끊김 재현·원본 포인터
├── 15_all_towns_30kph/   # 8/31 inventory, 9/1 v16, 9/2 최신 9-map matrix 포인터
├── 20_30kph_control_ab/  # A/B, 비교 판정, 선택 baseline C 회귀
├── 30_60kph/             # 버전별 60 kph 탐색 주행
├── 40_visuals/           # 최종 선택 화면의 manifest/발행 영역
├── 50_reports/           # 기계 생성 요약
├── 90_quarantine/        # 정리 대상 격리 영역; 판정 근거로 사용하지 않음
└── 99_integrity/         # 무결성·정리 계획
```

각 주행의 `result.json`, `runtime_health.json`, `diagnosis.json`,
`speed_profile.json`, bag과 provenance가 판정 원본이다. PNG/GIF는 해석을 돕는 화면
증거이며 수치 판정을 대신하지 않는다.

`artifacts/...` 링크는 이 workstation의 보존 원본을 가리키므로 GitHub에서는 열리지
않는다. GitHub 검토에는 위의 `docs/assets/...` 정리 화면과 이 보고서의 수치 표를
사용한다.

### 전체 Town 30 kph 계보

최신 `speed_30kph_camera_source_5hz` matrix는 실행 가능한 9개 map 모두 PASS했고
각 map의 직진·회전 18개 선택 trial도 18/18 PASS다. 19개 canonical map 중 나머지
10개 `BLOCKED`는 주행 실패가 아니다. 로컬에서 검증된 Lanelet2+PCD full-map bundle이
없거나 validation admission을 받지 못해 실행 대상에서 제외된 상태다.

- 2026-08-31 초기 inventory·map 준비도·원본 자료:
  [`validation-2026-08-31.md`](validation-2026-08-31.md),
  [`artifacts/validation/2026-08-31/maps`](../artifacts/validation/2026-08-31/maps/)
- 2026-09-01 v16 owned-window 최종 matrix와 화면 판독 기준선:
  [`validation-2026-09-01.md`](validation-2026-09-01.md),
  [`v16_owned_window_visual_audit.md`](assets/validation/2026-09-01/owned_window_visual_audit/v16_owned_window_visual_audit.md),
  [`v16 aggregate.json`](../artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/aggregate.json)
- 2026-09-02 최신 camera-source 5 Hz 전체 Town 결과:
  [`validation-2026-09-02-all-towns-camera-source-5hz.md`](validation-2026-09-02-all-towns-camera-source-5hz.md),
  [`latest aggregate.json`](../artifacts/validation/2026-09-02/autoware_vad_town_matrix_30kph_camera_source_5hz_v1/aggregate.json)

## 화면 끊김 원인과 수정 결과

### 수정 전 재현

Town07 직진의 기존 reliable image 경로에서 fresh CARLA를 세 번 실행했지만 모두
pre-engagement health gate에서 차단됐다. 따라서 이 세 시도는 제어 실패가 아니라
주행 전 runtime/camera 전달 실패 자료다.

| 시도 | RTF 중앙값 | 최소 camera wall rate | 6-camera receipt p95 중앙값 | CAM_FRONT publish 중앙값 |
|---|---:|---:|---:|---:|
| `attempt_001` | 0.243750 | 1.250 Hz | 650.811 ms | 616.364 ms |
| `attempt_002` | 0.243750 | 1.250 Hz | 660.407 ms | 618.526 ms |
| `attempt_003` | 0.243750 | 1.250 Hz | 656.549 ms | 620.360 ms |

설정된 source cadence는 `sensor_tick=0.2 s`, 즉 simulation time 기준 5 Hz였다.
RTF 0.24375를 곱한 예상 wall cadence는 약 1.219 Hz로 관측 1.25 Hz와 일치한다.
따라서 심한 멈춤은 “camera가 원래 1.25 Hz로 설정됨”이 아니라 simulation 전체가
약 0.244배로 느려지면서 5 sim-Hz가 wall 기준 약 1.25 Hz로 보인 현상이다.

CAM_FRONT publish 시간과 여섯 camera receipt 폭이 모두 약 0.62~0.66초였다는 점은
직렬화된 image publish 또는 DDS backpressure 경로가 강한 기여 후보임을 보인다.
다만 이 값만으로 CARLA render, image conversion, worker scheduling 중 하나를 단일
근본 원인으로 확정하지 않는다. “다른 곳에서 사용 중”이거나 PC 전체 부하가 원인이라는
증거도 이 재현만으로 성립하지 않는다.

- 재현 요약:
  [`runtime_stutter_summary.json`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/10_runtime_stutter/town07_straight_A_baseline_health_001/runtime_stutter_summary.json)
- 원본 무결성 포인터:
  [`source_pointers.json`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/10_runtime_stutter/town07_straight_A_baseline_health_001/source_pointers.json)

### 적용한 simulation 전용 transport 계약

`carla_vad_camera_source_5hz_best_effort_image_v2`는 여섯 raw image publisher,
VAD subscriber, RViz front-image subscriber를 `BestEffort / Volatile / KEEP_LAST /
depth 1`로 맞춘다. CameraInfo는 `Reliable / KEEP_LAST / depth 1`로 유지한다.
CycloneDDS는 loopback `lo`를 명시하고 `ROS_LOCALHOST_ONLY=0`으로 실행한다. XML이
interface를 소유하므로 `ROS_LOCALHOST_ONLY=1`을 동시에 쓰지 않는 것이 계약의 일부다.

| Provenance | SHA-256 |
|---|---|
| sensor mapping | `5c2877aeff20d3a58bcc7211a2f37116b6e17126e62839d8742f79fc185c38ef` |
| VAD model override | `d9cb08286f6cabc52275f42fbbbe598d25b989f9f43a0e268a1c0bd9523b17fb` |
| CycloneDDS loopback XML | `3f797de4343c68d049e15778b23b41ffee67b05c3f731077884179f08cfce289` |
| 차량 중심 RViz config | `47fb76d4066fa8c8a0d8c400716ac007bb084a77cb9aeeb988fee0512557a47b` |

수정 후 아래 네 실행은 자동 health retry 없이 모두 첫 시도에서 exact image graph와
3개 연속 8초 health window를 통과했다.

| 실행 | health/graph | 최소 RTF | 최소 camera Hz | 최소 bundle | 최악 receipt p95 |
|---|---|---:|---:|---:|---:|
| Town07 직진 30 kph C | PASS / PASS | 0.993750 | 5.000 | 100.0% | 5.878 ms |
| C-track 좌회전 30 kph C | PASS / PASS | 0.987500 | 5.000 | 100.0% | 6.200 ms |
| Town03 좌회전 30 kph C | PASS / PASS | 0.993750 | 5.000 | 100.0% | 5.938 ms |
| Town06 직진 60 kph v3 | PASS / PASS | 0.993750 | 5.000 | 100.0% | 5.941 ms |

이 결과는 끊김 경로를 실용적으로 제거한 회귀 증거다. 단일 내부 함수가 유일한 원인이었다는
인과 증명으로 확대하지 않는다.

## 30 kph 제어 A/B와 채택 결정

동일 route SHA를 묶고 한 번에 하나의 knob만 바꿨다. Town07 직진은 longitudinal
PID의 `max_i_effort 0.3 → 0.4`, 두 회전은 curvature preview `3 m → 5 m` 후보였다.

| 시나리오 | A/B route | A/B 최고속도 m/s | A/B 최대 CTE m | A/B raw→gated RMSE m/s | 결정과 이유 |
|---|---|---:|---:|---:|---|
| `town07_straight` | PASS / FAIL | 7.700 / 7.270 | 0.531 / 0.545 | 1.595 / 1.589 | **HOLD** — B가 속도 노출·goal 계약과 5% tracking 개선 계약 실패 |
| `c_track_turn` | PASS / PASS | 4.766 / 4.834 | 0.514 / 0.512 | 1.427 / 1.483 | **HOLD** — raw→gated RMSE 3.86% 악화 |
| `town03_turn` | PASS / PASS | 5.018 / 5.025 | 0.502 / 0.526 | 1.620 / 1.689 | **HOLD** — raw→gated RMSE 4.26% 악화 |

따라서 세 시나리오 모두 A baseline을 유지했다. 회전 B가 goal에 도착했거나 일부 지표가
좋아졌다는 이유만으로 채택하지 않았으며, 사전에 정의한 종합 gate를 사용했다.

- Town07 비교:
  [`A_baseline_vs_B_pid_i40_decision.json`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/20_30kph_control_ab/town07_straight/comparison/A_baseline_vs_B_pid_i40_decision.json)
- C-track 비교:
  [`A_baseline_vs_B_turn_preview_5m_decision.json`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/20_30kph_control_ab/c_track_turn/comparison/A_baseline_vs_B_turn_preview_5m_decision.json)
- Town03 비교:
  [`A_baseline_vs_B_turn_preview_5m_decision.json`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/20_30kph_control_ab/town03_turn/comparison/A_baseline_vs_B_turn_preview_5m_decision.json)

## 선택 baseline의 depth-1 회귀

사용자가 지정한 순서대로 `Town07/straight → C_track_1_0_7/turn → Town03/turn`을
재실행했다. 세 실행 모두 `attempt_001`, process exit 0, goal reached다.

| 시나리오 | 결과 | 최고속도 kph | 이동거리 m | 최대 CTE m | 최대 횡가속 m/s² | route RTF |
|---|---|---:|---:|---:|---:|---:|
| Town07 직진 | **PASS** | 27.625 | 209.546 | 0.528 | 0.422 | 0.998473 |
| C-track 좌회전 | **PASS** | 16.888 | 73.342 | 0.519 | 0.643 | 0.998448 |
| Town03 좌회전 | **PASS** | 18.196 | 72.355 | 0.503 | 0.638 | 0.998484 |

경로 도착과 별개로 세 진단 모두 `path_dominant`다. raw VAD 경로의 route-offset p95는
각각 `0.820 / 6.829 / 2.434 m`, raw→final 개입 p95는
`0.654 / 8.832 / 3.269 m`였다. 반면 실제 차량의 final-path 추종 최대오차는
`0.034 / 0.254 / 0.301 m`였다. 즉 제어된 최종 경로는 따라갔지만 raw VAD geometry가
완성됐다고 판정한 것은 아니며, 특히 회전 geometry 고도화가 계속 필요하다.

### 30 kph 화면과 분석

| 장면 | 전체화면 | 주행 GIF | 경로·제어 | 원본 JSON |
|---|---|---|---|---|
| Town07 직진 | [PNG](assets/validation/2026-09-02-runtime-control-campaign-v1/30kph/town07_straight/C_selected_baseline_regression/01_autoware_vehicle_centered_fullscreen.png) | [GIF](assets/validation/2026-09-02-runtime-control-campaign-v1/30kph/town07_straight/C_selected_baseline_regression/02_autoware_drive.gif) | [path](assets/validation/2026-09-02-runtime-control-campaign-v1/30kph/town07_straight/C_selected_baseline_regression/03_path_vs_control.png) | [result](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/20_30kph_control_ab/town07_straight/C_selected_baseline_depth1_loopback_regression_001/attempts/attempt_001/result.json) |
| C-track 좌회전 | [PNG](assets/validation/2026-09-02-runtime-control-campaign-v1/30kph/c_track_turn/C_selected_baseline_regression/01_autoware_vehicle_centered_fullscreen.png) | [GIF](assets/validation/2026-09-02-runtime-control-campaign-v1/30kph/c_track_turn/C_selected_baseline_regression/02_autoware_drive.gif) | [path](assets/validation/2026-09-02-runtime-control-campaign-v1/30kph/c_track_turn/C_selected_baseline_regression/03_path_vs_control.png) | [result](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/20_30kph_control_ab/c_track_turn/C_selected_baseline_depth1_loopback_regression_001/attempts/attempt_001/result.json) |
| Town03 좌회전 | [PNG](assets/validation/2026-09-02-runtime-control-campaign-v1/30kph/town03_turn/C_selected_baseline_regression/01_autoware_vehicle_centered_fullscreen.png) | [GIF](assets/validation/2026-09-02-runtime-control-campaign-v1/30kph/town03_turn/C_selected_baseline_regression/02_autoware_drive.gif) | [path](assets/validation/2026-09-02-runtime-control-campaign-v1/30kph/town03_turn/C_selected_baseline_regression/03_path_vs_control.png) | [result](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/20_30kph_control_ab/town03_turn/C_selected_baseline_depth1_loopback_regression_001/attempts/attempt_001/result.json) |

## 60 kph 현재 후보

<!-- CURRENT_60KPH_CANDIDATE_BEGIN -->

현재 active evidence는
`town06_straight_60kph_pilot_best_effort_image_depth1_v3`이다. 후속 v4가 생성되더라도
v3를 덮어쓰지 않고, v4가 health·camera·route-speed·provenance gate를 모두 만족할 때만
이 블록과 상단 판정을 갱신한다.

- Pilot:
  [`pilot_run.json`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/30_60kph/town06_straight_60kph_pilot_best_effort_image_depth1_v3/pilot_run.json)
- Trial:
  [`trial/attempt_001`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/30_60kph/town06_straight_60kph_pilot_best_effort_image_depth1_v3/trial/attempt_001/)

Town06의 물리적으로 직선인 `445.880 m` 경로를 `77.400 sim-s / 77.524 wall-s`,
RTF `0.998405`로 주행해 goal에 도착했다. 그러나 최고속도는 `10.100424 m/s`
(`36.3615 kph`)였고, 합격 조건 `>=15.000 m/s`를 1초 이상 유지한 시간은 `0.000 s`다.
따라서 경로 완료와 무관하게 pilot 상태는 **FAILED**다.

| 지표 | v3 관측값 |
|---|---:|
| 요청 target | 16.666667 m/s (60.000 kph) |
| planning/gated target 최대 | 16.666666 / 16.666666 m/s |
| 실제 최고속도 | 10.100424 m/s (36.3615 kph) |
| gated target - actual RMSE | 6.642 m/s |
| gate +1.5 m/s² limit duty | 42.687% |
| throttle near-saturation duty | 7.025% |
| brake active duty | 19.547% |
| 최대 CTE / 최대 횡가속 | 0.524 m / 0.437 m/s² |
| 이동거리 | 445.197 m |

현재 accel/brake map 속도축은 `13.89 m/s`(`50.004 kph`)까지라 60 kph target envelope를
포괄하지 않는다. 이번 실제 lookup 최고속도는 10.100 m/s라 runtime axis clamp는
관측되지 않았지만, 60 kph 도달 전에는 map 축 확장과 vehicle-specific calibration이
필수다. target 생성, acceleration gate, PID/actuation 응답을 분리한 A/B 없이 throttle만
키워 60 kph 성공으로 처리하지 않는다.

상세 병목 분리는 로컬 원본
[`town06_60kph_v3_speed_limit_analysis.md`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/50_reports/town06_60kph_v3_speed_limit_analysis.md)에
기록했다. 다음 순서는 geometry-only route corridor `0.5 → 0.2 m` A/B,
`max_throttle 0.4 → 0.5` 단독 probe, CARLA actuation map 재계측·확장이다. 곡률 안전
제한을 꺼서 60 kph를 만드는 방식은 사용하지 않는다.

경로 진단은 `path_dominant`다. raw VAD route-offset p95 `1.178 m`, raw→final 개입 p95
`1.892 m`에 비해 실제 final-path 추종 최대오차는 `0.057 m`였다. 최종 경로 추종은
안정적이지만 raw VAD geometry 품질 완료를 뜻하지 않는다.

| 화면/분석 | 파일 |
|---|---|
| 차량 중심 Autoware 전체화면 | [01_autoware_vehicle_centered_fullscreen.png](assets/validation/2026-09-02-runtime-control-campaign-v1/60kph/town06_straight/selected_pilot/01_autoware_vehicle_centered_fullscreen.png) |
| 차량 중심 주행 | [02_autoware_drive.gif](assets/validation/2026-09-02-runtime-control-campaign-v1/60kph/town06_straight/selected_pilot/02_autoware_drive.gif) |
| 경로·제어 | [03_path_vs_control.png](assets/validation/2026-09-02-runtime-control-campaign-v1/60kph/town06_straight/selected_pilot/03_path_vs_control.png) |
| 속도 | [05_speed_profile.png](assets/validation/2026-09-02-runtime-control-campaign-v1/60kph/town06_straight/selected_pilot/05_speed_profile.png) |
| 종방향 응답 | [longitudinal_response.png](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/30_60kph/town06_straight_60kph_pilot_best_effort_image_depth1_v3/trial/attempt_001/longitudinal_response.png) |
| runtime load | [runtime_load_analysis.png](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/30_60kph/town06_straight_60kph_pilot_best_effort_image_depth1_v3/trial/attempt_001/runtime_load_analysis.png) |

<!-- CURRENT_60KPH_CANDIDATE_END -->

## 차량 중심 전체화면 계약

30 kph C 3장과 60 kph v3 1장의 원본 PNG를 1920×1080으로 확인했다. 네 화면 모두
owned RViz window만 포함하며, 오른쪽 main viewport 중앙에 차량이 있고 다음 항목을
동시에 판독할 수 있다.

- `TopDownOrtho`, target frame `base_link`, center `(0, 0)`, scale `10`
- 기준 경로, VAD raw/후보, 최종 trajectory, 실제 주행 path
- 좌측 상단 VAD front camera
- Routing `Set`, Localization `Initialized`, control mode `AUTONOMOUS`

GIF는 960×540이다. 화면 상단의 순간 속도 숫자는 대표 frame 한 장의 값이므로 최고속도
판정에는 사용하지 않고, bag에서 생성한 `result.json`과 `speed_profile.json`을 사용한다.

## 최종 검증과 정리

- repository test suite: **907 passed, 1 skipped**
- sourced Autoware launch-package suite: **256 passed**
- cleanup 안전장치 최종 delta suite: **23 passed**
- CSV LF 생성 회귀 delta suite: **13 passed**
- 현재 캠페인 정리 화면: **53 assets**, `SHA256SUMS` 검증 통과
- 전체 Town 발행 화면: `SHA256SUMS` 검증 통과

실패·대체 산출물 24개와 이 작업이 소유한 `/tmp` 스크래치는 네 번의 hash-bound
rename quarantine으로 정리했다. 총 **23,954 entries / 93,917 files /
11,883,427,314 bytes**이며 삭제나 복사는 수행하지 않았다. 상세 plan, 보존 대상,
복원 journal은 로컬
[`99_integrity/CLEANUP.md`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/99_integrity/CLEANUP.md)에
있다. 실행기가 재사용하는 0-byte `autoware_e2e_*.lock`만 `/tmp`에 유지했다.

## VS Code 한글 입력

이 workstation은 `/home/a/.local/bin/code` wrapper가 X11의 VS Code snap에서
`GTK_IM_MODULE=xim`, `XMODIFIERS=@im=ibus`를 적용하고 있으며 `command -v code`도 이
wrapper를 가리킨다. 현재 IBus engine은 `hangul`이다. 이미 열린 창에는 환경이 소급
적용되지 않으므로 한글 입력이 다시 안 될 때만 작업을 저장한 뒤 VS Code를 완전히
종료하고 터미널에서 `code`로 다시 실행한다. 이 설정은 host-local이라 Git push에는
포함되지 않는다.

## 해석 경계와 다음 gate

- 종방향 속도는 `explicit_simulation_nominal` profile이 공급한다. VAD geometry는
  평가하지만 raw VAD velocity는 cruise target으로 평가하지 않는다.
- BestEffort/depth-1/loopback 변경은 localhost CARLA simulation의 backpressure 완화
  프로필이다. 실제 차량 network·sensor QoS에 그대로 적용할 근거가 아니다.
- 30 kph 결과는 선택 baseline 회귀 통과이고, raw VAD 경로 품질 완료 판정은 아니다.
- 60 kph는 현재 실패 증거를 보존한다. 다음 후보는 actuation-map coverage, 정지거리,
  acceleration/jerk gate, 반복성, 회전 곡률 제한을 통과해야 한다.
- 실제 차량은 vehicle-specific actuation calibration, 폐쇄 구간, 안전 운전자,
  비상정지와 독립 safety gate 전까지 `real_vehicle_ready=false`를 유지한다.
