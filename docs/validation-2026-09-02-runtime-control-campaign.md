# Autoware VAD runtime·제어 캠페인 검증

이 문서는 2026-09-02 canonical 캠페인에 모은 화면 끊김 진단, 30 kph 제어 A/B,
수정 프로필 회귀, 60 kph 탐색 주행과 2026-09-03 geometry-only 후속 A/B를 한 곳에서
해석한다. 원본은 아래 폴더이며, 이전 전체 Town 9-map/18-trial 결과를 대체하지 않는다.

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
| 60 kph geometry A/B | **HOLD** | corridor 0.20 m 후보는 곡률·CTE를 일부 개선했지만 최고 35.114 kph, 중간 저속 cap 43회로 채택 조건 미달 |
| 60 kph production smoothing preflight | **HOLD** | 전체 active geometry 399개를 재현했지만 주행 중 곡률 기준 위반 29/264로 live A/B 미승인 |
| 60 kph endpoint-tapered C1 preflight | **PREFLIGHT FAILED (운영 HOLD)** | 공간 C1 보정은 25/399 snapshot을 fail-closed reject했고 accepted-only 최대 곡률도 기준을 넘어 live 미승인 |
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
기록했다. 곡률 안전 제한을 꺼서 60 kph를 만드는 방식은 사용하지 않는다.

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

### 2026-09-03 geometry-only v4 A/B: HOLD

첫 개선 단계로 전용 `--geometry-ab-route-corridor-0p2` 옵션을 추가하고 같은 Town06
직선 route에서 corridor half-width만 `0.50 → 0.20 m`로 바꿨다. 직선에서는 동작하지
않지만 route-manager invariant를 만족하기 위해 turn-outward width도 함께
`0.50 → 0.20 m`로 직렬화했다. speed profile, controller, command gate, throttle,
actuation map과 trajectory code는 byte-identical이며 source route SHA-256도 두 실행 모두
`ae019ba6f839935919e7b11fa3a3131255849bfbc7ae191b8a517a3182233018`이다.
동일하다고 검증한 speed-profile 대상은 측정 결과 파일 자체가 아니라 설정·계약
invariant다.

| 지표 | A: v3 corridor 0.50 m | B: v4 corridor 0.20 m | 해석 |
|---|---:|---:|---|
| conditioned 곡률 p95 (1/m) | 0.019054 | 0.014826 | 22.2% 감소, 하지만 15 m/s 허용 기준 0.004444에는 미달 |
| conditioned 곡률 최대 (1/m) | 0.063131 | 0.046651 | 26.1% 감소 |
| mid-route 4–9 m/s cap 표본 / 노출 | 51 / 10.2 s | 43 / 8.6 s | full eligible sample-hold 기준 일부 감소했지만 제거되지 않음 |
| 실제 최고속도 | 10.100 m/s (36.362 kph) | 9.754 m/s (35.114 kph) | 3.4% 악화 |
| 15 m/s 이상 연속 노출 | 0.0 s | 0.0 s | 두 실행 모두 속도 계약 FAIL |
| 최대 CTE | 0.524 m | 0.221 m | 57.9% 감소 |
| 최대 횡가속 | 0.437 m/s² | 0.341 m/s² | 22.0% 감소 |
| 최대 trajectory correction | 4.601 m | 4.243 m | 7.8% 감소 |
| goal / runtime·camera | 도착 / PASS | 도착 / PASS | 실행 환경은 비교 가능 |

따라서 B는 `PARTIAL_IMPROVEMENT_INSUFFICIENT`, geometry **HOLD**, 독립 60 kph speed
contract **FAIL**이다. 횡방향 안정성 지표는 좋아졌지만 저속 cap 제거와 15 m/s 노출을
만족하지 못했으므로 v3를 active reference로 유지한다. v4 자체의 evidence integrity는
**PASS**, simulation pilot acceptance는 **FAILED**, real-vehicle readiness는
**BLOCKED**다.

후속 validator는 이를 `physical_goal_completion_status=PASS`,
`speed_exposure_contract_status=FAILED`, `full_stack_route_test_status=FAILED`로 분리해
기록한다. CARLA preflight/completion/cleanup과 포트 해제, route/result/bag, 실제
route-manager parameter dump, trajectory code, actuation manifest/map까지 SHA로 묶었고
v4 evidence integrity는 이 강화된 계약에서도 **PASS**다.

- A/B 기계 판정:
  [`town06_60kph_geometry_corridor_ab_v4.md`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/50_reports/town06_60kph_geometry_corridor_ab_v4.md)
- 비교 차트:
  [`town06_60kph_geometry_corridor_ab_v4.png`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/50_reports/town06_60kph_geometry_corridor_ab_v4.png)
- B pilot 계약:
  [`pilot_run.json`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/30_60kph/town06_straight_60kph_geometry_corridor_0p2_v4/pilot_run.json)
- B 차량 중심 전체화면:
  [`01_autoware_vehicle_centered_fullscreen.png`](assets/validation/2026-09-02-runtime-control-campaign-v1/60kph/town06_straight/B_geometry_corridor_0p2_hold/01_autoware_vehicle_centered_fullscreen.png)
- B 차량 중심 주행 GIF·경로/제어 분석:
  [`B_geometry_corridor_0p2_hold`](assets/validation/2026-09-02-runtime-control-campaign-v1/60kph/town06_straight/B_geometry_corridor_0p2_hold/)
- Git 발행용 A/B 비교 차트:
  [`09_A_selected_pilot_v3_vs_B_geometry_corridor_0p2_HOLD.png`](assets/validation/2026-09-02-runtime-control-campaign-v1/60kph/town06_straight/comparison/09_A_selected_pilot_v3_vs_B_geometry_corridor_0p2_HOLD.png)

### 일반적인 원인과 이 캠페인의 해결 순서

고속 목표에 못 미치는 문제는 보통 (1) 경로 곡률·목표점 제동이 만드는 planning speed
cap, (2) controller·command gate의 가속도/jerk 제한, (3) converter throttle clamp,
(4) actuation map과 차량 동역학 불일치, (5) 기어·항력·route 길이, (6) 낮은 runtime
cadence나 host 부하가 결합해 발생한다. 이번 증거에서는 목표 16.667 m/s가 planning과
gate까지 전달됐고 RTF 약 0.998, camera bundle 100%라 (6)은 현재 60 kph 병목이 아니다.
주된 1차 병목은 직선에서 생긴 conditioned path 곡률과 그에 따른 `-2.0 m/s²`
feed-forward이며, 그 다음은 `+1.5 m/s²` gate 포화와 고속에서 크게 낮아지는 실제 가속
응답이다.

Corridor 다음 첫 후보는 strength `10 → 10,000`만 바꾸는 geometry smoothing으로
격리했다. Live 옵션을 먼저 열지 않고, clipping 지점과 점 간 heading noise가 실제
route-manager 순서에서 줄어드는지를 아래 frozen-bag 전 구간 preflight로 검사했다.
채택에는 곡률 p95와 절대 최대 모두 `≤0.004444 1/m`, mid-route 4–9 m/s cap 제거,
correction·endpoint·CTE·지연 gate를 동시에 요구한다. 그 전에는 throttle 후보를
진행하지 않는다.

보존된 v4 bag의 대표 frozen trajectory에 추가 Whittaker smoothing만 적용한
ROS-graph-free sweep에서는 strength `10,000`이 곡률 p95 `0.002836 1/m`, 최대
`0.002942 1/m`, 원본 대비 최대 편차 약 `0.0365 m`를 보였다. 이는 다음 후보 범위를
고르는 screening 결과일 뿐이며 live 출력·시간 연속성·전체 snapshot·goal을 증명하지
않는다. 특히 이미 strength 10이 적용된 최종 trajectory에 다시 평활한 결과이고
stop-anchor를 재현하지 않았으며, route CTE 최대 `0.2037 m`가 실제 corridor `0.20 m`를
조금 넘어 production에서는 재-clipping될 수 있다. 따라서 production 함수·stop-anchor·
0.20 m corridor를 사용한 전체 snapshot preflight를 먼저 수행했고, PASS일 때만
straight-only 전용 flag와 fail-closed provenance를 열도록 했다. 입력 digest와 전체 sweep은
[`town06_60kph_smoothing_offline_screen_v1`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/50_reports/town06_60kph_smoothing_offline_screen_v1/README.md)에
보존했다.

#### Production-equivalent 전체 snapshot preflight: HOLD

보존 v4 bag의 raw trajectory 433개를 같은 ROS header stamp의 callback 결과와 모두
1:1로 묶었다. 이 중 399개는 active shaped trajectory, 34개는 goal 이후 3-point stop이며,
추가 final 67개는 timer stop refresh로 식별했다. Frozen production code와 실제 parameter
dump로 strength 10을 재생성했을 때 geometry 399/399가 PASS했고 최대 XY 오차는
`3.884e-6 m`였다. 녹화 전 내부 상태가 없는 시작 29개는 scalar/time 동일성에서만
명시 제외했으며 이후 370/370은 strict PASS다.

| Production gate 지표 | strength 10 | 후보 strength 10,000 | 기준/판정 |
|---|---:|---:|---|
| moving-midroute adjacent 곡률 p95 (1/m) | 0.024988 | 0.009569 | ≤0.004444, **FAIL** |
| moving-midroute adjacent 곡률 최대 (1/m) | 0.051510 | 0.023849 | ≤0.004444, **FAIL** |
| 곡률 위반 snapshot | - | 29/264 | 0, **FAIL** |
| 4–9 m/s horizon cap | 43/264 | 9/264 | 0, **FAIL** |
| strength 간 최대 경로 편차 | - | 0.050015 m | ≤0.05 m, **FAIL** |
| endpoint yaw 최대 변화 | - | 0.043316 rad | ≤0.02 rad, **FAIL** |
| smoothing rejection / corridor / fixed anchor | 0 | 0 / ≤0.20 m / PASS | hard safety **PASS** |
| solve p95 / 최대 | 9.750 / 64.828 ms | 9.786 / 55.119 ms | ≤20 / 50 ms, 최대 **FAIL** |

따라서 이 후보는 geometry를 크게 개선했어도 `HOLD`이며
`live_ab_authorized=false`, `real_vehicle_ready=false`다. 실행 코드는 PASS일 때만 한 번의
Town06 straight CARLA A/B를 열도록 했고, 이번에는 live v5를 실행하지 않았다.

- Production preflight 보고서:
  [`README.md`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/50_reports/town06_60kph_smoothing_production_preflight_v1/README.md)
- 기계 판독 결과:
  [`summary.json`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/50_reports/town06_60kph_smoothing_production_preflight_v1/summary.json)
- 전 구간 차트:
  [`comparison.png`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/50_reports/town06_60kph_smoothing_production_preflight_v1/comparison.png)

같은 production pipeline의 추가 read-only sweep에서도 단순 strength 해법은 없었다.
`3k/10k/30k/100k`는 각각 곡률 p95/최대가
`0.010787/0.029868`, `0.009569/0.023849`, `0.007786/0.023758`,
`0.006694/0.018772 1/m`로 절대 기준을 넘었다. `300k/1M`은 accepted snapshot만 보면
좋아지지만 0.10 m deviation guard rejection이 각각 1/2개 발생해 fallback의 최대 곡률이
`0.149276 1/m`가 되므로 사용할 수 없다. `hard → soft`만 바꾼 single-knob screen도
고정 terminal 근처 꺾임 때문에 p95/최대 `1.589793/1.761330 1/m`로 악화됐다.

#### Endpoint-tapered spatial-C1 prototype: PREFLIGHT FAILED / LIVE HOLD

다음 offline 단계로 scalar saturation transition 양단을 C1으로 만들고, 실제 공간
correction에는 고정 endpoint/stop anchor에서 값과 station 미분이 모두 0인 quintic
product taper를 적용했다. 이 접선 보존과 0.20 m corridor를 동시에 만족할 수 없는
snapshot은 hard corner로 바꾸지 않고 fail-closed reject한다. 기존 `legacy` 기본값과
manager/launch/wrapper는 바꾸지 않았으며 live option도 만들지 않았다.

| C1 production preflight 지표 | 결과 | 판정 |
|---|---:|---|
| raw/final pairing / active | 433/433 / 399 | reconstruction PASS |
| prototype rejection | 25/399 (moving-midroute 11) | zero-rejection hard gate **FAIL** |
| accepted-only moving-midroute 곡률 p95 / 최대 | 0.004430 / 0.008904 1/m | p95만 ≤0.004444; 최대 **FAIL** |
| accepted-only 곡률 위반 | 12/253 | **FAIL** |
| rejection fallback 포함 곡률 p95 / 최대 | 0.006902 / 0.149276 1/m | live 출력으로 부적합 |
| route corridor / fixed endpoint·stop / selection deviation | ≤0.20 m / PASS / 0.044225 m | integrity PASS |
| endpoint yaw / 4–9 m/s cap / solve max | 0.043316 rad / 6 / 54.282 ms | 각 기준 **FAIL** |

accepted subset의 p95가 처음으로 목표 아래에 들어온 것은 경계 처리가 방향상 유효하다는
진단 결과다. 그러나 25개 reject를 숨기면 실제 route manager는 원본 fallback을 내보내고,
그 혼합 최대 곡률이 `0.149276 1/m`가 되므로 채택할 수 없다. 결론은
`live_ab_authorized=false`, `real_vehicle_ready=false`, 운영 **HOLD**다.

- C1 상세 보고서:
  [`README.md`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/50_reports/town06_60kph_endpoint_tapered_c1_corridor_preflight_v1/README.md)
- 기계 판독 결과:
  [`summary.json`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/50_reports/town06_60kph_endpoint_tapered_c1_corridor_preflight_v1/summary.json)
- 비교 차트:
  [`comparison.png`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/50_reports/town06_60kph_endpoint_tapered_c1_corridor_preflight_v1/comparison.png)

이 raw bag과 생성 보고서는 ignored `artifacts/`에 있어 새 clone에는 포함되지 않는다.
tracked preflight script는 사용자가 보존한 동일 trial을 `--trial-dir`로 명시할 때만
재생성할 수 있으며, raw evidence 없이 과거 수치를 재현했다고 표시하지 않는다.

다음 geometry 후보는 post-saturation을 더 강하게 하는 방식이 아니라, boundary
endpoint의 outward tangent 자체를 제약에 포함하는 route-relative constrained
spline/QP다. 이 후보도 동일 399-snapshot gate를 먼저 통과하기 전에는 live나 throttle
A/B로 진행하지 않는다.

Geometry가 채택된 뒤에만 `max_throttle 0.4 → 0.5`를 단독 진단 A/B로 실행한다. v3에서
역산된 필요 throttle 최대가 약 0.415이므로 이것만으로 60 kph를 해결할 가능성은 낮다.
그 다음 CARLA 전용 고속 actuation calibration으로 현재 `13.89 m/s`까지만 있는 map
속도축을 최소 60 kph target 이상으로 재계측·확장하고, 반복 측정으로 command 대비 실제
가속 응답을 맞춘다. controller/gate 변경은 그 후 각각 별도 A/B로 취급한다.

현재 5 Hz camera는 60 kph에서 frame당 약 3.33 m를 이동한다. Town06 직선 pilot의
runtime 증거는 건강하지만, 60 kph 회전은 이 결과로 승인되지 않는다. 회전 시험 전에는
sensor/VAD cadence·지연, 곡률별 횡가속, 제동거리와 비상정지를 별도 gate로 검증한다.

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

- repository test suite: **998 passed, 1 skipped**
- sourced Autoware launch-package suite: **267 passed**
- 이번 변경 파일 집중 회귀: **335 passed**
- clone/build/model/setup 안전 guard 집중 회귀: **33 passed**
- 60 kph C1 geometry 집중 회귀: **82 passed**
- cleanup 안전장치 최종 delta suite: **23 passed**
- CSV LF 생성 회귀 delta suite: **13 passed**
- 현재 캠페인 정리 화면: **62 assets**, `SHA256SUMS` 검증 통과
- 전체 Town 발행 화면: `SHA256SUMS` 검증 통과

발행기는 `route_alignment.json`까지 exact SHA로 묶고, prior manifest의 소유권을
asset schema에서 재도출한다. Output 변경은 `O_NOFOLLOW` dir-fd와 nonblocking flock으로
고정해 재서명 ownership injection, parent symlink swap, 동시 실행을 fail-closed로 막았다.

실패·대체 산출물 24개와 이 작업이 소유한 `/tmp` 스크래치는 네 번의 hash-bound
rename quarantine으로 정리했다. 총 **23,954 entries / 93,917 files /
11,883,427,314 bytes**이며 삭제나 복사는 수행하지 않았다. 상세 plan, 보존 대상,
복원 journal은 로컬
[`99_integrity/CLEANUP.md`](../artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1/99_integrity/CLEANUP.md)에
있다. 실행기가 재사용하는 0-byte `autoware_e2e_*.lock`과 발행기 실제 두 대상의
0-byte flock만 `/tmp`에 유지했다. 테스트별 발행 lock은 각 테스트 임시 폴더로 분리해
workstation `/tmp`에 찌꺼기를 남기지 않는다.

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
- 60 kph geometry v4, strength-10,000과 endpoint-tapered C1 preflight는 부분 개선만
  확인한 HOLD/FAIL 증거로 보존한다. 다음 constrained spline/QP 후보부터 같은 399-snapshot gate,
  actuation-map coverage, 정지거리, acceleration/jerk gate, 반복성, 회전 곡률 제한을
  단계별로 통과해야 한다.
- 실제 차량은 vehicle-specific actuation calibration, 폐쇄 구간, 안전 운전자,
  비상정지와 독립 safety gate 전까지 `real_vehicle_ready=false`를 유지한다.
