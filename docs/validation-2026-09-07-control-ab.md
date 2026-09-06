# 2026-09-07 30 km/h 제어 A/B 및 60 km/h readiness 검증

## 한눈에 보는 결론

30 km/h에서 요청한 순서인 Town07 직진, C-track 회전, Town03 회전의 strict
six-camera 10 Hz 계측과 제어 A/B를 완료했다. 이번 세 신규 후보 중 승격된 설정은 없다.
최종 운용 설정은 세 경로 모두 기존 baseline을 유지한다.

- Town07 `longitudinal_recovery_2p0`: **NO-GO**
- C-track `turn_preview_5m`: 독립 안전 screen은 PASS했지만 정식 A/B는 **HOLD → NO-GO**
- 이전 C-track `turn_preview_10m`: 보존한 선행 실험에서 `17.651 m > 15.0 m` 경로
  보정 safety fault로 **NO-GO**
- Town03 `turn_preview_10m`: 속도 tracking은 좋아졌지만 주 지표와 보정량 gate가 악화되어
  **HOLD → NO-GO**
- 60 km/h Town06 strict 10 Hz 1차 실행: 경로 완주는 **PASS**했지만 최고
  `36.44 km/h`, 전체 구간 camera receipt p95 `45.715 ms > 40 ms`로
  **NO-GO**다. 실차 readiness는 계속 `BLOCKED`다.

최신 전체화면 PNG, 주행 GIF, 경로·조향·속도·지연 그래프, A/B 판정 JSON/PNG,
실패 진단과 60 km/h 과거 근거는
[단일 검증 폴더](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/)에
카테고리별로 정리했다. 원시 bag, MKV, console/stack log와 임시 파일은 이 발행본에
복사하지 않았다.

이번 주행의 차량 제어 주체는 **Autoware VAD + Autoware controller**다. Portable E2E
physical-v1은 동일한 six-camera 입력으로 10 Hz 추론했지만 shadow 전용 topic에만
발행했다. 즉, 아래 결과를 Portable 모델의 폐루프 주행이나 실차 승인으로 해석하면
안 된다.

## 30 km/h 실행 결과

| 경로 / arm | owner | 최고속도 | 최대 CTE | 최대 경로 보정 | 최대 횡가속 | sim 시간 | 최종 판정 |
|---|---:|---:|---:|---:|---:|---:|---|
| Town07 baseline | FAIL¹ | 7.564 m/s | 0.552 m | 1.821 m | 0.479 m/s² | 46.1 s | baseline 유지, 해당 arm은 pair 제외 |
| Town07 recovery 2.0 | PASS | 7.699 m/s | 0.519 m | 1.775 m | 0.572 m/s² | 46.5 s | NO-GO |
| C-track baseline 3 m | PASS | 4.681 m/s | 0.528 m | 14.633 m | 0.643 m/s² | 36.65 s | **선택 유지** |
| C-track preview 5 m | PASS | 4.894 m/s | 0.545 m | 13.177 m | 0.700 m/s² | 34.8 s | HOLD, NO-GO |
| Town03 baseline 3 m | PASS | 5.083 m/s | 0.498 m | 8.610 m | 0.638 m/s² | 42.4 s | **선택 유지** |
| Town03 preview 10 m | PASS | 5.118 m/s | 0.505 m | 8.937 m | 0.590 m/s² | 37.0 s | HOLD, NO-GO |

¹ Town07 baseline은 goal에는 도달했지만 `>=7.5 m/s`가 `0.05 s`뿐이어서 기존
`>=1.0 s` speed-exposure 계약을 통과하지 못했다. 실패를 favorable rerun으로 지우지
않았으며 이 pair를 성능 비교 분모에 넣지 않았다. “baseline 유지”는 실패한 한 실행을
PASS로 바꾼다는 뜻이 아니라 검증되지 않은 후보로 기본 설정을 교체하지 않는다는 뜻이다.

위 여섯 arm은 모두 Portable shadow 분석 `EVIDENCE_VALID`, 10 Hz 주장
`ESTABLISHED_FOR_SHADOW_ONLY`였다. source 인접 간격은 전부
`100,000,001~100,000,002 ns`, 위반 `0`이었다. camera bundle coverage는
`99.705~100%`, 추론 p99는 `37.905~40.239 ms`였고 shadow accepted 수는
arm별 `441~553`이었다.

## A/B 해석

### Town07 직진: recovery 1.5 → 2.0 m/s²

후보는 최고속도와 7.5 m/s 이상 유지 시간을 늘렸지만 승격 조건에는 부족했다.

- 최고속도 `7.699 < 7.78 m/s`
- 7.5 m/s 이상 `2.75 < 3.5 s`
- target tracking RMSE `1.942 → 2.009 m/s`, 약 `3.49%` 악화
- planning 평균 증가는 약 `0.015 m/s`, 실제 평균 변화는 약 `-0.002 m/s`

병목은 longitudinal PID나 actuation saturation보다 경로 형상에 가깝다. 최종 경로의
peak curvature p95가 약 `0.041~0.042 1/m`이고 lateral cap `1.2 m/s²`를 적용하면
약 `5.3~5.4 m/s` 속도 제한이 생긴다. “직진” route에도 command 전환과 waypoint
경계가 있으며 최고속도 구간이 종점 직전 약 16 m에만 나타났다. 안전 gate를 높여 이를
숨기지 않고 후보를 폐기했다.

- [후보 전체화면](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/01_town07_straight/B_recovery_2p0_no_go/visuals/01_vehicle_centered_fullscreen.png)
- [후보 전체 주행 GIF](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/01_town07_straight/B_recovery_2p0_no_go/visuals/09_autoware_drive_5fps.gif)
- [반복 판정](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/01_town07_straight/comparison/town07_control_ab_repeat_summary.png)

### C-track 회전: preview 3 → 5 → 10 m

5 m 후보는 먼저 독립 strict safety screen을 통과했다. 정식 A/B에서도 두 arm 모두
goal, CTE, 횡가속, 경로 보정과 10 Hz 계측 조건을 통과했고 후보의 최대 보정량은
`14.633 → 13.177 m`로 줄었다. 그러나 primary raw-to-gated RMSE 비율이
`0.9792`로, 요구한 `<=0.90` 개선에 못 미쳐 첫 유효 pair에서 HOLD가 났다.
승격 정책은 모든 6 pair가 ACCEPT여야 하므로 나머지를 실행해 좋은 결과를 골라도
승격할 수 없으며, 사전 선언한 futility 규칙에 따라 종료했다.

10 m 후보는 앞선 campaign의 4번째 pair에서 runtime health와 shadow evidence가
정상이었지만 경로 보정 `17.651 m`로 불변 안전 한도 `15.0 m`를 넘었다. 이 safety
failure는 재실행으로 대체하지 않았다.

- [baseline 전체화면](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/02_c_track_turn/A_baseline_selected/visuals/01_vehicle_centered_fullscreen.png)
- [5 m 후보 전체 주행 GIF](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/02_c_track_turn/B_preview_5m_hold/visuals/09_autoware_drive_5fps.gif)
- [3 m 대 5 m 비교](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/02_c_track_turn/comparison/A_baseline_vs_B_turn_preview_5m_decision.png)
- [10 m geometry failure](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/04_rejected_and_runtime_diagnostics/C_track_preview_10m_geometry_fail/visuals/07_route_result.png)

### Town03 회전: preview 3 → 10 m

두 arm 모두 route와 계측 gate는 PASS했다. 후보는 target tracking RMSE를
`1.904 → 1.663 m/s`로 줄이고 실행 시간을 `42.4 → 37.0 s`로 단축했지만,
primary raw-to-gated RMSE는 `1.477 → 1.543 m/s`로 악화했다. 경로 보정도
`8.610 → 8.937 m`, 즉 `+0.326 m`로 허용 증가 `+0.25 m`를 넘었다. 더구나 같은
10 m 설정이 C-track에서 safety fault를 냈으므로 map 전역 승격은 불가능하다.

- [후보 전체화면](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/03_town03_turn/B_preview_10m_no_go/visuals/01_vehicle_centered_fullscreen.png)
- [후보 전체 주행 GIF](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/03_town03_turn/B_preview_10m_no_go/visuals/09_autoware_drive_5fps.gif)
- [3 m 대 10 m 비교](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/03_town03_turn/comparison/A_baseline_vs_B_turn_preview_10m_decision.png)

## 화면 끊김 원인 분리

정상 기준 실행은 source 10 Hz, accepted output 약 `9.986 Hz`, RTF 약 `0.994`,
추론 p99 약 `40.33 ms`, source loss와 timeout `0`이었다. RViz redraw는 10 fps지만
발행 GIF는 저장 용량을 줄이기 위해 5 fps다. 따라서 GIF의 계단 같은 움직임은 실제
camera frame 유실과 같지 않다.

반면 C-track 10 m campaign의 pair04 baseline에서는 source sim 간격은 여전히 정확한
100 ms였지만 camera별 wall-arrival 최대 간격이 `0.311~0.323 s`까지 밀리고,
route 평가 구간의 run queue peak가 `75`,
accepted-status 최대 공백 `3.634 s`, rejection `16`, camera bundle timeout `2`가
발생했다. 이는 camera Hz 설정이 아니라 host scheduling/runtime stall로 분류해 해당
arm을 `INVALID_INFRA`로 제외했다. timeout을 늘리거나 실패를 숨기지 않았다.

- [정상 camera barrier 기준](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/04_rejected_and_runtime_diagnostics/healthy_camera_reference/)
- [간헐 host stall 근거](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/04_rejected_and_runtime_diagnostics/host_stall/)
- [host stall 검토용 route-window 표본](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/04_rejected_and_runtime_diagnostics/host_stall/host_runtime_stall_summary.json)

## 60 km/h Town06 strict 10 Hz 1차 실행

과거 5 Hz 결과를 그대로 반복하지 않고 generic strict six-camera 10 Hz 옵션으로
Town06 full-stack 직진을 한 번 실행했다. 제어 주체는 Autoware VAD route manager와
Autoware controller이며 Portable 모델·node·bundle은 전혀 로드하지 않았다. 경로
`445.146 m`를 주행해 goal에는 도달했지만 이 실행의 최종 판정은 `NO-GO`다.

| 구분 | 결과 | 판정 |
|---|---:|---:|
| 물리 경로 완주 | goal 도달, 잔여 약 `0.787 m` | PASS |
| 최고 속도 | `10.123258 m/s = 36.4437 km/h` | FAIL |
| `>=15 m/s` 유지 | `0.0 s`, 요구 `>=1.0 s` | FAIL |
| route RTF | `0.929612` | PASS (`>=0.9`) |
| 최대 CTE / 경로 보정 | `0.544238 / 4.374918 m` | PASS |
| 최대 횡가속 | `0.618619 m/s²` | PASS |
| VAD 출력 / inference p99 | `9.4676 Hz / 44.0786 ms` | 관측값 |
| pre-engagement camera health | 8초 창 3개 연속 PASS | PASS |
| standalone post-run camera cadence | 약 10 Hz, coverage `99.884%` | PASS |
| full-run six-camera receipt p95 | `45.715 ms`, 요구 `<=40 ms` | FAIL |
| 최종 simulation acceptance | speed + camera full-run 실패 | FAILED |
| 실차 readiness | simulation-only, 미보정 actuation | BLOCKED |

### 카메라 PASS와 FAIL이 함께 기록된 이유

시작 전 health gate의 세 창은 각 카메라 `80/80` frame, exact same-stamp bundle,
약 10 Hz, receipt p95 `5.87`, `5.87`, `6.01 ms`로 모두 PASS했다. 주행 후 standalone
cadence 검사도 source 간격 `0.100000002 s`, bundle stamp span `0`, bundle
`864/865 = 99.884%`라 PASS했다. 이 검사는 source cadence와 coverage 계약이다.

최종 acceptance gate는 전체 주행의 직렬 publish 시간까지 더 엄격하게 본다. 전체
864 bundle의 receipt p50은 `4.221 ms`였지만 p95가 `45.715 ms`, p99
`58.737 ms`, 최대 `69.120 ms`였다. `40 ms` 초과가 `54/864 = 6.25%`였고 느린
bundle은 simulation `35.5~37.4`, `66.6~68.4`, `80.8~82.7 s` 세 구간에 모였다.
bridge 자체 `camera_bundle_total` 경고도 각 시작점에서 `54.291`, `51.117`,
`64.582 ms`를 기록해 단순 rosbag timestamp 흔들림만으로 볼 수 없다.

record count parity 실패는 별도로 분리했다. 다섯 camera_info topic은 865개,
`CAM_FRONT_LEFT`만 864개다. raw bag을 확인하면 누락된 값은 맨 첫 stamp
`30.200000450 s` 하나뿐이고 이후 `30.300000451~116.600001737 s`의 864개는 모두
exact match다. recorder log에서 좌전방 subscription이 가장 늦게 붙었고, 같은 첫
stamp의 VAD inference는 six-camera 조립 후 정상 publish됐다. 따라서 이는 bridge
중간 frame loss가 아니라 **rosbag 시작 경계 계측 artifact**다. 그러나 사전 선언한
전체 record parity gate를 사후에 완화하지 않았고, 실제 p95 실패도 독립적으로 남으므로
최종 camera transport 판정은 `FAILED`를 유지한다.

### strict v2 계측 하네스 보강 상태

위 v1 실패 원인을 확인한 뒤 후속 실행 계약을
`carla_vad_camera_source_10hz_strict_v2`로 분리했다. 기존 v1 artifact의 당시
규칙·판정·ID는 그대로 보존하며 이름이나 결과를 소급 변경하지 않는다. 현재 validator는
legacy gate semantics의 offline 재검증만 지원하고, 신규 v1 발급이나 byte-identical
runtime replay를 보장하지 않는다. v2에는 다음 fail-closed 조건이 추가됐다.

- rosbag과 PTY relay를 하나의 owned process group에 두고 강제 종료 때도 child가
  남지 않게 한다.
- paused recorder가 6개 camera_info subscription을 모두 알린 뒤에만 SPACE로 재개하고,
  요청/응답 시각과 evidence SHA-256을 route 평가 경계에 묶는다.
- source period의 최대 간격뿐 아니라 최소 간격도
  `0.099995~0.100005 s`로 제한해 숨은 50 ms 중복/가속 frame을 거부한다.
- pre-engagement health는 0.1초 wall-edge guard로 정상적인 직렬 bundle 경계를
  보존하되, window 내부의 모든 camera record가 정확히 한 complete bundle에 소비되지
  않으면 실패한다.
- full bag은 rosbag 시작·종료의 불완전 union stamp를 각 가장자리에서 최대 하나만
  명시적으로 제외하고, 남은 내부 stamp 전부가 six-camera exact 1:1인지 증명한다.
  내부 누락·중복·비증가 stamp나 두 번째 edge partial은 실패한다.
- Portable node/output 부재는 각 8초 winning window 종료 시점의 ROS graph snapshot
  세 개이며 window 전체 연속 관찰을 뜻하지 않는다. launcher의 실행 금지와 함께
  검사한다.
- route 완료 후 Autoware stack을 실제로 종료하고 log가 flush된 다음
  `autoware_carla_interface`의 shutdown-only worker 오류를 다시 검사한다.

이 하네스 보강은 unit/static 회귀와 로컬 ROS recorder 스모크까지 통과했지만, CARLA
Town06 live v2 재실행은 아직 하지 않았다. 따라서 이 문서의 60 km/h 실측 최종값은
계속 v1 `NO-GO`이며 새 PASS를 주장하지 않는다.

host 전체 포화는 관측되지 않았다. route 구간 CPU busy p95는 `58.4%`, GPU SM p95는
`46.9%`였고 iowait, swap, thermal throttle은 없었다. 당시 별도 raw pidstat 조사에서는
첫 지연 구간과 겹쳐 `unattended-upgr`가 최대 한 core 수준으로 관측됐고 GUI/원격
도구도 함께 실행 중이었다. 다만 발행 v1 `runtime_load_analysis.json`은
`unattended_upgrades`와 `remote_desktop`을 별도 그룹으로 저장하지 않았고 원시
pidstat/log도 발행본에서 제외했다. 따라서 발행본만으로 이 프로세스들을 지연 원인으로
재검증하거나 확정할 수 없다. 개선 analyzer의 그룹 분리는 향후 v2 실행에서 발행한다.
고정 직렬 순서는 항상 front에서 시작해 back-right로 끝났다. 현재 발행 근거는 국소 CPU
scheduling, image conversion 또는 DDS publish/backpressure 경로를 후보로 좁히지만
단일 원인을 확정하지는 못한다.

- [차량 중심 Autoware 전체화면](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/05_60kph_readiness/strict10_live_v1/visuals/01_vehicle_centered_fullscreen.png)
- [Autoware 전체 주행 GIF](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/05_60kph_readiness/strict10_live_v1/visuals/02_autoware_drive_5fps.gif)
- [직진 경로·제어 분석 GIF](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/05_60kph_readiness/strict10_live_v1/visuals/10_town06_straight_path_control.gif)
- [속도 실패 그래프](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/05_60kph_readiness/strict10_live_v1/visuals/04_speed_profile.png)
- [카메라/VAD/host runtime 분석](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/05_60kph_readiness/strict10_live_v1/visuals/06_runtime_load.png)
- [최종 기계 판정 JSON](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/05_60kph_readiness/strict10_live_v1/evidence/pilot_acceptance_gate.json)

대표 화면은 RViz 소유 창 전체를 1920×1080으로 보존했고 차량은 우측 지도 viewport
중앙에 있다. 화면의 `36 km/h`는 관측 최고 속도와 일치한다. 우측 하단 `10 fps`는
RViz redraw 설정, 발행 GIF는 5 fps, camera source는 10 Hz로 서로 다른 값이다.
따라서 GIF의 계단감만으로 센서 frame loss를 판단하지 않는다.

### 60 km/h가 나오지 않은 종방향 근거와 다음 순서

명시적 planning overlay와 gated target은 `16.667 m/s`까지 올라갔지만 실제 차량은
`10.123 m/s`에서 멈췄다. gated positive acceleration limit `1.5 m/s²`는 누적
`33.0 s`, accel command 최대 `0.4`와 near-saturation은 `6.3 s`였다. 이후 경로
속도 cap과 종점 감속이 개입했다. 이는 목표 숫자가 전달되지 않은 문제는 아니지만,
현행 map의 속도 축이 `13.89 m/s = 50.004 km/h`에서 끝나고 CARLA 60 km/h 보정이
없으므로 한 원인을 단정하거나 throttle 상한만 올릴 수 없다.

다음 순서는 아래처럼 고정한다.

1. strict 녹화기는 paused 상태에서 모든 six-camera subscription을 확인한 뒤 같은
   measurement 시작점에서 resume해 시작 경계 비대칭을 제거한다.
2. `unattended-upgrades` 같은 background CPU 작업과 불필요한 GUI/원격 화면 부하를
   확인한 뒤 strict transport 1회를 재검증한다. 합격 조건 `exact parity`와
   `receipt p95 <=40 ms`는 유지한다.
3. 긴 직진 calibration route에서 accel/brake map을 최소 `16.667 m/s` 이상으로
   확장하고 controller/gate A/B를 별도 수행한다.
4. production smoothing과 endpoint-C1 frozen snapshot `399/399` geometry preflight를
   통과시킨다. lateral cap을 올려 형상 문제를 숨기지 않는다.
5. 위 조건을 모두 만족한 뒤 Town06 60 km/h behavior qualification 1회만 수행하고,
   하나라도 실패하면 multi-Town/turn A/B로 확대하지 않는다.

과거 5 Hz 실행, strict 10 Hz live v1, geometry·actuation 분석은
[60 km/h readiness 폴더](assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/05_60kph_readiness/)에
각각 분리해 보존했다. 30 km/h로 학습한 Portable physical-v1을 60 km/h 행동 모델로
주장하거나 actuator에 연결하지 않는다.

## 발행본 구조와 검증법

발행 폴더에는 30 km/h 여섯 주요 arm, 독립 C-track 5 m screen, C-track 10 m
geometry failure, 정상/host-stall 카메라 사례, 과거 5 Hz 60 km/h 실행, strict 10 Hz
live v1과 세 geometry preflight를 분리했다. 원시 bag, MKV와 실행 log는 제외했다.

Git clone 뒤 발행 파일이 손상되지 않았는지는 repository root에서 다음처럼 확인한다.

```bash
python3 scripts/e2e/curate_2026_09_07_control_validation.py --verify-only

cd docs/assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1
sha256sum -c 00_summary/SHA256SUMS
```

첫 명령은 manifest에 적힌 214개 관리 파일의 경로, 크기, SHA-256과 PNG/GIF 해상도를
검사하고 목록 밖 파일도 거부한다. `01_vehicle_centered_fullscreen.png`는 전부 정확히
1920×1080이어야 한다. `SHA256SUMS` 자신과 `publication_manifest.json`은 self-hash
순환을 피하려고 checksum 목록에서 제외된다. verifier는 manifest에 기록된
`SHA256SUMS`의 hash와 내용까지 검사하고, manifest 자체는 publication ID와 전체
managed set으로 별도 검증한다. 따라서 관리 파일 214개와 checksum 행 213개의 차이는
정상이다.

원시 artifact가 있는 동일 PC에서 발행본을 처음 다시 생성할 때만 다음 명령을 쓴다.
이미 존재하는 발행 폴더를 묵시적으로 덮어쓰지 않으므로 잘못된 과거 자료와 새 자료가
섞이지 않는다.

```bash
python3 scripts/e2e/curate_2026_09_07_control_validation.py
```

같은 publication ID를 의도적으로 다시 만들 때만 `--replace`를 추가한다. curator는
기존 manifest의 ID가 다르거나 대상이 symlink/일반 디렉터리가 아니면 교체를 거부하며,
새 발행본을 staging한 뒤 원자적으로 바꾼다.

CARLA/Autoware를 처음부터 설치하고 30 km/h strict trial을 재현하는 절차와 private
runtime bundle 인자 준비는 [초보자용 실행 가이드](BEGINNER_QUICKSTART_KO.md)를 따른다.
실차에서는 이 simulation profile을 그대로 사용하지 않는다.
