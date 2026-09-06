# 2026-09-06 30 kph Autoware + Portable E2E shadow 검증

## 결론

Town07 직진, C-track 좌회전, Town03 좌회전을 같은 physical-v1 runtime,
six-camera 10 Hz sensor 계약, Autoware VAD hybrid 제어 조건으로 각각 cold-start해
검증했다. 최종 선택 실행은 **3/3 route PASS**이며, 세 실행에 동시에 붙인 Portable
E2E shadow도 모두 `EVIDENCE_VALID`, `ESTABLISHED_FOR_SHADOW_ONLY`, 요구 조건
`17/17`을 통과했다.

이번 결과에서 차량은 Autoware VAD route manager와 Autoware controller가 제어했다.
Portable E2E physical-v1은 동일 입력으로 경로를 계산했지만 `/planning/portable_e2e/`
격리 topic에만 발행했고 canonical `/planning/trajectory`나 actuator를 소유하지 않았다.
따라서 이 보고서는 **Autoware 폐루프 주행 + Portable 모델의 동시 10 Hz shadow 검증**이지,
Portable 모델이 직접 주행했거나 실차 제어 승인을 받았다는 보고서가 아니다.

정리된 전체 화면, 주행 GIF, 경로·조향·속도·지연 분석과 exact route는
[30 kph v3 발행 폴더](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/)에서
한 번에 확인한다.

## 최종 선택 결과

| 시나리오 | route | 최고속도 | 최대 CTE | 최대 경로 보정 | 최대 횡가속 | shadow 입력 / 구간 |
|---|---:|---:|---:|---:|---:|---:|
| Town07 직진 | **PASS** | 27.645 kph | 0.522 m | 1.800 m | 0.462 m/s² | 547 / 54.6 s |
| C-track 좌회전 | **PASS** | 16.729 kph | 0.517 m | 12.622 m | 0.707 m/s² | 508 / 50.7 s |
| Town03 좌회전 | **PASS** | 18.511 kph | 0.494 m | 8.965 m | 0.624 m/s² | 497 / 49.6 s |

공통 route gate는 goal reached, 최대 CTE `<=1.0 m`, 최대 경로 보정
`<=15.0 m`, 최대 횡가속 `<=1.8 m/s²`다. Town07 직진에는 추가로
`>=27 kph`를 `>=1.0 s` 유지하는 gate가 있으며 실제 `2.95 s`를 기록했다.
두 회전은 곡률과 횡가속 제한이 우선이므로 minimum sustained-speed gate가 `0`이다.
따라서 `30 kph`는 simulation target profile 이름이지 회전 중 계속 30 kph였다는
뜻이 아니다.

모든 선택 실행은 다음도 함께 통과했다.

- CARLA owner preflight, trial completion, owner cleanup과 port 반환
- runtime health, exact map probe와 source/aligned route binding
- Portable runtime bundle, source checkpoint, model config, corpus, rig, contract hash
- shadow measurement arm, zero baseline, complete terminal seal과 bag recheck
- status/output correlation, camera bundle, stale/drop/reject/timeout 분모 검사
- 전체 PNG 1920x1080, GIF decode, owned RViz 창과 shell exclusion 검사

## 10 Hz와 화면 끊김 판정

최신 profile은 CARLA six-camera를 source-native 10 Hz로 생성하고 bridge publish cap을
11 Hz로 둔다. callback이 밀릴 때 가장 최신 frame으로 건너뛰지 않고 가장 오래된
완성 six-camera frame부터 FIFO로 처리한다. 분석기 schema v2는 평균 rate만 보지 않고
모든 인접 source anchor가 `100,000,000 ns ±5,000 ns`인지 검사한다.

| 시나리오 | source rate | source 최소~최대 간격 | 위반 | status wall 최대 간격 | inference p99 / 최대 |
|---|---:|---:|---:|---:|---:|
| Town07 직진 | 9.999999851 Hz | 100.000001~100.000002 ms | 0 | 146.788 ms | 40.408 / 49.778 ms |
| C-track 좌회전 | 9.999999851 Hz | 100.000001~100.000002 ms | 0 | 155.690 ms | 38.482 / 47.436 ms |
| Town03 좌회전 | 9.999999851 Hz | 100.000001~100.000002 ms | 0 | 138.353 ms | 36.862 / 46.283 ms |

세 실행 모두 inference `>100 ms`는 `0`, input reject, stale drop, capacity eviction,
pending expiry, settle timeout과 camera timeout도 `0`이다. trajectory/path의 wall rate도
약 `9.986 Hz`였다. 따라서 이번 선택 실행에서 주행이 끊겨 보이는 원인을 “카메라가
5 Hz라서” 또는 “10 Hz source frame이 중간에 유실돼서”라고 볼 근거는 없다.

RViz 자체는 대표 화면에서 `31 fps`였지만 발행 GIF는 용량을 위해 **5 fps**로 만든다.
GIF가 실제 주행보다 계단처럼 보일 수 있고, decoded frame이 모두 서로 다르다는 검사는
정지·중복 영상이 아님을 보여 줄 뿐 주관적 부드러움을 보장하지 않는다. 남은 약
`138~156 ms` wall gap은 OS scheduling, CARLA rendering, DDS와 분석 callback을 합친
전체 runtime jitter 범위로 기록하며, 다음 성능 A/B에서는 raw MKV와 host telemetry를
같이 비교한다.

## 경로 분석

### Town07 직진

route 길이 `210.598 m` 중 `209.613 m`를 주행하고 goal 잔여 거리는 `0.739 m`였다.
최대 경로 보정 `1.800 m`로 회전 두 경로보다 작았고 27 kph 이상 지속 gate도 통과했다.
다만 자동 진단 분류는 `path_dominant`이므로 raw/final 경로 차이가 없었다고 해석하지
않는다.

- [전체 Autoware 화면](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/01_town07_straight/visuals/01_autoware_vehicle_centered_fullscreen.png)
- [전체 주행 GIF](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/01_town07_straight/visuals/02_autoware_drive_5fps.gif)
- [경로와 제어 비교](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/01_town07_straight/analysis/02_path_vs_control.png)

### C-track 좌회전

route 길이 `73.858 m` 중 `73.287 m`를 주행하고 goal 잔여 거리는 `0.765 m`였다.
최대 CTE는 낮았지만 경로 보정 `12.622 m`가 `15 m` 한도에 상대적으로 가깝다.
첫 동일-profile 실행은 `15.966 m`로 gate를 넘었고, threshold나 설정을 완화하지 않은
새 cold-start 선택 실행이 통과했다. 이는 repeatability 위험을 숨기지 않기 위한 기록이며
단 한 번의 선택 PASS를 통계적 안정성으로 확대하지 않는다.

- [전체 Autoware 화면](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/02_c_track_turn/visuals/01_autoware_vehicle_centered_fullscreen.png)
- [전체 주행 GIF](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/02_c_track_turn/visuals/02_autoware_drive_5fps.gif)
- [경로와 제어 비교](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/02_c_track_turn/analysis/02_path_vs_control.png)

### Town03 좌회전

route 길이 `73.122 m` 중 `72.340 m`를 주행하고 goal 잔여 거리는 `0.784 m`였다.
최대 CTE `0.494 m`, 경로 보정 `8.965 m`, 최대 횡가속 `0.624 m/s²`로 통과했다.

- [전체 Autoware 화면](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/03_town03_turn/visuals/01_autoware_vehicle_centered_fullscreen.png)
- [전체 주행 GIF](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/03_town03_turn/visuals/02_autoware_drive_5fps.gif)
- [경로와 제어 비교](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/03_town03_turn/analysis/02_path_vs_control.png)

## 화면 계약

세 화면은 desktop 전체를 무차별 캡처한 것이 아니라 해당 stack process group이 소유한
RViz 창만 캡처했다. 입력 창 `1850x1016`을 확대·축소하지 않고 좌우 `35 px`, 상하
`32 px` 검은 여백으로 1920x1080에 맞췄다. 차량은 모니터 전체가 아니라 실제 지도 영역인
오른쪽 viewport의 중심 부근에 유지된다. 왼쪽에는 전방 카메라와 Routing,
Localization, AUTONOMOUS 상태가 보이고 지도에는 reference/actual, VAD raw/final과
Portable shadow를 포함한 7개 경로 topic이 동시에 보인다.

Town07/C-track/Town03 전체 주행 GIF는 각각 `410/387/380` frame을 모두 decode했고
각각 `410/387/380`개가 byte 기준으로 고유했다. 경로·제어 GIF도 각각
`227/112/118` frame을 모두 decode했다.

## 로그 경계와 남은 경고

선택 실행 세 개는 exit code `0`, traceback `0`, SIGINT 뒤 exception `0`이고 interface와
관련 process가 clean exit했다. 그러나 log level 문자열을 모두 0이라고 주장하지 않는다.

- VehicleCmdFilter clamp 진단이 `ERROR` level로 Town07 30회, C-track 26회,
  Town03 28회 기록됐다. 이는 formal process failure가 아니지만 후속 종방향 A/B에서
  saturation 분포와 함께 줄여야 한다.
- geometry smoothing rejection 경고가 C-track 69회, Town03 14회 있었다.
- 각 실행 종료에 `Spin thread did not terminate within timeout`이 1회 있었고 바로 뒤
  CARLA resource cleanup과 interface clean exit가 이어졌다. 주행 중 source drop이나
  shadow timeout은 아니다. 원인은 context 종료 전에 spin thread를 join하던 순서였고,
  campaign 뒤 `rclpy.shutdown -> join -> destroy_node` 순서의 재적용 가능 patch를
  추가했다. 실제 Town07 CARLA/interface SIGINT smoke에서 해당 timeout, traceback,
  `ExternalShutdownException`과 process death가 모두 0인 것을 확인했다.
- Portable shadow log의 유일한 공통 경고는 모델이 `shadow_only`이고 canonical control
  topic을 발행하지 않는다는 의도된 안전 경고다.

## 재현 식별자

| 항목 | SHA-256 |
|---|---|
| physical-v1 runtime bundle | `bdd3daf605e269a8d90ea8e56ab1691e7e49db50e3f2100c7b66469813569d4e` |
| source checkpoint | `df2a4b75a978f35c213894c56cf3a905f547734ee8099e8142133bdc9bd3ae09` |
| model config | `cbf591196084509fa96eda35a197c0bc7eb2c8a251cd812d71a109441710d65d` |
| training corpus | `17c248440efca864e6c322ca5a1602d08cd1a0eabfa71e10545049181a86e073` |
| CARLA rig | `9c41a11824585a73c639a9d10b5be032dd578ea0e97f140418a692d41a2390da` |
| Common10 runtime contract | `6f1a7a82abe39a0cc16b42df2b191d2cdae735243a22e8f8cb47bf11faaa4642` |
| sensor mapping | `c4598f8a7920ca8d1df072ed940a17176d9019ad53381ba3cc25bec666ef805c` |
| CARLA wrapper | `460cd66799130b34b8faecf4058aa605e90c61b0d157c8e54d0a82f7d0ca2257` |

runtime bundle과 checkpoint 원본은 private 연구 artifact라 Git에 넣지 않는다. 공개
SHA-256은 동일 입력인지 검증하기 위한 식별자이지 모델 안전 인증값이 아니다.

## 코드와 패키지 회귀 검증

최종 문서와 종료 patch까지 반영한 작업 트리에서 다음 검증을 다시 실행했다.

- 최상위 Python 회귀: `1755 passed, 5 skipped`
- `autoware_e2e_vad_launch/test` 별도 회귀: `362 passed`
- `autoware_carla_interface`, `autoware_map_loader`,
  `autoware_e2e_vad_launch` colcon 결과: `949 tests, 0 errors, 0 failures,
  124 skipped`
- CARLA interface 실제 Town07 four-camera SIGINT smoke: 종료 코드 `0`, spin timeout,
  traceback, `ExternalShutdownException`, process death와 잔존 process/port 모두 `0`
- 발행본 `SHA256SUMS` 전 항목과 원시 선택본 `SELECTED_SHA256SUMS` 전 항목 일치

`skipped`는 ament 환경에서 조건부로 제외되는 검사이며 failure로 바꾸어 숨기지 않았다.
Python directory 두 개에는 이름이 같은 test module이 있으므로 Quick Start에 적은 것처럼
서로 다른 pytest process로 실행했다.

## 다음 진행 순서

1. C-track 좌회전을 여러 cold-start/seed로 반복해 `15 m` correction margin의 분포를
   먼저 확보한다.
2. 30 kph에서 C-track과 Town03의 raw-to-final 보정 및 geometry smoothing을 A/B로
   줄이되 CTE, 횡가속, goal gate는 완화하지 않는다.
3. 같은 source-native 10 Hz/FIFO 계약으로 실행 가능한 모든 Town의 직진·회전을
   확장한다. 준비되지 않은 custom map은 실패 실행으로 포장하지 않고 `BLOCKED`로 둔다.
4. Portable 모델은 candidate collapse와 open-loop 오차를 개선하고, 독립 test split과
   장애물·정지·ACC·차선변경 label/head를 순차 추가한다.
5. learned closed-loop는 독립 safety selector와 fallback/MRM을 붙인 뒤 30 kph 직진부터
   별도 승인한다.
6. 60 kph는 30 kph 반복 기준선 이후 직진 전용 simulation pilot부터 다시 시작하고,
   정지거리, 곡률, actuator saturation과 emergency fallback gate를 별도로 적용한다.

실차 단계는 그 뒤다. 실제 sensor 위치·화각·intrinsic/extrinsic, hardware timestamp,
차량 동역학과 actuation map, 폐쇄 시험장 safety case가 별도로 통과하기 전에는 이
runtime을 actuator에 연결하지 않는다.
