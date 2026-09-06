# 30 kph Autoware + Portable E2E shadow 검증 자료

이 폴더는 2026-09-06에 같은 runtime·sensor rig·10 Hz 계약으로 실행한 최종 선택
3개 시나리오를 한 곳에 정리한 발행본이다. Town07 직진, C-track 좌회전,
Town03 좌회전 모두 route goal과 정해진 제어 품질 gate를 통과했고, 동시에 실행한
Portable E2E physical-v1 shadow도 각 주행에서 `EVIDENCE_VALID`, 10 Hz
`ESTABLISHED_FOR_SHADOW_ONLY`, 요구 조건 `17/17`을 통과했다.

중요한 해석 경계가 있다. 차량 명령은 **Autoware VAD route-manager hybrid와
Autoware 제어기**가 만들었다. Portable E2E는 같은 센서 입력을 읽고 경로를 출력해
비교한 `shadow_only` 모델이며 차량을 직접 제어하지 않았다. 따라서 이 결과는
Portable 모델의 실차 제어 승인이나 순수 E2E 폐루프 성공을 뜻하지 않는다.

## 결과 요약

| 시나리오 | route | 최고속도 | 최대 CTE | 최대 경로 보정 | 최대 횡가속 | shadow 입력 |
|---|---:|---:|---:|---:|---:|---:|
| Town07 직진 | PASS | 27.65 kph | 0.522 m | 1.800 m | 0.462 m/s² | 547 / 54.6 s |
| C-track 좌회전 | PASS | 16.73 kph | 0.517 m | 12.622 m | 0.707 m/s² | 508 / 50.7 s |
| Town03 좌회전 | PASS | 18.51 kph | 0.494 m | 8.965 m | 0.624 m/s² | 497 / 49.6 s |

공통 한도는 CTE `1.0 m`, 경로 보정 `15.0 m`, 횡가속 `1.8 m/s²`다.
Town07 직진은 `27 kph 이상 1초` gate에 대해 `2.95초`를 기록했다. 회전 경로는
곡률과 횡가속 제한을 우선하므로 최저 지속속도 gate가 `0`이다. 즉 `30 kph profile`
이라는 이름은 목표 상한 profile을 뜻하며 두 회전이 실제 30 kph에 도달했다는 뜻은
아니다.

## 1. Town07 직진

- [전체 Autoware 화면 PNG](01_town07_straight/visuals/01_autoware_vehicle_centered_fullscreen.png)
- [전체 주행 GIF, 5 fps](01_town07_straight/visuals/02_autoware_drive_5fps.gif)
- [경로·제어 애니메이션](01_town07_straight/visuals/03_path_control_animation_5fps.gif)
- [route 결과](01_town07_straight/analysis/01_route_result.png)
- [경로와 제어 비교](01_town07_straight/analysis/02_path_vs_control.png)
- [조향 추종](01_town07_straight/analysis/03_steering_tracking.png)
- [속도 profile](01_town07_straight/analysis/04_speed_profile.png)
- [종방향 응답](01_town07_straight/analysis/05_longitudinal_response.png)
- [E2E 지연](01_town07_straight/analysis/06_e2e_latency.png)
- [간단 수치 JSON](01_town07_straight/evidence/scenario_summary.json),
  [10 Hz shadow 원본 분석](01_town07_straight/evidence/shadow_evidence_analysis.json),
  [실행 route](01_town07_straight/route/source_route.json)

![Town07 centered Autoware view](01_town07_straight/visuals/01_autoware_vehicle_centered_fullscreen.png)

## 2. C-track 좌회전

- [전체 Autoware 화면 PNG](02_c_track_turn/visuals/01_autoware_vehicle_centered_fullscreen.png)
- [전체 주행 GIF, 5 fps](02_c_track_turn/visuals/02_autoware_drive_5fps.gif)
- [경로·제어 애니메이션](02_c_track_turn/visuals/03_path_control_animation_5fps.gif)
- [route 결과](02_c_track_turn/analysis/01_route_result.png)
- [경로와 제어 비교](02_c_track_turn/analysis/02_path_vs_control.png)
- [조향 추종](02_c_track_turn/analysis/03_steering_tracking.png)
- [속도 profile](02_c_track_turn/analysis/04_speed_profile.png)
- [종방향 응답](02_c_track_turn/analysis/05_longitudinal_response.png)
- [E2E 지연](02_c_track_turn/analysis/06_e2e_latency.png)
- [간단 수치 JSON](02_c_track_turn/evidence/scenario_summary.json),
  [10 Hz shadow 원본 분석](02_c_track_turn/evidence/shadow_evidence_analysis.json),
  [실행 route](02_c_track_turn/route/source_route.json)

![C-track centered Autoware view](02_c_track_turn/visuals/01_autoware_vehicle_centered_fullscreen.png)

## 3. Town03 좌회전

- [전체 Autoware 화면 PNG](03_town03_turn/visuals/01_autoware_vehicle_centered_fullscreen.png)
- [전체 주행 GIF, 5 fps](03_town03_turn/visuals/02_autoware_drive_5fps.gif)
- [경로·제어 애니메이션](03_town03_turn/visuals/03_path_control_animation_5fps.gif)
- [route 결과](03_town03_turn/analysis/01_route_result.png)
- [경로와 제어 비교](03_town03_turn/analysis/02_path_vs_control.png)
- [조향 추종](03_town03_turn/analysis/03_steering_tracking.png)
- [속도 profile](03_town03_turn/analysis/04_speed_profile.png)
- [종방향 응답](03_town03_turn/analysis/05_longitudinal_response.png)
- [E2E 지연](03_town03_turn/analysis/06_e2e_latency.png)
- [간단 수치 JSON](03_town03_turn/evidence/scenario_summary.json),
  [10 Hz shadow 원본 분석](03_town03_turn/evidence/shadow_evidence_analysis.json),
  [실행 route](03_town03_turn/route/source_route.json)

![Town03 centered Autoware view](03_town03_turn/visuals/01_autoware_vehicle_centered_fullscreen.png)

## 화면과 10 Hz를 읽는 방법

세 전체 화면은 root desktop이 아니라 실행 stack이 소유한 RViz 창만 1920x1080으로
기록했다. terminal, 알림, dock은 포함하지 않았다. `base_link` 기준 TopDownOrtho를
사용하므로 차량은 모니터 전체 중앙이 아니라 **오른쪽 지도 viewport의 중앙**에
유지된다. 왼쪽에는 전방 카메라와 Routing, Localization, AUTONOMOUS 상태가 보이고,
지도에는 reference, actual, raw/final VAD, Portable shadow를 포함한 7개 경로 topic이
함께 보인다.

GIF는 발행 용량을 위해 5 fps로 인코딩했다. 세 GIF의 모든 frame은 decode됐고
연속 frame byte 중복은 없었지만, 이것만으로 영상의 체감 부드러움을 증명하지는
않는다. 센서 계약은 GIF와 별도로 CARLA source stamp의 **모든 인접 간격**을 검사했다.
세 실행 모두 `100,000,001~100,000,002 ns`, 허용 오차 `±5,000 ns`, 위반 `0`이었다.
집계 평균만 10 Hz인 것이 아니라 중간 source frame 누락이 없다는 뜻이다.

## 알려진 경고와 다음 판정

- C-track의 첫 동일-profile 반복은 최대 경로 보정 `15.966 m`로 `15 m` gate를
  초과했다. 설정이나 gate를 완화하지 않고 새 cold-start로 다시 실행한 선택본이
  `12.622 m`로 통과했다. 따라서 이 한 선택본을 통계적 반복성으로 확대 해석하지
  않으며 다음 단계에서 다회 반복 gate를 적용한다.
- 선택본 3개는 exit code `0`, goal reached, owned cleanup, runtime health와 shadow
  binding 검증을 통과했고 Python traceback은 `0`이다. stack의 `ERROR` 표시는
  VehicleCmdFilter clamp 진단(Town07 30, C-track 26, Town03 28)이며 process 실패 수와
  동일하지 않다. C-track/Town03에는 geometry smoothing rejection 경고도 남아 있어
  후속 A/B tuning 대상으로 기록한다.
- 각 선택본에는 종료 순서 때문에 `Spin thread did not terminate within timeout`
  경고가 한 번 남지만, 뒤이어 CARLA resource cleanup과 interface clean exit가
  확인됐다. 이는 주행 중 10 Hz 입력 누락이나 route 실패가 아니다. campaign 뒤 종료
  순서를 수정한 tracked patch를 추가했고 실제 Town07 SIGINT smoke에서 같은 경고와
  traceback이 0인 것을 확인했다. 기존 선택본의 immutable log 사실은 그대로 기록한다.
- 원시 rosbag, MKV, host telemetry, stack log와 private runtime은 `artifacts/`에만
  보관하며 Git 발행본에는 넣지 않았다. 발행본은 시각 자료, 필요한 분석 결과,
  비식별 evidence와 exact route만 포함한다.

기계 판독용 전체 요약은 [campaign_summary.json](campaign_summary.json), 각 파일의
무결성은 `SHA256SUMS`, 발행 파일 목록은 `publication_manifest.json`에서 확인한다.
세 실행과 같은 calibration 입력은
[CARLA Common10 rig JSON](config/carla_common10_rig.json)으로 exact byte를 함께 발행했다.
