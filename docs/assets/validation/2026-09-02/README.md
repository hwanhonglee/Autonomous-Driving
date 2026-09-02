# 2026-09-02 CTrack 30 kph camera-cadence evidence

이 디렉터리는 `C_track_1_0_7`에서 수행한 CARLA camera source cadence A/B의
SHA-bound 발행본이다. 비교 대상은 다음과 같다.

- baseline: `speed_30kph`, 여섯 RGB camera `sensor_tick=0.0 s`
- candidate: `speed_30kph_camera_source_5hz`, 여섯 RGB camera
  `sensor_tick=0.2 s`
- 공통: ROS publish 5 Hz, `vad_route_manager` BLAS 계열 thread 각각 1,
  직진과 좌회전 route identity 동일

Candidate의 직진과 좌회전은 full-stack route, speed, camera/VAD integrity gate를
모두 통과했다. 여섯 camera의 simulation-stamp cadence는 모두 5 Hz이고 최대 stamp
gap은 `0.200 s`였다.

이 발행본은 CTrack만 선택한 기능·무결성 A/B이므로 matrix 상태가 의도적으로
`INCOMPLETE (1/9 runnable maps)`이다. 전체 Town 검증의 기준 결과는
[`../2026-09-01`](../2026-09-01/README.md)의 9-map, 18-trial PASS다.

## 빠른 화면 확인

| 직진 | 좌회전 |
|---|---|
| ![CTrack straight](c_track_1_0_7/autoware_vad/straight/autoware_rviz_fullscreen.png) | ![CTrack left turn](c_track_1_0_7/autoware_vad/turn/autoware_rviz_fullscreen.png) |

샘플 frame 0/25/50/75/99의 차량 중앙 유지 여부는
[`visual_audit/straight_contact.png`](visual_audit/straight_contact.png)와
[`visual_audit/turn_contact.png`](visual_audit/turn_contact.png)에서 확인할 수 있다.

## 주요 파일

- [`camera_cadence_ab/camera_cadence_ab.md`](camera_cadence_ab/camera_cadence_ab.md): A/B 수치와 integrity gate
- [`camera_cadence_ab/camera_cadence_ab.png`](camera_cadence_ab/camera_cadence_ab.png): 비교 그래프
- [`host_telemetry_summary.json`](host_telemetry_summary.json): route-window 및 campaign-window CPU/GPU 측정 요약
- [`publication_manifest.json`](publication_manifest.json): 범위, source root, route/media 계약
- [`SHA256SUMS`](SHA256SUMS): 이 디렉터리의 발행 파일 무결성

각 trial 디렉터리에는 1920×1080 전체화면 PNG, drive GIF, 경로·제어 분석 그래프,
결과 JSON, runtime/launch/stack log, RViz 및 sensor mapping provenance가 함께 있다.

## 해석 경계

이 결과는 `vad_route_manager_hybrid` simulation screening이다. VAD geometry는
평가했지만 종방향 속도는 explicit CARLA simulation profile이 제공하므로 raw VAD
velocity 검증이나 실제 차량 준비 완료를 뜻하지 않는다. 30 kph는 target profile이며,
직진 PASS 계약은 실제 속도 `>=7.5 m/s`를 1초 이상 유지하는 것이다. 회전은 곡률
제한 경로이므로 30 kph 유지 조건이 없다.
