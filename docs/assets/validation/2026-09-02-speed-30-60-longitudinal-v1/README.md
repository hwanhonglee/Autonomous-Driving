# 30/60 kph speed and longitudinal validation evidence

이 디렉터리는 2026-09-02 Town06 동일 직선 경로의 30/60 kph 비교와 기존
30 kph 9-map/18-trial matrix의 종방향 후처리 결과를 묶은 SHA 검증 자료다.

결론은 다음처럼 분리한다.

- 60 kph pilot: 경로 완료와 카메라·화면·횡방향 gate는 통과했지만 최고
  `35.195 kph`, `>=54 kph` 노출 `0 s`이므로 주행 판정은 `FAIL`이다.
- 동일 445.880 m 경로 30 kph: 최고 `27.866 kph`, `>=27 kph` 연속
  `45.8 s`, goal reached로 `PASS`다.
- 기존 전체 Town 30 kph: 실행 가능한 9개 맵의 직진·회전 18개 분석이 모두
  `complete`이고 converter current-speed lookup clamp는 0건이다.
- 모든 결과는 explicit simulation speed profile을 사용하는 CARLA screening이며
  `real_vehicle_ready=false`다.

## 주요 파일

- [동일 경로 30/60 비교 그래프](same_route_30_vs_60.png) / [JSON](same_route_30_vs_60.json)
- 60 kph: [전체 화면](60kph/autoware_rviz_fullscreen.png), [주행 GIF](60kph/autoware_rviz_drive.gif), [5-frame 중앙 유지](60kph/autoware_rviz_five_frame_contact.png), [종방향 응답](60kph/longitudinal_response.png), [RTF/load](60kph/runtime_load_analysis.png), [정정된 pilot 요약](60kph/pilot_summary.json)
- 동일 경로 30 kph: [전체 화면](30kph_same_route/autoware_rviz_fullscreen.png), [주행 GIF](30kph_same_route/autoware_rviz_drive.gif), [5-frame 중앙 유지](30kph_same_route/autoware_rviz_five_frame_contact.png), [종방향 응답](30kph_same_route/longitudinal_response.png), [RTF/load](30kph_same_route/runtime_load_analysis.png), [run 요약](30kph_same_route/comparison_run.json)
- 전체 Town 30 kph: [집계 그래프](all_towns_30kph/matrix_longitudinal_response.png), [JSON](all_towns_30kph/matrix_longitudinal_response.json), [CSV](all_towns_30kph/matrix_longitudinal_response.csv), [18장 차량 중심 화면](all_towns_visual_audit/v16_owned_window_contact_sheet.png)

각 `all_towns_30kph/<map>/` 아래에는 직진과 회전의 종방향 응답 그래프가
따로 있다. 기존 전체 화면 PNG/GIF 원본은
[`../2026-09-02-all-towns-camera-source-5hz/`](../2026-09-02-all-towns-camera-source-5hz/README.md)에
보존되어 있다.

## 무결성

```bash
cd /home/a/autoware_e2e/docs/assets/validation/2026-09-02-speed-30-60-longitudinal-v1
sha256sum -c SHA256SUMS
```

`pilot_run_legacy_pre_correction.json`은 최초 wrapper 판정을 삭제하지 않기 위해
보존한 파일이다. actuation target을 converter lookup 속도로 잘못 해석하던 필드는
권위 자료로 사용하지 않는다. 현재 해석은 `pilot_summary.json`,
`actuation_map_runtime_coverage.json`, `same_route_30_vs_60.json` 순으로 확인한다.
