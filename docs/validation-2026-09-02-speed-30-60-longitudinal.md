# 30/60 kph 속도·종방향 검증

> **Historical supporting result.** 현재 단일 검토 진입점은
> [runtime·제어 캠페인 보고서](validation-2026-09-02-runtime-control-campaign.md)와
> [정리 화면 폴더](assets/validation/2026-09-02-runtime-control-campaign-v1/README.md)다.
> 이 문서와 별도 asset 폴더는 이후 depth-1 v3 이전의 동일 경로 비교 및 전체 Town
> 종방향 후처리 근거이므로 중복 실패 찌꺼기로 삭제하지 않고 historical로 보존한다.

> 2026-09-02에 Town06의 동일한 445.880 m 직선 경로로 60 kph pilot 1회와
> 30 kph 기준 주행 1회를 full-stack으로 실행했다. 기존 전체 Town 30 kph
> 9-map/18-trial은 새로 주행하지 않고 보존된 bag에 종방향 후처리를 추가했다.

## 결론

- **60 kph pilot은 FAIL**이다. 목표 지점에는 도착했지만 최고속도는
  `35.195 kph`였고, 판정 조건인 `>=54 kph` 연속 1초 노출은 `0 s`였다.
- **동일 경로 30 kph 기준 주행은 PASS**다. 최고속도 `27.866 kph`,
  `>=27 kph` 연속 `45.8 s`, goal reached를 기록했다.
- 두 주행 모두 6-camera source `5 sim-Hz`, 최악 stamp gap `0.200000003 s`,
  VAD queue drop 0, 차량 중심 RViz 전체화면과 후보·기준·최종 경로 표시를 확인했다.
- 전체 Town 30 kph 저장 주행의 종방향 분석은 **18/18 complete**다. 실제
  converter lookup은 모두 actuation-map 속도축 내부였고 runtime clamp는 0건이다.
- 이 결과는 CARLA simulation screening이다. VAD geometry는 평가하지만 raw VAD
  cruise velocity는 평가하지 않았고 `real_vehicle_ready=false`다.

[SHA 결속 비교 JSON](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/same_route_30_vs_60.json)과
[publication manifest](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/publication_manifest.json)가
권위 기록이다.

![동일 경로 30/60 kph 비교](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/same_route_30_vs_60.png)

## 동일 경로 실행 결과

두 주행의 `aligned_route.json` SHA-256은 모두
`092125a8bf07276719242b0cb9e8a087031838b9e8fb0f9cb8f37f4cd4b75996`다.
Town06 직선 경로의 길이는 `445.879952 m`이며, 서로 byte-identical임을 확인했다.

| 항목 | 30 kph 기준 | 60 kph pilot |
|---|---:|---:|
| 주행 판정 | **PASS** | **FAIL** |
| 목표속도 | 30.000 kph | 60.000 kph |
| 최고 관측속도 | 27.866 kph | 35.195 kph |
| 목표 대비 최고속도 | 92.885% | 58.659% |
| 판정 속도 / 요구시간 | >=27 kph / 1 s | >=54 kph / 1 s |
| 연속 노출 | 45.800 s | 0.000 s |
| goal reached | true | true |
| 이동거리 | 445.129 m | 445.137 m |
| 최대 절대 CTE | 0.532 m | 0.525 m |
| 최대 횡가속도 | 0.423 m/s² | 0.358 m/s² |
| simulation / wall time | 70.650 / 284.367 s | 78.100 / 148.252 s |
| result 기준 RTF | 0.248 | 0.527 |
| camera bundle | 99.729% | 100.000% |
| 최악 camera stamp gap | 0.200 s | 0.200 s |

60 kph는 단순히 goal reached만으로 성공 처리하지 않았다. 최초 wrapper가 만든
`pilot_run.json`은 이 해석 이전 기록으로 보존하고, 정정된
[`pilot_summary.json`](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/60kph/pilot_summary.json)을
최종 판정에 사용한다.

## 종방향 응답

| 항목 | 30 kph 기준 | 60 kph pilot |
|---|---:|---:|
| gated target - actual RMSE | 1.485 m/s | 6.644 m/s |
| gate 양의 가속도 상한 sample duty | 19.838% | 43.547% |
| gate 상한 최장 연속시간 | 8.900 s | 8.300 s |
| throttle near-saturation sample duty | 0.000% | 4.014% |
| throttle near-saturation 최장시간 | 0.000 s | 3.400 s |

60 kph 주행은 gated target 평균 `11.105 m/s`에 대해 실제 속도 평균이
`5.192 m/s`였고, 목표 추종 오차와 gate 제한 duty가 30 kph보다 크게 증가했다.
동시에 throttle near-saturation은 전체의 약 4%뿐이므로, 현재 증거는 단순한
actuator 최대출력 고정 상태보다 목표 생성·가감속 제한·제어 응답을 함께 조정해야
함을 가리킨다. 단일 원인은 아직 분리하지 않았다.

60 kph target envelope `16.667 m/s`는 현재 actuation map의 최고 속도축
`13.89 m/s`를 넘는다. 다만 converter가 조회하는 값은 target이 아니라 현재 odometry
종속속도의 절댓값이다. 이번 실제 최고속도 `9.776 m/s`에서는 축을 넘지 않았고
runtime clamping도 관측되지 않았다. 60 kph를 실제로 도달시키기 전에는 map 축 확장과
차량별 calibration을 별도 gate로 완료해야 한다.

![60 kph 종방향 응답](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/60kph/longitudinal_response.png)

![30 kph 종방향 응답](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/30kph_same_route/longitudinal_response.png)

## 경로 분석

비교 경로 자체는 최대 chord deviation 약 `0.000002 m`, 최대 곡률 약
`4.8e-7 /m`인 직선이다. 60 kph 주행의 VAD raw path는 기준 경로에서 최대
`5.155 m`(`p95 1.166 m`) 벗어났고, raw-to-final 개입은 최대 `4.656 m`
(`p95 1.780 m`)였다. 최종 path는 기준 경로 대비 최대 `0.500 m`로 제한됐고,
차량의 final path 추종 오차는 최대 `0.06196 m`(`p95 0.03391 m`)였다.

즉 차량이 최종 궤적은 잘 따라갔지만 raw VAD geometry에는 큰 보정 개입이 있었다.
분석 판정 `path_dominant`를 유지하며, 이 60 kph goal 도착 화면을 raw VAD 경로 품질
완료로 해석하지 않는다.

![60 kph 경로와 제어](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/60kph/path_vs_control.png)

## 차량 중심 Autoware 화면

두 주행의 대표 PNG는 `1920x1080`, GIF는 `960x540`이다. owned RViz window에서
차량을 중앙에 두고 기준 경로, VAD 후보, 최종 경로, 전방 카메라, 주행 상태가 함께
보이도록 캡처했다.

### 60 kph pilot

![60 kph 전체화면](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/60kph/autoware_rviz_fullscreen.png)

![60 kph 차량 중심 주행](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/60kph/autoware_rviz_drive.gif)

[5-frame 차량 중심 contact sheet](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/60kph/autoware_rviz_five_frame_contact.png)와
[candidate 원본 PNG](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/60kph/autoware_rviz_candidate.png)도
같이 보존했다.

### 동일 경로 30 kph

![30 kph 전체화면](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/30kph_same_route/autoware_rviz_fullscreen.png)

![30 kph 차량 중심 주행](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/30kph_same_route/autoware_rviz_drive.gif)

[5-frame 차량 중심 contact sheet](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/30kph_same_route/autoware_rviz_five_frame_contact.png)와
[candidate 원본 PNG](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/30kph_same_route/autoware_rviz_candidate.png)도
같이 보존했다.

## 화면 끊김 원인 재검증

5 Hz source는 RTF 1에서도 화면을 20/30/60 fps보다 계단식으로 보이게 한다. 하지만
이번에 보인 심한 멈춤은 source stamp 누락이 아니라 낮은 RTF와 여섯 camera의 wall-time
전달 지연이 겹친 결과다.

| 구간 | RTF | 화면 기준 VAD/camera 갱신 | 6-camera 동일 stamp 수신 폭 | VAD inference 평균 |
|---|---:|---:|---:|---:|
| 30 kph 전체 | 0.248 | 1.239 Hz | 평균 648.25 ms | 34.04 ms |
| 60 kph 초기 저속구간 | 0.249 | 1.245 Hz | 평균 647.11 ms | 34.01 ms |
| 60 kph 회복구간 | 0.999 | 4.993 Hz | 평균 9.46 ms | 31.74 ms |

30 kph 주행은 early RTF `0.2498`에서 late `0.2459`로 VAD 전체 분석 window
`310.6 wall-s` 동안 회복하지 않았다. 반면 60 kph 주행은 simulation `30.1 s`
부근에서 RTF가
`0.2490 -> 0.9986`으로 급회복했고 camera 수신 폭도 약 68배 줄었다. 30 kph 차량은
그 60 kph 회복 위치를 약 `0.122 m` 이내로 지나갔지만 RTF 약 `0.2497`, 수신 폭
`644.05 ms` 상태가 유지됐다.

30 kph 구간의 host CPU 평균은 `9.68%`, GPU 평균은 `14.91%`, iowait와 swap은
0이었다. VAD inference도 약 34 ms로 안정적이고 queue drop이 없었다. 따라서 현재
자료로는 다음 항목이 심한 끊김의 충분조건이 아니다.

- camera source Hz 또는 frame 누락
- VAD inference 시간
- PC 전체 CPU/GPU 포화
- 요청 목표속도
- 해당 도로 위치나 단순한 경과시간 warm-up

병목 범위는 stateful CARLA render/scheduler, image conversion, reliable DDS publish,
여섯 camera worker 동기화·backpressure 경로로 좁혀졌다. 다만 aggregate idle이 높아도
CARLA 단일 game/render thread나 render fence 대기는 배제할 수 없으며, 이번 관찰만으로
그중 하나를 확정하지 않는다.

![30 kph 지속 저 RTF](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/30kph_same_route/runtime_load_analysis.png)

![60 kph 일시 저하 후 회복](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/60kph/runtime_load_analysis.png)

## 전체 Town 30 kph 고도화 결과

기존 9개 실행 가능 맵의 직진·회전 18개 선택 trial에 동일 분석기를 적용했다.
개별 주행·화면·camera gate는
[전체 Town 보고서](validation-2026-09-02-all-towns-camera-source-5hz.md)에 있고,
이번에는 종방향 목표·실속도·가감속 gate·actuation-map lookup을 추가했다.
여기서 18/18은 canonical 19개 중 실행 자산이 확인된 9개 맵만을 뜻하며, 나머지
10개 맵의 `BLOCKED` 상태를 다른 맵 결과로 대체하지 않았다.

| 집계 | 직진 9개 | 회전 9개 |
|---|---:|---:|
| 최고속도 범위 | 27.644-27.865 kph | 16.983-24.227 kph |
| 목표 대비 최고속도 범위 | 92.147-92.884% | 56.611-80.758% |
| gate 상한 sample duty 범위 | 14.244-41.869% | 7.108-30.542% |
| gated target - actual RMSE 범위 | 1.745-2.134 m/s | 1.793-2.651 m/s |
| throttle near-saturation | 전부 0% | 전부 0% |
| runtime velocity clamp | 0/9 | 0/9 |

- `town07/straight`의 gate 상한 duty가 `41.869%`로 가장 높았다.
- `c_track_1_0_7/turn`은 최고속도 `16.983 kph`로 가장 낮고 추종 RMSE
  `2.651 m/s`로 가장 높았다.
- `town03/turn`의 측정 가속도 outlier 비율이 `1.838%`로 가장 높았다.
- 회전 trial에는 곡률·횡가속 제한이 있으므로 30 kph 유지 노출 gate를 적용하지 않았다.

![전체 Town 종방향 집계](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/all_towns_30kph/matrix_longitudinal_response.png)

![전체 Town 차량 중심 18장](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/all_towns_visual_audit/v16_owned_window_contact_sheet.png)

맵별 직진·회전 그래프는
[`all_towns_30kph/`](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/all_towns_30kph/)에
각각 분리했다.

## 시도 이력과 멈춤 상태

동일 경로 30 kph의 `attempt_001`은 guarded trial 디렉터리에 lifecycle JSON이 먼저
생겨 pre-trial에서 거절됐다. 차량과 Autoware 주행은 시작되지 않았으며 기록을 삭제하지
않았다. 선택된 `attempt_002`는 PASS다.

주행 종료 후 남은 PID/PGID `2460883`은 이 실행이 띄운 `pidstat` telemetry였고,
파일 descriptor 때문에 runtime lock을 잡고 있었다. cleanup helper로 종료한 뒤 CARLA
포트 `2000/2100` 미사용과 lock 해제를 확인해
[`post_run_owned_telemetry_cleanup.json`](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/30kph_same_route/post_run_owned_telemetry_cleanup.json)에
결속했다. 따라서 그 lock busy는 다른 사용자의 CARLA 작업 근거가 아니다. 이것은 주행
중 화면 끊김 원인과도 별개의 종료 정리 문제다.

## 다음 단계

1. 30 kph 기준선에서 camera worker 단계별 wall timestamp를 추가하고, reliable DDS,
   image conversion, pre-roll/cache 조건을 하나씩 A/B 비교한다.
2. `town07/straight`, `c_track_1_0_7/turn`, `town03/turn`을 우선 대상으로 target
   smoothing, acceleration gate와 PID 응답을 조정한 뒤 반복 주행으로 회귀를 막는다.
3. 60 kph는 actuation map 속도축·제동거리·경로 길이·반복성 gate를 먼저 확장하고,
   동일 직선에서 54 kph 노출을 통과시킨 뒤 곡률별 제한이 있는 회전 프로필로 진행한다.
4. 실제 차량 적용은 vehicle-specific actuation calibration, 안전 운전자·폐쇄 구간,
   비상정지와 독립 safety gate가 준비되기 전에는 시작하지 않는다.

## 자료와 무결성

- 문서용 전체 묶음: [`docs/assets/.../2026-09-02-speed-30-60-longitudinal-v1`](assets/validation/2026-09-02-speed-30-60-longitudinal-v1/README.md)
- 60 kph 원본: [`artifacts/.../autoware_vad_60kph_town06_straight_pilot_v1`](../artifacts/validation/2026-09-02/autoware_vad_60kph_town06_straight_pilot_v1/)
- 동일 경로 30 kph 원본: [`artifacts/.../autoware_vad_30kph_town06_long_straight_comparison_v1`](../artifacts/validation/2026-09-02/autoware_vad_30kph_town06_long_straight_comparison_v1/)
- 전체 Town 30 kph 원본: [`artifacts/.../autoware_vad_town_matrix_30kph_camera_source_5hz_v1`](../artifacts/validation/2026-09-02/autoware_vad_town_matrix_30kph_camera_source_5hz_v1/)

최종 회귀 검증은 관련 Python test `225 passed`,
`autoware_e2e_vad_launch` package build 성공 및 colcon test `258 tests, 0 errors,
0 failures`, 변경 Python 17개 flake8, shell 3개 `bash -n` 통과다. 문서 묶음은
checksum 자신을 제외한 68개 파일을 아래 명령으로 검증했다. 비교·집계 단계는
route/result/rosbag/latency/runtime/actuation 원본의 SHA 결속이 하나라도 맞지 않으면
결과를 만들지 않는 fail-closed 계약으로 재검증했다.

```bash
cd /home/a/autoware_e2e/docs/assets/validation/2026-09-02-speed-30-60-longitudinal-v1
sha256sum -c SHA256SUMS
```
