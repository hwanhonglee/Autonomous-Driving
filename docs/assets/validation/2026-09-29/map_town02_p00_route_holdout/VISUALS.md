# Town02 p00 학습 A/B: 전체 화각 비교 화면

<!-- HH_260929 - Keep fixed-frame map evidence tied to the exact archived A/B inputs. -->

640×360 원본 RGB, CARLA의 7클래스 정답(GT), 학습 팔 A와 B의 예측을 **같은 프레임**에서 비교한다. 네 패널 모두 원본 화각 전체를 유지하며 자르거나 리사이즈하지 않았다. A는 Town01 497장으로, B는 같은 497장에 Town02_Opt p00의 232장을 추가하여 각각 새로 학습했다.

| 자료 | 고정 프레임 PNG | 3장 샘플 GIF |
| --- | --- | --- |
| Town02_Opt p02 내부 경로 보류, 232장 | [처음 000](visuals/p02_000_full_fov_rgb_gt_a_b.png) · [중간 115](visuals/p02_115_full_fov_rgb_gt_a_b.png) · [끝 231](visuals/p02_231_full_fov_rgb_gt_a_b.png) | [p02 GIF](visuals/p02_three_fixed_frames_sampled.gif) |
| Town01 기존 DEV, 118장 | [처음 000](visuals/town01_000_full_fov_rgb_gt_a_b.png) · [중간 058](visuals/town01_058_full_fov_rgb_gt_a_b.png) · [끝 117](visuals/town01_117_full_fov_rgb_gt_a_b.png) | [Town01 GIF](visuals/town01_three_fixed_frames_sampled.gif) |

프레임 번호는 프레임별 점수를 확인하기 전에 처음·시간상 중간·끝으로 고정했다. GIF는 세 정지 화면을 순서대로 넘기는 자료이며 연속 주행 촬영이나 실제 재생 속도가 아니다. 색 범례는 각 PNG에 있고 GT와 A/B에 동일하게 적용했다. 미세한 픽셀 비교에는 색 수가 제한되는 GIF보다 PNG를 사용한다.

이 화면에서 Town02 p02는 과거 학습·분석에 이미 노출된 **내부 경로 보류**이므로 새 독립 DEV 또는 봉인 TEST가 아니다. Town01은 이미 알려진 보호 지표다. [A/B 실행 결과](../../../../../storage/experiments/training/2026-09-29/map_town02_p00_holdout_run_v1/summary.json)는 p02 차선 IoU가 B에서 **+3.305%p** 상승했으나 Town01 차선 IoU는 **−2.496%p** 하락하여 사전 관문에 실패했다. 독립 DEV 장소는 0곳이고, 두 가중치 모두 승격되지 않았다. 이 PNG/GIF는 정적 의미분할 결과이며 HD map 완성, Autoware 주행 화면, 폐루프 주행 성공 또는 실차 가능성의 증거가 아니다.

원본과 산출물의 SHA-256 및 정확한 프레임 번호는 [시각화 매니페스트](visuals/manifest.json)에 기록했다. 소스 데이터 매니페스트 SHA-256은 `f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834`, 실행 요약 SHA-256은 `bd0d9a100ca8e8420cbaea856c4566c3985683e7040782bcd7138f83d1d488f6`이다. 시각화 스크립트 SHA-256은 `f05b37a62af1a3cdb524471de4ba6fad2ba28cf549524215257c0ffac3a0fd70`, 테스트 스크립트 SHA-256은 `acec313e212697480c7beec804bd79e1e3d85828a9c772a77abe77782f1c93bb`이다. 매니페스트 자체 SHA-256은 `2c82ba58e025760128eb3108f5d2d884c5c4816cc0d44fcdc9975882c43de69c`이다.

로컬에 원본 NPZ와 예측 NPZ가 있는 경우 새 출력 폴더에 아래처럼 재생성할 수 있다. 스크립트는 원본 해시와 두 예측의 7×7 혼동행렬을 실행 기록과 대조하며 기존 출력물을 덮어쓰지 않는다.

```bash
python3 tools/workspace_python.py tools/e2e/render_map_town02_p00_holdout_visuals.py \
  --output docs/assets/validation/2026-09-29/map_town02_p00_route_holdout/visuals_reproduction
python3 tools/workspace_python.py -m pytest -- -q \
  learning/tests/test_render_map_town02_p00_holdout_visuals.py
```

현재 검증: 위 테스트 **2 PASS**. 생성된 PNG의 각 전체 화각 패널 픽셀을 원본 RGB·GT·A/B NPZ와 직접 비교했고, 8개 산출물 해시와 GIF 프레임 수를 확인했다.
