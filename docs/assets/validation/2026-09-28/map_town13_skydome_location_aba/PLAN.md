# Town13 정지 카메라 위치 A→B→A 진단: 실행 전 고정 계획

<!-- HH_260928 - Pre-register one owned stationary comparison of camera location before source-editor launch. -->

## 질문과 판정 범위

Town13 source-editor의 상단 하늘 경고와 검은 영역이 **촬영 위치에 따라 달라지는지** 확인한다. A1→B→A2 순서로 같은 카메라 액터를 옮겨 같은 자세로 촬영한다. 이는 하늘 돔 원인 확정 시험이나 지도 모델 검증이 아니다. 차량을 생성하지 않고 주행·지도 학습·모델 추론을 하지 않는다.

기존 [v2](../map_town13_render_sensor_scout/README.md)의 첫 카메라 행렬을 A 위치 정의에만 사용한다. v2는 차량에 부착한 이동 카메라였고 이 실험은 차량 없는 정지 카메라다. **새 A1/B/A2 내부 비교만** 위치 효과를 논할 수 있으며 v2와 픽셀 단위 동일 장면을 주장하지 않는다.

## 입력과 촬영 조건

| 항목 | 실행 전 고정값 |
| --- | --- |
| 소유 맵 | `/Game/Carla/Maps/Town13/Town13`; 프로필 `source_editor_0915_4ws`, 기존 5개 자산 SHA 검증 |
| source 실행 | UE4Editor `-game -vulkan -prefernvidia -quality-level=Epic -RenderOffScreen -nosound -unattended` |
| 카탈로그 | 2026-09-27 v2; SHA256 `ddd1832dc8456ccd47e1c47206d31e2af84a380fd91bdfb3be2665ae08686c2d` |
| A 경로 기준 | `town13_right_s0001_p00`, spawn 1797, SHA256 `6927c0de706e5ffb89afce25a75a892d56f7ee212b577002787605802839520b` |
| B 경로 기준 | `town13_right_s0002_p00`, spawn 2198, SHA256 `459b2c5d2e1bfaaa2926a5aa4e67bdfcddfbc59e49dd5c762431a1dd854c4386` |
| A 카메라 행렬 | v2 `mapping/frames/0000.npz`의 `world_from_camera`; 파일 SHA256 `7efdde3ba4231e30c7dc5d87bb8ce36c586d3d3943ff08d2dcba8fb23dd25b55` |
| A 카메라 위치 | CARLA `(-1822.863892, -789.351318, 156.897781)`m |
| B 이동량 | 두 경로 시작점 차이 `(+4535.238647, +1659.079651, +11.317398)`m; 평면 거리 4829.175383m |
| B 카메라 위치 | CARLA `(2712.374756, 869.728333, 168.215179)`m; A와 **같은 3×3 회전 행렬** |
| 카메라 | 월드에 직접 배치한 단일 RGB; 640×360, FOV 90°, sensor_tick 0, lens_k/lens_kcube 0, 후처리 false |
| 시간/날씨 | 렌더 켠 동기식 10Hz, ClearNoon; A1/B/A2 각각 30 tick 안정화 후 10 PNG 저장, 총 120 tick/30 PNG |
| 출력 | 새 경로 `storage/experiments/validation/2026-09-28/map_town13_skydome_location_aba_v1`; 기존 v1/v2 증거는 그대로 유지 |

A 기준의 실제 회전 값은 보관된 4×4 행렬을 CARLA transform으로 변환해 검증한다. B를 sky actor 중심이라고 부르지 않는다. 기존 로그에는 `BP_Sky_0`의 위치·반경이 없어 두 지점의 차이는 지도 내 지리적 위치, 장면 구조, 지형 고도 및 스트리밍 영역 차이를 함께 포함한다.

## 소유 및 실패 조건

소유자는 실행 전에 공용 workspace 비차단 lock을 잡고 다른 CARLA/UE4Editor 프로세스나 2100–2102 점유가 있으면 출력 생성 전 중단한다. 실행 소스·카탈로그·A/B 경로·v2 카메라 행렬·v2 성공 영수증을 새 결과의 `provenance/`에 복사하고 SHA를 다시 검사한다. 정확한 source map 전체 경로, source-editor 자산 해시, 시작·종료 RPC 영수증을 확인한다. 종료 시 소유한 process group만 정지하고 포트 해제를 확인한다. 실패 시 새 결과를 보존하며 자동 재시도하지 않는다.

worker는 초기/매 phase에 차량 없음, 모든 저장 카메라 이미지의 실제 transform이 고정 위치·회전과 일치하는지 확인한다. 각 원본 PNG의 상단 32px에서 `(R≥130,G≥120,B<100)`인 픽셀 ≥100개면 경고 탐지로 기록한다. 중앙 상단 `(x=100..499,y=30..74)`의 `max(R,G,B)<20` 비율을 기록한다. 이 수치는 장면 품질 관찰값이며 소유 실행 `PASS`는 오직 진단 수집 무결성 뜻이다. 정상 하늘 렌더, map DEV, 학습 데이터 승인 기준은 별도다.
