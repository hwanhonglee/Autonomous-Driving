# Town13 카메라 렌더·센서 단회 진단 계획

<!-- HH_260928 - Freeze a one-route render diagnostic before observing Town13 camera pixels. -->

## 목적과 분리된 판정

Town13 source-editor 맵에서 실제 전방 RGB·depth·semantic 카메라가 동기화되어 저장되는지, 정적 OpenDRIVE `Sign_Stop` 후보가 렌더 영상에서 보이는지를 **한 번만 진단**한다. 이 경로에는 정적 표지판 기회가 하나뿐이므로, 서로 다른 실제 표지판 3개를 요구하는 독립 지도 DEV 기준을 충족한다고 주장하지 않는다. CARLA BasicAgent 전문가 차량을 사용하며, 학습 모델 제어나 Autoware 폐루프 시험이 아니다. 이 결과는 학습 데이터셋에 자동 편입하지 않는다.

첫 v1 시도는 촬영 전 소스 변경 감지로 실패해 보존했다. 같은 고정 경로의 새 출력 v2는 [단회 재시도 계약](RETRY_PLAN.md)을 따른다. 아래 표의 `v1` 출력은 첫 시도 당시의 사전 등록값이다.

## 촬영 전 고정 계약

| 항목 | 고정값 |
| --- | --- |
| 소유 source 맵 | `/Game/Carla/Maps/Town13/Town13` (RPC에서 `Carla/Maps/Town13/Town13`도 정확히 허용) |
| 프로필 | `source_editor_0915_4ws` 및 기존 자산 5개 SHA256 계약 |
| 입력 카탈로그 | [v2 소유 결과](../../../../../storage/experiments/validation/2026-09-27/map_town13_route_catalog_v2/owner_result.json), catalog SHA256 `ddd1832dc8456ccd47e1c47206d31e2af84a380fd91bdfb3be2665ae08686c2d` |
| 경로 | `town13_right_s0001_p00`, 시작/끝 spawn 1797/8186, 길이 99.696541m |
| 경로 파일 SHA256 | `6927c0de706e5ffb89afce25a75a892d56f7ee212b577002787605802839520b` |
| 정적 우선 관찰 지점 | road 886, `Sign_Stop` ID 20662; 경로 약 20.876m, 횡거리 약 4.599m. **실제 가시성 미확인** |
| 수집 | 전방 640×360, FOV90°, RGB/depth/semantic, 10Hz × 240 frame, BasicAgent 15km/h 목표, ClearNoon, 빈 월드에서 신호등 무시 명시 |
| 고정 새 출력 | `storage/experiments/validation/2026-09-28/map_town13_render_sensor_scout_v1` |
| 소유 범위 | workspace lock, 로컬 2100–2102, 다른 CARLA/UE4Editor 없음, 종료 시 소유 프로세스 그룹만 정리 |

기존 [정적 경로 순위](../../2026-09-27/map_town13_static_sign_rank/README.md)의 6개 1-sign 경로 가운데 이 경로를 맨 앞 순위로 고른 것이다. 표지판 위치는 카메라 출력으로 선택하지 않았다. 첫 실행이 실패해도 자동 재시도하거나 다른 경로로 바꾸지 않는다. 새 시도는 실패 원인과 새 계획·출력으로 별도 등록한다. 실행 전에는 owner/collector 현재 소스 바이트를 `provenance/`에 기록하고 재해시한다.

## 단계별 허용 판정

1. 소유 실행 `PASS`: 정확한 source 자산/경로 SHA, full map 이름, 240개 RGB·depth·semantic 쌍, 실제 이동·충돌 0, 저장본 replay 일치, 정상 정리·포트 해제를 확인한다. `PASS`는 **센서 진단 성공**만 뜻한다.
2. 별도 객체 QA: 원본 전방 RGB와 semantic에서 동일 물체의 가시 영역, 프레임 수, 픽셀 크기, 거리·가림, XODR ID 연결의 불확실성을 기록한다. 보이지 않으면 실패를 보존한다. 이미지/GIF는 원본 프레임을 명시하며 차량 중심과 관찰 물체를 함께 보여준다.
3. 지도 DEV: 이 경로는 고유 정적 표지판 기회 1개이므로 이 단독 촬영으로는 **불합격**. 서로 다른 물체와 독립 site를 확보하는 새 경로·수집 계약을 별도로 설계한다.
4. GPU 학습/모델 승격: 이 촬영만으로는 하지 않는다. Town02 TRAIN 물체 검수와 독립 sign-capable DEV를 모두 통과한 뒤에만 사전 고정 D0/D1 비교로 넘어간다.

고정 카탈로그의 정적 좌표는 실제 카메라 RGB/semantic 가시성이나 도로 방향 적용성을 증명하지 않는다. 실차 사용 근거도 아니다.
