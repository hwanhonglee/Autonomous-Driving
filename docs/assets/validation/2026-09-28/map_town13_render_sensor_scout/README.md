# Town13 실제 카메라 진단: 센서는 통과, 화면 품질은 학습용 미달

<!-- HH_260928 - Separate successful raw sensor plumbing from rejected render quality and map DEV. -->

**결론:** [v2 소유 결과](../../../../../storage/experiments/validation/2026-09-28/map_town13_render_sensor_scout_v2/owner_result.json)는 센서 수집 `PASS`다. 그러나 전방 RGB **240/240장에 노란 skydome 경고가 렌더링**되고 화면 상부가 거의 검게 나와, 이 영상을 정상 주간 카메라 학습/표지판 DEV로 편입하지 않는다. 이 화면은 Autoware나 학습된 E2E 모델이 아니라 CARLA source-editor Town13을 BasicAgent가 주행하며 만든 **카메라 원본**이다. 지도 AI v2/B/C 추론은 실행하지 않았다.

첫 v1은 [변경된 소스 해시 감지로 촬영 전 중단](../../../../../storage/experiments/validation/2026-09-28/map_town13_render_sensor_scout_v1/owner_result.json)했고 보존했다. [새 계획](RETRY_PLAN.md)에 따른 v2는 정확한 Town13 전체 맵, 카탈로그/경로 SHA, 보관된 소스·자산 및 `ready/after_capture/stopped` 영수증을 검사했다. 소유 프로세스와 2100–2102 포트가 모두 해제됐다.

| 원본/화면 검사 | v2 결과 | 해석 |
| --- | ---: | --- |
| 640×360 RGB + depth + semantic 같은 프레임 쌍 | **240/240** 전수 디코드 | 저장본 지도 replay와 live 결과 정확히 일치 |
| 실제 BasicAgent 이동 | **93.786m**, 충돌 0 | 15km/h 목표, 신호등 무시를 명시한 빈 월드 진단. 240프레임 내 종점 도달은 **false** |
| 시맨틱 `traffic_sign` 태그 | 240/240 프레임에 >0 pixel | 서로 다른 표지판 240개가 아니라 같은/다른 객체의 픽셀. 실제 object identity는 미검증 |
| RGB 상단 32px의 노란 경고 픽셀 ≥100 | **240/240** | 화면에 `YOUR SCENE CONTAINS A SKYDOME MESH ...` 문구가 찍힘 |
| 중앙 상부 x=100–499/y=30–74의 거의 검정 pixel 비율 | 평균 **98.636%** | ClearNoon 설정이어도 정상 주간 배경처럼 보이지 않음. 해당 ROI만의 관찰값 |
| frame 60 / 230의 시맨틱 표지판 ROI | 각각 **11,885 / 994 pixel** | RGB에서 속도 제한판과 STOP 표지처럼 보이는 영역에 tag 8이 있음. XODR ID와의 동일성·도로 적용성 미검증 |

![전방 카메라 frame 60: 속도 제한판과 상단 경고](forward_sign_frame060.png)

![전방 카메라 frame 230: STOP 표지와 상단 경고](forward_stop_frame230.png)

[초반 표지판 접근 GIF](early_sign_approach.gif) · [후반 STOP 접근 GIF](stop_approach.gif). GIF는 원본 10Hz 중 48–73/210–239번 연속 프레임을 **8fps 재생**한 것이어서 실시간 속도를 나타내지 않는다. PNG는 대응하는 `frames/0060.png`와 `frames/0230.png`의 같은 해상도 재인코딩이다. 원본 SHA256은 각각 `147b93d105d0871c9281184e3e21dc6a7e75e1ba5e219b93c7ce4509f4549a9c`, `4d358be0ac777b2a10e247337ea89cfbfcea629c717b9efa1663ba4da9021c7a`다. 원본 240쌍과 [summary](../../../../../storage/experiments/validation/2026-09-28/map_town13_render_sensor_scout_v2/mapping/summary.json)는 `storage/`에 보존했다.

정적 XODR 순위가 이 경로에서 가리킨 `Sign_Stop` ID 20662는 시작 약 20.876m에 있다. frame 60의 실제 이동거리는 약 20.19m지만 화면의 큰 표지는 속도 제한판처럼 보인다. frame 230에는 STOP처럼 보이는 표지가 보인다. **가까운 거리와 semantic tag만으로 어느 RGB 물체가 그 XODR ID인지 연결하지 않는다.** 기존 정적 순위의 “한 경로 한 stop/yield 기록”과 렌더의 다른 종류 표지판은 서로 다른 범위의 집계다.

소스 로그에는 `BP_Sky_0` 갱신과 RGB/depth/semantic 센서 생성이 있고, source level에 하늘 관련 액터 참조도 있다. 따라서 단순히 `ClearNoon`을 빠뜨렸거나 sky actor가 없다고 단정하지 않는다. Unreal의 sky material과 dome 커버리지 상호작용, Town13의 촬영 위치/하늘 액터 중심 차이가 **가설**이지만 현재 로그는 dome 반경·transform을 기록하지 않아 원인 확정은 아니다. 다음은 자산을 수정하지 않고 동일 맵·날씨·렌더·카메라 조건에서 **촬영 위치만** 문제 지점과 sky actor 중심 근처 drivable 지점으로 바꾼 소규모 사전 등록 A/B다. 임의의 영상 밝기 보정만으로 원본 품질을 합격 처리하지 않는다. 그 뒤에도 [3개 표지판 거리·연결성 조사](../map_town13_sign_cluster_feasibility/README.md)에 따라 240프레임보다 긴 정당화된 route/capture를 별도 사전 등록해야 한다. 현재는 표지판 독립 DEV, 지도 모델 재학습/승격, HD map 완성, 주행 30개 기능 어느 것도 승인하지 않았다.
