# Town07 직진 comfortable_v3: 두 번 모두 미승인

<!-- HH_260906 - Preserve both same-condition attempts and separate scalar motion checks from control-record alignment. -->

같은 소스·조건으로 선언한 2회 실행 전체입니다. **속도·정지 스칼라 검사: 1회 실패 / 1회 충족. 제어 기록 연속성: 2회 모두 실패. 학습 데이터 승인: 0회.**

Town07 / Prius / ClearNoon / Low / 경로 210.598 m. 명령 순항은 28.8 km/h이고 실제 속도 상한은 30 km/h입니다. BasicAgent 전문가 수집이며 학습 모델 또는 Autoware 실시간 구동 화면이 아닙니다.

| 실행 | 20 Hz 상태 / 10 Hz 카메라 앵커 | 최고 속도 km/h | 연속 순항 s | 최종 목표거리 m | native 최소 가속도 m/s² | camera 최소 가속도 m/s² | 스칼라 검사 | 직전 제어 요청 불일치 |
|---|---:|---:|---:|---:|---:|---:|---|---:|
| run_001 | 1468 / 734 | 28.55377 | 8.65 | 0.831468 | -8.860301 | -4.751103 | FAIL | 79 / 1467 |
| run_002 | 1478 / 739 | 28.61337 | 8.60 | 0.845827 | -1.946938 | -0.903995 | 충족 | 85 / 1477 |

두 실행 모두 목표점 상류 1 m 안에서 0.1 m/s 이하로 2초 정지한 뒤 6.5초 tail을 기록했고 충돌·차선침범·비상개입은 0입니다. 연속 순항 조건은 7.8–8.2 m/s에서 5초 이상입니다. 1차 감속 위반은 native frame 3391→3392이며 decoder ±2.9 및 runtime +3/−6 m/s² 한계를 넘었습니다. 카메라 간격 감속도 decoder 한계를 넘었습니다. 한계나 라벨을 완화하지 않았습니다.

## 제어 기록의 별도 실패

1차 79개, 2차 85개 프레임에서 `current_control`이 직전 행의 `next_control`과 일치하지 않았습니다(페달·조향 절대오차 1e−6 기준). 모두 두 행 전 요청과는 일치하지만, 이것만으로 실제 액추에이터 지연인지 관측 캐시 지연인지 증명할 수 없습니다. 프레임을 재정렬하거나 적용 시점을 추정해 PASS로 바꾸지 않았습니다. 원래 JSON의 `*applied_control` 키와 화면의 Reported control은 물리 적용 시점 보증이 아닙니다.

기록된 실행 소스 10개는 시작 SHA·보관 바이트·종료 SHA·기록 Git 커밋과 모두 일치합니다. 2차에는 실행하지 않는 분석 파일의 수정 상태가 있었으므로 전체 작업트리가 깨끗했다고 주장하지 않습니다.

## 실제 카메라·경로 화면

6개 카메라의 전체 화각을 여백으로 맞추고 지도 패널의 차량을 중앙에 고정했습니다. 경로·과거 위치·관측된 미래 위치를 함께 보여줍니다. GIF는 카메라 5프레임마다 하나를 10 fps로 재생하는 약 5배속이며, 끝 프레임 포함으로 정확한 재생시간은 provenance에 기록했습니다. 10 Hz 모델 추론 성능 증명이 아닙니다.

화면에 보이는 체크무늬 도로 재질은 원래 Low 설정 카메라 영상에 있습니다. 보정·가림 처리하지 않았으며 Epic 설정을 포함한 시각 자산 품질 검사가 별도로 필요합니다. 전체 원본 JPEG 픽셀·전체 미래 XY 적합성 검사는 아직 완료되지 않았고, 표시한 원본 JPEG들의 SHA만 추가 검증했습니다.

### run_001

[전체 기록 GIF](visuals/run_001/whole_recording_accelerated.gif) · [시각화 provenance](visuals/run_001/visual_provenance.json)

[시작](visuals/run_001/01_start.png) · [순항](visuals/run_001/02_measured_cruise.png) · [코스트 진입](visuals/run_001/03_coast_entry.png) · [목표 정지](visuals/run_001/04_goal_dwell.png) · [마지막 관측](visuals/run_001/05_final_observation.png) · [최대 감속](visuals/run_001/06_maximum_observed_deceleration.png)

### run_002

[전체 기록 GIF](visuals/run_002/whole_recording_accelerated.gif) · [시각화 provenance](visuals/run_002/visual_provenance.json)

[시작](visuals/run_002/01_start.png) · [순항](visuals/run_002/02_measured_cruise.png) · [코스트 진입](visuals/run_002/03_coast_entry.png) · [목표 정지](visuals/run_002/04_goal_dwell.png) · [마지막 관측](visuals/run_002/05_final_observation.png) · [최대 감속](visuals/run_002/06_maximum_observed_deceleration.png)

## 재검증 범위

[독립 계측 보고서](summary.json) · [원본/공개본 SHA 연결](publication_manifest.json) · [전체 공개 파일 SHA256](SHA256SUMS)

공개 JSON은 개인정보 경로를 치환한 메타데이터 뷰이며 원본 SHA를 보존합니다. PNG/GIF는 원본 렌더 바이트 그대로입니다. 원본 주행·실패 기록은 비공개 artifacts에 보존되며 이 폴더만으로 원본을 재실행할 수는 없습니다. 테스트셋·GPU 학습·모델 제어·자동 배포는 수행하지 않았습니다.
