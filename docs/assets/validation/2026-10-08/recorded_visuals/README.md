# 차량 중심 기록 화면: 전체 GIF와 처음·중간·끝 PNG

<!-- HH_261008 - Publish actual stored-state renderings with whole declared intervals, explicit absent outputs and honest visual/control provenance. -->

**전체 기록 렌더링을 완료했습니다.** 직접 추론918프레임·ROS 실제 관측29프레임의 GIF 두 개, 처음·중간·끝 PNG3장, 전체1,836행 상태 CSV를 보존했습니다. 실제38.39초, 렌더 실패0·종료 오류0입니다. 최초 결과의 용량 실패도 삭제하지 않았습니다.

이것은 **실제 저장된 위치·상태·선택 궤적에서 만든 진단 화면**입니다. CARLA 카메라 주행 촬영, Autoware/RViz 화면 캡처, 새 자율주행이나 실제 차량 몸체 그림이 아닙니다. 기록된 제어는 시작 구간의 expert 이후 learned 연구 실행이지만, 개별 프레임의 제어 출처는UNKNOWN으로 유지합니다. 원본 공개 ROS 출력 좌표도 보존되지 않았으므로 빨간 선을 실제 수신된 공개 메시지 좌표라고 부르지 않습니다.

## 먼저 중간 화면을 보세요

![실제 중간 기록459: 차량 기준점·원래 경로·주행 흔적·모델 선택 궤적](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v9/source_0459_DIRECT_ROS_status.png)

검은 점은 **기록된 rear-axle/base_link 기준점**이며 두 패널 가운데입니다. 회색은 원래 내비게이션 경로, 파랑은 그 시점까지 실제 이동한 흔적, 빨강은 저장된 모델 선택64점입니다. 왼쪽은 차량 좌표의근거리±24m이며 범위 밖 궤적은 잘릴 수 있습니다. 오른쪽은 차량을 중심으로 원래 전체 경로·그때까지 흔적·저장 궤적을 포함하도록 범위를 잡습니다. 실제 도로·차선 지도나 HD-map 그림은 아닙니다.

중간 화면에 `ROS same source: UNEXECUTED`가 표시됩니다. 원래 ROS 실험은 앞29개만 관측하고 실패했으므로 중간 이후를 성공 화면으로 채우지 않았습니다. 처음과 마지막은 `PREFILL`이라 모델 궤적이 없습니다. 없는 빨간 선을 만들어 넣지 않습니다.

## GIF·PNG·상태를 열기

| 파일 | 실제 포함 범위 |
| --- | --- |
| [직접 추론 전체 GIF](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v9/DIRECT_whole_actual_interval.gif) | 원래918프레임 전부. 약91.7초 기록 구간, 마지막 표시 지연0ms |
| [ROS 실제 관측 GIF](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v9/ROS_whole_actual_interval.gif) | 원래29프레임 전부, 마지막 실제 실패 포함. 약2.8초 기록 구간 |
| [처음 PNG](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v9/source_0000_DIRECT_ROS_status.png) | 원래순번0·PREFILL |
| [중간 PNG](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v9/source_0459_DIRECT_ROS_status.png) | 원래순번459·저장 궤적64점 |
| [끝 PNG](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v9/source_0917_DIRECT_ROS_status.png) | 원래순번917·PREFILL |
| [전체 상태 CSV](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v9/complete918_per_arm_status.csv.gz) | 918×두 조건=1,836행. 실패·거절·미실행 포함 |

PNG는960×480, GIF는480×240입니다. 읽기 쉬운 정지 화면은 PNG를 사용하세요. GIF는 프레임을 버리지 않고 픽셀만 줄였으며 시간 간격은 실제 기록에서 centisecond 단위로 표현합니다. 새 시뮬레이션 FPS나 런타임 지연의 재현 영상이 아닙니다. GIF의반복 재생이 새로운 native 프레임을 만든다는 뜻도 아닙니다.

GIF 두 개의 실제 합은11,336,666바이트로 기존12MiB 한도 안입니다. MEDIANCUT16 설정은 양자화 요청이며 모든 파일 색상표 항목 수16을 뜻하지 않습니다. 독립 binary 검산에서 두 GIF의 첫 프레임은256항목 표, 나머지945프레임은16항목 표를 실제 확인했습니다. 실제 사용하는 픽셀 색상 수는 별도 해독 검증 전UNKNOWN입니다.

## 어디까지 확인했나요

ROOT는 고정된 PNG3장을 직접 열어 두 패널의 기준점 중심, 회색 경로·파란 과거 흔적·중간 빨간 궤적, 처음/끝의 궤적 부재, ROS 미실행 표시를 확인했습니다. [3장 실제 시각 확인 기록](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v9_ROOT_three_PNG_visual_check_v1.json)을 보존했습니다. 전체947장의 PNG나 GIF 픽셀을 모두 눈으로 검토한 것은 아닙니다. [독립 전체 검산](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v9_independent_postcheck_v1.json)은646개 지정 경로의 앞뒤 전체 해시·파일 세대, 원래8개 기록의 CRC/EOF, 1,836행CSV, 947개GIF 프레임/시간/끝, PNG3장 CRC를 확인했습니다. 두 번째 렌더링·픽셀 해독·주행 시험은 하지 않았습니다.

원본 DIRECT 상태는 PREFILL54·REJECTED26·TRAJECTORY838개입니다. ROS 관측29개는 PREFILL10·REJECTED4·TRAJECTORY14·FAILED1개이며 나머지889개는UNEXECUTED입니다. **그림 생성 PASS가 원래 주행이나 ROS 지연 PASS로 바뀐 것은 아닙니다.** 메인0/9·하위0/30, 새NN·학습·ROS·CARLA·GPU 실행0회입니다.

[실제 결과·전체 source/input seals](../../../../../storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v9/result.json) · [원문 ROOT 계획](../../../../../storage/experiments/plans/2026-10-08/recorded_vehicle_centered_gallery_v9/ROOT_plan.json) · [별도 전체 렌더 검토](../../../../../storage/experiments/plans/2026-10-08/recorded_vehicle_centered_gallery_v9/ROOT_execution_review_v1.json).

## 과거 실패는 어떻게 처리했나요

V6 전체 GIF는509프레임 뒤 저장 한도에 도달해 중단됐습니다. 원래 불완전 GIF와전체CSV·실패 기록을 그대로 보존합니다. V7/probeV1은 종료 오류가 원래 오류를 덮을 수 있어 **소스 단계에서만 중단**했으며 실제 렌더 실패라고 꾸미지 않습니다. V8은 소스 검사만 했고, 별도5장 실제 검사에서 저장 크기가 너무 커 전체를 실행하지 않았습니다. V9는 새5장 검사와 별도 ROOT 권한 후 위전체구간을 실제 완료했습니다. 기준을 낮추거나 실패 장면을 빼지 않았습니다.

[오늘 전체 현황](../README.md) · [실제 주행 지연 분석](../driving_ros/README.md) · [정식 기능의 합격 기준](../../../../portable-e2e-feature-roadmap.md).
