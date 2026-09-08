# 카메라 6개 선택만으로 20 ms를 맞출 수 있나요?

<!-- HH_260906 - Enumerate all subset interval bounds from the already inspected report, without selecting a rig or reopening native data. -->

이번 DB의 **기존 nearest-CAM_F0 매칭 방식에서는 맞출 수 없습니다.**
8개 중 6개를 고르는 28가지 조합을 모두 계산했습니다.

| 계산 범위 | 6개 카메라 시각 차이의 보수적인 하한 |
|---|---:|
| F0 포함 여부와 무관한 모든 28개 조합 | 26.554 ms 이상 |
| F0를 포함하는 21개 조합 | 30.200 ms 이상 |
| 현재 common10 허용값 | 20 ms |

각 채널의 관측 offset이 `[최소, 최대]` 안에 있으므로, 어떤 6개를 선택해도
그 시각 범위는 `max(각 최소값) - min(각 최대값)`보다 작을 수 없습니다.
서로 다른 시점에서 얻은 극값을 자유롭게 조합한 **보수적인 하한**이므로 정확한
개별 시점의 6-camera skew를 재계산했다고 주장하지 않습니다.

[28개 조합의 계산값](result.json)과 [계산 코드](audit_nuplan_camera_subset_intervals.py)를 남깁니다.
[기존 메타데이터 보고서](../original_execution/report.json)의 SHA를 대조했으며,
DB·카메라 payload를 다시 읽지 않았습니다. 관련 합성 테스트 13개를 통과했습니다.

이 결과는 화각·보정값에 맞는 6개 카메라를 선택한 결과도, 모든 동기화 방법의 불가능성
증명도 아닙니다. 시간 차이를 입력에 명시하는 새 모델이나 운동 보상 방법은 별도 설계·검증
대상입니다. 이번에 timestamp·20 ms 기준·현재 모델 입력 규격을 변경하지 않았고,
데이터 승인·학습·실차 승인도 하지 않았습니다.

상위 폴더의 원본 publication manifest는 최초 10개 게시 파일을 그대로 보존합니다.
이 후속 계산은 이 하위 폴더의 별도 SHA 목록으로 검증합니다.
