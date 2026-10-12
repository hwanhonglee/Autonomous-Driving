# Town05_Opt Epic v2 지도 DEV 원본 감사

<!-- HH_260906 - Qualify owned native evidence while preserving DEV, TEST and model-inference boundaries. -->

**결론: 원본 수집 감사 통과, 데이터셋 편입·모델 평가는 미실행.** 사전 [Epic v2 계획](PLAN_V2.md)의 소유 실행·원본 240쌍·0.25m 간격 최소 120장 조건을 충족했다. 기존 [Low v1 불일치](V1_DEVIATION.md) 자료는 계속 부적격이며 이번 수치에 섞지 않았다. 이 결과는 전체 209.77m 경로가 아니라 BasicAgent가 실제 이동한 **94.31m 앞부분**의 Town05_Opt DEV 원본이다. 수집 종료 시 `agent_goal_reached=false`였지만 경로 완주는 사전 자격 조건이 아니었다.

| 감사 항목 | Epic v2 결과 |
|---|---:|
| 원본 전방 RGB/시맨틱 쌍 | **240/240**, 전체 화각 640×360, 전수 디코드 |
| 0.25m 위치 간격 보존 | **232장**, 사전 최소 120장 통과 |
| 프레임·시간 | 연속 CARLA 프레임 240장, 10 Hz 간격 최대 오차 1.49×10⁻⁹초 |
| 실행 품질·경로 | `Epic`, Town05_Opt straight p00 SHA256 `efb8ccca533f25cae54b2eb7e03274c15795211488128185334ff2bc55f4744c` |
| 소유 실행·소스 | exit 0, 실행 소스 archive **13/13** 해시 일치 |
| 물리·재생·정리 | 충돌 0, owner의 저장 원본 live/replay 비교 일치, lifecycle ready/after/stopped PASS·포트 해제 |

[기계 판독 감사 JSON](epic_v2_audit.json)에는 소유·route·lifecycle 해시, 원본 240쌍 각각의 RGB/semantic NPZ 해시·픽셀 수, 보존 프레임 목록이 있다. [전체 화각 RGB와 CARLA oracle 라벨 예시](epic_v2_rgb_oracle_full_fov.png)는 보존된 첫·중간·마지막 프레임이다. 그림은 모델 예측이 아니다.

| CARLA 의미 클래스 | 원본 240장 픽셀 | 보존 232장 픽셀 |
|---|---:|---:|
| other | 20,267,623 | 19,563,136 |
| road | 26,269,899 | 25,368,331 |
| sidewalk | 6,662,004 | 6,508,496 |
| lane marking | 1,335,355 | 1,289,336 |
| traffic light fixture | 124,763 | 120,720 |
| traffic sign | **265** | **256** |
| pole | 636,091 | 602,525 |

**표지판 성능 gate에는 부적합하다.** 보존 232장 모두 tag 8이 있지만 각 프레임의 총량이 **1픽셀 208장·2픽셀 24장**뿐이다. 이 픽셀이 RGB에서 독립적으로 확인되는 표지판 개체인지는 검사되지 않았다. 표지판 IoU·정밀도는 이 원본만으로 안정적인 합격 기준을 만들 수 없다. 도로·차선의 새 Town 개발 자료로도 **관측된 94.31m 구간만** 대표한다.

원본은 `storage/experiments/training/2026-09-27/map_town05_independent_dev_capture_epic_v2/mapping`에 그대로 있다. [감사 코드](../../../../../tools/e2e/audit_map_town05_dev_capture.py)는 보관된 source 13개 해시를 직접 재계산하고 RGB/semantic 원본을 모두 디코드했다. owner의 `live_replay_exact_match=true`는 해시가 일치하는 보관 수집기에서 저장된 NPZ를 다시 투영·재생하여 live snapshot과 비교한 결과이며, 이 감사 코드는 재생 계산 자체를 별도로 실행하지 않았다. 모델 추론·점수, TRAIN 편입, 지도 모델 승격, 정밀지도 승인과 주행 모델 제어는 하지 않았다.
