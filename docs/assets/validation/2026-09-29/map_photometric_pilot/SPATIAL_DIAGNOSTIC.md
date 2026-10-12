# 사진측광 후보의 차선 오류 위치 — 반복 DEV 후속 분석

<!-- HH_260929 - Expose near-ego gains and far-field regressions without turning post-hoc DEV into approval. -->

**결론:** 밝기·무채색 변형 후보는 Town05_Opt의 차량 가까운 차선 탐지를 실제로 늘렸지만, 기존 Town01의 먼 영역에는 오탐을 늘렸다. 따라서 전체 탐색 관문은 **FAIL**, 후보 미채택이다. 이 문서는 완료된 두 체크포인트의 저장 예측을 **사후** 분석한 것이며, 새 실험의 사전 목표나 독립 DEV가 아니다.

| 반복 DEV·영역 | 차선 정답 픽셀 | D1 IoU → 후보 IoU | D1 TP → 후보 TP | D1 FP → 후보 FP |
| --- | ---: | ---: | ---: | ---: |
| Town05 전체 화면 | 1,289,336 | **7.325% → 13.483%** | 98,939 → 222,000 | 61,362 → 357,167 |
| Town05 차량 전방 중앙 | 500,722 | **0.0096% → 11.203%** | 48 → 60,556 | 35 → 39,829 |
| Town01 전체 화면 | 45,917 | **89.364% → 83.880%** | 45,306 → 45,061 | 4,781 → 7,804 |
| Town01 위쪽 반 화면 | 6,266 | **63.115% → 42.871%** | 5,686 → 5,620 | 2,743 → 6,843 |
| Town01 차량 전방 중앙 | 14,380 | **93.858% → 96.800%** | 14,380 → 14,367 | 941 → 462 |

차량 전방 중앙은 `x=160..479, y=180..359`, 위쪽 반 화면은 `y=0..179`다. 이 영역들은 전체 화면과 중첩되므로 독립 표본 5개가 아니다. Town05 중앙의 후보 차선 recall은 `60,556/500,722 = 12.09%`에 불과하다. 상대적으로 큰 증가를 **실사용 가능한 차선 지도**로 해석하면 안 된다. Town01 전체 퇴행도 모든 위치에서 나빠진 것이 아니라 위쪽 반 화면의 차선 FP 증가가 두드러진다. 사진측광 증강이 원인이라는 인과 분해는 이 한 후보·한 seed로 할 수 없다.

[재계산 JSON](spatial_diagnostic.json) SHA-256 `6296efd7a1bfa26f22ee67f73af2e4f5b90d385e7b95c3d8b512c6c366d03e1f`에는 두 Town의 full/far/near_full/near_ego 도로·차선 TP/FP/FN·IoU·recall·precision과 각 입력 SHA가 있다. [읽기 전용 감사기](../../../../../tools/e2e/audit_map_photometric_spatial.py)는 전체 화면의 7×7 혼동행렬을 [기존 D1 지표](../../../../../storage/experiments/training/2026-09-29/map_road_lane_ab_run_v1/D1/metrics.json)와 [후보 지표](remote_run_v1/metrics.json)에 먼저 대조한 뒤 영역별 수치를 낸다. 해당 두 지표의 체크포인트, 학습 계약, DEV 사용 범위는 [사전 계획](PLAN.md)과 [실행 보고](README.md)를 따른다. 학습·TEST·GPU·CARLA 실행은 하지 않는다.

```bash
# HH_260929 - A fresh output path is required for an optional saved replay.
python3 tools/workspace_python.py tools/e2e/audit_map_photometric_spatial.py --output /tmp/map_photo_spatial_replay.json
python3 tools/workspace_python.py -m pytest -- -q learning/tests/test_audit_map_photometric_spatial.py
```

다음 모델 작업은 이 반복 DEV에서 변형 확률·강도를 여러 번 골라내는 것이 아니다. 신규 TRAIN의 실제 어두운 노면·무채색 차선 자료와, 모델 점수를 보기 전에 잠근 미사용 평가 site가 우선이다. 도로·차선 외 5클래스와 주행 안전성도 따로 보호해야 한다.
