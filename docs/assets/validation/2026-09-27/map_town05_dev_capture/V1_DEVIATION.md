# Town05_Opt 첫 원본 수집 — 품질 설정 불일치

<!-- HH_260906 - Preserve a completed but ineligible capture without retroactively editing its preregistration. -->

사전 [첫 계획](PLAN.md)은 CARLA 렌더링 품질을 명시하지 않았습니다. 실제 첫 [소유 실행](../../../../../storage/experiments/training/2026-09-27/map_town05_independent_dev_capture_v1/owner_result.json)은 `--quality Low`로 종료 코드 0, Town05_Opt 240프레임·충돌 0·live/replay 일치·CARLA 종료 PASS를 기록했습니다. 그러나 비교할 Town03 지도 TRAIN 두 회차의 소유 계획은 `quality=Epic`입니다. 수집 완료 후 확인한 이 차이를 사후에 계획 변경으로 숨기지 않습니다.

첫 회차는 **원본 정찰로만 보존**하고 지도 DEV로 승인하지 않습니다. 이 결정을 내릴 때 AI v2/B/C의 Town05_Opt 예측이나 점수는 만들지 않았습니다. 같은 경로를 새 출력에 `Epic`으로 다시 수집하는 계획은 [별도 v2 계획](PLAN_V2.md)에 사전에 고정합니다. 첫 회차를 삭제·덮어쓰기하거나 두 품질의 프레임을 섞지 않습니다.
