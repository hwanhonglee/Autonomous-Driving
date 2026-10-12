# 주행·지도 공통 사이트 분할 검사기

<!-- HH_260929 - Publish only conservative current permissions, not retrospectively cleaned independent validation. -->

**결과: 독립 DEV로 승인된 Town은 0개.** [기계 판독형 원장](../../../../../learning/config/site_split_guard_20260929.json)과 [검사기](../../../../../tools/e2e/site_split_guard.py)를 추가했다. 이는 앞으로 실행할 작업의 사전 차단기다. 과거 수집·학습 이력을 재분류하거나 이미 노출된 Town을 새 독립 DEV로 만드는 기능이 아니다. 현재 학습기에 자동 연결되어 있지는 않아, 다음 수집기·학습기의 입력 검사에도 별도로 연결해야 한다. 검사기 회귀시험 12개가 통과했다.

| 사이트 | 지금 허용하는 좁은 범위 | 차단하는 잘못된 해석 |
| --- | --- | --- |
| Town01·Town02_Opt | 이미 보관된 지도 TRAIN 재사용 | Town01 같은 Town 내부 DEV를 독립 사이트로 주장 |
| Town03 | 기존 주행 DEV의 탐색 재사용/오프라인 조사 | 과거 지도 TRAIN 노출을 없던 것으로 간주 |
| Town05_Opt | 이미 반복된 지도 DEV 분석 | 새 미사용 DEV·표지판 합격·자동 승격 주장 |
| Town12 | 주행 TRAIN 후보의 오프라인 정적 탐색 | 수집 승인·데이터 편입·주행 검증 주장 |
| Town13 | 지도 렌더 품질 진단만 | 경고 영상 TRAIN/DEV 편입 |
| Town04·06·07·10HD_Opt·15 | 새 TRAIN/DEV 작업 없음 | 초기 expert suite의 오래된 `train` 표기만으로 봉인 해제 |

이 원장은 Town04·Town07 등 과거 노출 때문에 *순수한 독립 TEST가 남았다*고 단정하지도 않는다. Town07의 역사적 주행 TRAIN과 후속 held-out 요구는 충돌하며, 새 분할 계약과 모델 노출 장부가 확정될 때까지 검증셋으로 사용하지 않는다. Town13은 실제 tile/camera 안전 문제가 해결될 때까지 영상 자격을 복구하지 않는다.

<!-- HH_260929 - Make any prospective site admission a reviewed code-and-registry change. -->
등록된 19개 사이트의 집합과 허용 역할도 코드에서 고정했다. 새 사이트를 JSON에만 추가하거나, 기존 미검증 사이트를 JSON만 바꿔 TRAIN으로 열면 검사 자체가 실패한다. 새 적격성은 원본·분할 근거를 만든 뒤 코드·원장을 함께 검토해야 한다. CLI는 세 옵션을 모두 주거나 모두 생략해야 한다.

```bash
# HH_260929 - Read-only examples; a denial exits with status 2.
python3 tools/workspace_python.py tools/e2e/site_split_guard.py
python3 tools/workspace_python.py tools/e2e/site_split_guard.py --site Town12 --model drive --operation offline_train_scout
python3 tools/workspace_python.py tools/e2e/site_split_guard.py --site Town07 --model map --operation reuse_exploratory_dev
python3 tools/workspace_python.py -m pytest -- -q learning/tests/test_site_split_guard.py
```

원장 근거는 [새 지도 DEV 적격성](../map_new_dev_feasibility/README.md), [기존 지도 분할 계획](../../2026-09-27/map_next_training_design/README.md), [주행 자료 계획](../../2026-09-27/drive_next_data_plan/README.md), [원래 expert suite](../../../../../tools/e2e/carla_expert_suite.yaml)다. 검사기가 `ALLOW`를 내더라도 원본/라벨 품질, 경로·세션 중복, 정확한 live 지도·차체, 모델 성능, 폐루프 안전성은 별도로 검증해야 한다.
