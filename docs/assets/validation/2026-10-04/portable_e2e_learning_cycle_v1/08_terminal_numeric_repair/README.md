# 종점 계산 개선 — 같은 1,660개 검사 전부 통과

<!-- HH_261004 - Report actual full numerical improvement without calling counterfactual inputs a drive or a feature approval. -->

실제 저장된 GOOD6000 원출력 415개를 각각 남은 거리 4조건에 넣었습니다. 같은 1,660개 검사에서 이전 421개 실패를 해결했고, 기존 통과 사례의 퇴행은 없었습니다. **실제 종점까지 주행한 결과나 정식 기능 완료는 아닙니다.**

| 남은 거리 조건 | 이전 V1 | 수정 V2 |
| --- | ---: | ---: |
| 17.9m | 415/415 | 415/415 |
| 10m | 0/415 | 415/415 |
| 5m | 411/415 | 415/415 |
| 2.5m | 413/415 | 415/415 |
| 합계 | **1,239/1,660** | **1,660/1,660** |

이전에는 두 종류의 격자를 합치면서 서로 거의 겹치는 점이 생겼습니다. 약 10⁻¹²~10⁻¹⁰m짜리 구간에서 제곱근 반올림 오차가 가속도 계산에 증폭됐습니다. 고정밀로 실제 출력 숫자를 다시 계산해도 제한을 벗어났으므로, 마지막 검사만 정밀하게 하거나 허용 오차를 늘리는 것으로 처리하지 않았습니다.

새 격자는 실제 출발점·첫 100ms 경계·원래 곡률 출력의 2m 경계·실제 남은 거리로 구간을 나누고, 각 구간을 균등하게 쪼갭니다. 실제 경계를 움직이거나 가까운 점을 임의로 지우지 않습니다. **원래 속도·가속도·곡률·횡가속도 검사, 모델 원출력, 제어 법칙은 그대로**입니다. 원본 V1과 실패 421개도 보존했습니다.

로컬 CPU4/interop1에서 새 검사 전체의 p95는 **10.18ms**, 최대는 **54.29ms**였습니다. 여기에는 종점 geometry·출처 해시·witness·수치 decoder·기존 계획 검사·preview/PI가 포함됩니다. 카메라 처리·NN 추론·CARLA·명령 적용 시간은 제외되므로 **100ms 센서→계획 통과를 뜻하지 않습니다.** 이전 실행과의 시간 감소율도 주장하지 않습니다.

이 자료의 종점 위치는 같은 실제 경로 위에 만든 수치 시험 조건입니다. 저장된 영상으로 그 종점에서 모델이 추론하거나 실제 주행한 것은 아닙니다. 새 종점 처리는 내비게이션 보조 제어이며 학습된 STOP 기능도 아닙니다.

다음은 새 실행기에 연결한 뒤 실제 전체 경로의 목표점 도달·차체·지연을 확인하는 것입니다. 현재 완료 수는 메인 0/9·하위 0/30을 유지합니다.

- [V1 전체 실패 요약](../../../../../../storage/experiments/diagnostics/2026-10-04/navigation_terminal_counterfactual_benchmark_v1/summary.json)
- [V2 전체 결과·동일 슬롯 비교](../../../../../../storage/experiments/diagnostics/2026-10-04/navigation_terminal_counterfactual_benchmark_v2/summary.json)
- [V2 전체 1,660개와 warmup 4개 원본](../../../../../../storage/experiments/diagnostics/2026-10-04/navigation_terminal_counterfactual_benchmark_v2/all_warmup_and_measured_cells.jsonl)
- [측정 전 사전 선언](../../../../../../storage/experiments/diagnostics/2026-10-04/navigation_terminal_counterfactual_benchmark_v2/prospective_declaration.json)
- [새 격자 코드](../../../../../../learning/portable_e2e/terminaldecoder_v2_20261004.py)
- [같은 제어 법칙을 사용하는 새 연결](../../../../../../learning/portable_e2e/navigation_terminal_retiming_v2_20261004.py)
