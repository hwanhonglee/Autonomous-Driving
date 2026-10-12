# 07 — 학습이 충분한가? 전체 맵 현황과 저속 곡률 실패 재현

<!-- HH_260906 - Audit training coverage and preserved raw failures without claiming all-map driving, convergence, or new training. -->

**충분하지 않다.** 6회는 같은 작은 데이터로 손실 A/B를 seed 3개에서 비교한 한 실험이다.
새 Portable 모델의 직접 제어 검증은 여전히 **0/30 기능**이다. 기존 VAD의 여러 맵 PASS,
BasicAgent 수집, shadow 추론, 새 모델의 저장 영상 평가를 서로 합산하지 않는다.

이번 후속 작업은 **새 검사 코드 구현·기존 기록 재현·맵별 계획 보강**이다.
새 GPU 학습·새 CARLA 주행·Autoware 실행·모델 교체는 하지 않았다.
앞서 실제 수행한 학습 6회는 [05번 결과](../05_stopmix_regret_learning/README.md)에 그대로 있다.

## 1. 실제 데이터와 학습량을 다시 계산했다

| 구분 | 기록된 episode / 맵 | 구간 | 원본 기록 길이 |
| --- | ---: | --- | ---: |
| TRAIN | 3 / 3 | Town07 직진, C-track 좌, Town01 우 | 133.900002초 |
| 개발 VAL | 1 / 1 | Town03 우 | 40.150001초 |

위 길이는 episode metadata의 끝−시작이며 준비·tail을 포함한다. 순수 이동 시간이나
통계적 독립성을 증명한 값이 아니다. 날씨는 모두 ClearNoon이다.
10 Hz의 연속된 완전한 6.4초 미래창이라면 이웃 창의 미래 시점 63/64, 즉 **98.44%**가
겹친다. 이번 metadata-only 검사 자체가 영상 cadence나 모든 valid mask를 재검사한 것은 아니다.

한 fit은 1,540 step, 표본 노출 6,155회 = 전체 5 epoch + 다음 epoch의 420표본이다.
6개 전체는 9,240 step·36,930회 노출이다. **새 장면 36,930개가 아니다.**
같은 데이터 반복이나 초기화 seed 증가는 맵·날씨·주행 의도 다양성을 늘리지 않는다.

학습 loss는 내려갔지만 충분히 수렴했다고 판단할 수 없다. 저장된 값은 optimizer 실행 중
batch 평균이며 epoch별 고정 VAL 곡선은 없고, 최종 checkpoint만 평가했다.
아래 B의 3초·6.4초 ADE는 세 seed 모두 기존 절대 기준을 초과한다.

![실제 학습 기록과 최종 VAL 오차](figures/training_sufficiency.png)

[실제 metadata·학습 노출 검사 JSON](training_coverage.json) ·
[그림에 사용한 숫자](figures/plotted_values.json)

새 학습 범위 검사는 dataset.json + episode.json 4개만 열었다. 원본 sample/JPEG/TEST/
checkpoint payload를 읽거나 모델을 다시 추론하지 않았다. 학습 실행 기록과 해시 대조의
범위는 JSON의 `limitations`를 따른다. 모든 결과를 재평가한 두 번째 학습 실험이 아니다.

## 2. 전체 맵·직진·회전을 실행 가능성과 분리했다

[전체 19개 맵 승인표](../../../../../portable-e2e-map-acceptance-2026-09-14.md)에 기존 경로와
막힌 맵을 빠짐없이 적었다. 새 비실행형 planner는 19맵 × 직진/좌/우/정지 유지/재출발 =
**95개 계획 항목**을 생성한다. 이것은 기존 **30개 기능의 분모와 다른 표**이며,
95개가 모두 실행·승인됐다는 뜻이 아니다.

- 현재 packaged 자산과 기존 직진/회전 경로가 연결된 맵은 9개다. 오늘 전부 재부팅하거나
  새 모델로 재주행한 것은 아니다. Opt는 원본 Town과 별도로 표시한다.
- 새로운 planner는 Town04·Town15를 잠그고 카탈로그 경로도 열지 않는다.
  나머지 준비된 8맵의 카탈로그 metadata 16개에서 기하 후보 161개를 확인했다.
  전체 맵 문서의 과거 VAD 후보 182개와 차이는 Town04의 21개다. TEST 정답을 읽은 차이가 아니다.
- Town06·Town10HD_Opt는 이 카탈로그에서 좌회전 후보가 0개다. 맵 전체에 좌회전이 없다는
  뜻이 아니므로 새 경로 탐색으로 채워야 한다.
- 기존 suite는 Town03/Town04를 TRAIN으로 선언하지만 현재 연구에서는 **Town03=DEV,
  Town04=잠긴 TEST**다. 새 planner가 이 불일치를 표시하며 자동 split 변경은 하지 않는다.
- Woraksan은 suite의 `ready` 표기와 달리 지정 level/XODR가 없어 차단한다. 서버 executable·
  Python API 파일/해시도 확인한다. 현재 파일 존재가 rendering 성공이나 주행 성공은 아니다.

[새 맵별 계획 JSON](map_coverage_plan.json)의 모든 항목은 `BLOCKED`, 실행 명령은 빈 목록이다.
경로만 있으면 대량 수집을 실행하게 하지 않고 **새 수집 계획·split·정답 품질 승인**을 요구한다.
이는 새 planner의 방어 장치이며, 과거 suite/runner의 기본 동작까지 수정한 것은 아니다.

## 3. v5의 곡률 실패를 원본 그대로 재현했다

대상은 [06번 Town07 BasicAgent v5 파일럿](../06_brake_hold_pilot/README.md) 하나다.
카메라/상태/미래창의 frame·timestamp·phase와 저장된 곡률 값·실패 flag를 대조한다.
다른 물리 지표의 실패 수는 봉인된 보고서에서 보존하며 이번 코드에서 모두 재계산한 것은 아니다.

| 위치 | 실패한 고유 100 ms 시점 | 겹친 미래창에서의 실패 횟수 |
| --- | ---: | ---: |
| 출발 저속 구간 | 6 | 264 |
| 저속 무페달 감속 | 32 | 284 |
| 첫 서비스 제동 경계 | 1 | 64 |
| 합계 | **39** | **612** |

기존 **134/666 미래창 실패**는 그대로다. 모든 39시점은 실제 driving phase에 있다.
준비 phase에서 시작한 창도 미래의 출발을 포함하므로 실패할 수 있다.
39개 중 29개는 창의 **첫 점**에서만 실패한다. 첫 점은 이전 궤적의 방향이 아니라
anchor 차량의 정면 방향을 기준으로 비교한다. 이것을 모두 도로의 급커브라고 읽으면 안 된다.

측정 결과 짧은 이동거리의 방향 계산, actor 기준점과 가상 후방 기준점의 차이, 출발·제동
경계가 함께 영향을 준다. 예를 들어 출발 tick 52는 actor 곡률 약 0.123 / 가상 base 0.350,
tick 55는 약 0.086 / 0.375 rad/m이다. 제동 경계 tick 646은 actor 약 3.59 /
가상 base 약 16.06 rad/m이며, 해당 100 ms의 base 이동거리는 약 5.857 mm다.

감속 tick 631/632/641/642에는 actor X 좌표의 30.518 µm 변화와 0 변화가 교대한다.
해당 좌표 크기에서 float32 간격 두 칸에 해당하지만, 이는 **수치 민감도 관찰**이지
실패를 삭제할 수 있는 보정 오차 상한의 증명이 아니다. 제동 경계에는 brake뿐 아니라
steer도 동시에 바뀌므로 brake 하나가 원인이라고 단정하지 않는다.

[독립 재현 요약](curvature/summary.json) · [39개 시점·612개 창의 상세 기록](curvature/witnesses.jsonl)

![실패 위치와 actor·가상 base의 실제 변위 비교](curvature_figures/curvature_conditioning.png)

612회 중 첫 점의 정면 기준 비교가 36회, 앞선 미세 이동이 판정 최소거리보다 작아 정면
기준이 계속 유지된 비교가 49회, 이전 판정 가능한 이동 방향과의 비교가 527회다.
첫 점이 아닌 모든 경우에 바로 앞 구간 방향을 사용하는 것도 아니다.

**해석의 한계:** 이 mm 규모의 원시 곡률 문제를 해결한다고 모델의 수 m 장기 경로 오차가
해결되는 것은 아니다. 수집 정답의 표현/품질과 모델의 일반화 성능은 별도의 과제다.
actor 좌표로 정답을 바꾸거나 warmup/정지 구간을 삭제·retime하지 않았고 기존 gate도 유지했다.
06번 자료는 계속 학습 미편입이다.

## 4. 다음 실행 순서와 사용자 요구 반영

<!-- HH_260906 - Keep follow-up experiments prospective and preserve the established environment, split and evidence restrictions. -->

1. **측정/기준점·제어 경계 분리:** 이번 39개 기록을 기준으로 앞바퀴 조향 변경과 제동 전환,
   측정 기준점과 body/path 방향의 차이를 분리한 다음 실험을 사전 고정한다.
   새 측정 계약이 필요하면 별도 버전으로 검증하고 기존 실패를 소급 PASS로 고치지 않는다.
2. **독립 데이터 확대:** 우선 Town07 직진 → C-track 회전 → Town03 회전 순서를 유지한다.
   Town03은 DEV 전용이고 현재 VAL 우회전과 과거 VAD/shadow 좌회전도 구별한다.
   TRAIN/DEV 각각 직진·좌·우·정지 유지·재출발을 확보하고, episode/route group 단위 split,
   정지/재출발 원인 label, 날씨·세션 다양성을 수집 전에 고정한다.
3. **다음 전체 모델 학습:** 승인된 새 데이터 버전과 고정 개발 평가를 사용한다.
   단순히 fit 횟수를 채우지 않고 절대 오차·정지/회전/지속 주행 퇴행과 맵별 성능을 통과해야 한다.
   수집 품질 미달 자료를 GPU를 채우기 위해 넣지 않는다.
4. **실제 구동 검증:** export 동등성 → 10 Hz shadow → fallback/제어 소유권 → 제한된 CARLA
   learned closed-loop. 30 km/h는 상한이며 회전은 감속한다. 60 km/h와 실차 제어는 후속 별도 단계다.

원격은 **physical GPU 0·개인 hwanhong venv만**, 로컬은 계측·시뮬레이션·연결 검증이다.
이번에도 설치·재부팅·GPU 1 사용·타 작업 종료·Git push는 하지 않았다.
10 Hz 입력, 맵별 직진/좌/우 별도 기록, 실제 주체 표시, ego 중심 경로/궤적 화면 요구를 유지한다.
이번 PNG는 학습 수치 분석이며 새 주행 촬영물이 아니다. 실제 6카메라·차량 중심 경로 화면은
[변경하지 않은 06번 원본 화면](../06_brake_hold_pilot/04_zero_pedal_coast_to_service_brake_hold.png)에서 확인한다.

## 5. 코드 검사와 재현 방법

신규 도구 테스트 97개를 통과했다. 기존 수집·학습 회귀와 합친 검사는 **636 passed / 1 skipped**다.
한 skip은 로컬 exact-resume의 보안 weights-only 로딩 조건이며 보안 우회로 통과시키지 않았다.
[코드/출력 무결성 검사](code_verification.json)에 소스·JUnit·복사한 결과의 SHA를 남겼다.
기존 01–06 공개 payload 146개와 native 학습 소스 18개는 그대로다.

저장소 루트에서 실행한다. 이 검사에는 별도 설치·GPU·SSH가 필요 없지만, 재현 대상의
실제 로컬 metadata·원시 기록이 있어야 한다. `git clone`만으로 비공개 대용량 원시 데이터까지
생기지는 않는다. 결과 JSON/PNG는 이 폴더에서 바로 볼 수 있다.

```bash
python3 -m pytest -q \
  tests/test_audit_portable_training_coverage.py \
  tests/test_audit_carla_curvature_conditioning.py \
  tests/test_plan_portable_map_coverage.py

python3 -m scripts.e2e.plan_portable_map_coverage \
  --catalog-maps-root artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps \
  --output-json artifacts/training/2026-09-14/map_coverage_recheck.json
```

두 번째 명령은 **계획만 생성**한다. 출력 파일이 이미 있으면 덮어쓰지 않으므로 새 파일명을
지정한다. TEST나 차단된 맵을 실행하기 위해 기존 suite로 우회하지 않는다.
실제 곡률 기록만 다시 검산하고 출력도 만들지 않으려면:

```bash
python3 scripts/e2e/audit_carla_curvature_conditioning.py \
  --trial-root artifacts/training/2026-09-14/brake_hold_goal_stop_v5/run_001 \
  --evidence-dir docs/assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/06_brake_hold_pilot \
  --expected-evidence-seal-sha256 aa58bfcabc25920d77a22e9f2bab92fda2a703cc6e5c8b0b9f1ebf396f0c967e \
  --expected-script-sha256 165b1545ee476349bdb58cd4e8fc610a85242ff5d44a1c0edd8024a954ad1385 \
  --verify-only
```

원시 기록·봉인한 코드 버전이 다르면 실패하는 것이 정상이다. 실행하려고 해시·기준을
임의 변경하지 않는다. 새 실험은 별도 버전/계획에서 시작한다.

[당일 전체 목록](../README.md) · [전체 목표·30개 기능 현황](../../../../../portable-e2e-status-2026-09-14.md)
