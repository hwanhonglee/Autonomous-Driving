# 다른 비-TEST 우회전 site: 고정 범위 정적 조사

<!-- HH_260929 - Keep the negative static result separate from trainable driving evidence. -->

**결론: `NO_STATIC_BODY_PASS_IN_BOUNDED_SEARCH`.** Town03 DEV, Town02_Opt TRAIN 후보, Town05_Opt 분할 충돌 DEV 후보의 packaged OpenDRIVE에서 고정한 endpoint 30개 조합을 검색했다. 총 **16,905쌍** 중 원래 `speed_30kph` *물리 형상* profile을 통과한 62개 전체를 원 경로 해시로 재구성하여 명목 Prius 차체와 의도 차로 10cm inset sweep을 검사했다. **PASS 0, FAIL 57, 지도 차로 연결이 5mm 허용 간격보다 커 판정 불가 5**였다. 따라서 이번 범위에서 새 전문가 수집을 사전 등록할 수 있는 경로는 없다. 이 형상 profile 이름은 30km/h 운전 승인이라는 뜻이 아니다.

| 지도와 역할 | endpoint 쌍 | 물리 형상 PASS | 명목 차체 결과 | 독립성·신호 기록 |
| --- | ---: | ---: | --- | --- |
| Town03, 기존 주행 DEV | 7,154 | 20 | FAIL 20 | 16개는 기존 DEV 우회전과 같은 junction 730. 다른 junction 238의 4개는 모두 10cm 차체 실패이고 경로에 XODR 신호 기록이 있다. |
| Town02_Opt, TRAIN 후보 | 2,615 | 4 | FAIL 4 | 모두 junction 20의 한 물리 연결로, 경로에 신호 기록이 있다. |
| Town05_Opt, 역할 충돌 | 7,136 | 38 | FAIL 33, UNSUPPORTED 5 | junction 562의 11개 모두 10cm 차체 실패(그중 신호 기록 없음 2개). junction 720의 5개는 차로 다각형 연결을 증명할 수 없다. 이전 전역 TRAIN 노출과 새 DEV 제안이 충돌한다. |

[최종 정적 경로 보고서](report_v3.json)는 source XODR·split 자료·검사 코드·planner 해시, connector/endpoint별 단계 수, 신호 `signal`/`signalReference`, junction ID와 모든 물리 형상 통과 경로의 해시를 보존한다. [62개 전량 차체 감사](body_all_report.json)는 각 경로의 raw/inset body·sweep 침범 수와 판정 불가 이유를 보존한다. 같은 물리 교차로의 도로 ID 전이를 다른 junction으로 세지 않았고, 열거된 connector를 단지 지나간 뒤 **다른** junction에서 RIGHT를 수행한 경로도 제외했다. 처음 두 [중간 보고서](report.json)·[v2](report_v2.json)의 신호·junction 식별 범위 수정 내역은 [V2 계획](PLAN_V2.md)·[V3 계획](PLAN_V3.md)에 남겨 두었다. 정식 결론은 v3와 전량 감사만 사용한다.

분할 근거는 고정된 [주행 연구 dataset manifest](../../../../../storage/datasets/prepared/carla-common10-trainval-research-20260914-v1/dataset.json)(SHA256 `c73900cedce870a0f55663fde08b3a64b90fae139506fbe2099fdc8bf991c4a9`), [최근 통합 지도 분할 계획](../../2026-09-27/map_next_training_design/README.md)(SHA256 `656d38b18d39b1847c2b5ced606140edf97456c530128a1cc965a2358aafe630`), 그리고 [다음 주행 자료 계획](../../2026-09-27/drive_next_data_plan/README.md)이다. Town03의 과거 지도 TRAIN 노출과 Town05의 TRAIN/DEV 충돌 때문에 다른 물리 경로를 찾더라도 독립 DEV로 자동 승격할 수 없다. Town04/Town15 TEST 및 예약/held-out Town06/Town07/Town10HD_Opt의 route·영상·라벨은 조사하지 않았다. Town13은 현재 live 렌더 상태 미해결이라 이번 범위에서 제외했다.

이 결과는 **고정된 세 지도·30 endpoint 조합의 부정 결과**이지 비-TEST 전체에 우회전 site가 없다는 증명은 아니다. 검사에 사용한 것은 packaged XODR와 이전 Town01에서 기록한 Prius bbox이며 live 지도 해시·해당 지도 actor bbox는 일치 확인하지 않았다. XODR 신호 기록만으로 실제 정지 여부를 단정하지 않는다. 실제 `>0.5m/s` 30초 연속 이동, 각 anchor의 64점/6.4초 미래, 원래 아홉 운동 한계, 실제 차체·충돌·목표 ≤2.5m는 관측하지 않았다. `capture_authorized=false`, `dataset_admitted=false`, `training_executed=false`, learned closed-loop 검증 0이다.

재현은 기존 로컬 CARLA Python API를 **오프라인 지도 파서**로만 사용한다. 새 출력 경로를 지정하고 저장된 보고서 SHA256 `0f183bbeedb09e1ec391cd13e2bc45cb09d5af8ef5d4743c7a0aef5ac6a51b2a` 및 전량 감사 SHA256 `1573823a8800ada0c5a718afc700cb8f701c24f5a05faf003482669fd94ef944`와 비교한다.

```bash
source tools/e2e/env.sh
python3 tools/workspace_python.py -m pytest -- -q learning/tests/test_scout_other_non_test_right_20260929.py
audit_dir="$(mktemp -d)"
python3 tools/workspace_python.py tools/e2e/scout_other_non_test_right_20260929.py \
  --output "$audit_dir/report_v3.json"
python3 tools/workspace_python.py tools/e2e/audit_other_non_test_right_body_20260929.py \
  --output "$audit_dir/body_all_report.json"
sha256sum "$audit_dir/report_v3.json" "$audit_dir/body_all_report.json"
```

오프라인 재현에서 회전/site·신호 범위 단위 검사 **5개 PASS**, 두 검색기 문법 검사 PASS, 62개 route 전량 SHA 재구성 PASS였다. CARLA 서버·SSH·GPU·설치·TEST 접근·새 원본 수집·모델 학습은 없었다.
