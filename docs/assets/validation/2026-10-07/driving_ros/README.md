# 주행 모델 연결: 직접 계산과 ROS를 구분해서 보기

<!-- HH_261007 - Explain inference without training or actuation and retain the genuine direct/DDS failure denominator. -->

오늘은 **새 학습 없이 기존 주행 모델을 계산하고 연결 오류를 찾는 작업**입니다. 모델 계산을 실행했다는 것과 학습했다는 것은 다릅니다. 학습은 정답과 오차로 가중치를 업데이트하지만, 이번 추론에서는 가중치를 읽기만 합니다. 차량 제어 명령도 보내지 않습니다.

## 어떤 입력과 모델인가

로컬에 보존된 Town03 약209m 기록의 전체 918개 카메라 입력 묶음을 사용합니다. 한 묶음에는 카메라 6개, 위치·가속·조향·보정·TF·시각과 경로 정보가 있습니다. JPEG 원본 5,508개와 보조 입력 전체를 해시로 고정했습니다. 이미 알려진 진단 자료이며 새로운 독립 TEST가 아닙니다.

기존 GOOD 6,000-step 가중치와 실제 입력 구성·안전 거절·100ms 기준은 그대로입니다. 기록을 보내는 프로그램은 모델 없는 송신기이고, 수신기가 기존 모델을 계산합니다. 사람이 지금 운전하거나 teacher가 지금 차량을 움직이는 시험은 아닙니다.

## 비교의 순서

```text
같은 전체 기록 → 직접 모델 계산 → 입력·원시 출력·변환 궤적 기록
             → 소유한 ROS 송신기 → 소유한 ROS 수신기의 같은 모델
                                  → 실제 Trajectory/Path 송수신 기록
```

직접 계산과 ROS 계산은 동시에 실행하지 않습니다. ROS 경유에서 직접 계산과 동일한 입력·모델 출력·유효 궤적·실제 발행 시각인지 비교합니다. 실패하거나 실행하지 못한 뒤쪽 입력도 전체 918개 분모에 남깁니다.

## 최신 V11: 지연을 세분화했으며 성능은 여전히 미합격

<!-- HH_261007 - Explain actual measured stage costs, complete failures and unknown-anchor rows without turning profiling into a driving improvement. -->

**V11 실제 계측은 99.90초에 종료됐습니다.** 직접 계산 918개는 완료했고 ROS 관측은 43개입니다. 완전한 궤적/Path 26쌍 중 검사용 기록을 포함한 전체 ACK 100ms를 만족한 것은 **6/26쌍**입니다. 성능 개선판으로 채택하지 않습니다.

관측 43개 모두 입력·텐서·5개 원시 출력 비교는 일치했지만, 지연 거절 때문에 상태가 같은 것은 40개입니다. 과거 문맥 준비 10개·원래 안전 거절 4개·궤적 26개·출력 전 ROS 누적 지연 거절 2개·마지막 실패 1개와 뒤쪽 **875개 미실행**을 유지했습니다. 마지막 42번은 첫 Trajectory 진입 **100.649665ms**에서 거절돼 그 시점의 출력은 0개이며, ACK는 **116.596795ms**였습니다. 앞쪽 실제 수신 출력 52개는 26쌍입니다.

이번에는 모든 관측 묶음에서 실제 입력 19개씩, 총 **817개 메시지·8,170개 단계**를 기록했습니다. 원래 입력 입장·CDR·시각으로 연결된 메시지는 798개이고, 마지막 실패 응답의 19개는 **UNKNOWN**으로 남겼습니다. 모르는 연결을 마지막 ordinal에 맞춰 꾸미지 않았습니다.

| 측정 | 관측 43개 묶음당 평균 | 해석 |
| --- | ---: | --- |
| 실제 native 수신과 endpoint 조회 19회 합 | 1.70ms | 메시지 내부 실제 송신자 확인 포함. 빈 수신 polling 비용은 이 표에 없음 |
| 메시지별 소유 프로세스 확인 19회 합 | 4.62ms | 실행 소유자와 상대 PID 세대 확인. 필수 검사를 제거하지 않음 |
| 수신 callback의 그래프 확인 합 | 0.05ms | provenance 때의 전체 검사와 다른 메시지의 분기 비용. 추론 직전·출력 검사는 별도 |
| 송신자 그래프 대조 / native 정보 검증 합 | 0.80 / 0.17ms | 실제 GID·형식·정보 확인 |
| 수신 메시지 CDR 생성 / 해시 합 | 0.59 / 0.26ms | 원래 내용을 한 번 계산. 기록만을 위해 추가 NN 없음 |
| 입력 시작 → 모든 ROS 입력 준비 | 32.71ms | 송신 작업·수신·대기·스케줄링이 함께 포함 |
| 기존 ROS 입장 검사 / 관측 frame 구성 | 1.35 / 0.37ms | 실제 준비 시각 사이의 원래 단계 |
| 모델 판단 wrapper 전체 | 57.67ms | 원래 판단과 결과 부착 포함. 실제 차량 전체 지연과 다름 |

따라서 **DDS 수신 자체가 수십 ms를 차지한다고 결론 낼 수 없습니다.** 단계 합을 전체 wall 시간이나 순수 네트워크 지연으로 취급하지 않으며, 나머지 입력 시간은 송신 측·스케줄링 등을 추가로 계측해야 합니다. 실제 카메라 FPS의 원인 검사도 아닙니다. 계측용 기록 비용 자체도 원래 ACK 완료 시간에 포함했습니다.

[V11 실제 결과](../../../../../storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v11/result.json), [전체 실행 선언과 검토](../../../../../storage/experiments/plans/2026-10-07/temporal_recorded_ros_shadow_v11/ROOT_review.json), [43개 전체 지연 그림](../../../../../storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v11_v4/ROS_prefix_all43.png), [전체 918+918행 CSV](../../../../../storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v11_v4/complete_timing.csv), [단계 8,170행 무손실 압축 CSV](../../../../../storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v11_v4/actual_local_stages.csv.gz)를 보존했습니다. 그래프는 실제 수치의 진단 렌더링이지 Autoware/RViz 캡처가 아닙니다. 코드 검사 **253개**, 기록 그래프 검사 **25개** 통과도 기능 승인이 아닙니다.

[별도 전량 후검산](../../../../../storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v11_independent_postcheck_v1.json)에서 현재/보존 소스 94개·입력 5,895개·생성 ROS 소스 42개·설정 2개, 139개 계획/실행 파일과 V8~V10 원본 불변, 소유 PID 4세대 종료를 확인했습니다. 이 검산자는 V11 코드 공동작성자이므로 **독립적인 두 번째 계측 실행·미공개 TEST 평가가 아닙니다.** 원래 NN 단독 시간 필드는 관측 43행 중 33행에만 있어 나머지 준비 10행은 UNKNOWN으로 유지했습니다. 실제 NN이 실행되지 않았다는 뜻이나 0ms라는 뜻은 아닙니다.

## V10: 검사 위치 변경만으로는 개선되지 않음

<!-- HH_261007 - Preserve the actual failed V10 interval and all873 unexecuted anchors without claiming an improvement from a shorter prefix. -->

**V10도 실행을 종료했으며 미합격입니다.** 전체 918개 직접 추론 뒤 실제 ROS 기록 45개를 남겼습니다. 입력·텐서·5개 원시 출력은 관측 45개에서 일치했지만, 완전한 출력 29쌍 중 전체 ACK 100ms를 만족한 것은 **5/29쌍**입니다. V9보다 길게 처리한 것도 아니므로 개선판으로 채택하지 않습니다.

| 구분 | V8 | V9 | V10 |
| --- | ---: | ---: | ---: |
| 직접 처리 / 선언 분모 | 918/918 | 918/918 | 918/918 |
| 실제 ROS 처리 / 미실행 | 9 / 909 | 50 / 868 | 45 / 873 |
| 완전한 Trajectory/Path 쌍 | 3 | 35 | 29 |
| 완전한 쌍 중 전체 ACK 100ms 충족 | 3 | 8 | 5 |
| 마지막 Path 진입 / 판정 | 101.350913ms / 거절 | 100.242099ms / 거절 | 100.513677ms / 거절 |
| 전체 판정 | FAIL | FAIL | FAIL |

V10의 45개는 과거 문맥 준비 10개·원래 안전 거절 4개·궤적 29개·ROS 전체 지연 거절 1개·마지막 실패 1개입니다. 마지막 44번은 Trajectory가 실제 발행·수신됐지만 Path가 거절됐고, 전체 ACK는 **113.549160ms**였습니다. 30번은 궤적을 보내기 전 지연 검사에서 거절돼 실제 출력 호출 0개입니다. 두 실패 형태와 뒤쪽 873개를 모두 유지합니다.

이번 실행은 **97.66초**였습니다. [V10 실제 원본](../../../../../storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v10/result.json), [전체 선언과 ROOT 검토](../../../../../storage/experiments/plans/2026-10-07/temporal_recorded_ros_shadow_v10/ROOT_review.json), [45개 전체 지연 그림](../../../../../storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v10_v3/ROS_prefix_all45.png), [전체 918+918행 CSV](../../../../../storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v10_v3/complete_timing.csv)를 보존했습니다. 그림은 실제 수치의 진단 렌더링이지 Autoware/RViz 캡처가 아닙니다. 코드 검사 187개·그림 검사 19개 통과도 주행 기능 승인이 아닙니다.

[V10 독립 전량 확인](../../../../../storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v10_independent_postcheck_v1.json)에서 현재·보존 소스 90개, 입력 5,893개, 생성 ROS 소스 42개, 환경 설정 2개와 이전 V8·V9 원본 불변을 확인했습니다. 실제 입력 855개·ACK 45개·출력 59개의 수신 근거를 확인했고, 소유 프로세스 4개는 모두 종료됐습니다. 전체 그래프의 연속 부재를 독립적으로 증명했다는 뜻은 아닙니다.

## V9 실제 결과: 50건까지 확인했지만 아직 미합격

<!-- HH_261007 - Report completed V9 with its authentic failed partial output and full ACK timing instead of a source-ready claim. -->

**V9 실행은 종료됐습니다.** 직접 계산은 918/918개 완료했고, 실제 ROS 기록은 50개입니다. 그중 35개에서 완전한 Trajectory/Path 쌍이 직접 계산과 일치했지만, 검사용 해시까지 포함한 전체 ACK 완료 100ms를 만족한 쌍은 **8/35개**뿐입니다. 출력 내용 일치와 제시간에 작동하는 것은 다릅니다.

| 항목 | V9 실제 결과 |
| --- | --- |
| 전체 분모 | 918개. ROS 비실패 49개 + 실패 1개 + 미실행 868개 |
| 입력·텐서·원시 출력 | 관측 50개에서 입력 CDR·텐서·5개 원시 head 출력 일치. 실패 시점의 실제 계산도 보존 |
| 실제 출력 | 완전한 35쌍, 부분 Trajectory 1개: 실제 수신 71개 |
| 완전한 출력과 전체 ACK 100ms | 8/35쌍. 실패·준비·거절을 포함한 관측 50행 중 전체 ACK 제시간 완료는 17행 |
| 마지막 49번 | Trajectory 발행·수신 후 Path 진입 **100.242099ms**로 거절. 전체 ACK **113.830434ms** |
| 검사용 해시·원시 출력 기록 | 발행 뒤에 실행하되 ACK에는 포함. 실제 평균 **9.79ms**, 최대 **14.08ms** |
| 전체 실행 | **98.41초**, 전체 판정 FAIL. 학습·차량 구동·모델 채택 없음 |

[V9 원본 결과](../../../../../storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v9/result.json)와 [독립 전량 확인서](../../../../../storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v9_independent_postcheck_v1.json)를 보존했습니다. 전체 현재/보존 소스 86개·입력 5,891개·생성 ROS 소스 42개·설정 2개와 소유 프로세스 종료를 확인했습니다. 뒤쪽 868개를 정상으로 채우지 않았습니다.

같은 앞쪽 9행에서 전체 ACK 평균은 V8 104.40ms → V9 103.05ms였지만, 처음 세 유효 쌍의 ACK는 오히려 조금 늘었습니다. 실행 한 번씩과 서로 다른 관측 길이만으로 지속적인 성능 향상을 주장하지 않습니다. 해시 위치 변경만으로 10Hz를 해결한 것도 아닙니다.

[ROS 관측 50개 전체 시간 그림](../../../../../storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v9_v2/ROS_prefix_all50.png), [직접 계산 전체 918개 그림](../../../../../storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v9_v2/direct_full918.png), [전체 918+918행 CSV](../../../../../storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v9_v2/complete_timing.csv)를 추가했습니다. 이는 **실제 기록 수치의 진단 그래프이며 실제 Autoware/RViz 스크린샷은 아닙니다.** 마지막 실패 계산 시간을 다른 시점이나 0으로 대체하지 않고 동일 시점의 원래 기록에서 읽습니다. 미실행 시간은 빈칸입니다. [그림 작성 코드 V2](../../../../../tools/e2e/report_recorded_ros_latency_20261007_v2.py)의 19개 검사도 통과했습니다.

## V8: 앞선 연결과 지연 실패

<!-- HH_261007 - Report actual6-message sender proof and the real nine-row prefix, including an emitted partial trajectory and the complete909 failed tail. -->

**직접 계산 918개는 완료했고, ROS 경유 비교는 9번째 시점에서 실패했습니다. 현재 이 실행은 종료됐습니다.** 시스템 ROS를 고치지 않는 프로젝트 전용 호환 코드에서 [실제 작은 송수신 6/6건](../../../../../storage/experiments/diagnostics/2026-10-07/owned_dds_info_bridge_v5/result.json)이 먼저 통과했습니다. 원래 수신 handle로 공개 DDS API의 실제 endpoint GUID를 조회하고, 원래 RMW bytes와 조회 근거도 보존했습니다. 그래프에서 이름만 가져와 송신자로 꾸민 것이 아닙니다. 설치된 Cyclone 플러그인·기존 라이브러리도 실행 전후 해시로 확인합니다.

| 항목 | V8 실제 결과 |
| --- | --- |
| 전체 직접 계산 | 918/918 처리 완료. 가중치 업데이트 없음 |
| ROS 처리 목록 | 정상·거절·과거 문맥 준비 8개 + 실패 1개 + 미실행 909개 |
| 같은 입력 확인 | 관측 9개 모두 송신/수신 CDR 일치. 실패 행의 모델 결과를 정상 동등성으로 세지 않음 |
| 같은 텐서·원시 출력 | 비교 가능한 앞쪽 8개에서 일치. 뒤쪽 909개는 미검증 |
| 실제 궤적/Path 전달 | 5·6·7번의 세 쌍이 직접 계산과 일치하고 100ms 이내 발행·완료 |
| 실패한 8번 | Trajectory는 실제 발행·수신됨. 이어지는 Path 진입이 **101.350913ms**여서 발행 거절. 부분 출력 보존 |
| 전체 실행 시간 | 93.28초. 전체 DDS 동등성·10Hz·주행 기능은 미합격 |

[전체 원본 결과](../../../../../storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v8/result.json), [실제 발행 호출과 거절](../../../../../storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v8/subscriber_output_calls.jsonl), [실제 출력 수신](../../../../../storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v8/publisher_received_outputs.jsonl)을 함께 보존합니다. 실패 당시 원래 모델 판단·마지막 실제 수신 정보·918개 비교 목록도 남겼습니다. 부분 성공을 첫 주행 기능의 완료로 세지 않습니다.

## 이전 V8의 지연 분석

<!-- HH_261007 - Separate original policy time, attachment/audit residual, actual trajectory publication and full ACK completion instead of blaming camera FPS. -->

앞쪽 성공 시점에서 입력 송신부터 모델 계산 시작까지 약30~33ms, 원래 모델 판단은 약47~51ms였습니다. 판단과 검사용 기록을 포함한 시간은 약57~60ms입니다. 그 차이 약9~10ms에는 결과 부착과 텐서/원시 출력 해시가 함께 들어 있어 순수 해시 비용으로 단정하지 않습니다. 8번의 입력 준비는 약37.5ms로 늘었고, 마지막 Path 호출이 기준을 넘었습니다. 처음 계산하거나 문맥이 다시 준비되는 시점의 긴 지연도 그대로 남깁니다.

현재 실패는 **기록 입력의 실행·검사 경로 지연**입니다. 실제 카메라의 낮은 FPS 때문이라고 결론 내릴 수 없고, 모델 가중치의 잘못만으로 설명할 수도 없습니다. 모델 계산 시간·메시지 준비·검사 비용을 따로 개선해야 합니다.

[전체 직접 계산 918개 시간 그림](../../../../../storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v8_v1/direct_full918.png)과 [ROS 관측 9개 전체 시간 그림](../../../../../storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v8_v1/ROS_prefix_all9.png), [전체 918+918행 CSV](../../../../../storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v8_v1/complete_timing.csv)를 만들었습니다. **실제 기록된 수치의 진단 그래프이며 Autoware/RViz 스크린샷은 아닙니다.** 미실행 909개 시간은 빈칸이지 0ms가 아닙니다. [작성 코드](../../../../../tools/e2e/report_recorded_ros_latency_20261007_v1.py)의 순수 검사 17개도 통과했습니다.

V9는 이 해시 위치 변경을 실제 실행했으며, 결과와 한계는 위에 구분했습니다. 이어 V10의 반복적인 전체 ROS 그래프 확인 위치 변경도 실행했고, V11에서 실제 수신 비용을 세분화했습니다. 메시지마다 소유 PID·실제 DDS 송신자·타입·내용 확인을 유지하고, provenance 수신·실제 살아 있는 노드의 추론 직전·출력 발행 시 전체 그래프를 확인했습니다. 검사 사이 잠깐 나타났다 사라진 외부 노드까지 계속 없었다고 증명하는 것은 아닙니다. NN·가중치·입력·거절·100ms 기준은 그대로입니다.

## V4 실제 결과

이 아래 V4는 이전 실패 기록입니다. 최신 V11 결과는 위 별도 구간에서 확인합니다.


| 항목 | 실제 결과 |
| --- | --- |
| 직접 계산 | 918/918 완료, NN 918회, 추가 중복 추론 없음 |
| 직접 궤적 출력 | 838개. ROS에 발행한 출력은 아님 |
| 과거 입력 준비 | 54개. 이 단계에서도 실제 NN은 실행됐음 |
| 안전 거절 | 역방향 이동 4개·주행/제동 여유 거리 부족 22개 |
| 직접 판단 시간 | 평균54.61ms·최대99.01ms |
| 직접 판단과 검사 기록을 합친 시간 | 평균88.12ms·최대138.54ms, 100ms 초과28개 |
| DDS 경유 | 수신0/918, 콜백 정보 인자 누락으로 실패 |
| 기능 승인 / 전체 주행10Hz / Autoware 전체 구동 | 모두 미입증 |

[전체 실제 결과](../../../../../storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v4/result.json), [직접 추론 전체 기록](../../../../../storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v4/direct_anchors.jsonl), [독립 전량 검산](../../../../../storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v4_independent_postcheck_v1.json)을 보존합니다. 해당 실행은 종료됐고 학습이 뒤에서 계속되고 있는 상태가 아닙니다.

## 해결한 API 호환 문제와 남은 지연

설치된 ROS Python 실행기는 콜백에 메시지 한 개만 전달하며, native API도 발신자 GID를 반환하지 않습니다. [모델 없는 실제 검사](../../../../../storage/experiments/diagnostics/2026-10-07/native_dds_info_probe_v1/result.json)로 확인했습니다. 단순히 콜백 인자를 없애면 발신자 검증까지 사라지므로 그렇게 합격 처리하지 않습니다.

실제 native bridge V5의 6건 검사와 기록 재생 V8~V11의 상태는 위에서 구분했습니다. 실제 발신자와 내용을 확인하는 연결은 작동했지만 전체 지연 기준은 미합격입니다. V1~V3의 실패, native bridge V4의 실행 전 검토 중단, 당시 소스와 선언은 별도로 보존하며 덮어쓰지 않습니다. 준비 검사 통과를 실제 송수신이나 주행 기능 승인으로 세지 않습니다.

## 이후 순서

작은 실제 송수신 검사는 이미 6/6건 통과했고, 기록 재생 V8~V11까지 실행했습니다. **다음은 아직 미계측인 송신·대기 비용과 전체 ACK 경로를 구분하는 일**입니다. 검사나 기록 비용을 타이머 밖으로 숨겨 통과시키지 않습니다. 공간 조건이 충족되면 같은 가중치의 GPU 출력·지연을 비교하고, 그 후 새 폐루프 주행·독립 평가를 통과해야 첫 목표 RTE-01 직진 차로 유지를 완료로 셉니다. 현재는 계속 메인0/9·세부0/30입니다.

<!-- HH_261007 - Record a source-reviewed executor candidate without calling it implemented or hiding timing and ownership costs. -->

[추가 executor 소스 검토](../../../../../storage/experiments/diagnostics/2026-10-07/executor_membership_source_review_v1.json)에서 `spin_once`마다 노드를 등록·제거하는 것을 확인했습니다. 설치본은 두 번 guard를 깨우지만 callback iterator를 재사용하므로 실제 waitset 재구성 횟수와 시간 이득은 아직 **UNKNOWN**입니다. 후보는 subscriber의 정확한 소유 노드만 한 번 등록하고 종료 시 분리하는 방식이며 **구현·실행하지 않았습니다**. startup probe를 남기지 않고, 파괴·정리 예외와 publisher의 바깥 `node=None` 상태까지 검사해야 합니다. `destroy_node()`만 호출하면 executor 소속도 제거된다고 가정하면 안 됩니다. 송신 JPEG 읽기·직렬화·실제 발행·poll 간격 계측과 함께 별도 새 리비전에서 검증해야 합니다.

과거 약209m 시험의 [실제 주행 결과와 시각 자료](../../2026-10-05/portable_e2e_learning_cycle_v1/01_town03_full_route209_v5/README.md)는 당시 실패를 포함한 원래 결과입니다. 오늘의 ROS 기록 검사는 새 실주행 영상이나 실제 RViz 캡처를 만든 시험이 아닙니다.
