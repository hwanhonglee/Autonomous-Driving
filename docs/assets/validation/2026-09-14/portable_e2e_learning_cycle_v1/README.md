# 9월 14일 — 데이터 진단·제동 개선·자체 전체 모델 학습

<!-- HH_260906 - Publish the new data-readiness diagnostic separately from the sealed September 9 experiments. -->

이 폴더는 9월 14일의 **새 검사**를 모은다. 9월 9일 학습 결과의 복구·게시와 구분하며,
기존 날짜의 실행 증거·원본 그림·checksum은 이번 작업으로 바꾸지 않았다.

| 분류 | 내용 | 상태 |
| --- | --- | --- |
| 모델 설계 참고 | [METEOR 공식 자료 검토·자체 모델 반영 계획](../../../../portable-e2e-meteor-review-2026-09-14.md) | 공식 소스/입출력 검토; 설치·추론 없음 |
| 01 데이터 준비 상태 | [TRAIN/VAL 장면·정지·보조 정답 현황](01_data_readiness/README.md) | 실제 CPU 감사 완료; 학습 승인 아님 |
| 02 정지 미세 움직임 | [서비스·주차 브레이크 ABBA 실측과 6카메라·위치 PNG](02_stationary_motion/README.md) | 4회 완료·1,880상태·5,640 JPEG; 학습 편입 아님 |
| 03 저속 무페달 감속 | [45초 coast 반복·저속 속도 부호 교대·6카메라 PNG](03_low_speed_coast/README.md) | 2회 완료·2,026상태·6,072 JPEG; 완전 정지·학습 승인 아님 |
| 04 저속 감속→서비스 제동 | [무페달/제동 ABBA·경계 잔차·차량 중심 6카메라 PNG](04_coast_brake_transition/README.md) | 4회 완료·4,052상태·12,144 JPEG; 제동 후 정지 관측·학습 편입 아님 |
| 05 실제 전체 모델 학습 | [STOPMIX 전체 모델 6회 학습·A/B 지표·ego 중심 경로 PNG 72장](05_stopmix_regret_learning/README.md) | GPU 0 학습 6회·후속 검사 18회·독립 검산 완료; 상대 비교 2/3·절대 기준 0/3로 미채택 |
| 06 자연 감속→브레이크 유지 | [Town07 v5 원본 6카메라·ego 중심 경로·실제 제동 기록](06_brake_hold_pilot/README.md) | 1회 완료·정지 유지 관측; 미래 곡률 134/666 실패로 학습 미편입 |
| 07 학습량·전체 맵·곡률 재현 | [3개 학습 episode의 실제 범위·19맵 계획·39시점 원본 재현·분석 PNG](07_coverage_and_curvature/README.md) | 검사 구현·실행 완료; 636 passed / 1 skipped; 추가 학습·새 주행 아님 |
| 08 저속 조향 분리 실험 | [실제 3회 수집·잔여 출발/제동 경계·PNG 15장·GIF 3개](08_low_speed_steering_abba/README.md) | 4회 계획 중 3회 완료·A2 포트 사전 검사 차단; 실패 관측 감소·학습 미편입 |
| 09 정지 제동 램프 | [완전한 ABBA 4회·제어/실측 궤적 비교·PNG 25장·GIF 4개](09_hold_brake_ramp/README.md) | 네 회차 모두 115/671구간·3시점 실패; 램프 개선 없음·학습 미편입 |

현재 모델·센서 ABI·판정 기준은 유지한다. GPU 1, 시스템/Conda, 원격 다른 작업은 변경하지 않았다.
이 폴더에는 새로운 학습 모델 자율주행 영상이 없다. 02번은 실제 로컬 CARLA의 정지 상태,
03/04번은 약 1.8 km/h 출발 후 무페달 감속과 서비스 제동 비교 계측이다.
04번의 첫 제동 간격에는 미세 후방 위치 변화와 속도 적분 잔차도 남아 있다.
Autoware 실행이나 학습 모델 제어가 아니다.
02번의 초기 실패 2회와 포트 대기 기록도 별도 보존했다.
METEOR 가중치와 demo도 받거나 재배포하지 않았다.

<!-- HH_260906 - Distinguish the new remote legacy-data research from local unadmitted capture. -->
05번은 **서버의 실제 자체 모델 학습**이고, 06번은 **로컬 BasicAgent 데이터 수집·검사**다.
05번은 15:29:30~15:58:47 KST에 정상 종료했으며, 학습 9,240 step·36,930회 표본 노출과
모든 결과를 검산했다. 현재 해당 학습 작업은 완료된 상태이지 계속 백그라운드 학습 중인 것은 아니다.
05번은 기존 TRAIN 1,147개·VAL 337개만 사용한다. TEST payload와 02~04/06 진단 자료는
학습에 넣지 않았다. 독립 정지/회전 데이터와 유효한 보조 정답 확보는 별도의 다음 데이터 작업으로 남는다.

<!-- HH_260906 - Distinguish fit repetition from independent map coverage and retain the new audit's non-execution scope. -->
07번에서 TRAIN은 **3 episode·3맵·133.9초**, VAL은 **1 episode·1맵·40.15초**임을 확인했다.
6회 학습이 충분하거나 모든 맵에서 주행한다는 결론은 아니다. 19맵·5장면 유형의 95개 계획
항목에 split/맵 자산/정지 의도 등의 부족분을 표시했다. 이는 30개 기능과 다른 분모다.
v5 곡률의 612개 중복 실패를 39시점으로 재현했으며 실패 flag는 그대로 보존했다.

<!-- HH_260906 - Add the fresh incomplete ABBA without rewriting earlier evidence or treating reduced failures as admission. -->
08번에서는 같은 Town07 직진에서 기존 조향 A1과 저속 조향 0인 B1/B2를 실제 수집했다.
곡률 실패는 A1 **134/666구간·39시점**, B1/B2 각각 **115/671구간·3시점**이다.
마지막 A2는 포트 2101 점유 검사에서 실행 전 차단되어 재시도하지 않았다. 완성된 ABBA
판정은 없으며, 남은 출발 2시점·제동 1시점 때문에 새 데이터 학습도 하지 않았다.
3회 원본 13,218 JPEG와 ego 중심 PNG 15장·실제 프레임 GIF 3개를 보존했다.
이번에 원격 작업을 실행한 것은 아니며 05번 학습은 이미 종료된 상태다.

<!-- HH_260906 - Append the complete negative v7 result without rewriting the incomplete v6 evidence. -->
09번은 정상 정지 제동만 즉시/0.5초 램프로 비교한 **별도 4회 실측**이다. 포트 사전 관측과
기존 소유 실행기 검사를 네 회차 모두 통과했고 자동 재시도는 없었다. 브레이크 요청/읽기는 달랐지만
관측한 물리 상태와 출발 2·제동 1시점의 곡률 실패는 그대로다. 5,888상태·17,664 JPEG를 검사하고
PNG 25장·GIF 4개 및 원시 미래 창을 보존했다. 관련 회귀는 **958 passed / 1 skipped**다.
이번 새 자료도 학습 미편입이며 원격 학습을 추가 실행하지 않았다. 다음은 경계의 위치·속도·좌표 기준
분리 진단이며, 이번 음성 결과를 숨기거나 같은 데이터를 반복 학습해 해결됐다고 보고하지 않는다.

[전체 현황·9개 영역/30개 기능](../../../../portable-e2e-status-2026-09-14.md) ·
[검사 결과 JSON](01_data_readiness/data_readiness.json)
