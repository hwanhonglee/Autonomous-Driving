# 직진은 유지했지만 실제 수신 지연은 미달입니다

<!-- HH_261004 - Separate native driving, frozen callback failure and a new exact-object causal diagnosis. -->

이번 로컬 CARLA 시험은 종료됐습니다. GOOD6000 모델의 곡률·속도 출력으로 주행했고 교사는 출발 구간에만 사용했습니다. 정식 기능 완료는 아직 메인 0/9·하위 0/30입니다.

| 검사 | 실제 결과 | 판단 |
| --- | --- | --- |
| 전체 기록 | 45초·상태 900개·6카메라 450묶음·ACK 904개 | 전체 실패 구간까지 보존 |
| 실제 모델 제어 | 약 96.37m·차량 중심 오차 p95 2.83cm, 최대 3.61cm | 이 저속 직진 구간은 잘 따라갔지만 전체 경로·독립 평가 아님 |
| 충돌·차선·차체/sweep | 원래 검사 통과 | 안전 기준을 낮추지 않음 |
| 실제 영상 객체와 콜백 연결 | 같은 프레임 번호의 서로 다른 객체를 기존 판정 코드가 거부 | 원래 v2 **FAIL 유지** |
| 별도 읽기 전용 원인 분석 | 실제 소비 객체를 연결하면 수신→계획 최대 109.83ms·수신→ACK 최대 111.28ms | **100ms 미달**, 연결 수정만으로 해결되지 않음 |

같은 프레임 번호라고 같은 영상 바이트는 아니었습니다. 다음 수정은 실제 모델이 소비한 영상의 객체 식별값·프레임·시각·SHA가 모두 맞는 단 하나의 콜백만 연결합니다. 더 빠른 콜백을 임의로 골라 시간을 줄이지 않고, 사용하지 않은 콜백과 실패도 남깁니다. 이 원인 분석은 원본 판정이나 기존 승인 수를 바꾸지 않습니다.

다음 순서는 지연 구간 계측 → 필요한 실행 최적화 → 같은 기준의 새 실제 시험 → 전체 209.37m 목표점 접근 확인입니다. 목표점 접근용 내비게이션 보조 처리와 모델의 학습된 정지 기능은 구분합니다. 독립 TEST와 실제 Autoware 어댑터 검증 없이 기능 완료로 세지 않습니다.

- [실제 전체 45초 GIF·차량 중심 PNG 5장](README.md)
- [원래 FAIL 진단](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_callback45_v2/diagnostic_report.json)
- [실행 전 고정 선언](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_callback45_v2/declaration.json)
- [게시 후 전체 원본·JPEG·미디어 재검증](../../../../../../storage/experiments/diagnostics/2026-10-04/callback45_v2_publication_postcheck/ROOT_actual_whole_run_and_media.json)

화면은 실제 기록 기반 CARLA 진단 렌더링이며 Autoware/RViz 캡처가 아닙니다. 설치·재부팅·다른 사용자의 작업·GPU1·실차 actuation은 건드리지 않았습니다.
