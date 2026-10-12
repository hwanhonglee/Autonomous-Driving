# V3 — 모델 주행 전에 종료된 저장·원복 오류

<!-- HH_261004 - Publish complete failure metadata only; absent sensor pixels cannot become genuine screenshots or driving evidence. -->

**이번 실행은 모델 성능을 평가하지 못했다.** 첫 정지 준비 구간의 상태 1개·카메라 묶음 1개를 기록한 뒤 저장 오류로 종료됐다. NN 추론·학습 모델 명령·교사 출발 명령은 모두 0개다. 원래 계획한 전체 209m 주행을 완료한 것이 아니다.

오류는 두 가지다.

1. **저장기의 입력 검사:** 여섯 작업자가 모두 `immutable exact640x360 original BGRA source required`로 거부했다. 원래 지연 저장기는 실제 콜백 객체를 보관하지만 새 검사는 원본을 `bytes`로만 허용했다. 원복용 모델 입력의 별도 불변 복사본과 저장기에 들어가는 실제 객체를 혼동했다. 정확한 실패 조건·실제 버퍼 타입은 당시 기록하지 않아, 타입 불일치는 원인 후보이지 직접 관측한 값은 아니다. V4에서는 먼저 입력 형식을 기록하고 원본/불변 복사/압축 뒤 바이트 해시를 전부 검증한다.
2. **신호등 개수 검사:** 실제 원본은 38개이며 첫 신호등의 객체 번호가 43이다. 새 helper는 개수를 43으로 잘못 고정했다. 따라서 원복 tick도 실행하지 못했다. 새 수정본은 38개 전체의 정확한 구성원을 새 실행에서 다시 확인하고, 차량·센서·소유권·물리 설정 검사를 유지한다.

JPEG는 **0장**이다. 실제 PNG/GIF를 만들 원본 픽셀이 남지 않아 미디어를 생성하지 않았다. 엄격한 게시 검증도 원래 실패 진단에서 차체 요약이 생성되지 않아 거부했다. 없는 파일이나 화면을 채워 넣지 않는다.

전체 원본·소스·실패 기록 **1,958파일**, 게시 소스 14개를 종료 후 다시 해시해 변화가 없음을 확인했다. 소유한 프로세스·포트 정리는 끝났지만, 캡처 manifest의 신호등 원복 실패와 원래 인코딩 오류는 보존했다. 이 자료는 학습에 사용하지 않는다.

다음은 같은 GOOD6000·CPU4·20Hz 물리·10Hz 카메라·속도/제어/100ms 기준을 유지한 **별도 V4 수정본**이다. 이번 실패를 덮어쓰거나 기능 승인으로 바꾸지 않는다. 정식 완료는 메인 0/9·하위 0/30 그대로다.

[전체 실패 원본](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v3/diagnostic_report.json), [원래 오류 로그](../../../../../../storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v3/collector.log), [전체 파일 사후 봉인](../../../../../../storage/experiments/plans/2026-10-04/town03_known_dev_straight_full209_v3/ROOT_actual_publication_caller.json), [독립 전량 검산·미디어 미생성](../../../../../../storage/experiments/diagnostics/2026-10-04/town03_full209_v3_preNN_failure_postcheck/ROOT_complete_failure_receipt.json).
