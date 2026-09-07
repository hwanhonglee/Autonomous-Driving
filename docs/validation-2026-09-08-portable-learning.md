# 2026-09-08 Portable E2E 학습·수집 개선

<!-- HH_260906 - Link the dated work ledger without presenting an ongoing campaign as deployment completion. -->

[최신 결과 폴더](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/README.md)에
정답 데이터 진단, 생성기를 고정한 선택기 9회 학습, 로컬 자연 정지 수집 개선을 분리해 기록합니다.

<!-- HH_260906 - Add completed pedal measurements and corrected warmup accounting without promoting data or models. -->
로컬의 [고정 페달 12개 계측](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/04_pedal_response_calibration/README.md)은
완료됐습니다. 일정한 페달에서도 출발 가속과 저속 급정지가 남아 기존 제어기 전환만을 원인으로
단정할 수 없습니다. 특정 물리 내부 원인은 미확정이며, 앞선 자연 정지 수집 두 시도는 모두 품질 FAIL로
학습에 채택하지 않았습니다. 다음은 이 계측에 근거한 원인 분리·수집 제어 개선과 재검증입니다.

[Warmup·과거 입력 감사](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/06_warmup_history_audit/README.md)는
v3의 warmup 앵커 train105/1,147·val35/337 포함을 확인했습니다. 9월 5일 문서의 잘못된 제외
설명을 정정했으며, 기존 데이터·평가 분모는 유지합니다. 향후 history-only 방안은 미채택 제안입니다.

원격은 개인 venv·GPU0의 학습/평가 전용, 로컬은 CARLA 데이터 수집·추론/제어 검증 전용입니다.
새 선택기들은 세 seed 전체 기준을 통과하지 못해 승격하지 않았습니다. 학습 모델의
폐루프 주행·실차 제어 승인은 없으며 기존 shadow 모델을 유지합니다.

사용자 요청에 따른 이번 작업 경계는 **2026-09-08 오전 10시 KST**입니다.
실제 완료·실패·진행 상태와 다음 순서는 위 결과 폴더의 기록을 확인합니다.

<!-- HH_260906 - Report completed raw driving evidence without interpreting scalar success as dataset or model approval. -->
[Town07 전체 경로 재시험과 실제 화면](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/10_comfortable_v3_pilot/README.md)에
2회 전체 결과와 PNG 12장·GIF 2개를 추가했습니다. 최고 속도 약 28.6 km/h·8.6초 이상
순항·목표 앞 자연 정지는 확인했지만, 1회 급감속 초과와 두 실행의 제어 기록 시점 불일치가
있어 모두 미승인입니다. 다음 비교는 서버의 명령 수신 확인과 동일 프레임 계측입니다.
카메라 Low 설정의 체크무늬 도로도 별도 정지 상태 영상 검사로 분리합니다.
