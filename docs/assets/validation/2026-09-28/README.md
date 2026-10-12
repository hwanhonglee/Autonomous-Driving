# 2026-09-28: 주행·지도 다음 데이터 관문

<!-- HH_260928 - Keep offline feasibility, owned simulator capture, and learned-model approval distinct. -->

**현재 모델 판정:** 주행 AH와 지도 AI v2 체크포인트는 존재하지만 새 후보를 승격하지 않았다. AH의 Town03 DEV 우회전 ADE/FDE/속도 절대 기준은 미달이고 학습 모델 폐루프 승인 기능은 **9개 상위/30개 하위 중 0/30**이다. 지도 B/C 후속은 Town01 DEV 차선·표지판 보호 기준에 실패해 AI v2를 유지한다. 이번 날짜에 PRO6000 GPU 학습·SSH·패키지 설치는 하지 않았다. 여기 적힌 CARLA 차량은 BasicAgent 전문가이며 학습 E2E 주행이나 Autoware 화면이 아니다.

| 갈래 | 오늘의 실제 진전 | 아직 통과하지 못한 것 |
| --- | --- | --- |
| 지도 | [Town13 표지판 3개 거리/연결성](map_town13_sign_cluster_feasibility/README.md): 기존 240장으로 필요한 최소 구간 148.309m를 촬영하기 어렵다는 정적 판정. [source-editor 카메라 원본](map_town13_render_sensor_scout/README.md): v2에서 240 RGB/depth/semantic 쌍·93.786m·충돌0·replay/cleanup PASS, PNG/GIF. [정지 카메라 A→B→A](map_town13_skydome_location_aba/README.md): A1 10장 보존 | v2의 **240/240 RGB**, A1의 **10/10 RGB**에 skydome 경고. B 이동 직후 UE4Editor `SIGSEGV`로 B/A2 0장, A/B 위치 판정 불가. owner `FAIL`, 소유 process-group 정리 판정 false(현재 포트는 비어 있음). 학습/DEV 영상 품질 불합격·독립 DEV 미확인 |
| 주행 | [Town01 다른 진입 방향 우회전 단회 원본](drive_town01_j87_prereg/FAILED_TRIAL.md): live 맵 SHA 일치, 실제 우회전 약75.3°, 20/10Hz·6카메라/ACK 보존, 차량 중심 화면·전체 경로 PNG. 정적 명목 10cm 차체/스윕 침범0 | **수집 실패:** 신호 대기로 추정되는 31.95초 저속 구간 때문에 고정 창 연속 이동 28.35초<30초, 목표 거리 2.552m>2.500m. 완료 tail 없음. 9개 운동/실제 차체 전량 합격 아님. V5 가까운 다른 방향이 같은 교차로일 수 있어 독립 site 2개로 세지 않음 |
| 독립 회전 탐색 | [비-TEST C-track 3,015쌍 오프라인 재생](drive_next_independent_scout/README.md): XODR·TRAIN split·검사 코드 해시 고정, 오른쪽 회전 연결로 65개에서 네 길이 조건을 전수 검사 | 물리 회전 기준 합격 **0개**. 가장 가까운 175.553m 경로도 5m 경사 18.007%로 허용 9.983% 초과. live 맵 일치·차체 여유·실제 주행은 미검증이며 수집 허가 없음 |

**다음 순서:** (1) 동일 Town01 회전을 반복 촬영하거나 실패 기준을 완화하지 않는다. 다른 비-TEST 물리 site에서 넓은 우회전을 정적으로 찾고 live XODR·신호 진입부·10cm 차체/스윕을 먼저 고정 검사한다. 수집 가능한 route가 생기면 **촬영 전에** 신호 대기와 연속 이동 30초의 창 계약, 종점 2.5m 계약을 사전 등록하고 선택형 `--observe-traffic-lights`로 신호 상태를 보존한다. (2) Town13은 [센서 스트림 휴면·파괴 충돌 분석](map_town13_skydome_location_aba/CRASH_ANALYSIS.md)에 따라 살아 있는 카메라의 장거리 이동을 중단했다. 위치마다 새 소유 source-editor를 쓰더라도 목표 타일 거주 여부를 Python/RPC에서 읽을 수 없어 안전한 대체 촬영 owner는 **아직 구현·실행하지 않았다**. 프로세스별 자동 크래시 전송 차단 인자는 추가했지만 **실행 효과는 미검증**이다. 타일 관문을 해결하고 정상 RGB를 확보한 뒤 실제 방향으로 연결된 3개 표지판의 긴 경로·프레임 수를 잠근다. **같은 A→B→A 출력은 재사용하지 않는다.** (3) 주행은 반복 TRAIN+독립 DEV, 지도는 sign-capable 독립 DEV와 라벨 객체 검수를 통과한 후에만 PRO6000 물리 GPU0·개인 venv에서 각각 동일 초기 가중치/seed 예산의 A/B 학습을 재개한다. GPU1·공용 설치·TEST 데이터는 사용하지 않는다.

**시간/사용량:** 이번 Town01 단회는 CARLA 시뮬레이션 약97초의 원본을 남겼으나 QA 전에 실패 종료했다. 다음 독립 경로 수집은 맵 시작·원본 검사에 따라 수십 분 이상 달라질 수 있다. Town13 렌더 수리와 충돌 원인은 미확정이라 소요 시간을 확약하지 않는다. UE4 크래시 로그에는 Epic CrashReportClient **자동 전송 성공** 기록이 있어 추가 source-editor 실행은 크래시·전송 통제 후 판단한다. 대화 크레딧·토큰 잔액은 이 작업공간에서 조회할 수 없다. 오늘 실행한 모델 재학습은 **0회**다.
