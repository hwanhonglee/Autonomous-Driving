# 03 — CARLA·Autoware·수집

<!-- HH_260906 - Keep owned simulation launch and real control boundaries visible. -->

- [전용 CARLA 실행·정리 wrapper](../../../tools/e2e/run_owned_carla_expert_trial.sh)
- [Autoware launch 설정](../../../autonomy/autoware_e2e_vad_launch/launch)
- [설치·환경 사전 진단](../../../tools/e2e/bootstrap_preflight.sh)
- [초보자 시작 가이드](../../BEGINNER_QUICKSTART_KO.md)

로컬 시뮬레이션 전 다른 테스트의 GPU·포트와 workspace lock을 확인합니다.
`client.load_world()`나 임의 프로세스 종료로 다른 테스트를 방해하지 않습니다.
기본 에이전트 주행·VAD 주행·자체 Portable 주행은 서로 다른 결과입니다.

지도 수집 실행 예는 [실험별 재현 명령](../../assets/validation/2026-09-22/portable_e2e_learning_cycle_v1/22_mapping_live_replay/README.md)을 사용하세요.
