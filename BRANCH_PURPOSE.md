# CARLA E2E 주행과 지도 모델 연구

<!-- HH_261012 - Declare this branch as simulation research without authorizing real vehicle actuation or model promotion. -->

이 브랜치는 `autoware-e2e/v1.9.0-vad-carla`입니다. CARLA와 Autoware의 실행·수집·제어 시험, 자체 portable E2E 주행 모델과 별도 자동 지도 모델의 학습·평가에 사용합니다.

모델 출력·teacher 데이터·제어기 보정을 구분합니다. 시뮬레이션 보정값은 실차 설정이 아니며, 이 브랜치의 launch나 연구 모델을 실제 차량에 자동 연결하지 않습니다. 정식 승인 메인 0/9·하위 0/30과 기존 실패 분모를 유지합니다.

[세 브랜치의 용도와 재현 방법](docs/autoware-three-branch-guide.md) · [작업공간 구조](WORKSPACE.md) · [현재 학습과 검증](docs/assets/validation/2026-10-12/README.md)
