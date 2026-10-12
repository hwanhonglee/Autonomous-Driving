# Fix moving 플랫폼 Autoware 통합 준비

<!-- HH_261012 - Reserve this branch for a separately commissioned vehicle interface without adopting CARLA control calibration. -->

이 브랜치는 `autoware-platform/v1.9.0-fixmoving`입니다. 공통 Autoware 소스 snapshot에서 실차 플랫폼의 인터페이스·센서·안전 연결을 개발하기 위해 분리했습니다. **플랫폼 통합 준비용이며 실차 구동 승인이나 완성된 vehicle interface가 아닙니다.**

Fix moving의 정확한 차량 모델과 기존 인터페이스 패키지는 아직 확인되지 않았습니다. CAN/DBW 프로토콜·차량 치수·actuator 보정·상태 메시지와 명령 주기·비상정지·운전자 개입 조건을 임의로 설정하지 않습니다.

CARLA 실행·보정·연구 코드는 공통 이력으로 남아 있지만 실차 launch로 사용하지 않습니다. 자체 portable E2E 모델은 연구 상태이며 자동으로 차량 제어에 연결하지 않습니다. 먼저 비구동 메시지·센서 보정·통신 중단 시 명령 차단·기록 재생을 검증하고 별도 승인 아래 차량 시험을 준비해야 합니다.

이 안내는 실행을 차단하는 소프트웨어 interlock이 아니라 브랜치의 목적과 제한입니다. 실제 안전 인터페이스의 구현과 시험을 대체하지 않습니다.

[세 브랜치와 재현 방법](docs/autoware-three-branch-guide.md) · [작업공간](WORKSPACE.md) · [자체 모델 현황](docs/assets/validation/2026-10-12/README.md)
