# HH_261001 — Town07 저속 스로틀 A/B v2: 실행 전 사전등록

**현재는 설계·합성 테스트·계획 선언 단계이며 CARLA 주행 결과가 없다.** [v1 선언](TOWN07_THROTTLE_ISOLATION_IMPLEMENTATION.md)은 보존하되 실행하지 않는다. v1은 spawn 이후 정지 준비가 0.05초뿐이어서 3.5초 정지 준비를 쓴 비교 V3와 시작 상태가 달랐다. v2는 그 차이를 줄이기 위해 3.5초, 20 Hz 기준 70개의 정지 tick을 반드시 통과시킨다. V3의 속도 램프/정상 제어 정책 전체를 동일하게 재현했다는 뜻은 아니다.

## 무엇을 고정했나

| 단계 | 고정 내용 및 실패 처리 |
| --- | --- |
| 원천 | Town07 직진 route, 동일 CARLA egg/OpenDRIVE, Prius, ClearNoon, 20 Hz·기존 substep 설정. 선언 계획에 실행 소스와 CARLA BasicAgent 파일 해시 포함 |
| 준비 | 1개 full-brake bootstrap tick은 별도 표기. 이어서 **70개 정지 warmup tick**, 매 tick full-brake ACK·원시 actor snapshot·제어 readback·충돌/차선 이벤트·신호·경로 상태를 보관 |
| warmup 종료 | `|vx|≤0.05 m/s`, 경로 진행 `0–0.25 m`, bootstrap 대비 수평 이동 `≤0.25 m`; 위반·이벤트·ACK/프레임 불일치면 실패 로그를 남기고 중단 |
| 분기 전 | BasicAgent 9 km/h, 정상 스로틀 상한 0.20/정상 제동 0, 비상 제동은 기존 코드 유지. 최대 800 native tick. 분기 속도 `2.35–2.55 m/s`, 목표 진행 거리 16 m 또는 25 m의 첫 교차, 초과 `≤0.25 m` |
| 분기 후 | 위치별 A–B–B–A, 총 8회. A 스로틀 `0.01`, B `0.20`; 조향/제동 각각 0; 회당 60 native tick, 3초. 카메라·학습 모델 호출 0 |
| 공통 즉시 중단 | 충돌·차선 이벤트, 적색 신호, `|vx|>2.7 m/s`, `|CTE|>1.0 m`, 제어 ACK/프레임 불일치, 디스크 하한. 비동기 이벤트의 무지연 도착은 증명하지 못해 0.2초 후 추가 확인 |

각 회차는 별도 CARLA 서버 소유권/포트·workspace 잠금 검사를 거친다. 계획 또는 소스 해시가 변하면 실행하지 않는다. 종료한 회차를 덮어쓰지 않고, 실패해도 원시 로그와 실패 표시를 보존한다. 앞 회차의 소유권·정리 확인 없이는 다음 번호로 넘어가지 않는다. 이 진단은 저스로틀 차체 반응의 실마리를 찾는 것이지, 물리 토크 계측이나 E2E 품질 승격이 아니다.

## 상태와 재개 방법

v2 합성 테스트는 **16 passed**이고, Python 문법 검사도 완료한다. 계획은 `storage/experiments/validation/2026-10-01/town07_throttle_isolation_plan_v2.json`, 실행 결과는 `storage/experiments/validation/2026-10-01/town07_throttle_isolation_v2/`에만 생성한다. 디스크에는 **5 GiB 보존선 + 128 MiB 시작 예산**이 있어야 한다. 현재는 부족하므로 실제 CARLA 실행을 하지 않는다. 기존 자료나 `/tmp`를 임의 삭제하여 공간을 만들지 않는다. 카메라가 없어도 CARLA 서버 자체의 GPU 사용은 배제되지 않는다.

공간·다른 CARLA 작업 점유가 해소된 다음, **읽기 전용** 사전점검부터 한다. 계획의 SHA를 매번 읽어 전달하되, 선언된 원본 파일·코드를 수정하지 않는다.

```bash
cd /home/a/autoware_e2e
THROTTLE_V2_PLAN_SHA="$(sha256sum storage/experiments/validation/2026-10-01/town07_throttle_isolation_plan_v2.json | awk '{print $1}')"
python3 tools/workspace_python.py -m scripts.e2e.run_owned_town07_throttle_isolation_v2_20261001 -- --mode preflight --trial 0 --plan-sha256 "$THROTTLE_V2_PLAN_SHA"
```

사전점검이 **통과한 경우에만** 같은 명령의 `--mode preflight`를 `--mode execute`로 바꾸어 **한 회차씩** 실행한다. `trial`은 0부터 7까지 고정 순서이고 자동 재시도는 없다. 결과 비교는 각 회차의 `throttle_result.json`과 원시 `episode.partial/states.jsonl`, `control_receipts.jsonl`을 모두 읽어야 한다. 핵심 수치는 동일한 20 Hz 원시 `Δvx/Δt < −8 m/s²` 횟수, 3초 순 진행 거리, 첫 `vx≤0.05 m/s` 시점이다.

어느 결과든 기존 V3의 학습 자료 편입, 스로틀 하한의 모델 적용, 9개 주기능/30개 세부기능 합격 또는 실차 승인으로 이어지지 않는다. 데이터·모델·제어의 다음 한 변수 A/B는 이번 진단 결과를 읽은 뒤 별도 사전등록한다.
