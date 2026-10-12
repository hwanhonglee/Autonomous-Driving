# HH_261001 — Town07 저속 스로틀 A/B v3: 정지 중 이동량 분리

**v3는 계획 선언과 합성 검사 단계이며 실제 CARLA 주행 결과는 없다.** [v1](TOWN07_THROTTLE_ISOLATION_IMPLEMENTATION.md)은 준비 시간이 부족하고 [v2](TOWN07_THROTTLE_ISOLATION_V2_WITHDRAWN.md)는 정상 시작 위치를 잘못 거부하므로 둘 다 실행하지 않는다. 그 소스·계획 SHA는 그대로 보존한다.

## 정정한 한 가지 시작 조건

동일 Town07 route SHA256 `8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9`를 쓴 기존 V3 전문가 원본 `storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v1/episode/states.jsonl` SHA256 `c0e6f48268d654cb83bd5608ee2829ea7e3f813c49860b2659c0cac22da5fc62`의 정지 준비 **70개 프레임 모두 진행 0.318099893 m**였다. 첫 frame은 1147이다. 이는 멈춘 차량의 시작 좌표이지 정지 준비 중 0.318 m 움직였다는 뜻이 아니다. v2가 종료 시 절대 진행을 `≤0.25 m`로 제한해 정상 상태를 실패로 분류한 점을 수정했다.

v3은 준비 첫 bootstrap 상태와 종료 상태를 **각각 절대 시작 통로 `0–0.5 m`** 안에 두고, 두 상태의 **진행 변화량 절댓값 `≤0.25 m`**, 수평 이동 `≤0.25 m`, 종료 종방향 속도 `|vx|≤0.05 m/s`를 별도로 검사한다. 합성 회귀 테스트는 원본 `0.318099893 m`가 이동량 0으로 통과하고, `0.251 m` 변화는 실패함을 확인한다. 이것은 관측 근거에 맞춘 사전 고정 범위이지, 실제 새 실행의 속도·차량 상태 동일성을 증명하지 않는다.

## 바뀌지 않은 고정 A/B

1개 full-brake bootstrap tick 뒤 20 Hz에서 **70개 full-brake 정지 warmup tick(3.5초)**을 매번 ACK하고 원시 actor snapshot, 제어 readback, 경로 진행/CTE, 신호·충돌·차선 이벤트를 남긴다. 이후 같은 BasicAgent 9 km/h 접근 조건으로 16 m와 25 m의 첫 목표 교차를 기다린다. 분기 속도는 `2.35–2.55 m/s`, 위치 초과는 `≤0.25 m`이고, 조건 미달은 실패로 보관한다. 분기 후에는 각 위치별 **A–B–B–A** 순서로 스로틀 A=`0.01` 또는 B=`0.20`만 바꿔 조향=0, 제동=0으로 60 native tick(3초)을 계측한다. 자동 재시도는 없다.

충돌·차선 이벤트, 적색 신호, `|vx|>2.7 m/s`, `|CTE|>1.0 m`, ACK/스냅샷/프레임 불일치 또는 디스크 하한은 실행 중단 사유다. 비동기 이벤트의 무지연 도착은 보장하지 못하므로 정지 준비와 분기 종료 뒤 각각 0.2초의 늦은 이벤트 확인을 더한다. 카메라·학습 모델 호출은 없으나 CARLA 서버의 GPU 사용을 배제하지 않는다. 소유형 실행기는 workspace 잠금, 다른 CARLA 프로세스/포트 확인, 소스 재검증, 실행 후 자기 프로세스 정리를 수행한다.

## 현재 관문과 재개

v3 계획 경로는 `storage/experiments/validation/2026-10-01/town07_throttle_isolation_plan_v3.json`, 회차별 새 출력은 `storage/experiments/validation/2026-10-01/town07_throttle_isolation_v3/`이다. 계획 파일의 SHA와 함께 CARLA egg, OpenDRIVE, route, 실행 소스·CARLA BasicAgent 소스를 선언 뒤 고정한다. **디스크 5 GiB 보존선 + 128 MiB 시작 예산이 부족하므로 지금은 CARLA를 실행하지 않는다.** 임의의 기존 데이터 또는 `/tmp` 삭제로 공간을 확보하지 않는다.

다른 CARLA 작업이 없고 공간이 충족된 뒤에도 첫 단계는 읽기 전용 사전점검이다.

```bash
cd /home/a/autoware_e2e
THROTTLE_V3_PLAN_SHA="$(sha256sum storage/experiments/validation/2026-10-01/town07_throttle_isolation_plan_v3.json | awk '{print $1}')"
python3 tools/workspace_python.py -m scripts.e2e.run_owned_town07_throttle_isolation_v3_20261001 -- --mode preflight --trial 0 --plan-sha256 "$THROTTLE_V3_PLAN_SHA"
```

사전점검과 자원 소유권 확인을 모두 통과한 후에만 **한 회차씩** `--mode execute`를 검토한다. 판독은 20 Hz 원시 `Δvx/Δt < −8 m/s²` 횟수, 3초 순 진행, 첫 `vx≤0.05 m/s` 시점을 A/B 양쪽에서 비교한다. 결과가 좋아도 즉시 모델 가중치·스로틀 하한을 바꾸거나 V3 자료를 학습 데이터로 승인하지 않는다. 주행·지도 모델, 9개 주기능/30개 세부기능, 실차 승인은 별도 단계다.
