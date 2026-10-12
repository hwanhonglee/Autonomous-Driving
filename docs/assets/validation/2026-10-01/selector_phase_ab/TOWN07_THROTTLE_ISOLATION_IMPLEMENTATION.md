# HH_261001 — Town07 저속 스로틀 분리 진단: 구현 상태

> **v1·v2 모두 실행 금지:** v1은 정지 준비가 0.05초뿐이다. v2는 3.5초 준비를 추가했지만, 실제 V3 시작 진행거리 `0.3181 m`를 절대 `≤0.25 m` 조건으로 잘못 거부한다. 두 계획·해시는 보존한다. [v2 결함 기록](TOWN07_THROTTLE_ISOLATION_V2_WITHDRAWN.md)을 확인하고 별도 v3만 검토한다.

이 문서는 [주행 품질 관문](DRIVING_NEXT_QUALITY_GATE.md)의 **학습 전 원인 분리 실험**을 실행 가능한 형태로 옮긴 상태를 기록한다. 주행 모델의 새 학습, 성능 개선, 9개 주기능 또는 실차 검증 결과가 아니다.

## 현재 상태

| 항목 | 상태 |
| --- | --- |
| 사전등록 | 8회 Town07 `16 m: A–B–B–A`, `25 m: A–B–B–A` 선언 완료 |
| 고정 변수 | A 스로틀 `0.01`, B `0.20`; 각 3초/20 Hz; 분기 속도 `2.35–2.55 m/s`; 분기 위치 목표에서 `+0.25 m` 이내 |
| 모델·카메라 | 호출/저장 없음; BasicAgent는 분기 전 접근에만 사용 |
| 안전 중단 | 충돌·차선 이벤트, 빨간 신호, 속도 `>2.7 m/s`, 경로 중심 편차 `>1.0 m`, 제어 ACK·프레임 불일치, 디스크 하한 |
| 실제 CARLA 주행 | **미실행**. 사전점검에서 디스크 보존선 부족으로 차단 |

계획 파일은 [`storage/experiments/validation/2026-10-01/town07_throttle_isolation_plan_v1.json`](../../../../../storage/experiments/validation/2026-10-01/town07_throttle_isolation_plan_v1.json)에 생성되었고 SHA256은 `89ef854b330949b1808850f7501cc3ebbd3f46f307b8095685b1115e782192af`이다. CARLA egg, Town07 OpenDRIVE, 원본 route, 로컬 실행 소스 및 CARLA BasicAgent 소스의 SHA가 이 계획에 포함된다. 선언 뒤 이들 파일을 고치면 검증이 실패하도록 되어 있다.

2026-10-01 사전점검의 실제 여유 공간은 `5,160,988,672` bytes, 요구치는 `5,502,926,848` bytes(5 GiB 보존선 + 128 MiB 시작 예산)로, 최소 `341,938,176` bytes 부족했다. 기존 실험 자료나 `/tmp`를 임의로 삭제하지 않았다. 새 실행 폴더도 만들지 않았으며 CARLA 서버·GPU·학습은 사용하지 않았다. CARLA를 나중에 실행하면 렌더링 GPU 사용은 별도로 확인해야 한다. 카메라가 0개라는 사실만으로 서버 GPU 무사용을 보장하지 않는다.

## 보존된 v1 명령 — 실행하지 말 것

아래 명령은 v1 선언을 재현하는 참고용이다. 실제 진단의 재개 대상은 별도 v2이다.

```bash
cd /home/a/autoware_e2e
python3 tools/workspace_python.py -m scripts.e2e.run_owned_town07_throttle_isolation_20261001 -- --mode preflight --trial 0 --plan-sha256 89ef854b330949b1808850f7501cc3ebbd3f46f307b8095685b1115e782192af
```

v1 실행은 권장하지 않는다. 향후 v2만 검토한다. A–B–B–A 순서를 바꾸거나 실패 회차를 조용히 재시도하지 않는 원칙은 동일하다.

실행이 가능해져도 주요 비교는 **원시 20 Hz `Δvx/Δt < −8 m/s²` 횟수, 3초 순 진행 거리, 첫 `vx≤0.05 m/s` 시각**이다. 고정 스로틀의 CARLA 제어 ACK는 실제 구동 토크 증거가 아니고, 비동기 이벤트 센서의 무지연 검출도 입증되지 않는다. 이 결과만으로 스로틀 하한을 주행 정책에 넣거나 V3를 학습 자료로 편입하지 않는다. A/B 양쪽 결과를 모두 보관하고, 원인 분리 이후에만 별도 모델/제어 A/B를 사전등록한다.

합성 테스트: `python3 tools/workspace_python.py -m pytest -- -q learning/tests/test_town07_throttle_isolation_20261001.py` → **15 passed**. 이 검사는 프로토콜·안전 중단·60프레임 집계·소유 순서를 점검하며 CARLA 물리 결과를 대신하지 않는다.
