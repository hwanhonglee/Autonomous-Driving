# Town02 p00 A/B: 보관 체크포인트의 CPU 재추론

<!-- HH_260929 - Verify that archived weights reproduce nearly the same predictions across a different CPU runtime. -->

**결과:** 보관한 A/B 체크포인트 두 개를 로컬 **CPU**에서 다시 읽어 Town01 118장과 Town02 p02 232장을 각각 재추론했다. 네 조합의 700프레임에서 원격 GPU 실행의 저장 예측과 픽셀 일치율은 모두 **99.987% 이상**이었다. 이 검사는 체크포인트가 실제 저장 예측과 연결되는지를 더 강하게 확인한다. CPU의 PyTorch 1.8.0a0와 원격 GPU 실행은 수치 연산이 달라 경계 픽셀까지 완전 동일하지 않으므로 `100% 일치`라고 쓰지 않는다.

| 팔 / 평가 경로 | 프레임 | 다른 픽셀 / 전체 | 픽셀 일치율 | 클래스 IoU 차이 최대값 |
| --- | ---: | ---: | ---: | ---: |
| A / Town01 | 118 | 2,055 / 27,187,200 | 99.99244% | 0.02488%p |
| A / Town02 p02 | 232 | 6,490 / 53,452,800 | 99.98786% | 0.02603%p |
| B / Town01 | 118 | 1,169 / 27,187,200 | 99.99570% | 0.01571%p |
| B / Town02 p02 | 232 | 6,403 / 53,452,800 | 99.98802% | 0.01747%p |

[CPU 재추론 결과 JSON](../../../../../storage/experiments/validation/2026-09-29/map_town02_p00_checkpoint_cpu_v1.json)은 각 조합의 체크포인트/예측 SHA-256, 최악 프레임, 7클래스 IoU 차이를 모두 보관한다. JSON SHA-256은 `91f25ef8c2770e843ca73fd24c2707d0c7aa549b3ea713dc6d3f8ad6be3da38d`다. [스크립트](../../../../../tools/e2e/verify_map_town02_p00_checkpoint_cpu.py)는 데이터 manifest, 실행 요약, 보관 모델 소스, 체크포인트 두 개의 고정 SHA를 확인한 **뒤에만** 체크포인트를 읽는다. 패키지 네 분할 중 평가 두 분할의 NPZ 해시와 프레임 수를 확인하고, 새로운 모델 인스턴스로 추론한다. 99.5%는 큰 불일치를 잡기 위한 느슨한 **일관성 검사** 기준이지 지도 품질 합격선이 아니다.

로컬 재현 명령은 CPU만 사용하며 새 출력 파일명이 필요하다. 기존 JSON을 덮어쓰지 않는다.

```bash
cd /home/a/autoware_e2e
python3 tools/workspace_python.py tools/e2e/verify_map_town02_p00_checkpoint_cpu.py \
  --data storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1 \
  --run storage/experiments/training/2026-09-29/map_town02_p00_holdout_run_v1 \
  --output storage/experiments/validation/2026-09-29/map_town02_p00_checkpoint_cpu_reproduction.json
python3 tools/workspace_python.py -m pytest -- -q \
  learning/tests/test_verify_map_town02_p00_checkpoint_cpu.py
```

[단위 시험](../../../../../learning/tests/test_verify_map_town02_p00_checkpoint_cpu.py) **3 PASS**와 실제 700프레임 재추론을 확인했다. CPU 일치성은 [저장 예측의 독립 통계 감사](AUDIT.md)를 보강하지만 B의 Town01 차선·신호등 퇴행을 없애지 않는다. 사전 관문 실패, B 미채택, 독립 DEV 0곳, learned closed-loop 주행 승인 0/30은 그대로다. 이 화면과 수치는 Autoware 실제 주행이나 HD 지도 완성의 증거가 아니다.
