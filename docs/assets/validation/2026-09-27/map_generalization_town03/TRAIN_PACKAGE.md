# Town03 지도 TRAIN 패키지 준비 결과

<!-- HH_260906 - Record the frozen split and owner-provenant Town03 packaging boundary. -->

**상태: 로컬 TRAIN 자료 포장·검증 완료, 모델 학습 미실행, 모델 채택 미승인.** 이 패키지는 CARLA BasicAgent가 실제 물리 주행으로 기록한 전방 RGB와 CARLA 의미 라벨을 지도 의미 분할 학습용으로 묶은 것이다. 학습된 모델이 차량을 주행한 자료가 아니며, 정밀지도 완성이나 실차 승인을 뜻하지 않는다.

| 분할 | 프레임 | 구성 | 검사 |
|---|---:|---|---|
| TRAIN | 913 | 기존 Town01 497장 + Town03 직진 232장 + 회전 184장 | 기존 497장의 RGB/정답 배열 prefix가 원본과 정확히 일치 |
| DEV | 118 | 기존 Town01 자료 그대로 | 원본 NPZ와 SHA-256 바이트 일치; 반복 열람한 개발 자료 |
| Town07 | 0 | TRAIN/DEV 어느 곳에도 포함하지 않음 | 향후 봉인된 단회 평가용 |

Town03 두 원본은 각 240장, 총 480장이다. 소유 실행의 exit 0, 경로 SHA, 소스 아카이브 SHA, CARLA ready/after-capture/stopped PASS, 충돌 0, live/replay 일치, cleanup 오류 0, 전방 RGB·의미 라벨 쌍 480개를 확인했다. 같은 Town03 좌표계 안에서만 0.25m 중복 제거를 적용했다. 직진 232장과 회전 184장의 보존 목록, 각 RGB/의미 라벨 SHA, 클래스 픽셀 합계는 [독립 감사](README.md)의 JSON과 일치한다. Town01·Town07 좌표와 Town03 XY 거리를 비교하지 않았다.

로컬 출력: `storage/experiments/training/2026-09-27/map_generalization_town03_train_package_v1/`.

| 파일 | SHA-256 |
|---|---|
| `manifest.json` | `e5f4ec6ab46f16f2f2167f4b453abe4bea829ac9bce7a1215a64d7a41444de0d` |
| `train.npz` | `1328dd608eb642cc8c4b0d9be9d07cb391639a6a67b426fe2bbdb703e300e440` |
| `dev.npz` | `417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3` |

기존 AI v2 `manifest.json` SHA는 `e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c`, 기존 TRAIN NPZ SHA는 `69dae433616cf27690310af9ff1923ef3b2e67641fa935e2a2f77116eea02ace`다. 새 DEV NPZ SHA는 기존과 같다. 패키저와 계약 단위 검사 **23개 통과**. 패키저 `--preflight-only`는 출력 폴더를 만들지 않는다.

재현 명령은 다음과 같다. 실행 시 출력 경로가 이미 있으면 덮어쓰지 않고 중단한다.

```bash
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' python3 tools/workspace_python.py \
  tools/e2e/prepare_map_expansion_ai.py --town03-generalization-train --preflight-only \
  storage/datasets/prepared/map_expanded_20260926_ai_v2 \
  storage/experiments/training/2026-09-27/map_generalization_town03_straight_p03 \
  storage/experiments/training/2026-09-27/map_generalization_town03_right_p03 \
  storage/experiments/training/2026-09-27/map_generalization_town03_train_package_v1
```

다음 단계는 새 패키지와 갱신된 `map_expansion_contract.py`를 개인 PRO6000 작업 공간으로 전송하고, **물리 GPU 0만 사용**하는 개인 venv에서 CPU 사전검사를 통과한 뒤 사전 등록한 단일 2,000-step B 후보를 학습하는 것이다. 패키지 설치, GPU 1 사용, 자동 재학습·체크포인트 승격은 하지 않는다. 학습 후에도 Town07 단회 평가 게이트를 통과하기 전까지 모델 성능 개선은 주장할 수 없다.
