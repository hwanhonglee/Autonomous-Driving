# 남은 두 항목 확인 — 차선 ID 원인 확인, 차체 외곽은 실측 자료 필요

후속 결과: [Z — Prius 차체 실측 및 외곽 표본 검사 완료](../17_prius_geometry_z/README.md).

<!-- HH_260906 - Record lane correspondence evidence without inventing vehicle dimensions. -->

## 1. 교차로 차선 ID 불일치

가장 가까운 차선을 자동 선택하지 않고, 원본의 road_id/lane_id를 직접 지정해 같은 위치를 찾았습니다.

| 지도 | 재검사 지점 | 원본 차선 존재 | 원본 위치와 최대 차이 |
|---|---:|---:|---:|
| Town01 | 6 | 6/6 | 0.2213 mm |
| C_track | 7 | 7/7 | 0.0772 mm |

원본 차선이 사라지거나 다른 곳으로 이동한 현상은 해당 13지점에서 발견하지 못했습니다. 교차로의 겹치는 연결 차선/접속부에서 최근접 차선 선택이 달라지는 것으로 설명됩니다. 이후 차체 검사에서는 최근접 차선 하나를 무조건 믿지 말고 원본의 차선 연결 관계를 사용해야 합니다.

이 결과는 해당 지점의 위치 일치 증거입니다. 수집 당시 전체 XODR 해시가 없으므로 지도 전체의 역사적 동일성을 인증한 것은 아닙니다. 검색은 도로 길이 방향 0.1 m 이하 간격의 탐색과 주변 0.0005 m 이하 간격의 재탐색으로 수행했습니다.

## 2. Prius 차체 외곽

확인한 로컬 artifacts의 JSON/JSONL에는 bounding box/vehicle extent 자료가 없었습니다. 수집 manifest에는 차량 종류와 휠베이스는 있으나, 실제 길이·폭·차체 중심의 base_link 상대 오프셋을 확정하기에 부족합니다. 인접 Autoware sample 차량 설정을 Prius의 실측값으로 대신 사용하지 않았습니다.

현재 로컬 리스닝 포트 확인에서도 기존 CARLA RPC 서버를 찾지 못했습니다. 이 확인 작업에서는 새 서버나 차량을 생성하지 않았습니다. 따라서 **차체 전체 차선 내 포함 여부는 미검증**입니다. 차선 안에 들어간다고 결론내릴 수 없습니다.

다음 측정은 별도 소유 CARLA 세션에서 같은 Prius의 `bounding_box.location`, `bounding_box.extent`, 회전 및 actor→base_link 변환을 기록하는 것입니다. 그 후 의도한 차선 경계에 대해 회전하는 차체 외곽을 검사해야 합니다. 새 시뮬레이터 실행과 차량 생성은 이번 읽기 전용 확인과 구분해 진행합니다.

## 실행과 원자료

- 학습, 설치, GPU 사용, 기존 프로세스 변경 없음. 모델 성능/기능 완료 수 변화 없음.
- 스크립트: `scripts/e2e/audit_lane_identity_y.py`
- 원자료: `artifacts/training/2026-09-22/lane_identity_y_v1/results.json` — 지도·입력·스크립트 SHA256과 13지점 상세 포함.
- CARLA 서버 없이 기존 Python 3.10 CARLA 패키지와 저장된 XODR만 사용했습니다.

```bash
PYTHONPATH=.:/home/a/carla/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg \
python3 scripts/e2e/audit_lane_identity_y.py \
  --input artifacts/training/2026-09-22/offline_map_x_v3/results.json \
  --output artifacts/training/2026-09-22/lane_identity_y_recheck.json
```

출력 파일은 새 경로를 사용해야 합니다. 원자료와 지도, 기존 의존성이 필요합니다.
