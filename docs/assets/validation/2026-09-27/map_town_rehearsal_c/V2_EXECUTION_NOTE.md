# 지도 후보 C 실행기 v2: CPU 환경 계약 수정

<!-- HH_260906 - Preserve the failed v1 receipt and use a fresh source/run path for one corrected execution. -->

**상태: v2 코드·합성 테스트 완료, 원격 전송·학습 미실행.** 후보 C의 [학습·평가 사전 기준](PLAN.md)은 그대로다. Town07 보류 평가의 이미지·라벨·예측·점수는 열지 않았다.

## 왜 v1이 멈췄는가

보존된 [v1 실행 로그](mapping_town_rehearsal_20260927_v1.launch.log)에는 `ValueError: K deterministic GPU environment differs`가 있다. 동결된 `r.check_run_environment`가 `CUDA_VISIBLE_DEVICES`에 물리 GPU0 UUID `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`를 요구하는데, v1 runner의 CPU `--validate-only` 분기는 이 값을 비웠다. [v1 종료 기록](mapping_town_rehearsal_20260927_v1.exit.json)은 `phase=cpu_validate`, `returncode=1`, `gpu0_preflight=null`, `optimization_run_count=0`, `automatic_promotion=false`다. 따라서 **v1에서 학습된 후보 C 체크포인트는 없다.** v1 로그·종료 기록과 원격 v1 경로는 삭제하거나 덮어쓰지 않는다.

## 변경 범위와 증거

- 새 `run_map_town_rehearsal_20260927_v2.py`는 CPU 검증과 이후 학습 모두에 위 UUID를 설정한다. CPU `--validate-only`는 학습기의 `gpu0_lease`·`preflight_gpu`·모델 초기화·최적화 이전에 종료한다. UUID 설정만으로 이 분기를 학습으로 간주하지 않는다. 별도 read-only `nvidia-smi`가 물리 GPU0 사용량/활용률 모두 0인지 확인해야 학습 단계로 넘어간다.
- 후보 C trainer SHA256 `e1808cb77df8bc7c28d5019eb46c2d40d7e756d2f45ce59d01b43d944ab04de2`는 **v1과 동일**하다. 913장 TRAIN/118장 DEV·부모/B 체크포인트·지원 소스 해시, Town01:Town03 배치 7:1, 2,000 step, seed, loss, optimizer, crop, Town07 평가 기준도 변경하지 않았다.
- v2 runner SHA256 `09933bcb398258ed32534b236bf1074b1778c2c1d2f01f04a7d0b3e775b2da5e`. 새 원격 경로는 `source_worktrees/map_town_rehearsal_20260927_v2`, `runs/diagnostics/map_town_rehearsal_20260927_v2`, `runs/diagnostics/mapping_town_rehearsal_20260927_v2.launch.log`, `runs/diagnostics/mapping_town_rehearsal_20260927_v2.exit.json`이다. 기존 v1 파일에 쓰지 않는다.
- 합성 테스트는 고정 경로·SHA 거부·CPU/GPU 단계 모두 GPU0 UUID·GPU 점유/실패 시 학습 0회·성공 시 학습 정확히 1회·재시도/자동 승격 없음에 통과했다. 이는 모델 성능 검증이 아니다.

## 원격 전송과 단회 실행 — 아직 실행하지 않음

아래 명령은 저장소 `/home/a/autoware_e2e`에서 실행하는 **검토용 예시**다. 개인 작업공간 이외에는 설치/기록하지 않으며 GPU1을 사용하지 않는다. 먼저 v2 runner와 C trainer의 로컬 SHA를 확인하고, 원격에서는 새 source 디렉터리만 생성한다.

```bash
sha256sum tools/e2e/train_map_town_rehearsal_c.py tools/e2e/run_map_town_rehearsal_20260927_v2.py
ssh -p 6000 -i /home/a/.ssh/pro6000_ed25519_v2 pro6000@220.90.5.14 'mkdir -p /home/pro6000/personal/hwanhong/portable_e2e/source_worktrees/map_town_rehearsal_20260927_v2'
rsync -av -e 'ssh -p 6000 -i /home/a/.ssh/pro6000_ed25519_v2' -- tools/e2e/train_map_town_rehearsal_c.py tools/e2e/run_map_town_rehearsal_20260927_v2.py tools/e2e/map_expansion_contract.py learning/portable_e2e/map_perception.py learning/portable_e2e/map_learning_balance.py tools/e2e/dual_recovery_objectives.py pro6000@220.90.5.14:/home/pro6000/personal/hwanhong/portable_e2e/source_worktrees/map_town_rehearsal_20260927_v2/
ssh -p 6000 -i /home/a/.ssh/pro6000_ed25519_v2 pro6000@220.90.5.14 'cd /home/pro6000/personal/hwanhong/portable_e2e && /home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/bin/python source_worktrees/map_town_rehearsal_20260927_v2/run_map_town_rehearsal_20260927_v2.py'
```

v2 실행기가 새 output/log/receipt 존재를 먼저 거부하고, 입력·소스 SHA와 부모/B 계약을 CPU에서 확인한다. 성공하더라도 C는 배포 승인 모델이 아니며, 우선 기존 Town01 DEV 보호 관문을 확인한다. 관문을 통과하기 전에는 Town07을 열지 않는다.
