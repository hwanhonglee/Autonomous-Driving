# Portable E2E 학습·운용 가이드

> 기준일: 2026-09-03
> 이 문서는 현재 저장소에 실제로 존재하는 기능과 앞으로 실행할 절차를 구분한다.
> 아직 학습된 checkpoint도, 입증된 주행 성능도, 실차 제어 승인도 없다.

## 1. 지금 가능한 것

현재 `portable_e2e`에는 다음 코드가 구현되어 있다.

1. CARLA와 실차 자료를 같은 형식으로 표현하는 `common_10hz_v1` 데이터 계약
2. 6-camera 순서, timestamp, calibration/TF, trajectory, split 누수와 image SHA-256을
   검사하는 validator
3. 검증된 manifest를 framework-neutral example로 읽는 loader
4. 아무 ML package도 설치하지 않고 실행하는 CPU control-flow smoke
5. 여섯 JPEG, calibration, 최근 1초 ego history와 route를 읽는 PyTorch dataset
6. 6개의 6.4초 `(x, y, speed)` 후보를 출력하는 1,053,278-parameter baseline
7. best-of-K trajectory loss, 실제 backprop/optimizer, checkpoint와 exact resume
8. `val`/`test` 전용 open-loop evaluator, 차량 중심 trajectory PNG, 공정성 검사가 있는
   report A/B comparison

`portable_e2e.control_flow_smoke`는 실제 영상을 열지 않고 PyTorch, CUDA, ROS 2, CARLA를
import하지 않는다. `portable_e2e.train`은 반대로 실제 JPEG를 decode하고 신경망을
학습한다. 두 프로그램은 서로 대체 관계가 아니다.

현재 **없는 것**도 분명하다.

- 실제 데이터로 학습을 끝낸 checkpoint
- 장애물 회피·차선 변경·정지·ACC 성능이 검증된 multi-task model
- Autoware runtime inference adapter와 safety selector
- CARLA closed-loop 합격 결과 또는 target PC의 10 Hz latency 결과
- 실차 replay/shadow/폐쇄 시험장 승인

따라서 지금 생성되는 `.pt`는 연구용 checkpoint일 뿐 actuator 명령으로 사용하면 안 된다.
모델 출력도 throttle/brake/steering이 아니라 후보 trajectory다.

## 2. 로컬 PC, Git 저장소, 원격 학습 서버의 역할

코드는 로컬에서 수정·검사하고 Git 원격 저장소를 통해 학습 서버로 전달한다. 대용량
데이터와 학습 결과는 Git에 넣지 않는다. 공개 문서에는 서버 주소, 사용자명, 개인 key
경로 또는 계정별 절대 경로를 기록하지 않는다.

```text
로컬 개발 PC: $REPO_ROOT
        │  test → commit → push
        ▼
Git 원격 저장소: $REPOSITORY_URL
branch: autoware-e2e/v1.9.0-vad-carla
        │  pull --ff-only
        ▼
원격 code: $REPO_ROOT

원격 data: $PORTABLE_E2E_ROOT/datasets
원격 runs: $PORTABLE_E2E_ROOT/runs
원격 venv: $PORTABLE_E2E_ROOT/venvs/py312
```

Autoware workspace 형태는 `autoware_e2e/src/...`가 맞다. 학습 코어는 저장소의
`portable_e2e/`이며 학습만 할 때 `colcon build`는 필요 없다. ROS 2 adapter를 Autoware에
통합할 때만 해당 workspace를 별도로 빌드한다.

### 2.1 처음 clone하는 PC

Git LFS의 과거 PNG/GIF/MCAP을 받지 않고 코드만 clone하려면 다음과 같이 실행한다.

```bash
export PORTABLE_E2E_ROOT="${PORTABLE_E2E_ROOT:-$HOME/portable_e2e}"
export REPO_ROOT="$PORTABLE_E2E_ROOT/autoware_e2e"
export REPOSITORY_URL='<repository-clone-url>'

mkdir -p "$PORTABLE_E2E_ROOT"
cd "$PORTABLE_E2E_ROOT"

git -c filter.lfs.smudge= \
  -c filter.lfs.process= \
  -c filter.lfs.required=false \
  clone --single-branch \
  --branch autoware-e2e/v1.9.0-vad-carla \
  "$REPOSITORY_URL" "$REPO_ROOT"

git -C "$REPO_ROOT" status --short --branch
```

`<repository-clone-url>`은 권한이 있는 HTTPS 또는 SSH clone URL로 바꾼다. 저장소가 이미
`$REPO_ROOT`에 있으면 다시 clone하지 않고 2.3절의 갱신 절차를 따른다.

### 2.2 로컬 변경을 검토하고 Git 원격 저장소에 보내기

이 작업은 로컬 저장소에서 한다. `git add .`로 대용량 dataset이나 관계없는 수정까지 묶지
말고, 검토한 code/document 경로만 명시한다. 아래 절차는 시작할 때 index가 비어 있지 않으면
중단한다. 기존 staged 변경을 자동으로 unstage하거나 덮어쓰지 않는다.

```bash
export REPO_ROOT="${REPO_ROOT:-$(git rev-parse --show-toplevel)}"
cd "$REPO_ROOT"
git status --short --branch
git diff --check
git diff

# 이전 작업에서 staged된 항목이 하나라도 있으면 먼저 소유자와 내용을 확인한다.
if ! git diff --cached --quiet --; then
  printf 'index is not empty; commit or intentionally unstage it first\n' >&2
  git diff --cached --name-status
  exit 1
fi

# 예시다. 실제로 검토한 파일만 배열에 한 줄씩 추가한다.
reviewed_paths=(
  portable_e2e/contract.py
  tests/test_portable_e2e_contract.py
)
git add -- "${reviewed_paths[@]}"
git diff --cached --name-status
git diff --cached --check
git diff --cached
git commit -m 'feat(e2e): describe the reviewed change here'
git push origin autoware-e2e/v1.9.0-vad-carla
```

commit 전에 해당 변경의 unit test와 smoke가 통과했는지 확인한다. `datasets/`, `runs/`,
checkpoint와 인증 정보는 staging하지 않는다.

### 2.3 기존 원격 저장소 갱신

로컬의 검증·commit·push가 끝난 뒤에만 원격에서 실행한다.

```bash
export PORTABLE_E2E_ROOT="${PORTABLE_E2E_ROOT:-$HOME/portable_e2e}"
export REPO_ROOT="$PORTABLE_E2E_ROOT/autoware_e2e"

git -C "$REPO_ROOT" status --short --branch
git -C "$REPO_ROOT" pull --ff-only
git -C "$REPO_ROOT" rev-parse HEAD
```

첫 명령에 수정 파일이 나오면 pull하지 말고 원인을 먼저 확인한다. 원격에서
`git reset --hard`, 강제 checkout 또는 `git clean`으로 다른 작업을 지우지 않는다.

## 3. 2026-09-03에 확인된 학습 환경 상태

프로젝트 전용 Python 환경은 다음 상태다.

- Python: 3.12.3
- 위치: `$PORTABLE_E2E_ROOT/venvs/py312`
- package: `pip 24.0`만 존재
- PyTorch, NumPy, Pillow: **아직 설치하지 않음**
- 확인 과정에서 시스템 Python, Conda base, CUDA driver 또는 다른 프로젝트 환경에 설치한
  항목: 없음

공유 GPU의 점유 상태는 계속 바뀌므로 문서에 특정 사용자·프로세스·메모리 관측값을 남기지
않는다. 실행 직전에 9절의 query로 다시 확인하고, 다른 작업을 종료하거나 reset하지 않는다.

### 3.1 전송을 마친 과거 CARLA 자료

```text
$PORTABLE_E2E_ROOT/
└── datasets/legacy/carla_2026-08-31/source_tree
```

- 검증된 raw/export pair: 11쌍
- regular file: 7,805개
- regular-file payload: 646,642,112 bytes
- 전송 후 `rsync -acn --itemize-changes` 차이: 없음
- legacy export sample: 1,129개
- 원본 camera cadence: 4 Hz 또는 5 Hz

이 자료는 전송이 정확하다는 것만 확인되었다. `common_10hz_v1` planning 자격 자료가 아니며,
현재 목적은 legacy adapter와 parser/schema smoke다. 같은 JPG를 반복하거나 timestamp만
바꾸어 10 Hz로 표시하지 않는다.

### 3.2 다운로드·검증을 마친 Bench2Drive Mini

```text
$PORTABLE_E2E_ROOT/tmp/downloads/
└── bench2drive/legacy-mini-10-research-only/archives
```

공식 legacy Mini manifest의 archive 10/10개를 size와 SHA-256으로 검증했다. 아직 압축을
풀거나 common10으로 변환하거나 학습하지 않았다. 라이선스 표기가 배포 위치에 따라
일치하지 않으므로 현재는 **research/parser smoke 전용**이다. 차량용·상업용 데이터나
재배포 가능한 산출물로 간주하지 않는다. 세부 source와 검증 명령은
[데이터셋 조사 문서](portable-e2e-datasets.md)를 따른다.

### 3.3 원본만 격리한 nuScenes v1.0-mini

```text
$PORTABLE_E2E_ROOT/datasets/raw/nuscenes/v1.0-mini/
└── research-only-pending-terms-review/v1.0-mini.tgz
```

- 공식 익명 tutorial URL에서 받은 byte: **4,167,696,325**
- SHA-256: `943037abbb3b26b3070dc76504a43eb440503b00baf9ac2f1538d9c03fc9298f`
- streaming tar audit: 31,252 entries, regular file 31,224개, directory 28개
- symlink/hardlink/기타 entry, 위험 경로와 중복 이름: 모두 0
- 상태: **archive intact, 미해제·미변환·미학습**

다운로드 허용과 실차·상업용 학습 권리는 같은 말이 아니다. 그래서 약관 검토가 끝날 때까지
research-only 경로에 두며 Git에 넣거나 재배포하지 않는다. CAN bus expansion도 아직 받지
않았으므로 route까지 준비됐다고 표현하지 않는다.

### 3.4 nuPlan v1.1 mini 제한 subset 원본 반입

```text
$PORTABLE_E2E_ROOT/datasets/raw/nuplan/v1.1-mini/
└── research-only-pending-terms-review
    ├── nuplan-v1.1_mini.zip
    ├── nuplan-maps-v1.0.zip
    └── nuplan-v1.1_mini_camera_0.zip  # 분할 다운로드 진행 중
```

| archive | official object byte | 현재 검증 상태 |
|---|---:|---|
| mini DB | 8,550,100,030 | exact byte + local SHA-256 기록 + 전체 ZIP payload CRC PASS, 미해제 |
| map v1.0 | 971,557,640 | exact byte + local SHA-256 기록 + 전체 ZIP payload CRC PASS, 미해제 |
| mini camera group 0 | 52,219,710,368 | 분할 다운로드 진행 중; final archive 검증 전 |

계획 압축 합계는 61,741,368,038 bytes다. 완료된 DB와 map은 원본을 추출하지 않고 모든
member payload를 읽어 CRC를 확인했다. SHA-256은 공개 official checksum과 대조한 값이
아니라 local provenance 기록이다. camera group 0도 최종 조립 후 exact byte, local SHA-256,
전체 payload CRC를 모두 통과해야 완료로 표시한다.

이 3-file 구성은 완전한 nuPlan devkit dataset이 아니라 7개 log의 custom camera adapter
smoke용 제한 subset이다. 최종 archive 검사를 통과해도 약관·용도 승인, 안전한 압축 해제,
DB↔camera join, common10 변환과 학습은 별도 단계다. nuPlan dataset terms는 devkit code의
Apache-2.0 license와 별개이며, 익명 download 경로가 이용조건을 면제하지 않는다.

## 4. 매 SSH 접속 때 하는 안전한 환경 설정

아래 설정은 현재 terminal에만 적용된다. `.bashrc`, Conda base 또는 시스템 CUDA를
수정하지 않는다.

```bash
export PORTABLE_E2E_ROOT="${PORTABLE_E2E_ROOT:-$HOME/portable_e2e}"
export REPO_ROOT="$PORTABLE_E2E_ROOT/autoware_e2e"

source "$PORTABLE_E2E_ROOT/venvs/py312/bin/activate"
export PIP_CACHE_DIR="$PORTABLE_E2E_ROOT/cache/pip"
export PYTHONDONTWRITEBYTECODE=1
export CUDA_VISIBLE_DEVICES=''
export CUBLAS_WORKSPACE_CONFIG=:4096:8

cd "$REPO_ROOT"
which python
python --version
python -m pip list
git status --short --branch
```

정상이라면 Python 경로가 `.../venvs/py312/bin/python`이다. `CUDA_VISIBLE_DEVICES=''`는
이 terminal의 Python에서 GPU를 숨길 뿐, 다른 사람의 process를 정지하거나 VRAM을
해제하지 않는다.

## 5. package 설치 전 실행하는 CPU 검사

저장소 root에서 실행한다.

```bash
: "${PORTABLE_E2E_ROOT:?4절의 환경 설정을 먼저 실행하세요}"

CUDA_VISIBLE_DEVICES='' \
PYTHONDONTWRITEBYTECODE=1 \
python -m portable_e2e.control_flow_smoke

python -m portable_e2e.validate --help
```

정상 smoke의 핵심 상태는 다음과 같다.

```json
{
  "status": "PIPELINE_CONTRACT_PASS",
  "production_model_created": false,
  "images_decoded": false,
  "gpu_used": false,
  "optimizer_is_toy": true
}
```

dummy checkpoint와 재개를 확인할 때는 `.dummy-smoke.json`을 실제 `.pt`와 구분한다.

```bash
dummy_run="$PORTABLE_E2E_ROOT/runs/control_flow"

python -m portable_e2e.control_flow_smoke \
  --steps 40 \
  --stop-at-step 7 \
  --checkpoint "$dummy_run/checkpoint.dummy-smoke.json"

python -m portable_e2e.control_flow_smoke \
  --steps 40 \
  --checkpoint "$dummy_run/checkpoint.dummy-smoke.json" \
  --resume
```

기존 dummy file은 기본적으로 덮어쓰지 않는다. 새로 시작하려면 다른 경로를 쓰고, 정말
같은 dummy file을 교체하려는 경우에만 `--overwrite-dummy-checkpoint`를 사용한다.

## 6. 학습에 넣을 수 있는 데이터 조건

학습 root는 다음과 같은 완성된 dataset이어야 한다.

```text
<dataset_root>/
├── dataset.json
├── rigs/<rig_id>.json
└── episodes/<episode_id>/
    ├── episode.json
    ├── source_manifest.json
    ├── collection_config.json
    ├── route_geometry.json
    ├── samples.jsonl
    └── images/
        ├── CAM_FRONT/
        ├── CAM_BACK/
        ├── CAM_FRONT_LEFT/
        ├── CAM_BACK_LEFT/
        ├── CAM_FRONT_RIGHT/
        └── CAM_BACK_RIGHT/
```

학습 전에는 image SHA-256까지 포함한 planning 검사를 통과시킨다.

```bash
dataset_root="$PORTABLE_E2E_ROOT/datasets/prepared/<dataset_id>"
validation_report="$PORTABLE_E2E_ROOT/runs/dataset-validation/<dataset_id>-planning-v1.json"

python -m portable_e2e.validate "$dataset_root" \
  --mode planning \
  --output "$validation_report"
```

실제 `portable_e2e.train`과 `portable_e2e.evaluate` CLI도 내부 검증 모드를 `planning`으로
고정한다. 낮은 단계의 `schema` 검증은 adapter 개발용 validator에서만 사용하며, 실제
학습·평가 CLI에는 이를 우회하는 옵션이 없다.

`--skip-image-sha256`는 선언된 digest와의 **비교만** 생략한다. JPEG 전체 bytes는 여전히
읽고 container/scan을 검사하므로 저-I/O 모드가 아니다. 그 결과는 image integrity가
`NOT_RUN`이므로 release 학습에 사용할 수 없다. 실제 trainer/evaluator는 image hash 검사를
끄는 옵션을 제공하지 않는다. validator도 같은 report를 기본적으로 덮어쓰지 않으므로 새
version 경로를 사용하고, 내용을 검토한 뒤 의도적으로 교체할 때만 `--replace-output`을 쓴다.

중요한 자격 조건은 다음과 같다.

- 실제 관측 rate가 nominal 10 Hz이고 유효 rate가 최소 9.5 Hz일 것
- 여섯 카메라의 고정 순서, 실제 rig K/D/TF와 timestamp가 있을 것
- 최근 ego state와 0.1초 간격 64-point future label이 인과적으로 생성될 것
- 연속 drive/route/site/day가 `train`, `val`, `test`에 걸쳐 섞이지 않을 것
- `train`과 `val`/`test`에 서로 다른 split fingerprint가 있을 것
- CARLA와 real provenance를 유지하고 domain별 metric을 따로 낼 수 있을 것

4/5 Hz legacy 자료와 아직 변환하지 않은 Bench2Drive archive를 `dataset_root`로 직접 넣으면
안 된다.

## 7. 내일 제안할 전용 venv 설치

이 절의 명령은 **오늘 실행한 기록이 아니라, GPU 사용 전 검토할 제안**이다. 사용자 승인
후 프로젝트 전용 venv 안에서만 실행한다. `PIP_REQUIRE_VIRTUALENV=true` 때문에 가상환경이
활성화되지 않았으면 pip가 중단한다.

```bash
export PORTABLE_E2E_ROOT="${PORTABLE_E2E_ROOT:-$HOME/portable_e2e}"
export REPO_ROOT="$PORTABLE_E2E_ROOT/autoware_e2e"

source "$PORTABLE_E2E_ROOT/venvs/py312/bin/activate"
export PIP_CACHE_DIR="$PORTABLE_E2E_ROOT/cache/pip"
export PIP_REQUIRE_VIRTUALENV=true
export CUDA_VISIBLE_DEVICES=''
export CUBLAS_WORKSPACE_CONFIG=:4096:8
cd "$REPO_ROOT"

python -m pip install \
  --index-url https://download.pytorch.org/whl/cu130 \
  'torch==2.13.0'

python -m pip install \
  --requirement portable_e2e/config/training_py312.requirements.txt
```

두 번째 requirements의 현재 고정값은 `numpy==2.5.2`, `Pillow==12.3.0`이다. 시스템
`apt`, CUDA driver, Conda base 또는 다른 프로젝트의 venv는 수정하지 않는다.

설치 직후에도 GPU는 숨긴 상태로 version과 CLI import만 확인한다.

```bash
CUDA_VISIBLE_DEVICES='' python - <<'PY'
import numpy
import PIL
import torch

print("torch", torch.__version__, "torch CUDA", torch.version.cuda)
print("numpy", numpy.__version__)
print("Pillow", PIL.__version__)
print("CUDA visible to this check", torch.cuda.is_available())
PY

CUDA_VISIBLE_DEVICES='' python -m portable_e2e.train --help
CUDA_VISIBLE_DEVICES='' python -m portable_e2e.evaluate --help
CUDA_VISIBLE_DEVICES='' python -m portable_e2e.compare --help
```

현재 checkpoint loader는 안전한 `torch.load(..., weights_only=True)` 지원을 요구한다.
제안한 PyTorch version보다 오래된 package를 임의로 사용하면 resume/evaluate가 거부될 수
있다.

## 8. 실제 PyTorch CPU one-step을 먼저 실행하기

7절 설치와 6절 planning validation이 모두 끝난 뒤에만 실행한다. `<dataset_id>`와
`<run_id>`는 실제 값으로 바꾸며, `run_dir`은 **아직 존재하지 않는 새 경로**를 사용한다.

```bash
dataset_root="$PORTABLE_E2E_ROOT/datasets/prepared/<dataset_id>"
cpu_run="$PORTABLE_E2E_ROOT/runs/<run_id>-cpu-one-step"

CUDA_VISIBLE_DEVICES='' python -m portable_e2e.train "$dataset_root" \
  --run-dir "$cpu_run" \
  --split train \
  --device cpu \
  --limit-samples 4 \
  --batch-size 1 \
  --num-workers 0 \
  --max-steps 1 \
  --checkpoint-interval 1
```

성공 시 다음 파일을 확인한다.

```text
<cpu_run>/run.json
<cpu_run>/metrics.jsonl
<cpu_run>/checkpoints/latest.pt
```

one-step 성공은 decode, forward, loss, backward, optimizer와 checkpoint write가 연결됐다는
뜻뿐이다. loss 수치가 주행 성능을 증명하지 않는다.

같은 corpus의 별도 `val` split을 한 sample만 평가한다. output directory도 새 경로여야 한다.

```bash
cpu_eval="$PORTABLE_E2E_ROOT/runs/<run_id>-cpu-eval-val"

CUDA_VISIBLE_DEVICES='' python -m portable_e2e.evaluate "$dataset_root" \
  --checkpoint "$cpu_run/checkpoints/latest.pt" \
  --output-dir "$cpu_eval" \
  --split val \
  --device cpu \
  --limit-samples 1 \
  --batch-size 1 \
  --num-workers 0 \
  --render-count 1
```

`train` sample을 평가용으로 다시 쓰면 evaluator가 거부한다. `metrics.json`과
`trajectories/*.png`가 생겨도 이는 open-loop random/one-step 결과이며 성능 claim이 아니다.

## 9. GPU가 실제로 빈 뒤 한 번만 짧게 실행하기

먼저 별도 terminal에서 GPU 상태를 확인한다.

```bash
nvidia-smi --query-gpu=index,name,memory.total,memory.used,utilization.gpu \
  --format=csv
nvidia-smi --query-compute-apps=gpu_uuid,pid,process_name,used_memory \
  --format=csv
```

선택할 physical GPU에 다른 compute process가 없고 메모리와 utilization이 안정적으로
비어 있는지 다시 확인한다. process가 보이면 기다린다. PID 종료, `nvidia-smi --gpu-reset`,
reboot는 하지 않는다.

아래 `gpu_index=0`은 예시다. **확인한 idle physical index로만** 바꾼다.
`CUDA_VISIBLE_DEVICES`로 한 장만 노출했기 때문에 Python 안에서는 그 카드가 `cuda:0`이다.

```bash
gpu_index=0
dataset_root="$PORTABLE_E2E_ROOT/datasets/prepared/<dataset_id>"
gpu_run="$PORTABLE_E2E_ROOT/runs/<run_id>-gpu-one-step"

CUDA_VISIBLE_DEVICES="$gpu_index" python -m portable_e2e.train "$dataset_root" \
  --run-dir "$gpu_run" \
  --split train \
  --device cuda:0 \
  --limit-samples 4 \
  --batch-size 1 \
  --num-workers 0 \
  --max-steps 1 \
  --checkpoint-interval 1
```

GPU one-step이 끝난 뒤 같은 logical device에서 별도 `val` sample을 한 번 평가한다.

```bash
gpu_eval="$PORTABLE_E2E_ROOT/runs/<run_id>-gpu-eval-val"

CUDA_VISIBLE_DEVICES="$gpu_index" python -m portable_e2e.evaluate "$dataset_root" \
  --checkpoint "$gpu_run/checkpoints/latest.pt" \
  --output-dir "$gpu_eval" \
  --split val \
  --device cuda:0 \
  --limit-samples 1 \
  --batch-size 1 \
  --num-workers 0 \
  --render-count 1
```

이 두 명령이 통과하기 전에는 batch를 키우거나 긴 학습을 시작하지 않는다. 다음은
32~128개 고정 sample overfit, 고정 validation, 전체 baseline 순이다.

## 10. 중단 재개와 A/B 비교

checkpoint에는 model/train/loss config, train split fingerprint, corpus fingerprint,
train episode ID, runtime ABI와 device ABI가 기록된다. exact resume은 기존 run과 같은
Python/package/device를 요구하고 `max_steps`만 늘릴 수 있다. CUDA 실행에서는 7절에서 설정한
`CUBLAS_WORKSPACE_CONFIG=:4096:8`도 유지한다.

```bash
CUDA_VISIBLE_DEVICES="$gpu_index" python -m portable_e2e.train "$dataset_root" \
  --run-dir "$gpu_run" \
  --split train \
  --device cuda:0 \
  --limit-samples 4 \
  --batch-size 1 \
  --num-workers 0 \
  --max-steps 5 \
  --checkpoint-interval 1 \
  --resume
```

one-step run을 재개할 때는 `batch-size`, `limit-samples`, seed, optimizer 설정과 device
ABI가 처음 run과 정확히 같아야 하며 `max-steps`만 늘릴 수 있다. 32~128 sample overfit은
기존 one-step run을 변형하지 말고 새 run ID와 처음부터 고정한 config로 시작한다.

동일한 `val` split, sample 수, batch size, device, runtime/hardware로 생성한 두 평가만
비교할 수 있다.

```bash
python -m portable_e2e.compare \
  "$PORTABLE_E2E_ROOT/runs/model-a-eval/metrics.json" \
  "$PORTABLE_E2E_ROOT/runs/model-b-eval/metrics.json" \
  --output-json "$PORTABLE_E2E_ROOT/runs/ab-comparison/comparison.json" \
  --output-markdown "$PORTABLE_E2E_ROOT/runs/ab-comparison/comparison.md"
```

comparison tool은 dataset/corpus fingerprint, split, sample count, batch size, device,
runtime/hardware나 metric set이 다르면 공정하지 않다고 거부한다. evaluator는 본 계측 전에
첫 batch 1개를 `num_workers=0`으로 warm-up하고 그 시간은 forward latency에서 제외한다.
A/B 비교에서는 warm-up batch/sample 수까지 같아야 하지만 실제 warm-up 소요 시간은 같을
필요가 없다. runtime에는 PyTorch intra-op/inter-op thread 수와 OMP/MKL thread 환경도
기록되므로 이 조건까지 동일해야 한다.

여기서 산출한 latency는 공유 서버의 CPU/GPU 자원이나 OS/GPU scheduler를 예약하지 않은
**참고값**이다. 전용 자원, 고정 power/clock, 반복 run과 p50/p95/p99를 갖춘 target PC
benchmark를 대신하지 않으며, 이 표도 closed-loop 또는 차량 제어 승인이 아니다.

## 11. 실제 데이터 받기와 자체 자료 전송

실제 환경 학습을 위해 공개 데이터를 받는 방향은 맞지만, 먼저 사용 권한과 adapter
적합성을 확인한다.

1. nuScenes 사용 시점의 dataset 약관을 읽고 프로젝트 용도에 맞는지 확인한다.
2. 받은 `v1.0-mini` 원본으로 6-camera/calibration/timestamp adapter를 먼저 검증하고,
   route 검증이 필요할 때 version에 맞는 CAN bus expansion을 별도 승인해 받는다.
3. nuScenes mini는 **schema/adapter smoke 전용**이다. 12 Hz camera를 image 복제·합성
   없이 10 Hz로 thinning하면 약 166.7 ms gap이 생겨 현재 `common_10hz_v1`의
   p99 150 ms planning cadence gate를 통과하지 못한다. 실측 후 별도 profile/contract를
   검토·고정하기 전에는 정식 common10 학습 corpus에 넣지 않고, 20초 scene를 여러 개
   이어 붙여 30초라고 만들지 않는다.
4. nuPlan mini DB와 map의 raw archive 반입·전체 payload CRC 검사는 끝났고 camera group
   0은 분할 다운로드 중이다. 최종 archive 검사가 끝나도 바로 설치·압축 해제·학습하지
   않는다. Dataset Terms와 intended use를 기록한 뒤 별도 staging에서 group 0의 7개 log만
   mini DB와 fail-closed로 join하는 adapter smoke를 수행한다.
5. download URL, version, license/terms URL, 동의일, 원본 filename/size/SHA-256과 변환 code
   commit을 source manifest에 보존한다.
6. 원본은 `datasets/raw` 또는 `tmp/downloads`, 변환 중 자료는 `datasets/staging`, validator를
   통과한 immutable 결과만 `datasets/prepared`에 둔다.

인증 cookie, password, API token을 Git, shell history, 문서 또는 dataset manifest에 넣지
않는다. 다운로드 권한이 있어도 원본·변환본·checkpoint 재배포 권한이 자동으로 생기는 것은
아니다. 상세 비교와 공식 링크는 [데이터셋 조사 문서](portable-e2e-datasets.md)에 있다.

로컬에서 자체 수집 자료를 서버 staging으로 보낼 때는 기존 폴더를 재사용하지 않는다.
새 목적지로 전송한 뒤 양쪽 regular file의 상대 경로·byte·SHA-256 manifest가 정확히 같은지
검사하고, 마지막 `--delete --dry-run`도 아무것도 출력하지 않아야 한다. 이 검사는 빠진 파일뿐
아니라 서버에만 남은 찌꺼기도 검출한다. source나 destination 안에 symlink, FIFO, socket,
device 같은 항목이 있으면 fail-closed한다. 서버 주소, 사용자명과 key 경로는 공개 저장소에
쓰지 않고 로컬 `~/.ssh/config`의 alias로 관리한다.

```sshconfig
Host training-server
    HostName <server-address>
    User <server-user>
    Port <ssh-port>
    IdentityFile <private-key-path>
    IdentitiesOnly yes
```

config를 저장하고 권한을 `chmod 600 ~/.ssh/config`으로 제한한 뒤, 다음 명령이 password를
묻지 않고 성공하는지 확인한다.

```bash
ssh -o BatchMode=yes training-server 'printf "SSH key login OK\n"'
```

그 다음 `LOCAL_DATASET`, 원격의 `REMOTE_PORTABLE_E2E_ROOT`, `DATASET_ID`를 실제 값으로
설정한다. `DATASET_ID`에는 영문 소문자, 숫자, 점, 밑줄, 하이픈만 사용한다. 예시는 새 전송
경로와 새 report만 허용하므로 같은 이름을 다시 실행하면 중단된다.

```bash
set -eu

REMOTE_ALIAS='training-server'
REPO_ROOT="${REPO_ROOT:-$(git rev-parse --show-toplevel)}"
: "${LOCAL_DATASET:?export LOCAL_DATASET=/absolute/path/to/dataset}"
: "${REMOTE_PORTABLE_E2E_ROOT:?export REMOTE_PORTABLE_E2E_ROOT=/absolute/remote/path/to/portable_e2e}"
DATASET_ID='my-drive-2026-09-03-v1'
REPORT_DIR="$REPO_ROOT/runs/transfer-validation"
LOCAL_REPORT="$REPORT_DIR/$DATASET_ID.local.json"
REMOTE_REPORT="$REPORT_DIR/$DATASET_ID.remote.json"

# 아래 명령에 안전하게 전달할 수 있는 절대 경로만 허용한다.
case "$REMOTE_PORTABLE_E2E_ROOT" in
  /*) ;;
  *) printf 'remote root is not absolute\n' >&2; exit 2 ;;
esac
case "$REMOTE_PORTABLE_E2E_ROOT" in
  *[!A-Za-z0-9_./-]*) printf 'remote root has unsupported characters\n' >&2; exit 2 ;;
esac

REMOTE_REPO_ROOT="$REMOTE_PORTABLE_E2E_ROOT/autoware_e2e"
REMOTE_DATASET="$REMOTE_PORTABLE_E2E_ROOT/datasets/staging/$DATASET_ID"

case "$DATASET_ID" in
  ''|*[!a-z0-9._-]*) printf 'invalid DATASET_ID: %s\n' "$DATASET_ID" >&2; exit 2 ;;
esac

test -d "$LOCAL_DATASET"
test ! -L "$LOCAL_DATASET"
mkdir -p "$REPORT_DIR"
test ! -e "$LOCAL_REPORT"
test ! -e "$REMOTE_REPORT"

cd "$REPO_ROOT"
CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
  python3 scripts/e2e/verify_tree_manifest.py "$LOCAL_DATASET" \
  > "$LOCAL_REPORT"

ssh "$REMOTE_ALIAS" \
  "test ! -e '$REMOTE_DATASET' && mkdir -p '$REMOTE_DATASET'"

rsync -rt --partial --info=progress2 \
  "$LOCAL_DATASET/" \
  "$REMOTE_ALIAS:$REMOTE_DATASET/"

ssh "$REMOTE_ALIAS" \
  "cd '$REMOTE_REPO_ROOT' && \
   CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
   python3 scripts/e2e/verify_tree_manifest.py '$REMOTE_DATASET'" \
  > "$REMOTE_REPORT"

python3 - "$LOCAL_REPORT" "$REMOTE_REPORT" <<'PY'
import json
import pathlib
import sys

local = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
remote = json.loads(pathlib.Path(sys.argv[2]).read_text(encoding="utf-8"))
keys = ("manifest_sha256", "file_count", "total_size_bytes")
if not local["valid"] or not remote["valid"]:
    raise SystemExit("tree manifest validation failed; inspect both JSON reports")
if any(local["source"][key] != remote["source"][key] for key in keys):
    raise SystemExit("source and remote tree manifests differ")
print("tree manifest comparison: PASS")
print("files:", local["source"]["file_count"])
print("bytes:", local["source"]["total_size_bytes"])
print("sha256:", local["source"]["manifest_sha256"])
PY

rsync -rcn --delete --itemize-changes \
  "$LOCAL_DATASET/" \
  "$REMOTE_ALIAS:$REMOTE_DATASET/"
```

마지막 명령에 출력이 있으면 PASS가 아니다. `--delete`가 포함돼도 `-n/--dry-run`이라 이
명령 자체는 서버 파일을 지우지 않는다. 원인을 확인하고 새 `DATASET_ID`로 처음부터 다시
전송한다. 검증 실패 폴더를 `prepared`로 이동하거나 학습에 넣지 않는다. remote shell
안에서는 원격 절대 경로만 보고, 로컬 경로가 원격에도 존재한다고 가정하지 않는다.

## 12. 실행 순서와 완료 기준

현재 다음 순서가 안전하다.

1. Bench2Drive Mini를 별도 staging에 풀고 parser/common10 변환 재현성만 검사
2. legacy 11쌍 adapter를 만들되 4/5 Hz `NOT_QUALIFIED`를 그대로 기록
3. 신규 CARLA 10 Hz straight/turn/stop 자료를 36.4초 이상 수집하고 planning validation
4. 격리된 nuScenes mini의 약관 확인 후 schema/real-adapter smoke
   (`common_10hz_v1` planning은 별도 profile/contract 전까지 불통과)
5. 내일 승인 후 전용 venv package 설치와 CPU one-step
6. GPU가 실제 idle일 때 단일 GPU one-step과 val one-sample 평가
7. 32~128 sample overfit → M1 baseline → 같은 조건의 model A/B
8. CARLA 30 km/h closed-loop → real replay/shadow → 별도 60 km/h gate

실차용이라고 부르려면 최소한 provenance가 고정된 train/val/test, unseen real replay,
Autoware adapter의 reject/fallback, independent safety gate, target PC 10 Hz latency, CARLA
closed-loop, 실차 shadow와 폐쇄 시험장 검증이 모두 필요하다. 공개도로 자동제어는 이 문서의
승인 범위가 아니다.
