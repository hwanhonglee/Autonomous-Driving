#!/usr/bin/env bash

# Read-only first command for a newly cloned Autoware E2E workspace.
#
# This script deliberately does not source env.sh: env.sh requires ROS 2 and
# the CARLA Python egg and therefore cannot explain a partially prepared clone.

set -u
set -o pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
expected_branch="${AUTOWARE_E2E_PREFLIGHT_EXPECTED_BRANCH:-autoware-e2e/v1.9.0-vad-carla}"
strict=false
carla_override=""
map_override=""

usage() {
  cat <<'EOF'
Usage: bash scripts/e2e/bootstrap_preflight.sh [options]

새 clone에서도 실행 가능한 읽기 전용 환경 진단입니다. 설치, 다운로드,
빌드, 파일 생성, 프로세스 종료를 수행하지 않습니다.

Options:
  --strict             BLOCK이 하나라도 있으면 종료 코드 1을 반환합니다.
  --carla-root PATH    검사할 CARLA 0.9.15 설치 경로를 지정합니다.
  --map PATH           검사할 Autoware full-map bundle 경로를 지정합니다.
  -h, --help           이 도움말을 표시합니다.

Environment equivalents:
  CARLA_ROOT, AUTOWARE_E2E_DATA_PATH, AUTOWARE_E2E_PREFLIGHT_MAP_PATH
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --strict)
      strict=true
      shift
      ;;
    --carla-root)
      if [[ $# -lt 2 || -z "$2" || "$2" == -* ]]; then
        echo "--carla-root에는 경로가 필요합니다." >&2
        exit 2
      fi
      carla_override="$2"
      shift 2
      ;;
    --map)
      if [[ $# -lt 2 || -z "$2" || "$2" == -* ]]; then
        echo "--map에는 경로가 필요합니다." >&2
        exit 2
      fi
      map_override="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '알 수 없는 옵션: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

passes=0
warnings=0
blockers=0

pass() {
  passes=$((passes + 1))
  printf '[PASS]  %-24s %s\n' "$1" "$2"
}

warn() {
  warnings=$((warnings + 1))
  printf '[WARN]  %-24s %s\n' "$1" "$2"
}

block() {
  blockers=$((blockers + 1))
  printf '[BLOCK] %-24s %s\n' "$1" "$2"
}

info() {
  printf '[INFO]  %-24s %s\n' "$1" "$2"
}

command_check() {
  local command_name="$1"
  local purpose="$2"
  if command -v "${command_name}" >/dev/null 2>&1; then
    pass "tool.${command_name}" "${purpose}"
  else
    block "tool.${command_name}" "없음 — ${purpose}에 필요"
  fi
}

optional_command_check() {
  local command_name="$1"
  local purpose="$2"
  if command -v "${command_name}" >/dev/null 2>&1; then
    pass "tool.${command_name}" "${purpose}"
  else
    warn "tool.${command_name}" "없음 — ${purpose} 기능은 사용할 수 없음"
  fi
}

python_module_check() {
  local module_name="$1"
  local purpose="$2"
  if PYTHONDONTWRITEBYTECODE=1 python3 -c \
    "import importlib.util,sys; sys.exit(0 if importlib.util.find_spec('${module_name}') else 1)" \
    >/dev/null 2>&1; then
    pass "python.${module_name}" "module 발견 — ${purpose}"
  else
    block "python.${module_name}" "module을 찾지 못함 — ${purpose}에 필요"
  fi
}

file_count=0
missing_file_count=0
check_required_files() {
  local label="$1"
  shift
  file_count=$#
  missing_file_count=0
  local path
  for path in "$@"; do
    if [[ ! -f "${path}" ]]; then
      missing_file_count=$((missing_file_count + 1))
    fi
  done
  if [[ ${missing_file_count} -eq 0 ]]; then
    pass "${label}" "${file_count}/${file_count} 파일 확인"
  else
    block "${label}" "$((file_count - missing_file_count))/${file_count} 파일 확인; ${missing_file_count}개 누락 또는 broken symlink"
  fi
}

printf 'Autoware E2E clone bootstrap preflight (READ-ONLY)\n'
printf 'workspace: %s\n' "${root}"
printf 'contract: Autoware 1.9.0 / ROS 2 Humble / CARLA 0.9.15 / amd64\n\n'

printf '%s\n' '1) Git checkout'
if git_dir="$(git -C "${root}" rev-parse --git-dir 2>/dev/null)"; then
  pass "git.repository" "Git checkout 확인 (${git_dir})"
  current_branch="$(git -C "${root}" branch --show-current 2>/dev/null || true)"
  if [[ "${current_branch}" == "${expected_branch}" ]]; then
    pass "git.branch" "${current_branch}"
  elif [[ -n "${current_branch}" ]]; then
    block "git.branch" "현재 ${current_branch}; 재현 기준은 ${expected_branch}"
  else
    block "git.branch" "detached HEAD; 재현 기준 branch는 ${expected_branch}"
  fi
  commit="$(git -C "${root}" rev-parse --short=12 HEAD 2>/dev/null || true)"
  info "git.commit" "${commit:-unknown}"
  worktree_status=""
  if worktree_status="$(
    timeout 10s env GIT_OPTIONAL_LOCKS=0 \
      git -C "${root}" status --porcelain --untracked-files=normal 2>&1
  )"; then
    if [[ -z "${worktree_status}" ]]; then
      pass "git.worktree" "tracked/untracked drift 없음"
    else
      worktree_change_count="$(wc -l <<<"${worktree_status}" | tr -d ' ')"
      block "git.worktree" "${worktree_change_count}개 변경; 표시 commit과 실행 byte가 다름"
      info "git.worktree_preview" "$(printf '%s\n' "${worktree_status}" | head -n 3 | tr '\n' ';')"
    fi
  else
    worktree_status_code=$?
    block "git.worktree" "status 검사 실패(exit=${worktree_status_code}); ${worktree_status//$'\n'/ }"
  fi

  if git -C "${root}" lfs version >/dev/null 2>&1; then
    pass "git.lfs" "Git LFS client 사용 가능"
  else
    warn "git.lfs" "Git LFS 없음; 발행 PNG/GIF/MCAP 실데이터를 받을 수 없음"
  fi
  lfs_sample_count=0
  lfs_pointer_samples=()
  for lfs_extension in png gif; do
    lfs_sample="$(git -C "${root}" ls-files "docs/assets/validation/2026-09-02-runtime-control-campaign-v1/**/*.${lfs_extension}" | head -n 1)"
    if [[ -n "${lfs_sample}" && -f "${root}/${lfs_sample}" ]]; then
      lfs_sample_count=$((lfs_sample_count + 1))
      if LC_ALL=C grep -aqm1 '^version https://git-lfs.github.com/spec/v1$' \
        "${root}/${lfs_sample}"; then
        lfs_pointer_samples+=("${lfs_sample}")
      fi
    fi
  done
  if (( ${#lfs_pointer_samples[@]} > 0 )); then
    warn "git.lfs_assets" "${#lfs_pointer_samples[@]}/${lfs_sample_count} 대표 PNG/GIF가 pointer임; 선택 pull 필요"
    info "git.lfs_pointer" "${lfs_pointer_samples[*]}"
  elif (( lfs_sample_count == 2 )); then
    pass "git.lfs_assets" "대표 PNG/GIF가 실제 파일로 checkout됨"
  else
    warn "git.lfs_assets" "대표 PNG/GIF sample을 모두 찾지 못함"
  fi
else
  block "git.repository" "${root}가 Git checkout이 아님"
fi

printf '\n%s\n' '2) Host support boundary'
kernel="$(uname -s 2>/dev/null || printf unknown)"
architecture="$(uname -m 2>/dev/null || printf unknown)"
os_id="unknown"
os_version="unknown"
os_pretty="unknown"
os_release_path="${AUTOWARE_E2E_PREFLIGHT_OS_RELEASE_PATH:-/etc/os-release}"
if [[ -r "${os_release_path}" ]]; then
  os_id="$(sed -n 's/^ID=//p' "${os_release_path}" | head -n 1 | tr -d '"')"
  os_version="$(sed -n 's/^VERSION_ID=//p' "${os_release_path}" | head -n 1 | tr -d '"')"
  os_pretty="$(sed -n 's/^PRETTY_NAME=//p' "${os_release_path}" | head -n 1 | tr -d '"')"
fi
if [[ "${kernel}" == "Linux" && "${os_id}" == "ubuntu" && "${os_version}" == "22.04" ]]; then
  pass "host.os" "${os_pretty}"
else
  block "host.os" "${os_pretty} (${kernel}); full stack 검증 범위는 native Ubuntu 22.04"
fi
if [[ "${architecture}" == "x86_64" ]]; then
  pass "host.arch" "x86_64/amd64"
else
  block "host.arch" "${architecture}; CARLA egg와 pinned spconv bundle은 amd64 전용"
fi

proc_version_path="${AUTOWARE_E2E_PREFLIGHT_PROC_VERSION_PATH:-/proc/version}"
if [[ -r "${proc_version_path}" ]] && grep -Eqi 'microsoft|wsl' "${proc_version_path}"; then
  block "host.wsl" "WSL 감지; 이 프로젝트의 CARLA/Vulkan 폐루프는 WSL에서 검증되지 않음"
fi
if [[ -e /.dockerenv ]] || grep -Eqs '(docker|containerd|kubepods)' /proc/1/cgroup 2>/dev/null; then
  warn "host.container" "container 감지; 이 branch에는 검증된 project image/compose/GPU·GUI wiring이 없음"
else
  pass "host.native" "일반 host 환경"
fi

cpu_count="$(getconf _NPROCESSORS_ONLN 2>/dev/null || printf 0)"
if [[ "${cpu_count}" =~ ^[0-9]+$ ]] && (( cpu_count >= 8 )); then
  pass "resource.cpu" "${cpu_count} logical CPUs (권장 하한 8)"
else
  warn "resource.cpu" "${cpu_count} logical CPUs; build와 simulation이 매우 느릴 수 있음"
fi

memory_kib=0
if [[ -r /proc/meminfo ]]; then
  memory_kib="$(awk '/^MemTotal:/ {print $2; exit}' /proc/meminfo)"
fi
memory_gib=$((memory_kib / 1024 / 1024))
if (( memory_gib >= 32 )); then
  pass "resource.ram" "${memory_gib} GiB (32 GiB 이상 권장)"
elif (( memory_gib >= 16 )); then
  warn "resource.ram" "${memory_gib} GiB; full build는 병렬도를 1로 낮춰야 할 수 있음"
else
  block "resource.ram" "${memory_gib} GiB; full build/주행에 부족할 가능성이 큼"
fi

disk_available_kib="$(df -Pk "${root}" 2>/dev/null | awk 'NR==2 {print $4}')"
if [[ "${disk_available_kib}" =~ ^[0-9]+$ ]]; then
  disk_available_gib=$((disk_available_kib / 1024 / 1024))
  if (( disk_available_gib >= 100 )); then
    pass "resource.disk" "${disk_available_gib} GiB free (초기 구성 권장 여유 100 GiB 이상)"
  elif (( disk_available_gib >= 60 )); then
    warn "resource.disk" "${disk_available_gib} GiB free; CARLA·vendor·build·맵을 합치면 빠듯함"
  else
    block "resource.disk" "${disk_available_gib} GiB free; 초기 구성에 최소 약 60 GiB 이상 필요"
  fi
else
  warn "resource.disk" "사용 가능 용량을 읽지 못함"
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  gpu_info="$(timeout 10s nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader 2>&1)"
  gpu_status=$?
  if [[ ${gpu_status} -eq 0 && -n "${gpu_info}" ]]; then
    pass "gpu.nvidia" "$(printf '%s' "${gpu_info}" | head -n 1)"
  elif [[ ${gpu_status} -eq 124 ]]; then
    block "gpu.nvidia" "nvidia-smi가 10초 안에 응답하지 않음; driver/NVML hang 가능"
  else
    block "gpu.nvidia" "nvidia-smi 실패 — ${gpu_info//$'\n'/ }"
  fi
else
  block "gpu.nvidia" "nvidia-smi 없음; TensorRT VAD와 CARLA renderer에는 NVIDIA GPU/driver 필요"
fi

printf '\n%s\n' '3) Host tools and ROS'
for entry in \
  'bash:프로젝트 shell wrapper 실행' \
  'git:source checkout/import' \
  'python3:분석·route·검증 도구' \
  'curl:고정 artifact 다운로드' \
  'sha256sum:artifact 무결성 검사' \
  'timeout:멈춘 host/GPU probe 제한' \
  'apt-get:고정 CUDA/TensorRT package 다운로드' \
  'apt-cache:NVIDIA package source 사전검사' \
  'dpkg-deb:workspace-local vendor package 압축 해제' \
  'vcs:autoware.repos source import' \
  'rosdep:ROS system dependency 해석' \
  'colcon:Autoware workspace build' \
  'cmake:C/C++ package 구성' \
  'make:C/C++ package build' \
  'pcl_transform_point_cloud:packaged Town PCD 좌표 변환'; do
  command_check "${entry%%:*}" "${entry#*:}"
done
for entry in \
  'numpy:route·map 수치 처리' \
  'scipy:route·map 공간 계산' \
  'open3d:pointcloud map 검사'; do
  python_module_check "${entry%%:*}" "${entry#*:}"
done

ros_setup="${AUTOWARE_E2E_PREFLIGHT_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ -f "${ros_setup}" ]]; then
  pass "ros.humble" "${ros_setup}"
else
  block "ros.humble" "${ros_setup} 없음"
fi
if [[ -d "${root}/ansible" ]]; then
  pass "installer.ansible" "legacy setup-dev-env.sh의 local collection 존재"
else
  warn "installer.ansible" "clone에는 ./ansible이 없어 setup-dev-env.sh를 신규 설치기로 실행할 수 없음"
fi

printf '\n%s\n' '4) Visual evidence prerequisites (optional for headless runs)'
for entry in \
  'ffmpeg:RViz 화면 녹화와 GIF encode' \
  'ffprobe:녹화 결과 frame 검증' \
  'xdpyinfo:X11 전체 화면 크기 확인' \
  'xprop:소유 RViz window 검증' \
  'xwininfo:소유 RViz window geometry 검증'; do
  optional_command_check "${entry%%:*}" "${entry#*:}"
done
display_info=""
if [[ -n "${DISPLAY:-}" ]]; then
  display_info="$(timeout 5s xdpyinfo -display "${DISPLAY}" 2>/dev/null || true)"
fi
if [[ -n "${display_info}" ]]; then
  pass "visual.display" "DISPLAY=${DISPLAY} 연결 가능"
  display_dimensions="$(
    awk '/dimensions:/ {print $2; exit}' <<< "${display_info}"
  )"
  if [[ "${display_dimensions}" =~ ^([0-9]+)x([0-9]+)$ ]]; then
    display_width="${BASH_REMATCH[1]}"
    display_height="${BASH_REMATCH[2]}"
    if (( display_width >= 1280 && display_width <= 1920 &&
          display_height >= 720 && display_height <= 1080 )); then
      pass "visual.screen_geometry" "${display_dimensions}; screen 사전조건 범위, 실제 owned window는 trial에서 재검증"
    else
      warn "visual.screen_geometry" "X11 screen ${display_dimensions}; --capture-desktop은 실제 RViz 창 1280x720..1920x1080 필요"
    fi
  else
    warn "visual.screen_geometry" "X11 screen dimensions를 읽지 못함"
  fi
else
  warn "visual.display" "X11 DISPLAY가 없거나 5초 안에 응답하지 않음; --visualize/--capture-desktop 사용 불가"
fi
if [[ "${XDG_SESSION_TYPE:-}" == "x11" ]]; then
  pass "visual.session" "X11 (검증된 desktop capture 경로)"
else
  warn "visual.session" "${XDG_SESSION_TYPE:-unknown}; owned-window capture는 X11에서만 검증됨"
fi

printf '\n%s\n' '5) Imported Autoware source'
manifest="${root}/autoware.repos"
expected_repositories=0
present_repositories=0
git_repositories=0
matching_revisions=0
missing_repositories=()
invalid_repositories=()
mismatched_repositories=()
if [[ -f "${manifest}" ]]; then
  while IFS=$'\t' read -r relative_path expected_revision; do
    expected_repositories=$((expected_repositories + 1))
    repository_path="${root}/src/${relative_path}"
    if [[ -d "${repository_path}" ]]; then
      present_repositories=$((present_repositories + 1))
      if git -C "${repository_path}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        git_repositories=$((git_repositories + 1))
        actual_revision="$(git -C "${repository_path}" rev-parse HEAD 2>/dev/null || true)"
        pinned_revision="$(
          git -C "${repository_path}" rev-parse --verify \
            "${expected_revision}^{commit}" 2>/dev/null || true
        )"
        if [[ -n "${actual_revision}" && "${actual_revision}" == "${pinned_revision}" ]]; then
          matching_revisions=$((matching_revisions + 1))
        else
          mismatched_repositories+=("${relative_path}")
        fi
      else
        invalid_repositories+=("${relative_path}")
      fi
    else
      missing_repositories+=("${relative_path}")
    fi
  done < <(
    awk '
      /^  [^ #][^:]*:[[:space:]]*(#.*)?$/ {
        path = $0
        sub(/^  /, "", path)
        sub(/:[[:space:]]*(#.*)?$/, "", path)
        next
      }
      path != "" && /^    version:[[:space:]]*/ {
        revision = $0
        sub(/^    version:[[:space:]]*/, "", revision)
        sub(/[[:space:]]+#.*/, "", revision)
        gsub(/^["'\'']|["'\'']$/, "", revision)
        print path "\t" revision
        path = ""
      }
    ' "${manifest}"
  )
fi
if (( expected_repositories == 0 )); then
  block "source.manifest" "autoware.repos를 읽지 못했거나 repository 항목이 없음"
elif (( present_repositories == expected_repositories )); then
  pass "source.layout" "manifest 경로 ${present_repositories}/${expected_repositories} 존재"
else
  block "source.layout" "manifest 경로 ${present_repositories}/${expected_repositories} 존재; ${#missing_repositories[@]}개 누락"
  if (( ${#missing_repositories[@]} > 0 )); then
    missing_preview="$(printf '%s, ' "${missing_repositories[@]:0:3}")"
    info "source.missing" "${missing_preview%, }$([[ ${#missing_repositories[@]} -gt 3 ]] && printf ', ...')"
  fi
fi
if (( present_repositories > 0 )); then
  if (( git_repositories == present_repositories )); then
    pass "source.git" "${git_repositories}/${present_repositories} Git checkout 확인"
  else
    block "source.git" "${git_repositories}/${present_repositories} 경로만 Git checkout; ${#invalid_repositories[@]}개가 유효하지 않음"
  fi
  if (( matching_revisions == expected_repositories )); then
    pass "source.revision" "HEAD ${matching_revisions}/${expected_repositories}가 manifest tag/commit과 일치"
  elif (( present_repositories == expected_repositories )); then
    block "source.revision" "HEAD ${matching_revisions}/${expected_repositories} 일치; ${#mismatched_repositories[@]}개 mismatch/unresolved"
    if (( ${#mismatched_repositories[@]} > 0 )); then
      mismatch_preview="$(printf '%s, ' "${mismatched_repositories[@]:0:3}")"
      info "source.mismatch" "${mismatch_preview%, }$([[ ${#mismatched_repositories[@]} -gt 3 ]] && printf ', ...')"
    fi
  fi
fi

printf '\n%s\n' '6) External CARLA runtime'
carla_root="${carla_override:-${CARLA_ROOT:-}}"
if [[ -z "${carla_root}" ]]; then
  for candidate in \
    "${root}/../carla-autoware-universe/CARLA_0.9.15" \
    /opt/carla-simulator \
    /opt/carla; do
    if [[ -x "${candidate}/CarlaUE4.sh" ]]; then
      carla_root="${candidate}"
      break
    fi
  done
fi
if [[ -n "${carla_root}" && -x "${carla_root}/CarlaUE4.sh" ]]; then
  pass "carla.runtime" "${carla_root}/CarlaUE4.sh"
  carla_egg="${carla_root}/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
  if [[ -f "${carla_egg}" ]]; then
    pass "carla.python" "CARLA 0.9.15 / Python 3.10 / x86_64 egg"
  else
    block "carla.python" "정확한 0.9.15 Python 3.10 x86_64 egg 누락"
  fi
else
  carla_root="${carla_root:-${root}/../carla-autoware-universe/CARLA_0.9.15}"
  block "carla.runtime" "${carla_root}/CarlaUE4.sh 없음; CARLA는 Git에 포함되지 않음"
fi

printf '\n%s\n' '7) VAD model artifacts'
data_path="${AUTOWARE_E2E_DATA_PATH:-${root}/data/ml_models}"
model_root="${data_path}/vad/v0.1"
model_files=(
  "${model_root}/vad-carla-tiny_backbone.onnx"
  "${model_root}/vad-carla-tiny_head.onnx"
  "${model_root}/vad-carla-tiny_head_no_prev.onnx"
  "${model_root}/vad-carla-tiny.param.json"
)
check_required_files "model.vad_v0.1" "${model_files[@]}"
model_sha256=(
  "04b925f2750fd1c4adf16b5aae9c149d0baa39185e99d141232ebe20bddba4da"
  "31f49a592a764ce82bbe6e26d0bfc99dc8a9613628884dc73dd5e67521ff3e9e"
  "6a89d479e0717b1e526f1aa3a1137c631a9ef854e810d0328a035aae11818c2a"
  "03d3187fed3c70f761456afc2e93e18c1765113b1c1580ffa78ab42b10dbd179"
)
model_hash_failures=0
if (( missing_file_count == 0 )); then
  for index in "${!model_files[@]}"; do
    actual_model_sha256="$(sha256sum -- "${model_files[index]}" | awk '{print $1}')"
    if [[ "${actual_model_sha256}" != "${model_sha256[index]}" ]]; then
      model_hash_failures=$((model_hash_failures + 1))
    fi
  done
  if (( model_hash_failures == 0 )); then
    pass "model.integrity" "4/4 download contract SHA-256 일치"
  else
    block "model.integrity" "${model_hash_failures}/4 SHA-256 mismatch; 손상/다른 model 사용 금지"
  fi
fi
engine_count=0
for engine in \
  "${model_root}/vad-carla-tiny_backbone.engine" \
  "${model_root}/vad-carla-tiny_head.engine" \
  "${model_root}/vad-carla-tiny_head_no_prev.engine"; do
  [[ -s "${engine}" ]] && engine_count=$((engine_count + 1))
done
if (( engine_count == 3 )); then
  pass "model.engine_cache" "3/3 non-empty GPU-local TensorRT engine cache 존재"
  info "model.engine_scope" "cache provenance/GPU compatibility는 doctor와 실제 VAD load에서만 확정"
else
  warn "model.engine_cache" "${engine_count}/3 non-empty; ONNX가 있으면 첫 VAD 실행 때 생성됨"
fi

printf '\n%s\n' '8) Full-map bundle'
map_path="${map_override:-${AUTOWARE_E2E_PREFLIGHT_MAP_PATH:-${root}/data/maps/Town01_full}}"
map_files=(
  "${map_path}/lanelet2_map.osm"
  "${map_path}/pointcloud_map.pcd"
  "${map_path}/map_projector_info.yaml"
  "${map_path}/map_bundle.json"
)
check_required_files "map.full_bundle" "${map_files[@]}"
info "map.path" "${map_path}"
if (( missing_file_count == 0 )); then
  map_integrity="$({
    python3 - "${map_path}" <<'PY'
import hashlib
import json
import math
from pathlib import Path
import re
import sys


root = Path(sys.argv[1])
metadata = json.loads((root / "map_bundle.json").read_text(encoding="utf-8"))
if metadata.get("schema_version") != 1:
    raise RuntimeError("map_bundle schema_version must be 1")
if not isinstance(metadata.get("profile"), str) or not metadata["profile"]:
    raise RuntimeError("map_bundle profile is missing")
if not isinstance(metadata.get("canonical_carla_map"), str):
    raise RuntimeError("map_bundle canonical_carla_map is missing")
canonical_carla_map = metadata["canonical_carla_map"]
if (
    not canonical_carla_map.startswith("/Game/")
    or canonical_carla_map == "/Game/"
    or canonical_carla_map.endswith("/")
):
    raise RuntimeError("map_bundle canonical_carla_map must name a non-empty /Game/ level")
if not isinstance(metadata.get("status"), str) or not metadata["status"]:
    raise RuntimeError("map_bundle status is missing")

transform = metadata.get("carla_to_map_transform")
if not isinstance(transform, dict):
    raise RuntimeError("map_bundle carla_to_map_transform is missing")
for field in ("x_m", "y_m", "z_m", "yaw_rad"):
    value = transform.get(field)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"carla_to_map_transform.{field} is not numeric")
    if not math.isfinite(float(value)):
        raise RuntimeError(f"carla_to_map_transform.{field} is not finite")

lanelet = metadata.get("lanelet2")
generated = metadata.get("pointcloud_generated")
if isinstance(lanelet, dict) and isinstance(lanelet.get("file"), dict) and isinstance(generated, dict):
    records = {
        "lanelet2_map.osm": lanelet["file"],
        "pointcloud_map.pcd": generated,
    }
else:
    sources = metadata.get("bundle_sources")
    if not isinstance(sources, dict):
        raise RuntimeError("unsupported map_bundle asset identity schema")
    records = {
        "lanelet2_map.osm": sources.get("lanelet2_map"),
        "pointcloud_map.pcd": sources.get("pointcloud_map"),
    }

for name, record in records.items():
    if not isinstance(record, dict):
        raise RuntimeError(f"{name} identity is missing")
    expected_hash = record.get("sha256") or record.get("expected_sha256")
    expected_size = record.get("size_bytes") or record.get("expected_size_bytes")
    if not isinstance(expected_hash, str) or not re.fullmatch(r"[0-9a-f]{64}", expected_hash):
        raise RuntimeError(f"{name} SHA-256 identity is invalid")
    if isinstance(expected_size, bool) or not isinstance(expected_size, int) or expected_size <= 0:
        raise RuntimeError(f"{name} size identity is invalid")
    path = root / name
    if path.stat().st_size != expected_size:
        raise RuntimeError(f"{name} size mismatch")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    if digest.hexdigest() != expected_hash:
        raise RuntimeError(f"{name} SHA-256 mismatch")

projector_text = (root / "map_projector_info.yaml").read_text(encoding="utf-8")
projector_type = metadata.get("projector_type")
if not isinstance(projector_type, str) or not re.search(
    rf"(?m)^projector_type:\s*{re.escape(projector_type)}\s*$", projector_text
):
    raise RuntimeError("map projector metadata mismatch")
print(f"profile={metadata['profile']} status={metadata['status']} assets=2/2")
PY
  } 2>&1)"
  map_integrity_status=$?
  if (( map_integrity_status == 0 )); then
    pass "map.integrity" "${map_integrity} SHA-256/size/projector 일치"
    map_bundle_status="$(
      python3 - "${map_path}/map_bundle.json" <<'PY'
import json
from pathlib import Path
import sys


print(json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))["status"])
PY
    )"
    case "${map_bundle_status}" in
      full_map_structurally_ready_not_vad_validated|live_hybrid_validated_xy_with_3d_residual)
        pass "map.admission" "${map_bundle_status}; route/runtime 입력으로 허용"
        ;;
      reference_only*)
        block "map.admission" "${map_bundle_status}; 참고용 bundle은 runtime map으로 사용 금지"
        ;;
      ready_with_single_anchor_alignment)
        block "map.admission" "${map_bundle_status}; single-anchor custom map은 별도 runtime/route admission 전 사용 금지"
        ;;
      *)
        block "map.admission" "알 수 없는 status=${map_bundle_status}; matrix 계약에 명시적으로 승인 필요"
        ;;
    esac
  else
    block "map.integrity" "${map_integrity//$'\n'/ }"
  fi
fi

printf '\n%s\n' '9) Project-local build dependencies'
cuda_root="${AUTOWARE_E2E_CUDA_ROOT:-${root}/data/vendor/cuda-12.8/root/usr/local/cuda-12.8}"
tensorrt_root="${AUTOWARE_E2E_TENSORRT_ROOT:-${root}/data/vendor/tensorrt-10.8/root/usr}"
spconv_root="${AUTOWARE_E2E_SPCONV_ROOT:-${root}/data/vendor/spconv-cu128/root/usr/local}"
acados_root="${AUTOWARE_E2E_ACADOS_ROOT:-${root}/data/vendor/acados-v0.5.3}"
if [[ -x "${cuda_root}/bin/nvcc" ]]; then
  pass "vendor.cuda" "project-local CUDA 12.8 nvcc 존재"
else
  block "vendor.cuda" "${cuda_root}/bin/nvcc 없음"
  if apt-cache show cuda-nvcc-12-8 >/dev/null 2>&1; then
    pass "vendor.cuda_source" "cuda-nvcc-12-8 package를 현재 apt source에서 확인"
  else
    block "vendor.cuda_source" "cuda-nvcc-12-8 package 없음; upstream NVIDIA task/repository 준비 필요"
  fi
fi
if [[ -f "${tensorrt_root}/include/NvInfer.h" && -e "${tensorrt_root}/lib64/libnvinfer.so" ]]; then
  pass "vendor.tensorrt" "project-local TensorRT headers/library 존재"
else
  block "vendor.tensorrt" "TensorRT 10.8 project-local prefix 불완전"
  if apt-cache show 'libnvinfer10=10.8.0.43-1+cuda12.8' >/dev/null 2>&1; then
    pass "vendor.tensorrt_source" "pinned libnvinfer10 10.8 package를 현재 apt source에서 확인"
  else
    block "vendor.tensorrt_source" "pinned libnvinfer10 10.8 package 없음; upstream NVIDIA task/repository 준비 필요"
  fi
fi
if [[ -f "${spconv_root}/lib/libspconv.so" ]]; then
  pass "vendor.spconv" "pinned amd64 spconv 존재"
else
  block "vendor.spconv" "${spconv_root}/lib/libspconv.so 없음"
fi
if [[ -f "${acados_root}/cmake/acadosConfig.cmake" ]]; then
  pass "vendor.acados" "acados v0.5.3 local install 존재"
else
  block "vendor.acados" "${acados_root}/cmake/acadosConfig.cmake 없음"
fi

printf '\n%s\n' '10) Workspace build'
if [[ -f "${root}/install/local_setup.bash" ]]; then
  pass "build.install_space" "${root}/install/local_setup.bash"
else
  block "build.install_space" "install/ 없음; Git clone에는 build 결과가 포함되지 않음"
fi
runtime_libraries=(
  "${root}/install/autoware_tensorrt_vad/lib/libautoware_tensorrt_vad_lib.so"
  "${root}/install/autoware_tensorrt_plugins/share/autoware_tensorrt_plugins/plugins/libautoware_tensorrt_plugins.so"
)
check_required_files "build.vad_runtime" "${runtime_libraries[@]}"
info "readiness.scope" "file/metadata readiness만 판정; ABI·GPU engine load·ROS graph·폐루프 주행은 doctor/runtime gate 대상"

printf '\nSummary\n'
printf '  PASS=%d  WARN=%d  BLOCK=%d\n' "${passes}" "${warnings}" "${blockers}"
if (( blockers == 0 )); then
  printf '  status: READY_FOR_DOCTOR\n'
else
  printf '  status: NOT_READY (%d blocker%s)\n' "${blockers}" "$([[ ${blockers} -eq 1 ]] || printf s)"
fi
printf '  mutation: NONE (설치/다운로드/빌드/파일 생성/프로세스 종료 없음)\n'

cat <<'EOF'

권장 다음 순서 (아래 명령은 표시만 했으며 실행하지 않았습니다)

  1. 발행된 PNG/GIF까지 보려면 Git LFS를 설치한 뒤:
     git lfs install
     git lfs pull --include='docs/assets/validation/2026-09-02-runtime-control-campaign-v1/**' --exclude=''

  2. 지원 OS가 아니거나 ROS/tool이 누락됐다면 먼저
     docs/BEGINNER_QUICKSTART_KO.md의 "호스트 준비"를 따르세요.
     주의: 현재 clone에는 ./ansible이 없으므로 setup-dev-env.sh를 바로 실행하지 마세요.
     공식 exact provisioning source 확인:
     git clone --branch 1.9.0 --depth 1 https://github.com/autowarefoundation/autoware.git ../autoware-upstream-1.9.0
     test "$(git -C ../autoware-upstream-1.9.0 rev-parse HEAD)" = 10718787ba6e28f038a0cb29ff99cc627b5abfd2
     설치는 위 checkout의 공식 Ansible 안내와 docs/BEGINNER_QUICKSTART_KO.md를 읽은 뒤 수동 실행하세요.

  3. source가 누락됐다면:
     mkdir -p src
     vcs import src < autoware.repos

  4. 별도로 받은 CARLA 0.9.15 경로를 현재 terminal에 지정:
     export CARLA_ROOT=/absolute/path/to/CARLA_0.9.15

  5. 공식 VAD v0.1 ONNX를 checksum 검증하며 받기:
     scripts/e2e/download_vad_models.sh

  6. 고정 manifest와 별도로 확보한 Lanelet2/CARLA source로 Town01 inventory:
     export AUTOWARE_E2E_TOWN_LANELET_ROOT=/absolute/path/to/autoware-contents/maps
     export CARLA_ROOT=/absolute/path/to/CARLA_0.9.15
     python3 scripts/e2e/prepare_packaged_town_full_maps.py --map town01
     READY_TO_PREPARE이고 오류가 없을 때만 같은 명령 끝에 --prepare를 붙이세요.

  7. 위 입력이 모두 준비된 뒤 Full stack 빌드:
     scripts/e2e/build_full.sh

  8. 다시 엄격 진단하고 기존 runtime doctor 실행:
     bash scripts/e2e/bootstrap_preflight.sh --strict
     scripts/e2e/doctor.sh
EOF

if [[ "${strict}" == "true" && ${blockers} -gt 0 ]]; then
  exit 1
fi
exit 0
