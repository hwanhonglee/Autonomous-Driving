#!/usr/bin/env bash
# HH_260906 - Capture bounded independent BasicAgent episodes without learned actuation.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"
if [[ "${1:-}" == "--help" ]]; then
  printf 'Usage: bash %s town04|town01 [1|2]\n' "$0"
  exit 0
fi
scene="${1:?select town04 or town01}"
campaign_attempt="${2:-1}"
[[ "$campaign_attempt" == 1 || "$campaign_attempt" == 2 ]]
case "$scene" in
  town04) map_name=Town04; split=test; scenario=straight ;;
  town01) map_name=Town01; split=train; scenario=right ;;
  *) printf 'Unsupported scene: %s\n' "$scene" >&2; exit 2 ;;
esac
source scripts/e2e/env.sh
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
campaign_root="$repo_root/artifacts/training/2026-09-07/independent_common10_v1"
scene_root="$campaign_root/${scene}_${scenario}_${split}"
if [[ "$campaign_attempt" == 2 ]]; then
  # HH_260906 - Preserve an unsuccessful preflight under its original campaign path.
  scene_root="${scene_root}_retry_02"
fi
test ! -e "$scene_root"
available_bytes="$(df -B1 --output=avail "$repo_root" | tail -n 1 | tr -d ' ')"
(( available_bytes > 10737418240 )) || { printf 'Less than 10 GiB free\n' >&2; exit 2; }
if pgrep -f '[C]arlaUE4-Linux-Shipping' >/dev/null; then
  printf 'Existing CARLA process; refusing concurrent world ownership\n' >&2
  exit 2
fi
if ss -ltn | awk '{print $4}' | rg -q ':210[012]$'; then
  printf 'CARLA ports already occupied\n' >&2
  exit 2
fi
mapping="$repo_root/autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml"
calibration="$repo_root/src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml"
carla_egg="$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
test "$(sha256sum "$mapping" | awk '{print $1}')" = 9aaff2befed7ad12376b2e04bbdd51bd1808a3bafe39d87a6f6b241dbcca3136
test "$(sha256sum "$calibration" | awk '{print $1}')" = 5022cd1de5b48e9c824b6f2f8c59991fa665eeaf7a7bafd084c88acdb65e4bea
test "$(sha256sum "$carla_egg" | awk '{print $1}')" = 6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a
mkdir -p "$scene_root"
date -Is > "$scene_root/started_at.txt"
git rev-parse HEAD > "$scene_root/source_git_head.txt"
git status --short > "$scene_root/source_git_status.txt"
sha256sum "$mapping" "$calibration" "$carla_egg" \
  scripts/e2e/collect_carla_vad_expert.py \
  scripts/e2e/export_carla_vad_expert.py \
  scripts/e2e/prepare_carla_common10_dataset.py \
  scripts/e2e/prepare_carla_expert_route_catalog.py \
  scripts/e2e/run_independent_common10_capture_20260907.sh \
  > "$scene_root/input_sha256.txt"
# HH_260906 - Archive the exact runner bytes so later fixes cannot obscure provenance.
cp -- "${BASH_SOURCE[0]}" "$scene_root/runner_source.sh"
carla_pid=''

owned_group_alive() {
  [[ -n "$carla_pid" ]] && ps -eo pgid=,stat= | awk -v group="$carla_pid" '$1 == group && $2 !~ /^Z/ { found=1 } END { exit !found }'
}

cleanup() {
  # HH_260906 - Signal only the process group created by this runner and verify shutdown.
  local status=$?
  trap - EXIT INT TERM HUP
  if [[ -n "$carla_pid" ]]; then
    kill -TERM -- "-$carla_pid" 2>/dev/null || true
    for _ in {1..20}; do
      owned_group_alive || break
      sleep 1
    done
    if owned_group_alive; then
      kill -KILL -- "-$carla_pid" 2>/dev/null || true
      sleep 1
    fi
    wait "$carla_pid" 2>/dev/null || true
    if owned_group_alive; then
      printf 'FAIL: owned CARLA group survived\n' > "$scene_root/cleanup_status.txt"
      status=1
    else
      printf 'PASS: no live members in owned CARLA process group %s\n' "$carla_pid" > "$scene_root/cleanup_status.txt"
    fi
  fi
  printf '%s\n' "$status" > "$scene_root/runner_exit_status.txt"
  date -Is > "$scene_root/finished_at.txt"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

setsid bash scripts/e2e/run_carla_map.sh "$map_name" \
  --port 2100 --quality Epic --startup-timeout-sec 90 \
  -- -windowed -ResX=1280 -ResY=720 -nosound \
  > "$scene_root/carla_server.log" 2>&1 &
carla_pid=$!
printf '%s\n' "$carla_pid" > "$scene_root/owned_carla_pgid.txt"
for _ in {1..100}; do
  kill -0 "$carla_pid" 2>/dev/null || { tail -n 50 "$scene_root/carla_server.log"; exit 1; }
  if rg -q 'CARLA_READY' "$scene_root/carla_server.log"; then break; fi
  sleep 1
done
rg -q 'CARLA_READY' "$scene_root/carla_server.log"
test "$(ps -o pgid= -p "$carla_pid" | tr -d ' ')" = "$carla_pid"
printf 'READY %s; collecting %s expert episode\n' "$map_name" "$split"

if [[ "$scene" == town04 ]]; then
  source_route="$repo_root/artifacts/validation/2026-09-02/autoware_vad_town_matrix_30kph_camera_source_5hz_v1/maps/town04/catalog/straight/routes/town04/straight/town04_straight_s0000_p00.json"
  test "$(sha256sum "$source_route" | awk '{print $1}')" = ee5335a664392136c7738b96ec639a9cffb116db6d5171e8d7af600a83cc1b6e
else
  timeout --signal=TERM --kill-after=20s 300s \
    python3 scripts/e2e/prepare_carla_expert_route_catalog.py \
      --map-id town01 --active-server-profile packaged_0915 \
      --host 127.0.0.1 --port 2100 --map-load-settle-sec 0 \
      --output-root "$scene_root/route_catalog" \
      --scenarios left,right --seeds 1 --pairs-per-seed 1 \
      --min-distance 200 --max-distance 230 --preferred-distance 210 \
      --sampling-resolution 1 --physical-turn-profile speed_30kph \
      --max-traces 100000 > "$scene_root/catalog.log" 2>&1
  source_route="$scene_root/route_catalog/routes/town01/right/town01_right_s0001_p00.json"
fi
test -f "$source_route"
route_stem="$(basename "$source_route" .json)"
aligned_route="$scene_root/${route_stem}.aligned.json"
python3 scripts/e2e/align_carla_route_to_map.py \
  "$source_route" "data/maps/${map_name}_full" \
  --output "$aligned_route" --json > "$scene_root/route_alignment.json"

qualified=false
for attempt in 1 2; do
  # HH_260906 - Preserve failed attempts and retry the same predeclared route only once.
  run_root="$scene_root/run_00${attempt}"
  mkdir "$run_root"
  printf 'CAPTURE_START %s attempt=%s\n' "$scene" "$attempt"
  set +e
  timeout --signal=TERM --kill-after=20s 360s \
    python3 scripts/e2e/collect_carla_vad_expert.py \
      "$run_root/episode" "$aligned_route" \
      --host 127.0.0.1 --port 2100 \
      --physics-hz 20 --capture-hz 10 --target-speed-kmh 30 \
      --max-duration-sec 120 \
      --stationary-warmup-sec 3.5 --stationary-tail-sec 6.5 \
      --spawn-z-offset-m 0.5 --weather ClearNoon --seed 1 \
      --mapping "$mapping" --calibration "$calibration" \
      --basic-agent-base-min-distance-m 2.0 \
      --basic-agent-distance-ratio 0.2 \
      --basic-agent-lateral-kp 1.95 --basic-agent-lateral-ki 0.05 \
      --basic-agent-lateral-kd 0.20 --basic-agent-max-steering 0.8 \
      --basic-agent-lane-offset-m 0.0 > "$run_root/collector.log" 2>&1
  collect_status=$?
  set -e
  printf '%s\n' "$collect_status" > "$run_root/collector_exit_status.txt"
  if (( collect_status != 0 )); then
    tail -n 20 "$run_root/collector.log"
    continue
  fi
  capture_horizons="$(python3 -c 'print(",".join(f"{i/10:.1f}" for i in range(1,65)))')"
  set +e
  timeout --signal=TERM --kill-after=10s 120s \
    python3 scripts/e2e/export_carla_vad_expert.py \
      --input "$run_root/episode" --output "$run_root/analysis/vad_export_64" \
      --horizons "$capture_horizons" > "$run_root/export.log" 2>&1
  export_status=$?
  set -e
  printf '%s\n' "$export_status" > "$run_root/export_exit_status.txt"
  if (( export_status != 0 )); then
    tail -n 20 "$run_root/export.log"
    continue
  fi
  timeout --signal=TERM --kill-after=10s 180s \
    python3 scripts/e2e/render_carla_vad_expert.py \
      --episode "$run_root/episode" --export "$run_root/analysis/vad_export_64" \
      --output-png "$run_root/visual/centered_overview.png" \
      --output-gif "$run_root/visual/centered_drive.gif" \
      --width 1920 --height 1080 --fps 5 --max-frames 200 \
      > "$run_root/render.log" 2>&1
  printf '%s\n' "$run_root/episode" > "$scene_root/export_validated_episode.txt"
  printf 'EXPERT_EXPORT_PASS %s %s\n' "$scene" "$run_root/episode"
  qualified=true
  break
done
[[ "$qualified" == true ]]
# HH_260906 - Common10 conversion requires separately reviewed converter provenance.
printf 'PENDING_REVIEWED_CONVERSION_SOURCE\n' > "$scene_root/common10_conversion_status.txt"
