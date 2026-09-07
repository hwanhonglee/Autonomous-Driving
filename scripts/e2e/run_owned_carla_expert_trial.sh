#!/usr/bin/env bash
# HH_260906 - Bound an expert-only capture to one owned CARLA generation and retain failed trials.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh
source scripts/e2e/process_group_cleanup.sh
source scripts/e2e/workspace_runtime_lock.sh

usage() {
  echo "Usage: run_owned_carla_expert_trial.sh OUTPUT_ROOT ROUTE_JSON [--port PORT] [--quality Low|Epic] [--wall-timeout-sec SEC] [--finish-before-utc YYYY-MM-DDTHH:MM:SSZ] [--capture-mode expert|actuation-response] [-- COLLECTOR_OPTIONS...]"
}
if [[ $# -lt 2 ]]; then usage >&2; exit 2; fi
output_root="$(realpath -m -- "$1")"
route_file="$(realpath -- "$2")"
shift 2
port=2100
quality=Low
wall_timeout=900
finish_before_utc=""
capture_mode=expert
collector_options=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; port="$2"; shift 2 ;;
    --quality) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; quality="$2"; shift 2 ;;
    --wall-timeout-sec) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; wall_timeout="$2"; shift 2 ;;
    --finish-before-utc) [[ $# -ge 2 && -n "$2" ]] || { usage >&2; exit 2; }; finish_before_utc="$2"; shift 2 ;;
    --capture-mode) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; capture_mode="$2"; shift 2 ;;
    --) shift; collector_options=("$@"); break ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ "${port}" =~ ^[0-9]+$ ]] && (( port >= 1024 && port <= 65533 )) || exit 2
[[ "${wall_timeout}" =~ ^[0-9]+$ ]] && (( wall_timeout > 0 && wall_timeout <= 3600 )) || exit 2
[[ "${quality}" == Low || "${quality}" == Epic ]] || exit 2
# HH_260906 - Reserve startup, timeout escalation, and owned cleanup before an explicit UTC work boundary.
check_finish_budget() {
  python3 - "${finish_before_utc}" "${wall_timeout}" "$1" <<'PY'
from datetime import datetime, timezone
import re
import sys
text, capture_seconds, overhead_seconds = sys.argv[1:]
if text:
    if re.fullmatch(r'[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z', text) is None:
        raise SystemExit('finish boundary must use YYYY-MM-DDTHH:MM:SSZ')
    try:
        deadline = datetime.fromisoformat(text[:-1] + '+00:00')
    except ValueError:
        raise SystemExit('finish boundary is not a valid UTC datetime')
    remaining = (deadline - datetime.now(timezone.utc)).total_seconds()
    needed = int(capture_seconds) + int(overhead_seconds)
    if remaining < needed:
        raise SystemExit(f'Insufficient time before finish boundary: {remaining:.1f}s remains, {needed}s reserved')
PY
}
check_finish_budget 330 || exit 2
# HH_260906 - Permit only named repository workers, never an arbitrary supplied executable.
case "${capture_mode}" in
  expert)
    worker_module=scripts.e2e.collect_carla_vad_expert
    worker_path=scripts/e2e/collect_carla_vad_expert.py
    worker_output_name=episode ;;
  actuation-response)
    worker_module=scripts.e2e.calibrate_carla_low_speed_response
    worker_path=scripts/e2e/calibrate_carla_low_speed_response.py
    worker_output_name=actuation ;;
  *) usage >&2; exit 2 ;;
esac
if [[ -e "${output_root}" || -L "${output_root}" ]]; then
  echo "Refusing to overwrite an expert trial: ${output_root}" >&2
  exit 2
fi
for option in "${collector_options[@]}"; do
  case "${option}" in
    --help|-h)
      # HH_260906 - A help request must never become a successful preflight followed by a world launch.
      usage
      exit 0 ;;
    --host|--host=*|--port|--port=*|--allow-map-load)
      echo "Collector cannot redirect the owned simulator or load another world." >&2
      exit 2 ;;
  esac
done
# HH_260906 - Resolve the complete strict collector argv before starting any simulator.
python3 - "${worker_module}" "${output_root}/${worker_output_name}" "${route_file}" "${port}" "${collector_options[@]}" <<'PY'
import importlib
import sys
module, output, route, port, *options = sys.argv[1:]
parse_args = importlib.import_module(module).parse_args
args = parse_args([output, route, '--host', '127.0.0.1', '--port', port, *options])
if args.host != '127.0.0.1' or args.port != int(port) or args.allow_map_load:
    raise SystemExit('collector argv escapes the owned simulator')
PY
map_name="$(python3 - "${route_file}" <<'PY'
# HH_260906 - Read only the existing route map; never infer a different target world.
import json
from pathlib import Path
import re
import sys
route = json.loads(Path(sys.argv[1]).read_text())
name = route.get('town')
if not isinstance(name, str) or re.fullmatch(r'[A-Za-z0-9_]+', name) is None:
    raise SystemExit('route has no safe town identity')
print(name)
PY
)"
if [[ "${map_name}" == C_track_1_0_7 && "${quality}" != Epic ]]; then
  echo "Packaged C-track requires Epic quality; the Low LOD crash is already known." >&2
  exit 2
fi
e2e_acquire_workspace_runtime_lock "owned expert collection"
python3 - "${port}" <<'PY'
# HH_260906 - Occupied ports are a blocker, not authority to kill their owner.
import socket
import sys
for port in range(int(sys.argv[1]), int(sys.argv[1]) + 3):
    with socket.socket() as probe:
        try:
            probe.bind(('127.0.0.1', port))
        except OSError:
            raise SystemExit(f'CARLA port {port} is already occupied')
PY
# HH_260906 - Claim only a new final directory atomically; never reuse logs after a race.
mkdir -p "$(dirname "${output_root}")"
mkdir -- "${output_root}"
mkdir -- "${output_root}/lifecycle"
python3 - "${output_root}/owner_plan.json" "${route_file}" "${map_name}" "${port}" "${quality}" "${wall_timeout}" "${capture_mode}" "${worker_path}" "${worker_output_name}" "${finish_before_utc}" "${collector_options[@]}" <<'PY'
# HH_260906 - Preserve the source and command contract even when simulator startup fails.
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
path, route, town, port, quality, timeout, mode, worker, output_name, finish_before, *options = sys.argv[1:]
sources = ('scripts/e2e/run_owned_carla_expert_trial.sh', 'scripts/e2e/run_carla_map.sh',
    'scripts/e2e/process_group_cleanup.sh', 'scripts/e2e/workspace_runtime_lock.sh',
    'scripts/e2e/probe_carla_server.py', 'scripts/e2e/env.sh',
    'scripts/e2e/collect_carla_vad_expert.py', 'scripts/e2e/carla_goal_stop_profile.py', worker)
if mode == 'actuation-response':
    # HH_260906 - Archive the prospective coast/ramp contract together with its importing worker.
    sources += ('scripts/e2e/carla_low_speed_response_matrix.py',)
def git(*args):
    return subprocess.run(['git', *args], check=True, capture_output=True,
        text=True, timeout=15).stdout.strip()
# HH_260906 - Keep exact source bytes privately; a prelaunch hash alone cannot replay an uncommitted revision.
source_hashes = {}
for name in dict.fromkeys(sources):
    payload = Path(name).read_bytes()
    archived = Path(path).parent / 'provenance' / name
    archived.parent.mkdir(parents=True, exist_ok=True)
    with archived.open('xb') as stream:
        stream.write(payload)
    source_hashes[name] = hashlib.sha256(payload).hexdigest()
with Path(path).open('x') as stream:
    json.dump({'planned_at_utc': datetime.now(timezone.utc).isoformat(),
        'schema': 'portable_e2e.owned_expert_trial.v1',
        'source_head_commit': git('rev-parse', 'HEAD'),
        'source_worktree_status': git('status', '--porcelain', '--untracked-files=all'),
        'source_sha256': source_hashes, 'source_bytes_archived': True,
        'route_path': route, 'route_sha256': hashlib.sha256(Path(route).read_bytes()).hexdigest(),
        'map': town, 'host': '127.0.0.1', 'port': int(port), 'quality': quality,
        'capture_mode': mode, 'worker_path': worker,
        'collector_wall_timeout_sec': int(timeout), 'server_startup_timeout_sec': 180,
        'finish_before_utc': finish_before or None,
        'finish_budget_policy': {'prelaunch_overhead_sec': 330, 'precapture_overhead_sec': 120,
            'notice': 'Admission budget checks, not a hard real-time OS guarantee; no unrelated processes are signaled.'},
        'collector_argv': [str(Path(path).parent / output_name), route, '--host', '127.0.0.1', '--port', port, *options],
        'server_extra_options': ['-RenderOffScreen', '-nosound'],
        'learned_model_control': False, 'vehicle_control_approved': False}, stream, indent=2)
PY
server_pid=""
collector_pid=""
cleanup_status=0
probe() {
  local stage="$1"
  shift
  python3 scripts/e2e/probe_carla_server.py \
    --host 127.0.0.1 --port "${port}" --timeout 4 --expected-map "${map_name}" \
    --owner-pid "${server_pid}" --owner-pgid "${server_pid}" \
    --generation-id "expert_${server_pid}" --stage "${stage}" \
    --server-log "${output_root}/server.log" --output "${output_root}/lifecycle/${stage}.json" "$@"
}
cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  if [[ -n "${collector_pid}" ]]; then
    e2e_stop_owned_process_group "${collector_pid}" "${collector_pid}" 20 5 2 || cleanup_status=1
  fi
  if [[ -n "${server_pid}" ]]; then
    e2e_stop_owned_process_group "${server_pid}" "${server_pid}" 20 5 2 || cleanup_status=1
    probe stopped --expect-stopped || cleanup_status=1
  fi
  if (( cleanup_status != 0 )); then status=1; fi
  python3 - "${output_root}/owner_result.json" "${status}" "${capture_mode}" <<'PY'
# HH_260906 - Preserve exit status independently of whether the expert route or comfort checks passed.
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
plan = json.loads(Path(sys.argv[1]).with_name('owner_plan.json').read_text())
source_checks = {}
for name, expected in plan['source_sha256'].items():
    source = Path(name)
    archived = Path(sys.argv[1]).parent / 'provenance' / name
    try:
        source_checks[name] = source.is_file() and archived.is_file() and all(
            hashlib.sha256(p.read_bytes()).hexdigest() == expected for p in (source, archived))
    except OSError:
        source_checks[name] = False
final_status = int(sys.argv[2]) if all(source_checks.values()) else 1
with Path(sys.argv[1]).open('x') as stream:
    json.dump({'completed_at_utc': datetime.now(timezone.utc).isoformat(), 'exit_code': final_status,
        'capture_mode': sys.argv[3],
        'source_bytes_unchanged_and_archived': all(source_checks.values()), 'source_checks': source_checks,
        'learned_model_control': False, 'vehicle_control_approved': False}, stream, indent=2)
raise SystemExit(0 if all(source_checks.values()) else 1)
PY
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
check_finish_budget 330 || exit 2
setsid bash scripts/e2e/run_carla_map.sh "${map_name}" --port "${port}" --quality "${quality}" \
  --startup-timeout-sec 180 -- -RenderOffScreen -nosound >"${output_root}/server.log" 2>&1 &
server_pid=$!
deadline=$((SECONDS + 195))
while (( SECONDS < deadline )); do
  if rg -q '^CARLA_READY ' "${output_root}/server.log"; then break; fi
  kill -0 "${server_pid}" 2>/dev/null || break
  sleep 1
done
rg -q '^CARLA_READY ' "${output_root}/server.log" || {
  echo "Owned CARLA failed to become ready; inspect retained server.log." >&2; exit 1;
}
probe ready
python3 - "${output_root}/owner_started.json" "${server_pid}" "${route_file}" "${map_name}" "${port}" "${quality}" <<'PY'
# HH_260906 - Bind the exact route and owned simulator generation before capture.
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
path, pid, route, town, port, quality = sys.argv[1:]
with Path(path).open('x') as stream:
    json.dump({'started_at_utc': datetime.now(timezone.utc).isoformat(), 'server_pid': int(pid),
        'server_pgid': int(pid), 'route_sha256': hashlib.sha256(Path(route).read_bytes()).hexdigest(),
        'map': town, 'port': int(port), 'quality': quality, 'learned_model_control': False}, stream, indent=2)
PY
# HH_260906 - Recheck after actual startup; refuse capture and clean only the owned server if the budget shrank.
check_finish_budget 120 || exit 2
setsid timeout --signal=INT --kill-after=15 "${wall_timeout}" \
  python3 "${worker_path}" "${output_root}/${worker_output_name}" "${route_file}" \
  --host 127.0.0.1 --port "${port}" "${collector_options[@]}" >"${output_root}/collector.log" 2>&1 &
collector_pid=$!
set +e
wait "${collector_pid}"
collector_status=$?
set -e
e2e_stop_owned_process_group "${collector_pid}" "${collector_pid}" 5 3 1 || exit 1
collector_pid=""
probe after_capture || exit 1
exit "${collector_status}"
