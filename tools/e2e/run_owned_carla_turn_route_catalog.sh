#!/usr/bin/env bash
# HH_260906 - Own exactly one packaged-Town long-turn catalog search without collecting an episode.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"

usage() {
  echo "Usage: tools/e2e/run_owned_carla_turn_route_catalog.sh town01|town01_v5|town01_multi|town03|town05_opt [--inspect-catalog-argv]";
}
if [[ $# -eq 1 && ( "$1" == "--help" || "$1" == "-h" ) ]]; then
  usage
  exit 0
fi
inspect_catalog_argv=0
if [[ $# -eq 2 && "$2" == "--inspect-catalog-argv" ]]; then
  inspect_catalog_argv=1
  set -- "$1"
fi
if [[ $# -ne 1 ]]; then
  usage >&2
  exit 2
fi
map_id="$1"
preferred_distance=210
search_mode=isolated_turn
case "${map_id}" in
  town01)
    town=Town01
    role=TRAIN
    seeds=2,3
    # HH_260906 - Preserve V1 import, V2 signal, and V3 service PATH failures in fresh paths.
    output_root="${root}/storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4" ;;
  town01_v5)
    # HH_260906 - Search a different endpoint chord without relaxing serialized route geometry or fixed QA bounds.
    map_id=town01
    town=Town01
    role=TRAIN_CANDIDATE
    seeds=4,5
    preferred_distance=160
    output_root="${root}/storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v5" ;;
  town01_multi)
    # HH_260906 - Search new right-only multi-maneuver routes without applying a whole-route isolated-turn gate.
    map_id=town01
    town=Town01
    role=TRAIN_CANDIDATE
    seeds=6,7
    preferred_distance=160
    search_mode=multi_maneuver_right
    output_root="${root}/storage/experiments/training/2026-09-27/drive_long_route_town01_multi_train_catalog_v1" ;;
  town03)
    town=Town03
    role=DEV
    seeds=1,2
    output_root="${root}/storage/experiments/validation/2026-09-27/drive_long_route_town03_dev_catalog_v1" ;;
  town05_opt)
    # HH_260906 - Keep Town05_Opt as a provisional integrated DEV site; the older global manifest still calls it TRAIN.
    town=Town05_Opt
    role=PROVISIONAL_DEV
    seeds=1,2
    output_root="${root}/storage/experiments/validation/2026-09-27/drive_long_route_town05_opt_dev_catalog_v1" ;;
  *)
    usage >&2
    exit 2 ;;
esac

# HH_260906 - Reject an existing catalog even if it is incomplete or a symlink.
if (( inspect_catalog_argv == 0 )) && [[ -e "${output_root}" || -L "${output_root}" ]]; then
  echo "Refusing to overwrite catalog: ${output_root}" >&2
  exit 2
fi

source tools/e2e/env.sh
source tools/e2e/process_group_cleanup.sh
source tools/e2e/workspace_runtime_lock.sh

port=2100
quality=Epic
catalog_timeout_sec=1200
catalog_args=(
  --map-id "${map_id}" --active-server-profile packaged_0915
  --host 127.0.0.1 --port "${port}" --map-load-settle-sec 0
  --output-root "${output_root}"
  --scenarios left,right --seeds "${seeds}" --pairs-per-seed 8
  --min-distance 200 --max-distance 230 --preferred-distance "${preferred_distance}"
  --sampling-resolution 1 --physical-turn-profile speed_30kph
  --max-traces 100000
)
if [[ "${search_mode}" == "multi_maneuver_right" ]]; then
  # HH_260906 - Keep the preregistered 240-300 m search separate from the existing 200-230 m isolated-turn argv.
  catalog_args=(
    --map-id town01 --active-server-profile packaged_0915
    --host 127.0.0.1 --port "${port}" --map-load-settle-sec 0
    --output-root "${output_root}"
    --scenarios right --seeds 6,7 --pairs-per-seed 8
    --min-distance 240 --max-distance 300 --preferred-distance 160
    --sampling-resolution 1 --max-traces 100000
  )
fi

# HH_260906 - Parse the immutable search argv before creating output or launching CARLA.
python3 - "${search_mode}" "${catalog_args[@]}" <<'PY'
import runpy
import sys
runpy.run_path('tools/workspace_python.py')['configure']()
from scripts.e2e.prepare_carla_expert_route_catalog import (
    TURN_GEOMETRY_ARGUMENTS,
    _load_runtime_modules,
    parse_args,
)
mode = sys.argv[1]
args = parse_args(sys.argv[2:])
if args.allow_map_load or args.endpoint_waypoint_spacing_m is not None:
    raise SystemExit('catalog-only map safety contract changed')
if mode == 'multi_maneuver_right':
    if (args.map_id != 'town01' or args.active_server_profile != 'packaged_0915'
            or args.host != '127.0.0.1' or args.port != 2100
            or args.map_load_settle_sec != 0 or args.scenarios != ('right',)
            or args.seeds != (6, 7) or args.pairs_per_seed != 8
            or args.min_distance != 240 or args.max_distance != 300
            or args.preferred_distance != 160 or args.sampling_resolution != 1
            or args.max_traces != 100000
            or args.physical_turn_profile is not None
            or args.physical_straight_profile is not None
            or any(getattr(args, name) is not None for name in TURN_GEOMETRY_ARGUMENTS)):
        raise SystemExit('catalog-only multi-maneuver contract changed')
elif mode == 'isolated_turn':
    if args.physical_turn_profile != 'speed_30kph' or args.scenarios != ('left', 'right'):
        raise SystemExit('catalog-only turn contract changed')
else:
    raise SystemExit('unsupported catalog search mode')
carla, helper = _load_runtime_modules()
if carla.__name__ != 'carla' or helper.__name__ != 'scripts.e2e.prepare_carla_route':
    raise SystemExit('catalog runtime helper did not resolve from the canonical tools namespace')
PY
if (( inspect_catalog_argv != 0 )); then
  printf '%s\0' "${catalog_args[@]}"
  exit 0
fi

e2e_acquire_workspace_runtime_lock "owned long-turn catalog search"
python3 - "${port}" <<'PY'
# HH_260906 - Occupied CARLA ports belong to someone else and are never killed.
import socket
import sys
for port in range(int(sys.argv[1]), int(sys.argv[1]) + 3):
    with socket.socket() as probe:
        try:
            probe.bind(('127.0.0.1', port))
        except OSError:
            raise SystemExit(f'CARLA port {port} is already occupied')
PY
if [[ -e "${output_root}" || -L "${output_root}" ]]; then
  echo "Catalog output appeared while waiting for lock: ${output_root}" >&2
  exit 2
fi
mkdir -p "$(dirname "${output_root}")"
mkdir -- "${output_root}"
mkdir -- "${output_root}/lifecycle" "${output_root}/provenance"

python3 - "${output_root}" "${map_id}" "${town}" "${role}" "${search_mode}" "${port}" "${quality}" "${catalog_timeout_sec}" "${catalog_args[@]}" <<'PY'
# HH_260906 - Archive exact source bytes and fixed argv before the owned map starts.
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
output, map_id, town, role, search_mode, port, quality, timeout, *args = sys.argv[1:]
root = Path(output)
sources = (
    'tools/e2e/run_owned_carla_turn_route_catalog.sh',
    'tools/e2e/prepare_carla_expert_route_catalog.py',
    'tools/e2e/prepare_carla_route.py',
    'tools/e2e/physical_turn_geometry.py',
    'tools/e2e/inventory_carla_training_maps.py',
    'tools/e2e/carla_expert_suite.yaml',
    'tools/e2e/run_carla_map.sh',
    'tools/e2e/probe_carla_server.py',
    'tools/e2e/process_group_cleanup.sh',
    'tools/e2e/workspace_runtime_lock.sh',
    'tools/e2e/env.sh',
    'tools/workspace_python.py',
    'tools/canonical_workspace.py',
)
hashes = {}
for name in sources:
    source = Path(name)
    payload = source.read_bytes()
    archived = root / 'provenance' / name
    archived.parent.mkdir(parents=True, exist_ok=True)
    with archived.open('xb') as stream:
        stream.write(payload)
    hashes[name] = hashlib.sha256(payload).hexdigest()
commit = subprocess.run(['git', 'rev-parse', 'HEAD'], check=True,
    capture_output=True, text=True, timeout=15).stdout.strip()
plan = {
    'schema': 'portable_e2e.owned_turn_route_catalog.v1',
    'planned_at_utc': datetime.now(timezone.utc).isoformat(),
    'source_head_commit': commit,
    'source_sha256': hashes,
    'source_bytes_archived': True,
    'map_id': map_id,
    'map': town,
    'split_intent': role,
    'search_mode': search_mode,
    'host': '127.0.0.1',
    'port': int(port),
    'quality': quality,
    'catalog_timeout_sec': int(timeout),
    'catalog_argv': args,
    'allow_map_load': False,
    'capture_executed': False,
    'training_executed': False,
    'dataset_admission_granted': False,
    'learned_model_control': False,
}
with (root / 'owner_plan.json').open('x', encoding='utf-8') as stream:
    json.dump(plan, stream, indent=2)
    stream.write('\n')
PY

server_pid=""
catalog_pid=""
cleanup_status=0
probe() {
  local stage="$1"
  shift
  python3 tools/workspace_python.py tools/e2e/probe_carla_server.py \
    --host 127.0.0.1 --port "${port}" --timeout 4 --expected-map "${town}" \
    --owner-pid "${server_pid}" --owner-pgid "${server_pid}" \
    --generation-id "turn_catalog_${server_pid}" --stage "${stage}" \
    --server-log "${output_root}/server.log" \
    --output "${output_root}/lifecycle/${stage}.json" "$@"
}
cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  if [[ -n "${catalog_pid}" ]]; then
    e2e_stop_owned_process_group "${catalog_pid}" "${catalog_pid}" 20 5 2 || cleanup_status=1
  fi
  if [[ -n "${server_pid}" ]]; then
    e2e_stop_owned_process_group "${server_pid}" "${server_pid}" 20 5 2 || cleanup_status=1
    probe stopped --expect-stopped || cleanup_status=1
  fi
  if (( cleanup_status != 0 )); then status=1; fi
  python3 - "${output_root}" "${status}" <<'PY'
# HH_260906 - A complete catalog is only a candidate search, never dataset admission.
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
root = Path(sys.argv[1])
status = int(sys.argv[2])
plan = json.loads((root / 'owner_plan.json').read_text())
source_checks = {}
for name, expected in plan['source_sha256'].items():
    try:
        source_checks[name] = all(
            path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == expected
            for path in (Path(name), root / 'provenance' / name)
        )
    except OSError:
        source_checks[name] = False
catalog_file = root / 'route_catalog.json'
catalog = None
catalog_sha = None
try:
    if catalog_file.is_file() and not catalog_file.is_symlink():
        payload = catalog_file.read_bytes()
        catalog_sha = hashlib.sha256(payload).hexdigest()
        catalog = json.loads(payload)
except (OSError, ValueError):
    catalog = None
right = []
if isinstance(catalog, dict):
    right = [item for item in catalog.get('routes', ()) if item.get('scenario') == 'right']
    if (catalog.get('status') != 'complete' or catalog.get('map_id') != plan['map_id']
            or catalog.get('server', {}).get('map_load_performed') is not False
            or not right):
        status = 1
    elif plan['search_mode'] == 'multi_maneuver_right':
        if (any(item.get('scenario') != 'right' for item in catalog.get('routes', ()))
                or any('physical_turn_preflight' in item for item in right)):
            status = 1
    elif any(item.get('physical_turn_preflight', {}).get('status') != 'PASS' for item in right):
        status = 1
else:
    status = 1
lifecycle = {}
for stage in ('ready', 'after_catalog', 'stopped'):
    path = root / 'lifecycle' / f'{stage}.json'
    try:
        lifecycle[stage] = json.loads(path.read_text())
    except (OSError, ValueError):
        lifecycle[stage] = None
if not all(source_checks.values()) or any(item is None for item in lifecycle.values()):
    status = 1
result = {
    'schema': 'portable_e2e.owned_turn_route_catalog_result.v1',
    'completed_at_utc': datetime.now(timezone.utc).isoformat(),
    'exit_code': status,
    'source_bytes_unchanged_and_archived': all(source_checks.values()),
    'source_checks': source_checks,
    'catalog_path': str(catalog_file),
    'catalog_sha256': catalog_sha,
    'catalog_status': catalog.get('status') if isinstance(catalog, dict) else None,
    'right_candidate_count': len(right),
    'lifecycle_receipts_present': all(item is not None for item in lifecycle.values()),
    'capture_executed': False,
    'training_executed': False,
    'dataset_admission_granted': False,
    'learned_model_control': False,
}
with (root / 'owner_result.json').open('x', encoding='utf-8') as stream:
    json.dump(result, stream, indent=2)
    stream.write('\n')
raise SystemExit(status)
PY
  exit $?
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

# HH_260906 - Cold-start only the declared packaged world; never call client.load_world.
setsid bash tools/e2e/run_carla_map.sh "${town}" --port "${port}" --quality "${quality}" \
  --startup-timeout-sec 180 -- -RenderOffScreen -nosound >"${output_root}/server.log" 2>&1 &
server_pid=$!
deadline=$((SECONDS + 195))
while (( SECONDS < deadline )); do
  # HH_260906 - User services lack the editor-bundled rg; grep -a handles CARLA's binary-tainted log.
  if grep -a -q '^CARLA_READY ' "${output_root}/server.log"; then break; fi
  kill -0 "${server_pid}" 2>/dev/null || break
  sleep 1
done
grep -a -q '^CARLA_READY ' "${output_root}/server.log" || {
  echo "Owned CARLA did not become ready; retaining server.log." >&2
  exit 1
}
probe ready
python3 - "${output_root}/owner_started.json" "${server_pid}" "${town}" "${port}" <<'PY'
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
path, pid, town, port = sys.argv[1:]
with Path(path).open('x', encoding='utf-8') as stream:
    json.dump({'started_at_utc': datetime.now(timezone.utc).isoformat(),
        'server_pid': int(pid), 'server_pgid': int(pid), 'map': town,
        'port': int(port), 'capture_executed': False}, stream, indent=2)
    stream.write('\n')
PY

# HH_260906 - Time-bound the one fixed catalog worker and retain every failure for audit.
python3 - "${output_root}/owner_plan.json" <<'PY'
# HH_260906 - Refuse to search if an archived implementation changed during CARLA startup.
import hashlib
import json
from pathlib import Path
import sys
path = Path(sys.argv[1])
plan = json.loads(path.read_text())
for name, expected in plan['source_sha256'].items():
    for source in (Path(name), path.parent / 'provenance' / name):
        if not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest() != expected:
            raise SystemExit(f'catalog source changed before worker start: {name}')
PY
setsid timeout --signal=INT --kill-after=15 "${catalog_timeout_sec}" \
  python3 tools/workspace_python.py -m scripts.e2e.prepare_carla_expert_route_catalog -- \
  "${catalog_args[@]}" >"${output_root}/catalog.log" 2>&1 &
catalog_pid=$!
set +e
wait "${catalog_pid}"
catalog_status=$?
set -e
e2e_stop_owned_process_group "${catalog_pid}" "${catalog_pid}" 5 3 1 || exit 1
catalog_pid=""
probe after_catalog || exit 1
exit "${catalog_status}"
