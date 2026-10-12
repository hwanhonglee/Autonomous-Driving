#!/usr/bin/env bash
# HH_261001 - Own only the two pinned four-camera TRAIN-site appearance diagnostics.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source tools/e2e/env.sh
export PYTHONDONTWRITEBYTECODE=1
[[ "${CARLA_ROOT}" == /home/a/carla-autoware-universe/CARLA_0.9.15 ]] || { echo 'Pinned packaged runtime required.' >&2; exit 2; }
source tools/e2e/process_group_cleanup.sh
source tools/e2e/workspace_runtime_lock.sh

usage() {
  echo "Usage: run_owned_map_paired_rgb_20261001.sh OUTPUT_ROOT PINNED_TRAIN_ROUTE_JSON [--execute] [--port PORT] [--finish-before-utc YYYY-MM-DDTHH:MM:SSZ]"
}
if [[ $# -lt 2 ]]; then usage >&2; exit 2; fi
output_root="$(realpath -m -- "$1")"
route_file="$(realpath -- "$2")"
shift 2
port=2100
quality=Epic
wall_timeout=300
execute=0
finish_before_utc=""
capture_mode=mapping-paired-rgb
collector_options=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute) execute=1; shift ;;
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
[[ "${quality}" == Epic && "${capture_mode}" == mapping-paired-rgb && "${wall_timeout}" == 300 ]] || { echo "Paired capture requires fixed Epic/300s/named mode." >&2; exit 2; }
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
  expert-pedal-caps)
    # HH_260906 - Separate fixed pedal-cap research from the unchanged production/expert collector entrypoint.
    worker_module=scripts.e2e.collect_carla_pedal_caps_aj
    worker_path=scripts/e2e/collect_carla_pedal_caps_aj.py
    worker_output_name=episode ;;
  expert-goal-stop-ak)
    # HH_260906 - Archive a separate measured low-speed hold experiment without changing the frozen AJ worker.
    worker_module=scripts.e2e.collect_carla_goal_stop_ak
    worker_path=scripts/e2e/collect_carla_goal_stop_ak.py
    worker_output_name=episode ;;
  expert-frame-alignment-al)
    # HH_260906 - Separate rear-axle planner/PID frame experiment from the frozen AK worker.
    worker_module=scripts.e2e.collect_carla_frame_alignment_al
    worker_path=scripts/e2e/collect_carla_frame_alignment_al.py
    worker_output_name=episode ;;
  expert-low-speed-steering-an)
    # HH_260906 - Isolate one post-emergency normal-steering guard from the fixed AM trial.
    worker_module=scripts.e2e.collect_carla_low_speed_steering_an
    worker_path=scripts/e2e/collect_carla_low_speed_steering_an.py
    worker_output_name=episode ;;
  expert-launch-steering-ao)
    # HH_260906 - Archive a separate launch-only normal-steering factor on the fixed AN chain.
    worker_module=scripts.e2e.collect_carla_launch_steering_ao
    worker_path=scripts/e2e/collect_carla_launch_steering_ao.py
    worker_output_name=episode ;;
  mapping-paired-rgb)
    # HH_260906 - Isolate kinematic oracle mapping from expert driving and training data.
    worker_module=scripts.e2e.collect_map_paired_rgb_20261001
    worker_path=scripts/e2e/collect_map_paired_rgb_20261001.py
    worker_output_name=mapping ;;
  mapping-ctrack-v2)
    # HH_260929 - Own a separate, byte-pinned C-track sensor capture without changing frozen v1.
    worker_module=scripts.e2e.collect_carla_mapping_ctrack_v2
    worker_path=scripts/e2e/collect_carla_mapping_ctrack_v2.py
    worker_output_name=mapping ;;
  vehicle-geometry)
    # HH_260906 - Measure one physics-disabled Prius without collecting or driving an episode.
    worker_module=scripts.e2e.measure_prius_geometry_z
    worker_path=scripts/e2e/measure_prius_geometry_z.py
    worker_output_name=geometry ;;
  expert)
    worker_module=scripts.e2e.collect_carla_vad_expert
    worker_path=scripts/e2e/collect_carla_vad_expert.py
    worker_output_name=episode ;;
  actuation-response)
    worker_module=scripts.e2e.calibrate_carla_low_speed_response
    worker_path=scripts/e2e/calibrate_carla_low_speed_response.py
    worker_output_name=actuation ;;
  stationary-camera)
    # HH_260906 - Restrict material-quality checks to a named full-brake camera probe, never a driving worker.
    worker_module=scripts.e2e.probe_carla_stationary_camera_quality
    worker_path=scripts/e2e/probe_carla_stationary_camera_quality.py
    worker_output_name=camera_audit ;;
  stationary-motion)
    # HH_260906 - Isolate fixed held-brake micromotion measurements from driving and training episodes.
    worker_module=scripts.e2e.probe_carla_stationary_motion
    worker_path=scripts/e2e/probe_carla_stationary_motion.py
    worker_output_name=motion_probe ;;
  low-speed-coast)
    # HH_260906 - Keep bounded motion-history diagnostics separate from expert training and held-brake captures.
    worker_module=scripts.e2e.probe_carla_low_speed_coast
    worker_path=scripts/e2e/probe_carla_low_speed_coast.py
    worker_output_name=coast_probe ;;
  coast-brake-transition)
    # HH_260906 - Bound a declared low-speed brake transition without changing earlier coast observations.
    worker_module=scripts.e2e.probe_carla_coast_brake_transition
    worker_path=scripts/e2e/probe_carla_coast_brake_transition.py
    worker_output_name=brake_probe ;;
  *) usage >&2; exit 2 ;;
esac
if [[ -e "${output_root}" || -L "${output_root}" ]]; then
  echo "Refusing to overwrite an expert trial: ${output_root}" >&2
  exit 2
fi
moving_task_opt_in=0
for option in "${collector_options[@]}"; do
  case "${option}" in
    --help|-h)
      # HH_260906 - A help request must never become a successful preflight followed by a world launch.
      usage
      exit 0 ;;
    --host|--host=*|--port|--port=*|--allow-map-load)
      echo "Collector cannot redirect the owned simulator or load another world." >&2
      exit 2 ;;
    --wall-timing)
      # HH_260906 - Only the expert collector imports the optional timing helper.
      [[ "${capture_mode}" == expert || "${capture_mode}" == expert-pedal-caps || "${capture_mode}" == expert-goal-stop-ak || "${capture_mode}" == expert-frame-alignment-al || "${capture_mode}" == expert-low-speed-steering-an || "${capture_mode}" == expert-launch-steering-ao ]] || { echo "Wall timing is expert-only." >&2; exit 2; } ;;
    --moving-task-plan|--moving-task-plan=*|--moving-task-plan-sha256|--moving-task-plan-sha256=*)
      [[ "${capture_mode}" == expert ]] || { echo "Moving-task capture is expert-only." >&2; exit 2; }
      moving_task_opt_in=1 ;;
  esac
done
if (( moving_task_opt_in )) && [[ "${quality}" != Epic || "${wall_timeout}" != 300 ]]; then
  echo "Moving-task capture requires Epic quality and the declared 300-second owner budget." >&2
  exit 2
fi
# HH_260906 - Resolve the complete strict collector argv before starting any simulator.
python3 - "${worker_module}" "${output_root}/${worker_output_name}" "${route_file}" "${port}" "${capture_mode}" "${collector_options[@]}" <<'PY'
# HH_260906 - Configure explicit canonical imports without filesystem aliases.
import runpy
runpy.run_path('tools/workspace_python.py')['configure']()
import importlib
import sys
module, output, route, port, mode, *options = sys.argv[1:]
parse_args = importlib.import_module(module).parse_args
args = parse_args([output, route, '--host', '127.0.0.1', '--port', port, *options])
if args.host != '127.0.0.1' or args.port != int(port) or args.allow_map_load:
    raise SystemExit('collector argv escapes the owned simulator')
if (getattr(args, 'moving_task_plan', None) is not None
        or getattr(args, 'moving_task_plan_sha256', None) is not None) and mode != 'expert':
    raise SystemExit('moving-task opt-in is expert-only')
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
if (( ! execute )); then
  echo "PAIRED_APPEARANCE_PREFLIGHT_PASS map=${map_name} port=${port} frames=240 no_simulator_launched=true"
  exit 0
fi
e2e_acquire_workspace_runtime_lock "owned paired appearance collection"
python3 - <<'PY'
from pathlib import Path
for path in Path('/proc').iterdir():
    if not path.name.isdigit():
        continue
    try:
        argv=(path/'cmdline').read_bytes().split(bytes([0]))
        if argv and argv[0].endswith((b'/UE4Editor',b'/CarlaUE4-Linux-Shipping')):
            raise SystemExit('another simulator is already running')
    except (FileNotFoundError,ProcessLookupError,PermissionError):
        continue
PY
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
# HH_260906 - Retain logical provenance keys while reading canonical source bytes.
from pathlib import Path
from learning.portable_e2e.workspace_paths import source_path
# HH_260906 - Configure explicit canonical imports without filesystem aliases.
import runpy
runpy.run_path('tools/workspace_python.py')['configure']()
# HH_260906 - Preserve the source and command contract even when simulator startup fails.
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
path, route, town, port, quality, timeout, mode, worker, output_name, finish_before, *options = sys.argv[1:]
sources = ('scripts/e2e/run_owned_map_paired_rgb_20261001.sh',
    'scripts/e2e/audit_map_paired_rgb_20261001.py',
    'scripts/e2e/audit_ctrack_isolated_moving_20261001.py',
    'tools/workspace_python.py', 'tools/canonical_workspace.py',
    'portable_e2e/workspace_paths.py',
    'learning/tests/test_map_paired_rgb_20261001.py',
    'docs/assets/validation/2026-10-01/ctrack_external_maps/PAIRED_APPEARANCE_PLAN.md',
    'scripts/e2e/run_owned_carla_expert_trial.sh', 'scripts/e2e/run_carla_map.sh',
    'scripts/e2e/process_group_cleanup.sh', 'scripts/e2e/workspace_runtime_lock.sh',
    'scripts/e2e/probe_carla_server.py', 'scripts/e2e/env.sh',
    'scripts/e2e/collect_carla_vad_expert.py', 'scripts/e2e/carla_goal_stop_profile.py', worker,
    'portable_e2e/model.py', 'portable_e2e/runtime_contract.py')
if mode == 'expert-goal-stop-ak':
    # HH_260906 - The AK worker imports the frozen AJ pedal wrapper; archive both exact bytes.
    sources += ('scripts/e2e/collect_carla_pedal_caps_aj.py',)
if mode == 'expert-frame-alignment-al':
    # HH_260906 - AL imports the frozen AK and AJ wrappers; archive all three exact worker revisions.
    sources += ('scripts/e2e/collect_carla_goal_stop_ak.py',
                'scripts/e2e/collect_carla_pedal_caps_aj.py')
if mode == 'expert-low-speed-steering-an':
    # HH_260906 - AN imports AL/AK/AJ and must archive the complete scoped worker chain.
    sources += ('scripts/e2e/collect_carla_frame_alignment_al.py',
                'scripts/e2e/collect_carla_goal_stop_ak.py',
                'scripts/e2e/collect_carla_pedal_caps_aj.py')
if mode == 'expert-launch-steering-ao':
    # HH_260906 - AO imports AN/AL/AK/AJ and must archive the entire nested worker chain.
    sources += ('scripts/e2e/collect_carla_low_speed_steering_an.py',
                'scripts/e2e/collect_carla_frame_alignment_al.py',
                'scripts/e2e/collect_carla_goal_stop_ak.py',
                'scripts/e2e/collect_carla_pedal_caps_aj.py')
if mode == 'actuation-response':
    # HH_260906 - Archive the prospective coast/ramp contract together with its importing worker.
    sources += ('scripts/e2e/carla_low_speed_response_matrix.py',)
if mode in ('mapping-probe', 'mapping-ctrack-v2', 'mapping-paired-rgb'):
    # HH_260906 - Archive the exact live/replay fusion and projection implementation.
    sources += ('portable_e2e/mapping.py', 'portable_e2e/mapping_projection.py')
if mode == 'mapping-ctrack-v2':
    # HH_260929 - Bind V2 permission, both frozen parents and the duplicated v1 loop provenance.
    sources += ('scripts/e2e/site_split_guard_v2.py', 'config/site_split_guard_20260929_v2.json',
        'scripts/e2e/site_split_guard.py', 'config/site_split_guard_20260929.json',
        'scripts/e2e/collect_carla_mapping_probe.py')
if mode in ('stationary-motion', 'low-speed-coast', 'coast-brake-transition'):
    # HH_260906 - Archive every reused lock, cleanup, physics and camera helper imported by the new probe.
    sources += ('scripts/e2e/probe_carla_stationary_camera_quality.py',
                'scripts/e2e/calibrate_carla_low_speed_response.py',
                'scripts/e2e/carla_low_speed_response_matrix.py',
                'scripts/e2e/carla_physics_substeps.py')
if mode in ('low-speed-coast', 'coast-brake-transition'):
    # HH_260906 - The coast worker reuses immutable stationary observation helpers, not their held-brake run loop.
    sources += ('scripts/e2e/probe_carla_stationary_motion.py',)
if mode == 'coast-brake-transition':
    # HH_260906 - Bind the unchanged coast preflight/helpers imported by the separate transition worker.
    sources += ('scripts/e2e/probe_carla_low_speed_coast.py',)
# HH_260906 - The explicit timing revision adds one helper; historical ten-source expert archives remain exact.
wall_timing = mode in ('expert', 'expert-pedal-caps', 'expert-goal-stop-ak',
                       'expert-frame-alignment-al', 'expert-low-speed-steering-an',
                       'expert-launch-steering-ao') and '--wall-timing' in options
if wall_timing:
    sources += ('scripts/e2e/carla_wall_timing.py',)
# HH_260906 - Parse the exact expert argv again so both flag syntaxes archive only an explicitly selected helper.
substep_profile = 'inherited'
if mode == 'expert' and any(option == '--physics-substep-profile' or option.startswith('--physics-substep-profile=') for option in options):
    from scripts.e2e.collect_carla_vad_expert import parse_args
    substep_profile = parse_args([str(Path(path).parent / output_name), route, '--host', '127.0.0.1', '--port', port, *options]).physics_substep_profile
if substep_profile != 'inherited':
    sources += ('scripts/e2e/carla_physics_substeps.py',)
# HH_260906 - Only the new opt-in archives a prospective task declaration and its exact input dependencies.
moving_task = None
moving_document = None
if mode == 'expert' and any(option in ('--moving-task-plan', '--moving-task-plan-sha256')
        or option.startswith(('--moving-task-plan=', '--moving-task-plan-sha256=')) for option in options):
    from scripts.e2e.collect_carla_vad_expert import moving_capture_module, parse_args
    args = parse_args([str(Path(path).parent / output_name), route, '--host', '127.0.0.1', '--port', port, *options])
    moving_document = moving_capture_module(args).configuration_from_args(args)
    if moving_document['owner_wall_timeout_sec'] != int(timeout):
        raise SystemExit('moving owner budget differs from its prospective declaration')
    declared = datetime.fromisoformat(moving_document['declared_at_utc'].replace('Z', '+00:00'))
    if declared.tzinfo is None or declared > datetime.now(timezone.utc):
        raise SystemExit('moving task must be declared before its owned launch')
    sources += ('scripts/e2e/carla_moving_capture.py', 'scripts/e2e/carla_physics_substeps.py')
    def archive_input(source, expected, relative):
        source = Path(source)
        if ('..' in source.parts or source.is_symlink() or not source.is_file() or any(p.is_symlink() for p in source.parents)
                or source.stat().st_size > 16 * 1024 * 1024):
            raise SystemExit('moving input must be a bounded regular file')
        payload = source.read_bytes()
        if len(payload) > 16 * 1024 * 1024 or hashlib.sha256(payload).hexdigest() != expected:
            raise SystemExit('moving input changed before source archival')
        archived = Path(path).parent / relative
        archived.parent.mkdir(parents=True, exist_ok=True)
        with archived.open('xb') as stream:
            stream.write(payload)
        return {'path': str(source.absolute()), 'sha256': expected, 'archived_path': relative}
    plan_ref = archive_input(args.moving_task_plan, args.moving_task_plan_sha256, 'provenance/moving_task_plan.json')
    input_refs = {}
    if set(moving_document['input_paths']) != {'route_file', 'mapping', 'calibration'} or set(moving_document['input_sha256']) != set(moving_document['input_paths']):
        raise SystemExit('moving task input closure differs')
    for name, source in moving_document['input_paths'].items():
        input_refs[name] = archive_input(source, moving_document['input_sha256'][name], 'provenance/moving_inputs/' + name)
    moving_task = {'schema': 'carla.moving_task_capture_plan.v1', 'declared_at_utc': moving_document['declared_at_utc'],
        'plan': plan_ref, 'inputs': input_refs, 'source_and_input_bytes_archived': True,
        'completion_contract': 'task_complete_and_PENDING_QA_not_catalog_goal_or_dataset_admission'}
def git(*args):
    return subprocess.run(['git', *args], check=True, capture_output=True,
        text=True, timeout=15).stdout.strip()
# HH_260906 - Keep exact source bytes privately; a prelaunch hash alone cannot replay an uncommitted revision.
source_hashes = {}
for name in dict.fromkeys(sources):
    payload = source_path(Path.cwd(), name).read_bytes()
    archived = Path(path).parent / 'provenance' / name
    archived.parent.mkdir(parents=True, exist_ok=True)
    with archived.open('xb') as stream:
        stream.write(payload)
    source_hashes[name] = hashlib.sha256(payload).hexdigest()
external_sources = {}
if mode == 'mapping-paired-rgb':
    from scripts.e2e.collect_map_paired_rgb_20261001 import PACKAGE
    for source in sorted((PACKAGE/'PythonAPI/carla/agents').rglob('*.py')):
        relative = 'provenance/carla_pythonapi/'+str(source.relative_to(PACKAGE/'PythonAPI/carla'))
        payload = source.read_bytes()
        archived = Path(path).parent/relative
        archived.parent.mkdir(parents=True,exist_ok=True)
        with archived.open('xb') as stream:
            stream.write(payload)
        external_sources[str(source.relative_to(PACKAGE))] = {
            'path':str(source),'sha256':hashlib.sha256(payload).hexdigest(),'archived_path':relative}
if mode in ('expert-frame-alignment-al', 'expert-low-speed-steering-an', 'expert-launch-steering-ao'):
    # HH_260906 - Archive AL's pinned getters and AN/AO's pinned BasicAgent emergency override.
    import os
    if mode == 'expert-launch-steering-ao':
        from scripts.e2e.collect_carla_launch_steering_ao import UPSTREAM_SOURCE_SHA256
    elif mode == 'expert-low-speed-steering-an':
        from scripts.e2e.collect_carla_low_speed_steering_an import UPSTREAM_SOURCE_SHA256
    else:
        from scripts.e2e.collect_carla_frame_alignment_al import UPSTREAM_SOURCE_SHA256
    carla_root = os.environ.get('CARLA_ROOT')
    if not carla_root:
        raise SystemExit('AL requires an explicit CARLA_ROOT for source provenance')
    source_root = Path(carla_root).resolve(strict=True) / 'PythonAPI/carla/agents/navigation'
    for filename, digest in UPSTREAM_SOURCE_SHA256.items():
        source = source_root / filename
        if source.is_symlink() or not source.is_file():
            raise SystemExit('AL CARLA PythonAPI source is not a regular file')
        payload = source.read_bytes()
        if hashlib.sha256(payload).hexdigest() != digest:
            raise SystemExit('AL CARLA PythonAPI source differs from pinned controller contract')
        relative = 'provenance/carla_pythonapi/agents/navigation/' + filename
        archived = Path(path).parent / relative
        archived.parent.mkdir(parents=True, exist_ok=True)
        with archived.open('xb') as stream:
            stream.write(payload)
        external_sources[filename] = {'path': str(source), 'sha256': digest,
                                      'archived_path': relative}
if moving_document is not None and any(source_hashes.get(name) != expected for name, expected in moving_document['source_sha256'].items()):
    raise SystemExit('moving declared source differs from owned source archive')
with Path(path).open('x') as stream:
    json.dump({'planned_at_utc': datetime.now(timezone.utc).isoformat(),
        'schema': 'portable_e2e.owned_expert_trial.v1',
        'source_head_commit': git('rev-parse', 'HEAD'),
        'source_worktree_status': git('status', '--porcelain', '--untracked-files=all'),
        # HH_260906 - Preserve the exact bound definitions so later unrelated model edits cannot invalidate historical evidence.
        'source_sha256': source_hashes, 'source_bytes_archived': True,
        'bounds_source_bytes_archived': True,
        **({'external_source_provenance': external_sources,
            'external_source_bytes_archived': True} if external_sources else {}),
        **({'wall_timing_enabled': True, 'wall_timing_source_bytes_archived': True,
            'wall_timing_schema': 'carla.expert_wall_timing.v1'} if wall_timing else {}),
        **({'physics_substeps_source_bytes_archived': True, 'physics_substep_profile': substep_profile,
            'physics_substeps_schema': 'carla.physics_substep_experiment.v1'} if substep_profile != 'inherited' else {}),
        **({'moving_task_capture': moving_task} if moving_task is not None else {}),
        'route_path': route, 'route_sha256': hashlib.sha256(Path(route).read_bytes()).hexdigest(),
        'map': town, 'host': '127.0.0.1', 'port': int(port), 'quality': quality,
        'capture_mode': mode, 'worker_path': worker,
        'collector_wall_timeout_sec': int(timeout), 'server_startup_timeout_sec': 180,
        'finish_before_utc': finish_before or None,
        'finish_budget_policy': {'prelaunch_overhead_sec': 330, 'precapture_overhead_sec': 120,
            'notice': 'Admission budget checks, not a hard real-time OS guarantee; no unrelated processes are signaled.'},
        'collector_argv': [str(Path(path).parent / output_name), route, '--host', '127.0.0.1', '--port', port, *options],
        'server_extra_options': ['-RenderOffScreen', '-nosound'],
        'learned_model_control': False, 'vehicle_control_approved': False,
        'intended_role':'known_train_routes_paired_appearance_diagnostic',
        'dataset_admitted':False,'train_eligible':False,'ctrack_train_eligible':False,
        **({'training_data_admitted': False, 'independent_dev_approved': False,
            'test_use_authorized': False} if mode == 'mapping-ctrack-v2' else {})}, stream, indent=2)
PY
server_pid=""
collector_pid=""
cleanup_status=0
probe() {
  local stage="$1"
  shift
  python3 tools/workspace_python.py tools/e2e/probe_carla_server.py \
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
# HH_260906 - Retain logical provenance keys while reading canonical source bytes.
from pathlib import Path
from learning.portable_e2e.workspace_paths import source_path
# HH_260906 - Preserve exit status independently of whether the expert route or comfort checks passed.
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
plan = json.loads(Path(sys.argv[1]).with_name('owner_plan.json').read_text())
source_checks = {}
for name, expected in plan['source_sha256'].items():
    source = source_path(Path.cwd(), name)
    archived = Path(sys.argv[1]).parent / 'provenance' / name
    try:
        source_checks[name] = source.is_file() and archived.is_file() and all(
            hashlib.sha256(p.read_bytes()).hexdigest() == expected for p in (source, archived))
    except OSError:
        source_checks[name] = False
external_checks = {}
for name, ref in plan.get('external_source_provenance', {}).items():
    paths = (Path(ref['path']), Path(sys.argv[1]).parent / ref['archived_path'])
    try:
        external_checks[name] = all(p.is_file() and not p.is_symlink()
            and hashlib.sha256(p.read_bytes()).hexdigest() == ref['sha256'] for p in paths)
    except OSError:
        external_checks[name] = False
all_sources_unchanged = all(source_checks.values()) and all(external_checks.values())
final_status = int(sys.argv[2]) if all_sources_unchanged else 1
moving_proof = None
if 'moving_task_capture' in plan:
    # HH_260906 - Pending candidate completion is distinct from goal success and cannot grant dataset admission.
    refs = {'plan': plan['moving_task_capture']['plan'], **plan['moving_task_capture']['inputs']}
    checks = {}
    for name, ref in refs.items():
        paths = (Path(ref['path']), Path(sys.argv[1]).parent / ref['archived_path'])
        try:
            checks[name] = all('..' not in p.parts and p.is_file() and not p.is_symlink() and not any(parent.is_symlink() for parent in p.parents)
                and hashlib.sha256(p.read_bytes()).hexdigest() == ref['sha256'] for p in paths)
        except OSError:
            checks[name] = False
    manifest_path = Path(plan['collector_argv'][0]) / 'manifest.json'
    manifest, result, manifest_sha = {}, {}, None
    try:
        if manifest_path.is_symlink() or not manifest_path.is_file() or manifest_path.stat().st_size > 16 * 1024 * 1024:
            raise ValueError('missing or unsafe final candidate manifest')
        payload = manifest_path.read_bytes()
        if len(payload) > 16 * 1024 * 1024:
            raise ValueError('candidate manifest grew past its bound')
        manifest = json.loads(payload)
        result = manifest.get('result', {})
        manifest_sha = hashlib.sha256(payload).hexdigest()
    except (OSError, ValueError, AttributeError):
        manifest, result = {}, {}
    complete = (isinstance(result, dict) and manifest.get('status') == 'complete'
        and isinstance(manifest.get('cleanup'), dict) and manifest['cleanup'].get('completed') is True
        and result.get('task_complete') is True and result.get('goal_reached') is False
        and result.get('admission_status') == 'PENDING_QA' and result.get('dataset_conversion_performed') is False
        and all(key not in row for row in (manifest, result) for key in ('training_data_approved', 'development_only')))
    moving_proof = {'source_plan_and_inputs_unchanged_and_archived': all(checks.values()), 'input_checks': checks,
        'final_manifest_path': str(manifest_path), 'final_manifest_sha256': manifest_sha,
        'capture_complete_pending_qa': complete, 'dataset_admission_granted': False,
        'observed_dataset_conversion_performed': result.get('dataset_conversion_performed') if isinstance(result, dict) else None}
    if not all(checks.values()) or (final_status == 0 and not complete):
        final_status = 1
with Path(sys.argv[1]).open('x') as stream:
    json.dump({'completed_at_utc': datetime.now(timezone.utc).isoformat(), 'exit_code': final_status,
        'capture_mode': sys.argv[3],
        'source_bytes_unchanged_and_archived': all_sources_unchanged, 'source_checks': source_checks,
        'external_source_checks': external_checks,
        **({'moving_task_capture': moving_proof} if moving_proof is not None else {}),
        'learned_model_control': False, 'vehicle_control_approved': False,
        'intended_role':'known_train_routes_paired_appearance_diagnostic',
        'dataset_admitted':False,'train_eligible':False,'ctrack_train_eligible':False,
        **({'training_data_admitted': False, 'independent_dev_approved': False,
            'test_use_authorized': False} if sys.argv[3] == 'mapping-ctrack-v2' else {})}, stream, indent=2)
raise SystemExit(0 if all_sources_unchanged and (moving_proof is None or final_status == int(sys.argv[2])) else 1)
PY
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
check_finish_budget 330 || exit 2
setsid bash tools/e2e/run_carla_map.sh "${map_name}" --port "${port}" --quality "${quality}" \
  --startup-timeout-sec 180 -- -RenderOffScreen -nosound >"${output_root}/server.log" 2>&1 &
server_pid=$!
deadline=$((SECONDS + 195))
while (( SECONDS < deadline )); do
  # HH_260906 - Owned user services have no editor-bundled rg; read the binary-tainted CARLA log with system grep.
  if grep -a -q '^CARLA_READY ' "${output_root}/server.log"; then break; fi
  kill -0 "${server_pid}" 2>/dev/null || break
  sleep 1
done
grep -a -q '^CARLA_READY ' "${output_root}/server.log" || {
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
if (( moving_task_opt_in )); then
  python3 - "${output_root}/owner_plan.json" <<'PY'
# HH_260906 - Refuse changed task/source bytes after owned startup but before starting the collector child.
import hashlib
import json
from pathlib import Path
import sys
path = Path(sys.argv[1])
plan = json.loads(path.read_text())
refs = [plan['moving_task_capture']['plan'], *plan['moving_task_capture']['inputs'].values()]
from learning.portable_e2e.workspace_paths import source_path
refs += [{'path': str(source_path(Path.cwd(), name)), 'sha256': digest, 'archived_path': 'provenance/' + name}
         for name, digest in plan['source_sha256'].items()]
for ref in refs:
    for source in (Path(ref['path']), path.parent / ref['archived_path']):
        if ('..' in source.parts or source.is_symlink() or not source.is_file() or any(p.is_symlink() for p in source.parents)
                or hashlib.sha256(source.read_bytes()).hexdigest() != ref['sha256']):
            raise SystemExit('moving source/input changed before collector launch')
PY
fi
# HH_260906 - Preserve the preflight module package so collector-relative imports also resolve at execution.
setsid timeout --signal=INT --kill-after=15 "${wall_timeout}" \
  python3 tools/workspace_python.py -m "${worker_module}" -- "${output_root}/${worker_output_name}" "${route_file}" \
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
