#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

usage() {
  cat <<'EOF'
Usage: scripts/e2e/run_carla_map.sh MAP [--port PORT] [--quality Low|Epic] [--startup-timeout-sec SEC] [-- CARLA_ARGS...]

Cold-start the packaged CARLA 0.9.15 build on MAP without client.load_world.
The shipping build ignores map command-line overrides, so this wrapper writes a
workspace-local generated Engine.ini and passes it with -EngineINI. The shared
packaged DefaultEngine.ini is never modified.

Example:
  scripts/e2e/run_carla_map.sh Town04 --port 2100 --quality Low -- -RenderOffScreen -nosound
  scripts/e2e/run_carla_map.sh C_track_1_0_7 --port 2100 --quality Epic -- -RenderOffScreen -nosound
EOF
}

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 2
fi
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi

map_name="$1"
shift
port=2100
quality="${CARLA_QUALITY:-Low}"
startup_timeout_sec=90
carla_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      [[ $# -ge 2 ]] || { echo "--port requires a value" >&2; exit 2; }
      port="$2"
      shift 2
      ;;
    --startup-timeout-sec)
      [[ $# -ge 2 ]] || { echo "--startup-timeout-sec requires a value" >&2; exit 2; }
      startup_timeout_sec="$2"
      shift 2
      ;;
    --quality)
      [[ $# -ge 2 ]] || { echo "--quality requires a value" >&2; exit 2; }
      quality="$2"
      shift 2
      ;;
    --)
      shift
      carla_args+=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown wrapper option: $1 (put CARLA options after --)" >&2
      exit 2
      ;;
  esac
done

if [[ ! "${map_name}" =~ ^[A-Za-z0-9_]+$ ]]; then
  echo "unsafe map name: ${map_name}" >&2
  exit 2
fi
if [[ ! "${port}" =~ ^[0-9]+$ ]] || (( port < 1 || port > 65535 )); then
  echo "port must be an integer in [1, 65535]" >&2
  exit 2
fi
if [[ ! "${startup_timeout_sec}" =~ ^[0-9]+$ ]] || (( startup_timeout_sec < 1 )); then
  echo "startup timeout must be a positive integer" >&2
  exit 2
fi
if [[ "${quality}" != "Low" && "${quality}" != "Epic" ]]; then
  echo "quality must be Low or Epic" >&2
  exit 2
fi

content_root="${CARLA_ROOT}/CarlaUE4/Content"
config_path="${CARLA_ROOT}/CarlaUE4/Config/DefaultEngine.ini"
mapfile -t level_paths < <(find "${content_root}" -type f -name "${map_name}.umap" -print)
if [[ "${#level_paths[@]}" -eq 0 ]]; then
  echo "packaged CARLA map not found: ${map_name}.umap" >&2
  exit 2
fi
if [[ "${#level_paths[@]}" -ne 1 ]]; then
  printf 'packaged CARLA map name is ambiguous: %s\n' "${level_paths[@]}" >&2
  exit 2
fi
level_path="${level_paths[0]}"
if [[ ! -f "${config_path}" ]]; then
  echo "packaged CARLA config not found: ${config_path}" >&2
  exit 2
fi
if pgrep -f "${CARLA_ROOT}/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" >/dev/null; then
  echo "packaged CARLA is already running; stop it before changing maps" >&2
  exit 2
fi

engine_ini_dir="${root}/data/generated/carla_engine_ini"
engine_ini_path="${engine_ini_dir}/${map_name}.Engine.ini"
mkdir -p "${engine_ini_dir}"
staged_path="$(mktemp "${engine_ini_dir}/.${map_name}.Engine.ini.staged.XXXXXX")"
child_pid=""

cleanup() {
  local status=$?
  if [[ -f "${staged_path}" ]]; then
    rm -f "${staged_path}"
  fi
  if [[ -n "${child_pid}" ]] && kill -0 "${child_pid}" 2>/dev/null; then
    kill -TERM "${child_pid}" 2>/dev/null || true
    wait "${child_pid}" 2>/dev/null || true
  fi
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

level_relative="${level_path#${content_root}/}"
level_package="/Game/${level_relative%.umap}"
map_path="${level_package}.${map_name}"
printf '%s\n' \
  '[/Script/EngineSettings.GameMapsSettings]' \
  "EditorStartupMap=${map_path}" \
  "GameDefaultMap=${map_path}" \
  "ServerDefaultMap=${map_path}" \
  "TransitionMap=${map_path}" >"${staged_path}"
replacement_count="$(grep -Ec "^(EditorStartupMap|GameDefaultMap|ServerDefaultMap|TransitionMap)=${map_path}$" "${staged_path}")"
if [[ "${replacement_count}" -ne 4 ]]; then
  echo "refusing to start: expected four generated map settings, got ${replacement_count}" >&2
  exit 2
fi
mv -f "${staged_path}" "${engine_ini_path}"

"${CARLA_ROOT}/CarlaUE4.sh" \
  -prefernvidia \
  -quality-level="${quality}" \
  -carla-port="${port}" \
  -EngineINI="${engine_ini_path}" \
  "${carla_args[@]}" &
child_pid=$!

python3 - "${port}" "${startup_timeout_sec}" "${map_name}" <<'PY'
import sys
import time

import carla

port = int(sys.argv[1])
timeout = int(sys.argv[2])
expected = sys.argv[3]
deadline = time.monotonic() + timeout
last_error = "CARLA did not answer"
while time.monotonic() < deadline:
    try:
        # A CARLA client can remain unusable after its first RPC timeout, so
        # create a fresh connection for every startup probe.
        client = carla.Client("127.0.0.1", port)
        client.set_timeout(2.0)
        world = client.get_world()
        actual = str(world.get_map().name).rstrip("/").rsplit("/", 1)[-1]
        if actual != expected:
            raise RuntimeError(f"server map mismatch: expected={expected} actual={actual}")
        print(
            f"CARLA_READY map={world.get_map().name} "
            f"spawns={len(world.get_map().get_spawn_points())} port={port}"
        )
        break
    except Exception as error:
        last_error = str(error)
        time.sleep(1.0)
else:
    raise SystemExit(f"CARLA startup failed: {last_error}")
PY

echo "CARLA_ENGINE_INI path=${engine_ini_path}"

set +e
wait "${child_pid}"
status=$?
set -e
child_pid=""
exit "${status}"
