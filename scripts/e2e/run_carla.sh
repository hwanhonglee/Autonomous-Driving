#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

if [[ ! -x "${CARLA_ROOT}/CarlaUE4.sh" ]]; then
  echo "CARLA executable not found: ${CARLA_ROOT}/CarlaUE4.sh" >&2
  exit 1
fi

exec "${CARLA_ROOT}/CarlaUE4.sh" -prefernvidia -quality-level="${CARLA_QUALITY:-Low}" "$@"
