#!/usr/bin/env bash
# HH_261001 - Prepare the pinned CARLA environment for the scoped paired map owner.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source tools/e2e/env.sh
export PYTHONDONTWRITEBYTECODE=1
exec python3 tools/workspace_python.py -m scripts.e2e.run_owned_map_paired_town01_p02_p04_20261001 -- "$@"
