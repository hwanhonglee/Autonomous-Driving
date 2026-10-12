#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source tools/e2e/env.sh

exec python3 tools/workspace_python.py tools/e2e/run_route_suite.py "$@"
