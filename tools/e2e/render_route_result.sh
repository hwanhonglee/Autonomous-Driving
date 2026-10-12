#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"

exec python3 tools/workspace_python.py tools/e2e/render_route_result.py "$@"
