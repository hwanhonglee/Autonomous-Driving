#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
contract="${root}/scripts/e2e/b2d_vad_training_contract.py"

# The Python contract defaults to a read-only check. Installation, network use,
# and creation of .venv-b2d happen only when the caller passes --install.
if [[ $# -eq 0 ]]; then
  set -- --check
fi

exec python3 "${contract}" "$@"
