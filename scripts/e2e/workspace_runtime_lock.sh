#!/usr/bin/env bash

# Source this helper and call e2e_acquire_workspace_runtime_lock before reading
# or mutating shared src/build/install state for a live campaign or build.
e2e_acquire_workspace_runtime_lock() {
  local lock_label="${1:-workspace operation}"
  local lock_root
  local lock_path
  local inherited_fd="${AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD:-}"
  local inherited_target=""

  lock_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
  lock_path="${lock_root}/data/locks/autoware_e2e_runtime.lock"
  mkdir -p "$(dirname "${lock_path}")"

  if [[ "${inherited_fd}" =~ ^[0-9]+$ && -e "/proc/$$/fd/${inherited_fd}" ]]; then
    inherited_target="$(readlink -f "/proc/$$/fd/${inherited_fd}")"
    if [[ "${inherited_target}" == "${lock_path}" ]]; then
      return 0
    fi
    echo "Invalid inherited Autoware E2E workspace lock descriptor." >&2
    return 1
  fi

  if ! command -v flock >/dev/null 2>&1; then
    echo "flock is required to protect shared Autoware E2E state." >&2
    return 1
  fi
  exec {workspace_runtime_lock_fd}>"${lock_path}"
  if ! flock -n "${workspace_runtime_lock_fd}"; then
    echo "Another build or matrix owns shared Autoware E2E state; ${lock_label} cannot start." >&2
    return 1
  fi
  export AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD="${workspace_runtime_lock_fd}"
}
