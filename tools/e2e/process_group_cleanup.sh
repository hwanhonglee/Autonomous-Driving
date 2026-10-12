#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this file from an owning helper; do not execute it directly." >&2
  exit 2
fi

e2e_process_group_exists() {
  local pgid="${1:-}"
  [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] || return 1
  kill -0 -- "-${pgid}" 2>/dev/null
}

e2e_wait_for_process_group_exit() {
  local pgid="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))

  while e2e_process_group_exists "${pgid}" && (( SECONDS < deadline )); do
    sleep 0.2
  done
  ! e2e_process_group_exists "${pgid}"
}

e2e_stop_owned_process_group() {
  local pgid="${1:-}"
  local leader_pid="${2:-}"
  local interrupt_timeout_sec="${3:-30}"
  local terminate_timeout_sec="${4:-5}"
  local kill_timeout_sec="${5:-2}"

  if [[ -z "${pgid}" ]]; then
    return 0
  fi
  if [[ ! "${pgid}" =~ ^[1-9][0-9]*$ ||
        ! "${leader_pid}" =~ ^[1-9][0-9]*$ ]]; then
    echo "Refusing to stop an invalid process group: pgid=${pgid} leader=${leader_pid}" >&2
    return 2
  fi

  local caller_pgid
  caller_pgid="$(ps -o pgid= -p "$$" | tr -d '[:space:]')"
  if [[ -n "${caller_pgid}" && "${pgid}" == "${caller_pgid}" ]]; then
    echo "Refusing to signal the helper's own process group ${pgid}." >&2
    return 2
  fi

  # Check the PGID, not only the original leader PID. ros2 launch can leave
  # children in this owned group after its group leader has already exited.
  if e2e_process_group_exists "${pgid}"; then
    kill -s INT -- "-${pgid}" 2>/dev/null || true
    e2e_wait_for_process_group_exit "${pgid}" "${interrupt_timeout_sec}" || true
  fi
  if e2e_process_group_exists "${pgid}"; then
    kill -s TERM -- "-${pgid}" 2>/dev/null || true
    e2e_wait_for_process_group_exit "${pgid}" "${terminate_timeout_sec}" || true
  fi
  if e2e_process_group_exists "${pgid}"; then
    kill -s KILL -- "-${pgid}" 2>/dev/null || true
    e2e_wait_for_process_group_exit "${pgid}" "${kill_timeout_sec}" || true
  fi

  wait "${leader_pid}" 2>/dev/null || true
  if e2e_process_group_exists "${pgid}"; then
    echo "Owned process group ${pgid} is still present after forced cleanup." >&2
    return 1
  fi
  return 0
}
