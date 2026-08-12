#!/usr/bin/env bash

# HH_260811 - Prepare the current PC1 files as independent IONIQ EV 308 v1.1 snapshots.
# This script never pushes and never reads or stores a GitHub token.

set -Eeuo pipefail
umask 022

readonly AUTOWARE_ROOT="/home/a/autoware"
readonly AUTOWARE_SRC="${AUTOWARE_ROOT}/src"
readonly ROS2_WS_ROOT="/home/a/ros2_ws"
readonly RELEASE_DIR="${AUTOWARE_SRC}/migration_work/releases/PC1_v1.1"
readonly ROS2_SOCKETCAN_SOURCE="${AUTOWARE_SRC}/sensor_component/ros2_socketcan"
readonly ROS2_WS_PAYLOAD="${RELEASE_DIR}/payload/ros2_ws"
readonly METADATA_BACKUP_ROOT="/home/a/PC1_v1.1_git_metadata_backup"

readonly REMOTE_URL="https://github.com/hwanhonglee/Autonomous-Driving.git"
readonly AUTOWARE_BRANCH="h2_i/IONIQ_EV_308/PC1_rocket/autoware_universe/v1.1"
readonly AUTOWARE_TAG="IONIQ_EV_308_PC1_a"
readonly ROS2_WS_BRANCH="h2_i/IONIQ_EV_308/PC1_rocket/ros2_ws/v1.1"
readonly ROS2_WS_TAG="IONIQ_EV_308_PC1_r"

die()
{
  printf '[PREPARE_IN_PLACE] ERROR: %s\n' "$*" >&2
  exit 1
}

info()
{
  printf '[PREPARE_IN_PLACE] %s\n' "$*"
}

require_command()
{
  command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"
}

configure_identity()
{
  local repository="$1"
  local author_name author_email

  author_name="${PC1_GIT_AUTHOR_NAME:-$(git config --global --get user.name 2>/dev/null || true)}"
  author_email="${PC1_GIT_AUTHOR_EMAIL:-$(git config --global --get user.email 2>/dev/null || true)}"
  author_name="${author_name:-hwanhonglee}"
  author_email="${author_email:-hwanhonglee@users.noreply.github.com}"

  git -C "$repository" config user.name "$author_name"
  git -C "$repository" config user.email "$author_email"
  git -C "$repository" config core.autocrlf false
  info "commit identity: ${author_name} <${author_email}> (repository-local)"
}

ensure_origin()
{
  local repository="$1"
  local current_url

  if git -C "$repository" remote get-url origin >/dev/null 2>&1; then
    current_url="$(git -C "$repository" remote get-url origin)"
    [[ "$current_url" == "$REMOTE_URL" ]] || die "unexpected origin in ${repository}: ${current_url}"
  else
    git -C "$repository" remote add origin "$REMOTE_URL"
  fi
}

move_nested_git_metadata()
{
  local source_path="$1"
  local backup_name="$2"
  local backup_path="${METADATA_BACKUP_ROOT}/${backup_name}"

  if [[ -e "$source_path" ]]; then
    [[ ! -e "$backup_path" ]] || die "both metadata source and backup exist: ${source_path}, ${backup_path}"
    mv -- "$source_path" "$backup_path"
    info "moved nested Git metadata to recoverable backup: ${backup_path}"
  elif [[ -e "$backup_path" ]]; then
    info "nested Git metadata was already moved: ${backup_path}"
  else
    info "nested Git metadata not present: ${source_path}"
  fi
}

check_secrets_and_size()
{
  local matches large_file
  local secret_pattern
  secret_pattern='-----BEGIN ([A-Z0-9 ]+ )?PRIVATE KEY-----|github_pat_[A-Za-z0-9_]{20,}|gh[pousr]_[A-Za-z0-9]{20,}|AKIA[0-9A-Z]{16}|sk-(proj-)?[A-Za-z0-9_-]{20,}|https://[^/@[:space:]]+:[^/@[:space:]]+@github\.com'

  matches="$(rg --hidden --files-with-matches --glob '!**/.git/**' --regexp "$secret_pattern" "$AUTOWARE_SRC" 2>/dev/null || true)"
  if [[ -n "$matches" ]]; then
    printf '%s\n' "$matches" >&2
    die "possible credential-bearing files found; nothing was committed"
  fi

  large_file="$(find "$AUTOWARE_SRC" -type f -size +99M -print -quit)"
  [[ -z "$large_file" ]] || die "GitHub 100 MB limit risk: ${large_file}"
}

assert_flat_index()
{
  local repository="$1"

  if git -C "$repository" ls-files --stage | grep -q '^160000 '; then
    die "gitlink/submodule entry remains in ${repository}"
  fi
  if git -C "$repository" ls-files | grep -Eq '(^|/)\.gitmodules$'; then
    die ".gitmodules remains tracked in ${repository}"
  fi
}

compare_autoware_files_to_index()
{
  local expected tracked difference
  expected="$(mktemp /tmp/pc1-autoware-expected.XXXXXX)"
  tracked="$(mktemp /tmp/pc1-autoware-tracked.XXXXXX)"
  difference="$(mktemp /tmp/pc1-autoware-difference.XXXXXX)"

  find "$AUTOWARE_ROOT" \
    \( -path "${AUTOWARE_ROOT}/.git" -o -path "${AUTOWARE_ROOT}/build" -o \
       -path "${AUTOWARE_ROOT}/install" -o -path "${AUTOWARE_ROOT}/log" \) -prune -o \
    \( -type f -o -type l \) -printf '%P\n' | LC_ALL=C sort >"$expected"
  git -C "$AUTOWARE_ROOT" -c core.quotePath=false ls-files | LC_ALL=C sort >"$tracked"

  if ! cmp -s "$expected" "$tracked"; then
    comm -3 "$expected" "$tracked" >"$difference"
    sed -n '1,30p' "$difference" >&2
    die "Autoware working files and Git index differ; commit stopped to prevent omission"
  fi

  info "Autoware omission check passed: $(wc -l <"$tracked") tracked files/symlinks"
}

compare_ros2_ws_files_to_index()
{
  local expected tracked difference
  expected="$(mktemp /tmp/pc1-ros2-ws-expected.XXXXXX)"
  tracked="$(mktemp /tmp/pc1-ros2-ws-tracked.XXXXXX)"
  difference="$(mktemp /tmp/pc1-ros2-ws-difference.XXXXXX)"

  find "$ROS2_WS_ROOT" -path "${ROS2_WS_ROOT}/.git" -prune -o \
    \( -type f -o -type l \) -printf '%P\n' | LC_ALL=C sort >"$expected"
  git -C "$ROS2_WS_ROOT" -c core.quotePath=false ls-files | LC_ALL=C sort >"$tracked"

  if ! cmp -s "$expected" "$tracked"; then
    comm -3 "$expected" "$tracked" >"$difference"
    sed -n '1,30p' "$difference" >&2
    die "ros2_ws working files and Git index differ; commit stopped to prevent omission"
  fi

  info "ros2_ws omission check passed: $(wc -l <"$tracked") tracked files/symlinks"
}

create_or_verify_tag()
{
  local repository="$1"
  local tag_name="$2"
  local head tag_target
  head="$(git -C "$repository" rev-parse HEAD)"

  if git -C "$repository" rev-parse --verify --quiet "refs/tags/${tag_name}" >/dev/null; then
    tag_target="$(git -C "$repository" rev-list -n 1 "$tag_name")"
    [[ "$tag_target" == "$head" ]] || die "local tag ${tag_name} points to a different commit"
  else
    git -C "$repository" tag "$tag_name" "$head"
  fi
}

prepare_autoware()
{
  local current_branch marker
  marker="${AUTOWARE_ROOT}/.git/pc1_v1_1_owner"

  mkdir -p "$METADATA_BACKUP_ROOT"
  move_nested_git_metadata "${AUTOWARE_SRC}/.git" "autoware_src_empty.git"
  move_nested_git_metadata \
    "${AUTOWARE_SRC}/universe/autoware.universe/autoware_tools/.git" \
    "autoware_tools.git"

  if find "$AUTOWARE_SRC" -mindepth 1 -name .git -print -quit | grep -q .; then
    die "an unhandled nested .git remains under ${AUTOWARE_SRC}"
  fi
  if find "$AUTOWARE_SRC" -name .gitmodules -print -quit | grep -q .; then
    die "a .gitmodules file remains under ${AUTOWARE_SRC}"
  fi

  if [[ ! -d "${AUTOWARE_ROOT}/.git" ]]; then
    git init --quiet --initial-branch="$AUTOWARE_BRANCH" "$AUTOWARE_ROOT"
    printf '%s\n' 'IONIQ_EV_308_PC1_a in-place repository' >"$marker"
  else
    [[ -f "$marker" ]] || die "existing ${AUTOWARE_ROOT}/.git was not created by this release helper"
  fi

  current_branch="$(git -C "$AUTOWARE_ROOT" branch --show-current)"
  [[ "$current_branch" == "$AUTOWARE_BRANCH" ]] || die "unexpected Autoware branch: ${current_branch}"
  ensure_origin "$AUTOWARE_ROOT"
  configure_identity "$AUTOWARE_ROOT"
  install -m 0755 "${RELEASE_DIR}/push_autoware_v1_1.sh" "${AUTOWARE_ROOT}/push_v1_1.sh"

  git -C "$AUTOWARE_ROOT" add --all -- .
  git -C "$AUTOWARE_ROOT" add --all --force -- src
  git -C "$AUTOWARE_ROOT" rm -r --cached --ignore-unmatch -- build install log >/dev/null
  assert_flat_index "$AUTOWARE_ROOT"
  compare_autoware_files_to_index

  if ! git -C "$AUTOWARE_ROOT" diff --cached --quiet; then
    git -C "$AUTOWARE_ROOT" commit -m 'release(pc1): snapshot IONIQ EV 308 Autoware v1.1'
  fi
  git -C "$AUTOWARE_ROOT" rev-parse --verify HEAD >/dev/null
  [[ "$(git -C "$AUTOWARE_ROOT" rev-list --max-parents=0 HEAD)" == "$(git -C "$AUTOWARE_ROOT" rev-parse HEAD)" ]] || \
    die "Autoware snapshot is not an independent root commit"
  create_or_verify_tag "$AUTOWARE_ROOT" "$AUTOWARE_TAG"
  [[ -z "$(git -C "$AUTOWARE_ROOT" status --porcelain --untracked-files=all)" ]] || \
    die "Autoware repository is not clean after commit"
}

prepare_ros2_ws()
{
  local assembly current_branch marker
  assembly="$(mktemp -d /tmp/pc1-ros2-ws-assembly.XXXXXX)"

  mkdir -p "${assembly}/src/ros2_socketcan" "${assembly}/docs/v1.1"
  rsync -a --delete --exclude='.git/' --exclude='.gitmodules' \
    "${ROS2_SOCKETCAN_SOURCE}/" "${assembly}/src/ros2_socketcan/"
  rsync -a --delete --exclude='.git/' --exclude='.gitmodules' \
    "${ROS2_WS_PAYLOAD}/" "${assembly}/"
  rsync -a --delete --exclude='payload/' --exclude='.git/' --exclude='.gitmodules' \
    "${RELEASE_DIR}/" "${assembly}/docs/v1.1/"
  install -m 0755 "${RELEASE_DIR}/push_ros2_ws_v1_1.sh" "${assembly}/push_v1_1.sh"

  if [[ -d "$ROS2_WS_ROOT" && ! -d "${ROS2_WS_ROOT}/.git" ]] && \
    find "$ROS2_WS_ROOT" -mindepth 1 -print -quit | grep -q .; then
    die "${ROS2_WS_ROOT} already contains files and is not this release repository"
  fi
  mkdir -p "$ROS2_WS_ROOT"

  marker="${ROS2_WS_ROOT}/.git/pc1_v1_1_owner"
  if [[ ! -d "${ROS2_WS_ROOT}/.git" ]]; then
    git init --quiet --initial-branch="$ROS2_WS_BRANCH" "$ROS2_WS_ROOT"
    printf '%s\n' 'IONIQ_EV_308_PC1_r in-place repository' >"$marker"
  else
    [[ -f "$marker" ]] || die "existing ${ROS2_WS_ROOT}/.git was not created by this release helper"
  fi

  current_branch="$(git -C "$ROS2_WS_ROOT" branch --show-current)"
  [[ "$current_branch" == "$ROS2_WS_BRANCH" ]] || die "unexpected ros2_ws branch: ${current_branch}"
  ensure_origin "$ROS2_WS_ROOT"
  configure_identity "$ROS2_WS_ROOT"

  rsync -a --delete --exclude='/.git/' "${assembly}/" "${ROS2_WS_ROOT}/"
  if find "$ROS2_WS_ROOT" -mindepth 2 -name .git -print -quit | grep -q .; then
    die "a nested .git remains under ${ROS2_WS_ROOT}"
  fi
  if find "$ROS2_WS_ROOT" -name .gitmodules -print -quit | grep -q .; then
    die "a .gitmodules file remains under ${ROS2_WS_ROOT}"
  fi

  git -C "$ROS2_WS_ROOT" add --all --force -- .
  assert_flat_index "$ROS2_WS_ROOT"
  compare_ros2_ws_files_to_index

  if ! git -C "$ROS2_WS_ROOT" diff --cached --quiet; then
    git -C "$ROS2_WS_ROOT" commit -m 'release(pc1): snapshot IONIQ EV 308 ros2_ws v1.1'
  fi
  git -C "$ROS2_WS_ROOT" rev-parse --verify HEAD >/dev/null
  [[ "$(git -C "$ROS2_WS_ROOT" rev-list --max-parents=0 HEAD)" == "$(git -C "$ROS2_WS_ROOT" rev-parse HEAD)" ]] || \
    die "ros2_ws snapshot is not an independent root commit"
  create_or_verify_tag "$ROS2_WS_ROOT" "$ROS2_WS_TAG"
  [[ -z "$(git -C "$ROS2_WS_ROOT" status --porcelain --untracked-files=all)" ]] || \
    die "ros2_ws repository is not clean after commit"
}

main()
{
  [[ "$#" -eq 0 ]] || die "this script takes no arguments"
  [[ "$(id -u)" -ne 0 ]] || die "do not run this script as root"

  for command_name in git rsync rg find cmp comm sed install; do
    require_command "$command_name"
  done
  [[ -d "$AUTOWARE_SRC" ]] || die "missing source tree: ${AUTOWARE_SRC}"
  [[ -d "$ROS2_SOCKETCAN_SOURCE" ]] || die "missing ros2_socketcan source: ${ROS2_SOCKETCAN_SOURCE}"
  [[ -d "$ROS2_WS_PAYLOAD" ]] || die "missing ros2_ws payload: ${ROS2_WS_PAYLOAD}"

  check_secrets_and_size
  prepare_autoware
  prepare_ros2_ws

  info "local commits and lightweight tags are ready; nothing was pushed"
  printf '\ncd /home/a/autoware && ./push_v1_1.sh\n'
  printf 'cd /home/a/ros2_ws && ./push_v1_1.sh\n'
}

main "$@"
