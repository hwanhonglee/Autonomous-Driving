#!/usr/bin/env bash

set -Eeuo pipefail

readonly EXPECTED_ROOT="/home/a/autoware"
readonly EXPECTED_BRANCH="h2_i/IONIQ_EV_308/PC1_rocket/autoware_universe/v1.1"
readonly EXPECTED_TAG="IONIQ_EV_308_PC1_a"
readonly EXPECTED_REMOTE="https://github.com/hwanhonglee/Autonomous-Driving.git"

die()
{
  printf '[push_v1_1] ERROR: %s\n' "$*" >&2
  exit 1
}

root="$(git rev-parse --show-toplevel)"
[[ "$root" == "$EXPECTED_ROOT" ]] || die "run this helper from ${EXPECTED_ROOT}"
[[ "$(git branch --show-current)" == "$EXPECTED_BRANCH" ]] || die "unexpected branch"
[[ "$(git remote get-url origin)" == "$EXPECTED_REMOTE" ]] || die "unexpected origin"
[[ -z "$(git status --porcelain --untracked-files=all)" ]] || die "commit all changes before push"
[[ "$(git rev-list -n 1 "$EXPECTED_TAG")" == "$(git rev-parse HEAD)" ]] || die "tag does not point to HEAD"
! git ls-files --stage | grep -q '^160000 ' || die "gitlink/submodule remains"
! git ls-files | grep -Eq '(^|/)\.gitmodules$' || die ".gitmodules remains"

git push --atomic --set-upstream origin \
  "HEAD:refs/heads/${EXPECTED_BRANCH}" \
  "refs/tags/${EXPECTED_TAG}:refs/tags/${EXPECTED_TAG}"

printf '[push_v1_1] pushed %s and %s\n' "$EXPECTED_BRANCH" "$EXPECTED_TAG"
