#!/usr/bin/env bash

set -Eeuo pipefail

readonly EXPECTED_ROOT="/home/a/autoware"
readonly EXPECTED_BRANCH="h2_i/IONIQ_EV_308/PC1_rocket/autoware_universe/v1.1"
readonly EXPECTED_TAG="IONIQ_EV_308_PC1_a"
readonly EXPECTED_REMOTE="https://github.com/hwanhonglee/Autonomous-Driving.git"
# HH_260812 - The first public snapshot used this tag target. The operator explicitly approved
# one conditional move after the v1.1 evidence PR is merged; every other unexpected target blocks.
readonly APPROVED_OLD_TAG_TARGET="2523b6620d4c6e657e06983e3d7c1e5340dd4611"

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
! git ls-files --stage | grep -q '^160000 ' || die "gitlink/submodule remains"
! git ls-files | grep -Eq '(^|/)\.gitmodules$' || die ".gitmodules remains"

head="$(git rev-parse HEAD)"
remote_tag="$(git ls-remote --refs origin "refs/tags/${EXPECTED_TAG}" | awk 'NR == 1 {print $1}')"

if [[ -z "$remote_tag" ]]; then
  git tag "$EXPECTED_TAG" "$head"
  git push --atomic --set-upstream origin \
    "HEAD:refs/heads/${EXPECTED_BRANCH}" \
    "refs/tags/${EXPECTED_TAG}:refs/tags/${EXPECTED_TAG}"
elif [[ "$remote_tag" == "$head" ]]; then
  git tag --force "$EXPECTED_TAG" "$head" >/dev/null
  git push --atomic --set-upstream origin \
    "HEAD:refs/heads/${EXPECTED_BRANCH}" \
    "refs/tags/${EXPECTED_TAG}:refs/tags/${EXPECTED_TAG}"
elif [[ "$remote_tag" == "$APPROVED_OLD_TAG_TARGET" ]]; then
  git tag --force "$EXPECTED_TAG" "$head" >/dev/null
  git push --atomic --set-upstream \
    --force-with-lease="refs/tags/${EXPECTED_TAG}:${APPROVED_OLD_TAG_TARGET}" \
    origin \
    "HEAD:refs/heads/${EXPECTED_BRANCH}" \
    "refs/tags/${EXPECTED_TAG}:refs/tags/${EXPECTED_TAG}"
else
  die "remote tag points to unexpected commit: ${remote_tag}"
fi

printf '[push_v1_1] pushed %s and %s\n' "$EXPECTED_BRANCH" "$EXPECTED_TAG"
