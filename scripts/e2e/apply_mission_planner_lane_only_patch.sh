#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${root}/scripts/e2e/workspace_runtime_lock.sh"
e2e_acquire_workspace_runtime_lock "Town05 finite-route patch application"

manifest_file="${AUTOWARE_E2E_MISSION_PATCH_MANIFEST:-${root}/patches/autoware_mission_planner_lane_only_no_area.manifest.json}"
universe_repository="${AUTOWARE_E2E_UNIVERSE_REPOSITORY:-${root}/src/universe/autoware_universe}"
core_repository="${AUTOWARE_E2E_CORE_REPOSITORY:-${root}/src/core/autoware_core}"
extension_repository="${AUTOWARE_E2E_LANELET2_EXTENSION_REPOSITORY:-${root}/src/core/autoware_lanelet2_extension}"
mission_patch="${AUTOWARE_E2E_MISSION_PATCH_FILE:-${root}/patches/autoware_mission_planner_lane_only_no_area.patch}"
core_patch="${AUTOWARE_E2E_CORE_ROUTE_PATCH_FILE:-${root}/patches/autoware_route_handler_finite_path.patch}"
extension_patch="${AUTOWARE_E2E_LANELET2_EXTENSION_PATCH_FILE:-${root}/patches/autoware_lanelet2_extension_zero_length_resample.patch}"

python3 - \
  "${manifest_file}" \
  "${extension_repository}" "${extension_patch}" \
  "${core_repository}" "${core_patch}" \
  "${universe_repository}" "${mission_patch}" <<'PY'
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys


EXPECTED_NAMES = (
    "autoware_lanelet2_extension",
    "autoware_core",
    "autoware_universe",
)
FORBIDDEN_PATCH_PREFIXES = (
    "GIT binary patch",
    "Binary files ",
    "rename from ",
    "rename to ",
    "copy from ",
    "copy to ",
    "new file mode ",
    "deleted file mode ",
)


def fail(message: str) -> None:
    raise SystemExit(message)


def git(repository: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    completed = subprocess.run(
        ["git", "-C", str(repository), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if check and completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        fail(f"git {' '.join(args)} failed in {repository}: {detail}")
    return completed


manifest_path = Path(sys.argv[1])
if manifest_path.is_symlink() or not manifest_path.is_file():
    fail(f"Town05 finite-route patch manifest is missing or a symlink: {manifest_path}")
try:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError) as error:
    fail(f"cannot read Town05 finite-route patch manifest: {error}")
if manifest.get("schema_version") != 2:
    fail("Town05 finite-route patch manifest schema_version must be 2")
repositories = manifest.get("repositories")
if not isinstance(repositories, list) or [
    item.get("name") if isinstance(item, dict) else None for item in repositories
] != list(EXPECTED_NAMES):
    fail(f"Town05 finite-route manifest repositories must be {EXPECTED_NAMES}")

overrides = {
    name: (Path(sys.argv[2 + index * 2]), Path(sys.argv[3 + index * 2]))
    for index, name in enumerate(EXPECTED_NAMES)
}
validated: list[dict[str, object]] = []

for contract in repositories:
    name = contract["name"]
    repository, patch_path = overrides[name]
    if git(repository, "rev-parse", "--is-inside-work-tree", check=False).returncode != 0:
        fail(f"{name} repository is missing: {repository}")
    expected_commit = contract.get("git_commit")
    expected_tag = contract.get("git_tag")
    if (
        not isinstance(expected_commit, str)
        or re.fullmatch(r"[0-9a-f]{40}", expected_commit) is None
        or not isinstance(expected_tag, str)
        or not expected_tag
    ):
        fail(f"{name} revision contract is invalid")
    actual_commit = git(repository, "rev-parse", "HEAD").stdout.strip()
    if actual_commit != expected_commit:
        fail(
            f"{name} revision differs from the pinned patch base: "
            f"expected={expected_commit} actual={actual_commit}"
        )
    if patch_path.is_symlink() or not patch_path.is_file():
        fail(f"{name} patch is missing or a symlink: {patch_path}")

    files = contract.get("files")
    if not isinstance(files, list) or not files:
        fail(f"{name} patch manifest files must be a non-empty list")
    expected_paths: list[str] = []
    states: set[str] = set()
    repository_resolved = repository.resolve()
    for index, item in enumerate(files):
        if not isinstance(item, dict):
            fail(f"{name} patch manifest files[{index}] is invalid")
        relative = item.get("path")
        base_sha = item.get("base_sha256")
        patched_sha = item.get("patched_sha256")
        if (
            not isinstance(relative, str)
            or not relative
            or Path(relative).is_absolute()
            or relative in expected_paths
            or any(
                not isinstance(value, str)
                or re.fullmatch(r"[0-9a-f]{64}", value) is None
                for value in (base_sha, patched_sha)
            )
        ):
            fail(f"{name} patch manifest files[{index}] is invalid")
        source = (repository / relative).resolve()
        try:
            source.relative_to(repository_resolved)
        except ValueError:
            fail(f"{name} patch manifest path escapes repository: {relative}")
        if source.is_symlink() or not source.is_file():
            fail(f"{name} patch source is missing or a symlink: {relative}")
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        if digest == base_sha:
            states.add("BASE")
        elif digest == patched_sha:
            states.add("PATCHED")
        else:
            fail(f"{name} patch source SHA256 drift: {relative} actual={digest}")
        expected_paths.append(relative)
    if len(states) != 1:
        fail(f"{name} patch sources are in a mixed state: {sorted(states)}")

    patch = patch_path.read_text(encoding="utf-8")
    for forbidden in FORBIDDEN_PATCH_PREFIXES:
        if any(line.startswith(forbidden) for line in patch.splitlines()):
            fail(f"{name} patch contains a forbidden binary/path operation: {forbidden}")
    headers = list(re.finditer(r"^diff --git a/(\S+) b/(\S+)$", patch, re.MULTILINE))
    actual_paths = [match.group(1) for match in headers if match.group(1) == match.group(2)]
    old_paths = re.findall(r"^--- a/(\S+)$", patch, re.MULTILINE)
    new_paths = re.findall(r"^\+\+\+ b/(\S+)$", patch, re.MULTILINE)
    if (
        len(actual_paths) != len(headers)
        or actual_paths != expected_paths
        or old_paths != expected_paths
        or new_paths != expected_paths
    ):
        fail(
            f"{name} patch file set differs from the exact manifest: "
            f"expected={expected_paths} actual={actual_paths}"
        )

    state = states.pop()
    direction = ["apply", "--reverse", "--check"] if state == "PATCHED" else ["apply", "--check"]
    checked = git(repository, *direction, str(patch_path), check=False)
    if checked.returncode != 0:
        fail(f"{name} {state.lower()} bytes do not check against the pinned patch")
    validated.append(
        {
            "name": name,
            "repository": repository,
            "patch": patch_path,
            "files": files,
            "state": state,
        }
    )

applied = 0
for item in validated:
    name = str(item["name"])
    repository = item["repository"]
    patch_path = item["patch"]
    if item["state"] == "PATCHED":
        print(f"{name} finite-route patch is already applied (APPLIED).")
        continue
    git(repository, "apply", str(patch_path))
    for source_contract in item["files"]:
        relative = source_contract["path"]
        digest = hashlib.sha256((repository / relative).read_bytes()).hexdigest()
        if digest != source_contract["patched_sha256"]:
            fail(f"{name} patch verification failed after apply: {relative}")
    if git(repository, "apply", "--reverse", "--check", str(patch_path), check=False).returncode != 0:
        fail(f"{name} patch reverse-check failed after apply")
    applied += 1
    print(f"Applied {name} finite-route patch.")

if applied == 0:
    print("Autoware mission planner lane-only fallback patch set is already applied (APPLIED).")
else:
    print("Applied Autoware mission planner lane-only fallback and Town05 finite-route patch set.")
PY
