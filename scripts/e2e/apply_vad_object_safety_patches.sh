#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${root}/scripts/e2e/workspace_runtime_lock.sh"
e2e_acquire_workspace_runtime_lock "VAD object-safety patch application"

manifest="${AUTOWARE_E2E_VAD_OBJECT_SAFETY_MANIFEST:-${root}/patches/autoware_vad_object_safety.manifest.json}"
repository="${AUTOWARE_E2E_UNIVERSE_REPOSITORY:-${root}/src/universe/autoware_universe}"

python3 - "${root}" "${manifest}" "${repository}" <<'PY'
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys


def fail(message: str) -> None:
    raise SystemExit(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(repository: Path, *args: str, check: bool = True):
    result = subprocess.run(
        ["git", "-C", str(repository), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if check and result.returncode:
        fail(result.stderr.strip() or result.stdout.strip() or f"git {' '.join(args)} failed")
    return result


root = Path(sys.argv[1]).resolve()
manifest_path = Path(sys.argv[2])
repository = Path(sys.argv[3]).resolve()
if manifest_path.is_symlink() or not manifest_path.is_file():
    fail(f"VAD object-safety manifest is missing or a symlink: {manifest_path}")
try:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError) as error:
    fail(f"cannot read VAD object-safety manifest: {error}")
if manifest.get("schema_version") != 1:
    fail("VAD object-safety manifest schema_version must be 1")

repository_contract = manifest.get("repository")
if not isinstance(repository_contract, dict):
    fail("VAD object-safety repository contract is missing")
expected_commit = repository_contract.get("git_commit")
if not isinstance(expected_commit, str) or re.fullmatch(r"[0-9a-f]{40}", expected_commit) is None:
    fail("VAD object-safety repository commit is invalid")
actual_commit = git(repository, "rev-parse", "HEAD").stdout.strip()
if actual_commit != expected_commit:
    fail(f"autoware_universe HEAD drift: expected={expected_commit} actual={actual_commit}")

for index, prerequisite in enumerate(manifest.get("prerequisite_patches") or []):
    if not isinstance(prerequisite, dict):
        fail(f"prerequisite_patches[{index}] is invalid")
    relative = prerequisite.get("path")
    expected_sha = prerequisite.get("sha256")
    if not isinstance(relative, str) or not isinstance(expected_sha, str):
        fail(f"prerequisite_patches[{index}] contract is invalid")
    path = (root / relative).resolve()
    try:
        path.relative_to(root)
    except ValueError:
        fail(f"prerequisite patch escapes workspace: {relative}")
    if path.is_symlink() or not path.is_file() or sha256(path) != expected_sha:
        fail(f"prerequisite patch SHA256 drift: {relative}")

patches = manifest.get("patches")
if not isinstance(patches, list) or not patches:
    fail("VAD object-safety patches must be a non-empty list")
validated = []
for patch_index, contract in enumerate(patches):
    if not isinstance(contract, dict):
        fail(f"patches[{patch_index}] is invalid")
    name = contract.get("name")
    patch_relative = contract.get("path")
    expected_patch_sha = contract.get("sha256")
    files = contract.get("files")
    if (
        not isinstance(name, str)
        or not isinstance(patch_relative, str)
        or not isinstance(expected_patch_sha, str)
        or re.fullmatch(r"[0-9a-f]{64}", expected_patch_sha) is None
        or not isinstance(files, list)
        or not files
    ):
        fail(f"patches[{patch_index}] contract is invalid")
    patch_path = (root / patch_relative).resolve()
    try:
        patch_path.relative_to(root)
    except ValueError:
        fail(f"patch escapes workspace: {patch_relative}")
    if patch_path.is_symlink() or not patch_path.is_file() or sha256(patch_path) != expected_patch_sha:
        fail(f"patch SHA256 drift: {patch_relative}")

    patch_text = patch_path.read_text(encoding="utf-8")
    actual_paths = re.findall(r"^diff --git a/(\S+) b/(\S+)$", patch_text, re.MULTILINE)
    expected_paths = [item.get("path") if isinstance(item, dict) else None for item in files]
    if any(left != right for left, right in actual_paths) or [left for left, _ in actual_paths] != expected_paths:
        fail(f"{name} patch file set differs from manifest")

    states = set()
    for file_index, item in enumerate(files):
        if not isinstance(item, dict):
            fail(f"{name} files[{file_index}] is invalid")
        relative = item.get("path")
        base_state = item.get("base_state")
        base_sha = item.get("base_sha256")
        patched_sha = item.get("patched_sha256")
        if (
            not isinstance(relative, str)
            or base_state not in {"file", "absent"}
            or not isinstance(patched_sha, str)
            or re.fullmatch(r"[0-9a-f]{64}", patched_sha) is None
            or (base_state == "file" and (
                not isinstance(base_sha, str) or re.fullmatch(r"[0-9a-f]{64}", base_sha) is None
            ))
        ):
            fail(f"{name} files[{file_index}] contract is invalid")
        source = (repository / relative).resolve()
        try:
            source.relative_to(repository)
        except ValueError:
            fail(f"{name} source escapes repository: {relative}")
        if source.is_symlink():
            fail(f"{name} source is a symlink: {relative}")
        if not source.exists():
            if base_state == "absent":
                states.add("BASE")
                continue
            fail(f"{name} source is missing: {relative}")
        if not source.is_file():
            fail(f"{name} source is not a regular file: {relative}")
        digest = sha256(source)
        if digest == patched_sha:
            states.add("PATCHED")
        elif base_state == "file" and digest == base_sha:
            states.add("BASE")
        else:
            fail(f"{name} source SHA256 drift: {relative} actual={digest}")
    if len(states) != 1:
        fail(f"{name} sources are in a mixed state: {sorted(states)}")
    state = states.pop()
    check_args = ["apply", "--reverse", "--check"] if state == "PATCHED" else ["apply", "--check"]
    if git(repository, *check_args, str(patch_path), check=False).returncode:
        fail(f"{name} {state.lower()} bytes do not match its patch")
    validated.append((name, patch_path, files, state))

for name, patch_path, files, state in validated:
    if state == "PATCHED":
        print(f"{name} patch is already applied (APPLIED).")
        continue
    git(repository, "apply", str(patch_path))
    for item in files:
        source = repository / item["path"]
        if not source.is_file() or sha256(source) != item["patched_sha256"]:
            fail(f"{name} verification failed after apply: {item['path']}")
    if git(repository, "apply", "--reverse", "--check", str(patch_path), check=False).returncode:
        fail(f"{name} reverse-check failed after apply")
    print(f"Applied {name} patch.")
PY
