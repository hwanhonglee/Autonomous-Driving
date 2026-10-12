#!/usr/bin/env python3
"""Export exact imported-repository revisions and reviewed local source patches.

HH_261012 - Publish imported Autoware changes without changing their Git state,
exposing private authentication material, or claiming real-vehicle readiness.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import tempfile
from typing import Any
from urllib.parse import urlsplit


class SnapshotError(RuntimeError):
    """A fail-closed publication or source-consistency rejection."""


# HH_261012 - Limit new imported files to readable source/configuration formats.
SOURCE_SUFFIXES = frozenset({
    ".c", ".cc", ".cpp", ".cxx", ".cu", ".cuh", ".h", ".hh", ".hpp",
    ".hxx", ".py", ".sh", ".cmake", ".xml", ".yaml", ".yml", ".json",
    ".md", ".txt", ".rviz", ".repos", ".patch", ".in",
})
SOURCE_NAMES = frozenset({"CMakeLists.txt", "Makefile", "LICENSE", "NOTICE"})
MAX_SOURCE_BYTES = 8 * 1024 * 1024
SECRET_NAME = re.compile(
    r"(^|[._-])(credential|password|passwd|passphrase|secret|private[-_]?key|"
    r"authorized_keys|known_hosts|keyring)([._-]|$)|^id_(rsa|ed25519|ecdsa|dsa)(\.|$)",
    re.IGNORECASE,
)
SECRET_BYTES = re.compile(
    rb"-----BEGIN (?:OPENSSH|RSA|EC|DSA) PRIVATE KEY-----|"
    rb"\bgh[pousr]_[A-Za-z0-9]{20,}\b|\bgithub_pat_[A-Za-z0-9_]{20,}\b|"
    rb"\bAKIA[A-Z0-9]{16}\b"
)
SECRET_LITERAL = re.compile(
    r"[\"']?(?:password|passwd|passphrase|api[_-]?key|access[_-]?token|"
    r"auth[_-]?token|secret)[\"']?\s*[:=]\s*[\"']([^\"'\n]+)[\"']",
    re.IGNORECASE,
)
PLACEHOLDERS = frozenset({"password", "passwd", "secret", "token", "changeme",
                          "change_me", "your_password", "your_token", "example",
                          "placeholder", "redacted", "<redacted>"})


def digest(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def git(repo: Path, *args: str, input_bytes: bytes | None = None,
        environment: dict[str, str] | None = None,
        allowed_codes: tuple[int, ...] = (0,)) -> bytes:
    # HH_261012 - Suppress optional index refreshes and external diff/textconv tools.
    env = dict(os.environ)
    for key in ("GIT_INDEX_FILE", "GIT_DIR", "GIT_WORK_TREE", "GIT_COMMON_DIR"):
        env.pop(key, None)
    env.update({"GIT_OPTIONAL_LOCKS": "0", "GIT_TERMINAL_PROMPT": "0"})
    if environment:
        env.update(environment)
    completed = subprocess.run(["git", "-C", str(repo), *args], input=input_bytes,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               env=env, check=False, timeout=60)
    if completed.returncode not in allowed_codes:
        # Git errors may contain URLs/content: do not echo raw stderr or arguments.
        raise SnapshotError(f"Git operation failed in {repo.name}: exit {completed.returncode}")
    return completed.stdout


def relative_path(value: str) -> str:
    p = PurePosixPath(value)
    if (not value or p.is_absolute() or ".." in p.parts or ".git" in p.parts
            or any(ord(c) < 32 or ord(c) == 127 for c in value)
            or "\\" in value or p.as_posix() != value):
        raise SnapshotError("Ambiguous or unsafe repository path rejected")
    return value


def no_symlink(path: Path, boundary: Path) -> None:
    try:
        rel = path.relative_to(boundary)
    except ValueError as exc:
        raise SnapshotError("Path is outside the declared workspace") from exc
    current = boundary
    if current.is_symlink():
        raise SnapshotError("Symlink workspace boundary rejected")
    for component in rel.parts:
        current = current / component
        if current.is_symlink():
            raise SnapshotError("Symlink publication input/output rejected")


def safe_text(value: bytes, name: str, *, max_bytes: int = MAX_SOURCE_BYTES) -> None:
    if len(value) > max_bytes or b"\0" in value:
        raise SnapshotError(f"Oversized or binary source rejected: {name}")
    try:
        text = value.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise SnapshotError(f"Non-UTF8 source rejected: {name}") from exc
    if SECRET_BYTES.search(value):
        raise SnapshotError(f"Private credential marker rejected: {name}")
    for match in SECRET_LITERAL.finditer(text):
        candidate = match.group(1).strip()
        if (candidate.lower() not in PLACEHOLDERS
                and not candidate.startswith(("${", "$", "<", "{{"))):
            raise SnapshotError(f"Potential credential literal rejected: {name}")


def source_file(repo: Path, name: str, *, untracked: bool) -> tuple[bytes, dict[str, Any]]:
    name = relative_path(name)
    path = repo / name
    no_symlink(path, repo)
    if SECRET_NAME.search(path.name) or path.suffix.lower() in {".pem", ".key", ".p12"}:
        raise SnapshotError(f"Private filename rejected: {name}")
    if untracked and path.suffix.lower() not in SOURCE_SUFFIXES and path.name not in SOURCE_NAMES:
        raise SnapshotError(f"Unreviewed untracked format rejected: {name}")
    try:
        before = path.stat(follow_symlinks=False)
        if not stat.S_ISREG(before.st_mode):
            raise SnapshotError(f"Nonregular source rejected: {name}")
        if before.st_size > MAX_SOURCE_BYTES:
            raise SnapshotError(f"Oversized source rejected: {name}")
        value = path.read_bytes()
        after = path.stat(follow_symlinks=False)
    except OSError as exc:
        raise SnapshotError(f"Unreadable source rejected: {name}") from exc
    if (before.st_ino, before.st_dev, before.st_size, before.st_mtime_ns, before.st_mode) != (
            after.st_ino, after.st_dev, after.st_size, after.st_mtime_ns, after.st_mode):
        raise SnapshotError(f"Source changed while reading: {name}")
    safe_text(value, name)
    return value, {"path": name, "sha256": digest(value), "bytes": len(value),
                   "mode": stat.S_IMODE(after.st_mode)}


def safe_remote(value: str) -> str:
    # HH_261012 - Never put embedded tokens, passwords, or URL query data into manifests.
    if SECRET_BYTES.search(value.encode()):
        raise SnapshotError("Credential-bearing remote URL rejected")
    if value.startswith("git@") and ":" in value:
        host, tail = value[4:].split(":", 1)
        if (not re.fullmatch(r"[A-Za-z0-9.-]+", host) or not tail
                or any(c.isspace() for c in value) or "?" in tail or "#" in tail):
            raise SnapshotError("Ambiguous SSH remote rejected")
        return value
    parsed = urlsplit(value)
    if (parsed.scheme not in {"https", "ssh"} or not parsed.hostname
            or parsed.password or parsed.query or parsed.fragment
            or (parsed.scheme == "https" and parsed.username is not None)
            or (parsed.scheme == "ssh" and parsed.username not in {None, "git"})
            or any(c.isspace() for c in value)):
        raise SnapshotError("Credential-bearing or unsupported remote URL rejected")
    return value


def status_paths(raw: bytes) -> list[tuple[str, str]]:
    entries: list[tuple[str, str]] = []
    for item in raw.split(b"\0"):
        if not item:
            continue
        try:
            status_code = item[:2].decode("ascii")
            name = item[3:].decode("utf-8")
        except UnicodeDecodeError as exc:
            raise SnapshotError("Non-UTF8 Git status rejected") from exc
        if (len(item) < 4 or item[2:3] != b" "
                or status_code != "??" and any(c not in " MAD" for c in status_code)):
            raise SnapshotError("Rename, conflict, submodule, or ambiguous Git status rejected")
        entries.append((status_code, relative_path(name)))
    return entries


def discover_repositories(source_root: Path) -> list[Path]:
    if not source_root.is_dir() or source_root.is_symlink():
        raise SnapshotError("Declared imported source root is missing or linked")
    repos: list[Path] = []
    for directory, dirs, files in os.walk(source_root, followlinks=False):
        here = Path(directory)
        if ".git" in files:
            raise SnapshotError("Linked-worktree Git marker requires separate explicit review")
        if ".git" in dirs:
            if (here / ".git").is_symlink():
                raise SnapshotError("Symlink Git directory rejected")
            repos.append(here)
            dirs[:] = []
        else:
            dirs[:] = sorted(d for d in dirs if not (here / d).is_symlink())
    if not repos:
        raise SnapshotError("No imported Git repositories found")
    return sorted(repos)


def capture_repository(repo: Path, source_root: Path) -> tuple[dict[str, Any], bytes]:
    head = git(repo, "rev-parse", "HEAD").decode().strip()
    if not re.fullmatch(r"[0-9a-f]{40}", head):
        raise SnapshotError("Unsupported HEAD object identifier")
    remotes: dict[str, list[str]] = {}
    for remote in git(repo, "remote").decode().splitlines():
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", remote):
            raise SnapshotError("Ambiguous remote name rejected")
        urls = git(repo, "remote", "get-url", "--all", remote).decode().splitlines()
        remotes[remote] = [safe_remote(url) for url in urls]
        push_urls = git(repo, "remote", "get-url", "--push", "--all", remote).decode().splitlines()
        if push_urls != urls:
            remotes[f"{remote}:push"] = [safe_remote(url) for url in push_urls]
    if "origin" not in remotes or len(remotes["origin"]) != 1:
        raise SnapshotError("Exactly one origin URL is required for exact.repos")
    raw_status = git(repo, "status", "--porcelain=v1", "-z", "--untracked-files=all",
                     "--ignore-submodules=none")
    entries = status_paths(raw_status)
    index_name = git(repo, "rev-parse", "--git-path", "index").decode().strip()
    index = Path(index_name)
    if not index.is_absolute():
        index = repo / index
    no_symlink(index, repo)
    if index.is_symlink() or not index.is_file():
        raise SnapshotError("Missing or linked repository index rejected")
    index_bytes = index.read_bytes()
    patch = git(repo, "diff", "--binary", "--no-ext-diff", "--no-textconv",
                "--no-renames", "HEAD", "--")
    source_seals: list[dict[str, Any]] = []
    for code, name in entries:
        if code == "??":
            _, seal = source_file(repo, name, untracked=True)
            source_seals.append({**seal, "kind": "untracked"})
            addition = git(repo, "diff", "--no-index", "--binary", "--no-ext-diff",
                           "--no-textconv", "--", "/dev/null", name, allowed_codes=(1,))
            patch += addition
        else:
            if (repo / name).is_symlink():
                raise SnapshotError(f"Symlink publication source rejected: {name}")
            if SECRET_NAME.search(Path(name).name) or Path(name).suffix.lower() in {".pem", ".key", ".p12"}:
                raise SnapshotError(f"Private filename rejected: {name}")
            if not (repo / name).exists():
                old = git(repo, "show", f"HEAD:{name}")
                safe_text(old, name)
                source_seals.append({"path": name, "kind": "deleted",
                                     "head_bytes": len(old), "head_sha256": digest(old)})
            else:
                _, seal = source_file(repo, name, untracked=False)
                source_seals.append({**seal, "kind": "tracked"})
    # Scan patch text too: deleted context can contain a secret absent in current files.
    if len(patch) > 64 * 1024 * 1024:
        raise SnapshotError("Repository patch is beyond the declared 64MiB bound")
    if patch:
        safe_text(patch, repo.relative_to(source_root).as_posix() + " patch",
                  max_bytes=64 * 1024 * 1024)
    snapshot = {"path": repo.relative_to(source_root).as_posix(), "head": head,
                "remotes": remotes, "branch": git(repo, "branch", "--show-current").decode().strip(),
                "status": [{"code": code, "path": name} for code, name in entries],
                "status_sha256": digest(raw_status), "index_sha256": digest(index_bytes),
                "source_files": source_seals, "patch_sha256": digest(patch), "patch_bytes": len(patch)}
    if digest(index.read_bytes()) != snapshot["index_sha256"]:
        raise SnapshotError("Repository index changed during capture")
    return snapshot, patch


def check_patch(repo: Path, patch: bytes) -> None:
    if not patch:
        return
    # HH_261012 - Read-tree writes only an ephemeral index, never a repository's index.
    with tempfile.TemporaryDirectory(prefix="hh_autoware_patch_check_") as temporary:
        alternate = {"GIT_INDEX_FILE": str(Path(temporary) / "index")}
        git(repo, "read-tree", "HEAD", environment=alternate)
        git(repo, "apply", "--cached", "--check", "--binary", "--whitespace=nowarn",
            input_bytes=patch, environment=alternate)


def exact_repos(repositories: list[dict[str, Any]]) -> bytes:
    # JSON quoted strings are valid YAML scalar strings; no YAML dependency is needed.
    lines = ["# HH_261012 - Exact imported revisions; apply the manifest's patches afterwards.",
             "repositories:"]
    for item in repositories:
        lines.extend([f"  {json.dumps(item['path'])}:", "    type: git",
                      f"    url: {json.dumps(item['remotes']['origin'][0])}",
                      f"    version: {json.dumps(item['head'])}"])
    return ("\n".join(lines) + "\n").encode()


def export_snapshot(workspace: Path, output: Path) -> dict[str, Any]:
    workspace = workspace.absolute()
    output = output.absolute()
    no_symlink(workspace, workspace)
    no_symlink(output, workspace)
    if output.exists() or output.is_symlink():
        raise SnapshotError("Existing export output is never overwritten")
    rel = output.relative_to(workspace)
    if (not rel.parts or rel.parts[0] not in {"autonomy", "docs"}
            or (rel.parts[0] == "autonomy" and len(rel.parts) > 1
                and rel.parts[1] in {"src", "build", "install", "log", "canonical_rebuild", "layout_check"})):
        raise SnapshotError("Output must be a fresh publication folder, not runtime/private storage")
    source_root = workspace / "autonomy" / "src"
    no_symlink(source_root, workspace)
    repos = discover_repositories(source_root)
    captured = [capture_repository(repo, source_root) for repo in repos]
    for repo, (_, patch) in zip(repos, captured):
        check_patch(repo, patch)
    second = [capture_repository(repo, source_root) for repo in repos]
    if captured != second or discover_repositories(source_root) != repos:
        raise SnapshotError("Imported source/HEAD/status/index changed before publication")
    # HH_261012 - Only generated bundle files are written; original repositories remain read-only.
    output.mkdir(parents=True, exist_ok=False)
    (output / "patches").mkdir()
    published: dict[str, bytes] = {}
    descriptions: list[dict[str, Any]] = []
    for index, (item, patch) in enumerate(captured):
        record = dict(item)
        record["patch"] = None
        if patch:
            patch_name = f"patches/{index:02d}_{item['path'].replace('/', '__')}.patch"
            published[patch_name] = patch
            record["patch"] = patch_name
        record["cached_patch_check"] = "PASS" if patch else "NOT_NEEDED_CLEAN_REPOSITORY"
        descriptions.append(record)
    published["exact.repos"] = exact_repos(descriptions)
    for name, value in published.items():
        with (output / name).open("xb") as target:
            target.write(value)
    after = [capture_repository(repo, source_root) for repo in repos]
    if captured != after or discover_repositories(source_root) != repos:
        raise SnapshotError("Export incomplete: imported sources changed during output writing")
    manifest = {
        "schema": "HH_active_autoware_source_snapshot_v1",
        "provenance": "HH_261012 - Export exact revisions and reviewed complete local source changes.",
        "status": "COMPLETED_SOURCE_EXPORT_NOT_VEHICLE_APPROVAL",
        "import_root": "autonomy/src", "repository_count": len(repos),
        "dirty_repository_count": sum(bool(patch) for _, patch in captured),
        "tracked_status_count": sum(sum(item["code"] != "??" for item in entry["status"])
                                    for entry, _ in captured),
        "untracked_source_count": sum(sum(item["kind"] == "untracked" for item in entry["source_files"]) for entry, _ in captured),
        "source_before_after_equal": True, "original_git_index_unchanged": True,
        "seal_scope": "Exact HEAD/remotes/branch/status/index, complete HEAD patch, and changed/untracked source bytes; not every clean working-tree file.",
        "repositories": descriptions,
        "generated_files": [{"path": name, "bytes": len(value), "sha256": digest(value)}
                            for name, value in sorted(published.items())],
        "real_vehicle_actuation_authorized": False,
        "model_promotion_authorized": False,
    }
    manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode()
    with (output / "manifest.json").open("xb") as target:
        target.write(manifest_bytes)
    published["manifest.json"] = manifest_bytes
    sums = "".join(f"{digest(value)}  {name}\n" for name, value in sorted(published.items()))
    with (output / "SHA256SUMS").open("x", encoding="utf-8") as target:
        target.write(sums)
    return manifest


def verify_bundle(workspace: Path, bundle: Path, *, check_clean_bases: bool = False) -> dict[str, Any]:
    """HH_261012 - Verify saved bytes; optionally check patches against clean imported bases."""
    workspace, bundle = workspace.absolute(), bundle.absolute()
    no_symlink(bundle, workspace)
    no_symlink(bundle / "SHA256SUMS", bundle)
    no_symlink(bundle / "manifest.json", bundle)
    try:
        manifest_bytes = (bundle / "manifest.json").read_bytes()
        manifest = json.loads(manifest_bytes)
        sums_bytes = (bundle / "SHA256SUMS").read_bytes()
    except (OSError, ValueError) as exc:
        raise SnapshotError("Bundle metadata is unreadable or invalid") from exc
    if (manifest.get("schema") != "HH_active_autoware_source_snapshot_v1"
            or manifest.get("status") != "COMPLETED_SOURCE_EXPORT_NOT_VEHICLE_APPROVAL"
            or manifest.get("source_before_after_equal") is not True
            or manifest.get("original_git_index_unchanged") is not True
            or manifest.get("real_vehicle_actuation_authorized") is not False):
        raise SnapshotError("Unsupported or incomplete source manifest")
    generated: dict[str, dict[str, Any]] = {}
    for item in manifest.get("generated_files", []):
        name = relative_path(item["path"])
        if name in generated or name in {"manifest.json", "SHA256SUMS"}:
            raise SnapshotError("Duplicate or invalid generated-file manifest entry")
        generated[name] = item
    expected_sums: dict[str, str] = {}
    try:
        for line in sums_bytes.decode("ascii").splitlines():
            sha, name = line.split("  ", 1)
            name = relative_path(name)
            if not re.fullmatch(r"[0-9a-f]{64}", sha) or name in expected_sums:
                raise SnapshotError("Invalid or duplicate SHA256SUMS entry")
            expected_sums[name] = sha
    except (UnicodeDecodeError, ValueError) as exc:
        raise SnapshotError("Invalid SHA256SUMS syntax") from exc
    if set(expected_sums) != set(generated) | {"manifest.json"}:
        raise SnapshotError("Manifest and SHA256SUMS coverage differ")
    expected_files = set(generated) | {"manifest.json", "SHA256SUMS"}
    observed_files: set[str] = set()
    for directory, dirs, files in os.walk(bundle, followlinks=False):
        here = Path(directory)
        for component in dirs + files:
            no_symlink(here / component, bundle)
        for name in files:
            path = here / name
            if not path.is_file():
                raise SnapshotError("Nonregular bundle output rejected")
            observed_files.add(path.relative_to(bundle).as_posix())
    if observed_files != expected_files:
        raise SnapshotError("Unexpected or missing files in publication bundle")
    values: dict[str, bytes] = {"manifest.json": manifest_bytes}
    for name in generated:
        no_symlink(bundle / name, bundle)
        if not (bundle / name).is_file():
            raise SnapshotError("Generated file is missing or nonregular")
        values[name] = (bundle / name).read_bytes()
    for name, value in values.items():
        if digest(value) != expected_sums[name]:
            raise SnapshotError(f"Saved bundle checksum mismatch: {name}")
        if name in generated and (len(value) != generated[name]["bytes"]
                                  or digest(value) != generated[name]["sha256"]):
            raise SnapshotError(f"Saved file differs from manifest: {name}")
    repositories = manifest.get("repositories", [])
    if len(repositories) != manifest["repository_count"]:
        raise SnapshotError("Declared repository denominator differs")
    paths: set[str] = set()
    patch_names: set[str] = set()
    for item in repositories:
        name = relative_path(item["path"])
        if name in paths or not re.fullmatch(r"[0-9a-f]{40}", item["head"]):
            raise SnapshotError("Duplicate or invalid repository declaration")
        paths.add(name)
        for urls in item["remotes"].values():
            for url in urls:
                safe_remote(url)
        patch_name = item.get("patch")
        if patch_name is not None:
            patch_name = relative_path(patch_name)
            if (not patch_name.startswith("patches/") or not patch_name.endswith(".patch")
                    or patch_name in patch_names or patch_name not in generated):
                raise SnapshotError("Ambiguous patch declaration")
            patch_names.add(patch_name)
            value = values[patch_name]
            safe_text(value, patch_name, max_bytes=64 * 1024 * 1024)
            if digest(value) != item["patch_sha256"] or len(value) != item["patch_bytes"]:
                raise SnapshotError("Repository patch metadata differs")
        elif item["patch_bytes"] != 0 or item["patch_sha256"] != digest(b""):
            raise SnapshotError("Clean repository has inconsistent patch metadata")
        if check_clean_bases:
            repo = workspace / "autonomy/src" / name
            no_symlink(repo, workspace)
            head = git(repo, "rev-parse", "HEAD").decode().strip()
            if head != item["head"]:
                raise SnapshotError("Replay base HEAD differs from exact manifest")
            status = git(repo, "status", "--porcelain=v1", "-z", "--untracked-files=all",
                         "--ignore-submodules=none")
            if status:
                raise SnapshotError("Replay base is dirty; no source apply is performed")
            index_name = git(repo, "rev-parse", "--git-path", "index").decode().strip()
            index = Path(index_name)
            if not index.is_absolute():
                index = repo / index
            no_symlink(index, repo)
            before = index.read_bytes()
            check_patch(repo, values[patch_name] if patch_name else b"")
            after_status = git(repo, "status", "--porcelain=v1", "-z", "--untracked-files=all",
                               "--ignore-submodules=none")
            if (index.read_bytes() != before or after_status
                    or git(repo, "rev-parse", "HEAD").decode().strip() != head):
                raise SnapshotError("Replay check changed source repository state")
    if set(generated) != patch_names | {"exact.repos"}:
        raise SnapshotError("Unexpected files in bundle manifest")
    if values["exact.repos"] != exact_repos(repositories):
        raise SnapshotError("exact.repos does not match repository manifest")
    if len(patch_names) != manifest["dirty_repository_count"]:
        raise SnapshotError("Dirty repository denominator differs")
    return {"status": "VERIFIED_SAVED_BYTES_AND_CLEAN_BASE_CHECK" if check_clean_bases
            else "VERIFIED_SAVED_BYTES_ONLY", "repository_count": len(repositories),
            "checked_files": len(values), "source_apply_count": 0,
            "authenticity_note": "Checksums establish saved-byte consistency, not an external signature."}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace-root", type=Path,
                        default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--verify", action="store_true", help="Verify an existing completed bundle without exporting")
    parser.add_argument("--check-only", action="store_true", help="Verify bundle and check clean matching imported HEAD bases; never apply")
    args = parser.parse_args()
    try:
        if args.verify or args.check_only:
            result = verify_bundle(args.workspace_root, args.output_dir,
                                   check_clean_bases=args.check_only)
            print(json.dumps(result))
            return 0
        result = export_snapshot(args.workspace_root, args.output_dir)
    except (SnapshotError, OSError, subprocess.TimeoutExpired, KeyError, TypeError, ValueError) as exc:
        print(json.dumps({"status": "FAILED_CLOSED", "reason": str(exc)}))
        return 1
    print(json.dumps({"status": result["status"], "repository_count": result["repository_count"],
                      "dirty_repository_count": result["dirty_repository_count"],
                      "untracked_source_count": result["untracked_source_count"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
