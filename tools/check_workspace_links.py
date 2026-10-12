"""HH_260906 - Audit Markdown file destinations without depending on legacy root symlinks."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("hh_layout", ROOT / "tools/migrate_workspace_layout.py")
_layout = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_layout)
MOVES = _layout.MOVES
FROZEN_20260929_SOURCE = ROOT / "docs/assets/validation/2026-09-29/map_lane_weight_pilot/remote_run_v1/source"
FROZEN_PHOTOMETRIC_SOURCE = ROOT / "docs/assets/validation/2026-09-29/map_photometric_pilot/remote_run_v1/source"


def is_immutable_source_snapshot(path):
    """HH_260929 - Do not rebase links inside byte-locked remote source receipts."""
    return ((path.parent == FROZEN_20260929_SOURCE
             and path.name in ("audit.md", "plan.md"))
            or (path.parent == FROZEN_PHOTOMETRIC_SOURCE and path.name == "plan.md"))


def destinations(text):
    # HH_260906 - Preserve examples and archived commands inside fenced or inline code.
    fenced = False
    fence_char = ""
    fence_size = 0
    offset = 0
    for line_number, line in enumerate(text.splitlines(keepends=True), 1):
        marker = re.match(r"^\s{0,3}(`{3,}|~{3,})", line)
        if marker:
            token = marker[1]
            if not fenced:
                fenced, fence_char, fence_size = True, token[0], len(token)
            elif token[0] == fence_char and len(token) >= fence_size:
                fenced = False
            offset += len(line)
            continue
        if not fenced:
            masked = re.sub(r"(`+).*?\1", lambda m: " " * len(m[0]), line)
            # HH_260906 - Support common inline/image links and reference definitions; retain titles.
            pattern = r'\]\(\s*(?P<inline><[^>\n]+>|[^\s()]+(?:\([^()]*\)[^\s()]*)*)|^\s{0,3}\[[^]\n]+\]:\s*(?P<reference><[^>\n]+>|\S+)'
            for match in re.finditer(pattern, masked):
                group = "inline" if match["inline"] is not None else "reference"
                start, end = match.span(group)
                yield offset + start, offset + end, line_number, line[start:end]
        offset += len(line)


def inspect_link(root, document, raw):
    angled = raw.startswith("<") and raw.endswith(">")
    value = raw[1:-1] if angled else raw
    if not value or value.startswith(("#", "//")) or re.match(r"^[a-zA-Z][a-zA-Z0-9+.-]*:", value):
        return None
    path_text = re.split(r"[?#]", value, maxsplit=1)[0]
    suffix = value[len(path_text):]
    target = Path(os.path.abspath(document.parent / unquote(path_text)))
    try:
        relative = target.relative_to(root)
    except ValueError:
        return dict(status="outside_workspace", destination=raw)
    parts = relative.parts
    canonical = root / MOVES[parts[0]] / Path(*parts[1:]) if parts and parts[0] in MOVES else target
    legacy = canonical != target
    exists = canonical.exists()
    replacement = os.path.relpath(canonical, document.parent).replace(os.sep, "/") + suffix
    if angled or " " in replacement:
        replacement = "<" + replacement + ">"
    return dict(status="legacy" if legacy and exists else "missing" if not exists else "ok",
                destination=raw, canonical=canonical.relative_to(root).as_posix(),
                replacement=replacement if legacy and exists else raw)


def audit(root, documents):
    records, edits = [], {}
    for path in sorted(documents):
        text = path.read_text(encoding="utf-8")
        replacements = []
        for start, end, line, raw in destinations(text):
            item = inspect_link(root, path, raw)
            if item is None:
                continue
            records.append(dict(document=path.relative_to(root).as_posix(), line=line, **item))
            if item["status"] == "legacy":
                replacements.append((start, end, item["replacement"]))
        updated = text
        for start, end, replacement in reversed(replacements):
            updated = updated[:start] + replacement + updated[end:]
        if updated != text:
            edits[path] = (text, updated)
    return records, edits


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fix", action="store_true", help="Rewrite only existing canonical destinations; back up changed documents.")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    paths = set(ROOT.glob("*.md"))
    for folder in ("docs", "tools", "learning"):
        paths.update(p for p in (ROOT / folder).rglob("*.md") if not p.is_symlink())
    # HH_260929 - Archived remote inputs have original relative links and pinned SHA-256 bytes.
    snapshots = {path for path in paths if is_immutable_source_snapshot(path)}
    paths.difference_update(snapshots)
    records, edits = audit(ROOT, paths)
    archive = None
    if args.fix and edits:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        archive = ROOT / ".workspace_archive" / ("markdown_links_" + stamp)
        archive.mkdir(parents=True, exist_ok=False)
        changes = []
        for path, (before, after) in edits.items():
            # HH_260906 - Refuse concurrent document edits; never replace raw evidence or absent targets.
            payload = path.read_bytes()
            if payload.decode("utf-8") != before:
                raise RuntimeError("document changed during audit: " + str(path))
            backup = archive / path.relative_to(ROOT)
            backup.parent.mkdir(parents=True, exist_ok=True)
            backup.write_bytes(payload)
            path.write_text(after, encoding="utf-8")
            changes.append(dict(path=path.relative_to(ROOT).as_posix(),
                                before_sha256=hashlib.sha256(payload).hexdigest(),
                                after_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        (archive / "changes.json").write_text(json.dumps(changes, indent=2) + "\n")
        records, _ = audit(ROOT, paths)
    result = dict(scope="Navigational Markdown destinations; excludes byte-locked remote source snapshots, anchors, external URLs, HTML and runtime imports",
                  documents=len(paths), local_links=len(records),
                  excluded_immutable_source_snapshots=len(snapshots),
                  changed_documents=len(edits) if args.fix else 0,
                  backup=str(archive) if archive else None,
                  counts={status:sum(row["status"] == status for row in records)
                          for status in ("ok", "legacy", "missing", "outside_workspace")},
                  issues=[row for row in records if row["status"] != "ok"])
    if args.report:
        with args.report.open("x", encoding="utf-8") as output:
            output.write(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({key:value for key,value in result.items() if key != "issues"}, indent=2, ensure_ascii=False))
    return int(bool(result["counts"]["missing"] or result["counts"]["legacy"]))


if __name__ == "__main__":
    raise SystemExit(main())
