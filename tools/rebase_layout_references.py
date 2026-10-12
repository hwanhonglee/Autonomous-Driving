"""HH_260906 - Mechanical path updates for relocated tests and Markdown; preserve edit backups."""
import hashlib
import json
import os
from pathlib import Path
import re
from migrate_workspace_layout import MOVES, ROOT


def repository_expression():
    return 'next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())'


def main():
    archive = ROOT / '.workspace_archive/layout_v1_20260922/reference_edits'
    archive.mkdir(exist_ok=False)
    changes = []
    def save(path, text):
        old = path.read_text()
        if text == old:
            return
        relative = path.relative_to(ROOT)
        backup = archive / relative
        backup.parent.mkdir(parents=True, exist_ok=True)
        backup.write_bytes(path.read_bytes())
        path.write_text(text)
        changes.append(dict(path=str(relative), before=hashlib.sha256(old.encode()).hexdigest(),
                            after=hashlib.sha256(text.encode()).hexdigest()))
    for folder, depth in [('learning/tests', 1), ('autonomy/autoware_e2e_vad_launch/test', 2)]:
        pattern = r'Path\(__file__\)(?:\.resolve\(\))?\.parents\[' + str(depth) + r'\]'
        for path in (ROOT / folder).rglob('*.py'):
            old = path.read_text()
            text = re.sub(pattern, repository_expression(), old)
            if text != old:
                text = '# HH_260906 - Locate the repository independently of the relocated test directory.\n' + text
            save(path, text)
    # HH_260906 - Rebase only moved human-readable guides, not immutable historical evidence.
    for oldbase, newbase in MOVES.items():
        if oldbase not in ('workflows', 'portable_e2e', 'config', 'scripts'):
            continue
        for path in (ROOT / newbase).rglob('*.md'):
            if path.is_symlink():
                continue
            oldpath = ROOT / oldbase / path.relative_to(ROOT / newbase)
            def link(match):
                target = match.group(1)
                if '://' in target or target.startswith(('/', '#', '<')):
                    return match.group(0)
                base, marker, fragment = target.partition('#')
                absolute = Path(os.path.abspath(oldpath.parent / base))
                if not absolute.is_relative_to(ROOT):
                    return match.group(0)
                relative = absolute.relative_to(ROOT)
                if relative.parts and relative.parts[0] in MOVES:
                    absolute = ROOT / MOVES[relative.parts[0]] / Path(*relative.parts[1:])
                result = os.path.relpath(absolute, path.parent) + (marker + fragment if marker else '')
                return '](' + result + ')'
            save(path, re.sub(r'\]\(([^)]+)\)', link, path.read_text()))
    (archive / 'manifest.json').write_text(json.dumps(changes, indent=2))
    print('REFERENCE_FILES_UPDATED', len(changes))


if __name__ == '__main__':
    main()
