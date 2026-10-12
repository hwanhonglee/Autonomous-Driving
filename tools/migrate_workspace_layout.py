"""HH_260906 - One-time same-filesystem layout migration with legacy compatibility paths."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
MOVES = {
    'src': 'autonomy/src', 'autoware_e2e_vad_launch': 'autonomy/autoware_e2e_vad_launch',
    'build': 'autonomy/build', 'install': 'autonomy/install', 'log': 'autonomy/log',
    'cmake': 'autonomy/cmake', 'include': 'autonomy/include', 'patches': 'autonomy/patches',
    'portable_e2e': 'learning/portable_e2e', 'config': 'learning/config', 'tests': 'learning/tests',
    'scripts': 'tools', 'data': 'storage/assets', 'datasets': 'storage/datasets',
    'artifacts': 'storage/experiments', 'workflows': 'docs/workflows',
}


def remap(path):
    relative = path.relative_to(ROOT)
    first = relative.parts[0]
    return ROOT / MOVES[first] / Path(*relative.parts[1:]) if first in MOVES else path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    for old, new in MOVES.items():
        source, target = ROOT / old, ROOT / new
        if not source.is_dir() or source.is_symlink() or target.exists():
            raise ValueError('fresh real source and absent destination required: ' + old)
        if source.stat().st_dev != ROOT.stat().st_dev:
            raise ValueError('refuse cross-filesystem migration')
    print(json.dumps(MOVES, indent=2))
    if not args.apply:
        return
    archive = ROOT / '.workspace_archive/layout_v1_20260922'
    archive.mkdir(exist_ok=False)
    # HH_260906 - Save the pre-existing dirty state without staging or resetting any user changes.
    (archive / 'git_status_before.bin').write_bytes(subprocess.check_output(
        ['git', 'status', '--porcelain=v1', '-z'], cwd=ROOT))
    records, links = {}, []
    for old in MOVES:
        base = ROOT / old
        records[old] = dict(inode=base.stat().st_ino, device=base.stat().st_dev)
        for directory, dirs, files in os.walk(base, followlinks=False):
            for name in dirs + files:
                path = Path(directory) / name
                if path.is_symlink():
                    links.append((path, os.readlink(path), path.resolve(strict=False), path.exists()))
    (archive / 'symlinks_before.json').write_text(json.dumps([
        [str(p.relative_to(ROOT)), target, str(resolved), exists] for p, target, resolved, exists in links], indent=2))
    for old, new in MOVES.items():
        source, target = ROOT / old, ROOT / new
        target.parent.mkdir(parents=True, exist_ok=True)
        source.rename(target)
        source.symlink_to(new, target_is_directory=True)
        assert target.stat().st_ino == records[old]['inode']
    changed = []
    for oldpath, target, resolved, existed in links:
        newpath = remap(oldpath)
        expected = remap(resolved) if resolved.is_relative_to(ROOT) else resolved
        if newpath.resolve(strict=False) != expected.resolve(strict=False):
            # HH_260906 - Only replace the verified symlink itself, never its target.
            assert newpath.is_symlink() and os.readlink(newpath) == target
            replacement = os.path.relpath(expected, newpath.parent)
            newpath.unlink()
            newpath.symlink_to(replacement, target_is_directory=expected.is_dir())
            changed.append(dict(path=str(newpath.relative_to(ROOT)), before=target, after=replacement))
        if existed and not newpath.exists():
            raise ValueError('newly broken link: ' + str(newpath))
    result = dict(timestamp_utc=datetime.now(timezone.utc).isoformat(), moves=MOVES,
                  original_directory_inodes=records, changed_symlinks=changed,
                  checked_symlinks=len(links), files_deleted=0, compatibility_links=True)
    (archive / 'migration.json').write_text(json.dumps(result, indent=2))
    print('MOVED_WITH_COMPATIBILITY', len(MOVES), 'LINKS_CHECKED', len(links), 'LINKS_REBASED', len(changed))


if __name__ == '__main__':
    main()
