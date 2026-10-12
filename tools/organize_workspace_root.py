"""HH_260906 - Relocate only the fixed, historical root matrix outputs without deleting evidence."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
TARGETS = ('SUMMARY.md', 'aggregate.json', 'matrix_plan.json', 'maps')
DESTINATION = Path('artifacts/validation/2026-08-31/legacy_root_matrix')


def inventory(root):
    rows = {}
    for name in TARGETS:
        path = root / name
        if not path.exists() or path.is_symlink():
            raise ValueError('missing or linked source: ' + name)
        files = sorted(path.rglob('*')) if path.is_dir() else [path]
        for item in files:
            if item.is_symlink():
                raise ValueError('linked evidence is not movable')
            if item.is_file():
                relative = item.relative_to(root).as_posix()
                if subprocess.run(['git', 'ls-files', '--error-unmatch', '--', relative],
                                  cwd=root, capture_output=True).returncode == 0:
                    raise ValueError('tracked file is not movable: ' + relative)
                rows[relative] = hashlib.sha256(item.read_bytes()).hexdigest()
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--apply', action='store_true')
    a = p.parse_args()
    before = inventory(ROOT)
    destination = ROOT / DESTINATION
    if destination.exists():
        raise ValueError('refuse existing destination')
    print(json.dumps(dict(targets=TARGETS, files=len(before), destination=str(destination),
                          apply=a.apply), indent=2))
    if not a.apply:
        return
    if inventory(ROOT) != before:
        raise ValueError('source changed during preflight')
    destination.mkdir(parents=True, exist_ok=False)
    # HH_260906 - Same-filesystem rename keeps every byte and relative map link intact.
    for name in TARGETS:
        (ROOT / name).rename(destination / name)
    for relative, digest in before.items():
        if hashlib.sha256((destination / relative).read_bytes()).hexdigest() != digest:
            raise ValueError('moved evidence digest differs: ' + relative)
    record = dict(timestamp_utc=datetime.now(timezone.utc).isoformat(),
                  moved={name: str(DESTINATION / name) for name in TARGETS},
                  file_sha256=before, files_deleted=0,
                  restore='Move the four entries back only if their original paths are absent.')
    with (ROOT / 'docs/workflows/05_maintenance/root_reorganization.json').open('x') as stream:
        json.dump(record, stream, indent=2)


if __name__ == '__main__':
    main()
