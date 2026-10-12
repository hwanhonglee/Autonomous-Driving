#!/usr/bin/env python3
"""HH_260906 - Probe exact remaining aliases under the runtime lock, always restoring them."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
TARGETS = {
    'scripts': 'tools', 'portable_e2e': 'learning/portable_e2e',
    'data': 'storage/assets', 'datasets': 'storage/datasets',
    'artifacts': 'storage/experiments',
}


def probe(root, names, check):
    # HH_260906 - Never unlink directories, unknown links, or their underlying data.
    for name in names:
        path = root / name
        if not path.is_symlink() or os.readlink(path) != TARGETS[name]:
            raise ValueError('unexpected alias: ' + name)
        target = root / TARGETS[name]
        if not target.is_dir() or target.is_symlink():
            raise ValueError('invalid canonical target: ' + name)
    removed = []
    try:
        for name in names:
            (root / name).unlink()
            removed.append(name)
        return check()
    finally:
        for name in removed:
            if os.path.lexists(root / name):
                raise RuntimeError('refusing to replace a newly created path: ' + name)
            (root / name).symlink_to(TARGETS[name], target_is_directory=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--aliases', nargs='+', choices=TARGETS, default=list(TARGETS))
    parser.add_argument('--log-dir', type=Path, required=True)
    parser.add_argument('--collect-only', action='store_true')
    parser.add_argument('--test-path', action='append', type=Path,
                        help='Explicit existing test file/directory inside learning/tests')
    args = parser.parse_args()
    with (ROOT / 'storage/assets/locks/autoware_e2e_runtime.lock').open('a') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        args.log_dir.mkdir(parents=True, exist_ok=False)
        env = dict(os.environ, CUDA_VISIBLE_DEVICES='', PYTHONDONTWRITEBYTECODE='1',
                   PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
                   AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD=str(lock.fileno()))
        import shlex
        paths = args.test_path or [ROOT/'learning/tests']
        for path in paths:
            if not path.resolve().is_relative_to(ROOT/'learning/tests') or not path.exists():
                raise ValueError('test path must exist within learning/tests')
        command = ('source tools/e2e/env.sh && python3 tools/workspace_python.py -m pytest -- '
                   '-ra -q -p no:cacheprovider ' + ' '.join(shlex.quote(str(path)) for path in paths))
        if args.collect_only:
            command += ' --collect-only'
        with (args.log_dir / 'pytest.log').open('w') as stream:
            result = probe(ROOT, args.aliases, lambda: subprocess.run(
                ['bash', '-c', command], cwd=ROOT, env=env, stdout=stream,
                stderr=subprocess.STDOUT, pass_fds=(lock.fileno(),), check=False))
        record = dict(aliases_probed=args.aliases, restored=True,
                      returncode=result.returncode, permanent_removals=0)
        (args.log_dir / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
        print(json.dumps(record))
        return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
